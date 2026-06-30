#!/usr/bin/env python3
"""
Virtual Betaflight SITL RC sender.

This tool sends synthetic rc_packet datagrams to Betaflight SITL UDP :9004 so
Gazebo/Betaflight tests can run without a physical transmitter. Channel order is
the same AETR order used by sitl_rc_bridge.py:

  CH1 roll, CH2 pitch, CH3 throttle, CH4 yaw, CH5/AUX1 arm,
  CH6/AUX2 Kenet state, CH7/AUX3 flight mode.
"""

from __future__ import annotations

import argparse
import socket
import time
from dataclasses import dataclass
from pathlib import Path

from sitl_log import JsonlLogger
from sitl_rc_bridge import clamp_rc, pack_rc_packet


ROLL_CH = 0
PITCH_CH = 1
THROTTLE_CH = 2
YAW_CH = 3
ARM_CH = 4
KENET_CH = 5
MODE_CH = 6


@dataclass(frozen=True)
class RcStep:
    name: str
    seconds: float
    channels: list[int]
    start_throttle: int | None = None
    end_throttle: int | None = None

    def channels_at(self, elapsed: float) -> list[int]:
        channels = list(self.channels)
        if self.start_throttle is not None and self.end_throttle is not None:
            if self.seconds <= 0:
                fraction = 1.0
            else:
                fraction = max(0.0, min(1.0, elapsed / self.seconds))
            throttle = self.start_throttle + (self.end_throttle - self.start_throttle) * fraction
            channels[THROTTLE_CH] = clamp_rc(throttle)
        return channels


def make_virtual_channels(
    *,
    roll: int = 1500,
    pitch: int = 1500,
    throttle: int = 1000,
    yaw: int = 1500,
    arm_pwm: int = 1000,
    kenet_pwm: int = 1000,
    mode_pwm: int = 1000,
) -> list[int]:
    channels = [1500] * 16
    channels[ROLL_CH] = clamp_rc(roll)
    channels[PITCH_CH] = clamp_rc(pitch)
    channels[THROTTLE_CH] = clamp_rc(throttle)
    channels[YAW_CH] = clamp_rc(yaw)
    channels[ARM_CH] = clamp_rc(arm_pwm)
    channels[KENET_CH] = clamp_rc(kenet_pwm)
    channels[MODE_CH] = clamp_rc(mode_pwm)
    return channels


def rc_summary(channels: list[int]) -> str:
    return ",".join(str(value) for value in channels[:8])


def manual_step(args: argparse.Namespace) -> RcStep:
    arm_pwm = 2000 if args.arm else args.arm_pwm
    return RcStep(
        "manual",
        args.duration,
        make_virtual_channels(
            roll=args.roll,
            pitch=args.pitch,
            throttle=args.throttle,
            yaw=args.yaw,
            arm_pwm=arm_pwm,
            kenet_pwm=args.kenet_pwm,
            mode_pwm=args.mode_pwm,
        ),
    )


def has_nudge(args: argparse.Namespace) -> bool:
    return any(getattr(args, name, None) is not None for name in ("nudge_roll", "nudge_pitch", "nudge_yaw"))


def nudge_axis(args: argparse.Namespace, name: str, nudged: bool) -> int:
    value = getattr(args, "nudge_%s" % name, None)
    if nudged and value is not None:
        return value
    return getattr(args, name)


def takeoff_channels(args: argparse.Namespace, *, throttle: int, arm_pwm: int, nudged: bool = False) -> list[int]:
    return make_virtual_channels(
        roll=nudge_axis(args, "roll", nudged),
        pitch=nudge_axis(args, "pitch", nudged),
        throttle=throttle,
        yaw=nudge_axis(args, "yaw", nudged),
        arm_pwm=arm_pwm,
        kenet_pwm=args.kenet_pwm,
        mode_pwm=args.mode_pwm,
    )


def takeoff_steps(args: argparse.Namespace) -> list[RcStep]:
    low = takeoff_channels(
        args,
        throttle=1000,
        arm_pwm=1000,
    )
    armed_low = takeoff_channels(
        args,
        throttle=1000,
        arm_pwm=2000,
    )
    throttle = takeoff_channels(
        args,
        throttle=args.throttle,
        arm_pwm=2000,
    )
    throttle_nudged = takeoff_channels(
        args,
        throttle=args.throttle,
        arm_pwm=2000,
        nudged=True,
    )
    steps = [
        RcStep("boot-low", args.low_seconds, low),
        RcStep("arm-low-throttle", args.arm_seconds, armed_low),
        RcStep(
            "throttle-ramp",
            args.ramp_seconds,
            armed_low,
            start_throttle=1000,
            end_throttle=args.throttle,
        ),
    ]
    if has_nudge(args):
        delay = args.nudge_delay_seconds
        if delay is None:
            delay = args.low_seconds + args.arm_seconds + args.ramp_seconds
        hold_delay = max(0.0, delay - args.low_seconds - args.arm_seconds - args.ramp_seconds)
        neutral_seconds = min(args.hold_seconds, hold_delay)
        nudge_seconds = max(0.0, args.hold_seconds - neutral_seconds)
        if neutral_seconds > 0:
            steps.append(RcStep("takeoff-hold", neutral_seconds, throttle))
        if nudge_seconds > 0:
            steps.append(RcStep("takeoff-hold-nudge", nudge_seconds, throttle_nudged))
    else:
        steps.append(RcStep("takeoff-hold", args.hold_seconds, throttle))
    if args.disarm_seconds > 0:
        steps.append(RcStep("disarm-low", args.disarm_seconds, low))
    return steps


def build_steps(args: argparse.Namespace) -> list[RcStep]:
    if args.script == "manual":
        return [manual_step(args)]
    if args.script == "takeoff":
        return takeoff_steps(args)
    raise ValueError("unknown script: %s" % args.script)


def send_channels(sock: socket.socket | None, channels: list[int], host: str, port: int) -> int:
    if sock is None:
        return 0
    return sock.sendto(pack_rc_packet(channels), (host, port))


def run(args: argparse.Namespace) -> int:
    steps = build_steps(args)
    period = 1.0 / args.rate_hz
    print_period = 1.0 / args.print_hz
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) if args.send else None
    logger = JsonlLogger(Path(args.log_file), metadata={
        "tool": "sitl_virtual_rc",
        "script": args.script,
        "send": args.send,
        "host": args.host,
        "port": args.port,
        "rate_hz": args.rate_hz,
    }) if args.log_file else None
    send_count = 0
    next_print = 0.0

    if args.send:
        print("virtual RC sending to %s:%d at %.1fHz" % (args.host, args.port, args.rate_hz))
    else:
        print("virtual RC dry-run; add --send to transmit UDP packets")

    try:
        for step in steps:
            step_started = time.monotonic()
            if logger is not None:
                logger.write(
                    "virtual_rc_step_start",
                    step=step.name,
                    seconds=step.seconds,
                    channels=step.channels,
                )
            while True:
                elapsed = time.monotonic() - step_started
                if step.seconds > 0 and elapsed >= step.seconds:
                    break
                channels = step.channels_at(elapsed)
                sent = send_channels(sock, channels, args.host, args.port)
                if args.send:
                    send_count += 1
                now = time.monotonic()
                if args.once or now >= next_print:
                    print(
                        "step=%s t=%.2f/%s bytes=%d rc_us=%s" %
                        (
                            step.name,
                            elapsed,
                            "%.2f" % step.seconds if step.seconds > 0 else "inf",
                            sent,
                            rc_summary(channels),
                        ),
                        flush=True,
                    )
                    if logger is not None:
                        logger.write(
                            "virtual_rc_frame",
                            step=step.name,
                            elapsed=elapsed,
                            seconds=step.seconds,
                            sent_bytes=sent,
                            channels=channels,
                        )
                    next_print = now + print_period
                if args.once:
                    if logger is not None:
                        logger.write("virtual_rc_summary", sent_packets=send_count)
                    print("sent_packets=%d" % send_count)
                    return 0
                time.sleep(period)
                if step.seconds <= 0:
                    continue
        if logger is not None:
            logger.write("virtual_rc_summary", sent_packets=send_count)
        print("sent_packets=%d" % send_count)
        return 0
    except KeyboardInterrupt:
        print("interrupted; sent_packets=%d" % send_count)
        return 130
    finally:
        if sock is not None:
            sock.close()
        if logger is not None:
            logger.close()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=9004)
    parser.add_argument("--rate-hz", type=float, default=50.0)
    parser.add_argument("--print-hz", type=float, default=5.0)
    parser.add_argument("--send", action="store_true", help="Transmit UDP RC packets")
    parser.add_argument("--once", action="store_true", help="Send/print one frame and exit")
    parser.add_argument("--script", choices=["manual", "takeoff"], default="manual")
    parser.add_argument("--log-file", default=None, help="Optional JSONL path for sent virtual RC frames")

    parser.add_argument("--roll", type=int, default=1500)
    parser.add_argument("--pitch", type=int, default=1500)
    parser.add_argument("--throttle", type=int, default=1000,
                        help="Manual throttle, or takeoff target throttle")
    parser.add_argument("--yaw", type=int, default=1500)
    parser.add_argument("--arm", action="store_true", help="Set CH5/AUX1 high in manual mode")
    parser.add_argument("--arm-pwm", type=int, default=1000,
                        help="Manual-mode CH5/AUX1 value when --arm is not used")
    parser.add_argument("--kenet-pwm", type=int, default=1000,
                        help="CH6/AUX2 Kenet state value")
    parser.add_argument("--mode-pwm", type=int, default=1000,
                        help="CH7/AUX3 flight-mode value, e.g. 1500 for ANGLE")

    parser.add_argument("--duration", type=float, default=0.0,
                        help="Manual script duration; 0 means run until Ctrl-C")
    parser.add_argument("--low-seconds", type=float, default=5.0)
    parser.add_argument("--arm-seconds", type=float, default=3.0)
    parser.add_argument("--ramp-seconds", type=float, default=3.0)
    parser.add_argument("--hold-seconds", type=float, default=5.0)
    parser.add_argument("--disarm-seconds", type=float, default=1.0)
    parser.add_argument("--nudge-delay-seconds", type=float, default=None,
                        help="In takeoff mode, switch to nudge roll/pitch/yaw after this script time")
    parser.add_argument("--nudge-roll", type=int, default=None,
                        help="Roll PWM to apply after --nudge-delay-seconds")
    parser.add_argument("--nudge-pitch", type=int, default=None,
                        help="Pitch PWM to apply after --nudge-delay-seconds")
    parser.add_argument("--nudge-yaw", type=int, default=None,
                        help="Yaw PWM to apply after --nudge-delay-seconds")

    args = parser.parse_args()
    for name in ("roll", "pitch", "throttle", "yaw", "arm_pwm", "kenet_pwm", "mode_pwm"):
        value = getattr(args, name)
        if not 1000 <= value <= 2000:
            parser.error("--%s must be between 1000 and 2000" % name.replace("_", "-"))
    for name in ("rate_hz", "print_hz"):
        if getattr(args, name) <= 0:
            parser.error("--%s must be positive" % name.replace("_", "-"))
    for name in ("duration", "low_seconds", "arm_seconds", "ramp_seconds", "hold_seconds", "disarm_seconds"):
        if getattr(args, name) < 0:
            parser.error("--%s must not be negative" % name.replace("_", "-"))
    if args.nudge_delay_seconds is not None and args.nudge_delay_seconds < 0:
        parser.error("--nudge-delay-seconds must not be negative")
    for name in ("nudge_roll", "nudge_pitch", "nudge_yaw"):
        value = getattr(args, name)
        if value is not None and not 1000 <= value <= 2000:
            parser.error("--%s must be between 1000 and 2000" % name.replace("_", "-"))
    return args


if __name__ == "__main__":
    raise SystemExit(run(parse_args()))
