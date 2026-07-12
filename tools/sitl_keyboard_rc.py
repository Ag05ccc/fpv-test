#!/usr/bin/env python3
"""
Keyboard-driven virtual RC for Betaflight SITL.

Flies the simulated drone from the terminal with no physical hardware by
sending Betaflight SITL rc_packet datagrams to UDP :9004 (same packet format
as tools/sitl_rc_bridge.py, Python stdlib only).

Key map:
  w / s      throttle up/down (LATCHED, +/- --throttle-step per press)
  x          snap throttle to 1000
  a / d      yaw left/right (MOMENTARY, auto-centers after --stick-hold)
  j / l      roll left/right (MOMENTARY)
  i / k      pitch nose-forward/back (MOMENTARY)
  e          toggle ARM (CH5/AUX1)
  space      PANIC: disarm + throttle 1000 + center sticks
  1 / 2 / 3  Kenet state CH6/AUX2 = 1000 / 1500 / 2000
  q or ESC   quit (sends a safe-exit burst: ARM 1000, throttle 1000)

CH7/AUX3 flight mode is forced to --mode-pwm (default 1500, ANGLE).
Startup state is safe: disarmed, throttle 1000, sticks centered, Kenet idle.
"""

import argparse
import os
import select
import socket
import sys
import time

from sitl_rc_bridge import clamp_rc, pack_rc_packet
from sitl_rc_channels import (
    ARM_CH,
    AUTOPILOT_MODE_CHANNEL,
    KENET_STATE_CH,
    PITCH_CH,
    ROLL_CH,
    THROTTLE_CH,
    YAW_CH,
)


RC_CHANNEL_COUNT = 16
STICK_NEUTRAL = 1500
THROTTLE_MIN = 1000
THROTTLE_MAX = 2000
ARM_ON_PWM = 2000
ARM_OFF_PWM = 1000
KENET_IDLE_PWM = 1000
SAFE_EXIT_PACKET_COUNT = 10
STATUS_PERIOD_S = 0.5
ESC_KEY = "\x1b"
QUIT_KEYS = ("q", ESC_KEY)

# key -> (pilot channel index, deflection sign). 'i' is nose forward (+pitch).
STICK_KEYS = {
    "a": (YAW_CH, -1),
    "d": (YAW_CH, +1),
    "j": (ROLL_CH, -1),
    "l": (ROLL_CH, +1),
    "i": (PITCH_CH, +1),
    "k": (PITCH_CH, -1),
}

KENET_STATE_KEYS = {
    "1": 1000,
    "2": 1500,
    "3": 2000,
}


class KeyboardRcState:
    """Pure keyboard-to-RC state machine.

    All timing comes in through explicit `now` arguments so tests never sleep.
    No terminal, no socket, no wall clock in here.
    """

    def __init__(self, throttle_step=20, stick_step=150, stick_hold=0.30,
                 mode_pwm=1500, yaw_authority=0):
        self.throttle_step = int(throttle_step)
        self.stick_step = int(stick_step)
        self.stick_hold = float(stick_hold)
        # Yaw safety clamp: this plant spins on any sustained yaw beyond
        # ~+/-10 us (measured 2026-07-04, even at a stable yaw rate-PID). 0
        # disables the clamp; a small value keeps manual yaw inside the stable
        # window so a held a/d key cannot spin the drone.
        self.yaw_authority = int(yaw_authority)
        self.mode_pwm = clamp_rc(mode_pwm)
        self.throttle = THROTTLE_MIN
        self.armed = False
        self.kenet_pwm = KENET_IDLE_PWM
        self.quit_requested = False
        self._stick_deflection = {ROLL_CH: 0, PITCH_CH: 0, YAW_CH: 0}
        self._stick_deadline = {ROLL_CH: 0.0, PITCH_CH: 0.0, YAW_CH: 0.0}

    def apply_key(self, key, now):
        """Apply one keypress at time `now`. Returns True if the key did something."""
        key = key.lower()
        if key == "w":
            self.throttle = min(THROTTLE_MAX, self.throttle + self.throttle_step)
        elif key == "s":
            self.throttle = max(THROTTLE_MIN, self.throttle - self.throttle_step)
        elif key == "x":
            self.throttle = THROTTLE_MIN
        elif key in STICK_KEYS:
            channel, direction = STICK_KEYS[key]
            # One step only: a repeat press refreshes the hold timer, the
            # opposite key crosses straight to the opposite deflection.
            self._stick_deflection[channel] = direction * self.stick_step
            self._stick_deadline[channel] = now + self.stick_hold
        elif key == "e":
            self.armed = not self.armed
        elif key == " ":
            self.panic()
        elif key in KENET_STATE_KEYS:
            self.kenet_pwm = KENET_STATE_KEYS[key]
        elif key in QUIT_KEYS:
            self.quit_requested = True
        else:
            return False
        return True

    def panic(self):
        self.armed = False
        self.throttle = THROTTLE_MIN
        for channel in self._stick_deflection:
            self._stick_deflection[channel] = 0
            self._stick_deadline[channel] = 0.0

    def channels(self, now):
        """Return the 16 pilot/AETR RC channel values at time `now`."""
        channels = [STICK_NEUTRAL] * RC_CHANNEL_COUNT
        channels[THROTTLE_CH] = self.throttle
        for channel, deflection in self._stick_deflection.items():
            if deflection and now < self._stick_deadline[channel]:
                if channel == YAW_CH and self.yaw_authority > 0:
                    deflection = max(-self.yaw_authority,
                                     min(self.yaw_authority, deflection))
                channels[channel] = clamp_rc(STICK_NEUTRAL + deflection)
        channels[ARM_CH] = ARM_ON_PWM if self.armed else ARM_OFF_PWM
        channels[KENET_STATE_CH] = self.kenet_pwm
        channels[AUTOPILOT_MODE_CHANNEL] = self.mode_pwm
        return channels


def extract_keys(text):
    """Filter terminal escape sequences out of a raw stdin chunk.

    A lone ESC is a real quit request, but arrow/function keys arrive as
    multi-character sequences (ESC [ ... or ESC O ...) inside the same read
    chunk; those must not be mistaken for an ESC quit.
    """
    keys = []
    i = 0
    while i < len(text):
        ch = text[i]
        if ch == ESC_KEY and i + 1 < len(text) and text[i + 1] in "[O":
            i += 2
            while i < len(text) and text[i] in "0123456789;":
                i += 1
            i += 1  # final byte of the sequence
            continue
        keys.append(ch)
        i += 1
    return keys


def safe_exit_channels(mode_pwm=1500):
    """Channels for the quit-time safety burst: disarmed, throttle low."""
    channels = [STICK_NEUTRAL] * RC_CHANNEL_COUNT
    channels[THROTTLE_CH] = THROTTLE_MIN
    channels[ARM_CH] = ARM_OFF_PWM
    channels[KENET_STATE_CH] = KENET_IDLE_PWM
    channels[AUTOPILOT_MODE_CHANNEL] = clamp_rc(mode_pwm)
    return channels


def format_status(channels, armed, send_count, verbose=False):
    shown = channels if verbose else channels[:8]
    return "rc_us=%s | %s | sent=%d" % (
        ",".join(str(value) for value in shown),
        "ARMED" if armed else "DISARMED",
        send_count,
    )


def print_key_help():
    print("Keyboard RC controls:")
    print("  w/s throttle +/- (latched)   x throttle to 1000")
    print("  a/d yaw   j/l roll   i/k pitch (momentary, auto-center)")
    print("  e ARM toggle   space PANIC (disarm + throttle 1000)")
    print("  1/2/3 Kenet state idle/armed/tracking   q or ESC quit")


def run(args):
    import termios
    import tty

    if not sys.stdin.isatty():
        raise SystemExit("sitl_keyboard_rc needs an interactive terminal (stdin is not a TTY)")

    state = KeyboardRcState(
        throttle_step=args.throttle_step,
        stick_step=args.stick_step,
        stick_hold=args.stick_hold,
        mode_pwm=args.mode_pwm,
        yaw_authority=args.yaw_authority,
    )
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    stdin_fd = sys.stdin.fileno()
    old_termios = termios.tcgetattr(stdin_fd)
    period = 1.0 / args.rate_hz
    deadline = None if args.duration <= 0 else time.monotonic() + args.duration
    next_send = time.monotonic()
    next_status = 0.0
    send_count = 0

    print("host=%s port=%d rate=%.1fHz mode_pwm=%d" %
          (args.host, args.port, args.rate_hz, args.mode_pwm))
    print_key_help()

    tty.setcbreak(stdin_fd)
    try:
        while not state.quit_requested:
            now = time.monotonic()
            if deadline is not None and now >= deadline:
                break

            ready, _, _ = select.select([stdin_fd], [], [], max(0.0, next_send - now))
            if ready:
                try:
                    data = os.read(stdin_fd, 64)
                except OSError:
                    data = b""
                for key in extract_keys(data.decode("ascii", errors="ignore")):
                    handled = state.apply_key(key, time.monotonic())
                    if args.verbose and handled:
                        sys.stdout.write("\rkey=%r%s\n" % (key, " " * 40))

            now = time.monotonic()
            if now >= next_send:
                channels = state.channels(now)
                sock.sendto(pack_rc_packet(channels), (args.host, args.port))
                send_count += 1
                next_send = now + period
                if now >= next_status:
                    line = format_status(channels, state.armed, send_count, args.verbose)
                    sys.stdout.write("\r" + line + "  ")
                    sys.stdout.flush()
                    next_status = now + STATUS_PERIOD_S
    except KeyboardInterrupt:
        pass
    finally:
        try:
            burst_channels = safe_exit_channels(args.mode_pwm)
            for _ in range(SAFE_EXIT_PACKET_COUNT):
                sock.sendto(pack_rc_packet(burst_channels), (args.host, args.port))
                time.sleep(period)
        finally:
            termios.tcsetattr(stdin_fd, termios.TCSADRAIN, old_termios)
            sock.close()
            sys.stdout.write("\nsafe-exit burst sent (%d packets, ARM=1000, throttle=1000)\n" %
                             SAFE_EXIT_PACKET_COUNT)


def parse_args():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=9004)
    parser.add_argument("--rate-hz", type=float, default=50.0)
    parser.add_argument("--throttle-step", type=int, default=20,
                        help="Throttle change per w/s keypress (latched)")
    parser.add_argument("--stick-step", type=int, default=150,
                        help="Momentary stick deflection per keypress, from 1500")
    parser.add_argument("--stick-hold", type=float, default=0.30,
                        help="Seconds a stick deflection persists without a new keypress")
    parser.add_argument("--yaw-authority", type=int, default=0,
                        help="Clamp yaw stick deflection to +/- this (us) from "
                             "center. 0 = no clamp. Small value (e.g. 10) keeps "
                             "manual yaw in the stable window so a held a/d key "
                             "cannot spin the drone (plant spins on real yaw).")
    parser.add_argument("--mode-pwm", type=int, default=1500,
                        help="Forced CH7/AUX3 flight-mode PWM (1500 = ANGLE)")
    parser.add_argument("--duration", type=float, default=0.0,
                        help="Stop after N seconds; 0 means run until quit")
    parser.add_argument("--verbose", action="store_true",
                        help="Print handled keys and all 16 channels in the status line")
    args = parser.parse_args()
    if args.rate_hz <= 0:
        parser.error("--rate-hz must be positive")
    if args.throttle_step <= 0:
        parser.error("--throttle-step must be positive")
    if args.stick_step <= 0:
        parser.error("--stick-step must be positive")
    if args.stick_hold <= 0:
        parser.error("--stick-hold must be positive")
    if args.yaw_authority < 0:
        parser.error("--yaw-authority must be >= 0")
    if not 1000 <= args.mode_pwm <= 2000:
        parser.error("--mode-pwm must be between 1000 and 2000")
    return args


if __name__ == "__main__":
    run(parse_args())
