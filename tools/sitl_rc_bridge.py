#!/usr/bin/env python3
"""
Minimal Betaflight SITL RC bridge.

Reads a Linux joystick device (/dev/input/jsX) with Python stdlib only and can:
  - print live axis/button values for mapping (--dry-run)
  - send Betaflight SITL rc_packet datagrams to UDP :9004 (--send)

Edit CHANNEL_MAP below after identifying your transmitter's axis/button layout.
"""

import argparse
import errno
import fcntl
import json
import os
import select
import socket
import struct
import time
from pathlib import Path

from sitl_rc_channels import ARM_CH, AUTOPILOT_MODE_CHANNEL, KENET_STATE_CH, YAW_CH


JS_EVENT_FORMAT = "<IhBB"
JS_EVENT_SIZE = struct.calcsize(JS_EVENT_FORMAT)
JS_EVENT_BUTTON = 0x01
JS_EVENT_AXIS = 0x02
JS_EVENT_INIT = 0x80
EV_EVENT_FORMAT = "llHHi"
EV_EVENT_SIZE = struct.calcsize(EV_EVENT_FORMAT)
EV_KEY = 0x01
EV_ABS = 0x03
AXIS_MAX = 32767.0


# TBS Joystick mapping found with tools/rc_monitor.py:
#   axis 0..3 -> AETR sticks
#   axis 4    -> two-state switch
#   axis 5    -> second three-state switch
#   axis 6    -> three-state switch
#
# Betaflight channel order here is AETR: roll, pitch, throttle, yaw, AUX...
CHANNEL_MAP = {
    "roll": {"channel": 0, "source": "axis", "index": 0, "invert": False},
    "pitch": {"channel": 1, "source": "axis", "index": 1, "invert": True},
    "throttle": {"channel": 2, "source": "axis", "index": 2, "invert": False},
    "yaw": {"channel": 3, "source": "axis", "index": 3, "invert": False},
    "arm_aux1": {"channel": 4, "source": "axis", "index": 4, "two_pos": True},
    "mode_aux2": {"channel": 5, "source": "axis", "index": 6, "three_pos": True},
    "autopilot_mode_aux3": {"channel": 6, "source": "axis", "index": 5, "three_pos": True},
}


class LinuxJoystick:
    def __init__(self, device):
        self.device = device
        self.fd = None
        self.axes = {}
        self.buttons = {}

    def open(self):
        self.fd = os.open(self.device, os.O_RDONLY | os.O_NONBLOCK)

    def close(self):
        if self.fd is not None:
            os.close(self.fd)
            self.fd = None

    def poll(self, timeout=0):
        ready, _, _ = select.select([self.fd], [], [], timeout)
        if not ready:
            return []

        events = []
        while True:
            try:
                data = os.read(self.fd, JS_EVENT_SIZE)
            except BlockingIOError:
                break
            except OSError as e:
                if e.errno in (errno.EAGAIN, errno.EWOULDBLOCK):
                    break
                raise

            if not data or len(data) < JS_EVENT_SIZE:
                break

            t_ms, value, event_type, number = struct.unpack(JS_EVENT_FORMAT, data)
            is_init = bool(event_type & JS_EVENT_INIT)
            event_type &= ~JS_EVENT_INIT

            if event_type == JS_EVENT_AXIS:
                self.axes[number] = value
                kind = "axis"
            elif event_type == JS_EVENT_BUTTON:
                self.buttons[number] = value
                kind = "button"
            else:
                kind = "unknown"

            events.append((t_ms, kind, number, value, is_init))
        return events


def _ioc(direction, type_, nr, size):
    nr_bits = 8
    type_bits = 8
    size_bits = 14
    nr_shift = 0
    type_shift = nr_shift + nr_bits
    size_shift = type_shift + type_bits
    dir_shift = size_shift + size_bits
    return (direction << dir_shift) | (ord(type_) << type_shift) | (nr << nr_shift) | (size << size_shift)


def _eviocgabs(abs_code):
    return _ioc(2, "E", 0x40 + abs_code, struct.calcsize("iiiiii"))


def normalize_abs_value(value, minimum, maximum):
    if maximum <= minimum:
        return int(value)
    centered = ((value - minimum) / float(maximum - minimum)) * 2.0 - 1.0
    return int(max(-AXIS_MAX, min(AXIS_MAX, round(centered * AXIS_MAX))))


class LinuxEventJoystick:
    """Read Linux evdev joystick events and expose the same fields as LinuxJoystick."""

    def __init__(self, device):
        self.device = device
        self.fd = None
        self.axes = {}
        self.buttons = {}
        self.abs_ranges = {}

    def open(self):
        self.fd = os.open(self.device, os.O_RDONLY | os.O_NONBLOCK)
        self._read_abs_state()

    def close(self):
        if self.fd is not None:
            os.close(self.fd)
            self.fd = None

    def _read_abs_state(self):
        for code in range(0, 64):
            buf = bytearray(struct.calcsize("iiiiii"))
            try:
                fcntl.ioctl(self.fd, _eviocgabs(code), buf, True)
            except OSError:
                continue
            value, minimum, maximum, _fuzz, _flat, _resolution = struct.unpack("iiiiii", buf)
            if minimum == 0 and maximum == 0 and value == 0:
                continue
            self.abs_ranges[code] = (minimum, maximum)
            self.axes[code] = normalize_abs_value(value, minimum, maximum)

    def poll(self, timeout=0):
        ready, _, _ = select.select([self.fd], [], [], timeout)
        if not ready:
            return []

        events = []
        while True:
            try:
                data = os.read(self.fd, EV_EVENT_SIZE)
            except BlockingIOError:
                break
            except OSError as e:
                if e.errno in (errno.EAGAIN, errno.EWOULDBLOCK):
                    break
                raise

            if not data or len(data) < EV_EVENT_SIZE:
                break

            sec, usec, event_type, code, value = struct.unpack(EV_EVENT_FORMAT, data)
            t_ms = sec * 1000 + usec // 1000
            if event_type == EV_ABS:
                minimum, maximum = self.abs_ranges.get(code, (-AXIS_MAX, AXIS_MAX))
                normalized = normalize_abs_value(value, minimum, maximum)
                self.axes[code] = normalized
                events.append((t_ms, "axis", code, normalized, False))
            elif event_type == EV_KEY:
                self.buttons[code] = value
                events.append((t_ms, "button", code, value, False))
        return events


def open_input_device(device):
    real = os.path.basename(os.path.realpath(device))
    if real.startswith("event"):
        return LinuxEventJoystick(device)
    return LinuxJoystick(device)


def axis_to_rc(value, invert=False):
    if invert:
        value = -value
    rc = 1500 + (value / AXIS_MAX) * 500
    return clamp_rc(rc)


def axis_to_three_pos_rc(value, invert=False):
    if invert:
        value = -value
    if value < -AXIS_MAX / 3:
        return 1000
    if value > AXIS_MAX / 3:
        return 2000
    return 1500


def axis_to_two_pos_rc(value, invert=False):
    if invert:
        value = -value
    return 2000 if value > 0 else 1000


def button_to_rc(value):
    return 2000 if value else 1000


def clamp_rc(value):
    return int(max(1000, min(2000, round(value))))


def make_channels(joystick, mapping):
    channels = [1500] * 16
    for name, cfg in mapping.items():
        ch = cfg["channel"]
        if not 0 <= ch < len(channels):
            raise ValueError("Invalid channel for %s: %s" % (name, ch))

        source = cfg.get("source")
        index = cfg.get("index")
        if source == "axis":
            raw = joystick.axes.get(index, 0)
            if cfg.get("two_pos"):
                channels[ch] = axis_to_two_pos_rc(raw, cfg.get("invert", False))
            elif cfg.get("three_pos"):
                channels[ch] = axis_to_three_pos_rc(raw, cfg.get("invert", False))
            else:
                channels[ch] = axis_to_rc(raw, cfg.get("invert", False))
        elif source == "button":
            raw = joystick.buttons.get(index, 0)
            channels[ch] = button_to_rc(raw)
        elif source == "fixed":
            channels[ch] = clamp_rc(cfg.get("value", 1500))
        else:
            raise ValueError("Invalid source for %s: %s" % (name, source))
    return channels


def apply_forced_mode_pwm(channels, value):
    """Force CH7/AUX3 for Betaflight flight-mode testing when a switch is absent."""
    if value is None:
        return channels
    channels[AUTOPILOT_MODE_CHANNEL] = clamp_rc(value)
    return channels


def apply_forced_channel_pwm(channels, channel, value):
    """Force one RC channel when a physical switch is absent or not mapped yet."""
    if value is None:
        return channels
    channels[channel] = clamp_rc(value)
    return channels


def apply_yaw_authority(channels, limit):
    """Clamp yaw stick to center +/- limit (us). This plant spins on any
    sustained yaw beyond ~+/-10 us even at a stable yaw rate-PID (measured
    2026-07-04), so limiting the yaw the pilot can command keeps manual RC in
    the stable window. limit <= 0 disables the clamp."""
    if not limit or limit <= 0:
        return channels
    channels[YAW_CH] = clamp_rc(max(1500 - limit, min(1500 + limit, channels[YAW_CH])))
    return channels


def pack_rc_packet(channels):
    return struct.pack("<d16H", time.time(), *channels)


class JsonlLogger:
    def __init__(self, path):
        self.path = Path(path) if path else None
        self.handle = None
        if self.path is not None:
            self.path.parent.mkdir(parents=True, exist_ok=True)
            self.handle = self.path.open("w", encoding="utf-8")

    def write(self, event, **fields):
        if self.handle is None:
            return
        record = {"event": event, **fields}
        self.handle.write(json.dumps(record, separators=(",", ":")) + "\n")
        self.handle.flush()

    def close(self):
        if self.handle is not None:
            self.handle.close()
            self.handle = None


def event_records(events):
    return [
        {
            "time_ms": t_ms,
            "kind": kind,
            "index": number,
            "value": value,
            "init": is_init,
        }
        for t_ms, kind, number, value, is_init in events
    ]


def rc_snapshot(joystick, channels, events, send_count):
    return {
        "time": time.time(),
        "device": joystick.device,
        "axes": {str(index): value for index, value in sorted(joystick.axes.items())},
        "buttons": {str(index): value for index, value in sorted(joystick.buttons.items())},
        "channels": list(channels),
        "channels8": list(channels[:8]),
        "events": event_records(events),
        "send_count": send_count,
    }


def print_events(events):
    for t_ms, kind, number, value, is_init in events:
        init = " init" if is_init else ""
        print("%10d %-6s %2d value=%6d%s" % (t_ms, kind, number, value, init))


def print_changed_events(events, include_init=False):
    for t_ms, kind, number, value, is_init in events:
        if is_init and not include_init:
            continue
        print("%10d changed %-6s %2d value=%6d" %
              (t_ms, kind, number, value))


def print_state(joystick, channels=None):
    axes = " ".join(
        "a%d:%+6d" % (index, joystick.axes[index])
        for index in sorted(joystick.axes)
    )
    buttons = " ".join(
        "b%d:%d" % (index, joystick.buttons[index])
        for index in sorted(joystick.buttons)
    )
    if channels is None:
        print("axes: %s | buttons: %s" % (axes, buttons))
    else:
        print(
            "axes_raw: %s | buttons: %s | rc_us: %s" %
            (axes, buttons, ",".join(str(ch) for ch in channels[:8]))
        )


def run(args):
    joystick = open_input_device(args.device)
    joystick.open()
    sock = None
    logger = JsonlLogger(args.log_jsonl)
    send_count = 0
    next_send = time.monotonic()
    next_print = time.monotonic()
    next_log = time.monotonic()
    deadline = None if args.duration <= 0 else time.monotonic() + args.duration
    period = 1.0 / args.rate_hz
    print_period = 1.0 / args.print_hz

    if args.send:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    print("device=%s host=%s port=%d rate=%.1fHz" %
          (args.device, args.host, args.port, args.rate_hz))
    print("Move sticks/switches to identify axes/buttons. Ctrl-C to quit.")
    logger.write(
        "session_start",
        device=args.device,
        host=args.host,
        port=args.port,
        rate_hz=args.rate_hz,
        force_mode_pwm=args.force_mode_pwm,
        force_arm_pwm=args.force_arm_pwm,
        force_kenet_pwm=args.force_kenet_pwm,
    )

    try:
        while True:
            if deadline is not None and time.monotonic() >= deadline:
                break

            events = joystick.poll(timeout=0.02)
            if args.events and events:
                print_events(events)
            if args.changes and events:
                print_changed_events(events, args.show_init)

            channels = make_channels(joystick, CHANNEL_MAP)
            apply_forced_mode_pwm(channels, args.force_mode_pwm)
            apply_forced_channel_pwm(channels, ARM_CH, args.force_arm_pwm)
            apply_forced_channel_pwm(channels, KENET_STATE_CH, args.force_kenet_pwm)
            apply_yaw_authority(channels, args.yaw_authority)

            now = time.monotonic()
            if args.send and now >= next_send:
                sent = sock.sendto(pack_rc_packet(channels), (args.host, args.port))
                send_count += 1
                if args.verbose and (send_count == 1 or now >= next_print):
                    print("sent #%d bytes=%d to %s:%d rc_us=%s" % (
                        send_count,
                        sent,
                        args.host,
                        args.port,
                        ",".join(str(ch) for ch in channels[:8]),
                    ))
                    next_print = now + print_period
                next_send = now + period
                if args.once:
                    break

            if args.dry_run and (events or now >= next_print):
                print_state(joystick, channels)
                next_print = now + print_period

            if args.log_jsonl and (events or now >= next_log):
                logger.write("rc_sample", **rc_snapshot(joystick, channels, events, send_count))
                next_log = now + print_period

            time.sleep(0.005)
    except KeyboardInterrupt:
        pass
    finally:
        if sock:
            sock.close()
        logger.close()
        joystick.close()


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--device", default="/dev/input/js0")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=9004)
    parser.add_argument("--rate-hz", type=float, default=50.0)
    parser.add_argument("--print-hz", type=float, default=5.0,
                        help="Dry-run status print rate when no events arrive")
    parser.add_argument("--dry-run", action="store_true",
                        help="Print live axis/button/RC values")
    parser.add_argument("--send", action="store_true",
                        help="Send RC packets to Betaflight SITL UDP port")
    parser.add_argument("--events", action="store_true",
                        help="Print raw joystick events as they arrive")
    parser.add_argument("--changes", action="store_true",
                        help="Print only non-init axis/button changes")
    parser.add_argument("--show-init", action="store_true",
                        help="Include init events in --changes output")
    parser.add_argument("--verbose", action="store_true",
                        help="Print send status while --send is active")
    parser.add_argument("--once", action="store_true",
                        help="Send one packet and exit")
    parser.add_argument("--duration", type=float, default=0.0,
                        help="Stop after N seconds; 0 means run until Ctrl-C")
    parser.add_argument("--force-mode-pwm", type=int, default=None,
                        help="Force CH7/AUX3 to this PWM value, e.g. 1500 for ANGLE mode tests")
    parser.add_argument("--force-arm-pwm", type=int, default=None,
                        help="Force CH5/AUX1 ARM to this PWM value when the physical ARM switch is absent")
    parser.add_argument("--force-kenet-pwm", type=int, default=None,
                        help="Force CH6/AUX2 Kenet state to this PWM value when the physical state switch is absent")
    parser.add_argument("--yaw-authority", type=int, default=0,
                        help="Clamp yaw stick to center +/- this (us). 0 = no "
                             "clamp. Small value (e.g. 10) keeps manual RC yaw "
                             "in the stable window; the plant spins on real yaw.")
    parser.add_argument("--log-jsonl",
                        help="Write raw joystick axes/buttons and RC channels to this JSONL file")
    args = parser.parse_args()
    if not args.dry_run and not args.send and not args.events and not args.changes:
        parser.error("choose --dry-run, --send, --events, and/or --changes")
    if args.rate_hz <= 0:
        parser.error("--rate-hz must be positive")
    if args.print_hz <= 0:
        parser.error("--print-hz must be positive")
    if args.force_mode_pwm is not None and not 1000 <= args.force_mode_pwm <= 2000:
        parser.error("--force-mode-pwm must be between 1000 and 2000")
    if args.yaw_authority < 0:
        parser.error("--yaw-authority must be >= 0")
    if args.force_arm_pwm is not None and not 1000 <= args.force_arm_pwm <= 2000:
        parser.error("--force-arm-pwm must be between 1000 and 2000")
    if args.force_kenet_pwm is not None and not 1000 <= args.force_kenet_pwm <= 2000:
        parser.error("--force-kenet-pwm must be between 1000 and 2000")
    return args


if __name__ == "__main__":
    run(parse_args())
