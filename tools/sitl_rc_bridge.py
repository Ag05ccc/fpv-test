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
import os
import select
import socket
import struct
import time

from sitl_rc_channels import AUTOPILOT_MODE_CHANNEL


JS_EVENT_FORMAT = "<IhBB"
JS_EVENT_SIZE = struct.calcsize(JS_EVENT_FORMAT)
JS_EVENT_BUTTON = 0x01
JS_EVENT_AXIS = 0x02
JS_EVENT_INIT = 0x80
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


def pack_rc_packet(channels):
    return struct.pack("<d16H", time.time(), *channels)


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
    joystick = LinuxJoystick(args.device)
    joystick.open()
    sock = None
    send_count = 0
    next_send = time.monotonic()
    next_print = time.monotonic()
    deadline = None if args.duration <= 0 else time.monotonic() + args.duration
    period = 1.0 / args.rate_hz
    print_period = 1.0 / args.print_hz

    if args.send:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    print("device=%s host=%s port=%d rate=%.1fHz" %
          (args.device, args.host, args.port, args.rate_hz))
    print("Move sticks/switches to identify axes/buttons. Ctrl-C to quit.")

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

            time.sleep(0.005)
    except KeyboardInterrupt:
        pass
    finally:
        if sock:
            sock.close()
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
    args = parser.parse_args()
    if not args.dry_run and not args.send and not args.events and not args.changes:
        parser.error("choose --dry-run, --send, --events, and/or --changes")
    if args.rate_hz <= 0:
        parser.error("--rate-hz must be positive")
    if args.print_hz <= 0:
        parser.error("--print-hz must be positive")
    if args.force_mode_pwm is not None and not 1000 <= args.force_mode_pwm <= 2000:
        parser.error("--force-mode-pwm must be between 1000 and 2000")
    return args


if __name__ == "__main__":
    run(parse_args())
