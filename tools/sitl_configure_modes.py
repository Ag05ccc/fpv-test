#!/usr/bin/env python3
"""Configure Betaflight SITL mode ranges over MSP/TCP."""

from __future__ import annotations

import argparse
import socket
import sys
from dataclasses import dataclass
from pathlib import Path


SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from sitl_msp import msp_encode, read_msp_frame


MSP_MODE_RANGES = 34
MSP_SET_MODE_RANGE = 35
MSP_EEPROM_WRITE = 250

CHANNEL_RANGE_MIN = 900
CHANNEL_STEP_US = 25


@dataclass(frozen=True)
class ModeRange:
    index: int
    permanent_id: int
    aux_channel_index: int
    start_us: int
    end_us: int

    @property
    def start_step(self) -> int:
        return us_to_step(self.start_us)

    @property
    def end_step(self) -> int:
        return us_to_step(self.end_us)

    def payload(self) -> bytes:
        return bytes([
            self.index,
            self.permanent_id,
            self.aux_channel_index,
            self.start_step,
            self.end_step,
        ])

    def is_usable(self) -> bool:
        return self.start_step < self.end_step


def us_to_step(value: int) -> int:
    delta = value - CHANNEL_RANGE_MIN
    if delta < 0 or delta % CHANNEL_STEP_US:
        raise ValueError("mode range value must be 900 + 25*n, got %d" % value)
    step = delta // CHANNEL_STEP_US
    if not 0 <= step <= 48:
        raise ValueError("mode range step must be between 0 and 48, got %d" % step)
    return step


def step_to_us(step: int) -> int:
    return CHANNEL_RANGE_MIN + step * CHANNEL_STEP_US


def msp_command(host: str, port: int, code: int, payload: bytes, timeout: float) -> bytes:
    with socket.create_connection((host, port), timeout=timeout) as sock:
        sock.settimeout(timeout)
        sock.sendall(msp_encode(code, payload))
        for _ in range(10):
            response_code, response_payload = read_msp_frame(sock)
            if response_code == code:
                return response_payload
    raise RuntimeError("MSP response %d not received" % code)


def read_mode_ranges(host: str, port: int, timeout: float) -> list[ModeRange]:
    payload = msp_command(host, port, MSP_MODE_RANGES, b"", timeout)
    ranges: list[ModeRange] = []
    for index, offset in enumerate(range(0, len(payload) - 3, 4)):
        permanent_id, aux_channel_index, start_step, end_step = payload[offset:offset + 4]
        ranges.append(ModeRange(
            index=index,
            permanent_id=permanent_id,
            aux_channel_index=aux_channel_index,
            start_us=step_to_us(start_step),
            end_us=step_to_us(end_step),
        ))
    return ranges


def default_ranges(args: argparse.Namespace) -> list[ModeRange]:
    return [
        ModeRange(0, 0, 0, args.arm_start, args.arm_end),  # ARM on AUX1 / CH5
        ModeRange(1, 50, 1, args.msp_override_start, args.msp_override_end),  # MSP Override on AUX2 / CH6
        ModeRange(2, 1, 2, args.angle_start, args.angle_end),  # ANGLE on AUX3 / CH7
        ModeRange(3, 2, 2, args.horizon_start, args.horizon_end),  # HORIZON on AUX3 / CH7 high
    ]


def clear_ranges(count: int) -> list[ModeRange]:
    return [ModeRange(index, 0, 0, 900, 900) for index in range(count)]


def format_range(item: ModeRange) -> str:
    usable = "on" if item.is_usable() else "off"
    return (
        "index=%d permanent_id=%d aux=%d range=%d-%d %s" %
        (item.index, item.permanent_id, item.aux_channel_index, item.start_us, item.end_us, usable)
    )


def apply_ranges(args: argparse.Namespace, ranges: list[ModeRange]) -> None:
    for item in ranges:
        if args.dry_run:
            print("dry-run set %s payload=%s" % (format_range(item), item.payload().hex()))
            continue
        msp_command(args.host, args.port, MSP_SET_MODE_RANGE, item.payload(), args.timeout)
        print("set %s" % format_range(item))


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=5761)
    parser.add_argument("--timeout", type=float, default=2.0)
    parser.add_argument("--clear-count", type=int, default=10)
    parser.add_argument("--arm-start", type=int, default=1600)
    parser.add_argument("--arm-end", type=int, default=2100)
    parser.add_argument("--msp-override-start", type=int, default=1700)
    parser.add_argument("--msp-override-end", type=int, default=2100)
    parser.add_argument("--angle-start", type=int, default=1300)
    parser.add_argument("--angle-end", type=int, default=1700)
    parser.add_argument("--horizon-start", type=int, default=1700)
    parser.add_argument("--horizon-end", type=int, default=2100)
    parser.add_argument("--save", action="store_true", help="Persist mode ranges with MSP_EEPROM_WRITE")
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()
    if args.clear_count < 4:
        parser.error("--clear-count must be at least 4")
    for name in (
        "arm_start",
        "arm_end",
        "msp_override_start",
        "msp_override_end",
        "angle_start",
        "angle_end",
        "horizon_start",
        "horizon_end",
    ):
        try:
            us_to_step(getattr(args, name))
        except ValueError as exc:
            parser.error("--%s: %s" % (name.replace("_", "-"), exc))
    return args


def main() -> int:
    args = parse_args()
    desired = clear_ranges(args.clear_count)
    # Override cleared slots 0..3 with the configured ranges.
    desired[0:4] = default_ranges(args)

    if not args.dry_run:
        before = [item for item in read_mode_ranges(args.host, args.port, args.timeout) if item.is_usable()]
        print("before usable ranges:")
        for item in before:
            print("  %s" % format_range(item))
        if not before:
            print("  -")

    apply_ranges(args, desired)

    if args.save and not args.dry_run:
        msp_command(args.host, args.port, MSP_EEPROM_WRITE, b"", args.timeout)
        print("saved with MSP_EEPROM_WRITE")

    if not args.dry_run:
        after = [item for item in read_mode_ranges(args.host, args.port, args.timeout) if item.is_usable()]
        print("after usable ranges:")
        for item in after:
            print("  %s" % format_range(item))
        if not after:
            print("  -")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
