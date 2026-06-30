#!/usr/bin/env python3
"""Read or set Betaflight SITL PID P/I/D values over MSP/TCP."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import Any


SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from sitl_msp import MSP_EEPROM_WRITE, MSP_PID, MSP_SET_PID, msp_command, msp_request  # noqa: E402


PID_NAMES = ["roll", "pitch", "yaw", "level", "mag"]


def parse_pid_payload(payload: bytes) -> list[dict[str, Any]]:
    items: list[dict[str, Any]] = []
    for index, offset in enumerate(range(0, len(payload) - 2, 3)):
        p, i, d = payload[offset:offset + 3]
        items.append({
            "index": index,
            "name": PID_NAMES[index] if index < len(PID_NAMES) else "pid%d" % index,
            "p": p,
            "i": i,
            "d": d,
        })
    return items


def encode_pid_payload(items: list[dict[str, Any]]) -> bytes:
    payload = bytearray()
    for item in items:
        for key in ("p", "i", "d"):
            value = int(item[key])
            if not 0 <= value <= 255:
                raise ValueError("%s %s must fit in one byte" % (item.get("name", "pid"), key))
            payload.append(value)
    return bytes(payload)


def update_axis_pid(
    items: list[dict[str, Any]],
    axis_name: str,
    *,
    p: int | None = None,
    i: int | None = None,
    d: int | None = None,
) -> list[dict[str, Any]]:
    updated = [dict(item) for item in items]
    for item in updated:
        if item["name"] != axis_name:
            continue
        if p is not None:
            item["p"] = p
        if i is not None:
            item["i"] = i
        if d is not None:
            item["d"] = d
        return updated
    raise ValueError("PID axis not found: %s" % axis_name)


def read_pid_config(host: str, port: int, timeout: float) -> list[dict[str, Any]]:
    items = parse_pid_payload(msp_request(host, port, MSP_PID, timeout))
    if len(items) < 3:
        raise RuntimeError("invalid MSP_PID payload")
    return items


def format_pid(label: str, items: list[dict[str, Any]]) -> str:
    parts = [
        "%s=%d/%d/%d" % (item["name"], item["p"], item["i"], item["d"])
        for item in items
    ]
    return "%s %s" % (label, " ".join(parts))


def parse_pid_value(value: str) -> int:
    try:
        parsed = int(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("PID value must be an integer") from exc
    if not 0 <= parsed <= 255:
        raise argparse.ArgumentTypeError("PID value must be 0..255")
    return parsed


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=5761)
    parser.add_argument("--timeout", type=float, default=2.0)
    parser.add_argument("--yaw-p", type=parse_pid_value, default=None)
    parser.add_argument("--yaw-i", type=parse_pid_value, default=None)
    parser.add_argument("--yaw-d", type=parse_pid_value, default=None)
    parser.add_argument("--pitch-p", type=parse_pid_value, default=None)
    parser.add_argument("--pitch-i", type=parse_pid_value, default=None)
    parser.add_argument("--pitch-d", type=parse_pid_value, default=None)
    parser.add_argument("--zero-yaw", action="store_true", help="Set yaw P/I/D to 0/0/0")
    parser.add_argument("--zero-pitch", action="store_true", help="Set pitch P/I/D to 0/0/0")
    parser.add_argument("--save", action="store_true", help="Persist with MSP_EEPROM_WRITE")
    args = parser.parse_args()
    if args.zero_yaw:
        args.yaw_p = 0
        args.yaw_i = 0
        args.yaw_d = 0
    if args.zero_pitch:
        args.pitch_p = 0
        args.pitch_i = 0
        args.pitch_d = 0
    return args


def main() -> int:
    args = parse_args()
    before = read_pid_config(args.host, args.port, args.timeout)
    print(format_pid("before", before))

    desired = before
    requested_axes: list[tuple[str, tuple[int | None, int | None, int | None]]] = []
    if any(value is not None for value in (args.pitch_p, args.pitch_i, args.pitch_d)):
        requested_axes.append(("pitch", (args.pitch_p, args.pitch_i, args.pitch_d)))
        desired = update_axis_pid(desired, "pitch", p=args.pitch_p, i=args.pitch_i, d=args.pitch_d)
    if any(value is not None for value in (args.yaw_p, args.yaw_i, args.yaw_d)):
        requested_axes.append(("yaw", (args.yaw_p, args.yaw_i, args.yaw_d)))
        desired = update_axis_pid(desired, "yaw", p=args.yaw_p, i=args.yaw_i, d=args.yaw_d)

    if requested_axes:
        msp_command(args.host, args.port, MSP_SET_PID, encode_pid_payload(desired), args.timeout)
        after = read_pid_config(args.host, args.port, args.timeout)
        print(format_pid("after", after))
        expected_by_axis = {item["name"]: item for item in desired}
        actual_by_axis = {item["name"]: item for item in after}
        for axis_name, _ in requested_axes:
            expected = expected_by_axis[axis_name]
            actual = actual_by_axis[axis_name]
            for key in ("p", "i", "d"):
                if actual[key] != expected[key]:
                    print("result=FAIL %s %s did not update" % (axis_name, key), file=sys.stderr)
                    return 1
        if args.save:
            msp_command(args.host, args.port, MSP_EEPROM_WRITE, b"", args.timeout)
            print("saved with MSP_EEPROM_WRITE")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
