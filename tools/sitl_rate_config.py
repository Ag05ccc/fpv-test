#!/usr/bin/env python3
"""Read or set Betaflight SITL RC/rate-profile values over MSP/TCP."""

from __future__ import annotations

import argparse
import struct
import sys
from pathlib import Path
from typing import Any


SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from sitl_msp import MSP_EEPROM_WRITE, MSP_RC_TUNING, MSP_SET_RC_TUNING, msp_command, msp_request  # noqa: E402


RATE_FIELDS = (
    "roll_rc_rate",
    "roll_rc_expo",
    "roll_rate",
    "pitch_rate",
    "yaw_rate",
    "tpa_rate",
    "thr_mid",
    "thr_expo",
    "tpa_breakpoint",
    "yaw_rc_expo",
    "yaw_rc_rate",
    "pitch_rc_rate",
    "pitch_rc_expo",
    "throttle_limit_type",
    "throttle_limit_percent",
    "roll_rate_limit",
    "pitch_rate_limit",
    "yaw_rate_limit",
    "rates_type",
    "thr_hover",
)


def parse_rate_payload(payload: bytes) -> dict[str, Any]:
    if len(payload) < 10:
        raise ValueError("MSP_RC_TUNING payload too short")
    config: dict[str, Any] = {"payload_len": len(payload)}
    config["roll_rc_rate"] = payload[0]
    config["roll_rc_expo"] = payload[1]
    config["roll_rate"] = payload[2]
    config["pitch_rate"] = payload[3]
    config["yaw_rate"] = payload[4]
    config["tpa_rate"] = payload[5]
    config["thr_mid"] = payload[6]
    config["thr_expo"] = payload[7]
    config["tpa_breakpoint"] = struct.unpack_from("<H", payload, 8)[0]
    if len(payload) >= 11:
        config["yaw_rc_expo"] = payload[10]
    if len(payload) >= 12:
        config["yaw_rc_rate"] = payload[11]
    if len(payload) >= 13:
        config["pitch_rc_rate"] = payload[12]
    if len(payload) >= 14:
        config["pitch_rc_expo"] = payload[13]
    if len(payload) >= 16:
        config["throttle_limit_type"] = payload[14]
        config["throttle_limit_percent"] = payload[15]
    if len(payload) >= 22:
        config["roll_rate_limit"] = struct.unpack_from("<H", payload, 16)[0]
        config["pitch_rate_limit"] = struct.unpack_from("<H", payload, 18)[0]
        config["yaw_rate_limit"] = struct.unpack_from("<H", payload, 20)[0]
    if len(payload) >= 23:
        config["rates_type"] = payload[22]
    if len(payload) >= 24:
        config["thr_hover"] = payload[23]
    return config


def _byte(config: dict[str, Any], key: str) -> int:
    value = int(config[key])
    if not 0 <= value <= 255:
        raise ValueError("%s must fit in one byte" % key)
    return value


def _u16(config: dict[str, Any], key: str) -> bytes:
    value = int(config[key])
    if not 0 <= value <= 65535:
        raise ValueError("%s must fit in uint16" % key)
    return struct.pack("<H", value)


def encode_rate_payload(config: dict[str, Any]) -> bytes:
    payload_len = int(config.get("payload_len", 24))
    if payload_len < 10:
        raise ValueError("payload_len must be at least 10")
    payload = bytearray()
    for key in ("roll_rc_rate", "roll_rc_expo", "roll_rate", "pitch_rate", "yaw_rate", "tpa_rate", "thr_mid", "thr_expo"):
        payload.append(_byte(config, key))
    payload += _u16(config, "tpa_breakpoint")
    optional_layout = [
        ("yaw_rc_expo", "byte"),
        ("yaw_rc_rate", "byte"),
        ("pitch_rc_rate", "byte"),
        ("pitch_rc_expo", "byte"),
        ("throttle_limit_type", "byte"),
        ("throttle_limit_percent", "byte"),
        ("roll_rate_limit", "u16"),
        ("pitch_rate_limit", "u16"),
        ("yaw_rate_limit", "u16"),
        ("rates_type", "byte"),
        ("thr_hover", "byte"),
    ]
    for key, kind in optional_layout:
        if len(payload) >= payload_len:
            break
        if key not in config:
            raise ValueError("config missing %s for payload_len=%d" % (key, payload_len))
        if kind == "byte":
            payload.append(_byte(config, key))
        else:
            payload += _u16(config, key)
    if len(payload) != payload_len:
        raise ValueError("encoded payload length %d != %d" % (len(payload), payload_len))
    return bytes(payload)


def update_rate_config(
    config: dict[str, Any],
    *,
    yaw_rc_rate: int | None = None,
    yaw_rate: int | None = None,
    yaw_rate_limit: int | None = None,
    pitch_rc_rate: int | None = None,
    pitch_rate: int | None = None,
    pitch_rate_limit: int | None = None,
) -> dict[str, Any]:
    updated = dict(config)
    if yaw_rc_rate is not None:
        updated["yaw_rc_rate"] = yaw_rc_rate
    if yaw_rate is not None:
        updated["yaw_rate"] = yaw_rate
    if yaw_rate_limit is not None:
        updated["yaw_rate_limit"] = yaw_rate_limit
    if pitch_rc_rate is not None:
        updated["pitch_rc_rate"] = pitch_rc_rate
    if pitch_rate is not None:
        updated["pitch_rate"] = pitch_rate
    if pitch_rate_limit is not None:
        updated["pitch_rate_limit"] = pitch_rate_limit
    return updated


def read_rate_config(host: str, port: int, timeout: float) -> dict[str, Any]:
    return parse_rate_payload(msp_request(host, port, MSP_RC_TUNING, timeout))


def format_rate_config(label: str, config: dict[str, Any]) -> str:
    return (
        "%s yaw_rc_rate=%s yaw_rate=%s yaw_rate_limit=%s "
        "pitch_rc_rate=%s pitch_rate=%s pitch_rate_limit=%s "
        "roll_limit=%s pitch_limit=%s rates_type=%s"
    ) % (
        label,
        config.get("yaw_rc_rate", "-"),
        config.get("yaw_rate", "-"),
        config.get("yaw_rate_limit", "-"),
        config.get("pitch_rc_rate", "-"),
        config.get("pitch_rate", "-"),
        config.get("pitch_rate_limit", "-"),
        config.get("roll_rate_limit", "-"),
        config.get("pitch_rate_limit", "-"),
        config.get("rates_type", "-"),
    )


def parse_byte_value(value: str) -> int:
    try:
        parsed = int(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("value must be an integer") from exc
    if not 0 <= parsed <= 255:
        raise argparse.ArgumentTypeError("value must be 0..255")
    return parsed


def parse_u16_value(value: str) -> int:
    try:
        parsed = int(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("value must be an integer") from exc
    if not 0 <= parsed <= 65535:
        raise argparse.ArgumentTypeError("value must be 0..65535")
    return parsed


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=5761)
    parser.add_argument("--timeout", type=float, default=2.0)
    parser.add_argument("--yaw-rc-rate", type=parse_byte_value, default=None)
    parser.add_argument("--yaw-rate", type=parse_byte_value, default=None,
                        help="Betaflight yaw super-rate value")
    parser.add_argument("--yaw-rate-limit", type=parse_u16_value, default=None,
                        help="Betaflight yaw rate_limit in deg/s; 0 disables limit")
    parser.add_argument("--pitch-rc-rate", type=parse_byte_value, default=None)
    parser.add_argument("--pitch-rate", type=parse_byte_value, default=None,
                        help="Betaflight pitch super-rate value")
    parser.add_argument("--pitch-rate-limit", type=parse_u16_value, default=None,
                        help="Betaflight pitch rate_limit in deg/s; 0 disables limit")
    parser.add_argument("--save", action="store_true", help="Persist with MSP_EEPROM_WRITE")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    before = read_rate_config(args.host, args.port, args.timeout)
    print(format_rate_config("before", before))
    fields = (
        "yaw_rc_rate",
        "yaw_rate",
        "yaw_rate_limit",
        "pitch_rc_rate",
        "pitch_rate",
        "pitch_rate_limit",
    )
    if any(getattr(args, key) is not None for key in fields):
        desired = update_rate_config(
            before,
            yaw_rc_rate=args.yaw_rc_rate,
            yaw_rate=args.yaw_rate,
            yaw_rate_limit=args.yaw_rate_limit,
            pitch_rc_rate=args.pitch_rc_rate,
            pitch_rate=args.pitch_rate,
            pitch_rate_limit=args.pitch_rate_limit,
        )
        msp_command(args.host, args.port, MSP_SET_RC_TUNING, encode_rate_payload(desired), args.timeout)
        after = read_rate_config(args.host, args.port, args.timeout)
        print(format_rate_config("after", after))
        for key in fields:
            expected = desired.get(key)
            if getattr(args, key) is not None and after.get(key) != expected:
                print("result=FAIL %s did not update" % key, file=sys.stderr)
                return 1
        if args.save:
            msp_command(args.host, args.port, MSP_EEPROM_WRITE, b"", args.timeout)
            print("saved with MSP_EEPROM_WRITE")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
