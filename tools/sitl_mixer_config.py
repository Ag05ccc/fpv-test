#!/usr/bin/env python3
"""Read or set Betaflight SITL mixer config over MSP/TCP."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path


SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from sitl_msp import (  # noqa: E402
    MSP_EEPROM_WRITE,
    MSP_MIXER_CONFIG,
    MSP_SET_MIXER_CONFIG,
    encode_mixer_config,
    msp_command,
    msp_request,
    parse_mixer_config,
)


def parse_on_off(value: str) -> bool:
    normalized = value.strip().lower()
    if normalized in ("1", "on", "true", "yes"):
        return True
    if normalized in ("0", "off", "false", "no"):
        return False
    raise argparse.ArgumentTypeError("expected on/off, true/false, or 1/0")


def read_mixer_config(host: str, port: int, timeout: float) -> dict:
    config = parse_mixer_config(msp_request(host, port, MSP_MIXER_CONFIG, timeout))
    if not config.get("valid"):
        raise RuntimeError("invalid MSP_MIXER_CONFIG payload")
    return config


def format_config(label: str, config: dict) -> str:
    yaw = config.get("yaw_motors_reversed")
    yaw_text = "ON" if yaw else "OFF"
    return "%s mixer_mode=%s yaw_motors_reversed=%s" % (label, config.get("mixer_mode"), yaw_text)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=5761)
    parser.add_argument("--timeout", type=float, default=2.0)
    parser.add_argument("--yaw-motors-reversed", type=parse_on_off, default=None,
                        help="Set yaw_motors_reversed while preserving the current mixer mode")
    parser.add_argument("--save", action="store_true", help="Persist with MSP_EEPROM_WRITE")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    before = read_mixer_config(args.host, args.port, args.timeout)
    print(format_config("before", before))

    if args.yaw_motors_reversed is not None:
        payload = encode_mixer_config(before["mixer_mode"], args.yaw_motors_reversed)
        msp_command(args.host, args.port, MSP_SET_MIXER_CONFIG, payload, args.timeout)
        after = read_mixer_config(args.host, args.port, args.timeout)
        print(format_config("after", after))
        if after["yaw_motors_reversed"] != args.yaw_motors_reversed:
            print("result=FAIL yaw_motors_reversed did not update", file=sys.stderr)
            return 1
        if args.save:
            msp_command(args.host, args.port, MSP_EEPROM_WRITE, b"", args.timeout)
            print("saved with MSP_EEPROM_WRITE")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
