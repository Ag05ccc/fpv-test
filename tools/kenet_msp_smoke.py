#!/usr/bin/env python3
"""Smoke-test Kenet's production MSP transport against FC or Betaflight SITL."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from kenet.msp import MSPConnection  # noqa: E402
from kenet.rc_channels import pilot_to_msp_rc_channels  # noqa: E402


def parse_channels(value: str) -> list[int]:
    try:
        channels = [int(part.strip()) for part in value.split(",") if part.strip()]
    except ValueError as exc:
        raise argparse.ArgumentTypeError("channels must be comma-separated integers") from exc
    if not 8 <= len(channels) <= 16:
        raise argparse.ArgumentTypeError("expected 8..16 RC channels")
    bad = [channel for channel in channels if not 1000 <= channel <= 2000]
    if bad:
        raise argparse.ArgumentTypeError("RC channels must be in 1000..2000")
    return channels


def resolve_port(args: argparse.Namespace) -> str:
    if args.msp_tcp:
        return args.msp_tcp if args.msp_tcp.startswith("tcp://") else "tcp://%s" % args.msp_tcp
    return args.port


def expected_msp_rc_readback(pilot_channels: list[int]) -> list[int]:
    """Return expected MSP_RC order after sending pilot/AETR channels."""
    return pilot_to_msp_rc_channels(pilot_channels)


def rc_readback_mismatches(pilot_channels: list[int], rc_after: list[int] | None) -> list[str]:
    if not rc_after:
        return ["MSP_RC readback missing after MSP_SET_RAW_RC"]
    expected = expected_msp_rc_readback(pilot_channels)
    if len(rc_after) < len(expected):
        return ["MSP_RC readback shorter than sent frame: got %d, expected at least %d" %
                (len(rc_after), len(expected))]
    mismatches = []
    for index, expected_value in enumerate(expected):
        actual_value = rc_after[index]
        if actual_value != expected_value:
            mismatches.append("MSP_RC[%d] expected %d, got %d" %
                              (index, expected_value, actual_value))
    return mismatches


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="/dev/ttyAMA0",
                        help="FC serial port, or host:port/tcp://host:port")
    parser.add_argument("--msp-tcp", default=None,
                        help="Betaflight SITL MSP TCP endpoint, e.g. 127.0.0.1:5761")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--timeout", type=float, default=1.0)
    parser.add_argument("--set-raw-rc", type=parse_channels, default=None,
                        help="Optional comma-separated 8..16 channel MSP_SET_RAW_RC frame")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    conn = MSPConnection(resolve_port(args), args.baudrate, args.timeout)
    try:
        conn.connect()
        api = conn.get_api_version()
        rc = conn.get_rc_channels()
        attitude = conn.get_attitude()
        if args.set_raw_rc is not None:
            conn.send_rc(args.set_raw_rc)
            rc_after = conn.get_rc_channels()
        else:
            rc_after = None
    finally:
        conn.disconnect()

    failures = []
    if api is None:
        failures.append("MSP_API_VERSION failed")
    if not rc:
        failures.append("MSP_RC failed")
    if attitude is None:
        failures.append("MSP_ATTITUDE failed")
    if args.set_raw_rc is not None:
        readback_failures = rc_readback_mismatches(args.set_raw_rc, rc_after)
        failures.extend(readback_failures)
    else:
        readback_failures = []

    print("MSP smoke: %s" % ("PASS" if not failures else "FAIL"))
    print("port: %s" % resolve_port(args))
    print("api: %s" % (api if api is not None else "-"))
    print("rc: %s" % (rc if rc else "-"))
    print("attitude: %s" % (attitude if attitude is not None else "-"))
    if args.set_raw_rc is not None:
        print("set_raw_rc: %s" % args.set_raw_rc)
        print("set_raw_rc_expected_msp_rc: %s" %
              expected_msp_rc_readback(args.set_raw_rc))
        print("rc_after: %s" % (rc_after if rc_after else "-"))
        print("set_raw_rc_check: %s" % ("PASS" if not readback_failures else "FAIL"))
    if failures:
        for failure in failures:
            print("failure: %s" % failure)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
