#!/usr/bin/env python3
"""Verify Betaflight SITL ARM/MSP Override/ANGLE/HORIZON modes over MSP."""

from __future__ import annotations

import argparse
import socket
import sys
import time
from dataclasses import dataclass
from pathlib import Path


SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from sitl_msp import (  # noqa: E402
    MSP_BOXIDS,
    MSP_BOXNAMES,
    MSP_STATUS_EX,
    msp_request_many,
    parse_box_ids,
    parse_box_names,
    parse_status_ex,
)
from sitl_rc_bridge import pack_rc_packet  # noqa: E402
from sitl_virtual_rc import make_virtual_channels, rc_summary  # noqa: E402


@dataclass(frozen=True)
class ModeCase:
    name: str
    arm_pwm: int
    kenet_pwm: int
    mode_pwm: int
    expected_active: tuple[str, ...]
    expected_inactive: tuple[str, ...]

    def channels(self) -> list[int]:
        return make_virtual_channels(
            throttle=1000,
            arm_pwm=self.arm_pwm,
            kenet_pwm=self.kenet_pwm,
            mode_pwm=self.mode_pwm,
        )


def default_cases() -> list[ModeCase]:
    return [
        ModeCase(
            name="all-low",
            arm_pwm=1000,
            kenet_pwm=1000,
            mode_pwm=1000,
            expected_active=(),
            expected_inactive=("ARM", "MSP OVERRIDE", "ANGLE", "HORIZON"),
        ),
        ModeCase(
            name="angle-mid",
            arm_pwm=1000,
            kenet_pwm=1000,
            mode_pwm=1500,
            expected_active=("ANGLE",),
            expected_inactive=("ARM", "MSP OVERRIDE", "HORIZON"),
        ),
        ModeCase(
            name="horizon-high",
            arm_pwm=1000,
            kenet_pwm=1000,
            mode_pwm=2000,
            expected_active=("HORIZON",),
            expected_inactive=("ARM", "MSP OVERRIDE", "ANGLE"),
        ),
        ModeCase(
            name="arm-angle",
            arm_pwm=2000,
            kenet_pwm=1000,
            mode_pwm=1500,
            expected_active=("ARM", "ANGLE"),
            expected_inactive=("MSP OVERRIDE", "HORIZON"),
        ),
        ModeCase(
            name="kenet-tracking-angle",
            arm_pwm=2000,
            kenet_pwm=2000,
            mode_pwm=1500,
            expected_active=("ARM", "MSP OVERRIDE", "ANGLE"),
            expected_inactive=("HORIZON",),
        ),
    ]


def send_channels(channels: list[int], host: str, port: int, seconds: float, rate_hz: float) -> int:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    deadline = time.monotonic() + seconds
    period = 1.0 / rate_hz
    count = 0
    try:
        while time.monotonic() < deadline:
            sock.sendto(pack_rc_packet(channels), (host, port))
            count += 1
            time.sleep(period)
    finally:
        sock.close()
    return count


def read_status(host: str, port: int, timeout: float) -> dict:
    metadata = msp_request_many(host, port, [MSP_BOXNAMES, MSP_BOXIDS], timeout)
    names = parse_box_names(metadata[MSP_BOXNAMES])
    ids = parse_box_ids(metadata[MSP_BOXIDS])
    status_payload = msp_request_many(host, port, [MSP_STATUS_EX], timeout)[MSP_STATUS_EX]
    return parse_status_ex(status_payload, names, ids)


def validate_status(case: ModeCase, status: dict) -> list[str]:
    failures: list[str] = []
    active = set(status.get("active_modes") or [])
    if not status.get("valid"):
        failures.append("MSP_STATUS_EX invalid")
    if not status.get("active_modes_valid"):
        failures.append("BOX metadata unavailable")
    for mode in case.expected_active:
        if mode not in active:
            failures.append("%s expected active" % mode)
    for mode in case.expected_inactive:
        if mode in active:
            failures.append("%s expected inactive" % mode)
    return failures


def run_case(args: argparse.Namespace, case: ModeCase) -> list[str]:
    channels = case.channels()
    count = send_channels(channels, args.rc_host, args.rc_port, args.send_seconds, args.rate_hz)
    status = read_status(args.msp_host, args.msp_port, args.timeout)
    failures = validate_status(case, status)
    active_modes = ",".join(status.get("active_modes") or []) or "-"
    arming_disable = ",".join(status.get("arming_disable_names") or []) or "-"
    result = "PASS" if not failures else "FAIL"
    print(
        "%s case=%s sent=%d rc=%s active=%s arming_disable=%s" %
        (result, case.name, count, rc_summary(channels), active_modes, arming_disable)
    )
    for failure in failures:
        print("  - %s" % failure)
    return failures


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--rc-host", default="127.0.0.1")
    parser.add_argument("--rc-port", type=int, default=9004)
    parser.add_argument("--msp-host", default="127.0.0.1")
    parser.add_argument("--msp-port", type=int, default=5761)
    parser.add_argument("--send-seconds", type=float, default=0.4)
    parser.add_argument("--rate-hz", type=float, default=50.0)
    parser.add_argument("--timeout", type=float, default=2.0)
    args = parser.parse_args(argv)
    if args.send_seconds <= 0:
        parser.error("--send-seconds must be positive")
    if args.rate_hz <= 0:
        parser.error("--rate-hz must be positive")
    return args


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    failures: list[str] = []
    try:
        for case in default_cases():
            failures.extend(run_case(args, case))
    except OSError as exc:
        print("MSP status check failed: %s" % exc)
        print("Start Betaflight SITL and close Betaflight Configurator before retrying.")
        return 2
    except RuntimeError as exc:
        print("MSP status check failed: %s" % exc)
        print("Start Betaflight SITL and close Betaflight Configurator before retrying.")
        return 2
    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(main())
