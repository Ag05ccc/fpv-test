#!/usr/bin/env python3
"""Check the physical RC switch contract before re-enabling joystick SITL tests."""

from __future__ import annotations

import argparse
import os
import sys
import time
from pathlib import Path


SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from sitl_rc_bridge import CHANNEL_MAP, apply_forced_mode_pwm, make_channels, open_input_device  # noqa: E402
from sitl_rc_channels import ARM_CH, AUTOPILOT_MODE_CHANNEL, KENET_STATE_CH, channel_label  # noqa: E402


LOW = "low"
MID = "mid"
HIGH = "high"

SWITCH_EXPECTATIONS = [
    {
        "role": "ARM",
        "map_key": "arm_aux1",
        "channel": ARM_CH,
        "expected_positions": (LOW, HIGH),
        "purpose": "Betaflight ARM only",
    },
    {
        "role": "Kenet state",
        "map_key": "mode_aux2",
        "channel": KENET_STATE_CH,
        "expected_positions": (LOW, MID, HIGH),
        "purpose": "Kenet IDLE / AI-ARMED / TRACKING",
    },
    {
        "role": "Autopilot mode",
        "map_key": "autopilot_mode_aux3",
        "channel": AUTOPILOT_MODE_CHANNEL,
        "expected_positions": (LOW, MID, HIGH),
        "purpose": "Betaflight flight mode, ANGLE at mid/1500",
    },
]


def pwm_bucket(value: int) -> str:
    if value <= 1250:
        return LOW
    if value >= 1750:
        return HIGH
    return MID


def validate_channel_map_contract(mapping: dict = CHANNEL_MAP) -> list[str]:
    failures: list[str] = []
    for expectation in SWITCH_EXPECTATIONS:
        cfg = mapping.get(expectation["map_key"])
        if cfg is None:
            failures.append("%s mapping missing" % expectation["map_key"])
            continue
        if cfg.get("channel") != expectation["channel"]:
            failures.append(
                "%s channel=%s expected=%s"
                % (expectation["map_key"], cfg.get("channel"), expectation["channel"])
            )
        if len(expectation["expected_positions"]) == 2 and not cfg.get("two_pos"):
            failures.append("%s must be two_pos" % expectation["map_key"])
        if len(expectation["expected_positions"]) == 3 and not cfg.get("three_pos"):
            failures.append("%s must be three_pos" % expectation["map_key"])
    return failures


def evaluate_switch_observations(
    observed: dict[int, set[str]],
    force_mode_pwm: int | None = None,
) -> list[dict[str, object]]:
    results: list[dict[str, object]] = []
    for expectation in SWITCH_EXPECTATIONS:
        channel = expectation["channel"]
        expected = set(expectation["expected_positions"])
        seen = set(observed.get(channel, set()))
        forced = channel == AUTOPILOT_MODE_CHANNEL and force_mode_pwm is not None
        if forced:
            forced_position = pwm_bucket(force_mode_pwm)
            seen.add(forced_position)
            expected = {MID}
        missing = sorted(expected - seen)
        results.append(
            {
                "role": expectation["role"],
                "channel": channel,
                "label": channel_label(channel),
                "purpose": expectation["purpose"],
                "expected": sorted(expected),
                "seen": sorted(seen),
                "missing": missing,
                "forced": forced,
                "ok": not missing,
            }
        )
    return results


def collect_observations(args: argparse.Namespace) -> dict[int, set[str]]:
    joystick = open_input_device(args.device)
    joystick.open()
    deadline = time.monotonic() + args.duration
    observed: dict[int, set[str]] = {item["channel"]: set() for item in SWITCH_EXPECTATIONS}
    next_print = time.monotonic()
    try:
        while time.monotonic() < deadline:
            joystick.poll(timeout=0.02)
            channels = make_channels(joystick, CHANNEL_MAP)
            apply_forced_mode_pwm(channels, args.force_mode_pwm)
            for channel in observed:
                observed[channel].add(pwm_bucket(channels[channel]))
            now = time.monotonic()
            if args.verbose and now >= next_print:
                print("rc_us=%s" % ",".join(str(value) for value in channels[:8]))
                next_print = now + 1.0
            time.sleep(0.01)
    finally:
        joystick.close()
    return observed


def print_instructions(args: argparse.Namespace) -> None:
    print("device=%s duration=%.1fs" % (args.device, args.duration))
    print("Move these controls during the window:")
    print("  CH5/AUX1 ARM: low and high")
    print("  CH6/AUX2 Kenet state: low, mid, high")
    if args.force_mode_pwm is None:
        print("  CH7/AUX3 mode: low, mid, high (mid/1500 is ANGLE)")
    else:
        print("  CH7/AUX3 mode: forced to %d by --force-mode-pwm" % args.force_mode_pwm)


def print_results(results: list[dict[str, object]]) -> None:
    for result in results:
        status = "PASS" if result["ok"] else "FAIL"
        forced = " forced" if result["forced"] else ""
        print(
            "%s %s %s%s expected=%s seen=%s purpose=%s"
            % (
                status,
                result["label"],
                result["role"],
                forced,
                ",".join(result["expected"]),
                ",".join(result["seen"]) if result["seen"] else "-",
                result["purpose"],
            )
        )
        if result["missing"]:
            print("  missing=%s" % ",".join(result["missing"]))


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--device", default="/dev/input/js0")
    parser.add_argument("--duration", type=float, default=12.0)
    parser.add_argument("--force-mode-pwm", type=int, default=None,
                        help="Match sitl_rc_bridge.py CH7 forcing, usually 1500 for ANGLE")
    parser.add_argument("--verbose", action="store_true")
    args = parser.parse_args(argv)
    if args.duration <= 0:
        parser.error("--duration must be positive")
    if args.force_mode_pwm is not None and not 1000 <= args.force_mode_pwm <= 2000:
        parser.error("--force-mode-pwm must be between 1000 and 2000")
    return args


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    contract_failures = validate_channel_map_contract()
    if contract_failures:
        print("mapping contract FAIL:")
        for failure in contract_failures:
            print("  - %s" % failure)
        return 1
    if not os.path.exists(args.device):
        print("device not found: %s" % args.device)
        print("physical RC remains isolated; plug in the transmitter or keep using virtual RC")
        return 2
    print_instructions(args)
    observed = collect_observations(args)
    results = evaluate_switch_observations(observed, args.force_mode_pwm)
    print_results(results)
    return 0 if all(result["ok"] for result in results) else 1


if __name__ == "__main__":
    raise SystemExit(main())
