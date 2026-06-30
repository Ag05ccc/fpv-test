#!/usr/bin/env python3
"""Check Kenet controller pitch/yaw direction signs without SITL hardware."""

from __future__ import annotations

import argparse
import sys
import time
from dataclasses import dataclass
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from kenet.controller import FlightController, PIDGains  # noqa: E402
from kenet.pipeline import PipelineConfig  # noqa: E402
from kenet.tracker import TrackResult  # noqa: E402


@dataclass(frozen=True)
class DirectionCase:
    name: str
    result: TrackResult
    yaw_relation: str
    pitch_relation: str


def compare(value: int, center: int, relation: str) -> bool:
    if relation == "gt":
        return value > center
    if relation == "lt":
        return value < center
    if relation == "eq":
        return value == center
    raise ValueError("unknown relation: %s" % relation)


def relation_symbol(relation: str) -> str:
    return {"gt": ">", "lt": "<", "eq": "="}[relation]


def make_config(args: argparse.Namespace) -> PipelineConfig:
    return PipelineConfig(
        frame_width=args.frame_width,
        frame_height=args.frame_height,
        desired_target_width=args.desired_target_width,
        deadband=args.deadband,
        size_deadband=args.size_deadband,
        max_rc_rate=100000.0,
        gcs_enabled=False,
        yaw_pid=PIDGains(
            kp=args.yaw_kp,
            ki=0.0,
            kd=0.0,
            output_min=-args.yaw_limit,
            output_max=args.yaw_limit,
        ),
        forward_pid=PIDGains(
            kp=args.forward_kp,
            ki=0.0,
            kd=0.0,
            output_min=-args.forward_limit,
            output_max=args.forward_limit,
        ),
    )


def direction_cases(args: argparse.Namespace) -> list[DirectionCase]:
    cx = args.frame_width / 2.0
    cy = args.frame_height / 2.0
    desired = args.desired_target_width
    offset = args.target_offset_px
    size_delta = args.target_size_delta_px
    return [
        DirectionCase(
            "centered",
            TrackResult(found=True, bbox=(0, 0, desired, desired), center=(cx, cy)),
            "eq",
            "eq",
        ),
        DirectionCase(
            "target_right",
            TrackResult(found=True, bbox=(0, 0, desired, desired), center=(cx + offset, cy)),
            "gt",
            "eq",
        ),
        DirectionCase(
            "target_left",
            TrackResult(found=True, bbox=(0, 0, desired, desired), center=(cx - offset, cy)),
            "lt",
            "eq",
        ),
        DirectionCase(
            "target_small",
            TrackResult(found=True, bbox=(0, 0, desired - size_delta, desired - size_delta), center=(cx, cy)),
            "eq",
            "gt",
        ),
        DirectionCase(
            "target_large",
            TrackResult(found=True, bbox=(0, 0, desired + size_delta, desired + size_delta), center=(cx, cy)),
            "eq",
            "lt",
        ),
        DirectionCase(
            "target_lost",
            TrackResult(found=False),
            "eq",
            "eq",
        ),
    ]


def run_case(cfg: PipelineConfig, case: DirectionCase, settle_seconds: float) -> dict[str, object]:
    controller = FlightController(cfg)
    controller.update(case.result)
    if settle_seconds > 0:
        time.sleep(settle_seconds)
    controller.update(case.result)
    channels = controller.channels
    yaw = channels[cfg.yaw_ch]
    pitch = channels[cfg.pitch_ch]
    yaw_ok = compare(yaw, cfg.rc_center, case.yaw_relation)
    pitch_ok = compare(pitch, cfg.rc_center, case.pitch_relation)
    static_ok = (
        channels[cfg.roll_ch] == cfg.rc_center
        and channels[cfg.throttle_ch] == cfg.throttle_neutral
    )
    return {
        "name": case.name,
        "ok": yaw_ok and pitch_ok and static_ok,
        "yaw": yaw,
        "pitch": pitch,
        "roll": channels[cfg.roll_ch],
        "throttle": channels[cfg.throttle_ch],
        "expected_yaw": case.yaw_relation,
        "expected_pitch": case.pitch_relation,
        "yaw_error": controller.yaw_error,
        "forward_error": controller.forward_error,
        "yaw_output": controller.yaw_output,
        "forward_output": controller.forward_output,
    }


def run_checks(args: argparse.Namespace) -> list[dict[str, object]]:
    cfg = make_config(args)
    return [run_case(cfg, case, args.settle_seconds) for case in direction_cases(args)]


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--frame-width", type=int, default=640)
    parser.add_argument("--frame-height", type=int, default=480)
    parser.add_argument("--desired-target-width", type=float, default=120.0)
    parser.add_argument("--target-offset-px", type=float, default=60.0)
    parser.add_argument("--target-size-delta-px", type=float, default=35.0)
    parser.add_argument("--deadband", type=float, default=10.0)
    parser.add_argument("--size-deadband", type=float, default=15.0)
    parser.add_argument("--yaw-kp", type=float, default=0.8)
    parser.add_argument("--forward-kp", type=float, default=0.4)
    parser.add_argument("--yaw-limit", type=float, default=300.0)
    parser.add_argument("--forward-limit", type=float, default=250.0)
    parser.add_argument("--settle-seconds", type=float, default=0.001)
    args = parser.parse_args(argv)
    if args.frame_width <= 0 or args.frame_height <= 0:
        parser.error("--frame-width and --frame-height must be positive")
    if args.desired_target_width <= 0:
        parser.error("--desired-target-width must be positive")
    if args.target_offset_px <= args.deadband:
        parser.error("--target-offset-px must be greater than --deadband")
    if args.target_size_delta_px <= args.size_deadband:
        parser.error("--target-size-delta-px must be greater than --size-deadband")
    if args.settle_seconds < 0:
        parser.error("--settle-seconds must be non-negative")
    return args


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    results = run_checks(args)
    failures = [result for result in results if not result["ok"]]

    print("PID direction check: %s" % ("PASS" if not failures else "FAIL"))
    for result in results:
        print(
            "%s: %s yaw=%d (%s 1500) pitch=%d (%s 1500) "
            "roll=%d throttle=%d errors=%.1f/%.1f outputs=%.1f/%.1f"
            % (
                result["name"],
                "PASS" if result["ok"] else "FAIL",
                result["yaw"],
                relation_symbol(result["expected_yaw"]),
                result["pitch"],
                relation_symbol(result["expected_pitch"]),
                result["roll"],
                result["throttle"],
                result["yaw_error"],
                result["forward_error"],
                result["yaw_output"],
                result["forward_output"],
            )
        )
    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(main())
