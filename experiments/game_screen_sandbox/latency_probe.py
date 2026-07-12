#!/usr/bin/env python3
"""Measure visual response latency after a virtual input command."""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Any

import cv2
import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from capture_window import (  # noqa: E402
    FrameSource,
    SyntheticFrameSource,
    VideoFileFrameSource,
    make_region_source,
    parse_region,
    resolve_capture_region,
)
from virtual_input import AxisCommand, DryRunInputAdapter, InputAdapter, UInputAdapter  # noqa: E402


PASS = "PASS"
FAIL = "FAIL"
WAITING = "WAITING"
AXIS_NAMES = ("yaw", "pitch", "roll", "throttle")
SHIFT_COMPONENTS = ("x", "y")


@dataclass(frozen=True)
class LatencyProbeResult:
    status: str
    latency_ms: float | None
    frames_before: int
    frames_after: int
    threshold: float
    baseline_diff: float
    max_diff: float
    command: AxisCommand
    notes: list[str]
    response_shift_x_px: float | None = None
    response_shift_y_px: float | None = None
    response_shift_score: float | None = None
    direction_status: str | None = None
    expected_shift_component: str | None = None
    expected_shift_sign: int | None = None
    min_shift_px: float | None = None

    def as_dict(self) -> dict[str, Any]:
        return {
            "status": self.status,
            "latency_ms": self.latency_ms,
            "frames_before": self.frames_before,
            "frames_after": self.frames_after,
            "threshold": self.threshold,
            "baseline_diff": self.baseline_diff,
            "max_diff": self.max_diff,
            "command": self.command.as_dict(),
            "notes": self.notes,
            "response_shift_x_px": self.response_shift_x_px,
            "response_shift_y_px": self.response_shift_y_px,
            "response_shift_score": self.response_shift_score,
            "direction_status": self.direction_status,
            "expected_shift_component": self.expected_shift_component,
            "expected_shift_sign": self.expected_shift_sign,
            "min_shift_px": self.min_shift_px,
        }


@dataclass(frozen=True)
class AxisSweepResult:
    status: str
    axes: tuple[str, ...]
    axis_value: float
    passed_axes: tuple[str, ...]
    waiting_axes: tuple[str, ...]
    failed_axes: tuple[str, ...]
    results: tuple[tuple[str, LatencyProbeResult], ...]
    notes: list[str]
    expected_shifts: dict[str, tuple[str, int]] | None = None
    min_shift_px: float | None = None

    def as_dict(self) -> dict[str, Any]:
        return {
            "status": self.status,
            "axes": list(self.axes),
            "axis_value": self.axis_value,
            "passed_axes": list(self.passed_axes),
            "waiting_axes": list(self.waiting_axes),
            "failed_axes": list(self.failed_axes),
            "axis_statuses": {
                axis: result.status for axis, result in self.results
            },
            "axis_results": {
                axis: result.as_dict() for axis, result in self.results
            },
            "expected_shifts": {
                axis: "%s%s" % ("+" if sign > 0 else "-", component)
                for axis, (component, sign) in (self.expected_shifts or {}).items()
            } or None,
            "min_shift_px": self.min_shift_px,
            "notes": self.notes,
        }


def frame_diff(a: np.ndarray, b: np.ndarray) -> float:
    if a.shape != b.shape:
        raise ValueError("frame shapes differ: %s vs %s" % (a.shape, b.shape))
    gray_a = cv2.cvtColor(a, cv2.COLOR_BGR2GRAY) if a.ndim == 3 else a
    gray_b = cv2.cvtColor(b, cv2.COLOR_BGR2GRAY) if b.ndim == 3 else b
    return float(cv2.absdiff(gray_a, gray_b).mean())


def median_pairwise_diff(frames: list[np.ndarray]) -> float:
    if len(frames) < 2:
        return 0.0
    diffs = [frame_diff(frames[i - 1], frames[i]) for i in range(1, len(frames))]
    return float(np.median(diffs)) if diffs else 0.0


def estimate_frame_shift(a: np.ndarray, b: np.ndarray) -> tuple[float, float, float]:
    """Estimate signed image translation from frame a to frame b."""
    if a.shape != b.shape:
        raise ValueError("frame shapes differ: %s vs %s" % (a.shape, b.shape))
    gray_a = cv2.cvtColor(a, cv2.COLOR_BGR2GRAY) if a.ndim == 3 else a
    gray_b = cv2.cvtColor(b, cv2.COLOR_BGR2GRAY) if b.ndim == 3 else b
    shift, score = cv2.phaseCorrelate(
        np.asarray(gray_a, dtype=np.float32),
        np.asarray(gray_b, dtype=np.float32),
    )
    return float(shift[0]), float(shift[1]), float(score)


def parse_axis_shift_expectations(value: str | None) -> dict[str, tuple[str, int]]:
    if not value:
        return {}
    expectations: dict[str, tuple[str, int]] = {}
    for raw_part in value.split(","):
        part = raw_part.strip()
        if not part:
            continue
        if ":" not in part:
            raise ValueError("expected shift entry must look like axis:+x")
        axis, raw_shift = [piece.strip() for piece in part.split(":", 1)]
        if axis not in AXIS_NAMES:
            raise ValueError("unknown axis in expected shift: %s" % axis)
        if len(raw_shift) != 2 or raw_shift[0] not in ("+", "-"):
            raise ValueError("expected shift for %s must look like +x or -y" % axis)
        component = raw_shift[1]
        if component not in SHIFT_COMPONENTS:
            raise ValueError("expected shift component for %s must be x or y" % axis)
        expectations[axis] = (component, 1 if raw_shift[0] == "+" else -1)
    return expectations


def evaluate_expected_shift(
    result: LatencyProbeResult,
    *,
    component: str,
    sign: int,
    min_shift_px: float,
) -> LatencyProbeResult:
    if component not in SHIFT_COMPONENTS:
        raise ValueError("component must be x or y")
    if sign not in (-1, 1):
        raise ValueError("sign must be -1 or 1")
    if min_shift_px < 0:
        raise ValueError("min_shift_px must be non-negative")
    measured = (
        result.response_shift_x_px
        if component == "x" else result.response_shift_y_px
    )
    direction_notes = list(result.notes)
    if result.status != PASS:
        direction_status = result.status
    elif measured is None:
        direction_status = WAITING
        direction_notes.append("visual response shift could not be estimated")
    elif abs(measured) < min_shift_px:
        direction_status = WAITING
        direction_notes.append(
            "visual response shift %.3fpx is below %.3fpx on %s"
            % (measured, min_shift_px, component)
        )
    elif measured * sign > 0:
        direction_status = PASS
        direction_notes.append(
            "visual response moved in expected %s%s direction"
            % ("+" if sign > 0 else "-", component)
        )
    else:
        direction_status = FAIL
        direction_notes.append(
            "visual response moved opposite expected %s%s direction"
            % ("+" if sign > 0 else "-", component)
        )
    return replace(
        result,
        status=direction_status,
        notes=direction_notes,
        direction_status=direction_status,
        expected_shift_component=component,
        expected_shift_sign=sign,
        min_shift_px=min_shift_px,
    )


def measure_visual_latency(
    source: FrameSource,
    adapter: InputAdapter,
    *,
    command: AxisCommand,
    fps: float,
    pre_duration_s: float = 0.5,
    post_duration_s: float = 1.0,
    min_diff: float = 3.0,
    baseline_multiplier: float = 3.0,
) -> LatencyProbeResult:
    if fps <= 0:
        raise ValueError("fps must be positive")
    if pre_duration_s <= 0 or post_duration_s <= 0:
        raise ValueError("pre/post duration must be positive")
    if min_diff < 0:
        raise ValueError("min_diff must be non-negative")
    if baseline_multiplier < 1:
        raise ValueError("baseline_multiplier must be >= 1")

    pre_count = max(2, int(math.ceil(pre_duration_s * fps)))
    post_count = max(1, int(math.ceil(post_duration_s * fps)))
    stream = source.frames(pre_duration_s + post_duration_s + 1.0 / fps)
    pre_items = []
    try:
        for _ in range(pre_count):
            pre_items.append(next(stream))
    except StopIteration:
        return LatencyProbeResult(
            status=FAIL,
            latency_ms=None,
            frames_before=len(pre_items),
            frames_after=0,
            threshold=min_diff,
            baseline_diff=0.0,
            max_diff=0.0,
            command=command,
            notes=["not enough pre-command frames"],
        )

    pre_frames = [item.frame for item in pre_items]
    baseline = pre_frames[-1]
    baseline_diff = median_pairwise_diff(pre_frames)
    threshold = max(min_diff, baseline_diff * baseline_multiplier)
    max_seen = 0.0
    after = 0
    latency_ms: float | None = None
    response_shift_x_px: float | None = None
    response_shift_y_px: float | None = None
    response_shift_score: float | None = None

    command_ts = time.monotonic()
    try:
        adapter.send(command)
        for item in stream:
            after += 1
            diff = frame_diff(baseline, item.frame)
            max_seen = max(max_seen, diff)
            if diff >= threshold:
                latency_ms = max(0.0, (item.timestamp - command_ts) * 1000.0)
                try:
                    shift_x, shift_y, shift_score = estimate_frame_shift(
                        baseline,
                        item.frame,
                    )
                    response_shift_x_px = shift_x
                    response_shift_y_px = shift_y
                    response_shift_score = shift_score
                except Exception:
                    response_shift_x_px = None
                    response_shift_y_px = None
                    response_shift_score = None
                break
            if after >= post_count:
                break
    finally:
        adapter.neutral()

    if latency_ms is None:
        return LatencyProbeResult(
            status=WAITING,
            latency_ms=None,
            frames_before=len(pre_items),
            frames_after=after,
            threshold=threshold,
            baseline_diff=baseline_diff,
            max_diff=max_seen,
            command=command,
            notes=["no visual response crossed the threshold"],
        )
    return LatencyProbeResult(
        status=PASS,
        latency_ms=latency_ms,
        frames_before=len(pre_items),
        frames_after=after,
        threshold=threshold,
        baseline_diff=baseline_diff,
        max_diff=max_seen,
        command=command,
        notes=["visual response detected"],
        response_shift_x_px=response_shift_x_px,
        response_shift_y_px=response_shift_y_px,
        response_shift_score=response_shift_score,
    )


def axis_command(axis: str, value: float) -> AxisCommand:
    if axis not in AXIS_NAMES:
        raise ValueError("unknown axis: %s" % axis)
    kwargs = {name: 0.0 for name in AXIS_NAMES}
    kwargs[axis] = value
    return AxisCommand(**kwargs).clamped()


def normalize_axes(axes: list[str] | tuple[str, ...] | str) -> tuple[str, ...]:
    if isinstance(axes, str):
        raw_axes = [part.strip() for part in axes.split(",")]
    else:
        raw_axes = [str(part).strip() for part in axes]
    selected = tuple(axis for axis in raw_axes if axis)
    if not selected:
        raise ValueError("at least one axis must be selected")
    unknown = [axis for axis in selected if axis not in AXIS_NAMES]
    if unknown:
        raise ValueError("unknown axis name(s): %s" % ",".join(unknown))
    return selected


def measure_axis_response_sweep(
    source: FrameSource,
    adapter: InputAdapter,
    *,
    axes: list[str] | tuple[str, ...] | str,
    axis_value: float,
    fps: float,
    pre_duration_s: float = 0.5,
    post_duration_s: float = 1.0,
    min_diff: float = 3.0,
    baseline_multiplier: float = 3.0,
    expected_shifts: dict[str, tuple[str, int]] | None = None,
    min_shift_px: float = 0.5,
) -> AxisSweepResult:
    selected_axes = normalize_axes(axes)
    expected_shifts = dict(expected_shifts or {})
    unknown_expected = [axis for axis in expected_shifts if axis not in selected_axes]
    if unknown_expected:
        raise ValueError(
            "expected shift provided for unselected axis/axes: %s"
            % ",".join(sorted(unknown_expected))
        )
    if min_shift_px < 0:
        raise ValueError("min_shift_px must be non-negative")
    results: list[tuple[str, LatencyProbeResult]] = []
    passed: list[str] = []
    waiting: list[str] = []
    failed: list[str] = []
    for axis in selected_axes:
        result = measure_visual_latency(
            source,
            adapter,
            command=axis_command(axis, axis_value),
            fps=fps,
            pre_duration_s=pre_duration_s,
            post_duration_s=post_duration_s,
            min_diff=min_diff,
            baseline_multiplier=baseline_multiplier,
        )
        if axis in expected_shifts:
            component, sign = expected_shifts[axis]
            result = evaluate_expected_shift(
                result,
                component=component,
                sign=sign,
                min_shift_px=min_shift_px,
            )
        results.append((axis, result))
        if result.status == PASS:
            passed.append(axis)
        elif result.status == WAITING:
            waiting.append(axis)
        else:
            failed.append(axis)

    if failed:
        status = FAIL
        notes = ["one or more axis probes failed"]
    elif waiting:
        status = WAITING
        notes = [
            "one or more axes produced no visual response or no signed direction proof",
            "check the simulator/game Controls -> RC Channels or axis binding page",
        ]
    else:
        status = PASS
        notes = [
            "all selected axes produced visual response"
            if not expected_shifts else
            "all selected axes produced visual response in the expected direction"
        ]
    return AxisSweepResult(
        status=status,
        axes=selected_axes,
        axis_value=axis_value,
        passed_axes=tuple(passed),
        waiting_axes=tuple(waiting),
        failed_axes=tuple(failed),
        results=tuple(results),
        notes=notes,
        expected_shifts=expected_shifts or None,
        min_shift_px=min_shift_px if expected_shifts else None,
    )


def make_source(args: argparse.Namespace) -> FrameSource:
    if args.synthetic:
        return SyntheticFrameSource(width=args.width, height=args.height, fps=args.fps)
    if args.video:
        return VideoFileFrameSource(args.video)
    region = resolve_capture_region(
        region=args.region,
        window_title=args.window_title,
        window_exact=args.window_exact,
        window_case_sensitive=args.window_case_sensitive,
        window_min_width=args.window_min_width,
        window_min_height=args.window_min_height,
    )
    return make_region_source(region, fps=args.fps, backend=args.capture_backend)


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--synthetic", action="store_true")
    mode.add_argument("--video")
    mode.add_argument("--region", type=parse_region)
    mode.add_argument("--window-title")
    parser.add_argument("--capture-backend", choices=["auto", "mss", "ffmpeg"], default="auto")
    parser.add_argument("--window-exact", action="store_true")
    parser.add_argument("--window-case-sensitive", action="store_true")
    parser.add_argument("--window-min-width", type=int, default=32)
    parser.add_argument("--window-min-height", type=int, default=32)
    parser.add_argument("--fps", type=float, default=10.0)
    parser.add_argument("--pre-duration", type=float, default=0.5)
    parser.add_argument("--post-duration", type=float, default=1.0)
    parser.add_argument("--min-diff", type=float, default=3.0)
    parser.add_argument("--baseline-multiplier", type=float, default=3.0)
    parser.add_argument("--axis-sweep", action="store_true",
                        help="Probe each selected RC/game axis for a visual response")
    parser.add_argument("--axes", default="yaw,pitch",
                        help="Comma-separated axes for --axis-sweep")
    parser.add_argument("--axis-value", type=float, default=0.05)
    parser.add_argument("--axis-expected-shifts", default="",
                        help="Optional signed visual shift checks, e.g. yaw:+x,pitch:-y")
    parser.add_argument("--axis-min-shift-px", type=float, default=0.5)
    parser.add_argument("--yaw", type=float, default=0.05)
    parser.add_argument("--pitch", type=float, default=0.0)
    parser.add_argument("--roll", type=float, default=0.0)
    parser.add_argument("--throttle", type=float, default=0.0)
    parser.add_argument("--uinput", action="store_true")
    parser.add_argument("--ack-live-input", action="store_true")
    parser.add_argument("--report-path", type=Path)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    args = parser.parse_args(argv)
    if args.fps <= 0:
        parser.error("--fps must be positive")
    if args.pre_duration <= 0 or args.post_duration <= 0:
        parser.error("--pre-duration/--post-duration must be positive")
    if args.min_diff < 0:
        parser.error("--min-diff must be non-negative")
    if args.baseline_multiplier < 1:
        parser.error("--baseline-multiplier must be >= 1")
    if not 0 < abs(args.axis_value) <= 1:
        parser.error("--axis-value magnitude must be in (0, 1]")
    if args.axis_sweep:
        try:
            args.axes = normalize_axes(args.axes)
        except ValueError as exc:
            parser.error(str(exc))
    if args.axis_expected_shifts and not args.axis_sweep:
        parser.error("--axis-expected-shifts requires --axis-sweep")
    try:
        args.axis_expected_shifts = parse_axis_shift_expectations(args.axis_expected_shifts)
    except ValueError as exc:
        parser.error(str(exc))
    if args.axis_min_shift_px < 0:
        parser.error("--axis-min-shift-px must be non-negative")
    if args.uinput and not args.ack_live_input:
        parser.error("--uinput requires --ack-live-input")
    if args.width <= 0 or args.height <= 0:
        parser.error("--width/--height must be positive")
    if args.window_min_width <= 0 or args.window_min_height <= 0:
        parser.error("--window-min-width/--window-min-height must be positive")
    return args


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    source = make_source(args)
    adapter: InputAdapter = UInputAdapter() if args.uinput else DryRunInputAdapter()
    command = AxisCommand(
        yaw=args.yaw,
        pitch=args.pitch,
        roll=args.roll,
        throttle=args.throttle,
    ).clamped()
    try:
        if args.axis_sweep:
            result = measure_axis_response_sweep(
                source,
                adapter,
                axes=args.axes,
                axis_value=args.axis_value,
                fps=args.fps,
                pre_duration_s=args.pre_duration,
                post_duration_s=args.post_duration,
                min_diff=args.min_diff,
                baseline_multiplier=args.baseline_multiplier,
                expected_shifts=args.axis_expected_shifts,
                min_shift_px=args.axis_min_shift_px,
            )
        else:
            result = measure_visual_latency(
                source,
                adapter,
                command=command,
                fps=args.fps,
                pre_duration_s=args.pre_duration,
                post_duration_s=args.post_duration,
                min_diff=args.min_diff,
                baseline_multiplier=args.baseline_multiplier,
            )
    finally:
        adapter.close()
    if args.report_path:
        args.report_path.parent.mkdir(parents=True, exist_ok=True)
        args.report_path.write_text(
            json.dumps(result.as_dict(), indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
    if isinstance(result, AxisSweepResult):
        axis_statuses = ",".join(
            "%s:%s" % (axis, probe.status) for axis, probe in result.results
        )
        print("axis-sweep %s axes=%s" % (result.status, axis_statuses))
    else:
        latency_note = (
            " latency_ms=%.1f" % result.latency_ms
            if result.latency_ms is not None else ""
        )
        print("latency %s%s frames_before=%d frames_after=%d max_diff=%.2f threshold=%.2f" % (
            result.status,
            latency_note,
            result.frames_before,
            result.frames_after,
            result.max_diff,
            result.threshold,
        ))
    return 0 if result.status == PASS else 2 if result.status == WAITING else 1


if __name__ == "__main__":
    raise SystemExit(main())
