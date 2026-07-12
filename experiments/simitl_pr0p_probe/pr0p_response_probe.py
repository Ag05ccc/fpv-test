#!/usr/bin/env python3
"""Command-response probe for the isolated SimITL/pr0p experiment."""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Iterable

import cv2
import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]
SANDBOX_DIR = REPO_ROOT / "experiments" / "game_screen_sandbox"
if str(SANDBOX_DIR) not in sys.path:
    sys.path.insert(0, str(SANDBOX_DIR))
PROBE_DIR = Path(__file__).resolve().parent
if str(PROBE_DIR) not in sys.path:
    sys.path.insert(0, str(PROBE_DIR))

from capture_window import FrameSource, make_region_source, parse_region  # noqa: E402
from pr0p_capture_probe import (  # noqa: E402
    DEFAULT_LOG_DIR,
    DEFAULT_WINDOW_TITLES,
    apply_relative_crop,
    find_pr0p_window,
)
from virtual_input import AxisCommand, DryRunInputAdapter, UInputAdapter  # noqa: E402
from x11_window import parse_size  # noqa: E402


PASS = "PASS"
WAITING = "WAITING"
FAIL = "FAIL"
AXES = ("yaw", "pitch", "roll", "throttle")
IMAGE_AXES = ("x", "y")


@dataclass
class ResponseProbeResult:
    status: str
    summary: str
    metrics: dict[str, Any] = field(default_factory=dict)
    notes: list[str] = field(default_factory=list)

    def as_dict(self) -> dict[str, Any]:
        return {
            "status": self.status,
            "summary": self.summary,
            "metrics": self.metrics,
            "notes": self.notes,
        }


def sign_of(value: float, *, eps: float = 1e-6) -> int:
    if value > eps:
        return 1
    if value < -eps:
        return -1
    return 0


def command_for_axis(axis: str, magnitude: float) -> AxisCommand:
    if axis not in AXES:
        raise ValueError("unknown axis: %s" % axis)
    kwargs = {name: 0.0 for name in AXES}
    kwargs[axis] = magnitude
    return AxisCommand(**kwargs).clamped()


def frame_shift(reference: np.ndarray, frame: np.ndarray) -> tuple[float, float, float]:
    """Estimate image translation from reference to frame.

    Returns `(dx, dy, response)`, where positive dx means the later image is
    shifted right relative to the reference.
    """
    if reference.shape != frame.shape:
        raise ValueError("frame shapes differ: %s vs %s" % (reference.shape, frame.shape))
    gray_a = cv2.cvtColor(reference, cv2.COLOR_BGR2GRAY) if reference.ndim == 3 else reference
    gray_b = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if frame.ndim == 3 else frame
    a = gray_a.astype(np.float32)
    b = gray_b.astype(np.float32)
    shift, response = cv2.phaseCorrelate(a, b)
    return float(shift[0]), float(shift[1]), float(response)


def median_pairwise_shift(frames: list[np.ndarray]) -> tuple[float, float, float]:
    if len(frames) < 2:
        return (0.0, 0.0, 0.0)
    shifts = [
        frame_shift(frames[index - 1], frames[index])
        for index in range(1, len(frames))
    ]
    return (
        float(np.median([item[0] for item in shifts])),
        float(np.median([item[1] for item in shifts])),
        float(np.median([item[2] for item in shifts])),
    )


def evaluate_direction(
    *,
    projection_px: float,
    expected_sign: int,
    min_shift_px: float,
    max_shift_px: float,
) -> tuple[str, str, list[str]]:
    magnitude = abs(projection_px)
    observed_sign = sign_of(projection_px)
    if magnitude < min_shift_px:
        return (
            WAITING,
            "visual response did not cross the minimum shift threshold",
            ["NO_VISUAL_RESPONSE"],
        )
    if magnitude > max_shift_px:
        return (
            FAIL,
            "visual response exceeded the runaway shift threshold",
            ["SPIN_RUNAWAY"],
        )
    if expected_sign and observed_sign != expected_sign:
        return (
            FAIL,
            "visual response moved in the opposite direction",
            ["DIRECTION_SIGN_FAIL"],
        )
    return (
        PASS,
        "visual response moved in the expected direction",
        ["direction sign matched" if expected_sign else "direction sign recorded without assertion"],
    )


def measure_visual_response(
    source: FrameSource,
    adapter,
    *,
    command: AxisCommand,
    axis: str,
    image_axis: str,
    expected_sign: int,
    fps: float,
    pre_duration_s: float,
    post_duration_s: float,
    min_shift_px: float,
    max_shift_px: float,
) -> ResponseProbeResult:
    if fps <= 0:
        raise ValueError("fps must be positive")
    if pre_duration_s <= 0 or post_duration_s <= 0:
        raise ValueError("pre/post duration must be positive")
    if min_shift_px < 0 or max_shift_px <= 0:
        raise ValueError("shift thresholds must be positive")
    if min_shift_px > max_shift_px:
        raise ValueError("min_shift_px must be <= max_shift_px")
    if image_axis not in IMAGE_AXES:
        raise ValueError("image_axis must be x or y")

    pre_count = max(2, int(math.ceil(pre_duration_s * fps)))
    post_count = max(1, int(math.ceil(post_duration_s * fps)))
    stream = source.frames(pre_duration_s + post_duration_s + 1.0 / fps)
    pre_items = []
    try:
        for _ in range(pre_count):
            pre_items.append(next(stream))
    except StopIteration:
        return ResponseProbeResult(
            status=FAIL,
            summary="not enough pre-command frames",
            metrics={
                "axis": axis,
                "command": command.as_dict(),
                "frames_before": len(pre_items),
                "frames_after": 0,
            },
            notes=["CAPTURE_TOO_SHORT"],
        )

    pre_frames = [item.frame for item in pre_items]
    baseline = pre_frames[-1]
    baseline_dx, baseline_dy, baseline_response = median_pairwise_shift(pre_frames)
    post_shifts: list[tuple[float, float, float]] = []

    try:
        adapter.send(command)
        for _ in range(post_count):
            try:
                item = next(stream)
            except StopIteration:
                break
            post_shifts.append(frame_shift(baseline, item.frame))
    finally:
        adapter.neutral()

    if not post_shifts:
        return ResponseProbeResult(
            status=FAIL,
            summary="not enough post-command frames",
            metrics={
                "axis": axis,
                "command": command.as_dict(),
                "frames_before": len(pre_items),
                "frames_after": 0,
            },
            notes=["CAPTURE_TOO_SHORT"],
        )

    dx = float(np.median([item[0] for item in post_shifts]))
    dy = float(np.median([item[1] for item in post_shifts]))
    response = float(np.median([item[2] for item in post_shifts]))
    projection_px = dx if image_axis == "x" else dy
    status, summary, notes = evaluate_direction(
        projection_px=projection_px,
        expected_sign=expected_sign,
        min_shift_px=min_shift_px,
        max_shift_px=max_shift_px,
    )
    metrics = {
        "axis": axis,
        "image_axis": image_axis,
        "expected_sign": expected_sign,
        "observed_sign": sign_of(projection_px),
        "projection_px": projection_px,
        "median_dx_px": dx,
        "median_dy_px": dy,
        "median_phase_response": response,
        "baseline_dx_px": baseline_dx,
        "baseline_dy_px": baseline_dy,
        "baseline_phase_response": baseline_response,
        "frames_before": len(pre_items),
        "frames_after": len(post_shifts),
        "fps": fps,
        "pre_duration_s": pre_duration_s,
        "post_duration_s": post_duration_s,
        "min_shift_px": min_shift_px,
        "max_shift_px": max_shift_px,
        "command": command.as_dict(),
    }
    return ResponseProbeResult(
        status=status,
        summary=summary,
        metrics=metrics,
        notes=notes,
    )


def make_source_from_selection(
    *,
    region: dict[str, int] | None,
    crop: dict[str, int] | None,
    window_titles: Iterable[str],
    exact: bool,
    case_sensitive: bool,
    min_width: int,
    min_height: int,
    preferred_size: tuple[int, int] | None,
    allow_common_non_fpv_windows: bool,
    backend: str,
    fps: float,
) -> tuple[FrameSource | None, dict[str, Any], ResponseProbeResult | None]:
    selected_region = region
    selected_window = None
    visible_sample: list[dict[str, Any]] = []
    if selected_region is None:
        excluded_terms = () if allow_common_non_fpv_windows else None
        kwargs: dict[str, Any] = {}
        if excluded_terms is not None:
            kwargs["excluded_terms"] = excluded_terms
        match, windows, window_error = find_pr0p_window(
            window_titles,
            exact=exact,
            case_sensitive=case_sensitive,
            min_width=min_width,
            min_height=min_height,
            preferred_size=preferred_size,
            **kwargs,
        )
        visible_sample = [
            {
                "window_id": window.window_id,
                "title": window.title,
                "class_text": window.class_text,
                "region": window.region(),
            }
            for window in windows[:12]
        ]
        if match is None:
            return None, {}, ResponseProbeResult(
                status=WAITING,
                summary="no matching pr0p/FPV window is visible yet",
                metrics={
                    "window_titles": list(window_titles),
                    "window_error": window_error,
                    "visible_windows_sample": visible_sample,
                },
                notes=["CAPTURE_NO_WINDOW", "start pr0p/local race or pass --region"],
            )
        selected_window = match.as_dict()
        selected_region = match.region()

    try:
        capture_region = apply_relative_crop(selected_region, crop)
    except ValueError as exc:
        return None, {}, ResponseProbeResult(
            status=FAIL,
            summary="capture crop is invalid",
            metrics={
                "selected_region": selected_region,
                "crop": crop,
                "error": str(exc),
            },
            notes=["CAPTURE_CROP_INVALID"],
        )

    try:
        source = make_region_source(capture_region, fps=fps, backend=backend)
    except Exception as exc:
        return None, {}, ResponseProbeResult(
            status=WAITING,
            summary="no usable real screen capture backend is available",
            metrics={
                "backend": backend,
                "capture_region": capture_region,
                "error": str(exc),
            },
            notes=["CAPTURE_BACKEND_UNAVAILABLE"],
        )
    return source, {
        "selected_region": selected_region,
        "capture_region": capture_region,
        "crop": crop,
        "selected_window": selected_window,
        "visible_windows_sample": visible_sample,
    }, None


def run_response_probe(
    *,
    run_id: str,
    region: dict[str, int] | None,
    crop: dict[str, int] | None,
    window_titles: Iterable[str],
    exact: bool,
    case_sensitive: bool,
    min_width: int,
    min_height: int,
    preferred_size: tuple[int, int] | None,
    allow_common_non_fpv_windows: bool,
    backend: str,
    fps: float,
    axis: str,
    magnitude: float,
    image_axis: str,
    expected_sign: int,
    pre_duration_s: float,
    post_duration_s: float,
    min_shift_px: float,
    max_shift_px: float,
    use_uinput: bool,
) -> ResponseProbeResult:
    source, selection_metrics, early = make_source_from_selection(
        region=region,
        crop=crop,
        window_titles=window_titles,
        exact=exact,
        case_sensitive=case_sensitive,
        min_width=min_width,
        min_height=min_height,
        preferred_size=preferred_size,
        allow_common_non_fpv_windows=allow_common_non_fpv_windows,
        backend=backend,
        fps=fps,
    )
    if early is not None:
        early.metrics["run_id"] = run_id
        early.metrics["real_input_sent"] = False
        return early
    assert source is not None

    command = command_for_axis(axis, magnitude)
    adapter = UInputAdapter() if use_uinput else DryRunInputAdapter()
    try:
        result = measure_visual_response(
            source,
            adapter,
            command=command,
            axis=axis,
            image_axis=image_axis,
            expected_sign=expected_sign,
            fps=fps,
            pre_duration_s=pre_duration_s,
            post_duration_s=post_duration_s,
            min_shift_px=min_shift_px,
            max_shift_px=max_shift_px,
        )
    finally:
        adapter.close()
    result.metrics.update(selection_metrics)
    result.metrics["run_id"] = run_id
    result.metrics["real_input_sent"] = use_uinput
    result.metrics["adapter"] = adapter.__class__.__name__
    if not use_uinput:
        result.status = WAITING
        result.summary = "direction metric is informational because no real input was sent"
        result.notes.append("DRY_RUN_ONLY")
    return result


def build_markdown(result: ResponseProbeResult, *, run_id: str) -> str:
    lines = [
        "# SimITL / pr0p Command Response Probe",
        "",
        "Run: `%s`" % run_id,
        "Verdict: `%s`" % result.status,
        "",
        result.summary,
        "",
        "| Metric | Value |",
        "| --- | --- |",
    ]
    for key in (
        "axis",
        "image_axis",
        "expected_sign",
        "observed_sign",
        "projection_px",
        "median_dx_px",
        "median_dy_px",
        "real_input_sent",
    ):
        if key in result.metrics:
            lines.append("| %s | `%s` |" % (key, result.metrics[key]))
    lines.extend([
        "",
        "## Metrics",
        "",
        "```json",
        json.dumps(result.metrics, indent=2, sort_keys=True),
        "```",
    ])
    for note in result.notes:
        lines.append("- %s" % note)
    lines.append("")
    return "\n".join(lines)


def write_reports(result: ResponseProbeResult, log_dir: Path, *, run_id: str) -> tuple[Path, Path]:
    log_dir.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime("%Y%m%d-%H%M%S", time.localtime())
    json_path = log_dir / ("%s-%s-response-probe.json" % (stamp, run_id))
    md_path = log_dir / ("%s-%s-response-probe.md" % (stamp, run_id))
    json_path.write_text(
        json.dumps(result.as_dict(), indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    md_path.write_text(build_markdown(result, run_id=run_id), encoding="utf-8")
    return json_path, md_path


def parse_expected_sign(value: str) -> int:
    normalized = value.strip().lower()
    if normalized in ("-1", "negative", "neg", "left", "up"):
        return -1
    if normalized in ("0", "none", "record"):
        return 0
    if normalized in ("1", "+1", "positive", "pos", "right", "down"):
        return 1
    raise argparse.ArgumentTypeError("expected sign must be -1, 0, or 1")


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-id", default="pr0p-response")
    parser.add_argument("--log-dir", type=Path, default=DEFAULT_LOG_DIR)
    parser.add_argument("--region", type=parse_region)
    parser.add_argument("--crop", type=parse_region)
    parser.add_argument("--window-title", action="append", dest="window_titles")
    parser.add_argument("--window-exact", action="store_true")
    parser.add_argument("--window-case-sensitive", action="store_true")
    parser.add_argument("--allow-common-non-fpv-windows", action="store_true")
    parser.add_argument("--window-min-width", type=int, default=160)
    parser.add_argument("--window-min-height", type=int, default=120)
    parser.add_argument("--preferred-size", type=parse_size)
    parser.add_argument("--backend", choices=["auto", "mss", "ffmpeg"], default="auto")
    parser.add_argument("--fps", type=float, default=15.0)
    parser.add_argument("--axis", choices=AXES, default="yaw")
    parser.add_argument("--magnitude", type=float, default=0.05)
    parser.add_argument("--image-axis", choices=IMAGE_AXES, default="x")
    parser.add_argument("--expected-sign", type=parse_expected_sign, default=0)
    parser.add_argument("--pre-duration", type=float, default=0.5)
    parser.add_argument("--post-duration", type=float, default=1.0)
    parser.add_argument("--min-shift-px", type=float, default=2.0)
    parser.add_argument("--max-shift-px", type=float, default=200.0)
    parser.add_argument("--uinput", action="store_true",
                        help="Send a real uinput command")
    parser.add_argument("--ack-live-input", action="store_true",
                        help="Required before --uinput sends real OS input")
    args = parser.parse_args(argv)
    if args.window_titles is None:
        args.window_titles = list(DEFAULT_WINDOW_TITLES)
    if args.window_min_width <= 0 or args.window_min_height <= 0:
        parser.error("--window-min-width/--window-min-height must be positive")
    if args.fps <= 0:
        parser.error("--fps must be positive")
    if args.pre_duration <= 0 or args.post_duration <= 0:
        parser.error("--pre-duration/--post-duration must be positive")
    if args.min_shift_px < 0:
        parser.error("--min-shift-px must be non-negative")
    if args.max_shift_px <= 0:
        parser.error("--max-shift-px must be positive")
    if args.min_shift_px > args.max_shift_px:
        parser.error("--min-shift-px must be <= --max-shift-px")
    if args.uinput and not args.ack_live_input:
        parser.error("--uinput requires --ack-live-input")
    return args


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    result = run_response_probe(
        run_id=args.run_id,
        region=args.region,
        crop=args.crop,
        window_titles=args.window_titles,
        exact=args.window_exact,
        case_sensitive=args.window_case_sensitive,
        min_width=args.window_min_width,
        min_height=args.window_min_height,
        preferred_size=args.preferred_size,
        allow_common_non_fpv_windows=args.allow_common_non_fpv_windows,
        backend=args.backend,
        fps=args.fps,
        axis=args.axis,
        magnitude=args.magnitude,
        image_axis=args.image_axis,
        expected_sign=args.expected_sign,
        pre_duration_s=args.pre_duration,
        post_duration_s=args.post_duration,
        min_shift_px=args.min_shift_px,
        max_shift_px=args.max_shift_px,
        use_uinput=args.uinput,
    )
    json_path, md_path = write_reports(result, args.log_dir, run_id=args.run_id)
    print("simitl-pr0p-response %s report=%s summary=%s real_input_sent=%s" % (
        result.status,
        json_path,
        md_path,
        result.metrics.get("real_input_sent"),
    ))
    print(result.summary)
    return 1 if result.status == FAIL else 0


if __name__ == "__main__":
    raise SystemExit(main())
