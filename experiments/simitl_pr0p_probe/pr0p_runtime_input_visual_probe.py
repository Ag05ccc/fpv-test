#!/usr/bin/env python3
"""Visual runtime input probe for pr0p Controls -> RC Channels.

This probe is meant for the pr0p controls page, ideally cropped around the RC
channel bars. It sends a short virtual RC pulse and checks whether the captured
UI pixels change beyond the idle baseline. It does not prove vehicle dynamics;
it proves that the running pr0p UI/runtime is seeing the input source.
"""

from __future__ import annotations

import argparse
import json
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

from capture_window import FrameSource, parse_region  # noqa: E402
from pr0p_capture_probe import DEFAULT_LOG_DIR, DEFAULT_WINDOW_TITLES  # noqa: E402
from pr0p_response_probe import make_source_from_selection  # noqa: E402
from virtual_input import AxisCommand, DryRunInputAdapter, UInputAdapter  # noqa: E402
from x11_window import parse_size  # noqa: E402


PASS = "PASS"
WAITING = "WAITING"
FAIL = "FAIL"
AXES = ("yaw", "pitch", "roll", "throttle")


@dataclass
class RuntimeInputVisualResult:
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


def command_for_axis(axis: str, magnitude: float) -> AxisCommand:
    if axis not in AXES:
        raise ValueError("unknown axis: %s" % axis)
    values = {name: 0.0 for name in AXES}
    values[axis] = magnitude
    return AxisCommand(**values).clamped()


def frame_diff_metrics(reference: np.ndarray, frame: np.ndarray, *, pixel_threshold: int) -> dict[str, float]:
    if reference.shape != frame.shape:
        raise ValueError("frame shapes differ: %s vs %s" % (reference.shape, frame.shape))
    diff = cv2.absdiff(reference, frame)
    if diff.ndim == 3:
        diff_gray = cv2.cvtColor(diff, cv2.COLOR_BGR2GRAY)
    else:
        diff_gray = diff
    return {
        "mean_absdiff": float(np.mean(diff_gray)),
        "max_absdiff": float(np.max(diff_gray)),
        "changed_ratio": float(np.mean(diff_gray >= pixel_threshold)),
    }


def summarize_change(
    reference: np.ndarray,
    frames: list[np.ndarray],
    *,
    pixel_threshold: int,
) -> dict[str, float]:
    if not frames:
        return {
            "max_mean_absdiff": 0.0,
            "median_mean_absdiff": 0.0,
            "max_changed_ratio": 0.0,
            "median_changed_ratio": 0.0,
            "max_absdiff": 0.0,
        }
    diffs = [
        frame_diff_metrics(reference, frame, pixel_threshold=pixel_threshold)
        for frame in frames
    ]
    return {
        "max_mean_absdiff": max(item["mean_absdiff"] for item in diffs),
        "median_mean_absdiff": float(np.median([item["mean_absdiff"] for item in diffs])),
        "max_changed_ratio": max(item["changed_ratio"] for item in diffs),
        "median_changed_ratio": float(np.median([item["changed_ratio"] for item in diffs])),
        "max_absdiff": max(item["max_absdiff"] for item in diffs),
    }


def summarize_pairwise_baseline(frames: list[np.ndarray], *, pixel_threshold: int) -> dict[str, float]:
    if len(frames) < 2:
        return {
            "max_mean_absdiff": 0.0,
            "median_mean_absdiff": 0.0,
            "max_changed_ratio": 0.0,
            "median_changed_ratio": 0.0,
            "max_absdiff": 0.0,
        }
    diffs = [
        frame_diff_metrics(frames[index - 1], frames[index], pixel_threshold=pixel_threshold)
        for index in range(1, len(frames))
    ]
    return {
        "max_mean_absdiff": max(item["mean_absdiff"] for item in diffs),
        "median_mean_absdiff": float(np.median([item["mean_absdiff"] for item in diffs])),
        "max_changed_ratio": max(item["changed_ratio"] for item in diffs),
        "median_changed_ratio": float(np.median([item["changed_ratio"] for item in diffs])),
        "max_absdiff": max(item["max_absdiff"] for item in diffs),
    }


def evaluate_runtime_visual_change(
    *,
    pulse_summary: dict[str, float],
    baseline_summary: dict[str, float],
    min_mean_absdiff: float,
    min_changed_ratio: float,
    max_baseline_mean_absdiff: float,
    baseline_multiplier: float,
) -> tuple[str, str, list[str]]:
    baseline_mean = baseline_summary["max_mean_absdiff"]
    pulse_mean = pulse_summary["max_mean_absdiff"]
    pulse_ratio = pulse_summary["max_changed_ratio"]
    if baseline_mean > max_baseline_mean_absdiff:
        return (
            WAITING,
            "capture baseline is moving too much for a reliable input-visual gate",
            ["RUNTIME_INPUT_BASELINE_UNSTABLE", "crop a static Controls -> RC Channels area"],
        )
    enough_absolute_change = pulse_mean >= min_mean_absdiff or pulse_ratio >= min_changed_ratio
    enough_relative_change = pulse_mean >= max(
        min_mean_absdiff,
        baseline_mean * baseline_multiplier,
    )
    if enough_absolute_change and enough_relative_change:
        return (
            PASS,
            "runtime UI changed while the virtual RC pulse was active",
            ["RUNTIME_INPUT_VISUAL_CHANGE_DETECTED"],
        )
    return (
        WAITING,
        "runtime UI did not visibly change while the virtual RC pulse was active",
        ["NO_RUNTIME_INPUT_VISUAL_CHANGE"],
    )


def measure_runtime_input_visual_change(
    source: FrameSource,
    adapter,
    *,
    command: AxisCommand,
    axis: str,
    fps: float,
    pre_duration_s: float,
    pulse_duration_s: float,
    neutral_duration_s: float,
    pixel_threshold: int,
    min_mean_absdiff: float,
    min_changed_ratio: float,
    max_baseline_mean_absdiff: float,
    baseline_multiplier: float,
) -> RuntimeInputVisualResult:
    if fps <= 0:
        raise ValueError("fps must be positive")
    if pre_duration_s <= 0 or pulse_duration_s <= 0:
        raise ValueError("pre/pulse duration must be positive")
    if neutral_duration_s < 0:
        raise ValueError("neutral duration must be non-negative")
    if pixel_threshold < 0 or pixel_threshold > 255:
        raise ValueError("pixel threshold must be in 0..255")
    pre_count = max(2, int(round(pre_duration_s * fps)))
    pulse_count = max(2, int(round(pulse_duration_s * fps)))
    neutral_count = max(0, int(round(neutral_duration_s * fps)))
    stream = source.frames(pre_duration_s + pulse_duration_s + neutral_duration_s + 1.0)

    pre_frames: list[np.ndarray] = []
    try:
        for _ in range(pre_count):
            pre_frames.append(next(stream).frame)
    except StopIteration:
        return RuntimeInputVisualResult(
            status=FAIL,
            summary="not enough pre-pulse frames",
            metrics={"axis": axis, "frames_before": len(pre_frames), "frames_pulse": 0},
            notes=["CAPTURE_TOO_SHORT"],
        )

    reference = pre_frames[-1]
    pulse_frames: list[np.ndarray] = []
    neutral_frames: list[np.ndarray] = []
    try:
        adapter.send(command)
        for _ in range(pulse_count):
            try:
                pulse_frames.append(next(stream).frame)
            except StopIteration:
                break
    finally:
        adapter.neutral()

    for _ in range(neutral_count):
        try:
            neutral_frames.append(next(stream).frame)
        except StopIteration:
            break

    if not pulse_frames:
        return RuntimeInputVisualResult(
            status=FAIL,
            summary="not enough pulse frames",
            metrics={"axis": axis, "frames_before": len(pre_frames), "frames_pulse": 0},
            notes=["CAPTURE_TOO_SHORT"],
        )

    baseline_summary = summarize_pairwise_baseline(pre_frames, pixel_threshold=pixel_threshold)
    pulse_summary = summarize_change(reference, pulse_frames, pixel_threshold=pixel_threshold)
    neutral_summary = summarize_change(reference, neutral_frames, pixel_threshold=pixel_threshold)
    status, summary, notes = evaluate_runtime_visual_change(
        pulse_summary=pulse_summary,
        baseline_summary=baseline_summary,
        min_mean_absdiff=min_mean_absdiff,
        min_changed_ratio=min_changed_ratio,
        max_baseline_mean_absdiff=max_baseline_mean_absdiff,
        baseline_multiplier=baseline_multiplier,
    )
    return RuntimeInputVisualResult(
        status=status,
        summary=summary,
        metrics={
            "axis": axis,
            "command": command.as_dict(),
            "frames_before": len(pre_frames),
            "frames_pulse": len(pulse_frames),
            "frames_neutral": len(neutral_frames),
            "fps": fps,
            "pre_duration_s": pre_duration_s,
            "pulse_duration_s": pulse_duration_s,
            "neutral_duration_s": neutral_duration_s,
            "pixel_threshold": pixel_threshold,
            "min_mean_absdiff": min_mean_absdiff,
            "min_changed_ratio": min_changed_ratio,
            "max_baseline_mean_absdiff": max_baseline_mean_absdiff,
            "baseline_multiplier": baseline_multiplier,
            "baseline_summary": baseline_summary,
            "pulse_summary": pulse_summary,
            "neutral_summary": neutral_summary,
        },
        notes=notes,
    )


def run_runtime_input_visual_probe(
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
    pre_duration_s: float,
    pulse_duration_s: float,
    neutral_duration_s: float,
    pixel_threshold: int,
    min_mean_absdiff: float,
    min_changed_ratio: float,
    max_baseline_mean_absdiff: float,
    baseline_multiplier: float,
    use_uinput: bool,
    device_name: str,
) -> RuntimeInputVisualResult:
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
        return RuntimeInputVisualResult(
            status=early.status,
            summary=early.summary,
            metrics={**early.metrics, "run_id": run_id, "real_input_sent": False},
            notes=list(early.notes),
        )
    assert source is not None

    adapter = UInputAdapter(name=device_name) if use_uinput else DryRunInputAdapter()
    try:
        result = measure_runtime_input_visual_change(
            source,
            adapter,
            command=command_for_axis(axis, magnitude),
            axis=axis,
            fps=fps,
            pre_duration_s=pre_duration_s,
            pulse_duration_s=pulse_duration_s,
            neutral_duration_s=neutral_duration_s,
            pixel_threshold=pixel_threshold,
            min_mean_absdiff=min_mean_absdiff,
            min_changed_ratio=min_changed_ratio,
            max_baseline_mean_absdiff=max_baseline_mean_absdiff,
            baseline_multiplier=baseline_multiplier,
        )
    finally:
        adapter.close()
    result.metrics.update(selection_metrics)
    result.metrics["run_id"] = run_id
    result.metrics["real_input_sent"] = use_uinput
    result.metrics["adapter"] = adapter.__class__.__name__
    result.metrics["device_name"] = device_name
    if not use_uinput:
        result.status = WAITING
        result.summary = "runtime input visual metric is informational because no real input was sent"
        result.notes.append("DRY_RUN_ONLY")
    return result


def build_markdown(result: RuntimeInputVisualResult, *, run_id: str) -> str:
    lines = [
        "# SimITL / pr0p Runtime Input Visual Probe",
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
        "real_input_sent",
        "adapter",
        "frames_before",
        "frames_pulse",
        "frames_neutral",
    ):
        if key in result.metrics:
            lines.append("| %s | `%s` |" % (key, result.metrics[key]))
    for section in ("baseline_summary", "pulse_summary", "neutral_summary"):
        summary = result.metrics.get(section)
        if isinstance(summary, dict):
            lines.append("| %s.max_mean_absdiff | `%s` |" % (
                section,
                summary.get("max_mean_absdiff"),
            ))
            lines.append("| %s.max_changed_ratio | `%s` |" % (
                section,
                summary.get("max_changed_ratio"),
            ))
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


def write_reports(result: RuntimeInputVisualResult, log_dir: Path, *, run_id: str) -> tuple[Path, Path]:
    log_dir.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime("%Y%m%d-%H%M%S", time.localtime())
    json_path = log_dir / ("%s-%s-runtime-input-visual.json" % (stamp, run_id))
    md_path = log_dir / ("%s-%s-runtime-input-visual.md" % (stamp, run_id))
    json_path.write_text(json.dumps(result.as_dict(), indent=2, sort_keys=True) + "\n",
                         encoding="utf-8")
    md_path.write_text(build_markdown(result, run_id=run_id), encoding="utf-8")
    return json_path, md_path


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-id", default="pr0p-runtime-input-visual")
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
    parser.add_argument("--backend", choices=("auto", "mss", "ffmpeg"), default="auto")
    parser.add_argument("--fps", type=float, default=12.0)
    parser.add_argument("--axis", choices=AXES, default="yaw")
    parser.add_argument("--magnitude", type=float, default=0.6)
    parser.add_argument("--pre-duration", type=float, default=0.5)
    parser.add_argument("--pulse-duration", type=float, default=1.0)
    parser.add_argument("--neutral-duration", type=float, default=0.4)
    parser.add_argument("--pixel-threshold", type=int, default=18)
    parser.add_argument("--min-mean-absdiff", type=float, default=1.0)
    parser.add_argument("--min-changed-ratio", type=float, default=0.004)
    parser.add_argument("--max-baseline-mean-absdiff", type=float, default=1.0)
    parser.add_argument("--baseline-multiplier", type=float, default=3.0)
    parser.add_argument("--device-name", default="Kenet Game Sandbox")
    parser.add_argument("--uinput", action="store_true",
                        help="Send a real uinput pulse")
    parser.add_argument("--ack-live-input", action="store_true",
                        help="Required before --uinput sends real OS input")
    args = parser.parse_args(argv)
    if args.window_titles is None:
        args.window_titles = list(DEFAULT_WINDOW_TITLES)
    if args.window_min_width <= 0 or args.window_min_height <= 0:
        parser.error("--window-min-width/--window-min-height must be positive")
    if args.fps <= 0:
        parser.error("--fps must be positive")
    if args.pre_duration <= 0 or args.pulse_duration <= 0:
        parser.error("--pre-duration/--pulse-duration must be positive")
    if args.neutral_duration < 0:
        parser.error("--neutral-duration must be non-negative")
    if not 0 <= args.pixel_threshold <= 255:
        parser.error("--pixel-threshold must be in 0..255")
    if args.min_mean_absdiff < 0 or args.min_changed_ratio < 0:
        parser.error("--min-mean-absdiff/--min-changed-ratio must be non-negative")
    if args.max_baseline_mean_absdiff < 0:
        parser.error("--max-baseline-mean-absdiff must be non-negative")
    if args.baseline_multiplier < 1.0:
        parser.error("--baseline-multiplier must be >= 1")
    if args.uinput and not args.ack_live_input:
        parser.error("--uinput requires --ack-live-input")
    return args


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    result = run_runtime_input_visual_probe(
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
        pre_duration_s=args.pre_duration,
        pulse_duration_s=args.pulse_duration,
        neutral_duration_s=args.neutral_duration,
        pixel_threshold=args.pixel_threshold,
        min_mean_absdiff=args.min_mean_absdiff,
        min_changed_ratio=args.min_changed_ratio,
        max_baseline_mean_absdiff=args.max_baseline_mean_absdiff,
        baseline_multiplier=args.baseline_multiplier,
        use_uinput=args.uinput,
        device_name=args.device_name,
    )
    json_path, md_path = write_reports(result, args.log_dir, run_id=args.run_id)
    print("simitl-pr0p-runtime-input-visual %s report=%s summary=%s real_input_sent=%s" % (
        result.status,
        json_path,
        md_path,
        result.metrics.get("real_input_sent"),
    ))
    print(result.summary)
    return 1 if result.status == FAIL else 0


if __name__ == "__main__":
    raise SystemExit(main())
