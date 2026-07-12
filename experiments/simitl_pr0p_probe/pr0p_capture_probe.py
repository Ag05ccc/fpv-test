#!/usr/bin/env python3
"""Image capture probe for the isolated SimITL/pr0p experiment."""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Iterable

import cv2

REPO_ROOT = Path(__file__).resolve().parents[2]
SANDBOX_DIR = REPO_ROOT / "experiments" / "game_screen_sandbox"
if str(SANDBOX_DIR) not in sys.path:
    sys.path.insert(0, str(SANDBOX_DIR))

from capture_window import (  # noqa: E402
    CaptureFrame,
    FrameSource,
    is_nonblank,
    make_region_source,
    parse_region,
)
from x11_window import (  # noqa: E402
    X11Window,
    find_window_by_title,
    list_x11_windows,
    parse_size,
    visible_windows,
)


PASS = "PASS"
WAITING = "WAITING"
FAIL = "FAIL"
DEFAULT_LOG_DIR = Path("logs/simitl_pr0p")
DEFAULT_WINDOW_TITLES = ("pr0p",)
DEFAULT_EXCLUDED_WINDOW_TERMS = (
    "visual studio code",
    "google chrome",
    "mozilla firefox",
    "chromium",
    "terminal",
    "konsole",
    "xterm",
    "x-terminal-emulator",
    "/tmp/fpv-test-simitl-pr0p/updater",
    "pr0p updater",
)


@dataclass
class CaptureProbeResult:
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


def apply_relative_crop(region: dict[str, int], crop: dict[str, int] | None) -> dict[str, int]:
    if crop is None:
        return dict(region)
    left = int(region["left"]) + int(crop["left"])
    top = int(region["top"]) + int(crop["top"])
    width = int(crop["width"])
    height = int(crop["height"])
    if crop["left"] < 0 or crop["top"] < 0:
        raise ValueError("crop left/top must be non-negative")
    if width <= 0 or height <= 0:
        raise ValueError("crop width/height must be positive")
    if crop["left"] + width > region["width"] or crop["top"] + height > region["height"]:
        raise ValueError("crop must fit inside the selected window/region")
    return {
        "left": left,
        "top": top,
        "width": width,
        "height": height,
    }


def window_sample(windows: Iterable[X11Window], *, limit: int = 12) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for window in list(windows)[:limit]:
        rows.append({
            "window_id": window.window_id,
            "title": window.title,
            "class_text": window.class_text,
            "region": window.region(),
        })
    return rows


def is_excluded_window(window: X11Window, excluded_terms: Iterable[str]) -> bool:
    haystack = "%s %s" % (window.title.lower(), window.class_text.lower())
    return any(term.lower() in haystack for term in excluded_terms)


def find_pr0p_window(
    titles: Iterable[str],
    *,
    exact: bool,
    case_sensitive: bool,
    min_width: int,
    min_height: int,
    preferred_size: tuple[int, int] | None,
    excluded_terms: Iterable[str] = DEFAULT_EXCLUDED_WINDOW_TERMS,
) -> tuple[X11Window | None, list[X11Window], str | None]:
    try:
        windows = visible_windows(
            list_x11_windows(),
            min_width=min_width,
            min_height=min_height,
        )
        candidate_windows = [
            window for window in windows
            if not is_excluded_window(window, excluded_terms)
        ]
    except (OSError, RuntimeError, subprocess.CalledProcessError) as exc:
        return None, [], str(exc)
    for title in titles:
        match = find_window_by_title(
            title,
            candidate_windows,
            exact=exact,
            case_sensitive=case_sensitive,
            min_width=min_width,
            min_height=min_height,
            preferred_size=preferred_size,
        )
        if match is not None:
            return match, windows, None
    return None, windows, None


def analyze_capture_source(
    source: FrameSource,
    *,
    duration_s: float,
    min_fps: float,
    sample_frame_path: Path,
    min_std: float = 1.0,
) -> CaptureProbeResult:
    sample_frame_path.parent.mkdir(parents=True, exist_ok=True)
    first_t: float | None = None
    last_t: float | None = None
    width = 0
    height = 0
    frames = 0
    nonblank_frames = 0
    first_frame_std: float | None = None
    max_frame_std = 0.0
    saved_sample = False

    try:
        for item in source.frames(duration_s):
            frame = item.frame
            if first_t is None:
                first_t = item.timestamp
                height, width = frame.shape[:2]
                first_frame_std = float(frame.std())
                if not cv2.imwrite(str(sample_frame_path), frame):
                    raise RuntimeError("could not write sample frame: %s" % sample_frame_path)
                saved_sample = True
            last_t = item.timestamp
            frames += 1
            frame_std = float(frame.std())
            max_frame_std = max(max_frame_std, frame_std)
            if is_nonblank(frame, min_std=min_std):
                nonblank_frames += 1
    except Exception as exc:
        return CaptureProbeResult(
            status=FAIL,
            summary="capture source raised an error",
            metrics={
                "error": str(exc),
                "duration_s": duration_s,
                "min_fps": min_fps,
                "sample_frame_path": str(sample_frame_path),
            },
            notes=["CAPTURE_SOURCE_ERROR"],
        )

    measured_s = max(0.0, (last_t or first_t or 0.0) - (first_t or 0.0))
    fps = frames / measured_s if measured_s > 0 else float(frames)
    nonblank_ratio = nonblank_frames / frames if frames else 0.0
    metrics = {
        "frames": frames,
        "duration_s": measured_s,
        "requested_duration_s": duration_s,
        "fps": fps,
        "min_fps": min_fps,
        "width": width,
        "height": height,
        "nonblank_frames": nonblank_frames,
        "nonblank_ratio": nonblank_ratio,
        "first_frame_std": first_frame_std,
        "max_frame_std": max_frame_std,
        "sample_frame_path": str(sample_frame_path) if saved_sample else None,
        "min_std": min_std,
    }

    if frames == 0:
        return CaptureProbeResult(
            status=FAIL,
            summary="capture produced no frames",
            metrics=metrics,
            notes=["CAPTURE_NO_FRAMES"],
        )
    if nonblank_frames == 0:
        return CaptureProbeResult(
            status=FAIL,
            summary="capture frames are blank",
            metrics=metrics,
            notes=["CAPTURE_BLACK_FRAME"],
        )
    if fps < min_fps:
        return CaptureProbeResult(
            status=FAIL,
            summary="capture FPS is below the configured minimum",
            metrics=metrics,
            notes=["CAPTURE_TOO_SLOW"],
        )
    return CaptureProbeResult(
        status=PASS,
        summary="screen capture produced stable nonblank frames",
        metrics=metrics,
        notes=["sample frame is saved for visual confirmation"],
    )


def run_capture_probe(
    *,
    run_id: str,
    log_dir: Path,
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
    duration_s: float,
    min_fps: float,
    min_std: float,
) -> CaptureProbeResult:
    log_dir.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime("%Y%m%d-%H%M%S", time.localtime())
    sample_frame_path = log_dir / ("%s-%s-capture-sample.png" % (stamp, run_id))

    selected_window: dict[str, Any] | None = None
    visible_sample: list[dict[str, Any]] = []
    window_error: str | None = None
    selected_region = region
    if selected_region is None:
        match, windows, window_error = find_pr0p_window(
            window_titles,
            exact=exact,
            case_sensitive=case_sensitive,
            min_width=min_width,
            min_height=min_height,
            preferred_size=preferred_size,
            excluded_terms=() if allow_common_non_fpv_windows else DEFAULT_EXCLUDED_WINDOW_TERMS,
        )
        visible_sample = window_sample(windows)
        if match is None:
            return CaptureProbeResult(
                status=WAITING,
                summary="no matching pr0p/FPV window is visible yet",
                metrics={
                    "window_titles": list(window_titles),
                    "window_error": window_error,
                    "visible_windows_sample": visible_sample,
                    "excluded_window_terms": (
                        [] if allow_common_non_fpv_windows else list(DEFAULT_EXCLUDED_WINDOW_TERMS)
                    ),
                },
                notes=["CAPTURE_NO_WINDOW", "start pr0p/local race or pass --region"],
            )
        selected_window = match.as_dict()
        selected_region = match.region()

    try:
        capture_region = apply_relative_crop(selected_region, crop)
    except ValueError as exc:
        return CaptureProbeResult(
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
        return CaptureProbeResult(
            status=WAITING,
            summary="no usable real screen capture backend is available",
            metrics={
                "backend": backend,
                "capture_region": capture_region,
                "error": str(exc),
            },
            notes=["CAPTURE_BACKEND_UNAVAILABLE"],
        )

    result = analyze_capture_source(
        source,
        duration_s=duration_s,
        min_fps=min_fps,
        sample_frame_path=sample_frame_path,
        min_std=min_std,
    )
    result.metrics.update({
        "run_id": run_id,
        "backend": backend,
        "requested_fps": fps,
        "selected_region": selected_region,
        "capture_region": capture_region,
        "crop": crop,
        "selected_window": selected_window,
        "visible_windows_sample": visible_sample,
        "excluded_window_terms": (
            [] if allow_common_non_fpv_windows else list(DEFAULT_EXCLUDED_WINDOW_TERMS)
        ),
    })
    return result


def build_markdown(result: CaptureProbeResult, *, run_id: str) -> str:
    lines = [
        "# SimITL / pr0p Capture Probe",
        "",
        "Run: `%s`" % run_id,
        "Verdict: `%s`" % result.status,
        "",
        result.summary,
        "",
        "| Metric | Value |",
        "| --- | --- |",
    ]
    for key in ("frames", "fps", "width", "height", "nonblank_ratio", "sample_frame_path"):
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


def write_reports(result: CaptureProbeResult, log_dir: Path, *, run_id: str) -> tuple[Path, Path]:
    log_dir.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime("%Y%m%d-%H%M%S", time.localtime())
    json_path = log_dir / ("%s-%s-capture-probe.json" % (stamp, run_id))
    md_path = log_dir / ("%s-%s-capture-probe.md" % (stamp, run_id))
    json_path.write_text(
        json.dumps(result.as_dict(), indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    md_path.write_text(build_markdown(result, run_id=run_id), encoding="utf-8")
    return json_path, md_path


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-id", default="pr0p-capture")
    parser.add_argument("--log-dir", type=Path, default=DEFAULT_LOG_DIR)
    parser.add_argument("--region", type=parse_region,
                        help="Absolute screen region: left,top,width,height")
    parser.add_argument("--crop", type=parse_region,
                        help="Relative crop inside the selected window/region")
    parser.add_argument("--window-title", action="append", dest="window_titles",
                        help="Window title substring. Repeatable.")
    parser.add_argument("--window-exact", action="store_true")
    parser.add_argument("--window-case-sensitive", action="store_true")
    parser.add_argument("--allow-common-non-fpv-windows", action="store_true",
                        help="Do not exclude editor/browser/terminal windows from title matches")
    parser.add_argument("--window-min-width", type=int, default=160)
    parser.add_argument("--window-min-height", type=int, default=120)
    parser.add_argument("--preferred-size", type=parse_size)
    parser.add_argument("--backend", choices=["auto", "mss", "ffmpeg"], default="auto")
    parser.add_argument("--fps", type=float, default=30.0)
    parser.add_argument("--duration", type=float, default=5.0)
    parser.add_argument("--min-fps", type=float, default=20.0)
    parser.add_argument("--min-std", type=float, default=1.0)
    args = parser.parse_args(argv)
    if args.window_min_width <= 0 or args.window_min_height <= 0:
        parser.error("--window-min-width/--window-min-height must be positive")
    if args.fps <= 0:
        parser.error("--fps must be positive")
    if args.duration <= 0:
        parser.error("--duration must be positive")
    if args.min_fps < 0:
        parser.error("--min-fps must be non-negative")
    if args.min_std < 0:
        parser.error("--min-std must be non-negative")
    if args.window_titles is None:
        args.window_titles = list(DEFAULT_WINDOW_TITLES)
    return args


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    result = run_capture_probe(
        run_id=args.run_id,
        log_dir=args.log_dir,
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
        duration_s=args.duration,
        min_fps=args.min_fps,
        min_std=args.min_std,
    )
    json_path, md_path = write_reports(result, args.log_dir, run_id=args.run_id)
    sample = result.metrics.get("sample_frame_path")
    print("simitl-pr0p-capture %s report=%s summary=%s sample=%s" % (
        result.status,
        json_path,
        md_path,
        sample,
    ))
    print(result.summary)
    return 1 if result.status == FAIL else 0


if __name__ == "__main__":
    raise SystemExit(main())
