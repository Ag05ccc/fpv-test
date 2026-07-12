#!/usr/bin/env python3
"""Prepare/validate a tracking bbox for the isolated SimITL/pr0p probe."""

from __future__ import annotations

import argparse
import json
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

REPO_ROOT = Path(__file__).resolve().parents[2]
SANDBOX_DIR = REPO_ROOT / "experiments" / "game_screen_sandbox"
if str(SANDBOX_DIR) not in sys.path:
    sys.path.insert(0, str(SANDBOX_DIR))
PROBE_DIR = Path(__file__).resolve().parent
if str(PROBE_DIR) not in sys.path:
    sys.path.insert(0, str(PROBE_DIR))

from bbox_tool import prepare_bbox_artifacts  # noqa: E402
from capture_window import FrameSource, make_region_source, parse_region  # noqa: E402
from pr0p_capture_probe import (  # noqa: E402
    DEFAULT_LOG_DIR,
    DEFAULT_WINDOW_TITLES,
    apply_relative_crop,
    find_pr0p_window,
)
from screen_tracking_loop import parse_bbox  # noqa: E402
from x11_window import parse_size  # noqa: E402


PASS = "PASS"
WAITING = "WAITING"
FAIL = "FAIL"


@dataclass
class BboxProbeResult:
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


def analyze_bbox_source(
    source: FrameSource,
    *,
    bbox: tuple[int, int, int, int] | None,
    frame_path: Path,
    overlay_path: Path | None,
    duration_s: float,
) -> BboxProbeResult:
    try:
        check = prepare_bbox_artifacts(
            source,
            bbox=bbox,
            frame_path=frame_path,
            overlay_path=overlay_path,
            duration_s=duration_s,
        )
    except Exception as exc:
        return BboxProbeResult(
            status=FAIL,
            summary="bbox frame capture raised an error",
            metrics={"error": str(exc), "frame_path": str(frame_path)},
            notes=["BBOX_CAPTURE_ERROR"],
        )
    summary = {
        PASS: "bbox is valid for the captured frame",
        WAITING: "captured sample frame; bbox has not been selected yet",
        FAIL: "bbox is invalid for the captured frame",
    }.get(check.status, "bbox probe completed")
    metrics = check.as_dict()
    metrics["bbox_cli"] = ",".join(str(v) for v in check.bbox) if check.bbox else None
    return BboxProbeResult(
        status=check.status,
        summary=summary,
        metrics=metrics,
        notes=list(check.notes),
    )


def run_bbox_probe(
    *,
    run_id: str,
    log_dir: Path,
    region: dict[str, int] | None,
    crop: dict[str, int] | None,
    window_titles: list[str],
    exact: bool,
    case_sensitive: bool,
    min_width: int,
    min_height: int,
    preferred_size: tuple[int, int] | None,
    allow_common_non_fpv_windows: bool,
    backend: str,
    fps: float,
    duration_s: float,
    bbox: tuple[int, int, int, int] | None,
    frame_path: Path | None,
    overlay_path: Path | None,
) -> BboxProbeResult:
    log_dir.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime("%Y%m%d-%H%M%S", time.localtime())
    resolved_frame_path = frame_path or log_dir / ("%s-%s-target-frame.png" % (stamp, run_id))

    selected_region = region
    selected_window = None
    visible_sample: list[dict[str, Any]] = []
    if selected_region is None:
        kwargs: dict[str, Any] = {}
        if allow_common_non_fpv_windows:
            kwargs["excluded_terms"] = ()
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
            return BboxProbeResult(
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
        return BboxProbeResult(
            status=FAIL,
            summary="capture crop is invalid",
            metrics={"selected_region": selected_region, "crop": crop, "error": str(exc)},
            notes=["CAPTURE_CROP_INVALID"],
        )

    try:
        source = make_region_source(capture_region, fps=fps, backend=backend)
    except Exception as exc:
        return BboxProbeResult(
            status=WAITING,
            summary="no usable real screen capture backend is available",
            metrics={"backend": backend, "capture_region": capture_region, "error": str(exc)},
            notes=["CAPTURE_BACKEND_UNAVAILABLE"],
        )

    result = analyze_bbox_source(
        source,
        bbox=bbox,
        frame_path=resolved_frame_path,
        overlay_path=overlay_path,
        duration_s=duration_s,
    )
    result.metrics.update({
        "run_id": run_id,
        "backend": backend,
        "fps": fps,
        "duration_s": duration_s,
        "selected_region": selected_region,
        "capture_region": capture_region,
        "crop": crop,
        "selected_window": selected_window,
        "visible_windows_sample": visible_sample,
    })
    return result


def build_markdown(result: BboxProbeResult, *, run_id: str) -> str:
    lines = [
        "# SimITL / pr0p Bbox Tool",
        "",
        "Run: `%s`" % run_id,
        "Verdict: `%s`" % result.status,
        "",
        result.summary,
        "",
        "| Metric | Value |",
        "| --- | --- |",
    ]
    for key in ("frame_path", "overlay_path", "bbox_cli", "frame_width", "frame_height"):
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


def write_reports(result: BboxProbeResult, log_dir: Path, *, run_id: str) -> tuple[Path, Path]:
    log_dir.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime("%Y%m%d-%H%M%S", time.localtime())
    json_path = log_dir / ("%s-%s-bbox-tool.json" % (stamp, run_id))
    md_path = log_dir / ("%s-%s-bbox-tool.md" % (stamp, run_id))
    json_path.write_text(
        json.dumps(result.as_dict(), indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    md_path.write_text(build_markdown(result, run_id=run_id), encoding="utf-8")
    return json_path, md_path


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-id", default="pr0p-bbox")
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
    parser.add_argument("--fps", type=float, default=5.0)
    parser.add_argument("--duration", type=float, default=1.0)
    parser.add_argument("--bbox", type=parse_bbox)
    parser.add_argument("--frame-path", type=Path)
    parser.add_argument("--overlay-path", type=Path)
    args = parser.parse_args(argv)
    if args.window_titles is None:
        args.window_titles = list(DEFAULT_WINDOW_TITLES)
    if args.window_min_width <= 0 or args.window_min_height <= 0:
        parser.error("--window-min-width/--window-min-height must be positive")
    if args.fps <= 0:
        parser.error("--fps must be positive")
    if args.duration <= 0:
        parser.error("--duration must be positive")
    return args


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    result = run_bbox_probe(
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
        bbox=args.bbox,
        frame_path=args.frame_path,
        overlay_path=args.overlay_path,
    )
    json_path, md_path = write_reports(result, args.log_dir, run_id=args.run_id)
    print("simitl-pr0p-bbox %s report=%s summary=%s frame=%s overlay=%s bbox=%s" % (
        result.status,
        json_path,
        md_path,
        result.metrics.get("frame_path"),
        result.metrics.get("overlay_path"),
        result.metrics.get("bbox_cli"),
    ))
    print(result.summary)
    return 1 if result.status == FAIL else 0


if __name__ == "__main__":
    raise SystemExit(main())
