#!/usr/bin/env python3
"""Capture a frame and validate/visualize a target bbox for the sandbox."""

from __future__ import annotations

import argparse
import json
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable

import cv2

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
from screen_tracking_loop import parse_bbox  # noqa: E402


PASS = "PASS"
FAIL = "FAIL"
WAITING = "WAITING"


@dataclass(frozen=True)
class BboxCheck:
    status: str
    frame_path: str
    overlay_path: str | None
    bbox: tuple[int, int, int, int] | None
    frame_width: int
    frame_height: int
    notes: list[str]

    def as_dict(self) -> dict[str, Any]:
        return {
            "status": self.status,
            "frame_path": self.frame_path,
            "overlay_path": self.overlay_path,
            "bbox": list(self.bbox) if self.bbox else None,
            "frame_width": self.frame_width,
            "frame_height": self.frame_height,
            "notes": self.notes,
        }


def default_output_path(prefix: str, suffix: str = ".png") -> Path:
    stamp = time.strftime("%Y%m%d-%H%M%S", time.localtime())
    return REPO_ROOT / "logs" / "game_screen_sandbox" / ("%s-%s%s" % (stamp, prefix, suffix))


def bbox_inside_frame(bbox: tuple[int, int, int, int], *, width: int, height: int) -> bool:
    x, y, w, h = bbox
    return x >= 0 and y >= 0 and w > 0 and h > 0 and x + w <= width and y + h <= height


def draw_bbox(frame, bbox: tuple[int, int, int, int], *, label: str = "target"):
    x, y, w, h = bbox
    overlay = frame.copy()
    cv2.rectangle(overlay, (x, y), (x + w, y + h), (0, 255, 255), 3)
    cv2.circle(overlay, (x + w // 2, y + h // 2), 5, (0, 0, 255), -1)
    cv2.putText(
        overlay,
        "%s %d,%d,%d,%d" % (label, x, y, w, h),
        (max(0, x), max(20, y - 10)),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (0, 255, 255),
        2,
        cv2.LINE_AA,
    )
    return overlay


def select_bbox_interactively(frame, *, window_name: str = "Select target bbox"):
    bbox = cv2.selectROI(window_name, frame, showCrosshair=True, fromCenter=False)
    cv2.destroyWindow(window_name)
    x, y, w, h = [int(part) for part in bbox]
    if w <= 0 or h <= 0:
        return None
    return x, y, w, h


def first_frame(source: FrameSource, *, duration_s: float):
    for item in source.frames(duration_s):
        return item.frame
    raise RuntimeError("no frame captured")


def prepare_bbox_artifacts(
    source: FrameSource,
    *,
    bbox: tuple[int, int, int, int] | None,
    frame_path: Path,
    overlay_path: Path | None = None,
    duration_s: float = 1.0,
    interactive_select: bool = False,
    select_fn: Callable[[Any], tuple[int, int, int, int] | None] | None = None,
) -> BboxCheck:
    frame = first_frame(source, duration_s=duration_s)
    height, width = frame.shape[:2]
    frame_path.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(frame_path), frame):
        raise RuntimeError("could not write frame: %s" % frame_path)

    selected_bbox = bbox
    notes: list[str] = []
    if selected_bbox is None and interactive_select:
        selector = select_fn or (lambda image: select_bbox_interactively(image))
        selected_bbox = selector(frame)
        if selected_bbox is None:
            return BboxCheck(
                status=WAITING,
                frame_path=str(frame_path),
                overlay_path=None,
                bbox=None,
                frame_width=width,
                frame_height=height,
                notes=["interactive bbox selection was cancelled"],
            )
        notes.append("bbox selected interactively")

    if selected_bbox is None:
        return BboxCheck(
            status=WAITING,
            frame_path=str(frame_path),
            overlay_path=None,
            bbox=None,
            frame_width=width,
            frame_height=height,
            notes=[
                "open the frame image, choose target bbox as x,y,w,h, then rerun with --bbox",
            ],
        )

    if not bbox_inside_frame(selected_bbox, width=width, height=height):
        return BboxCheck(
            status=FAIL,
            frame_path=str(frame_path),
            overlay_path=None,
            bbox=selected_bbox,
            frame_width=width,
            frame_height=height,
            notes=["bbox is outside the captured frame"],
        )

    resolved_overlay = overlay_path or frame_path.with_name("%s-bbox%s" % (
        frame_path.stem,
        frame_path.suffix,
    ))
    overlay = draw_bbox(frame, selected_bbox)
    if not cv2.imwrite(str(resolved_overlay), overlay):
        raise RuntimeError("could not write bbox overlay: %s" % resolved_overlay)
    return BboxCheck(
        status=PASS,
        frame_path=str(frame_path),
        overlay_path=str(resolved_overlay),
        bbox=selected_bbox,
        frame_width=width,
        frame_height=height,
        notes=notes + ["use this bbox with phase_runner.py --tracker-bbox"],
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


def waiting_without_frame(*, frame_path: Path, note: str) -> BboxCheck:
    return BboxCheck(
        status=WAITING,
        frame_path=str(frame_path),
        overlay_path=None,
        bbox=None,
        frame_width=0,
        frame_height=0,
        notes=[note],
    )


def is_missing_window_error(exc: RuntimeError) -> bool:
    return str(exc).startswith("no visible X11 window matched title:")


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
    parser.add_argument("--bbox", type=parse_bbox)
    parser.add_argument("--interactive-select", action="store_true",
                        help="Open the captured frame and select the target ROI")
    parser.add_argument("--frame-path", type=Path)
    parser.add_argument("--overlay-path", type=Path)
    parser.add_argument("--report-path", type=Path)
    parser.add_argument("--duration", type=float, default=1.0)
    parser.add_argument("--fps", type=float, default=5.0)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    args = parser.parse_args(argv)
    if args.duration <= 0:
        parser.error("--duration must be positive")
    if args.fps <= 0:
        parser.error("--fps must be positive")
    if args.width <= 0 or args.height <= 0:
        parser.error("--width/--height must be positive")
    if args.window_min_width <= 0 or args.window_min_height <= 0:
        parser.error("--window-min-width/--window-min-height must be positive")
    return args


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    frame_path = args.frame_path or default_output_path("target-frame")
    try:
        source = make_source(args)
        result = prepare_bbox_artifacts(
            source,
            bbox=args.bbox,
            frame_path=frame_path,
            overlay_path=args.overlay_path,
            duration_s=args.duration,
            interactive_select=args.interactive_select,
        )
    except RuntimeError as exc:
        if not is_missing_window_error(exc):
            raise
        result = waiting_without_frame(
            frame_path=frame_path,
            note="%s; open the window or pass the correct --window-title" % exc,
        )
    if args.report_path:
        args.report_path.parent.mkdir(parents=True, exist_ok=True)
        args.report_path.write_text(
            json.dumps(result.as_dict(), indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
    overlay_note = " overlay=%s" % result.overlay_path if result.overlay_path else ""
    bbox_note = " bbox=%s" % ",".join(str(v) for v in result.bbox) if result.bbox else ""
    print("bbox %s frame=%s%s%s size=%dx%d" % (
        result.status,
        result.frame_path,
        overlay_note,
        bbox_note,
        result.frame_width,
        result.frame_height,
    ))
    return 0 if result.status == PASS else 2 if result.status == WAITING else 1


if __name__ == "__main__":
    raise SystemExit(main())
