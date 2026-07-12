#!/usr/bin/env python3
"""Frame capture helpers for the game/screen sandbox.

This module has no Gazebo or Betaflight dependency. Real window capture is
optional and uses `mss` only when requested; tests can use SyntheticFrameSource.
"""

from __future__ import annotations

import argparse
import importlib.util
import math
import os
import shutil
import subprocess
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Iterator, Protocol

import cv2
import numpy as np

from x11_window import resolve_window_region


@dataclass(frozen=True)
class CaptureFrame:
    frame: np.ndarray
    timestamp: float
    index: int
    source: str


@dataclass(frozen=True)
class CaptureStats:
    frames: int
    duration_s: float
    fps: float
    width: int
    height: int
    nonblank: bool


class FrameSource(Protocol):
    def frames(self, duration_s: float | None = None) -> Iterator[CaptureFrame]:
        ...


def is_nonblank(frame: np.ndarray, *, min_std: float = 1.0) -> bool:
    """Return true if a frame carries visible content."""
    if frame.size == 0:
        return False
    return float(frame.std()) >= min_std


def parse_region(value: str) -> dict[str, int]:
    parts = [int(part.strip()) for part in value.split(",")]
    if len(parts) != 4:
        raise argparse.ArgumentTypeError("region must be left,top,width,height")
    left, top, width, height = parts
    if width <= 0 or height <= 0:
        raise argparse.ArgumentTypeError("region width/height must be positive")
    return {"left": left, "top": top, "width": width, "height": height}


def module_available(name: str) -> bool:
    return importlib.util.find_spec(name) is not None


class SyntheticFrameSource:
    """Deterministic target moving on a textured background."""

    def __init__(
        self,
        *,
        width: int = 640,
        height: int = 480,
        fps: float = 30.0,
        target_size: int = 80,
        target_speed_px_s: float = 40.0,
    ):
        if width <= 0 or height <= 0:
            raise ValueError("width/height must be positive")
        if fps <= 0:
            raise ValueError("fps must be positive")
        self.width = width
        self.height = height
        self.fps = fps
        self.target_size = target_size
        self.target_speed_px_s = target_speed_px_s

    def target_bbox_at(self, t: float) -> tuple[int, int, int, int]:
        cx = self.width / 2 + np.sin(t * 0.8) * self.width * 0.25
        cy = self.height / 2 + np.cos(t * 0.5) * self.height * 0.10
        half = self.target_size // 2
        x1 = int(max(0, min(self.width - self.target_size, cx - half)))
        y1 = int(max(0, min(self.height - self.target_size, cy - half)))
        return (x1, y1, self.target_size, self.target_size)

    def frames(self, duration_s: float | None = None) -> Iterator[CaptureFrame]:
        dt = 1.0 / self.fps
        total = int((duration_s if duration_s is not None else 1.0) * self.fps)
        start = time.monotonic()
        for index in range(max(0, total)):
            t = index * dt
            frame = self._make_frame(t)
            yield CaptureFrame(frame=frame, timestamp=start + t, index=index, source="synthetic")

    def _make_frame(self, t: float) -> np.ndarray:
        frame = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        # A mild texture helps trackers and catches black-frame mistakes.
        frame[:, :, 0] = np.linspace(20, 80, self.width, dtype=np.uint8)
        frame[:, :, 1] = np.linspace(30, 120, self.height, dtype=np.uint8)[:, None]
        frame[:, :, 2] = 50
        x1, y1, w, h = self.target_bbox_at(t)
        x2 = x1 + w
        y2 = y1 + h
        cv2.rectangle(frame, (x1, y1), (x2, y2), (40, 220, 245), -1)
        cv2.rectangle(frame, (x1 + 8, y1 + 8), (x2 - 8, y2 - 8), (10, 80, 120), 3)
        cv2.line(frame, (x1, y1), (x2, y2), (255, 255, 255), 2)
        return frame


class VideoFileFrameSource:
    def __init__(self, path: str | Path):
        self.path = Path(path)

    def frames(self, duration_s: float | None = None) -> Iterator[CaptureFrame]:
        cap = cv2.VideoCapture(str(self.path))
        if not cap.isOpened():
            raise RuntimeError("could not open video: %s" % self.path)
        fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
        dt = 1.0 / fps
        start = time.monotonic()
        index = 0
        try:
            while True:
                if duration_s is not None and index * dt >= duration_s:
                    break
                ok, frame = cap.read()
                if not ok:
                    break
                yield CaptureFrame(frame=frame, timestamp=start + index * dt, index=index,
                                   source=str(self.path))
                index += 1
        finally:
            cap.release()


class MSSRegionFrameSource:
    """Optional real screen capture using mss."""

    def __init__(self, region: dict[str, int], *, fps: float = 30.0):
        if fps <= 0:
            raise ValueError("fps must be positive")
        self.region = dict(region)
        self.fps = fps

    def frames(self, duration_s: float | None = None) -> Iterator[CaptureFrame]:
        try:
            import mss  # type: ignore
        except ImportError as exc:
            raise RuntimeError("mss is required for screen capture; install it in fpv_env") from exc

        interval = 1.0 / self.fps
        start = time.monotonic()
        index = 0
        with mss.mss() as grabber:
            while True:
                now = time.monotonic()
                if duration_s is not None and now - start >= duration_s:
                    break
                shot = grabber.grab(self.region)
                bgra = np.asarray(shot)
                frame = cv2.cvtColor(bgra, cv2.COLOR_BGRA2BGR)
                yield CaptureFrame(frame=frame, timestamp=now, index=index, source="mss")
                index += 1
                elapsed = time.monotonic() - now
                if elapsed < interval:
                    time.sleep(interval - elapsed)


class FFmpegX11RegionFrameSource:
    """Real X11 region capture through ffmpeg/x11grab."""

    def __init__(
        self,
        region: dict[str, int],
        *,
        fps: float = 30.0,
        display: str | None = None,
        ffmpeg_bin: str = "ffmpeg",
    ):
        if fps <= 0:
            raise ValueError("fps must be positive")
        self.region = dict(region)
        self.fps = fps
        self.display = display or os.environ.get("DISPLAY")
        self.ffmpeg_bin = ffmpeg_bin

    @property
    def frame_bytes(self) -> int:
        return self.region["width"] * self.region["height"] * 3

    def command(self, frame_count: int | None = None) -> list[str]:
        if not self.display:
            raise RuntimeError("DISPLAY is required for ffmpeg x11grab capture")
        source = "%s+%d,%d" % (self.display, self.region["left"], self.region["top"])
        cmd = [
            self.ffmpeg_bin,
            "-nostdin",
            "-hide_banner",
            "-loglevel",
            "error",
            "-f",
            "x11grab",
            "-draw_mouse",
            "0",
            "-video_size",
            "%dx%d" % (self.region["width"], self.region["height"]),
            "-framerate",
            "%.6g" % self.fps,
            "-i",
            source,
            "-pix_fmt",
            "bgr24",
            "-f",
            "rawvideo",
        ]
        if frame_count is not None:
            cmd.extend(["-frames:v", str(frame_count)])
        cmd.append("pipe:1")
        return cmd

    def frames(self, duration_s: float | None = None) -> Iterator[CaptureFrame]:
        if shutil.which(self.ffmpeg_bin) is None:
            raise RuntimeError("ffmpeg is required for x11grab capture")
        frame_count = None
        if duration_s is not None:
            frame_count = max(1, int(math.ceil(duration_s * self.fps)))
        process = subprocess.Popen(
            self.command(frame_count),
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )
        assert process.stdout is not None
        assert process.stderr is not None
        emitted = 0
        start = time.monotonic()
        try:
            while frame_count is None or emitted < frame_count:
                raw = process.stdout.read(self.frame_bytes)
                if len(raw) < self.frame_bytes:
                    break
                frame = np.frombuffer(raw, dtype=np.uint8).reshape(
                    (self.region["height"], self.region["width"], 3)
                ).copy()
                yield CaptureFrame(
                    frame=frame,
                    timestamp=start + emitted / self.fps,
                    index=emitted,
                    source="ffmpeg-x11grab",
                )
                emitted += 1
        finally:
            if process.poll() is None:
                process.terminate()
                try:
                    process.wait(timeout=2.0)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=2.0)
        stderr = process.stderr.read().decode("utf-8", errors="replace").strip()
        if emitted == 0 and process.returncode not in (0, None):
            raise RuntimeError("ffmpeg x11grab failed: %s" % (stderr or process.returncode))


def make_region_source(
    region: dict[str, int],
    *,
    fps: float,
    backend: str = "auto",
) -> FrameSource:
    if backend == "mss":
        return MSSRegionFrameSource(region, fps=fps)
    if backend == "ffmpeg":
        return FFmpegX11RegionFrameSource(region, fps=fps)
    if backend != "auto":
        raise ValueError("unknown capture backend: %s" % backend)
    if module_available("mss"):
        return MSSRegionFrameSource(region, fps=fps)
    if shutil.which("ffmpeg") and os.environ.get("DISPLAY"):
        return FFmpegX11RegionFrameSource(region, fps=fps)
    raise RuntimeError("no real screen capture backend available; try mss or ffmpeg/x11grab")


def resolve_capture_region(
    *,
    region: dict[str, int] | None = None,
    window_title: str | None = None,
    window_exact: bool = False,
    window_case_sensitive: bool = False,
    window_min_width: int = 32,
    window_min_height: int = 32,
) -> dict[str, int]:
    if region is not None:
        return region
    if window_title:
        return resolve_window_region(
            window_title,
            exact=window_exact,
            case_sensitive=window_case_sensitive,
            min_width=window_min_width,
            min_height=window_min_height,
        )
    raise ValueError("a region or window title is required")


def sample_source(source: FrameSource, *, duration_s: float) -> CaptureStats:
    first_shape: tuple[int, int] | None = None
    nonblank = False
    first_t: float | None = None
    last_t: float | None = None
    count = 0
    for item in source.frames(duration_s):
        if first_shape is None:
            first_shape = item.frame.shape[:2]
            first_t = item.timestamp
        last_t = item.timestamp
        nonblank = nonblank or is_nonblank(item.frame)
        count += 1
    if not first_shape:
        return CaptureStats(frames=0, duration_s=0.0, fps=0.0, width=0, height=0, nonblank=False)
    measured = max(0.0, (last_t or first_t or 0.0) - (first_t or 0.0))
    fps = count / measured if measured > 0 else float(count)
    height, width = first_shape
    return CaptureStats(frames=count, duration_s=measured, fps=fps, width=width,
                        height=height, nonblank=nonblank)


def save_first_frame(source: FrameSource, path: str | Path, *, duration_s: float) -> Path:
    output = Path(path)
    output.parent.mkdir(parents=True, exist_ok=True)
    for item in source.frames(duration_s):
        if not cv2.imwrite(str(output), item.frame):
            raise RuntimeError("could not write frame: %s" % output)
        return output
    raise RuntimeError("no frame available to save")


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--synthetic", action="store_true")
    mode.add_argument("--video")
    mode.add_argument("--region", type=parse_region,
                      help="Screen region: left,top,width,height")
    mode.add_argument("--window-title",
                      help="Find an X11 window by title and capture its region")
    parser.add_argument("--backend", choices=["auto", "mss", "ffmpeg"], default="auto",
                        help="Capture backend for --region")
    parser.add_argument("--window-exact", action="store_true")
    parser.add_argument("--window-case-sensitive", action="store_true")
    parser.add_argument("--window-min-width", type=int, default=32)
    parser.add_argument("--window-min-height", type=int, default=32)
    parser.add_argument("--duration", type=float, default=5.0)
    parser.add_argument("--fps", type=float, default=30.0)
    parser.add_argument("--save-frame",
                        help="Optional path for one captured frame image")
    args = parser.parse_args(argv)
    if args.duration <= 0:
        parser.error("--duration must be positive")
    if args.fps <= 0:
        parser.error("--fps must be positive")
    if args.window_min_width <= 0 or args.window_min_height <= 0:
        parser.error("--window-min-width/--window-min-height must be positive")
    return args


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    if args.synthetic:
        source: FrameSource = SyntheticFrameSource(fps=args.fps)
    elif args.video:
        source = VideoFileFrameSource(args.video)
    else:
        region = resolve_capture_region(
            region=args.region,
            window_title=args.window_title,
            window_exact=args.window_exact,
            window_case_sensitive=args.window_case_sensitive,
            window_min_width=args.window_min_width,
            window_min_height=args.window_min_height,
        )
        source = make_region_source(region, fps=args.fps, backend=args.backend)
    stats = sample_source(source, duration_s=args.duration)
    if args.save_frame:
        save_first_frame(source, args.save_frame, duration_s=max(args.duration, 1.0 / args.fps))
    verdict = "PASS" if stats.frames > 0 and stats.nonblank else "FAIL"
    frame_note = " saved=%s" % args.save_frame if args.save_frame else ""
    print("capture %s frames=%d fps=%.1f size=%dx%d nonblank=%s%s" % (
        verdict, stats.frames, stats.fps, stats.width, stats.height, stats.nonblank,
        frame_note))
    return 0 if verdict == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
