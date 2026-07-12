#!/usr/bin/env python3
"""Minimal capture -> tracker -> PID -> input loop for the game sandbox."""

from __future__ import annotations

import argparse
import math
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Protocol

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))
TOOLS_DIR = REPO_ROOT / "tools"
if str(TOOLS_DIR) not in sys.path:
    sys.path.insert(0, str(TOOLS_DIR))

from capture_window import (  # noqa: E402
    CaptureFrame,
    FrameSource,
    SyntheticFrameSource,
    VideoFileFrameSource,
    make_region_source,
    parse_region,
    resolve_capture_region,
)
from virtual_input import (  # noqa: E402
    AxisCommand,
    DryRunInputAdapter,
    InputAdapter,
    UInputAdapter,
    command_from_channels,
)
from kenet.controller import FlightController, PIDGains  # noqa: E402
from kenet.pipeline import PipelineConfig  # noqa: E402
from kenet.tracker import ObjectTracker, TrackResult, TrackerType  # noqa: E402
from sitl_log import JsonlLogger, make_log_path, resolve_log_dir  # noqa: E402


class TrackerLike(Protocol):
    @property
    def is_initialized(self) -> bool:
        ...

    def init(self, frame, bbox) -> None:
        ...

    def update(self, frame) -> TrackResult:
        ...


@dataclass
class LoopConfig:
    duration_s: float = 5.0
    hz: float = 15.0
    initial_bbox: tuple[int, int, int, int] | None = None
    yaw_scale: float = 1.0 / 300.0
    pitch_scale: float = 1.0 / 250.0
    enable_pitch: bool = False
    # Hold pitch at zero for the first N frames so approach starts from a
    # yaw-centered, settled state instead of fighting the takeoff transient.
    pitch_delay_frames: int = 0
    log_dir: Path | None = None
    run_id: str = "game-screen-sandbox"
    tracker_type: str = TrackerType.CSRT
    frame_width: int = 640
    frame_height: int = 480
    yaw_kp: float = 0.8
    yaw_ki: float = 0.05
    yaw_kd: float = 0.15
    forward_kp: float = 0.4
    forward_ki: float = 0.02
    forward_kd: float = 0.1
    yaw_output_limit: float = 120.0
    forward_output_limit: float = 80.0
    desired_target_width: float = 120.0


@dataclass
class LoopSummary:
    frames: int = 0
    target_found: int = 0
    loss_events: int = 0
    max_abs_yaw_axis: float = 0.0
    max_abs_pitch_axis: float = 0.0
    max_abs_horizontal_error: float = 0.0
    max_abs_forward_error: float = 0.0
    initial_horizontal_error: float | None = None
    last_horizontal_error: float | None = None
    initial_target_width: float | None = None
    last_target_width: float | None = None
    min_target_width: float | None = None
    max_target_width: float | None = None
    first_target_center: tuple[float, float] | None = None
    last_target_center: tuple[float, float] | None = None
    min_target_center_x: float | None = None
    max_target_center_x: float | None = None
    min_target_center_y: float | None = None
    max_target_center_y: float | None = None
    command_samples: list[AxisCommand] = field(default_factory=list)
    horizontal_error_samples: list[float] = field(default_factory=list)
    forward_error_samples: list[float] = field(default_factory=list)

    @property
    def found_ratio(self) -> float:
        return self.target_found / self.frames if self.frames else 0.0

    @property
    def target_center_span_px(self) -> float | None:
        if (
            self.min_target_center_x is None
            or self.max_target_center_x is None
            or self.min_target_center_y is None
            or self.max_target_center_y is None
        ):
            return None
        return math.hypot(
            self.max_target_center_x - self.min_target_center_x,
            self.max_target_center_y - self.min_target_center_y,
        )

    @property
    def target_center_travel_px(self) -> float | None:
        if self.first_target_center is None or self.last_target_center is None:
            return None
        return math.hypot(
            self.last_target_center[0] - self.first_target_center[0],
            self.last_target_center[1] - self.first_target_center[1],
        )

    @property
    def horizontal_error_reduction_px(self) -> float | None:
        if self.initial_horizontal_error is None or self.last_horizontal_error is None:
            return None
        return abs(self.initial_horizontal_error) - abs(self.last_horizontal_error)

    @property
    def horizontal_error_reduction_ratio(self) -> float | None:
        if self.initial_horizontal_error is None or self.last_horizontal_error is None:
            return None
        initial_abs = abs(self.initial_horizontal_error)
        if initial_abs == 0:
            return 1.0 if abs(self.last_horizontal_error) == 0 else 0.0
        return self.horizontal_error_reduction_px / initial_abs

    @staticmethod
    def rms(values: list[float]) -> float | None:
        if not values:
            return None
        return math.sqrt(sum(value * value for value in values) / len(values))

    @staticmethod
    def percentile(values: list[float], pct: float) -> float | None:
        if not values:
            return None
        ordered = sorted(values)
        index = int(round((len(ordered) - 1) * pct))
        return ordered[max(0, min(len(ordered) - 1, index))]

    def as_dict(self) -> dict[str, Any]:
        width_trend = (
            self.last_target_width - self.initial_target_width
            if self.initial_target_width is not None and self.last_target_width is not None
            else None
        )
        return {
            "frames": self.frames,
            "target_found": self.target_found,
            "found_ratio": self.found_ratio,
            "loss_events": self.loss_events,
            "max_abs_yaw_axis": self.max_abs_yaw_axis,
            "max_abs_pitch_axis": self.max_abs_pitch_axis,
            "max_abs_horizontal_error": self.max_abs_horizontal_error,
            "max_abs_forward_error": self.max_abs_forward_error,
            "initial_horizontal_error": self.initial_horizontal_error,
            "last_horizontal_error": self.last_horizontal_error,
            "initial_abs_horizontal_error": (
                abs(self.initial_horizontal_error)
                if self.initial_horizontal_error is not None else None
            ),
            "last_abs_horizontal_error": (
                abs(self.last_horizontal_error)
                if self.last_horizontal_error is not None else None
            ),
            "horizontal_error_reduction_px": self.horizontal_error_reduction_px,
            "horizontal_error_reduction_ratio": self.horizontal_error_reduction_ratio,
            "horizontal_error_rms": self.rms(self.horizontal_error_samples),
            "horizontal_error_p95": self.percentile(self.horizontal_error_samples, 0.95),
            "forward_error_rms": self.rms(self.forward_error_samples),
            "forward_error_p95": self.percentile(self.forward_error_samples, 0.95),
            "initial_target_width": self.initial_target_width,
            "last_target_width": self.last_target_width,
            "min_target_width": self.min_target_width,
            "max_target_width": self.max_target_width,
            "target_width_trend": width_trend,
            "first_target_center": list(self.first_target_center) if self.first_target_center else None,
            "last_target_center": list(self.last_target_center) if self.last_target_center else None,
            "target_center_span_px": self.target_center_span_px,
            "target_center_travel_px": self.target_center_travel_px,
        }


def default_bbox(width: int, height: int, size: int = 100) -> tuple[int, int, int, int]:
    return (width // 2 - size // 2, height // 2 - size // 2, size, size)


def build_pipeline_config(config: LoopConfig) -> PipelineConfig:
    return PipelineConfig(
        frame_width=config.frame_width,
        frame_height=config.frame_height,
        gcs_enabled=False,
        yaw_pid=PIDGains(
            kp=config.yaw_kp,
            ki=config.yaw_ki,
            kd=config.yaw_kd,
            output_min=-config.yaw_output_limit,
            output_max=config.yaw_output_limit,
        ),
        forward_pid=PIDGains(
            kp=config.forward_kp,
            ki=config.forward_ki,
            kd=config.forward_kd,
            output_min=-config.forward_output_limit,
            output_max=config.forward_output_limit,
        ),
        desired_target_width=config.desired_target_width,
    )


def controller_command(controller: FlightController, cfg: PipelineConfig,
                       *, enable_pitch: bool, yaw_scale: float,
                       pitch_scale: float) -> AxisCommand:
    raw = command_from_channels(controller.channels, cfg)
    yaw = max(-1.0, min(1.0, controller.yaw_output * yaw_scale))
    pitch = max(-1.0, min(1.0, controller.forward_output * pitch_scale)) if enable_pitch else 0.0
    return AxisCommand(yaw=yaw, pitch=pitch, roll=0.0, throttle=0.0).clamped()


def run_loop(
    source: FrameSource,
    tracker: TrackerLike,
    input_adapter: InputAdapter,
    config: LoopConfig,
    *,
    logger: JsonlLogger | None = None,
) -> LoopSummary:
    if config.duration_s <= 0:
        raise ValueError("duration_s must be positive")
    if config.hz <= 0:
        raise ValueError("hz must be positive")

    cfg = build_pipeline_config(config)
    controller = FlightController(cfg)
    summary = LoopSummary()
    bbox = config.initial_bbox
    was_found: bool | None = None

    try:
        for item in source.frames(config.duration_s):
            frame = item.frame
            if bbox is None:
                height, width = frame.shape[:2]
                bbox = default_bbox(width, height, cfg.track_bbox_size)
                controller.set_frame_center(width, height)
            if not tracker.is_initialized:
                tracker.init(frame, bbox)
            result = tracker.update(frame)
            controller.update(result)
            command = controller_command(
                controller, cfg,
                enable_pitch=(config.enable_pitch
                              and summary.frames >= config.pitch_delay_frames),
                yaw_scale=config.yaw_scale,
                pitch_scale=config.pitch_scale,
            )
            input_adapter.send(command)

            summary.frames += 1
            if result.found:
                summary.target_found += 1
            elif was_found:
                summary.loss_events += 1
            was_found = result.found
            summary.max_abs_yaw_axis = max(summary.max_abs_yaw_axis, abs(command.yaw))
            summary.max_abs_pitch_axis = max(summary.max_abs_pitch_axis, abs(command.pitch))
            summary.max_abs_horizontal_error = max(
                summary.max_abs_horizontal_error, abs(controller.yaw_error))
            summary.max_abs_forward_error = max(
                summary.max_abs_forward_error, abs(controller.forward_error))
            summary.horizontal_error_samples.append(abs(controller.yaw_error))
            summary.forward_error_samples.append(abs(controller.forward_error))
            if result.found:
                if summary.initial_horizontal_error is None:
                    summary.initial_horizontal_error = controller.yaw_error
                summary.last_horizontal_error = controller.yaw_error
            if result.found and result.bbox:
                target_width = float(result.bbox[2])
                if summary.initial_target_width is None:
                    summary.initial_target_width = target_width
                summary.last_target_width = target_width
                summary.min_target_width = (
                    target_width if summary.min_target_width is None
                    else min(summary.min_target_width, target_width)
                )
                summary.max_target_width = (
                    target_width if summary.max_target_width is None
                    else max(summary.max_target_width, target_width)
                )
            if result.found and result.center:
                center = (float(result.center[0]), float(result.center[1]))
                if summary.first_target_center is None:
                    summary.first_target_center = center
                summary.last_target_center = center
                summary.min_target_center_x = (
                    center[0] if summary.min_target_center_x is None
                    else min(summary.min_target_center_x, center[0])
                )
                summary.max_target_center_x = (
                    center[0] if summary.max_target_center_x is None
                    else max(summary.max_target_center_x, center[0])
                )
                summary.min_target_center_y = (
                    center[1] if summary.min_target_center_y is None
                    else min(summary.min_target_center_y, center[1])
                )
                summary.max_target_center_y = (
                    center[1] if summary.max_target_center_y is None
                    else max(summary.max_target_center_y, center[1])
                )
            summary.command_samples.append(command)

            if logger:
                logger.write(
                    "game_screen_sample",
                    frame_index=item.index,
                    frame_source=item.source,
                    capture_timestamp=item.timestamp,
                    target_found=result.found,
                    target_bbox=list(result.bbox) if result.bbox else None,
                    target_center=list(result.center) if result.center else None,
                    yaw_error=controller.yaw_error,
                    forward_error=controller.forward_error,
                    yaw_output=controller.yaw_output,
                    forward_output=controller.forward_output,
                    command=command.as_dict(),
                )
    finally:
        input_adapter.neutral()
    return summary


def make_source(args: argparse.Namespace) -> FrameSource:
    if args.synthetic:
        return SyntheticFrameSource(width=args.width, height=args.height, fps=args.hz)
    if args.video:
        return VideoFileFrameSource(args.video)
    if args.region or args.window_title:
        region = resolve_capture_region(
            region=args.region,
            window_title=args.window_title,
            window_exact=args.window_exact,
            window_case_sensitive=args.window_case_sensitive,
            window_min_width=args.window_min_width,
            window_min_height=args.window_min_height,
        )
        return make_region_source(region, fps=args.hz, backend=args.capture_backend)
    raise ValueError("a frame source is required")


def parse_bbox(value: str) -> tuple[int, int, int, int]:
    parts = [int(part.strip()) for part in value.split(",")]
    if len(parts) != 4:
        raise argparse.ArgumentTypeError("bbox must be x,y,w,h")
    x, y, w, h = parts
    if w <= 0 or h <= 0:
        raise argparse.ArgumentTypeError("bbox width/height must be positive")
    return (x, y, w, h)


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
    parser.add_argument("--duration", type=float, default=5.0)
    parser.add_argument("--hz", type=float, default=15.0)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--bbox", type=parse_bbox, default=None)
    parser.add_argument("--tracker", choices=[TrackerType.CSRT, TrackerType.KCF],
                        default=TrackerType.CSRT)
    parser.add_argument("--enable-pitch", action="store_true")
    parser.add_argument("--pitch-delay-frames", type=int, default=0)
    parser.add_argument("--dry-run", action="store_true", default=True)
    parser.add_argument("--uinput", action="store_true")
    parser.add_argument("--ack-live-input", action="store_true")
    parser.add_argument("--live-max-duration", type=float, default=2.0)
    parser.add_argument("--log-dir", default=None)
    parser.add_argument("--run-id", default="game-screen-sandbox")
    parser.add_argument("--yaw-kp", type=float, default=0.8)
    parser.add_argument("--yaw-ki", type=float, default=0.05)
    parser.add_argument("--yaw-kd", type=float, default=0.15)
    parser.add_argument("--forward-kp", type=float, default=0.4)
    parser.add_argument("--forward-ki", type=float, default=0.02)
    parser.add_argument("--forward-kd", type=float, default=0.1)
    parser.add_argument("--desired-target-width", type=float, default=120.0)
    args = parser.parse_args(argv)
    if args.duration <= 0:
        parser.error("--duration must be positive")
    if args.hz <= 0:
        parser.error("--hz must be positive")
    if args.width <= 0 or args.height <= 0:
        parser.error("--width/--height must be positive")
    if args.window_min_width <= 0 or args.window_min_height <= 0:
        parser.error("--window-min-width/--window-min-height must be positive")
    if args.desired_target_width <= 0:
        parser.error("--desired-target-width must be positive")
    if args.pitch_delay_frames < 0:
        parser.error("--pitch-delay-frames must be non-negative")
    if args.live_max_duration <= 0:
        parser.error("--live-max-duration must be positive")
    if args.uinput and not args.ack_live_input:
        parser.error("--uinput requires --ack-live-input")
    if args.uinput and args.duration > args.live_max_duration:
        parser.error("--duration must be <= --live-max-duration when --uinput is used")
    return args


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    source = make_source(args)
    tracker = ObjectTracker(args.tracker)
    adapter = UInputAdapter() if args.uinput else DryRunInputAdapter()
    log_dir = resolve_log_dir(args.log_dir, repo_root=REPO_ROOT).parent / "game_screen_sandbox"
    log_path = make_log_path(log_dir, args.run_id)
    config = LoopConfig(
        duration_s=args.duration,
        hz=args.hz,
        initial_bbox=args.bbox,
        enable_pitch=args.enable_pitch,
        pitch_delay_frames=args.pitch_delay_frames,
        log_dir=log_dir,
        run_id=args.run_id,
        tracker_type=args.tracker,
        frame_width=args.width,
        frame_height=args.height,
        yaw_kp=args.yaw_kp,
        yaw_ki=args.yaw_ki,
        yaw_kd=args.yaw_kd,
        forward_kp=args.forward_kp,
        forward_ki=args.forward_ki,
        forward_kd=args.forward_kd,
        desired_target_width=args.desired_target_width,
    )
    with JsonlLogger(log_path, metadata={
        "tool": "game_screen_sandbox",
        "run_id": args.run_id,
        "source": (
            "synthetic" if args.synthetic
            else args.video or (
                "window:%s:%s" % (args.window_title, args.capture_backend)
                if args.window_title else "region:%s" % args.capture_backend
            )
        ),
        "adapter": adapter.__class__.__name__,
    }) as logger:
        summary = run_loop(source, tracker, adapter, config, logger=logger)
        logger.write("game_screen_summary", **summary.as_dict())
    adapter.close()
    verdict = "PASS" if summary.frames > 0 and summary.found_ratio > 0 else "FAIL"
    print("game-screen %s frames=%d found_ratio=%.2f yaw_axis=%.3f pitch_axis=%.3f log=%s" % (
        verdict,
        summary.frames,
        summary.found_ratio,
        summary.max_abs_yaw_axis,
        summary.max_abs_pitch_axis,
        log_path,
    ))
    return 0 if verdict == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
