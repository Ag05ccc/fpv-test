#!/usr/bin/env python3
"""Closed-loop simple game driven through an input adapter."""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Iterator

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))
TOOLS_DIR = REPO_ROOT / "tools"
if str(TOOLS_DIR) not in sys.path:
    sys.path.insert(0, str(TOOLS_DIR))

from capture_window import CaptureFrame  # noqa: E402
from screen_tracking_loop import LoopConfig, run_loop  # noqa: E402
from simple_target_game import (  # noqa: E402
    GameCommand,
    GameConfig,
    GameState,
    render_frame,
    step_state,
    target_screen_bbox,
    target_world_center,
)
from sitl_log import JsonlLogger, make_log_path, resolve_log_dir  # noqa: E402
from virtual_input import AxisCommand  # noqa: E402
from kenet.tracker import ObjectTracker, TrackerType  # noqa: E402


PASS = "PASS"
FAIL = "FAIL"
DEFAULT_LOG_DIR = Path("logs/game_screen_sandbox")


@dataclass
class AdapterLoopConfig:
    duration_s: float = 5.0
    hz: float = 20.0
    width: int = 640
    height: int = 480
    target_size: int = 90
    initial_offset_x: float = 180.0
    target_motion_radius_x: float = 0.0
    target_motion_radius_y: float = 0.0
    target_period_s: float = 8.0
    camera_speed_px_s: float = 260.0
    yaw_kp: float = 0.9
    yaw_ki: float = 0.0
    yaw_kd: float = 0.0
    yaw_output_limit: float = 180.0
    yaw_scale: float = 1.0 / 260.0
    min_found_ratio: float = 0.9
    min_initial_abs_error_px: float = 80.0
    max_final_abs_error_px: float = 35.0
    min_error_reduction_ratio: float = 0.65
    max_yaw_axis: float = 0.8
    run_id: str = "simple-game-adapter-loop"
    log_dir: Path = DEFAULT_LOG_DIR
    tracker_type: str = TrackerType.KCF


@dataclass
class AdapterLoopSummary:
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


class SimpleGameInputAdapter:
    """InputAdapter implementation that stores commands for the next frame."""

    def __init__(self):
        self.commands: list[tuple[float, AxisCommand]] = []
        self.last_command = AxisCommand()
        self.closed = False

    def send(self, command: AxisCommand) -> None:
        if self.closed:
            raise RuntimeError("adapter is closed")
        self.last_command = command.clamped()
        self.commands.append((time.monotonic(), self.last_command))

    def neutral(self) -> None:
        self.send(AxisCommand())

    def close(self) -> None:
        if not self.closed:
            self.neutral()
            self.closed = True


def build_game_config(config: AdapterLoopConfig) -> GameConfig:
    return GameConfig(
        width=config.width,
        height=config.height,
        target_size=config.target_size,
        target_motion_radius_x=config.target_motion_radius_x,
        target_motion_radius_y=config.target_motion_radius_y,
        target_period_s=config.target_period_s,
        camera_speed_px_s=config.camera_speed_px_s,
    )


def initial_offset_state(game: GameConfig, offset_x: float) -> GameState:
    target_x, target_y = target_world_center(0.0, game)
    half_w = game.width / 2.0
    half_h = game.height / 2.0
    camera_x = max(half_w, min(game.world_width - half_w, target_x - offset_x))
    camera_y = max(half_h, min(game.world_height - half_h, target_y))
    return GameState(
        t=0.0,
        camera_x=camera_x,
        camera_y=camera_y,
        command=GameCommand(),
    )


class SimpleGameAdapterFrameSource:
    """Frame source whose next frame is driven by the adapter command."""

    def __init__(
        self,
        game: GameConfig,
        adapter: SimpleGameInputAdapter,
        *,
        initial_offset_x: float,
        fps: float,
    ):
        if fps <= 0:
            raise ValueError("fps must be positive")
        self.game = game
        self.adapter = adapter
        self.fps = fps
        self.dt = 1.0 / fps
        self.state = initial_offset_state(game, initial_offset_x)
        self.initial_camera_x = self.state.camera_x
        self.initial_camera_y = self.state.camera_y
        self.applied_commands: list[AxisCommand] = []
        self.camera_x_samples: list[float] = [self.state.camera_x]
        self.camera_y_samples: list[float] = [self.state.camera_y]

    @property
    def initial_bbox(self) -> tuple[int, int, int, int]:
        return target_screen_bbox(self.state, self.game)

    def frames(self, duration_s: float | None = None) -> Iterator[CaptureFrame]:
        frame_count = int((duration_s if duration_s is not None else 1.0) * self.fps)
        start = time.monotonic()
        for index in range(max(0, frame_count)):
            frame = render_frame(self.state, self.game)
            yield CaptureFrame(
                frame=frame,
                timestamp=start + index * self.dt,
                index=index,
                source="simple-game-adapter",
            )
            command = self.adapter.last_command
            self.applied_commands.append(command)
            self.state = step_state(
                self.state,
                GameCommand(yaw=command.yaw, pitch=command.pitch),
                self.dt,
                self.game,
            )
            self.camera_x_samples.append(self.state.camera_x)
            self.camera_y_samples.append(self.state.camera_y)


def evaluate_metrics(
    metrics: dict[str, Any],
    config: AdapterLoopConfig,
) -> tuple[str, str, list[str]]:
    notes: list[str] = []
    if metrics["frames"] <= 0:
        return FAIL, "adapter-driven game loop produced no frames", ["NO_FRAMES"]
    if metrics["found_ratio"] < config.min_found_ratio:
        notes.append("FOUND_RATIO_LOW")
    if metrics["initial_abs_error_px"] < config.min_initial_abs_error_px:
        notes.append("INITIAL_ERROR_TOO_SMALL")
    if metrics["final_abs_error_px"] > config.max_final_abs_error_px:
        notes.append("FINAL_ERROR_TOO_HIGH")
    if metrics["error_reduction_ratio"] < config.min_error_reduction_ratio:
        notes.append("ERROR_REDUCTION_LOW")
    if metrics["max_abs_yaw_axis"] > config.max_yaw_axis:
        notes.append("YAW_AXIS_LIMIT_EXCEEDED")
    if metrics["nonneutral_adapter_commands"] <= 0:
        notes.append("NO_NONNEUTRAL_ADAPTER_COMMANDS")
    if metrics["applied_nonneutral_commands"] <= 0:
        notes.append("NO_APPLIED_GAME_COMMANDS")
    if not metrics["neutralized"]:
        notes.append("INPUT_NOT_NEUTRALIZED")
    if metrics["camera_x_delta_px"] <= 0:
        notes.append("CAMERA_DID_NOT_MOVE_TOWARD_RIGHT_TARGET")
    status = PASS if not notes else FAIL
    summary = (
        "adapter-driven simple game loop centers the target"
        if status == PASS else
        "adapter-driven simple game loop did not meet gates"
    )
    return status, summary, notes


def run_adapter_closed_loop(
    config: AdapterLoopConfig,
    *,
    logger: JsonlLogger | None = None,
) -> AdapterLoopSummary:
    if config.duration_s <= 0:
        raise ValueError("duration_s must be positive")
    if config.hz <= 0:
        raise ValueError("hz must be positive")
    if config.width <= 0 or config.height <= 0:
        raise ValueError("width/height must be positive")

    game = build_game_config(config)
    adapter = SimpleGameInputAdapter()
    source = SimpleGameAdapterFrameSource(
        game,
        adapter,
        initial_offset_x=config.initial_offset_x,
        fps=config.hz,
    )
    tracker = ObjectTracker(config.tracker_type)
    loop_config = LoopConfig(
        duration_s=config.duration_s,
        hz=config.hz,
        initial_bbox=source.initial_bbox,
        yaw_scale=config.yaw_scale,
        pitch_scale=0.0,
        enable_pitch=False,
        run_id=config.run_id,
        tracker_type=config.tracker_type,
        frame_width=config.width,
        frame_height=config.height,
        yaw_kp=config.yaw_kp,
        yaw_ki=config.yaw_ki,
        yaw_kd=config.yaw_kd,
        yaw_output_limit=config.yaw_output_limit,
    )
    summary = run_loop(source, tracker, adapter, loop_config, logger=logger)
    neutralized = adapter.commands[-1][1].is_neutral() if adapter.commands else False
    nonneutral_adapter_commands = sum(
        1 for _timestamp, command in adapter.commands if not command.is_neutral()
    )
    applied_nonneutral_commands = sum(
        1 for command in source.applied_commands if not command.is_neutral()
    )
    adapter.close()

    first_center = summary.first_target_center
    last_center = summary.last_target_center
    initial_abs_error = (
        abs(first_center[0] - config.width / 2.0)
        if first_center is not None else float("inf")
    )
    final_abs_error = (
        abs(last_center[0] - config.width / 2.0)
        if last_center is not None else float("inf")
    )
    reduction = (
        (initial_abs_error - final_abs_error) / initial_abs_error
        if initial_abs_error > 0 and math.isfinite(initial_abs_error)
        else 0.0
    )
    metrics = {
        **summary.as_dict(),
        "initial_offset_x": config.initial_offset_x,
        "initial_abs_error_px": initial_abs_error,
        "final_abs_error_px": final_abs_error,
        "error_reduction_ratio": reduction,
        "adapter_commands": len(adapter.commands),
        "nonneutral_adapter_commands": nonneutral_adapter_commands,
        "applied_game_commands": len(source.applied_commands),
        "applied_nonneutral_commands": applied_nonneutral_commands,
        "neutralized": neutralized,
        "initial_camera_x": source.initial_camera_x,
        "final_camera_x": source.state.camera_x,
        "camera_x_delta_px": source.state.camera_x - source.initial_camera_x,
        "initial_camera_y": source.initial_camera_y,
        "final_camera_y": source.state.camera_y,
        "duration_s": config.duration_s,
        "hz": config.hz,
        "tracker_type": config.tracker_type,
        "adapter": adapter.__class__.__name__,
        "frame_source": source.__class__.__name__,
        "real_input": False,
        "control_applied_through_input_adapter": True,
        "source_reads_adapter_command": True,
        "max_final_abs_error_px": config.max_final_abs_error_px,
        "min_error_reduction_ratio": config.min_error_reduction_ratio,
        "max_yaw_axis": config.max_yaw_axis,
    }
    status, summary_text, notes = evaluate_metrics(metrics, config)
    return AdapterLoopSummary(status=status, summary=summary_text, metrics=metrics, notes=notes)


def build_markdown(result: AdapterLoopSummary, *, run_id: str) -> str:
    lines = [
        "# Simple Game Adapter Closed Loop",
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
        "frames",
        "found_ratio",
        "initial_abs_error_px",
        "final_abs_error_px",
        "error_reduction_ratio",
        "max_abs_yaw_axis",
        "adapter_commands",
        "nonneutral_adapter_commands",
        "applied_nonneutral_commands",
        "camera_x_delta_px",
        "neutralized",
        "control_applied_through_input_adapter",
    ):
        lines.append("| %s | `%s` |" % (key, result.metrics.get(key)))
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


def write_reports(
    result: AdapterLoopSummary,
    log_dir: Path,
    *,
    run_id: str,
) -> tuple[Path, Path]:
    log_dir.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime("%Y%m%d-%H%M%S", time.localtime())
    json_path = log_dir / ("%s-%s-simple-game-adapter.json" % (stamp, run_id))
    md_path = log_dir / ("%s-%s-simple-game-adapter.md" % (stamp, run_id))
    json_path.write_text(json.dumps(result.as_dict(), indent=2, sort_keys=True) + "\n",
                         encoding="utf-8")
    md_path.write_text(build_markdown(result, run_id=run_id), encoding="utf-8")
    return json_path, md_path


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-id", default="simple-game-adapter-loop")
    parser.add_argument("--log-dir", type=Path, default=DEFAULT_LOG_DIR)
    parser.add_argument("--duration", type=float, default=5.0)
    parser.add_argument("--hz", type=float, default=20.0)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--initial-offset-x", type=float, default=180.0)
    parser.add_argument("--max-final-error", type=float, default=35.0)
    parser.add_argument("--min-error-reduction", type=float, default=0.65)
    parser.add_argument("--min-found-ratio", type=float, default=0.9)
    parser.add_argument("--max-yaw-axis", type=float, default=0.8)
    args = parser.parse_args(argv)
    if args.duration <= 0:
        parser.error("--duration must be positive")
    if args.hz <= 0:
        parser.error("--hz must be positive")
    if args.width <= 0 or args.height <= 0:
        parser.error("--width/--height must be positive")
    if args.max_final_error < 0:
        parser.error("--max-final-error must be non-negative")
    if not 0 <= args.min_error_reduction <= 1:
        parser.error("--min-error-reduction must be in [0, 1]")
    if not 0 <= args.min_found_ratio <= 1:
        parser.error("--min-found-ratio must be in [0, 1]")
    if not 0 < args.max_yaw_axis <= 1:
        parser.error("--max-yaw-axis must be in (0, 1]")
    return args


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    log_dir = resolve_log_dir(str(args.log_dir), repo_root=REPO_ROOT)
    log_path = make_log_path(log_dir, "%s-simple-game-adapter-loop" % args.run_id)
    config = AdapterLoopConfig(
        duration_s=args.duration,
        hz=args.hz,
        width=args.width,
        height=args.height,
        initial_offset_x=args.initial_offset_x,
        max_final_abs_error_px=args.max_final_error,
        min_error_reduction_ratio=args.min_error_reduction,
        min_found_ratio=args.min_found_ratio,
        max_yaw_axis=args.max_yaw_axis,
        run_id=args.run_id,
        log_dir=log_dir,
    )
    with JsonlLogger(log_path, metadata={
        "tool": "simple_game_adapter_loop",
        "run_id": args.run_id,
        "source": "in-process-simple-target-game",
        "adapter": "SimpleGameInputAdapter",
        "real_input": False,
        "control_applied_through_input_adapter": True,
    }) as logger:
        result = run_adapter_closed_loop(config, logger=logger)
        logger.write("simple_game_adapter_summary", **result.metrics)
    result.metrics["log_path"] = str(log_path)
    json_path, md_path = write_reports(result, log_dir, run_id=args.run_id)
    print("simple-game-adapter-loop %s report=%s summary=%s" % (
        result.status,
        json_path,
        md_path,
    ))
    print(result.summary)
    return 0 if result.status == PASS else 1


if __name__ == "__main__":
    raise SystemExit(main())
