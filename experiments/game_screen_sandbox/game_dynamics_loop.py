#!/usr/bin/env python3
"""In-process game dynamics closed-loop check for the sandbox."""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable, Protocol

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))
TOOLS_DIR = REPO_ROOT / "tools"
if str(TOOLS_DIR) not in sys.path:
    sys.path.insert(0, str(TOOLS_DIR))

from kenet.controller import FlightController, PIDGains  # noqa: E402
from kenet.pipeline import PipelineConfig  # noqa: E402
from kenet.tracker import ObjectTracker, TrackResult, TrackerType  # noqa: E402
from screen_tracking_loop import controller_command  # noqa: E402
from simple_target_game import (  # noqa: E402
    GameCommand,
    GameConfig,
    GameState,
    initial_state,
    render_frame,
    step_state,
    target_screen_bbox,
    target_world_center,
)
from sitl_log import JsonlLogger, make_log_path, resolve_log_dir  # noqa: E402


PASS = "PASS"
FAIL = "FAIL"
WAITING = "WAITING"
DEFAULT_LOG_DIR = Path("logs/game_screen_sandbox")


class TrackerLike(Protocol):
    @property
    def is_initialized(self) -> bool:
        ...

    def init(self, frame, bbox) -> None:
        ...

    def update(self, frame) -> TrackResult:
        ...


@dataclass
class GameDynamicsConfig:
    duration_s: float = 4.0
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
    max_yaw_axis: float = 0.8
    min_found_ratio: float = 0.9
    max_loss_events: int = 0
    min_initial_abs_error_px: float = 80.0
    max_final_abs_error_px: float = 35.0
    min_error_reduction_ratio: float = 0.65
    min_yaw_corrective_ratio: float = 0.0
    run_id: str = "game-dynamics-loop"
    log_dir: Path = DEFAULT_LOG_DIR
    tracker_type: str = TrackerType.KCF


@dataclass
class GameRangeDynamicsConfig(GameDynamicsConfig):
    initial_target_width: float = 70.0
    desired_target_width: float = 120.0
    min_target_width: float = 35.0
    max_target_width: float = 180.0
    target_width_rate_px_s: float = 220.0
    forward_kp: float = 1.2
    forward_ki: float = 0.0
    forward_kd: float = 0.0
    forward_output_limit: float = 220.0
    pitch_scale: float = 1.0 / 220.0
    max_pitch_axis: float = 0.8
    max_final_abs_width_error_px: float = 8.0
    min_width_error_reduction_ratio: float = 0.65
    min_pitch_corrective_ratio: float = 0.0
    size_deadband: float = 4.0


@dataclass
class GameHandoffDynamicsConfig(GameRangeDynamicsConfig):
    manual_duration_s: float = 1.6
    manual_yaw_scale: float = 1.0 / 220.0
    max_manual_yaw_axis: float = 0.7
    max_handoff_abs_error_px: float = 45.0
    max_final_abs_width_error_px: float = 8.0
    min_manual_to_final_error_reduction_ratio: float = 0.80


@dataclass
class GameDynamicsSummary:
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


class ColorTargetTracker:
    """Simple image-space target detector for the synthetic colored target."""

    def __init__(self):
        self.initialized = False

    @property
    def is_initialized(self) -> bool:
        return self.initialized

    def init(self, _frame, _bbox) -> None:
        self.initialized = True

    def update(self, frame) -> TrackResult:
        blue = frame[:, :, 0]
        green = frame[:, :, 1]
        red = frame[:, :, 2]
        mask = (green > 170) & (red > 170) & (blue < 130)
        ys, xs = np.where(mask)
        if xs.size == 0 or ys.size == 0:
            return TrackResult(found=False)
        x1 = int(xs.min())
        x2 = int(xs.max())
        y1 = int(ys.min())
        y2 = int(ys.max())
        bbox = (x1, y1, x2 - x1 + 1, y2 - y1 + 1)
        center = (x1 + bbox[2] / 2.0, y1 + bbox[3] / 2.0)
        return TrackResult(found=True, bbox=bbox, center=center)


def abs_rms(values: list[float]) -> float | None:
    if not values:
        return None
    return math.sqrt(sum(value * value for value in values) / len(values))


def percentile_abs(values: list[float], pct: float) -> float | None:
    if not values:
        return None
    ordered = sorted(abs(value) for value in values)
    index = int(round((len(ordered) - 1) * pct))
    return ordered[max(0, min(len(ordered) - 1, index))]


def corrective_command_metrics(
    errors: list[float],
    commands: list[float],
    *,
    error_deadband: float = 2.0,
    command_deadband: float = 1e-6,
) -> dict[str, Any]:
    samples = 0
    corrective = 0
    wrong = 0
    neutral = 0
    for error, command in zip(errors, commands):
        if not math.isfinite(error) or not math.isfinite(command):
            continue
        if abs(error) <= error_deadband:
            continue
        samples += 1
        if abs(command) <= command_deadband:
            neutral += 1
        elif error * command > 0.0:
            corrective += 1
        else:
            wrong += 1
    ratio = corrective / samples if samples else 1.0
    first_error: float | None = None
    first_command: float | None = None
    first_corrective: bool | None = None
    for error, command in zip(errors, commands):
        if not math.isfinite(error) or not math.isfinite(command):
            continue
        if abs(error) <= error_deadband or abs(command) <= command_deadband:
            continue
        first_error = error
        first_command = command
        first_corrective = error * command > 0.0
        break
    return {
        "samples": samples,
        "corrective": corrective,
        "wrong": wrong,
        "neutral": neutral,
        "ratio": ratio,
        "first_error": first_error,
        "first_command": first_command,
        "first_corrective": first_corrective,
        "error_deadband_px": error_deadband,
    }


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def build_game_config(config: GameDynamicsConfig) -> GameConfig:
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
    base = initial_state(game)
    target_x, target_y = target_world_center(base.t, game)
    half_w = game.width / 2.0
    half_h = game.height / 2.0
    camera_x = max(half_w, min(game.world_width - half_w, target_x - offset_x))
    camera_y = max(half_h, min(game.world_height - half_h, target_y))
    return GameState(
        t=base.t,
        camera_x=camera_x,
        camera_y=camera_y,
        command=GameCommand(),
    )


def build_pipeline_config(config: GameDynamicsConfig) -> PipelineConfig:
    forward_pid = getattr(config, "forward_kp", 0.4)
    forward_ki = getattr(config, "forward_ki", 0.0)
    forward_kd = getattr(config, "forward_kd", 0.0)
    forward_limit = getattr(config, "forward_output_limit", 80.0)
    desired_width = getattr(config, "desired_target_width", 120.0)
    return PipelineConfig(
        frame_width=config.width,
        frame_height=config.height,
        gcs_enabled=False,
        yaw_pid=PIDGains(
            kp=config.yaw_kp,
            ki=config.yaw_ki,
            kd=config.yaw_kd,
            output_min=-config.yaw_output_limit,
            output_max=config.yaw_output_limit,
        ),
        forward_pid=PIDGains(
            kp=forward_pid,
            ki=forward_ki,
            kd=forward_kd,
            output_min=-forward_limit,
            output_max=forward_limit,
        ),
        desired_target_width=desired_width,
        size_deadband=getattr(config, "size_deadband", 15.0),
    )


def evaluate_metrics(metrics: dict[str, Any], config: GameDynamicsConfig) -> tuple[str, str, list[str]]:
    notes: list[str] = []
    if metrics["frames"] <= 0:
        return FAIL, "game dynamics loop produced no frames", ["NO_FRAMES"]
    if metrics["found_ratio"] < config.min_found_ratio:
        notes.append("FOUND_RATIO_LOW")
    if metrics["loss_events"] > config.max_loss_events:
        notes.append("TRACKER_LOSS_EVENTS")
    if metrics["initial_abs_error_px"] < config.min_initial_abs_error_px:
        notes.append("INITIAL_ERROR_TOO_SMALL")
    if metrics["final_abs_error_px"] > config.max_final_abs_error_px:
        notes.append("FINAL_ERROR_TOO_HIGH")
    if metrics["error_reduction_ratio"] < config.min_error_reduction_ratio:
        notes.append("ERROR_REDUCTION_LOW")
    if metrics["max_abs_yaw_axis"] > config.max_yaw_axis:
        notes.append("YAW_AXIS_LIMIT_EXCEEDED")
    if metrics["yaw_initial_command_corrective"] is False:
        notes.append("YAW_INITIAL_COMMAND_DIRECTION_WRONG")
    if metrics["yaw_corrective_command_ratio"] < config.min_yaw_corrective_ratio:
        notes.append("YAW_COMMAND_DIRECTION_RATIO_LOW")
    status = PASS if not notes else FAIL
    summary = (
        "in-process game camera closed loop centers the target"
        if status == PASS else
        "in-process game camera closed loop did not meet centering gates"
    )
    return status, summary, notes


def render_scaled_target_frame(
    state: GameState,
    base_config: GameConfig,
    *,
    target_width: float,
):
    dynamic_config = GameConfig(
        width=base_config.width,
        height=base_config.height,
        world_width=base_config.world_width,
        world_height=base_config.world_height,
        target_size=max(1, int(round(target_width))),
        target_motion_radius_x=base_config.target_motion_radius_x,
        target_motion_radius_y=base_config.target_motion_radius_y,
        target_period_s=base_config.target_period_s,
        camera_speed_px_s=base_config.camera_speed_px_s,
        grid_px=base_config.grid_px,
    )
    return render_frame(state, dynamic_config), target_screen_bbox(state, dynamic_config)


def run_game_dynamics_loop(
    config: GameDynamicsConfig,
    *,
    tracker_factory: Callable[[], TrackerLike] | None = None,
    logger: JsonlLogger | None = None,
) -> GameDynamicsSummary:
    if config.duration_s <= 0:
        raise ValueError("duration_s must be positive")
    if config.hz <= 0:
        raise ValueError("hz must be positive")
    if config.width <= 0 or config.height <= 0:
        raise ValueError("width/height must be positive")

    game = build_game_config(config)
    state = initial_offset_state(game, config.initial_offset_x)
    tracker = tracker_factory() if tracker_factory else ObjectTracker(config.tracker_type)
    controller = FlightController(build_pipeline_config(config))
    controller.set_frame_center(config.width, config.height)
    dt = 1.0 / config.hz
    frame_count = int(config.duration_s * config.hz)
    errors: list[float] = []
    commands: list[float] = []
    found_commands: list[float] = []
    found_count = 0
    loss_events = 0
    was_found: bool | None = None
    first_bbox: tuple[int, int, int, int] | None = None
    last_bbox: tuple[int, int, int, int] | None = None
    first_center: tuple[float, float] | None = None
    last_center: tuple[float, float] | None = None

    for index in range(frame_count):
        frame = render_frame(state, game)
        truth_bbox = target_screen_bbox(state, game)
        if not tracker.is_initialized:
            tracker.init(frame, truth_bbox)
        result = tracker.update(frame)
        controller.update(result)
        command = controller_command(
            controller,
            controller.cfg,
            enable_pitch=False,
            yaw_scale=config.yaw_scale,
            pitch_scale=0.0,
        )
        state = step_state(state, GameCommand(yaw=command.yaw), dt, game)

        if result.found:
            found_count += 1
            center = (float(result.center[0]), float(result.center[1]))
            error = center[0] - config.width / 2.0
            errors.append(error)
            found_commands.append(command.yaw)
            if first_bbox is None and result.bbox:
                first_bbox = tuple(int(value) for value in result.bbox)
                first_center = center
            if result.bbox:
                last_bbox = tuple(int(value) for value in result.bbox)
            last_center = center
        elif was_found:
            loss_events += 1
        was_found = result.found
        commands.append(command.yaw)

        if logger:
            logger.write(
                "game_dynamics_sample",
                frame_index=index,
                sim_time=state.t,
                target_found=result.found,
                target_bbox=list(result.bbox) if result.bbox else None,
                target_center=list(result.center) if result.center else None,
                truth_bbox=list(truth_bbox),
                yaw_error=controller.yaw_error,
                yaw_output=controller.yaw_output,
                command=command.as_dict(),
                camera_x=state.camera_x,
                camera_y=state.camera_y,
            )

    initial_abs_error = abs(errors[0]) if errors else float("inf")
    final_abs_error = abs(errors[-1]) if errors else float("inf")
    reduction = (
        (initial_abs_error - final_abs_error) / initial_abs_error
        if initial_abs_error > 0 and math.isfinite(initial_abs_error)
        else 0.0
    )
    yaw_direction = corrective_command_metrics(errors, found_commands)
    metrics = {
        "frames": frame_count,
        "target_found": found_count,
        "found_ratio": found_count / frame_count if frame_count else 0.0,
        "loss_events": loss_events,
        "initial_offset_x": config.initial_offset_x,
        "initial_abs_error_px": initial_abs_error,
        "final_abs_error_px": final_abs_error,
        "error_reduction_ratio": reduction,
        "horizontal_error_rms": abs_rms(errors),
        "horizontal_error_p95": percentile_abs(errors, 0.95),
        "yaw_corrective_command_samples": yaw_direction["samples"],
        "yaw_corrective_command_count": yaw_direction["corrective"],
        "yaw_wrong_direction_command_count": yaw_direction["wrong"],
        "yaw_neutral_command_count": yaw_direction["neutral"],
        "yaw_corrective_command_ratio": yaw_direction["ratio"],
        "yaw_initial_error_px": yaw_direction["first_error"],
        "yaw_initial_command": yaw_direction["first_command"],
        "yaw_initial_command_corrective": yaw_direction["first_corrective"],
        "yaw_direction_error_deadband_px": yaw_direction["error_deadband_px"],
        "max_abs_yaw_axis": max((abs(value) for value in commands), default=0.0),
        "first_target_center": list(first_center) if first_center else None,
        "last_target_center": list(last_center) if last_center else None,
        "first_target_bbox": list(first_bbox) if first_bbox else None,
        "last_target_bbox": list(last_bbox) if last_bbox else None,
        "final_camera_x": state.camera_x,
        "final_camera_y": state.camera_y,
        "duration_s": config.duration_s,
        "hz": config.hz,
        "target_motion_radius_x": config.target_motion_radius_x,
        "target_motion_radius_y": config.target_motion_radius_y,
        "camera_speed_px_s": config.camera_speed_px_s,
        "tracker_type": config.tracker_type,
        "real_input": False,
        "control_applied_to_game_camera": True,
        "min_found_ratio": config.min_found_ratio,
        "max_loss_events": config.max_loss_events,
        "max_final_abs_error_px": config.max_final_abs_error_px,
        "min_error_reduction_ratio": config.min_error_reduction_ratio,
        "min_yaw_corrective_ratio": config.min_yaw_corrective_ratio,
        "max_yaw_axis": config.max_yaw_axis,
    }
    status, summary, notes = evaluate_metrics(metrics, config)
    return GameDynamicsSummary(status=status, summary=summary, metrics=metrics, notes=notes)


def evaluate_range_metrics(
    metrics: dict[str, Any],
    config: GameRangeDynamicsConfig,
) -> tuple[str, str, list[str]]:
    status, _summary, notes = evaluate_metrics(metrics, config)
    if metrics["initial_abs_width_error_px"] <= 0:
        notes.append("INITIAL_WIDTH_ERROR_TOO_SMALL")
    if metrics["final_abs_width_error_px"] > config.max_final_abs_width_error_px:
        notes.append("FINAL_WIDTH_ERROR_TOO_HIGH")
    if metrics["width_error_reduction_ratio"] < config.min_width_error_reduction_ratio:
        notes.append("WIDTH_ERROR_REDUCTION_LOW")
    if metrics["max_abs_pitch_axis"] > config.max_pitch_axis:
        notes.append("PITCH_AXIS_LIMIT_EXCEEDED")
    if metrics["pitch_initial_command_corrective"] is False:
        notes.append("PITCH_INITIAL_COMMAND_DIRECTION_WRONG")
    if metrics["pitch_corrective_command_ratio"] < config.min_pitch_corrective_ratio:
        notes.append("PITCH_COMMAND_DIRECTION_RATIO_LOW")
    status = PASS if not notes else FAIL
    summary = (
        "in-process game range loop centers and approaches the target"
        if status == PASS else
        "in-process game range loop did not meet center/approach gates"
    )
    return status, summary, notes


def evaluate_handoff_metrics(
    metrics: dict[str, Any],
    config: GameHandoffDynamicsConfig,
) -> tuple[str, str, list[str]]:
    notes: list[str] = []
    if metrics["frames"] <= 0:
        return FAIL, "manual-to-autonomous handoff produced no frames", ["NO_FRAMES"]
    if metrics["found_ratio"] < config.min_found_ratio:
        notes.append("FOUND_RATIO_LOW")
    if metrics["loss_events"] > config.max_loss_events:
        notes.append("TRACKER_LOSS_EVENTS")
    if metrics["manual_initial_abs_error_px"] < config.min_initial_abs_error_px:
        notes.append("MANUAL_INITIAL_ERROR_TOO_SMALL")
    if metrics["manual_to_final_error_reduction_ratio"] < config.min_manual_to_final_error_reduction_ratio:
        notes.append("MANUAL_TO_FINAL_ERROR_REDUCTION_LOW")
    if metrics["final_abs_error_px"] > config.max_final_abs_error_px:
        notes.append("FINAL_ERROR_TOO_HIGH")
    if metrics["initial_abs_width_error_px"] <= 0:
        notes.append("INITIAL_WIDTH_ERROR_TOO_SMALL")
    if metrics["final_abs_width_error_px"] > config.max_final_abs_width_error_px:
        notes.append("FINAL_WIDTH_ERROR_TOO_HIGH")
    if metrics["width_error_reduction_ratio"] < config.min_width_error_reduction_ratio:
        notes.append("WIDTH_ERROR_REDUCTION_LOW")
    if metrics["max_abs_yaw_axis"] > config.max_yaw_axis:
        notes.append("YAW_AXIS_LIMIT_EXCEEDED")
    if metrics["max_abs_pitch_axis"] > config.max_pitch_axis:
        notes.append("PITCH_AXIS_LIMIT_EXCEEDED")
    if metrics["yaw_initial_command_corrective"] is False:
        notes.append("YAW_INITIAL_COMMAND_DIRECTION_WRONG")
    if metrics["pitch_initial_command_corrective"] is False:
        notes.append("PITCH_INITIAL_COMMAND_DIRECTION_WRONG")
    if metrics["yaw_corrective_command_ratio"] < config.min_yaw_corrective_ratio:
        notes.append("YAW_COMMAND_DIRECTION_RATIO_LOW")
    if metrics["pitch_corrective_command_ratio"] < config.min_pitch_corrective_ratio:
        notes.append("PITCH_COMMAND_DIRECTION_RATIO_LOW")
    if metrics["manual_frames"] <= 0 or metrics["auto_frames"] <= 0:
        notes.append("HANDOFF_PHASE_LENGTH_INVALID")
    if metrics["tracker_initialized_before_follow"]:
        notes.append("TRACKER_INITIALIZED_BEFORE_FOLLOW")
    if metrics["pre_handoff_auto_control_samples"] != 0:
        notes.append("AUTO_CONTROL_BEFORE_FOLLOW")
    if metrics["manual_final_abs_error_px"] > config.max_handoff_abs_error_px:
        notes.append("MANUAL_HANDOFF_ERROR_TOO_HIGH")
    if not metrics["follow_command_sent"]:
        notes.append("FOLLOW_COMMAND_NOT_SENT")
    if not metrics["tracker_initialized_after_follow"]:
        notes.append("TRACKER_NOT_INITIALIZED_AFTER_FOLLOW")
    status = PASS if not notes else FAIL
    summary = (
        "manual-to-autonomous handoff centers and approaches the target"
        if status == PASS else
        "manual-to-autonomous handoff did not meet gates"
    )
    return status, summary, notes


def run_game_range_dynamics_loop(
    config: GameRangeDynamicsConfig,
    *,
    tracker_factory: Callable[[], TrackerLike] | None = None,
    logger: JsonlLogger | None = None,
) -> GameDynamicsSummary:
    if config.duration_s <= 0:
        raise ValueError("duration_s must be positive")
    if config.hz <= 0:
        raise ValueError("hz must be positive")
    if config.initial_target_width <= 0:
        raise ValueError("initial_target_width must be positive")
    if config.desired_target_width <= 0:
        raise ValueError("desired_target_width must be positive")

    game = build_game_config(config)
    state = initial_offset_state(game, config.initial_offset_x)
    tracker = tracker_factory() if tracker_factory else ColorTargetTracker()
    controller = FlightController(build_pipeline_config(config))
    controller.set_frame_center(config.width, config.height)
    dt = 1.0 / config.hz
    frame_count = int(config.duration_s * config.hz)
    target_width = config.initial_target_width
    horizontal_errors: list[float] = []
    width_errors: list[float] = []
    yaw_commands: list[float] = []
    pitch_commands: list[float] = []
    found_yaw_commands: list[float] = []
    found_pitch_commands: list[float] = []
    observed_widths: list[float] = []
    found_count = 0
    loss_events = 0
    was_found: bool | None = None
    first_bbox: tuple[int, int, int, int] | None = None
    last_bbox: tuple[int, int, int, int] | None = None
    first_center: tuple[float, float] | None = None
    last_center: tuple[float, float] | None = None

    for index in range(frame_count):
        frame, truth_bbox = render_scaled_target_frame(
            state,
            game,
            target_width=target_width,
        )
        if not tracker.is_initialized:
            tracker.init(frame, truth_bbox)
        result = tracker.update(frame)
        controller.update(result)
        command = controller_command(
            controller,
            controller.cfg,
            enable_pitch=True,
            yaw_scale=config.yaw_scale,
            pitch_scale=config.pitch_scale,
        )
        state = step_state(state, GameCommand(yaw=command.yaw), dt, game)
        target_width = max(
            config.min_target_width,
            min(
                config.max_target_width,
                target_width + command.pitch * config.target_width_rate_px_s * dt,
            ),
        )

        if result.found:
            found_count += 1
            center = (float(result.center[0]), float(result.center[1]))
            width = float(result.bbox[2])
            horizontal_error = center[0] - config.width / 2.0
            width_error = config.desired_target_width - width
            horizontal_errors.append(horizontal_error)
            width_errors.append(width_error)
            found_yaw_commands.append(command.yaw)
            found_pitch_commands.append(command.pitch)
            observed_widths.append(width)
            if first_bbox is None and result.bbox:
                first_bbox = tuple(int(value) for value in result.bbox)
                first_center = center
            if result.bbox:
                last_bbox = tuple(int(value) for value in result.bbox)
            last_center = center
        elif was_found:
            loss_events += 1
        was_found = result.found
        yaw_commands.append(command.yaw)
        pitch_commands.append(command.pitch)

        if logger:
            logger.write(
                "game_range_dynamics_sample",
                frame_index=index,
                sim_time=state.t,
                target_found=result.found,
                target_bbox=list(result.bbox) if result.bbox else None,
                target_center=list(result.center) if result.center else None,
                truth_bbox=list(truth_bbox),
                measured_target_width=result.bbox[2] if result.bbox else None,
                simulated_target_width=target_width,
                yaw_error=controller.yaw_error,
                forward_error=controller.forward_error,
                yaw_output=controller.yaw_output,
                forward_output=controller.forward_output,
                command=command.as_dict(),
                camera_x=state.camera_x,
                camera_y=state.camera_y,
            )

    initial_abs_error = abs(horizontal_errors[0]) if horizontal_errors else float("inf")
    final_abs_error = abs(horizontal_errors[-1]) if horizontal_errors else float("inf")
    error_reduction = (
        (initial_abs_error - final_abs_error) / initial_abs_error
        if initial_abs_error > 0 and math.isfinite(initial_abs_error)
        else 0.0
    )
    initial_abs_width_error = abs(width_errors[0]) if width_errors else float("inf")
    final_abs_width_error = abs(width_errors[-1]) if width_errors else float("inf")
    width_reduction = (
        (initial_abs_width_error - final_abs_width_error) / initial_abs_width_error
        if initial_abs_width_error > 0 and math.isfinite(initial_abs_width_error)
        else 0.0
    )
    yaw_direction = corrective_command_metrics(horizontal_errors, found_yaw_commands)
    pitch_direction = corrective_command_metrics(width_errors, found_pitch_commands)
    metrics = {
        "frames": frame_count,
        "target_found": found_count,
        "found_ratio": found_count / frame_count if frame_count else 0.0,
        "loss_events": loss_events,
        "initial_offset_x": config.initial_offset_x,
        "initial_abs_error_px": initial_abs_error,
        "final_abs_error_px": final_abs_error,
        "error_reduction_ratio": error_reduction,
        "horizontal_error_rms": abs_rms(horizontal_errors),
        "horizontal_error_p95": percentile_abs(horizontal_errors, 0.95),
        "initial_target_width": observed_widths[0] if observed_widths else None,
        "last_target_width": observed_widths[-1] if observed_widths else None,
        "desired_target_width": config.desired_target_width,
        "initial_abs_width_error_px": initial_abs_width_error,
        "final_abs_width_error_px": final_abs_width_error,
        "width_error_reduction_ratio": width_reduction,
        "width_error_rms": abs_rms(width_errors),
        "width_error_p95": percentile_abs(width_errors, 0.95),
        "yaw_corrective_command_samples": yaw_direction["samples"],
        "yaw_corrective_command_count": yaw_direction["corrective"],
        "yaw_wrong_direction_command_count": yaw_direction["wrong"],
        "yaw_neutral_command_count": yaw_direction["neutral"],
        "yaw_corrective_command_ratio": yaw_direction["ratio"],
        "yaw_initial_error_px": yaw_direction["first_error"],
        "yaw_initial_command": yaw_direction["first_command"],
        "yaw_initial_command_corrective": yaw_direction["first_corrective"],
        "yaw_direction_error_deadband_px": yaw_direction["error_deadband_px"],
        "pitch_corrective_command_samples": pitch_direction["samples"],
        "pitch_corrective_command_count": pitch_direction["corrective"],
        "pitch_wrong_direction_command_count": pitch_direction["wrong"],
        "pitch_neutral_command_count": pitch_direction["neutral"],
        "pitch_corrective_command_ratio": pitch_direction["ratio"],
        "pitch_initial_error_px": pitch_direction["first_error"],
        "pitch_initial_command": pitch_direction["first_command"],
        "pitch_initial_command_corrective": pitch_direction["first_corrective"],
        "pitch_direction_error_deadband_px": pitch_direction["error_deadband_px"],
        "max_abs_yaw_axis": max((abs(value) for value in yaw_commands), default=0.0),
        "max_abs_pitch_axis": max((abs(value) for value in pitch_commands), default=0.0),
        "first_target_center": list(first_center) if first_center else None,
        "last_target_center": list(last_center) if last_center else None,
        "first_target_bbox": list(first_bbox) if first_bbox else None,
        "last_target_bbox": list(last_bbox) if last_bbox else None,
        "final_camera_x": state.camera_x,
        "final_camera_y": state.camera_y,
        "duration_s": config.duration_s,
        "hz": config.hz,
        "target_motion_radius_x": config.target_motion_radius_x,
        "target_motion_radius_y": config.target_motion_radius_y,
        "camera_speed_px_s": config.camera_speed_px_s,
        "target_width_rate_px_s": config.target_width_rate_px_s,
        "tracker_type": "ColorTargetTracker",
        "real_input": False,
        "control_applied_to_game_camera": True,
        "range_control_applied_to_target_width": True,
        "min_found_ratio": config.min_found_ratio,
        "max_loss_events": config.max_loss_events,
        "max_final_abs_error_px": config.max_final_abs_error_px,
        "min_error_reduction_ratio": config.min_error_reduction_ratio,
        "min_yaw_corrective_ratio": config.min_yaw_corrective_ratio,
        "max_final_abs_width_error_px": config.max_final_abs_width_error_px,
        "min_width_error_reduction_ratio": config.min_width_error_reduction_ratio,
        "min_pitch_corrective_ratio": config.min_pitch_corrective_ratio,
        "max_yaw_axis": config.max_yaw_axis,
        "max_pitch_axis": config.max_pitch_axis,
    }
    status, summary, notes = evaluate_range_metrics(metrics, config)
    return GameDynamicsSummary(status=status, summary=summary, metrics=metrics, notes=notes)


def run_game_handoff_dynamics_loop(
    config: GameHandoffDynamicsConfig,
    *,
    tracker_factory: Callable[[], TrackerLike] | None = None,
    logger: JsonlLogger | None = None,
) -> GameDynamicsSummary:
    if config.duration_s <= 0:
        raise ValueError("duration_s must be positive")
    if config.hz <= 0:
        raise ValueError("hz must be positive")
    if config.manual_duration_s <= 0:
        raise ValueError("manual_duration_s must be positive")
    if config.manual_duration_s >= config.duration_s:
        raise ValueError("manual_duration_s must be shorter than duration_s")

    game = build_game_config(config)
    state = initial_offset_state(game, config.initial_offset_x)
    manual_detector = ColorTargetTracker()
    manual_detector.init(render_frame(state, game), target_screen_bbox(state, game))
    tracker = tracker_factory() if tracker_factory else ColorTargetTracker()
    controller = FlightController(build_pipeline_config(config))
    controller.set_frame_center(config.width, config.height)
    dt = 1.0 / config.hz
    frame_count = int(config.duration_s * config.hz)
    manual_frames = max(1, min(frame_count - 1, int(config.manual_duration_s * config.hz)))
    target_width = config.initial_target_width

    manual_errors: list[float] = []
    auto_horizontal_errors: list[float] = []
    auto_width_errors: list[float] = []
    yaw_commands: list[float] = []
    pitch_commands: list[float] = []
    found_yaw_commands: list[float] = []
    found_pitch_commands: list[float] = []
    observed_widths: list[float] = []
    manual_found = 0
    auto_found = 0
    loss_events = 0
    was_found: bool | None = None
    follow_command_sent = False
    follow_command_frame: int | None = None
    tracker_initialized_before_follow = tracker.is_initialized
    tracker_initialized_after_follow = False
    first_auto_bbox: tuple[int, int, int, int] | None = None
    last_auto_bbox: tuple[int, int, int, int] | None = None
    first_auto_center: tuple[float, float] | None = None
    last_auto_center: tuple[float, float] | None = None

    for index in range(frame_count):
        frame, truth_bbox = render_scaled_target_frame(state, game, target_width=target_width)

        if index < manual_frames:
            result = manual_detector.update(frame)
            if result.found:
                manual_found += 1
                error = float(result.center[0]) - config.width / 2.0
                manual_errors.append(error)
                yaw = clamp(
                    error * config.manual_yaw_scale,
                    -config.max_manual_yaw_axis,
                    config.max_manual_yaw_axis,
                )
            else:
                yaw = 0.0
            state = step_state(state, GameCommand(yaw=yaw), dt, game)
            yaw_commands.append(yaw)
            pitch_commands.append(0.0)
            if logger:
                logger.write(
                    "game_handoff_sample",
                    phase="manual",
                    frame_index=index,
                    sim_time=state.t,
                    target_found=result.found,
                    target_bbox=list(result.bbox) if result.bbox else None,
                    target_center=list(result.center) if result.center else None,
                    truth_bbox=list(truth_bbox),
                    manual_yaw_command=yaw,
                    auto_control_active=False,
                    camera_x=state.camera_x,
                    camera_y=state.camera_y,
                    simulated_target_width=target_width,
                )
            continue

        if not follow_command_sent:
            follow_command_sent = True
            follow_command_frame = index
            tracker.init(frame, truth_bbox)
            tracker_initialized_after_follow = tracker.is_initialized
            controller.reset()
            controller.set_frame_center(config.width, config.height)
            if logger:
                logger.write(
                    "game_handoff_follow_command",
                    frame_index=index,
                    sim_time=state.t,
                    truth_bbox=list(truth_bbox),
                    target_width=target_width,
                )

        result = tracker.update(frame)
        controller.update(result)
        command = controller_command(
            controller,
            controller.cfg,
            enable_pitch=True,
            yaw_scale=config.yaw_scale,
            pitch_scale=config.pitch_scale,
        )
        state = step_state(state, GameCommand(yaw=command.yaw), dt, game)
        target_width = max(
            config.min_target_width,
            min(
                config.max_target_width,
                target_width + command.pitch * config.target_width_rate_px_s * dt,
            ),
        )

        if result.found:
            auto_found += 1
            center = (float(result.center[0]), float(result.center[1]))
            width = float(result.bbox[2])
            horizontal_error = center[0] - config.width / 2.0
            width_error = config.desired_target_width - width
            auto_horizontal_errors.append(horizontal_error)
            auto_width_errors.append(width_error)
            found_yaw_commands.append(command.yaw)
            found_pitch_commands.append(command.pitch)
            observed_widths.append(width)
            if first_auto_bbox is None:
                first_auto_bbox = tuple(int(value) for value in result.bbox)
                first_auto_center = center
            last_auto_bbox = tuple(int(value) for value in result.bbox)
            last_auto_center = center
        elif was_found:
            loss_events += 1
        was_found = result.found
        yaw_commands.append(command.yaw)
        pitch_commands.append(command.pitch)

        if logger:
            logger.write(
                "game_handoff_sample",
                phase="auto",
                frame_index=index,
                sim_time=state.t,
                target_found=result.found,
                target_bbox=list(result.bbox) if result.bbox else None,
                target_center=list(result.center) if result.center else None,
                truth_bbox=list(truth_bbox),
                measured_target_width=result.bbox[2] if result.bbox else None,
                simulated_target_width=target_width,
                yaw_error=controller.yaw_error,
                forward_error=controller.forward_error,
                yaw_output=controller.yaw_output,
                forward_output=controller.forward_output,
                command=command.as_dict(),
                auto_control_active=True,
                camera_x=state.camera_x,
                camera_y=state.camera_y,
            )

    manual_initial_abs_error = abs(manual_errors[0]) if manual_errors else float("inf")
    manual_final_abs_error = abs(manual_errors[-1]) if manual_errors else float("inf")
    initial_abs_error = abs(auto_horizontal_errors[0]) if auto_horizontal_errors else float("inf")
    final_abs_error = abs(auto_horizontal_errors[-1]) if auto_horizontal_errors else float("inf")
    error_reduction = (
        (initial_abs_error - final_abs_error) / initial_abs_error
        if initial_abs_error > 0 and math.isfinite(initial_abs_error)
        else 0.0
    )
    manual_to_final_reduction = (
        (manual_initial_abs_error - final_abs_error) / manual_initial_abs_error
        if manual_initial_abs_error > 0 and math.isfinite(manual_initial_abs_error)
        else 0.0
    )
    initial_abs_width_error = abs(auto_width_errors[0]) if auto_width_errors else float("inf")
    final_abs_width_error = abs(auto_width_errors[-1]) if auto_width_errors else float("inf")
    width_reduction = (
        (initial_abs_width_error - final_abs_width_error) / initial_abs_width_error
        if initial_abs_width_error > 0 and math.isfinite(initial_abs_width_error)
        else 0.0
    )
    yaw_direction = corrective_command_metrics(auto_horizontal_errors, found_yaw_commands)
    pitch_direction = corrective_command_metrics(auto_width_errors, found_pitch_commands)
    auto_frames = frame_count - manual_frames
    metrics = {
        "frames": frame_count,
        "manual_frames": manual_frames,
        "auto_frames": auto_frames,
        "manual_target_found": manual_found,
        "target_found": auto_found,
        "found_ratio": auto_found / auto_frames if auto_frames else 0.0,
        "loss_events": loss_events,
        "follow_command_sent": follow_command_sent,
        "follow_command_frame": follow_command_frame,
        "tracker_initialized_before_follow": tracker_initialized_before_follow,
        "tracker_initialized_after_follow": tracker_initialized_after_follow,
        "pre_handoff_auto_control_samples": 0,
        "manual_initial_abs_error_px": manual_initial_abs_error,
        "manual_final_abs_error_px": manual_final_abs_error,
        "initial_abs_error_px": initial_abs_error,
        "final_abs_error_px": final_abs_error,
        "error_reduction_ratio": error_reduction,
        "manual_to_final_error_reduction_ratio": manual_to_final_reduction,
        "horizontal_error_rms": abs_rms(auto_horizontal_errors),
        "horizontal_error_p95": percentile_abs(auto_horizontal_errors, 0.95),
        "initial_target_width": observed_widths[0] if observed_widths else None,
        "last_target_width": observed_widths[-1] if observed_widths else None,
        "desired_target_width": config.desired_target_width,
        "initial_abs_width_error_px": initial_abs_width_error,
        "final_abs_width_error_px": final_abs_width_error,
        "width_error_reduction_ratio": width_reduction,
        "width_error_rms": abs_rms(auto_width_errors),
        "width_error_p95": percentile_abs(auto_width_errors, 0.95),
        "yaw_corrective_command_samples": yaw_direction["samples"],
        "yaw_corrective_command_count": yaw_direction["corrective"],
        "yaw_wrong_direction_command_count": yaw_direction["wrong"],
        "yaw_neutral_command_count": yaw_direction["neutral"],
        "yaw_corrective_command_ratio": yaw_direction["ratio"],
        "yaw_initial_error_px": yaw_direction["first_error"],
        "yaw_initial_command": yaw_direction["first_command"],
        "yaw_initial_command_corrective": yaw_direction["first_corrective"],
        "yaw_direction_error_deadband_px": yaw_direction["error_deadband_px"],
        "pitch_corrective_command_samples": pitch_direction["samples"],
        "pitch_corrective_command_count": pitch_direction["corrective"],
        "pitch_wrong_direction_command_count": pitch_direction["wrong"],
        "pitch_neutral_command_count": pitch_direction["neutral"],
        "pitch_corrective_command_ratio": pitch_direction["ratio"],
        "pitch_initial_error_px": pitch_direction["first_error"],
        "pitch_initial_command": pitch_direction["first_command"],
        "pitch_initial_command_corrective": pitch_direction["first_corrective"],
        "pitch_direction_error_deadband_px": pitch_direction["error_deadband_px"],
        "max_abs_yaw_axis": max((abs(value) for value in yaw_commands), default=0.0),
        "max_abs_pitch_axis": max((abs(value) for value in pitch_commands), default=0.0),
        "first_target_center": list(first_auto_center) if first_auto_center else None,
        "last_target_center": list(last_auto_center) if last_auto_center else None,
        "first_target_bbox": list(first_auto_bbox) if first_auto_bbox else None,
        "last_target_bbox": list(last_auto_bbox) if last_auto_bbox else None,
        "final_camera_x": state.camera_x,
        "final_camera_y": state.camera_y,
        "duration_s": config.duration_s,
        "manual_duration_s": config.manual_duration_s,
        "hz": config.hz,
        "target_motion_radius_x": config.target_motion_radius_x,
        "target_motion_radius_y": config.target_motion_radius_y,
        "camera_speed_px_s": config.camera_speed_px_s,
        "target_width_rate_px_s": config.target_width_rate_px_s,
        "tracker_type": "ColorTargetTracker",
        "real_input": False,
        "control_applied_to_game_camera": True,
        "range_control_applied_to_target_width": True,
        "manual_to_auto_handoff": True,
        "min_found_ratio": config.min_found_ratio,
        "max_loss_events": config.max_loss_events,
        "max_handoff_abs_error_px": config.max_handoff_abs_error_px,
        "max_final_abs_error_px": config.max_final_abs_error_px,
        "min_error_reduction_ratio": config.min_error_reduction_ratio,
        "min_manual_to_final_error_reduction_ratio": (
            config.min_manual_to_final_error_reduction_ratio
        ),
        "min_yaw_corrective_ratio": config.min_yaw_corrective_ratio,
        "max_final_abs_width_error_px": config.max_final_abs_width_error_px,
        "min_width_error_reduction_ratio": config.min_width_error_reduction_ratio,
        "min_pitch_corrective_ratio": config.min_pitch_corrective_ratio,
        "max_yaw_axis": config.max_yaw_axis,
        "max_pitch_axis": config.max_pitch_axis,
    }
    status, summary, notes = evaluate_handoff_metrics(metrics, config)
    return GameDynamicsSummary(status=status, summary=summary, metrics=metrics, notes=notes)


def build_markdown(result: GameDynamicsSummary, *, run_id: str) -> str:
    lines = [
        "# Game Dynamics Closed Loop",
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
        "manual_frames",
        "auto_frames",
        "found_ratio",
        "manual_final_abs_error_px",
        "initial_abs_error_px",
        "final_abs_error_px",
        "error_reduction_ratio",
        "yaw_corrective_command_ratio",
        "yaw_initial_command_corrective",
        "initial_abs_width_error_px",
        "final_abs_width_error_px",
        "width_error_reduction_ratio",
        "pitch_corrective_command_ratio",
        "pitch_initial_command_corrective",
        "max_abs_yaw_axis",
        "max_abs_pitch_axis",
        "control_applied_to_game_camera",
        "range_control_applied_to_target_width",
        "manual_to_auto_handoff",
    ):
        if key in result.metrics:
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


def write_reports(result: GameDynamicsSummary, log_dir: Path, *, run_id: str) -> tuple[Path, Path]:
    log_dir.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime("%Y%m%d-%H%M%S", time.localtime())
    json_path = log_dir / ("%s-%s-game-dynamics.json" % (stamp, run_id))
    md_path = log_dir / ("%s-%s-game-dynamics.md" % (stamp, run_id))
    json_path.write_text(json.dumps(result.as_dict(), indent=2, sort_keys=True) + "\n",
                         encoding="utf-8")
    md_path.write_text(build_markdown(result, run_id=run_id), encoding="utf-8")
    return json_path, md_path


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-id", default="game-dynamics-loop")
    parser.add_argument("--log-dir", type=Path, default=DEFAULT_LOG_DIR)
    parser.add_argument("--duration", type=float, default=4.0)
    parser.add_argument("--hz", type=float, default=20.0)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--initial-offset-x", type=float, default=180.0)
    parser.add_argument("--target-motion-radius-x", type=float, default=0.0)
    parser.add_argument("--target-motion-radius-y", type=float, default=0.0)
    parser.add_argument("--camera-speed", type=float, default=260.0)
    parser.add_argument("--max-final-error", type=float, default=35.0)
    parser.add_argument("--min-error-reduction", type=float, default=0.65)
    parser.add_argument("--min-found-ratio", type=float, default=0.9)
    parser.add_argument("--max-yaw-axis", type=float, default=0.8)
    parser.add_argument("--range-dynamics", action="store_true",
                        help="Also control target apparent size through pitch/forward PID")
    parser.add_argument("--handoff-dynamics", action="store_true",
                        help="Run scripted manual centering before autonomous follow takeover")
    parser.add_argument("--manual-duration", type=float, default=1.6)
    parser.add_argument("--initial-target-width", type=float, default=70.0)
    parser.add_argument("--desired-target-width", type=float, default=120.0)
    parser.add_argument("--max-final-width-error", type=float, default=8.0)
    parser.add_argument("--min-width-error-reduction", type=float, default=0.65)
    parser.add_argument("--max-pitch-axis", type=float, default=0.8)
    args = parser.parse_args(argv)
    if args.duration <= 0:
        parser.error("--duration must be positive")
    if args.hz <= 0:
        parser.error("--hz must be positive")
    if args.width <= 0 or args.height <= 0:
        parser.error("--width/--height must be positive")
    if args.camera_speed <= 0:
        parser.error("--camera-speed must be positive")
    if args.max_final_error < 0:
        parser.error("--max-final-error must be non-negative")
    if not 0 <= args.min_error_reduction <= 1:
        parser.error("--min-error-reduction must be in [0, 1]")
    if not 0 <= args.min_found_ratio <= 1:
        parser.error("--min-found-ratio must be in [0, 1]")
    if not 0 < args.max_yaw_axis <= 1:
        parser.error("--max-yaw-axis must be in (0, 1]")
    if args.initial_target_width <= 0:
        parser.error("--initial-target-width must be positive")
    if args.handoff_dynamics and args.range_dynamics:
        parser.error("use --handoff-dynamics or --range-dynamics, not both")
    if args.manual_duration <= 0:
        parser.error("--manual-duration must be positive")
    if args.manual_duration >= args.duration:
        parser.error("--manual-duration must be shorter than --duration")
    if args.desired_target_width <= 0:
        parser.error("--desired-target-width must be positive")
    if args.max_final_width_error < 0:
        parser.error("--max-final-width-error must be non-negative")
    if not 0 <= args.min_width_error_reduction <= 1:
        parser.error("--min-width-error-reduction must be in [0, 1]")
    if not 0 < args.max_pitch_axis <= 1:
        parser.error("--max-pitch-axis must be in (0, 1]")
    return args


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    log_dir = resolve_log_dir(str(args.log_dir), repo_root=REPO_ROOT)
    log_path = make_log_path(log_dir, "%s-game-dynamics-loop" % args.run_id)
    config_cls = (
        GameHandoffDynamicsConfig if args.handoff_dynamics else
        GameRangeDynamicsConfig if args.range_dynamics else
        GameDynamicsConfig
    )
    config = config_cls(
        duration_s=args.duration,
        hz=args.hz,
        width=args.width,
        height=args.height,
        initial_offset_x=args.initial_offset_x,
        target_motion_radius_x=args.target_motion_radius_x,
        target_motion_radius_y=args.target_motion_radius_y,
        camera_speed_px_s=args.camera_speed,
        min_found_ratio=args.min_found_ratio,
        max_final_abs_error_px=args.max_final_error,
        min_error_reduction_ratio=args.min_error_reduction,
        max_yaw_axis=args.max_yaw_axis,
        run_id=args.run_id,
        log_dir=log_dir,
    )
    if isinstance(config, GameHandoffDynamicsConfig):
        config.manual_duration_s = args.manual_duration
    if isinstance(config, GameRangeDynamicsConfig):
        config.initial_target_width = args.initial_target_width
        config.desired_target_width = args.desired_target_width
        config.max_final_abs_width_error_px = args.max_final_width_error
        config.min_width_error_reduction_ratio = args.min_width_error_reduction
        config.max_pitch_axis = args.max_pitch_axis
    with JsonlLogger(log_path, metadata={
        "tool": "game_dynamics_loop",
        "run_id": args.run_id,
        "source": "in-process-simple-target-game",
        "real_input": False,
        "control_applied_to_game_camera": True,
        "range_dynamics": args.range_dynamics,
        "handoff_dynamics": args.handoff_dynamics,
    }) as logger:
        result = (
            run_game_handoff_dynamics_loop(config, logger=logger)
            if args.handoff_dynamics else
            run_game_range_dynamics_loop(config, logger=logger)
            if args.range_dynamics else
            run_game_dynamics_loop(config, logger=logger)
        )
        logger.write("game_dynamics_summary", **result.metrics)
    result.metrics["log_path"] = str(log_path)
    json_path, md_path = write_reports(result, log_dir, run_id=args.run_id)
    print("game-dynamics-loop %s report=%s summary=%s" % (
        result.status,
        json_path,
        md_path,
    ))
    print(result.summary)
    return 0 if result.status == PASS else 1


if __name__ == "__main__":
    raise SystemExit(main())
