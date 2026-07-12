#!/usr/bin/env python3
"""Objective-flow closed loop for the isolated simple game sandbox.

This gate models the intended user flow without Gazebo, Betaflight, pr0p,
screen capture, or OS input:

1. A manual phase nudges the camera toward the target.
2. A follow button event is emitted.
3. Tracker initialization and PID control start only after follow.
4. Yaw and pitch commands are sent through an InputAdapter-like object.
5. The simple game applies the adapter command to the next rendered frame.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))
TOOLS_DIR = REPO_ROOT / "tools"
if str(TOOLS_DIR) not in sys.path:
    sys.path.insert(0, str(TOOLS_DIR))

from game_dynamics_loop import (  # noqa: E402
    ColorTargetTracker,
    GameHandoffDynamicsConfig,
    TrackerLike,
    abs_rms,
    build_game_config,
    build_pipeline_config,
    clamp,
    corrective_command_metrics,
    evaluate_handoff_metrics,
    initial_offset_state,
    percentile_abs,
    render_scaled_target_frame,
)
from screen_tracking_loop import controller_command  # noqa: E402
from simple_target_game import GameCommand, step_state, target_screen_bbox  # noqa: E402
from sitl_log import JsonlLogger, make_log_path, resolve_log_dir  # noqa: E402
from virtual_input import AxisCommand, normalize_button_name  # noqa: E402
from kenet.controller import FlightController  # noqa: E402


PASS = "PASS"
FAIL = "FAIL"
DEFAULT_LOG_DIR = Path("logs/game_screen_sandbox")


@dataclass
class ObjectiveLoopConfig(GameHandoffDynamicsConfig):
    run_id: str = "simple-game-objective-loop"
    follow_button: str = "south"
    min_adapter_nonneutral_commands: int = 4
    min_applied_nonneutral_commands: int = 4
    min_follow_button_events: int = 2


@dataclass
class ObjectiveLoopSummary:
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


class ObjectiveInputAdapter:
    """InputAdapter-like recorder used by the in-process objective gate."""

    def __init__(self):
        self.commands: list[tuple[float, AxisCommand]] = []
        self.button_events: list[tuple[float, str, bool]] = []
        self.last_command = AxisCommand()
        self.closed = False

    def send(self, command: AxisCommand) -> None:
        if self.closed:
            raise RuntimeError("adapter is closed")
        self.last_command = command.clamped()
        self.commands.append((time.monotonic(), self.last_command))

    def press_button(self, button: str, pressed: bool = True) -> None:
        if self.closed:
            raise RuntimeError("adapter is closed")
        self.button_events.append(
            (time.monotonic(), normalize_button_name(button), bool(pressed))
        )

    def neutral(self) -> None:
        self.press_button("south", False)
        self.press_button("east", False)
        self.send(AxisCommand())

    def close(self) -> None:
        if not self.closed:
            self.neutral()
            self.closed = True


def evaluate_objective_metrics(
    metrics: dict[str, Any],
    config: ObjectiveLoopConfig,
) -> tuple[str, str, list[str]]:
    status, _summary, notes = evaluate_handoff_metrics(metrics, config)
    notes = list(notes)
    if not metrics.get("manual_commands_through_input_adapter"):
        notes.append("MANUAL_COMMANDS_NOT_THROUGH_ADAPTER")
    if not metrics.get("autopilot_commands_through_input_adapter"):
        notes.append("AUTOPILOT_COMMANDS_NOT_THROUGH_ADAPTER")
    if not metrics.get("control_applied_through_input_adapter"):
        notes.append("CONTROL_NOT_APPLIED_THROUGH_ADAPTER")
    if metrics.get("nonneutral_adapter_commands", 0) < config.min_adapter_nonneutral_commands:
        notes.append("ADAPTER_NONNEUTRAL_COMMANDS_LOW")
    if metrics.get("applied_nonneutral_commands", 0) < config.min_applied_nonneutral_commands:
        notes.append("APPLIED_NONNEUTRAL_COMMANDS_LOW")
    if metrics.get("follow_button_events", 0) < config.min_follow_button_events:
        notes.append("FOLLOW_BUTTON_EVENTS_LOW")
    if not metrics.get("neutralized"):
        notes.append("INPUT_NOT_NEUTRALIZED")
    status = PASS if not notes else FAIL
    summary = (
        "objective-flow simple game loop follows the target through the adapter"
        if status == PASS else
        "objective-flow simple game loop did not meet gates"
    )
    return status, summary, notes


def run_objective_closed_loop(
    config: ObjectiveLoopConfig,
    *,
    tracker_factory: Callable[[], TrackerLike] | None = None,
    logger: JsonlLogger | None = None,
    adapter: ObjectiveInputAdapter | None = None,
) -> ObjectiveLoopSummary:
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
    manual_detector.init(
        render_scaled_target_frame(state, game, target_width=config.initial_target_width)[0],
        target_screen_bbox(state, game),
    )
    tracker = tracker_factory() if tracker_factory else ColorTargetTracker()
    controller = FlightController(build_pipeline_config(config))
    controller.set_frame_center(config.width, config.height)
    input_adapter = adapter or ObjectiveInputAdapter()
    dt = 1.0 / config.hz
    frame_count = int(config.duration_s * config.hz)
    manual_frames = max(1, min(frame_count - 1, int(config.manual_duration_s * config.hz)))
    target_width = config.initial_target_width

    manual_errors: list[float] = []
    auto_horizontal_errors: list[float] = []
    auto_width_errors: list[float] = []
    yaw_commands: list[float] = []
    pitch_commands: list[float] = []
    applied_yaw_commands: list[float] = []
    applied_pitch_commands: list[float] = []
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
    pre_handoff_auto_control_samples = 0
    first_auto_bbox: tuple[int, int, int, int] | None = None
    last_auto_bbox: tuple[int, int, int, int] | None = None
    first_auto_center: tuple[float, float] | None = None
    last_auto_center: tuple[float, float] | None = None

    try:
        for index in range(frame_count):
            frame, truth_bbox = render_scaled_target_frame(
                state,
                game,
                target_width=target_width,
            )

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
                input_adapter.send(AxisCommand(yaw=yaw))
                applied = input_adapter.last_command
                state = step_state(
                    state,
                    GameCommand(yaw=applied.yaw, pitch=0.0),
                    dt,
                    game,
                )
                yaw_commands.append(yaw)
                pitch_commands.append(0.0)
                applied_yaw_commands.append(applied.yaw)
                applied_pitch_commands.append(applied.pitch)
                if logger:
                    logger.write(
                        "objective_loop_sample",
                        phase="manual",
                        frame_index=index,
                        sim_time=state.t,
                        target_found=result.found,
                        target_bbox=list(result.bbox) if result.bbox else None,
                        target_center=list(result.center) if result.center else None,
                        truth_bbox=list(truth_bbox),
                        command=applied.as_dict(),
                        auto_control_active=False,
                        camera_x=state.camera_x,
                        camera_y=state.camera_y,
                        simulated_target_width=target_width,
                    )
                continue

            if not follow_command_sent:
                follow_command_sent = True
                follow_command_frame = index
                input_adapter.press_button(config.follow_button, True)
                input_adapter.press_button(config.follow_button, False)
                tracker.init(frame, truth_bbox)
                tracker_initialized_after_follow = tracker.is_initialized
                controller.reset()
                controller.set_frame_center(config.width, config.height)
                if logger:
                    logger.write(
                        "objective_loop_follow_command",
                        frame_index=index,
                        sim_time=state.t,
                        truth_bbox=list(truth_bbox),
                        target_width=target_width,
                        follow_button=config.follow_button,
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
            input_adapter.send(command)
            applied = input_adapter.last_command
            state = step_state(state, GameCommand(yaw=applied.yaw), dt, game)
            target_width = max(
                config.min_target_width,
                min(
                    config.max_target_width,
                    target_width + applied.pitch * config.target_width_rate_px_s * dt,
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
                found_yaw_commands.append(applied.yaw)
                found_pitch_commands.append(applied.pitch)
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
            applied_yaw_commands.append(applied.yaw)
            applied_pitch_commands.append(applied.pitch)

            if logger:
                logger.write(
                    "objective_loop_sample",
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
                    command=applied.as_dict(),
                    auto_control_active=True,
                    camera_x=state.camera_x,
                    camera_y=state.camera_y,
                )
    finally:
        input_adapter.close()

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
    nonneutral_adapter_commands = sum(
        1 for _timestamp, command in input_adapter.commands if not command.is_neutral()
    )
    applied_nonneutral_commands = sum(
        1
        for yaw, pitch in zip(applied_yaw_commands, applied_pitch_commands)
        if abs(yaw) > 1e-6 or abs(pitch) > 1e-6
    )
    follow_button_events = sum(
        1 for _timestamp, button, _pressed in input_adapter.button_events
        if button == normalize_button_name(config.follow_button)
    )
    neutralized = (
        input_adapter.commands[-1][1].is_neutral()
        if input_adapter.commands else False
    )
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
        "follow_button": normalize_button_name(config.follow_button),
        "follow_button_events": follow_button_events,
        "tracker_initialized_before_follow": tracker_initialized_before_follow,
        "tracker_initialized_after_follow": tracker_initialized_after_follow,
        "pre_handoff_auto_control_samples": pre_handoff_auto_control_samples,
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
        "max_abs_applied_yaw_axis": max((abs(value) for value in applied_yaw_commands), default=0.0),
        "max_abs_applied_pitch_axis": max((abs(value) for value in applied_pitch_commands), default=0.0),
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
        "adapter": input_adapter.__class__.__name__,
        "adapter_commands": len(input_adapter.commands),
        "button_events": len(input_adapter.button_events),
        "nonneutral_adapter_commands": nonneutral_adapter_commands,
        "applied_adapter_commands": len(applied_yaw_commands),
        "applied_nonneutral_commands": applied_nonneutral_commands,
        "neutralized": neutralized,
        "real_input": False,
        "input_adapter_contract": True,
        "manual_commands_through_input_adapter": True,
        "autopilot_commands_through_input_adapter": True,
        "control_applied_through_input_adapter": True,
        "control_applied_to_game_camera": True,
        "range_control_applied_to_target_width": True,
        "manual_to_auto_handoff": True,
        "sandbox_only": True,
        "gazebo_independent": True,
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
        "min_adapter_nonneutral_commands": config.min_adapter_nonneutral_commands,
        "min_applied_nonneutral_commands": config.min_applied_nonneutral_commands,
        "min_follow_button_events": config.min_follow_button_events,
    }
    status, summary, notes = evaluate_objective_metrics(metrics, config)
    return ObjectiveLoopSummary(status=status, summary=summary, metrics=metrics, notes=notes)


def build_markdown(result: ObjectiveLoopSummary, *, run_id: str) -> str:
    lines = [
        "# Simple Game Objective Loop",
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
        "final_abs_error_px",
        "final_abs_width_error_px",
        "manual_to_auto_handoff",
        "follow_button_events",
        "control_applied_through_input_adapter",
        "nonneutral_adapter_commands",
        "applied_nonneutral_commands",
        "neutralized",
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
    result: ObjectiveLoopSummary,
    log_dir: Path,
    *,
    run_id: str,
) -> tuple[Path, Path]:
    log_dir.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime("%Y%m%d-%H%M%S", time.localtime())
    json_path = log_dir / ("%s-%s-simple-game-objective.json" % (stamp, run_id))
    md_path = log_dir / ("%s-%s-simple-game-objective.md" % (stamp, run_id))
    json_path.write_text(json.dumps(result.as_dict(), indent=2, sort_keys=True) + "\n",
                         encoding="utf-8")
    md_path.write_text(build_markdown(result, run_id=run_id), encoding="utf-8")
    return json_path, md_path


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-id", default="simple-game-objective-loop")
    parser.add_argument("--log-dir", type=Path, default=DEFAULT_LOG_DIR)
    parser.add_argument("--duration", type=float, default=6.0)
    parser.add_argument("--hz", type=float, default=20.0)
    parser.add_argument("--manual-duration", type=float, default=1.6)
    parser.add_argument("--initial-offset-x", type=float, default=180.0)
    parser.add_argument("--initial-target-width", type=float, default=70.0)
    parser.add_argument("--desired-target-width", type=float, default=120.0)
    parser.add_argument("--max-final-error", type=float, default=35.0)
    parser.add_argument("--max-final-width-error", type=float, default=8.0)
    parser.add_argument("--min-found-ratio", type=float, default=0.9)
    parser.add_argument("--max-yaw-axis", type=float, default=0.8)
    parser.add_argument("--max-pitch-axis", type=float, default=0.8)
    args = parser.parse_args(argv)
    if args.duration <= 0:
        parser.error("--duration must be positive")
    if args.hz <= 0:
        parser.error("--hz must be positive")
    if args.manual_duration <= 0:
        parser.error("--manual-duration must be positive")
    if args.manual_duration >= args.duration:
        parser.error("--manual-duration must be shorter than --duration")
    if args.initial_target_width <= 0:
        parser.error("--initial-target-width must be positive")
    if args.desired_target_width <= 0:
        parser.error("--desired-target-width must be positive")
    if args.max_final_error < 0 or args.max_final_width_error < 0:
        parser.error("final error limits must be non-negative")
    if not 0 <= args.min_found_ratio <= 1:
        parser.error("--min-found-ratio must be in [0, 1]")
    if not 0 < args.max_yaw_axis <= 1:
        parser.error("--max-yaw-axis must be in (0, 1]")
    if not 0 < args.max_pitch_axis <= 1:
        parser.error("--max-pitch-axis must be in (0, 1]")
    return args


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    log_dir = resolve_log_dir(str(args.log_dir), repo_root=REPO_ROOT)
    log_path = make_log_path(log_dir, "%s-simple-game-objective-loop" % args.run_id)
    config = ObjectiveLoopConfig(
        duration_s=args.duration,
        hz=args.hz,
        manual_duration_s=args.manual_duration,
        initial_offset_x=args.initial_offset_x,
        initial_target_width=args.initial_target_width,
        desired_target_width=args.desired_target_width,
        max_final_abs_error_px=args.max_final_error,
        max_final_abs_width_error_px=args.max_final_width_error,
        min_found_ratio=args.min_found_ratio,
        max_yaw_axis=args.max_yaw_axis,
        max_pitch_axis=args.max_pitch_axis,
        run_id=args.run_id,
        log_dir=log_dir,
    )
    with JsonlLogger(log_path, metadata={
        "tool": "simple_game_objective_loop",
        "run_id": args.run_id,
        "source": "in-process-simple-target-game-objective",
        "adapter": "ObjectiveInputAdapter",
        "real_input": False,
        "gazebo_independent": True,
    }) as logger:
        result = run_objective_closed_loop(config, logger=logger)
        logger.write("simple_game_objective_summary", **result.metrics)
    result.metrics["log_path"] = str(log_path)
    json_path, md_path = write_reports(result, log_dir, run_id=args.run_id)
    print("simple-game-objective-loop %s report=%s summary=%s" % (
        result.status,
        json_path,
        md_path,
    ))
    print(result.summary)
    return 0 if result.status == PASS else 1


if __name__ == "__main__":
    raise SystemExit(main())
