#!/usr/bin/env python3
"""Plan the first post-extended-follow yaw-only target phase.

This runner is deliberately conservative. It can execute a deterministic
synthetic yaw-only moving-target regression, but it does not launch pr0p and it
does not send real OS input. The first real pr0p phase is now an airborne
static-target yaw convergence proof: the vehicle moves while a selected static
target stays tracked, yaw error decreases, pitch remains disabled, and the
tracker does not lock onto screen-fixed OSD.
"""

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
TOOLS_DIR = REPO_ROOT / "tools"
for item in (SANDBOX_DIR, TOOLS_DIR):
    if str(item) not in sys.path:
        sys.path.insert(0, str(item))
PROBE_DIR = Path(__file__).resolve().parent
if str(PROBE_DIR) not in sys.path:
    sys.path.insert(0, str(PROBE_DIR))

from capture_window import SyntheticFrameSource  # noqa: E402
from pr0p_capture_probe import DEFAULT_LOG_DIR  # noqa: E402
from pr0p_tracking_probe import load_bbox_file  # noqa: E402
from screen_tracking_loop import LoopConfig, parse_bbox, run_loop  # noqa: E402
from virtual_input import DryRunInputAdapter  # noqa: E402
from kenet.tracker import ObjectTracker, TrackerType  # noqa: E402
from sitl_log import JsonlLogger, make_log_path  # noqa: E402


PASS = "PASS"
WAITING = "WAITING"
FAIL = "FAIL"
DEFAULT_SYNTHETIC_DURATION_S = 4.0
DEFAULT_SYNTHETIC_HZ = 20.0
DEFAULT_MIN_FOUND_RATIO = 0.90
DEFAULT_MAX_LOSS_EVENTS = 0
DEFAULT_MIN_CENTER_MOTION_PX = 30.0
DEFAULT_MAX_YAW_AXIS = 0.8
DEFAULT_MAX_PITCH_AXIS = 0.0
DEFAULT_DESIRED_TARGET_WIDTH = 120.0
DEFAULT_REAL_DURATION_S = 30.0
DEFAULT_REAL_MIN_FOUND_RATIO = 0.90
DEFAULT_REAL_MAX_LOSS_EVENTS = 2
DEFAULT_REAL_MIN_IMAGE_MOTION_PX = 30.0
DEFAULT_REAL_MIN_TARGET_MOTION_PX = DEFAULT_REAL_MIN_IMAGE_MOTION_PX
DEFAULT_REAL_MAX_YAW_AXIS = 0.8
DEFAULT_REAL_MIN_CENTER_ERROR_REDUCTION_RATIO = 0.50
DEFAULT_REAL_MAX_FINAL_CENTER_ERROR_PX = 40.0
DEFAULT_REAL_MIN_INITIAL_CENTER_ERROR_PX = 30.0


def command_block(*parts: str) -> str:
    return " \\\n  ".join(parts)


def bbox_arg(bbox: tuple[int, int, int, int] | None) -> str:
    return ",".join(str(item) for item in bbox) if bbox else "x,y,w,h"


def load_json_report(path: Path | None) -> dict[str, Any] | None:
    if path is None:
        return None
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except Exception:
        return None
    return data if isinstance(data, dict) else None


def step_by_name(report: dict[str, Any] | None, name: str) -> dict[str, Any] | None:
    if not report or not isinstance(report.get("steps"), list):
        return None
    for step in report["steps"]:
        if isinstance(step, dict) and step.get("name") == name:
            return step
    return None


def metric_float(metrics: dict[str, Any], name: str) -> float | None:
    try:
        return float(metrics[name])
    except (KeyError, TypeError, ValueError):
        return None


def metric_int(metrics: dict[str, Any], name: str) -> int | None:
    try:
        return int(metrics[name])
    except (KeyError, TypeError, ValueError):
        return None


def list_pair(value: Any) -> tuple[float, float] | None:
    if not isinstance(value, list) or len(value) < 2:
        return None
    try:
        return float(value[0]), float(value[1])
    except (TypeError, ValueError):
        return None


def capture_width(metrics: dict[str, Any]) -> float | None:
    capture_region = metrics.get("capture_region")
    if isinstance(capture_region, dict):
        try:
            return float(capture_region["width"])
        except (KeyError, TypeError, ValueError):
            return None
    return metric_float(metrics, "frame_width")


def horizontal_error_metrics(metrics: dict[str, Any]) -> tuple[float | None, float | None, float | None, float | None]:
    initial_abs = metric_float(metrics, "initial_abs_horizontal_error")
    final_abs = metric_float(metrics, "last_abs_horizontal_error")
    reduction_px = metric_float(metrics, "horizontal_error_reduction_px")
    reduction_ratio = metric_float(metrics, "horizontal_error_reduction_ratio")
    if initial_abs is not None and final_abs is not None:
        if reduction_px is None:
            reduction_px = initial_abs - final_abs
        if reduction_ratio is None:
            reduction_ratio = 1.0 if initial_abs == 0 and final_abs == 0 else (
                reduction_px / initial_abs if initial_abs else 0.0
            )
        return initial_abs, final_abs, reduction_px, reduction_ratio

    width = capture_width(metrics)
    first_center = list_pair(metrics.get("first_target_center"))
    last_center = list_pair(metrics.get("last_target_center"))
    if width is None or first_center is None or last_center is None:
        return initial_abs, final_abs, reduction_px, reduction_ratio
    initial_abs = abs(first_center[0] - width / 2.0)
    final_abs = abs(last_center[0] - width / 2.0)
    reduction_px = initial_abs - final_abs
    reduction_ratio = 1.0 if initial_abs == 0 and final_abs == 0 else (
        reduction_px / initial_abs if initial_abs else 0.0
    )
    return initial_abs, final_abs, reduction_px, reduction_ratio


def resolve_nested_report_path(path: str | None, *, base: Path) -> Path | None:
    if not path:
        return None
    nested = Path(path)
    if nested.is_absolute():
        return nested
    if nested.exists():
        return nested
    return base.parent / nested


def resolve_bbox_argument(
    *,
    bbox: tuple[int, int, int, int] | None,
    bbox_file: Path | None,
    parser: argparse.ArgumentParser | None = None,
) -> tuple[int, int, int, int] | None:
    if bbox and bbox_file:
        message = "use either --bbox or --bbox-file, not both"
        if parser:
            parser.error(message)
        raise ValueError(message)
    if not bbox_file:
        return bbox
    try:
        return load_bbox_file(bbox_file)
    except Exception as exc:
        if parser:
            parser.error(str(exc))
        raise


@dataclass
class MovingTargetStep:
    name: str
    status: str
    summary: str
    command: str = ""
    report: str | None = None
    metrics: dict[str, Any] = field(default_factory=dict)
    notes: list[str] = field(default_factory=list)

    def as_dict(self) -> dict[str, Any]:
        return {
            "name": self.name,
            "status": self.status,
            "summary": self.summary,
            "command": self.command,
            "report": self.report,
            "metrics": self.metrics,
            "notes": self.notes,
        }


@dataclass
class MovingTargetYawReport:
    run_id: str
    started_at: str
    status: str
    summary: str
    next_action: str
    steps: list[MovingTargetStep] = field(default_factory=list)
    commands: dict[str, str] = field(default_factory=dict)
    metrics: dict[str, Any] = field(default_factory=dict)

    def as_dict(self) -> dict[str, Any]:
        return {
            "run_id": self.run_id,
            "started_at": self.started_at,
            "status": self.status,
            "summary": self.summary,
            "next_action": self.next_action,
            "steps": [step.as_dict() for step in self.steps],
            "commands": self.commands,
            "metrics": self.metrics,
        }


def synthetic_yaw_command(
    *,
    run_id: str,
    duration_s: float,
    hz: float,
    min_found_ratio: float,
    max_loss_events: int,
    min_center_motion_px: float,
    max_yaw_axis: float,
    desired_target_width: float,
) -> str:
    return command_block(
        "fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_moving_target_yaw_plan.py",
        "--execute-synthetic",
        "--duration %s" % duration_s,
        "--hz %s" % hz,
        "--min-found-ratio %s" % min_found_ratio,
        "--max-loss-events %s" % max_loss_events,
        "--min-center-motion %s" % min_center_motion_px,
        "--max-yaw-axis %s" % max_yaw_axis,
        "--desired-target-width %s" % desired_target_width,
        "--run-id %s-synthetic-yaw" % run_id,
    )


def real_pr0p_yaw_command(
    *,
    run_id: str,
    bbox: tuple[int, int, int, int] | None,
    duration_s: float,
    min_found_ratio: float,
    max_loss_events: int,
    min_image_motion_px: float,
    min_center_error_reduction_ratio: float,
    max_final_center_error_px: float,
) -> str:
    return command_block(
        "fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_moving_target_acceptance_runner.py",
        "--bbox %s" % bbox_arg(bbox),
        "--run-live-gate",
        "--ack-live-input",
        "--ack-airborne-static-target",
        "--real-duration %s" % duration_s,
        "--real-min-found-ratio %s" % min_found_ratio,
        "--real-max-loss-events %s" % max_loss_events,
        "--real-min-image-motion %s" % min_image_motion_px,
        "--real-min-center-error-reduction %s" % min_center_error_reduction_ratio,
        "--real-max-final-center-error %s" % max_final_center_error_px,
        "--run-id %s-real-pr0p-moving-target-yaw" % run_id,
    )


def evaluate_real_pr0p_yaw_evidence(
    *,
    command: str,
    real_tracking_report: Path | None,
    ack_real_moving_target: bool,
    real_duration_s: float,
    real_min_found_ratio: float,
    real_max_loss_events: int,
    real_min_target_motion_px: float,
    real_max_yaw_axis: float,
    bbox: tuple[int, int, int, int] | None,
    real_min_center_error_reduction_ratio: float = DEFAULT_REAL_MIN_CENTER_ERROR_REDUCTION_RATIO,
    real_max_final_center_error_px: float = DEFAULT_REAL_MAX_FINAL_CENTER_ERROR_PX,
    real_min_initial_center_error_px: float = DEFAULT_REAL_MIN_INITIAL_CENTER_ERROR_PX,
) -> MovingTargetStep:
    base_metrics = {
        "scenario": "airborne_static_target_yaw",
        "bbox": list(bbox) if bbox else None,
        "required_duration_s": real_duration_s,
        "required_min_found_ratio": real_min_found_ratio,
        "required_max_loss_events": real_max_loss_events,
        "required_min_image_motion_px": real_min_target_motion_px,
        "required_min_target_motion_px": real_min_target_motion_px,
        "required_max_yaw_axis": real_max_yaw_axis,
        "required_min_center_error_reduction_ratio": real_min_center_error_reduction_ratio,
        "required_max_final_center_error_px": real_max_final_center_error_px,
        "required_min_initial_center_error_px": real_min_initial_center_error_px,
        "real_tracking_report": str(real_tracking_report) if real_tracking_report else None,
        "ack_airborne_static_target": ack_real_moving_target,
        "ack_real_moving_target": ack_real_moving_target,
        "real_input": True,
        "enable_pitch": False,
    }
    if real_tracking_report is None:
        return MovingTargetStep(
            name="real_pr0p_moving_target_yaw",
            status=WAITING,
            summary="real pr0p airborne static-target yaw evidence is not available yet",
            command=command,
            notes=[
                "select a static in-scene target while the vehicle is controlled from the FPV view",
                "keep this phase yaw-only; add pitch/approach only after this gate is stable",
                "this command proves vehicle-image motion plus yaw-error convergence, not a moving target yet",
            ],
            metrics=base_metrics,
        )

    acceptance = load_json_report(real_tracking_report)
    if acceptance is None:
        return MovingTargetStep(
            name="real_pr0p_moving_target_yaw",
            status=FAIL,
            summary="real pr0p moving-target tracking report could not be read",
            command=command,
            notes=["REAL_TRACKING_REPORT_UNREADABLE"],
            metrics=base_metrics,
        )

    live_step = step_by_name(acceptance, "pr0p_tracking_live")
    live_report_path = resolve_nested_report_path(
        str(live_step.get("report")) if live_step else None,
        base=real_tracking_report,
    )
    live_probe = load_json_report(live_report_path)
    acceptance_metrics = (
        acceptance.get("metrics") if isinstance(acceptance.get("metrics"), dict) else {}
    )
    probe_metrics = (
        live_probe.get("metrics") if isinstance(live_probe, dict)
        and isinstance(live_probe.get("metrics"), dict) else {}
    )
    acceptance_status = str(acceptance.get("status", WAITING))
    live_status = str(live_step.get("status", WAITING)) if live_step else WAITING
    probe_status = str(live_probe.get("status", WAITING)) if live_probe else WAITING
    duration_s = metric_float(acceptance_metrics, "duration_s")
    found_ratio = metric_float(probe_metrics, "found_ratio")
    loss_events = metric_int(probe_metrics, "loss_events")
    image_motion_px = metric_float(probe_metrics, "target_center_span_px")
    target_travel_px = metric_float(probe_metrics, "target_center_travel_px")
    initial_abs_error, final_abs_error, error_reduction_px, error_reduction_ratio = (
        horizontal_error_metrics(probe_metrics)
    )
    max_abs_yaw_axis = metric_float(probe_metrics, "max_abs_yaw_axis")
    max_abs_pitch_axis = metric_float(probe_metrics, "max_abs_pitch_axis")
    enable_pitch = probe_metrics.get("enable_pitch")
    real_input_sent = probe_metrics.get("real_input_sent")
    post_loop_fc = probe_metrics.get("post_loop_fc")
    post_loop_armed = post_loop_fc.get("armed") if isinstance(post_loop_fc, dict) else None
    probe_notes = live_probe.get("notes") if isinstance(live_probe, dict) else []
    screen_fixed_lock_suspected = (
        probe_metrics.get("screen_fixed_lock_suspected") is True
        or (isinstance(probe_notes, list) and "SCREEN_FIXED_LOCK_SUSPECTED" in probe_notes)
    )

    metrics = {
        **base_metrics,
        "tracking_acceptance_status": acceptance_status,
        "tracking_live_status": live_status,
        "tracking_probe_status": probe_status,
        "tracking_probe_report": str(live_report_path) if live_report_path else None,
        "duration_s": duration_s,
        "found_ratio": found_ratio,
        "loss_events": loss_events,
        "image_motion_px": image_motion_px,
        "target_center_span_px": image_motion_px,
        "target_center_travel_px": target_travel_px,
        "initial_abs_horizontal_error": initial_abs_error,
        "last_abs_horizontal_error": final_abs_error,
        "horizontal_error_reduction_px": error_reduction_px,
        "horizontal_error_reduction_ratio": error_reduction_ratio,
        "max_abs_yaw_axis": max_abs_yaw_axis,
        "max_abs_pitch_axis": max_abs_pitch_axis,
        "probe_enable_pitch": enable_pitch,
        "probe_real_input_sent": real_input_sent,
        "post_loop_armed": post_loop_armed,
        "screen_fixed_lock_suspected": screen_fixed_lock_suspected,
    }

    missing: list[str] = []
    failed: list[str] = []
    if not ack_real_moving_target:
        missing.append("ack_airborne_static_target")
    if acceptance_status == FAIL:
        failed.append("tracking_acceptance_failed")
    elif acceptance_status != PASS:
        missing.append("tracking_acceptance_pass")
    if live_status == FAIL:
        failed.append("pr0p_tracking_live_failed")
    elif live_status != PASS:
        missing.append("pr0p_tracking_live_pass")
    if probe_status == FAIL:
        failed.append("tracking_probe_failed")
    elif probe_status != PASS:
        missing.append("tracking_probe_pass")
    if duration_s is None or duration_s < real_duration_s:
        failed.append("real_duration_too_short")
    if found_ratio is None or found_ratio < real_min_found_ratio:
        failed.append("real_found_ratio_low")
    if loss_events is None or loss_events > real_max_loss_events:
        failed.append("real_loss_events_high")
    if image_motion_px is None or image_motion_px < real_min_target_motion_px:
        failed.append("real_image_motion_low")
    if initial_abs_error is None or initial_abs_error < real_min_initial_center_error_px:
        failed.append("real_initial_center_error_low")
    if final_abs_error is None or final_abs_error > real_max_final_center_error_px:
        failed.append("real_final_center_error_high")
    if error_reduction_ratio is None or error_reduction_ratio < real_min_center_error_reduction_ratio:
        failed.append("real_center_error_not_converged")
    if max_abs_yaw_axis is None or max_abs_yaw_axis > real_max_yaw_axis:
        failed.append("real_yaw_unbounded")
    if max_abs_pitch_axis is None or max_abs_pitch_axis > 0.0:
        failed.append("real_pitch_not_zero")
    if enable_pitch is not False:
        failed.append("real_enable_pitch_not_false")
    if real_input_sent is not True:
        failed.append("real_input_not_sent")
    if post_loop_armed is not True:
        failed.append("real_post_loop_not_armed")
    if screen_fixed_lock_suspected:
        failed.append("real_screen_fixed_lock_suspected")

    if failed:
        return MovingTargetStep(
            name="real_pr0p_moving_target_yaw",
            status=FAIL,
            summary="real pr0p airborne static-target yaw evidence failed: %s" % ", ".join(failed[:4]),
            command=command,
            report=str(real_tracking_report),
            metrics=metrics,
            notes=failed + missing,
        )
    if missing:
        return MovingTargetStep(
            name="real_pr0p_moving_target_yaw",
            status=WAITING,
            summary="real pr0p airborne static-target yaw evidence is incomplete: %s" % ", ".join(missing[:4]),
            command=command,
            report=str(real_tracking_report),
            metrics=metrics,
            notes=missing,
        )
    return MovingTargetStep(
        name="real_pr0p_moving_target_yaw",
        status=PASS,
        summary="real pr0p airborne static-target yaw convergence is proven",
        command=command,
        report=str(real_tracking_report),
        metrics=metrics,
        notes=["yaw-only airborne static-target live evidence passed"],
    )


def run_synthetic_yaw_regression(
    *,
    run_id: str,
    log_dir: Path,
    duration_s: float,
    hz: float,
    min_found_ratio: float,
    max_loss_events: int,
    min_center_motion_px: float,
    max_yaw_axis: float,
    desired_target_width: float,
) -> MovingTargetStep:
    source = SyntheticFrameSource(width=640, height=480, fps=hz, target_size=80)
    tracker = ObjectTracker(TrackerType.KCF)
    adapter = DryRunInputAdapter()
    log_path = make_log_path(log_dir, "%s-moving-target-yaw-synthetic" % run_id)
    config = LoopConfig(
        duration_s=duration_s,
        hz=hz,
        initial_bbox=source.target_bbox_at(0.0),
        enable_pitch=False,
        run_id=run_id,
        tracker_type=TrackerType.KCF,
        frame_width=source.width,
        frame_height=source.height,
        yaw_kp=0.8,
        yaw_ki=0.0,
        yaw_kd=0.0,
        yaw_output_limit=120.0,
        desired_target_width=desired_target_width,
    )
    with JsonlLogger(log_path, metadata={
        "tool": "pr0p_moving_target_yaw_plan",
        "phase": "synthetic_moving_target_yaw",
        "run_id": run_id,
        "source": "synthetic-moving-target",
        "adapter": adapter.__class__.__name__,
        "real_input": False,
        "enable_pitch": False,
    }) as logger:
        summary = run_loop(source, tracker, adapter, config, logger=logger)
        logger.write("pr0p_moving_target_yaw_synthetic_summary", **summary.as_dict())
    adapter.close()

    neutralized = adapter.commands[-1][1].is_neutral() if adapter.commands else False
    center_motion_px = summary.target_center_span_px or 0.0
    center_motion_ok = center_motion_px >= min_center_motion_px
    found_ok = summary.found_ratio >= min_found_ratio
    loss_ok = summary.loss_events <= max_loss_events
    yaw_bounded = summary.max_abs_yaw_axis <= max_yaw_axis
    pitch_zero = summary.max_abs_pitch_axis <= DEFAULT_MAX_PITCH_AXIS
    ok = (
        summary.frames > 0
        and found_ok
        and loss_ok
        and center_motion_ok
        and yaw_bounded
        and pitch_zero
        and neutralized
    )
    notes: list[str] = [
        "dry-run only: no joystick command is sent",
        "yaw-only: pitch/approach output must remain zero",
    ]
    if not center_motion_ok:
        notes.append("TARGET_MOTION_TOO_LOW")
    if not found_ok:
        notes.append("FOUND_RATIO_LOW")
    if not loss_ok:
        notes.append("LOSS_EVENTS_HIGH")
    if not yaw_bounded or not pitch_zero:
        notes.append("PID_COMMAND_UNBOUNDED")
    if not neutralized:
        notes.append("NOT_NEUTRALIZED")
    return MovingTargetStep(
        name="synthetic_moving_target_yaw",
        status=PASS if ok else FAIL,
        summary=(
            "synthetic moving-target yaw-only tracker/PID regression passed"
            if ok else "synthetic moving-target yaw-only tracker/PID regression failed"
        ),
        report=str(log_path),
        metrics={
            **summary.as_dict(),
            "source": "synthetic-moving-target",
            "initial_bbox": list(source.target_bbox_at(0.0)),
            "duration_s": duration_s,
            "hz": hz,
            "min_found_ratio": min_found_ratio,
            "max_loss_events": max_loss_events,
            "min_center_motion_px": min_center_motion_px,
            "center_motion_ok": center_motion_ok,
            "max_yaw_axis": max_yaw_axis,
            "yaw_bounded": yaw_bounded,
            "pitch_zero": pitch_zero,
            "neutralized": neutralized,
            "enable_pitch": False,
            "real_input": False,
            "log_path": str(log_path),
        },
        notes=notes,
    )


def build_plan_report(
    *,
    run_id: str,
    log_dir: Path,
    bbox: tuple[int, int, int, int] | None,
    execute_synthetic: bool,
    duration_s: float,
    hz: float,
    min_found_ratio: float,
    max_loss_events: int,
    min_center_motion_px: float,
    max_yaw_axis: float,
    desired_target_width: float,
    real_duration_s: float,
    real_min_found_ratio: float,
    real_max_loss_events: int,
    real_min_target_motion_px: float,
    real_max_yaw_axis: float,
    real_min_center_error_reduction_ratio: float = DEFAULT_REAL_MIN_CENTER_ERROR_REDUCTION_RATIO,
    real_max_final_center_error_px: float = DEFAULT_REAL_MAX_FINAL_CENTER_ERROR_PX,
    real_min_initial_center_error_px: float = DEFAULT_REAL_MIN_INITIAL_CENTER_ERROR_PX,
    real_tracking_report: Path | None = None,
    ack_real_moving_target: bool = False,
) -> MovingTargetYawReport:
    commands = {
        "synthetic_moving_target_yaw": synthetic_yaw_command(
            run_id=run_id,
            duration_s=duration_s,
            hz=hz,
            min_found_ratio=min_found_ratio,
            max_loss_events=max_loss_events,
            min_center_motion_px=min_center_motion_px,
            max_yaw_axis=max_yaw_axis,
            desired_target_width=desired_target_width,
        ),
        "real_pr0p_moving_target_yaw_live": real_pr0p_yaw_command(
            run_id=run_id,
            bbox=bbox,
            duration_s=real_duration_s,
            min_found_ratio=real_min_found_ratio,
            max_loss_events=real_max_loss_events,
            min_image_motion_px=real_min_target_motion_px,
            min_center_error_reduction_ratio=real_min_center_error_reduction_ratio,
            max_final_center_error_px=real_max_final_center_error_px,
        ),
    }
    if execute_synthetic:
        synthetic_step = run_synthetic_yaw_regression(
            run_id=run_id,
            log_dir=log_dir,
            duration_s=duration_s,
            hz=hz,
            min_found_ratio=min_found_ratio,
            max_loss_events=max_loss_events,
            min_center_motion_px=min_center_motion_px,
            max_yaw_axis=max_yaw_axis,
            desired_target_width=desired_target_width,
        )
    else:
        synthetic_step = MovingTargetStep(
            name="synthetic_moving_target_yaw",
            status=WAITING,
            summary="synthetic moving-target yaw-only regression has not been run",
            command=commands["synthetic_moving_target_yaw"],
            notes=["rerun with --execute-synthetic; this sends no real input"],
            metrics={"real_input": False},
        )

    real_step = evaluate_real_pr0p_yaw_evidence(
        command=commands["real_pr0p_moving_target_yaw_live"],
        real_tracking_report=real_tracking_report,
        ack_real_moving_target=ack_real_moving_target,
        real_duration_s=real_duration_s,
        real_min_found_ratio=real_min_found_ratio,
        real_max_loss_events=real_max_loss_events,
        real_min_target_motion_px=real_min_target_motion_px,
        real_max_yaw_axis=real_max_yaw_axis,
        bbox=bbox,
        real_min_center_error_reduction_ratio=real_min_center_error_reduction_ratio,
        real_max_final_center_error_px=real_max_final_center_error_px,
        real_min_initial_center_error_px=real_min_initial_center_error_px,
    )
    steps = [synthetic_step, real_step]
    if synthetic_step.status == FAIL or real_step.status == FAIL:
        status = FAIL
        summary = "airborne static-target yaw-only evidence failed"
        next_action = "Inspect the failed airborne yaw step before adding pitch or approach."
    elif synthetic_step.status == PASS and real_step.status == PASS:
        status = PASS
        summary = "airborne static-target yaw-only phase is proven"
        next_action = "Yaw-only airborne static-target evidence is stable; approach/pitch can be planned behind a new measured gate."
    else:
        status = WAITING
        summary = "airborne static-target yaw-only phase is planned; real pr0p convergence evidence is still missing"
        next_action = (
            "Select a static target from the FPV view, then run the "
            "real_pr0p_moving_target_yaw_live command while keeping pitch/approach disabled."
        )
    return MovingTargetYawReport(
        run_id=run_id,
        started_at=time.strftime("%Y-%m-%dT%H:%M:%S%z", time.localtime()),
        status=status,
        summary=summary,
        next_action=next_action,
        steps=steps,
        commands=commands,
        metrics={
            "bbox": list(bbox) if bbox else None,
            "log_dir": str(log_dir),
            "execute_synthetic": execute_synthetic,
            "synthetic_status": synthetic_step.status,
            "real_pr0p_status": real_step.status,
            "real_tracking_report": str(real_tracking_report) if real_tracking_report else None,
            "ack_airborne_static_target": ack_real_moving_target,
            "ack_real_moving_target": ack_real_moving_target,
            "evidence_only": True,
            "real_input_sent": False,
        },
    )


def build_markdown(report: MovingTargetYawReport) -> str:
    lines = [
        "# pr0p Airborne Static Target Yaw Plan",
        "",
        "Run: `%s`" % report.run_id,
        "Started: `%s`" % report.started_at,
        "Status: `%s`" % report.status,
        "",
        report.summary,
        "",
        "Next action: %s" % report.next_action,
        "",
        "| Step | Status | Summary |",
        "| --- | --- | --- |",
    ]
    for step in report.steps:
        lines.append("| %s | `%s` | %s |" % (step.name, step.status, step.summary))
    lines.extend(["", "## Commands", ""])
    for name, command in report.commands.items():
        lines.extend(["### %s" % name, "", "```bash", command, "```", ""])
    lines.extend(["", "## Metrics", "", "```json"])
    lines.append(json.dumps(report.metrics, indent=2, sort_keys=True))
    lines.extend(["```", ""])
    return "\n".join(lines)


def write_reports(report: MovingTargetYawReport, log_dir: Path) -> tuple[Path, Path]:
    log_dir.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime("%Y%m%d-%H%M%S", time.localtime())
    json_path = log_dir / ("%s-%s-moving-target-yaw.json" % (stamp, report.run_id))
    md_path = log_dir / ("%s-%s-moving-target-yaw.md" % (stamp, report.run_id))
    json_path.write_text(json.dumps(report.as_dict(), indent=2, sort_keys=True) + "\n",
                         encoding="utf-8")
    md_path.write_text(build_markdown(report), encoding="utf-8")
    return json_path, md_path


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-id", default="pr0p-moving-target-yaw")
    parser.add_argument("--log-dir", type=Path, default=DEFAULT_LOG_DIR)
    parser.add_argument("--bbox", type=parse_bbox)
    parser.add_argument("--bbox-file", type=Path)
    parser.add_argument("--execute-synthetic", action="store_true")
    parser.add_argument("--duration", type=float, default=DEFAULT_SYNTHETIC_DURATION_S)
    parser.add_argument("--hz", type=float, default=DEFAULT_SYNTHETIC_HZ)
    parser.add_argument("--min-found-ratio", type=float, default=DEFAULT_MIN_FOUND_RATIO)
    parser.add_argument("--max-loss-events", type=int, default=DEFAULT_MAX_LOSS_EVENTS)
    parser.add_argument("--min-center-motion", type=float,
                        default=DEFAULT_MIN_CENTER_MOTION_PX)
    parser.add_argument("--max-yaw-axis", type=float, default=DEFAULT_MAX_YAW_AXIS)
    parser.add_argument("--desired-target-width", type=float,
                        default=DEFAULT_DESIRED_TARGET_WIDTH)
    parser.add_argument("--real-duration", type=float, default=DEFAULT_REAL_DURATION_S)
    parser.add_argument("--real-min-found-ratio", type=float,
                        default=DEFAULT_REAL_MIN_FOUND_RATIO)
    parser.add_argument("--real-max-loss-events", type=int,
                        default=DEFAULT_REAL_MAX_LOSS_EVENTS)
    parser.add_argument("--real-min-image-motion", "--real-min-target-motion",
                        dest="real_min_image_motion", type=float,
                        default=DEFAULT_REAL_MIN_IMAGE_MOTION_PX)
    parser.add_argument("--real-max-yaw-axis", type=float,
                        default=DEFAULT_REAL_MAX_YAW_AXIS)
    parser.add_argument("--real-min-center-error-reduction", type=float,
                        default=DEFAULT_REAL_MIN_CENTER_ERROR_REDUCTION_RATIO)
    parser.add_argument("--real-max-final-center-error", type=float,
                        default=DEFAULT_REAL_MAX_FINAL_CENTER_ERROR_PX)
    parser.add_argument("--real-min-initial-center-error", type=float,
                        default=DEFAULT_REAL_MIN_INITIAL_CENTER_ERROR_PX)
    parser.add_argument("--real-tracking-report", type=Path)
    parser.add_argument("--ack-airborne-static-target", action="store_true",
                        help="Confirm the supplied real report tracked a static in-scene target while the vehicle moved")
    parser.add_argument("--ack-real-moving-target", action="store_true",
                        help="Legacy alias for --ack-airborne-static-target")
    args = parser.parse_args(argv)
    if args.duration <= 0:
        parser.error("--duration must be positive")
    if args.hz <= 0:
        parser.error("--hz must be positive")
    if not 0 <= args.min_found_ratio <= 1:
        parser.error("--min-found-ratio must be in [0, 1]")
    if args.max_loss_events < 0:
        parser.error("--max-loss-events must be non-negative")
    if args.min_center_motion < 0:
        parser.error("--min-center-motion must be non-negative")
    if args.max_yaw_axis < 0:
        parser.error("--max-yaw-axis must be non-negative")
    if args.desired_target_width <= 0:
        parser.error("--desired-target-width must be positive")
    if args.real_duration <= 0:
        parser.error("--real-duration must be positive")
    if not 0 <= args.real_min_found_ratio <= 1:
        parser.error("--real-min-found-ratio must be in [0, 1]")
    if args.real_max_loss_events < 0:
        parser.error("--real-max-loss-events must be non-negative")
    if args.real_min_image_motion < 0:
        parser.error("--real-min-image-motion must be non-negative")
    if args.real_max_yaw_axis < 0:
        parser.error("--real-max-yaw-axis must be non-negative")
    if not 0 <= args.real_min_center_error_reduction <= 1:
        parser.error("--real-min-center-error-reduction must be in [0, 1]")
    if args.real_max_final_center_error < 0:
        parser.error("--real-max-final-center-error must be non-negative")
    if args.real_min_initial_center_error < 0:
        parser.error("--real-min-initial-center-error must be non-negative")
    args.bbox = resolve_bbox_argument(
        bbox=args.bbox,
        bbox_file=args.bbox_file,
        parser=parser,
    )
    return args


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    report = build_plan_report(
        run_id=args.run_id,
        log_dir=args.log_dir,
        bbox=args.bbox,
        execute_synthetic=args.execute_synthetic,
        duration_s=args.duration,
        hz=args.hz,
        min_found_ratio=args.min_found_ratio,
        max_loss_events=args.max_loss_events,
        min_center_motion_px=args.min_center_motion,
        max_yaw_axis=args.max_yaw_axis,
        desired_target_width=args.desired_target_width,
        real_duration_s=args.real_duration,
        real_min_found_ratio=args.real_min_found_ratio,
        real_max_loss_events=args.real_max_loss_events,
        real_min_target_motion_px=args.real_min_image_motion,
        real_max_yaw_axis=args.real_max_yaw_axis,
        real_min_center_error_reduction_ratio=args.real_min_center_error_reduction,
        real_max_final_center_error_px=args.real_max_final_center_error,
        real_min_initial_center_error_px=args.real_min_initial_center_error,
        real_tracking_report=args.real_tracking_report,
        ack_real_moving_target=(
            args.ack_airborne_static_target or args.ack_real_moving_target
        ),
    )
    json_path, md_path = write_reports(report, args.log_dir)
    print("simitl-pr0p-moving-target-yaw %s report=%s summary=%s" % (
        report.status,
        json_path,
        md_path,
    ))
    print(report.next_action)
    return 1 if report.status == FAIL else 0


if __name__ == "__main__":
    raise SystemExit(main())
