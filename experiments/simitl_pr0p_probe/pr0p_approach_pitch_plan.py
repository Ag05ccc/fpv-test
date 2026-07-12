#!/usr/bin/env python3
"""Plan the approach/pitch phase after moving-target yaw-only passes."""

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

from game_dynamics_loop import GameRangeDynamicsConfig, run_game_range_dynamics_loop  # noqa: E402
from pr0p_capture_probe import DEFAULT_LOG_DIR  # noqa: E402
from pr0p_tracking_probe import load_bbox_file  # noqa: E402
from screen_tracking_loop import parse_bbox  # noqa: E402
from sitl_log import JsonlLogger, make_log_path  # noqa: E402


PASS = "PASS"
WAITING = "WAITING"
FAIL = "FAIL"
DEFAULT_SYNTHETIC_DURATION_S = 5.0
DEFAULT_SYNTHETIC_HZ = 20.0
DEFAULT_MIN_FOUND_RATIO = 0.90
DEFAULT_MAX_LOSS_EVENTS = 0
DEFAULT_INITIAL_TARGET_WIDTH = 70.0
DEFAULT_DESIRED_TARGET_WIDTH = 120.0
DEFAULT_MAX_FINAL_WIDTH_ERROR = 8.0
DEFAULT_MIN_WIDTH_ERROR_REDUCTION = 0.65
DEFAULT_MAX_YAW_AXIS = 0.8
DEFAULT_MAX_PITCH_AXIS = 0.8
DEFAULT_REAL_DURATION_S = 20.0
DEFAULT_REAL_MIN_FOUND_RATIO = 0.90
DEFAULT_REAL_MAX_LOSS_EVENTS = 2
DEFAULT_REAL_MAX_FINAL_WIDTH_ERROR = 12.0
DEFAULT_REAL_MIN_WIDTH_ERROR_REDUCTION = 0.50
DEFAULT_REAL_MIN_INITIAL_WIDTH_ERROR = 10.0
DEFAULT_REAL_MAX_PITCH_AXIS = 0.8
DEFAULT_REAL_MIN_PITCH_AXIS = 0.02


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
class ApproachPitchStep:
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
class ApproachPitchReport:
    run_id: str
    started_at: str
    status: str
    summary: str
    next_action: str
    steps: list[ApproachPitchStep] = field(default_factory=list)
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


def synthetic_approach_command(
    *,
    run_id: str,
    duration_s: float,
    hz: float,
    initial_target_width: float,
    desired_target_width: float,
    max_final_width_error: float,
    min_width_error_reduction: float,
    max_pitch_axis: float,
) -> str:
    return command_block(
        "fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_approach_pitch_plan.py",
        "--execute-synthetic",
        "--duration %s" % duration_s,
        "--hz %s" % hz,
        "--initial-target-width %s" % initial_target_width,
        "--desired-target-width %s" % desired_target_width,
        "--max-final-width-error %s" % max_final_width_error,
        "--min-width-error-reduction %s" % min_width_error_reduction,
        "--max-pitch-axis %s" % max_pitch_axis,
        "--run-id %s-synthetic-approach" % run_id,
    )


def real_approach_command(
    *,
    run_id: str,
    bbox: tuple[int, int, int, int] | None,
    duration_s: float,
    min_found_ratio: float,
    max_loss_events: int,
    desired_target_width: float,
    max_pitch_axis: float,
) -> str:
    return command_block(
        "fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_tracking_acceptance_runner.py",
        "--bbox %s" % bbox_arg(bbox),
        "--run-live-gates",
        "--ack-live-input",
        "--arm-first",
        "--arm-throttle -0.4",
        "--enable-pitch",
        "--duration %s" % duration_s,
        "--min-found-ratio %s" % min_found_ratio,
        "--max-loss-events %s" % max_loss_events,
        "--desired-target-width %s" % desired_target_width,
        "--max-abs-pitch-axis %s" % max_pitch_axis,
        "--run-id %s-real-pr0p-approach-pitch" % run_id,
    )


def run_synthetic_approach_regression(
    *,
    run_id: str,
    log_dir: Path,
    duration_s: float,
    hz: float,
    min_found_ratio: float,
    max_loss_events: int,
    initial_target_width: float,
    desired_target_width: float,
    max_final_width_error: float,
    min_width_error_reduction: float,
    max_yaw_axis: float,
    max_pitch_axis: float,
) -> ApproachPitchStep:
    log_path = make_log_path(log_dir, "%s-approach-pitch-synthetic" % run_id)
    config = GameRangeDynamicsConfig(
        duration_s=duration_s,
        hz=hz,
        initial_offset_x=160.0,
        initial_target_width=initial_target_width,
        desired_target_width=desired_target_width,
        max_final_abs_width_error_px=max_final_width_error,
        min_width_error_reduction_ratio=min_width_error_reduction,
        min_found_ratio=min_found_ratio,
        max_loss_events=max_loss_events,
        max_yaw_axis=max_yaw_axis,
        max_pitch_axis=max_pitch_axis,
        run_id=run_id,
        log_dir=log_dir,
    )
    with JsonlLogger(log_path, metadata={
        "tool": "pr0p_approach_pitch_plan",
        "phase": "synthetic_range_approach",
        "run_id": run_id,
        "source": "in-process-range-dynamics",
        "real_input": False,
        "enable_pitch": True,
    }) as logger:
        result = run_game_range_dynamics_loop(config, logger=logger)
        logger.write("pr0p_approach_pitch_synthetic_summary", **result.metrics)
    metrics = {
        **result.metrics,
        "log_path": str(log_path),
        "real_input": False,
        "enable_pitch": True,
    }
    return ApproachPitchStep(
        name="synthetic_range_approach",
        status=result.status,
        summary=result.summary,
        report=str(log_path),
        metrics=metrics,
        notes=result.notes + ["dry-run only: no joystick command is sent"],
    )


def evaluate_real_approach_evidence(
    *,
    command: str,
    real_tracking_report: Path | None,
    ack_real_approach_target: bool,
    real_duration_s: float,
    real_min_found_ratio: float,
    real_max_loss_events: int,
    desired_target_width: float,
    real_max_final_width_error: float,
    real_min_width_error_reduction: float,
    real_min_initial_width_error: float,
    real_max_pitch_axis: float,
    real_min_pitch_axis: float,
    bbox: tuple[int, int, int, int] | None,
) -> ApproachPitchStep:
    base_metrics = {
        "bbox": list(bbox) if bbox else None,
        "required_duration_s": real_duration_s,
        "required_min_found_ratio": real_min_found_ratio,
        "required_max_loss_events": real_max_loss_events,
        "desired_target_width": desired_target_width,
        "required_max_final_width_error": real_max_final_width_error,
        "required_min_width_error_reduction": real_min_width_error_reduction,
        "required_min_initial_width_error": real_min_initial_width_error,
        "required_max_pitch_axis": real_max_pitch_axis,
        "required_min_pitch_axis": real_min_pitch_axis,
        "real_tracking_report": str(real_tracking_report) if real_tracking_report else None,
        "ack_real_approach_target": ack_real_approach_target,
        "real_input": True,
        "enable_pitch": True,
    }
    if real_tracking_report is None:
        return ApproachPitchStep(
            name="real_pr0p_approach_pitch",
            status=WAITING,
            summary="real pr0p approach/pitch evidence is not available yet",
            command=command,
            metrics=base_metrics,
            notes=[
                "run a live tracking acceptance with --enable-pitch after yaw-only moving target passes",
                "refresh this report with --real-tracking-report and --ack-real-approach-target",
            ],
        )

    acceptance = load_json_report(real_tracking_report)
    if acceptance is None:
        return ApproachPitchStep(
            name="real_pr0p_approach_pitch",
            status=FAIL,
            summary="real pr0p approach tracking report could not be read",
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
    initial_width = metric_float(probe_metrics, "initial_target_width")
    last_width = metric_float(probe_metrics, "last_target_width")
    max_abs_pitch_axis = metric_float(probe_metrics, "max_abs_pitch_axis")
    max_abs_yaw_axis = metric_float(probe_metrics, "max_abs_yaw_axis")
    enable_pitch = probe_metrics.get("enable_pitch")
    real_input_sent = probe_metrics.get("real_input_sent")
    initial_width_error = (
        abs(desired_target_width - initial_width) if initial_width is not None else None
    )
    final_width_error = (
        abs(desired_target_width - last_width) if last_width is not None else None
    )
    width_error_reduction = (
        (initial_width_error - final_width_error) / initial_width_error
        if initial_width_error and final_width_error is not None else None
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
        "initial_target_width": initial_width,
        "last_target_width": last_width,
        "initial_abs_width_error_px": initial_width_error,
        "final_abs_width_error_px": final_width_error,
        "width_error_reduction_ratio": width_error_reduction,
        "max_abs_pitch_axis": max_abs_pitch_axis,
        "max_abs_yaw_axis": max_abs_yaw_axis,
        "probe_enable_pitch": enable_pitch,
        "probe_real_input_sent": real_input_sent,
    }

    missing: list[str] = []
    failed: list[str] = []
    if not ack_real_approach_target:
        missing.append("ack_real_approach_target")
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
    if initial_width_error is None or initial_width_error < real_min_initial_width_error:
        failed.append("real_initial_width_error_low")
    if final_width_error is None or final_width_error > real_max_final_width_error:
        failed.append("real_final_width_error_high")
    if width_error_reduction is None or width_error_reduction < real_min_width_error_reduction:
        failed.append("real_width_error_reduction_low")
    if max_abs_pitch_axis is None or max_abs_pitch_axis < real_min_pitch_axis:
        failed.append("real_pitch_not_used")
    if max_abs_pitch_axis is None or max_abs_pitch_axis > real_max_pitch_axis:
        failed.append("real_pitch_unbounded")
    if enable_pitch is not True:
        failed.append("real_enable_pitch_not_true")
    if real_input_sent is not True:
        failed.append("real_input_not_sent")

    if failed:
        return ApproachPitchStep(
            name="real_pr0p_approach_pitch",
            status=FAIL,
            summary="real pr0p approach/pitch evidence failed: %s" % ", ".join(failed[:4]),
            command=command,
            report=str(real_tracking_report),
            metrics=metrics,
            notes=failed + missing,
        )
    if missing:
        return ApproachPitchStep(
            name="real_pr0p_approach_pitch",
            status=WAITING,
            summary="real pr0p approach/pitch evidence is incomplete: %s" % ", ".join(missing[:4]),
            command=command,
            report=str(real_tracking_report),
            metrics=metrics,
            notes=missing,
        )
    return ApproachPitchStep(
        name="real_pr0p_approach_pitch",
        status=PASS,
        summary="real pr0p approach/pitch control is proven",
        command=command,
        report=str(real_tracking_report),
        metrics=metrics,
        notes=["approach/pitch live evidence passed"],
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
    initial_target_width: float,
    desired_target_width: float,
    max_final_width_error: float,
    min_width_error_reduction: float,
    max_yaw_axis: float,
    max_pitch_axis: float,
    real_duration_s: float,
    real_min_found_ratio: float,
    real_max_loss_events: int,
    real_max_final_width_error: float,
    real_min_width_error_reduction: float,
    real_min_initial_width_error: float,
    real_max_pitch_axis: float,
    real_min_pitch_axis: float,
    real_tracking_report: Path | None,
    ack_real_approach_target: bool,
) -> ApproachPitchReport:
    commands = {
        "synthetic_range_approach": synthetic_approach_command(
            run_id=run_id,
            duration_s=duration_s,
            hz=hz,
            initial_target_width=initial_target_width,
            desired_target_width=desired_target_width,
            max_final_width_error=max_final_width_error,
            min_width_error_reduction=min_width_error_reduction,
            max_pitch_axis=max_pitch_axis,
        ),
        "real_pr0p_approach_pitch_live": real_approach_command(
            run_id=run_id,
            bbox=bbox,
            duration_s=real_duration_s,
            min_found_ratio=real_min_found_ratio,
            max_loss_events=real_max_loss_events,
            desired_target_width=desired_target_width,
            max_pitch_axis=real_max_pitch_axis,
        ),
    }
    if execute_synthetic:
        synthetic_step = run_synthetic_approach_regression(
            run_id=run_id,
            log_dir=log_dir,
            duration_s=duration_s,
            hz=hz,
            min_found_ratio=min_found_ratio,
            max_loss_events=max_loss_events,
            initial_target_width=initial_target_width,
            desired_target_width=desired_target_width,
            max_final_width_error=max_final_width_error,
            min_width_error_reduction=min_width_error_reduction,
            max_yaw_axis=max_yaw_axis,
            max_pitch_axis=max_pitch_axis,
        )
    else:
        synthetic_step = ApproachPitchStep(
            name="synthetic_range_approach",
            status=WAITING,
            summary="synthetic range/approach regression has not been run",
            command=commands["synthetic_range_approach"],
            notes=["rerun with --execute-synthetic; this sends no real input"],
            metrics={"real_input": False, "enable_pitch": True},
        )
    real_step = evaluate_real_approach_evidence(
        command=commands["real_pr0p_approach_pitch_live"],
        real_tracking_report=real_tracking_report,
        ack_real_approach_target=ack_real_approach_target,
        real_duration_s=real_duration_s,
        real_min_found_ratio=real_min_found_ratio,
        real_max_loss_events=real_max_loss_events,
        desired_target_width=desired_target_width,
        real_max_final_width_error=real_max_final_width_error,
        real_min_width_error_reduction=real_min_width_error_reduction,
        real_min_initial_width_error=real_min_initial_width_error,
        real_max_pitch_axis=real_max_pitch_axis,
        real_min_pitch_axis=real_min_pitch_axis,
        bbox=bbox,
    )
    steps = [synthetic_step, real_step]
    if synthetic_step.status == FAIL or real_step.status == FAIL:
        status = FAIL
        summary = "approach/pitch evidence failed"
        next_action = "Inspect the failed approach/pitch step before using it for autonomous follow."
    elif synthetic_step.status == PASS and real_step.status == PASS:
        status = PASS
        summary = "approach/pitch phase is proven"
        next_action = "Approach/pitch evidence is stable; manual-to-autonomous handoff can be planned behind a new gate."
    else:
        status = WAITING
        summary = "approach/pitch phase is planned; real pr0p approach evidence is still missing"
        next_action = (
            "Run the real_pr0p_approach_pitch_live command only after moving-target yaw-only passes, "
            "then refresh with --real-tracking-report and --ack-real-approach-target."
        )
    return ApproachPitchReport(
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
            "ack_real_approach_target": ack_real_approach_target,
            "desired_target_width": desired_target_width,
            "evidence_only": True,
            "real_input_sent": False,
        },
    )


def build_markdown(report: ApproachPitchReport) -> str:
    lines = [
        "# pr0p Approach Pitch Plan",
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


def write_reports(report: ApproachPitchReport, log_dir: Path) -> tuple[Path, Path]:
    log_dir.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime("%Y%m%d-%H%M%S", time.localtime())
    json_path = log_dir / ("%s-%s-approach-pitch.json" % (stamp, report.run_id))
    md_path = log_dir / ("%s-%s-approach-pitch.md" % (stamp, report.run_id))
    json_path.write_text(json.dumps(report.as_dict(), indent=2, sort_keys=True) + "\n",
                         encoding="utf-8")
    md_path.write_text(build_markdown(report), encoding="utf-8")
    return json_path, md_path


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-id", default="pr0p-approach-pitch")
    parser.add_argument("--log-dir", type=Path, default=DEFAULT_LOG_DIR)
    parser.add_argument("--bbox", type=parse_bbox)
    parser.add_argument("--bbox-file", type=Path)
    parser.add_argument("--execute-synthetic", action="store_true")
    parser.add_argument("--duration", type=float, default=DEFAULT_SYNTHETIC_DURATION_S)
    parser.add_argument("--hz", type=float, default=DEFAULT_SYNTHETIC_HZ)
    parser.add_argument("--min-found-ratio", type=float, default=DEFAULT_MIN_FOUND_RATIO)
    parser.add_argument("--max-loss-events", type=int, default=DEFAULT_MAX_LOSS_EVENTS)
    parser.add_argument("--initial-target-width", type=float,
                        default=DEFAULT_INITIAL_TARGET_WIDTH)
    parser.add_argument("--desired-target-width", type=float,
                        default=DEFAULT_DESIRED_TARGET_WIDTH)
    parser.add_argument("--max-final-width-error", type=float,
                        default=DEFAULT_MAX_FINAL_WIDTH_ERROR)
    parser.add_argument("--min-width-error-reduction", type=float,
                        default=DEFAULT_MIN_WIDTH_ERROR_REDUCTION)
    parser.add_argument("--max-yaw-axis", type=float, default=DEFAULT_MAX_YAW_AXIS)
    parser.add_argument("--max-pitch-axis", type=float, default=DEFAULT_MAX_PITCH_AXIS)
    parser.add_argument("--real-duration", type=float, default=DEFAULT_REAL_DURATION_S)
    parser.add_argument("--real-min-found-ratio", type=float,
                        default=DEFAULT_REAL_MIN_FOUND_RATIO)
    parser.add_argument("--real-max-loss-events", type=int,
                        default=DEFAULT_REAL_MAX_LOSS_EVENTS)
    parser.add_argument("--real-max-final-width-error", type=float,
                        default=DEFAULT_REAL_MAX_FINAL_WIDTH_ERROR)
    parser.add_argument("--real-min-width-error-reduction", type=float,
                        default=DEFAULT_REAL_MIN_WIDTH_ERROR_REDUCTION)
    parser.add_argument("--real-min-initial-width-error", type=float,
                        default=DEFAULT_REAL_MIN_INITIAL_WIDTH_ERROR)
    parser.add_argument("--real-max-pitch-axis", type=float,
                        default=DEFAULT_REAL_MAX_PITCH_AXIS)
    parser.add_argument("--real-min-pitch-axis", type=float,
                        default=DEFAULT_REAL_MIN_PITCH_AXIS)
    parser.add_argument("--real-tracking-report", type=Path)
    parser.add_argument("--ack-real-approach-target", action="store_true",
                        help="Confirm the supplied report used a valid approach target scenario")
    args = parser.parse_args(argv)
    if args.duration <= 0:
        parser.error("--duration must be positive")
    if args.hz <= 0:
        parser.error("--hz must be positive")
    if not 0 <= args.min_found_ratio <= 1:
        parser.error("--min-found-ratio must be in [0, 1]")
    if args.max_loss_events < 0:
        parser.error("--max-loss-events must be non-negative")
    if args.initial_target_width <= 0:
        parser.error("--initial-target-width must be positive")
    if args.desired_target_width <= 0:
        parser.error("--desired-target-width must be positive")
    if args.max_final_width_error < 0:
        parser.error("--max-final-width-error must be non-negative")
    if not 0 <= args.min_width_error_reduction <= 1:
        parser.error("--min-width-error-reduction must be in [0, 1]")
    if args.max_yaw_axis < 0 or args.max_pitch_axis < 0:
        parser.error("--max-yaw-axis/--max-pitch-axis must be non-negative")
    if args.real_duration <= 0:
        parser.error("--real-duration must be positive")
    if not 0 <= args.real_min_found_ratio <= 1:
        parser.error("--real-min-found-ratio must be in [0, 1]")
    if args.real_max_loss_events < 0:
        parser.error("--real-max-loss-events must be non-negative")
    if args.real_max_final_width_error < 0:
        parser.error("--real-max-final-width-error must be non-negative")
    if not 0 <= args.real_min_width_error_reduction <= 1:
        parser.error("--real-min-width-error-reduction must be in [0, 1]")
    if args.real_min_initial_width_error < 0:
        parser.error("--real-min-initial-width-error must be non-negative")
    if args.real_max_pitch_axis < 0 or args.real_min_pitch_axis < 0:
        parser.error("--real pitch axis limits must be non-negative")
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
        initial_target_width=args.initial_target_width,
        desired_target_width=args.desired_target_width,
        max_final_width_error=args.max_final_width_error,
        min_width_error_reduction=args.min_width_error_reduction,
        max_yaw_axis=args.max_yaw_axis,
        max_pitch_axis=args.max_pitch_axis,
        real_duration_s=args.real_duration,
        real_min_found_ratio=args.real_min_found_ratio,
        real_max_loss_events=args.real_max_loss_events,
        real_max_final_width_error=args.real_max_final_width_error,
        real_min_width_error_reduction=args.real_min_width_error_reduction,
        real_min_initial_width_error=args.real_min_initial_width_error,
        real_max_pitch_axis=args.real_max_pitch_axis,
        real_min_pitch_axis=args.real_min_pitch_axis,
        real_tracking_report=args.real_tracking_report,
        ack_real_approach_target=args.ack_real_approach_target,
    )
    json_path, md_path = write_reports(report, args.log_dir)
    print("simitl-pr0p-approach-pitch %s report=%s summary=%s" % (
        report.status,
        json_path,
        md_path,
    ))
    print(report.next_action)
    return 1 if report.status == FAIL else 0


if __name__ == "__main__":
    raise SystemExit(main())
