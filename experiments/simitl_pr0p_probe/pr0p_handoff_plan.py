#!/usr/bin/env python3
"""Plan the manual-to-autonomous handoff phase after approach/pitch passes."""

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

from game_dynamics_loop import GameHandoffDynamicsConfig, run_game_handoff_dynamics_loop  # noqa: E402
from pr0p_capture_probe import DEFAULT_LOG_DIR  # noqa: E402
from pr0p_tracking_probe import load_bbox_file  # noqa: E402
from screen_tracking_loop import parse_bbox  # noqa: E402
from sitl_log import JsonlLogger, make_log_path  # noqa: E402


PASS = "PASS"
WAITING = "WAITING"
FAIL = "FAIL"
DEFAULT_SYNTHETIC_DURATION_S = 6.0
DEFAULT_SYNTHETIC_HZ = 20.0
DEFAULT_MANUAL_DURATION_S = 1.6
DEFAULT_MIN_FOUND_RATIO = 0.90
DEFAULT_MAX_LOSS_EVENTS = 0
DEFAULT_INITIAL_TARGET_WIDTH = 70.0
DEFAULT_DESIRED_TARGET_WIDTH = 120.0
DEFAULT_MAX_HANDOFF_ERROR = 45.0
DEFAULT_MAX_FINAL_ERROR = 35.0
DEFAULT_MAX_FINAL_WIDTH_ERROR = 8.0
DEFAULT_MIN_ERROR_REDUCTION = 0.65
DEFAULT_MIN_MANUAL_TO_FINAL_REDUCTION = 0.80
DEFAULT_MIN_WIDTH_ERROR_REDUCTION = 0.65
DEFAULT_MAX_YAW_AXIS = 0.8
DEFAULT_MAX_PITCH_AXIS = 0.8
DEFAULT_REAL_MIN_FOUND_RATIO = 0.90
DEFAULT_REAL_MAX_LOSS_EVENTS = 2
DEFAULT_REAL_MAX_HANDOFF_ERROR = 55.0
DEFAULT_REAL_MAX_FINAL_ERROR = 45.0
DEFAULT_REAL_MAX_FINAL_WIDTH_ERROR = 15.0
DEFAULT_REAL_MIN_MANUAL_TO_FINAL_REDUCTION = 0.50
DEFAULT_REAL_MIN_WIDTH_ERROR_REDUCTION = 0.45
DEFAULT_REAL_MAX_YAW_AXIS = 0.8
DEFAULT_REAL_MAX_PITCH_AXIS = 0.8


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
class HandoffStep:
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
class HandoffReport:
    run_id: str
    started_at: str
    status: str
    summary: str
    next_action: str
    steps: list[HandoffStep] = field(default_factory=list)
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


def synthetic_handoff_command(
    *,
    run_id: str,
    duration_s: float,
    hz: float,
    manual_duration_s: float,
    max_handoff_error: float,
    max_final_error: float,
    max_final_width_error: float,
) -> str:
    return command_block(
        "fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_handoff_plan.py",
        "--execute-synthetic",
        "--duration %s" % duration_s,
        "--hz %s" % hz,
        "--manual-duration %s" % manual_duration_s,
        "--max-handoff-error %s" % max_handoff_error,
        "--max-final-error %s" % max_final_error,
        "--max-final-width-error %s" % max_final_width_error,
        "--run-id %s-synthetic-handoff" % run_id,
    )


def real_handoff_command(
    *,
    run_id: str,
    bbox: tuple[int, int, int, int] | None,
) -> str:
    return command_block(
        "fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_handoff_acceptance_runner.py",
        "--bbox %s" % bbox_arg(bbox),
        "--run-live-gate",
        "--ack-live-input",
        "--ack-real-handoff",
        "--run-id %s-real-pr0p-handoff" % run_id,
    )


def real_handoff_refresh_command(
    *,
    run_id: str,
    bbox: tuple[int, int, int, int] | None,
) -> str:
    return command_block(
        "fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_handoff_plan.py",
        "--bbox %s" % bbox_arg(bbox),
        "--execute-synthetic",
        "--real-handoff-report path/to/real-handoff-report.json",
        "--ack-real-handoff",
        "--run-id %s-real-pr0p-handoff-refresh" % run_id,
    )


def run_synthetic_handoff_regression(
    *,
    run_id: str,
    log_dir: Path,
    duration_s: float,
    hz: float,
    manual_duration_s: float,
    min_found_ratio: float,
    max_loss_events: int,
    initial_target_width: float,
    desired_target_width: float,
    max_handoff_error: float,
    max_final_error: float,
    max_final_width_error: float,
    min_error_reduction: float,
    min_manual_to_final_reduction: float,
    min_width_error_reduction: float,
    max_yaw_axis: float,
    max_pitch_axis: float,
) -> HandoffStep:
    log_path = make_log_path(log_dir, "%s-handoff-synthetic" % run_id)
    config = GameHandoffDynamicsConfig(
        duration_s=duration_s,
        hz=hz,
        manual_duration_s=manual_duration_s,
        initial_offset_x=180.0,
        initial_target_width=initial_target_width,
        desired_target_width=desired_target_width,
        min_found_ratio=min_found_ratio,
        max_loss_events=max_loss_events,
        max_handoff_abs_error_px=max_handoff_error,
        max_final_abs_error_px=max_final_error,
        max_final_abs_width_error_px=max_final_width_error,
        min_error_reduction_ratio=min_error_reduction,
        min_manual_to_final_error_reduction_ratio=min_manual_to_final_reduction,
        min_width_error_reduction_ratio=min_width_error_reduction,
        max_yaw_axis=max_yaw_axis,
        max_pitch_axis=max_pitch_axis,
        run_id=run_id,
        log_dir=log_dir,
    )
    with JsonlLogger(log_path, metadata={
        "tool": "pr0p_handoff_plan",
        "phase": "synthetic_manual_to_auto_handoff",
        "run_id": run_id,
        "source": "in-process-handoff-dynamics",
        "real_input": False,
        "manual_to_auto_handoff": True,
    }) as logger:
        result = run_game_handoff_dynamics_loop(config, logger=logger)
        logger.write("pr0p_handoff_synthetic_summary", **result.metrics)
    metrics = {
        **result.metrics,
        "log_path": str(log_path),
        "real_input": False,
        "manual_to_auto_handoff": True,
    }
    return HandoffStep(
        name="synthetic_manual_to_auto_handoff",
        status=result.status,
        summary=result.summary,
        report=str(log_path),
        metrics=metrics,
        notes=result.notes + ["dry-run only: no joystick command is sent"],
    )


def evaluate_real_handoff_evidence(
    *,
    command: str,
    real_handoff_report: Path | None,
    ack_real_handoff: bool,
    min_found_ratio: float,
    max_loss_events: int,
    max_handoff_error: float,
    max_final_error: float,
    max_final_width_error: float,
    min_manual_to_final_reduction: float,
    min_width_error_reduction: float,
    max_yaw_axis: float,
    max_pitch_axis: float,
    bbox: tuple[int, int, int, int] | None,
) -> HandoffStep:
    base_metrics = {
        "bbox": list(bbox) if bbox else None,
        "real_handoff_report": str(real_handoff_report) if real_handoff_report else None,
        "ack_real_handoff": ack_real_handoff,
        "required_min_found_ratio": min_found_ratio,
        "required_max_loss_events": max_loss_events,
        "required_max_handoff_error": max_handoff_error,
        "required_max_final_error": max_final_error,
        "required_max_final_width_error": max_final_width_error,
        "required_min_manual_to_final_reduction": min_manual_to_final_reduction,
        "required_min_width_error_reduction": min_width_error_reduction,
        "required_max_yaw_axis": max_yaw_axis,
        "required_max_pitch_axis": max_pitch_axis,
        "real_input": True,
        "manual_to_auto_handoff": True,
    }
    if real_handoff_report is None:
        return HandoffStep(
            name="real_pr0p_manual_to_auto_handoff",
            status=WAITING,
            summary="real pr0p manual-to-autonomous handoff evidence is not available yet",
            command=command,
            metrics=base_metrics,
            notes=[
                "record a real handoff report: manual target centering, follow command, then autonomous tracking",
                "refresh with --real-handoff-report and --ack-real-handoff",
            ],
        )
    report = load_json_report(real_handoff_report)
    if report is None:
        return HandoffStep(
            name="real_pr0p_manual_to_auto_handoff",
            status=FAIL,
            summary="real pr0p handoff report could not be read",
            command=command,
            report=str(real_handoff_report),
            metrics=base_metrics,
            notes=["REAL_HANDOFF_REPORT_UNREADABLE"],
        )
    metrics = report.get("metrics") if isinstance(report.get("metrics"), dict) else {}
    found_ratio = metric_float(metrics, "found_ratio")
    loss_events = metric_int(metrics, "loss_events")
    manual_final_error = metric_float(metrics, "manual_final_abs_error_px")
    final_error = metric_float(metrics, "final_abs_error_px")
    final_width_error = metric_float(metrics, "final_abs_width_error_px")
    manual_to_final_reduction = metric_float(metrics, "manual_to_final_error_reduction_ratio")
    width_reduction = metric_float(metrics, "width_error_reduction_ratio")
    max_abs_yaw_axis = metric_float(metrics, "max_abs_yaw_axis")
    max_abs_pitch_axis = metric_float(metrics, "max_abs_pitch_axis")
    merged_metrics = {
        **base_metrics,
        **metrics,
        "handoff_status": str(report.get("status", WAITING)),
    }
    missing: list[str] = []
    failed: list[str] = []
    if not ack_real_handoff:
        missing.append("ack_real_handoff")
    if report.get("status") == FAIL:
        failed.append("real_handoff_failed")
    elif report.get("status") != PASS:
        missing.append("real_handoff_pass")
    if metrics.get("follow_command_sent") is not True:
        failed.append("follow_command_not_sent")
    if metrics.get("tracker_initialized_before_follow") is not False:
        failed.append("tracker_initialized_before_follow")
    if metrics.get("tracker_initialized_after_follow") is not True:
        failed.append("tracker_not_initialized_after_follow")
    if metrics.get("pre_handoff_auto_control_samples") != 0:
        failed.append("auto_control_before_follow")
    if metrics.get("manual_to_auto_handoff") is not True:
        failed.append("manual_to_auto_handoff_missing")
    if metrics.get("real_input_sent") is not True:
        failed.append("real_input_not_sent")
    if found_ratio is None or found_ratio < min_found_ratio:
        failed.append("found_ratio_low")
    if loss_events is None or loss_events > max_loss_events:
        failed.append("loss_events_high")
    if manual_final_error is None or manual_final_error > max_handoff_error:
        failed.append("handoff_error_high")
    if final_error is None or final_error > max_final_error:
        failed.append("final_error_high")
    if final_width_error is None or final_width_error > max_final_width_error:
        failed.append("final_width_error_high")
    if manual_to_final_reduction is None or manual_to_final_reduction < min_manual_to_final_reduction:
        failed.append("manual_to_final_reduction_low")
    if width_reduction is None or width_reduction < min_width_error_reduction:
        failed.append("width_reduction_low")
    if max_abs_yaw_axis is None or max_abs_yaw_axis > max_yaw_axis:
        failed.append("yaw_unbounded")
    if max_abs_pitch_axis is None or max_abs_pitch_axis > max_pitch_axis:
        failed.append("pitch_unbounded")
    if failed:
        return HandoffStep(
            name="real_pr0p_manual_to_auto_handoff",
            status=FAIL,
            summary="real pr0p handoff evidence failed: %s" % ", ".join(failed[:4]),
            command=command,
            report=str(real_handoff_report),
            metrics=merged_metrics,
            notes=failed + missing,
        )
    if missing:
        return HandoffStep(
            name="real_pr0p_manual_to_auto_handoff",
            status=WAITING,
            summary="real pr0p handoff evidence is incomplete: %s" % ", ".join(missing[:4]),
            command=command,
            report=str(real_handoff_report),
            metrics=merged_metrics,
            notes=missing,
        )
    return HandoffStep(
        name="real_pr0p_manual_to_auto_handoff",
        status=PASS,
        summary="real pr0p manual-to-autonomous handoff is proven",
        command=command,
        report=str(real_handoff_report),
        metrics=merged_metrics,
        notes=["manual-to-autonomous handoff evidence passed"],
    )


def build_plan_report(
    *,
    run_id: str,
    log_dir: Path,
    bbox: tuple[int, int, int, int] | None,
    execute_synthetic: bool,
    duration_s: float,
    hz: float,
    manual_duration_s: float,
    min_found_ratio: float,
    max_loss_events: int,
    initial_target_width: float,
    desired_target_width: float,
    max_handoff_error: float,
    max_final_error: float,
    max_final_width_error: float,
    min_error_reduction: float,
    min_manual_to_final_reduction: float,
    min_width_error_reduction: float,
    max_yaw_axis: float,
    max_pitch_axis: float,
    real_handoff_report: Path | None,
    ack_real_handoff: bool,
) -> HandoffReport:
    commands = {
        "synthetic_manual_to_auto_handoff": synthetic_handoff_command(
            run_id=run_id,
            duration_s=duration_s,
            hz=hz,
            manual_duration_s=manual_duration_s,
            max_handoff_error=max_handoff_error,
            max_final_error=max_final_error,
            max_final_width_error=max_final_width_error,
        ),
        "real_pr0p_handoff_live": real_handoff_command(run_id=run_id, bbox=bbox),
        "real_pr0p_handoff_refresh": real_handoff_refresh_command(run_id=run_id, bbox=bbox),
    }
    if execute_synthetic:
        synthetic_step = run_synthetic_handoff_regression(
            run_id=run_id,
            log_dir=log_dir,
            duration_s=duration_s,
            hz=hz,
            manual_duration_s=manual_duration_s,
            min_found_ratio=min_found_ratio,
            max_loss_events=max_loss_events,
            initial_target_width=initial_target_width,
            desired_target_width=desired_target_width,
            max_handoff_error=max_handoff_error,
            max_final_error=max_final_error,
            max_final_width_error=max_final_width_error,
            min_error_reduction=min_error_reduction,
            min_manual_to_final_reduction=min_manual_to_final_reduction,
            min_width_error_reduction=min_width_error_reduction,
            max_yaw_axis=max_yaw_axis,
            max_pitch_axis=max_pitch_axis,
        )
    else:
        synthetic_step = HandoffStep(
            name="synthetic_manual_to_auto_handoff",
            status=WAITING,
            summary="synthetic handoff regression has not been run",
            command=commands["synthetic_manual_to_auto_handoff"],
            notes=["rerun with --execute-synthetic; this sends no real input"],
            metrics={"real_input": False, "manual_to_auto_handoff": True},
        )
    real_step = evaluate_real_handoff_evidence(
        command=(
            commands["real_pr0p_handoff_live"]
            if real_handoff_report is None
            else commands["real_pr0p_handoff_refresh"]
        ),
        real_handoff_report=real_handoff_report,
        ack_real_handoff=ack_real_handoff,
        min_found_ratio=DEFAULT_REAL_MIN_FOUND_RATIO,
        max_loss_events=DEFAULT_REAL_MAX_LOSS_EVENTS,
        max_handoff_error=DEFAULT_REAL_MAX_HANDOFF_ERROR,
        max_final_error=DEFAULT_REAL_MAX_FINAL_ERROR,
        max_final_width_error=DEFAULT_REAL_MAX_FINAL_WIDTH_ERROR,
        min_manual_to_final_reduction=DEFAULT_REAL_MIN_MANUAL_TO_FINAL_REDUCTION,
        min_width_error_reduction=DEFAULT_REAL_MIN_WIDTH_ERROR_REDUCTION,
        max_yaw_axis=DEFAULT_REAL_MAX_YAW_AXIS,
        max_pitch_axis=DEFAULT_REAL_MAX_PITCH_AXIS,
        bbox=bbox,
    )
    steps = [synthetic_step, real_step]
    if synthetic_step.status == FAIL or real_step.status == FAIL:
        status = FAIL
        summary = "manual-to-autonomous handoff evidence failed"
        next_action = "Inspect the failed handoff step before using this flow as the operating mode."
    elif synthetic_step.status == PASS and real_step.status == PASS:
        status = PASS
        summary = "manual-to-autonomous handoff is proven"
        next_action = "The independent pr0p path has the full manual target-to-autonomous follow chain; repeat live validation before tuning speed."
    else:
        status = WAITING
        summary = "manual-to-autonomous handoff is planned; real pr0p handoff evidence is still missing"
        next_action = "Record a real handoff report after approach/pitch passes, then refresh with --real-handoff-report and --ack-real-handoff."
    return HandoffReport(
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
            "real_handoff_report": str(real_handoff_report) if real_handoff_report else None,
            "ack_real_handoff": ack_real_handoff,
            "evidence_only": True,
            "real_input_sent": False,
        },
    )


def build_markdown(report: HandoffReport) -> str:
    lines = [
        "# pr0p Manual To Autonomous Handoff Plan",
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


def write_reports(report: HandoffReport, log_dir: Path) -> tuple[Path, Path]:
    log_dir.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime("%Y%m%d-%H%M%S", time.localtime())
    json_path = log_dir / ("%s-%s-handoff.json" % (stamp, report.run_id))
    md_path = log_dir / ("%s-%s-handoff.md" % (stamp, report.run_id))
    json_path.write_text(json.dumps(report.as_dict(), indent=2, sort_keys=True) + "\n",
                         encoding="utf-8")
    md_path.write_text(build_markdown(report), encoding="utf-8")
    return json_path, md_path


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-id", default="pr0p-handoff")
    parser.add_argument("--log-dir", type=Path, default=DEFAULT_LOG_DIR)
    parser.add_argument("--bbox", type=parse_bbox)
    parser.add_argument("--bbox-file", type=Path)
    parser.add_argument("--execute-synthetic", action="store_true")
    parser.add_argument("--duration", type=float, default=DEFAULT_SYNTHETIC_DURATION_S)
    parser.add_argument("--hz", type=float, default=DEFAULT_SYNTHETIC_HZ)
    parser.add_argument("--manual-duration", type=float, default=DEFAULT_MANUAL_DURATION_S)
    parser.add_argument("--min-found-ratio", type=float, default=DEFAULT_MIN_FOUND_RATIO)
    parser.add_argument("--max-loss-events", type=int, default=DEFAULT_MAX_LOSS_EVENTS)
    parser.add_argument("--initial-target-width", type=float,
                        default=DEFAULT_INITIAL_TARGET_WIDTH)
    parser.add_argument("--desired-target-width", type=float,
                        default=DEFAULT_DESIRED_TARGET_WIDTH)
    parser.add_argument("--max-handoff-error", type=float,
                        default=DEFAULT_MAX_HANDOFF_ERROR)
    parser.add_argument("--max-final-error", type=float,
                        default=DEFAULT_MAX_FINAL_ERROR)
    parser.add_argument("--max-final-width-error", type=float,
                        default=DEFAULT_MAX_FINAL_WIDTH_ERROR)
    parser.add_argument("--min-error-reduction", type=float,
                        default=DEFAULT_MIN_ERROR_REDUCTION)
    parser.add_argument("--min-manual-to-final-reduction", type=float,
                        default=DEFAULT_MIN_MANUAL_TO_FINAL_REDUCTION)
    parser.add_argument("--min-width-error-reduction", type=float,
                        default=DEFAULT_MIN_WIDTH_ERROR_REDUCTION)
    parser.add_argument("--max-yaw-axis", type=float, default=DEFAULT_MAX_YAW_AXIS)
    parser.add_argument("--max-pitch-axis", type=float, default=DEFAULT_MAX_PITCH_AXIS)
    parser.add_argument("--real-handoff-report", type=Path)
    parser.add_argument("--ack-real-handoff", action="store_true",
                        help="Confirm the supplied report was captured from a real manual-to-autonomous handoff")
    args = parser.parse_args(argv)
    if args.duration <= 0:
        parser.error("--duration must be positive")
    if args.hz <= 0:
        parser.error("--hz must be positive")
    if args.manual_duration <= 0 or args.manual_duration >= args.duration:
        parser.error("--manual-duration must be positive and shorter than --duration")
    if not 0 <= args.min_found_ratio <= 1:
        parser.error("--min-found-ratio must be in [0, 1]")
    if args.max_loss_events < 0:
        parser.error("--max-loss-events must be non-negative")
    if args.initial_target_width <= 0 or args.desired_target_width <= 0:
        parser.error("--initial-target-width/--desired-target-width must be positive")
    for name in (
        "max_handoff_error",
        "max_final_error",
        "max_final_width_error",
        "max_yaw_axis",
        "max_pitch_axis",
    ):
        if getattr(args, name) < 0:
            parser.error("--%s must be non-negative" % name.replace("_", "-"))
    for name in (
        "min_error_reduction",
        "min_manual_to_final_reduction",
        "min_width_error_reduction",
    ):
        if not 0 <= getattr(args, name) <= 1:
            parser.error("--%s must be in [0, 1]" % name.replace("_", "-"))
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
        manual_duration_s=args.manual_duration,
        min_found_ratio=args.min_found_ratio,
        max_loss_events=args.max_loss_events,
        initial_target_width=args.initial_target_width,
        desired_target_width=args.desired_target_width,
        max_handoff_error=args.max_handoff_error,
        max_final_error=args.max_final_error,
        max_final_width_error=args.max_final_width_error,
        min_error_reduction=args.min_error_reduction,
        min_manual_to_final_reduction=args.min_manual_to_final_reduction,
        min_width_error_reduction=args.min_width_error_reduction,
        max_yaw_axis=args.max_yaw_axis,
        max_pitch_axis=args.max_pitch_axis,
        real_handoff_report=args.real_handoff_report,
        ack_real_handoff=args.ack_real_handoff,
    )
    json_path, md_path = write_reports(report, args.log_dir)
    print("simitl-pr0p-handoff %s report=%s summary=%s" % (
        report.status,
        json_path,
        md_path,
    ))
    print(report.next_action)
    return 1 if report.status == FAIL else 0


if __name__ == "__main__":
    raise SystemExit(main())
