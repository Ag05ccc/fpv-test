#!/usr/bin/env python3
"""Build guarded real pr0p airborne static-target yaw-only evidence."""

from __future__ import annotations

import argparse
import json
import shlex
import subprocess
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable

PROBE_DIR = Path(__file__).resolve().parent
if str(PROBE_DIR) not in sys.path:
    sys.path.insert(0, str(PROBE_DIR))

from pr0p_capture_probe import DEFAULT_LOG_DIR  # noqa: E402
from pr0p_decision_report import load_json_report  # noqa: E402
from pr0p_live_run_manifest import report_sort_key  # noqa: E402
from pr0p_moving_target_yaw_plan import (  # noqa: E402
    DEFAULT_REAL_DURATION_S,
    DEFAULT_REAL_MAX_LOSS_EVENTS,
    DEFAULT_REAL_MAX_YAW_AXIS,
    DEFAULT_REAL_MAX_FINAL_CENTER_ERROR_PX,
    DEFAULT_REAL_MIN_FOUND_RATIO,
    DEFAULT_REAL_MIN_CENTER_ERROR_REDUCTION_RATIO,
    DEFAULT_REAL_MIN_INITIAL_CENTER_ERROR_PX,
    DEFAULT_REAL_MIN_TARGET_MOTION_PX,
    evaluate_real_pr0p_yaw_evidence,
)
from pr0p_tracking_probe import load_bbox_file, parse_bbox  # noqa: E402


PASS = "PASS"
WAITING = "WAITING"
FAIL = "FAIL"
SKIPPED = "SKIPPED"
EXTENDED_FOLLOW_DURATION_S = 20.0
EXTENDED_FOLLOW_MIN_FOUND_RATIO = 0.95
EXTENDED_FOLLOW_MAX_LOSS_EVENTS = 0


@dataclass
class MovingTargetAcceptanceStep:
    name: str
    status: str
    summary: str
    command: str = ""
    report: str | None = None
    returncode: int | None = None
    stdout: str = ""
    stderr: str = ""
    notes: list[str] = field(default_factory=list)
    metrics: dict[str, Any] = field(default_factory=dict)

    def as_dict(self) -> dict[str, Any]:
        return {
            "name": self.name,
            "status": self.status,
            "summary": self.summary,
            "command": self.command,
            "report": self.report,
            "returncode": self.returncode,
            "stdout": self.stdout,
            "stderr": self.stderr,
            "notes": self.notes,
            "metrics": self.metrics,
        }


@dataclass
class MovingTargetAcceptanceReport:
    run_id: str
    started_at: str
    status: str
    summary: str
    next_action: str
    steps: list[MovingTargetAcceptanceStep] = field(default_factory=list)
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


def command_block(*parts: str) -> str:
    return " \\\n  ".join(parts)


def command_argv(command: str) -> list[str]:
    return shlex.split(command.replace("\\\n", " "))


def bbox_arg(bbox: tuple[int, int, int, int] | None) -> str:
    return ",".join(str(item) for item in bbox) if bbox else "x,y,w,h"


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


def step_by_name(report: dict[str, Any] | None, name: str) -> dict[str, Any] | None:
    if not report or not isinstance(report.get("steps"), list):
        return None
    for step in report["steps"]:
        if isinstance(step, dict) and step.get("name") == name:
            return step
    return None


def extended_follow_prerequisite_notes(report: dict[str, Any] | None) -> tuple[list[str], dict[str, Any]]:
    metrics = report.get("metrics") if isinstance(report, dict) and isinstance(report.get("metrics"), dict) else {}
    live_step = step_by_name(report, "pr0p_tracking_live")
    duration_s = metric_float(metrics, "duration_s")
    min_found_ratio = metric_float(metrics, "min_found_ratio")
    max_loss_events = metric_int(metrics, "max_loss_events")
    run_live_gates = metrics.get("run_live_gates") is True
    missing: list[str] = []
    if not isinstance(report, dict) or report.get("status") != PASS:
        missing.append("tracking_acceptance_pass")
    if not live_step or live_step.get("status") != PASS:
        missing.append("pr0p_tracking_live_pass")
    if duration_s is None or duration_s < EXTENDED_FOLLOW_DURATION_S:
        missing.append("extended_follow_duration_s")
    if min_found_ratio is None or min_found_ratio < EXTENDED_FOLLOW_MIN_FOUND_RATIO:
        missing.append("extended_follow_min_found_ratio")
    if max_loss_events is None or max_loss_events > EXTENDED_FOLLOW_MAX_LOSS_EVENTS:
        missing.append("extended_follow_max_loss_events")
    if not run_live_gates:
        missing.append("extended_follow_live_gates")
    return missing, {
        "duration_s": duration_s,
        "min_found_ratio": min_found_ratio,
        "max_loss_events": max_loss_events,
        "run_live_gates": run_live_gates,
    }


def parsed_status(stdout: str, *, returncode: int) -> str:
    first_line = stdout.splitlines()[0] if stdout.splitlines() else ""
    for token in first_line.split():
        if token in {PASS, WAITING, FAIL}:
            return token
    return PASS if returncode == 0 else FAIL


def parsed_report_path(stdout: str) -> str | None:
    for token in stdout.replace("\n", " ").split():
        if token.startswith("report="):
            return token.split("=", 1)[1]
    return None


def tracking_command(
    *,
    run_id: str,
    bbox: tuple[int, int, int, int] | None,
    duration_s: float,
    min_found_ratio: float,
    max_loss_events: int,
    max_abs_yaw_axis: float,
    min_center_error_reduction_ratio: float,
    max_final_center_error_px: float,
    takeoff_delay_frames: int,
    takeoff_boost_frames: int,
    settle_throttle: float,
) -> str:
    return command_block(
        "fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_tracking_acceptance_runner.py",
        "--bbox %s" % bbox_arg(bbox),
        "--run-live-gates",
        "--ack-live-input",
        "--arm-first",
        "--arm-throttle -0.4",
        "--takeoff-delay-frames %d" % takeoff_delay_frames,
        "--takeoff-boost-frames %d" % takeoff_boost_frames,
        "--settle-throttle %s" % settle_throttle,
        "--duration %s" % duration_s,
        "--min-found-ratio %s" % min_found_ratio,
        "--max-loss-events %s" % max_loss_events,
        "--max-abs-yaw-axis %s" % max_abs_yaw_axis,
        "--max-abs-pitch-axis 0.0",
        "--run-id %s-real-pr0p-moving-target-yaw" % run_id,
    )


def bbox_step(bbox: tuple[int, int, int, int] | None) -> MovingTargetAcceptanceStep:
    if bbox is None:
        return MovingTargetAcceptanceStep(
            name="moving_target_bbox",
            status=WAITING,
            summary="target bbox is missing",
            notes=["select a static FPV target and pass --bbox or --bbox-file"],
        )
    return MovingTargetAcceptanceStep(
        name="moving_target_bbox",
        status=PASS,
        summary="static target bbox is present",
        notes=["bbox=%s" % bbox_arg(bbox)],
    )


def extended_follow_prerequisite_step(log_dir: Path) -> MovingTargetAcceptanceStep:
    paths = sorted(log_dir.glob("*-tracking-acceptance.json"), key=report_sort_key)
    if not paths:
        return MovingTargetAcceptanceStep(
            name="extended_follow_prerequisite",
            status=WAITING,
            summary="extended follow evidence is missing",
            notes=["run extended_follow_live before moving-target yaw"],
        )
    latest_missing: tuple[Path, list[str], dict[str, Any]] | None = None
    for path in reversed(paths):
        report = load_json_report(path)
        missing, metrics = extended_follow_prerequisite_notes(report)
        if not missing:
            return MovingTargetAcceptanceStep(
                name="extended_follow_prerequisite",
                status=PASS,
                summary="extended follow evidence is PASS",
                report=str(path),
                metrics=metrics,
            )
        if latest_missing is None:
            latest_missing = (path, missing, metrics)

    path, missing, metrics = latest_missing
    return MovingTargetAcceptanceStep(
        name="extended_follow_prerequisite",
        status=WAITING,
        summary="extended follow evidence is incomplete",
        report=str(path),
        notes=missing,
        metrics=metrics,
    )


def run_external_command(
    command: str,
    *,
    command_runner: Callable[..., subprocess.CompletedProcess[str]],
    timeout_s: float,
) -> MovingTargetAcceptanceStep:
    try:
        completed = command_runner(
            command_argv(command),
            capture_output=True,
            text=True,
            timeout=timeout_s,
        )
    except Exception as exc:  # pragma: no cover - subprocess failures vary
        return MovingTargetAcceptanceStep(
            name="moving_target_tracking_probe",
            status=FAIL,
            summary="tracking command could not be executed",
            command=command,
            notes=[str(exc)],
        )
    status = parsed_status(completed.stdout or "", returncode=completed.returncode)
    if completed.returncode != 0 and status != WAITING:
        status = FAIL
    return MovingTargetAcceptanceStep(
        name="moving_target_tracking_probe",
        status=status,
        summary="tracking command returned %s" % status,
        command=command,
        report=parsed_report_path(completed.stdout or ""),
        returncode=completed.returncode,
        stdout=completed.stdout or "",
        stderr=completed.stderr or "",
    )


def moving_step_to_acceptance_step(step) -> MovingTargetAcceptanceStep:
    return MovingTargetAcceptanceStep(
        name="real_pr0p_moving_target_yaw",
        status=step.status,
        summary=step.summary,
        command=step.command,
        report=step.report,
        notes=step.notes,
        metrics=step.metrics,
    )


def build_moving_target_acceptance(
    *,
    run_id: str,
    log_dir: Path,
    bbox: tuple[int, int, int, int] | None,
    run_live_gate: bool,
    ack_live_input: bool,
    ack_real_moving_target: bool,
    tracking_report: Path | None,
    real_duration_s: float,
    real_min_found_ratio: float,
    real_max_loss_events: int,
    real_min_target_motion_px: float,
    real_max_yaw_axis: float,
    real_min_center_error_reduction_ratio: float,
    real_max_final_center_error_px: float,
    real_min_initial_center_error_px: float,
    takeoff_delay_frames: int,
    takeoff_boost_frames: int,
    settle_throttle: float,
    timeout_s: float,
    command_runner: Callable[..., subprocess.CompletedProcess[str]] = subprocess.run,
) -> MovingTargetAcceptanceReport:
    command = tracking_command(
        run_id=run_id,
        bbox=bbox,
        duration_s=real_duration_s,
        min_found_ratio=real_min_found_ratio,
        max_loss_events=real_max_loss_events,
        max_abs_yaw_axis=real_max_yaw_axis,
        min_center_error_reduction_ratio=real_min_center_error_reduction_ratio,
        max_final_center_error_px=real_max_final_center_error_px,
        takeoff_delay_frames=takeoff_delay_frames,
        takeoff_boost_frames=takeoff_boost_frames,
        settle_throttle=settle_throttle,
    )
    steps = [bbox_step(bbox), extended_follow_prerequisite_step(log_dir)]
    if run_live_gate:
        if not all(step.status == PASS for step in steps):
            command_step = MovingTargetAcceptanceStep(
                name="moving_target_tracking_probe",
                status=SKIPPED,
                summary="skipped because bbox or extended-follow prerequisite is not PASS",
                command=command,
                notes=["MOVING_TARGET_PREREQUISITES_NOT_PASS"],
            )
        else:
            command_step = run_external_command(
                command,
                command_runner=command_runner,
                timeout_s=timeout_s,
            )
            if command_step.report:
                tracking_report = Path(command_step.report)
    elif tracking_report is not None:
        command_step = MovingTargetAcceptanceStep(
            name="moving_target_tracking_probe",
            status=PASS,
            summary="using supplied airborne static-target tracking report",
            command=command,
            report=str(tracking_report),
            notes=["supplied report must come from a static target while the vehicle moves"],
        )
    else:
        command_step = MovingTargetAcceptanceStep(
            name="moving_target_tracking_probe",
            status=WAITING,
            summary="requires --run-live-gate --ack-live-input --ack-airborne-static-target",
            command=command,
            notes=["plan-only: no real input is sent"],
        )
    steps.append(command_step)

    real_step = evaluate_real_pr0p_yaw_evidence(
        command=command,
        real_tracking_report=tracking_report,
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
    steps.append(moving_step_to_acceptance_step(real_step))

    if any(step.status == FAIL for step in steps):
        status = FAIL
        summary = "real airborne static-target yaw evidence failed"
        next_action = "Inspect the failed airborne static-target acceptance step before refreshing moving_target_yaw_plan."
    elif any(step.status != PASS for step in steps):
        status = WAITING
        summary = "real airborne static-target yaw evidence is waiting"
        next_action = "After extended_follow PASS, run this with --run-live-gate --ack-live-input --ack-airborne-static-target or pass a captured --tracking-report."
    else:
        status = PASS
        summary = "real airborne static-target yaw evidence is ready for moving_target_yaw_plan"
        next_action = "Refresh pr0p_moving_target_yaw_plan.py with this tracking report as --real-tracking-report and --ack-airborne-static-target."

    return MovingTargetAcceptanceReport(
        run_id=run_id,
        started_at=time.strftime("%Y-%m-%dT%H:%M:%S%z", time.localtime()),
        status=status,
        summary=summary,
        next_action=next_action,
        steps=steps,
        commands={"real_pr0p_moving_target_yaw_live": command},
        metrics={
            "bbox": list(bbox) if bbox else None,
            "log_dir": str(log_dir),
            "run_live_gate": run_live_gate,
            "ack_live_input": ack_live_input,
            "ack_real_moving_target": ack_real_moving_target,
            "tracking_report": str(tracking_report) if tracking_report else None,
            "real_min_center_error_reduction_ratio": real_min_center_error_reduction_ratio,
            "real_max_final_center_error_px": real_max_final_center_error_px,
            "real_min_initial_center_error_px": real_min_initial_center_error_px,
            "takeoff_delay_frames": takeoff_delay_frames,
            "takeoff_boost_frames": takeoff_boost_frames,
            "settle_throttle": settle_throttle,
            "real_input_sent": False,
            "evidence_only": True,
            "real_status": real_step.status,
        },
    )


def build_markdown(report: MovingTargetAcceptanceReport) -> str:
    lines = [
        "# pr0p Real Airborne Static Target Yaw Acceptance Runner",
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


def write_reports(report: MovingTargetAcceptanceReport, log_dir: Path) -> tuple[Path, Path]:
    log_dir.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime("%Y%m%d-%H%M%S", time.localtime())
    json_path = log_dir / ("%s-%s-moving-target-acceptance.json" % (stamp, report.run_id))
    md_path = log_dir / ("%s-%s-moving-target-acceptance.md" % (stamp, report.run_id))
    json_path.write_text(json.dumps(report.as_dict(), indent=2, sort_keys=True) + "\n",
                         encoding="utf-8")
    md_path.write_text(build_markdown(report), encoding="utf-8")
    return json_path, md_path


def resolve_bbox_argument(
    *,
    bbox: tuple[int, int, int, int] | None,
    bbox_file: Path | None,
    parser: argparse.ArgumentParser,
) -> tuple[int, int, int, int] | None:
    if bbox and bbox_file:
        parser.error("use either --bbox or --bbox-file, not both")
    if not bbox_file:
        return bbox
    try:
        return load_bbox_file(bbox_file)
    except Exception as exc:
        parser.error(str(exc))
    return None


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-id", default="pr0p-moving-target-acceptance")
    parser.add_argument("--log-dir", type=Path, default=DEFAULT_LOG_DIR)
    parser.add_argument("--bbox", type=parse_bbox)
    parser.add_argument("--bbox-file", type=Path)
    parser.add_argument("--run-live-gate", action="store_true")
    parser.add_argument("--ack-live-input", action="store_true")
    parser.add_argument("--ack-airborne-static-target", action="store_true")
    parser.add_argument("--ack-real-moving-target", action="store_true")
    parser.add_argument("--tracking-report", type=Path)
    parser.add_argument("--real-duration", type=float, default=DEFAULT_REAL_DURATION_S)
    parser.add_argument("--real-min-found-ratio", type=float,
                        default=DEFAULT_REAL_MIN_FOUND_RATIO)
    parser.add_argument("--real-max-loss-events", type=int,
                        default=DEFAULT_REAL_MAX_LOSS_EVENTS)
    parser.add_argument("--real-min-image-motion", "--real-min-target-motion",
                        dest="real_min_image_motion", type=float,
                        default=DEFAULT_REAL_MIN_TARGET_MOTION_PX)
    parser.add_argument("--real-max-yaw-axis", type=float,
                        default=DEFAULT_REAL_MAX_YAW_AXIS)
    parser.add_argument("--real-min-center-error-reduction", type=float,
                        default=DEFAULT_REAL_MIN_CENTER_ERROR_REDUCTION_RATIO)
    parser.add_argument("--real-max-final-center-error", type=float,
                        default=DEFAULT_REAL_MAX_FINAL_CENTER_ERROR_PX)
    parser.add_argument("--real-min-initial-center-error", type=float,
                        default=DEFAULT_REAL_MIN_INITIAL_CENTER_ERROR_PX)
    parser.add_argument("--takeoff-delay-frames", type=int, default=10)
    parser.add_argument("--takeoff-boost-frames", type=int, default=10)
    parser.add_argument("--settle-throttle", type=float, default=-0.26)
    parser.add_argument("--timeout", type=float, default=300.0)
    args = parser.parse_args(argv)
    args.ack_airborne_static_target = (
        args.ack_airborne_static_target or args.ack_real_moving_target
    )
    if args.run_live_gate and not args.ack_live_input:
        parser.error("--run-live-gate requires --ack-live-input")
    if args.run_live_gate and not args.ack_airborne_static_target:
        parser.error("--run-live-gate requires --ack-airborne-static-target")
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
    if args.takeoff_delay_frames < 0:
        parser.error("--takeoff-delay-frames must be non-negative")
    if args.takeoff_boost_frames < 0:
        parser.error("--takeoff-boost-frames must be non-negative")
    if not -1.0 <= args.settle_throttle <= 1.0:
        parser.error("--settle-throttle must be within [-1.0, 1.0]")
    if args.timeout <= 0:
        parser.error("--timeout must be positive")
    args.bbox = resolve_bbox_argument(
        bbox=args.bbox,
        bbox_file=args.bbox_file,
        parser=parser,
    )
    return args


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    report = build_moving_target_acceptance(
        run_id=args.run_id,
        log_dir=args.log_dir,
        bbox=args.bbox,
        run_live_gate=args.run_live_gate,
        ack_live_input=args.ack_live_input,
        ack_real_moving_target=args.ack_airborne_static_target,
        tracking_report=args.tracking_report,
        real_duration_s=args.real_duration,
        real_min_found_ratio=args.real_min_found_ratio,
        real_max_loss_events=args.real_max_loss_events,
        real_min_target_motion_px=args.real_min_image_motion,
        real_max_yaw_axis=args.real_max_yaw_axis,
        real_min_center_error_reduction_ratio=args.real_min_center_error_reduction,
        real_max_final_center_error_px=args.real_max_final_center_error,
        real_min_initial_center_error_px=args.real_min_initial_center_error,
        takeoff_delay_frames=args.takeoff_delay_frames,
        takeoff_boost_frames=args.takeoff_boost_frames,
        settle_throttle=args.settle_throttle,
        timeout_s=args.timeout,
    )
    json_path, md_path = write_reports(report, args.log_dir)
    print("simitl-pr0p-moving-target-acceptance %s report=%s summary=%s" % (
        report.status,
        json_path,
        md_path,
    ))
    print(report.next_action)
    return 1 if report.status == FAIL else 0


if __name__ == "__main__":
    raise SystemExit(main())
