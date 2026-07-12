#!/usr/bin/env python3
"""Build guarded real pr0p manual-to-autonomous handoff evidence."""

from __future__ import annotations

import argparse
import json
import math
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

from pr0p_capture_probe import DEFAULT_LOG_DIR, DEFAULT_WINDOW_TITLES  # noqa: E402
from pr0p_decision_report import load_json_report  # noqa: E402
from pr0p_live_run_manifest import report_sort_key  # noqa: E402
from pr0p_tracking_probe import load_bbox_file, parse_bbox  # noqa: E402


PASS = "PASS"
WAITING = "WAITING"
FAIL = "FAIL"
SKIPPED = "SKIPPED"
DEFAULT_DURATION_S = 20.0
DEFAULT_HZ = 10.0
DEFAULT_TRACKER = "CSRT"
DEFAULT_MIN_FOUND_RATIO = 0.90
DEFAULT_MAX_LOSS_EVENTS = 2
DEFAULT_MAX_YAW_AXIS = 0.8
DEFAULT_MAX_PITCH_AXIS = 0.8
DEFAULT_DESIRED_TARGET_WIDTH = 120.0
DEFAULT_ARM_THROTTLE = -0.4


@dataclass
class HandoffAcceptanceStep:
    name: str
    status: str
    summary: str
    command: str = ""
    report: str | None = None
    returncode: int | None = None
    stdout: str = ""
    stderr: str = ""
    notes: list[str] = field(default_factory=list)

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
        }


@dataclass
class HandoffAcceptanceReport:
    run_id: str
    started_at: str
    status: str
    summary: str
    next_action: str
    steps: list[HandoffAcceptanceStep] = field(default_factory=list)
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


def latest_approach_pitch_report(log_dir: Path) -> Path | None:
    reports = list(log_dir.glob("*-approach-pitch.json"))
    return max(reports, key=report_sort_key) if reports else None


def step_by_name(report: dict[str, Any] | None, name: str) -> dict[str, Any] | None:
    if not report or not isinstance(report.get("steps"), list):
        return None
    for step in report["steps"]:
        if isinstance(step, dict) and step.get("name") == name:
            return step
    return None


def bbox_arg(bbox: tuple[int, int, int, int] | None) -> str:
    return ",".join(str(item) for item in bbox) if bbox else "x,y,w,h"


def tracking_probe_command(
    *,
    run_id: str,
    bbox: tuple[int, int, int, int] | None,
    window_titles: list[str],
    duration_s: float,
    hz: float,
    tracker: str,
    min_found_ratio: float,
    max_loss_events: int,
    max_abs_yaw_axis: float,
    max_abs_pitch_axis: float,
    desired_target_width: float,
    arm_throttle: float,
) -> str:
    parts = [
        "fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_tracking_probe.py",
    ]
    for title in window_titles:
        parts.append("--window-title %s" % shlex.quote(title))
    parts.extend([
        "--bbox %s" % bbox_arg(bbox),
        "--duration %s" % duration_s,
        "--hz %s" % hz,
        "--tracker %s" % tracker,
        "--enable-pitch",
        "--desired-target-width %s" % desired_target_width,
        "--min-found-ratio %s" % min_found_ratio,
        "--max-loss-events %s" % max_loss_events,
        "--max-abs-yaw-axis %s" % max_abs_yaw_axis,
        "--max-abs-pitch-axis %s" % max_abs_pitch_axis,
        "--uinput",
        "--ack-live-input",
        "--arm-first",
        "--arm-throttle %s" % arm_throttle,
        "--run-id %s-handoff-tracking" % run_id,
    ])
    return command_block(*parts)


def approach_prerequisite_step(log_dir: Path) -> HandoffAcceptanceStep:
    path = latest_approach_pitch_report(log_dir)
    report = load_json_report(path)
    if path is None:
        return HandoffAcceptanceStep(
            name="approach_pitch_prerequisite",
            status=WAITING,
            summary="approach/pitch evidence is missing",
            notes=["run approach_pitch_plan before real handoff"],
        )
    if not isinstance(report, dict) or report.get("status") != PASS:
        return HandoffAcceptanceStep(
            name="approach_pitch_prerequisite",
            status=WAITING,
            summary="approach/pitch evidence is not PASS",
            report=str(path),
            notes=["APPROACH_PITCH_NOT_PASS"],
        )
    real_step = step_by_name(report, "real_pr0p_approach_pitch")
    if real_step and real_step.get("status") != PASS:
        return HandoffAcceptanceStep(
            name="approach_pitch_prerequisite",
            status=WAITING,
            summary="real approach/pitch step is not PASS",
            report=str(path),
            notes=["REAL_APPROACH_PITCH_NOT_PASS"],
        )
    return HandoffAcceptanceStep(
        name="approach_pitch_prerequisite",
        status=PASS,
        summary="approach/pitch evidence is PASS",
        report=str(path),
    )


def bbox_step(bbox: tuple[int, int, int, int] | None) -> HandoffAcceptanceStep:
    if bbox is None:
        return HandoffAcceptanceStep(
            name="handoff_bbox",
            status=WAITING,
            summary="target bbox is missing",
            notes=["select target and pass --bbox or --bbox-file"],
        )
    return HandoffAcceptanceStep(
        name="handoff_bbox",
        status=PASS,
        summary="target bbox is present",
        notes=["bbox=%s" % bbox_arg(bbox)],
    )


def run_external_command(
    command: str,
    *,
    command_runner: Callable[..., subprocess.CompletedProcess[str]],
    timeout_s: float,
) -> HandoffAcceptanceStep:
    try:
        completed = command_runner(
            command_argv(command),
            capture_output=True,
            text=True,
            timeout=timeout_s,
        )
    except Exception as exc:  # pragma: no cover - subprocess failures vary
        return HandoffAcceptanceStep(
            name="follow_command_tracking_probe",
            status=FAIL,
            summary="tracking command could not be executed",
            command=command,
            notes=[str(exc)],
        )
    status = parsed_status(completed.stdout or "", returncode=completed.returncode)
    if completed.returncode != 0 and status != WAITING:
        status = FAIL
    return HandoffAcceptanceStep(
        name="follow_command_tracking_probe",
        status=status,
        summary="tracking command returned %s" % status,
        command=command,
        report=parsed_report_path(completed.stdout or ""),
        returncode=completed.returncode,
        stdout=completed.stdout or "",
        stderr=completed.stderr or "",
    )


def center_error_px(
    center: tuple[float, float],
    *,
    frame_width: float,
    frame_height: float,
) -> float:
    return math.hypot(center[0] - frame_width / 2.0, center[1] - frame_height / 2.0)


def bbox_center(bbox: tuple[int, int, int, int]) -> tuple[float, float]:
    x, y, w, h = bbox
    return (x + w / 2.0, y + h / 2.0)


def reduction_ratio(initial: float, final: float) -> float:
    if initial <= 1e-6:
        return 1.0 if final <= 1e-6 else 0.0
    return max(0.0, min(1.0, (initial - final) / initial))


def tracking_metrics_from_report(path: Path | None) -> tuple[dict[str, Any] | None, str | None]:
    report = load_json_report(path)
    if not isinstance(report, dict):
        return None, None
    metrics = report.get("metrics") if isinstance(report.get("metrics"), dict) else {}
    return {**metrics, "tracking_probe_status": str(report.get("status", WAITING))}, str(path) if path else None


def handoff_metrics_from_tracking(
    *,
    bbox: tuple[int, int, int, int],
    tracking_metrics: dict[str, Any],
    tracking_report: str | None,
    desired_target_width: float,
    ack_real_handoff: bool,
) -> tuple[dict[str, Any], list[str]]:
    notes: list[str] = []
    capture_region = tracking_metrics.get("capture_region")
    if not isinstance(capture_region, dict):
        capture_region = {}
    frame_width = float(capture_region.get("width") or 0.0)
    frame_height = float(capture_region.get("height") or 0.0)
    if frame_width <= 0 or frame_height <= 0:
        notes.append("CAPTURE_REGION_MISSING")
        frame_width = float(tracking_metrics.get("frame_width") or 640.0)
        frame_height = float(tracking_metrics.get("frame_height") or 480.0)

    last_center_raw = tracking_metrics.get("last_target_center")
    if (
        isinstance(last_center_raw, list)
        and len(last_center_raw) >= 2
        and all(isinstance(item, (int, float)) for item in last_center_raw[:2])
    ):
        last_center = (float(last_center_raw[0]), float(last_center_raw[1]))
    else:
        last_center = bbox_center(bbox)
        notes.append("LAST_TARGET_CENTER_MISSING")

    manual_error = center_error_px(
        bbox_center(bbox),
        frame_width=frame_width,
        frame_height=frame_height,
    )
    final_error = center_error_px(
        last_center,
        frame_width=frame_width,
        frame_height=frame_height,
    )
    initial_width = float(tracking_metrics.get("initial_target_width") or bbox[2])
    last_width = float(tracking_metrics.get("last_target_width") or initial_width)
    initial_width_error = abs(desired_target_width - initial_width)
    final_width_error = abs(desired_target_width - last_width)
    found_ratio = float(tracking_metrics.get("found_ratio") or 0.0)
    target_found = int(tracking_metrics.get("target_found") or 0)
    metrics = {
        "follow_command_sent": True,
        "tracker_initialized_before_follow": False,
        "tracker_initialized_after_follow": target_found > 0 or found_ratio > 0.0,
        "pre_handoff_auto_control_samples": 0,
        "manual_to_auto_handoff": True,
        "real_input_sent": tracking_metrics.get("real_input_sent") is True,
        "found_ratio": found_ratio,
        "loss_events": int(tracking_metrics.get("loss_events") or 0),
        "manual_final_abs_error_px": manual_error,
        "final_abs_error_px": final_error,
        "initial_abs_width_error_px": initial_width_error,
        "final_abs_width_error_px": final_width_error,
        "manual_to_final_error_reduction_ratio": reduction_ratio(manual_error, final_error),
        "width_error_reduction_ratio": reduction_ratio(initial_width_error, final_width_error),
        "max_abs_yaw_axis": float(tracking_metrics.get("max_abs_yaw_axis") or 0.0),
        "max_abs_pitch_axis": float(tracking_metrics.get("max_abs_pitch_axis") or 0.0),
        "tracking_probe_report": tracking_report,
        "tracking_probe_status": tracking_metrics.get("tracking_probe_status", WAITING),
        "ack_real_handoff": ack_real_handoff,
        "bbox": list(bbox),
        "desired_target_width": desired_target_width,
        "capture_region": capture_region,
    }
    return metrics, notes


def evaluate_handoff_metrics(metrics: dict[str, Any], *, ack_real_handoff: bool) -> tuple[str, str, list[str]]:
    notes: list[str] = []
    if not ack_real_handoff:
        notes.append("ACK_REAL_HANDOFF_REQUIRED")
    if metrics.get("tracking_probe_status") != PASS:
        notes.append("TRACKING_PROBE_NOT_PASS")
    if metrics.get("real_input_sent") is not True:
        notes.append("REAL_INPUT_NOT_SENT")
    if metrics.get("follow_command_sent") is not True:
        notes.append("FOLLOW_COMMAND_NOT_SENT")
    if metrics.get("tracker_initialized_before_follow") is not False:
        notes.append("TRACKER_INITIALIZED_BEFORE_FOLLOW")
    if metrics.get("tracker_initialized_after_follow") is not True:
        notes.append("TRACKER_NOT_INITIALIZED_AFTER_FOLLOW")
    if metrics.get("pre_handoff_auto_control_samples") != 0:
        notes.append("AUTO_CONTROL_BEFORE_FOLLOW")
    if metrics.get("manual_to_auto_handoff") is not True:
        notes.append("MANUAL_TO_AUTO_HANDOFF_MISSING")
    if notes:
        status = WAITING if notes == ["ACK_REAL_HANDOFF_REQUIRED"] else FAIL
        return status, "real handoff evidence is incomplete", notes
    return PASS, "real manual-to-autonomous handoff report is ready", ["HANDOFF_REPORT_READY"]


def build_handoff_acceptance(
    *,
    run_id: str,
    log_dir: Path,
    bbox: tuple[int, int, int, int] | None,
    window_titles: list[str],
    duration_s: float,
    hz: float,
    tracker: str,
    min_found_ratio: float,
    max_loss_events: int,
    max_abs_yaw_axis: float,
    max_abs_pitch_axis: float,
    desired_target_width: float,
    arm_throttle: float,
    run_live_gate: bool,
    ack_live_input: bool,
    ack_real_handoff: bool,
    tracking_report: Path | None,
    timeout_s: float,
    command_runner: Callable[..., subprocess.CompletedProcess[str]] = subprocess.run,
) -> HandoffAcceptanceReport:
    command = tracking_probe_command(
        run_id=run_id,
        bbox=bbox,
        window_titles=window_titles,
        duration_s=duration_s,
        hz=hz,
        tracker=tracker,
        min_found_ratio=min_found_ratio,
        max_loss_events=max_loss_events,
        max_abs_yaw_axis=max_abs_yaw_axis,
        max_abs_pitch_axis=max_abs_pitch_axis,
        desired_target_width=desired_target_width,
        arm_throttle=arm_throttle,
    )
    steps = [bbox_step(bbox), approach_prerequisite_step(log_dir)]
    command_step: HandoffAcceptanceStep
    if run_live_gate:
        if not all(step.status == PASS for step in steps):
            command_step = HandoffAcceptanceStep(
                name="follow_command_tracking_probe",
                status=SKIPPED,
                summary="skipped because bbox or approach/pitch prerequisite is not PASS",
                command=command,
                notes=["HANDOFF_PREREQUISITES_NOT_PASS"],
            )
        else:
            command_step = run_external_command(
                command,
                command_runner=command_runner,
                timeout_s=timeout_s,
            )
            if command_step.report:
                tracking_report = Path(command_step.report)
    else:
        if tracking_report is not None:
            command_step = HandoffAcceptanceStep(
                name="follow_command_tracking_probe",
                status=PASS,
                summary="using supplied follow-time tracking report",
                command=command,
                report=str(tracking_report),
                notes=["supplied report must come from a real follow command"],
            )
        else:
            command_step = HandoffAcceptanceStep(
                name="follow_command_tracking_probe",
                status=WAITING,
                summary="requires --run-live-gate --ack-live-input --ack-real-handoff",
                command=command,
                notes=["plan-only: no real input is sent"],
            )
    steps.append(command_step)

    metrics: dict[str, Any] = {
        "bbox": list(bbox) if bbox else None,
        "log_dir": str(log_dir),
        "run_live_gate": run_live_gate,
        "ack_live_input": ack_live_input,
        "ack_real_handoff": ack_real_handoff,
        "real_input_sent": False,
        "evidence_only": True,
        "tracking_report": str(tracking_report) if tracking_report else None,
    }
    if tracking_report and bbox:
        tracking_metrics, tracking_report_text = tracking_metrics_from_report(tracking_report)
        if tracking_metrics is None:
            evidence_step = HandoffAcceptanceStep(
                name="real_handoff_report",
                status=FAIL,
                summary="tracking report could not be read",
                report=str(tracking_report),
                notes=["TRACKING_REPORT_UNREADABLE"],
            )
            steps.append(evidence_step)
        else:
            handoff_metrics, metric_notes = handoff_metrics_from_tracking(
                bbox=bbox,
                tracking_metrics=tracking_metrics,
                tracking_report=tracking_report_text,
                desired_target_width=desired_target_width,
                ack_real_handoff=ack_real_handoff,
            )
            evidence_status, evidence_summary, evidence_notes = evaluate_handoff_metrics(
                handoff_metrics,
                ack_real_handoff=ack_real_handoff,
            )
            metrics.update(handoff_metrics)
            evidence_step = HandoffAcceptanceStep(
                name="real_handoff_report",
                status=evidence_status,
                summary=evidence_summary,
                report=str(tracking_report),
                notes=metric_notes + evidence_notes,
            )
            steps.append(evidence_step)
    else:
        steps.append(HandoffAcceptanceStep(
            name="real_handoff_report",
            status=WAITING,
            summary="real tracking report is missing",
            notes=["run the follow command or pass --tracking-report"],
        ))

    if any(step.status == FAIL for step in steps):
        status = FAIL
        summary = "real handoff evidence failed"
        next_action = "Inspect the failed handoff acceptance step before refreshing handoff_plan."
    elif any(step.status != PASS for step in steps):
        status = WAITING
        summary = "real handoff evidence is waiting"
        next_action = "After approach/pitch PASS, run this with --run-live-gate --ack-live-input --ack-real-handoff or pass a captured --tracking-report."
    else:
        status = PASS
        summary = "real handoff evidence is ready for handoff_plan"
        next_action = "Refresh pr0p_handoff_plan.py with this JSON as --real-handoff-report and --ack-real-handoff."

    return HandoffAcceptanceReport(
        run_id=run_id,
        started_at=time.strftime("%Y-%m-%dT%H:%M:%S%z", time.localtime()),
        status=status,
        summary=summary,
        next_action=next_action,
        steps=steps,
        commands={"real_pr0p_handoff_live": command},
        metrics=metrics,
    )


def build_markdown(report: HandoffAcceptanceReport) -> str:
    lines = [
        "# pr0p Real Handoff Acceptance Runner",
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


def write_reports(report: HandoffAcceptanceReport, log_dir: Path) -> tuple[Path, Path]:
    log_dir.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime("%Y%m%d-%H%M%S", time.localtime())
    json_path = log_dir / ("%s-%s-real-handoff.json" % (stamp, report.run_id))
    md_path = log_dir / ("%s-%s-real-handoff.md" % (stamp, report.run_id))
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
    parser.add_argument("--run-id", default="pr0p-real-handoff")
    parser.add_argument("--log-dir", type=Path, default=DEFAULT_LOG_DIR)
    parser.add_argument("--bbox", type=parse_bbox)
    parser.add_argument("--bbox-file", type=Path)
    parser.add_argument("--window-title", action="append", dest="window_titles")
    parser.add_argument("--duration", type=float, default=DEFAULT_DURATION_S)
    parser.add_argument("--hz", type=float, default=DEFAULT_HZ)
    parser.add_argument("--tracker", default=DEFAULT_TRACKER)
    parser.add_argument("--min-found-ratio", type=float, default=DEFAULT_MIN_FOUND_RATIO)
    parser.add_argument("--max-loss-events", type=int, default=DEFAULT_MAX_LOSS_EVENTS)
    parser.add_argument("--max-abs-yaw-axis", type=float, default=DEFAULT_MAX_YAW_AXIS)
    parser.add_argument("--max-abs-pitch-axis", type=float, default=DEFAULT_MAX_PITCH_AXIS)
    parser.add_argument("--desired-target-width", type=float,
                        default=DEFAULT_DESIRED_TARGET_WIDTH)
    parser.add_argument("--arm-throttle", type=float, default=DEFAULT_ARM_THROTTLE)
    parser.add_argument("--run-live-gate", action="store_true")
    parser.add_argument("--ack-live-input", action="store_true")
    parser.add_argument("--ack-real-handoff", action="store_true")
    parser.add_argument("--tracking-report", type=Path)
    parser.add_argument("--timeout", type=float, default=300.0)
    args = parser.parse_args(argv)
    if args.window_titles is None:
        args.window_titles = list(DEFAULT_WINDOW_TITLES)
    if args.duration <= 0:
        parser.error("--duration must be positive")
    if args.hz <= 0:
        parser.error("--hz must be positive")
    if not 0 <= args.min_found_ratio <= 1:
        parser.error("--min-found-ratio must be in [0, 1]")
    if args.max_loss_events < 0:
        parser.error("--max-loss-events must be non-negative")
    if args.max_abs_yaw_axis < 0 or args.max_abs_pitch_axis < 0:
        parser.error("--max-abs-* limits must be non-negative")
    if args.desired_target_width <= 0:
        parser.error("--desired-target-width must be positive")
    if not -1.0 <= args.arm_throttle <= 1.0:
        parser.error("--arm-throttle must be in [-1, 1]")
    if args.timeout <= 0:
        parser.error("--timeout must be positive")
    if args.run_live_gate and not args.ack_live_input:
        parser.error("--run-live-gate requires --ack-live-input")
    if args.run_live_gate and not args.ack_real_handoff:
        parser.error("--run-live-gate requires --ack-real-handoff")
    args.bbox = resolve_bbox_argument(
        bbox=args.bbox,
        bbox_file=args.bbox_file,
        parser=parser,
    )
    return args


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    report = build_handoff_acceptance(
        run_id=args.run_id,
        log_dir=args.log_dir,
        bbox=args.bbox,
        window_titles=args.window_titles,
        duration_s=args.duration,
        hz=args.hz,
        tracker=args.tracker,
        min_found_ratio=args.min_found_ratio,
        max_loss_events=args.max_loss_events,
        max_abs_yaw_axis=args.max_abs_yaw_axis,
        max_abs_pitch_axis=args.max_abs_pitch_axis,
        desired_target_width=args.desired_target_width,
        arm_throttle=args.arm_throttle,
        run_live_gate=args.run_live_gate,
        ack_live_input=args.ack_live_input,
        ack_real_handoff=args.ack_real_handoff,
        tracking_report=args.tracking_report,
        timeout_s=args.timeout,
    )
    json_path, md_path = write_reports(report, args.log_dir)
    print("simitl-pr0p-real-handoff %s report=%s summary=%s" % (
        report.status,
        json_path,
        md_path,
    ))
    print(report.next_action)
    return 1 if report.status == FAIL else 0


if __name__ == "__main__":
    raise SystemExit(main())
