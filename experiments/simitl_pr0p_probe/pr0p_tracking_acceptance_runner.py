#!/usr/bin/env python3
"""Plan or run gated pr0p live tracker/PID acceptance."""

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

from pr0p_capture_probe import DEFAULT_LOG_DIR, DEFAULT_WINDOW_TITLES  # noqa: E402
from pr0p_decision_report import latest_manifest_report, load_json_report  # noqa: E402
from pr0p_live_run_manifest import report_sort_key  # noqa: E402
from pr0p_tracking_probe import parse_bbox  # noqa: E402


PASS = "PASS"
WAITING = "WAITING"
FAIL = "FAIL"
SKIPPED = "SKIPPED"

REQUIRED_MANIFEST_PHASES = (
    "P3-capture",
    "P4-input-readiness",
    "P4-input-mapping",
    "P6-tracking-pid-dry-run",
)
REQUIRED_RESPONSE_STEPS = (
    "pr0p_yaw_response_live",
    "pr0p_pitch_response_live",
)


@dataclass
class TrackingAcceptanceStep:
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
class TrackingAcceptanceReport:
    run_id: str
    started_at: str
    status: str
    summary: str
    steps: list[TrackingAcceptanceStep] = field(default_factory=list)
    metrics: dict[str, Any] = field(default_factory=dict)

    def as_dict(self) -> dict[str, Any]:
        return {
            "run_id": self.run_id,
            "started_at": self.started_at,
            "status": self.status,
            "summary": self.summary,
            "steps": [step.as_dict() for step in self.steps],
            "metrics": self.metrics,
        }


def command_block(*parts: str) -> str:
    separator = " \\\n  "
    return separator.join(parts)


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


def latest_response_acceptance_report(log_dir: Path) -> Path | None:
    reports = list(log_dir.glob("*-response-acceptance.json"))
    return max(reports, key=report_sort_key) if reports else None


def step_statuses(report: dict[str, Any] | None, *, key: str) -> dict[str, str]:
    if not report or not isinstance(report.get("steps"), list):
        return {}
    statuses: dict[str, str] = {}
    for step in report["steps"]:
        if isinstance(step, dict) and step.get(key):
            statuses[str(step[key])] = str(step.get("status", WAITING))
    return statuses


def tracking_command(
    *,
    run_id: str,
    bbox: tuple[int, int, int, int],
    window_title: str,
    duration_s: float,
    hz: float,
    tracker: str,
    live: bool,
    enable_pitch: bool,
    min_found_ratio: float,
    max_loss_events: int,
    max_abs_yaw_axis: float,
    max_abs_pitch_axis: float,
    desired_target_width: float,
    pitch_delay_frames: int = 0,
    arm_first: bool = False,
    arm_throttle: float = -0.4,
    arm_hover_wait_s: float = 0.5,
    takeoff_delay_frames: int = 10,
    takeoff_boost_frames: int = 10,
    takeoff_ramp_frames: int = 0,
    settle_throttle: float = -0.26,
    track_throttle_hold: bool = False,
    track_throttle_kp: float = 0.10,
) -> str:
    bbox_text = ",".join(str(item) for item in bbox)
    parts = [
        "fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_tracking_probe.py",
        "--window-title %s" % shlex.quote(window_title),
        "--bbox %s" % bbox_text,
        "--duration %s" % duration_s,
        "--hz %s" % hz,
        "--tracker %s" % tracker,
        "--min-found-ratio %s" % min_found_ratio,
        "--max-loss-events %d" % max_loss_events,
        "--max-abs-yaw-axis %s" % max_abs_yaw_axis,
        "--max-abs-pitch-axis %s" % max_abs_pitch_axis,
        "--desired-target-width %s" % desired_target_width,
    ]
    if enable_pitch:
        parts.append("--enable-pitch")
        if pitch_delay_frames > 0:
            parts.append("--pitch-delay-frames %d" % pitch_delay_frames)
    if live:
        parts.extend(["--uinput", "--ack-live-input"])
        if arm_first:
            parts.extend([
                "--arm-first",
                "--arm-throttle %s" % arm_throttle,
                "--arm-hover-wait %s" % arm_hover_wait_s,
                "--takeoff-delay-frames %d" % takeoff_delay_frames,
                "--takeoff-boost-frames %d" % takeoff_boost_frames,
                "--settle-throttle %s" % settle_throttle,
            ])
            if takeoff_ramp_frames > 0:
                parts.append("--takeoff-ramp-frames %d" % takeoff_ramp_frames)
            if track_throttle_hold:
                parts.append("--track-throttle-hold")
                parts.append("--track-throttle-kp %s" % track_throttle_kp)
    parts.append("--run-id %s-pr0p-tracking-%s" % (run_id, "live" if live else "dry"))
    return command_block(*parts)


def prerequisite_steps(
    *,
    log_dir: Path,
    bbox: tuple[int, int, int, int] | None,
) -> list[TrackingAcceptanceStep]:
    steps: list[TrackingAcceptanceStep] = []
    if bbox is None:
        steps.append(TrackingAcceptanceStep(
            name="pr0p_tracking_bbox",
            status=WAITING,
            summary="tracking bbox has not been selected yet",
            notes=["pass --bbox x,y,w,h after selecting a target"],
        ))
    else:
        steps.append(TrackingAcceptanceStep(
            name="pr0p_tracking_bbox",
            status=PASS,
            summary="tracking bbox is present",
            notes=["bbox=%s" % ",".join(str(item) for item in bbox)],
        ))

    manifest_path = latest_manifest_report(log_dir)
    manifest = load_json_report(manifest_path)
    manifest_statuses = step_statuses(manifest, key="phase")
    missing_manifest = [
        phase
        for phase in REQUIRED_MANIFEST_PHASES
        if manifest_statuses.get(phase) != PASS
    ]
    if manifest_path is None:
        steps.append(TrackingAcceptanceStep(
            name="pr0p_tracking_manifest_prerequisites",
            status=WAITING,
            summary="no live manifest is available yet",
            notes=["run pr0p_live_run_manifest.py after P6 dry-run evidence"],
        ))
    elif missing_manifest:
        steps.append(TrackingAcceptanceStep(
            name="pr0p_tracking_manifest_prerequisites",
            status=WAITING,
            summary="tracking manifest prerequisites are not all PASS",
            report=str(manifest_path),
            notes=["MISSING_OR_WAITING:%s" % ",".join(missing_manifest)],
        ))
    else:
        steps.append(TrackingAcceptanceStep(
            name="pr0p_tracking_manifest_prerequisites",
            status=PASS,
            summary="capture, input mapping, and P6 dry-run are proven",
            report=str(manifest_path),
            notes=["P6_DRY_RUN_REQUIRED_FOR_LIVE_TRACKING"],
        ))

    response_path = latest_response_acceptance_report(log_dir)
    response = load_json_report(response_path)
    response_statuses = step_statuses(response, key="name")
    missing_response = [
        step
        for step in REQUIRED_RESPONSE_STEPS
        if response_statuses.get(step) != PASS
    ]
    if response_path is None:
        steps.append(TrackingAcceptanceStep(
            name="pr0p_tracking_response_prerequisites",
            status=WAITING,
            summary="no response acceptance report is available yet",
            notes=["run pr0p_response_acceptance_runner.py before live tracking"],
        ))
    elif not isinstance(response, dict) or response.get("status") != PASS:
        missing = ",".join(missing_response) if missing_response else "response_acceptance_status"
        steps.append(TrackingAcceptanceStep(
            name="pr0p_tracking_response_prerequisites",
            status=WAITING,
            summary="signed response acceptance is not PASS",
            report=str(response_path),
            notes=["MISSING_OR_WAITING:%s" % missing],
        ))
    else:
        steps.append(TrackingAcceptanceStep(
            name="pr0p_tracking_response_prerequisites",
            status=PASS,
            summary="signed yaw and pitch response acceptance is proven",
            report=str(response_path),
            notes=["RESPONSE_ACCEPTANCE_REQUIRED_FOR_LIVE_TRACKING"],
        ))
    return steps


def prerequisites_pass(steps: list[TrackingAcceptanceStep]) -> bool:
    return all(step.status == PASS for step in steps)


def planned_step(name: str, command: str, *, status: str, summary: str, notes: list[str]) -> TrackingAcceptanceStep:
    return TrackingAcceptanceStep(
        name=name,
        status=status,
        summary=summary,
        command=command,
        notes=notes,
    )


def run_external_command(
    name: str,
    command: str,
    *,
    command_runner: Callable[..., subprocess.CompletedProcess[str]] = subprocess.run,
    timeout_s: float,
) -> TrackingAcceptanceStep:
    try:
        completed = command_runner(
            command_argv(command),
            capture_output=True,
            text=True,
            timeout=timeout_s,
        )
    except Exception as exc:  # pragma: no cover - subprocess failures vary
        return TrackingAcceptanceStep(
            name=name,
            status=FAIL,
            summary="command could not be executed",
            command=command,
            notes=[str(exc)],
        )
    status = parsed_status(completed.stdout or "", returncode=completed.returncode)
    if completed.returncode != 0 and status != WAITING:
        status = FAIL
    return TrackingAcceptanceStep(
        name=name,
        status=status,
        summary="command returned %s" % status,
        command=command,
        report=parsed_report_path(completed.stdout or ""),
        returncode=completed.returncode,
        stdout=completed.stdout or "",
        stderr=completed.stderr or "",
    )


def summarize_status(steps: list[TrackingAcceptanceStep], *, ran_live_gates: bool) -> tuple[str, str]:
    if any(step.status == FAIL for step in steps):
        return FAIL, "one or more tracking acceptance commands failed"
    if ran_live_gates and any(
        step.name == "pr0p_tracking_live" and step.status == PASS
        for step in steps
    ):
        return PASS, "live tracker/PID gate passed"
    return WAITING, "tracking acceptance is waiting for prerequisite or live gated evidence"


def run_tracking_acceptance(
    *,
    run_id: str,
    log_dir: Path,
    bbox: tuple[int, int, int, int] | None,
    execute_dry_run: bool,
    run_live_gates: bool,
    window_title: str,
    duration_s: float,
    hz: float,
    tracker: str,
    enable_pitch: bool,
    min_found_ratio: float,
    max_loss_events: int,
    max_abs_yaw_axis: float,
    max_abs_pitch_axis: float,
    desired_target_width: float,
    timeout_s: float,
    pitch_delay_frames: int = 0,
    arm_first: bool = False,
    arm_throttle: float = -0.4,
    arm_hover_wait_s: float = 0.5,
    takeoff_delay_frames: int = 10,
    takeoff_boost_frames: int = 10,
    takeoff_ramp_frames: int = 0,
    settle_throttle: float = -0.26,
    track_throttle_hold: bool = False,
    track_throttle_kp: float = 0.10,
    command_runner: Callable[..., subprocess.CompletedProcess[str]] = subprocess.run,
) -> TrackingAcceptanceReport:
    steps = prerequisite_steps(log_dir=log_dir, bbox=bbox)
    command_bbox = bbox or (0, 0, 1, 1)
    dry_command = tracking_command(
        run_id=run_id,
        bbox=command_bbox,
        window_title=window_title,
        duration_s=duration_s,
        hz=hz,
        tracker=tracker,
        live=False,
        enable_pitch=enable_pitch,
        min_found_ratio=min_found_ratio,
        max_loss_events=max_loss_events,
        max_abs_yaw_axis=max_abs_yaw_axis,
        max_abs_pitch_axis=max_abs_pitch_axis,
        desired_target_width=desired_target_width,
    )
    live_command = tracking_command(
        run_id=run_id,
        bbox=command_bbox,
        window_title=window_title,
        duration_s=duration_s,
        hz=hz,
        tracker=tracker,
        live=True,
        enable_pitch=enable_pitch,
        min_found_ratio=min_found_ratio,
        max_loss_events=max_loss_events,
        max_abs_yaw_axis=max_abs_yaw_axis,
        max_abs_pitch_axis=max_abs_pitch_axis,
        desired_target_width=desired_target_width,
        pitch_delay_frames=pitch_delay_frames,
        arm_first=arm_first,
        arm_throttle=arm_throttle,
        arm_hover_wait_s=arm_hover_wait_s,
        takeoff_delay_frames=takeoff_delay_frames,
        takeoff_boost_frames=takeoff_boost_frames,
        takeoff_ramp_frames=takeoff_ramp_frames,
        settle_throttle=settle_throttle,
        track_throttle_hold=track_throttle_hold,
        track_throttle_kp=track_throttle_kp,
    )

    if execute_dry_run:
        if bbox is None:
            steps.append(planned_step(
                "pr0p_tracking_dry_run",
                dry_command,
                status=SKIPPED,
                summary="skipped because bbox is missing",
                notes=["select bbox before running dry tracking"],
            ))
        else:
            steps.append(run_external_command(
                "pr0p_tracking_dry_run",
                dry_command,
                command_runner=command_runner,
                timeout_s=timeout_s,
            ))
    else:
        steps.append(planned_step(
            "pr0p_tracking_dry_run",
            dry_command,
            status=WAITING,
            summary="requires --execute-dry-run",
            notes=["dry-run sends no real OS input"],
        ))

    if run_live_gates:
        if not prerequisites_pass(steps[:3]):
            steps.append(planned_step(
                "pr0p_tracking_live",
                live_command,
                status=SKIPPED,
                summary="skipped because live tracking prerequisites are not PASS",
                notes=["prove bbox, P6 dry-run, and response acceptance first"],
            ))
        else:
            steps.append(run_external_command(
                "pr0p_tracking_live",
                live_command,
                command_runner=command_runner,
                timeout_s=timeout_s,
            ))
    else:
        steps.append(planned_step(
            "pr0p_tracking_live",
            live_command,
            status=WAITING,
            summary="requires --run-live-gates and --ack-live-input",
            notes=["runs only after response acceptance and P6 dry-run are PASS"],
        ))

    status, summary = summarize_status(steps, ran_live_gates=run_live_gates)
    return TrackingAcceptanceReport(
        run_id=run_id,
        started_at=time.strftime("%Y-%m-%dT%H:%M:%S%z", time.localtime()),
        status=status,
        summary=summary,
        steps=steps,
        metrics={
            "bbox": list(bbox) if bbox else None,
            "execute_dry_run": execute_dry_run,
            "run_live_gates": run_live_gates,
            "log_dir": str(log_dir),
            "window_title": window_title,
            "duration_s": duration_s,
            "hz": hz,
            "tracker": tracker,
            "enable_pitch": enable_pitch,
            "min_found_ratio": min_found_ratio,
            "max_loss_events": max_loss_events,
            "max_abs_yaw_axis": max_abs_yaw_axis,
            "max_abs_pitch_axis": max_abs_pitch_axis,
            "desired_target_width": desired_target_width,
            "arm_first": arm_first,
            "arm_throttle": arm_throttle,
            "arm_hover_wait_s": arm_hover_wait_s,
            "takeoff_delay_frames": takeoff_delay_frames,
            "takeoff_boost_frames": takeoff_boost_frames,
            "settle_throttle": settle_throttle,
        },
    )


def build_markdown(report: TrackingAcceptanceReport) -> str:
    lines = [
        "# SimITL / pr0p Tracking Acceptance Runner",
        "",
        "Run: `%s`" % report.run_id,
        "Started: `%s`" % report.started_at,
        "Verdict: `%s`" % report.status,
        "",
        report.summary,
        "",
        "| Step | Status | Report |",
        "| --- | --- | --- |",
    ]
    for step in report.steps:
        lines.append("| %s | `%s` | `%s` |" % (
            step.name,
            step.status,
            step.report or "-",
        ))
    lines.extend(["", "## Steps", ""])
    for step in report.steps:
        lines.extend(["### %s" % step.name, "", step.summary, ""])
        if step.command:
            lines.extend(["```bash", step.command, "```", ""])
        for note in step.notes:
            lines.append("- %s" % note)
        if step.notes:
            lines.append("")
    lines.extend([
        "## Metrics",
        "",
        "```json",
        json.dumps(report.metrics, indent=2, sort_keys=True),
        "```",
        "",
    ])
    return "\n".join(lines)


def write_reports(report: TrackingAcceptanceReport, log_dir: Path) -> tuple[Path, Path]:
    log_dir.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime("%Y%m%d-%H%M%S", time.localtime())
    json_path = log_dir / ("%s-%s-tracking-acceptance.json" % (stamp, report.run_id))
    md_path = log_dir / ("%s-%s-tracking-acceptance.md" % (stamp, report.run_id))
    json_path.write_text(json.dumps(report.as_dict(), indent=2, sort_keys=True) + "\n",
                         encoding="utf-8")
    md_path.write_text(build_markdown(report), encoding="utf-8")
    return json_path, md_path


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-id", default="pr0p-tracking-acceptance")
    parser.add_argument("--log-dir", type=Path, default=DEFAULT_LOG_DIR)
    parser.add_argument("--bbox", type=parse_bbox)
    parser.add_argument("--execute-dry-run", action="store_true")
    parser.add_argument("--run-live-gates", action="store_true")
    parser.add_argument("--ack-live-input", action="store_true")
    parser.add_argument("--window-title", default=DEFAULT_WINDOW_TITLES[0])
    parser.add_argument("--duration", type=float, default=5.0)
    parser.add_argument("--hz", type=float, default=10.0)
    parser.add_argument("--tracker", choices=("CSRT", "KCF"), default="CSRT")
    parser.add_argument("--enable-pitch", action="store_true")
    parser.add_argument("--pitch-delay-frames", type=int, default=0,
                        help="Forwarded to the probe: hold pitch at zero for the first "
                             "N loop frames so approach starts after yaw settles")
    parser.add_argument("--min-found-ratio", type=float, default=0.85)
    parser.add_argument("--max-loss-events", type=int, default=0)
    parser.add_argument("--max-abs-yaw-axis", type=float, default=0.6)
    parser.add_argument("--max-abs-pitch-axis", type=float, default=0.6)
    parser.add_argument("--desired-target-width", type=float, default=120.0)
    parser.add_argument("--timeout", type=float, default=180.0)
    parser.add_argument("--arm-first", action="store_true",
                        help="Arm the FC and hold hover throttle during live tracking "
                             "so PID output steers a flying vehicle")
    parser.add_argument("--arm-throttle", type=float, default=-0.4)
    parser.add_argument("--arm-hover-wait", type=float, default=0.5)
    parser.add_argument("--takeoff-delay-frames", type=int, default=10)
    parser.add_argument("--takeoff-boost-frames", type=int, default=10)
    parser.add_argument("--takeoff-ramp-frames", type=int, default=0)
    parser.add_argument("--settle-throttle", type=float, default=-0.26)
    parser.add_argument("--track-throttle-hold", action="store_true",
                        help="Forwarded to the probe: vision-based throttle hold that "
                             "keeps the target near the vertical frame center")
    parser.add_argument("--track-throttle-kp", type=float, default=0.10)
    args = parser.parse_args(argv)
    if args.run_live_gates and not args.ack_live_input:
        parser.error("--run-live-gates requires --ack-live-input")
    if not -1.0 <= args.arm_throttle <= 1.0:
        parser.error("--arm-throttle must be within [-1.0, 1.0]")
    if args.arm_hover_wait < 0:
        parser.error("--arm-hover-wait must be non-negative")
    if args.takeoff_delay_frames < 0:
        parser.error("--takeoff-delay-frames must be non-negative")
    if args.takeoff_boost_frames < 0:
        parser.error("--takeoff-boost-frames must be non-negative")
    if not -1.0 <= args.settle_throttle <= 1.0:
        parser.error("--settle-throttle must be within [-1.0, 1.0]")
    if args.duration <= 0:
        parser.error("--duration must be positive")
    if args.hz <= 0:
        parser.error("--hz must be positive")
    if not 0 <= args.min_found_ratio <= 1:
        parser.error("--min-found-ratio must be between 0 and 1")
    if args.max_loss_events < 0:
        parser.error("--max-loss-events must be non-negative")
    if args.max_abs_yaw_axis < 0 or args.max_abs_pitch_axis < 0:
        parser.error("--max-abs-* limits must be non-negative")
    if args.desired_target_width <= 0:
        parser.error("--desired-target-width must be positive")
    if args.pitch_delay_frames < 0:
        parser.error("--pitch-delay-frames must be non-negative")
    if args.timeout <= 0:
        parser.error("--timeout must be positive")
    return args


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    report = run_tracking_acceptance(
        run_id=args.run_id,
        log_dir=args.log_dir,
        bbox=args.bbox,
        execute_dry_run=args.execute_dry_run,
        run_live_gates=args.run_live_gates,
        window_title=args.window_title,
        duration_s=args.duration,
        hz=args.hz,
        tracker=args.tracker,
        enable_pitch=args.enable_pitch,
        min_found_ratio=args.min_found_ratio,
        max_loss_events=args.max_loss_events,
        max_abs_yaw_axis=args.max_abs_yaw_axis,
        max_abs_pitch_axis=args.max_abs_pitch_axis,
        desired_target_width=args.desired_target_width,
        timeout_s=args.timeout,
        pitch_delay_frames=args.pitch_delay_frames,
        arm_first=args.arm_first,
        arm_throttle=args.arm_throttle,
        arm_hover_wait_s=args.arm_hover_wait,
        takeoff_delay_frames=args.takeoff_delay_frames,
        takeoff_boost_frames=args.takeoff_boost_frames,
        takeoff_ramp_frames=args.takeoff_ramp_frames,
        settle_throttle=args.settle_throttle,
        track_throttle_hold=args.track_throttle_hold,
        track_throttle_kp=args.track_throttle_kp,
    )
    json_path, md_path = write_reports(report, args.log_dir)
    print("simitl-pr0p-tracking-acceptance %s report=%s summary=%s" % (
        report.status,
        json_path,
        md_path,
    ))
    print(report.summary)
    return 1 if report.status == FAIL else 0


if __name__ == "__main__":
    raise SystemExit(main())
