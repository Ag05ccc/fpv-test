#!/usr/bin/env python3
"""Plan or run signed pr0p yaw/pitch response acceptance gates."""

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
from pr0p_decision_report import latest_manifest_report, load_json_report  # noqa: E402


PASS = "PASS"
WAITING = "WAITING"
FAIL = "FAIL"
SKIPPED = "SKIPPED"
PLANNED = "PLANNED"

REQUIRED_PREREQ_PHASES = (
    "P3-capture",
    "P4-input-readiness",
    "P4-input-mapping",
    "P5-uinput-rc-effect-throttle-low",
    "P5-uinput-status-effect-throttle-clear",
    "P5-uinput-rc-effect-aux1-high",
    "P5-uinput-aux1-arm-status",
)


@dataclass
class ResponseAcceptanceStep:
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
class ResponseAcceptanceReport:
    run_id: str
    started_at: str
    status: str
    summary: str
    steps: list[ResponseAcceptanceStep] = field(default_factory=list)
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


def live_response_command(
    *,
    run_id: str,
    axis: str,
    magnitude: float,
    image_axis: str,
    expected_sign: int,
    min_shift_px: float,
    max_shift_px: float,
    startup_wait_s: float,
    arm_first: bool = False,
    arm_wait_s: float = 12.0,
    arm_throttle: float = -0.35,
    measure: str = "visual",
    attitude_min_delta_deg: float = 10.0,
) -> str:
    parts = [
        "fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_live_session_runner.py",
        "--launch-pr0p",
        "--ack-live-launch",
        "--send-ui",
        "--ack-live-ui",
        "--hold-uinput",
        "--ack-live-input",
        "--run-live-response",
        "--response-axis %s" % axis,
        "--response-magnitude %s" % magnitude,
        "--response-image-axis %s" % image_axis,
        "--response-expected-sign %d" % expected_sign,
        "--response-min-shift-px %s" % min_shift_px,
        "--response-max-shift-px %s" % max_shift_px,
        "--startup-wait %s" % startup_wait_s,
    ]
    if arm_first:
        parts.extend([
            "--response-arm-first",
            "--response-arm-wait %s" % arm_wait_s,
            "--response-arm-throttle %s" % arm_throttle,
        ])
    if measure != "visual":
        parts.extend([
            "--response-measure %s" % measure,
            "--response-attitude-min-delta %s" % attitude_min_delta_deg,
        ])
    parts.append("--run-id %s-pr0p-response-%s" % (run_id, axis))
    return command_block(*parts)


def manifest_step_statuses(report: dict[str, Any] | None) -> dict[str, str]:
    if not report or not isinstance(report.get("steps"), list):
        return {}
    statuses: dict[str, str] = {}
    for step in report["steps"]:
        if isinstance(step, dict) and step.get("phase"):
            statuses[str(step["phase"])] = str(step.get("status", WAITING))
    return statuses


def prerequisite_step(log_dir: Path) -> ResponseAcceptanceStep:
    manifest_path = latest_manifest_report(log_dir)
    manifest = load_json_report(manifest_path)
    statuses = manifest_step_statuses(manifest)
    missing = [
        phase
        for phase in REQUIRED_PREREQ_PHASES
        if statuses.get(phase) != PASS
    ]
    if manifest_path is None:
        return ResponseAcceptanceStep(
            name="pr0p_response_prerequisites",
            status=WAITING,
            summary="no live manifest is available yet",
            report=None,
            notes=["run pr0p_live_run_manifest.py after the latest AUX1 evidence"],
        )
    if missing:
        return ResponseAcceptanceStep(
            name="pr0p_response_prerequisites",
            status=WAITING,
            summary="response prerequisites are not all PASS",
            report=str(manifest_path),
            notes=["MISSING_OR_WAITING:%s" % ",".join(missing)],
        )
    return ResponseAcceptanceStep(
        name="pr0p_response_prerequisites",
        status=PASS,
        summary="capture, input mapping, throttle-low, AUX1, and ARM status are proven",
        report=str(manifest_path),
        notes=["AUX1_ARM_STATUS_REQUIRED_FOR_RESPONSE"],
    )


def planned_step(name: str, command: str, *, status: str, summary: str, notes: list[str]) -> ResponseAcceptanceStep:
    return ResponseAcceptanceStep(
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
) -> ResponseAcceptanceStep:
    try:
        completed = command_runner(
            command_argv(command),
            capture_output=True,
            text=True,
            timeout=timeout_s,
        )
    except Exception as exc:  # pragma: no cover - subprocess failures vary
        return ResponseAcceptanceStep(
            name=name,
            status=FAIL,
            summary="command could not be executed",
            command=command,
            notes=[str(exc)],
        )
    status = parsed_status(completed.stdout or "", returncode=completed.returncode)
    if completed.returncode != 0 and status != WAITING:
        status = FAIL
    return ResponseAcceptanceStep(
        name=name,
        status=status,
        summary="command returned %s" % status,
        command=command,
        report=parsed_report_path(completed.stdout or ""),
        returncode=completed.returncode,
        stdout=completed.stdout or "",
        stderr=completed.stderr or "",
    )


def summarize_status(steps: list[ResponseAcceptanceStep], *, ran_live_gates: bool) -> tuple[str, str]:
    if any(step.status == FAIL for step in steps):
        return FAIL, "one or more response acceptance commands failed"
    live_steps = {
        step.name: step.status
        for step in steps
        if step.name in {"pr0p_yaw_response_live", "pr0p_pitch_response_live"}
    }
    if (
        ran_live_gates
        and live_steps.get("pr0p_yaw_response_live") == PASS
        and live_steps.get("pr0p_pitch_response_live") == PASS
    ):
        return PASS, "signed yaw and pitch response gates passed"
    return WAITING, "response acceptance is waiting for prerequisite or live gated evidence"


def run_response_acceptance(
    *,
    run_id: str,
    log_dir: Path,
    run_live_gates: bool,
    yaw_expected_sign: int,
    pitch_expected_sign: int,
    yaw_magnitude: float,
    pitch_magnitude: float,
    min_shift_px: float,
    max_shift_px: float,
    startup_wait_s: float,
    timeout_s: float,
    arm_first: bool = False,
    arm_wait_s: float = 12.0,
    arm_throttle: float = -0.35,
    measure: str = "visual",
    attitude_min_delta_deg: float = 10.0,
    command_runner: Callable[..., subprocess.CompletedProcess[str]] = subprocess.run,
) -> ResponseAcceptanceReport:
    yaw_command = live_response_command(
        run_id=run_id,
        axis="yaw",
        magnitude=yaw_magnitude,
        image_axis="x",
        expected_sign=yaw_expected_sign,
        min_shift_px=min_shift_px,
        max_shift_px=max_shift_px,
        startup_wait_s=startup_wait_s,
        arm_first=arm_first,
        arm_wait_s=arm_wait_s,
        arm_throttle=arm_throttle,
        measure=measure,
        attitude_min_delta_deg=attitude_min_delta_deg,
    )
    pitch_command = live_response_command(
        run_id=run_id,
        axis="pitch",
        magnitude=pitch_magnitude,
        image_axis="y",
        expected_sign=pitch_expected_sign,
        min_shift_px=min_shift_px,
        max_shift_px=max_shift_px,
        startup_wait_s=startup_wait_s,
        arm_first=arm_first,
        arm_wait_s=arm_wait_s,
        arm_throttle=arm_throttle,
        measure=measure,
        attitude_min_delta_deg=attitude_min_delta_deg,
    )

    prereq = prerequisite_step(log_dir)
    steps = [prereq]
    if not run_live_gates:
        steps.append(planned_step(
            "pr0p_yaw_response_live",
            yaw_command,
            status=WAITING,
            summary="requires --run-live-gates and live ack flags",
            notes=["runs only after pr0p_response_prerequisites is PASS"],
        ))
        steps.append(planned_step(
            "pr0p_pitch_response_live",
            pitch_command,
            status=WAITING,
            summary="requires --run-live-gates and yaw response PASS first",
            notes=["runs only after pr0p_yaw_response_live is PASS"],
        ))
    elif prereq.status != PASS:
        steps.append(planned_step(
            "pr0p_yaw_response_live",
            yaw_command,
            status=SKIPPED,
            summary="skipped because response prerequisites are not PASS",
            notes=["prove AUX1 ARM status before live response"],
        ))
        steps.append(planned_step(
            "pr0p_pitch_response_live",
            pitch_command,
            status=SKIPPED,
            summary="skipped because response prerequisites are not PASS",
            notes=["prove AUX1 ARM status before live response"],
        ))
    else:
        yaw_step = run_external_command(
            "pr0p_yaw_response_live",
            yaw_command,
            command_runner=command_runner,
            timeout_s=timeout_s,
        )
        steps.append(yaw_step)
        if yaw_step.status == PASS:
            steps.append(run_external_command(
                "pr0p_pitch_response_live",
                pitch_command,
                command_runner=command_runner,
                timeout_s=timeout_s,
            ))
        else:
            steps.append(planned_step(
                "pr0p_pitch_response_live",
                pitch_command,
                status=SKIPPED,
                summary="skipped because yaw response did not pass",
                notes=["run after pr0p_yaw_response_live is PASS"],
            ))

    status, summary = summarize_status(steps, ran_live_gates=run_live_gates)
    return ResponseAcceptanceReport(
        run_id=run_id,
        started_at=time.strftime("%Y-%m-%dT%H:%M:%S%z", time.localtime()),
        status=status,
        summary=summary,
        steps=steps,
        metrics={
            "run_live_gates": run_live_gates,
            "log_dir": str(log_dir),
            "yaw_expected_sign": yaw_expected_sign,
            "pitch_expected_sign": pitch_expected_sign,
            "yaw_magnitude": yaw_magnitude,
            "pitch_magnitude": pitch_magnitude,
            "min_shift_px": min_shift_px,
            "max_shift_px": max_shift_px,
            "startup_wait_s": startup_wait_s,
        },
    )


def build_markdown(report: ResponseAcceptanceReport) -> str:
    lines = [
        "# SimITL / pr0p Response Acceptance Runner",
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
        lines.extend([
            "### %s" % step.name,
            "",
            step.summary,
            "",
        ])
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


def write_reports(report: ResponseAcceptanceReport, log_dir: Path) -> tuple[Path, Path]:
    log_dir.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime("%Y%m%d-%H%M%S", time.localtime())
    json_path = log_dir / ("%s-%s-response-acceptance.json" % (stamp, report.run_id))
    md_path = log_dir / ("%s-%s-response-acceptance.md" % (stamp, report.run_id))
    json_path.write_text(json.dumps(report.as_dict(), indent=2, sort_keys=True) + "\n",
                         encoding="utf-8")
    md_path.write_text(build_markdown(report), encoding="utf-8")
    return json_path, md_path


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-id", default="pr0p-response-acceptance")
    parser.add_argument("--log-dir", type=Path, default=DEFAULT_LOG_DIR)
    parser.add_argument("--run-live-gates", action="store_true")
    parser.add_argument("--ack-live-launch", action="store_true")
    parser.add_argument("--ack-live-ui", action="store_true")
    parser.add_argument("--ack-live-input", action="store_true")
    parser.add_argument("--yaw-expected-sign", type=int, choices=(-1, 0, 1), default=1)
    parser.add_argument("--pitch-expected-sign", type=int, choices=(-1, 0, 1), default=-1)
    parser.add_argument("--yaw-magnitude", type=float, default=0.03)
    parser.add_argument("--pitch-magnitude", type=float, default=0.03)
    parser.add_argument("--min-shift-px", type=float, default=2.0)
    parser.add_argument("--max-shift-px", type=float, default=200.0)
    parser.add_argument("--startup-wait", type=float, default=8.0)
    parser.add_argument("--timeout", type=float, default=240.0)
    parser.add_argument("--arm-first", action="store_true",
                        help="Arm the FC (throttle-low then AUX1-high) and hold hover "
                             "throttle during each response pulse")
    parser.add_argument("--arm-wait", type=float, default=12.0)
    parser.add_argument("--arm-throttle", type=float, default=-0.35)
    parser.add_argument("--measure", default="visual", choices=("visual", "attitude"),
                        help="attitude reads signed MSP_ATTITUDE deltas; use it when the "
                             "FPV view is sky-dominated and pixel shift cannot measure response")
    parser.add_argument("--attitude-min-delta", type=float, default=10.0)
    args = parser.parse_args(argv)
    if args.run_live_gates and not (
        args.ack_live_launch and args.ack_live_ui and args.ack_live_input
    ):
        parser.error("--run-live-gates requires --ack-live-launch --ack-live-ui --ack-live-input")
    if args.yaw_magnitude <= 0 or args.pitch_magnitude <= 0:
        parser.error("--yaw-magnitude/--pitch-magnitude must be positive")
    if args.min_shift_px < 0:
        parser.error("--min-shift-px must be non-negative")
    if args.max_shift_px <= 0:
        parser.error("--max-shift-px must be positive")
    if args.min_shift_px > args.max_shift_px:
        parser.error("--min-shift-px must be <= --max-shift-px")
    if args.startup_wait < 0:
        parser.error("--startup-wait must be non-negative")
    if args.timeout <= 0:
        parser.error("--timeout must be positive")
    if args.arm_wait < 0:
        parser.error("--arm-wait must be non-negative")
    if not -1.0 <= args.arm_throttle <= 1.0:
        parser.error("--arm-throttle must be within [-1.0, 1.0]")
    if args.attitude_min_delta <= 0:
        parser.error("--attitude-min-delta must be positive")
    return args


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    report = run_response_acceptance(
        run_id=args.run_id,
        log_dir=args.log_dir,
        run_live_gates=args.run_live_gates,
        yaw_expected_sign=args.yaw_expected_sign,
        pitch_expected_sign=args.pitch_expected_sign,
        yaw_magnitude=args.yaw_magnitude,
        pitch_magnitude=args.pitch_magnitude,
        min_shift_px=args.min_shift_px,
        max_shift_px=args.max_shift_px,
        startup_wait_s=args.startup_wait,
        timeout_s=args.timeout,
        arm_first=args.arm_first,
        arm_wait_s=args.arm_wait,
        arm_throttle=args.arm_throttle,
        measure=args.measure,
        attitude_min_delta_deg=args.attitude_min_delta,
    )
    json_path, md_path = write_reports(report, args.log_dir)
    print("simitl-pr0p-response-acceptance %s report=%s summary=%s" % (
        report.status,
        json_path,
        md_path,
    ))
    print(report.summary)
    return 1 if report.status == FAIL else 0


if __name__ == "__main__":
    raise SystemExit(main())
