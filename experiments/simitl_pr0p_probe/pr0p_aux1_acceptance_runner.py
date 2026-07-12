#!/usr/bin/env python3
"""Plan or run the pr0p AUX1/ARM acceptance sequence."""

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

from independent_sim_readiness import build_commands  # noqa: E402
from pr0p_capture_probe import DEFAULT_LOG_DIR  # noqa: E402
from screen_tracking_loop import parse_bbox  # noqa: E402


PASS = "PASS"
WAITING = "WAITING"
FAIL = "FAIL"
SKIPPED = "SKIPPED"
PLANNED = "PLANNED"


@dataclass
class Aux1StepResult:
    name: str
    status: str
    summary: str
    command: str
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
class Aux1AcceptanceReport:
    run_id: str
    started_at: str
    status: str
    summary: str
    steps: list[Aux1StepResult] = field(default_factory=list)
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


def command_argv(command: str) -> list[str]:
    normalized = command.replace("\\\n", " ")
    return shlex.split(normalized)


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


def run_external_command(
    name: str,
    command: str,
    *,
    command_runner: Callable[..., subprocess.CompletedProcess[str]] = subprocess.run,
    timeout_s: float,
) -> Aux1StepResult:
    argv = command_argv(command)
    try:
        completed = command_runner(
            argv,
            capture_output=True,
            text=True,
            timeout=timeout_s,
        )
    except Exception as exc:  # pragma: no cover - exact subprocess failures vary
        return Aux1StepResult(
            name=name,
            status=FAIL,
            summary="command could not be executed",
            command=command,
            notes=[str(exc)],
        )
    status = parsed_status(completed.stdout or "", returncode=completed.returncode)
    if completed.returncode != 0 and status != WAITING:
        status = FAIL
    return Aux1StepResult(
        name=name,
        status=status,
        summary="command returned %s" % status,
        command=command,
        report=parsed_report_path(completed.stdout or ""),
        returncode=completed.returncode,
        stdout=completed.stdout or "",
        stderr=completed.stderr or "",
    )


def planned_step(name: str, command: str, *, status: str, summary: str, notes: list[str]) -> Aux1StepResult:
    return Aux1StepResult(
        name=name,
        status=status,
        summary=summary,
        command=command,
        notes=notes,
    )


def summarize_status(steps: list[Aux1StepResult], *, ran_live_gates: bool) -> tuple[str, str]:
    if any(step.status == FAIL for step in steps):
        return FAIL, "one or more AUX1 acceptance commands failed"
    live_steps = {
        step.name: step.status
        for step in steps
        if step.name in {"pr0p_aux1_rc_effect", "pr0p_aux1_arm_status"}
    }
    if (
        ran_live_gates
        and live_steps.get("pr0p_aux1_rc_effect") == PASS
        and live_steps.get("pr0p_aux1_arm_status") == PASS
    ):
        return PASS, "AUX1 RC effect and AUX1 arm-status gates passed"
    return WAITING, "AUX1 acceptance is waiting for live gated evidence"


def run_aux1_acceptance(
    *,
    run_id: str,
    log_dir: Path,
    bbox: tuple[int, int, int, int] | None,
    execute_dry_patch: bool,
    apply_config_patch: bool,
    run_live_gates: bool,
    timeout_s: float,
    command_runner: Callable[..., subprocess.CompletedProcess[str]] = subprocess.run,
) -> Aux1AcceptanceReport:
    commands = build_commands(run_id, bbox)
    steps: list[Aux1StepResult] = []

    dry_command = commands["pr0p_aux1_config_patch_dry"]
    write_command = commands["pr0p_aux1_config_patch_write"]
    rc_command = commands["pr0p_aux1_rc_effect"]
    arm_command = commands["pr0p_aux1_arm_status"]

    if execute_dry_patch or apply_config_patch:
        steps.append(run_external_command(
            "pr0p_aux1_config_patch_dry",
            dry_command,
            command_runner=command_runner,
            timeout_s=timeout_s,
        ))
    else:
        steps.append(planned_step(
            "pr0p_aux1_config_patch_dry",
            dry_command,
            status=PLANNED,
            summary="dry-run the AUX1 config patch",
            notes=["no config write is performed in plan-only mode"],
        ))

    if apply_config_patch:
        steps.append(run_external_command(
            "pr0p_aux1_config_patch_write",
            write_command,
            command_runner=command_runner,
            timeout_s=timeout_s,
        ))
    else:
        steps.append(planned_step(
            "pr0p_aux1_config_patch_write",
            write_command,
            status=WAITING,
            summary="requires --apply-config-patch --ack-config-write",
            notes=["writes pr0p input.json only when explicit ack flags are used"],
        ))

    if run_live_gates:
        rc_step = run_external_command(
            "pr0p_aux1_rc_effect",
            rc_command,
            command_runner=command_runner,
            timeout_s=timeout_s,
        )
        steps.append(rc_step)
        if rc_step.status == PASS:
            steps.append(run_external_command(
                "pr0p_aux1_arm_status",
                arm_command,
                command_runner=command_runner,
                timeout_s=timeout_s,
            ))
        else:
            steps.append(planned_step(
                "pr0p_aux1_arm_status",
                arm_command,
                status=SKIPPED,
                summary="skipped because AUX1 RC-effect did not pass",
                notes=["run after pr0p_aux1_rc_effect is PASS"],
            ))
    else:
        steps.append(planned_step(
            "pr0p_aux1_rc_effect",
            rc_command,
            status=WAITING,
            summary="requires --run-live-gates and live ack flags",
            notes=["launches pr0p and sends real OS input only with explicit ack flags"],
        ))
        steps.append(planned_step(
            "pr0p_aux1_arm_status",
            arm_command,
            status=WAITING,
            summary="requires pr0p_aux1_rc_effect PASS first",
            notes=["launches pr0p and sends real OS input only with explicit ack flags"],
        ))

    status, summary = summarize_status(steps, ran_live_gates=run_live_gates)
    return Aux1AcceptanceReport(
        run_id=run_id,
        started_at=time.strftime("%Y-%m-%dT%H:%M:%S%z", time.localtime()),
        status=status,
        summary=summary,
        steps=steps,
        metrics={
            "bbox": list(bbox) if bbox else None,
            "execute_dry_patch": execute_dry_patch,
            "apply_config_patch": apply_config_patch,
            "run_live_gates": run_live_gates,
            "log_dir": str(log_dir),
        },
    )


def build_markdown(report: Aux1AcceptanceReport) -> str:
    lines = [
        "# SimITL / pr0p AUX1 Acceptance Runner",
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
    lines.extend([
        "",
        "## Steps",
        "",
    ])
    for step in report.steps:
        lines.extend([
            "### %s" % step.name,
            "",
            step.summary,
            "",
            "```bash",
            step.command,
            "```",
            "",
        ])
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


def write_reports(report: Aux1AcceptanceReport, log_dir: Path) -> tuple[Path, Path]:
    log_dir.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime("%Y%m%d-%H%M%S", time.localtime())
    json_path = log_dir / ("%s-%s-aux1-acceptance.json" % (stamp, report.run_id))
    md_path = log_dir / ("%s-%s-aux1-acceptance.md" % (stamp, report.run_id))
    json_path.write_text(json.dumps(report.as_dict(), indent=2, sort_keys=True) + "\n",
                         encoding="utf-8")
    md_path.write_text(build_markdown(report), encoding="utf-8")
    return json_path, md_path


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-id", default="pr0p-aux1-acceptance")
    parser.add_argument("--log-dir", type=Path, default=DEFAULT_LOG_DIR)
    parser.add_argument("--bbox", type=parse_bbox)
    parser.add_argument("--execute-dry-patch", action="store_true")
    parser.add_argument("--apply-config-patch", action="store_true")
    parser.add_argument("--ack-config-write", action="store_true")
    parser.add_argument("--run-live-gates", action="store_true")
    parser.add_argument("--ack-live-launch", action="store_true")
    parser.add_argument("--ack-live-ui", action="store_true")
    parser.add_argument("--ack-live-input", action="store_true")
    parser.add_argument("--timeout", type=float, default=180.0)
    args = parser.parse_args(argv)
    if args.timeout <= 0:
        parser.error("--timeout must be positive")
    if args.apply_config_patch and not args.ack_config_write:
        parser.error("--apply-config-patch requires --ack-config-write")
    if args.run_live_gates and not (
        args.ack_live_launch and args.ack_live_ui and args.ack_live_input
    ):
        parser.error("--run-live-gates requires --ack-live-launch --ack-live-ui --ack-live-input")
    return args


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    report = run_aux1_acceptance(
        run_id=args.run_id,
        log_dir=args.log_dir,
        bbox=args.bbox,
        execute_dry_patch=args.execute_dry_patch,
        apply_config_patch=args.apply_config_patch,
        run_live_gates=args.run_live_gates,
        timeout_s=args.timeout,
    )
    json_path, md_path = write_reports(report, args.log_dir)
    print("simitl-pr0p-aux1-acceptance %s report=%s summary=%s" % (
        report.status,
        json_path,
        md_path,
    ))
    print(report.summary)
    return 1 if report.status == FAIL else 0


if __name__ == "__main__":
    raise SystemExit(main())
