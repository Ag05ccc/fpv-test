#!/usr/bin/env python3
"""Safely continue the game/screen sandbox from the current status report."""

from __future__ import annotations

import argparse
import json
import shlex
import subprocess
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))
TOOLS_DIR = REPO_ROOT / "tools"
if str(TOOLS_DIR) not in sys.path:
    sys.path.insert(0, str(TOOLS_DIR))

from sandbox_status_report import (  # noqa: E402
    DEFAULT_BBOX_FILE,
    DEFAULT_EVIDENCE_STALE_AFTER_S,
    DEFAULT_LOG_DIR,
    READY,
    REJECT,
    WAITING,
    SandboxStatusReport,
    build_status_report,
    status_by_name,
    write_reports as write_status_reports,
)
from latency_probe import normalize_axes, parse_axis_shift_expectations  # noqa: E402
from sitl_log import resolve_log_dir, timestamp_slug  # noqa: E402


SANDBOX_RESUME_READY = "SANDBOX_RESUME_READY"
SANDBOX_RESUME_PROGRESS = "SANDBOX_RESUME_PROGRESS"
PASS = "PASS"
PLANNED = "PLANNED"
SKIPPED = "SKIPPED"
BLOCKED = "BLOCKED"
TIMEOUT = "TIMEOUT"


@dataclass(frozen=True)
class CommandRunResult:
    returncode: int
    stdout: str = ""
    stderr: str = ""
    timed_out: bool = False


@dataclass
class SandboxResumeRunnerReport:
    run_id: str
    started_at: str
    status: str
    summary: str
    steps: list[dict[str, Any]] = field(default_factory=list)
    evidence_paths: dict[str, str | None] = field(default_factory=dict)
    metrics: dict[str, Any] = field(default_factory=dict)

    def as_dict(self) -> dict[str, Any]:
        return {
            "run_id": self.run_id,
            "started_at": self.started_at,
            "status": self.status,
            "summary": self.summary,
            "steps": self.steps,
            "evidence_paths": self.evidence_paths,
            "metrics": self.metrics,
        }


def output_tail(text: str, limit: int = 1200) -> str:
    if len(text) <= limit:
        return text
    return text[-limit:]


def run_shell_command(command: str, *, timeout_s: float) -> CommandRunResult:
    try:
        completed = subprocess.run(
            shlex.split(command),
            cwd=REPO_ROOT,
            text=True,
            capture_output=True,
            timeout=timeout_s,
            check=False,
        )
    except subprocess.TimeoutExpired as exc:
        return CommandRunResult(
            returncode=124,
            stdout=exc.stdout or "",
            stderr=exc.stderr or "",
            timed_out=True,
        )
    return CommandRunResult(
        returncode=completed.returncode,
        stdout=completed.stdout,
        stderr=completed.stderr,
        timed_out=False,
    )


def command_status_from_returncode(result: CommandRunResult) -> str:
    if result.timed_out:
        return TIMEOUT
    if result.returncode == 0:
        return PASS
    if result.returncode == 2:
        return WAITING
    return REJECT


def command_summary_from_status(status: str) -> str:
    if status == PASS:
        return "command completed successfully"
    if status == WAITING:
        return "command ran but reported WAITING"
    if status == TIMEOUT:
        return "command timed out"
    if status == REJECT:
        return "command failed"
    return "command was not executed"


def base_step(command: dict[str, Any]) -> dict[str, Any]:
    return {
        "name": command.get("name", "unknown"),
        "status": PLANNED,
        "summary": "not evaluated yet",
        "safety_class": command.get("safety_class", "unknown"),
        "sends_uinput": bool(command.get("sends_uinput")),
        "requires_ack_live_input": bool(command.get("requires_ack_live_input")),
        "requires_edit": bool(command.get("requires_edit")),
        "requires_user_interaction": bool(command.get("requires_user_interaction")),
        "requires_pass": list(command.get("requires_pass", [])),
        "unmet_requires_pass": [],
        "expected_status": command.get("expected_status"),
        "expected_report_glob": command.get("expected_report_glob"),
        "expected_report_path": command.get("expected_report_path"),
        "unblocks": command.get("unblocks", []),
        "command": command.get("command", ""),
    }


def evaluate_resume_command(
    command: dict[str, Any],
    *,
    item_statuses: dict[str, str],
    execute_safe: bool,
    execute_live_input: bool,
    ack_live_input: bool,
    executed_count: int,
    max_commands: int,
    timeout_s: float,
) -> tuple[dict[str, Any], int]:
    step = base_step(command)
    name = str(command.get("name", "unknown"))
    if name == "status":
        step["status"] = SKIPPED
        step["summary"] = "status was already evaluated by sandbox_resume_runner"
        return step, executed_count
    if command.get("requires_edit"):
        step["status"] = BLOCKED
        step["summary"] = "manual edit is required before this command can run"
        return step, executed_count
    if command.get("requires_user_interaction"):
        step["status"] = BLOCKED
        step["summary"] = "manual UI interaction is required before this command can run"
        return step, executed_count
    if command.get("sends_uinput"):
        if not execute_live_input:
            step["status"] = BLOCKED
            step["summary"] = "live input command requires --execute-live-input"
            return step, executed_count
        if not ack_live_input:
            step["status"] = BLOCKED
            step["summary"] = "live input command requires --ack-live-input"
            return step, executed_count
    requires_pass = list(command.get("requires_pass", []))
    unmet = [
        "%s=%s" % (name, item_statuses.get(name, "UNKNOWN"))
        for name in requires_pass
        if item_statuses.get(name) != PASS
    ]
    if unmet:
        step["status"] = BLOCKED
        step["unmet_requires_pass"] = unmet
        step["summary"] = "required status items are not PASS: %s" % ", ".join(unmet)
        return step, executed_count
    elif not execute_safe:
        step["status"] = PLANNED
        step["summary"] = "safe command is available; rerun with --execute-safe"
        return step, executed_count

    if executed_count >= max_commands:
        step["status"] = PLANNED
        step["summary"] = "execution limit reached before this command"
        return step, executed_count

    result = run_shell_command(str(command.get("command", "")), timeout_s=timeout_s)
    step["status"] = command_status_from_returncode(result)
    step["summary"] = command_summary_from_status(step["status"])
    step["returncode"] = result.returncode
    step["stdout_tail"] = output_tail(result.stdout)
    step["stderr_tail"] = output_tail(result.stderr)
    return step, executed_count + 1


def decide_resume_status(
    *,
    status_report: SandboxStatusReport,
    post_status_report: SandboxStatusReport | None = None,
    steps: list[dict[str, Any]],
) -> str:
    if status_report.status == REJECT:
        return REJECT
    if post_status_report is not None:
        if post_status_report.status == REJECT:
            return REJECT
        if post_status_report.status == READY:
            return SANDBOX_RESUME_READY
    if any(step.get("status") in {REJECT, TIMEOUT} for step in steps):
        return REJECT
    if any(step.get("status") == PASS for step in steps):
        return SANDBOX_RESUME_PROGRESS
    if status_report.status == READY:
        return SANDBOX_RESUME_READY
    return WAITING


def step_brief(step: dict[str, Any] | None) -> dict[str, Any] | None:
    if step is None:
        return None
    return {
        "name": step.get("name"),
        "status": step.get("status"),
        "summary": step.get("summary"),
        "safety_class": step.get("safety_class"),
        "sends_uinput": bool(step.get("sends_uinput")),
        "requires_pass": list(step.get("requires_pass", [])),
        "unmet_requires_pass": list(step.get("unmet_requires_pass", [])),
        "command": step.get("command"),
    }


def build_step_summary(steps: list[dict[str, Any]]) -> dict[str, Any]:
    status_counts: dict[str, int] = {}
    for step in steps:
        status = str(step.get("status", "UNKNOWN"))
        status_counts[status] = status_counts.get(status, 0) + 1
    first_planned = next(
        (step for step in steps if step.get("status") == PLANNED),
        None,
    )
    first_blocked = next(
        (step for step in steps if step.get("status") == BLOCKED),
        None,
    )
    first_unmet = next(
        (step for step in steps if step.get("unmet_requires_pass")),
        None,
    )
    first_failure = next(
        (step for step in steps if step.get("status") in {REJECT, TIMEOUT}),
        None,
    )
    return {
        "total_count": len(steps),
        "status_counts": status_counts,
        "sends_uinput_count": sum(1 for step in steps if step.get("sends_uinput")),
        "executed_count": sum(1 for step in steps if step.get("returncode") is not None),
        "first_planned_step": step_brief(first_planned),
        "first_blocked_step": step_brief(first_blocked),
        "first_unmet_requirement_step": step_brief(first_unmet),
        "first_failure_step": step_brief(first_failure),
    }


def build_resume_decision(
    *,
    status: str,
    status_report: SandboxStatusReport,
    steps: list[dict[str, Any]],
    executed_count: int,
) -> dict[str, Any]:
    first_blocked = next(
        (step for step in steps if step.get("status") == BLOCKED),
        None,
    )
    first_unmet = next(
        (step for step in steps if step.get("unmet_requires_pass")),
        None,
    )
    first_planned = next(
        (step for step in steps if step.get("status") == PLANNED),
        None,
    )
    first_failure = next(
        (step for step in steps if step.get("status") in {REJECT, TIMEOUT}),
        None,
    )
    executed_steps = [
        step.get("name")
        for step in steps
        if step.get("status") in {PASS, WAITING, REJECT, TIMEOUT}
        and step.get("returncode") is not None
    ]
    if first_failure is not None:
        decision = "failure"
    elif first_unmet is not None:
        decision = "missing_required_status"
    elif first_planned is not None:
        decision = "command_available"
    elif first_blocked is not None:
        decision = "manual_or_ack_required"
    elif status == SANDBOX_RESUME_READY:
        decision = "ready"
    else:
        decision = "waiting"
    return {
        "decision": decision,
        "next_operator_action": (
            status_report.next_actions[0]
            if status_report.next_actions else None
        ),
        "next_action_candidates": list(status_report.next_actions[:3]),
        "first_blocked_step": step_brief(first_blocked),
        "first_unmet_requirement_step": step_brief(first_unmet),
        "first_planned_step": step_brief(first_planned),
        "first_failure_step": step_brief(first_failure),
        "executed_steps": executed_steps,
        "executed_count": executed_count,
    }


def resume_summary(status: str) -> str:
    if status == SANDBOX_RESUME_READY:
        return "requested sandbox status gate is already ready"
    if status == SANDBOX_RESUME_PROGRESS:
        return "safe resume command executed; post-status is still waiting"
    if status == REJECT:
        return "sandbox resume runner hit a blocking failure"
    return "sandbox resume runner is waiting for setup, manual edit, or explicit execution flags"


def build_markdown(report: SandboxResumeRunnerReport) -> str:
    decision = report.metrics.get("resume_decision", {})
    step_summary = report.metrics.get("resume_step_summary", {})
    lines = [
        "# Game Screen Sandbox Resume Runner",
        "",
        "Run: `%s`" % report.run_id,
        "Started: `%s`" % report.started_at,
        "Status: `%s`" % report.status,
        "",
        report.summary,
        "",
        "## Resume Decision",
        "",
        "| Field | Value |",
        "| --- | --- |",
        "| decision | `%s` |" % decision.get("decision", "unknown"),
        "| next_operator_action | %s |" % (
            decision.get("next_operator_action") or "none"
        ),
        "| first_unmet_requirement | %s |" % (
            ", ".join(
                (
                    decision.get("first_unmet_requirement_step") or {}
                ).get("unmet_requires_pass", [])
            ) or "none"
        ),
        "| first_planned_step | %s |" % (
            (decision.get("first_planned_step") or {}).get("name") or "none"
        ),
        "| first_blocked_step | %s |" % (
            (decision.get("first_blocked_step") or {}).get("name") or "none"
        ),
        "| step_status_counts | `%s` |" % json.dumps(
            step_summary.get("status_counts", {})
            if isinstance(step_summary, dict) else
            {},
            sort_keys=True,
        ),
        "",
        "## Steps",
        "",
        "| Name | Status | Safety | Sends uinput | Blocked by | Summary |",
        "| --- | --- | --- | --- | --- | --- |",
    ]
    for step in report.steps:
        lines.append("| %s | `%s` | %s | %s | %s | %s |" % (
            step.get("name", "unknown"),
            step.get("status", "UNKNOWN"),
            step.get("safety_class", "unknown"),
            "yes" if step.get("sends_uinput") else "no",
            ", ".join(step.get("unmet_requires_pass", [])) or "none",
            str(step.get("summary", "")).replace("|", "\\|"),
        ))
    lines.extend([
        "",
        "## Evidence",
        "",
        "| Artifact | Path |",
        "| --- | --- |",
    ])
    for key, value in report.evidence_paths.items():
        lines.append("| %s | `%s` |" % (key, value or "none"))
    lines.extend([
        "",
        "## Metrics",
        "",
        "```json",
        json.dumps(report.metrics, indent=2, sort_keys=True),
        "```",
        "",
    ])
    return "\n".join(lines)


def write_reports(
    report: SandboxResumeRunnerReport,
    log_dir: Path,
) -> tuple[Path, Path]:
    slug = "%s-%s" % (timestamp_slug(), report.run_id)
    json_path = log_dir / ("%s-sandbox-resume-runner.json" % slug)
    md_path = log_dir / ("%s-sandbox-resume-runner.md" % slug)
    report.evidence_paths["resume_runner_json"] = str(json_path)
    report.evidence_paths["resume_runner_md"] = str(md_path)
    json_path.write_text(json.dumps(report.as_dict(), indent=2, sort_keys=True) + "\n",
                         encoding="utf-8")
    md_path.write_text(build_markdown(report), encoding="utf-8")
    return json_path, md_path


def run_resume(
    *,
    run_id: str,
    log_dir: Path,
    window_title: str | None,
    bbox_file: Path,
    window_exact: bool,
    window_case_sensitive: bool,
    window_min_width: int,
    window_min_height: int,
    require_live_input: bool,
    require_live_follow: bool,
    execute_safe: bool,
    execute_live_input: bool,
    ack_live_input: bool,
    max_commands: int,
    timeout_s: float,
    axis_sweep_axes: tuple[str, ...] = ("yaw", "pitch"),
    axis_expected_shifts: dict[str, tuple[str, int]] | None = None,
    axis_min_shift_px: float = 0.5,
    require_fresh_evidence: bool = False,
    evidence_stale_after_s: float = DEFAULT_EVIDENCE_STALE_AFTER_S,
) -> SandboxResumeRunnerReport:
    log_dir.mkdir(parents=True, exist_ok=True)
    started_at = time.strftime("%Y-%m-%dT%H:%M:%S%z", time.localtime())
    status_report = build_status_report(
        run_id="%s-status" % run_id,
        log_dir=log_dir,
        window_title=window_title,
        bbox_file=bbox_file,
        window_exact=window_exact,
        window_case_sensitive=window_case_sensitive,
        window_min_width=window_min_width,
        window_min_height=window_min_height,
        require_live_input=require_live_input,
        require_live_follow=require_live_follow,
        require_fresh_evidence=require_fresh_evidence,
        evidence_stale_after_s=evidence_stale_after_s,
        axis_sweep_axes=axis_sweep_axes,
        axis_expected_shifts=axis_expected_shifts,
        axis_min_shift_px=axis_min_shift_px,
    )
    status_json, status_md = write_status_reports(status_report, log_dir)
    steps: list[dict[str, Any]] = []
    executed_count = 0
    item_statuses = status_by_name(status_report.items)
    for command in status_report.resume_commands:
        step, executed_count = evaluate_resume_command(
            command,
            item_statuses=item_statuses,
            execute_safe=execute_safe,
            execute_live_input=execute_live_input,
            ack_live_input=ack_live_input,
            executed_count=executed_count,
            max_commands=max_commands,
            timeout_s=timeout_s,
        )
        steps.append(step)
    post_status_report: SandboxStatusReport | None = None
    evidence_paths={
        "status_report_json": str(status_json),
        "status_report_md": str(status_md),
        "post_status_report_json": None,
        "post_status_report_md": None,
    }
    if executed_count > 0:
        post_status_report = build_status_report(
            run_id="%s-post-status" % run_id,
            log_dir=log_dir,
            window_title=window_title,
            bbox_file=bbox_file,
            window_exact=window_exact,
            window_case_sensitive=window_case_sensitive,
            window_min_width=window_min_width,
            window_min_height=window_min_height,
            require_live_input=require_live_input,
            require_live_follow=require_live_follow,
            require_fresh_evidence=require_fresh_evidence,
            evidence_stale_after_s=evidence_stale_after_s,
            axis_sweep_axes=axis_sweep_axes,
            axis_expected_shifts=axis_expected_shifts,
            axis_min_shift_px=axis_min_shift_px,
        )
        post_status_json, post_status_md = write_status_reports(post_status_report, log_dir)
        evidence_paths["post_status_report_json"] = str(post_status_json)
        evidence_paths["post_status_report_md"] = str(post_status_md)
    status = decide_resume_status(
        status_report=status_report,
        post_status_report=post_status_report,
        steps=steps,
    )
    resume_decision = build_resume_decision(
        status=status,
        status_report=status_report,
        steps=steps,
        executed_count=executed_count,
    )
    resume_step_summary = build_step_summary(steps)
    return SandboxResumeRunnerReport(
        run_id=run_id,
        started_at=started_at,
        status=status,
        summary=resume_summary(status),
        steps=steps,
        evidence_paths=evidence_paths,
        metrics={
            "window_title": window_title,
            "bbox_file": str(bbox_file),
            "axis_sweep_axes": list(axis_sweep_axes),
            "axis_expected_shifts": {
                axis: "%s%s" % ("+" if sign > 0 else "-", component)
                for axis, (component, sign) in (axis_expected_shifts or {}).items()
            } or None,
            "axis_min_shift_px": axis_min_shift_px if axis_expected_shifts else None,
            "require_fresh_evidence": require_fresh_evidence,
            "evidence_stale_after_s": evidence_stale_after_s,
            "execute_safe": execute_safe,
            "execute_live_input": execute_live_input,
            "ack_live_input": ack_live_input,
            "max_commands": max_commands,
            "executed_count": executed_count,
            "resume_decision": resume_decision,
            "resume_step_summary": resume_step_summary,
            "status_report": status_report.as_dict(),
            "post_status_report": (
                post_status_report.as_dict() if post_status_report is not None else None
            ),
            "scope": (
                "resume runner; live input requires --execute-live-input "
                "and --ack-live-input"
            ),
        },
    )


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-id", default="sandbox-resume")
    parser.add_argument("--log-dir", type=Path, default=DEFAULT_LOG_DIR)
    parser.add_argument("--window-title", default="pr0p")
    parser.add_argument("--bbox-file", type=Path, default=DEFAULT_BBOX_FILE)
    parser.add_argument("--window-exact", action="store_true")
    parser.add_argument("--window-case-sensitive", action="store_true")
    parser.add_argument("--window-min-width", type=int, default=32)
    parser.add_argument("--window-min-height", type=int, default=32)
    parser.add_argument("--require-live-input", action="store_true")
    parser.add_argument("--require-live-follow", action="store_true")
    parser.add_argument("--axis-sweep-axes", default="yaw,pitch",
                        help="Axes used in generated live-input readiness commands")
    parser.add_argument("--axis-expected-shifts", default="",
                        help="Optional signed visual shift checks, e.g. yaw:+x,pitch:-y")
    parser.add_argument("--axis-min-shift-px", type=float, default=0.5)
    parser.add_argument(
        "--require-fresh-evidence",
        action="store_true",
        help="Require required evidence files to be fresh before reporting READY",
    )
    parser.add_argument(
        "--evidence-stale-after-s",
        type=float,
        default=DEFAULT_EVIDENCE_STALE_AFTER_S,
        help="Evidence freshness threshold used with --require-fresh-evidence",
    )
    parser.add_argument("--execute-safe", action="store_true",
                        help="Run non-uinput, non-edit resume commands")
    parser.add_argument("--execute-live-input", action="store_true",
                        help="Allow live input resume commands after explicit ack")
    parser.add_argument("--ack-live-input", action="store_true",
                        help="Required together with --execute-live-input")
    parser.add_argument("--max-commands", type=int, default=1)
    parser.add_argument("--timeout", type=float, default=30.0)
    parser.add_argument("--json", action="store_true")
    args = parser.parse_args(argv)
    if args.window_min_width <= 0 or args.window_min_height <= 0:
        parser.error("--window-min-width/--window-min-height must be positive")
    try:
        args.axis_sweep_axes = normalize_axes(args.axis_sweep_axes)
    except ValueError as exc:
        parser.error(str(exc))
    try:
        args.axis_expected_shifts = parse_axis_shift_expectations(
            args.axis_expected_shifts
        )
    except ValueError as exc:
        parser.error(str(exc))
    if args.axis_min_shift_px < 0:
        parser.error("--axis-min-shift-px must be non-negative")
    if args.evidence_stale_after_s < 0:
        parser.error("--evidence-stale-after-s must be non-negative")
    unknown_expected = [
        axis for axis in args.axis_expected_shifts
        if axis not in args.axis_sweep_axes
    ]
    if unknown_expected:
        parser.error(
            "--axis-expected-shifts contains unselected axis/axes: %s"
            % ",".join(sorted(unknown_expected))
        )
    if args.max_commands <= 0:
        parser.error("--max-commands must be positive")
    if args.timeout <= 0:
        parser.error("--timeout must be positive")
    if args.execute_live_input and not args.ack_live_input:
        parser.error("--execute-live-input requires --ack-live-input")
    return args


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    log_dir = resolve_log_dir(str(args.log_dir), repo_root=REPO_ROOT)
    report = run_resume(
        run_id=args.run_id,
        log_dir=log_dir,
        window_title=args.window_title,
        bbox_file=args.bbox_file,
        window_exact=args.window_exact,
        window_case_sensitive=args.window_case_sensitive,
        window_min_width=args.window_min_width,
        window_min_height=args.window_min_height,
        require_live_input=args.require_live_input,
        require_live_follow=args.require_live_follow,
        axis_sweep_axes=args.axis_sweep_axes,
        axis_expected_shifts=args.axis_expected_shifts,
        axis_min_shift_px=args.axis_min_shift_px,
        require_fresh_evidence=args.require_fresh_evidence,
        evidence_stale_after_s=args.evidence_stale_after_s,
        execute_safe=args.execute_safe,
        execute_live_input=args.execute_live_input,
        ack_live_input=args.ack_live_input,
        max_commands=args.max_commands,
        timeout_s=args.timeout,
    )
    json_path, md_path = write_reports(report, log_dir)
    if args.json:
        data = report.as_dict()
        print(json.dumps(data, indent=2, sort_keys=True))
    else:
        print("sandbox-resume-runner %s report=%s summary=%s" % (
            report.status,
            json_path,
            md_path,
        ))
    if report.status in {SANDBOX_RESUME_READY, SANDBOX_RESUME_PROGRESS}:
        return 0
    if report.status == WAITING:
        return 2
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
