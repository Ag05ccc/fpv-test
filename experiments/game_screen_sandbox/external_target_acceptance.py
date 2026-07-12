#!/usr/bin/env python3
"""Status-only acceptance gate for a real external game/sim target window."""

from __future__ import annotations

import argparse
import json
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

from latency_probe import normalize_axes, parse_axis_shift_expectations  # noqa: E402
from isolation_audit import (  # noqa: E402
    PASS as AUDIT_PASS,
    audit_process_delta,
    process_snapshot,
)
from sandbox_status_report import (  # noqa: E402
    DEFAULT_BBOX_FILE,
    DEFAULT_EVIDENCE_STALE_AFTER_S,
    DEFAULT_LOG_DIR,
    PASS as STATUS_PASS,
    READY,
    REJECT,
    WAITING,
    SandboxStatusReport,
    build_status_report,
    write_reports as write_status_reports,
)
from sitl_log import resolve_log_dir, timestamp_slug  # noqa: E402


EXTERNAL_TARGET_READY = "EXTERNAL_TARGET_READY"


@dataclass
class ExternalTargetAcceptanceReport:
    run_id: str
    started_at: str
    status: str
    summary: str
    status_report_json: str | None = None
    status_report_md: str | None = None
    next_actions: list[str] = field(default_factory=list)
    evidence_paths: dict[str, str | None] = field(default_factory=dict)
    metrics: dict[str, Any] = field(default_factory=dict)

    def as_dict(self) -> dict[str, Any]:
        return {
            "run_id": self.run_id,
            "started_at": self.started_at,
            "status": self.status,
            "summary": self.summary,
            "status_report_json": self.status_report_json,
            "status_report_md": self.status_report_md,
            "next_actions": self.next_actions,
            "evidence_paths": self.evidence_paths,
            "metrics": self.metrics,
        }


def acceptance_mode_requirements(mode: str) -> tuple[bool, bool]:
    if mode == "dry-run":
        return False, False
    if mode == "live-input":
        return True, False
    if mode == "live-follow":
        return False, True
    raise ValueError("unsupported acceptance mode: %s" % mode)


def acceptance_status_from_status_report(status_report: SandboxStatusReport) -> str:
    if status_report.status == READY:
        return EXTERNAL_TARGET_READY
    if status_report.status == REJECT:
        return REJECT
    return WAITING


def acceptance_summary(status: str, mode: str) -> str:
    if status == EXTERNAL_TARGET_READY:
        return "external game/sim target is accepted for %s" % mode
    if status == REJECT:
        return "external game/sim target acceptance has a blocking failure"
    return "external game/sim target acceptance is waiting for stronger evidence"


def command_queue_entry(
    command: dict[str, Any],
    *,
    item_statuses: dict[str, str],
) -> dict[str, Any]:
    requires_pass = list(command.get("requires_pass", []))
    unmet_requires_pass = [
        "%s=%s" % (name, item_statuses.get(name, "UNKNOWN"))
        for name in requires_pass
        if item_statuses.get(name) != STATUS_PASS
    ]
    if unmet_requires_pass:
        command_state = "BLOCKED"
    elif command.get("requires_edit"):
        command_state = "MANUAL_EDIT"
    elif command.get("requires_user_interaction"):
        command_state = "MANUAL_UI"
    elif command.get("sends_uinput") or command.get("requires_ack_live_input"):
        command_state = "ACK_REQUIRED"
    else:
        command_state = "AVAILABLE"
    return {
        "name": command.get("name", "unknown"),
        "command_state": command_state,
        "purpose": command.get("purpose"),
        "safety_class": command.get("safety_class", "unknown"),
        "sends_uinput": bool(command.get("sends_uinput")),
        "requires_ack_live_input": bool(command.get("requires_ack_live_input")),
        "requires_edit": bool(command.get("requires_edit")),
        "requires_user_interaction": bool(command.get("requires_user_interaction")),
        "requires_pass": requires_pass,
        "unmet_requires_pass": unmet_requires_pass,
        "expected_status": command.get("expected_status"),
        "unblocks": list(command.get("unblocks", [])),
        "command": command.get("command", ""),
    }


def build_operator_command_queue(
    status_report: SandboxStatusReport,
) -> list[dict[str, Any]]:
    item_statuses = {item.name: item.status for item in status_report.items}
    return [
        command_queue_entry(command, item_statuses=item_statuses)
        for command in status_report.resume_commands
        if command.get("name") != "status"
    ]


def build_operator_command_summary(
    operator_command_queue: list[dict[str, Any]],
) -> dict[str, Any]:
    state_counts: dict[str, int] = {}
    for command in operator_command_queue:
        state = str(command.get("command_state", "UNKNOWN"))
        state_counts[state] = state_counts.get(state, 0) + 1
    return {
        "total_count": len(operator_command_queue),
        "state_counts": state_counts,
        "sends_uinput_count": sum(
            1 for command in operator_command_queue
            if command.get("sends_uinput")
        ),
        "first_available_command": next(
            (
                command for command in operator_command_queue
                if command.get("command_state") == "AVAILABLE"
            ),
            None,
        ),
        "first_blocked_command": next(
            (
                command for command in operator_command_queue
                if command.get("command_state") == "BLOCKED"
            ),
            None,
        ),
        "first_ack_required_command": next(
            (
                command for command in operator_command_queue
                if command.get("command_state") == "ACK_REQUIRED"
            ),
            None,
        ),
    }


def build_acceptance_decision(
    *,
    status: str,
    mode: str,
    status_report: SandboxStatusReport,
    isolation_status: str,
    next_actions: list[str],
    operator_command_queue: list[dict[str, Any]],
    operator_command_summary: dict[str, Any],
) -> dict[str, Any]:
    status_metrics = status_report.metrics
    first_blocking_stage = status_metrics.get("first_blocking_stage")
    freshness_gate = status_metrics.get("freshness_gate", {})
    return {
        "decision": status,
        "acceptance_mode": mode,
        "status_report_status": status_report.status,
        "isolation_status": isolation_status,
        "freshness_gate_status": (
            freshness_gate.get("status")
            if isinstance(freshness_gate, dict) else
            None
        ),
        "first_blocking_stage": first_blocking_stage,
        "next_operator_action": next_actions[0] if next_actions else None,
        "next_operator_command": (
            operator_command_queue[0] if operator_command_queue else None
        ),
        "first_available_command": operator_command_summary.get(
            "first_available_command"
        ),
        "first_blocked_command": operator_command_summary.get(
            "first_blocked_command"
        ),
        "first_ack_required_command": operator_command_summary.get(
            "first_ack_required_command"
        ),
        "required_fresh_evidence": (
            freshness_gate.get("required_keys", [])
            if isinstance(freshness_gate, dict) else
            []
        ),
        "stale_evidence": (
            freshness_gate.get("stale_keys", [])
            if isinstance(freshness_gate, dict) else
            []
        ),
        "missing_evidence": (
            freshness_gate.get("missing_keys", [])
            if isinstance(freshness_gate, dict) else
            []
        ),
    }


def build_markdown(report: ExternalTargetAcceptanceReport) -> str:
    status_report = report.metrics.get("status_report", {})
    status_metrics = (
        status_report.get("metrics", {}) if isinstance(status_report, dict) else {}
    )
    ladder = status_metrics.get("readiness_ladder", [])
    freshness_gate = status_metrics.get("freshness_gate", {})
    isolation_audit = report.metrics.get("isolation_audit", {})
    acceptance_decision = report.metrics.get("acceptance_decision", {})
    operator_command_queue = report.metrics.get("operator_command_queue", [])
    operator_command_summary = report.metrics.get("operator_command_summary", {})
    first_stage = (
        acceptance_decision.get("first_blocking_stage")
        if isinstance(acceptance_decision, dict) else
        None
    )
    lines = [
        "# External Target Acceptance",
        "",
        "Run: `%s`" % report.run_id,
        "Started: `%s`" % report.started_at,
        "Status: `%s`" % report.status,
        "",
        report.summary,
        "",
        "## Acceptance Decision",
        "",
        "Next operator action: `%s`" % (
            acceptance_decision.get("next_operator_action")
            if isinstance(acceptance_decision, dict) else
            None
        ),
        "",
        "First blocking stage: `%s`" % (
            first_stage.get("name")
            if isinstance(first_stage, dict) else
            "none"
        ),
        "",
        "Next operator command: `%s`" % (
            acceptance_decision.get("next_operator_command", {}).get("name")
            if (
                isinstance(acceptance_decision, dict)
                and isinstance(acceptance_decision.get("next_operator_command"), dict)
            ) else
            "none"
        ),
        "",
        "Command states: `%s`" % json.dumps(
            operator_command_summary.get("state_counts", {})
            if isinstance(operator_command_summary, dict) else
            {},
            sort_keys=True,
        ),
        "",
        "## Scope",
        "",
        "- Status-only: does not launch Gazebo, Betaflight, simulator, capture, or uinput.",
        "- Accepts only current evidence for the selected external game/sim window.",
        "",
        "## Next Actions",
        "",
    ]
    if report.next_actions:
        lines.extend("- %s" % action for action in report.next_actions)
    else:
        lines.append("- no next action")
    lines.extend([
        "",
        "## Operator Command Queue",
        "",
        "| Name | State | Safety | Sends uinput | Requires | Blocked by | Command |",
        "| --- | --- | --- | --- | --- | --- | --- |",
    ])
    for command in operator_command_queue if isinstance(operator_command_queue, list) else []:
        if not isinstance(command, dict):
            continue
        requirements = ", ".join(command.get("requires_pass", []))
        if command.get("requires_ack_live_input"):
            requirements = (requirements + ", " if requirements else "") + "ack-live-input"
        if command.get("requires_edit"):
            requirements = (requirements + ", " if requirements else "") + "manual-edit"
        if command.get("requires_user_interaction"):
            requirements = (requirements + ", " if requirements else "") + "manual-ui"
        lines.append("| %s | %s | %s | %s | %s | %s | `%s` |" % (
            command.get("name", "unknown"),
            command.get("command_state", "UNKNOWN"),
            command.get("safety_class", "unknown"),
            "yes" if command.get("sends_uinput") else "no",
            requirements or "none",
            ", ".join(command.get("unmet_requires_pass", [])) or "none",
            command.get("command", ""),
        ))
    lines.extend([
        "",
        "## Readiness Ladder",
        "",
        "| Stage | Name | Status | Blocking |",
        "| --- | --- | --- | --- |",
    ])
    for stage in ladder if isinstance(ladder, list) else []:
        if not isinstance(stage, dict):
            continue
        blocking = (
            ", ".join(stage.get("unmet_requires_pass", []))
            or ", ".join(stage.get("unmet_requires_any_pass", []))
            or "none"
        )
        lines.append("| %s | %s | %s | %s |" % (
            stage.get("stage", "unknown"),
            stage.get("name", "unknown"),
            stage.get("status", "UNKNOWN"),
            blocking.replace("|", "\\|"),
        ))
    lines.extend([
        "",
        "## Freshness Gate",
        "",
        "Status: `%s`" % (
            freshness_gate.get("status", "UNKNOWN")
            if isinstance(freshness_gate, dict) else
            "UNKNOWN"
        ),
        "",
        "Required keys: `%s`" % ", ".join(
            freshness_gate.get("required_keys", [])
            if isinstance(freshness_gate, dict) else
            []
        ),
        "",
        "## Process Boundary",
        "",
        "Status: `%s`" % (
            isolation_audit.get("status", "UNKNOWN")
            if isinstance(isolation_audit, dict) else
            "UNKNOWN"
        ),
        "",
        (
            isolation_audit.get("summary", "no process-boundary summary")
            if isinstance(isolation_audit, dict) else
            "no process-boundary summary"
        ),
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
    report: ExternalTargetAcceptanceReport,
    log_dir: Path,
) -> tuple[Path, Path]:
    log_dir.mkdir(parents=True, exist_ok=True)
    slug = "%s-%s" % (timestamp_slug(), report.run_id)
    json_path = log_dir / ("%s-external-target-acceptance.json" % slug)
    md_path = log_dir / ("%s-external-target-acceptance.md" % slug)
    report.evidence_paths["external_target_acceptance_json"] = str(json_path)
    report.evidence_paths["external_target_acceptance_md"] = str(md_path)
    json_path.write_text(json.dumps(report.as_dict(), indent=2, sort_keys=True) + "\n",
                         encoding="utf-8")
    md_path.write_text(build_markdown(report), encoding="utf-8")
    return json_path, md_path


def run_external_acceptance(
    *,
    run_id: str,
    log_dir: Path,
    window_title: str | None,
    bbox_file: Path,
    mode: str,
    window_exact: bool,
    window_case_sensitive: bool,
    window_min_width: int,
    window_min_height: int,
    require_fresh_evidence: bool,
    evidence_stale_after_s: float,
    axis_sweep_axes: tuple[str, ...],
    axis_expected_shifts: dict[str, tuple[str, int]] | None,
    axis_min_shift_px: float,
) -> ExternalTargetAcceptanceReport:
    log_dir.mkdir(parents=True, exist_ok=True)
    forbidden_processes_before = process_snapshot()
    require_live_input, require_live_follow = acceptance_mode_requirements(mode)
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
    isolation_audit = audit_process_delta(
        forbidden_processes_before,
        process_snapshot(),
    )
    status = acceptance_status_from_status_report(status_report)
    if isolation_audit.status != AUDIT_PASS:
        status = REJECT
    started_at = time.strftime("%Y-%m-%dT%H:%M:%S%z", time.localtime())
    evidence_paths = {
        "status_report_json": str(status_json),
        "status_report_md": str(status_md),
    }
    next_actions = (
        ["stop forbidden Gazebo/Betaflight processes before external acceptance"]
        if isolation_audit.status != AUDIT_PASS else
        list(status_report.next_actions)
    )
    operator_command_queue = build_operator_command_queue(status_report)
    operator_command_summary = build_operator_command_summary(operator_command_queue)
    acceptance_decision = build_acceptance_decision(
        status=status,
        mode=mode,
        status_report=status_report,
        isolation_status=isolation_audit.status,
        next_actions=next_actions,
        operator_command_queue=operator_command_queue,
        operator_command_summary=operator_command_summary,
    )
    return ExternalTargetAcceptanceReport(
        run_id=run_id,
        started_at=started_at,
        status=status,
        summary=acceptance_summary(status, mode),
        status_report_json=str(status_json),
        status_report_md=str(status_md),
        next_actions=next_actions,
        evidence_paths=evidence_paths,
        metrics={
            "acceptance_decision": acceptance_decision,
            "acceptance_mode": mode,
            "operator_command_queue": operator_command_queue,
            "operator_command_summary": operator_command_summary,
            "window_title": window_title,
            "bbox_file": str(bbox_file),
            "require_live_input": require_live_input,
            "require_live_follow": require_live_follow,
            "require_fresh_evidence": require_fresh_evidence,
            "evidence_stale_after_s": evidence_stale_after_s,
            "axis_sweep_axes": list(axis_sweep_axes),
            "axis_expected_shifts": {
                axis: "%s%s" % ("+" if sign > 0 else "-", component)
                for axis, (component, sign) in (axis_expected_shifts or {}).items()
            } or None,
            "axis_min_shift_px": axis_min_shift_px if axis_expected_shifts else None,
            "isolation_status": isolation_audit.status,
            "isolation_audit": isolation_audit.as_dict(),
            "status_report": status_report.as_dict(),
            "scope": (
                "status-only external target acceptance; does not launch Gazebo, "
                "Betaflight, simulator, capture, or uinput"
            ),
        },
    )


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-id", default="external-target-acceptance")
    parser.add_argument("--log-dir", type=Path, default=DEFAULT_LOG_DIR)
    parser.add_argument("--window-title", default="pr0p")
    parser.add_argument("--bbox-file", type=Path, default=DEFAULT_BBOX_FILE)
    parser.add_argument(
        "--mode",
        choices=("dry-run", "live-input", "live-follow"),
        default="live-follow",
        help="Required external evidence level; live-follow is the project target",
    )
    parser.add_argument("--window-exact", action="store_true")
    parser.add_argument("--window-case-sensitive", action="store_true")
    parser.add_argument("--window-min-width", type=int, default=32)
    parser.add_argument("--window-min-height", type=int, default=32)
    parser.add_argument("--allow-stale-evidence", action="store_true")
    parser.add_argument(
        "--evidence-stale-after-s",
        type=float,
        default=DEFAULT_EVIDENCE_STALE_AFTER_S,
    )
    parser.add_argument("--axis-sweep-axes", default="yaw,pitch")
    parser.add_argument("--axis-expected-shifts", default="")
    parser.add_argument("--axis-min-shift-px", type=float, default=0.5)
    parser.add_argument("--json", action="store_true")
    args = parser.parse_args(argv)
    if args.window_min_width <= 0 or args.window_min_height <= 0:
        parser.error("--window-min-width/--window-min-height must be positive")
    if args.evidence_stale_after_s < 0:
        parser.error("--evidence-stale-after-s must be non-negative")
    if args.axis_min_shift_px < 0:
        parser.error("--axis-min-shift-px must be non-negative")
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
    unknown_expected = [
        axis for axis in args.axis_expected_shifts
        if axis not in args.axis_sweep_axes
    ]
    if unknown_expected:
        parser.error(
            "--axis-expected-shifts contains unselected axis/axes: %s"
            % ",".join(sorted(unknown_expected))
        )
    return args


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    log_dir = resolve_log_dir(str(args.log_dir), repo_root=REPO_ROOT)
    report = run_external_acceptance(
        run_id=args.run_id,
        log_dir=log_dir,
        window_title=args.window_title,
        bbox_file=args.bbox_file,
        mode=args.mode,
        window_exact=args.window_exact,
        window_case_sensitive=args.window_case_sensitive,
        window_min_width=args.window_min_width,
        window_min_height=args.window_min_height,
        require_fresh_evidence=not args.allow_stale_evidence,
        evidence_stale_after_s=args.evidence_stale_after_s,
        axis_sweep_axes=args.axis_sweep_axes,
        axis_expected_shifts=args.axis_expected_shifts,
        axis_min_shift_px=args.axis_min_shift_px,
    )
    json_path, md_path = write_reports(report, log_dir)
    if args.json:
        print(json.dumps(report.as_dict(), indent=2, sort_keys=True))
    else:
        print("external-target-acceptance %s report=%s summary=%s" % (
            report.status,
            json_path,
            md_path,
        ))
        for action in report.next_actions:
            print("next: %s" % action)
    if report.status == EXTERNAL_TARGET_READY:
        return 0
    if report.status == WAITING:
        return 2
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
