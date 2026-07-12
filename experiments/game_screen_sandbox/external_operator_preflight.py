#!/usr/bin/env python3
"""Status-only operator preflight for a real external game/sim window."""

from __future__ import annotations

import argparse
import json
import os
import shlex
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

from external_target_acceptance import (  # noqa: E402
    build_operator_command_queue,
    build_operator_command_summary,
)
from latency_probe import normalize_axes, parse_axis_shift_expectations  # noqa: E402
from sandbox_status_report import (  # noqa: E402
    DEFAULT_BBOX_FILE,
    DEFAULT_EVIDENCE_STALE_AFTER_S,
    DEFAULT_LOG_DIR,
    PASS,
    READY,
    REJECT,
    WAITING,
    SandboxStatusReport,
    build_status_report,
    write_reports as write_status_reports,
)
from sitl_log import resolve_log_dir, timestamp_slug  # noqa: E402
from window_safety import tooling_window_rejection_reason  # noqa: E402
from x11_window import find_window_by_title, list_x11_windows, visible_windows  # noqa: E402


OPERATOR_PREFLIGHT_READY = "OPERATOR_PREFLIGHT_READY"


@dataclass
class ExternalOperatorPreflightReport:
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


def preflight_status_from_status_report(status_report: SandboxStatusReport) -> str:
    if status_report.status == READY:
        return OPERATOR_PREFLIGHT_READY
    if status_report.status == REJECT:
        return REJECT
    return WAITING


def preflight_summary(status: str) -> str:
    if status == OPERATOR_PREFLIGHT_READY:
        return "external operator preflight is ready"
    if status == REJECT:
        return "external operator preflight found a blocking failure"
    return "external operator preflight is waiting for setup evidence"


def step_brief(step: dict[str, Any] | None) -> dict[str, Any] | None:
    if not isinstance(step, dict):
        return None
    return {
        "name": step.get("name"),
        "command_state": step.get("command_state"),
        "safety_class": step.get("safety_class"),
        "sends_uinput": bool(step.get("sends_uinput")),
        "requires_pass": list(step.get("requires_pass", [])),
        "unmet_requires_pass": list(step.get("unmet_requires_pass", [])),
        "command": step.get("command"),
    }


def shell_command(parts: list[str | Path | float | int]) -> str:
    return " ".join(shlex.quote(str(part)) for part in parts)


def format_axes(axes: tuple[str, ...]) -> str:
    return ",".join(axes)


def format_expected_shifts(
    expected_shifts: dict[str, tuple[str, int]] | None,
) -> str:
    if not expected_shifts:
        return ""
    return ",".join(
        "%s:%s%s" % (axis, "+" if sign > 0 else "-", component)
        for axis, (component, sign) in sorted(expected_shifts.items())
    )


def expected_shift_command_args(
    expected_shifts: dict[str, tuple[str, int]] | None,
    min_shift_px: float,
) -> list[str | float]:
    if not expected_shifts:
        return []
    return [
        "--axis-expected-shifts",
        format_expected_shifts(expected_shifts),
        "--axis-min-shift-px",
        min_shift_px,
    ]


def operator_preflight_command_for_title(
    *,
    title: str,
    bbox_file: Path,
    require_live_input: bool,
    require_live_follow: bool,
    require_fresh_evidence: bool,
    evidence_stale_after_s: float,
    axis_sweep_axes: tuple[str, ...],
    axis_expected_shifts: dict[str, tuple[str, int]] | None,
    axis_min_shift_px: float,
    max_window_candidates: int,
) -> str:
    return shell_command([
        "fpv_env/bin/python",
        "experiments/game_screen_sandbox/external_operator_preflight.py",
        "--window-title",
        title,
        "--window-exact",
        "--bbox-file",
        bbox_file,
        *(["--require-live-follow"] if require_live_follow else []),
        *(["--require-live-input"] if require_live_input else []),
        *(
            [
                "--require-fresh-evidence",
                "--evidence-stale-after-s",
                evidence_stale_after_s,
            ]
            if require_fresh_evidence else
            []
        ),
        *(
            ["--axis-sweep-axes", format_axes(axis_sweep_axes)]
            if axis_sweep_axes != ("yaw", "pitch") else
            []
        ),
        *expected_shift_command_args(axis_expected_shifts, axis_min_shift_px),
        "--max-window-candidates",
        max_window_candidates,
    ])


def bbox_frame_command_for_title(*, title: str, bbox_file: Path) -> str:
    return shell_command([
        "fpv_env/bin/python",
        "experiments/game_screen_sandbox/bbox_tool.py",
        "--window-title",
        title,
        "--window-exact",
        "--capture-backend",
        "ffmpeg",
        "--frame-path",
        bbox_file.with_name("%s-frame.png" % bbox_file.stem),
    ])


def bbox_frame_command_for_region(
    *,
    region: dict[str, int],
    bbox_file: Path,
) -> str:
    return shell_command([
        "fpv_env/bin/python",
        "experiments/game_screen_sandbox/bbox_tool.py",
        "--region",
        "%d,%d,%d,%d" % (
            region["left"],
            region["top"],
            region["width"],
            region["height"],
        ),
        "--capture-backend",
        "ffmpeg",
        "--frame-path",
        bbox_file.with_name("%s-frame.png" % bbox_file.stem),
    ])


def bbox_write_command_for_region(
    *,
    region: dict[str, int],
    bbox_file: Path,
) -> str:
    return shell_command([
        "fpv_env/bin/python",
        "experiments/game_screen_sandbox/bbox_tool.py",
        "--region",
        "%d,%d,%d,%d" % (
            region["left"],
            region["top"],
            region["width"],
            region["height"],
        ),
        "--capture-backend",
        "ffmpeg",
        "--bbox",
        "X,Y,W,H",
        "--frame-path",
        bbox_file.with_name("%s-frame.png" % bbox_file.stem),
        "--report-path",
        bbox_file,
    ])


def bbox_interactive_command_for_region(
    *,
    region: dict[str, int],
    bbox_file: Path,
) -> str:
    return shell_command([
        "fpv_env/bin/python",
        "experiments/game_screen_sandbox/bbox_tool.py",
        "--region",
        "%d,%d,%d,%d" % (
            region["left"],
            region["top"],
            region["width"],
            region["height"],
        ),
        "--capture-backend",
        "ffmpeg",
        "--interactive-select",
        "--frame-path",
        bbox_file.with_name("%s-frame.png" % bbox_file.stem),
        "--report-path",
        bbox_file,
    ])


def dry_run_sequence_region_command_for_candidate(
    *,
    title: str,
    region: dict[str, int],
    bbox_file: Path,
) -> str:
    return shell_command([
        "fpv_env/bin/python",
        "experiments/game_screen_sandbox/external_dry_run_sequence.py",
        "--capture-region",
        "%d,%d,%d,%d" % (
            region["left"],
            region["top"],
            region["width"],
            region["height"],
        ),
        "--capture-window-title",
        title,
        "--window-exact",
        "--capture-backend",
        "ffmpeg",
        "--tracker-bbox-file",
        bbox_file,
        "--duration",
        2,
        "--hz",
        10,
        "--enable-pitch",
        "--desired-target-width",
        120,
        "--run-id",
        "external-region-dry-sequence",
    ])


def build_window_discovery(
    *,
    window_title: str | None,
    exact: bool,
    case_sensitive: bool,
    min_width: int,
    min_height: int,
    max_candidates: int,
    bbox_file: Path,
    require_live_input: bool,
    require_live_follow: bool,
    require_fresh_evidence: bool,
    evidence_stale_after_s: float,
    axis_sweep_axes: tuple[str, ...],
    axis_expected_shifts: dict[str, tuple[str, int]] | None,
    axis_min_shift_px: float,
) -> dict[str, Any]:
    if not window_title:
        return {
            "status": WAITING,
            "reason": "NO_WINDOW_TITLE",
            "target_title": window_title,
            "candidates": [],
        }
    if not os.environ.get("DISPLAY"):
        return {
            "status": WAITING,
            "reason": "DISPLAY_NOT_SET",
            "target_title": window_title,
            "candidates": [],
        }
    try:
        windows = list_x11_windows()
        visible = visible_windows(
            windows,
            min_width=min_width,
            min_height=min_height,
        )
        basic_candidate_pool = [
            window for window in visible
            if is_operator_window_candidate(window.as_dict())
        ]
        excluded_candidates = []
        candidate_pool = []
        for window in basic_candidate_pool:
            rejection_reason = tooling_window_rejection_reason(window.as_dict())
            if rejection_reason:
                excluded = window.as_dict()
                excluded["rejection_reason"] = rejection_reason
                excluded_candidates.append(excluded)
                continue
            candidate_pool.append(window)
        match = find_window_by_title(
            window_title,
            candidate_pool,
            exact=exact,
            case_sensitive=case_sensitive,
            min_width=min_width,
            min_height=min_height,
        )
    except Exception as exc:
        return {
            "status": WAITING,
            "reason": "WINDOW_DISCOVERY_ERROR",
            "target_title": window_title,
            "error": str(exc),
            "candidates": [],
        }
    match_id = match.window_id if match is not None else None
    ordered = sorted(
        candidate_pool,
        key=lambda window: (
            0 if window.window_id == match_id else 1,
            -window.area,
            window.title.lower(),
        ),
    )
    candidates = []
    for window in ordered[:max_candidates]:
        region = window.region()
        candidates.append({
            "window_id": window.window_id,
            "title": window.title,
            "class_text": window.class_text,
            "width": window.width,
            "height": window.height,
            "region": region,
            "matches_requested_title": window.window_id == match_id,
            "use_window_title_arg": window.title,
            "rerun_operator_preflight_command": operator_preflight_command_for_title(
                title=window.title,
                bbox_file=bbox_file,
                require_live_input=require_live_input,
                require_live_follow=require_live_follow,
                require_fresh_evidence=require_fresh_evidence,
                evidence_stale_after_s=evidence_stale_after_s,
                axis_sweep_axes=axis_sweep_axes,
                axis_expected_shifts=axis_expected_shifts,
                axis_min_shift_px=axis_min_shift_px,
                max_window_candidates=max_candidates,
            ),
            "capture_bbox_frame_command": bbox_frame_command_for_title(
                title=window.title,
                bbox_file=bbox_file,
            ),
            "capture_bbox_frame_region_command": bbox_frame_command_for_region(
                region=region,
                bbox_file=bbox_file,
            ),
            "write_bbox_file_region_command": bbox_write_command_for_region(
                region=region,
                bbox_file=bbox_file,
            ),
            "interactive_bbox_select_region_command": (
                bbox_interactive_command_for_region(
                    region=region,
                    bbox_file=bbox_file,
                )
            ),
            "dry_run_sequence_region_command": (
                dry_run_sequence_region_command_for_candidate(
                    title=window.title,
                    region=region,
                    bbox_file=bbox_file,
                )
            ),
        })
    return {
        "status": "MATCHED" if match is not None else WAITING,
        "reason": "MATCHED" if match is not None else "NO_MATCH",
        "target_title": window_title,
        "visible_count": len(visible),
        "basic_candidate_pool_count": len(basic_candidate_pool),
        "candidate_pool_count": len(candidate_pool),
        "excluded_candidate_count": len(excluded_candidates),
        "excluded_candidates": excluded_candidates,
        "candidate_count": len(candidates),
        "matched_window": match.as_dict() if match is not None else None,
        "candidates": candidates,
    }


def is_operator_window_candidate(window: dict[str, Any]) -> bool:
    title = str(window.get("title", "")).strip()
    class_text = str(window.get("class_text", "")).lower()
    lowered = title.lower()
    if not title:
        return False
    if lowered == "mutter guard window":
        return False
    if lowered.startswith("desktop icons"):
        return False
    if "gjs" in class_text and lowered.startswith("desktop"):
        return False
    return True


def status_items_by_name(status_report: SandboxStatusReport) -> dict[str, str]:
    return {item.name: item.status for item in status_report.items}


def candidate_action_for_status(
    *,
    candidate: dict[str, Any],
    candidate_index: int,
    item_statuses: dict[str, str],
) -> dict[str, Any]:
    if item_statuses.get("target_bbox") != PASS:
        next_step = "capture_bbox_frame_region"
        next_command = candidate.get("capture_bbox_frame_region_command")
        expected_status = WAITING
        safety_class = "capture_only"
        unblocks = ["target_bbox"]
    elif (
        item_statuses.get("external_window_preflight") != PASS
        or item_statuses.get("external_follow_session") != PASS
    ):
        next_step = "dry_run_sequence_region"
        next_command = candidate.get("dry_run_sequence_region_command")
        expected_status = "EXTERNAL_DRY_RUN_READY"
        safety_class = "dry_run_capture"
        unblocks = ["external_window_preflight", "external_follow_session"]
    else:
        next_step = "status_only"
        next_command = candidate.get("rerun_operator_preflight_command")
        expected_status = "READY_OR_WAITING_STATUS_REPORT"
        safety_class = "status_only"
        unblocks = ["operator_preflight_decision"]
    return {
        "candidate_index": candidate_index,
        "title": candidate.get("title"),
        "window_id": candidate.get("window_id"),
        "region": candidate.get("region"),
        "matches_requested_title": bool(candidate.get("matches_requested_title")),
        "next_step": next_step,
        "next_command": next_command,
        "expected_status": expected_status,
        "safety_class": safety_class,
        "sends_uinput": False,
        "write_bbox_file_region_command": (
            candidate.get("write_bbox_file_region_command")
        ),
        "interactive_bbox_select_region_command": (
            candidate.get("interactive_bbox_select_region_command")
        ),
        "requires_user_judgement": True,
        "unblocks": unblocks,
        "guard": (
            "Only run this command after confirming this candidate is the "
            "intended simulator/FPV view."
        ),
    }


def build_candidate_action_plan(
    *,
    status_report: SandboxStatusReport,
    window_discovery: dict[str, Any],
) -> list[dict[str, Any]]:
    candidates = (
        window_discovery.get("candidates", [])
        if isinstance(window_discovery, dict) else
        []
    )
    item_statuses = status_items_by_name(status_report)
    plan = []
    for index, candidate in enumerate(candidates, start=1):
        if not isinstance(candidate, dict):
            continue
        plan.append(candidate_action_for_status(
            candidate=candidate,
            candidate_index=index,
            item_statuses=item_statuses,
        ))
    return plan


def build_preflight_decision(
    *,
    status: str,
    status_report: SandboxStatusReport,
    operator_command_summary: dict[str, Any],
    window_discovery: dict[str, Any],
    candidate_action_plan: list[dict[str, Any]],
) -> dict[str, Any]:
    status_metrics = status_report.metrics
    first_blocking_stage = status_metrics.get("first_blocking_stage")
    freshness_gate = status_metrics.get("freshness_gate", {})
    first_available = operator_command_summary.get("first_available_command")
    first_blocked = operator_command_summary.get("first_blocked_command")
    first_ack = operator_command_summary.get("first_ack_required_command")
    recommended = first_available or first_blocked or first_ack
    if first_available is not None:
        command_gate = "safe_command_available"
    elif first_blocked is not None:
        command_gate = "blocked_by_dependency"
    elif first_ack is not None:
        command_gate = "live_ack_required"
    else:
        command_gate = "no_operator_command"
    recommended_candidate_action = (
        candidate_action_plan[0] if candidate_action_plan else None
    )
    return {
        "decision": status,
        "status_report_status": status_report.status,
        "freshness_gate_status": (
            freshness_gate.get("status")
            if isinstance(freshness_gate, dict) else
            None
        ),
        "readiness_stage": (
            first_blocking_stage.get("name")
            if isinstance(first_blocking_stage, dict) else
            "ready"
            if status == OPERATOR_PREFLIGHT_READY else
            "unknown"
        ),
        "first_blocking_stage": first_blocking_stage,
        "next_operator_action": (
            status_report.next_actions[0] if status_report.next_actions else None
        ),
        "next_action_candidates": list(status_report.next_actions[:3]),
        "command_gate": command_gate,
        "recommended_command": step_brief(recommended),
        "first_available_command": step_brief(first_available),
        "first_blocked_command": step_brief(first_blocked),
        "first_ack_required_command": step_brief(first_ack),
        "candidate_action_gate": (
            "manual_candidate_selection"
            if recommended_candidate_action is not None else
            "no_candidate_action"
        ),
        "recommended_candidate_action": recommended_candidate_action,
        "safe_to_execute_now": first_available is not None,
        "live_ack_required": first_available is None and first_ack is not None,
        "window_discovery_status": window_discovery.get("status"),
        "window_discovery_reason": window_discovery.get("reason"),
        "window_title_candidates": [
            candidate.get("title")
            for candidate in window_discovery.get("candidates", [])
            if isinstance(candidate, dict)
        ],
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
        "excluded_window_candidates": [
            {
                "title": candidate.get("title"),
                "window_id": candidate.get("window_id"),
                "rejection_reason": candidate.get("rejection_reason"),
            }
            for candidate in window_discovery.get("excluded_candidates", [])
            if isinstance(candidate, dict)
        ],
    }


def build_markdown(report: ExternalOperatorPreflightReport) -> str:
    decision = report.metrics.get("operator_preflight_decision", {})
    command_summary = report.metrics.get("operator_command_summary", {})
    queue = report.metrics.get("operator_command_queue", [])
    status_report = report.metrics.get("status_report", {})
    window_discovery = report.metrics.get("window_discovery", {})
    candidate_action_plan = report.metrics.get("candidate_action_plan", [])
    items = status_report.get("items", []) if isinstance(status_report, dict) else []
    lines = [
        "# External Operator Preflight",
        "",
        "Run: `%s`" % report.run_id,
        "Started: `%s`" % report.started_at,
        "Status: `%s`" % report.status,
        "",
        report.summary,
        "",
        "## Decision",
        "",
        "| Field | Value |",
        "| --- | --- |",
        "| readiness_stage | `%s` |" % decision.get("readiness_stage", "unknown"),
        "| command_gate | `%s` |" % decision.get("command_gate", "unknown"),
        "| next_operator_action | %s |" % (
            decision.get("next_operator_action") or "none"
        ),
        "| recommended_command | `%s` |" % (
            (decision.get("recommended_command") or {}).get("name") or "none"
        ),
        "| recommended_candidate_action | `%s` |" % (
            (decision.get("recommended_candidate_action") or {}).get("next_step")
            or "none"
        ),
        "| command_state_counts | `%s` |" % json.dumps(
            command_summary.get("state_counts", {})
            if isinstance(command_summary, dict) else
            {},
            sort_keys=True,
        ),
        "| window_discovery | `%s` |" % (
            window_discovery.get("reason")
            if isinstance(window_discovery, dict) else
            "unknown"
        ),
        "",
        "## Window Candidates",
        "",
        "| Title | Window | Size | Region | Match |",
        "| --- | --- | --- | --- | --- |",
    ]
    for candidate in (
        window_discovery.get("candidates", [])
        if isinstance(window_discovery, dict) else
        []
    ):
        if not isinstance(candidate, dict):
            continue
        region = candidate.get("region", {})
        lines.append("| %s | `%s` | %sx%s | %s,%s %sx%s | %s |" % (
            str(candidate.get("title", "")).replace("|", "\\|"),
            candidate.get("window_id", "unknown"),
            candidate.get("width", "?"),
            candidate.get("height", "?"),
            region.get("left", "?") if isinstance(region, dict) else "?",
            region.get("top", "?") if isinstance(region, dict) else "?",
            region.get("width", "?") if isinstance(region, dict) else "?",
            region.get("height", "?") if isinstance(region, dict) else "?",
            "yes" if candidate.get("matches_requested_title") else "no",
        ))
    if not (
        isinstance(window_discovery, dict)
        and window_discovery.get("candidates")
    ):
        lines.append("| none | `none` | - | - | no |")
    lines.extend([
        "",
        "## Excluded Window Candidates",
        "",
        "| Title | Window | Reason |",
        "| --- | --- | --- |",
    ])
    for candidate in (
        window_discovery.get("excluded_candidates", [])
        if isinstance(window_discovery, dict) else
        []
    ):
        if not isinstance(candidate, dict):
            continue
        lines.append("| %s | `%s` | `%s` |" % (
            str(candidate.get("title", "")).replace("|", "\\|"),
            candidate.get("window_id", "unknown"),
            candidate.get("rejection_reason", "unknown"),
        ))
    if not (
        isinstance(window_discovery, dict)
        and window_discovery.get("excluded_candidates")
    ):
        lines.append("| none | `none` | `none` |")
    lines.extend([
        "",
        "## Candidate Action Plan",
        "",
        "| # | Title | Next step | Expected | Sends uinput | Command |",
        "| --- | --- | --- | --- | --- | --- |",
    ])
    for action in candidate_action_plan if isinstance(candidate_action_plan, list) else []:
        if not isinstance(action, dict):
            continue
        lines.append("| %s | %s | `%s` | `%s` | %s | `%s` |" % (
            action.get("candidate_index", "?"),
            str(action.get("title", "")).replace("|", "\\|"),
            action.get("next_step", "unknown"),
            action.get("expected_status", "unknown"),
            "yes" if action.get("sends_uinput") else "no",
            action.get("next_command", ""),
        ))
    if not candidate_action_plan:
        lines.append("| none | none | `none` | `none` | no | `none` |")
    lines.extend([
        "",
        "## Window Candidate Commands",
        "",
    ])
    for index, candidate in enumerate(
        window_discovery.get("candidates", [])
        if isinstance(window_discovery, dict) else
        [],
        start=1,
    ):
        if not isinstance(candidate, dict):
            continue
        lines.extend([
            "%d. %s" % (
                index,
                str(candidate.get("title", "")).replace("\n", " "),
            ),
            "",
            "```bash",
            str(candidate.get("rerun_operator_preflight_command", "")),
            "```",
            "",
            "```bash",
            str(
                candidate.get("capture_bbox_frame_region_command")
                or candidate.get("capture_bbox_frame_command", "")
            ),
            "```",
            "",
            "```bash",
            str(candidate.get("dry_run_sequence_region_command", "")),
            "```",
            "",
        ])
    if not (
        isinstance(window_discovery, dict)
        and window_discovery.get("candidates")
    ):
        lines.append("No visible candidate window was available.")
    lines.extend([
        "",
        "## Status Items",
        "",
        "| Name | Status | Summary |",
        "| --- | --- | --- |",
    ])
    for item in items:
        if not isinstance(item, dict):
            continue
        lines.append("| %s | `%s` | %s |" % (
            item.get("name", "unknown"),
            item.get("status", "UNKNOWN"),
            str(item.get("summary", "")).replace("|", "\\|"),
        ))
    lines.extend([
        "",
        "## Operator Commands",
        "",
        "| Name | State | Safety | Sends uinput | Blocked by | Command |",
        "| --- | --- | --- | --- | --- | --- |",
    ])
    for command in queue if isinstance(queue, list) else []:
        if not isinstance(command, dict):
            continue
        lines.append("| %s | `%s` | %s | %s | %s | `%s` |" % (
            command.get("name", "unknown"),
            command.get("command_state", "UNKNOWN"),
            command.get("safety_class", "unknown"),
            "yes" if command.get("sends_uinput") else "no",
            ", ".join(command.get("unmet_requires_pass", [])) or "none",
            command.get("command", ""),
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
    report: ExternalOperatorPreflightReport,
    log_dir: Path,
) -> tuple[Path, Path]:
    slug = "%s-%s" % (timestamp_slug(), report.run_id)
    json_path = log_dir / ("%s-external-operator-preflight.json" % slug)
    md_path = log_dir / ("%s-external-operator-preflight.md" % slug)
    report.evidence_paths["operator_preflight_json"] = str(json_path)
    report.evidence_paths["operator_preflight_md"] = str(md_path)
    json_path.write_text(json.dumps(report.as_dict(), indent=2, sort_keys=True) + "\n",
                         encoding="utf-8")
    md_path.write_text(build_markdown(report), encoding="utf-8")
    return json_path, md_path


def run_operator_preflight(
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
    require_fresh_evidence: bool,
    evidence_stale_after_s: float,
    max_window_candidates: int = 8,
    axis_sweep_axes: tuple[str, ...] = ("yaw", "pitch"),
    axis_expected_shifts: dict[str, tuple[str, int]] | None = None,
    axis_min_shift_px: float = 0.5,
) -> ExternalOperatorPreflightReport:
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
    window_discovery = build_window_discovery(
        window_title=window_title,
        exact=window_exact,
        case_sensitive=window_case_sensitive,
        min_width=window_min_width,
        min_height=window_min_height,
        max_candidates=max_window_candidates,
        bbox_file=bbox_file,
        require_live_input=require_live_input,
        require_live_follow=require_live_follow,
        require_fresh_evidence=require_fresh_evidence,
        evidence_stale_after_s=evidence_stale_after_s,
        axis_sweep_axes=axis_sweep_axes,
        axis_expected_shifts=axis_expected_shifts,
        axis_min_shift_px=axis_min_shift_px,
    )
    operator_command_queue = build_operator_command_queue(status_report)
    operator_command_summary = build_operator_command_summary(operator_command_queue)
    candidate_action_plan = build_candidate_action_plan(
        status_report=status_report,
        window_discovery=window_discovery,
    )
    status = preflight_status_from_status_report(status_report)
    decision = build_preflight_decision(
        status=status,
        status_report=status_report,
        operator_command_summary=operator_command_summary,
        window_discovery=window_discovery,
        candidate_action_plan=candidate_action_plan,
    )
    return ExternalOperatorPreflightReport(
        run_id=run_id,
        started_at=started_at,
        status=status,
        summary=preflight_summary(status),
        status_report_json=str(status_json),
        status_report_md=str(status_md),
        next_actions=list(status_report.next_actions),
        evidence_paths={
            "status_report_json": str(status_json),
            "status_report_md": str(status_md),
        },
        metrics={
            "window_title": window_title,
            "bbox_file": str(bbox_file),
            "require_live_input": require_live_input,
            "require_live_follow": require_live_follow,
            "require_fresh_evidence": require_fresh_evidence,
            "evidence_stale_after_s": evidence_stale_after_s,
            "max_window_candidates": max_window_candidates,
            "axis_sweep_axes": list(axis_sweep_axes),
            "axis_expected_shifts": {
                axis: "%s%s" % ("+" if sign > 0 else "-", component)
                for axis, (component, sign) in (axis_expected_shifts or {}).items()
            } or None,
            "axis_min_shift_px": axis_min_shift_px if axis_expected_shifts else None,
            "operator_preflight_decision": decision,
            "window_discovery": window_discovery,
            "candidate_action_plan": candidate_action_plan,
            "operator_command_queue": operator_command_queue,
            "operator_command_summary": operator_command_summary,
            "status_report": status_report.as_dict(),
            "scope": (
                "operator preflight only; does not launch simulator, capture, "
                "Gazebo, Betaflight, or uinput"
            ),
        },
    )


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-id", default="external-operator-preflight")
    parser.add_argument("--log-dir", type=Path, default=DEFAULT_LOG_DIR)
    parser.add_argument("--window-title", default="pr0p")
    parser.add_argument("--bbox-file", type=Path, default=DEFAULT_BBOX_FILE)
    parser.add_argument("--window-exact", action="store_true")
    parser.add_argument("--window-case-sensitive", action="store_true")
    parser.add_argument("--window-min-width", type=int, default=32)
    parser.add_argument("--window-min-height", type=int, default=32)
    parser.add_argument("--require-live-input", action="store_true")
    parser.add_argument("--require-live-follow", action="store_true")
    parser.add_argument("--require-fresh-evidence", action="store_true")
    parser.add_argument(
        "--evidence-stale-after-s",
        type=float,
        default=DEFAULT_EVIDENCE_STALE_AFTER_S,
    )
    parser.add_argument("--axis-sweep-axes", default="yaw,pitch")
    parser.add_argument("--axis-expected-shifts", default="")
    parser.add_argument("--axis-min-shift-px", type=float, default=0.5)
    parser.add_argument("--max-window-candidates", type=int, default=8)
    parser.add_argument("--json", action="store_true")
    args = parser.parse_args(argv)
    if args.window_min_width <= 0 or args.window_min_height <= 0:
        parser.error("--window-min-width/--window-min-height must be positive")
    if args.evidence_stale_after_s < 0:
        parser.error("--evidence-stale-after-s must be non-negative")
    if args.axis_min_shift_px < 0:
        parser.error("--axis-min-shift-px must be non-negative")
    if args.max_window_candidates <= 0:
        parser.error("--max-window-candidates must be positive")
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
    report = run_operator_preflight(
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
        require_fresh_evidence=args.require_fresh_evidence,
        evidence_stale_after_s=args.evidence_stale_after_s,
        max_window_candidates=args.max_window_candidates,
        axis_sweep_axes=args.axis_sweep_axes,
        axis_expected_shifts=args.axis_expected_shifts,
        axis_min_shift_px=args.axis_min_shift_px,
    )
    json_path, md_path = write_reports(report, log_dir)
    if args.json:
        print(json.dumps(report.as_dict(), indent=2, sort_keys=True))
    else:
        print("external-operator-preflight %s report=%s summary=%s" % (
            report.status,
            json_path,
            md_path,
        ))
    if report.status == OPERATOR_PREFLIGHT_READY:
        return 0
    if report.status == WAITING:
        return 2
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
