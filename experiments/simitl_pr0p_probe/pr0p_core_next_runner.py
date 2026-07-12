#!/usr/bin/env python3
"""Plan or safely run the current pr0p core next command."""

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
SANDBOX_DIR = PROBE_DIR.parent / "game_screen_sandbox"
if str(SANDBOX_DIR) not in sys.path:
    sys.path.insert(0, str(SANDBOX_DIR))

from pr0p_capture_probe import DEFAULT_LOG_DIR  # noqa: E402
from pr0p_core_goal_report import (  # noqa: E402
    CoreGoalGate,
    CoreGoalReport,
    build_core_goal_report,
    resolve_bbox_argument,
    write_reports as write_core_goal_reports,
)
from pr0p_isolation_check import (  # noqa: E402
    DEFAULT_ROOTS as ISOLATION_ROOTS,
    run_isolation_check,
    write_reports as write_isolation_reports,
)
from pr0p_tracking_probe import load_bbox_file  # noqa: E402
from screen_tracking_loop import parse_bbox  # noqa: E402


PASS = "PASS"
WAITING = "WAITING"
FAIL = "FAIL"
PLANNED = "PLANNED"
UNKNOWN = "UNKNOWN"
INDEPENDENT_READY = "INDEPENDENT_SIM_READY"
OPERATOR_READY = "OPERATOR_PREFLIGHT_READY"
DEFAULT_GAME_LOG_DIR = Path("logs/game_screen_sandbox")

LIVE_COMMAND_MARKERS = (
    "--run-live-gates",
    "--run-live-gate",
    "--ack-live-input",
    "--ack-live-launch",
    "--ack-live-ui",
    "--launch-pr0p",
    "--hold-uinput",
    "--uinput",
)
PLACEHOLDER_MARKERS = ("x,y,w,h",)
COMMAND_OBJECTIVE_GATES = {
    "core_status": "core_status",
    "rc_manual_flight_plan": "rc_manual_flight",
    "rc_manual_flight_live": "rc_manual_flight",
    "acceptance_chain_plan": "autopilot_control",
    "acceptance_chain_live": "autopilot_control",
    "tracking_acceptance_plan": "camera_tracker",
    "tracking_acceptance_dry": "camera_tracker",
    "tracking_acceptance_live": "autopilot_control",
    "extended_follow_live": "extended_follow",
    "moving_target_yaw_plan": "moving_target_yaw",
    "airborne_static_yaw_live": "moving_target_yaw",
    "airborne_static_yaw_refresh": "moving_target_yaw",
    "approach_pitch_plan": "approach_pitch",
    "approach_pitch_live": "approach_pitch",
    "approach_pitch_refresh": "approach_pitch",
    "handoff_plan": "handoff",
    "handoff_live": "handoff",
    "handoff_refresh": "handoff",
    "response_acceptance_plan": "autopilot_control",
    "response_acceptance_live": "autopilot_control",
}


@dataclass
class CoreNextReport:
    run_id: str
    started_at: str
    status: str
    summary: str
    next_action: str
    core_report: str | None
    next_command_key: str | None
    next_command: str | None
    command_is_live: bool
    executed: bool
    returncode: int | None = None
    stdout: str = ""
    stderr: str = ""
    command_report: str | None = None
    notes: list[str] = field(default_factory=list)
    metrics: dict[str, Any] = field(default_factory=dict)

    def as_dict(self) -> dict[str, Any]:
        return {
            "run_id": self.run_id,
            "started_at": self.started_at,
            "status": self.status,
            "summary": self.summary,
            "next_action": self.next_action,
            "core_report": self.core_report,
            "next_command_key": self.next_command_key,
            "next_command": self.next_command,
            "command_is_live": self.command_is_live,
            "executed": self.executed,
            "returncode": self.returncode,
            "stdout": self.stdout,
            "stderr": self.stderr,
            "command_report": self.command_report,
            "notes": self.notes,
            "metrics": self.metrics,
        }


def is_live_command(command: str | None) -> bool:
    if not command:
        return False
    return any(marker in command for marker in LIVE_COMMAND_MARKERS)


def command_argv(command: str) -> list[str]:
    return shlex.split(command.replace("\\\n", " "))


def has_placeholder(command: str | None) -> bool:
    if not command:
        return False
    return any(marker in command for marker in PLACEHOLDER_MARKERS)


def objective_gate_for_command_key(command_key: str | None) -> str | None:
    if command_key is None:
        return None
    return COMMAND_OBJECTIVE_GATES.get(command_key)


def gate_snapshot(gate: CoreGoalGate | None) -> dict[str, Any]:
    if gate is None:
        return {
            "status": UNKNOWN,
            "summary": "objective gate is not present in the current core report",
            "missing": [],
            "failed": [],
        }
    return {
        "status": gate.status,
        "summary": gate.summary,
        "missing": list(gate.missing),
        "failed": list(gate.failed),
    }


def annotate_next_objective_gate(*, metrics: dict[str, Any], core: CoreGoalReport) -> None:
    objective_gate = objective_gate_for_command_key(core.next_command_key)
    available_gates = {gate.name: gate for gate in [*core.gates, *core.post_core_gates]}
    snapshot = gate_snapshot(available_gates.get(objective_gate)) if objective_gate else None
    metrics["next_objective_gate"] = objective_gate
    metrics["next_objective_gate_snapshot"] = snapshot
    if snapshot is not None:
        metrics["next_objective_gate_status"] = snapshot["status"]
        metrics["next_objective_gate_missing"] = snapshot["missing"]
        metrics["next_objective_gate_failed"] = snapshot["failed"]


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


def isolation_precheck(
    *,
    run_id: str,
    log_dir: Path,
) -> dict[str, Any]:
    result = run_isolation_check(roots=list(ISOLATION_ROOTS))
    json_path, md_path = write_isolation_reports(
        result,
        log_dir,
        run_id="%s-isolation-precheck" % run_id,
    )
    return {
        "status": result.status,
        "summary": result.summary,
        "report": str(json_path),
        "summary_report": str(md_path),
        "notes": list(result.notes),
        "violation_count": len(result.metrics.get("violations", [])),
    }


def independent_readiness_precheck(
    *,
    run_id: str,
    log_dir: Path,
    game_log_dir: Path,
    bbox: tuple[int, int, int, int] | None,
    max_evidence_age_s: float,
) -> dict[str, Any]:
    from independent_sim_readiness import build_readiness  # noqa: WPS433

    report = build_readiness(
        run_id="%s-live-precheck" % run_id,
        game_log_dir=game_log_dir,
        pr0p_log_dir=log_dir,
        bbox=bbox,
        window_title="pr0p",
        yaw_expected_sign=0,
        pitch_expected_sign=0,
        max_evidence_age_s=max_evidence_age_s,
    )
    return {
        "status": report.status,
        "summary": report.summary,
        "next_action": report.next_action,
        "reasons": report.reasons,
        "objective_completion_status": report.objective_completion_status,
        "objective_completion_missing": report.objective_completion_missing,
        "objective_completion_failed": report.objective_completion_failed,
    }


def operator_readiness_precheck(
    *,
    run_id: str,
    game_log_dir: Path,
    bbox_file: Path | None,
    selected_bbox: tuple[int, int, int, int] | None,
    window_title: str,
    max_evidence_age_s: float,
) -> dict[str, Any]:
    if bbox_file is None:
        return {
            "status": WAITING,
            "summary": "operator readiness needs a bbox file for the selected target",
            "next_actions": ["rerun core-next with --bbox-file or --operator-bbox-file"],
            "evidence_paths": {},
            "decision": {
                "readiness_stage": "target_setup",
                "command_gate": "bbox_file_missing",
            },
        }
    try:
        operator_bbox = load_bbox_file(bbox_file)
    except Exception as exc:
        return {
            "status": WAITING,
            "summary": "operator readiness could not read the selected target bbox file",
            "next_actions": ["rerun core-next with a readable --bbox-file or --operator-bbox-file"],
            "evidence_paths": {"operator_bbox_file": str(bbox_file)},
            "selected_bbox": list(selected_bbox) if selected_bbox else None,
            "operator_bbox": None,
            "decision": {
                "readiness_stage": "target_setup",
                "command_gate": "bbox_file_unreadable",
                "error": str(exc),
            },
        }
    if selected_bbox is not None and operator_bbox != selected_bbox:
        return {
            "status": WAITING,
            "summary": "operator readiness bbox does not match the command target bbox",
            "next_actions": [
                "rerun core-next with the same --bbox-file used to select the live target",
            ],
            "evidence_paths": {"operator_bbox_file": str(bbox_file)},
            "selected_bbox": list(selected_bbox),
            "operator_bbox": list(operator_bbox),
            "decision": {
                "readiness_stage": "target_setup",
                "command_gate": "bbox_file_mismatch",
            },
        }

    from external_operator_preflight import (  # noqa: WPS433
        run_operator_preflight,
        write_reports as write_operator_preflight_reports,
    )

    report = run_operator_preflight(
        run_id="%s-operator-precheck" % run_id,
        log_dir=game_log_dir,
        window_title=window_title,
        bbox_file=bbox_file,
        window_exact=True,
        window_case_sensitive=False,
        window_min_width=32,
        window_min_height=32,
        require_live_input=False,
        require_live_follow=True,
        require_fresh_evidence=True,
        evidence_stale_after_s=max_evidence_age_s,
        max_window_candidates=8,
        axis_sweep_axes=("yaw", "pitch"),
        axis_expected_shifts=None,
        axis_min_shift_px=0.5,
    )
    json_path, md_path = write_operator_preflight_reports(report, game_log_dir)
    decision = (
        report.metrics.get("operator_preflight_decision")
        if isinstance(report.metrics.get("operator_preflight_decision"), dict)
        else {}
    )
    return {
        "status": report.status,
        "summary": report.summary,
        "next_actions": report.next_actions,
        "evidence_paths": {
            **report.evidence_paths,
            "operator_preflight_json": str(json_path),
            "operator_preflight_md": str(md_path),
            "operator_bbox_file": str(bbox_file),
        },
        "selected_bbox": list(selected_bbox) if selected_bbox else None,
        "operator_bbox": list(operator_bbox),
        "decision": decision,
    }


def annotate_live_execution_safety(
    *,
    metrics: dict[str, Any],
    command: str | None,
    live: bool,
    placeholder: bool,
    require_independent_ready_for_live: bool,
) -> None:
    precheck = metrics.get("independent_readiness_precheck")
    prechecked = isinstance(precheck, dict)
    if not live:
        metrics["live_precheck_status"] = "NOT_LIVE"
        metrics["safe_to_execute_live_with_ack"] = None
        metrics["safe_to_execute_live_blockers"] = []
        return

    if not require_independent_ready_for_live:
        live_precheck_status = "NOT_REQUIRED"
    elif prechecked:
        live_precheck_status = str(precheck.get("status", UNKNOWN))
    else:
        live_precheck_status = "NOT_CHECKED"

    blockers: list[str] = []
    if not command:
        blockers.append("NEXT_COMMAND_MISSING")
    if placeholder:
        blockers.append("PLACEHOLDER_COMMAND")
    if require_independent_ready_for_live:
        if not prechecked:
            blockers.append("INDEPENDENT_READINESS_NOT_CHECKED")
        elif live_precheck_status != INDEPENDENT_READY:
            blockers.append("INDEPENDENT_READINESS_NOT_READY")

    metrics["live_precheck_status"] = live_precheck_status
    metrics["safe_to_execute_live_with_ack"] = not blockers
    metrics["safe_to_execute_live_blockers"] = blockers
    refresh_combined_live_execution_safety(metrics=metrics, live=live)


def unique(items: list[str]) -> list[str]:
    seen: set[str] = set()
    result: list[str] = []
    for item in items:
        if item in seen:
            continue
        seen.add(item)
        result.append(item)
    return result


def refresh_combined_live_execution_safety(
    *,
    metrics: dict[str, Any],
    live: bool,
) -> None:
    if not live:
        metrics["safe_to_execute_live_with_all_prechecks"] = None
        metrics["safe_to_execute_live_required_blockers"] = []
        return
    blockers = unique([
        *list(metrics.get("safe_to_execute_live_blockers", [])),
        *list(metrics.get("safe_to_execute_live_operator_blockers", [])),
    ])
    metrics["safe_to_execute_live_with_all_prechecks"] = not blockers
    metrics["safe_to_execute_live_required_blockers"] = blockers


def annotate_operator_precheck_details(
    *,
    metrics: dict[str, Any],
    live: bool,
) -> None:
    precheck = metrics.get("operator_readiness_precheck")
    decision = (
        precheck.get("decision", {})
        if isinstance(precheck, dict) else
        {}
    )
    if not isinstance(decision, dict):
        decision = {}
    next_actions = (
        precheck.get("next_actions", [])
        if isinstance(precheck, dict) else
        []
    )
    if not isinstance(next_actions, list):
        next_actions = []
    metrics["operator_readiness_stage"] = (
        decision.get("readiness_stage") if live else None
    )
    metrics["operator_readiness_command_gate"] = (
        decision.get("command_gate") if live else None
    )
    metrics["operator_readiness_next_action"] = (
        next_actions[0] if live and next_actions else None
    )
    metrics["operator_readiness_recommended_command"] = (
        decision.get("recommended_command") if live else None
    )
    metrics["operator_readiness_recommended_candidate_action"] = (
        decision.get("recommended_candidate_action") if live else None
    )
    metrics["operator_readiness_safe_to_execute_now"] = (
        decision.get("safe_to_execute_now") if live else None
    )
    metrics["operator_readiness_live_ack_required"] = (
        decision.get("live_ack_required") if live else None
    )
    metrics["operator_readiness_freshness_gate_status"] = (
        decision.get("freshness_gate_status") if live else None
    )
    metrics["operator_readiness_required_fresh_evidence"] = (
        list(decision.get("required_fresh_evidence", []))
        if live and isinstance(decision.get("required_fresh_evidence", []), list)
        else []
    )
    metrics["operator_readiness_stale_evidence"] = (
        list(decision.get("stale_evidence", []))
        if live and isinstance(decision.get("stale_evidence", []), list)
        else []
    )
    metrics["operator_readiness_missing_evidence"] = (
        list(decision.get("missing_evidence", []))
        if live and isinstance(decision.get("missing_evidence", []), list)
        else []
    )
    metrics["operator_readiness_window_discovery_status"] = (
        decision.get("window_discovery_status") if live else None
    )
    metrics["operator_readiness_window_discovery_reason"] = (
        decision.get("window_discovery_reason") if live else None
    )
    metrics["operator_readiness_window_title_candidates"] = (
        list(decision.get("window_title_candidates", []))
        if live and isinstance(decision.get("window_title_candidates", []), list)
        else []
    )
    metrics["operator_readiness_excluded_window_candidates"] = (
        list(decision.get("excluded_window_candidates", []))
        if live and isinstance(decision.get("excluded_window_candidates", []), list)
        else []
    )
    metrics["operator_readiness_next_command_packet"] = (
        operator_next_command_packet(metrics=metrics, live=live)
    )


def operator_next_command_packet(
    *,
    metrics: dict[str, Any],
    live: bool,
) -> dict[str, Any] | None:
    if not live:
        return None
    candidate = metrics.get("operator_readiness_recommended_candidate_action")
    if isinstance(candidate, dict) and candidate.get("next_command"):
        return {
            "source": "recommended_candidate_action",
            "name": candidate.get("next_step"),
            "command": candidate.get("next_command"),
            "safety_class": candidate.get("safety_class"),
            "sends_uinput": bool(candidate.get("sends_uinput")),
            "requires_ack_live_input": bool(candidate.get("requires_ack_live_input")),
            "requires_user_judgement": bool(candidate.get("requires_user_judgement")),
            "unmet_requires_pass": list(candidate.get("unmet_requires_pass", [])),
            "unblocks": list(candidate.get("unblocks", [])),
            "guard": candidate.get("guard"),
        }
    recommended = metrics.get("operator_readiness_recommended_command")
    if isinstance(recommended, dict) and recommended.get("command"):
        unmet = list(recommended.get("unmet_requires_pass", []))
        return {
            "source": "recommended_command",
            "name": recommended.get("name"),
            "command": recommended.get("command"),
            "command_state": recommended.get("command_state"),
            "safety_class": recommended.get("safety_class"),
            "sends_uinput": bool(recommended.get("sends_uinput")),
            "requires_ack_live_input": bool(recommended.get("requires_ack_live_input")),
            "requires_pass": list(recommended.get("requires_pass", [])),
            "unmet_requires_pass": unmet,
            "guard": (
                "Resolve unmet prerequisites before expecting this command to pass."
                if unmet else
                "Review the command and run it only against the intended FPV/sim window."
            ),
        }
    return None


def operator_next_command_blockers(
    packet: dict[str, Any] | None,
    *,
    ack_operator_candidate: bool,
) -> list[str]:
    if not isinstance(packet, dict):
        return ["OPERATOR_NEXT_COMMAND_MISSING"]
    command = packet.get("command")
    if not isinstance(command, str) or not command.strip():
        return ["OPERATOR_NEXT_COMMAND_MISSING"]
    blockers: list[str] = []
    if bool(packet.get("sends_uinput")) or is_live_command(command):
        blockers.append("OPERATOR_NEXT_COMMAND_UINPUT_OR_LIVE")
    unmet = packet.get("unmet_requires_pass", [])
    if isinstance(unmet, list) and unmet:
        blockers.append("OPERATOR_NEXT_COMMAND_UNMET_REQUIREMENTS")
    if packet.get("requires_user_judgement") and not ack_operator_candidate:
        blockers.append("OPERATOR_NEXT_COMMAND_REQUIRES_CANDIDATE_ACK")
    if has_placeholder(command):
        blockers.append("OPERATOR_NEXT_COMMAND_PLACEHOLDER")
    return blockers


def operator_precheck_compact(precheck: dict[str, Any] | None) -> dict[str, Any]:
    if not isinstance(precheck, dict):
        return {
            "status": UNKNOWN,
            "stage": None,
            "next_action": None,
            "missing_evidence": [],
            "stale_evidence": [],
            "window_discovery_status": None,
            "window_discovery_reason": None,
        }
    decision = precheck.get("decision", {})
    if not isinstance(decision, dict):
        decision = {}
    next_actions = precheck.get("next_actions", [])
    if not isinstance(next_actions, list):
        next_actions = []
    missing = decision.get("missing_evidence", [])
    stale = decision.get("stale_evidence", [])
    return {
        "status": precheck.get("status", UNKNOWN),
        "stage": decision.get("readiness_stage"),
        "next_action": next_actions[0] if next_actions else None,
        "missing_evidence": list(missing) if isinstance(missing, list) else [],
        "stale_evidence": list(stale) if isinstance(stale, list) else [],
        "window_discovery_status": decision.get("window_discovery_status"),
        "window_discovery_reason": decision.get("window_discovery_reason"),
    }


def annotate_operator_execution_safety(
    *,
    metrics: dict[str, Any],
    live: bool,
    precheck_operator_readiness: bool,
    require_operator_ready_for_live: bool,
) -> None:
    precheck = metrics.get("operator_readiness_precheck")
    prechecked = isinstance(precheck, dict)
    if not live:
        metrics["operator_precheck_status"] = "NOT_LIVE"
        metrics["safe_to_execute_live_with_operator_precheck"] = None
        metrics["safe_to_execute_live_operator_blockers"] = []
        annotate_operator_precheck_details(metrics=metrics, live=live)
        refresh_combined_live_execution_safety(metrics=metrics, live=live)
        return

    if not require_operator_ready_for_live:
        operator_status = "NOT_REQUIRED"
    elif prechecked:
        operator_status = str(precheck.get("status", UNKNOWN))
    elif precheck_operator_readiness:
        operator_status = "NOT_RUN"
    else:
        operator_status = "NOT_CHECKED"

    blockers: list[str] = []
    if not require_operator_ready_for_live:
        blockers = []
    elif not prechecked:
        blockers.append("OPERATOR_READINESS_NOT_CHECKED")
    elif operator_status != OPERATOR_READY:
        blockers.append("OPERATOR_READINESS_NOT_READY")

    metrics["operator_precheck_status"] = operator_status
    metrics["safe_to_execute_live_with_operator_precheck"] = not blockers
    metrics["safe_to_execute_live_operator_blockers"] = blockers
    annotate_operator_precheck_details(metrics=metrics, live=live)
    refresh_combined_live_execution_safety(metrics=metrics, live=live)


def annotate_live_blocker_summary(
    *,
    metrics: dict[str, Any],
    live: bool,
) -> None:
    if not live:
        metrics["live_blocker_summary"] = {
            "status": "NOT_LIVE",
            "safe_to_execute_live": None,
            "blockers": [],
        }
        return
    objective_snapshot = metrics.get("next_objective_gate_snapshot")
    if not isinstance(objective_snapshot, dict):
        objective_snapshot = {}
    recommended_command = metrics.get("operator_readiness_recommended_command")
    if not isinstance(recommended_command, dict):
        recommended_command = None
    recommended_candidate = metrics.get(
        "operator_readiness_recommended_candidate_action"
    )
    if not isinstance(recommended_candidate, dict):
        recommended_candidate = None
    command_packet = metrics.get("operator_readiness_next_command_packet")
    if not isinstance(command_packet, dict):
        command_packet = None
    metrics["live_blocker_summary"] = {
        "status": (
            "SAFE_TO_EXECUTE"
            if metrics.get("safe_to_execute_live_with_all_prechecks") is True else
            "BLOCKED"
        ),
        "safe_to_execute_live": metrics.get("safe_to_execute_live_with_all_prechecks"),
        "blockers": list(metrics.get("safe_to_execute_live_required_blockers", [])),
        "isolation_status": metrics.get("isolation_status"),
        "isolation_report": metrics.get("isolation_report"),
        "next_command_key": metrics.get("next_command_key"),
        "next_objective_gate": metrics.get("next_objective_gate"),
        "next_objective_gate_status": metrics.get("next_objective_gate_status"),
        "next_objective_gate_missing": list(
            objective_snapshot.get("missing", [])
            if isinstance(objective_snapshot.get("missing", []), list) else
            []
        ),
        "next_objective_gate_failed": list(
            objective_snapshot.get("failed", [])
            if isinstance(objective_snapshot.get("failed", []), list) else
            []
        ),
        "objective_requirement_missing": list(
            metrics.get("objective_requirement_missing", [])
        ),
        "objective_requirement_failed": list(
            metrics.get("objective_requirement_failed", [])
        ),
        "independent_readiness_status": metrics.get("live_precheck_status"),
        "operator_readiness_status": metrics.get("operator_precheck_status"),
        "operator_readiness_stage": metrics.get("operator_readiness_stage"),
        "operator_next_action": metrics.get("operator_readiness_next_action"),
        "operator_window_discovery_status": metrics.get(
            "operator_readiness_window_discovery_status"
        ),
        "operator_window_discovery_reason": metrics.get(
            "operator_readiness_window_discovery_reason"
        ),
        "operator_window_title_candidates": list(
            metrics.get("operator_readiness_window_title_candidates", [])
        ),
        "operator_excluded_window_candidates": list(
            metrics.get("operator_readiness_excluded_window_candidates", [])
        ),
        "operator_missing_evidence": list(
            metrics.get("operator_readiness_missing_evidence", [])
        ),
        "operator_stale_evidence": list(
            metrics.get("operator_readiness_stale_evidence", [])
        ),
        "operator_recommended_command_name": (
            recommended_command.get("name") if recommended_command else None
        ),
        "operator_recommended_command_safety_class": (
            recommended_command.get("safety_class") if recommended_command else None
        ),
        "operator_recommended_command_sends_uinput": (
            bool(recommended_command.get("sends_uinput"))
            if recommended_command else
            None
        ),
        "operator_recommended_command_unmet_requires_pass": (
            list(recommended_command.get("unmet_requires_pass", []))
            if recommended_command else
            []
        ),
        "operator_recommended_candidate_next_step": (
            recommended_candidate.get("next_step") if recommended_candidate else None
        ),
        "operator_recommended_candidate_sends_uinput": (
            bool(recommended_candidate.get("sends_uinput"))
            if recommended_candidate else
            None
        ),
        "operator_next_command_source": (
            command_packet.get("source") if command_packet else None
        ),
        "operator_next_command_name": (
            command_packet.get("name") if command_packet else None
        ),
        "operator_next_command": (
            command_packet.get("command") if command_packet else None
        ),
        "operator_next_command_sends_uinput": (
            bool(command_packet.get("sends_uinput"))
            if command_packet else
            None
        ),
        "operator_next_command_unmet_requires_pass": (
            list(command_packet.get("unmet_requires_pass", []))
            if command_packet else
            []
        ),
        "operator_next_command_guard": (
            command_packet.get("guard") if command_packet else None
        ),
    }


def run_core_next(
    *,
    run_id: str,
    log_dir: Path,
    bbox: tuple[int, int, int, int] | None,
    allow_physical_mapping: bool = False,
    max_evidence_age_s: float,
    execute_next: bool,
    ack_live_command: bool,
    timeout_s: float,
    execute_operator_next: bool = False,
    ack_operator_candidate: bool = False,
    game_log_dir: Path = DEFAULT_GAME_LOG_DIR,
    bbox_file: Path | None = None,
    operator_bbox_file: Path | None = None,
    operator_window_title: str = "pr0p",
    require_independent_ready_for_live: bool = True,
    require_operator_ready_for_live: bool = True,
    precheck_live_readiness: bool = False,
    precheck_operator_readiness: bool = False,
    command_runner: Callable[..., subprocess.CompletedProcess[str]] = subprocess.run,
) -> CoreNextReport:
    core = build_core_goal_report(
        run_id="%s-core" % run_id,
        log_dir=log_dir,
        bbox=bbox,
        allow_physical_mapping=allow_physical_mapping,
        max_evidence_age_s=max_evidence_age_s,
    )
    core_json, _core_md = write_core_goal_reports(core, log_dir)
    isolation = isolation_precheck(run_id=run_id, log_dir=log_dir)
    command = core.next_command
    live = is_live_command(command)
    placeholder = has_placeholder(command)
    metrics: dict[str, Any] = {
        "bbox": list(bbox) if bbox else None,
        "allow_physical_mapping": allow_physical_mapping,
        "log_dir": str(log_dir),
        "max_evidence_age_s": max_evidence_age_s,
        "execute_next": execute_next,
        "ack_live_command": ack_live_command,
        "game_log_dir": str(game_log_dir),
        "bbox_file": str(bbox_file) if bbox_file else None,
        "operator_bbox_file": str(operator_bbox_file or bbox_file) if (operator_bbox_file or bbox_file) else None,
        "operator_window_title": operator_window_title,
        "require_independent_ready_for_live": require_independent_ready_for_live,
        "require_operator_ready_for_live": require_operator_ready_for_live,
        "precheck_live_readiness": precheck_live_readiness,
        "precheck_operator_readiness": precheck_operator_readiness,
        "isolation_precheck": isolation,
        "isolation_status": isolation["status"],
        "isolation_report": isolation["report"],
        "isolation_violation_count": isolation["violation_count"],
        "core_status": core.status,
        "core_completion_status": core.completion_status,
        "core_completion_missing": core.completion_missing,
        "core_completion_failed": core.completion_failed,
        "objective_requirement_statuses": core.metrics.get(
            "objective_requirement_statuses",
            {},
        ),
        "objective_requirement_missing": core.metrics.get(
            "objective_requirement_missing",
            [],
        ),
        "objective_requirement_failed": core.metrics.get(
            "objective_requirement_failed",
            [],
        ),
        "command_has_placeholder": placeholder,
    }
    annotate_next_objective_gate(metrics=metrics, core=core)
    metrics["next_command_key"] = core.next_command_key
    resolved_operator_bbox_file = operator_bbox_file or bbox_file
    if not command:
        annotate_live_execution_safety(
            metrics=metrics,
            command=command,
            live=live,
            placeholder=placeholder,
            require_independent_ready_for_live=require_independent_ready_for_live,
        )
        annotate_operator_execution_safety(
            metrics=metrics,
            live=live,
            precheck_operator_readiness=precheck_operator_readiness,
            require_operator_ready_for_live=require_operator_ready_for_live,
        )
        annotate_live_blocker_summary(metrics=metrics, live=live)
        return CoreNextReport(
            run_id=run_id,
            started_at=time.strftime("%Y-%m-%dT%H:%M:%S%z", time.localtime()),
            status=WAITING,
            summary="core report did not provide a next command",
            next_action=core.next_action,
            core_report=str(core_json),
            next_command_key=core.next_command_key,
            next_command=None,
            command_is_live=live,
            executed=False,
            notes=["NEXT_COMMAND_MISSING"],
            metrics=metrics,
        )
    if (
        live
        and require_independent_ready_for_live
        and precheck_live_readiness
        and not placeholder
    ):
        metrics["independent_readiness_precheck"] = independent_readiness_precheck(
            run_id=run_id,
            log_dir=log_dir,
            game_log_dir=game_log_dir,
            bbox=bbox,
            max_evidence_age_s=max_evidence_age_s,
        )
    if live and (precheck_operator_readiness or execute_operator_next) and not placeholder:
        metrics["operator_readiness_precheck"] = operator_readiness_precheck(
            run_id=run_id,
            game_log_dir=game_log_dir,
            bbox_file=resolved_operator_bbox_file,
            selected_bbox=bbox,
            window_title=operator_window_title,
            max_evidence_age_s=max_evidence_age_s,
        )
    annotate_live_execution_safety(
        metrics=metrics,
        command=command,
        live=live,
        placeholder=placeholder,
        require_independent_ready_for_live=require_independent_ready_for_live,
    )
    annotate_operator_execution_safety(
        metrics=metrics,
        live=live,
        precheck_operator_readiness=precheck_operator_readiness,
        require_operator_ready_for_live=require_operator_ready_for_live,
    )
    annotate_live_blocker_summary(metrics=metrics, live=live)
    if isolation["status"] != PASS:
        return CoreNextReport(
            run_id=run_id,
            started_at=time.strftime("%Y-%m-%dT%H:%M:%S%z", time.localtime()),
            status=FAIL,
            summary="Gazebo-independent isolation precheck failed",
            next_action="Inspect the isolation report before running core-next or operator-next commands.",
            core_report=str(core_json),
            next_command_key=core.next_command_key,
            next_command=command,
            command_is_live=live,
            executed=False,
            notes=["GAZEBO_INDEPENDENCE_FAILED"],
            metrics=metrics,
        )
    if execute_operator_next:
        packet = metrics.get("operator_readiness_next_command_packet")
        packet = packet if isinstance(packet, dict) else None
        blockers = operator_next_command_blockers(
            packet,
            ack_operator_candidate=ack_operator_candidate,
        )
        metrics["operator_next_execution_requested"] = True
        metrics["operator_next_execution_blockers"] = blockers
        metrics["operator_next_command_executed"] = False
        if blockers:
            return CoreNextReport(
                run_id=run_id,
                started_at=time.strftime("%Y-%m-%dT%H:%M:%S%z", time.localtime()),
                status=WAITING,
                summary="operator next command is blocked by guard",
                next_action=(
                    packet.get("guard")
                    if packet and packet.get("guard") else
                    "Resolve operator next command blockers before executing it."
                ),
                core_report=str(core_json),
                next_command_key=core.next_command_key,
                next_command=command,
                command_is_live=live,
                executed=False,
                notes=blockers,
                metrics=metrics,
            )
        operator_command = str(packet["command"])
        metrics["operator_next_command_executed"] = True
        metrics["executed_command_role"] = "operator_next"
        metrics["executed_command"] = operator_command
        try:
            completed = command_runner(
                command_argv(operator_command),
                capture_output=True,
                text=True,
                timeout=timeout_s,
            )
        except Exception as exc:  # pragma: no cover - subprocess failures vary
            return CoreNextReport(
                run_id=run_id,
                started_at=time.strftime("%Y-%m-%dT%H:%M:%S%z", time.localtime()),
                status=FAIL,
                summary="operator next command could not be executed",
                next_action="Inspect the operator command and runner error before retrying.",
                core_report=str(core_json),
                next_command_key=core.next_command_key,
                next_command=command,
                command_is_live=live,
                executed=True,
                notes=["OPERATOR_NEXT_COMMAND_EXECUTION_FAILED", str(exc)],
                metrics=metrics,
            )
        status = parsed_status(completed.stdout or "", returncode=completed.returncode)
        if completed.returncode != 0 and status != WAITING:
            status = FAIL
        post_operator_precheck = operator_readiness_precheck(
            run_id="%s-post-operator" % run_id,
            game_log_dir=game_log_dir,
            bbox_file=resolved_operator_bbox_file,
            selected_bbox=bbox,
            window_title=operator_window_title,
            max_evidence_age_s=max_evidence_age_s,
        )
        post_operator_summary = operator_precheck_compact(post_operator_precheck)
        metrics["post_operator_readiness_precheck"] = post_operator_precheck
        metrics["post_operator_readiness_summary"] = post_operator_summary
        metrics["post_operator_precheck_status"] = post_operator_summary["status"]
        metrics["post_operator_readiness_stage"] = post_operator_summary["stage"]
        metrics["post_operator_next_action"] = post_operator_summary["next_action"]
        metrics["post_operator_missing_evidence"] = post_operator_summary[
            "missing_evidence"
        ]
        metrics["post_operator_stale_evidence"] = post_operator_summary[
            "stale_evidence"
        ]
        metrics["post_operator_window_discovery_status"] = post_operator_summary[
            "window_discovery_status"
        ]
        metrics["post_operator_window_discovery_reason"] = post_operator_summary[
            "window_discovery_reason"
        ]
        return CoreNextReport(
            run_id=run_id,
            started_at=time.strftime("%Y-%m-%dT%H:%M:%S%z", time.localtime()),
            status=status,
            summary="operator next command returned %s" % status,
            next_action=(
                "Operator next command completed; post-operator readiness is %s."
                % post_operator_summary["status"]
            ),
            core_report=str(core_json),
            next_command_key=core.next_command_key,
            next_command=command,
            command_is_live=live,
            executed=True,
            returncode=completed.returncode,
            stdout=completed.stdout or "",
            stderr=completed.stderr or "",
            command_report=parsed_report_path(completed.stdout or ""),
            notes=["OPERATOR_NEXT_COMMAND_EXECUTED"],
            metrics=metrics,
        )
    if not execute_next:
        precheck = metrics.get("independent_readiness_precheck")
        precheck_ready = (
            isinstance(precheck, dict)
            and precheck.get("status") == INDEPENDENT_READY
        )
        precheck_not_ready = isinstance(precheck, dict) and not precheck_ready
        operator_precheck = metrics.get("operator_readiness_precheck")
        operator_ready = (
            isinstance(operator_precheck, dict)
            and operator_precheck.get("status") == OPERATOR_READY
        )
        operator_precheck_not_ready = (
            isinstance(operator_precheck, dict) and not operator_ready
        )
        notes = [
            "PLAN_ONLY",
            "LIVE_COMMAND" if live else "DRY_COMMAND",
            "PLACEHOLDER_COMMAND" if placeholder else "BOUND_COMMAND",
        ]
        if precheck_ready:
            notes.append("INDEPENDENT_READINESS_READY")
        elif precheck_not_ready:
            notes.append("INDEPENDENT_READINESS_NOT_READY")
        if operator_ready:
            notes.append("OPERATOR_READINESS_READY")
        elif operator_precheck_not_ready:
            notes.append("OPERATOR_READINESS_NOT_READY")
        return CoreNextReport(
            run_id=run_id,
            started_at=time.strftime("%Y-%m-%dT%H:%M:%S%z", time.localtime()),
            status=WAITING if (precheck_not_ready or operator_precheck_not_ready) else PLANNED,
            summary=(
                "live readiness precheck is not ready"
                if precheck_not_ready else
                "operator readiness precheck is not ready"
                if operator_precheck_not_ready else
                "next command planned but not executed"
            ),
            next_action=(
                str(precheck.get("next_action"))
                if precheck_not_ready
                else "; ".join(str(item) for item in operator_precheck.get("next_actions", [])[:2])
                if operator_precheck_not_ready and isinstance(operator_precheck, dict)
                else "Review next_command, then rerun with --execute-next when ready."
            ),
            core_report=str(core_json),
            next_command_key=core.next_command_key,
            next_command=command,
            command_is_live=live,
            executed=False,
            notes=notes,
            metrics=metrics,
        )
    if placeholder:
        annotate_live_execution_safety(
            metrics=metrics,
            command=command,
            live=live,
            placeholder=placeholder,
            require_independent_ready_for_live=require_independent_ready_for_live,
        )
        annotate_operator_execution_safety(
            metrics=metrics,
            live=live,
            precheck_operator_readiness=precheck_operator_readiness,
            require_operator_ready_for_live=require_operator_ready_for_live,
        )
        annotate_live_blocker_summary(metrics=metrics, live=live)
        return CoreNextReport(
            run_id=run_id,
            started_at=time.strftime("%Y-%m-%dT%H:%M:%S%z", time.localtime()),
            status=WAITING,
            summary="next command still contains placeholder arguments",
            next_action="Select a target bbox and rerun with --bbox x,y,w,h replaced by real numbers.",
            core_report=str(core_json),
            next_command_key=core.next_command_key,
            next_command=command,
            command_is_live=live,
            executed=False,
            notes=["PLACEHOLDER_COMMAND"],
            metrics=metrics,
        )
    if live and not ack_live_command:
        annotate_live_execution_safety(
            metrics=metrics,
            command=command,
            live=live,
            placeholder=placeholder,
            require_independent_ready_for_live=require_independent_ready_for_live,
        )
        annotate_operator_execution_safety(
            metrics=metrics,
            live=live,
            precheck_operator_readiness=precheck_operator_readiness,
            require_operator_ready_for_live=require_operator_ready_for_live,
        )
        annotate_live_blocker_summary(metrics=metrics, live=live)
        return CoreNextReport(
            run_id=run_id,
            started_at=time.strftime("%Y-%m-%dT%H:%M:%S%z", time.localtime()),
            status=WAITING,
            summary="next command is live and requires explicit ack",
            next_action="Rerun with --execute-next --ack-live-command after the simulator UI and RC Channels are ready.",
            core_report=str(core_json),
            next_command_key=core.next_command_key,
            next_command=command,
            command_is_live=live,
            executed=False,
            notes=["LIVE_COMMAND_REQUIRES_ACK"],
            metrics=metrics,
        )
    if live and require_independent_ready_for_live:
        precheck = independent_readiness_precheck(
            run_id=run_id,
            log_dir=log_dir,
            game_log_dir=game_log_dir,
            bbox=bbox,
            max_evidence_age_s=max_evidence_age_s,
        )
        metrics["independent_readiness_precheck"] = precheck
        annotate_live_execution_safety(
            metrics=metrics,
            command=command,
            live=live,
            placeholder=placeholder,
            require_independent_ready_for_live=require_independent_ready_for_live,
        )
        annotate_live_blocker_summary(metrics=metrics, live=live)
        if precheck["status"] != INDEPENDENT_READY:
            return CoreNextReport(
                run_id=run_id,
                started_at=time.strftime("%Y-%m-%dT%H:%M:%S%z", time.localtime()),
                status=WAITING,
                summary="independent readiness precheck is not ready",
                next_action=str(precheck["next_action"]),
                core_report=str(core_json),
                next_command_key=core.next_command_key,
                next_command=command,
                command_is_live=live,
                executed=False,
                notes=["INDEPENDENT_READINESS_NOT_READY"],
                metrics=metrics,
            )

    if live and (precheck_operator_readiness or require_operator_ready_for_live):
        operator_precheck = metrics.get("operator_readiness_precheck")
        if not isinstance(operator_precheck, dict):
            operator_precheck = operator_readiness_precheck(
                run_id=run_id,
                game_log_dir=game_log_dir,
                bbox_file=resolved_operator_bbox_file,
                selected_bbox=bbox,
                window_title=operator_window_title,
                max_evidence_age_s=max_evidence_age_s,
            )
            metrics["operator_readiness_precheck"] = operator_precheck
            annotate_operator_execution_safety(
                metrics=metrics,
                live=live,
                precheck_operator_readiness=precheck_operator_readiness,
                require_operator_ready_for_live=require_operator_ready_for_live,
            )
            annotate_live_blocker_summary(metrics=metrics, live=live)
        if require_operator_ready_for_live and operator_precheck["status"] != OPERATOR_READY:
            return CoreNextReport(
                run_id=run_id,
                started_at=time.strftime("%Y-%m-%dT%H:%M:%S%z", time.localtime()),
                status=WAITING,
                summary="operator readiness precheck is not ready",
                next_action="; ".join(str(item) for item in operator_precheck.get("next_actions", [])[:2]),
                core_report=str(core_json),
                next_command_key=core.next_command_key,
                next_command=command,
                command_is_live=live,
                executed=False,
                notes=["OPERATOR_READINESS_NOT_READY"],
                metrics=metrics,
            )

    try:
        completed = command_runner(
            command_argv(command),
            capture_output=True,
            text=True,
            timeout=timeout_s,
        )
    except Exception as exc:  # pragma: no cover - subprocess failures vary
        return CoreNextReport(
            run_id=run_id,
            started_at=time.strftime("%Y-%m-%dT%H:%M:%S%z", time.localtime()),
            status=FAIL,
            summary="next command could not be executed",
            next_action="Inspect the command and runner error before retrying.",
            core_report=str(core_json),
            next_command_key=core.next_command_key,
            next_command=command,
            command_is_live=live,
            executed=True,
            notes=[str(exc)],
            metrics=metrics,
        )

    status = parsed_status(completed.stdout or "", returncode=completed.returncode)
    if completed.returncode != 0 and status != WAITING:
        status = FAIL
    return CoreNextReport(
        run_id=run_id,
        started_at=time.strftime("%Y-%m-%dT%H:%M:%S%z", time.localtime()),
        status=status,
        summary="next command returned %s" % status,
        next_action="Refresh pr0p_core_goal_report.py after reviewing the command output.",
        core_report=str(core_json),
        next_command_key=core.next_command_key,
        next_command=command,
        command_is_live=live,
        executed=True,
        returncode=completed.returncode,
        stdout=completed.stdout or "",
        stderr=completed.stderr or "",
        command_report=parsed_report_path(completed.stdout or ""),
        notes=[],
        metrics=metrics,
    )


def build_markdown(report: CoreNextReport) -> str:
    lines = [
        "# pr0p Core Next Runner",
        "",
        "Run: `%s`" % report.run_id,
        "Started: `%s`" % report.started_at,
        "Status: `%s`" % report.status,
        "",
        report.summary,
        "",
        "Next action: %s" % report.next_action,
        "",
        "Core report: `%s`" % (report.core_report or "-"),
        "Next command key: `%s`" % (report.next_command_key or "-"),
        "Command is live: `%s`" % report.command_is_live,
        "Executed: `%s`" % report.executed,
        "",
    ]
    if report.next_command:
        lines.extend(["```bash", report.next_command, "```", ""])
    if report.command_report:
        lines.extend(["Command report: `%s`" % report.command_report, ""])
    if report.notes:
        lines.extend(["## Notes", ""])
        for note in report.notes:
            lines.append("- %s" % note)
        lines.append("")
    lines.extend(["## Metrics", "", "```json"])
    lines.append(json.dumps(report.metrics, indent=2, sort_keys=True))
    lines.extend(["```", ""])
    return "\n".join(lines)


def write_reports(report: CoreNextReport, log_dir: Path) -> tuple[Path, Path]:
    log_dir.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime("%Y%m%d-%H%M%S", time.localtime())
    json_path = log_dir / ("%s-%s-core-next.json" % (stamp, report.run_id))
    md_path = log_dir / ("%s-%s-core-next.md" % (stamp, report.run_id))
    json_path.write_text(json.dumps(report.as_dict(), indent=2, sort_keys=True) + "\n",
                         encoding="utf-8")
    md_path.write_text(build_markdown(report), encoding="utf-8")
    return json_path, md_path


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-id", default="pr0p-core-next")
    parser.add_argument("--log-dir", type=Path, default=DEFAULT_LOG_DIR)
    parser.add_argument("--game-log-dir", type=Path, default=DEFAULT_GAME_LOG_DIR)
    parser.add_argument("--bbox", type=parse_bbox)
    parser.add_argument("--bbox-file", type=Path)
    parser.add_argument("--operator-bbox-file", type=Path)
    parser.add_argument("--operator-window-title", default="pr0p")
    parser.add_argument("--allow-physical-mapping", action="store_true",
                        help="Propagate physical RC mapping acceptance to next commands")
    parser.add_argument("--max-evidence-age-s", type=float, default=24 * 60 * 60)
    parser.add_argument("--execute-next", action="store_true")
    parser.add_argument("--ack-live-command", action="store_true")
    parser.add_argument(
        "--execute-operator-next",
        action="store_true",
        help="Run the guarded non-uinput operator_next_command_packet instead of the core live command",
    )
    parser.add_argument(
        "--ack-operator-candidate",
        action="store_true",
        help="Acknowledge that the selected external-window candidate is the intended FPV/sim view",
    )
    parser.add_argument(
        "--precheck-live-readiness",
        action="store_true",
        help="Run the read-only independent readiness precheck in plan mode for live commands",
    )
    parser.add_argument(
        "--precheck-operator-readiness",
        action="store_true",
        help="Run the status-only operator/window/bbox readiness precheck for live commands",
    )
    parser.add_argument(
        "--skip-operator-readiness-precheck",
        action="store_true",
        help="Allow live execution without the operator/window/bbox readiness precheck",
    )
    parser.add_argument("--timeout", type=float, default=300.0)
    args = parser.parse_args(argv)
    if args.max_evidence_age_s < 0:
        parser.error("--max-evidence-age-s must be non-negative")
    if args.ack_live_command and not args.execute_next:
        parser.error("--ack-live-command requires --execute-next")
    if args.execute_next and args.execute_operator_next:
        parser.error("use --execute-next or --execute-operator-next, not both")
    if args.ack_operator_candidate and not args.execute_operator_next:
        parser.error("--ack-operator-candidate requires --execute-operator-next")
    if args.timeout <= 0:
        parser.error("--timeout must be positive")
    if args.precheck_operator_readiness and args.skip_operator_readiness_precheck:
        parser.error("use --precheck-operator-readiness or --skip-operator-readiness-precheck, not both")
    args.bbox = resolve_bbox_argument(
        bbox=args.bbox,
        bbox_file=args.bbox_file,
        parser=parser,
    )
    return args


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    report = run_core_next(
        run_id=args.run_id,
        log_dir=args.log_dir,
        bbox=args.bbox,
        allow_physical_mapping=args.allow_physical_mapping,
        max_evidence_age_s=args.max_evidence_age_s,
        execute_next=args.execute_next,
        ack_live_command=args.ack_live_command,
        timeout_s=args.timeout,
        execute_operator_next=args.execute_operator_next,
        ack_operator_candidate=args.ack_operator_candidate,
        game_log_dir=args.game_log_dir,
        bbox_file=args.bbox_file,
        operator_bbox_file=args.operator_bbox_file,
        operator_window_title=args.operator_window_title,
        require_operator_ready_for_live=not args.skip_operator_readiness_precheck,
        precheck_live_readiness=args.precheck_live_readiness,
        precheck_operator_readiness=args.precheck_operator_readiness,
    )
    json_path, md_path = write_reports(report, args.log_dir)
    print("simitl-pr0p-core-next %s report=%s summary=%s" % (
        report.status,
        json_path,
        md_path,
    ))
    print(report.next_action)
    return 1 if report.status == FAIL else 0


if __name__ == "__main__":
    raise SystemExit(main())
