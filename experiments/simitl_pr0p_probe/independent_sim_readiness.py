#!/usr/bin/env python3
"""Combined readiness report for the Gazebo-independent sim path."""

from __future__ import annotations

import argparse
import json
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

PROBE_DIR = Path(__file__).resolve().parent
if str(PROBE_DIR) not in sys.path:
    sys.path.insert(0, str(PROBE_DIR))
SANDBOX_DIR = PROBE_DIR.parent / "game_screen_sandbox"
if str(SANDBOX_DIR) not in sys.path:
    sys.path.insert(0, str(SANDBOX_DIR))

from pr0p_capture_probe import DEFAULT_LOG_DIR as PR0P_LOG_DIR  # noqa: E402
from pr0p_decision_report import (  # noqa: E402
    FAIL,
    PROMOTE_CANDIDATE,
    REJECT,
    WAITING,
    evaluate_decision as evaluate_pr0p_decision,
    latest_manifest_report,
    latest_suite_report,
    load_json_report,
)
from pr0p_live_run_manifest import build_manifest  # noqa: E402
from pr0p_tracking_probe import load_bbox_file  # noqa: E402
from screen_tracking_loop import parse_bbox  # noqa: E402
from decision_report import (  # noqa: E402
    SIMPLE_WINDOW_REQUIRED_PHASES as GAME_SCREEN_SIMPLE_WINDOW_REQUIRED_PHASES,
    SYNTHETIC_REQUIRED_PHASES as GAME_SCREEN_SYNTHETIC_REQUIRED_PHASES,
)


PASS = "PASS"
STALE = "STALE"
MISSING = "MISSING"
UNKNOWN = "UNKNOWN"
MISMATCH = "MISMATCH"
INDEPENDENT_SIM_READY = "INDEPENDENT_SIM_READY"
GAME_SCREEN_READY = "SIMPLE_SANDBOX_READY"
GAME_SCREEN_SCOPE = "isolated game/screen sandbox only; not pr0p/Betaflight promotion"
GAME_SCREEN_LOG_DIR = Path("logs/game_screen_sandbox")
DEFAULT_MAX_EVIDENCE_AGE_S = 24.0 * 60.0 * 60.0
PROVEN_LIVE_PROFILE = (
    "--arm-first",
    "--arm-throttle -0.4",
    "--measure attitude",
    "--attitude-min-delta 10.0",
)
TRACKING_LIVE_PROFILE = (
    "--arm-first",
    "--arm-throttle -0.4",
)
EXTENDED_FOLLOW_PROFILE = (
    "--duration 20.0",
    "--min-found-ratio 0.95",
    "--max-loss-events 0",
)


@dataclass
class ReadinessReport:
    run_id: str
    started_at: str
    status: str
    summary: str
    next_action: str
    objective_completion_status: str
    objective_completion_summary: str
    objective_completion_missing: list[str] = field(default_factory=list)
    objective_completion_failed: list[str] = field(default_factory=list)
    evidence_paths: dict[str, str | None] = field(default_factory=dict)
    reasons: list[str] = field(default_factory=list)
    commands: dict[str, str] = field(default_factory=dict)
    metrics: dict[str, Any] = field(default_factory=dict)

    def as_dict(self) -> dict[str, Any]:
        return {
            "run_id": self.run_id,
            "started_at": self.started_at,
            "status": self.status,
            "summary": self.summary,
            "next_action": self.next_action,
            "objective_completion_status": self.objective_completion_status,
            "objective_completion_summary": self.objective_completion_summary,
            "objective_completion_missing": self.objective_completion_missing,
            "objective_completion_failed": self.objective_completion_failed,
            "evidence_paths": self.evidence_paths,
            "reasons": self.reasons,
            "commands": self.commands,
            "metrics": self.metrics,
        }


def report_sort_key(path: Path) -> tuple[int, str]:
    try:
        mtime_ns = path.stat().st_mtime_ns
    except OSError:
        mtime_ns = -1
    return mtime_ns, path.name


def evidence_freshness(
    path: Path | None,
    *,
    now_s: float,
    max_age_s: float,
) -> dict[str, Any]:
    if path is None:
        return {
            "status": MISSING,
            "path": None,
            "age_s": None,
            "max_age_s": max_age_s,
        }
    try:
        mtime_s = path.stat().st_mtime
    except OSError as exc:
        return {
            "status": MISSING,
            "path": str(path),
            "age_s": None,
            "max_age_s": max_age_s,
            "error": str(exc),
        }
    age_s = max(0.0, now_s - mtime_s)
    return {
        "status": PASS if age_s <= max_age_s else STALE,
        "path": str(path),
        "age_s": age_s,
        "max_age_s": max_age_s,
    }


def normalized_path(path_text: str | None) -> str | None:
    if not path_text:
        return None
    return str(Path(path_text).expanduser().resolve())


def manifest_suite_consistency(
    manifest_report: dict[str, Any] | None,
    manifest_path: Path | None,
    suite_path: Path | None,
) -> dict[str, Any]:
    if manifest_path is None:
        return {
            "status": MISSING,
            "manifest_path": None,
            "manifest_suite_report": None,
            "latest_suite": str(suite_path) if suite_path else None,
        }
    manifest_suite = (
        str(manifest_report.get("suite_report"))
        if isinstance(manifest_report, dict) and manifest_report.get("suite_report")
        else None
    )
    if manifest_suite is None:
        return {
            "status": MISMATCH,
            "manifest_path": str(manifest_path),
            "manifest_suite_report": None,
            "latest_suite": str(suite_path) if suite_path else None,
            "reason": "manifest does not declare suite_report",
        }
    if suite_path is None:
        return {
            "status": MISMATCH,
            "manifest_path": str(manifest_path),
            "manifest_suite_report": manifest_suite,
            "latest_suite": None,
            "reason": "manifest declares suite_report but latest suite is missing",
        }
    manifest_suite_normalized = normalized_path(manifest_suite)
    latest_suite_normalized = normalized_path(str(suite_path))
    status = PASS if manifest_suite_normalized == latest_suite_normalized else MISMATCH
    return {
        "status": status,
        "manifest_path": str(manifest_path),
        "manifest_suite_report": manifest_suite,
        "latest_suite": str(suite_path),
        "manifest_suite_normalized": manifest_suite_normalized,
        "latest_suite_normalized": latest_suite_normalized,
    }


def latest_game_screen_decision(log_dir: Path) -> tuple[Path | None, dict[str, Any] | None]:
    candidates: list[tuple[Path, dict[str, Any]]] = []
    for path in log_dir.glob("*-decision.json"):
        data = load_json_report(path)
        if not isinstance(data, dict):
            continue
        decision = data.get("decision")
        scope = data.get("metrics", {}).get("scope") if isinstance(data.get("metrics"), dict) else None
        if decision in {GAME_SCREEN_READY, WAITING, REJECT} and scope == GAME_SCREEN_SCOPE:
            candidates.append((path, data))
    if not candidates:
        return None, None
    path, data = max(candidates, key=lambda item: report_sort_key(item[0]))
    return path, data


def dict_statuses(value: Any) -> dict[str, str]:
    if not isinstance(value, dict):
        return {}
    return {
        str(key): str(status)
        for key, status in value.items()
    }


def list_strings(value: Any) -> list[str]:
    if not isinstance(value, list):
        return []
    return [str(item) for item in value]


def game_screen_decision_schema(report: dict[str, Any] | None) -> dict[str, Any]:
    if report is None:
        return {
            "status": MISSING,
            "missing_synthetic_required_phases": list(GAME_SCREEN_SYNTHETIC_REQUIRED_PHASES),
            "missing_simple_window_required_phases": list(GAME_SCREEN_SIMPLE_WINDOW_REQUIRED_PHASES),
            "non_pass_synthetic_phases": list(GAME_SCREEN_SYNTHETIC_REQUIRED_PHASES),
            "non_pass_simple_window_phases": list(GAME_SCREEN_SIMPLE_WINDOW_REQUIRED_PHASES),
        }
    metrics = report.get("metrics") if isinstance(report.get("metrics"), dict) else {}
    declared_synthetic_required = list_strings(metrics.get("synthetic_required_phases"))
    declared_simple_required = list_strings(metrics.get("simple_window_required_phases"))
    synthetic_statuses = dict_statuses(metrics.get("synthetic_statuses"))
    simple_window_statuses = dict_statuses(metrics.get("simple_window_phase_statuses"))
    missing_synthetic_required = [
        phase for phase in GAME_SCREEN_SYNTHETIC_REQUIRED_PHASES
        if phase not in declared_synthetic_required
    ]
    missing_simple_required = [
        phase for phase in GAME_SCREEN_SIMPLE_WINDOW_REQUIRED_PHASES
        if phase not in declared_simple_required
    ]
    non_pass_synthetic = [
        phase for phase in GAME_SCREEN_SYNTHETIC_REQUIRED_PHASES
        if synthetic_statuses.get(phase) != PASS
    ]
    non_pass_simple = [
        phase for phase in GAME_SCREEN_SIMPLE_WINDOW_REQUIRED_PHASES
        if simple_window_statuses.get(phase) != PASS
    ]
    schema_ok = not (
        missing_synthetic_required
        or missing_simple_required
        or non_pass_synthetic
        or non_pass_simple
    )
    return {
        "status": PASS if schema_ok else MISMATCH,
        "required_synthetic_phases": list(GAME_SCREEN_SYNTHETIC_REQUIRED_PHASES),
        "required_simple_window_phases": list(GAME_SCREEN_SIMPLE_WINDOW_REQUIRED_PHASES),
        "declared_synthetic_required_phases": declared_synthetic_required,
        "declared_simple_window_required_phases": declared_simple_required,
        "synthetic_statuses": synthetic_statuses,
        "simple_window_phase_statuses": simple_window_statuses,
        "missing_synthetic_required_phases": missing_synthetic_required,
        "missing_simple_window_required_phases": missing_simple_required,
        "non_pass_synthetic_phases": non_pass_synthetic,
        "non_pass_simple_window_phases": non_pass_simple,
    }


def command_block(*parts: str) -> str:
    separator = " \\\n  "
    return separator.join(parts)


def build_commands(
    run_id: str,
    bbox: tuple[int, int, int, int] | None,
    *,
    game_log_dir: Path = GAME_SCREEN_LOG_DIR,
) -> dict[str, str]:
    bbox_arg = ",".join(str(value) for value in bbox) if bbox else "x,y,w,h"
    game_log_arg = str(game_log_dir)
    return {
        "game_screen_acceptance": command_block(
            "fpv_env/bin/python experiments/game_screen_sandbox/sandbox_acceptance_runner.py",
            "--include-simple-window",
            "--run-id %s-game-screen" % run_id,
        ),
        "pr0p_install_discovery": command_block(
            "fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_install_probe.py",
            "--install-root /tmp/fpv-test-simitl-pr0p",
            "--run-id %s-pr0p-install" % run_id,
        ),
        "pr0p_install_download": command_block(
            "fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_install_probe.py",
            "--install-root /tmp/fpv-test-simitl-pr0p",
            "--download",
            "--run-id %s-pr0p-install-download" % run_id,
        ),
        "pr0p_client_probe": command_block(
            "fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_client_probe.py",
            "--install-root /tmp/fpv-test-simitl-pr0p",
            "--run-id %s-pr0p-client" % run_id,
        ),
        "pr0p_updater_runner_dry": command_block(
            "fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_updater_runner.py",
            "--install-root /tmp/fpv-test-simitl-pr0p",
            "--run-id %s-pr0p-updater-dry" % run_id,
        ),
        "pr0p_updater_runner_launch": command_block(
            "fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_updater_runner.py",
            "--install-root /tmp/fpv-test-simitl-pr0p",
            "--launch-updater",
            "--ack-external-binary",
            "--leave-running",
            "--ack-leave-running",
            "--run-id %s-pr0p-updater-launch" % run_id,
        ),
        "pr0p_live_session_dry": command_block(
            "fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_live_session_runner.py",
            "--run-id %s-pr0p-live-session-dry" % run_id,
        ),
        "pr0p_safe_suite": command_block(
            "fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_probe_suite.py",
            "--run-id %s-pr0p-suite" % run_id,
        ),
        "pr0p_safe_suite_with_bbox": command_block(
            "fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_probe_suite.py",
            "--tracking-bbox %s" % bbox_arg,
            "--run-id %s-pr0p-suite-bbox" % run_id,
        ),
        "pr0p_live_manifest": command_block(
            "fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_live_run_manifest.py",
            "--bbox %s" % bbox_arg if bbox else "# add --bbox x,y,w,h after target selection",
            "--run-id %s-pr0p-live" % run_id,
        ),
        "pr0p_runtime_input_visual": command_block(
            "fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_runtime_input_visual_probe.py",
            "--uinput",
            "--ack-live-input",
            "--axis yaw",
            "--magnitude 0.6",
            "--run-id %s-pr0p-rc-visual" % run_id,
        ),
        "pr0p_aux1_config_patch_dry": command_block(
            "fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_input_config_patch.py",
            "--role aux1",
            "--run-id %s-pr0p-aux1-config-dry" % run_id,
        ),
        "pr0p_aux1_config_patch_write": command_block(
            "fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_input_config_patch.py",
            "--role aux1",
            "--write",
            "--ack-config-write",
            "--run-id %s-pr0p-aux1-config-write" % run_id,
        ),
        "pr0p_aux1_mapping_assistant_dry": command_block(
            "fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_rc_channel_mapping_assistant.py",
            "--role aux1",
            "--run-id %s-pr0p-aux1-map-dry" % run_id,
        ),
        "pr0p_aux1_mapping_assistant_live": command_block(
            "fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_rc_channel_mapping_assistant.py",
            "--uinput",
            "--ack-live-input",
            "--role aux1",
            "--run-id %s-pr0p-aux1-map-live" % run_id,
        ),
        "pr0p_aux1_rc_effect": command_block(
            "fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_live_session_runner.py",
            "--launch-pr0p",
            "--ack-live-launch",
            "--send-ui",
            "--ack-live-ui",
            "--hold-uinput",
            "--ack-live-input",
            "--run-rc-effect",
            "--rc-effect-axis aux1",
            "--rc-effect-magnitude 1.0",
            "--rc-effect-expected-channel aux1",
            "--rc-effect-expected-direction higher",
            "--startup-wait 8",
            "--run-id %s-pr0p-aux1-rc-effect" % run_id,
        ),
        "pr0p_aux1_arm_status": command_block(
            "fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_live_session_runner.py",
            "--launch-pr0p",
            "--ack-live-launch",
            "--send-ui",
            "--ack-live-ui",
            "--hold-uinput",
            "--ack-live-input",
            "--run-aux-arm-status",
            "--aux-arm-throttle-magnitude 1.0",
            "--aux-arm-aux1-magnitude 1.0",
            "--aux-arm-baseline-samples 20",
            "--aux-arm-during-samples 6",
            "--aux-arm-interval 1.0",
            "--aux-arm-settle 0.5",
            "--startup-wait 8",
            "--run-id %s-pr0p-aux1-arm-status" % run_id,
        ),
        "pr0p_aux1_acceptance_plan": command_block(
            "fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_aux1_acceptance_runner.py",
            "--bbox %s" % bbox_arg if bbox else "# add --bbox x,y,w,h after target selection",
            "--run-id %s-pr0p-aux1-acceptance-plan" % run_id,
        ),
        "pr0p_aux1_acceptance_live": command_block(
            "fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_aux1_acceptance_runner.py",
            "--bbox %s" % bbox_arg if bbox else "# add --bbox x,y,w,h after target selection",
            "--execute-dry-patch",
            "--run-live-gates",
            "--ack-live-launch",
            "--ack-live-ui",
            "--ack-live-input",
            "--run-id %s-pr0p-aux1-acceptance-live" % run_id,
        ),
        "pr0p_response_acceptance_plan": command_block(
            "fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_response_acceptance_runner.py",
            "--run-id %s-pr0p-response-acceptance-plan" % run_id,
        ),
        "pr0p_response_acceptance_live": command_block(
            "fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_response_acceptance_runner.py",
            "--run-live-gates",
            "--ack-live-launch",
            "--ack-live-ui",
            "--ack-live-input",
            *PROVEN_LIVE_PROFILE,
            "--run-id %s-pr0p-response-acceptance-live" % run_id,
        ),
        "pr0p_tracking_acceptance_plan": command_block(
            "fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_tracking_acceptance_runner.py",
            "--bbox %s" % bbox_arg if bbox else "# add --bbox x,y,w,h after target selection",
            "--run-id %s-pr0p-tracking-acceptance-plan" % run_id,
        ),
        "pr0p_tracking_acceptance_live": command_block(
            "fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_tracking_acceptance_runner.py",
            "--bbox %s" % bbox_arg if bbox else "# add --bbox x,y,w,h after target selection",
            "--run-live-gates",
            "--ack-live-input",
            *TRACKING_LIVE_PROFILE,
            "--run-id %s-pr0p-tracking-acceptance-live" % run_id,
        ),
        "pr0p_acceptance_chain_plan": command_block(
            "fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_acceptance_chain_runner.py",
            "--bbox %s" % bbox_arg if bbox else "# add --bbox x,y,w,h after target selection",
            "--game-log-dir %s" % game_log_arg,
            "--run-id %s-pr0p-acceptance-chain-plan" % run_id,
        ),
        "pr0p_acceptance_state": command_block(
            "fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_acceptance_state_report.py",
            "--bbox %s" % bbox_arg if bbox else "# add --bbox x,y,w,h after target selection",
            "--run-id %s-pr0p-acceptance-state" % run_id,
        ),
        "pr0p_core_goal_status": command_block(
            "fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_core_goal_report.py",
            "--bbox %s" % bbox_arg if bbox else "# add --bbox x,y,w,h after target selection",
            "--run-id %s-pr0p-core-goal" % run_id,
        ),
        "pr0p_core_next_plan": command_block(
            "fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_core_next_runner.py",
            "--bbox %s" % bbox_arg if bbox else "# add --bbox x,y,w,h after target selection",
            "--run-id %s-pr0p-core-next-plan" % run_id,
        ),
        "pr0p_core_next_execute_with_ack": command_block(
            "fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_core_next_runner.py",
            "--bbox %s" % bbox_arg if bbox else "# add --bbox x,y,w,h after target selection",
            "--execute-next",
            "--ack-live-command",
            "--run-id %s-pr0p-core-next-execute" % run_id,
        ),
        "pr0p_acceptance_chain_live": command_block(
            "fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_acceptance_chain_runner.py",
            "--bbox %s" % bbox_arg if bbox else "# add --bbox x,y,w,h after target selection",
            "--execute-dry-patch",
            "--run-live-gates",
            "--ack-live-launch",
            "--ack-live-ui",
            "--ack-live-input",
            *PROVEN_LIVE_PROFILE,
            "--game-log-dir %s" % game_log_arg,
            "--run-id %s-pr0p-acceptance-chain-live" % run_id,
        ),
        "pr0p_extended_follow_live": command_block(
            "fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_acceptance_chain_runner.py",
            "--bbox %s" % bbox_arg if bbox else "# add --bbox x,y,w,h after target selection",
            "--execute-dry-patch",
            "--run-live-gates",
            "--ack-live-launch",
            "--ack-live-ui",
            "--ack-live-input",
            *PROVEN_LIVE_PROFILE,
            *EXTENDED_FOLLOW_PROFILE,
            "--game-log-dir %s" % game_log_arg,
            "--run-id %s-pr0p-extended-follow-live" % run_id,
        ),
        "pr0p_decision": command_block(
            "fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_decision_report.py",
            "--run-id %s-pr0p-decision" % run_id,
        ),
    }


def suite_phase_details(report: dict[str, Any] | None) -> dict[str, dict[str, Any]]:
    if not report or not isinstance(report.get("steps"), list):
        return {}
    details: dict[str, dict[str, Any]] = {}
    for step in report["steps"]:
        if not isinstance(step, dict) or not step.get("phase"):
            continue
        phase = str(step["phase"])
        metrics = step.get("metrics") if isinstance(step.get("metrics"), dict) else {}
        install_scan_present = isinstance(metrics.get("install_scan"), dict)
        install_scan = metrics.get("install_scan") if install_scan_present else {}
        client_candidates = install_scan.get("client_candidates") if isinstance(install_scan.get("client_candidates"), list) else []
        executables = install_scan.get("executables") if isinstance(install_scan.get("executables"), list) else []
        details[phase] = {
            "status": str(step.get("status", UNKNOWN)),
            "summary": str(step.get("summary", "")),
            "install_scan_present": install_scan_present,
            "client_candidate_count": len(client_candidates),
            "executable_names": [
                str(item.get("name"))
                for item in executables
                if isinstance(item, dict) and item.get("name")
            ],
        }
    return details


def decide_next_action(
    *,
    game_decision: str,
    pr0p_decision: str,
    pr0p_decision_reasons: list[str],
    game_freshness_status: str,
    pr0p_freshness_status: str,
    pr0p_manifest_freshness_status: str,
    pr0p_manifest_suite_status: str,
    pr0p_phase_details: dict[str, dict[str, Any]],
    pr0p_manifest_next_action: str | None,
) -> str:
    if game_freshness_status == STALE:
        return "Rerun the game-screen acceptance command because its latest readiness evidence is stale."
    if game_decision != GAME_SCREEN_READY:
        return "Run the game-screen acceptance command before trusting tracker/PID dry-run evidence."
    if pr0p_freshness_status == STALE:
        return "Rerun the pr0p safe suite because its latest suite evidence is stale."
    p1 = pr0p_phase_details.get("P1-install-discovery", {})
    p1_status = p1.get("status", UNKNOWN)
    p1_summary = p1.get("summary") or "no install-discovery summary"
    if p1_status in (MISSING, UNKNOWN):
        return "Run the isolated pr0p install discovery command before starting pr0p or a local race."
    if p1_status == FAIL:
        return "Fix isolated pr0p install discovery or install-root access before starting pr0p."
    if p1_status == WAITING:
        return (
            "Download or install pr0p in the isolated root first "
            "(P1-install-discovery: %s), then rerun the safe suite."
        ) % p1_summary
    p1_client = pr0p_phase_details.get("P1-client-executable", {})
    p1_client_status = p1_client.get("status", UNKNOWN)
    if p1_client_status == FAIL:
        return "Fix the isolated pr0p client executable probe before starting pr0p."
    if p1_client_status in (MISSING, UNKNOWN, WAITING):
        return (
            "Run the pr0p updater runner dry-run, then launch the updater with "
            "explicit ack or manually, and rerun the pr0p client probe until a "
            "client executable is visible."
        )
    if (
        p1_status == PASS
        and p1.get("install_scan_present")
        and p1.get("client_candidate_count") == 0
    ):
        return (
            "Run the pr0p updater runner dry-run, then launch the updater with "
            "explicit ack or manually, and rerun the pr0p client probe until a "
            "client executable is visible."
        )
    if pr0p_decision != PROMOTE_CANDIDATE:
        if pr0p_manifest_freshness_status == MISSING and pr0p_decision_reasons == [
            "MISSING_LIVE_MANIFEST_EVIDENCE"
        ]:
            return "Generate a live manifest from the latest acceptance evidence before marking the independent sim path ready."
        if pr0p_manifest_freshness_status == STALE and pr0p_decision_reasons == [
            "EVIDENCE_STALE"
        ]:
            return "Regenerate the pr0p live manifest because its latest promotion evidence is stale."
        if pr0p_manifest_suite_status == MISMATCH:
            return "Regenerate the pr0p live manifest from the latest safe suite before marking the independent sim path ready."
        return pr0p_manifest_next_action or "Run the pr0p safe suite, then generate the live manifest."
    return (
        "Both independent paths are ready; run pr0p_core_goal_report.py or "
        "pr0p_core_next_runner.py to continue the measured extended-follow, "
        "yaw, approach, and handoff gates."
    )


def build_readiness(
    *,
    run_id: str,
    game_log_dir: Path,
    pr0p_log_dir: Path,
    bbox: tuple[int, int, int, int] | None,
    window_title: str,
    yaw_expected_sign: int,
    pitch_expected_sign: int,
    max_evidence_age_s: float = DEFAULT_MAX_EVIDENCE_AGE_S,
    now_s: float | None = None,
) -> ReadinessReport:
    if now_s is None:
        now_s = time.time()
    game_path, game_report = latest_game_screen_decision(game_log_dir)
    raw_game_decision = str(game_report.get("decision", UNKNOWN)) if game_report else UNKNOWN
    suite_path = latest_suite_report(pr0p_log_dir)
    suite_report = load_json_report(suite_path)
    manifest_path = latest_manifest_report(pr0p_log_dir)
    manifest_report = load_json_report(manifest_path)
    game_freshness = evidence_freshness(
        game_path,
        now_s=now_s,
        max_age_s=max_evidence_age_s,
    )
    game_schema = game_screen_decision_schema(game_report)
    pr0p_freshness = evidence_freshness(
        suite_path,
        now_s=now_s,
        max_age_s=max_evidence_age_s,
    )
    manifest_freshness = evidence_freshness(
        manifest_path,
        now_s=now_s,
        max_age_s=max_evidence_age_s,
    )
    manifest_suite = manifest_suite_consistency(
        manifest_report,
        manifest_path,
        suite_path,
    )
    game_decision = raw_game_decision
    if game_freshness["status"] != PASS:
        game_decision = game_freshness["status"]
    elif game_schema["status"] != PASS:
        game_decision = game_schema["status"]
    manifest_preview = build_manifest(
        run_id="%s-manifest-preview" % run_id,
        log_dir=pr0p_log_dir,
        suite_report=suite_path,
        bbox=bbox,
        window_title=window_title,
        yaw_expected_sign=yaw_expected_sign,
        pitch_expected_sign=pitch_expected_sign,
    )
    decision_source = manifest_report if manifest_report is not None else suite_report
    decision_path = manifest_path if manifest_report is not None else suite_path
    pr0p_decision_report = evaluate_pr0p_decision(
        decision_source,
        evidence_path=decision_path,
        max_evidence_age_s=max_evidence_age_s,
        now_s=now_s,
    )
    phase_details = suite_phase_details(suite_report)
    from pr0p_core_goal_report import build_core_goal_report  # noqa: WPS433

    core_goal_preview = build_core_goal_report(
        run_id="%s-core-goal-preview" % run_id,
        log_dir=pr0p_log_dir,
        bbox=bbox,
        max_evidence_age_s=max_evidence_age_s,
        now_s=now_s,
    )
    pr0p_decision = pr0p_decision_report.decision
    if pr0p_freshness["status"] != PASS:
        pr0p_decision = WAITING
    if manifest_freshness["status"] != PASS:
        pr0p_decision = WAITING
    if manifest_suite["status"] != PASS:
        pr0p_decision = WAITING
    commands = build_commands(run_id, bbox, game_log_dir=game_log_dir)
    reasons: list[str] = []
    if game_freshness["status"] == STALE:
        reasons.append("GAME_SCREEN_EVIDENCE_STALE")
    elif game_freshness["status"] == MISSING:
        reasons.append("GAME_SCREEN_EVIDENCE_MISSING")
    if game_decision != GAME_SCREEN_READY:
        reasons.append("GAME_SCREEN_NOT_READY:%s" % game_decision)
    if game_schema["status"] == MISMATCH:
        reasons.append("GAME_SCREEN_DECISION_SCHEMA_MISMATCH")
    if pr0p_freshness["status"] == STALE:
        reasons.append("PR0P:EVIDENCE_STALE")
    elif pr0p_freshness["status"] == MISSING:
        reasons.append("PR0P:EVIDENCE_MISSING")
    if manifest_freshness["status"] == STALE:
        reasons.append("PR0P:LIVE_MANIFEST_EVIDENCE_STALE")
    elif manifest_freshness["status"] == MISSING:
        reasons.append("PR0P:LIVE_MANIFEST_EVIDENCE_MISSING")
    if manifest_suite["status"] == MISMATCH:
        reasons.append("PR0P:LIVE_MANIFEST_SUITE_MISMATCH")
    if pr0p_decision_report.decision != PROMOTE_CANDIDATE:
        reasons.extend("PR0P:%s" % reason for reason in pr0p_decision_report.reasons)

    if game_decision == GAME_SCREEN_READY and pr0p_decision == PROMOTE_CANDIDATE:
        status = INDEPENDENT_SIM_READY
        summary = "Gazebo-independent sim path has the required promotion evidence"
    elif pr0p_decision_report.decision == REJECT and pr0p_freshness["status"] == PASS:
        status = REJECT
        summary = "Gazebo-independent sim path has a rejecting pr0p evidence gate"
    else:
        status = WAITING
        summary = "Gazebo-independent sim path is waiting for fresh live evidence"

    return ReadinessReport(
        run_id=run_id,
        started_at=time.strftime("%Y-%m-%dT%H:%M:%S%z", time.localtime()),
        status=status,
        summary=summary,
        next_action=decide_next_action(
            game_decision=game_decision,
            pr0p_decision=pr0p_decision,
            pr0p_decision_reasons=pr0p_decision_report.reasons,
            game_freshness_status=str(game_freshness["status"]),
            pr0p_freshness_status=str(pr0p_freshness["status"]),
            pr0p_manifest_freshness_status=str(manifest_freshness["status"]),
            pr0p_manifest_suite_status=str(manifest_suite["status"]),
            pr0p_phase_details=phase_details,
            pr0p_manifest_next_action=manifest_preview.next_action,
        ),
        objective_completion_status=core_goal_preview.completion_status,
        objective_completion_summary=core_goal_preview.completion_summary,
        objective_completion_missing=core_goal_preview.completion_missing,
        objective_completion_failed=core_goal_preview.completion_failed,
        evidence_paths={
            "game_screen_decision": str(game_path) if game_path else None,
            "pr0p_suite": str(suite_path) if suite_path else None,
            "pr0p_live_manifest": str(manifest_path) if manifest_path else None,
        },
        reasons=reasons,
        commands=commands,
        metrics={
            "objective_completion_status": core_goal_preview.completion_status,
            "objective_completion_summary": core_goal_preview.completion_summary,
            "objective_completion_missing": core_goal_preview.completion_missing,
            "objective_completion_failed": core_goal_preview.completion_failed,
            "core_goal_status": core_goal_preview.status,
            "core_goal_next_command_key": core_goal_preview.next_command_key,
            "core_goal_next_action": core_goal_preview.next_action,
            "core_goal_gate_statuses": {
                gate.name: gate.status for gate in core_goal_preview.gates
            },
            "core_goal_post_core_gate_statuses": {
                gate.name: gate.status for gate in core_goal_preview.post_core_gates
            },
            "max_evidence_age_s": max_evidence_age_s,
            "evidence_freshness": {
                "game_screen_decision": game_freshness,
                "pr0p_suite": pr0p_freshness,
                "pr0p_live_manifest": manifest_freshness,
            },
            "game_screen_decision_schema": game_schema,
            "pr0p_manifest_suite_consistency": manifest_suite,
            "raw_game_screen_decision": raw_game_decision,
            "game_screen_decision": game_decision,
            "game_screen_summary": game_report.get("summary") if game_report else None,
            "raw_pr0p_decision": pr0p_decision_report.decision,
            "pr0p_decision": pr0p_decision,
            "pr0p_summary": pr0p_decision_report.summary,
            "pr0p_manifest_status": manifest_preview.status,
            "pr0p_manifest_next_action": manifest_preview.next_action,
            "pr0p_phase_statuses": pr0p_decision_report.metrics.get("phase_statuses", {}),
            "pr0p_preview_phase_statuses": {
                step.phase: step.status
                for step in manifest_preview.steps
            },
            "pr0p_phase_details": phase_details,
            "scope": (
                "combined Gazebo-independent readiness; reads game_screen_sandbox "
                "and simitl_pr0p_probe evidence without launching simulator, "
                "Gazebo, Betaflight, capture, or uinput"
            ),
        },
    )


def build_markdown(report: ReadinessReport) -> str:
    lines = [
        "# Independent Sim Readiness",
        "",
        "Run: `%s`" % report.run_id,
        "Started: `%s`" % report.started_at,
        "Status: `%s`" % report.status,
        "Objective completion: `%s`" % report.objective_completion_status,
        "",
        report.summary,
        "",
        report.objective_completion_summary,
        "",
        "Next action: %s" % report.next_action,
        "",
        "## Evidence",
        "",
        "| Evidence | Path |",
        "| --- | --- |",
    ]
    for key, path in report.evidence_paths.items():
        lines.append("| %s | `%s` |" % (key, path or "none"))
    lines.extend([
        "",
        "## Reasons",
        "",
    ])
    if report.reasons:
        lines.extend("- %s" % reason for reason in report.reasons)
    else:
        lines.append("- all required combined-readiness gates are present")
    lines.extend([
        "",
        "## Commands",
        "",
    ])
    for name, command in report.commands.items():
        lines.extend([
            "### %s" % name,
            "",
            "```bash",
            command,
            "```",
            "",
        ])
    lines.extend([
        "## Metrics",
        "",
        "```json",
        json.dumps(report.metrics, indent=2, sort_keys=True),
        "```",
        "",
    ])
    return "\n".join(lines)


def write_reports(report: ReadinessReport, log_dir: Path) -> tuple[Path, Path]:
    log_dir.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime("%Y%m%d-%H%M%S", time.localtime())
    json_path = log_dir / ("%s-%s-independent-sim-readiness.json" % (stamp, report.run_id))
    md_path = log_dir / ("%s-%s-independent-sim-readiness.md" % (stamp, report.run_id))
    json_path.write_text(json.dumps(report.as_dict(), indent=2, sort_keys=True) + "\n",
                         encoding="utf-8")
    md_path.write_text(build_markdown(report), encoding="utf-8")
    return json_path, md_path


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-id", default="independent-sim-readiness")
    parser.add_argument("--log-dir", type=Path, default=PR0P_LOG_DIR)
    parser.add_argument("--game-log-dir", type=Path, default=GAME_SCREEN_LOG_DIR)
    parser.add_argument("--bbox", type=parse_bbox)
    parser.add_argument("--bbox-file", type=Path)
    parser.add_argument("--window-title", default="pr0p")
    parser.add_argument("--yaw-expected-sign", type=int, choices=(-1, 0, 1), default=0)
    parser.add_argument("--pitch-expected-sign", type=int, choices=(-1, 0, 1), default=0)
    parser.add_argument("--max-evidence-age-s", type=float,
                        default=DEFAULT_MAX_EVIDENCE_AGE_S)
    args = parser.parse_args(argv)
    if args.max_evidence_age_s < 0:
        parser.error("--max-evidence-age-s must be non-negative")
    if args.bbox and args.bbox_file:
        parser.error("use either --bbox or --bbox-file, not both")
    if args.bbox_file:
        try:
            args.bbox = load_bbox_file(args.bbox_file)
        except Exception as exc:
            parser.error(str(exc))
    return args


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    report = build_readiness(
        run_id=args.run_id,
        game_log_dir=args.game_log_dir,
        pr0p_log_dir=args.log_dir,
        bbox=args.bbox,
        window_title=args.window_title,
        yaw_expected_sign=args.yaw_expected_sign,
        pitch_expected_sign=args.pitch_expected_sign,
        max_evidence_age_s=args.max_evidence_age_s,
    )
    json_path, md_path = write_reports(report, args.log_dir)
    print("independent-sim-readiness %s report=%s summary=%s" % (
        report.status,
        json_path,
        md_path,
    ))
    print(report.next_action)
    return 1 if report.status == REJECT else 0


if __name__ == "__main__":
    raise SystemExit(main())
