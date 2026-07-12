#!/usr/bin/env python3
"""High-level status for the actual pr0p sim goals.

This report is intentionally small: it answers whether the isolated pr0p path
can support RC/manual flight, camera-to-tracker, and autopilot control. It reads
existing evidence only; it does not launch pr0p, send OS input, or write config.
"""

from __future__ import annotations

import argparse
import json
import shlex
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

PROBE_DIR = Path(__file__).resolve().parent
if str(PROBE_DIR) not in sys.path:
    sys.path.insert(0, str(PROBE_DIR))

from pr0p_acceptance_state_report import (  # noqa: E402
    DEFAULT_MAX_EVIDENCE_AGE_S,
    build_acceptance_state,
)
from pr0p_capture_probe import DEFAULT_LOG_DIR  # noqa: E402
from pr0p_decision_report import (  # noqa: E402
    evidence_freshness,
    latest_suite_report,
    load_json_report,
)
from pr0p_live_run_manifest import (  # noqa: E402
    latest_tracking_acceptance_report,
    report_sort_key,
)
from pr0p_tracking_probe import load_bbox_file  # noqa: E402
from screen_tracking_loop import parse_bbox  # noqa: E402


PASS = "PASS"
WAITING = "WAITING"
FAIL = "FAIL"
UNKNOWN = "UNKNOWN"
COMPLETE = "COMPLETE"
IN_PROGRESS = "IN_PROGRESS"
FAILED = "FAILED"
EXTENDED_FOLLOW_DURATION_S = 20.0
EXTENDED_FOLLOW_MIN_FOUND_RATIO = 0.95
EXTENDED_FOLLOW_MAX_LOSS_EVENTS = 0
APPROACH_REAL_DURATION_S = 20.0
APPROACH_REAL_MIN_FOUND_RATIO = 0.90
APPROACH_REAL_MAX_LOSS_EVENTS = 2
APPROACH_DESIRED_TARGET_WIDTH = 120.0
APPROACH_REAL_MAX_YAW_AXIS = 0.8
APPROACH_REAL_MAX_PITCH_AXIS = 0.8
APPROACH_TAKEOFF_DELAY_FRAMES = 10
APPROACH_TAKEOFF_BOOST_FRAMES = 10
APPROACH_SETTLE_THROTTLE = -0.26
HANDOFF_LIVE_DURATION_S = 20.0
HANDOFF_LIVE_HZ = 10.0
HANDOFF_LIVE_TRACKER = "CSRT"
HANDOFF_LIVE_MIN_FOUND_RATIO = 0.90
HANDOFF_LIVE_MAX_LOSS_EVENTS = 2
HANDOFF_LIVE_MAX_YAW_AXIS = 0.8
HANDOFF_LIVE_MAX_PITCH_AXIS = 0.8
HANDOFF_LIVE_DESIRED_TARGET_WIDTH = 120.0
HANDOFF_LIVE_ARM_THROTTLE = -0.4
MOVING_TARGET_YAW_REQUIRED_STEPS = (
    "synthetic_moving_target_yaw",
    "real_pr0p_moving_target_yaw",
)
APPROACH_PITCH_REQUIRED_STEPS = (
    "synthetic_range_approach",
    "real_pr0p_approach_pitch",
)
HANDOFF_REQUIRED_STEPS = (
    "synthetic_manual_to_auto_handoff",
    "real_pr0p_manual_to_auto_handoff",
)
POST_CORE_GOAL_SEQUENCE = (
    "extended_follow",
    "moving_target_yaw",
    "approach_pitch",
    "handoff",
)

OBJECTIVE_REQUIREMENT_GATES = (
    (
        "manual_rc_flight",
        "rc_manual_flight",
        "manual RC flight, arm, and bounded roll/pitch/yaw response",
    ),
    (
        "camera_image_to_tracker",
        "camera_tracker",
        "FPV image capture, bbox selection, tracker, and PID dry/live evidence",
    ),
    (
        "autopilot_pid_control",
        "autopilot_control",
        "signed response plus live PID/autopilot command control",
    ),
    (
        "extended_closed_loop_follow",
        "extended_follow",
        "20 s bounded closed-loop follow before target phases",
    ),
    (
        "airborne_static_yaw_centering",
        "moving_target_yaw",
        "airborne yaw-only centering before pitch/approach",
    ),
    (
        "pitch_approach_control",
        "approach_pitch",
        "pitch-enabled approach with bbox-width error reduction",
    ),
    (
        "manual_to_auto_handoff",
        "handoff",
        "manual target selection to autonomous follow handoff",
    ),
)

RC_MANUAL_PHASES = (
    "P2-websocket",
    "P2-msp-readonly",
    "P2-fc-status-readonly",
    "P2-mode-ranges-readonly",
    "P4-input-readiness",
    "P4-input-mapping",
    "P4-rc-channels-visual",
    "P5-uinput-rc-effect-throttle-low",
    "P5-uinput-status-effect-throttle-clear",
    "P5-uinput-rc-effect-aux1-high",
    "P5-uinput-aux1-arm-status",
)

CAMERA_TRACKER_PHASES = (
    "P3-capture",
    "P6-synthetic-e2e-dry-run",
    "P6-synthetic-log-check",
    "P6-tracking-pid-dry-run",
    "P6-tracking-log-check",
)

# The safe suite cannot run live response gates, so autopilot control is
# proven through the freshness-checked acceptance-state reports instead:
# pr0p_response_acceptance requires signed yaw and pitch response PASS and
# pr0p_tracking_acceptance requires the live tracker/PID control PASS.
AUTOPILOT_CONTROL_PHASES: tuple[str, ...] = ()


def latest_rc_manual_flight_report(log_dir: Path) -> Path | None:
    reports = list(log_dir.glob("*-rc-manual-flight.json"))
    return max(reports, key=report_sort_key) if reports else None


def latest_moving_target_yaw_report(log_dir: Path) -> Path | None:
    reports = list(log_dir.glob("*-moving-target-yaw.json"))
    return max(reports, key=report_sort_key) if reports else None


def latest_moving_target_acceptance_report(log_dir: Path) -> Path | None:
    reports = list(log_dir.glob("*-moving-target-acceptance.json"))
    return max(reports, key=report_sort_key) if reports else None


def latest_approach_pitch_report(log_dir: Path) -> Path | None:
    reports = list(log_dir.glob("*-approach-pitch.json"))
    return max(reports, key=report_sort_key) if reports else None


def approach_tracking_profile_matches(report: dict[str, Any] | None) -> bool:
    if not isinstance(report, dict):
        return False
    metrics = report.get("metrics") if isinstance(report.get("metrics"), dict) else {}
    live_step = step_by_name(report, "pr0p_tracking_live")
    live_command = str(live_step.get("command", "")) if live_step else ""
    duration_s = metric_float(metrics, "duration_s")
    min_found_ratio = metric_float(metrics, "min_found_ratio")
    max_loss_events = metric_int(metrics, "max_loss_events")
    command_duration_s = command_float_value(live_command, "--duration")
    command_min_found_ratio = command_float_value(live_command, "--min-found-ratio")
    command_max_loss_events = command_int_value(live_command, "--max-loss-events")
    command_max_pitch_axis = command_float_value(live_command, "--max-abs-pitch-axis")
    return (
        report.get("status") == PASS
        and live_step is not None
        and live_step.get("status") == PASS
        and metrics.get("run_live_gates") is True
        and (
            metrics.get("enable_pitch") is True
            or command_flag_present(live_command, "--enable-pitch")
        )
        and duration_s is not None
        and duration_s >= APPROACH_REAL_DURATION_S
        and min_found_ratio is not None
        and min_found_ratio >= APPROACH_REAL_MIN_FOUND_RATIO
        and max_loss_events is not None
        and max_loss_events <= APPROACH_REAL_MAX_LOSS_EVENTS
        and command_duration_s is not None
        and command_duration_s >= APPROACH_REAL_DURATION_S
        and command_min_found_ratio is not None
        and command_min_found_ratio >= APPROACH_REAL_MIN_FOUND_RATIO
        and command_max_loss_events is not None
        and command_max_loss_events <= APPROACH_REAL_MAX_LOSS_EVENTS
        and command_flag_present(live_command, "--enable-pitch")
        and command_max_pitch_axis is not None
        and command_max_pitch_axis <= APPROACH_REAL_MAX_PITCH_AXIS
    )


def latest_approach_tracking_acceptance_report(log_dir: Path) -> Path | None:
    reports = sorted(log_dir.glob("*-tracking-acceptance.json"), key=report_sort_key)
    for path in reversed(reports):
        if approach_tracking_profile_matches(load_json_report(path)):
            return path
    return None


def latest_handoff_report(log_dir: Path) -> Path | None:
    reports = [
        path for path in log_dir.glob("*-handoff.json")
        if not path.name.endswith("-real-handoff.json")
    ]
    return max(reports, key=report_sort_key) if reports else None


def latest_real_handoff_acceptance_report(log_dir: Path) -> Path | None:
    reports = list(log_dir.glob("*-real-handoff.json"))
    return max(reports, key=report_sort_key) if reports else None


def extended_follow_profile_matches(report: dict[str, Any] | None) -> bool:
    if not isinstance(report, dict):
        return False
    metrics = report.get("metrics") if isinstance(report.get("metrics"), dict) else {}
    live_step = step_by_name(report, "pr0p_tracking_live")
    live_command = str(live_step.get("command", "")) if live_step else ""
    duration_s = metric_float(metrics, "duration_s")
    min_found_ratio = metric_float(metrics, "min_found_ratio")
    max_loss_events = metric_int(metrics, "max_loss_events")
    command_duration_s = command_float_value(live_command, "--duration")
    command_min_found_ratio = command_float_value(live_command, "--min-found-ratio")
    command_max_loss_events = command_int_value(live_command, "--max-loss-events")
    return (
        metrics.get("run_live_gates") is True
        and command_flag_present(live_command, "--arm-first")
        and duration_s is not None
        and duration_s >= EXTENDED_FOLLOW_DURATION_S
        and min_found_ratio is not None
        and min_found_ratio >= EXTENDED_FOLLOW_MIN_FOUND_RATIO
        and max_loss_events is not None
        and max_loss_events <= EXTENDED_FOLLOW_MAX_LOSS_EVENTS
        and command_duration_s is not None
        and command_duration_s >= EXTENDED_FOLLOW_DURATION_S
        and command_min_found_ratio is not None
        and command_min_found_ratio >= EXTENDED_FOLLOW_MIN_FOUND_RATIO
        and command_max_loss_events is not None
        and command_max_loss_events <= EXTENDED_FOLLOW_MAX_LOSS_EVENTS
    )


def latest_extended_follow_acceptance_report(log_dir: Path) -> Path | None:
    reports = sorted(log_dir.glob("*-tracking-acceptance.json"), key=report_sort_key)
    for path in reversed(reports):
        if extended_follow_profile_matches(load_json_report(path)):
            return path
    return latest_tracking_acceptance_report(log_dir)


def command_block(*parts: str) -> str:
    return " \\\n  ".join(parts)


def bbox_arg(bbox: tuple[int, int, int, int] | None) -> str:
    return ",".join(str(item) for item in bbox) if bbox else "x,y,w,h"


def physical_mapping_arg(allow_physical_mapping: bool) -> tuple[str, ...]:
    return ("--allow-physical-mapping",) if allow_physical_mapping else ()


def proven_live_profile_args() -> tuple[str, ...]:
    return (
        "--arm-first",
        "--arm-throttle -0.4",
        "--measure attitude",
        "--attitude-min-delta 10.0",
    )


def tracking_live_profile_args() -> tuple[str, ...]:
    return (
        "--arm-first",
        "--arm-throttle -0.4",
    )


def extended_follow_args() -> tuple[str, ...]:
    return (
        "--duration %s" % EXTENDED_FOLLOW_DURATION_S,
        "--min-found-ratio %s" % EXTENDED_FOLLOW_MIN_FOUND_RATIO,
        "--max-loss-events %s" % EXTENDED_FOLLOW_MAX_LOSS_EVENTS,
    )


def resolve_bbox_argument(
    *,
    bbox: tuple[int, int, int, int] | None,
    bbox_file: Path | None,
    parser: argparse.ArgumentParser | None = None,
) -> tuple[int, int, int, int] | None:
    if bbox and bbox_file:
        message = "use either --bbox or --bbox-file, not both"
        if parser:
            parser.error(message)
        raise ValueError(message)
    if not bbox_file:
        return bbox
    try:
        return load_bbox_file(bbox_file)
    except Exception as exc:
        if parser:
            parser.error(str(exc))
        raise


def build_core_commands(
    *,
    run_id: str,
    bbox: tuple[int, int, int, int] | None,
    allow_physical_mapping: bool = False,
    airborne_static_tracking_report: Path | str | None = None,
    approach_tracking_report: Path | str | None = None,
    real_handoff_report: Path | str | None = None,
) -> dict[str, str]:
    bbox_value = bbox_arg(bbox)
    mapping_flag = physical_mapping_arg(allow_physical_mapping)
    live_profile = proven_live_profile_args()
    tracking_profile = tracking_live_profile_args()
    extended_profile = extended_follow_args()
    airborne_static_tracking_arg = (
        str(airborne_static_tracking_report)
        if airborne_static_tracking_report
        else "path/to/airborne-static-tracking-acceptance.json"
    )
    approach_tracking_arg = (
        str(approach_tracking_report)
        if approach_tracking_report
        else "path/to/approach-tracking-acceptance.json"
    )
    real_handoff_arg = (
        str(real_handoff_report)
        if real_handoff_report
        else "path/to/real-handoff-report.json"
    )
    return {
        "core_status": command_block(
            "fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_core_goal_report.py",
            *mapping_flag,
            "--bbox %s" % bbox_value,
            "--run-id %s-core-status" % run_id,
        ),
        "rc_manual_flight_plan": command_block(
            "fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_rc_manual_flight_runner.py",
            *mapping_flag,
            "--bbox %s" % bbox_value,
            "--run-id %s-rc-manual-flight-plan" % run_id,
        ),
        "rc_manual_flight_live": command_block(
            "fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_rc_manual_flight_runner.py",
            *mapping_flag,
            "--bbox %s" % bbox_value,
            "--execute-dry-patch",
            "--run-live-gates",
            "--ack-live-launch",
            "--ack-live-ui",
            "--ack-live-input",
            *live_profile,
            "--run-id %s-rc-manual-flight-live" % run_id,
        ),
        "acceptance_chain_plan": command_block(
            "fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_acceptance_chain_runner.py",
            *mapping_flag,
            "--bbox %s" % bbox_value,
            "--run-id %s-acceptance-chain-plan" % run_id,
        ),
        "acceptance_chain_live": command_block(
            "fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_acceptance_chain_runner.py",
            *mapping_flag,
            "--bbox %s" % bbox_value,
            "--execute-dry-patch",
            "--run-live-gates",
            "--ack-live-launch",
            "--ack-live-ui",
            "--ack-live-input",
            *live_profile,
            "--run-id %s-acceptance-chain-live" % run_id,
        ),
        "extended_follow_live": command_block(
            "fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_tracking_acceptance_runner.py",
            "--bbox %s" % bbox_value,
            "--run-live-gates",
            "--ack-live-input",
            *tracking_profile,
            *extended_profile,
            "--run-id %s-extended-follow-live" % run_id,
        ),
        "moving_target_yaw_plan": command_block(
            "fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_moving_target_yaw_plan.py",
            "--bbox %s" % bbox_value,
            "--execute-synthetic",
            "--run-id %s-moving-target-yaw-plan" % run_id,
        ),
        "airborne_static_yaw_live": command_block(
            "fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_moving_target_acceptance_runner.py",
            "--bbox %s" % bbox_value,
            "--run-live-gate",
            "--ack-live-input",
            "--ack-airborne-static-target",
            "--run-id %s-airborne-static-yaw-live" % run_id,
        ),
        "airborne_static_yaw_refresh": command_block(
            "fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_moving_target_yaw_plan.py",
            "--bbox %s" % bbox_value,
            "--execute-synthetic",
            "--real-tracking-report %s" % airborne_static_tracking_arg,
            "--ack-airborne-static-target",
            "--run-id %s-airborne-static-yaw-refresh" % run_id,
        ),
        "approach_pitch_plan": command_block(
            "fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_approach_pitch_plan.py",
            "--bbox %s" % bbox_value,
            "--execute-synthetic",
            "--run-id %s-approach-pitch-plan" % run_id,
        ),
        "approach_pitch_live": command_block(
            "fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_tracking_acceptance_runner.py",
            "--bbox %s" % bbox_value,
            "--run-live-gates",
            "--ack-live-input",
            "--arm-first",
            "--arm-throttle -0.4",
            "--takeoff-delay-frames %d" % APPROACH_TAKEOFF_DELAY_FRAMES,
            "--takeoff-boost-frames %d" % APPROACH_TAKEOFF_BOOST_FRAMES,
            "--settle-throttle %s" % APPROACH_SETTLE_THROTTLE,
            "--enable-pitch",
            "--duration %s" % APPROACH_REAL_DURATION_S,
            "--min-found-ratio %s" % APPROACH_REAL_MIN_FOUND_RATIO,
            "--max-loss-events %s" % APPROACH_REAL_MAX_LOSS_EVENTS,
            "--max-abs-yaw-axis %s" % APPROACH_REAL_MAX_YAW_AXIS,
            "--max-abs-pitch-axis %s" % APPROACH_REAL_MAX_PITCH_AXIS,
            "--desired-target-width %s" % APPROACH_DESIRED_TARGET_WIDTH,
            "--run-id %s-approach-pitch-live" % run_id,
        ),
        "approach_pitch_refresh": command_block(
            "fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_approach_pitch_plan.py",
            "--bbox %s" % bbox_value,
            "--execute-synthetic",
            "--real-tracking-report %s" % approach_tracking_arg,
            "--ack-real-approach-target",
            "--run-id %s-approach-pitch-refresh" % run_id,
        ),
        "handoff_plan": command_block(
            "fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_handoff_plan.py",
            "--bbox %s" % bbox_value,
            "--execute-synthetic",
            "--run-id %s-handoff-plan" % run_id,
        ),
        "handoff_live": command_block(
            "fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_handoff_acceptance_runner.py",
            "--bbox %s" % bbox_value,
            "--duration %s" % HANDOFF_LIVE_DURATION_S,
            "--hz %s" % HANDOFF_LIVE_HZ,
            "--tracker %s" % HANDOFF_LIVE_TRACKER,
            "--min-found-ratio %s" % HANDOFF_LIVE_MIN_FOUND_RATIO,
            "--max-loss-events %s" % HANDOFF_LIVE_MAX_LOSS_EVENTS,
            "--max-abs-yaw-axis %s" % HANDOFF_LIVE_MAX_YAW_AXIS,
            "--max-abs-pitch-axis %s" % HANDOFF_LIVE_MAX_PITCH_AXIS,
            "--desired-target-width %s" % HANDOFF_LIVE_DESIRED_TARGET_WIDTH,
            "--arm-throttle %s" % HANDOFF_LIVE_ARM_THROTTLE,
            "--run-live-gate",
            "--ack-live-input",
            "--ack-real-handoff",
            "--run-id %s-handoff-live" % run_id,
        ),
        "handoff_refresh": command_block(
            "fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_handoff_plan.py",
            "--bbox %s" % bbox_value,
            "--execute-synthetic",
            "--real-handoff-report %s" % real_handoff_arg,
            "--ack-real-handoff",
            "--run-id %s-handoff-refresh" % run_id,
        ),
        "tracking_acceptance_plan": command_block(
            "fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_tracking_acceptance_runner.py",
            "--bbox %s" % bbox_value,
            "--run-id %s-tracking-acceptance-plan" % run_id,
        ),
        "tracking_acceptance_dry": command_block(
            "fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_tracking_acceptance_runner.py",
            "--bbox %s" % bbox_value,
            "--execute-dry-run",
            "--run-id %s-tracking-acceptance-dry" % run_id,
        ),
        "tracking_acceptance_live": command_block(
            "fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_tracking_acceptance_runner.py",
            "--bbox %s" % bbox_value,
            "--run-live-gates",
            "--ack-live-input",
            *tracking_profile,
            "--run-id %s-tracking-acceptance-live" % run_id,
        ),
        "response_acceptance_plan": command_block(
            "fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_response_acceptance_runner.py",
            "--run-id %s-response-acceptance-plan" % run_id,
        ),
        "response_acceptance_live": command_block(
            "fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_response_acceptance_runner.py",
            "--run-live-gates",
            "--ack-live-launch",
            "--ack-live-ui",
            "--ack-live-input",
            *live_profile,
            "--run-id %s-response-acceptance-live" % run_id,
        ),
    }


@dataclass
class CoreGoalGate:
    name: str
    status: str
    summary: str
    missing: list[str] = field(default_factory=list)
    failed: list[str] = field(default_factory=list)
    evidence: dict[str, str | None] = field(default_factory=dict)

    def as_dict(self) -> dict[str, Any]:
        return {
            "name": self.name,
            "status": self.status,
            "summary": self.summary,
            "missing": self.missing,
            "failed": self.failed,
            "evidence": self.evidence,
        }


@dataclass
class CoreGoalReport:
    run_id: str
    started_at: str
    status: str
    summary: str
    next_action: str
    completion_status: str
    completion_summary: str
    completion_missing: list[str] = field(default_factory=list)
    completion_failed: list[str] = field(default_factory=list)
    next_command_key: str | None = None
    next_command: str | None = None
    gates: list[CoreGoalGate] = field(default_factory=list)
    post_core_gates: list[CoreGoalGate] = field(default_factory=list)
    commands: dict[str, str] = field(default_factory=dict)
    metrics: dict[str, Any] = field(default_factory=dict)

    def as_dict(self) -> dict[str, Any]:
        return {
            "run_id": self.run_id,
            "started_at": self.started_at,
            "status": self.status,
            "summary": self.summary,
            "next_action": self.next_action,
            "completion_status": self.completion_status,
            "completion_summary": self.completion_summary,
            "completion_missing": self.completion_missing,
            "completion_failed": self.completion_failed,
            "next_command_key": self.next_command_key,
            "next_command": self.next_command,
            "gates": [gate.as_dict() for gate in self.gates],
            "post_core_gates": [gate.as_dict() for gate in self.post_core_gates],
            "commands": self.commands,
            "metrics": self.metrics,
        }


def gate_status_for_requirements(
    *,
    requirement_id: str,
    gate_name: str,
    summary: str,
    gates_by_name: dict[str, CoreGoalGate],
) -> dict[str, Any]:
    gate = gates_by_name.get(gate_name)
    if gate is None:
        return {
            "id": requirement_id,
            "status": WAITING,
            "summary": "%s is waiting for prerequisite gates" % summary,
            "gate": gate_name,
            "missing": [gate_name],
            "failed": [],
            "evidence": {},
        }
    return {
        "id": requirement_id,
        "status": gate.status,
        "summary": gate.summary,
        "gate": gate.name,
        "missing": list(gate.missing),
        "failed": list(gate.failed),
        "evidence": dict(gate.evidence),
    }


def objective_requirements_for(
    *,
    bbox: tuple[int, int, int, int] | None,
    gates: list[CoreGoalGate],
    post_core_gates: list[CoreGoalGate],
    commands: dict[str, str],
) -> list[dict[str, Any]]:
    gates_by_name = {gate.name: gate for gate in [*gates, *post_core_gates]}
    gazebo_command_keys = [
        name for name, command in commands.items()
        if "gazebo" in command.lower()
    ]
    requirements: list[dict[str, Any]] = [
        {
            "id": "gazebo_independent_path",
            "status": FAIL if gazebo_command_keys else PASS,
            "summary": (
                "core commands are isolated from Gazebo"
                if not gazebo_command_keys
                else "core commands still reference Gazebo"
            ),
            "gate": None,
            "missing": [],
            "failed": gazebo_command_keys,
            "evidence": {
                "command_keys": sorted(commands),
                "gazebo_command_keys": gazebo_command_keys,
            },
        },
        {
            "id": "target_bbox_selected",
            "status": PASS if bbox else WAITING,
            "summary": (
                "target bbox is selected"
                if bbox
                else "target bbox is not selected yet"
            ),
            "gate": None,
            "missing": [] if bbox else ["target_bbox"],
            "failed": [],
            "evidence": {
                "bbox": list(bbox) if bbox else None,
            },
        },
    ]
    for requirement_id, gate_name, summary in OBJECTIVE_REQUIREMENT_GATES:
        requirements.append(gate_status_for_requirements(
            requirement_id=requirement_id,
            gate_name=gate_name,
            summary=summary,
            gates_by_name=gates_by_name,
        ))
    return requirements


def requirement_statuses(requirements: list[dict[str, Any]]) -> dict[str, str]:
    return {
        str(requirement["id"]): str(requirement["status"])
        for requirement in requirements
    }


def requirement_missing(requirements: list[dict[str, Any]]) -> list[str]:
    return [
        str(requirement["id"])
        for requirement in requirements
        if requirement.get("status") not in {PASS, FAIL}
    ]


def requirement_failed(requirements: list[dict[str, Any]]) -> list[str]:
    return [
        str(requirement["id"])
        for requirement in requirements
        if requirement.get("status") == FAIL
    ]


def suite_phase_statuses(report: dict[str, Any] | None) -> dict[str, str]:
    if not report or not isinstance(report.get("steps"), list):
        return {}
    statuses: dict[str, str] = {}
    for step in report["steps"]:
        if not isinstance(step, dict) or not step.get("phase"):
            continue
        statuses[str(step["phase"])] = str(step.get("status", UNKNOWN))
    return statuses


def step_by_name(report: dict[str, Any] | None, name: str) -> dict[str, Any] | None:
    if not report or not isinstance(report.get("steps"), list):
        return None
    for step in report["steps"]:
        if isinstance(step, dict) and step.get("name") == name:
            return step
    return None


def command_tokens(command: str | None) -> list[str]:
    if not command:
        return []
    return shlex.split(command.replace("\\\n", " "))


def command_flag_present(command: str | None, flag: str) -> bool:
    return flag in command_tokens(command)


def command_float_value(command: str | None, flag: str) -> float | None:
    tokens = command_tokens(command)
    try:
        index = tokens.index(flag)
        return float(tokens[index + 1])
    except (ValueError, IndexError):
        return None


def command_int_value(command: str | None, flag: str) -> int | None:
    tokens = command_tokens(command)
    try:
        index = tokens.index(flag)
        return int(tokens[index + 1])
    except (ValueError, IndexError):
        return None


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


def moving_acceptance_tracking_report_path(
    report: dict[str, Any] | None,
) -> Path | None:
    if not isinstance(report, dict):
        return None
    metrics = report.get("metrics") if isinstance(report.get("metrics"), dict) else {}
    raw_path = metrics.get("tracking_report")
    if isinstance(raw_path, str) and raw_path:
        return Path(raw_path)
    step = step_by_name(report, "moving_target_tracking_probe")
    raw_path = step.get("report") if step else None
    if isinstance(raw_path, str) and raw_path:
        return Path(raw_path)
    return None


def moving_acceptance_ready_for_refresh(
    *,
    report: dict[str, Any] | None,
    freshness: dict[str, Any],
    tracking_report: Path | None,
) -> bool:
    return (
        isinstance(report, dict)
        and report.get("status") == PASS
        and freshness.get("status") == PASS
        and tracking_report is not None
        and tracking_report.is_file()
    )


def real_handoff_ready_for_refresh(
    *,
    report: dict[str, Any] | None,
    freshness: dict[str, Any],
) -> bool:
    return (
        isinstance(report, dict)
        and report.get("status") == PASS
        and freshness.get("status") == PASS
    )


def evaluate_phase_gate(
    *,
    name: str,
    required_phases: tuple[str, ...],
    phase_statuses: dict[str, str],
    extra_statuses: dict[str, str],
    summary_pass: str,
    summary_waiting: str,
) -> CoreGoalGate:
    missing: list[str] = []
    failed: list[str] = []
    for phase in required_phases:
        status = phase_statuses.get(phase, UNKNOWN)
        if status == FAIL:
            failed.append(phase)
        elif status != PASS:
            missing.append(phase)
    for label, status in extra_statuses.items():
        if status == FAIL:
            failed.append(label)
        elif status != PASS:
            missing.append(label)
    if failed:
        status = FAIL
        summary = "%s failed: %s" % (name, ", ".join(failed))
    elif missing:
        status = WAITING
        summary = "%s: %s" % (summary_waiting, ", ".join(missing[:4]))
    else:
        status = PASS
        summary = summary_pass
    return CoreGoalGate(
        name=name,
        status=status,
        summary=summary,
        missing=missing,
        failed=failed,
        evidence={
            "suite_phases": ",".join(required_phases),
            "extra_statuses": json.dumps(extra_statuses, sort_keys=True),
        },
    )


def evaluate_manual_flight_gate(
    *,
    manual_report: dict[str, Any] | None,
    manual_path: Path | None,
    manual_freshness: dict[str, Any],
    phase_statuses: dict[str, str],
    acceptance_statuses: dict[str, str],
) -> CoreGoalGate:
    if manual_report is None:
        return evaluate_phase_gate(
            name="rc_manual_flight",
            required_phases=RC_MANUAL_PHASES,
            phase_statuses=phase_statuses,
            extra_statuses={
                "pr0p_aux1_acceptance": acceptance_statuses.get("pr0p_aux1_acceptance", UNKNOWN),
            },
            summary_pass="RC input, AUX1/ARM, and bounded yaw/pitch/roll response are proven",
            summary_waiting="RC/manual flight evidence is missing",
        )

    raw_status = str(manual_report.get("status", UNKNOWN))
    stages = manual_report.get("stages") if isinstance(manual_report.get("stages"), list) else []
    missing = [
        str(stage.get("name", UNKNOWN))
        for stage in stages
        if (
            isinstance(stage, dict)
            and str(stage.get("status", UNKNOWN)) not in {PASS, "SKIPPED"}
        )
    ]
    failed = [
        str(stage.get("name", UNKNOWN))
        for stage in stages
        if isinstance(stage, dict) and str(stage.get("status", UNKNOWN)) == FAIL
    ]
    if manual_freshness.get("status") != PASS:
        status = WAITING
        summary = "RC/manual-flight evidence is stale or missing"
        if not missing:
            missing = ["rc_manual_flight_fresh_evidence"]
    elif raw_status == PASS:
        status = PASS
        summary = str(manual_report.get("summary") or "RC/manual-flight readiness is proven")
    elif raw_status == FAIL:
        status = FAIL
        summary = str(manual_report.get("summary") or "RC/manual-flight readiness failed")
    else:
        status = WAITING
        summary = str(manual_report.get("summary") or "RC/manual-flight readiness is waiting")
        if not missing:
            missing = ["rc_manual_flight_pass_evidence"]
    return CoreGoalGate(
        name="rc_manual_flight",
        status=status,
        summary=summary,
        missing=missing,
        failed=failed,
        evidence={
            "manual_flight_report": str(manual_path) if manual_path else None,
            "manual_flight_status": raw_status,
            "manual_flight_freshness": json.dumps(manual_freshness, sort_keys=True),
        },
    )


def evaluate_extended_follow_gate(
    *,
    tracking_report: dict[str, Any] | None,
    tracking_path: Path | None,
    tracking_freshness: dict[str, Any],
) -> CoreGoalGate:
    evidence: dict[str, str | None] = {
        "tracking_acceptance_report": str(tracking_path) if tracking_path else None,
        "tracking_acceptance_freshness": json.dumps(tracking_freshness, sort_keys=True),
        "required_duration_s": str(EXTENDED_FOLLOW_DURATION_S),
        "required_min_found_ratio": str(EXTENDED_FOLLOW_MIN_FOUND_RATIO),
        "required_max_loss_events": str(EXTENDED_FOLLOW_MAX_LOSS_EVENTS),
    }
    if tracking_report is None:
        return CoreGoalGate(
            name="extended_follow",
            status=WAITING,
            summary="20 s bounded follow evidence is missing",
            missing=["extended_follow_live"],
            evidence=evidence,
        )

    raw_status = str(tracking_report.get("status", UNKNOWN))
    live_step = step_by_name(tracking_report, "pr0p_tracking_live")
    live_status = str(live_step.get("status", UNKNOWN)) if live_step else UNKNOWN
    live_command = str(live_step.get("command", "")) if live_step else ""
    metrics = tracking_report.get("metrics") if isinstance(tracking_report.get("metrics"), dict) else {}
    duration_s = metric_float(metrics, "duration_s")
    min_found_ratio = metric_float(metrics, "min_found_ratio")
    max_loss_events = metric_int(metrics, "max_loss_events")
    run_live_gates = metrics.get("run_live_gates") is True
    command_duration_s = command_float_value(live_command, "--duration")
    command_min_found_ratio = command_float_value(live_command, "--min-found-ratio")
    command_max_loss_events = command_int_value(live_command, "--max-loss-events")
    evidence.update({
        "tracking_acceptance_status": raw_status,
        "tracking_live_status": live_status,
        "tracking_live_report": str(live_step.get("report")) if live_step else None,
        "duration_s": str(duration_s) if duration_s is not None else None,
        "min_found_ratio": str(min_found_ratio) if min_found_ratio is not None else None,
        "max_loss_events": str(max_loss_events) if max_loss_events is not None else None,
        "run_live_gates": str(run_live_gates),
        "command_duration_s": str(command_duration_s) if command_duration_s is not None else None,
        "command_min_found_ratio": (
            str(command_min_found_ratio) if command_min_found_ratio is not None else None
        ),
        "command_max_loss_events": (
            str(command_max_loss_events) if command_max_loss_events is not None else None
        ),
    })

    failed: list[str] = []
    missing: list[str] = []
    if raw_status == FAIL:
        failed.append("tracking_acceptance")
    if live_status == FAIL:
        failed.append("pr0p_tracking_live")
    if failed:
        return CoreGoalGate(
            name="extended_follow",
            status=FAIL,
            summary="20 s bounded follow evidence failed: %s" % ", ".join(failed),
            failed=failed,
            evidence=evidence,
        )

    if tracking_freshness.get("status") != PASS:
        missing.append("extended_follow_fresh_evidence")
    if raw_status != PASS:
        missing.append("tracking_acceptance_pass")
    if live_status != PASS:
        missing.append("pr0p_tracking_live_pass")
    if not run_live_gates:
        missing.append("extended_follow_live_gates")
    if duration_s is None or duration_s < EXTENDED_FOLLOW_DURATION_S:
        missing.append("extended_follow_duration_s")
    if min_found_ratio is None or min_found_ratio < EXTENDED_FOLLOW_MIN_FOUND_RATIO:
        missing.append("extended_follow_min_found_ratio")
    if max_loss_events is None or max_loss_events > EXTENDED_FOLLOW_MAX_LOSS_EVENTS:
        missing.append("extended_follow_max_loss_events")
    if not command_flag_present(live_command, "--arm-first"):
        missing.append("extended_follow_arm_first")
    if command_duration_s is None or command_duration_s < EXTENDED_FOLLOW_DURATION_S:
        missing.append("extended_follow_command_duration_s")
    if (
        command_min_found_ratio is None
        or command_min_found_ratio < EXTENDED_FOLLOW_MIN_FOUND_RATIO
    ):
        missing.append("extended_follow_command_min_found_ratio")
    if (
        command_max_loss_events is None
        or command_max_loss_events > EXTENDED_FOLLOW_MAX_LOSS_EVENTS
    ):
        missing.append("extended_follow_command_max_loss_events")
    if missing:
        return CoreGoalGate(
            name="extended_follow",
            status=WAITING,
            summary="20 s bounded follow evidence is incomplete: %s" % ", ".join(missing[:4]),
            missing=missing,
            evidence=evidence,
        )

    return CoreGoalGate(
        name="extended_follow",
        status=PASS,
        summary="20 s bounded follow is proven with live tracker/PID control",
        evidence=evidence,
    )


def evaluate_moving_target_yaw_gate(
    *,
    moving_report: dict[str, Any] | None,
    moving_path: Path | None,
    moving_freshness: dict[str, Any],
) -> CoreGoalGate:
    evidence: dict[str, str | None] = {
        "moving_target_yaw_report": str(moving_path) if moving_path else None,
        "moving_target_yaw_freshness": json.dumps(moving_freshness, sort_keys=True),
        "required_steps": ",".join(MOVING_TARGET_YAW_REQUIRED_STEPS),
    }
    if moving_report is None:
        return CoreGoalGate(
            name="moving_target_yaw",
            status=WAITING,
            summary="airborne static-target yaw-only evidence is missing",
            missing=["moving_target_yaw_plan"],
            evidence=evidence,
        )

    raw_status = str(moving_report.get("status", UNKNOWN))
    step_statuses = {
        name: str(step_by_name(moving_report, name).get("status", UNKNOWN))
        if step_by_name(moving_report, name) else UNKNOWN
        for name in MOVING_TARGET_YAW_REQUIRED_STEPS
    }
    metrics = moving_report.get("metrics") if isinstance(moving_report.get("metrics"), dict) else {}
    evidence.update({
        "moving_target_yaw_status": raw_status,
        "moving_target_yaw_step_statuses": json.dumps(step_statuses, sort_keys=True),
        "synthetic_status": str(metrics.get("synthetic_status", UNKNOWN)),
        "real_pr0p_status": str(metrics.get("real_pr0p_status", UNKNOWN)),
    })

    failed = [name for name, status in step_statuses.items() if status == FAIL]
    missing = [
        name
        for name, status in step_statuses.items()
        if status not in {PASS, FAIL}
    ]
    if raw_status == FAIL:
        failed.append("moving_target_yaw")
    elif raw_status != PASS:
        missing.append("moving_target_yaw_pass")
    if moving_freshness.get("status") != PASS:
        missing.append("moving_target_yaw_fresh_evidence")
    if failed:
        return CoreGoalGate(
            name="moving_target_yaw",
            status=FAIL,
            summary="airborne static-target yaw-only evidence failed: %s" % ", ".join(failed[:4]),
            failed=failed,
            missing=missing,
            evidence=evidence,
        )
    if missing:
        return CoreGoalGate(
            name="moving_target_yaw",
            status=WAITING,
            summary="airborne static-target yaw-only evidence is incomplete: %s" % ", ".join(missing[:4]),
            missing=missing,
            evidence=evidence,
        )
    return CoreGoalGate(
        name="moving_target_yaw",
        status=PASS,
        summary="airborne static-target yaw-only follow is proven",
        evidence=evidence,
    )


def evaluate_approach_pitch_gate(
    *,
    approach_report: dict[str, Any] | None,
    approach_path: Path | None,
    approach_freshness: dict[str, Any],
) -> CoreGoalGate:
    evidence: dict[str, str | None] = {
        "approach_pitch_report": str(approach_path) if approach_path else None,
        "approach_pitch_freshness": json.dumps(approach_freshness, sort_keys=True),
        "required_steps": ",".join(APPROACH_PITCH_REQUIRED_STEPS),
    }
    if approach_report is None:
        return CoreGoalGate(
            name="approach_pitch",
            status=WAITING,
            summary="approach/pitch evidence is missing",
            missing=["approach_pitch_plan"],
            evidence=evidence,
        )

    raw_status = str(approach_report.get("status", UNKNOWN))
    step_statuses = {
        name: str(step_by_name(approach_report, name).get("status", UNKNOWN))
        if step_by_name(approach_report, name) else UNKNOWN
        for name in APPROACH_PITCH_REQUIRED_STEPS
    }
    metrics = approach_report.get("metrics") if isinstance(approach_report.get("metrics"), dict) else {}
    evidence.update({
        "approach_pitch_status": raw_status,
        "approach_pitch_step_statuses": json.dumps(step_statuses, sort_keys=True),
        "synthetic_status": str(metrics.get("synthetic_status", UNKNOWN)),
        "real_pr0p_status": str(metrics.get("real_pr0p_status", UNKNOWN)),
    })

    failed = [name for name, status in step_statuses.items() if status == FAIL]
    missing = [
        name
        for name, status in step_statuses.items()
        if status not in {PASS, FAIL}
    ]
    if raw_status == FAIL:
        failed.append("approach_pitch")
    elif raw_status != PASS:
        missing.append("approach_pitch_pass")
    if approach_freshness.get("status") != PASS:
        missing.append("approach_pitch_fresh_evidence")
    if failed:
        return CoreGoalGate(
            name="approach_pitch",
            status=FAIL,
            summary="approach/pitch evidence failed: %s" % ", ".join(failed[:4]),
            failed=failed,
            missing=missing,
            evidence=evidence,
        )
    if missing:
        return CoreGoalGate(
            name="approach_pitch",
            status=WAITING,
            summary="approach/pitch evidence is incomplete: %s" % ", ".join(missing[:4]),
            missing=missing,
            evidence=evidence,
        )
    return CoreGoalGate(
        name="approach_pitch",
        status=PASS,
        summary="approach/pitch follow is proven",
        evidence=evidence,
    )


def evaluate_handoff_gate(
    *,
    handoff_report: dict[str, Any] | None,
    handoff_path: Path | None,
    handoff_freshness: dict[str, Any],
) -> CoreGoalGate:
    evidence: dict[str, str | None] = {
        "handoff_report": str(handoff_path) if handoff_path else None,
        "handoff_freshness": json.dumps(handoff_freshness, sort_keys=True),
        "required_steps": ",".join(HANDOFF_REQUIRED_STEPS),
    }
    if handoff_report is None:
        return CoreGoalGate(
            name="handoff",
            status=WAITING,
            summary="manual-to-autonomous handoff evidence is missing",
            missing=["handoff_plan"],
            evidence=evidence,
        )

    raw_status = str(handoff_report.get("status", UNKNOWN))
    step_statuses = {
        name: str(step_by_name(handoff_report, name).get("status", UNKNOWN))
        if step_by_name(handoff_report, name) else UNKNOWN
        for name in HANDOFF_REQUIRED_STEPS
    }
    metrics = handoff_report.get("metrics") if isinstance(handoff_report.get("metrics"), dict) else {}
    evidence.update({
        "handoff_status": raw_status,
        "handoff_step_statuses": json.dumps(step_statuses, sort_keys=True),
        "synthetic_status": str(metrics.get("synthetic_status", UNKNOWN)),
        "real_pr0p_status": str(metrics.get("real_pr0p_status", UNKNOWN)),
    })

    failed = [name for name, status in step_statuses.items() if status == FAIL]
    missing = [
        name
        for name, status in step_statuses.items()
        if status not in {PASS, FAIL}
    ]
    if raw_status == FAIL:
        failed.append("handoff")
    elif raw_status != PASS:
        missing.append("handoff_pass")
    if handoff_freshness.get("status") != PASS:
        missing.append("handoff_fresh_evidence")
    if failed:
        return CoreGoalGate(
            name="handoff",
            status=FAIL,
            summary="manual-to-autonomous handoff evidence failed: %s" % ", ".join(failed[:4]),
            failed=failed,
            missing=missing,
            evidence=evidence,
        )
    if missing:
        return CoreGoalGate(
            name="handoff",
            status=WAITING,
            summary="manual-to-autonomous handoff evidence is incomplete: %s" % ", ".join(missing[:4]),
            missing=missing,
            evidence=evidence,
        )
    return CoreGoalGate(
        name="handoff",
        status=PASS,
        summary="manual target selection to autonomous follow handoff is proven",
        evidence=evidence,
    )


def blocked_by(gate: CoreGoalGate, *, blocker: CoreGoalGate) -> CoreGoalGate:
    if gate.status == FAIL:
        return gate
    evidence = dict(gate.evidence)
    evidence.update({
        "blocked_by": blocker.name,
        "raw_status": gate.status,
        "raw_summary": gate.summary,
        "raw_missing": json.dumps(gate.missing),
    })
    return CoreGoalGate(
        name=gate.name,
        status=WAITING,
        summary="%s is blocked until %s passes" % (gate.name, blocker.name),
        missing=[blocker.name],
        failed=[],
        evidence=evidence,
    )


def apply_core_goal_dependencies(gates: list[CoreGoalGate]) -> list[CoreGoalGate]:
    by_name = {gate.name: gate for gate in gates}
    rc_gate = by_name["rc_manual_flight"]
    camera_gate = by_name["camera_tracker"]
    autopilot_gate = by_name["autopilot_control"]
    if rc_gate.status != PASS:
        camera_gate = blocked_by(camera_gate, blocker=rc_gate)
        autopilot_gate = blocked_by(autopilot_gate, blocker=rc_gate)
    elif camera_gate.status != PASS:
        autopilot_gate = blocked_by(autopilot_gate, blocker=camera_gate)
    return [rc_gate, camera_gate, autopilot_gate]


def unique(items: list[str]) -> list[str]:
    seen: set[str] = set()
    result: list[str] = []
    for item in items:
        if item in seen:
            continue
        seen.add(item)
        result.append(item)
    return result


def completion_fields_for(
    gates: list[CoreGoalGate],
    post_core_gates: list[CoreGoalGate],
) -> tuple[str, str, list[str], list[str]]:
    post_by_name = {gate.name: gate for gate in post_core_gates}
    failed: list[str] = [
        gate.name for gate in [*gates, *post_core_gates] if gate.status == FAIL
    ]
    missing: list[str] = [
        gate.name for gate in gates if gate.status not in {PASS, FAIL}
    ]
    core_ready = all(gate.status == PASS for gate in gates)
    if core_ready:
        for name in POST_CORE_GOAL_SEQUENCE:
            gate = post_by_name.get(name)
            if gate is None:
                missing.append(name)
            elif gate.status == FAIL:
                failed.append(name)
            elif gate.status != PASS:
                missing.append(name)
    else:
        missing.extend(POST_CORE_GOAL_SEQUENCE)

    failed = unique(failed)
    missing = unique([item for item in missing if item not in failed])
    if failed:
        return (
            FAILED,
            "full independent pr0p goal has failed evidence: %s"
            % ", ".join(failed[:4]),
            missing,
            failed,
        )
    if missing:
        return (
            IN_PROGRESS,
            "full independent pr0p goal is not complete yet; missing: %s"
            % ", ".join(missing[:4]),
            missing,
            failed,
        )
    return (
        COMPLETE,
        "full independent pr0p goal is complete through manual-to-autonomous handoff",
        missing,
        failed,
    )


def next_action_for(
    gates: list[CoreGoalGate],
    post_core_gates: list[CoreGoalGate] | None = None,
) -> str:
    post_core_gates = post_core_gates or []
    by_name = {gate.name: gate for gate in gates}
    if by_name["rc_manual_flight"].status != PASS:
        return (
            "Focus on RC/manual flight first: open pr0p local race, bind RC "
            "Channels including AUX1/CH5, then run pr0p_rc_manual_flight_runner.py "
            "until AUX1/ARM and bounded yaw/pitch/roll response evidence pass."
        )
    if by_name["camera_tracker"].status != PASS:
        return (
            "Focus on image and tracker next: capture the FPV view, choose a "
            "bbox, and prove tracker/PID dry-run before live follow."
        )
    if by_name["autopilot_control"].status != PASS:
        return (
            "Focus on autopilot control next: run signed yaw/pitch response and "
            "live tracking acceptance so PID commands visibly drive the sim."
        )
    extended_follow = {gate.name: gate for gate in post_core_gates}.get("extended_follow")
    moving_target = {gate.name: gate for gate in post_core_gates}.get("moving_target_yaw")
    approach_pitch = {gate.name: gate for gate in post_core_gates}.get("approach_pitch")
    handoff = {gate.name: gate for gate in post_core_gates}.get("handoff")
    if handoff and handoff.status == PASS:
        return (
            "The independent pr0p path has RC/manual flight, camera/tracker, "
            "autopilot control, airborne static-target yaw, approach/pitch, and "
            "manual-to-autonomous handoff evidence. Repeat live validation before "
            "speed tuning or real-platform work."
        )
    if handoff and handoff.status == FAIL:
        return (
            "Approach/pitch is proven, but manual-to-autonomous handoff evidence "
            "failed; inspect the latest handoff report before treating this as "
            "the operating mode."
        )
    if handoff and handoff.status == WAITING:
        if (
            "real_pr0p_manual_to_auto_handoff" in handoff.missing
            and "synthetic_manual_to_auto_handoff" not in handoff.missing
        ):
            return (
                "Synthetic handoff evidence is ready; run handoff_live to capture "
                "real manual-to-autonomous handoff evidence."
            )
        return (
            "Approach/pitch evidence is proven; run handoff_plan to refresh the "
            "safe synthetic handoff regression before live operator handoff."
        )
    if approach_pitch and approach_pitch.status == PASS:
        return (
            "Approach/pitch evidence is proven; run handoff_plan to prove the "
            "manual target selection, follow command, and autonomous control handoff."
        )
    if approach_pitch and approach_pitch.status == FAIL:
        return (
            "Airborne static-target yaw is proven, but approach/pitch evidence failed; "
            "inspect the latest approach pitch report before using pitch in live follow."
        )
    if approach_pitch and approach_pitch.status == WAITING:
        if (
            "real_pr0p_approach_pitch" in approach_pitch.missing
            and "synthetic_range_approach" not in approach_pitch.missing
        ):
            return (
                "Synthetic approach/pitch evidence is ready; run approach_pitch_live "
                "to capture real pitch-enabled approach tracking evidence."
            )
        return (
            "Airborne static-target yaw-only evidence is proven; run "
            "approach_pitch_plan to refresh the safe synthetic range regression "
            "before live pitch/approach."
        )
    if moving_target and moving_target.status == PASS:
        return (
            "Airborne static-target yaw-only evidence is proven; run approach_pitch_plan "
            "before enabling pitch in live follow."
        )
    if moving_target and moving_target.status == FAIL:
        return (
            "Extended follow is proven, but airborne static-target yaw-only evidence failed; "
            "inspect the latest yaw report before adding pitch or approach."
        )
    if moving_target and moving_target.status == WAITING:
        if (
            "real_pr0p_moving_target_yaw" in moving_target.missing
            and "synthetic_moving_target_yaw" not in moving_target.missing
        ):
            return (
                "Synthetic yaw evidence is ready; run airborne_static_yaw_live to "
                "capture real airborne static-target yaw convergence evidence."
            )
        return (
            "Extended follow is proven; run moving_target_yaw_plan to refresh the "
            "safe synthetic yaw regression before live airborne static-target evidence."
        )
    if extended_follow and extended_follow.status == PASS:
        return (
            "Extended follow is proven; run moving_target_yaw_plan to start the "
            "airborne static-target yaw-only phase, and keep approach/pitch changes behind "
            "that measured gate."
        )
    if extended_follow and extended_follow.status == FAIL:
        return (
            "Core pr0p goals are proven, but the 20 s bounded follow gate failed; "
            "inspect the latest tracking acceptance report before retrying extended_follow_live."
        )
    return (
        "Core pr0p goals are proven; run extended_follow_live until the 20 s, "
        "0-loss bounded follow gate passes."
    )


def next_command_key_for(
    gates: list[CoreGoalGate],
    post_core_gates: list[CoreGoalGate] | None = None,
) -> str | None:
    if any(gate.status == FAIL for gate in gates):
        return "core_status"
    by_name = {gate.name: gate for gate in gates}
    if by_name["rc_manual_flight"].status != PASS:
        return "rc_manual_flight_live"
    if by_name["camera_tracker"].status != PASS:
        return "tracking_acceptance_dry"
    if by_name["autopilot_control"].status != PASS:
        return "acceptance_chain_live"
    post_core_gates = post_core_gates or []
    extended_follow = {gate.name: gate for gate in post_core_gates}.get("extended_follow")
    moving_target = {gate.name: gate for gate in post_core_gates}.get("moving_target_yaw")
    approach_pitch = {gate.name: gate for gate in post_core_gates}.get("approach_pitch")
    handoff = {gate.name: gate for gate in post_core_gates}.get("handoff")
    if handoff and handoff.status == PASS:
        return None
    if handoff:
        if (
            "real_pr0p_manual_to_auto_handoff" in handoff.missing
            and "synthetic_manual_to_auto_handoff" not in handoff.missing
        ):
            return "handoff_live"
        return "handoff_plan"
    if approach_pitch and approach_pitch.status == PASS:
        return "handoff_plan"
    if approach_pitch:
        if (
            "real_pr0p_approach_pitch" in approach_pitch.missing
            and "synthetic_range_approach" not in approach_pitch.missing
        ):
            return "approach_pitch_live"
        return "approach_pitch_plan"
    if moving_target and moving_target.status == PASS:
        return "approach_pitch_plan"
    if moving_target:
        if (
            "real_pr0p_moving_target_yaw" in moving_target.missing
            and "synthetic_moving_target_yaw" not in moving_target.missing
        ):
            return "airborne_static_yaw_live"
        return "moving_target_yaw_plan"
    if extended_follow and extended_follow.status == PASS:
        return "moving_target_yaw_plan"
    return "extended_follow_live"


def build_core_goal_report(
    *,
    run_id: str,
    log_dir: Path,
    bbox: tuple[int, int, int, int] | None,
    allow_physical_mapping: bool = False,
    max_evidence_age_s: float = DEFAULT_MAX_EVIDENCE_AGE_S,
    now_s: float | None = None,
) -> CoreGoalReport:
    if now_s is None:
        now_s = time.time()
    suite_path = latest_suite_report(log_dir)
    suite_report = load_json_report(suite_path)
    manual_path = latest_rc_manual_flight_report(log_dir)
    manual_report = load_json_report(manual_path)
    manual_freshness = evidence_freshness(
        manual_path,
        now_s=now_s,
        max_age_s=max_evidence_age_s,
    )
    tracking_acceptance_path = latest_extended_follow_acceptance_report(log_dir)
    tracking_acceptance_report = load_json_report(tracking_acceptance_path)
    tracking_acceptance_freshness = evidence_freshness(
        tracking_acceptance_path,
        now_s=now_s,
        max_age_s=max_evidence_age_s,
    )
    moving_target_yaw_path = latest_moving_target_yaw_report(log_dir)
    moving_target_yaw_report = load_json_report(moving_target_yaw_path)
    moving_target_yaw_freshness = evidence_freshness(
        moving_target_yaw_path,
        now_s=now_s,
        max_age_s=max_evidence_age_s,
    )
    moving_target_acceptance_path = latest_moving_target_acceptance_report(log_dir)
    moving_target_acceptance_report = load_json_report(moving_target_acceptance_path)
    moving_target_acceptance_freshness = evidence_freshness(
        moving_target_acceptance_path,
        now_s=now_s,
        max_age_s=max_evidence_age_s,
    )
    airborne_static_tracking_report = moving_acceptance_tracking_report_path(
        moving_target_acceptance_report,
    )
    airborne_static_refresh_ready = moving_acceptance_ready_for_refresh(
        report=moving_target_acceptance_report,
        freshness=moving_target_acceptance_freshness,
        tracking_report=airborne_static_tracking_report,
    )
    approach_pitch_path = latest_approach_pitch_report(log_dir)
    approach_pitch_report = load_json_report(approach_pitch_path)
    approach_pitch_freshness = evidence_freshness(
        approach_pitch_path,
        now_s=now_s,
        max_age_s=max_evidence_age_s,
    )
    approach_tracking_path = latest_approach_tracking_acceptance_report(log_dir)
    approach_tracking_report = load_json_report(approach_tracking_path)
    approach_tracking_freshness = evidence_freshness(
        approach_tracking_path,
        now_s=now_s,
        max_age_s=max_evidence_age_s,
    )
    approach_pitch_refresh_ready = (
        approach_tracking_profile_matches(approach_tracking_report)
        and approach_tracking_freshness.get("status") == PASS
    )
    handoff_path = latest_handoff_report(log_dir)
    handoff_report = load_json_report(handoff_path)
    handoff_freshness = evidence_freshness(
        handoff_path,
        now_s=now_s,
        max_age_s=max_evidence_age_s,
    )
    real_handoff_path = latest_real_handoff_acceptance_report(log_dir)
    real_handoff_report = load_json_report(real_handoff_path)
    real_handoff_freshness = evidence_freshness(
        real_handoff_path,
        now_s=now_s,
        max_age_s=max_evidence_age_s,
    )
    handoff_refresh_ready = real_handoff_ready_for_refresh(
        report=real_handoff_report,
        freshness=real_handoff_freshness,
    )
    phases = suite_phase_statuses(suite_report)
    acceptance = build_acceptance_state(
        run_id="%s-acceptance-state" % run_id,
        log_dir=log_dir,
        bbox=bbox,
        max_evidence_age_s=max_evidence_age_s,
        now_s=now_s,
    )
    acceptance_statuses = {stage.name: stage.status for stage in acceptance.stages}
    raw_gates = [
        evaluate_manual_flight_gate(
            manual_report=manual_report,
            manual_path=manual_path,
            manual_freshness=manual_freshness,
            phase_statuses=phases,
            acceptance_statuses=acceptance_statuses,
        ),
        evaluate_phase_gate(
            name="camera_tracker",
            required_phases=CAMERA_TRACKER_PHASES,
            phase_statuses=phases,
            extra_statuses={
                "pr0p_tracking_acceptance": acceptance_statuses.get("pr0p_tracking_acceptance", UNKNOWN),
            },
            summary_pass="FPV capture and tracker/PID evidence are proven",
            summary_waiting="camera/tracker evidence is missing",
        ),
        evaluate_phase_gate(
            name="autopilot_control",
            required_phases=AUTOPILOT_CONTROL_PHASES,
            phase_statuses=phases,
            extra_statuses={
                "pr0p_response_acceptance": acceptance_statuses.get("pr0p_response_acceptance", UNKNOWN),
                "pr0p_tracking_acceptance": acceptance_statuses.get("pr0p_tracking_acceptance", UNKNOWN),
            },
            summary_pass="signed response and live tracking control evidence are proven",
            summary_waiting="autopilot control evidence is missing",
        ),
    ]
    gates = apply_core_goal_dependencies(raw_gates)
    post_core_gates: list[CoreGoalGate] = []
    if all(gate.status == PASS for gate in gates):
        extended_gate = evaluate_extended_follow_gate(
            tracking_report=tracking_acceptance_report,
            tracking_path=tracking_acceptance_path,
            tracking_freshness=tracking_acceptance_freshness,
        )
        post_core_gates.append(extended_gate)
        if extended_gate.status == PASS:
            moving_gate = evaluate_moving_target_yaw_gate(
                moving_report=moving_target_yaw_report,
                moving_path=moving_target_yaw_path,
                moving_freshness=moving_target_yaw_freshness,
            )
            post_core_gates.append(moving_gate)
            if moving_gate.status == PASS:
                approach_gate = evaluate_approach_pitch_gate(
                    approach_report=approach_pitch_report,
                    approach_path=approach_pitch_path,
                    approach_freshness=approach_pitch_freshness,
                )
                post_core_gates.append(approach_gate)
                if approach_gate.status == PASS:
                    post_core_gates.append(evaluate_handoff_gate(
                        handoff_report=handoff_report,
                        handoff_path=handoff_path,
                        handoff_freshness=handoff_freshness,
                    ))
    if any(gate.status == FAIL for gate in gates):
        status = FAIL
        summary = "one or more core pr0p goals failed"
    elif all(gate.status == PASS for gate in gates):
        status = PASS
        summary = "RC/manual flight, camera/tracker, and autopilot control are proven"
    else:
        status = WAITING
        summary = "core pr0p goals are still waiting for live evidence"
    commands = build_core_commands(
        run_id=run_id,
        bbox=bbox,
        allow_physical_mapping=allow_physical_mapping,
        airborne_static_tracking_report=(
            airborne_static_tracking_report if airborne_static_refresh_ready else None
        ),
        approach_tracking_report=(
            approach_tracking_path if approach_pitch_refresh_ready else None
        ),
        real_handoff_report=real_handoff_path if handoff_refresh_ready else None,
    )
    objective_requirements = objective_requirements_for(
        bbox=bbox,
        gates=gates,
        post_core_gates=post_core_gates,
        commands=commands,
    )
    next_command_key = next_command_key_for(gates, post_core_gates)
    if (
        airborne_static_refresh_ready
        and next_command_key in {"moving_target_yaw_plan", "airborne_static_yaw_live"}
    ):
        next_command_key = "airborne_static_yaw_refresh"
    if (
        approach_pitch_refresh_ready
        and next_command_key in {"approach_pitch_plan", "approach_pitch_live"}
    ):
        next_command_key = "approach_pitch_refresh"
    if (
        handoff_refresh_ready
        and next_command_key in {"handoff_plan", "handoff_live"}
    ):
        next_command_key = "handoff_refresh"
    next_action = next_action_for(gates, post_core_gates)
    if next_command_key == "airborne_static_yaw_refresh":
        next_action = (
            "Real airborne static-target tracking evidence is ready; run "
            "airborne_static_yaw_refresh to turn it into the canonical "
            "moving_target_yaw PASS/FAIL gate before approach/pitch."
        )
    elif next_command_key == "approach_pitch_refresh":
        next_action = (
            "Real pitch-enabled approach tracking evidence is ready; run "
            "approach_pitch_refresh to turn it into the canonical approach_pitch "
            "PASS/FAIL gate before handoff."
        )
    elif next_command_key == "handoff_refresh":
        next_action = (
            "Real manual-to-autonomous handoff evidence is ready; run "
            "handoff_refresh to turn it into the canonical handoff PASS/FAIL gate."
        )
    completion_status, completion_summary, completion_missing, completion_failed = (
        completion_fields_for(gates, post_core_gates)
    )
    return CoreGoalReport(
        run_id=run_id,
        started_at=time.strftime("%Y-%m-%dT%H:%M:%S%z", time.localtime()),
        status=status,
        summary=summary,
        next_action=next_action,
        completion_status=completion_status,
        completion_summary=completion_summary,
        completion_missing=completion_missing,
        completion_failed=completion_failed,
        next_command_key=next_command_key,
        next_command=commands.get(next_command_key),
        gates=gates,
        post_core_gates=post_core_gates,
        commands=commands,
        metrics={
            "bbox": list(bbox) if bbox else None,
            "allow_physical_mapping": allow_physical_mapping,
            "log_dir": str(log_dir),
            "suite_report": str(suite_path) if suite_path else None,
            "rc_manual_flight_report": str(manual_path) if manual_path else None,
            "rc_manual_flight_freshness": manual_freshness,
            "tracking_acceptance_report": (
                str(tracking_acceptance_path) if tracking_acceptance_path else None
            ),
            "tracking_acceptance_freshness": tracking_acceptance_freshness,
            "moving_target_yaw_report": (
                str(moving_target_yaw_path) if moving_target_yaw_path else None
            ),
            "moving_target_yaw_freshness": moving_target_yaw_freshness,
            "moving_target_acceptance_report": (
                str(moving_target_acceptance_path) if moving_target_acceptance_path else None
            ),
            "moving_target_acceptance_freshness": moving_target_acceptance_freshness,
            "airborne_static_tracking_report": (
                str(airborne_static_tracking_report)
                if airborne_static_tracking_report
                else None
            ),
            "airborne_static_yaw_refresh_ready": airborne_static_refresh_ready,
            "approach_pitch_report": (
                str(approach_pitch_path) if approach_pitch_path else None
            ),
            "approach_pitch_freshness": approach_pitch_freshness,
            "approach_tracking_acceptance_report": (
                str(approach_tracking_path) if approach_tracking_path else None
            ),
            "approach_tracking_acceptance_freshness": approach_tracking_freshness,
            "approach_pitch_refresh_ready": approach_pitch_refresh_ready,
            "handoff_report": str(handoff_path) if handoff_path else None,
            "handoff_freshness": handoff_freshness,
            "real_handoff_acceptance_report": (
                str(real_handoff_path) if real_handoff_path else None
            ),
            "real_handoff_acceptance_freshness": real_handoff_freshness,
            "handoff_refresh_ready": handoff_refresh_ready,
            "acceptance_state_status": acceptance.status,
            "acceptance_stage_statuses": acceptance_statuses,
            "post_core_gate_statuses": {
                gate.name: gate.status for gate in post_core_gates
            },
            "objective_requirements": objective_requirements,
            "objective_requirement_statuses": requirement_statuses(
                objective_requirements,
            ),
            "objective_requirement_missing": requirement_missing(
                objective_requirements,
            ),
            "objective_requirement_failed": requirement_failed(
                objective_requirements,
            ),
            "completion_status": completion_status,
            "completion_summary": completion_summary,
            "completion_missing": completion_missing,
            "completion_failed": completion_failed,
            "evidence_only": True,
            "real_input_sent": False,
        },
    )


def build_markdown(report: CoreGoalReport) -> str:
    lines = [
        "# pr0p Core Goal Status",
        "",
        "Run: `%s`" % report.run_id,
        "Started: `%s`" % report.started_at,
        "Status: `%s`" % report.status,
        "Completion: `%s`" % report.completion_status,
        "",
        report.summary,
        "",
        report.completion_summary,
        "",
        "Next action: %s" % report.next_action,
        "",
        "Next command key: `%s`" % (report.next_command_key or "-"),
        "",
    ]
    if report.next_command:
        lines.extend(["```bash", report.next_command, "```", ""])
    lines.extend([
        "| Core Goal | Status | Summary |",
        "| --- | --- | --- |",
    ])
    for gate in report.gates:
        lines.append("| %s | `%s` | %s |" % (gate.name, gate.status, gate.summary))
    if report.post_core_gates:
        lines.extend(["", "## Post-Core Gates", "", "| Gate | Status | Summary |", "| --- | --- | --- |"])
        for gate in report.post_core_gates:
            lines.append("| %s | `%s` | %s |" % (gate.name, gate.status, gate.summary))
    requirements = report.metrics.get("objective_requirements")
    if isinstance(requirements, list):
        lines.extend([
            "",
            "## Objective Requirements",
            "",
            "| Requirement | Status | Gate | Summary |",
            "| --- | --- | --- | --- |",
        ])
        for requirement in requirements:
            if not isinstance(requirement, dict):
                continue
            lines.append("| %s | `%s` | %s | %s |" % (
                requirement.get("id", "-"),
                requirement.get("status", UNKNOWN),
                requirement.get("gate") or "-",
                requirement.get("summary", ""),
            ))
    lines.extend(["", "## Commands", ""])
    for name, command in report.commands.items():
        lines.extend(["### %s" % name, "", "```bash", command, "```", ""])
    lines.extend(["", "## Metrics", "", "```json"])
    lines.append(json.dumps(report.metrics, indent=2, sort_keys=True))
    lines.extend(["```", ""])
    return "\n".join(lines)


def write_reports(report: CoreGoalReport, log_dir: Path) -> tuple[Path, Path]:
    log_dir.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime("%Y%m%d-%H%M%S", time.localtime())
    json_path = log_dir / ("%s-%s-core-goal.json" % (stamp, report.run_id))
    md_path = log_dir / ("%s-%s-core-goal.md" % (stamp, report.run_id))
    json_path.write_text(json.dumps(report.as_dict(), indent=2, sort_keys=True) + "\n",
                         encoding="utf-8")
    md_path.write_text(build_markdown(report), encoding="utf-8")
    return json_path, md_path


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-id", default="pr0p-core-goal")
    parser.add_argument("--log-dir", type=Path, default=DEFAULT_LOG_DIR)
    parser.add_argument("--bbox", type=parse_bbox)
    parser.add_argument("--bbox-file", type=Path)
    parser.add_argument("--allow-physical-mapping", action="store_true",
                        help="Propagate physical RC mapping acceptance to next commands")
    parser.add_argument("--max-evidence-age-s", type=float,
                        default=DEFAULT_MAX_EVIDENCE_AGE_S)
    args = parser.parse_args(argv)
    if args.max_evidence_age_s < 0:
        parser.error("--max-evidence-age-s must be non-negative")
    args.bbox = resolve_bbox_argument(
        bbox=args.bbox,
        bbox_file=args.bbox_file,
        parser=parser,
    )
    return args


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    report = build_core_goal_report(
        run_id=args.run_id,
        log_dir=args.log_dir,
        bbox=args.bbox,
        allow_physical_mapping=args.allow_physical_mapping,
        max_evidence_age_s=args.max_evidence_age_s,
    )
    json_path, md_path = write_reports(report, args.log_dir)
    print("simitl-pr0p-core-goal %s report=%s summary=%s" % (
        report.status,
        json_path,
        md_path,
    ))
    print(report.next_action)
    return 1 if report.status == FAIL else 0


if __name__ == "__main__":
    raise SystemExit(main())
