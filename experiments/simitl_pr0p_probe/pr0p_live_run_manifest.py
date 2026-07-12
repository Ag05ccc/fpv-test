#!/usr/bin/env python3
"""Generate a live-run manifest for the isolated SimITL/pr0p probe."""

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

from pr0p_capture_probe import DEFAULT_LOG_DIR, DEFAULT_WINDOW_TITLES  # noqa: E402
from screen_tracking_loop import parse_bbox  # noqa: E402


PASS = "PASS"
WAITING = "WAITING"
FAIL = "FAIL"
UNKNOWN = "UNKNOWN"
DEFAULT_MAX_ACCEPTANCE_EVIDENCE_AGE_S = 24.0 * 60.0 * 60.0


@dataclass
class ManifestStep:
    phase: str
    status: str
    summary: str
    command: str
    evidence: str | None = None
    notes: list[str] = field(default_factory=list)

    def as_dict(self) -> dict[str, Any]:
        return {
            "phase": self.phase,
            "status": self.status,
            "summary": self.summary,
            "command": self.command,
            "evidence": self.evidence,
            "notes": self.notes,
        }


@dataclass
class LiveManifest:
    run_id: str
    started_at: str
    status: str
    next_action: str
    suite_report: str | None
    steps: list[ManifestStep]
    metadata: dict[str, Any] = field(default_factory=dict)

    def as_dict(self) -> dict[str, Any]:
        return {
            "run_id": self.run_id,
            "started_at": self.started_at,
            "status": self.status,
            "next_action": self.next_action,
            "suite_report": self.suite_report,
            "steps": [step.as_dict() for step in self.steps],
            "metadata": self.metadata,
        }


def report_sort_key(path: Path) -> tuple[int, str]:
    try:
        mtime_ns = path.stat().st_mtime_ns
    except OSError:
        mtime_ns = -1
    return mtime_ns, path.name


def latest_suite_report(log_dir: Path) -> Path | None:
    reports = list(log_dir.glob("*-suite.json"))
    return max(reports, key=report_sort_key) if reports else None


def latest_runtime_input_visual_report(log_dir: Path) -> Path | None:
    reports = list(log_dir.glob("*-runtime-input-visual.json"))
    return max(reports, key=report_sort_key) if reports else None


def latest_fc_status_report(log_dir: Path) -> Path | None:
    reports = list(log_dir.glob("*-msp-ws-status.json"))
    return max(reports, key=report_sort_key) if reports else None


def latest_mode_ranges_report(log_dir: Path) -> Path | None:
    reports = list(log_dir.glob("*-msp-ws-mode-ranges.json"))
    return max(reports, key=report_sort_key) if reports else None


def rc_effect_matches(report: dict[str, Any] | None, expected_channel: str) -> bool:
    if not isinstance(report, dict):
        return False
    metrics = report.get("metrics")
    return isinstance(metrics, dict) and metrics.get("expected_channel") == expected_channel


def latest_uinput_rc_effect_report(log_dir: Path, *, expected_channel: str) -> Path | None:
    candidates = [
        path
        for path in log_dir.glob("*-uinput-rc-effect.json")
        if rc_effect_matches(load_json_report(path), expected_channel)
    ]
    return max(candidates, key=report_sort_key) if candidates else None


def latest_uinput_status_report(log_dir: Path) -> Path | None:
    reports = list(log_dir.glob("*-uinput-status.json"))
    return max(reports, key=report_sort_key) if reports else None


def latest_uinput_arm_report(log_dir: Path) -> Path | None:
    reports = list(log_dir.glob("*-uinput-arm.json"))
    return max(reports, key=report_sort_key) if reports else None


def latest_uinput_aux_arm_status_report(log_dir: Path) -> Path | None:
    reports = list(log_dir.glob("*-uinput-aux-arm-status.json"))
    return max(reports, key=report_sort_key) if reports else None


def latest_response_acceptance_report(log_dir: Path) -> Path | None:
    reports = list(log_dir.glob("*-response-acceptance.json"))
    return max(reports, key=report_sort_key) if reports else None


def latest_tracking_acceptance_report(log_dir: Path) -> Path | None:
    reports = list(log_dir.glob("*-tracking-acceptance.json"))
    return max(reports, key=report_sort_key) if reports else None


def latest_live_session_with_metric(
    log_dir: Path,
    metric: str,
    *,
    matcher=None,
) -> Path | None:
    candidates = []
    for path in log_dir.glob("*-live-session.json"):
        data = load_json_report(path)
        if isinstance(data, dict) and isinstance(data.get("metrics"), dict):
            metric_value = data["metrics"].get(metric)
            if metric_value and (matcher is None or matcher(metric_value)):
                candidates.append(path)
    return max(candidates, key=report_sort_key) if candidates else None


def load_live_session_metric(path: Path | None, metric: str) -> dict[str, Any] | None:
    data = load_json_report(path)
    if not isinstance(data, dict) or not isinstance(data.get("metrics"), dict):
        return None
    value = data["metrics"].get(metric)
    return value if isinstance(value, dict) else None


def load_suite_report(path: Path | None) -> dict[str, Any] | None:
    if path is None:
        return None
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return None
    if not isinstance(data, dict) or "steps" not in data:
        return None
    return data


def load_json_report(path: Path | None) -> dict[str, Any] | None:
    if path is None:
        return None
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return None
    return data if isinstance(data, dict) else None


def suite_step_map(suite: dict[str, Any] | None) -> dict[str, dict[str, Any]]:
    if not suite:
        return {}
    steps = suite.get("steps", [])
    if not isinstance(steps, list):
        return {}
    return {
        str(step.get("phase")): step
        for step in steps
        if isinstance(step, dict) and step.get("phase")
    }


def command_block(*parts: str) -> str:
    return " \\\n  ".join(parts)


def build_commands(
    *,
    run_id: str,
    window_title: str,
    bbox: tuple[int, int, int, int] | None,
    yaw_expected_sign: int,
    pitch_expected_sign: int,
) -> dict[str, str]:
    bbox_text = ",".join(str(v) for v in bbox) if bbox else "x,y,w,h"
    return {
        "P0-preflight": command_block(
            "fpv_env/bin/python experiments/simitl_pr0p_probe/preflight_probe.py",
            "--install-root /tmp/fpv-test-simitl-pr0p",
            "--run-id %s-p0" % run_id,
        ),
        "P0-isolation": command_block(
            "fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_isolation_check.py",
            "--run-id %s-p0-isolation" % run_id,
        ),
        "P1-install-discovery": command_block(
            "fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_install_probe.py",
            "--install-root /tmp/fpv-test-simitl-pr0p",
            "--run-id %s-p1" % run_id,
        ),
        "P1-download": command_block(
            "fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_install_probe.py",
            "--install-root /tmp/fpv-test-simitl-pr0p",
            "--download",
            "--run-id %s-p1-download" % run_id,
        ),
        "P1-client-executable": command_block(
            "fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_client_probe.py",
            "--install-root /tmp/fpv-test-simitl-pr0p",
            "--run-id %s-p1-client" % run_id,
        ),
        "P1-updater-runner-dry": command_block(
            "fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_updater_runner.py",
            "--install-root /tmp/fpv-test-simitl-pr0p",
            "--run-id %s-p1-updater-dry" % run_id,
        ),
        "P1-updater-runner-launch": command_block(
            "fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_updater_runner.py",
            "--install-root /tmp/fpv-test-simitl-pr0p",
            "--launch-updater",
            "--ack-external-binary",
            "--leave-running",
            "--ack-leave-running",
            "--run-id %s-p1-updater-launch" % run_id,
        ),
        "P2-websocket": command_block(
            "fpv_env/bin/python experiments/simitl_pr0p_probe/msp_ws_probe.py",
            "--host 127.0.0.1",
            "--port 5761",
            "--run-id %s-p2" % run_id,
        ),
        "P2-msp-readonly": command_block(
            "fpv_env/bin/python experiments/simitl_pr0p_probe/msp_ws_semantic_probe.py",
            "--host 127.0.0.1",
            "--port 5761",
            "--run-id %s-p2-msp" % run_id,
        ),
        "P2-fc-status-readonly": command_block(
            "fpv_env/bin/python experiments/simitl_pr0p_probe/msp_ws_status_probe.py",
            "--host 127.0.0.1",
            "--port 5761",
            "--run-id %s-p2-fc-status" % run_id,
        ),
        "P2-mode-ranges-readonly": command_block(
            "fpv_env/bin/python experiments/simitl_pr0p_probe/msp_ws_mode_ranges_probe.py",
            "--host 127.0.0.1",
            "--port 5761",
            "--run-id %s-p2-mode-ranges" % run_id,
        ),
        "P1-local-race-ui-smoke": command_block(
            "fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_ui_smoke.py",
            "--send-ui",
            "--ack-live-ui",
            "--run-id %s-ui-smoke" % run_id,
        ),
        "P1-live-session-dry": command_block(
            "fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_live_session_runner.py",
            "--run-id %s-live-session-dry" % run_id,
        ),
        "P1-live-session-runner": command_block(
            "fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_live_session_runner.py",
            "--launch-pr0p",
            "--ack-live-launch",
            "--send-ui",
            "--ack-live-ui",
            "--run-suite",
            "--run-id %s-live-session" % run_id,
        ),
        "P3-capture": command_block(
            "fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_capture_probe.py",
            "--window-title %s" % shell_quote(window_title),
            "--duration 30",
            "--fps 30",
            "--min-fps 20",
            "--run-id %s-p3" % run_id,
        ),
        "P4-input-readiness": command_block(
            "fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_virtual_input_probe.py",
            "--require-uinput",
            "--run-id %s-p4" % run_id,
        ),
        "P4-input-mapping": command_block(
            "fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_input_config_probe.py",
            "--run-id %s-p4-config" % run_id,
        ),
        "P4-input-config-patch-dry": command_block(
            "fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_input_config_patch.py",
            "--run-id %s-p4-config-patch-dry" % run_id,
        ),
        "P4-input-config-patch-write": command_block(
            "fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_input_config_patch.py",
            "--write",
            "--ack-config-write",
            "--run-id %s-p4-config-patch-write" % run_id,
        ),
        "P4-input-config-patch-aux1-dry": command_block(
            "fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_input_config_patch.py",
            "--role aux1",
            "--run-id %s-p4-config-patch-aux1-dry" % run_id,
        ),
        "P4-input-config-patch-aux1-write": command_block(
            "fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_input_config_patch.py",
            "--role aux1",
            "--write",
            "--ack-config-write",
            "--run-id %s-p4-config-patch-aux1-write" % run_id,
        ),
        "P4-input-config-restore-latest": command_block(
            "fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_input_config_restore.py",
            "--run-id %s-p4-config-restore-dry" % run_id,
        ),
        "P4-input-config-restore-write": command_block(
            "fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_input_config_restore.py",
            "--restore",
            "--ack-config-restore",
            "--run-id %s-p4-config-restore-write" % run_id,
        ),
        "P4-rc-channel-mapping-assistant": command_block(
            "fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_rc_channel_mapping_assistant.py",
            "--uinput",
            "--ack-live-input",
            "--role all",
            "--run-id %s-p4-rc-map" % run_id,
        ),
        "P4-rc-channel-mapping-assistant-aux1-dry": command_block(
            "fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_rc_channel_mapping_assistant.py",
            "--role aux1",
            "--run-id %s-p4-rc-map-aux1-dry" % run_id,
        ),
        "P4-rc-channel-mapping-assistant-aux1-live": command_block(
            "fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_rc_channel_mapping_assistant.py",
            "--uinput",
            "--ack-live-input",
            "--role aux1",
            "--run-id %s-p4-rc-map-aux1-live" % run_id,
        ),
        "P4-rc-channels-visual": command_block(
            "fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_runtime_input_visual_probe.py",
            "--uinput",
            "--ack-live-input",
            "--axis yaw",
            "--magnitude 0.6",
            "--run-id %s-p4-rc-visual" % run_id,
        ),
        "P4-input-live-smoke": command_block(
            "fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_virtual_input_probe.py",
            "--uinput-smoke",
            "--ack-live-input",
            "--yaw 0.03",
            "--hold-seconds 0.1",
            "--run-id %s-p4-live" % run_id,
        ),
        "P5-yaw-live": command_block(
            "fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_response_probe.py",
            "--uinput",
            "--ack-live-input",
            "--axis yaw",
            "--magnitude 0.03",
            "--image-axis x",
            "--expected-sign %d" % yaw_expected_sign,
            "--run-id %s-p5-yaw-live" % run_id,
        ),
        "P5-rc-baseline": command_block(
            "fpv_env/bin/python experiments/simitl_pr0p_probe/msp_ws_rc_probe.py",
            "--run-id %s-p5-rc-baseline" % run_id,
        ),
        "P5-uinput-rc-effect-throttle-low": command_block(
            "fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_live_session_runner.py",
            "--launch-pr0p",
            "--ack-live-launch",
            "--send-ui",
            "--ack-live-ui",
            "--hold-uinput",
            "--ack-live-input",
            "--run-rc-effect",
            "--rc-effect-axis throttle",
            "--rc-effect-magnitude 1.0",
            "--rc-effect-expected-channel throttle",
            "--rc-effect-expected-direction lower",
            "--run-id %s-p5-uinput-rc-throttle-low" % run_id,
        ),
        "P5-uinput-status-effect-throttle-clear": command_block(
            "fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_live_session_runner.py",
            "--launch-pr0p",
            "--ack-live-launch",
            "--send-ui",
            "--ack-live-ui",
            "--hold-uinput",
            "--ack-live-input",
            "--run-status-effect",
            "--status-effect-axis throttle",
            "--status-effect-magnitude 1.0",
            "--status-effect-blocker THROTTLE",
            "--run-id %s-p5-uinput-status-throttle-clear" % run_id,
        ),
        "P5-uinput-arm-button-effect": command_block(
            "fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_live_session_runner.py",
            "--launch-pr0p",
            "--ack-live-launch",
            "--send-ui",
            "--ack-live-ui",
            "--hold-uinput",
            "--ack-live-input",
            "--run-arm-button-effect",
            "--arm-button south",
            "--arm-button east",
            "--arm-throttle-magnitude 1.0",
            "--startup-wait 8",
            "--run-id %s-p5-uinput-arm-button" % run_id,
        ),
        "P5-uinput-rc-effect-aux1-high": command_block(
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
            "--run-id %s-p5-uinput-rc-aux1-high" % run_id,
        ),
        "P5-uinput-aux1-arm-status": command_block(
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
            "--startup-wait 8",
            "--run-id %s-p5-uinput-aux1-arm-status" % run_id,
        ),
        "P5-rc-loopback": command_block(
            "fpv_env/bin/python experiments/simitl_pr0p_probe/msp_ws_rc_probe.py",
            "--write",
            "--ack-live-msp-write",
            "--run-id %s-p5-rc-loopback" % run_id,
        ),
        "P5-persistent-uinput-live-response": command_block(
            "fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_live_session_runner.py",
            "--launch-pr0p",
            "--ack-live-launch",
            "--send-ui",
            "--ack-live-ui",
            "--hold-uinput",
            "--ack-live-input",
            "--run-live-response",
            "--response-axis throttle",
            "--response-magnitude 0.35",
            "--response-image-axis y",
            "--response-expected-sign 0",
            "--run-id %s-p5-persistent-uinput" % run_id,
        ),
        "P5-response-acceptance": command_block(
            "fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_response_acceptance_runner.py",
            "--run-live-gates",
            "--ack-live-launch",
            "--ack-live-ui",
            "--ack-live-input",
            "--yaw-expected-sign %d" % yaw_expected_sign,
            "--pitch-expected-sign %d" % pitch_expected_sign,
            "--run-id %s-p5-response-acceptance" % run_id,
        ),
        "P5-response-dry-run": command_block(
            "fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_response_probe.py",
            "--window-title %s" % shell_quote(window_title),
            "--axis yaw",
            "--magnitude 0.05",
            "--image-axis x",
            "--expected-sign %d" % yaw_expected_sign,
            "--run-id %s-p5-dry" % run_id,
        ),
        "P6-synthetic-e2e-dry-run": command_block(
            "fpv_env/bin/python experiments/game_screen_sandbox/phase_runner.py",
            "--duration 2.0",
            "--hz 10",
            "--min-fps 8",
            "--min-tracker-found-ratio 0.8",
            "--min-loop-found-ratio 0.8",
            "--log-dir logs/simitl_pr0p",
            "--run-id %s-p6-synthetic-e2e" % run_id,
        ),
        "P5-pitch-live": command_block(
            "fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_response_probe.py",
            "--uinput",
            "--ack-live-input",
            "--axis pitch",
            "--magnitude 0.03",
            "--image-axis y",
            "--expected-sign %d" % pitch_expected_sign,
            "--run-id %s-p5-pitch-live" % run_id,
        ),
        "P6-bbox-sample": command_block(
            "fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_bbox_tool.py",
            "--window-title %s" % shell_quote(window_title),
            "--run-id %s-p6-bbox" % run_id,
        ),
        "P6-bbox-overlay": command_block(
            "fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_bbox_tool.py",
            "--window-title %s" % shell_quote(window_title),
            "--bbox %s" % bbox_text,
            "--run-id %s-p6-bbox" % run_id,
        ),
        "P6-tracking-dry": command_block(
            "fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_tracking_probe.py",
            "--window-title %s" % shell_quote(window_title),
            "--bbox %s" % bbox_text,
            "--duration 5",
            "--hz 10",
            "--tracker CSRT",
            "--run-id %s-p6-dry" % run_id,
        ),
        "P6-tracking-pid-dry-run": command_block(
            "fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_tracking_probe.py",
            "--window-title %s" % shell_quote(window_title),
            "--bbox %s" % bbox_text,
            "--duration 5",
            "--hz 10",
            "--tracker CSRT",
            "--run-id %s-p6-dry" % run_id,
        ),
        "P6-tracking-log-check": command_block(
            "fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_tracking_log_check.py",
            "--run-id %s-p6-log" % run_id,
        ),
        "P6-synthetic-log-check": command_block(
            "fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_tracking_log_check.py",
            "--tracking-log path/to/synthetic-s4-loop.jsonl",
            "--run-id %s-p6-synthetic-log" % run_id,
        ),
        "P6-tracking-live": command_block(
            "fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_tracking_probe.py",
            "--window-title %s" % shell_quote(window_title),
            "--bbox %s" % bbox_text,
            "--uinput",
            "--ack-live-input",
            "--duration 5",
            "--hz 10",
            "--run-id %s-p6-live" % run_id,
        ),
        "P6-tracking-acceptance": command_block(
            "fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_tracking_acceptance_runner.py",
            "--bbox %s" % bbox_text,
            "--run-live-gates",
            "--ack-live-input",
            "--run-id %s-p6-tracking-acceptance" % run_id,
        ),
        "safe-suite": command_block(
            "fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_probe_suite.py",
            "--window-title %s" % shell_quote(window_title),
            "--tracking-bbox %s" % bbox_text if bbox else "# add --tracking-bbox x,y,w,h after bbox selection",
            "--run-id %s-suite" % run_id,
        ),
    }


def shell_quote(value: str) -> str:
    if value.replace("_", "").replace("-", "").replace(".", "").isalnum():
        return value
    return "'" + value.replace("'", "'\"'\"'") + "'"


def step_status(
    step_map: dict[str, dict[str, Any]],
    phase: str,
    *,
    fallback_summary: str,
) -> tuple[str, str, str | None, list[str]]:
    step = step_map.get(phase)
    if not step:
        return UNKNOWN, fallback_summary, None, ["no suite evidence yet"]
    return (
        str(step.get("status", UNKNOWN)),
        str(step.get("summary", fallback_summary)),
        step.get("report_md") or step.get("report_json"),
        list(step.get("notes") or []),
    )


def report_status(
    report: dict[str, Any] | None,
    path: Path | None,
    *,
    fallback_status: str,
    fallback_summary: str,
    fallback_notes: list[str],
) -> tuple[str, str, str | None, list[str]]:
    if not report:
        return fallback_status, fallback_summary, None, fallback_notes
    return (
        str(report.get("status", fallback_status)),
        str(report.get("summary", fallback_summary)),
        str(path) if path else None,
        list(report.get("notes") or []),
    )


def stale_report_status(
    report: dict[str, Any] | None,
    path: Path | None,
    *,
    fallback_status: str,
    fallback_summary: str,
    fallback_notes: list[str],
    max_age_s: float,
    stale_note: str,
) -> tuple[str, str, str | None, list[str]]:
    if not report or path is None:
        return fallback_status, fallback_summary, None, fallback_notes
    try:
        age_s = max(0.0, time.time() - path.stat().st_mtime)
    except OSError:
        age_s = None
    if age_s is not None and age_s > max_age_s:
        return (
            WAITING,
            "latest acceptance evidence is stale",
            str(path),
            list(report.get("notes") or []) + [
                stale_note,
                "age_s=%.3f" % age_s,
                "max_age_s=%.3f" % max_age_s,
            ],
        )
    return report_status(
        report,
        path,
        fallback_status=fallback_status,
        fallback_summary=fallback_summary,
        fallback_notes=fallback_notes,
    )


def step_by_phase(steps: list[ManifestStep], phase: str) -> ManifestStep | None:
    for step in steps:
        if step.phase == phase:
            return step
    return None


def has_fc_arm_blocker(step: ManifestStep | None) -> bool:
    if step is None or step.status != PASS:
        return False
    return any(
        note == "FC_NOT_ARMED" or note.startswith("ARMING_DISABLED:")
        for note in step.notes
    )


def choose_next_action(steps: list[ManifestStep], *, bbox: tuple[int, int, int, int] | None) -> str:
    status_by_phase = {step.phase: step.status for step in steps}
    if status_by_phase.get("P1-install-discovery") == FAIL:
        return "Fix pr0p download discovery or install root isolation before live testing."
    if status_by_phase.get("P1-install-discovery") in (UNKNOWN, WAITING):
        return "Run P1-download or install discovery until the updater is present in the isolated root."
    if status_by_phase.get("P1-client-executable") == FAIL:
        return "Fix the isolated pr0p client executable probe before live testing."
    if status_by_phase.get("P1-client-executable") != PASS:
        return "Run P1-updater-runner-dry, then launch the updater with explicit ack or manually, and rerun P1-client-executable."
    if status_by_phase.get("P0-isolation") == FAIL:
        return "Remove Gazebo/SITL coupling from the isolated pr0p/game-screen code before live testing."
    if status_by_phase.get("P2-websocket") != PASS:
        if status_by_phase.get("P4-input-mapping") in (WAITING, FAIL):
            return "Run the P4 input config patch dry-run or configure Controls -> RC Channels, then start pr0p/local race and rerun the safe suite."
        return "Start pr0p, enter a local race, then rerun the safe suite."
    if status_by_phase.get("P2-msp-readonly") != PASS:
        return "Keep pr0p in a local race and rerun P2-msp-readonly until MSP_API_VERSION responds."
    if status_by_phase.get("P2-fc-status-readonly") != PASS:
        return "Keep pr0p in a local race and rerun P2-fc-status-readonly until FC arm/mode status is readable."
    if status_by_phase.get("P2-mode-ranges-readonly") != PASS:
        return "Keep pr0p in a local race and run P2-mode-ranges-readonly to learn which AUX channel/range ARM expects."
    if status_by_phase.get("P3-capture") != PASS:
        return "Fix window title/region capture until P3 captures the FPV view."
    if status_by_phase.get("P4-input-readiness") != PASS:
        return "Fix uinput/virtual-controller readiness before live command tests."
    if status_by_phase.get("P4-input-mapping") != PASS:
        return "Run the P4 input config patch dry-run, then either apply the backup-backed patch or bind Controls -> RC Channels manually."
    if status_by_phase.get("P4-rc-channels-visual") != PASS:
        return "Open pr0p Controls -> RC Channels and run P4-rc-channels-visual until runtime input movement is visible."
    if bbox is None:
        return "Run P6-bbox-sample, choose x,y,w,h, then validate with P6-bbox-overlay."
    if status_by_phase.get("P6-tracking-pid-dry-run") != PASS:
        return "Run P6 tracking dry-run with the selected bbox until tracker/PID is bounded."
    if status_by_phase.get("P5-uinput-rc-effect-throttle-low") != PASS:
        return "Run P5-uinput-rc-effect-throttle-low to prove uinput can drive MSP_RC throttle below arm threshold before P5 live response."
    if status_by_phase.get("P5-uinput-status-effect-throttle-clear") != PASS:
        return "Run P5-uinput-status-effect-throttle-clear to prove throttle-low clears the THROTTLE arming blocker before chasing ARM AUX/button routing."
    if status_by_phase.get("P5-uinput-rc-effect-aux1-high") != PASS:
        arm_button_step = step_by_phase(steps, "P5-uinput-arm-button-effect")
        if arm_button_step and "NO_UINPUT_ARM_BUTTON_EFFECT" in arm_button_step.notes:
            return "Buttons did not activate ARM; bind AUX1/CH5 with P4-rc-channel-mapping-assistant-aux1-live or patch input config, then prove P5-uinput-rc-effect-aux1-high."
        return "ARM expects AUX1/CH5 high; run P4-input-config-patch-aux1-dry or P4-rc-channel-mapping-assistant-aux1-dry, then bind/apply and prove P5-uinput-rc-effect-aux1-high."
    if status_by_phase.get("P5-uinput-aux1-arm-status") != PASS:
        return "AUX1/CH5 moves; run P5-uinput-aux1-arm-status to prove throttle-low plus AUX1-high activates FC_ARMED/ARM mode."
    if status_by_phase.get("P5-response-acceptance") != PASS:
        return "Run P5-response-acceptance to prove signed yaw and pitch response before any live P6 tracking control."
    if has_fc_arm_blocker(step_by_phase(steps, "P2-fc-status-readonly")):
        return "AUX1 ARM status is proven despite baseline FC arm blockers; run the live P5 persistent-uinput response gate before any live P6 tracking control."
    if status_by_phase.get("P6-tracking-acceptance") != PASS:
        return "Run P6-tracking-acceptance after response acceptance to prove yaw-only live tracker/PID control."
    return "Run the live P5 persistent-uinput response gate before any live P6 tracking control."


def build_manifest(
    *,
    run_id: str,
    log_dir: Path,
    suite_report: Path | None,
    bbox: tuple[int, int, int, int] | None,
    window_title: str,
    yaw_expected_sign: int,
    pitch_expected_sign: int,
    max_acceptance_evidence_age_s: float = DEFAULT_MAX_ACCEPTANCE_EVIDENCE_AGE_S,
) -> LiveManifest:
    suite_path = suite_report or latest_suite_report(log_dir)
    suite = load_suite_report(suite_path)
    step_map = suite_step_map(suite)
    runtime_visual_path = latest_runtime_input_visual_report(log_dir)
    runtime_visual_report = load_json_report(runtime_visual_path)
    fc_status_path = latest_fc_status_report(log_dir)
    fc_status_report = load_json_report(fc_status_path)
    mode_ranges_path = latest_mode_ranges_report(log_dir)
    mode_ranges_report = load_json_report(mode_ranges_path)
    uinput_rc_effect_path = latest_uinput_rc_effect_report(log_dir, expected_channel="throttle")
    uinput_rc_effect_report = load_json_report(uinput_rc_effect_path)
    if uinput_rc_effect_report is None:
        uinput_rc_effect_path = latest_live_session_with_metric(
            log_dir,
            "live_rc_effect",
            matcher=lambda report: rc_effect_matches(report, "throttle"),
        )
        uinput_rc_effect_report = load_live_session_metric(uinput_rc_effect_path, "live_rc_effect")
    uinput_aux_effect_path = latest_uinput_rc_effect_report(log_dir, expected_channel="aux1")
    uinput_aux_effect_report = load_json_report(uinput_aux_effect_path)
    if uinput_aux_effect_report is None:
        uinput_aux_effect_path = latest_live_session_with_metric(
            log_dir,
            "live_rc_effect",
            matcher=lambda report: rc_effect_matches(report, "aux1"),
        )
        uinput_aux_effect_report = load_live_session_metric(uinput_aux_effect_path, "live_rc_effect")
    uinput_status_path = latest_uinput_status_report(log_dir)
    uinput_status_report = load_json_report(uinput_status_path)
    if uinput_status_report is None:
        uinput_status_path = latest_live_session_with_metric(log_dir, "live_status_effect")
        uinput_status_report = load_live_session_metric(uinput_status_path, "live_status_effect")
    uinput_arm_path = latest_uinput_arm_report(log_dir)
    uinput_arm_report = load_json_report(uinput_arm_path)
    if uinput_arm_report is None:
        uinput_arm_path = latest_live_session_with_metric(log_dir, "live_arm_effect")
        uinput_arm_report = load_live_session_metric(uinput_arm_path, "live_arm_effect")
    uinput_aux_arm_status_path = latest_uinput_aux_arm_status_report(log_dir)
    uinput_aux_arm_status_report = load_json_report(uinput_aux_arm_status_path)
    if uinput_aux_arm_status_report is None:
        uinput_aux_arm_status_path = latest_live_session_with_metric(log_dir, "live_aux_arm_status")
        uinput_aux_arm_status_report = load_live_session_metric(
            uinput_aux_arm_status_path,
            "live_aux_arm_status",
        )
    response_acceptance_path = latest_response_acceptance_report(log_dir)
    response_acceptance_report = load_json_report(response_acceptance_path)
    tracking_acceptance_path = latest_tracking_acceptance_report(log_dir)
    tracking_acceptance_report = load_json_report(tracking_acceptance_path)
    commands = build_commands(
        run_id=run_id,
        window_title=window_title,
        bbox=bbox,
        yaw_expected_sign=yaw_expected_sign,
        pitch_expected_sign=pitch_expected_sign,
    )

    manifest_steps: list[ManifestStep] = []
    for phase, fallback in (
        ("P0-isolation", "verify isolated code has no forbidden Gazebo runtime coupling"),
        ("P0-preflight", "run preflight or safe suite"),
        ("P1-install-discovery", "discover/download pr0p Linux updater into isolated root"),
        ("P1-client-executable", "verify a pr0p client executable is installed in the isolated root"),
        ("P2-websocket", "start pr0p local race and check websocket"),
        ("P2-msp-readonly", "prove read-only MSP semantics over websocket"),
        ("P3-capture", "capture pr0p FPV window"),
        ("P4-input-readiness", "check uinput readiness without sending input"),
        ("P4-input-mapping", "check pr0p control bindings for virtual input"),
        ("P4-input-config-patch-dry", "dry-run backup-safe pr0p input config patch"),
        ("P5-response-dry-run", "dry-run response gate is informational"),
        ("P6-synthetic-e2e-dry-run", "run synthetic capture/tracker/PID dry-run gates"),
        ("P6-synthetic-log-check", "verify the synthetic E2E S4 JSONL log"),
        ("P6-tracking-pid-dry-run", "select bbox and run tracking dry-run"),
        ("P6-tracking-log-check", "verify the P6 tracking JSONL log"),
    ):
        status, summary, evidence, notes = step_status(
            step_map,
            phase,
            fallback_summary=fallback,
        )
        manifest_steps.append(ManifestStep(
            phase=phase,
            status=status,
            summary=summary,
            command=commands.get(phase, commands.get("safe-suite", "")),
            evidence=evidence,
            notes=notes,
        ))

    rc_baseline_status, rc_baseline_summary, rc_baseline_evidence, rc_baseline_notes = step_status(
        step_map,
        "P5-rc-baseline",
        fallback_summary="read-only MSP_RC stability check before command tests",
    )
    suite_uinput_effect_status, suite_uinput_effect_summary, suite_uinput_effect_evidence, suite_uinput_effect_notes = step_status(
        step_map,
        "P5-uinput-rc-effect-throttle-low",
        fallback_summary="prove uinput throttle pulse changes MSP_RC throttle low",
    )
    uinput_effect_status, uinput_effect_summary, uinput_effect_evidence, uinput_effect_notes = report_status(
        uinput_rc_effect_report,
        uinput_rc_effect_path,
        fallback_status=suite_uinput_effect_status,
        fallback_summary=suite_uinput_effect_summary,
        fallback_notes=suite_uinput_effect_notes if suite_uinput_effect_notes else [
            "start pr0p/local race first",
            "sends real OS input only when executed with explicit ack",
            "read-only MSP; no MSP RC write is sent",
        ],
    )
    if uinput_rc_effect_report is None and suite_uinput_effect_evidence:
        uinput_effect_evidence = suite_uinput_effect_evidence
    suite_uinput_aux_status, suite_uinput_aux_summary, suite_uinput_aux_evidence, suite_uinput_aux_notes = step_status(
        step_map,
        "P5-uinput-rc-effect-aux1-high",
        fallback_summary="prove uinput AUX1 pulse changes MSP_RC CH5 high",
    )
    uinput_aux_status, uinput_aux_summary, uinput_aux_evidence, uinput_aux_notes = report_status(
        uinput_aux_effect_report,
        uinput_aux_effect_path,
        fallback_status=suite_uinput_aux_status,
        fallback_summary=suite_uinput_aux_summary,
        fallback_notes=suite_uinput_aux_notes if suite_uinput_aux_notes else [
            "requires pr0p input slot 4/AUX1 to be bound to the virtual AUX1 axis",
            "sends real OS input only when executed with explicit ack",
            "read-only MSP_RC; no MSP RC write is sent",
        ],
    )
    if uinput_aux_effect_report is None and suite_uinput_aux_evidence:
        uinput_aux_evidence = suite_uinput_aux_evidence
    suite_uinput_status_status, suite_uinput_status_summary, suite_uinput_status_evidence, suite_uinput_status_notes = step_status(
        step_map,
        "P5-uinput-status-effect-throttle-clear",
        fallback_summary="prove throttle-low uinput clears THROTTLE arming blocker",
    )
    uinput_status_status, uinput_status_summary, uinput_status_evidence, uinput_status_notes = report_status(
        uinput_status_report,
        uinput_status_path,
        fallback_status=suite_uinput_status_status,
        fallback_summary=suite_uinput_status_summary,
        fallback_notes=suite_uinput_status_notes if suite_uinput_status_notes else [
            "start pr0p/local race first",
            "sends real OS input only when executed with explicit ack",
            "read-only MSP status; no MSP RC write is sent",
        ],
    )
    if uinput_status_report is None and suite_uinput_status_evidence:
        uinput_status_evidence = suite_uinput_status_evidence
    suite_uinput_arm_status, suite_uinput_arm_summary, suite_uinput_arm_evidence, suite_uinput_arm_notes = step_status(
        step_map,
        "P5-uinput-arm-button-effect",
        fallback_summary="discover whether virtual controller buttons activate ARM mode/status",
    )
    uinput_arm_status, uinput_arm_summary, uinput_arm_evidence, uinput_arm_notes = report_status(
        uinput_arm_report,
        uinput_arm_path,
        fallback_status=suite_uinput_arm_status,
        fallback_summary=suite_uinput_arm_summary,
        fallback_notes=suite_uinput_arm_notes if suite_uinput_arm_notes else [
            "start pr0p/local race first",
            "keeps throttle low while testing south/east buttons",
            "read-only MSP status; no MSP RC write is sent",
        ],
    )
    if uinput_arm_report is None and suite_uinput_arm_evidence:
        uinput_arm_evidence = suite_uinput_arm_evidence
    suite_uinput_aux_arm_status, suite_uinput_aux_arm_summary, suite_uinput_aux_arm_evidence, suite_uinput_aux_arm_notes = step_status(
        step_map,
        "P5-uinput-aux1-arm-status",
        fallback_summary="prove throttle-low plus AUX1-high activates FC ARM mode/status",
    )
    uinput_aux_arm_status, uinput_aux_arm_summary, uinput_aux_arm_evidence, uinput_aux_arm_notes = report_status(
        uinput_aux_arm_status_report,
        uinput_aux_arm_status_path,
        fallback_status=suite_uinput_aux_arm_status,
        fallback_summary=suite_uinput_aux_arm_summary,
        fallback_notes=suite_uinput_aux_arm_notes if suite_uinput_aux_arm_notes else [
            "requires AUX1/CH5 to move high while throttle stays low",
            "sends real OS input only when executed with explicit ack",
            "read-only MSP status; no MSP RC write is sent",
        ],
    )
    if uinput_aux_arm_status_report is None and suite_uinput_aux_arm_evidence:
        uinput_aux_arm_evidence = suite_uinput_aux_arm_evidence
    suite_fc_status, suite_fc_summary, suite_fc_evidence, suite_fc_notes = step_status(
        step_map,
        "P2-fc-status-readonly",
        fallback_summary="read FC arm/mode status over MSP without sending commands",
    )
    fc_status, fc_summary, fc_evidence, fc_notes = report_status(
        fc_status_report,
        fc_status_path,
        fallback_status=suite_fc_status,
        fallback_summary=suite_fc_summary,
        fallback_notes=suite_fc_notes if suite_fc_notes else [
            "start pr0p/local race first",
            "use --samples 6 --interval 1 to separate transient arming flags",
            "read-only; sends no arm, throttle, or RC writes",
        ],
    )
    if fc_status_report is None and suite_fc_evidence:
        fc_evidence = suite_fc_evidence
    suite_mode_status, suite_mode_summary, suite_mode_evidence, suite_mode_notes = step_status(
        step_map,
        "P2-mode-ranges-readonly",
        fallback_summary="read Betaflight mode ranges to identify ARM AUX channel",
    )
    mode_status, mode_summary, mode_evidence, mode_notes = report_status(
        mode_ranges_report,
        mode_ranges_path,
        fallback_status=suite_mode_status,
        fallback_summary=suite_mode_summary,
        fallback_notes=suite_mode_notes if suite_mode_notes else [
            "start pr0p/local race first",
            "read-only; sends no arm, throttle, or mode writes",
        ],
    )
    if mode_ranges_report is None and suite_mode_evidence:
        mode_evidence = suite_mode_evidence
    suite_rc_visual_status, suite_rc_visual_summary, suite_rc_visual_evidence, suite_rc_visual_notes = step_status(
        step_map,
        "P4-rc-channels-visual",
        fallback_summary="visual gate for Controls -> RC Channels bar movement while pulsing virtual RC",
    )
    rc_visual_status, rc_visual_summary, rc_visual_evidence, rc_visual_notes = report_status(
        runtime_visual_report,
        runtime_visual_path,
        fallback_status=suite_rc_visual_status,
        fallback_summary=suite_rc_visual_summary,
        fallback_notes=suite_rc_visual_notes if suite_rc_visual_evidence else [
            "open pr0p Controls -> RC Channels first",
            "use --crop around the RC bars if other UI elements animate",
            "sends real OS input only when executed with explicit ack",
        ],
    )
    if runtime_visual_report is None and suite_rc_visual_evidence:
        rc_visual_evidence = suite_rc_visual_evidence
    p5_yaw_status, p5_yaw_summary, p5_yaw_evidence, p5_yaw_notes = step_status(
        step_map,
        "P5-yaw-live",
        fallback_summary="manual live yaw direction gate",
    )
    p5_pitch_status, p5_pitch_summary, p5_pitch_evidence, p5_pitch_notes = step_status(
        step_map,
        "P5-pitch-live",
        fallback_summary="manual live pitch direction gate",
    )
    p5_rc_loopback_status, p5_rc_loopback_summary, p5_rc_loopback_evidence, p5_rc_loopback_notes = step_status(
        step_map,
        "P5-rc-loopback",
        fallback_summary="optional low-throttle arm-low MSP_SET_RAW_RC loopback",
    )
    p5_persistent_status, p5_persistent_summary, p5_persistent_evidence, p5_persistent_notes = step_status(
        step_map,
        "P5-persistent-uinput-live-response",
        fallback_summary="launch pr0p with virtual RC already present and measure live visual response",
    )
    suite_p5_response_acceptance_status, suite_p5_response_acceptance_summary, suite_p5_response_acceptance_evidence, suite_p5_response_acceptance_notes = step_status(
        step_map,
        "P5-response-acceptance",
        fallback_summary="run signed yaw/pitch response acceptance after AUX1 ARM status passes",
    )
    p5_response_acceptance_status, p5_response_acceptance_summary, p5_response_acceptance_evidence, p5_response_acceptance_notes = stale_report_status(
        response_acceptance_report,
        response_acceptance_path,
        fallback_status=suite_p5_response_acceptance_status,
        fallback_summary=suite_p5_response_acceptance_summary,
        fallback_notes=suite_p5_response_acceptance_notes if suite_p5_response_acceptance_notes else [
            "requires P5-uinput-aux1-arm-status PASS first",
            "uses persistent uinput live response gates",
        ],
        max_age_s=max_acceptance_evidence_age_s,
        stale_note="STALE_RESPONSE_ACCEPTANCE",
    )
    if response_acceptance_report is None and suite_p5_response_acceptance_evidence:
        p5_response_acceptance_evidence = suite_p5_response_acceptance_evidence
    p6_live_status, p6_live_summary, p6_live_evidence, p6_live_notes = step_status(
        step_map,
        "P6-tracking-live",
        fallback_summary="manual live yaw-only tracker/PID control",
    )
    suite_p6_tracking_acceptance_status, suite_p6_tracking_acceptance_summary, suite_p6_tracking_acceptance_evidence, suite_p6_tracking_acceptance_notes = step_status(
        step_map,
        "P6-tracking-acceptance",
        fallback_summary="run gated yaw-only live tracker/PID acceptance",
    )
    p6_tracking_acceptance_status, p6_tracking_acceptance_summary, p6_tracking_acceptance_evidence, p6_tracking_acceptance_notes = stale_report_status(
        tracking_acceptance_report,
        tracking_acceptance_path,
        fallback_status=suite_p6_tracking_acceptance_status,
        fallback_summary=suite_p6_tracking_acceptance_summary,
        fallback_notes=suite_p6_tracking_acceptance_notes if suite_p6_tracking_acceptance_notes else [
            "requires bbox, P6 dry-run, and P5 response acceptance PASS",
            "sends real OS input only when executed with explicit ack",
        ],
        max_age_s=max_acceptance_evidence_age_s,
        stale_note="STALE_TRACKING_ACCEPTANCE",
    )
    if tracking_acceptance_report is None and suite_p6_tracking_acceptance_evidence:
        p6_tracking_acceptance_evidence = suite_p6_tracking_acceptance_evidence
    p2_insert_index = next(
        (
            index + 1
            for index, step in enumerate(manifest_steps)
            if step.phase == "P2-msp-readonly"
        ),
        len(manifest_steps),
    )
    manifest_steps.insert(p2_insert_index, ManifestStep(
        phase="P2-fc-status-readonly",
        status=fc_status,
        summary=fc_summary,
        command=commands["P2-fc-status-readonly"],
        evidence=fc_evidence,
        notes=fc_notes,
    ))
    manifest_steps.insert(p2_insert_index + 1, ManifestStep(
        phase="P2-mode-ranges-readonly",
        status=mode_status,
        summary=mode_summary,
        command=commands["P2-mode-ranges-readonly"],
        evidence=mode_evidence,
        notes=mode_notes,
    ))

    manifest_steps.extend([
        ManifestStep(
            phase="P1-download",
            status=UNKNOWN,
            summary="download Linux updater into isolated install root",
            command=commands["P1-download"],
            notes=["downloads only; does not execute the updater"],
        ),
        ManifestStep(
            phase="P1-updater-runner-dry",
            status=UNKNOWN,
            summary="plan the updater launch without executing the downloaded binary",
            command=commands["P1-updater-runner-dry"],
            notes=["does not execute the updater"],
        ),
        ManifestStep(
            phase="P1-updater-runner-launch",
            status=UNKNOWN,
            summary="launch the updater from the isolated root with explicit acknowledgements",
            command=commands["P1-updater-runner-launch"],
            notes=[
                "executes the downloaded updater only with explicit ack flags",
                "use when an interactive updater window should be left running",
            ],
        ),
        ManifestStep(
            phase="P1-local-race-ui-smoke",
            status=UNKNOWN,
            summary="optional UI automation to enter Local -> Time attack",
            command=commands["P1-local-race-ui-smoke"],
            notes=["sends real desktop UI clicks only when executed"],
        ),
        ManifestStep(
            phase="P1-live-session-dry",
            status=UNKNOWN,
            summary="dry-run the bounded pr0p launch/cleanup session plan",
            command=commands["P1-live-session-dry"],
            notes=["does not launch pr0p or send UI input"],
        ),
        ManifestStep(
            phase="P1-live-session-runner",
            status=UNKNOWN,
            summary="launch pr0p, optionally enter local race, run safe suite, then clean up",
            command=commands["P1-live-session-runner"],
            notes=[
                "real pr0p launch and real UI clicks require explicit ack flags",
                "checks Controls -> RC Channels mapping before treating live control as ready",
            ],
        ),
        ManifestStep(
            phase="P4-input-config-patch-write",
            status=UNKNOWN,
            summary="apply the pr0p input.json virtual mapping patch with a backup",
            command=commands["P4-input-config-patch-write"],
            notes=["writes pr0p input.json only when executed with explicit ack"],
        ),
        ManifestStep(
            phase="P4-input-config-patch-aux1-dry",
            status=UNKNOWN,
            summary="dry-run backup-safe pr0p input config patch for AUX1/CH5 ARM",
            command=commands["P4-input-config-patch-aux1-dry"],
            notes=["does not edit pr0p input.json"],
        ),
        ManifestStep(
            phase="P4-input-config-patch-aux1-write",
            status=UNKNOWN,
            summary="apply the AUX1/CH5 input config patch with a backup",
            command=commands["P4-input-config-patch-aux1-write"],
            notes=["writes pr0p input.json only when executed with explicit ack"],
        ),
        ManifestStep(
            phase="P4-input-config-restore-latest",
            status=UNKNOWN,
            summary="dry-run restore from the latest Codex input.json backup",
            command=commands["P4-input-config-restore-latest"],
            notes=["does not edit pr0p input.json"],
        ),
        ManifestStep(
            phase="P4-input-config-restore-write",
            status=UNKNOWN,
            summary="restore pr0p input.json from the latest Codex backup",
            command=commands["P4-input-config-restore-write"],
            notes=["writes pr0p input.json only when executed with explicit ack"],
        ),
        ManifestStep(
            phase="P4-rc-channel-mapping-assistant",
            status=UNKNOWN,
            summary="manual helper that pulses the Kenet virtual RC axes while binding pr0p channels",
            command=commands["P4-rc-channel-mapping-assistant"],
            notes=["sends real OS input only when executed with explicit ack"],
        ),
        ManifestStep(
            phase="P4-rc-channel-mapping-assistant-aux1-dry",
            status=UNKNOWN,
            summary="dry-run manual helper plan for binding AUX1/CH5 ARM",
            command=commands["P4-rc-channel-mapping-assistant-aux1-dry"],
            notes=["does not send OS input"],
        ),
        ManifestStep(
            phase="P4-rc-channel-mapping-assistant-aux1-live",
            status=UNKNOWN,
            summary="manual helper that pulses only AUX1/CH5 while binding ARM",
            command=commands["P4-rc-channel-mapping-assistant-aux1-live"],
            notes=["sends real OS input only when executed with explicit ack"],
        ),
        ManifestStep(
            phase="P4-rc-channels-visual",
            status=rc_visual_status,
            summary=rc_visual_summary,
            command=commands["P4-rc-channels-visual"],
            evidence=rc_visual_evidence,
            notes=rc_visual_notes,
        ),
        ManifestStep(
            phase="P4-input-live-smoke",
            status=UNKNOWN,
            summary="manual live input smoke; run only after neutral/stop plan",
            command=commands["P4-input-live-smoke"],
            notes=["sends real OS input only when executed"],
        ),
        ManifestStep(
            phase="P5-yaw-live",
            status=p5_yaw_status,
            summary=p5_yaw_summary,
            command=commands["P5-yaw-live"],
            evidence=p5_yaw_evidence,
            notes=p5_yaw_notes or ["requires P3 PASS and P4 PASS"],
        ),
        ManifestStep(
            phase="P5-rc-baseline",
            status=rc_baseline_status,
            summary=rc_baseline_summary,
            command=commands["P5-rc-baseline"],
            evidence=rc_baseline_evidence,
            notes=rc_baseline_notes or ["detects physical/controller input contention before MSP writes"],
        ),
        ManifestStep(
            phase="P5-uinput-rc-effect-throttle-low",
            status=uinput_effect_status,
            summary=uinput_effect_summary,
            command=commands["P5-uinput-rc-effect-throttle-low"],
            evidence=uinput_effect_evidence,
            notes=uinput_effect_notes,
        ),
        ManifestStep(
            phase="P5-uinput-status-effect-throttle-clear",
            status=uinput_status_status,
            summary=uinput_status_summary,
            command=commands["P5-uinput-status-effect-throttle-clear"],
            evidence=uinput_status_evidence,
            notes=uinput_status_notes,
        ),
        ManifestStep(
            phase="P5-uinput-arm-button-effect",
            status=uinput_arm_status,
            summary=uinput_arm_summary,
            command=commands["P5-uinput-arm-button-effect"],
            evidence=uinput_arm_evidence,
            notes=uinput_arm_notes,
        ),
        ManifestStep(
            phase="P5-uinput-rc-effect-aux1-high",
            status=uinput_aux_status,
            summary=uinput_aux_summary,
            command=commands["P5-uinput-rc-effect-aux1-high"],
            evidence=uinput_aux_evidence,
            notes=uinput_aux_notes,
        ),
        ManifestStep(
            phase="P5-uinput-aux1-arm-status",
            status=uinput_aux_arm_status,
            summary=uinput_aux_arm_summary,
            command=commands["P5-uinput-aux1-arm-status"],
            evidence=uinput_aux_arm_evidence,
            notes=uinput_aux_arm_notes,
        ),
        ManifestStep(
            phase="P5-rc-loopback",
            status=p5_rc_loopback_status,
            summary=p5_rc_loopback_summary,
            command=commands["P5-rc-loopback"],
            evidence=p5_rc_loopback_evidence,
            notes=p5_rc_loopback_notes or ["sends real MSP RC write only when executed"],
        ),
        ManifestStep(
            phase="P5-persistent-uinput-live-response",
            status=p5_persistent_status,
            summary=p5_persistent_summary,
            command=commands["P5-persistent-uinput-live-response"],
            evidence=p5_persistent_evidence,
            notes=p5_persistent_notes or [
                "sends real OS input only when executed with explicit ack",
                "stronger than creating a short-lived uinput device after pr0p is already running",
            ],
        ),
        ManifestStep(
            phase="P5-response-acceptance",
            status=p5_response_acceptance_status,
            summary=p5_response_acceptance_summary,
            command=commands["P5-response-acceptance"],
            evidence=p5_response_acceptance_evidence,
            notes=p5_response_acceptance_notes,
        ),
        ManifestStep(
            phase="P5-pitch-live",
            status=p5_pitch_status,
            summary=p5_pitch_summary,
            command=commands["P5-pitch-live"],
            evidence=p5_pitch_evidence,
            notes=p5_pitch_notes or ["run after yaw sign is understood"],
        ),
        ManifestStep(
            phase="P6-bbox-sample",
            status=UNKNOWN if bbox is None else PASS,
            summary="capture sample frame for target selection",
            command=commands["P6-bbox-sample"],
        ),
        ManifestStep(
            phase="P6-bbox-overlay",
            status=UNKNOWN if bbox is None else WAITING,
            summary="validate selected bbox overlay",
            command=commands["P6-bbox-overlay"],
        ),
        ManifestStep(
            phase="P6-tracking-live",
            status=p6_live_status,
            summary=p6_live_summary,
            command=commands["P6-tracking-live"],
            evidence=p6_live_evidence,
            notes=p6_live_notes or ["run only after P5 live direction gates are PASS"],
        ),
        ManifestStep(
            phase="P6-tracking-acceptance",
            status=p6_tracking_acceptance_status,
            summary=p6_tracking_acceptance_summary,
            command=commands["P6-tracking-acceptance"],
            evidence=p6_tracking_acceptance_evidence,
            notes=p6_tracking_acceptance_notes,
        ),
    ])

    core_phase_statuses = {
        step.phase: step.status
        for step in manifest_steps
        if step.phase in (
            "P0-preflight",
            "P1-install-discovery",
            "P1-client-executable",
            "P2-websocket",
            "P2-msp-readonly",
            "P2-fc-status-readonly",
            "P2-mode-ranges-readonly",
            "P3-capture",
            "P4-input-readiness",
            "P4-input-mapping",
        )
    }
    status = FAIL if any(status == FAIL for status in core_phase_statuses.values()) else WAITING
    if all(status == PASS for status in core_phase_statuses.values()) and bbox is not None:
        status = (
            PASS
            if (
                step_map.get("P6-tracking-pid-dry-run", {}).get("status") == PASS
                and p5_response_acceptance_status == PASS
                and p6_tracking_acceptance_status == PASS
            )
            else WAITING
        )
    return LiveManifest(
        run_id=run_id,
        started_at=time.strftime("%Y-%m-%dT%H:%M:%S%z", time.localtime()),
        status=status,
        next_action=choose_next_action(manifest_steps, bbox=bbox),
        suite_report=str(suite_path) if suite_path else None,
        steps=manifest_steps,
        metadata={
            "window_title": window_title,
            "bbox": list(bbox) if bbox else None,
            "yaw_expected_sign": yaw_expected_sign,
            "pitch_expected_sign": pitch_expected_sign,
            "max_acceptance_evidence_age_s": max_acceptance_evidence_age_s,
            "safe_suite_command": commands["safe-suite"],
            "runtime_input_visual_report": str(runtime_visual_path) if runtime_visual_path else None,
            "fc_status_report": str(fc_status_path) if fc_status_path else None,
            "mode_ranges_report": str(mode_ranges_path) if mode_ranges_path else None,
            "uinput_rc_effect_report": str(uinput_rc_effect_path) if uinput_rc_effect_path else None,
            "uinput_aux_effect_report": str(uinput_aux_effect_path) if uinput_aux_effect_path else None,
            "uinput_aux_arm_status_report": (
                str(uinput_aux_arm_status_path) if uinput_aux_arm_status_path else None
            ),
            "response_acceptance_report": (
                str(response_acceptance_path) if response_acceptance_path else None
            ),
            "tracking_acceptance_report": (
                str(tracking_acceptance_path) if tracking_acceptance_path else None
            ),
            "uinput_status_report": str(uinput_status_path) if uinput_status_path else None,
            "uinput_arm_report": str(uinput_arm_path) if uinput_arm_path else None,
        },
    )


def build_markdown(manifest: LiveManifest) -> str:
    lines = [
        "# SimITL / pr0p Live Run Manifest",
        "",
        "Run: `%s`" % manifest.run_id,
        "Started: `%s`" % manifest.started_at,
        "Verdict: `%s`" % manifest.status,
        "Suite evidence: `%s`" % (manifest.suite_report or "none"),
        "",
        "Next action: %s" % manifest.next_action,
        "",
        "## Steps",
        "",
        "| Phase | Status | Summary | Evidence |",
        "| --- | --- | --- | --- |",
    ]
    for step in manifest.steps:
        lines.append("| %s | %s | %s | `%s` |" % (
            step.phase,
            step.status,
            step.summary,
            step.evidence or "-",
        ))
    lines.extend([
        "",
        "## Commands",
        "",
    ])
    for step in manifest.steps:
        lines.extend([
            "### %s" % step.phase,
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
        "## Metadata",
        "",
        "```json",
        json.dumps(manifest.metadata, indent=2, sort_keys=True),
        "```",
        "",
    ])
    return "\n".join(lines)


def write_reports(manifest: LiveManifest, log_dir: Path) -> tuple[Path, Path]:
    log_dir.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime("%Y%m%d-%H%M%S", time.localtime())
    json_path = log_dir / ("%s-%s-live-manifest.json" % (stamp, manifest.run_id))
    md_path = log_dir / ("%s-%s-live-manifest.md" % (stamp, manifest.run_id))
    json_path.write_text(
        json.dumps(manifest.as_dict(), indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    md_path.write_text(build_markdown(manifest), encoding="utf-8")
    return json_path, md_path


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-id", default="pr0p-live")
    parser.add_argument("--log-dir", type=Path, default=DEFAULT_LOG_DIR)
    parser.add_argument("--suite-report", type=Path)
    parser.add_argument("--bbox", type=parse_bbox)
    parser.add_argument("--window-title", default=DEFAULT_WINDOW_TITLES[0])
    parser.add_argument("--yaw-expected-sign", type=int, choices=(-1, 0, 1), default=0)
    parser.add_argument("--pitch-expected-sign", type=int, choices=(-1, 0, 1), default=0)
    parser.add_argument(
        "--max-acceptance-evidence-age-s",
        type=float,
        default=DEFAULT_MAX_ACCEPTANCE_EVIDENCE_AGE_S,
        help="Maximum age for standalone response/tracking acceptance JSON evidence.",
    )
    args = parser.parse_args(argv)
    if args.max_acceptance_evidence_age_s < 0:
        parser.error("--max-acceptance-evidence-age-s must be non-negative")
    return args


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    manifest = build_manifest(
        run_id=args.run_id,
        log_dir=args.log_dir,
        suite_report=args.suite_report,
        bbox=args.bbox,
        window_title=args.window_title,
        yaw_expected_sign=args.yaw_expected_sign,
        pitch_expected_sign=args.pitch_expected_sign,
        max_acceptance_evidence_age_s=args.max_acceptance_evidence_age_s,
    )
    json_path, md_path = write_reports(manifest, args.log_dir)
    print("simitl-pr0p-live-manifest %s report=%s summary=%s" % (
        manifest.status,
        json_path,
        md_path,
    ))
    print(manifest.next_action)
    return 1 if manifest.status == FAIL else 0


if __name__ == "__main__":
    raise SystemExit(main())
