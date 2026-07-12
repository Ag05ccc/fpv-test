#!/usr/bin/env python3
"""Summarize current game/screen sandbox readiness without running gates."""

from __future__ import annotations

import argparse
import json
import os
import shlex
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from decision_report import latest_report, load_json_report, report_sort_key  # noqa: E402
from isolation_audit import process_snapshot  # noqa: E402
from latency_probe import normalize_axes, parse_axis_shift_expectations  # noqa: E402
from phase_runner import load_tracker_bbox_file  # noqa: E402
from virtual_input import probe_uinput_environment  # noqa: E402
from x11_window import find_window_by_title, list_x11_windows  # noqa: E402


PASS = "PASS"
WAITING = "WAITING"
REJECT = "REJECT"
READY = "READY"
EXTERNAL_LIVE_INPUT_READY = "EXTERNAL_LIVE_INPUT_READY"
EXTERNAL_LIVE_FOLLOW_COMPLETE = "EXTERNAL_LIVE_FOLLOW_COMPLETE"
EXTERNAL_FOLLOW_LIVE_RUN_COMPLETE = "EXTERNAL_FOLLOW_LIVE_RUN_COMPLETE"
RC_BINDING_READY = "RC_BINDING_READY"
DEFAULT_LOG_DIR = Path("logs/game_screen_sandbox")
DEFAULT_BBOX_FILE = DEFAULT_LOG_DIR / "pr0p-target-bbox.json"
DEFAULT_EVIDENCE_STALE_AFTER_S = 3600.0


@dataclass(frozen=True)
class StatusItem:
    name: str
    status: str
    summary: str
    metrics: dict[str, Any] = field(default_factory=dict)

    def as_dict(self) -> dict[str, Any]:
        return {
            "name": self.name,
            "status": self.status,
            "summary": self.summary,
            "metrics": self.metrics,
        }


@dataclass
class SandboxStatusReport:
    run_id: str
    started_at: str
    status: str
    summary: str
    items: list[StatusItem] = field(default_factory=list)
    next_actions: list[str] = field(default_factory=list)
    resume_commands: list[dict[str, Any]] = field(default_factory=list)
    evidence_paths: dict[str, str | None] = field(default_factory=dict)
    metrics: dict[str, Any] = field(default_factory=dict)

    def as_dict(self) -> dict[str, Any]:
        return {
            "run_id": self.run_id,
            "started_at": self.started_at,
            "status": self.status,
            "summary": self.summary,
            "items": [item.as_dict() for item in self.items],
            "next_actions": self.next_actions,
            "resume_commands": self.resume_commands,
            "evidence_paths": self.evidence_paths,
            "metrics": self.metrics,
        }


def matching_report(
    log_dir: Path,
    pattern: str,
    *,
    window_title: str | None,
    status_prefix: str | None = None,
    tracker_bbox: list[int] | None = None,
) -> tuple[Path | None, dict[str, Any] | None]:
    candidates: list[tuple[Path, dict[str, Any]]] = []
    for path in log_dir.glob(pattern):
        data = load_json_report(path)
        if not data:
            continue
        if status_prefix and not str(data.get("status", "")).startswith(status_prefix):
            continue
        if window_title:
            metrics = data.get("metrics", {})
            if not isinstance(metrics, dict):
                continue
            if metrics.get("window_title") != window_title:
                continue
        if tracker_bbox is not None and not report_matches_bbox(data, tracker_bbox):
            continue
        candidates.append((path, data))
    if not candidates:
        return None, None
    return max(candidates, key=lambda item: report_sort_key(item[0]))


def normalize_bbox(value: Any) -> list[int] | None:
    if not isinstance(value, (list, tuple)) or len(value) != 4:
        return None
    try:
        bbox = [int(part) for part in value]
    except (TypeError, ValueError):
        return None
    if bbox[2] <= 0 or bbox[3] <= 0:
        return None
    return bbox


def collect_report_bboxes(value: Any) -> list[list[int]]:
    found: list[list[int]] = []
    if isinstance(value, dict):
        for key, child in value.items():
            if key in {"bbox", "initial_bbox", "tracker_bbox"}:
                bbox = normalize_bbox(child)
                if bbox is not None:
                    found.append(bbox)
            found.extend(collect_report_bboxes(child))
    elif isinstance(value, list):
        for child in value:
            found.extend(collect_report_bboxes(child))
    return found


def report_matches_bbox(data: dict[str, Any], tracker_bbox: list[int]) -> bool:
    current = normalize_bbox(tracker_bbox)
    if current is None:
        return False
    return any(candidate == current for candidate in collect_report_bboxes(data))


def shell_command(parts: list[str | Path | float | int]) -> str:
    return " ".join(shlex.quote(str(part)) for part in parts)


def format_axes(axes: tuple[str, ...]) -> str:
    return ",".join(axes)


def format_expected_shifts(expected_shifts: dict[str, tuple[str, int]] | None) -> str:
    if not expected_shifts:
        return ""
    return ",".join(
        "%s:%s%s" % (axis, "+" if sign > 0 else "-", component)
        for axis, (component, sign) in sorted(expected_shifts.items())
    )


def expected_shifts_metric(
    expected_shifts: dict[str, tuple[str, int]] | None,
) -> dict[str, str]:
    return {
        axis: "%s%s" % ("+" if sign > 0 else "-", component)
        for axis, (component, sign) in (expected_shifts or {}).items()
    }


def axis_expectation_gaps(
    metrics: dict[str, Any],
    *,
    expected_shifts: dict[str, tuple[str, int]] | None,
    min_shift_px: float,
) -> list[str]:
    gaps: list[str] = []
    expected = expected_shifts_metric(expected_shifts)
    if expected:
        if metrics.get("axis_expected_shifts") != expected:
            gaps.append("AXIS_EXPECTED_SHIFTS_MISMATCH")
        observed_min = metrics.get("axis_min_shift_px")
        try:
            observed_min_float = float(observed_min)
        except (TypeError, ValueError):
            observed_min_float = None
        if observed_min_float != float(min_shift_px):
            gaps.append("AXIS_MIN_SHIFT_MISMATCH")
    return gaps


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


def status_by_name(items: list[StatusItem]) -> dict[str, str]:
    return {item.name: item.status for item in items}


READINESS_STAGES = [
    {
        "stage": "S0",
        "name": "local_sandbox_isolation",
        "requires_pass": [
            "local_acceptance",
            "gazebo_betaflight_process_boundary",
        ],
        "unblocks": ["target_setup"],
    },
    {
        "stage": "S1",
        "name": "target_setup",
        "requires_pass": ["target_window", "target_bbox"],
        "unblocks": ["dry_perception_control"],
    },
    {
        "stage": "S2",
        "name": "dry_perception_control",
        "requires_pass": [
            "external_window_preflight",
            "external_follow_session",
        ],
        "unblocks": ["control_binding_evidence", "live_input_readiness"],
    },
    {
        "stage": "S3",
        "name": "control_binding_evidence",
        "requires_pass": ["uinput_environment"],
        "requires_any_pass": [
            "rc_binding_assistant",
            "external_live_input_readiness",
            "external_live_follow_sequence",
        ],
        "unblocks": ["live_input_readiness"],
    },
    {
        "stage": "S4",
        "name": "live_input_readiness",
        "requires_pass": ["external_live_input_readiness"],
        "unblocks": ["bounded_live_follow"],
    },
    {
        "stage": "S5",
        "name": "bounded_live_follow",
        "requires_pass": ["external_live_follow_sequence"],
        "unblocks": ["closed_loop_pr0p_follow_proof"],
    },
]


def readiness_ladder(items: list[StatusItem]) -> list[dict[str, Any]]:
    statuses = status_by_name(items)
    ladder: list[dict[str, Any]] = []
    for definition in READINESS_STAGES:
        required = list(definition.get("requires_pass", []))
        any_required = list(definition.get("requires_any_pass", []))
        required_missing = [
            "%s=%s" % (name, statuses.get(name, "UNKNOWN"))
            for name in required
            if statuses.get(name) != PASS
        ]
        any_missing: list[str] = []
        if any_required and not any(statuses.get(name) == PASS for name in any_required):
            any_missing = [
                "%s=%s" % (name, statuses.get(name, "UNKNOWN"))
                for name in any_required
            ]
        checked_names = required + any_required
        if any(statuses.get(name) == REJECT for name in checked_names):
            stage_status = REJECT
        elif not required_missing and not any_missing:
            stage_status = PASS
        else:
            stage_status = WAITING
        ladder.append({
            "stage": definition["stage"],
            "name": definition["name"],
            "status": stage_status,
            "requires_pass": required,
            "requires_any_pass": any_required,
            "unmet_requires_pass": required_missing,
            "unmet_requires_any_pass": any_missing,
            "unblocks": list(definition.get("unblocks", [])),
            "item_statuses": {
                name: statuses.get(name, "UNKNOWN")
                for name in checked_names
            },
        })
    return ladder


def first_blocking_stage(ladder: list[dict[str, Any]]) -> dict[str, Any] | None:
    return next((stage for stage in ladder if stage.get("status") != PASS), None)


def evidence_mtime_iso(mtime_s: float) -> str:
    return time.strftime("%Y-%m-%dT%H:%M:%S%z", time.localtime(mtime_s))


def build_evidence_freshness(
    evidence_paths: dict[str, str | None],
    *,
    now_s: float | None = None,
    stale_after_s: float = DEFAULT_EVIDENCE_STALE_AFTER_S,
) -> dict[str, Any]:
    now = time.time() if now_s is None else now_s
    artifacts: dict[str, dict[str, Any]] = {}
    for key, value in evidence_paths.items():
        if not value:
            artifacts[key] = {
                "path": None,
                "exists": False,
                "freshness": "MISSING",
                "age_seconds": None,
                "mtime": None,
            }
            continue
        path = Path(value)
        if not path.exists():
            artifacts[key] = {
                "path": value,
                "exists": False,
                "freshness": "MISSING",
                "age_seconds": None,
                "mtime": None,
            }
            continue
        mtime = path.stat().st_mtime
        age = max(0.0, now - mtime)
        freshness = "FRESH" if age <= stale_after_s else "STALE"
        artifacts[key] = {
            "path": value,
            "exists": True,
            "freshness": freshness,
            "age_seconds": round(age, 3),
            "mtime": evidence_mtime_iso(mtime),
        }
    stale_keys = [
        key for key, data in artifacts.items()
        if data.get("freshness") == "STALE"
    ]
    missing_keys = [
        key for key, data in artifacts.items()
        if data.get("freshness") == "MISSING"
    ]
    fresh_keys = [
        key for key, data in artifacts.items()
        if data.get("freshness") == "FRESH"
    ]
    return {
        "stale_after_s": stale_after_s,
        "fresh_count": len(fresh_keys),
        "stale_count": len(stale_keys),
        "missing_count": len(missing_keys),
        "fresh_keys": fresh_keys,
        "stale_keys": stale_keys,
        "missing_keys": missing_keys,
        "artifacts": artifacts,
    }


def required_fresh_evidence_keys(
    *,
    require_live_input: bool,
    require_live_follow: bool,
) -> list[str]:
    keys = [
        "latest_acceptance",
        "matching_external_preflight",
        "matching_external_follow",
        "bbox_file",
    ]
    if require_live_input:
        keys.append("matching_external_live_input_readiness")
    if require_live_follow:
        keys.append("matching_external_live_follow_sequence")
    return keys


def build_freshness_gate(
    freshness: dict[str, Any],
    *,
    required_keys: list[str],
) -> dict[str, Any]:
    artifacts = freshness.get("artifacts", {})
    stale_keys: list[str] = []
    missing_keys: list[str] = []
    fresh_keys: list[str] = []
    for key in required_keys:
        data = artifacts.get(key, {})
        state = data.get("freshness") if isinstance(data, dict) else None
        if state == "FRESH":
            fresh_keys.append(key)
        elif state == "STALE":
            stale_keys.append(key)
        else:
            missing_keys.append(key)
    return {
        "status": PASS if not stale_keys and not missing_keys else WAITING,
        "required_keys": required_keys,
        "fresh_keys": fresh_keys,
        "stale_keys": stale_keys,
        "missing_keys": missing_keys,
    }


def build_resume_commands(
    *,
    items: list[StatusItem],
    window_title: str | None,
    bbox_file: Path,
    require_live_input: bool,
    require_live_follow: bool,
    require_fresh_evidence: bool,
    evidence_stale_after_s: float,
    axis_sweep_axes: tuple[str, ...],
    axis_expected_shifts: dict[str, tuple[str, int]] | None,
    axis_min_shift_px: float,
) -> list[dict[str, Any]]:
    statuses = status_by_name(items)
    title = window_title or "pr0p"
    run_slug = "".join(
        char.lower() if char.isalnum() else "-"
        for char in title
    ).strip("-") or "target-window"
    commands: list[dict[str, Any]] = [
        {
            "name": "status",
            "purpose": "read current sandbox readiness without launching capture or input",
            "safety_class": "status_only",
            "sends_uinput": False,
            "expected_status": "READY_OR_WAITING_STATUS_REPORT",
            "unblocks": ["resume_decision"],
            "expected_report_glob": "*-status-report.json",
            "command": shell_command([
                "fpv_env/bin/python",
                "experiments/game_screen_sandbox/sandbox_status_report.py",
                "--window-title",
                title,
                "--bbox-file",
                bbox_file,
                *(
                    ["--require-live-follow"]
                    if require_live_follow else
                    ["--require-live-input"]
                    if require_live_input else
                    []
                ),
                *(
                    ["--axis-sweep-axes", format_axes(axis_sweep_axes)]
                    if axis_sweep_axes != ("yaw", "pitch") else
                    []
                ),
                *(
                    [
                        "--require-fresh-evidence",
                        "--evidence-stale-after-s",
                        evidence_stale_after_s,
                    ]
                    if require_fresh_evidence else
                    []
                ),
                *expected_shift_command_args(axis_expected_shifts, axis_min_shift_px),
            ]),
        },
    ]
    if statuses.get("target_bbox") != PASS:
        commands.append({
            "name": "capture_bbox_frame",
            "purpose": "capture one frame for manual bbox selection",
            "safety_class": "capture_only",
            "sends_uinput": False,
            "requires_pass": ["target_window"],
            "expected_status": WAITING,
            "unblocks": ["target_bbox"],
            "command": shell_command([
                "fpv_env/bin/python",
                "experiments/game_screen_sandbox/bbox_tool.py",
                "--window-title",
                title,
                "--capture-backend",
                "ffmpeg",
                "--frame-path",
                bbox_file.with_name("%s-frame.png" % bbox_file.stem),
            ]),
        })
        commands.append({
            "name": "write_bbox_file_template",
            "purpose": "rerun after choosing the target bbox in the captured frame",
            "safety_class": "manual_bbox_write",
            "sends_uinput": False,
            "requires_edit": True,
            "requires_pass": ["target_window"],
            "expected_status": PASS,
            "unblocks": ["target_bbox"],
            "expected_report_path": str(bbox_file),
            "command": shell_command([
                "fpv_env/bin/python",
                "experiments/game_screen_sandbox/bbox_tool.py",
                "--window-title",
                title,
                "--capture-backend",
                "ffmpeg",
                "--bbox",
                "X,Y,W,H",
                "--frame-path",
                bbox_file.with_name("%s-frame.png" % bbox_file.stem),
                "--report-path",
                bbox_file,
            ]),
        })
        commands.append({
            "name": "interactive_bbox_select",
            "purpose": "capture one frame and select the target bbox interactively",
            "safety_class": "manual_bbox_select",
            "sends_uinput": False,
            "requires_user_interaction": True,
            "requires_pass": ["target_window"],
            "expected_status": PASS,
            "unblocks": ["target_bbox"],
            "expected_report_path": str(bbox_file),
            "command": shell_command([
                "fpv_env/bin/python",
                "experiments/game_screen_sandbox/bbox_tool.py",
                "--window-title",
                title,
                "--capture-backend",
                "ffmpeg",
                "--interactive-select",
                "--frame-path",
                bbox_file.with_name("%s-frame.png" % bbox_file.stem),
                "--report-path",
                bbox_file,
            ]),
        })
    if (
        statuses.get("target_bbox") == PASS
        and (
            statuses.get("external_window_preflight") != PASS
            or statuses.get("external_follow_session") != PASS
        )
    ):
        commands.append({
            "name": "dry_sequence",
            "purpose": "run status, preflight, and follow dry-run without uinput",
            "safety_class": "dry_run_capture",
            "sends_uinput": False,
            "requires_pass": [
                "target_window",
                "target_bbox",
                "gazebo_betaflight_process_boundary",
            ],
            "expected_status": "EXTERNAL_DRY_RUN_READY",
            "unblocks": [
                "external_window_preflight",
                "external_follow_session",
            ],
            "expected_report_glob": "*-external-dry-run-sequence.json",
            "command": shell_command([
                "fpv_env/bin/python",
                "experiments/game_screen_sandbox/external_dry_run_sequence.py",
                "--capture-window-title",
                title,
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
                "%s-dry-sequence" % run_slug,
            ]),
        })
    if (
        statuses.get("target_bbox") == PASS
        and (
            require_live_input
            or require_live_follow
            or statuses.get("external_live_input_readiness") != PASS
        )
        and statuses.get("external_window_preflight") == PASS
        and statuses.get("external_follow_session") == PASS
        and statuses.get("external_live_input_readiness") != PASS
        and statuses.get("rc_binding_assistant") != PASS
    ):
        commands.append({
            "name": "rc_binding_assistant",
            "purpose": "pulse virtual RC axes while the simulator Controls -> RC Channels page is open",
            "safety_class": "live_binding_ack_required",
            "sends_uinput": True,
            "requires_ack_live_input": True,
            "requires_pass": [
                "target_window",
                "target_bbox",
                "external_window_preflight",
                "external_follow_session",
                "uinput_environment",
                "gazebo_betaflight_process_boundary",
            ],
            "expected_status": PASS,
            "unblocks": ["rc_channel_binding"],
            "expected_report_glob": "*-rc-binding.json",
            "command": shell_command([
                "fpv_env/bin/python",
                "experiments/game_screen_sandbox/rc_binding_assistant.py",
                "--window-title",
                title,
                "--tracker-bbox-file",
                bbox_file,
                "--axes",
                "yaw,pitch,roll,throttle",
                "--axis-value",
                0.6,
                "--hold-seconds",
                0.8,
                "--neutral-seconds",
                0.3,
                "--uinput",
                "--ack-live-input",
                "--countdown-seconds",
                3,
                "--run-id",
                "%s-rc-binding" % run_slug,
            ]),
        })
    if (
        statuses.get("target_bbox") == PASS
        and (
            require_live_input
            or require_live_follow
            or statuses.get("external_live_input_readiness") != PASS
        )
    ):
        commands.append({
            "name": "live_input_readiness",
            "purpose": "verify visual response to virtual RC axis sweep",
            "safety_class": "live_input_ack_required",
            "sends_uinput": True,
            "requires_ack_live_input": True,
            "requires_pass": [
                "target_window",
                "target_bbox",
                "external_window_preflight",
                "external_follow_session",
                "uinput_environment",
                "gazebo_betaflight_process_boundary",
            ],
            "expected_status": EXTERNAL_LIVE_INPUT_READY,
            "unblocks": ["external_live_input_readiness"],
            "expected_report_glob": "*-external-live-input-readiness.json",
            "command": shell_command([
                "fpv_env/bin/python",
                "experiments/game_screen_sandbox/external_live_input_readiness.py",
                "--capture-window-title",
                title,
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
                "--ack-live-input",
                "--axis-sweep-axes",
                format_axes(axis_sweep_axes),
                *expected_shift_command_args(axis_expected_shifts, axis_min_shift_px),
                "--run-id",
                "%s-live-input-readiness" % run_slug,
            ]),
        })
    if (
        statuses.get("target_bbox") == PASS
        and (
            require_live_follow
            or statuses.get("external_live_follow_sequence") != PASS
        )
    ):
        commands.append({
            "name": "live_follow_sequence",
            "purpose": "run bounded live follow after live-input readiness",
            "safety_class": "live_follow_ack_required",
            "sends_uinput": True,
            "requires_ack_live_input": True,
            "requires_pass": [
                "target_window",
                "target_bbox",
                "external_window_preflight",
                "external_follow_session",
                "external_live_input_readiness",
                "uinput_environment",
                "gazebo_betaflight_process_boundary",
            ],
            "expected_status": EXTERNAL_LIVE_FOLLOW_COMPLETE,
            "unblocks": ["external_live_follow_sequence"],
            "expected_report_glob": "*-external-live-follow-sequence.json",
            "command": shell_command([
                "fpv_env/bin/python",
                "experiments/game_screen_sandbox/external_live_follow_sequence.py",
                "--capture-window-title",
                title,
                "--capture-backend",
                "ffmpeg",
                "--tracker-bbox-file",
                bbox_file,
                "--duration",
                1,
                "--hz",
                10,
                "--enable-pitch",
                "--desired-target-width",
                120,
                "--ack-live-input",
                "--axis-sweep-axes",
                format_axes(axis_sweep_axes),
                *expected_shift_command_args(axis_expected_shifts, axis_min_shift_px),
                "--live-max-duration",
                2,
                "--run-id",
                "%s-live-follow-sequence" % run_slug,
            ]),
        })
    return commands


def check_acceptance(log_dir: Path) -> tuple[StatusItem, Path | None]:
    path = latest_report(log_dir, "*-acceptance-run.json")
    data = load_json_report(path)
    if not data:
        return StatusItem(
            "local_acceptance",
            WAITING,
            "no local sandbox acceptance report found",
        ), path
    status = str(data.get("status", "UNKNOWN"))
    item_status = PASS if status == "SIMPLE_SANDBOX_READY" else REJECT if status == REJECT else WAITING
    return StatusItem(
        "local_acceptance",
        item_status,
        "latest local sandbox acceptance status is %s" % status,
        metrics={
            "status": status,
            "summary": data.get("summary"),
            "decision_report_md": data.get("decision_report_md"),
        },
    ), path


def check_bbox(path: Path) -> StatusItem:
    if not path.exists():
        return StatusItem(
            "target_bbox",
            WAITING,
            "target bbox file is missing",
            metrics={"path": str(path)},
        )
    try:
        bbox = load_tracker_bbox_file(path)
    except Exception as exc:
        return StatusItem(
            "target_bbox",
            REJECT,
            "target bbox file is invalid",
            metrics={"path": str(path), "error": str(exc)},
        )
    return StatusItem(
        "target_bbox",
        PASS,
        "target bbox file is usable",
        metrics={"path": str(path), "bbox": list(bbox)},
    )


def check_window(
    window_title: str | None,
    *,
    exact: bool,
    case_sensitive: bool,
    min_width: int,
    min_height: int,
    list_windows_fn: Callable[[], Any] = list_x11_windows,
) -> StatusItem:
    if not window_title:
        return StatusItem(
            "target_window",
            WAITING,
            "no target window title was provided",
        )
    if not os.environ.get("DISPLAY"):
        return StatusItem(
            "target_window",
            WAITING,
            "DISPLAY is not set, so X11 window discovery cannot run",
            metrics={"window_title": window_title},
        )
    try:
        windows = list_windows_fn()
        match = find_window_by_title(
            window_title,
            windows,
            exact=exact,
            case_sensitive=case_sensitive,
            min_width=min_width,
            min_height=min_height,
        )
    except Exception as exc:
        return StatusItem(
            "target_window",
            WAITING,
            "target window discovery is waiting",
            metrics={"window_title": window_title, "error": str(exc)},
        )
    if match is None:
        return StatusItem(
            "target_window",
            WAITING,
            "no visible window matched the target title",
            metrics={"window_title": window_title},
        )
    return StatusItem(
        "target_window",
        PASS,
        "target window is visible",
        metrics={"window": match.as_dict()},
    )


def check_matching_preflight(
    log_dir: Path,
    window_title: str | None,
    tracker_bbox: list[int] | None,
) -> tuple[StatusItem, Path | None]:
    if tracker_bbox is None:
        return StatusItem(
            "external_window_preflight",
            WAITING,
            "current bbox is required before matching external preflight evidence",
            metrics={"window_title": window_title, "tracker_bbox": None},
        ), None
    path, data = matching_report(
        log_dir,
        "*-external-window-preflight.json",
        window_title=window_title,
        tracker_bbox=tracker_bbox,
    )
    if not data:
        return StatusItem(
            "external_window_preflight",
            WAITING,
            "no matching external window preflight report found for current bbox",
            metrics={"window_title": window_title, "tracker_bbox": tracker_bbox},
        ), path
    status = str(data.get("status", "UNKNOWN"))
    item_status = (
        PASS if status in {"EXTERNAL_WINDOW_READY_DRY", "EXTERNAL_WINDOW_READY_LIVE_INPUT"}
        else REJECT if status == REJECT
        else WAITING
    )
    return StatusItem(
        "external_window_preflight",
        item_status,
        "latest matching external preflight status is %s" % status,
        metrics={
            "status": status,
            "reasons": data.get("reasons", []),
            "phase_statuses": data.get("metrics", {}).get("phase_statuses", {}),
            "tracker_bbox": tracker_bbox,
        },
    ), path


def check_matching_follow(
    log_dir: Path,
    window_title: str | None,
    tracker_bbox: list[int] | None,
) -> tuple[StatusItem, Path | None]:
    if tracker_bbox is None:
        return StatusItem(
            "external_follow_session",
            WAITING,
            "current bbox is required before matching external follow evidence",
            metrics={"window_title": window_title, "tracker_bbox": None},
        ), None
    path, data = matching_report(
        log_dir,
        "*-external-follow-session.json",
        window_title=window_title,
        tracker_bbox=tracker_bbox,
    )
    if not data:
        return StatusItem(
            "external_follow_session",
            WAITING,
            "no matching external follow session report found for current bbox",
            metrics={"window_title": window_title, "tracker_bbox": tracker_bbox},
        ), path
    status = str(data.get("status", "UNKNOWN"))
    item_status = (
        PASS if status in {"EXTERNAL_FOLLOW_DRY_RUN_READY", "EXTERNAL_FOLLOW_LIVE_RUN_COMPLETE"}
        else REJECT if status == REJECT
        else WAITING
    )
    loop_summary = data.get("metrics", {}).get("loop_summary", {})
    return StatusItem(
        "external_follow_session",
        item_status,
        "latest matching external follow status is %s" % status,
        metrics={
            "status": status,
            "reasons": data.get("reasons", []),
            "found_ratio": loop_summary.get("found_ratio") if isinstance(loop_summary, dict) else None,
            "loss_events": loop_summary.get("loss_events") if isinstance(loop_summary, dict) else None,
            "tracker_bbox": tracker_bbox,
        },
    ), path


def check_matching_live_input_readiness(
    log_dir: Path,
    window_title: str | None,
    tracker_bbox: list[int] | None,
    axis_expected_shifts: dict[str, tuple[str, int]] | None = None,
    axis_min_shift_px: float = 0.5,
) -> tuple[StatusItem, Path | None]:
    if tracker_bbox is None:
        return StatusItem(
            "external_live_input_readiness",
            WAITING,
            "current bbox is required before matching external live-input readiness evidence",
            metrics={"window_title": window_title, "tracker_bbox": None},
        ), None
    path, data = matching_report(
        log_dir,
        "*-external-live-input-readiness.json",
        window_title=window_title,
        tracker_bbox=tracker_bbox,
    )
    if not data:
        return StatusItem(
            "external_live_input_readiness",
            WAITING,
            "no matching external live-input readiness report found for current bbox",
            metrics={"window_title": window_title, "tracker_bbox": tracker_bbox},
        ), path
    status = str(data.get("status", "UNKNOWN"))
    item_status = (
        PASS if status == EXTERNAL_LIVE_INPUT_READY
        else REJECT if status == REJECT
        else WAITING
    )
    readiness_steps = data.get("metrics", {}).get("readiness_steps", [])
    step_statuses = {}
    if isinstance(readiness_steps, list):
        for step in readiness_steps:
            if isinstance(step, dict):
                step_statuses[str(step.get("step"))] = step.get("status")
    metrics = data.get("metrics", {})
    if not isinstance(metrics, dict):
        metrics = {}
    axis_preflight = metrics.get("axis_preflight", {})
    axis_preflight_metrics = (
        axis_preflight.get("metrics", {})
        if isinstance(axis_preflight, dict) else {}
    )
    phases = axis_preflight.get("phases", []) if isinstance(axis_preflight, dict) else []
    s5_phase: dict[str, Any] | None = None
    if isinstance(phases, list):
        for phase in phases:
            if isinstance(phase, dict) and phase.get("phase") == "S5-real":
                s5_phase = phase
                break
    s5_metrics = (
        s5_phase.get("metrics", {}) if isinstance(s5_phase, dict) else {}
    )
    proof_gaps: list[str] = []
    if status == EXTERNAL_LIVE_INPUT_READY:
        if step_statuses.get("dry_sequence") != "EXTERNAL_DRY_RUN_READY":
            proof_gaps.append("DRY_SEQUENCE_STEP_NOT_READY")
        if step_statuses.get("ack_live_input") != PASS:
            proof_gaps.append("ACK_STEP_NOT_PASS")
        if step_statuses.get("axis_response") != "EXTERNAL_WINDOW_READY_LIVE_INPUT":
            proof_gaps.append("AXIS_RESPONSE_STEP_NOT_READY")
        if (
            not isinstance(axis_preflight, dict)
            or axis_preflight.get("status") != "EXTERNAL_WINDOW_READY_LIVE_INPUT"
        ):
            proof_gaps.append("AXIS_PREFLIGHT_STATUS_NOT_LIVE_INPUT")
        if not isinstance(s5_phase, dict) or s5_phase.get("status") != PASS:
            proof_gaps.append("S5_PHASE_NOT_PASS")
        if not isinstance(s5_metrics, dict):
            proof_gaps.append("S5_METRICS_MISSING")
        else:
            if s5_metrics.get("real_input") is not True:
                proof_gaps.append("S5_REAL_INPUT_NOT_TRUE")
            if s5_metrics.get("adapter") != "UInputAdapter":
                proof_gaps.append("S5_ADAPTER_NOT_UINPUT")
            if s5_metrics.get("axis_sweep") is not True:
                proof_gaps.append("S5_AXIS_SWEEP_NOT_TRUE")
        proof_gaps.extend(
            axis_expectation_gaps(
                metrics,
                expected_shifts=axis_expected_shifts,
                min_shift_px=axis_min_shift_px,
            )
        )
    if status == EXTERNAL_LIVE_INPUT_READY and proof_gaps:
        item_status = WAITING
    return StatusItem(
        "external_live_input_readiness",
        item_status,
        (
            "matching live-input readiness lacks live uinput proof"
            if proof_gaps else
            "latest matching live-input readiness status is %s" % status
        ),
        metrics={
            "status": status,
            "reasons": data.get("reasons", []),
            "readiness_step_statuses": step_statuses,
            "proof_gaps": proof_gaps,
            "real_input": (
                s5_metrics.get("real_input")
                if isinstance(s5_metrics, dict) else None
            ),
            "adapter": (
                s5_metrics.get("adapter")
                if isinstance(s5_metrics, dict) else None
            ),
            "axis_sweep": (
                s5_metrics.get("axis_sweep")
                if isinstance(s5_metrics, dict) else None
            ),
            "axis_expected_shifts": metrics.get("axis_expected_shifts"),
            "axis_min_shift_px": metrics.get("axis_min_shift_px"),
            "tracker_bbox": tracker_bbox,
        },
    ), path


def check_matching_rc_binding(
    log_dir: Path,
    window_title: str | None,
    tracker_bbox: list[int] | None,
) -> tuple[StatusItem, Path | None]:
    if tracker_bbox is None:
        return StatusItem(
            "rc_binding_assistant",
            WAITING,
            "current bbox is required before matching RC binding evidence",
            metrics={"window_title": window_title, "tracker_bbox": None},
        ), None
    candidates: list[tuple[Path, dict[str, Any]]] = []
    for path in log_dir.glob("*-rc-binding.json"):
        data = load_json_report(path)
        if not data or str(data.get("status")) != PASS:
            continue
        metrics = data.get("metrics", {})
        if not isinstance(metrics, dict):
            continue
        if window_title and metrics.get("window_title") != window_title:
            continue
        if not report_matches_bbox(data, tracker_bbox):
            continue
        candidates.append((path, data))
    if not candidates:
        return StatusItem(
            "rc_binding_assistant",
            WAITING,
            "no matching RC binding assistant PASS report found for current bbox",
            metrics={"window_title": window_title, "tracker_bbox": tracker_bbox},
        ), None
    latest_path, latest_data = max(candidates, key=lambda item: report_sort_key(item[0]))
    ready_candidates: list[tuple[Path, dict[str, Any]]] = []
    for path, data in candidates:
        metrics = data.get("metrics", {})
        if not isinstance(metrics, dict):
            continue
        try:
            command_count = int(metrics.get("command_count") or 0)
        except (TypeError, ValueError):
            command_count = 0
        if (
            metrics.get("real_input") is True
            and metrics.get("ack_live_input") is True
            and metrics.get("neutralized") is True
            and command_count > 0
        ):
            ready_candidates.append((path, data))
    if not ready_candidates:
        latest_metrics = latest_data.get("metrics", {})
        if not isinstance(latest_metrics, dict):
            latest_metrics = {}
        return StatusItem(
            "rc_binding_assistant",
            WAITING,
            "matching RC binding report is dry-run or incomplete; live uinput binding evidence is required",
            metrics={
                "status": latest_data.get("status"),
                "window_title": latest_metrics.get("window_title"),
                "tracker_bbox": latest_metrics.get("tracker_bbox"),
                "axes": latest_metrics.get("axes"),
                "real_input": latest_metrics.get("real_input"),
                "ack_live_input": latest_metrics.get("ack_live_input"),
                "neutralized": latest_metrics.get("neutralized"),
                "command_count": latest_metrics.get("command_count"),
            },
        ), latest_path
    ready_path, ready_data = max(
        ready_candidates,
        key=lambda item: report_sort_key(item[0]),
    )
    metrics = ready_data.get("metrics", {})
    if not isinstance(metrics, dict):
        metrics = {}
    return StatusItem(
        "rc_binding_assistant",
        PASS,
        "matching live RC binding assistant evidence is ready",
        metrics={
            "status": ready_data.get("status"),
            "window_title": metrics.get("window_title"),
            "tracker_bbox": metrics.get("tracker_bbox"),
            "axes": metrics.get("axes"),
            "real_input": metrics.get("real_input"),
            "ack_live_input": metrics.get("ack_live_input"),
            "neutralized": metrics.get("neutralized"),
            "command_count": metrics.get("command_count"),
        },
    ), ready_path


def check_matching_live_follow_sequence(
    log_dir: Path,
    window_title: str | None,
    tracker_bbox: list[int] | None,
    axis_expected_shifts: dict[str, tuple[str, int]] | None = None,
    axis_min_shift_px: float = 0.5,
) -> tuple[StatusItem, Path | None]:
    if tracker_bbox is None:
        return StatusItem(
            "external_live_follow_sequence",
            WAITING,
            "current bbox is required before matching external live-follow evidence",
            metrics={"window_title": window_title, "tracker_bbox": None},
        ), None
    path, data = matching_report(
        log_dir,
        "*-external-live-follow-sequence.json",
        window_title=window_title,
        tracker_bbox=tracker_bbox,
    )
    if not data:
        return StatusItem(
            "external_live_follow_sequence",
            WAITING,
            "no matching external live-follow sequence report found for current bbox",
            metrics={"window_title": window_title, "tracker_bbox": tracker_bbox},
        ), path
    status = str(data.get("status", "UNKNOWN"))
    item_status = (
        PASS if status == EXTERNAL_LIVE_FOLLOW_COMPLETE
        else REJECT if status == REJECT
        else WAITING
    )
    metrics = data.get("metrics", {})
    if not isinstance(metrics, dict):
        metrics = {}
    sequence_steps = metrics.get("sequence_steps", [])
    step_statuses = {}
    if isinstance(sequence_steps, list):
        for step in sequence_steps:
            if isinstance(step, dict):
                step_statuses[str(step.get("step"))] = step.get("status")
    follow = metrics.get("follow", {})
    follow_metrics = follow.get("metrics", {}) if isinstance(follow, dict) else {}
    loop_summary = (
        follow_metrics.get("loop_summary", {})
        if isinstance(follow_metrics, dict) else {}
    )
    proof_gaps: list[str] = []
    if status == EXTERNAL_LIVE_FOLLOW_COMPLETE:
        if step_statuses.get("ack_live_input") != PASS:
            proof_gaps.append("ACK_STEP_NOT_PASS")
        if step_statuses.get("live_input_readiness") != EXTERNAL_LIVE_INPUT_READY:
            proof_gaps.append("READINESS_STEP_NOT_READY")
        if step_statuses.get("live_follow") != EXTERNAL_FOLLOW_LIVE_RUN_COMPLETE:
            proof_gaps.append("LIVE_FOLLOW_STEP_NOT_COMPLETE")
        if not isinstance(follow, dict) or follow.get("status") != EXTERNAL_FOLLOW_LIVE_RUN_COMPLETE:
            proof_gaps.append("FOLLOW_STATUS_NOT_LIVE_COMPLETE")
        if not isinstance(follow_metrics, dict):
            proof_gaps.append("FOLLOW_METRICS_MISSING")
        else:
            if follow_metrics.get("real_input") is not True:
                proof_gaps.append("FOLLOW_REAL_INPUT_NOT_TRUE")
            if follow_metrics.get("ack_live_input") is not True:
                proof_gaps.append("FOLLOW_ACK_NOT_TRUE")
            if follow_metrics.get("adapter") != "UInputAdapter":
                proof_gaps.append("FOLLOW_ADAPTER_NOT_UINPUT")
        proof_gaps.extend(
            axis_expectation_gaps(
                metrics,
                expected_shifts=axis_expected_shifts,
                min_shift_px=axis_min_shift_px,
            )
        )
    if status == EXTERNAL_LIVE_FOLLOW_COMPLETE and proof_gaps:
        item_status = WAITING
    return StatusItem(
        "external_live_follow_sequence",
        item_status,
        (
            "matching live-follow sequence lacks live uinput proof"
            if proof_gaps else
            "latest matching live-follow sequence status is %s" % status
        ),
        metrics={
            "status": status,
            "reasons": data.get("reasons", []),
            "sequence_step_statuses": step_statuses,
            "proof_gaps": proof_gaps,
            "real_input": (
                follow_metrics.get("real_input")
                if isinstance(follow_metrics, dict) else None
            ),
            "ack_live_input": (
                follow_metrics.get("ack_live_input")
                if isinstance(follow_metrics, dict) else None
            ),
            "adapter": (
                follow_metrics.get("adapter")
                if isinstance(follow_metrics, dict) else None
            ),
            "axis_expected_shifts": metrics.get("axis_expected_shifts"),
            "axis_min_shift_px": metrics.get("axis_min_shift_px"),
            "found_ratio": (
                loop_summary.get("found_ratio")
                if isinstance(loop_summary, dict) else None
            ),
            "loss_events": (
                loop_summary.get("loss_events")
                if isinstance(loop_summary, dict) else None
            ),
            "tracker_bbox": tracker_bbox,
        },
    ), path


def check_forbidden_processes() -> StatusItem:
    processes = process_snapshot()
    status = PASS if not processes else REJECT
    return StatusItem(
        "gazebo_betaflight_process_boundary",
        status,
        (
            "no forbidden Gazebo/Betaflight process is running"
            if status == PASS else
            "forbidden Gazebo/Betaflight process is running"
        ),
        metrics={"forbidden_processes": processes},
    )


def check_uinput() -> StatusItem:
    probe = probe_uinput_environment()
    return StatusItem(
        "uinput_environment",
        PASS if probe.available else WAITING,
        (
            "uinput is available"
            if probe.available else
            "uinput is not ready; dry-run path can still be used"
        ),
        metrics=probe.as_dict(),
    )


def next_actions_for(
    items: list[StatusItem],
    *,
    window_title: str | None,
    bbox_file: Path,
    require_live_input: bool,
    require_live_follow: bool,
) -> list[str]:
    status_by_name = {item.name: item.status for item in items}
    actions: list[str] = []
    if status_by_name.get("gazebo_betaflight_process_boundary") == REJECT:
        actions.append("stop forbidden Gazebo/Betaflight processes before sandbox work")
    if status_by_name.get("local_acceptance") != PASS:
        actions.append("run sandbox_acceptance_runner.py --include-simple-window")
    if status_by_name.get("target_window") != PASS:
        actions.append("open the external game/sim window or pass the correct --window-title")
    if status_by_name.get("target_bbox") != PASS:
        actions.append("create bbox file with bbox_tool.py at %s" % bbox_file)
    if status_by_name.get("external_window_preflight") != PASS:
        actions.append("run external_window_preflight.py for %s" % (window_title or "the target window"))
    if status_by_name.get("external_follow_session") != PASS:
        actions.append("run external_follow_session.py dry-run for %s" % (window_title or "the target window"))
    if (
        (require_live_input or require_live_follow)
        and status_by_name.get("external_window_preflight") == PASS
        and status_by_name.get("external_follow_session") == PASS
        and status_by_name.get("external_live_input_readiness") != PASS
        and status_by_name.get("external_live_follow_sequence") != PASS
        and status_by_name.get("rc_binding_assistant") != PASS
    ):
        actions.append(
            "run rc_binding_assistant.py on Controls -> RC Channels for %s if axes are not bound"
            % (window_title or "the target window")
        )
    if (
        require_live_input
        and status_by_name.get("external_live_input_readiness") != PASS
    ):
        actions.append(
            "run external_live_input_readiness.py --ack-live-input for %s"
            % (window_title or "the target window")
        )
    if (
        require_live_follow
        and status_by_name.get("external_live_follow_sequence") != PASS
    ):
        actions.append(
            "run external_live_follow_sequence.py --ack-live-input for %s"
            % (window_title or "the target window")
        )
    if not actions:
        if status_by_name.get("external_live_follow_sequence") == PASS:
            actions.append("external dry-run, live-input, and live-follow evidence are ready")
        elif status_by_name.get("external_live_input_readiness") == PASS:
            actions.append("external dry-run and live-input readiness evidence are ready")
        else:
            actions.append(
                "external dry-run path is ready; optional live input requires "
                "external_live_input_readiness.py --ack-live-input"
            )
    return actions


def overall_status(
    items: list[StatusItem],
    *,
    require_live_input: bool = False,
    require_live_follow: bool = False,
) -> str:
    if any(item.status == REJECT for item in items):
        return REJECT
    required = {
        "local_acceptance",
        "target_window",
        "target_bbox",
        "external_window_preflight",
        "external_follow_session",
        "gazebo_betaflight_process_boundary",
    }
    if require_live_input:
        required.add("external_live_input_readiness")
    if require_live_follow:
        required.add("external_live_follow_sequence")
    present = {item.name: item.status for item in items}
    if all(present.get(name) == PASS for name in required):
        return READY
    return WAITING


def build_status_report(
    *,
    run_id: str,
    log_dir: Path,
    window_title: str | None,
    bbox_file: Path,
    window_exact: bool,
    window_case_sensitive: bool,
    window_min_width: int,
    window_min_height: int,
    require_live_input: bool = False,
    require_live_follow: bool = False,
    require_fresh_evidence: bool = False,
    evidence_stale_after_s: float = DEFAULT_EVIDENCE_STALE_AFTER_S,
    axis_sweep_axes: tuple[str, ...] = ("yaw", "pitch"),
    axis_expected_shifts: dict[str, tuple[str, int]] | None = None,
    axis_min_shift_px: float = 0.5,
) -> SandboxStatusReport:
    acceptance_item, acceptance_path = check_acceptance(log_dir)
    bbox_item = check_bbox(bbox_file)
    window_item = check_window(
        window_title,
        exact=window_exact,
        case_sensitive=window_case_sensitive,
        min_width=window_min_width,
        min_height=window_min_height,
    )
    current_bbox = (
        list(bbox_item.metrics.get("bbox", []))
        if bbox_item.status == PASS else
        None
    )
    preflight_item, preflight_path = check_matching_preflight(
        log_dir,
        window_title,
        current_bbox,
    )
    follow_item, follow_path = check_matching_follow(
        log_dir,
        window_title,
        current_bbox,
    )
    live_input_item, live_input_path = check_matching_live_input_readiness(
        log_dir,
        window_title,
        current_bbox,
        axis_expected_shifts=axis_expected_shifts,
        axis_min_shift_px=axis_min_shift_px,
    )
    rc_binding_item, rc_binding_path = check_matching_rc_binding(
        log_dir,
        window_title,
        current_bbox,
    )
    live_follow_item, live_follow_path = check_matching_live_follow_sequence(
        log_dir,
        window_title,
        current_bbox,
        axis_expected_shifts=axis_expected_shifts,
        axis_min_shift_px=axis_min_shift_px,
    )
    process_item = check_forbidden_processes()
    uinput_item = check_uinput()
    items = [
        acceptance_item,
        window_item,
        bbox_item,
        preflight_item,
        follow_item,
        rc_binding_item,
        live_input_item,
        live_follow_item,
        process_item,
        uinput_item,
    ]
    status = overall_status(
        items,
        require_live_input=require_live_input,
        require_live_follow=require_live_follow,
    )
    ladder = readiness_ladder(items)
    blocking_stage = first_blocking_stage(ladder)
    evidence_paths = {
        "latest_acceptance": str(acceptance_path) if acceptance_path else None,
        "matching_external_preflight": str(preflight_path) if preflight_path else None,
        "matching_external_follow": str(follow_path) if follow_path else None,
        "matching_external_live_input_readiness": (
            str(live_input_path) if live_input_path else None
        ),
        "matching_rc_binding": str(rc_binding_path) if rc_binding_path else None,
        "matching_external_live_follow_sequence": (
            str(live_follow_path) if live_follow_path else None
        ),
        "bbox_file": str(bbox_file) if bbox_file.exists() else None,
    }
    evidence_freshness = build_evidence_freshness(
        evidence_paths,
        stale_after_s=evidence_stale_after_s,
    )
    freshness_gate = build_freshness_gate(
        evidence_freshness,
        required_keys=required_fresh_evidence_keys(
            require_live_input=require_live_input,
            require_live_follow=require_live_follow,
        ),
    )
    base_status = status
    freshness_blocked = (
        require_fresh_evidence
        and base_status == READY
        and freshness_gate.get("status") != PASS
    )
    if freshness_blocked:
        status = WAITING
    actions = next_actions_for(
        items,
        window_title=window_title,
        bbox_file=bbox_file,
        require_live_input=require_live_input,
        require_live_follow=require_live_follow,
    )
    if freshness_blocked:
        stale_or_missing = (
            list(freshness_gate.get("stale_keys", []))
            + list(freshness_gate.get("missing_keys", []))
        )
        actions.insert(
            0,
            "refresh stale/missing evidence for freshness gate: %s"
            % ", ".join(stale_or_missing),
        )
    summary = (
        "external game/screen sandbox is waiting for fresh setup evidence"
        if freshness_blocked else
        (
            "external game/screen sandbox bounded live follow is ready"
            if require_live_follow else
            "external game/screen sandbox live-input readiness is ready"
            if require_live_input else
            "external game/screen sandbox dry-run path is ready"
        ) if status == READY else
        "external game/screen sandbox status has a blocking failure"
        if status == REJECT else
        "external game/screen sandbox is waiting for setup evidence"
    )
    return SandboxStatusReport(
        run_id=run_id,
        started_at=time.strftime("%Y-%m-%dT%H:%M:%S%z", time.localtime()),
        status=status,
        summary=summary,
        items=items,
        next_actions=actions,
        resume_commands=build_resume_commands(
            items=items,
            window_title=window_title,
            bbox_file=bbox_file,
            require_live_input=require_live_input,
            require_live_follow=require_live_follow,
            require_fresh_evidence=require_fresh_evidence,
            evidence_stale_after_s=evidence_stale_after_s,
            axis_sweep_axes=axis_sweep_axes,
            axis_expected_shifts=axis_expected_shifts,
            axis_min_shift_px=axis_min_shift_px,
        ),
        evidence_paths=evidence_paths,
        metrics={
            "window_title": window_title,
            "log_dir": str(log_dir),
            "tracker_bbox": current_bbox,
            "require_live_input": require_live_input,
            "require_live_follow": require_live_follow,
            "require_fresh_evidence": require_fresh_evidence,
            "evidence_stale_after_s": evidence_stale_after_s,
            "base_status_without_freshness_gate": base_status,
            "axis_sweep_axes": list(axis_sweep_axes),
            "axis_expected_shifts": expected_shifts_metric(axis_expected_shifts) or None,
            "axis_min_shift_px": axis_min_shift_px if axis_expected_shifts else None,
            "readiness_ladder": ladder,
            "first_blocking_stage": blocking_stage,
            "evidence_freshness": evidence_freshness,
            "freshness_gate": freshness_gate,
            "scope": "status-only; does not launch Gazebo, Betaflight, simulator, capture, or uinput",
        },
    )


def build_markdown(report: SandboxStatusReport) -> str:
    lines = [
        "# Game Screen Sandbox Status",
        "",
        "Run: `%s`" % report.run_id,
        "Started: `%s`" % report.started_at,
        "Status: `%s`" % report.status,
        "",
        report.summary,
        "",
        "## Readiness Ladder",
        "",
        "| Stage | Name | Status | Blocking |",
        "| --- | --- | --- | --- |",
    ]
    for stage in report.metrics.get("readiness_ladder", []):
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
        "## Items",
        "",
        "| Item | Status | Summary |",
        "| --- | --- | --- |",
    ])
    for item in report.items:
        lines.append("| %s | %s | %s |" % (
            item.name,
            item.status,
            item.summary.replace("|", "\\|"),
        ))
    lines.extend(["", "## Next Actions", ""])
    lines.extend("- %s" % action for action in report.next_actions)
    lines.extend([
        "",
        "## Resume Commands",
        "",
        "| Name | Safety | Expected | Sends uinput | Command |",
        "| --- | --- | --- | --- | --- |",
    ])
    for command in report.resume_commands:
        lines.append("| %s | %s | %s | %s | `%s` |" % (
            command.get("name", "unknown"),
            command.get("safety_class", "unknown"),
            command.get("expected_status", "unknown"),
            "yes" if command.get("sends_uinput") else "no",
            command.get("command", ""),
        ))
    freshness = report.metrics.get("evidence_freshness", {})
    freshness_gate = report.metrics.get("freshness_gate", {})
    artifacts = freshness.get("artifacts", {}) if isinstance(freshness, dict) else {}
    lines.extend([
        "",
        "## Evidence Freshness",
        "",
        "Freshness gate: `%s`" % (
            freshness_gate.get("status", "UNKNOWN")
            if isinstance(freshness_gate, dict) else
            "UNKNOWN"
        ),
        "",
        "| Evidence | Freshness | Age s | Path |",
        "| --- | --- | --- | --- |",
    ])
    for key, data in artifacts.items():
        if not isinstance(data, dict):
            continue
        lines.append("| %s | %s | %s | `%s` |" % (
            key,
            data.get("freshness", "UNKNOWN"),
            data.get("age_seconds")
            if data.get("age_seconds") is not None else
            "n/a",
            data.get("path") or "none",
        ))
    lines.extend([
        "",
        "## Evidence",
        "",
        "| Evidence | Path |",
        "| --- | --- |",
    ])
    for key, path in report.evidence_paths.items():
        lines.append("| %s | `%s` |" % (key, path or "none"))
    lines.extend([
        "",
        "## Metrics",
        "",
        "```json",
        json.dumps(report.as_dict(), indent=2, sort_keys=True),
        "```",
        "",
    ])
    return "\n".join(lines)


def write_reports(report: SandboxStatusReport, log_dir: Path) -> tuple[Path, Path]:
    slug = time.strftime("%Y%m%d-%H%M%S", time.localtime()) + "-%s" % report.run_id
    json_path = log_dir / ("%s-status-report.json" % slug)
    md_path = log_dir / ("%s-status-report.md" % slug)
    json_path.write_text(json.dumps(report.as_dict(), indent=2, sort_keys=True) + "\n",
                         encoding="utf-8")
    md_path.write_text(build_markdown(report), encoding="utf-8")
    return json_path, md_path


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-id", default="sandbox-status")
    parser.add_argument("--log-dir", type=Path, default=DEFAULT_LOG_DIR)
    parser.add_argument("--window-title", default="pr0p")
    parser.add_argument("--bbox-file", type=Path, default=DEFAULT_BBOX_FILE)
    parser.add_argument("--window-exact", action="store_true")
    parser.add_argument("--window-case-sensitive", action="store_true")
    parser.add_argument("--window-min-width", type=int, default=32)
    parser.add_argument("--window-min-height", type=int, default=32)
    parser.add_argument("--require-live-input", action="store_true",
                        help="Require external_live_input_readiness.py evidence for READY")
    parser.add_argument("--require-live-follow", action="store_true",
                        help="Require external_live_follow_sequence.py evidence for READY")
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
    parser.add_argument("--json", action="store_true")
    parser.add_argument("--write-report", action="store_true")
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
    return args


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    report = build_status_report(
        run_id=args.run_id,
        log_dir=args.log_dir,
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
        axis_sweep_axes=args.axis_sweep_axes,
        axis_expected_shifts=args.axis_expected_shifts,
        axis_min_shift_px=args.axis_min_shift_px,
    )
    if args.write_report:
        json_path, md_path = write_reports(report, args.log_dir)
        report.evidence_paths["status_report_json"] = str(json_path)
        report.evidence_paths["status_report_md"] = str(md_path)
    if args.json:
        print(json.dumps(report.as_dict(), indent=2, sort_keys=True))
    else:
        print("sandbox-status %s %s" % (report.status, report.summary))
        for action in report.next_actions:
            print("next: %s" % action)
    if report.status == READY:
        return 0
    if report.status == WAITING:
        return 2
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
