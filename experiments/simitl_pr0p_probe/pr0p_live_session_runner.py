#!/usr/bin/env python3
"""Run a bounded pr0p live session with cleanup and evidence reports."""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable

PROBE_DIR = Path(__file__).resolve().parent
SANDBOX_DIR = PROBE_DIR.parent / "game_screen_sandbox"
if str(SANDBOX_DIR) not in sys.path:
    sys.path.insert(0, str(SANDBOX_DIR))

from preflight_probe import DEFAULT_INSTALL_ROOT, DEFAULT_LOG_DIR, DEFAULT_WS_HOST, DEFAULT_WS_PORT
from msp_uinput_arm_probe import measure_uinput_arm_effect
from msp_uinput_aux_arm_status_probe import measure_uinput_aux_arm_status
from msp_uinput_rc_effect_probe import command_for_axis as rc_command_for_axis
from msp_uinput_rc_effect_probe import measure_uinput_rc_effect
from msp_uinput_status_probe import measure_uinput_status_effect
from msp_uinput_attitude_response_probe import measure_uinput_attitude_response
from msp_ws_status_probe import run_msp_ws_status_once
from pr0p_arm_sequence import arm_and_hover_for_response, response_pulse_command
from pr0p_capture_probe import DEFAULT_WINDOW_TITLES
from pr0p_input_config_probe import (
    DEFAULT_EXPECTED_DEVICE,
    DEFAULT_INPUT_CONFIG,
    run_input_config_probe,
)
from pr0p_probe_suite import run_suite, write_reports as write_suite_reports
from pr0p_response_probe import (
    command_for_axis as response_command_for_axis,
    make_source_from_selection,
    measure_visual_response,
)
from pr0p_tracking_probe import parse_bbox
from pr0p_ui_smoke import run_ui_smoke
from virtual_input import AxisCommand, UInputAdapter, UInputUnavailable, probe_uinput_environment


PASS = "PASS"
WAITING = "WAITING"
FAIL = "FAIL"
DEFAULT_PR0P_EXECUTABLES = ("pr0p.x86_64", "pr0p")
PROCESS_PATTERN = r"pr0p\.x86_64|/tmp/fpv-test-simitl-pr0p/updater"


@dataclass
class LiveSessionResult:
    status: str
    summary: str
    metrics: dict[str, Any] = field(default_factory=dict)
    notes: list[str] = field(default_factory=list)

    def as_dict(self) -> dict[str, Any]:
        return {
            "status": self.status,
            "summary": self.summary,
            "metrics": self.metrics,
            "notes": self.notes,
        }


def combine_status(statuses: list[str]) -> str:
    if any(status == FAIL for status in statuses):
        return FAIL
    if any(status == WAITING for status in statuses):
        return WAITING
    return PASS


def find_pr0p_executable(install_root: Path) -> Path | None:
    for name in DEFAULT_PR0P_EXECUTABLES:
        candidate = install_root / name
        if candidate.is_file() and os.access(candidate, os.X_OK):
            return candidate
    if not install_root.exists():
        return None
    for candidate in sorted(install_root.iterdir()):
        lower = candidate.name.lower()
        if candidate.is_file() and os.access(candidate, os.X_OK) and "pr0p" in lower:
            return candidate
    return None


def list_pr0p_processes(pattern: str = PROCESS_PATTERN) -> list[str]:
    proc = subprocess.run(
        ["pgrep", "-af", pattern],
        check=False,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        timeout=2.0,
    )
    if proc.returncode not in (0, 1):
        return ["PGREP_ERROR: %s" % (proc.stderr.strip() or proc.stdout.strip())]
    return [line for line in proc.stdout.splitlines() if line.strip()]


def cleanup_process(process: Any, *, timeout: float) -> bool:
    if process is None:
        return True
    if process.poll() is not None:
        return True
    try:
        process.terminate()
        process.wait(timeout=timeout)
        return process.poll() is not None
    except subprocess.TimeoutExpired:
        process.kill()
        try:
            process.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            return False
        return process.poll() is not None


def new_process_lines(before: list[str], after: list[str]) -> list[str]:
    before_set = set(before)
    return [line for line in after if line not in before_set]


def run_live_response_with_adapter(
    *,
    adapter: Any,
    run_id: str,
    window_title: str,
    axis: str,
    magnitude: float,
    image_axis: str,
    expected_sign: int,
    fps: float,
    pre_duration_s: float,
    post_duration_s: float,
    min_shift_px: float,
    max_shift_px: float,
    arm_first: bool = False,
    arm_wait_s: float = 12.0,
    arm_settle_s: float = 2.0,
    arm_throttle: float = -0.35,
    hover_wait_s: float = 2.0,
    ws_host: str = DEFAULT_WS_HOST,
    ws_port: int = DEFAULT_WS_PORT,
    ws_path: str = "/",
    timeout: float = 1.0,
    max_frames: int = 8,
    measure: str = "visual",
    attitude_baseline_samples: int = 3,
    attitude_during_samples: int = 6,
    attitude_interval_s: float = 0.15,
    attitude_settle_s: float = 0.2,
    attitude_min_delta_deg: float = 10.0,
    attitude_max_delta_deg: float = 1080.0,
    status_reader: Callable[..., Any] = run_msp_ws_status_once,
    sleeper: Callable[[float], None] = time.sleep,
) -> dict[str, Any]:
    if measure == "attitude":
        arm_metrics = None
        if arm_first:
            armed, arm_metrics, arm_notes = arm_and_hover_for_response(
                adapter,
                ws_host=ws_host,
                ws_port=ws_port,
                ws_path=ws_path,
                timeout=timeout,
                max_frames=max_frames,
                arm_wait_s=arm_wait_s,
                arm_settle_s=arm_settle_s,
                arm_throttle=arm_throttle,
                hover_wait_s=hover_wait_s,
                status_reader=status_reader,
                sleeper=sleeper,
            )
            if not armed:
                return {
                    "status": WAITING,
                    "summary": "arm-first response could not arm the FC before the pulse",
                    "metrics": {
                        "run_id": run_id,
                        "axis": axis,
                        "measure": "attitude",
                        "arm_first": arm_metrics,
                        "real_input_sent": True,
                        "persistent_uinput": True,
                        "adapter": adapter.__class__.__name__,
                    },
                    "notes": arm_notes,
                }
        result = measure_uinput_attitude_response(
            adapter,
            axis=axis,
            pulse_command=response_pulse_command(
                axis, magnitude, arm_first=arm_first, arm_throttle=arm_throttle
            ),
            release_command=None,
            host=ws_host,
            port=ws_port,
            path=ws_path,
            timeout=timeout,
            max_frames=max_frames,
            baseline_samples=attitude_baseline_samples,
            during_samples=attitude_during_samples,
            interval_s=attitude_interval_s,
            settle_s=attitude_settle_s,
            expected_sign=expected_sign,
            min_delta_deg=attitude_min_delta_deg,
            max_delta_deg=attitude_max_delta_deg,
            sleeper=sleeper,
        )
        result.metrics["run_id"] = run_id
        result.metrics["measure"] = "attitude"
        result.metrics["real_input_sent"] = True
        result.metrics["persistent_uinput"] = True
        result.metrics["adapter"] = adapter.__class__.__name__
        if arm_metrics is not None:
            result.metrics["arm_first"] = arm_metrics
            result.notes.append("ARM_FIRST_ARMED")
        return result.as_dict()
    source, selection_metrics, early = make_source_from_selection(
        region=None,
        crop=None,
        window_titles=[window_title],
        exact=False,
        case_sensitive=False,
        min_width=160,
        min_height=120,
        preferred_size=None,
        allow_common_non_fpv_windows=False,
        backend="auto",
        fps=fps,
    )
    if early is not None:
        early.metrics["run_id"] = run_id
        early.metrics["real_input_sent"] = False
        early.metrics["persistent_uinput"] = True
        return early.as_dict()
    assert source is not None
    arm_metrics: dict[str, Any] | None = None
    if arm_first:
        armed, arm_metrics, arm_notes = arm_and_hover_for_response(
            adapter,
            ws_host=ws_host,
            ws_port=ws_port,
            ws_path=ws_path,
            timeout=timeout,
            max_frames=max_frames,
            arm_wait_s=arm_wait_s,
            arm_settle_s=arm_settle_s,
            arm_throttle=arm_throttle,
            hover_wait_s=hover_wait_s,
            status_reader=status_reader,
            sleeper=sleeper,
        )
        if not armed:
            return {
                "status": WAITING,
                "summary": "arm-first response could not arm the FC before the pulse",
                "metrics": {
                    "run_id": run_id,
                    "axis": axis,
                    "arm_first": arm_metrics,
                    "real_input_sent": True,
                    "persistent_uinput": True,
                    "adapter": adapter.__class__.__name__,
                    **selection_metrics,
                },
                "notes": arm_notes,
            }
    result = measure_visual_response(
        source,
        adapter,
        command=response_pulse_command(
            axis, magnitude, arm_first=arm_first, arm_throttle=arm_throttle
        ),
        axis=axis,
        image_axis=image_axis,
        expected_sign=expected_sign,
        fps=fps,
        pre_duration_s=pre_duration_s,
        post_duration_s=post_duration_s,
        min_shift_px=min_shift_px,
        max_shift_px=max_shift_px,
    )
    result.metrics.update(selection_metrics)
    result.metrics["run_id"] = run_id
    result.metrics["real_input_sent"] = True
    result.metrics["persistent_uinput"] = True
    result.metrics["adapter"] = adapter.__class__.__name__
    if arm_metrics is not None:
        result.metrics["arm_first"] = arm_metrics
        result.notes.append("ARM_FIRST_ARMED")
    return result.as_dict()


def run_rc_effect_with_adapter(
    *,
    adapter: Any,
    run_id: str,
    ws_host: str,
    ws_port: int,
    ws_path: str,
    axis: str,
    magnitude: float,
    expected_channel: str,
    expected_direction: str,
    timeout: float,
    max_frames: int,
    baseline_samples: int,
    during_samples: int,
    after_samples: int,
    interval_s: float,
    settle_s: float,
    min_delta: int,
    max_baseline_delta: int,
    throttle_low_threshold: int,
) -> dict[str, Any]:
    try:
        result = measure_uinput_rc_effect(
            adapter,
            command=rc_command_for_axis(axis, magnitude),
            host=ws_host,
            port=ws_port,
            path=ws_path,
            timeout=timeout,
            max_frames=max_frames,
            baseline_samples=baseline_samples,
            during_samples=during_samples,
            after_samples=after_samples,
            interval_s=interval_s,
            settle_s=settle_s,
            expected_channel=expected_channel,
            expected_direction=expected_direction,
            min_delta=min_delta,
            max_baseline_delta=max_baseline_delta,
            throttle_low_threshold=throttle_low_threshold,
        )
    except Exception as exc:
        return {
            "status": WAITING,
            "summary": "persistent uinput MSP_RC effect could not be measured yet",
            "metrics": {
                "run_id": run_id,
                "error": str(exc),
                "real_input_sent": False,
                "persistent_uinput": True,
            },
            "notes": ["UINPUT_RC_EFFECT_MEASURE_WAITING"],
        }
    result.metrics["run_id"] = run_id
    result.metrics["real_input_sent"] = True
    result.metrics["persistent_uinput"] = True
    result.metrics["adapter"] = adapter.__class__.__name__
    return result.as_dict()


def run_status_effect_with_adapter(
    *,
    adapter: Any,
    run_id: str,
    ws_host: str,
    ws_port: int,
    ws_path: str,
    axis: str,
    magnitude: float,
    blocker: str,
    require_all_clear: bool,
    timeout: float,
    max_frames: int,
    baseline_samples: int,
    during_samples: int,
    after_samples: int,
    interval_s: float,
    settle_s: float,
) -> dict[str, Any]:
    try:
        result = measure_uinput_status_effect(
            adapter,
            command=rc_command_for_axis(axis, magnitude),
            host=ws_host,
            port=ws_port,
            path=ws_path,
            timeout=timeout,
            max_frames=max_frames,
            baseline_samples=baseline_samples,
            during_samples=during_samples,
            after_samples=after_samples,
            interval_s=interval_s,
            settle_s=settle_s,
            blocker=blocker,
            require_all_clear=require_all_clear,
        )
    except Exception as exc:
        return {
            "status": WAITING,
            "summary": "persistent uinput FC status effect could not be measured yet",
            "metrics": {
                "run_id": run_id,
                "error": str(exc),
                "real_input_sent": False,
                "persistent_uinput": True,
            },
            "notes": ["UINPUT_STATUS_EFFECT_MEASURE_WAITING"],
        }
    result.metrics["run_id"] = run_id
    result.metrics["real_input_sent"] = True
    result.metrics["persistent_uinput"] = True
    result.metrics["adapter"] = adapter.__class__.__name__
    return result.as_dict()


def run_arm_button_effect_with_adapter(
    *,
    adapter: Any,
    run_id: str,
    ws_host: str,
    ws_port: int,
    ws_path: str,
    buttons: list[str],
    throttle_magnitude: float,
    timeout: float,
    max_frames: int,
    baseline_samples: int,
    during_samples: int,
    after_samples: int,
    interval_s: float,
    settle_s: float,
    button_hold_s: float,
) -> dict[str, Any]:
    try:
        result = measure_uinput_arm_effect(
            adapter,
            buttons=buttons,
            throttle_magnitude=throttle_magnitude,
            host=ws_host,
            port=ws_port,
            path=ws_path,
            timeout=timeout,
            max_frames=max_frames,
            baseline_samples=baseline_samples,
            during_samples=during_samples,
            after_samples=after_samples,
            interval_s=interval_s,
            settle_s=settle_s,
            button_hold_s=button_hold_s,
        )
    except Exception as exc:
        return {
            "status": WAITING,
            "summary": "persistent uinput ARM button effect could not be measured yet",
            "metrics": {
                "run_id": run_id,
                "error": str(exc),
                "real_input_sent": False,
                "persistent_uinput": True,
            },
            "notes": ["UINPUT_ARM_EFFECT_MEASURE_WAITING"],
        }
    result.metrics["run_id"] = run_id
    result.metrics["real_input_sent"] = True
    result.metrics["persistent_uinput"] = True
    result.metrics["adapter"] = adapter.__class__.__name__
    return result.as_dict()


def run_aux_arm_status_with_adapter(
    *,
    adapter: Any,
    run_id: str,
    ws_host: str,
    ws_port: int,
    ws_path: str,
    throttle_magnitude: float,
    aux1_magnitude: float,
    require_all_armed: bool,
    timeout: float,
    max_frames: int,
    baseline_samples: int,
    during_samples: int,
    after_samples: int,
    interval_s: float,
    settle_s: float,
) -> dict[str, Any]:
    try:
        result = measure_uinput_aux_arm_status(
            adapter,
            throttle_magnitude=throttle_magnitude,
            aux1_magnitude=aux1_magnitude,
            host=ws_host,
            port=ws_port,
            path=ws_path,
            timeout=timeout,
            max_frames=max_frames,
            baseline_samples=baseline_samples,
            during_samples=during_samples,
            after_samples=after_samples,
            interval_s=interval_s,
            settle_s=settle_s,
            require_all_armed=require_all_armed,
        )
    except Exception as exc:
        return {
            "status": WAITING,
            "summary": "persistent uinput AUX1 ARM status could not be measured yet",
            "metrics": {
                "run_id": run_id,
                "error": str(exc),
                "real_input_sent": False,
                "persistent_uinput": True,
            },
            "notes": ["UINPUT_AUX_ARM_STATUS_MEASURE_WAITING"],
        }
    result.metrics["run_id"] = run_id
    result.metrics["real_input_sent"] = True
    result.metrics["persistent_uinput"] = True
    result.metrics["adapter"] = adapter.__class__.__name__
    return result.as_dict()


def run_live_session(
    *,
    run_id: str,
    log_dir: Path,
    install_root: Path,
    input_config: Path,
    expected_device: str,
    launch_pr0p: bool,
    send_ui: bool,
    run_suite_after_launch: bool,
    startup_wait_s: float,
    cleanup_timeout_s: float,
    ws_host: str,
    ws_port: int,
    window_title: str,
    timeout: float,
    hold_uinput: bool = False,
    run_live_response_after_launch: bool = False,
    response_axis: str = "yaw",
    response_magnitude: float = 0.05,
    response_image_axis: str = "x",
    response_expected_sign: int = 0,
    response_fps: float = 15.0,
    response_pre_duration_s: float = 0.5,
    response_post_duration_s: float = 1.0,
    response_min_shift_px: float = 2.0,
    response_max_shift_px: float = 200.0,
    response_arm_first: bool = False,
    response_arm_wait_s: float = 12.0,
    response_arm_settle_s: float = 2.0,
    response_arm_throttle: float = -0.35,
    response_hover_wait_s: float = 2.0,
    response_measure: str = "visual",
    response_attitude_baseline_samples: int = 3,
    response_attitude_during_samples: int = 6,
    response_attitude_interval_s: float = 0.15,
    response_attitude_settle_s: float = 0.2,
    response_attitude_min_delta_deg: float = 10.0,
    response_attitude_max_delta_deg: float = 1080.0,
    tracking_bbox: tuple[int, int, int, int] | None = None,
    run_rc_effect_after_launch: bool = False,
    rc_effect_axis: str = "throttle",
    rc_effect_magnitude: float = 1.0,
    rc_effect_expected_channel: str = "throttle",
    rc_effect_expected_direction: str = "lower",
    rc_effect_max_frames: int = 8,
    rc_effect_baseline_samples: int = 3,
    rc_effect_during_samples: int = 3,
    rc_effect_after_samples: int = 2,
    rc_effect_interval_s: float = 0.1,
    rc_effect_settle_s: float = 0.2,
    rc_effect_min_delta: int = 25,
    rc_effect_max_baseline_delta: int = 4,
    rc_effect_throttle_low_threshold: int = 1050,
    run_status_effect_after_launch: bool = False,
    status_effect_axis: str = "throttle",
    status_effect_magnitude: float = 1.0,
    status_effect_blocker: str = "THROTTLE",
    status_effect_require_all_clear: bool = True,
    status_effect_max_frames: int = 8,
    status_effect_baseline_samples: int = 2,
    status_effect_during_samples: int = 4,
    status_effect_after_samples: int = 2,
    status_effect_interval_s: float = 0.1,
    status_effect_settle_s: float = 0.2,
    run_arm_button_effect_after_launch: bool = False,
    arm_buttons: list[str] | None = None,
    arm_throttle_magnitude: float = 1.0,
    arm_effect_max_frames: int = 8,
    arm_effect_baseline_samples: int = 2,
    arm_effect_during_samples: int = 3,
    arm_effect_after_samples: int = 2,
    arm_effect_interval_s: float = 0.1,
    arm_effect_settle_s: float = 0.2,
    arm_effect_button_hold_s: float = 0.2,
    run_aux_arm_status_after_launch: bool = False,
    aux_arm_throttle_magnitude: float = 1.0,
    aux_arm_aux1_magnitude: float = 1.0,
    aux_arm_require_all_armed: bool = False,
    aux_arm_max_frames: int = 8,
    aux_arm_baseline_samples: int = 2,
    aux_arm_during_samples: int = 4,
    aux_arm_after_samples: int = 2,
    aux_arm_interval_s: float = 0.1,
    aux_arm_settle_s: float = 0.2,
    popen_factory: Callable[..., Any] = subprocess.Popen,
    process_lister: Callable[[], list[str]] = list_pr0p_processes,
    ui_runner: Callable[..., Any] = run_ui_smoke,
    suite_runner: Callable[..., Any] = run_suite,
    uinput_adapter_factory: Callable[..., Any] = UInputAdapter,
    uinput_probe: Callable[[], Any] = probe_uinput_environment,
    live_response_runner: Callable[..., dict[str, Any]] = run_live_response_with_adapter,
    rc_effect_runner: Callable[..., dict[str, Any]] = run_rc_effect_with_adapter,
    status_effect_runner: Callable[..., dict[str, Any]] = run_status_effect_with_adapter,
    arm_button_effect_runner: Callable[..., dict[str, Any]] = run_arm_button_effect_with_adapter,
    aux_arm_status_runner: Callable[..., dict[str, Any]] = run_aux_arm_status_with_adapter,
) -> LiveSessionResult:
    log_dir.mkdir(parents=True, exist_ok=True)
    executable = find_pr0p_executable(install_root)
    before_processes = process_lister()
    input_config_result = run_input_config_probe(
        input_config=input_config,
        expected_device=expected_device,
        require_virtual_mapping=True,
        allow_generic_uinput_profile=True,
    )

    metrics: dict[str, Any] = {
        "run_id": run_id,
        "install_root": str(install_root),
        "executable": str(executable) if executable else None,
        "input_config": input_config_result.as_dict(),
        "processes_before": before_processes,
        "real_launch": False,
        "real_ui_input_sent": False,
        "suite_report_json": None,
        "suite_report_md": None,
        "suite_status": None,
        "persistent_uinput_requested": hold_uinput,
        "persistent_uinput_active_at_launch": False,
        "persistent_uinput_probe": None,
        "live_response": None,
        "live_rc_effect": None,
        "live_status_effect": None,
        "live_arm_effect": None,
        "live_aux_arm_status": None,
        "cleanup_performed": False,
        "cleanup_ok": True,
    }
    notes: list[str] = []

    if executable is None:
        metrics["processes_after"] = process_lister()
        return LiveSessionResult(
            status=WAITING,
            summary="pr0p executable is not installed in the isolated root",
            metrics=metrics,
            notes=["PR0P_EXECUTABLE_MISSING"],
        )

    if not launch_pr0p:
        metrics["planned_command"] = [str(executable)]
        metrics["processes_after"] = process_lister()
        status = PASS if input_config_result.status == PASS else WAITING
        notes.append("DRY_RUN_ONLY")
        if input_config_result.status != PASS:
            notes.append("PR0P_INPUT_MAPPING_NOT_READY")
        return LiveSessionResult(
            status=status,
            summary="bounded pr0p live-session plan generated without launching pr0p",
            metrics=metrics,
            notes=notes,
        )

    stdout_path = log_dir / ("%s-pr0p-stdout.log" % run_id)
    stderr_path = log_dir / ("%s-pr0p-stderr.log" % run_id)
    process = None
    adapter = None
    process_status = PASS
    try:
        if hold_uinput:
            probe = uinput_probe()
            metrics["persistent_uinput_probe"] = (
                probe.as_dict() if hasattr(probe, "as_dict") else probe
            )
            if not getattr(probe, "available", False):
                metrics["processes_after"] = process_lister()
                return LiveSessionResult(
                    status=WAITING,
                    summary="persistent uinput was requested, but /dev/uinput is not ready",
                    metrics=metrics,
                    notes=["UINPUT_PERMISSION_FAIL"],
                )
            adapter = uinput_adapter_factory(name=expected_device)
            adapter.neutral()
            metrics["persistent_uinput_active_at_launch"] = True

        with stdout_path.open("wb") as stdout, stderr_path.open("wb") as stderr:
            process = popen_factory(
                [str(executable)],
                cwd=str(install_root),
                stdout=stdout,
                stderr=stderr,
                start_new_session=True,
            )
            metrics["real_launch"] = True
            metrics["pid"] = getattr(process, "pid", None)
            metrics["stdout_log"] = str(stdout_path)
            metrics["stderr_log"] = str(stderr_path)
            time.sleep(startup_wait_s)
            if process.poll() is not None:
                process_status = FAIL
                notes.append("PR0P_EXITED_DURING_STARTUP")

            if process_status == PASS and send_ui:
                ui_result = ui_runner(
                    window_title=window_title,
                    timeout=timeout,
                    send_ui=True,
                )
                metrics["ui_smoke"] = ui_result.as_dict()
                metrics["real_ui_input_sent"] = True

            if process_status == PASS and run_suite_after_launch:
                suite = suite_runner(
                    run_id="%s-suite" % run_id,
                    log_dir=log_dir,
                    install_root=install_root,
                    ws_host=ws_host,
                    ws_port=ws_port,
                    timeout=timeout,
                    window_titles=[window_title],
                    capture_duration_s=0.5,
                    capture_fps=5.0,
                    capture_min_fps=1.0,
                    response_axis="yaw",
                    response_magnitude=0.05,
                    response_image_axis="x",
                    response_expected_sign=0,
                    tracking_bbox=tracking_bbox,
                )
                suite_json, suite_md = write_suite_reports(suite, log_dir)
                metrics["suite_status"] = suite.status
                metrics["suite_report_json"] = str(suite_json)
                metrics["suite_report_md"] = str(suite_md)

            if process_status == PASS and run_live_response_after_launch:
                if adapter is None:
                    process_status = WAITING
                    notes.append("PERSISTENT_UINPUT_REQUIRED_FOR_LIVE_RESPONSE")
                else:
                    metrics["live_response"] = live_response_runner(
                        adapter=adapter,
                        run_id="%s-live-response" % run_id,
                        window_title=window_title,
                        axis=response_axis,
                        magnitude=response_magnitude,
                        image_axis=response_image_axis,
                        expected_sign=response_expected_sign,
                        fps=response_fps,
                        pre_duration_s=response_pre_duration_s,
                        post_duration_s=response_post_duration_s,
                        min_shift_px=response_min_shift_px,
                        max_shift_px=response_max_shift_px,
                        arm_first=response_arm_first,
                        arm_wait_s=response_arm_wait_s,
                        arm_settle_s=response_arm_settle_s,
                        arm_throttle=response_arm_throttle,
                        hover_wait_s=response_hover_wait_s,
                        ws_host=ws_host,
                        ws_port=ws_port,
                        ws_path="/",
                        timeout=timeout,
                        measure=response_measure,
                        attitude_baseline_samples=response_attitude_baseline_samples,
                        attitude_during_samples=response_attitude_during_samples,
                        attitude_interval_s=response_attitude_interval_s,
                        attitude_settle_s=response_attitude_settle_s,
                        attitude_min_delta_deg=response_attitude_min_delta_deg,
                        attitude_max_delta_deg=response_attitude_max_delta_deg,
                    )

            if process_status == PASS and run_rc_effect_after_launch:
                if adapter is None:
                    process_status = WAITING
                    notes.append("PERSISTENT_UINPUT_REQUIRED_FOR_RC_EFFECT")
                else:
                    metrics["live_rc_effect"] = rc_effect_runner(
                        adapter=adapter,
                        run_id="%s-rc-effect" % run_id,
                        ws_host=ws_host,
                        ws_port=ws_port,
                        ws_path="/",
                        axis=rc_effect_axis,
                        magnitude=rc_effect_magnitude,
                        expected_channel=rc_effect_expected_channel,
                        expected_direction=rc_effect_expected_direction,
                        timeout=timeout,
                        max_frames=rc_effect_max_frames,
                        baseline_samples=rc_effect_baseline_samples,
                        during_samples=rc_effect_during_samples,
                        after_samples=rc_effect_after_samples,
                        interval_s=rc_effect_interval_s,
                        settle_s=rc_effect_settle_s,
                        min_delta=rc_effect_min_delta,
                        max_baseline_delta=rc_effect_max_baseline_delta,
                        throttle_low_threshold=rc_effect_throttle_low_threshold,
                    )

            if process_status == PASS and run_status_effect_after_launch:
                if adapter is None:
                    process_status = WAITING
                    notes.append("PERSISTENT_UINPUT_REQUIRED_FOR_STATUS_EFFECT")
                else:
                    metrics["live_status_effect"] = status_effect_runner(
                        adapter=adapter,
                        run_id="%s-status-effect" % run_id,
                        ws_host=ws_host,
                        ws_port=ws_port,
                        ws_path="/",
                        axis=status_effect_axis,
                        magnitude=status_effect_magnitude,
                        blocker=status_effect_blocker,
                        require_all_clear=status_effect_require_all_clear,
                        timeout=timeout,
                        max_frames=status_effect_max_frames,
                        baseline_samples=status_effect_baseline_samples,
                        during_samples=status_effect_during_samples,
                        after_samples=status_effect_after_samples,
                        interval_s=status_effect_interval_s,
                        settle_s=status_effect_settle_s,
                    )

            if process_status == PASS and run_arm_button_effect_after_launch:
                if adapter is None:
                    process_status = WAITING
                    notes.append("PERSISTENT_UINPUT_REQUIRED_FOR_ARM_EFFECT")
                else:
                    metrics["live_arm_effect"] = arm_button_effect_runner(
                        adapter=adapter,
                        run_id="%s-arm-effect" % run_id,
                        ws_host=ws_host,
                        ws_port=ws_port,
                        ws_path="/",
                        buttons=arm_buttons or ["south", "east"],
                        throttle_magnitude=arm_throttle_magnitude,
                        timeout=timeout,
                        max_frames=arm_effect_max_frames,
                        baseline_samples=arm_effect_baseline_samples,
                        during_samples=arm_effect_during_samples,
                        after_samples=arm_effect_after_samples,
                        interval_s=arm_effect_interval_s,
                        settle_s=arm_effect_settle_s,
                        button_hold_s=arm_effect_button_hold_s,
                    )

            if process_status == PASS and run_aux_arm_status_after_launch:
                if adapter is None:
                    process_status = WAITING
                    notes.append("PERSISTENT_UINPUT_REQUIRED_FOR_AUX_ARM_STATUS")
                else:
                    metrics["live_aux_arm_status"] = aux_arm_status_runner(
                        adapter=adapter,
                        run_id="%s-aux-arm-status" % run_id,
                        ws_host=ws_host,
                        ws_port=ws_port,
                        ws_path="/",
                        throttle_magnitude=aux_arm_throttle_magnitude,
                        aux1_magnitude=aux_arm_aux1_magnitude,
                        require_all_armed=aux_arm_require_all_armed,
                        timeout=timeout,
                        max_frames=aux_arm_max_frames,
                        baseline_samples=aux_arm_baseline_samples,
                        during_samples=aux_arm_during_samples,
                        after_samples=aux_arm_after_samples,
                        interval_s=aux_arm_interval_s,
                        settle_s=aux_arm_settle_s,
                    )
    except OSError as exc:
        process_status = FAIL
        metrics["launch_error"] = str(exc)
        notes.append("PR0P_LAUNCH_FAIL")
    except UInputUnavailable as exc:
        process_status = WAITING
        metrics["uinput_error"] = str(exc)
        notes.append("UINPUT_PERMISSION_FAIL")
    finally:
        metrics["cleanup_performed"] = process is not None
        metrics["cleanup_ok"] = cleanup_process(process, timeout=cleanup_timeout_s)
        if adapter is not None:
            try:
                adapter.close()
            except Exception as exc:
                metrics["persistent_uinput_close_error"] = str(exc)
                process_status = FAIL
                notes.append("PERSISTENT_UINPUT_CLOSE_FAIL")
        after_processes = process_lister()
        metrics["processes_after"] = after_processes
        lingering = new_process_lines(before_processes, after_processes)
        metrics["new_processes_after_cleanup"] = lingering
        if not metrics["cleanup_ok"] or lingering:
            process_status = FAIL
            notes.append("PROCESS_CLEANUP_FAIL")

    statuses = [process_status, input_config_result.status]
    if metrics.get("suite_status"):
        statuses.append(str(metrics["suite_status"]))
    if metrics.get("ui_smoke"):
        statuses.append(str(metrics["ui_smoke"]["status"]))
    if metrics.get("live_response"):
        statuses.append(str(metrics["live_response"]["status"]))
    if metrics.get("live_rc_effect"):
        statuses.append(str(metrics["live_rc_effect"]["status"]))
    if metrics.get("live_status_effect"):
        statuses.append(str(metrics["live_status_effect"]["status"]))
    if metrics.get("live_arm_effect"):
        statuses.append(str(metrics["live_arm_effect"]["status"]))
    if metrics.get("live_aux_arm_status"):
        statuses.append(str(metrics["live_aux_arm_status"]["status"]))
    status = combine_status(statuses)
    if input_config_result.status != PASS:
        notes.append("PR0P_INPUT_MAPPING_NOT_READY")
    summary = (
        "bounded pr0p live session completed and cleaned up"
        if status == PASS else
        "bounded pr0p live session needs more live evidence"
        if status == WAITING else
        "bounded pr0p live session failed"
    )
    return LiveSessionResult(status=status, summary=summary, metrics=metrics, notes=notes)


def build_markdown(result: LiveSessionResult, *, run_id: str) -> str:
    lines = [
        "# SimITL / pr0p Live Session Runner",
        "",
        "Run: `%s`" % run_id,
        "Verdict: `%s`" % result.status,
        "",
        result.summary,
        "",
        "| Metric | Value |",
        "| --- | --- |",
    ]
    for key in (
        "real_launch",
        "real_ui_input_sent",
        "cleanup_performed",
        "cleanup_ok",
        "suite_status",
    ):
        if key in result.metrics:
            lines.append("| %s | `%s` |" % (key, result.metrics[key]))
    lines.extend([
        "",
        "## Metrics",
        "",
        "```json",
        json.dumps(result.metrics, indent=2, sort_keys=True),
        "```",
    ])
    for note in result.notes:
        lines.append("- %s" % note)
    lines.append("")
    return "\n".join(lines)


def write_reports(result: LiveSessionResult, log_dir: Path, *, run_id: str) -> tuple[Path, Path]:
    log_dir.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime("%Y%m%d-%H%M%S", time.localtime())
    json_path = log_dir / ("%s-%s-live-session.json" % (stamp, run_id))
    md_path = log_dir / ("%s-%s-live-session.md" % (stamp, run_id))
    json_path.write_text(
        json.dumps(result.as_dict(), indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    md_path.write_text(build_markdown(result, run_id=run_id), encoding="utf-8")
    return json_path, md_path


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-id", default="pr0p-live-session")
    parser.add_argument("--log-dir", type=Path, default=DEFAULT_LOG_DIR)
    parser.add_argument("--install-root", type=Path, default=DEFAULT_INSTALL_ROOT)
    parser.add_argument("--input-config", type=Path, default=DEFAULT_INPUT_CONFIG)
    parser.add_argument("--expected-device", default=DEFAULT_EXPECTED_DEVICE)
    parser.add_argument("--launch-pr0p", action="store_true",
                        help="Start the pr0p binary from the isolated install root")
    parser.add_argument("--ack-live-launch", action="store_true",
                        help="Required before --launch-pr0p can execute pr0p")
    parser.add_argument("--send-ui", action="store_true",
                        help="Send real XTEST clicks to enter the local race path")
    parser.add_argument("--ack-live-ui", action="store_true",
                        help="Required before --send-ui can send desktop input")
    parser.add_argument("--run-suite", action="store_true",
                        help="Run the safe probe suite while the launched pr0p process is alive")
    parser.add_argument("--hold-uinput", action="store_true",
                        help="Create the virtual RC device before launching pr0p and keep it open")
    parser.add_argument("--ack-live-input", action="store_true",
                        help="Required before --hold-uinput or --run-live-response can touch uinput")
    parser.add_argument("--run-live-response", action="store_true",
                        help="Measure a live visual response with the persistent virtual RC device")
    parser.add_argument("--run-rc-effect", action="store_true",
                        help="Measure whether persistent uinput changes MSP_RC")
    parser.add_argument("--run-status-effect", action="store_true",
                        help="Measure whether persistent uinput clears an FC arming blocker")
    parser.add_argument("--run-arm-button-effect", action="store_true",
                        help="Measure whether persistent uinput buttons activate ARM mode/status")
    parser.add_argument("--run-aux-arm-status", action="store_true",
                        help="Measure FC ARM status while throttle-low and AUX1-high are held")
    parser.add_argument("--response-axis", default="yaw", choices=("yaw", "pitch", "roll", "throttle"))
    parser.add_argument("--response-magnitude", type=float, default=0.05)
    parser.add_argument("--response-image-axis", default="x", choices=("x", "y"))
    parser.add_argument("--response-expected-sign", type=int, choices=(-1, 0, 1), default=0)
    parser.add_argument("--response-fps", type=float, default=15.0)
    parser.add_argument("--response-pre-duration", type=float, default=0.5)
    parser.add_argument("--response-post-duration", type=float, default=1.0)
    parser.add_argument("--response-min-shift-px", type=float, default=2.0)
    parser.add_argument("--response-max-shift-px", type=float, default=200.0)
    parser.add_argument("--response-arm-first", action="store_true",
                        help="Arm via throttle-low then AUX1-high and hold hover throttle "
                             "before sending the response pulse")
    parser.add_argument("--response-arm-wait", type=float, default=12.0,
                        help="Seconds to hold throttle-low before raising AUX1 so "
                             "boot-grace/calibration blockers can clear")
    parser.add_argument("--response-arm-settle", type=float, default=2.0)
    parser.add_argument("--response-arm-throttle", type=float, default=-0.35,
                        help="Throttle axis value held during the pulse; +1.0 is low, "
                             "negative values are above mid")
    parser.add_argument("--response-hover-wait", type=float, default=2.0)
    parser.add_argument("--response-measure", default="visual", choices=("visual", "attitude"),
                        help="visual measures capture pixel shift; attitude reads the "
                             "signed MSP_ATTITUDE delta (render-independent)")
    parser.add_argument("--response-attitude-baseline-samples", type=int, default=3)
    parser.add_argument("--response-attitude-during-samples", type=int, default=6)
    parser.add_argument("--response-attitude-interval", type=float, default=0.15)
    parser.add_argument("--response-attitude-settle", type=float, default=0.2)
    parser.add_argument("--response-attitude-min-delta", type=float, default=10.0)
    parser.add_argument("--response-attitude-max-delta", type=float, default=1080.0)
    parser.add_argument("--rc-effect-axis", default="throttle", choices=("yaw", "pitch", "roll", "throttle", "aux1"))
    parser.add_argument("--rc-effect-magnitude", type=float, default=1.0)
    parser.add_argument("--rc-effect-expected-channel", default="throttle",
                        choices=("any", "roll", "pitch", "yaw", "throttle", "aux1"))
    parser.add_argument("--rc-effect-expected-direction", default="lower",
                        choices=("any", "higher", "lower"))
    parser.add_argument("--rc-effect-max-frames", type=int, default=8)
    parser.add_argument("--rc-effect-baseline-samples", type=int, default=3)
    parser.add_argument("--rc-effect-during-samples", type=int, default=3)
    parser.add_argument("--rc-effect-after-samples", type=int, default=2)
    parser.add_argument("--rc-effect-interval", type=float, default=0.1)
    parser.add_argument("--rc-effect-settle", type=float, default=0.2)
    parser.add_argument("--rc-effect-min-delta", type=int, default=25)
    parser.add_argument("--rc-effect-max-baseline-delta", type=int, default=4)
    parser.add_argument("--rc-effect-throttle-low-threshold", type=int, default=1050)
    parser.add_argument("--status-effect-axis", default="throttle", choices=("yaw", "pitch", "roll", "throttle"))
    parser.add_argument("--status-effect-magnitude", type=float, default=1.0)
    parser.add_argument("--status-effect-blocker", default="THROTTLE")
    parser.add_argument("--status-effect-last-sample-clear", action="store_true")
    parser.add_argument("--status-effect-max-frames", type=int, default=8)
    parser.add_argument("--status-effect-baseline-samples", type=int, default=2)
    parser.add_argument("--status-effect-during-samples", type=int, default=4)
    parser.add_argument("--status-effect-after-samples", type=int, default=2)
    parser.add_argument("--status-effect-interval", type=float, default=0.1)
    parser.add_argument("--status-effect-settle", type=float, default=0.2)
    parser.add_argument("--arm-button", action="append", choices=("south", "east"),
                        help="Candidate ARM button to test. Repeat to test multiple buttons.")
    parser.add_argument("--arm-throttle-magnitude", type=float, default=1.0)
    parser.add_argument("--arm-effect-max-frames", type=int, default=8)
    parser.add_argument("--arm-effect-baseline-samples", type=int, default=2)
    parser.add_argument("--arm-effect-during-samples", type=int, default=3)
    parser.add_argument("--arm-effect-after-samples", type=int, default=2)
    parser.add_argument("--arm-effect-interval", type=float, default=0.1)
    parser.add_argument("--arm-effect-settle", type=float, default=0.2)
    parser.add_argument("--arm-effect-button-hold", type=float, default=0.2)
    parser.add_argument("--aux-arm-throttle-magnitude", type=float, default=1.0)
    parser.add_argument("--aux-arm-aux1-magnitude", type=float, default=1.0)
    parser.add_argument("--aux-arm-require-all-armed", action="store_true")
    parser.add_argument("--aux-arm-max-frames", type=int, default=8)
    parser.add_argument("--aux-arm-baseline-samples", type=int, default=2)
    parser.add_argument("--aux-arm-during-samples", type=int, default=4)
    parser.add_argument("--aux-arm-after-samples", type=int, default=2)
    parser.add_argument("--aux-arm-interval", type=float, default=0.1)
    parser.add_argument("--aux-arm-settle", type=float, default=0.2)
    parser.add_argument("--tracking-bbox", type=parse_bbox,
                        help="Optional bbox forwarded to the safe suite when --run-suite is used")
    parser.add_argument("--startup-wait", type=float, default=3.0)
    parser.add_argument("--cleanup-timeout", type=float, default=3.0)
    parser.add_argument("--timeout", type=float, default=1.0)
    parser.add_argument("--ws-host", default=DEFAULT_WS_HOST)
    parser.add_argument("--ws-port", type=int, default=DEFAULT_WS_PORT)
    parser.add_argument("--window-title", default=DEFAULT_WINDOW_TITLES[0])
    args = parser.parse_args(argv)
    if args.launch_pr0p and not args.ack_live_launch:
        parser.error("--launch-pr0p requires --ack-live-launch")
    if args.send_ui and not args.ack_live_ui:
        parser.error("--send-ui requires --ack-live-ui")
    if args.send_ui and not args.launch_pr0p:
        parser.error("--send-ui requires --launch-pr0p")
    if args.run_suite and not args.launch_pr0p:
        parser.error("--run-suite requires --launch-pr0p")
    if args.hold_uinput and not args.ack_live_input:
        parser.error("--hold-uinput requires --ack-live-input")
    if args.hold_uinput and not args.launch_pr0p:
        parser.error("--hold-uinput requires --launch-pr0p")
    if args.run_live_response and not args.ack_live_input:
        parser.error("--run-live-response requires --ack-live-input")
    if args.run_live_response and not args.hold_uinput:
        parser.error("--run-live-response requires --hold-uinput")
    if args.run_live_response and not args.launch_pr0p:
        parser.error("--run-live-response requires --launch-pr0p")
    if args.run_rc_effect and not args.ack_live_input:
        parser.error("--run-rc-effect requires --ack-live-input")
    if args.run_rc_effect and not args.hold_uinput:
        parser.error("--run-rc-effect requires --hold-uinput")
    if args.run_rc_effect and not args.launch_pr0p:
        parser.error("--run-rc-effect requires --launch-pr0p")
    if args.run_status_effect and not args.ack_live_input:
        parser.error("--run-status-effect requires --ack-live-input")
    if args.run_status_effect and not args.hold_uinput:
        parser.error("--run-status-effect requires --hold-uinput")
    if args.run_status_effect and not args.launch_pr0p:
        parser.error("--run-status-effect requires --launch-pr0p")
    if args.run_arm_button_effect and not args.ack_live_input:
        parser.error("--run-arm-button-effect requires --ack-live-input")
    if args.run_arm_button_effect and not args.hold_uinput:
        parser.error("--run-arm-button-effect requires --hold-uinput")
    if args.run_arm_button_effect and not args.launch_pr0p:
        parser.error("--run-arm-button-effect requires --launch-pr0p")
    if args.run_aux_arm_status and not args.ack_live_input:
        parser.error("--run-aux-arm-status requires --ack-live-input")
    if args.run_aux_arm_status and not args.hold_uinput:
        parser.error("--run-aux-arm-status requires --hold-uinput")
    if args.run_aux_arm_status and not args.launch_pr0p:
        parser.error("--run-aux-arm-status requires --launch-pr0p")
    if args.startup_wait < 0:
        parser.error("--startup-wait must be non-negative")
    if args.cleanup_timeout <= 0:
        parser.error("--cleanup-timeout must be positive")
    if args.timeout <= 0:
        parser.error("--timeout must be positive")
    if args.response_fps <= 0:
        parser.error("--response-fps must be positive")
    if args.response_pre_duration <= 0 or args.response_post_duration <= 0:
        parser.error("--response-pre-duration/--response-post-duration must be positive")
    if args.response_min_shift_px < 0:
        parser.error("--response-min-shift-px must be non-negative")
    if args.response_max_shift_px <= 0:
        parser.error("--response-max-shift-px must be positive")
    if args.response_min_shift_px > args.response_max_shift_px:
        parser.error("--response-min-shift-px must be <= --response-max-shift-px")
    if args.response_arm_first and not args.run_live_response:
        parser.error("--response-arm-first requires --run-live-response")
    if args.response_arm_wait < 0 or args.response_arm_settle < 0 or args.response_hover_wait < 0:
        parser.error("--response-arm-wait/--response-arm-settle/--response-hover-wait must be non-negative")
    if not -1.0 <= args.response_arm_throttle <= 1.0:
        parser.error("--response-arm-throttle must be within [-1.0, 1.0]")
    if args.response_measure == "attitude" and args.response_axis == "throttle":
        parser.error("--response-measure attitude supports yaw/pitch/roll axes only")
    if args.response_attitude_baseline_samples <= 0 or args.response_attitude_during_samples <= 0:
        parser.error("--response-attitude sample counts must be positive")
    if args.response_attitude_interval < 0 or args.response_attitude_settle < 0:
        parser.error("--response-attitude-interval/--response-attitude-settle must be non-negative")
    if args.response_attitude_min_delta <= 0 or args.response_attitude_max_delta <= 0:
        parser.error("--response-attitude-min-delta/--response-attitude-max-delta must be positive")
    if args.response_attitude_min_delta > args.response_attitude_max_delta:
        parser.error("--response-attitude-min-delta must be <= --response-attitude-max-delta")
    if args.ws_port <= 0:
        parser.error("--ws-port must be positive")
    if args.rc_effect_max_frames <= 0:
        parser.error("--rc-effect-max-frames must be positive")
    if args.rc_effect_baseline_samples <= 0 or args.rc_effect_during_samples <= 0 or args.rc_effect_after_samples <= 0:
        parser.error("--rc-effect sample counts must be positive")
    if args.rc_effect_interval < 0 or args.rc_effect_settle < 0:
        parser.error("--rc-effect-interval/--rc-effect-settle must be non-negative")
    if args.rc_effect_min_delta <= 0 or args.rc_effect_max_baseline_delta < 0:
        parser.error("--rc-effect-min-delta must be positive and --rc-effect-max-baseline-delta non-negative")
    if args.rc_effect_throttle_low_threshold <= 0:
        parser.error("--rc-effect-throttle-low-threshold must be positive")
    if args.status_effect_max_frames <= 0:
        parser.error("--status-effect-max-frames must be positive")
    if args.status_effect_baseline_samples <= 0 or args.status_effect_during_samples <= 0 or args.status_effect_after_samples <= 0:
        parser.error("--status-effect sample counts must be positive")
    if args.status_effect_interval < 0 or args.status_effect_settle < 0:
        parser.error("--status-effect-interval/--status-effect-settle must be non-negative")
    if args.arm_effect_max_frames <= 0:
        parser.error("--arm-effect-max-frames must be positive")
    if args.arm_effect_baseline_samples <= 0 or args.arm_effect_during_samples <= 0 or args.arm_effect_after_samples <= 0:
        parser.error("--arm-effect sample counts must be positive")
    if args.arm_effect_interval < 0 or args.arm_effect_settle < 0 or args.arm_effect_button_hold < 0:
        parser.error("--arm-effect-interval/--arm-effect-settle/--arm-effect-button-hold must be non-negative")
    if args.aux_arm_max_frames <= 0:
        parser.error("--aux-arm-max-frames must be positive")
    if args.aux_arm_baseline_samples <= 0 or args.aux_arm_during_samples <= 0 or args.aux_arm_after_samples <= 0:
        parser.error("--aux-arm sample counts must be positive")
    if args.aux_arm_interval < 0 or args.aux_arm_settle < 0:
        parser.error("--aux-arm-interval/--aux-arm-settle must be non-negative")
    return args


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    result = run_live_session(
        run_id=args.run_id,
        log_dir=args.log_dir,
        install_root=args.install_root,
        input_config=args.input_config,
        expected_device=args.expected_device,
        launch_pr0p=args.launch_pr0p,
        send_ui=args.send_ui,
        run_suite_after_launch=args.run_suite,
        startup_wait_s=args.startup_wait,
        cleanup_timeout_s=args.cleanup_timeout,
        ws_host=args.ws_host,
        ws_port=args.ws_port,
        window_title=args.window_title,
        timeout=args.timeout,
        hold_uinput=args.hold_uinput,
        run_live_response_after_launch=args.run_live_response,
        response_axis=args.response_axis,
        response_magnitude=args.response_magnitude,
        response_image_axis=args.response_image_axis,
        response_expected_sign=args.response_expected_sign,
        response_fps=args.response_fps,
        response_pre_duration_s=args.response_pre_duration,
        response_post_duration_s=args.response_post_duration,
        response_min_shift_px=args.response_min_shift_px,
        response_max_shift_px=args.response_max_shift_px,
        response_arm_first=args.response_arm_first,
        response_arm_wait_s=args.response_arm_wait,
        response_arm_settle_s=args.response_arm_settle,
        response_arm_throttle=args.response_arm_throttle,
        response_hover_wait_s=args.response_hover_wait,
        response_measure=args.response_measure,
        response_attitude_baseline_samples=args.response_attitude_baseline_samples,
        response_attitude_during_samples=args.response_attitude_during_samples,
        response_attitude_interval_s=args.response_attitude_interval,
        response_attitude_settle_s=args.response_attitude_settle,
        response_attitude_min_delta_deg=args.response_attitude_min_delta,
        response_attitude_max_delta_deg=args.response_attitude_max_delta,
        tracking_bbox=args.tracking_bbox,
        run_rc_effect_after_launch=args.run_rc_effect,
        rc_effect_axis=args.rc_effect_axis,
        rc_effect_magnitude=args.rc_effect_magnitude,
        rc_effect_expected_channel=args.rc_effect_expected_channel,
        rc_effect_expected_direction=args.rc_effect_expected_direction,
        rc_effect_max_frames=args.rc_effect_max_frames,
        rc_effect_baseline_samples=args.rc_effect_baseline_samples,
        rc_effect_during_samples=args.rc_effect_during_samples,
        rc_effect_after_samples=args.rc_effect_after_samples,
        rc_effect_interval_s=args.rc_effect_interval,
        rc_effect_settle_s=args.rc_effect_settle,
        rc_effect_min_delta=args.rc_effect_min_delta,
        rc_effect_max_baseline_delta=args.rc_effect_max_baseline_delta,
        rc_effect_throttle_low_threshold=args.rc_effect_throttle_low_threshold,
        run_status_effect_after_launch=args.run_status_effect,
        status_effect_axis=args.status_effect_axis,
        status_effect_magnitude=args.status_effect_magnitude,
        status_effect_blocker=args.status_effect_blocker,
        status_effect_require_all_clear=not args.status_effect_last_sample_clear,
        status_effect_max_frames=args.status_effect_max_frames,
        status_effect_baseline_samples=args.status_effect_baseline_samples,
        status_effect_during_samples=args.status_effect_during_samples,
        status_effect_after_samples=args.status_effect_after_samples,
        status_effect_interval_s=args.status_effect_interval,
        status_effect_settle_s=args.status_effect_settle,
        run_arm_button_effect_after_launch=args.run_arm_button_effect,
        arm_buttons=args.arm_button,
        arm_throttle_magnitude=args.arm_throttle_magnitude,
        arm_effect_max_frames=args.arm_effect_max_frames,
        arm_effect_baseline_samples=args.arm_effect_baseline_samples,
        arm_effect_during_samples=args.arm_effect_during_samples,
        arm_effect_after_samples=args.arm_effect_after_samples,
        arm_effect_interval_s=args.arm_effect_interval,
        arm_effect_settle_s=args.arm_effect_settle,
        arm_effect_button_hold_s=args.arm_effect_button_hold,
        run_aux_arm_status_after_launch=args.run_aux_arm_status,
        aux_arm_throttle_magnitude=args.aux_arm_throttle_magnitude,
        aux_arm_aux1_magnitude=args.aux_arm_aux1_magnitude,
        aux_arm_require_all_armed=args.aux_arm_require_all_armed,
        aux_arm_max_frames=args.aux_arm_max_frames,
        aux_arm_baseline_samples=args.aux_arm_baseline_samples,
        aux_arm_during_samples=args.aux_arm_during_samples,
        aux_arm_after_samples=args.aux_arm_after_samples,
        aux_arm_interval_s=args.aux_arm_interval,
        aux_arm_settle_s=args.aux_arm_settle,
    )
    json_path, md_path = write_reports(result, args.log_dir, run_id=args.run_id)
    print("simitl-pr0p-live-session %s report=%s summary=%s" % (
        result.status,
        json_path,
        md_path,
    ))
    print(result.summary)
    return 1 if result.status == FAIL else 0


if __name__ == "__main__":
    raise SystemExit(main())
