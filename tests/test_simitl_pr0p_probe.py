import json
import os
import socket
import struct
import subprocess
import sys
import threading
import time
from pathlib import Path

import numpy as np
import pytest


ROOT = Path(__file__).resolve().parents[1]
PROBE = ROOT / "experiments" / "simitl_pr0p_probe"
if str(PROBE) not in sys.path:
    sys.path.insert(0, str(PROBE))

import pr0p_core_next_runner as pr0p_core_next_module  # noqa: E402

from msp_ws_probe import (  # noqa: E402
    PASS,
    build_handshake,
    expected_accept,
    parse_headers,
    probe_websocket,
)
from msp_ws_semantic_probe import (  # noqa: E402
    MSP_API_VERSION,
    MSPErrorFrame,
    encode_ws_frame,
    msp_checksum,
    msp_encode,
    parse_api_version,
    parse_msp_frame,
    read_ws_frame,
    run_msp_ws_semantic_probe,
)
from msp_ws_status_probe import (  # noqa: E402
    MSP_BOXIDS,
    MSP_BOXNAMES,
    MSP_STATUS,
    MSP_STATUS_EX,
    parse_args as parse_status_args,
    parse_status_basic,
    run_msp_ws_status_probe,
)
from msp_ws_mode_ranges_probe import (  # noqa: E402
    mode_range_dict,
    parse_args as parse_mode_ranges_args,
    parse_mode_ranges,
    run_msp_ws_mode_ranges_probe,
)
from msp_uinput_rc_effect_probe import (  # noqa: E402
    evaluate_rc_effect,
    measure_uinput_rc_effect,
    parse_args as parse_uinput_rc_effect_args,
)
from msp_uinput_status_probe import (  # noqa: E402
    evaluate_status_effect,
    measure_uinput_status_effect,
    parse_args as parse_uinput_status_args,
)
from msp_uinput_arm_probe import (  # noqa: E402
    evaluate_arm_effect,
    measure_uinput_arm_effect,
    parse_args as parse_uinput_arm_args,
)
from msp_uinput_aux_arm_status_probe import (  # noqa: E402
    evaluate_aux_arm_status,
    measure_uinput_aux_arm_status,
    parse_args as parse_uinput_aux_arm_status_args,
)
from msp_ws_rc_probe import (  # noqa: E402
    DEFAULT_CHANNELS,
    encode_rc_payload,
    parse_args as parse_rc_args,
    parse_rc_payload,
    rc_channel_deltas,
    rc_prefix_matches,
    rc_readback_mismatches,
    run_msp_ws_rc_probe,
)
from kenet.rc_channels import pilot_to_msp_rc_channels  # noqa: E402
from preflight_probe import (  # noqa: E402
    DEFAULT_WS_PORT,
    PreflightReport,
    ProbeCheck,
    build_markdown,
    probe_install_root,
)
from pr0p_capture_probe import (  # noqa: E402
    FAIL,
    WAITING,
    analyze_capture_source,
    apply_relative_crop,
    is_excluded_window,
    run_capture_probe,
)
from pr0p_virtual_input_probe import (  # noqa: E402
    command_from_args,
    dry_run_smoke,
    parse_args as parse_input_args,
    run_input_probe,
)
from pr0p_response_probe import (  # noqa: E402
    command_for_axis,
    evaluate_direction,
    frame_shift,
    measure_visual_response,
    parse_args as parse_response_args,
    run_response_probe,
)
from pr0p_runtime_input_visual_probe import (  # noqa: E402
    evaluate_runtime_visual_change,
    frame_diff_metrics,
    measure_runtime_input_visual_change,
    parse_args as parse_runtime_input_visual_args,
    run_runtime_input_visual_probe,
)
from pr0p_probe_suite import (  # noqa: E402
    SuiteReport,
    SuiteStep,
    build_markdown as build_suite_markdown,
    suite_status,
)
from pr0p_tracking_probe import (  # noqa: E402
    bbox_inside_region,
    evaluate_loop_summary,
    load_bbox_file,
    parse_args as parse_tracking_args,
    run_tracking_probe,
)
from pr0p_tracking_log_check import (  # noqa: E402
    latest_tracking_log,
    run_tracking_log_check,
)
from pr0p_bbox_tool import (  # noqa: E402
    analyze_bbox_source,
    run_bbox_probe,
)
from pr0p_live_run_manifest import (  # noqa: E402
    build_manifest,
    build_markdown as build_manifest_markdown,
    latest_suite_report,
    load_suite_report,
    parse_args as parse_live_manifest_args,
)
from pr0p_live_session_runner import (  # noqa: E402
    arm_and_hover_for_response,
    find_pr0p_executable,
    parse_args as parse_live_session_args,
    response_pulse_command,
    run_live_session,
)
from pr0p_decision_report import (  # noqa: E402
    PROMOTE_CANDIDATE,
    REJECT,
    choose_evidence,
    evaluate_decision,
    evidence_source_kind,
    latest_manifest_report,
    parse_args as parse_decision_args,
)
from independent_sim_readiness import (  # noqa: E402
    GAME_SCREEN_READY,
    GAME_SCREEN_SCOPE,
    GAME_SCREEN_SIMPLE_WINDOW_REQUIRED_PHASES,
    GAME_SCREEN_SYNTHETIC_REQUIRED_PHASES,
    INDEPENDENT_SIM_READY,
    build_markdown as build_readiness_markdown,
    build_readiness,
    latest_game_screen_decision,
    parse_args as parse_readiness_args,
)
from pr0p_aux1_acceptance_runner import (  # noqa: E402
    build_markdown as build_aux1_acceptance_markdown,
    parse_args as parse_aux1_acceptance_args,
    run_aux1_acceptance,
)
from pr0p_response_acceptance_runner import (  # noqa: E402
    build_markdown as build_response_acceptance_markdown,
    live_response_command,
    parse_args as parse_response_acceptance_args,
    run_response_acceptance,
)
from pr0p_tracking_acceptance_runner import (  # noqa: E402
    build_markdown as build_tracking_acceptance_markdown,
    parse_args as parse_tracking_acceptance_args,
    run_tracking_acceptance,
)
from pr0p_acceptance_chain_runner import (  # noqa: E402
    build_markdown as build_acceptance_chain_markdown,
    parse_args as parse_acceptance_chain_args,
    run_acceptance_chain,
)
from pr0p_acceptance_state_report import (  # noqa: E402
    STALE,
    build_acceptance_state,
    build_markdown as build_acceptance_state_markdown,
)
from pr0p_core_goal_report import (  # noqa: E402
    build_core_goal_report,
    build_markdown as build_core_goal_markdown,
    parse_args as parse_core_goal_args,
)
from pr0p_core_next_runner import (  # noqa: E402
    build_markdown as build_core_next_markdown,
    is_live_command,
    parse_args as parse_core_next_args,
    run_core_next,
)
from pr0p_approach_pitch_plan import (  # noqa: E402
    build_markdown as build_approach_pitch_markdown,
    build_plan_report as build_approach_pitch_plan,
    parse_args as parse_approach_pitch_args,
)
from pr0p_handoff_plan import (  # noqa: E402
    build_markdown as build_handoff_markdown,
    build_plan_report as build_handoff_plan,
    parse_args as parse_handoff_args,
)
from pr0p_handoff_acceptance_runner import (  # noqa: E402
    build_handoff_acceptance,
    build_markdown as build_handoff_acceptance_markdown,
    parse_args as parse_handoff_acceptance_args,
)
from pr0p_moving_target_yaw_plan import (  # noqa: E402
    build_markdown as build_moving_target_yaw_markdown,
    build_plan_report as build_moving_target_yaw_plan,
    parse_args as parse_moving_target_yaw_args,
)
from pr0p_moving_target_acceptance_runner import (  # noqa: E402
    build_markdown as build_moving_target_acceptance_markdown,
    build_moving_target_acceptance,
    parse_args as parse_moving_target_acceptance_args,
)
from pr0p_rc_manual_flight_runner import (  # noqa: E402
    build_markdown as build_rc_manual_flight_markdown,
    manual_response_command,
    parse_args as parse_rc_manual_flight_args,
    run_manual_flight_gate,
)
from pr0p_install_probe import (  # noqa: E402
    discover_linux_updater,
    is_allowed_install_root,
    run_install_probe,
    scan_install_root,
)
from pr0p_client_probe import (  # noqa: E402
    find_client_candidates,
    run_client_probe,
)
from pr0p_updater_runner import (  # noqa: E402
    find_updater,
    parse_args as parse_updater_args,
    run_updater,
)
from pr0p_isolation_check import (  # noqa: E402
    DEFAULT_ROOTS as DEFAULT_ISOLATION_ROOTS,
    run_isolation_check,
)
from pr0p_input_config_probe import (  # noqa: E402
    build_markdown as build_input_config_markdown,
    parse_binding_path,
    parse_input_bindings,
    run_input_config_probe,
)
from pr0p_input_config_patch import (  # noqa: E402
    parse_args as parse_input_config_patch_args,
    patch_input_config_data,
    run_input_config_patch,
    target_paths_by_role,
)
from pr0p_input_config_restore import (  # noqa: E402
    list_backups,
    parse_args as parse_input_config_restore_args,
    run_input_config_restore,
)
from pr0p_rc_channel_mapping_assistant import (  # noqa: E402
    build_pulse_events,
    command_for_role as mapping_command_for_role,
    expand_roles,
    parse_args as parse_mapping_assistant_args,
    run_mapping_assistant,
)
from pr0p_ui_smoke import (  # noqa: E402
    WindowGeometry,
    parse_args as parse_ui_args,
    parse_xwininfo_geometry,
    planned_clicks,
)
from capture_window import CaptureFrame, SyntheticFrameSource  # noqa: E402
from virtual_input import DryRunInputAdapter, normalize_button_name  # noqa: E402
from x11_window import X11Window  # noqa: E402
from kenet.tracker import TrackResult  # noqa: E402


class BlankFrameSource:
    def __init__(self, *, width=64, height=48, fps=10.0):
        self.width = width
        self.height = height
        self.fps = fps

    def frames(self, duration_s=None):
        total = int((duration_s or 1.0) * self.fps)
        start = 1000.0
        for index in range(total):
            yield CaptureFrame(
                frame=np.zeros((self.height, self.width, 3), dtype=np.uint8),
                timestamp=start + index / self.fps,
                index=index,
                source="blank",
            )


class ShiftAfterPreSource:
    def __init__(self, *, dx=8, dy=0, width=96, height=72, fps=10.0, pre_frames=3):
        self.dx = dx
        self.dy = dy
        self.width = width
        self.height = height
        self.fps = fps
        self.pre_frames = pre_frames

    def frames(self, duration_s=None):
        total = int((duration_s or 1.0) * self.fps)
        start = 2000.0
        base = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        base[20:44, 28:54] = (20, 220, 240)
        base[26:38, 34:48] = (255, 255, 255)
        shifted = np.zeros_like(base)
        y1 = max(0, 20 + self.dy)
        y2 = min(self.height, 44 + self.dy)
        x1 = max(0, 28 + self.dx)
        x2 = min(self.width, 54 + self.dx)
        if y2 > y1 and x2 > x1:
            shifted[y1:y2, x1:x2] = (20, 220, 240)
            shifted[max(0, 26 + self.dy):min(self.height, 38 + self.dy),
                    max(0, 34 + self.dx):min(self.width, 48 + self.dx)] = (255, 255, 255)
        for index in range(total):
            frame = base.copy() if index < self.pre_frames else shifted.copy()
            yield CaptureFrame(
                frame=frame,
                timestamp=start + index / self.fps,
                index=index,
                source="shift-after-pre",
            )


class StableTracker:
    def __init__(self, *_args, **_kwargs):
        self.initialized = False

    @property
    def is_initialized(self):
        return self.initialized

    def init(self, _frame, _bbox):
        self.initialized = True

    def update(self, _frame):
        return TrackResult(found=True, bbox=(42, 28, 24, 24), center=(54.0, 40.0))


def msp_response_frame(code: int, payload: bytes) -> bytes:
    return b"$M>" + bytes([len(payload), code]) + payload + bytes([msp_checksum(code, payload)])


def start_fake_msp_ws_server(response_payload: bytes):
    listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    listener.bind(("127.0.0.1", 0))
    listener.listen(1)
    listener.settimeout(5.0)
    port = listener.getsockname()[1]
    received = {}

    def serve():
        try:
            conn, _addr = listener.accept()
            with conn:
                conn.settimeout(5.0)
                request = bytearray()
                while b"\r\n\r\n" not in request:
                    request.extend(conn.recv(4096))
                text = request.decode("iso-8859-1")
                key = ""
                for line in text.split("\r\n"):
                    if line.lower().startswith("sec-websocket-key:"):
                        key = line.split(":", 1)[1].strip()
                response = (
                    "HTTP/1.1 101 Switching Protocols\r\n"
                    "Upgrade: websocket\r\n"
                    "Connection: Upgrade\r\n"
                    "Sec-WebSocket-Accept: %s\r\n"
                    "\r\n"
                ) % expected_accept(key)
                conn.sendall(response.encode("ascii"))
                _opcode, payload = read_ws_frame(conn)
                received["request_payload"] = payload
                conn.sendall(encode_ws_frame(response_payload, opcode=2, mask_key=b"\x00\x00\x00\x00"))
        finally:
            listener.close()

    thread = threading.Thread(target=serve, daemon=True)
    thread.start()
    return port, received, thread


def test_websocket_handshake_helpers_parse_status_and_accept():
    key = "dGhlIHNhbXBsZSBub25jZQ=="
    request = build_handshake("127.0.0.1", 5761, "/", key).decode("ascii")
    assert "Upgrade: websocket" in request
    assert expected_accept(key) == "s3pPLMBiTxaQ9kYGzzhZRbK+xOo="

    status, headers = parse_headers(
        b"HTTP/1.1 101 Switching Protocols\r\n"
        b"Upgrade: websocket\r\n"
        b"Connection: Upgrade\r\n"
        b"Sec-WebSocket-Accept: abc\r\n\r\n"
    )
    assert status == 101
    assert headers["upgrade"] == "websocket"
    assert headers["sec-websocket-accept"] == "abc"


def test_websocket_probe_passes_against_fake_connection(monkeypatch):
    class FakeSocket:
        def __init__(self):
            self.request = b""

        def __enter__(self):
            return self

        def __exit__(self, *_args):
            return None

        def settimeout(self, _timeout):
            return None

        def sendall(self, request):
            self.request = request

        def recv(self, _size):
            text = self.request.decode("iso-8859-1")
            key = ""
            for line in text.split("\r\n"):
                if line.lower().startswith("sec-websocket-key:"):
                    key = line.split(":", 1)[1].strip()
            response = (
                "HTTP/1.1 101 Switching Protocols\r\n"
                "Upgrade: websocket\r\n"
                "Connection: Upgrade\r\n"
                "Sec-WebSocket-Accept: %s\r\n"
                "\r\n"
            ) % expected_accept(key)
            return response.encode("ascii")

    monkeypatch.setattr("msp_ws_probe.socket.create_connection", lambda *_args, **_kwargs: FakeSocket())
    result = probe_websocket("127.0.0.1", 5761, "/", timeout=1.0)
    assert result.status == PASS
    assert result.metrics["status_code"] == 101
    assert result.metrics["accept_matches"] is True


def test_msp_ws_semantic_helpers_round_trip_api_version():
    request = msp_encode(MSP_API_VERSION)
    assert request.startswith(b"$M<")
    response = msp_response_frame(MSP_API_VERSION, bytes([0, 1, 45]))
    code, payload = parse_msp_frame(b"noise" + response, expected_code=MSP_API_VERSION)
    assert code == MSP_API_VERSION
    assert parse_api_version(payload) == {
        "protocol_version": 0,
        "api_major": 1,
        "api_minor": 45,
    }


def test_msp_ws_semantic_probe_passes_against_fake_server():
    response = msp_response_frame(MSP_API_VERSION, bytes([0, 2, 0]))
    port, received, thread = start_fake_msp_ws_server(response)
    result = run_msp_ws_semantic_probe(
        host="127.0.0.1",
        port=port,
        path="/",
        timeout=1.0,
        max_frames=2,
    )
    thread.join(timeout=2.0)
    assert result.status == PASS
    assert received["request_payload"] == msp_encode(MSP_API_VERSION)
    assert result.metrics["api_version"] == {
        "protocol_version": 0,
        "api_major": 2,
        "api_minor": 0,
    }


def status_ex_payload(first_flags=0, extra=b"", arming_flags=0):
    payload = bytearray()
    payload += struct.pack("<HHH", 250, 0, 0)
    payload += struct.pack("<I", first_flags)
    payload += bytes([0])
    payload += struct.pack("<H", 12)
    payload += bytes([3, 0])
    payload += bytes([len(extra)])
    payload += extra
    payload += bytes([29])
    payload += struct.pack("<I", arming_flags)
    payload += bytes([0, 0, 0, 0])
    return bytes(payload)


def status_basic_payload(first_flags=0):
    return struct.pack("<HHHI", 250, 0, 0, first_flags) + bytes([0])


def test_msp_ws_status_probe_reports_arm_and_modes(monkeypatch):
    def fake_request(**kwargs):
        code = kwargs["code"]
        if code == MSP_BOXNAMES:
            return b"ARM;ANGLE;HORIZON;", {"frames_read": 1}
        if code == MSP_BOXIDS:
            return bytes([0, 1, 2]), {"frames_read": 1}
        if code == MSP_STATUS_EX:
            return status_ex_payload(first_flags=0b011), {"frames_read": 1}
        raise AssertionError("unexpected MSP code %s" % code)

    monkeypatch.setattr("msp_ws_status_probe.request_msp_over_websocket", fake_request)
    result = run_msp_ws_status_probe(
        host="127.0.0.1",
        port=5761,
        path="/",
        timeout=1.0,
        max_frames=2,
    )
    assert result.status == PASS
    assert result.metrics["real_msp_write_sent"] is False
    assert result.metrics["status_source"] == "MSP_STATUS_EX"
    assert result.metrics["fc_status"]["armed"] is True
    assert result.metrics["fc_status"]["active_modes"] == ["ARM", "ANGLE"]
    assert "FC_ARMED" in result.notes


def test_msp_ws_status_probe_surfaces_not_armed_and_disable_flags(monkeypatch):
    def fake_request(**kwargs):
        code = kwargs["code"]
        if code == MSP_BOXNAMES:
            return b"ARM;ANGLE;", {"frames_read": 1}
        if code == MSP_BOXIDS:
            return bytes([0, 1]), {"frames_read": 1}
        if code == MSP_STATUS_EX:
            return status_ex_payload(first_flags=0, arming_flags=1 << 7), {"frames_read": 1}
        raise AssertionError("unexpected MSP code %s" % code)

    monkeypatch.setattr("msp_ws_status_probe.request_msp_over_websocket", fake_request)
    result = run_msp_ws_status_probe(
        host="127.0.0.1",
        port=5761,
        path="/",
        timeout=1.0,
        max_frames=2,
    )
    assert result.status == PASS
    assert result.metrics["fc_status"]["armed"] is False
    assert result.metrics["fc_status"]["arming_disable_names"] == ["THROTTLE"]
    assert "FC_NOT_ARMED" in result.notes
    assert "ARMING_DISABLED:THROTTLE" in result.notes


def test_msp_ws_status_probe_falls_back_to_basic_status(monkeypatch):
    def fake_request(**kwargs):
        code = kwargs["code"]
        if code == MSP_BOXNAMES:
            return b"ARM;ANGLE;", {"frames_read": 1}
        if code == MSP_BOXIDS:
            return bytes([0, 1]), {"frames_read": 1}
        if code == MSP_STATUS_EX:
            raise MSPErrorFrame("unsupported")
        if code == MSP_STATUS:
            return status_basic_payload(first_flags=0b1), {"frames_read": 1}
        raise AssertionError("unexpected MSP code %s" % code)

    monkeypatch.setattr("msp_ws_status_probe.request_msp_over_websocket", fake_request)
    result = run_msp_ws_status_probe(
        host="127.0.0.1",
        port=5761,
        path="/",
        timeout=1.0,
        max_frames=2,
    )
    assert result.status == PASS
    assert result.metrics["status_source"] == "MSP_STATUS"
    assert result.metrics["fc_status"]["armed"] is True
    assert "MSP_STATUS_EX_UNAVAILABLE" in result.notes


def test_msp_ws_status_probe_monitor_collects_multiple_samples(monkeypatch):
    status_payloads = [
        status_ex_payload(first_flags=0, arming_flags=(1 << 7) | (1 << 9) | (1 << 12)),
        status_ex_payload(first_flags=0, arming_flags=1 << 7),
    ]

    def fake_request(**kwargs):
        code = kwargs["code"]
        if code == MSP_BOXNAMES:
            return b"ARM;ANGLE;", {"frames_read": 1}
        if code == MSP_BOXIDS:
            return bytes([0, 1]), {"frames_read": 1}
        if code == MSP_STATUS_EX:
            return status_payloads.pop(0), {"frames_read": 1}
        raise AssertionError("unexpected MSP code %s" % code)

    monkeypatch.setattr("msp_ws_status_probe.request_msp_over_websocket", fake_request)
    result = run_msp_ws_status_probe(
        host="127.0.0.1",
        port=5761,
        path="/",
        timeout=1.0,
        max_frames=2,
        samples=2,
        interval_s=0.0,
    )
    summary = result.metrics["monitor_summary"]
    assert result.status == PASS
    assert summary["sample_count"] == 2
    assert summary["pass_count"] == 2
    assert summary["latest_fc_status"]["arming_disable_names"] == ["THROTTLE"]
    assert summary["arming_disable_names_seen"] == ["BOOTGRACE", "CALIB", "THROTTLE"]
    assert "THROTTLE_NOT_LOW_FOR_ARM" in result.notes


def test_msp_ws_status_basic_parser_handles_missing_box_metadata():
    status = parse_status_basic(status_basic_payload(first_flags=0b1))
    assert status["valid"] is True
    assert status["armed"] is None
    assert status["active_modes_valid"] is False


def test_msp_ws_status_probe_rejects_bad_sample_args():
    with pytest.raises(SystemExit):
        parse_status_args(["--samples", "0"])
    with pytest.raises(SystemExit):
        parse_status_args(["--interval", "-1"])


def test_msp_ws_mode_ranges_parse_arm_aux_channel():
    ranges = parse_mode_ranges(bytes([0, 0, 28, 48]))
    assert len(ranges) == 1
    row = mode_range_dict(ranges[0], box_names=["ARM"], box_ids=[0])
    assert row["mode_name"] == "ARM"
    assert row["channel_label"] == "AUX1 / CH5"
    assert row["start_us"] == 1600
    assert row["end_us"] == 2100
    assert row["usable"] is True


def test_msp_ws_mode_ranges_probe_reports_arm_range(monkeypatch):
    def fake_request(**kwargs):
        code = kwargs["code"]
        if code == MSP_BOXNAMES:
            return b"ARM;ANGLE;", {"frames_read": 1}
        if code == MSP_BOXIDS:
            return bytes([0, 1]), {"frames_read": 1}
        return bytes([0, 0, 28, 48]), {"frames_read": 1}

    monkeypatch.setattr("msp_ws_mode_ranges_probe.request_msp_over_websocket", fake_request)
    result = run_msp_ws_mode_ranges_probe(
        host="127.0.0.1",
        port=5761,
        path="/",
        timeout=1.0,
        max_frames=8,
    )
    assert result.status == PASS
    assert result.metrics["arm_ranges"][0]["channel_label"] == "AUX1 / CH5"
    assert "ARM_MODE_RANGE:AUX1 / CH5" in result.notes


def test_msp_ws_mode_ranges_probe_rejects_bad_args():
    with pytest.raises(SystemExit):
        parse_mode_ranges_args(["--max-frames", "0"])


def test_uinput_rc_effect_detects_throttle_low():
    status, summary, metrics, notes = evaluate_rc_effect(
        baseline_samples=[[1500, 1500, 1500, 1500], [1500, 1500, 1500, 1500]],
        during_samples=[[1500, 1500, 1500, 1000], [1500, 1500, 1500, 1000]],
        after_samples=[[1500, 1500, 1500, 1500]],
        expected_channel="throttle",
        expected_direction="lower",
        min_delta=25,
        max_baseline_delta=4,
        throttle_low_threshold=1050,
    )
    assert status == PASS
    assert "throttle" in summary
    assert metrics["expected_channel_delta"] == -500
    assert metrics["during_throttle_min"] == 1000
    assert "THROTTLE_LOW_REACHED" in notes


def test_uinput_rc_effect_detects_aux1_high():
    status, summary, metrics, notes = evaluate_rc_effect(
        baseline_samples=[[1500, 1500, 1500, 1000, 1000]],
        during_samples=[[1500, 1500, 1500, 1000, 1900]],
        after_samples=[[1500, 1500, 1500, 1000, 1000]],
        expected_channel="aux1",
        expected_direction="higher",
        min_delta=25,
        max_baseline_delta=4,
        throttle_low_threshold=1050,
    )
    assert status == PASS
    assert "aux1" in summary
    assert metrics["expected_channel_index"] == 4
    assert metrics["expected_channel_delta"] == 900
    assert "EXPECTED_CHANNEL_EFFECT_DETECTED" in notes


def test_uinput_rc_effect_waits_when_expected_channel_unchanged():
    status, summary, metrics, notes = evaluate_rc_effect(
        baseline_samples=[[1500, 1500, 1500, 1500]],
        during_samples=[[1500, 1500, 1700, 1500]],
        after_samples=[[1500, 1500, 1500, 1500]],
        expected_channel="throttle",
        expected_direction="lower",
        min_delta=25,
        max_baseline_delta=4,
        throttle_low_threshold=1050,
    )
    assert status == WAITING
    assert "expected channel" in summary
    assert metrics["changed_channels"][0]["label"] == "yaw"
    assert "EXPECTED_CHANNEL_UNCHANGED" in notes


def test_uinput_rc_effect_fails_wrong_expected_direction():
    status, _summary, metrics, notes = evaluate_rc_effect(
        baseline_samples=[[1500, 1500, 1500, 1500]],
        during_samples=[[1500, 1500, 1500, 1900]],
        after_samples=[[1500, 1500, 1500, 1500]],
        expected_channel="throttle",
        expected_direction="lower",
        min_delta=25,
        max_baseline_delta=4,
        throttle_low_threshold=1050,
    )
    assert status == FAIL
    assert metrics["expected_channel_delta"] == 400
    assert "EXPECTED_CHANNEL_DIRECTION_FAIL" in notes


def test_uinput_rc_effect_measure_neutralizes_after_sampling(monkeypatch):
    calls = []
    sample_sets = [
        [[1500, 1500, 1500, 1500]],
        [[1500, 1500, 1500, 1000]],
        [[1500, 1500, 1500, 1500]],
    ]

    def fake_sample(**kwargs):
        calls.append(kwargs["samples"])
        sample = sample_sets.pop(0)
        return sample, [{"rc": sample[0], "frames_read": 1}]

    monkeypatch.setattr("msp_uinput_rc_effect_probe.sample_msp_rc", fake_sample)
    adapter = DryRunInputAdapter()
    result = measure_uinput_rc_effect(
        adapter,
        command=command_for_axis("throttle", -1.0),
        host="127.0.0.1",
        port=5761,
        path="/",
        timeout=1.0,
        max_frames=2,
        baseline_samples=1,
        during_samples=1,
        after_samples=1,
        interval_s=0.0,
        settle_s=0.0,
        expected_channel="throttle",
        expected_direction="lower",
        min_delta=25,
        max_baseline_delta=4,
        throttle_low_threshold=1050,
    )
    assert result.status == PASS
    assert calls == [1, 1, 1]
    assert adapter.commands[-1][1].is_neutral()


def test_uinput_rc_effect_requires_ack_for_real_input():
    with pytest.raises(SystemExit):
        parse_uinput_rc_effect_args(["--uinput"])


def status_result_with_blockers(blocker_samples):
    return {
        "status": PASS,
        "metrics": {
            "samples": [
                {
                    "status": PASS,
                    "metrics": {
                        "fc_status": {
                            "arming_disable_names": blockers,
                        },
                    },
                }
                for blockers in blocker_samples
            ],
        },
    }


def test_uinput_status_effect_detects_throttle_blocker_clear():
    status, summary, metrics, notes = evaluate_status_effect(
        baseline_status=status_result_with_blockers([["THROTTLE"], ["THROTTLE"]]),
        during_status=status_result_with_blockers([[], []]),
        after_status=status_result_with_blockers([["THROTTLE"]]),
        blocker="THROTTLE",
        require_all_clear=True,
    )
    assert status == PASS
    assert "THROTTLE" in summary
    assert metrics["during_blockers_by_sample"] == [[], []]
    assert "UINPUT_CLEARED_THROTTLE" in notes


def test_uinput_status_effect_waits_when_blocker_remains():
    status, _summary, metrics, notes = evaluate_status_effect(
        baseline_status=status_result_with_blockers([["THROTTLE"]]),
        during_status=status_result_with_blockers([["THROTTLE"], []]),
        after_status=status_result_with_blockers([["THROTTLE"]]),
        blocker="THROTTLE",
        require_all_clear=True,
    )
    assert status == WAITING
    assert metrics["during_blockers_by_sample"] == [["THROTTLE"], []]
    assert "UINPUT_DID_NOT_CLEAR_THROTTLE" in notes


def test_uinput_status_effect_measure_neutralizes(monkeypatch):
    calls = []
    status_sets = [
        status_result_with_blockers([["THROTTLE"]]),
        status_result_with_blockers([[]]),
        status_result_with_blockers([["THROTTLE"]]),
    ]

    def fake_sample(**kwargs):
        calls.append(kwargs["samples"])
        return status_sets.pop(0)

    monkeypatch.setattr("msp_uinput_status_probe.sample_status", fake_sample)
    adapter = DryRunInputAdapter()
    result = measure_uinput_status_effect(
        adapter,
        command=command_for_axis("throttle", 1.0),
        host="127.0.0.1",
        port=5761,
        path="/",
        timeout=1.0,
        max_frames=2,
        baseline_samples=1,
        during_samples=1,
        after_samples=1,
        interval_s=0.0,
        settle_s=0.0,
        blocker="THROTTLE",
        require_all_clear=True,
    )
    assert result.status == PASS
    assert calls == [1, 1, 1]
    assert adapter.commands[-1][1].is_neutral()


def test_uinput_status_probe_requires_ack_for_real_input():
    with pytest.raises(SystemExit):
        parse_uinput_status_args(["--uinput"])


def status_result_with_modes(mode_samples, blocker_samples=None):
    blocker_samples = blocker_samples or [[] for _ in mode_samples]
    return {
        "status": PASS,
        "metrics": {
            "samples": [
                {
                    "status": PASS,
                    "metrics": {
                        "fc_status": {
                            "armed": "ARM" in modes,
                            "active_modes": modes,
                            "arming_disable_names": blockers,
                        },
                    },
                }
                for modes, blockers in zip(mode_samples, blocker_samples)
            ],
        },
    }


def test_uinput_arm_effect_detects_button_candidate():
    status, summary, metrics, notes = evaluate_arm_effect(
        baseline_status=status_result_with_modes([[]], [["THROTTLE"]]),
        candidate_results=[
            {
                "button": "south",
                "status": PASS,
                "arm_active_by_sample": [False],
                "active_modes_by_sample": [[]],
                "blockers_by_sample": [[]],
            },
            {
                "button": "east",
                "status": PASS,
                "arm_active_by_sample": [True],
                "active_modes_by_sample": [["ARM"]],
                "blockers_by_sample": [[]],
            },
        ],
        after_status=status_result_with_modes([[]], [[]]),
    )
    assert status == PASS
    assert "east" in summary
    assert metrics["candidates"][1]["button"] == "east"
    assert "UINPUT_ARM_BUTTON_DETECTED:east" in notes


def test_uinput_arm_effect_waits_without_candidate_effect():
    status, _summary, metrics, notes = evaluate_arm_effect(
        baseline_status=status_result_with_modes([[]], [["THROTTLE"]]),
        candidate_results=[
            {
                "button": "south",
                "status": PASS,
                "arm_active_by_sample": [False],
                "active_modes_by_sample": [[]],
                "blockers_by_sample": [["BOOTGRACE"]],
            },
        ],
        after_status=status_result_with_modes([[]], [[]]),
    )
    assert status == WAITING
    assert metrics["candidates"][0]["button"] == "south"
    assert "NO_UINPUT_ARM_BUTTON_EFFECT" in notes
    assert "ARM_BLOCKERS_DURING_BUTTON:BOOTGRACE" in notes


def test_uinput_arm_effect_measure_releases_button_and_neutralizes(monkeypatch):
    calls = []
    status_sets = [
        status_result_with_modes([[]], [["THROTTLE"]]),
        status_result_with_modes([["ARM"]], [[]]),
        status_result_with_modes([[]], [[]]),
    ]

    def fake_sample(**kwargs):
        calls.append(kwargs["samples"])
        return status_sets.pop(0)

    monkeypatch.setattr("msp_uinput_arm_probe.sample_status", fake_sample)
    adapter = DryRunInputAdapter()
    result = measure_uinput_arm_effect(
        adapter,
        buttons=["south"],
        throttle_magnitude=1.0,
        host="127.0.0.1",
        port=5761,
        path="/",
        timeout=1.0,
        max_frames=2,
        baseline_samples=1,
        during_samples=1,
        after_samples=1,
        interval_s=0.0,
        settle_s=0.0,
        button_hold_s=0.0,
    )
    assert result.status == PASS
    assert calls == [1, 1, 1]
    assert adapter.commands[-1][1].is_neutral()
    assert ("south", True) in [(button, pressed) for _ts, button, pressed in adapter.button_events]
    assert adapter.button_events[-2][1:] == ("south", False)


def test_uinput_arm_probe_requires_ack_for_real_input():
    with pytest.raises(SystemExit):
        parse_uinput_arm_args(["--uinput"])


def test_uinput_aux_arm_status_detects_armed_state():
    status, summary, metrics, notes = evaluate_aux_arm_status(
        baseline_status=status_result_with_modes([[]], [["THROTTLE"]]),
        during_status=status_result_with_modes([["ARM"]], [[]]),
        after_status=status_result_with_modes([[]], [["THROTTLE"]]),
        require_all_armed=False,
    )
    assert status == PASS
    assert "AUX1" in summary
    assert metrics["during_arm_active_by_sample"] == [True]
    assert "UINPUT_AUX1_ARMED" in notes
    assert "UINPUT_CLEARED_THROTTLE" in notes


def test_uinput_aux_arm_status_waits_when_arm_inactive_but_throttle_clear():
    status, _summary, metrics, notes = evaluate_aux_arm_status(
        baseline_status=status_result_with_modes([[]], [["THROTTLE"]]),
        during_status=status_result_with_modes([[]], [[]]),
        after_status=status_result_with_modes([[]], [["THROTTLE"]]),
        require_all_armed=True,
    )
    assert status == WAITING
    assert metrics["during_blockers_by_sample"] == [[]]
    assert "UINPUT_AUX1_DID_NOT_ARM" in notes
    assert "THROTTLE_CLEAR_BUT_ARM_INACTIVE" in notes


def test_uinput_aux_arm_status_measure_neutralizes(monkeypatch):
    calls = []
    status_sets = [
        status_result_with_modes([[]], [["THROTTLE"]]),
        status_result_with_modes([["ARM"]], [[]]),
        status_result_with_modes([[]], [["THROTTLE"]]),
    ]

    def fake_sample(**kwargs):
        calls.append(kwargs["samples"])
        return status_sets.pop(0)

    monkeypatch.setattr("msp_uinput_aux_arm_status_probe.sample_status", fake_sample)
    adapter = DryRunInputAdapter()
    result = measure_uinput_aux_arm_status(
        adapter,
        throttle_magnitude=1.0,
        aux1_magnitude=1.0,
        host="127.0.0.1",
        port=5761,
        path="/",
        timeout=1.0,
        max_frames=2,
        baseline_samples=1,
        during_samples=1,
        after_samples=1,
        interval_s=0.0,
        settle_s=0.0,
        require_all_armed=False,
    )
    assert result.status == PASS
    assert calls == [1, 1, 1]
    assert result.metrics["command"]["throttle"] == 1.0
    assert result.metrics["command"]["aux1"] == 1.0
    assert adapter.commands[-1][1].is_neutral()


def test_uinput_aux_arm_status_probe_requires_ack_for_real_input():
    with pytest.raises(SystemExit):
        parse_uinput_aux_arm_status_args(["--uinput"])


def test_virtual_input_normalizes_and_records_button_events():
    adapter = DryRunInputAdapter()
    assert normalize_button_name("BTN_SOUTH") == "south"
    adapter.press_button("east", True)
    adapter.neutral()
    assert adapter.button_events[0][1:] == ("east", True)
    assert adapter.button_events[-1][1:] == ("east", False)


def test_msp_ws_rc_helpers_payload_and_mapping():
    payload = encode_rc_payload(DEFAULT_CHANNELS)
    assert parse_rc_payload(payload) == DEFAULT_CHANNELS
    assert rc_channel_deltas([[1000, 1500], [1003, 1498], [999, 1502]]) == [4, 4]
    expected = pilot_to_msp_rc_channels(DEFAULT_CHANNELS)
    assert rc_readback_mismatches(DEFAULT_CHANNELS, expected) == []
    broken = list(expected)
    broken[2], broken[3] = broken[3], broken[2]
    assert rc_readback_mismatches(DEFAULT_CHANNELS, broken)
    assert rc_prefix_matches(expected + [1500], expected + [1000], len(expected))


def test_msp_ws_rc_probe_read_only_baseline_passes(monkeypatch):
    readback = pilot_to_msp_rc_channels(DEFAULT_CHANNELS)

    def fake_request(**_kwargs):
        return encode_rc_payload(readback), {"frames_read": 1, "bytes_read": len(readback) * 2}

    monkeypatch.setattr("msp_ws_rc_probe.request_msp_over_websocket", fake_request)
    result = run_msp_ws_rc_probe(
        host="127.0.0.1",
        port=5761,
        path="/",
        timeout=1.0,
        channels=DEFAULT_CHANNELS,
        write=False,
        settle_s=0.0,
        max_frames=2,
        baseline_samples=3,
        baseline_interval_s=0.0,
        max_baseline_delta=4,
        allow_unstable_baseline=False,
    )
    assert result.status == PASS
    assert result.metrics["real_msp_write_sent"] is False
    assert result.metrics["baseline_deltas"] == [0] * len(DEFAULT_CHANNELS)


def test_msp_ws_rc_probe_waits_on_moving_baseline(monkeypatch):
    samples = [
        pilot_to_msp_rc_channels(DEFAULT_CHANNELS),
        pilot_to_msp_rc_channels([1500, 1620, 1120, 1380, 1000, 1000, 1000, 1000]),
    ]

    def fake_request(**_kwargs):
        sample = samples.pop(0)
        return encode_rc_payload(sample), {"frames_read": 1}

    monkeypatch.setattr("msp_ws_rc_probe.request_msp_over_websocket", fake_request)
    result = run_msp_ws_rc_probe(
        host="127.0.0.1",
        port=5761,
        path="/",
        timeout=1.0,
        channels=DEFAULT_CHANNELS,
        write=True,
        settle_s=0.0,
        max_frames=2,
        baseline_samples=2,
        baseline_interval_s=0.0,
        max_baseline_delta=4,
        allow_unstable_baseline=False,
    )
    assert result.status == WAITING
    assert result.metrics["real_msp_write_sent"] is False
    assert "RC_INPUT_CONTENTION" in result.notes


def test_msp_ws_rc_probe_waits_when_write_is_acknowledged_but_not_latched(monkeypatch):
    baseline = [1500, 1500, 1500, 1025, 1500, 1500, 1500, 1500]
    request_calls = []

    def fake_request(**kwargs):
        request_calls.append(kwargs["code"])
        return encode_rc_payload(baseline), {"frames_read": 1}

    monkeypatch.setattr("msp_ws_rc_probe.request_msp_over_websocket", fake_request)
    monkeypatch.setattr(
        "msp_ws_rc_probe.send_raw_rc_once",
        lambda **_kwargs: {"ack": True, "frames_read": 1, "bytes_read": 6},
    )
    result = run_msp_ws_rc_probe(
        host="127.0.0.1",
        port=5761,
        path="/",
        timeout=1.0,
        channels=DEFAULT_CHANNELS,
        write=True,
        settle_s=0.0,
        max_frames=2,
        baseline_samples=2,
        baseline_interval_s=0.0,
        max_baseline_delta=4,
        allow_unstable_baseline=False,
    )
    assert result.status == WAITING
    assert result.metrics["real_msp_write_sent"] is True
    assert "MSP_RC_WRITE_NOT_LATCHED_OR_RX_OVERRIDDEN" in result.notes


def test_msp_ws_rc_probe_requires_ack_for_write():
    with pytest.raises(SystemExit):
        parse_rc_args(["--write"])


def test_preflight_install_root_and_markdown(tmp_path):
    check = probe_install_root(tmp_path / "install-root")
    assert check.status == PASS
    assert check.metrics["writable"] is True

    report = PreflightReport(
        run_id="unit",
        started_at="now",
        install_root=str(tmp_path),
        checks=[
            ProbeCheck("display_capture", PASS, "ok", {"display": ":1"}),
            ProbeCheck("msp_websocket_port", "WAITING", "not running", {
                "port": DEFAULT_WS_PORT,
            }),
        ],
    )
    text = build_markdown(report)
    assert "SimITL / pr0p Preflight Report" in text
    assert "| display_capture | PASS | ok |" in text
    assert "msp_websocket_port" in text


def test_capture_probe_passes_with_nonblank_source(tmp_path):
    source = SyntheticFrameSource(width=80, height=60, fps=20.0)
    result = analyze_capture_source(
        source,
        duration_s=0.25,
        min_fps=5.0,
        sample_frame_path=tmp_path / "sample.png",
    )
    assert result.status == PASS
    assert result.metrics["frames"] == 5
    assert result.metrics["nonblank_ratio"] == 1.0
    assert Path(result.metrics["sample_frame_path"]).exists()


def test_capture_probe_fails_blank_frames(tmp_path):
    result = analyze_capture_source(
        BlankFrameSource(),
        duration_s=0.3,
        min_fps=1.0,
        sample_frame_path=tmp_path / "blank.png",
    )
    assert result.status == FAIL
    assert "CAPTURE_BLACK_FRAME" in result.notes
    assert result.metrics["nonblank_frames"] == 0


def test_capture_probe_applies_relative_crop():
    region = {"left": 100, "top": 50, "width": 640, "height": 480}
    crop = {"left": 10, "top": 20, "width": 320, "height": 240}
    assert apply_relative_crop(region, crop) == {
        "left": 110,
        "top": 70,
        "width": 320,
        "height": 240,
    }
    with pytest.raises(ValueError):
        apply_relative_crop(region, {"left": 500, "top": 0, "width": 200, "height": 240})


def test_capture_probe_waits_when_window_is_missing(monkeypatch, tmp_path):
    monkeypatch.setattr("pr0p_capture_probe.list_x11_windows", lambda: [])
    result = run_capture_probe(
        run_id="unit",
        log_dir=tmp_path,
        region=None,
        crop=None,
        window_titles=["pr0p"],
        exact=False,
        case_sensitive=False,
        min_width=160,
        min_height=120,
        preferred_size=None,
        allow_common_non_fpv_windows=False,
        backend="auto",
        fps=30.0,
        duration_s=0.1,
        min_fps=1.0,
        min_std=1.0,
    )
    assert result.status == WAITING
    assert "CAPTURE_NO_WINDOW" in result.notes


def test_capture_probe_excludes_editor_false_positive():
    window = X11Window(
        window_id="0x1",
        title="Compare SITL Forge - fpv-test - Visual Studio Code",
        class_text='"code" "code"',
        width=1920,
        height=1080,
        rel_x=0,
        rel_y=0,
        abs_x=0,
        abs_y=0,
    )
    assert is_excluded_window(window, ["visual studio code", "code"])


def test_capture_probe_excludes_updater_terminal_false_positive(monkeypatch, tmp_path):
    updater_window = X11Window(
        window_id="0x1",
        title="/tmp/fpv-test-simitl-pr0p/updater",
        class_text='"terminator" "X-terminal-emulator"',
        width=814,
        height=548,
        rel_x=14,
        rel_y=49,
        abs_x=178,
        abs_y=223,
    )
    monkeypatch.setattr("pr0p_capture_probe.list_x11_windows", lambda: [updater_window])
    result = run_capture_probe(
        run_id="unit",
        log_dir=tmp_path,
        region=None,
        crop=None,
        window_titles=["pr0p"],
        exact=False,
        case_sensitive=False,
        min_width=160,
        min_height=120,
        preferred_size=None,
        allow_common_non_fpv_windows=False,
        backend="auto",
        fps=30.0,
        duration_s=0.1,
        min_fps=1.0,
        min_std=1.0,
    )
    assert is_excluded_window(updater_window, ["/tmp/fpv-test-simitl-pr0p/updater"])
    assert result.status == WAITING
    assert "CAPTURE_NO_WINDOW" in result.notes


def test_virtual_input_probe_dry_run_neutralizes():
    command = command_from_args(yaw=0.2, pitch=0.0, roll=0.0, throttle=0.0)
    result = dry_run_smoke(command, hold_seconds=0.0)
    assert result.status == PASS
    assert result.metrics["real_input_sent"] is False
    assert result.metrics["neutralized"] is True
    assert result.metrics["commands"][0]["yaw"] == 0.2
    assert result.metrics["commands"][-1]["yaw"] == 0.0


def test_virtual_input_probe_waits_when_uinput_required(monkeypatch):
    class FakeProbe:
        available = False

        def as_dict(self):
            return {
                "available": False,
                "evdev_available": False,
                "dev_uinput_exists": False,
                "dev_uinput_writable": False,
                "error": "missing",
            }

    monkeypatch.setattr("pr0p_virtual_input_probe.probe_uinput_environment", lambda: FakeProbe())
    result = run_input_probe(
        command=command_from_args(yaw=0.05, pitch=0.0, roll=0.0, throttle=0.0),
        hold_seconds=0.0,
        require_uinput=True,
        uinput_smoke=False,
    )
    assert result.status == WAITING
    assert result.metrics["real_input_sent"] is False
    assert "UINPUT_PERMISSION_FAIL" in result.notes


def test_virtual_input_probe_reports_ready_when_uinput_required(monkeypatch):
    class FakeProbe:
        available = True

        def as_dict(self):
            return {
                "available": True,
                "evdev_available": True,
                "dev_uinput_exists": True,
                "dev_uinput_writable": True,
                "error": None,
            }

    monkeypatch.setattr("pr0p_virtual_input_probe.probe_uinput_environment", lambda: FakeProbe())
    result = run_input_probe(
        command=command_from_args(yaw=0.05, pitch=0.0, roll=0.0, throttle=0.0),
        hold_seconds=0.0,
        require_uinput=True,
        uinput_smoke=False,
    )
    assert result.status == PASS
    assert "uinput prerequisites look ready" in result.summary
    assert result.metrics["real_input_sent"] is False


def test_input_config_probe_parses_binding_paths_and_flags_virtual_gap(tmp_path):
    assert parse_binding_path("<Joystick>/Stick/x") == ("Joystick", "Stick/x")
    config = tmp_path / "input.json"
    config.write_text(json.dumps({
        "axisFlips": [1.0, -1.0, 1.0, 1.0],
        "bindingOverrides": [
            json.dumps({"bindings": [{"id": "roll-id", "path": "<Joystick>/Stick/x"}]}),
            json.dumps({"bindings": [{"id": "pitch-id", "path": "<Joystick>/Stick/y"}]}),
            json.dumps({"bindings": [{"id": "thr-id", "path": "<Linux::MicrochipTechnologyInc::TeamBlackSheepTBSJoystick>/Z"}]}),
            json.dumps({"bindings": [{"id": "yaw-id", "path": "<Linux::MicrochipTechnologyInc::TeamBlackSheepTBSJoystick>/RotateX"}]}),
        ],
    }), encoding="utf-8")
    data = json.loads(config.read_text(encoding="utf-8"))
    bindings = parse_input_bindings(data)
    assert bindings[0].role == "roll"
    assert bindings[2].device == "Linux::MicrochipTechnologyInc::TeamBlackSheepTBSJoystick"

    virtual_result = run_input_config_probe(
        input_config=config,
        expected_device="Kenet Game Sandbox",
        require_virtual_mapping=True,
        allow_generic_uinput_profile=True,
    )
    assert virtual_result.status == WAITING
    assert "PR0P_VIRTUAL_INPUT_MAPPING_MISSING" in virtual_result.notes
    assert "REMAP_ROLES:throttle,yaw" in virtual_result.notes
    assert [
        row["role"] for row in virtual_result.metrics["controls_rebind_plan"]
    ] == ["throttle", "yaw"]
    assert virtual_result.metrics["controls_rebind_plan"][0]["controls_page"] == "Controls -> RC Channels"
    assert (
        virtual_result.metrics["controls_rebind_plan"][0]["expected_generic_path"]
        == "<Joystick>/RotateY"
    )
    markdown = build_input_config_markdown(virtual_result, run_id="unit")
    assert "Controls -> RC Channels" in markdown
    assert "| throttle | `<Linux::MicrochipTechnologyInc::TeamBlackSheepTBSJoystick>/Z` | `REMAP: RotateY` |" in markdown
    assert "| yaw | `<Linux::MicrochipTechnologyInc::TeamBlackSheepTBSJoystick>/RotateX` | `REMAP: RotateX` |" in markdown

    physical_result = run_input_config_probe(
        input_config=config,
        expected_device="Kenet Game Sandbox",
        require_virtual_mapping=False,
        allow_generic_uinput_profile=True,
    )
    assert physical_result.status == PASS


def test_input_config_probe_accepts_generic_joystick_uinput_profile(tmp_path):
    config = tmp_path / "input.json"
    config.write_text(json.dumps({
        "axisFlips": [1.0, -1.0, -1.0, 1.0],
        "bindingOverrides": [
            json.dumps({"bindings": [{"id": "roll-id", "path": "<Joystick>/Stick/x"}]}),
            json.dumps({"bindings": [{"id": "pitch-id", "path": "<Joystick>/Stick/y"}]}),
            json.dumps({"bindings": [{"id": "thr-id", "path": "<Joystick>/RotateY"}]}),
            json.dumps({"bindings": [{"id": "yaw-id", "path": "<Joystick>/RotateX"}]}),
        ],
    }), encoding="utf-8")
    result = run_input_config_probe(
        input_config=config,
        expected_device="Kenet Game Sandbox",
        require_virtual_mapping=True,
        allow_generic_uinput_profile=True,
    )
    assert result.status == PASS
    assert all(result.metrics["virtual_profile_by_role"].values())


def test_input_config_probe_reports_missing_primary_slots(tmp_path):
    config = tmp_path / "input.json"
    config.write_text(json.dumps({
        "axisFlips": [1.0],
        "bindingOverrides": [
            json.dumps({"bindings": [{"id": "roll-id", "path": "<Joystick>/Stick/x"}]}),
        ],
    }), encoding="utf-8")
    result = run_input_config_probe(
        input_config=config,
        expected_device="Kenet Game Sandbox",
        require_virtual_mapping=True,
        allow_generic_uinput_profile=True,
    )
    assert result.status == WAITING
    assert result.metrics["missing_primary_roles"] == ["pitch", "throttle", "yaw"]
    assert result.metrics["virtual_profile_by_role"] == {
        "roll": True,
        "pitch": False,
        "throttle": False,
        "yaw": False,
    }
    assert [
        row["role"] for row in result.metrics["controls_rebind_plan"]
    ] == ["pitch", "throttle", "yaw"]
    assert "REMAP_ROLES:pitch,throttle,yaw" in result.notes


def physical_mix_input_config() -> dict:
    return {
        "axisFlips": [1.0, -1.0, 1.0, 1.0],
        "guids": ["roll-id", "pitch-id", "thr-id", "yaw-id"],
        "bindingOverrides": [
            json.dumps({"bindings": [{"id": "roll-id", "path": "<Joystick>/Stick/x"}]}),
            json.dumps({"bindings": [{"id": "pitch-id", "path": "<Joystick>/Stick/y"}]}),
            json.dumps({"bindings": [{"id": "thr-id", "path": "<Linux::MicrochipTechnologyInc::TeamBlackSheepTBSJoystick>/Z"}]}),
            json.dumps({"bindings": [{"id": "yaw-id", "path": "<Linux::MicrochipTechnologyInc::TeamBlackSheepTBSJoystick>/RotateX"}]}),
        ],
    }


def test_input_config_patch_dry_run_builds_verified_virtual_mapping(tmp_path):
    config = tmp_path / "input.json"
    config.write_text(json.dumps(physical_mix_input_config()), encoding="utf-8")
    result = run_input_config_patch(
        input_config=config,
        log_dir=tmp_path,
        roles=["all"],
        expected_device="Kenet Game Sandbox",
        target_device="Joystick",
        write=False,
    )
    assert result.status == PASS
    assert result.metrics["real_config_write"] is False
    assert result.metrics["patched_probe_status"] == PASS
    assert result.metrics["changed_roles"] == ["throttle", "yaw"]
    assert result.metrics["target_paths_by_role"]["throttle"] == "<Joystick>/RotateY"
    assert "DRY_RUN_ONLY" in result.notes
    assert "TeamBlackSheep" in config.read_text(encoding="utf-8")

    patched, changes = patch_input_config_data(
        physical_mix_input_config(),
        roles=["throttle"],
        target_device="Joystick",
    )
    assert [change["role"] for change in changes] == ["throttle"]
    assert target_paths_by_role()["yaw"] == "<Joystick>/RotateX"
    assert "<Joystick>/RotateY" in patched["bindingOverrides"][2]
    assert "TeamBlackSheep" in patched["bindingOverrides"][3]

    partial = {"bindingOverrides": ["", "", "", ""]}
    patched_partial, _changes = patch_input_config_data(
        partial,
        roles=["all"],
        target_device="Joystick",
    )
    assert patched_partial["axisFlips"][:4] == [1.0, -1.0, 1.0, 1.0]


def test_input_config_patch_can_plan_aux1_slot_without_default_all(tmp_path):
    data = physical_mix_input_config()
    data["bindingOverrides"] += [""]
    data["axisFlips"] += [1.0]
    data["guids"] += ["aux1-id"]
    config = tmp_path / "input.json"
    config.write_text(json.dumps(data), encoding="utf-8")

    result = run_input_config_patch(
        input_config=config,
        log_dir=tmp_path,
        roles=["aux1"],
        expected_device="Kenet Game Sandbox",
        target_device="Joystick",
        write=False,
    )
    assert result.status == PASS
    assert result.metrics["changed_roles"] == ["aux1"]
    assert result.metrics["path_changes"][0]["slot"] == 4
    assert result.metrics["target_paths_by_role"]["aux1"] == "<Joystick>/Z"
    assert result.metrics["real_config_write"] is False
    assert config.read_text(encoding="utf-8") == json.dumps(data)

    patched, changes = patch_input_config_data(data, roles=["aux1"], target_device="Joystick")
    assert changes[0]["role"] == "aux1"
    assert "<Joystick>/Z" in patched["bindingOverrides"][4]
    assert len(patched["bindingOverrides"]) == 5

    primary_only, _changes = patch_input_config_data(data, roles=["all"], target_device="Joystick")
    assert primary_only["bindingOverrides"][4] == ""


def test_input_config_patch_writes_backup_when_acknowledged(tmp_path):
    config = tmp_path / "input.json"
    config.write_text(json.dumps(physical_mix_input_config()), encoding="utf-8")
    result = run_input_config_patch(
        input_config=config,
        log_dir=tmp_path,
        roles=["all"],
        expected_device="Kenet Game Sandbox",
        target_device="Joystick",
        write=True,
    )
    assert result.status == PASS
    assert result.metrics["real_config_write"] is True
    backup = Path(result.metrics["backup_path"])
    assert backup.exists()
    assert "TeamBlackSheep" in backup.read_text(encoding="utf-8")
    patched = config.read_text(encoding="utf-8")
    assert "<Joystick>/RotateY" in patched
    assert "<Joystick>/RotateX" in patched


def test_input_config_patch_requires_ack_for_write():
    with pytest.raises(SystemExit):
        parse_input_config_patch_args(["--write"])


def test_input_config_restore_waits_without_backup(tmp_path):
    config = tmp_path / "input.json"
    config.write_text(json.dumps(physical_mix_input_config()), encoding="utf-8")
    result = run_input_config_restore(
        input_config=config,
        log_dir=tmp_path,
        backup=None,
        restore=False,
        expected_device="Kenet Game Sandbox",
    )
    assert result.status == WAITING
    assert result.metrics["backup_count"] == 0
    assert "PR0P_INPUT_CONFIG_BACKUP_MISSING" in result.notes


def test_input_config_restore_dry_run_uses_latest_backup(tmp_path):
    config = tmp_path / "input.json"
    config.write_text(json.dumps({"bindingOverrides": []}), encoding="utf-8")
    older = tmp_path / "input.json.codex-backup-older"
    newer = tmp_path / "input.json.codex-backup-newer"
    older.write_text(json.dumps({"bindingOverrides": []}), encoding="utf-8")
    newer.write_text(json.dumps(physical_mix_input_config()), encoding="utf-8")
    os.utime(older, ns=(1_000_000_000, 1_000_000_000))
    os.utime(newer, ns=(2_000_000_000, 2_000_000_000))

    assert list_backups(config)[0] == newer
    result = run_input_config_restore(
        input_config=config,
        log_dir=tmp_path,
        backup=None,
        restore=False,
        expected_device="Kenet Game Sandbox",
    )
    assert result.status == PASS
    assert result.metrics["selected_backup"] == str(newer)
    assert result.metrics["real_config_write"] is False
    assert "DRY_RUN_ONLY" in result.notes
    assert "TeamBlackSheep" not in config.read_text(encoding="utf-8")


def test_input_config_restore_writes_current_snapshot_when_acknowledged(tmp_path):
    config = tmp_path / "input.json"
    config.write_text(json.dumps({"bindingOverrides": []}), encoding="utf-8")
    backup = tmp_path / "input.json.codex-backup-20260705"
    backup.write_text(json.dumps(physical_mix_input_config()), encoding="utf-8")
    result = run_input_config_restore(
        input_config=config,
        log_dir=tmp_path,
        backup=backup,
        restore=True,
        expected_device="Kenet Game Sandbox",
    )
    assert result.status == PASS
    assert result.metrics["real_config_write"] is True
    snapshot = Path(result.metrics["current_snapshot_path"])
    assert snapshot.exists()
    assert json.loads(snapshot.read_text(encoding="utf-8")) == {"bindingOverrides": []}
    assert "TeamBlackSheep" in config.read_text(encoding="utf-8")


def test_input_config_restore_rejects_non_matching_backup(tmp_path):
    config = tmp_path / "input.json"
    config.write_text(json.dumps(physical_mix_input_config()), encoding="utf-8")
    other = tmp_path / "other.json.codex-backup-20260705"
    other.write_text(json.dumps(physical_mix_input_config()), encoding="utf-8")
    result = run_input_config_restore(
        input_config=config,
        log_dir=tmp_path,
        backup=other,
        restore=False,
        expected_device="Kenet Game Sandbox",
    )
    assert result.status == FAIL
    assert "PR0P_INPUT_CONFIG_BACKUP_INVALID" in result.notes


def test_input_config_restore_requires_ack_for_restore():
    with pytest.raises(SystemExit):
        parse_input_config_restore_args(["--restore"])


def test_rc_channel_mapping_assistant_builds_safe_pulse_plan():
    assert expand_roles(["roll", "all", "yaw"]) == ["roll", "pitch", "throttle", "yaw", "aux1"]
    assert mapping_command_for_role("yaw", 0.4).yaw == 0.4
    assert mapping_command_for_role("aux1", 0.7).aux1 == 0.7
    events = build_pulse_events(
        ["throttle", "aux1"],
        amplitude=0.5,
        pulse_seconds=1.0,
        neutral_seconds=0.1,
        between_roles_seconds=0.0,
    )
    assert [event["direction"] for event in events[:4]] == [
        "positive",
        "neutral",
        "negative",
        "neutral",
    ]
    assert events[0]["command"]["throttle"] == 0.5
    assert events[2]["command"]["throttle"] == -0.5
    assert events[4]["command"]["aux1"] == 0.5
    assert events[6]["command"]["aux1"] == -0.5


def test_rc_channel_mapping_assistant_dry_run_never_sends_real_input():
    result = run_mapping_assistant(
        roles=["aux1"],
        use_uinput=False,
        device_name="Kenet Game Sandbox",
        amplitude=0.4,
        setup_delay_s=0.0,
        pulse_seconds=0.0,
        neutral_seconds=0.0,
        between_roles_seconds=0.0,
        hold_neutral_seconds=0.0,
    )
    assert result.status == PASS
    assert result.metrics["real_input_sent"] is False
    assert result.metrics["roles"] == ["aux1"]
    assert result.metrics["commands"][0] == {
        "roll": 0.0,
        "pitch": 0.0,
        "throttle": 0.0,
        "yaw": 0.0,
        "aux1": 0.0,
    }
    assert result.metrics["commands"][1]["aux1"] == 0.4
    assert result.metrics["commands"][3]["aux1"] == -0.4


def test_rc_channel_mapping_assistant_waits_when_uinput_unavailable(monkeypatch):
    class FakeProbe:
        available = False

        def as_dict(self):
            return {
                "available": False,
                "evdev_available": False,
                "dev_uinput_exists": False,
                "dev_uinput_writable": False,
                "error": "missing",
            }

    monkeypatch.setattr("pr0p_rc_channel_mapping_assistant.probe_uinput_environment", lambda: FakeProbe())
    result = run_mapping_assistant(
        roles=["roll"],
        use_uinput=True,
        device_name="Kenet Game Sandbox",
        amplitude=0.4,
        setup_delay_s=0.0,
        pulse_seconds=0.0,
        neutral_seconds=0.0,
        between_roles_seconds=0.0,
        hold_neutral_seconds=0.0,
    )
    assert result.status == WAITING
    assert result.metrics["real_input_sent"] is True
    assert "UINPUT_PERMISSION_FAIL" in result.notes


def test_rc_channel_mapping_assistant_requires_ack_for_live_input():
    with pytest.raises(SystemExit):
        parse_mapping_assistant_args(["--uinput"])


def test_virtual_input_probe_requires_ack_for_real_uinput():
    with pytest.raises(SystemExit):
        parse_input_args(["--uinput-smoke"])


def test_response_probe_estimates_frame_shift_direction():
    source = ShiftAfterPreSource(dx=7, dy=0, fps=10.0, pre_frames=1)
    frames = list(source.frames(0.3))
    dx, dy, response = frame_shift(frames[0].frame, frames[-1].frame)
    assert dx == pytest.approx(7.0, abs=0.5)
    assert abs(dy) < 0.5
    assert response > 0


def test_response_probe_measure_passes_expected_direction():
    source = ShiftAfterPreSource(dx=9, dy=0, fps=10.0, pre_frames=3)
    result = measure_visual_response(
        source,
        DryRunInputAdapter(),
        command=command_for_axis("yaw", 0.05),
        axis="yaw",
        image_axis="x",
        expected_sign=1,
        fps=10.0,
        pre_duration_s=0.3,
        post_duration_s=0.4,
        min_shift_px=2.0,
        max_shift_px=50.0,
    )
    assert result.status == PASS
    assert result.metrics["observed_sign"] == 1
    assert result.metrics["projection_px"] == pytest.approx(9.0, abs=0.75)


def test_response_probe_fails_opposite_direction():
    status, summary, notes = evaluate_direction(
        projection_px=-5.0,
        expected_sign=1,
        min_shift_px=2.0,
        max_shift_px=50.0,
    )
    assert status == FAIL
    assert "opposite" in summary
    assert "DIRECTION_SIGN_FAIL" in notes


def test_response_probe_waits_when_window_is_missing(monkeypatch, tmp_path):
    monkeypatch.setattr(
        "pr0p_response_probe.find_pr0p_window",
        lambda *_args, **_kwargs: (None, [], None),
    )
    result = run_response_probe(
        run_id="unit",
        region=None,
        crop=None,
        window_titles=["pr0p"],
        exact=False,
        case_sensitive=False,
        min_width=160,
        min_height=120,
        preferred_size=None,
        allow_common_non_fpv_windows=False,
        backend="auto",
        fps=10.0,
        axis="yaw",
        magnitude=0.05,
        image_axis="x",
        expected_sign=1,
        pre_duration_s=0.2,
        post_duration_s=0.2,
        min_shift_px=2.0,
        max_shift_px=50.0,
        use_uinput=False,
    )
    assert result.status == WAITING
    assert result.metrics["real_input_sent"] is False
    assert "CAPTURE_NO_WINDOW" in result.notes


def test_response_probe_dry_run_cannot_pass(monkeypatch):
    def fake_source(**_kwargs):
        return ShiftAfterPreSource(dx=8, fps=10.0, pre_frames=3), {}, None

    monkeypatch.setattr("pr0p_response_probe.make_source_from_selection", fake_source)
    result = run_response_probe(
        run_id="unit",
        region=None,
        crop=None,
        window_titles=["pr0p"],
        exact=False,
        case_sensitive=False,
        min_width=160,
        min_height=120,
        preferred_size=None,
        allow_common_non_fpv_windows=False,
        backend="auto",
        fps=10.0,
        axis="yaw",
        magnitude=0.05,
        image_axis="x",
        expected_sign=1,
        pre_duration_s=0.3,
        post_duration_s=0.4,
        min_shift_px=2.0,
        max_shift_px=50.0,
        use_uinput=False,
    )
    assert result.status == WAITING
    assert result.metrics["real_input_sent"] is False
    assert "DRY_RUN_ONLY" in result.notes


def test_response_probe_requires_ack_for_real_uinput():
    with pytest.raises(SystemExit):
        parse_response_args(["--uinput"])


def test_runtime_input_visual_probe_detects_pulse_change():
    source = ShiftAfterPreSource(dx=12, fps=10.0, pre_frames=3)
    result = measure_runtime_input_visual_change(
        source,
        DryRunInputAdapter(),
        command=command_for_axis("yaw", 0.5),
        axis="yaw",
        fps=10.0,
        pre_duration_s=0.3,
        pulse_duration_s=0.4,
        neutral_duration_s=0.0,
        pixel_threshold=10,
        min_mean_absdiff=0.5,
        min_changed_ratio=0.001,
        max_baseline_mean_absdiff=0.1,
        baseline_multiplier=3.0,
    )
    assert result.status == PASS
    assert result.metrics["pulse_summary"]["max_mean_absdiff"] > 0.5
    assert "RUNTIME_INPUT_VISUAL_CHANGE_DETECTED" in result.notes


def test_runtime_input_visual_probe_waits_without_visual_change():
    source = BlankFrameSource(fps=10.0)
    result = measure_runtime_input_visual_change(
        source,
        DryRunInputAdapter(),
        command=command_for_axis("yaw", 0.5),
        axis="yaw",
        fps=10.0,
        pre_duration_s=0.3,
        pulse_duration_s=0.4,
        neutral_duration_s=0.0,
        pixel_threshold=10,
        min_mean_absdiff=0.5,
        min_changed_ratio=0.001,
        max_baseline_mean_absdiff=0.1,
        baseline_multiplier=3.0,
    )
    assert result.status == WAITING
    assert "NO_RUNTIME_INPUT_VISUAL_CHANGE" in result.notes


def test_runtime_input_visual_probe_rejects_unstable_baseline():
    status, summary, notes = evaluate_runtime_visual_change(
        pulse_summary={"max_mean_absdiff": 10.0, "max_changed_ratio": 0.5},
        baseline_summary={"max_mean_absdiff": 2.0, "max_changed_ratio": 0.2},
        min_mean_absdiff=0.5,
        min_changed_ratio=0.001,
        max_baseline_mean_absdiff=0.1,
        baseline_multiplier=3.0,
    )
    assert status == WAITING
    assert "baseline" in summary
    assert "RUNTIME_INPUT_BASELINE_UNSTABLE" in notes


def test_runtime_input_visual_probe_diff_metrics():
    base = np.zeros((10, 10, 3), dtype=np.uint8)
    changed = base.copy()
    changed[2:4, 2:4] = (255, 255, 255)
    metrics = frame_diff_metrics(base, changed, pixel_threshold=10)
    assert metrics["mean_absdiff"] > 0
    assert metrics["changed_ratio"] == pytest.approx(0.04)


def test_runtime_input_visual_probe_dry_run_cannot_pass(monkeypatch):
    def fake_source(**_kwargs):
        return ShiftAfterPreSource(dx=10, fps=10.0, pre_frames=3), {}, None

    monkeypatch.setattr("pr0p_runtime_input_visual_probe.make_source_from_selection", fake_source)
    result = run_runtime_input_visual_probe(
        run_id="unit",
        region=None,
        crop=None,
        window_titles=["pr0p"],
        exact=False,
        case_sensitive=False,
        min_width=160,
        min_height=120,
        preferred_size=None,
        allow_common_non_fpv_windows=False,
        backend="auto",
        fps=10.0,
        axis="yaw",
        magnitude=0.5,
        pre_duration_s=0.3,
        pulse_duration_s=0.4,
        neutral_duration_s=0.0,
        pixel_threshold=10,
        min_mean_absdiff=0.5,
        min_changed_ratio=0.001,
        max_baseline_mean_absdiff=0.1,
        baseline_multiplier=3.0,
        use_uinput=False,
        device_name="Kenet Game Sandbox",
    )
    assert result.status == WAITING
    assert result.metrics["real_input_sent"] is False
    assert "DRY_RUN_ONLY" in result.notes


def test_runtime_input_visual_probe_requires_ack_for_real_uinput():
    with pytest.raises(SystemExit):
        parse_runtime_input_visual_args(["--uinput"])


def test_probe_suite_status_and_markdown():
    assert suite_status([PASS, PASS]) == PASS
    assert suite_status([PASS, WAITING]) == WAITING
    assert suite_status([PASS, FAIL, WAITING]) == FAIL

    report = SuiteReport(
        run_id="unit",
        started_at="now",
        steps=[
            SuiteStep(
                phase="P0-preflight",
                status=PASS,
                summary="ok",
                report_md="logs/p0.md",
            ),
            SuiteStep(
                phase="P3-capture",
                status=WAITING,
                summary="no window",
                notes=["CAPTURE_NO_WINDOW"],
            ),
        ],
    )
    text = build_suite_markdown(report)
    assert "SimITL / pr0p Probe Suite" in text
    assert "Verdict: `WAITING`" in text
    assert "| P3-capture | WAITING | no window |" in text
    assert "Safe suite defaults never send real OS input" in text


def test_tracking_probe_waits_without_bbox(tmp_path):
    result = run_tracking_probe(
        run_id="unit",
        log_dir=tmp_path,
        region={"left": 0, "top": 0, "width": 96, "height": 72},
        crop=None,
        window_titles=["pr0p"],
        exact=False,
        case_sensitive=False,
        min_width=160,
        min_height=120,
        preferred_size=None,
        allow_common_non_fpv_windows=False,
        backend="auto",
        bbox=None,
        duration_s=0.2,
        hz=5.0,
        tracker_type="CSRT",
        enable_pitch=False,
        use_uinput=False,
        min_found_ratio=0.8,
        max_loss_events=0,
        max_abs_yaw_axis=0.6,
        max_abs_pitch_axis=0.6,
        desired_target_width=120.0,
    )
    assert result.status == WAITING
    assert "TRACKER_NO_TARGET" in result.notes
    assert result.metrics["real_input_sent"] is False


def test_tracking_probe_bbox_inside_region_and_file(tmp_path):
    assert bbox_inside_region((1, 2, 10, 20), {"width": 96, "height": 72})
    assert not bbox_inside_region((90, 2, 10, 20), {"width": 96, "height": 72})
    path = tmp_path / "bbox.json"
    path.write_text('{"bbox": [1, 2, 10, 20]}\n', encoding="utf-8")
    assert load_bbox_file(path) == (1, 2, 10, 20)


def test_tracking_probe_evaluates_loop_summary_bounds():
    class Summary:
        frames = 10
        found_ratio = 1.0
        loss_events = 0
        max_abs_yaw_axis = 0.2
        max_abs_pitch_axis = 0.1

    status, summary, notes = evaluate_loop_summary(
        Summary(),
        min_found_ratio=0.9,
        max_loss_events=0,
        max_abs_yaw_axis=0.6,
        max_abs_pitch_axis=0.6,
    )
    assert status == PASS
    assert "bounded" in summary
    assert notes


def test_tracking_probe_dry_run_passes_with_fake_tracker(monkeypatch, tmp_path):
    def fake_selection(**_kwargs):
        return (
            SyntheticFrameSource(width=96, height=72, fps=5.0),
            {
                "selected_region": {"left": 0, "top": 0, "width": 96, "height": 72},
                "capture_region": {"left": 0, "top": 0, "width": 96, "height": 72},
                "crop": None,
                "selected_window": None,
                "visible_windows_sample": [],
            },
            None,
        )

    monkeypatch.setattr("pr0p_tracking_probe.make_source_from_selection", fake_selection)
    monkeypatch.setattr("pr0p_tracking_probe.ObjectTracker", StableTracker)
    result = run_tracking_probe(
        run_id="unit",
        log_dir=tmp_path,
        region=None,
        crop=None,
        window_titles=["pr0p"],
        exact=False,
        case_sensitive=False,
        min_width=160,
        min_height=120,
        preferred_size=None,
        allow_common_non_fpv_windows=False,
        backend="auto",
        bbox=(42, 28, 24, 24),
        duration_s=0.4,
        hz=5.0,
        tracker_type="CSRT",
        enable_pitch=False,
        use_uinput=False,
        min_found_ratio=0.8,
        max_loss_events=0,
        max_abs_yaw_axis=0.6,
        max_abs_pitch_axis=0.6,
        desired_target_width=120.0,
    )
    assert result.status == PASS
    assert result.metrics["real_input_sent"] is False
    assert result.metrics["found_ratio"] == 1.0
    assert "DRY_RUN_ONLY" in result.notes
    assert Path(result.metrics["log_path"]).exists()


def test_tracking_probe_requires_ack_for_real_uinput():
    with pytest.raises(SystemExit):
        parse_tracking_args(["--uinput", "--bbox", "1,2,10,20"])


def write_tracking_jsonl(path, *, samples=4, real_input_sent=False, summary_frames=None):
    rows = [
        {
            "event": "session_start",
            "metadata": {
                "tool": "simitl_pr0p_tracking_probe",
                "adapter": "DryRunInputAdapter" if not real_input_sent else "UInputAdapter",
                "real_input_sent": real_input_sent,
            },
        },
    ]
    for index in range(samples):
        rows.append({
            "event": "game_screen_sample",
            "frame_index": index,
            "target_found": True,
            "command": {
                "yaw": 0.05,
                "pitch": 0.0,
                "roll": 0.0,
                "throttle": 0.0,
            },
        })
    rows.append({
        "event": "simitl_pr0p_tracking_summary",
        "frames": samples if summary_frames is None else summary_frames,
        "found_ratio": 1.0,
        "loss_events": 0,
        "horizontal_error_rms": 12.0,
        "horizontal_error_p95": 15.0,
        "forward_error_rms": 0.0,
        "forward_error_p95": 0.0,
        "max_abs_yaw_axis": 0.05,
        "max_abs_pitch_axis": 0.0,
    })
    path.write_text("\n".join(json.dumps(row) for row in rows) + "\n", encoding="utf-8")


def test_tracking_log_check_accepts_game_screen_phase_s4_summary(tmp_path):
    log_path = tmp_path / "20260705-unit-p6-tracking.jsonl"
    write_tracking_jsonl(log_path)
    rows = [
        json.loads(line)
        for line in log_path.read_text(encoding="utf-8").splitlines()
    ]
    rows[-1]["event"] = "game_screen_phase_s4_summary"
    log_path.write_text("\n".join(json.dumps(row) for row in rows) + "\n", encoding="utf-8")
    result = run_tracking_log_check(
        log_path=log_path,
        log_dir=tmp_path,
        allow_latest=True,
        min_samples=3,
        min_found_ratio=0.85,
        max_loss_events=0,
        max_abs_yaw_axis=0.6,
        max_abs_pitch_axis=0.6,
        require_dry_run=True,
    )
    assert result.status == PASS


def test_tracking_log_check_passes_bounded_dry_run_log(tmp_path):
    log_path = tmp_path / "20260705-unit-p6-tracking.jsonl"
    write_tracking_jsonl(log_path)
    assert latest_tracking_log(tmp_path) == log_path
    result = run_tracking_log_check(
        log_path=log_path,
        log_dir=tmp_path,
        allow_latest=True,
        min_samples=3,
        min_found_ratio=0.85,
        max_loss_events=0,
        max_abs_yaw_axis=0.6,
        max_abs_pitch_axis=0.6,
        require_dry_run=True,
    )
    assert result.status == PASS
    assert result.metrics["sample_count"] == 4
    assert result.metrics["summary_frames"] == 4
    assert result.metrics["summary_horizontal_error_p95"] == 15.0
    assert "TRACKING_LOG_VERIFIED" in result.notes


def test_tracking_log_check_fails_frame_count_mismatch(tmp_path):
    log_path = tmp_path / "20260705-unit-p6-tracking.jsonl"
    write_tracking_jsonl(log_path, samples=4, summary_frames=3)
    result = run_tracking_log_check(
        log_path=log_path,
        log_dir=tmp_path,
        allow_latest=True,
        min_samples=3,
        min_found_ratio=0.85,
        max_loss_events=0,
        max_abs_yaw_axis=0.6,
        max_abs_pitch_axis=0.6,
        require_dry_run=True,
    )
    assert result.status == FAIL
    assert "TRACKING_LOG_FRAME_COUNT_MISMATCH" in result.notes


def test_tracking_log_check_waits_without_log(tmp_path):
    stale = tmp_path / "20260705-stale-p6-tracking.jsonl"
    write_tracking_jsonl(stale)
    result = run_tracking_log_check(
        log_path=None,
        log_dir=tmp_path,
        allow_latest=False,
        min_samples=3,
        min_found_ratio=0.85,
        max_loss_events=0,
        max_abs_yaw_axis=0.6,
        max_abs_pitch_axis=0.6,
        require_dry_run=True,
    )
    assert result.status == WAITING
    assert "TRACKING_LOG_MISSING" in result.notes

    latest_result = run_tracking_log_check(
        log_path=None,
        log_dir=tmp_path,
        allow_latest=True,
        min_samples=3,
        min_found_ratio=0.85,
        max_loss_events=0,
        max_abs_yaw_axis=0.6,
        max_abs_pitch_axis=0.6,
        require_dry_run=True,
    )
    assert latest_result.status == PASS


def test_tracking_log_check_rejects_live_input_log_when_dry_run_required(tmp_path):
    log_path = tmp_path / "20260705-unit-p6-tracking.jsonl"
    write_tracking_jsonl(log_path, real_input_sent=True)
    result = run_tracking_log_check(
        log_path=log_path,
        log_dir=tmp_path,
        allow_latest=True,
        min_samples=3,
        min_found_ratio=0.85,
        max_loss_events=0,
        max_abs_yaw_axis=0.6,
        max_abs_pitch_axis=0.6,
        require_dry_run=True,
    )
    assert result.status == FAIL
    assert "TRACKING_LOG_NOT_DRY_RUN" in result.notes


def test_bbox_tool_waits_without_bbox_and_writes_sample(tmp_path):
    result = analyze_bbox_source(
        SyntheticFrameSource(width=80, height=60, fps=5.0),
        bbox=None,
        frame_path=tmp_path / "frame.png",
        overlay_path=None,
        duration_s=0.2,
    )
    assert result.status == WAITING
    assert Path(result.metrics["frame_path"]).exists()
    assert result.metrics["overlay_path"] is None
    assert result.metrics["frame_width"] == 80


def test_bbox_tool_passes_valid_bbox_and_writes_overlay(tmp_path):
    result = analyze_bbox_source(
        SyntheticFrameSource(width=80, height=60, fps=5.0),
        bbox=(10, 10, 20, 20),
        frame_path=tmp_path / "frame.png",
        overlay_path=tmp_path / "overlay.png",
        duration_s=0.2,
    )
    assert result.status == PASS
    assert result.metrics["bbox_cli"] == "10,10,20,20"
    assert Path(result.metrics["overlay_path"]).exists()


def test_bbox_tool_waits_when_window_is_missing(monkeypatch, tmp_path):
    monkeypatch.setattr(
        "pr0p_bbox_tool.find_pr0p_window",
        lambda *_args, **_kwargs: (None, [], None),
    )
    result = run_bbox_probe(
        run_id="unit",
        log_dir=tmp_path,
        region=None,
        crop=None,
        window_titles=["pr0p"],
        exact=False,
        case_sensitive=False,
        min_width=160,
        min_height=120,
        preferred_size=None,
        allow_common_non_fpv_windows=False,
        backend="auto",
        fps=5.0,
        duration_s=0.2,
        bbox=None,
        frame_path=None,
        overlay_path=None,
    )
    assert result.status == WAITING
    assert "CAPTURE_NO_WINDOW" in result.notes


def test_ui_smoke_parses_xwininfo_and_scales_clicks():
    geometry = parse_xwininfo_geometry("""
      Absolute upper-left X:  685
      Absolute upper-left Y:  179
      Width: 828
      Height: 666
    """)
    assert geometry == WindowGeometry(x=685, y=179, width=828, height=666)
    clicks = planned_clicks(geometry)
    by_label = {click["label"]: click for click in clicks}
    assert by_label["skip-login"]["screen_x"] == 1135
    assert by_label["skip-login"]["screen_y"] == 534
    assert by_label["track-ok"]["screen_x"] == 939
    assert by_label["track-ok"]["screen_y"] == 805


def test_ui_smoke_requires_ack_for_real_clicks():
    with pytest.raises(SystemExit):
        parse_ui_args(["--send-ui"])


def write_generic_virtual_input_config(path: Path):
    path.write_text(json.dumps({
        "axisFlips": [1.0, -1.0, -1.0, 1.0],
        "bindingOverrides": [
            json.dumps({"bindings": [{"id": "roll-id", "path": "<Joystick>/Stick/x"}]}),
            json.dumps({"bindings": [{"id": "pitch-id", "path": "<Joystick>/Stick/y"}]}),
            json.dumps({"bindings": [{"id": "thr-id", "path": "<Joystick>/RotateY"}]}),
            json.dumps({"bindings": [{"id": "yaw-id", "path": "<Joystick>/RotateX"}]}),
        ],
    }), encoding="utf-8")


def test_live_session_runner_requires_ack_for_live_actions():
    with pytest.raises(SystemExit):
        parse_live_session_args(["--launch-pr0p"])
    with pytest.raises(SystemExit):
        parse_live_session_args(["--launch-pr0p", "--ack-live-launch", "--send-ui"])
    with pytest.raises(SystemExit):
        parse_live_session_args(["--run-suite"])
    with pytest.raises(SystemExit):
        parse_live_session_args(["--launch-pr0p", "--ack-live-launch", "--hold-uinput"])
    with pytest.raises(SystemExit):
        parse_live_session_args([
            "--launch-pr0p",
            "--ack-live-launch",
            "--ack-live-input",
            "--run-live-response",
        ])
    with pytest.raises(SystemExit):
        parse_live_session_args([
            "--launch-pr0p",
            "--ack-live-launch",
            "--ack-live-input",
            "--run-rc-effect",
        ])
    with pytest.raises(SystemExit):
        parse_live_session_args([
            "--launch-pr0p",
            "--ack-live-launch",
            "--ack-live-input",
            "--run-status-effect",
        ])
    with pytest.raises(SystemExit):
        parse_live_session_args([
            "--launch-pr0p",
            "--ack-live-launch",
            "--ack-live-input",
            "--run-aux-arm-status",
        ])


class _ArmRecordingAdapter:
    def __init__(self):
        self.commands = []
        self.neutral_calls = 0

    def send(self, command):
        self.commands.append(command)

    def neutral(self):
        self.neutral_calls += 1


class _FakeStatusResult:
    def __init__(self, fc_status):
        self.metrics = {"fc_status": fc_status}


def test_arm_and_hover_for_response_sends_arm_sequence():
    adapter = _ArmRecordingAdapter()
    sleeps = []
    ok, metrics, notes = arm_and_hover_for_response(
        adapter,
        ws_host="127.0.0.1",
        ws_port=5761,
        arm_wait_s=3.0,
        arm_settle_s=1.0,
        arm_throttle=-0.4,
        hover_wait_s=2.0,
        status_reader=lambda **_kw: _FakeStatusResult(
            {"armed": True, "active_modes": ["ARM"], "arming_disable_names": []}
        ),
        sleeper=sleeps.append,
    )
    assert ok is True
    assert notes == ["ARM_FIRST_ARMED"]
    assert metrics["armed_before_pulse"] is True
    assert sleeps == [3.0, 1.0, 2.0]
    assert len(adapter.commands) == 3
    assert adapter.commands[0].throttle == 1.0
    assert adapter.commands[0].aux1 == 0.0
    assert adapter.commands[1].throttle == 1.0
    assert adapter.commands[1].aux1 == 1.0
    assert adapter.commands[2].throttle == -0.4
    assert adapter.commands[2].aux1 == 1.0
    assert adapter.neutral_calls == 0


def test_arm_and_hover_for_response_waits_when_not_armed():
    adapter = _ArmRecordingAdapter()
    ok, metrics, notes = arm_and_hover_for_response(
        adapter,
        ws_host="127.0.0.1",
        ws_port=5761,
        arm_wait_s=0.0,
        arm_settle_s=0.0,
        arm_throttle=-0.4,
        hover_wait_s=0.0,
        status_reader=lambda **_kw: _FakeStatusResult(
            {"armed": False, "active_modes": [], "arming_disable_names": ["BOOTGRACE"]}
        ),
        sleeper=lambda _s: None,
    )
    assert ok is False
    assert "ARM_FIRST_NOT_ARMED" in notes
    assert "ARM_BLOCKERS:BOOTGRACE" in notes
    assert metrics["armed_before_pulse"] is False
    assert len(adapter.commands) == 2
    assert adapter.neutral_calls == 1


def test_response_pulse_command_holds_arm_axes():
    plain = response_pulse_command("yaw", 0.5, arm_first=False, arm_throttle=-0.3)
    assert plain.yaw == 0.5
    assert plain.throttle == 0.0
    assert plain.aux1 == 0.0
    armed = response_pulse_command("yaw", 0.5, arm_first=True, arm_throttle=-0.3)
    assert armed.yaw == 0.5
    assert armed.throttle == -0.3
    assert armed.aux1 == 1.0
    throttle_pulse = response_pulse_command("throttle", 0.8, arm_first=True, arm_throttle=-0.3)
    assert throttle_pulse.throttle == 0.8
    assert throttle_pulse.aux1 == 1.0


def test_live_session_parse_args_arm_first_requires_live_response():
    with pytest.raises(SystemExit):
        parse_live_session_args([
            "--launch-pr0p",
            "--ack-live-launch",
            "--hold-uinput",
            "--ack-live-input",
            "--response-arm-first",
        ])


def test_attitude_payload_parse_and_wrap():
    from msp_uinput_attitude_response_probe import (
        accumulated_delta,
        parse_attitude_payload,
        wrap_deg,
    )
    payload = struct.pack("<hhh", -123, 456, 270)
    parsed = parse_attitude_payload(payload)
    assert parsed == {"roll_deg": -12.3, "pitch_deg": 45.6, "yaw_deg": 270.0}
    assert parse_attitude_payload(None) is None
    assert parse_attitude_payload(b"\x00\x00") is None
    assert wrap_deg(190.0) == -170.0
    assert wrap_deg(-190.0) == 170.0
    assert accumulated_delta([350.0, 10.0, 30.0], wrap=True) == pytest.approx(40.0)
    assert accumulated_delta([10.0, 350.0], wrap=True) == pytest.approx(-20.0)
    assert accumulated_delta([0.0, 200.0], wrap=False) == pytest.approx(200.0)


def test_evaluate_attitude_response_signs():
    from msp_uinput_attitude_response_probe import evaluate_attitude_response
    status, _summary, notes = evaluate_attitude_response(
        axis="yaw", delta_deg=42.0, expected_sign=1,
        min_delta_deg=10.0, max_delta_deg=1080.0,
    )
    assert status == "PASS"
    assert "ATTITUDE_RESPONSE_DETECTED" in notes
    status, _summary, notes = evaluate_attitude_response(
        axis="yaw", delta_deg=-42.0, expected_sign=1,
        min_delta_deg=10.0, max_delta_deg=1080.0,
    )
    assert status == "FAIL"
    assert "ATTITUDE_SIGN_MISMATCH" in notes
    status, _summary, notes = evaluate_attitude_response(
        axis="yaw", delta_deg=2.0, expected_sign=1,
        min_delta_deg=10.0, max_delta_deg=1080.0,
    )
    assert status == "WAITING"
    assert "NO_ATTITUDE_RESPONSE" in notes
    status, _summary, notes = evaluate_attitude_response(
        axis="yaw", delta_deg=5000.0, expected_sign=1,
        min_delta_deg=10.0, max_delta_deg=1080.0,
    )
    assert status == "FAIL"
    assert "ATTITUDE_RESPONSE_UNBOUNDED" in notes


def test_hold_axes_adapter_merges_holds_and_soft_neutral():
    from pr0p_arm_sequence import HoldAxesAdapter
    from virtual_input import AxisCommand

    inner = _ArmRecordingAdapter()
    inner.closed = False
    inner.close = lambda: setattr(inner, "closed", True)
    hold = HoldAxesAdapter(inner, throttle=-0.4)
    hold.send(AxisCommand(yaw=0.3))
    assert inner.commands[-1].yaw == 0.3
    assert inner.commands[-1].throttle == -0.4
    assert inner.commands[-1].aux1 == 1.0
    hold.neutral()
    assert inner.commands[-1].yaw == 0.0
    assert inner.commands[-1].throttle == -0.4
    assert inner.commands[-1].aux1 == 1.0
    assert inner.neutral_calls == 0
    hold.close()
    assert inner.neutral_calls == 1
    assert inner.closed is True


def test_tracking_probe_arm_first_requires_uinput():
    from pr0p_tracking_probe import parse_args as parse_tracking_args
    with pytest.raises(SystemExit):
        parse_tracking_args(["--bbox", "1,2,3,4", "--arm-first"])
    args = parse_tracking_args([
        "--bbox", "1,2,3,4", "--uinput", "--ack-live-input", "--arm-first",
    ])
    assert args.arm_first is True
    assert args.arm_throttle == -0.4


def test_tracking_acceptance_live_command_includes_arm_first():
    from pr0p_tracking_acceptance_runner import tracking_command
    command = tracking_command(
        run_id="unit",
        bbox=(1, 2, 3, 4),
        window_title="pr0p",
        duration_s=5.0,
        hz=10.0,
        tracker="CSRT",
        live=True,
        enable_pitch=False,
        min_found_ratio=0.85,
        max_loss_events=0,
        max_abs_yaw_axis=0.6,
        max_abs_pitch_axis=0.6,
        desired_target_width=120.0,
        arm_first=True,
        arm_throttle=-0.4,
    )
    assert "--arm-first" in command
    assert "--arm-throttle -0.4" in command
    assert "--takeoff-delay-frames 10" in command
    assert "--takeoff-boost-frames 10" in command
    assert "--settle-throttle -0.26" in command
    dry = tracking_command(
        run_id="unit",
        bbox=(1, 2, 3, 4),
        window_title="pr0p",
        duration_s=5.0,
        hz=10.0,
        tracker="CSRT",
        live=False,
        enable_pitch=False,
        min_found_ratio=0.85,
        max_loss_events=0,
        max_abs_yaw_axis=0.6,
        max_abs_pitch_axis=0.6,
        desired_target_width=120.0,
        arm_first=True,
        arm_throttle=-0.4,
    )
    assert "--arm-first" not in dry


def test_tracking_acceptance_command_forwards_pitch_delay_frames():
    from pr0p_tracking_acceptance_runner import tracking_command
    live = tracking_command(
        run_id="unit",
        bbox=(1, 2, 3, 4),
        window_title="pr0p",
        duration_s=5.0,
        hz=10.0,
        tracker="CSRT",
        live=True,
        enable_pitch=True,
        min_found_ratio=0.85,
        max_loss_events=0,
        max_abs_yaw_axis=0.6,
        max_abs_pitch_axis=0.6,
        desired_target_width=120.0,
        pitch_delay_frames=30,
        arm_first=True,
    )
    assert "--enable-pitch" in live
    assert "--pitch-delay-frames 30" in live
    no_pitch = tracking_command(
        run_id="unit",
        bbox=(1, 2, 3, 4),
        window_title="pr0p",
        duration_s=5.0,
        hz=10.0,
        tracker="CSRT",
        live=True,
        enable_pitch=False,
        min_found_ratio=0.85,
        max_loss_events=0,
        max_abs_yaw_axis=0.6,
        max_abs_pitch_axis=0.6,
        desired_target_width=120.0,
        pitch_delay_frames=30,
        arm_first=True,
    )
    assert "--pitch-delay-frames" not in no_pitch


def test_tracking_probe_axis_limits_clamp_loop_outputs():
    from pr0p_tracking_probe import apply_axis_clamps
    from screen_tracking_loop import LoopConfig
    config = apply_axis_clamps(
        LoopConfig(enable_pitch=True),
        max_abs_yaw_axis=0.2,
        max_abs_pitch_axis=0.15,
    )
    assert config.yaw_output_limit * config.yaw_scale == pytest.approx(0.2)
    assert config.forward_output_limit * config.pitch_scale == pytest.approx(0.15)
    loose = apply_axis_clamps(
        LoopConfig(enable_pitch=True),
        max_abs_yaw_axis=10.0,
        max_abs_pitch_axis=10.0,
    )
    assert loose.yaw_output_limit == LoopConfig().yaw_output_limit
    assert loose.forward_output_limit == LoopConfig().forward_output_limit


def test_vertical_hold_throttle_signs_and_clamps():
    from pr0p_tracking_probe import vertical_hold_throttle
    base = -0.26
    kwargs = dict(frame_center_y=333.0, base=base, kp=0.10,
                  min_axis=base - 0.06, max_axis=base + 0.12, lost_bias=0.04)
    centered = vertical_hold_throttle(center_y=333.0, found=True, **kwargs)
    assert centered == pytest.approx(base)
    # target below center -> vehicle too high -> less thrust (axis toward +1)
    below = vertical_hold_throttle(center_y=500.0, found=True, **kwargs)
    assert below > base
    # target above center -> vehicle too low -> more thrust (axis toward -1)
    above = vertical_hold_throttle(center_y=150.0, found=True, **kwargs)
    assert above < base
    # clamps hold at extremes
    assert vertical_hold_throttle(center_y=10000.0, found=True, **kwargs) == base + 0.12
    assert vertical_hold_throttle(center_y=-10000.0, found=True, **kwargs) == base - 0.06
    # loss biases toward gentle descent, never full throttle-cut
    lost = vertical_hold_throttle(center_y=None, found=False, **kwargs)
    assert lost == pytest.approx(base + 0.04)


def test_last_result_tracker_proxy_records_center_y():
    from pr0p_tracking_probe import LastResultTrackerProxy

    class FakeResult:
        def __init__(self, found, center):
            self.found = found
            self.center = center
            self.bbox = (0, 0, 10, 10) if found else None

    class FakeTracker:
        is_initialized = True

        def __init__(self):
            self.results = [FakeResult(True, (100.0, 200.0)), FakeResult(False, None)]

        def init(self, frame, bbox):
            pass

        def update(self, frame):
            return self.results.pop(0)

    proxy = LastResultTrackerProxy(FakeTracker())
    proxy.update(None)
    assert proxy.last_found is True
    assert proxy.last_center_y == 200.0
    proxy.update(None)
    assert proxy.last_found is False
    assert proxy.last_center_y == 200.0


def test_takeoff_ramp_schedule_interpolates_to_settle():
    from pr0p_tracking_probe import takeoff_ramp_schedule
    schedule = takeoff_ramp_schedule(
        delay_frames=10, boost_frames=0, boost_throttle=-0.4,
        ramp_frames=4, settle_throttle=-0.26,
    )
    assert schedule[0] == (10, 1.0)
    ramp = schedule[1:]
    assert len(ramp) == 4
    values = [value for _, value in ramp]
    # strictly decreasing from ground toward settle, never reaching it
    assert all(a > b for a, b in zip(values, values[1:]))
    assert values[0] < 1.0
    assert values[-1] > -0.26
    no_ramp = takeoff_ramp_schedule(
        delay_frames=10, boost_frames=4, boost_throttle=-0.4,
        ramp_frames=0, settle_throttle=-0.26,
    )
    assert no_ramp == [(10, 1.0), (4, -0.4)]


def test_tracking_acceptance_command_forwards_track_throttle_hold():
    from pr0p_tracking_acceptance_runner import tracking_command
    live = tracking_command(
        run_id="unit",
        bbox=(1, 2, 3, 4),
        window_title="pr0p",
        duration_s=5.0,
        hz=10.0,
        tracker="CSRT",
        live=True,
        enable_pitch=True,
        min_found_ratio=0.85,
        max_loss_events=0,
        max_abs_yaw_axis=0.6,
        max_abs_pitch_axis=0.6,
        desired_target_width=120.0,
        arm_first=True,
        track_throttle_hold=True,
    )
    assert "--track-throttle-hold" in live


def test_tracking_probe_parses_pitch_delay_frames():
    from pr0p_tracking_probe import parse_args as parse_probe_args
    args = parse_probe_args([
        "--bbox", "1,2,3,4", "--enable-pitch", "--pitch-delay-frames", "25",
    ])
    assert args.pitch_delay_frames == 25
    default_args = parse_probe_args(["--bbox", "1,2,3,4"])
    assert default_args.pitch_delay_frames == 0
    with pytest.raises(SystemExit):
        parse_probe_args(["--bbox", "1,2,3,4", "--pitch-delay-frames", "-1"])


def test_response_acceptance_command_includes_measure_flag():
    command = live_response_command(
        run_id="unit",
        axis="yaw",
        magnitude=0.03,
        image_axis="x",
        expected_sign=1,
        min_shift_px=2.0,
        max_shift_px=200.0,
        startup_wait_s=8.0,
        measure="attitude",
        attitude_min_delta_deg=15.0,
    )
    assert "--response-measure attitude" in command
    assert "--response-attitude-min-delta 15.0" in command


def test_response_acceptance_command_includes_arm_first_flags():
    command = live_response_command(
        run_id="unit",
        axis="yaw",
        magnitude=0.03,
        image_axis="x",
        expected_sign=1,
        min_shift_px=2.0,
        max_shift_px=200.0,
        startup_wait_s=8.0,
        arm_first=True,
        arm_wait_s=10.0,
        arm_throttle=-0.3,
    )
    assert "--response-arm-first" in command
    assert "--response-arm-wait 10.0" in command
    assert "--response-arm-throttle -0.3" in command
    plain = live_response_command(
        run_id="unit",
        axis="yaw",
        magnitude=0.03,
        image_axis="x",
        expected_sign=1,
        min_shift_px=2.0,
        max_shift_px=200.0,
        startup_wait_s=8.0,
    )
    assert "--response-arm-first" not in plain


def test_manual_response_command_includes_attitude_profile_flags():
    command = manual_response_command(
        run_id="unit",
        axis="yaw",
        image_axis="x",
        magnitude=0.05,
        min_shift_px=2.0,
        max_shift_px=200.0,
        startup_wait_s=8.0,
        arm_first=True,
        arm_throttle=-0.4,
        measure="attitude",
        attitude_min_delta_deg=12.5,
    )

    assert "--response-arm-first" in command
    assert "--response-arm-throttle -0.4" in command
    assert "--response-measure attitude" in command
    assert "--response-attitude-min-delta 12.5" in command


def test_live_session_runner_dry_run_never_launches(tmp_path):
    install_root = tmp_path / "install"
    install_root.mkdir()
    executable = install_root / "pr0p.x86_64"
    executable.write_text("#!/bin/sh\n", encoding="utf-8")
    executable.chmod(0o755)
    config = tmp_path / "input.json"
    write_generic_virtual_input_config(config)
    launched = []

    def fake_popen(*_args, **_kwargs):
        launched.append(True)
        raise AssertionError("dry-run must not launch pr0p")

    result = run_live_session(
        run_id="unit",
        log_dir=tmp_path,
        install_root=install_root,
        input_config=config,
        expected_device="Kenet Game Sandbox",
        launch_pr0p=False,
        send_ui=False,
        run_suite_after_launch=False,
        startup_wait_s=0.0,
        cleanup_timeout_s=0.1,
        ws_host="127.0.0.1",
        ws_port=5761,
        window_title="pr0p",
        timeout=0.1,
        popen_factory=fake_popen,
        process_lister=lambda: [],
    )
    assert result.status == PASS
    assert result.metrics["real_launch"] is False
    assert result.metrics["planned_command"] == [str(executable)]
    assert launched == []


def test_live_session_runner_reports_input_mapping_gap_before_live_control(tmp_path):
    install_root = tmp_path / "install"
    install_root.mkdir()
    executable = install_root / "pr0p.x86_64"
    executable.write_text("#!/bin/sh\n", encoding="utf-8")
    executable.chmod(0o755)
    config = tmp_path / "input.json"
    config.write_text(json.dumps(physical_mix_input_config()), encoding="utf-8")

    result = run_live_session(
        run_id="unit",
        log_dir=tmp_path,
        install_root=install_root,
        input_config=config,
        expected_device="Kenet Game Sandbox",
        launch_pr0p=False,
        send_ui=False,
        run_suite_after_launch=False,
        startup_wait_s=0.0,
        cleanup_timeout_s=0.1,
        ws_host="127.0.0.1",
        ws_port=5761,
        window_title="pr0p",
        timeout=0.1,
        process_lister=lambda: [],
    )
    assert result.status == WAITING
    assert "PR0P_INPUT_MAPPING_NOT_READY" in result.notes
    assert (
        result.metrics["input_config"]["metrics"]["controls_rebind_plan"][0]["controls_page"]
        == "Controls -> RC Channels"
    )


def test_live_session_runner_cleans_launched_process(tmp_path):
    install_root = tmp_path / "install"
    install_root.mkdir()
    executable = install_root / "pr0p.x86_64"
    executable.write_text("#!/bin/sh\n", encoding="utf-8")
    executable.chmod(0o755)
    config = tmp_path / "input.json"
    write_generic_virtual_input_config(config)

    class FakeProcess:
        pid = 1234

        def __init__(self):
            self.returncode = None
            self.terminated = False
            self.killed = False

        def poll(self):
            return self.returncode

        def terminate(self):
            self.terminated = True
            self.returncode = 0

        def kill(self):
            self.killed = True
            self.returncode = -9

        def wait(self, timeout=None):
            return self.returncode

    process = FakeProcess()

    def fake_popen(*_args, **_kwargs):
        return process

    result = run_live_session(
        run_id="unit",
        log_dir=tmp_path,
        install_root=install_root,
        input_config=config,
        expected_device="Kenet Game Sandbox",
        launch_pr0p=True,
        send_ui=False,
        run_suite_after_launch=False,
        startup_wait_s=0.0,
        cleanup_timeout_s=0.1,
        ws_host="127.0.0.1",
        ws_port=5761,
        window_title="pr0p",
        timeout=0.1,
        popen_factory=fake_popen,
        process_lister=lambda: [],
    )
    assert result.status == PASS
    assert result.metrics["real_launch"] is True
    assert result.metrics["cleanup_performed"] is True
    assert result.metrics["cleanup_ok"] is True
    assert process.terminated is True
    assert process.killed is False


def test_live_session_runner_holds_persistent_uinput_through_live_response(tmp_path):
    install_root = tmp_path / "install"
    install_root.mkdir()
    executable = install_root / "pr0p.x86_64"
    executable.write_text("#!/bin/sh\n", encoding="utf-8")
    executable.chmod(0o755)
    config = tmp_path / "input.json"
    write_generic_virtual_input_config(config)
    events = []

    class FakeProbe:
        available = True

        def as_dict(self):
            return {"available": True}

    class FakeAdapter:
        def __init__(self, *, name):
            self.name = name
            self.closed = False
            events.append("adapter_init")

        def neutral(self):
            events.append("neutral")

        def close(self):
            self.closed = True
            events.append("adapter_close")

    class FakeProcess:
        pid = 4321

        def __init__(self):
            self.returncode = None

        def poll(self):
            return self.returncode

        def terminate(self):
            events.append("terminate")
            self.returncode = 0

        def kill(self):
            self.returncode = -9

        def wait(self, timeout=None):
            return self.returncode

    def fake_popen(*_args, **_kwargs):
        events.append("popen")
        return FakeProcess()

    def fake_live_response(**kwargs):
        events.append("live_response")
        assert kwargs["adapter"].name == "Kenet Game Sandbox"
        return {
            "status": PASS,
            "summary": "visual response moved",
            "metrics": {"persistent_uinput": True},
            "notes": [],
        }

    def fake_rc_effect(**kwargs):
        events.append("rc_effect")
        assert kwargs["adapter"].name == "Kenet Game Sandbox"
        assert kwargs["expected_channel"] == "throttle"
        return {
            "status": PASS,
            "summary": "throttle low reached",
            "metrics": {"persistent_uinput": True, "during_throttle_min": 1000},
            "notes": ["THROTTLE_LOW_REACHED"],
        }

    def fake_status_effect(**kwargs):
        events.append("status_effect")
        assert kwargs["adapter"].name == "Kenet Game Sandbox"
        assert kwargs["blocker"] == "THROTTLE"
        return {
            "status": PASS,
            "summary": "throttle blocker clear",
            "metrics": {"persistent_uinput": True},
            "notes": ["UINPUT_CLEARED_THROTTLE"],
        }

    def fake_arm_effect(**kwargs):
        events.append("arm_effect")
        assert kwargs["adapter"].name == "Kenet Game Sandbox"
        assert kwargs["buttons"] == ["south", "east"]
        return {
            "status": PASS,
            "summary": "ARM mode active",
            "metrics": {"persistent_uinput": True},
            "notes": ["UINPUT_ARM_BUTTON_DETECTED:south"],
        }

    def fake_aux_arm_status(**kwargs):
        events.append("aux_arm_status")
        assert kwargs["adapter"].name == "Kenet Game Sandbox"
        assert kwargs["throttle_magnitude"] == 1.0
        assert kwargs["aux1_magnitude"] == 1.0
        return {
            "status": PASS,
            "summary": "AUX1 armed",
            "metrics": {"persistent_uinput": True},
            "notes": ["UINPUT_AUX1_ARMED"],
        }

    result = run_live_session(
        run_id="unit",
        log_dir=tmp_path,
        install_root=install_root,
        input_config=config,
        expected_device="Kenet Game Sandbox",
        launch_pr0p=True,
        send_ui=False,
        run_suite_after_launch=False,
        startup_wait_s=0.0,
        cleanup_timeout_s=0.1,
        ws_host="127.0.0.1",
        ws_port=5761,
        window_title="pr0p",
        timeout=0.1,
        hold_uinput=True,
        run_live_response_after_launch=True,
        run_rc_effect_after_launch=True,
        run_status_effect_after_launch=True,
        run_arm_button_effect_after_launch=True,
        run_aux_arm_status_after_launch=True,
        popen_factory=fake_popen,
        process_lister=lambda: [],
        uinput_adapter_factory=FakeAdapter,
        uinput_probe=FakeProbe,
        live_response_runner=fake_live_response,
        rc_effect_runner=fake_rc_effect,
        status_effect_runner=fake_status_effect,
        arm_button_effect_runner=fake_arm_effect,
        aux_arm_status_runner=fake_aux_arm_status,
    )
    assert result.status == PASS
    assert result.metrics["persistent_uinput_active_at_launch"] is True
    assert result.metrics["live_response"]["status"] == PASS
    assert result.metrics["live_rc_effect"]["status"] == PASS
    assert result.metrics["live_status_effect"]["status"] == PASS
    assert result.metrics["live_arm_effect"]["status"] == PASS
    assert result.metrics["live_aux_arm_status"]["status"] == PASS
    assert events.index("adapter_init") < events.index("popen")
    assert events.index("popen") < events.index("live_response")
    assert events.index("popen") < events.index("rc_effect")
    assert events.index("popen") < events.index("status_effect")
    assert events.index("popen") < events.index("arm_effect")
    assert events.index("popen") < events.index("aux_arm_status")
    assert "adapter_close" in events


def test_live_session_runner_finds_executable_by_default_name(tmp_path):
    install_root = tmp_path / "install"
    install_root.mkdir()
    executable = install_root / "pr0p.x86_64"
    executable.write_text("#!/bin/sh\n", encoding="utf-8")
    executable.chmod(0o755)

    assert find_pr0p_executable(install_root) == executable


def test_updater_runner_requires_ack_for_external_binary():
    with pytest.raises(SystemExit):
        parse_updater_args(["--launch-updater"])
    with pytest.raises(SystemExit):
        parse_updater_args(["--leave-running"])
    with pytest.raises(SystemExit):
        parse_updater_args(["--launch-updater", "--ack-external-binary", "--leave-running"])


def test_updater_runner_dry_run_never_launches(monkeypatch, tmp_path):
    install_root = tmp_path / "install"
    install_root.mkdir()
    updater = install_root / "updater"
    updater.write_text("#!/bin/sh\n", encoding="utf-8")
    updater.chmod(0o755)
    monkeypatch.setattr("pr0p_install_probe.ALLOWED_INSTALL_ROOTS", (install_root,))
    launched = []

    def fake_popen(*_args, **_kwargs):
        launched.append(True)
        raise AssertionError("dry-run must not launch updater")

    result = run_updater(
        run_id="unit",
        log_dir=tmp_path,
        install_root=install_root,
        launch_updater=False,
        leave_running=False,
        startup_wait_s=0.0,
        cleanup_timeout_s=0.1,
        popen_factory=fake_popen,
        process_lister=lambda: [],
        window_lister=lambda: [],
    )
    assert result.status == PASS
    assert result.metrics["real_launch"] is False
    assert result.metrics["planned_command"] == [str(updater)]
    assert "DRY_RUN_ONLY" in result.notes
    assert launched == []


def test_updater_runner_cleans_launched_updater(monkeypatch, tmp_path):
    install_root = tmp_path / "install"
    install_root.mkdir()
    updater = install_root / "updater"
    updater.write_text("#!/bin/sh\n", encoding="utf-8")
    updater.chmod(0o755)
    monkeypatch.setattr("pr0p_install_probe.ALLOWED_INSTALL_ROOTS", (install_root,))

    class FakeProcess:
        pid = 4242

        def __init__(self):
            self.returncode = None
            self.terminated = False
            self.killed = False

        def poll(self):
            return self.returncode

        def terminate(self):
            self.terminated = True
            self.returncode = 0

        def kill(self):
            self.killed = True
            self.returncode = -9

        def wait(self, timeout=None):
            return self.returncode

    process = FakeProcess()

    def fake_popen(*_args, **_kwargs):
        return process

    result = run_updater(
        run_id="unit",
        log_dir=tmp_path,
        install_root=install_root,
        launch_updater=True,
        leave_running=False,
        startup_wait_s=0.0,
        cleanup_timeout_s=0.1,
        popen_factory=fake_popen,
        process_lister=lambda: [],
        window_lister=lambda: [],
    )
    assert find_updater(install_root) == updater
    assert result.status == PASS
    assert result.metrics["real_launch"] is True
    assert result.metrics["cleanup_performed"] is True
    assert process.terminated is True
    assert process.killed is False


def test_live_manifest_uses_latest_suite_and_next_action(tmp_path):
    older = tmp_path / "20260705-000000-old-suite.json"
    newer = tmp_path / "20260705-010000-new-suite.json"
    older.write_text('{"steps": []}\n', encoding="utf-8")
    newer.write_text(json.dumps({
        "status": WAITING,
        "steps": [
            {"phase": "P1-install-discovery", "status": PASS, "summary": "updater ok"},
            {"phase": "P1-client-executable", "status": PASS, "summary": "client ok"},
            {
                "phase": "P2-websocket",
                "status": WAITING,
                "summary": "websocket endpoint is not reachable yet",
                "report_md": "logs/ws.md",
            },
            {
                "phase": "P4-input-readiness",
                "status": PASS,
                "summary": "input ready",
            },
        ],
    }), encoding="utf-8")
    assert latest_suite_report(tmp_path) == newer
    assert load_suite_report(newer)["status"] == WAITING

    manifest = build_manifest(
        run_id="unit",
        log_dir=tmp_path,
        suite_report=None,
        bbox=None,
        window_title="pr0p",
        yaw_expected_sign=0,
        pitch_expected_sign=0,
    )
    assert manifest.suite_report == str(newer)
    assert "Start pr0p" in manifest.next_action
    text = build_manifest_markdown(manifest)
    assert "SimITL / pr0p Live Run Manifest" in text
    assert "P0-isolation" in text
    assert "P2-websocket" in text
    assert "ws.md" in text


def test_live_manifest_uses_newest_suite_by_mtime_not_name(tmp_path):
    name_newer_but_stale = tmp_path / "20260705-999999-stale-suite.json"
    name_older_but_fresh = tmp_path / "20260705-000000-fresh-suite.json"
    name_newer_but_stale.write_text('{"steps": []}\n', encoding="utf-8")
    name_older_but_fresh.write_text('{"steps": []}\n', encoding="utf-8")
    os.utime(name_newer_but_stale, ns=(1_000_000_000, 1_000_000_000))
    os.utime(name_older_but_fresh, ns=(2_000_000_000, 2_000_000_000))

    assert latest_suite_report(tmp_path) == name_older_but_fresh


def test_live_manifest_embeds_bbox_in_commands(tmp_path):
    suite_path = tmp_path / "suite.json"
    suite_path.write_text(json.dumps({
        "status": WAITING,
        "steps": [
            {"phase": "P1-install-discovery", "status": PASS, "summary": "ok"},
            {"phase": "P1-client-executable", "status": PASS, "summary": "ok"},
            {"phase": "P2-websocket", "status": PASS, "summary": "ok"},
            {"phase": "P2-msp-readonly", "status": PASS, "summary": "ok"},
            {"phase": "P2-fc-status-readonly", "status": PASS, "summary": "ok"},
            {"phase": "P2-mode-ranges-readonly", "status": PASS, "summary": "ok"},
            {"phase": "P3-capture", "status": PASS, "summary": "ok"},
            {"phase": "P4-input-readiness", "status": PASS, "summary": "ok"},
            {"phase": "P4-input-mapping", "status": PASS, "summary": "ok"},
            {"phase": "P6-tracking-pid-dry-run", "status": WAITING, "summary": "needs bbox"},
        ],
    }), encoding="utf-8")
    manifest = build_manifest(
        run_id="unit",
        log_dir=tmp_path,
        suite_report=suite_path,
        bbox=(1, 2, 30, 40),
        window_title="SITL Forge",
        yaw_expected_sign=1,
        pitch_expected_sign=-1,
    )
    commands = {step.phase: step.command for step in manifest.steps}
    assert "pr0p_isolation_check.py" in commands["P0-isolation"]
    assert "--bbox 1,2,30,40" in commands["P6-bbox-overlay"]
    assert "--window-title 'SITL Forge'" in commands["P6-tracking-pid-dry-run"]
    assert "experiments/game_screen_sandbox/phase_runner.py" in commands["P6-synthetic-e2e-dry-run"]
    assert "synthetic-s4-loop.jsonl" in commands["P6-synthetic-log-check"]
    assert "--expected-sign 1" in commands["P5-yaw-live"]
    assert "--expected-sign -1" in commands["P5-pitch-live"]
    assert "pr0p_live_session_runner.py" in commands["P5-uinput-rc-effect-throttle-low"]
    assert "--run-rc-effect" in commands["P5-uinput-rc-effect-throttle-low"]
    assert "--rc-effect-magnitude 1.0" in commands["P5-uinput-rc-effect-throttle-low"]
    assert "--rc-effect-expected-direction lower" in commands["P5-uinput-rc-effect-throttle-low"]
    assert "--run-status-effect" in commands["P5-uinput-status-effect-throttle-clear"]
    assert "--status-effect-blocker THROTTLE" in commands["P5-uinput-status-effect-throttle-clear"]
    assert "--run-arm-button-effect" in commands["P5-uinput-arm-button-effect"]
    assert "--arm-button south" in commands["P5-uinput-arm-button-effect"]
    assert "--arm-button east" in commands["P5-uinput-arm-button-effect"]
    assert "--startup-wait 8" in commands["P5-uinput-arm-button-effect"]
    assert "--role aux1" in commands["P4-input-config-patch-aux1-dry"]
    assert "--ack-config-write" in commands["P4-input-config-patch-aux1-write"]
    assert "--role aux1" in commands["P4-rc-channel-mapping-assistant-aux1-dry"]
    assert "--uinput" not in commands["P4-rc-channel-mapping-assistant-aux1-dry"]
    assert "--role aux1" in commands["P4-rc-channel-mapping-assistant-aux1-live"]
    assert "--uinput" in commands["P4-rc-channel-mapping-assistant-aux1-live"]
    assert "--ack-live-input" in commands["P4-rc-channel-mapping-assistant-aux1-live"]
    assert "--rc-effect-axis aux1" in commands["P5-uinput-rc-effect-aux1-high"]
    assert "--rc-effect-expected-channel aux1" in commands["P5-uinput-rc-effect-aux1-high"]
    assert "--run-aux-arm-status" in commands["P5-uinput-aux1-arm-status"]
    assert "--aux-arm-throttle-magnitude 1.0" in commands["P5-uinput-aux1-arm-status"]
    assert "--aux-arm-aux1-magnitude 1.0" in commands["P5-uinput-aux1-arm-status"]
    assert "--hold-uinput" in commands["P5-persistent-uinput-live-response"]
    assert "--run-live-response" in commands["P5-persistent-uinput-live-response"]
    assert "pr0p_runtime_input_visual_probe.py" in commands["P4-rc-channels-visual"]
    assert "--ack-live-input" in commands["P4-rc-channels-visual"]
    assert "P4-rc-channels-visual" in manifest.next_action


def test_live_manifest_reads_latest_runtime_input_visual_report(tmp_path):
    suite_path = tmp_path / "suite.json"
    suite_path.write_text(json.dumps({
        "status": WAITING,
        "steps": [
            {"phase": "P1-install-discovery", "status": PASS, "summary": "ok"},
            {"phase": "P1-client-executable", "status": PASS, "summary": "ok"},
            {"phase": "P2-websocket", "status": PASS, "summary": "ok"},
            {"phase": "P2-msp-readonly", "status": PASS, "summary": "ok"},
            {"phase": "P2-fc-status-readonly", "status": PASS, "summary": "ok"},
            {"phase": "P2-mode-ranges-readonly", "status": PASS, "summary": "ok"},
            {"phase": "P3-capture", "status": PASS, "summary": "ok"},
            {"phase": "P4-input-readiness", "status": PASS, "summary": "ok"},
            {"phase": "P4-input-mapping", "status": PASS, "summary": "ok"},
            {"phase": "P6-tracking-pid-dry-run", "status": WAITING, "summary": "needs bbox"},
        ],
    }), encoding="utf-8")
    visual_path = tmp_path / "20260707-runtime-input-visual.json"
    visual_path.write_text(json.dumps({
        "status": PASS,
        "summary": "runtime UI changed while the virtual RC pulse was active",
        "notes": ["RUNTIME_INPUT_VISUAL_CHANGE_DETECTED"],
    }), encoding="utf-8")

    manifest = build_manifest(
        run_id="unit",
        log_dir=tmp_path,
        suite_report=suite_path,
        bbox=(1, 2, 30, 40),
        window_title="pr0p",
        yaw_expected_sign=0,
        pitch_expected_sign=0,
    )
    steps = {step.phase: step for step in manifest.steps}
    assert steps["P4-rc-channels-visual"].status == PASS
    assert steps["P4-rc-channels-visual"].evidence == str(visual_path)
    assert "RUNTIME_INPUT_VISUAL_CHANGE_DETECTED" in steps["P4-rc-channels-visual"].notes


def test_live_manifest_prioritizes_fc_arm_blocker_before_p5(tmp_path):
    suite_path = tmp_path / "suite.json"
    suite_path.write_text(json.dumps({
        "status": WAITING,
        "steps": [
            {"phase": "P1-install-discovery", "status": PASS, "summary": "ok"},
            {"phase": "P1-client-executable", "status": PASS, "summary": "ok"},
            {"phase": "P2-websocket", "status": PASS, "summary": "ok"},
            {"phase": "P2-msp-readonly", "status": PASS, "summary": "ok"},
            {
                "phase": "P2-fc-status-readonly",
                "status": PASS,
                "summary": "status ok",
                "notes": ["FC_NOT_ARMED", "ARMING_DISABLED:THROTTLE"],
            },
            {"phase": "P2-mode-ranges-readonly", "status": PASS, "summary": "ok"},
            {"phase": "P3-capture", "status": PASS, "summary": "ok"},
            {"phase": "P4-input-readiness", "status": PASS, "summary": "ok"},
            {"phase": "P4-input-mapping", "status": PASS, "summary": "ok"},
            {"phase": "P6-tracking-pid-dry-run", "status": PASS, "summary": "ok"},
        ],
    }), encoding="utf-8")
    visual_path = tmp_path / "20260707-runtime-input-visual.json"
    visual_path.write_text(json.dumps({
        "status": PASS,
        "summary": "runtime UI changed while the virtual RC pulse was active",
    }), encoding="utf-8")

    manifest = build_manifest(
        run_id="unit",
        log_dir=tmp_path,
        suite_report=suite_path,
        bbox=(1, 2, 30, 40),
        window_title="pr0p",
        yaw_expected_sign=0,
        pitch_expected_sign=0,
    )
    assert "Run P5-uinput-rc-effect-throttle-low" in manifest.next_action
    assert "MSP_RC throttle" in manifest.next_action


def test_live_manifest_reads_nested_live_session_rc_effect(tmp_path):
    suite_path = tmp_path / "suite.json"
    suite_path.write_text(json.dumps({
        "status": WAITING,
        "steps": [
            {"phase": "P1-install-discovery", "status": PASS, "summary": "ok"},
            {"phase": "P1-client-executable", "status": PASS, "summary": "ok"},
            {"phase": "P2-websocket", "status": PASS, "summary": "ok"},
            {"phase": "P2-msp-readonly", "status": PASS, "summary": "ok"},
            {"phase": "P2-fc-status-readonly", "status": PASS, "summary": "ok"},
            {"phase": "P2-mode-ranges-readonly", "status": PASS, "summary": "ok"},
            {"phase": "P3-capture", "status": PASS, "summary": "ok"},
            {"phase": "P4-input-readiness", "status": PASS, "summary": "ok"},
            {"phase": "P4-input-mapping", "status": PASS, "summary": "ok"},
            {"phase": "P6-tracking-pid-dry-run", "status": PASS, "summary": "ok"},
        ],
    }), encoding="utf-8")
    visual_path = tmp_path / "20260707-runtime-input-visual.json"
    visual_path.write_text(json.dumps({"status": PASS, "summary": "visual ok"}), encoding="utf-8")
    session_path = tmp_path / "20260707-live-session.json"
    session_path.write_text(json.dumps({
        "status": PASS,
        "metrics": {
            "live_rc_effect": {
                "status": PASS,
                "summary": "throttle low reached",
                "metrics": {"expected_channel": "throttle"},
                "notes": ["THROTTLE_LOW_REACHED"],
            },
        },
    }), encoding="utf-8")
    aux_session = tmp_path / "20260708-aux-live-session.json"
    aux_session.write_text(json.dumps({
        "status": PASS,
        "metrics": {
            "live_rc_effect": {
                "status": PASS,
                "summary": "aux1 high reached",
                "metrics": {"expected_channel": "aux1"},
                "notes": ["EXPECTED_CHANNEL_EFFECT_DETECTED"],
            },
        },
    }), encoding="utf-8")

    manifest = build_manifest(
        run_id="unit",
        log_dir=tmp_path,
        suite_report=suite_path,
        bbox=(1, 2, 30, 40),
        window_title="pr0p",
        yaw_expected_sign=0,
        pitch_expected_sign=0,
    )
    steps = {step.phase: step for step in manifest.steps}
    assert steps["P5-uinput-rc-effect-throttle-low"].status == PASS
    assert steps["P5-uinput-rc-effect-throttle-low"].evidence == str(session_path)
    assert "THROTTLE_LOW_REACHED" in steps["P5-uinput-rc-effect-throttle-low"].notes


def test_live_manifest_reads_nested_live_session_status_effect(tmp_path):
    suite_path = tmp_path / "suite.json"
    suite_path.write_text(json.dumps({
        "status": WAITING,
        "steps": [
            {"phase": "P1-install-discovery", "status": PASS, "summary": "ok"},
            {"phase": "P1-client-executable", "status": PASS, "summary": "ok"},
            {"phase": "P2-websocket", "status": PASS, "summary": "ok"},
            {"phase": "P2-msp-readonly", "status": PASS, "summary": "ok"},
            {
                "phase": "P2-fc-status-readonly",
                "status": PASS,
                "summary": "status ok",
                "notes": ["FC_NOT_ARMED", "ARMING_DISABLED:THROTTLE"],
            },
            {"phase": "P2-mode-ranges-readonly", "status": PASS, "summary": "ok"},
            {"phase": "P3-capture", "status": PASS, "summary": "ok"},
            {"phase": "P4-input-readiness", "status": PASS, "summary": "ok"},
            {"phase": "P4-input-mapping", "status": PASS, "summary": "ok"},
            {"phase": "P6-tracking-pid-dry-run", "status": PASS, "summary": "ok"},
        ],
    }), encoding="utf-8")
    visual_path = tmp_path / "20260707-runtime-input-visual.json"
    visual_path.write_text(json.dumps({"status": PASS, "summary": "visual ok"}), encoding="utf-8")
    rc_session = tmp_path / "20260707-rc-live-session.json"
    rc_session.write_text(json.dumps({
        "status": PASS,
        "metrics": {
            "live_rc_effect": {
                "status": PASS,
                "summary": "throttle low reached",
                "metrics": {"expected_channel": "throttle"},
                "notes": ["THROTTLE_LOW_REACHED"],
            },
        },
    }), encoding="utf-8")
    status_session = tmp_path / "20260708-status-live-session.json"
    status_session.write_text(json.dumps({
        "status": PASS,
        "metrics": {
            "live_status_effect": {
                "status": PASS,
                "summary": "throttle blocker clear",
                "notes": ["UINPUT_CLEARED_THROTTLE"],
            },
        },
    }), encoding="utf-8")
    aux_session = tmp_path / "20260708-aux-live-session.json"
    aux_session.write_text(json.dumps({
        "status": PASS,
        "metrics": {
            "live_rc_effect": {
                "status": PASS,
                "summary": "aux1 high reached",
                "metrics": {"expected_channel": "aux1"},
                "notes": ["EXPECTED_CHANNEL_EFFECT_DETECTED"],
            },
        },
    }), encoding="utf-8")
    aux_session = tmp_path / "20260708-aux-live-session.json"
    aux_session.write_text(json.dumps({
        "status": PASS,
        "metrics": {
            "live_rc_effect": {
                "status": PASS,
                "summary": "aux1 high reached",
                "metrics": {"expected_channel": "aux1"},
                "notes": ["EXPECTED_CHANNEL_EFFECT_DETECTED"],
            },
        },
    }), encoding="utf-8")

    manifest = build_manifest(
        run_id="unit",
        log_dir=tmp_path,
        suite_report=suite_path,
        bbox=(1, 2, 30, 40),
        window_title="pr0p",
        yaw_expected_sign=0,
        pitch_expected_sign=0,
    )
    steps = {step.phase: step for step in manifest.steps}
    assert steps["P5-uinput-status-effect-throttle-clear"].status == PASS
    assert steps["P5-uinput-status-effect-throttle-clear"].evidence == str(status_session)
    assert "UINPUT_CLEARED_THROTTLE" in steps["P5-uinput-status-effect-throttle-clear"].notes
    assert "P5-uinput-aux1-arm-status" in manifest.next_action


def test_live_manifest_requires_aux_arm_status_after_aux1_rc_effect_without_baseline_blocker(tmp_path):
    suite_path = tmp_path / "suite.json"
    suite_path.write_text(json.dumps({
        "status": WAITING,
        "steps": [
            {"phase": "P1-install-discovery", "status": PASS, "summary": "ok"},
            {"phase": "P1-client-executable", "status": PASS, "summary": "ok"},
            {"phase": "P2-websocket", "status": PASS, "summary": "ok"},
            {"phase": "P2-msp-readonly", "status": PASS, "summary": "ok"},
            {"phase": "P2-fc-status-readonly", "status": PASS, "summary": "status ok"},
            {"phase": "P2-mode-ranges-readonly", "status": PASS, "summary": "ok"},
            {"phase": "P3-capture", "status": PASS, "summary": "ok"},
            {"phase": "P4-input-readiness", "status": PASS, "summary": "ok"},
            {"phase": "P4-input-mapping", "status": PASS, "summary": "ok"},
            {"phase": "P6-tracking-pid-dry-run", "status": PASS, "summary": "ok"},
        ],
    }), encoding="utf-8")
    visual_path = tmp_path / "20260707-runtime-input-visual.json"
    visual_path.write_text(json.dumps({"status": PASS, "summary": "visual ok"}), encoding="utf-8")
    session_path = tmp_path / "20260708-live-session.json"
    session_path.write_text(json.dumps({
        "status": PASS,
        "metrics": {
            "live_rc_effect": {
                "status": PASS,
                "summary": "aux1 high reached",
                "metrics": {"expected_channel": "aux1"},
                "notes": ["EXPECTED_CHANNEL_EFFECT_DETECTED"],
            },
            "live_status_effect": {
                "status": PASS,
                "summary": "throttle blocker clear",
                "notes": ["UINPUT_CLEARED_THROTTLE"],
            },
        },
    }), encoding="utf-8")
    throttle_session = tmp_path / "20260707-throttle-live-session.json"
    throttle_session.write_text(json.dumps({
        "status": PASS,
        "metrics": {
            "live_rc_effect": {
                "status": PASS,
                "summary": "throttle low reached",
                "metrics": {"expected_channel": "throttle"},
                "notes": ["THROTTLE_LOW_REACHED"],
            },
        },
    }), encoding="utf-8")

    manifest = build_manifest(
        run_id="unit",
        log_dir=tmp_path,
        suite_report=suite_path,
        bbox=(1, 2, 30, 40),
        window_title="pr0p",
        yaw_expected_sign=0,
        pitch_expected_sign=0,
    )
    steps = {step.phase: step for step in manifest.steps}
    assert steps["P5-uinput-rc-effect-aux1-high"].status == PASS
    assert "AUX1/CH5 moves" in manifest.next_action
    assert "P5-uinput-aux1-arm-status" in manifest.next_action


def test_live_manifest_reads_nested_live_session_arm_effect(tmp_path):
    suite_path = tmp_path / "suite.json"
    suite_path.write_text(json.dumps({
        "status": WAITING,
        "steps": [
            {"phase": "P1-install-discovery", "status": PASS, "summary": "ok"},
            {"phase": "P1-client-executable", "status": PASS, "summary": "ok"},
            {"phase": "P2-websocket", "status": PASS, "summary": "ok"},
            {"phase": "P2-msp-readonly", "status": PASS, "summary": "ok"},
            {
                "phase": "P2-fc-status-readonly",
                "status": PASS,
                "summary": "status ok",
                "notes": ["FC_NOT_ARMED", "ARMING_DISABLED:THROTTLE"],
            },
            {"phase": "P2-mode-ranges-readonly", "status": PASS, "summary": "ok"},
            {"phase": "P3-capture", "status": PASS, "summary": "ok"},
            {"phase": "P4-input-readiness", "status": PASS, "summary": "ok"},
            {"phase": "P4-input-mapping", "status": PASS, "summary": "ok"},
            {"phase": "P6-tracking-pid-dry-run", "status": PASS, "summary": "ok"},
        ],
    }), encoding="utf-8")
    visual_path = tmp_path / "20260707-runtime-input-visual.json"
    visual_path.write_text(json.dumps({"status": PASS, "summary": "visual ok"}), encoding="utf-8")
    rc_session = tmp_path / "20260707-rc-live-session.json"
    rc_session.write_text(json.dumps({
        "status": PASS,
        "metrics": {
            "live_rc_effect": {
                "status": PASS,
                "summary": "throttle low reached",
                "metrics": {"expected_channel": "throttle"},
                "notes": ["THROTTLE_LOW_REACHED"],
            },
            "live_status_effect": {
                "status": PASS,
                "summary": "throttle blocker clear",
                "notes": ["UINPUT_CLEARED_THROTTLE"],
            },
            "live_arm_effect": {
                "status": PASS,
                "summary": "ARM mode active",
                "notes": ["UINPUT_ARM_BUTTON_DETECTED:south"],
            },
        },
    }), encoding="utf-8")
    aux_session = tmp_path / "20260708-aux-live-session.json"
    aux_session.write_text(json.dumps({
        "status": PASS,
        "metrics": {
            "live_rc_effect": {
                "status": PASS,
                "summary": "aux1 high reached",
                "metrics": {"expected_channel": "aux1"},
                "notes": ["EXPECTED_CHANNEL_EFFECT_DETECTED"],
            },
        },
    }), encoding="utf-8")
    aux_arm_status_session = tmp_path / "20260709-aux-arm-status-live-session.json"
    aux_arm_status_session.write_text(json.dumps({
        "status": PASS,
        "metrics": {
            "live_aux_arm_status": {
                "status": PASS,
                "summary": "AUX1 high armed the FC",
                "notes": ["UINPUT_AUX1_ARMED", "FC_ARMED"],
            },
        },
    }), encoding="utf-8")

    manifest = build_manifest(
        run_id="unit",
        log_dir=tmp_path,
        suite_report=suite_path,
        bbox=(1, 2, 30, 40),
        window_title="pr0p",
        yaw_expected_sign=0,
        pitch_expected_sign=0,
    )
    steps = {step.phase: step for step in manifest.steps}
    assert steps["P5-uinput-arm-button-effect"].status == PASS
    assert steps["P5-uinput-arm-button-effect"].evidence == str(rc_session)
    assert steps["P5-uinput-rc-effect-aux1-high"].status == PASS
    assert steps["P5-uinput-rc-effect-aux1-high"].evidence == str(aux_session)
    assert steps["P5-uinput-aux1-arm-status"].status == PASS
    assert steps["P5-uinput-aux1-arm-status"].evidence == str(aux_arm_status_session)
    assert "UINPUT_ARM_BUTTON_DETECTED:south" in steps["P5-uinput-arm-button-effect"].notes
    assert "P5-response-acceptance" in manifest.next_action


def test_live_manifest_rejects_stale_standalone_acceptance_evidence(tmp_path):
    suite_path = tmp_path / "suite.json"
    suite_path.write_text(json.dumps({
        "status": PASS,
        "steps": [
            {"phase": "P1-install-discovery", "status": PASS, "summary": "ok"},
            {"phase": "P1-client-executable", "status": PASS, "summary": "ok"},
            {"phase": "P2-websocket", "status": PASS, "summary": "ok"},
            {"phase": "P2-msp-readonly", "status": PASS, "summary": "ok"},
            {"phase": "P2-fc-status-readonly", "status": PASS, "summary": "ok"},
            {"phase": "P2-mode-ranges-readonly", "status": PASS, "summary": "ok"},
            {"phase": "P3-capture", "status": PASS, "summary": "ok"},
            {"phase": "P4-input-readiness", "status": PASS, "summary": "ok"},
            {"phase": "P4-input-mapping", "status": PASS, "summary": "ok"},
            {"phase": "P4-rc-channels-visual", "status": PASS, "summary": "ok"},
            {"phase": "P5-yaw-live", "status": PASS, "summary": "ok"},
            {"phase": "P5-pitch-live", "status": PASS, "summary": "ok"},
            {"phase": "P5-uinput-rc-effect-throttle-low", "status": PASS, "summary": "ok"},
            {"phase": "P5-uinput-status-effect-throttle-clear", "status": PASS, "summary": "ok"},
            {"phase": "P5-uinput-arm-button-effect", "status": PASS, "summary": "ok"},
            {"phase": "P5-uinput-rc-effect-aux1-high", "status": PASS, "summary": "ok"},
            {"phase": "P5-uinput-aux1-arm-status", "status": PASS, "summary": "ok"},
            {"phase": "P5-persistent-uinput-live-response", "status": PASS, "summary": "ok"},
            {"phase": "P5-response-acceptance", "status": PASS, "summary": "ok"},
            {"phase": "P6-tracking-pid-dry-run", "status": PASS, "summary": "ok"},
            {"phase": "P6-tracking-live", "status": PASS, "summary": "ok"},
            {"phase": "P6-tracking-acceptance", "status": PASS, "summary": "ok"},
        ],
    }), encoding="utf-8")
    response_path = tmp_path / "20260707-unit-response-acceptance.json"
    tracking_path = tmp_path / "20260707-unit-tracking-acceptance.json"
    write_response_acceptance(response_path)
    write_tracking_acceptance(tracking_path)
    os.utime(response_path, ns=(1_000_000_000, 1_000_000_000))
    os.utime(tracking_path, ns=(1_000_000_000, 1_000_000_000))

    manifest = build_manifest(
        run_id="unit",
        log_dir=tmp_path,
        suite_report=suite_path,
        bbox=(1, 2, 30, 40),
        window_title="pr0p",
        yaw_expected_sign=0,
        pitch_expected_sign=0,
        max_acceptance_evidence_age_s=60.0,
    )

    steps = {step.phase: step for step in manifest.steps}
    assert manifest.status == WAITING
    assert manifest.metadata["max_acceptance_evidence_age_s"] == 60.0
    assert steps["P5-response-acceptance"].status == WAITING
    assert steps["P5-response-acceptance"].summary == "latest acceptance evidence is stale"
    assert steps["P5-response-acceptance"].evidence == str(response_path)
    assert "STALE_RESPONSE_ACCEPTANCE" in steps["P5-response-acceptance"].notes
    assert steps["P6-tracking-acceptance"].status == WAITING
    assert steps["P6-tracking-acceptance"].summary == "latest acceptance evidence is stale"
    assert steps["P6-tracking-acceptance"].evidence == str(tracking_path)
    assert "STALE_TRACKING_ACCEPTANCE" in steps["P6-tracking-acceptance"].notes


def test_live_manifest_rejects_negative_acceptance_evidence_age():
    with pytest.raises(SystemExit) as excinfo:
        parse_live_manifest_args(["--max-acceptance-evidence-age-s", "-1"])

    assert excinfo.value.code == 2


def test_live_manifest_mentions_rc_channel_mapping_when_offline_gate_knows_it(tmp_path):
    suite_path = tmp_path / "suite.json"
    suite_path.write_text(json.dumps({
        "status": WAITING,
        "steps": [
            {"phase": "P1-install-discovery", "status": PASS, "summary": "ok"},
            {"phase": "P1-client-executable", "status": PASS, "summary": "ok"},
            {"phase": "P2-websocket", "status": WAITING, "summary": "closed"},
            {"phase": "P4-input-mapping", "status": WAITING, "summary": "needs remap"},
        ],
    }), encoding="utf-8")
    manifest = build_manifest(
        run_id="unit",
        log_dir=tmp_path,
        suite_report=suite_path,
        bbox=None,
        window_title="pr0p",
        yaw_expected_sign=0,
        pitch_expected_sign=0,
    )
    assert "Controls -> RC Channels" in manifest.next_action
    assert "input config patch dry-run" in manifest.next_action
    commands = {step.phase: step.command for step in manifest.steps}
    assert "pr0p_input_config_patch.py" in commands["P4-input-config-patch-dry"]
    assert "--ack-config-write" in commands["P4-input-config-patch-write"]
    assert "pr0p_input_config_restore.py" in commands["P4-input-config-restore-latest"]
    assert "--ack-config-restore" in commands["P4-input-config-restore-write"]
    assert "pr0p_live_session_runner.py" in commands["P1-live-session-runner"]
    assert "--ack-live-launch" in commands["P1-live-session-runner"]
    assert "--ack-live-ui" in commands["P1-live-session-runner"]


def write_game_decision(
    path,
    *,
    decision=GAME_SCREEN_READY,
    scope=GAME_SCREEN_SCOPE,
    current_schema=True,
):
    metrics = {"scope": scope}
    if current_schema:
        metrics.update({
            "synthetic_required_phases": list(GAME_SCREEN_SYNTHETIC_REQUIRED_PHASES),
            "simple_window_required_phases": list(GAME_SCREEN_SIMPLE_WINDOW_REQUIRED_PHASES),
            "synthetic_statuses": {
                phase: PASS for phase in GAME_SCREEN_SYNTHETIC_REQUIRED_PHASES
            },
            "simple_window_phase_statuses": {
                phase: PASS for phase in GAME_SCREEN_SIMPLE_WINDOW_REQUIRED_PHASES
            },
            "simple_window_status": PASS,
        })
    path.write_text(json.dumps({
        "decision": decision,
        "summary": "game-screen decision",
        "metrics": metrics,
    }), encoding="utf-8")


def write_pr0p_suite(path, steps):
    path.write_text(json.dumps({
        "status": WAITING if any(step["status"] == WAITING for step in steps) else PASS,
        "steps": steps,
    }), encoding="utf-8")


def write_pr0p_manifest(path, steps, *, suite_report=None):
    data = {
        "status": WAITING if any(step["status"] == WAITING for step in steps) else PASS,
        "steps": steps,
    }
    if suite_report is not None:
        data.update({
            "suite_report": str(suite_report),
            "next_action": "ready for decision",
            "metadata": {"source": "unit"},
        })
    path.write_text(json.dumps(data), encoding="utf-8")


def write_response_acceptance(path, *, status=PASS, yaw_status=PASS, pitch_status=PASS):
    path.write_text(json.dumps({
        "status": status,
        "steps": [
            {"name": "pr0p_response_prerequisites", "status": PASS},
            {"name": "pr0p_yaw_response_live", "status": yaw_status},
            {"name": "pr0p_pitch_response_live", "status": pitch_status},
        ],
    }), encoding="utf-8")


def write_aux1_acceptance(path, *, status=PASS, rc_status=PASS, arm_status=PASS):
    path.write_text(json.dumps({
        "status": status,
        "summary": "aux1 acceptance",
        "steps": [
            {"name": "pr0p_aux1_rc_effect", "status": rc_status},
            {"name": "pr0p_aux1_arm_status", "status": arm_status},
        ],
    }), encoding="utf-8")


def write_tracking_acceptance(
    path,
    *,
    status=PASS,
    live_status=PASS,
    duration_s=4.0,
    min_found_ratio=0.85,
    max_loss_events=0,
    run_live_gates=True,
    arm_first=True,
    enable_pitch=False,
    desired_target_width=120.0,
    max_abs_pitch_axis=0.8,
):
    live_command = (
        "fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_tracking_probe.py "
        "--bbox 10,20,80,60 "
        "--duration %s "
        "--min-found-ratio %s "
        "--max-loss-events %s "
        "--max-abs-pitch-axis %s "
        "--desired-target-width %s "
        "--uinput "
        "--ack-live-input"
    ) % (duration_s, min_found_ratio, max_loss_events, max_abs_pitch_axis, desired_target_width)
    if enable_pitch:
        live_command = "%s --enable-pitch" % live_command
    if arm_first:
        live_command = "%s --arm-first --arm-throttle -0.4" % live_command
    path.write_text(json.dumps({
        "status": status,
        "summary": "tracking acceptance",
        "steps": [
            {"name": "pr0p_tracking_bbox", "status": PASS},
            {"name": "pr0p_tracking_response_prerequisites", "status": PASS},
            {
                "name": "pr0p_tracking_live",
                "status": live_status,
                "command": live_command,
                "report": "logs/tracking-live.json",
            },
        ],
        "metrics": {
            "bbox": [10, 20, 80, 60],
            "duration_s": duration_s,
            "min_found_ratio": min_found_ratio,
            "max_loss_events": max_loss_events,
            "run_live_gates": run_live_gates,
            "enable_pitch": enable_pitch,
            "desired_target_width": desired_target_width,
            "max_abs_pitch_axis": max_abs_pitch_axis,
        },
    }), encoding="utf-8")


def write_moving_target_yaw(
    path,
    *,
    status=WAITING,
    synthetic_status=PASS,
    real_status=WAITING,
):
    path.write_text(json.dumps({
        "status": status,
        "summary": "moving target yaw",
        "steps": [
            {
                "name": "synthetic_moving_target_yaw",
                "status": synthetic_status,
                "metrics": {"real_input": False, "enable_pitch": False},
            },
            {
                "name": "real_pr0p_moving_target_yaw",
                "status": real_status,
                "metrics": {"real_input": True, "enable_pitch": False},
            },
        ],
        "metrics": {
            "synthetic_status": synthetic_status,
            "real_pr0p_status": real_status,
            "real_input_sent": False,
        },
    }), encoding="utf-8")


def write_moving_target_acceptance(
    path,
    *,
    status=PASS,
    tracking_status=PASS,
    real_status=PASS,
    tracking_report=None,
):
    path.write_text(json.dumps({
        "status": status,
        "summary": "real airborne static-target yaw evidence",
        "steps": [
            {"name": "moving_target_bbox", "status": PASS},
            {"name": "extended_follow_prerequisite", "status": PASS},
            {
                "name": "moving_target_tracking_probe",
                "status": tracking_status,
                "report": str(tracking_report) if tracking_report else None,
            },
            {
                "name": "real_pr0p_moving_target_yaw",
                "status": real_status,
            },
        ],
        "metrics": {
            "tracking_report": str(tracking_report) if tracking_report else None,
            "real_status": real_status,
            "real_input_sent": False,
            "evidence_only": True,
        },
    }), encoding="utf-8")


def write_approach_pitch(
    path,
    *,
    status=WAITING,
    synthetic_status=PASS,
    real_status=WAITING,
):
    path.write_text(json.dumps({
        "status": status,
        "summary": "approach pitch",
        "steps": [
            {
                "name": "synthetic_range_approach",
                "status": synthetic_status,
                "metrics": {"real_input": False, "enable_pitch": True},
            },
            {
                "name": "real_pr0p_approach_pitch",
                "status": real_status,
                "metrics": {"real_input": True, "enable_pitch": True},
            },
        ],
        "metrics": {
            "synthetic_status": synthetic_status,
            "real_pr0p_status": real_status,
            "real_input_sent": False,
        },
    }), encoding="utf-8")


def write_handoff(
    path,
    *,
    status=WAITING,
    synthetic_status=PASS,
    real_status=WAITING,
):
    path.write_text(json.dumps({
        "status": status,
        "summary": "manual-to-autonomous handoff",
        "steps": [
            {
                "name": "synthetic_manual_to_auto_handoff",
                "status": synthetic_status,
                "metrics": {
                    "real_input": False,
                    "manual_to_auto_handoff": True,
                },
            },
            {
                "name": "real_pr0p_manual_to_auto_handoff",
                "status": real_status,
                "metrics": {
                    "real_input": True,
                    "manual_to_auto_handoff": True,
                },
            },
        ],
        "metrics": {
            "synthetic_status": synthetic_status,
            "real_pr0p_status": real_status,
            "real_input_sent": False,
        },
    }), encoding="utf-8")


def write_real_moving_tracking_evidence(
    tmp_path,
    *,
    target_motion=42.0,
    initial_error=100.0,
    final_error=20.0,
    screen_fixed=False,
    post_loop_armed=True,
):
    live_probe = tmp_path / "live-tracking-probe.json"
    reduction_px = initial_error - final_error
    reduction_ratio = reduction_px / initial_error if initial_error else 0.0
    live_probe.write_text(json.dumps({
        "status": PASS,
        "summary": "tracker/PID loop stayed bounded",
        "metrics": {
            "found_ratio": 0.96,
            "loss_events": 0,
            "target_center_span_px": target_motion,
            "target_center_travel_px": target_motion,
            "initial_abs_horizontal_error": initial_error,
            "last_abs_horizontal_error": final_error,
            "horizontal_error_reduction_px": reduction_px,
            "horizontal_error_reduction_ratio": reduction_ratio,
            "max_abs_yaw_axis": 0.3,
            "max_abs_pitch_axis": 0.0,
            "enable_pitch": False,
            "real_input_sent": True,
            "post_loop_fc": {"armed": post_loop_armed, "active_modes": ["ARM"] if post_loop_armed else []},
            "screen_fixed_lock_suspected": screen_fixed,
        },
        "notes": ["SCREEN_FIXED_LOCK_SUSPECTED"] if screen_fixed else [],
    }), encoding="utf-8")
    acceptance = tmp_path / "real-moving-tracking-acceptance.json"
    acceptance.write_text(json.dumps({
        "status": PASS,
        "summary": "live tracker/PID gate passed",
        "steps": [
            {"name": "pr0p_tracking_bbox", "status": PASS},
            {"name": "pr0p_tracking_response_prerequisites", "status": PASS},
            {
                "name": "pr0p_tracking_live",
                "status": PASS,
                "report": str(live_probe),
            },
        ],
        "metrics": {
            "duration_s": 30.0,
            "min_found_ratio": 0.9,
            "max_loss_events": 2,
            "run_live_gates": True,
        },
    }), encoding="utf-8")
    return acceptance


def build_moving_target_acceptance_for_test(tmp_path, **overrides):
    kwargs = {
        "run_id": "unit-moving-acceptance",
        "log_dir": tmp_path,
        "bbox": (10, 20, 80, 60),
        "run_live_gate": False,
        "ack_live_input": False,
        "ack_real_moving_target": False,
        "tracking_report": None,
        "real_duration_s": 30.0,
        "real_min_found_ratio": 0.9,
        "real_max_loss_events": 2,
        "real_min_target_motion_px": 30.0,
        "real_max_yaw_axis": 0.8,
        "real_min_center_error_reduction_ratio": 0.5,
        "real_max_final_center_error_px": 40.0,
        "real_min_initial_center_error_px": 30.0,
        "takeoff_delay_frames": 10,
        "takeoff_boost_frames": 10,
        "settle_throttle": -0.26,
        "timeout_s": 1.0,
    }
    kwargs.update(overrides)
    return build_moving_target_acceptance(**kwargs)


def write_real_approach_tracking_evidence(tmp_path):
    live_probe = tmp_path / "live-approach-tracking-probe.json"
    live_probe.write_text(json.dumps({
        "status": PASS,
        "summary": "tracker/PID loop stayed bounded",
        "metrics": {
            "found_ratio": 0.96,
            "loss_events": 0,
            "initial_target_width": 70.0,
            "last_target_width": 116.0,
            "max_abs_yaw_axis": 0.3,
            "max_abs_pitch_axis": 0.35,
            "enable_pitch": True,
            "real_input_sent": True,
        },
    }), encoding="utf-8")
    acceptance = tmp_path / "real-approach-tracking-acceptance.json"
    acceptance.write_text(json.dumps({
        "status": PASS,
        "summary": "live tracker/PID gate passed",
        "steps": [
            {"name": "pr0p_tracking_bbox", "status": PASS},
            {"name": "pr0p_tracking_response_prerequisites", "status": PASS},
            {
                "name": "pr0p_tracking_live",
                "status": PASS,
                "report": str(live_probe),
            },
        ],
        "metrics": {
            "duration_s": 20.0,
            "min_found_ratio": 0.9,
            "max_loss_events": 2,
            "run_live_gates": True,
        },
    }), encoding="utf-8")
    return acceptance


def write_real_handoff_evidence(tmp_path):
    report = tmp_path / "real-handoff-report.json"
    report.write_text(json.dumps({
        "status": PASS,
        "summary": "real manual-to-autonomous handoff passed",
        "metrics": {
            "follow_command_sent": True,
            "tracker_initialized_before_follow": False,
            "tracker_initialized_after_follow": True,
            "pre_handoff_auto_control_samples": 0,
            "manual_to_auto_handoff": True,
            "real_input_sent": True,
            "found_ratio": 0.96,
            "loss_events": 0,
            "manual_final_abs_error_px": 32.0,
            "final_abs_error_px": 18.0,
            "final_abs_width_error_px": 6.0,
            "manual_to_final_error_reduction_ratio": 0.72,
            "width_error_reduction_ratio": 0.58,
            "max_abs_yaw_axis": 0.35,
            "max_abs_pitch_axis": 0.30,
        },
    }), encoding="utf-8")
    return report


def write_real_handoff_acceptance(path, *, status=PASS):
    path.write_text(json.dumps({
        "status": status,
        "summary": "real handoff evidence is ready for handoff_plan",
        "steps": [
            {"name": "handoff_bbox", "status": PASS},
            {"name": "approach_pitch_prerequisite", "status": PASS},
            {
                "name": "follow_command_tracking_probe",
                "status": PASS,
                "report": "logs/follow-tracking-probe.json",
            },
            {
                "name": "real_handoff_report",
                "status": PASS,
                "report": "logs/follow-tracking-probe.json",
            },
        ],
        "metrics": {
            "follow_command_sent": True,
            "tracker_initialized_before_follow": False,
            "tracker_initialized_after_follow": True,
            "pre_handoff_auto_control_samples": 0,
            "manual_to_auto_handoff": True,
            "real_input_sent": False,
            "evidence_only": True,
            "found_ratio": 0.96,
            "loss_events": 0,
            "manual_final_abs_error_px": 32.0,
            "final_abs_error_px": 18.0,
            "final_abs_width_error_px": 6.0,
            "manual_to_final_error_reduction_ratio": 0.72,
            "width_error_reduction_ratio": 0.58,
            "max_abs_yaw_axis": 0.35,
            "max_abs_pitch_axis": 0.30,
        },
    }), encoding="utf-8")


def build_handoff_plan_for_test(tmp_path, **overrides):
    kwargs = {
        "run_id": "unit-handoff",
        "log_dir": tmp_path,
        "bbox": (10, 20, 80, 60),
        "execute_synthetic": False,
        "duration_s": 6.0,
        "hz": 20.0,
        "manual_duration_s": 1.6,
        "min_found_ratio": 0.90,
        "max_loss_events": 0,
        "initial_target_width": 70.0,
        "desired_target_width": 120.0,
        "max_handoff_error": 45.0,
        "max_final_error": 35.0,
        "max_final_width_error": 8.0,
        "min_error_reduction": 0.65,
        "min_manual_to_final_reduction": 0.80,
        "min_width_error_reduction": 0.65,
        "max_yaw_axis": 0.8,
        "max_pitch_axis": 0.8,
        "real_handoff_report": None,
        "ack_real_handoff": False,
    }
    kwargs.update(overrides)
    return build_handoff_plan(**kwargs)


def write_handoff_tracking_probe(path, *, status=PASS, real_input_sent=True):
    path.write_text(json.dumps({
        "status": status,
        "summary": "tracker/PID loop stayed bounded",
        "metrics": {
            "target_found": 20,
            "found_ratio": 0.96,
            "loss_events": 0,
            "max_abs_yaw_axis": 0.35,
            "max_abs_pitch_axis": 0.30,
            "real_input_sent": real_input_sent,
            "capture_region": {"x": 0, "y": 0, "width": 640, "height": 480},
            "last_target_center": [320.0, 240.0],
            "initial_target_width": 70.0,
            "last_target_width": 116.0,
        },
    }), encoding="utf-8")
    return path


def build_handoff_acceptance_for_test(tmp_path, **overrides):
    kwargs = {
        "run_id": "unit-real-handoff",
        "log_dir": tmp_path,
        "bbox": (10, 20, 80, 60),
        "window_titles": ["pr0p"],
        "duration_s": 20.0,
        "hz": 10.0,
        "tracker": "CSRT",
        "min_found_ratio": 0.90,
        "max_loss_events": 2,
        "max_abs_yaw_axis": 0.8,
        "max_abs_pitch_axis": 0.8,
        "desired_target_width": 120.0,
        "arm_throttle": -0.4,
        "run_live_gate": False,
        "ack_live_input": False,
        "ack_real_handoff": False,
        "tracking_report": None,
        "timeout_s": 1.0,
    }
    kwargs.update(overrides)
    return build_handoff_acceptance(**kwargs)


def write_rc_manual_flight(path, *, status=PASS, aux_status=PASS):
    manual_response_status = PASS if status == PASS and aux_status == PASS else "SKIPPED"
    path.write_text(json.dumps({
        "status": status,
        "summary": "RC/manual-flight readiness is proven" if status == PASS else "AUX1/CH5 arm channel is not ready",
        "stages": [
            {"name": "pr0p_client", "status": PASS, "summary": "client ok"},
            {"name": "primary_rc_mapping", "status": PASS, "summary": "mapping ok"},
            {"name": "aux1_arm_acceptance", "status": aux_status, "summary": "aux1"},
            {"name": "manual_yaw_response", "status": manual_response_status, "summary": "yaw"},
            {"name": "manual_pitch_response", "status": manual_response_status, "summary": "pitch"},
            {"name": "manual_roll_response", "status": manual_response_status, "summary": "roll"},
        ],
    }), encoding="utf-8")


def readiness_promotion_steps():
    return [
        {"phase": "P0-isolation", "status": PASS},
        {"phase": "P0-preflight", "status": PASS},
        {"phase": "P1-install-discovery", "status": PASS},
        {"phase": "P1-client-executable", "status": PASS},
        {"phase": "P2-websocket", "status": PASS},
        {"phase": "P2-msp-readonly", "status": PASS},
        {"phase": "P2-fc-status-readonly", "status": PASS},
        {"phase": "P2-mode-ranges-readonly", "status": PASS},
        {"phase": "P3-capture", "status": PASS},
        {"phase": "P4-input-readiness", "status": PASS},
        {"phase": "P4-input-mapping", "status": PASS},
        {"phase": "P4-rc-channels-visual", "status": PASS},
        {"phase": "P5-yaw-live", "status": PASS},
        {"phase": "P5-pitch-live", "status": PASS},
        {"phase": "P5-uinput-rc-effect-throttle-low", "status": PASS},
        {"phase": "P5-uinput-status-effect-throttle-clear", "status": PASS},
        {"phase": "P5-uinput-arm-button-effect", "status": PASS},
        {"phase": "P5-uinput-rc-effect-aux1-high", "status": PASS},
        {"phase": "P5-uinput-aux1-arm-status", "status": PASS},
        {"phase": "P5-rc-loopback", "status": PASS},
        {"phase": "P5-persistent-uinput-live-response", "status": PASS},
        {"phase": "P5-response-acceptance", "status": PASS},
        {"phase": "P6-synthetic-e2e-dry-run", "status": PASS},
        {"phase": "P6-synthetic-log-check", "status": PASS},
        {"phase": "P6-tracking-pid-dry-run", "status": PASS},
        {"phase": "P6-tracking-log-check", "status": PASS},
        {"phase": "P6-tracking-live", "status": PASS},
        {"phase": "P6-tracking-acceptance", "status": PASS},
    ]


def test_independent_readiness_filters_non_game_screen_decisions(tmp_path):
    good = tmp_path / "20260705-good-decision.json"
    noisy = tmp_path / "20260706-status-decision.json"
    write_game_decision(good)
    write_game_decision(noisy, decision=WAITING, scope="status-only external target acceptance")
    os.utime(good, ns=(1_000_000_000, 1_000_000_000))
    os.utime(noisy, ns=(2_000_000_000, 2_000_000_000))

    path, data = latest_game_screen_decision(tmp_path)
    assert path == good
    assert data["decision"] == GAME_SCREEN_READY


def test_independent_readiness_waits_on_live_pr0p_evidence(tmp_path):
    game_dir = tmp_path / "game"
    pr0p_dir = tmp_path / "pr0p"
    game_dir.mkdir()
    pr0p_dir.mkdir()
    write_game_decision(game_dir / "20260705-game-decision.json")
    write_pr0p_suite(pr0p_dir / "20260707-pr0p-suite.json", [
        {"phase": "P0-isolation", "status": PASS},
        {"phase": "P1-install-discovery", "status": PASS},
        {"phase": "P1-client-executable", "status": PASS},
        {"phase": "P2-websocket", "status": WAITING},
        {"phase": "P2-msp-readonly", "status": WAITING},
        {"phase": "P3-capture", "status": WAITING},
        {"phase": "P4-input-readiness", "status": PASS},
        {"phase": "P4-input-mapping", "status": PASS},
        {"phase": "P6-synthetic-e2e-dry-run", "status": PASS},
        {"phase": "P6-synthetic-log-check", "status": PASS},
        {"phase": "P6-tracking-pid-dry-run", "status": WAITING},
        {"phase": "P6-tracking-log-check", "status": WAITING},
    ])

    report = build_readiness(
        run_id="unit",
        game_log_dir=game_dir,
        pr0p_log_dir=pr0p_dir,
        bbox=None,
        window_title="pr0p",
        yaw_expected_sign=0,
        pitch_expected_sign=0,
    )
    assert report.status == WAITING
    assert report.metrics["game_screen_decision"] == GAME_SCREEN_READY
    assert report.metrics["pr0p_decision"] == WAITING
    assert report.metrics["evidence_freshness"]["game_screen_decision"]["status"] == PASS
    assert report.metrics["evidence_freshness"]["pr0p_suite"]["status"] == PASS
    assert "PR0P:MISSING_LIVE_PR0P_WEBSOCKET" in report.reasons
    assert report.next_action == "Start pr0p, enter a local race, then rerun the safe suite."
    assert "sandbox_acceptance_runner.py" in report.commands["game_screen_acceptance"]


def test_independent_readiness_prioritizes_install_before_live_pr0p(tmp_path):
    game_dir = tmp_path / "game"
    pr0p_dir = tmp_path / "pr0p"
    game_dir.mkdir()
    pr0p_dir.mkdir()
    write_game_decision(game_dir / "20260705-game-decision.json")
    write_pr0p_suite(pr0p_dir / "20260707-pr0p-suite.json", [
        {"phase": "P0-isolation", "status": PASS},
        {
            "phase": "P1-install-discovery",
            "status": WAITING,
            "summary": "Linux updater discovered; rerun with --download to place it in the install root",
        },
        {"phase": "P2-websocket", "status": WAITING},
        {"phase": "P2-msp-readonly", "status": WAITING},
        {"phase": "P3-capture", "status": WAITING},
        {"phase": "P4-input-readiness", "status": PASS},
        {"phase": "P4-input-mapping", "status": PASS},
        {"phase": "P6-synthetic-e2e-dry-run", "status": PASS},
        {"phase": "P6-synthetic-log-check", "status": PASS},
        {"phase": "P6-tracking-pid-dry-run", "status": WAITING},
        {"phase": "P6-tracking-log-check", "status": WAITING},
    ])

    report = build_readiness(
        run_id="unit",
        game_log_dir=game_dir,
        pr0p_log_dir=pr0p_dir,
        bbox=None,
        window_title="pr0p",
        yaw_expected_sign=0,
        pitch_expected_sign=0,
    )
    assert report.status == WAITING
    assert report.next_action == (
        "Download or install pr0p in the isolated root first "
        "(P1-install-discovery: Linux updater discovered; rerun with --download "
        "to place it in the install root), then rerun the safe suite."
    )
    assert report.metrics["pr0p_phase_details"]["P1-install-discovery"]["status"] == WAITING
    assert "pr0p_install_probe.py" in report.commands["pr0p_install_discovery"]
    assert "--download" in report.commands["pr0p_install_download"]
    assert "pr0p_live_session_runner.py" in report.commands["pr0p_live_session_dry"]
    assert "--role aux1" in report.commands["pr0p_aux1_config_patch_dry"]
    assert "--ack-config-write" in report.commands["pr0p_aux1_config_patch_write"]
    assert "--role aux1" in report.commands["pr0p_aux1_mapping_assistant_dry"]
    assert "--uinput" not in report.commands["pr0p_aux1_mapping_assistant_dry"]
    assert "--role aux1" in report.commands["pr0p_aux1_mapping_assistant_live"]
    assert "--uinput" in report.commands["pr0p_aux1_mapping_assistant_live"]
    assert "--ack-live-input" in report.commands["pr0p_aux1_mapping_assistant_live"]


def test_independent_readiness_waits_for_client_executable_after_updater(tmp_path):
    game_dir = tmp_path / "game"
    pr0p_dir = tmp_path / "pr0p"
    game_dir.mkdir()
    pr0p_dir.mkdir()
    write_game_decision(game_dir / "20260705-game-decision.json")
    write_pr0p_suite(pr0p_dir / "20260707-pr0p-suite.json", [
        {"phase": "P0-isolation", "status": PASS},
        {
            "phase": "P1-install-discovery",
            "status": PASS,
            "summary": "Linux updater is already present in the isolated install root",
            "metrics": {
                "install_scan": {
                    "client_candidates": [],
                    "executables": [{"name": "updater", "executable": True}],
                },
            },
        },
        {"phase": "P2-websocket", "status": WAITING},
        {"phase": "P2-msp-readonly", "status": WAITING},
        {"phase": "P3-capture", "status": WAITING},
        {"phase": "P4-input-readiness", "status": PASS},
        {"phase": "P4-input-mapping", "status": PASS},
        {"phase": "P6-synthetic-e2e-dry-run", "status": PASS},
        {"phase": "P6-synthetic-log-check", "status": PASS},
    ])

    report = build_readiness(
        run_id="unit",
        game_log_dir=game_dir,
        pr0p_log_dir=pr0p_dir,
        bbox=None,
        window_title="pr0p",
        yaw_expected_sign=0,
        pitch_expected_sign=0,
    )
    assert report.status == WAITING
    assert report.next_action == (
        "Run the pr0p updater runner dry-run, then launch the updater with "
        "explicit ack or manually, and rerun the pr0p client probe until a "
        "client executable is visible."
    )
    assert "pr0p_updater_runner.py" in report.commands["pr0p_updater_runner_dry"]
    assert "--ack-external-binary" in report.commands["pr0p_updater_runner_launch"]
    assert report.metrics["pr0p_phase_details"]["P1-install-discovery"] == {
        "client_candidate_count": 0,
        "executable_names": ["updater"],
        "install_scan_present": True,
        "status": PASS,
        "summary": "Linux updater is already present in the isolated install root",
    }


def test_independent_readiness_waits_on_stale_game_screen_evidence(tmp_path):
    game_dir = tmp_path / "game"
    pr0p_dir = tmp_path / "pr0p"
    game_dir.mkdir()
    pr0p_dir.mkdir()
    game_decision = game_dir / "20260705-game-decision.json"
    pr0p_suite = pr0p_dir / "20260707-pr0p-suite.json"
    write_game_decision(game_decision)
    write_pr0p_suite(pr0p_suite, [
        {"phase": "P0-isolation", "status": PASS},
        {"phase": "P1-install-discovery", "status": WAITING},
        {"phase": "P2-websocket", "status": WAITING},
        {"phase": "P2-msp-readonly", "status": WAITING},
        {"phase": "P3-capture", "status": WAITING},
        {"phase": "P4-input-readiness", "status": PASS},
        {"phase": "P4-input-mapping", "status": PASS},
        {"phase": "P6-synthetic-e2e-dry-run", "status": PASS},
        {"phase": "P6-synthetic-log-check", "status": PASS},
    ])
    os.utime(game_decision, ns=(1_000_000_000, 1_000_000_000))
    os.utime(pr0p_suite, ns=(2_000_000_000, 2_000_000_000))

    report = build_readiness(
        run_id="unit",
        game_log_dir=game_dir,
        pr0p_log_dir=pr0p_dir,
        bbox=None,
        window_title="pr0p",
        yaw_expected_sign=0,
        pitch_expected_sign=0,
        max_evidence_age_s=60.0,
        now_s=2_000_000_000.0,
    )
    assert report.status == WAITING
    assert report.metrics["raw_game_screen_decision"] == GAME_SCREEN_READY
    assert report.metrics["game_screen_decision"] == "STALE"
    assert "GAME_SCREEN_EVIDENCE_STALE" in report.reasons
    assert "GAME_SCREEN_NOT_READY:STALE" in report.reasons
    assert report.next_action == (
        "Rerun the game-screen acceptance command because its latest readiness "
        "evidence is stale."
    )


def test_independent_readiness_waits_on_old_game_screen_decision_schema(tmp_path):
    game_dir = tmp_path / "game"
    pr0p_dir = tmp_path / "pr0p"
    game_dir.mkdir()
    pr0p_dir.mkdir()
    write_game_decision(
        game_dir / "20260705-game-decision.json",
        current_schema=False,
    )
    steps = readiness_promotion_steps()
    suite_path = pr0p_dir / "20260707-pr0p-suite.json"
    write_pr0p_suite(suite_path, steps)
    write_pr0p_manifest(
        pr0p_dir / "20260707-pr0p-live-manifest.json",
        steps,
        suite_report=suite_path,
    )

    report = build_readiness(
        run_id="unit",
        game_log_dir=game_dir,
        pr0p_log_dir=pr0p_dir,
        bbox=(10, 20, 80, 60),
        window_title="pr0p",
        yaw_expected_sign=1,
        pitch_expected_sign=-1,
    )

    assert report.status == WAITING
    assert report.metrics["raw_game_screen_decision"] == GAME_SCREEN_READY
    assert report.metrics["game_screen_decision"] == "MISMATCH"
    assert report.metrics["game_screen_decision_schema"]["status"] == "MISMATCH"
    assert "S13-binding-dry" in report.metrics["game_screen_decision_schema"][
        "missing_synthetic_required_phases"
    ]
    assert "S13-binding-dry" in report.metrics["game_screen_decision_schema"][
        "non_pass_synthetic_phases"
    ]
    assert "GAME_SCREEN_DECISION_SCHEMA_MISMATCH" in report.reasons
    assert "GAME_SCREEN_NOT_READY:MISMATCH" in report.reasons
    assert report.next_action == (
        "Run the game-screen acceptance command before trusting tracker/PID "
        "dry-run evidence."
    )


def test_independent_readiness_promotes_only_when_both_paths_ready(tmp_path):
    game_dir = tmp_path / "game"
    pr0p_dir = tmp_path / "pr0p"
    game_dir.mkdir()
    pr0p_dir.mkdir()
    write_game_decision(game_dir / "20260705-game-decision.json")
    steps = readiness_promotion_steps()
    suite_path = pr0p_dir / "20260707-pr0p-suite.json"
    write_pr0p_suite(suite_path, steps)
    write_pr0p_manifest(
        pr0p_dir / "20260707-pr0p-live-manifest.json",
        steps,
        suite_report=suite_path,
    )

    report = build_readiness(
        run_id="unit",
        game_log_dir=game_dir,
        pr0p_log_dir=pr0p_dir,
        bbox=(10, 20, 80, 60),
        window_title="pr0p",
        yaw_expected_sign=1,
        pitch_expected_sign=-1,
    )
    assert report.status == INDEPENDENT_SIM_READY
    assert report.objective_completion_status == "IN_PROGRESS"
    assert report.objective_completion_missing == [
        "rc_manual_flight",
        "camera_tracker",
        "autopilot_control",
        "extended_follow",
        "moving_target_yaw",
        "approach_pitch",
        "handoff",
    ]
    assert report.objective_completion_failed == []
    assert report.reasons == []
    assert report.metrics["game_screen_decision_schema"]["status"] == PASS
    assert report.evidence_paths["pr0p_live_manifest"].endswith("-live-manifest.json")
    assert report.metrics["pr0p_manifest_suite_consistency"]["status"] == PASS
    assert report.metrics["objective_completion_status"] == "IN_PROGRESS"
    assert report.metrics["core_goal_status"] == WAITING
    assert report.metrics["core_goal_next_command_key"] == "rc_manual_flight_live"
    assert report.metrics["core_goal_gate_statuses"] == {
        "rc_manual_flight": WAITING,
        "camera_tracker": WAITING,
        "autopilot_control": WAITING,
    }
    assert report.metrics["core_goal_post_core_gate_statuses"] == {}
    assert report.next_action == (
        "Both independent paths are ready; run pr0p_core_goal_report.py or "
        "pr0p_core_next_runner.py to continue the measured extended-follow, "
        "yaw, approach, and handoff gates."
    )
    markdown = build_readiness_markdown(report)
    assert "Objective completion: `IN_PROGRESS`" in markdown
    assert "full independent pr0p goal is not complete yet" in markdown
    assert "--tracking-bbox 10,20,80,60" in report.commands["pr0p_safe_suite_with_bbox"]
    assert "pr0p_runtime_input_visual_probe.py" in report.commands["pr0p_runtime_input_visual"]
    assert "--rc-effect-axis aux1" in report.commands["pr0p_aux1_rc_effect"]
    assert "--rc-effect-expected-channel aux1" in report.commands["pr0p_aux1_rc_effect"]
    assert "--role aux1" in report.commands["pr0p_aux1_mapping_assistant_dry"]
    assert "--uinput" not in report.commands["pr0p_aux1_mapping_assistant_dry"]
    assert "--role aux1" in report.commands["pr0p_aux1_mapping_assistant_live"]
    assert "--uinput" in report.commands["pr0p_aux1_mapping_assistant_live"]
    assert "--ack-live-input" in report.commands["pr0p_aux1_mapping_assistant_live"]
    assert "--run-aux-arm-status" in report.commands["pr0p_aux1_arm_status"]
    assert "--aux-arm-aux1-magnitude 1.0" in report.commands["pr0p_aux1_arm_status"]
    assert "pr0p_aux1_acceptance_runner.py" in report.commands["pr0p_aux1_acceptance_plan"]
    assert "--bbox 10,20,80,60" in report.commands["pr0p_aux1_acceptance_plan"]
    assert "--run-live-gates" in report.commands["pr0p_aux1_acceptance_live"]
    assert "--ack-live-input" in report.commands["pr0p_aux1_acceptance_live"]
    assert "pr0p_response_acceptance_runner.py" in report.commands["pr0p_response_acceptance_plan"]
    assert "--run-live-gates" in report.commands["pr0p_response_acceptance_live"]
    assert "--ack-live-input" in report.commands["pr0p_response_acceptance_live"]
    assert "--arm-first" in report.commands["pr0p_response_acceptance_live"]
    assert "--measure attitude" in report.commands["pr0p_response_acceptance_live"]
    assert "pr0p_tracking_acceptance_runner.py" in report.commands["pr0p_tracking_acceptance_plan"]
    assert "--bbox 10,20,80,60" in report.commands["pr0p_tracking_acceptance_plan"]
    assert "--run-live-gates" in report.commands["pr0p_tracking_acceptance_live"]
    assert "--ack-live-input" in report.commands["pr0p_tracking_acceptance_live"]
    assert "--arm-first" in report.commands["pr0p_tracking_acceptance_live"]
    assert "--arm-throttle -0.4" in report.commands["pr0p_tracking_acceptance_live"]
    assert "pr0p_acceptance_chain_runner.py" in report.commands["pr0p_acceptance_chain_plan"]
    assert "--bbox 10,20,80,60" in report.commands["pr0p_acceptance_chain_plan"]
    assert "--game-log-dir" in report.commands["pr0p_acceptance_chain_plan"]
    assert "pr0p_acceptance_state_report.py" in report.commands["pr0p_acceptance_state"]
    assert "--bbox 10,20,80,60" in report.commands["pr0p_acceptance_state"]
    assert "pr0p_core_goal_report.py" in report.commands["pr0p_core_goal_status"]
    assert "--bbox 10,20,80,60" in report.commands["pr0p_core_goal_status"]
    assert "pr0p_core_next_runner.py" in report.commands["pr0p_core_next_plan"]
    assert "--execute-next" not in report.commands["pr0p_core_next_plan"]
    assert "pr0p_core_next_runner.py" in report.commands["pr0p_core_next_execute_with_ack"]
    assert "--execute-next" in report.commands["pr0p_core_next_execute_with_ack"]
    assert "--ack-live-command" in report.commands["pr0p_core_next_execute_with_ack"]
    assert "--execute-dry-patch" in report.commands["pr0p_acceptance_chain_live"]
    assert "--run-live-gates" in report.commands["pr0p_acceptance_chain_live"]
    assert "--ack-live-launch" in report.commands["pr0p_acceptance_chain_live"]
    assert "--ack-live-input" in report.commands["pr0p_acceptance_chain_live"]
    assert "--game-log-dir" in report.commands["pr0p_acceptance_chain_live"]
    assert "--measure attitude" in report.commands["pr0p_acceptance_chain_live"]
    assert "--duration 20.0" in report.commands["pr0p_extended_follow_live"]
    assert "--min-found-ratio 0.95" in report.commands["pr0p_extended_follow_live"]
    assert "--max-loss-events 0" in report.commands["pr0p_extended_follow_live"]


def test_independent_readiness_cli_accepts_bbox_file(tmp_path):
    bbox_file = tmp_path / "target-bbox.json"
    bbox_file.write_text(json.dumps({"bbox": [10, 20, 80, 60]}), encoding="utf-8")

    args = parse_readiness_args(["--bbox-file", str(bbox_file)])

    assert args.bbox == (10, 20, 80, 60)

    with pytest.raises(SystemExit):
        parse_readiness_args(["--bbox", "1,2,30,40", "--bbox-file", str(bbox_file)])


def test_independent_readiness_requires_live_manifest_after_suite_pass(tmp_path):
    game_dir = tmp_path / "game"
    pr0p_dir = tmp_path / "pr0p"
    game_dir.mkdir()
    pr0p_dir.mkdir()
    write_game_decision(game_dir / "20260705-game-decision.json")
    write_pr0p_suite(pr0p_dir / "20260707-pr0p-suite.json", readiness_promotion_steps())

    report = build_readiness(
        run_id="unit",
        game_log_dir=game_dir,
        pr0p_log_dir=pr0p_dir,
        bbox=(10, 20, 80, 60),
        window_title="pr0p",
        yaw_expected_sign=1,
        pitch_expected_sign=-1,
    )

    assert report.status == WAITING
    assert report.evidence_paths["pr0p_live_manifest"] is None
    assert "PR0P:LIVE_MANIFEST_EVIDENCE_MISSING" in report.reasons
    assert "PR0P:MISSING_LIVE_MANIFEST_EVIDENCE" in report.reasons
    assert report.next_action == (
        "Generate a live manifest from the latest acceptance evidence before "
        "marking the independent sim path ready."
    )


def test_independent_readiness_rejects_manifest_from_older_suite(tmp_path):
    game_dir = tmp_path / "game"
    pr0p_dir = tmp_path / "pr0p"
    game_dir.mkdir()
    pr0p_dir.mkdir()
    write_game_decision(game_dir / "20260705-game-decision.json")
    old_suite = pr0p_dir / "20260706-pr0p-suite.json"
    latest_suite = pr0p_dir / "20260707-pr0p-suite.json"
    manifest = pr0p_dir / "20260707-pr0p-live-manifest.json"
    steps = readiness_promotion_steps()
    write_pr0p_suite(old_suite, steps)
    write_pr0p_suite(latest_suite, steps)
    write_pr0p_manifest(manifest, steps, suite_report=old_suite)
    os.utime(old_suite, ns=(1_000_000_000, 1_000_000_000))
    os.utime(latest_suite, ns=(2_000_000_000, 2_000_000_000))
    os.utime(manifest, ns=(3_000_000_000, 3_000_000_000))

    report = build_readiness(
        run_id="unit",
        game_log_dir=game_dir,
        pr0p_log_dir=pr0p_dir,
        bbox=(10, 20, 80, 60),
        window_title="pr0p",
        yaw_expected_sign=1,
        pitch_expected_sign=-1,
        max_evidence_age_s=60.0,
        now_s=3.0,
    )

    assert report.status == WAITING
    assert "PR0P:LIVE_MANIFEST_SUITE_MISMATCH" in report.reasons
    assert report.metrics["pr0p_manifest_suite_consistency"]["status"] == "MISMATCH"
    assert report.metrics["pr0p_manifest_suite_consistency"]["manifest_suite_report"] == str(old_suite)
    assert report.metrics["pr0p_manifest_suite_consistency"]["latest_suite"] == str(latest_suite)
    assert report.next_action == (
        "Regenerate the pr0p live manifest from the latest safe suite before "
        "marking the independent sim path ready."
    )


def test_acceptance_state_report_waits_without_aux1_evidence(tmp_path):
    report = build_acceptance_state(
        run_id="unit",
        log_dir=tmp_path,
        bbox=(10, 20, 80, 60),
        max_evidence_age_s=60.0,
        now_s=100.0,
    )

    stages = {stage.name: stage for stage in report.stages}
    assert report.status == WAITING
    assert stages["pr0p_aux1_acceptance"].status == "MISSING"
    assert "pr0p_aux1_acceptance_live" in report.next_action
    assert "pr0p_aux1_acceptance_runner.py" in report.commands["pr0p_aux1_acceptance_live"]
    markdown = build_acceptance_state_markdown(report)
    assert "SimITL / pr0p Acceptance State" in markdown


def test_acceptance_state_report_rejects_stale_evidence(tmp_path):
    aux = tmp_path / "20260707-unit-aux1-acceptance.json"
    write_aux1_acceptance(aux)
    write_response_acceptance(tmp_path / "20260707-unit-response-acceptance.json")
    write_tracking_acceptance(tmp_path / "20260707-unit-tracking-acceptance.json")
    os.utime(aux, ns=(1_000_000_000, 1_000_000_000))

    report = build_acceptance_state(
        run_id="unit",
        log_dir=tmp_path,
        bbox=(10, 20, 80, 60),
        max_evidence_age_s=60.0,
        now_s=120.0,
    )

    stages = {stage.name: stage for stage in report.stages}
    assert report.status == WAITING
    assert stages["pr0p_aux1_acceptance"].status == STALE
    assert stages["pr0p_aux1_acceptance"].raw_status == PASS
    assert "stale" in report.next_action


def test_acceptance_state_report_passes_with_all_evidence(tmp_path):
    write_aux1_acceptance(tmp_path / "20260707-unit-aux1-acceptance.json")
    write_response_acceptance(tmp_path / "20260707-unit-response-acceptance.json")
    write_tracking_acceptance(tmp_path / "20260707-unit-tracking-acceptance.json")

    report = build_acceptance_state(
        run_id="unit",
        log_dir=tmp_path,
        bbox=(10, 20, 80, 60),
        max_evidence_age_s=600.0,
        now_s=time.time(),
    )

    assert report.status == PASS
    assert report.metrics["real_input_sent"] is False
    assert report.metrics["stage_statuses"] == {
        "pr0p_aux1_acceptance": PASS,
        "pr0p_response_acceptance": PASS,
        "pr0p_tracking_acceptance": PASS,
    }
    assert "chain-generated decision/readiness reports" in report.next_action
    assert "acceptance chain" in report.next_action


def test_core_goal_report_starts_with_rc_manual_flight(tmp_path):
    report = build_core_goal_report(
        run_id="unit",
        log_dir=tmp_path,
        bbox=(10, 20, 80, 60),
        max_evidence_age_s=60.0,
        now_s=100.0,
    )

    gates = {gate.name: gate for gate in report.gates}
    assert report.status == WAITING
    assert report.completion_status == "IN_PROGRESS"
    assert "rc_manual_flight" in report.completion_missing
    assert "extended_follow" in report.completion_missing
    assert gates["rc_manual_flight"].status == WAITING
    assert gates["camera_tracker"].status == WAITING
    assert gates["autopilot_control"].status == WAITING
    assert report.next_action.startswith("Focus on RC/manual flight first")
    assert report.next_command_key == "rc_manual_flight_live"
    assert "rc_manual_flight_live" in report.commands
    assert report.next_command == report.commands["rc_manual_flight_live"]
    assert "--ack-live-input" in report.commands["rc_manual_flight_live"]
    assert "acceptance_chain_live" in report.commands
    assert "extended_follow_live" in report.commands
    requirement_statuses = report.metrics["objective_requirement_statuses"]
    assert requirement_statuses["gazebo_independent_path"] == PASS
    assert requirement_statuses["target_bbox_selected"] == PASS
    assert requirement_statuses["manual_rc_flight"] == WAITING
    assert requirement_statuses["camera_image_to_tracker"] == WAITING
    assert requirement_statuses["autopilot_pid_control"] == WAITING
    assert requirement_statuses["extended_closed_loop_follow"] == WAITING
    assert requirement_statuses["airborne_static_yaw_centering"] == WAITING
    assert "gazebo_independent_path" not in report.metrics["objective_requirement_missing"]
    assert "manual_rc_flight" in report.metrics["objective_requirement_missing"]
    assert report.metrics["objective_requirement_failed"] == []
    markdown = build_core_goal_markdown(report)
    assert "pr0p Core Goal Status" in markdown
    assert "Completion: `IN_PROGRESS`" in markdown
    assert "rc_manual_flight" in markdown
    assert "## Objective Requirements" in markdown
    assert "gazebo_independent_path" in markdown
    assert "## Commands" in markdown
    assert "Next command key: `rc_manual_flight_live`" in markdown
    assert "rc_manual_flight_live" in markdown


def test_core_goal_report_passes_when_flight_tracker_and_control_evidence_pass(tmp_path):
    write_pr0p_suite(tmp_path / "20260707-pr0p-suite.json", readiness_promotion_steps())
    write_aux1_acceptance(tmp_path / "20260707-unit-aux1-acceptance.json")
    write_response_acceptance(tmp_path / "20260707-unit-response-acceptance.json")
    write_tracking_acceptance(tmp_path / "20260707-unit-tracking-acceptance.json")

    report = build_core_goal_report(
        run_id="unit",
        log_dir=tmp_path,
        bbox=(10, 20, 80, 60),
        max_evidence_age_s=600.0,
        now_s=time.time(),
    )

    gates = {gate.name: gate for gate in report.gates}
    assert report.status == PASS
    assert report.completion_status == "IN_PROGRESS"
    assert report.metrics["completion_status"] == "IN_PROGRESS"
    assert report.completion_missing == [
        "extended_follow",
        "moving_target_yaw",
        "approach_pitch",
        "handoff",
    ]
    assert gates["rc_manual_flight"].status == PASS
    assert gates["camera_tracker"].status == PASS
    assert gates["autopilot_control"].status == PASS
    assert report.metrics["real_input_sent"] is False
    post_core = {gate.name: gate for gate in report.post_core_gates}
    assert post_core["extended_follow"].status == WAITING
    assert "extended_follow_duration_s" in post_core["extended_follow"].missing
    assert report.metrics["post_core_gate_statuses"] == {"extended_follow": WAITING}
    assert report.next_command_key == "extended_follow_live"
    assert "extended_follow_live" in report.next_action
    assert report.next_command == report.commands["extended_follow_live"]
    assert "pr0p_tracking_acceptance_runner.py" in report.next_command
    assert "--arm-first" in report.next_command
    assert "--arm-throttle -0.4" in report.next_command
    assert "--run-live-gates" in report.next_command
    assert "--ack-live-input" in report.next_command
    assert "--ack-live-launch" not in report.next_command
    assert "--measure attitude" not in report.next_command
    assert "--duration 20.0" in report.next_command
    assert "--min-found-ratio 0.95" in report.next_command
    assert "--max-loss-events 0" in report.next_command
    requirement_statuses = report.metrics["objective_requirement_statuses"]
    assert requirement_statuses["manual_rc_flight"] == PASS
    assert requirement_statuses["camera_image_to_tracker"] == PASS
    assert requirement_statuses["autopilot_pid_control"] == PASS
    assert requirement_statuses["extended_closed_loop_follow"] == WAITING
    assert "extended_closed_loop_follow" in report.metrics["objective_requirement_missing"]


def test_core_goal_report_advances_after_extended_follow_passes(tmp_path):
    write_pr0p_suite(tmp_path / "20260707-pr0p-suite.json", readiness_promotion_steps())
    write_aux1_acceptance(tmp_path / "20260707-unit-aux1-acceptance.json")
    write_response_acceptance(tmp_path / "20260707-unit-response-acceptance.json")
    write_tracking_acceptance(
        tmp_path / "20260707-unit-tracking-acceptance.json",
        duration_s=20.0,
        min_found_ratio=0.95,
        max_loss_events=0,
    )

    report = build_core_goal_report(
        run_id="unit",
        log_dir=tmp_path,
        bbox=(10, 20, 80, 60),
        max_evidence_age_s=600.0,
        now_s=time.time(),
    )

    gates = {gate.name: gate for gate in report.gates}
    post_core = {gate.name: gate for gate in report.post_core_gates}
    assert report.status == PASS
    assert report.completion_status == "IN_PROGRESS"
    assert report.completion_missing == [
        "moving_target_yaw",
        "approach_pitch",
        "handoff",
    ]
    assert gates["rc_manual_flight"].status == PASS
    assert gates["camera_tracker"].status == PASS
    assert gates["autopilot_control"].status == PASS
    assert post_core["extended_follow"].status == PASS
    assert post_core["moving_target_yaw"].status == WAITING
    assert "moving_target_yaw_plan" in post_core["moving_target_yaw"].missing
    assert report.metrics["post_core_gate_statuses"] == {
        "extended_follow": PASS,
        "moving_target_yaw": WAITING,
    }
    assert report.next_command_key == "moving_target_yaw_plan"
    assert report.next_command == report.commands["moving_target_yaw_plan"]
    assert "pr0p_moving_target_yaw_plan.py" in report.next_command
    assert "--execute-synthetic" in report.next_command
    assert "--bbox 10,20,80,60" in report.next_command
    assert report.next_action.startswith("Extended follow is proven")
    markdown = build_core_goal_markdown(report)
    assert "## Post-Core Gates" in markdown
    assert "extended_follow" in markdown
    assert "moving_target_yaw" in markdown


def test_core_goal_report_keeps_extended_follow_when_newer_short_tracking_exists(tmp_path):
    write_pr0p_suite(tmp_path / "20260707-pr0p-suite.json", readiness_promotion_steps())
    write_aux1_acceptance(tmp_path / "20260707-unit-aux1-acceptance.json")
    write_response_acceptance(tmp_path / "20260707-unit-response-acceptance.json")
    extended = tmp_path / "20260707-010000-unit-extended-tracking-acceptance.json"
    write_tracking_acceptance(
        extended,
        duration_s=20.0,
        min_found_ratio=0.95,
        max_loss_events=0,
    )
    short = tmp_path / "20260707-020000-unit-short-tracking-acceptance.json"
    write_tracking_acceptance(
        short,
        duration_s=15.0,
        min_found_ratio=0.85,
        max_loss_events=0,
    )

    report = build_core_goal_report(
        run_id="unit",
        log_dir=tmp_path,
        bbox=(10, 20, 80, 60),
        max_evidence_age_s=600.0,
        now_s=time.time(),
    )

    post_core = {gate.name: gate for gate in report.post_core_gates}
    assert post_core["extended_follow"].status == PASS
    assert post_core["extended_follow"].evidence["tracking_acceptance_report"] == str(extended)
    assert report.next_command_key == "moving_target_yaw_plan"


def test_core_goal_report_waits_for_real_moving_target_after_synthetic_pass(tmp_path):
    write_pr0p_suite(tmp_path / "20260707-pr0p-suite.json", readiness_promotion_steps())
    write_aux1_acceptance(tmp_path / "20260707-unit-aux1-acceptance.json")
    write_response_acceptance(tmp_path / "20260707-unit-response-acceptance.json")
    write_tracking_acceptance(
        tmp_path / "20260707-unit-tracking-acceptance.json",
        duration_s=20.0,
        min_found_ratio=0.95,
        max_loss_events=0,
    )
    write_moving_target_yaw(
        tmp_path / "20260707-unit-moving-target-yaw.json",
        status=WAITING,
        synthetic_status=PASS,
        real_status=WAITING,
    )

    report = build_core_goal_report(
        run_id="unit",
        log_dir=tmp_path,
        bbox=(10, 20, 80, 60),
        max_evidence_age_s=600.0,
        now_s=time.time(),
    )

    post_core = {gate.name: gate for gate in report.post_core_gates}
    assert report.status == PASS
    assert post_core["extended_follow"].status == PASS
    assert post_core["moving_target_yaw"].status == WAITING
    assert "real_pr0p_moving_target_yaw" in post_core["moving_target_yaw"].missing
    assert report.next_command_key == "airborne_static_yaw_live"
    assert report.next_command == report.commands["airborne_static_yaw_live"]
    assert "pr0p_moving_target_acceptance_runner.py" in report.next_command
    assert "--run-live-gate" in report.next_command
    assert "--ack-airborne-static-target" in report.next_command
    assert report.next_action.startswith("Synthetic yaw evidence is ready")


def test_core_goal_report_refreshes_yaw_after_airborne_static_acceptance_passes(tmp_path):
    write_pr0p_suite(tmp_path / "20260707-pr0p-suite.json", readiness_promotion_steps())
    write_aux1_acceptance(tmp_path / "20260707-unit-aux1-acceptance.json")
    write_response_acceptance(tmp_path / "20260707-unit-response-acceptance.json")
    write_tracking_acceptance(
        tmp_path / "20260707-unit-tracking-acceptance.json",
        duration_s=20.0,
        min_found_ratio=0.95,
        max_loss_events=0,
    )
    tracking_report = write_real_moving_tracking_evidence(tmp_path)
    write_moving_target_yaw(
        tmp_path / "20260707-unit-moving-target-yaw.json",
        status=WAITING,
        synthetic_status=PASS,
        real_status=WAITING,
    )
    write_moving_target_acceptance(
        tmp_path / "20260707-unit-moving-target-acceptance.json",
        tracking_report=tracking_report,
    )

    report = build_core_goal_report(
        run_id="unit",
        log_dir=tmp_path,
        bbox=(10, 20, 80, 60),
        max_evidence_age_s=600.0,
        now_s=time.time(),
    )

    post_core = {gate.name: gate for gate in report.post_core_gates}
    assert report.status == PASS
    assert post_core["extended_follow"].status == PASS
    assert post_core["moving_target_yaw"].status == WAITING
    assert report.next_command_key == "airborne_static_yaw_refresh"
    assert report.next_command == report.commands["airborne_static_yaw_refresh"]
    assert "pr0p_moving_target_yaw_plan.py" in report.next_command
    assert "--execute-synthetic" in report.next_command
    assert "--real-tracking-report %s" % tracking_report in report.next_command
    assert "--ack-airborne-static-target" in report.next_command
    assert "--run-live-gate" not in report.next_command
    assert report.metrics["airborne_static_yaw_refresh_ready"] is True
    assert report.metrics["airborne_static_tracking_report"] == str(tracking_report)
    assert report.next_action.startswith("Real airborne static-target tracking evidence is ready")


def test_core_goal_report_does_not_refresh_yaw_with_missing_tracking_report(tmp_path):
    write_pr0p_suite(tmp_path / "20260707-pr0p-suite.json", readiness_promotion_steps())
    write_aux1_acceptance(tmp_path / "20260707-unit-aux1-acceptance.json")
    write_response_acceptance(tmp_path / "20260707-unit-response-acceptance.json")
    write_tracking_acceptance(
        tmp_path / "20260707-unit-tracking-acceptance.json",
        duration_s=20.0,
        min_found_ratio=0.95,
        max_loss_events=0,
    )
    missing_tracking_report = tmp_path / "deleted-live-tracking-acceptance.json"
    write_moving_target_yaw(
        tmp_path / "20260707-unit-moving-target-yaw.json",
        status=WAITING,
        synthetic_status=PASS,
        real_status=WAITING,
    )
    write_moving_target_acceptance(
        tmp_path / "20260707-unit-moving-target-acceptance.json",
        tracking_report=missing_tracking_report,
    )

    report = build_core_goal_report(
        run_id="unit",
        log_dir=tmp_path,
        bbox=(10, 20, 80, 60),
        max_evidence_age_s=600.0,
        now_s=time.time(),
    )

    post_core = {gate.name: gate for gate in report.post_core_gates}
    assert report.status == PASS
    assert post_core["extended_follow"].status == PASS
    assert post_core["moving_target_yaw"].status == WAITING
    assert "real_pr0p_moving_target_yaw" in post_core["moving_target_yaw"].missing
    assert report.metrics["moving_target_acceptance_report"] == str(
        tmp_path / "20260707-unit-moving-target-acceptance.json"
    )
    assert report.metrics["airborne_static_tracking_report"] == str(missing_tracking_report)
    assert report.metrics["airborne_static_yaw_refresh_ready"] is False
    assert report.next_command_key == "airborne_static_yaw_live"
    assert "pr0p_moving_target_acceptance_runner.py" in report.next_command
    assert "--run-live-gate" in report.next_command
    assert "--real-tracking-report" not in report.next_command


def test_core_goal_report_finishes_post_core_when_moving_target_passes(tmp_path):
    write_pr0p_suite(tmp_path / "20260707-pr0p-suite.json", readiness_promotion_steps())
    write_aux1_acceptance(tmp_path / "20260707-unit-aux1-acceptance.json")
    write_response_acceptance(tmp_path / "20260707-unit-response-acceptance.json")
    write_tracking_acceptance(
        tmp_path / "20260707-unit-tracking-acceptance.json",
        duration_s=20.0,
        min_found_ratio=0.95,
        max_loss_events=0,
    )
    write_moving_target_yaw(
        tmp_path / "20260707-unit-moving-target-yaw.json",
        status=PASS,
        synthetic_status=PASS,
        real_status=PASS,
    )

    report = build_core_goal_report(
        run_id="unit",
        log_dir=tmp_path,
        bbox=(10, 20, 80, 60),
        max_evidence_age_s=600.0,
        now_s=time.time(),
    )

    post_core = {gate.name: gate for gate in report.post_core_gates}
    assert report.status == PASS
    assert report.completion_status == "IN_PROGRESS"
    assert report.completion_missing == ["approach_pitch", "handoff"]
    assert report.completion_failed == []
    assert report.metrics["completion_status"] == "IN_PROGRESS"
    assert post_core["extended_follow"].status == PASS
    assert post_core["moving_target_yaw"].status == PASS
    assert post_core["approach_pitch"].status == WAITING
    assert "approach_pitch_plan" in post_core["approach_pitch"].missing
    assert report.next_command_key == "approach_pitch_plan"
    assert report.next_command == report.commands["approach_pitch_plan"]
    assert "pr0p_approach_pitch_plan.py" in report.next_command
    assert report.next_action.startswith("Airborne static-target yaw-only evidence is proven")


def test_core_goal_report_waits_for_real_approach_after_synthetic_pass(tmp_path):
    write_pr0p_suite(tmp_path / "20260707-pr0p-suite.json", readiness_promotion_steps())
    write_aux1_acceptance(tmp_path / "20260707-unit-aux1-acceptance.json")
    write_response_acceptance(tmp_path / "20260707-unit-response-acceptance.json")
    write_tracking_acceptance(
        tmp_path / "20260707-unit-tracking-acceptance.json",
        duration_s=20.0,
        min_found_ratio=0.95,
        max_loss_events=0,
    )
    write_moving_target_yaw(
        tmp_path / "20260707-unit-moving-target-yaw.json",
        status=PASS,
        synthetic_status=PASS,
        real_status=PASS,
    )
    write_approach_pitch(
        tmp_path / "20260707-unit-approach-pitch.json",
        status=WAITING,
        synthetic_status=PASS,
        real_status=WAITING,
    )

    report = build_core_goal_report(
        run_id="unit",
        log_dir=tmp_path,
        bbox=(10, 20, 80, 60),
        max_evidence_age_s=600.0,
        now_s=time.time(),
    )

    post_core = {gate.name: gate for gate in report.post_core_gates}
    assert report.status == PASS
    assert post_core["moving_target_yaw"].status == PASS
    assert post_core["approach_pitch"].status == WAITING
    assert "real_pr0p_approach_pitch" in post_core["approach_pitch"].missing
    assert report.next_command_key == "approach_pitch_live"
    assert report.next_command == report.commands["approach_pitch_live"]
    assert "pr0p_tracking_acceptance_runner.py" in report.next_command
    assert "--run-live-gates" in report.next_command
    assert "--ack-live-input" in report.next_command
    assert "--enable-pitch" in report.next_command
    assert report.next_action.startswith("Synthetic approach/pitch evidence is ready")


def test_core_goal_report_refreshes_approach_after_live_tracking_passes(tmp_path):
    write_pr0p_suite(tmp_path / "20260707-pr0p-suite.json", readiness_promotion_steps())
    write_aux1_acceptance(tmp_path / "20260707-unit-aux1-acceptance.json")
    write_response_acceptance(tmp_path / "20260707-unit-response-acceptance.json")
    write_tracking_acceptance(
        tmp_path / "20260707-unit-tracking-acceptance.json",
        duration_s=20.0,
        min_found_ratio=0.95,
        max_loss_events=0,
    )
    write_moving_target_yaw(
        tmp_path / "20260707-unit-moving-target-yaw.json",
        status=PASS,
        synthetic_status=PASS,
        real_status=PASS,
    )
    write_approach_pitch(
        tmp_path / "20260707-unit-approach-pitch.json",
        status=WAITING,
        synthetic_status=PASS,
        real_status=WAITING,
    )
    approach_tracking = tmp_path / "20260707-unit-approach-tracking-acceptance.json"
    write_tracking_acceptance(
        approach_tracking,
        duration_s=20.0,
        min_found_ratio=0.9,
        max_loss_events=2,
        enable_pitch=True,
        desired_target_width=120.0,
        max_abs_pitch_axis=0.8,
    )

    report = build_core_goal_report(
        run_id="unit",
        log_dir=tmp_path,
        bbox=(10, 20, 80, 60),
        max_evidence_age_s=600.0,
        now_s=time.time(),
    )

    post_core = {gate.name: gate for gate in report.post_core_gates}
    assert report.status == PASS
    assert post_core["moving_target_yaw"].status == PASS
    assert post_core["approach_pitch"].status == WAITING
    assert report.next_command_key == "approach_pitch_refresh"
    assert report.next_command == report.commands["approach_pitch_refresh"]
    assert "pr0p_approach_pitch_plan.py" in report.next_command
    assert "--execute-synthetic" in report.next_command
    assert "--real-tracking-report %s" % approach_tracking in report.next_command
    assert "--ack-real-approach-target" in report.next_command
    assert "--run-live-gates" not in report.next_command
    assert report.metrics["approach_pitch_refresh_ready"] is True
    assert report.metrics["approach_tracking_acceptance_report"] == str(approach_tracking)
    assert report.next_action.startswith("Real pitch-enabled approach tracking evidence is ready")


def test_core_goal_report_finishes_after_approach_pitch_passes(tmp_path):
    write_pr0p_suite(tmp_path / "20260707-pr0p-suite.json", readiness_promotion_steps())
    write_aux1_acceptance(tmp_path / "20260707-unit-aux1-acceptance.json")
    write_response_acceptance(tmp_path / "20260707-unit-response-acceptance.json")
    write_tracking_acceptance(
        tmp_path / "20260707-unit-tracking-acceptance.json",
        duration_s=20.0,
        min_found_ratio=0.95,
        max_loss_events=0,
    )
    write_moving_target_yaw(
        tmp_path / "20260707-unit-moving-target-yaw.json",
        status=PASS,
        synthetic_status=PASS,
        real_status=PASS,
    )
    write_approach_pitch(
        tmp_path / "20260707-unit-approach-pitch.json",
        status=PASS,
        synthetic_status=PASS,
        real_status=PASS,
    )

    report = build_core_goal_report(
        run_id="unit",
        log_dir=tmp_path,
        bbox=(10, 20, 80, 60),
        max_evidence_age_s=600.0,
        now_s=time.time(),
    )

    post_core = {gate.name: gate for gate in report.post_core_gates}
    assert report.status == PASS
    assert report.completion_status == "IN_PROGRESS"
    assert report.completion_missing == ["handoff"]
    assert report.completion_failed == []
    assert report.metrics["completion_status"] == "IN_PROGRESS"
    assert post_core["extended_follow"].status == PASS
    assert post_core["moving_target_yaw"].status == PASS
    assert post_core["approach_pitch"].status == PASS
    assert post_core["handoff"].status == WAITING
    assert "handoff_plan" in post_core["handoff"].missing
    assert report.next_command_key == "handoff_plan"
    assert report.next_command == report.commands["handoff_plan"]
    assert "pr0p_handoff_plan.py" in report.next_command
    assert report.next_action.startswith("Approach/pitch evidence is proven")


def test_core_goal_report_waits_for_real_handoff_after_synthetic_pass(tmp_path):
    write_pr0p_suite(tmp_path / "20260707-pr0p-suite.json", readiness_promotion_steps())
    write_aux1_acceptance(tmp_path / "20260707-unit-aux1-acceptance.json")
    write_response_acceptance(tmp_path / "20260707-unit-response-acceptance.json")
    write_tracking_acceptance(
        tmp_path / "20260707-unit-tracking-acceptance.json",
        duration_s=20.0,
        min_found_ratio=0.95,
        max_loss_events=0,
    )
    write_moving_target_yaw(
        tmp_path / "20260707-unit-moving-target-yaw.json",
        status=PASS,
        synthetic_status=PASS,
        real_status=PASS,
    )
    write_approach_pitch(
        tmp_path / "20260707-unit-approach-pitch.json",
        status=PASS,
        synthetic_status=PASS,
        real_status=PASS,
    )
    write_handoff(
        tmp_path / "20260707-unit-handoff.json",
        status=WAITING,
        synthetic_status=PASS,
        real_status=WAITING,
    )

    report = build_core_goal_report(
        run_id="unit",
        log_dir=tmp_path,
        bbox=(10, 20, 80, 60),
        max_evidence_age_s=600.0,
        now_s=time.time(),
    )

    post_core = {gate.name: gate for gate in report.post_core_gates}
    assert report.status == PASS
    assert post_core["approach_pitch"].status == PASS
    assert post_core["handoff"].status == WAITING
    assert "real_pr0p_manual_to_auto_handoff" in post_core["handoff"].missing
    assert report.next_command_key == "handoff_live"
    assert report.next_command == report.commands["handoff_live"]
    assert "pr0p_handoff_acceptance_runner.py" in report.next_command
    assert "--run-live-gate" in report.next_command
    assert "--ack-live-input" in report.next_command
    assert "--ack-real-handoff" in report.next_command
    assert report.next_action.startswith("Synthetic handoff evidence is ready")


def test_core_goal_report_refreshes_handoff_after_real_acceptance_passes(tmp_path):
    write_pr0p_suite(tmp_path / "20260707-pr0p-suite.json", readiness_promotion_steps())
    write_aux1_acceptance(tmp_path / "20260707-unit-aux1-acceptance.json")
    write_response_acceptance(tmp_path / "20260707-unit-response-acceptance.json")
    write_tracking_acceptance(
        tmp_path / "20260707-unit-tracking-acceptance.json",
        duration_s=20.0,
        min_found_ratio=0.95,
        max_loss_events=0,
    )
    write_moving_target_yaw(
        tmp_path / "20260707-unit-moving-target-yaw.json",
        status=PASS,
        synthetic_status=PASS,
        real_status=PASS,
    )
    write_approach_pitch(
        tmp_path / "20260707-unit-approach-pitch.json",
        status=PASS,
        synthetic_status=PASS,
        real_status=PASS,
    )
    write_handoff(
        tmp_path / "20260707-unit-handoff.json",
        status=WAITING,
        synthetic_status=PASS,
        real_status=WAITING,
    )
    real_handoff = tmp_path / "20260707-unit-real-handoff.json"
    write_real_handoff_acceptance(real_handoff)

    report = build_core_goal_report(
        run_id="unit",
        log_dir=tmp_path,
        bbox=(10, 20, 80, 60),
        max_evidence_age_s=600.0,
        now_s=time.time(),
    )

    post_core = {gate.name: gate for gate in report.post_core_gates}
    assert report.status == PASS
    assert post_core["approach_pitch"].status == PASS
    assert post_core["handoff"].status == WAITING
    assert report.metrics["handoff_report"] == str(tmp_path / "20260707-unit-handoff.json")
    assert report.metrics["real_handoff_acceptance_report"] == str(real_handoff)
    assert report.metrics["handoff_refresh_ready"] is True
    assert report.next_command_key == "handoff_refresh"
    assert report.next_command == report.commands["handoff_refresh"]
    assert "pr0p_handoff_plan.py" in report.next_command
    assert "--execute-synthetic" in report.next_command
    assert "--real-handoff-report %s" % real_handoff in report.next_command
    assert "--ack-real-handoff" in report.next_command
    assert "--run-live-gate" not in report.next_command
    assert report.next_action.startswith("Real manual-to-autonomous handoff evidence is ready")


def test_core_goal_report_does_not_treat_real_handoff_acceptance_as_canonical_handoff(tmp_path):
    write_pr0p_suite(tmp_path / "20260707-pr0p-suite.json", readiness_promotion_steps())
    write_aux1_acceptance(tmp_path / "20260707-unit-aux1-acceptance.json")
    write_response_acceptance(tmp_path / "20260707-unit-response-acceptance.json")
    write_tracking_acceptance(
        tmp_path / "20260707-unit-tracking-acceptance.json",
        duration_s=20.0,
        min_found_ratio=0.95,
        max_loss_events=0,
    )
    write_moving_target_yaw(
        tmp_path / "20260707-unit-moving-target-yaw.json",
        status=PASS,
        synthetic_status=PASS,
        real_status=PASS,
    )
    write_approach_pitch(
        tmp_path / "20260707-unit-approach-pitch.json",
        status=PASS,
        synthetic_status=PASS,
        real_status=PASS,
    )
    real_handoff = tmp_path / "20260707-unit-real-handoff.json"
    write_real_handoff_acceptance(real_handoff)

    report = build_core_goal_report(
        run_id="unit",
        log_dir=tmp_path,
        bbox=(10, 20, 80, 60),
        max_evidence_age_s=600.0,
        now_s=time.time(),
    )

    post_core = {gate.name: gate for gate in report.post_core_gates}
    assert post_core["handoff"].status == WAITING
    assert "handoff_plan" in post_core["handoff"].missing
    assert report.metrics["handoff_report"] is None
    assert report.metrics["real_handoff_acceptance_report"] == str(real_handoff)
    assert report.next_command_key == "handoff_refresh"


def test_core_goal_report_finishes_after_handoff_passes(tmp_path):
    write_pr0p_suite(tmp_path / "20260707-pr0p-suite.json", readiness_promotion_steps())
    write_aux1_acceptance(tmp_path / "20260707-unit-aux1-acceptance.json")
    write_response_acceptance(tmp_path / "20260707-unit-response-acceptance.json")
    write_tracking_acceptance(
        tmp_path / "20260707-unit-tracking-acceptance.json",
        duration_s=20.0,
        min_found_ratio=0.95,
        max_loss_events=0,
    )
    write_moving_target_yaw(
        tmp_path / "20260707-unit-moving-target-yaw.json",
        status=PASS,
        synthetic_status=PASS,
        real_status=PASS,
    )
    write_approach_pitch(
        tmp_path / "20260707-unit-approach-pitch.json",
        status=PASS,
        synthetic_status=PASS,
        real_status=PASS,
    )
    write_handoff(
        tmp_path / "20260707-unit-handoff.json",
        status=PASS,
        synthetic_status=PASS,
        real_status=PASS,
    )

    report = build_core_goal_report(
        run_id="unit",
        log_dir=tmp_path,
        bbox=(10, 20, 80, 60),
        max_evidence_age_s=600.0,
        now_s=time.time(),
    )

    post_core = {gate.name: gate for gate in report.post_core_gates}
    assert report.status == PASS
    assert report.completion_status == "COMPLETE"
    assert report.completion_missing == []
    assert report.completion_failed == []
    assert report.metrics["completion_status"] == "COMPLETE"
    assert post_core["extended_follow"].status == PASS
    assert post_core["moving_target_yaw"].status == PASS
    assert post_core["approach_pitch"].status == PASS
    assert post_core["handoff"].status == PASS
    assert report.next_command_key is None
    assert report.next_command is None
    assert report.next_action.startswith("The independent pr0p path has")
    assert all(
        status == PASS
        for status in report.metrics["objective_requirement_statuses"].values()
    )
    assert report.metrics["objective_requirement_missing"] == []
    assert report.metrics["objective_requirement_failed"] == []


def test_core_goal_report_uses_rc_manual_flight_runner_evidence(tmp_path):
    write_rc_manual_flight(tmp_path / "20260707-unit-rc-manual-flight.json")

    report = build_core_goal_report(
        run_id="unit",
        log_dir=tmp_path,
        bbox=(10, 20, 80, 60),
        max_evidence_age_s=600.0,
        now_s=time.time(),
    )

    gates = {gate.name: gate for gate in report.gates}
    assert report.status == WAITING
    assert gates["rc_manual_flight"].status == PASS
    assert gates["camera_tracker"].status == WAITING
    assert gates["autopilot_control"].status == WAITING
    assert report.metrics["rc_manual_flight_report"].endswith("-rc-manual-flight.json")
    assert report.next_command_key == "tracking_acceptance_dry"
    assert report.next_command == report.commands["tracking_acceptance_dry"]
    assert "--execute-dry-run" in report.next_command
    assert "--bbox 10,20,80,60" in report.commands["tracking_acceptance_plan"]
    assert report.next_action.startswith("Focus on image and tracker next")


def test_core_goal_report_keeps_manual_flight_waiting_reason(tmp_path):
    write_rc_manual_flight(
        tmp_path / "20260707-unit-rc-manual-flight.json",
        status=WAITING,
        aux_status=WAITING,
    )

    report = build_core_goal_report(
        run_id="unit",
        log_dir=tmp_path,
        bbox=(10, 20, 80, 60),
        max_evidence_age_s=600.0,
        now_s=time.time(),
    )

    gate = {gate.name: gate for gate in report.gates}["rc_manual_flight"]
    assert gate.status == WAITING
    assert gate.missing == ["aux1_arm_acceptance"]
    assert "AUX1/CH5" in gate.summary


def test_core_goal_report_blocks_downstream_until_manual_flight_passes(tmp_path):
    write_pr0p_suite(tmp_path / "20260707-pr0p-suite.json", readiness_promotion_steps())
    write_rc_manual_flight(
        tmp_path / "20260707-unit-rc-manual-flight.json",
        status=WAITING,
        aux_status=WAITING,
    )
    write_response_acceptance(tmp_path / "20260707-unit-response-acceptance.json")
    write_tracking_acceptance(tmp_path / "20260707-unit-tracking-acceptance.json")

    report = build_core_goal_report(
        run_id="unit",
        log_dir=tmp_path,
        bbox=(10, 20, 80, 60),
        max_evidence_age_s=600.0,
        now_s=time.time(),
    )

    gates = {gate.name: gate for gate in report.gates}
    assert report.status == WAITING
    assert gates["rc_manual_flight"].status == WAITING
    assert gates["camera_tracker"].status == WAITING
    assert gates["camera_tracker"].missing == ["rc_manual_flight"]
    assert gates["camera_tracker"].evidence["raw_status"] == PASS
    assert gates["autopilot_control"].status == WAITING
    assert gates["autopilot_control"].missing == ["rc_manual_flight"]
    assert gates["autopilot_control"].evidence["raw_status"] == PASS
    assert report.next_action.startswith("Focus on RC/manual flight first")


def test_core_goal_report_physical_mapping_flag_flows_to_commands(tmp_path):
    report = build_core_goal_report(
        run_id="unit",
        log_dir=tmp_path,
        bbox=(10, 20, 80, 60),
        allow_physical_mapping=True,
        max_evidence_age_s=60.0,
        now_s=100.0,
    )

    assert report.metrics["allow_physical_mapping"] is True
    assert "--allow-physical-mapping" in report.commands["rc_manual_flight_plan"]
    assert "--allow-physical-mapping" in report.commands["rc_manual_flight_live"]
    assert "--allow-physical-mapping" in report.commands["acceptance_chain_live"]
    assert "--allow-physical-mapping" not in report.commands["extended_follow_live"]
    assert "pr0p_tracking_acceptance_runner.py" in report.commands["extended_follow_live"]


def test_core_goal_report_cli_accepts_bbox_file(tmp_path):
    bbox_file = tmp_path / "target-bbox.json"
    bbox_file.write_text(json.dumps({"bbox": [10, 20, 80, 60]}), encoding="utf-8")

    args = parse_core_goal_args(["--bbox-file", str(bbox_file)])

    assert args.bbox == (10, 20, 80, 60)


def test_core_goal_report_cli_rejects_bbox_and_bbox_file(tmp_path):
    bbox_file = tmp_path / "target-bbox.json"
    bbox_file.write_text(json.dumps({"bbox": [10, 20, 80, 60]}), encoding="utf-8")

    with pytest.raises(SystemExit):
        parse_core_goal_args(["--bbox", "1,2,30,40", "--bbox-file", str(bbox_file)])


def test_moving_target_yaw_plan_waits_without_synthetic_execution(tmp_path):
    report = build_moving_target_yaw_plan(
        run_id="unit",
        log_dir=tmp_path,
        bbox=(10, 20, 80, 60),
        execute_synthetic=False,
        duration_s=1.0,
        hz=10.0,
        min_found_ratio=0.8,
        max_loss_events=0,
        min_center_motion_px=10.0,
        max_yaw_axis=0.8,
        desired_target_width=120.0,
        real_duration_s=30.0,
        real_min_found_ratio=0.9,
        real_max_loss_events=2,
        real_min_target_motion_px=30.0,
        real_max_yaw_axis=0.8,
        real_tracking_report=None,
        ack_real_moving_target=False,
    )

    steps = {step.name: step for step in report.steps}
    assert report.status == WAITING
    assert steps["synthetic_moving_target_yaw"].status == WAITING
    assert steps["real_pr0p_moving_target_yaw"].status == WAITING
    assert "pr0p_moving_target_acceptance_runner.py" in report.commands["real_pr0p_moving_target_yaw_live"]
    assert "--run-live-gate" in report.commands["real_pr0p_moving_target_yaw_live"]
    assert "--ack-airborne-static-target" in report.commands["real_pr0p_moving_target_yaw_live"]
    assert "--real-min-image-motion 30.0" in report.commands["real_pr0p_moving_target_yaw_live"]
    assert "--real-min-center-error-reduction 0.5" in report.commands["real_pr0p_moving_target_yaw_live"]
    assert "--execute-synthetic" in report.commands["synthetic_moving_target_yaw"]
    assert "--real-duration 30.0" in report.commands["real_pr0p_moving_target_yaw_live"]
    assert report.metrics["real_input_sent"] is False
    markdown = build_moving_target_yaw_markdown(report)
    assert "pr0p Airborne Static Target Yaw Plan" in markdown
    assert "real_pr0p_moving_target_yaw" in markdown


def test_moving_target_yaw_plan_runs_synthetic_yaw_only_regression(tmp_path):
    report = build_moving_target_yaw_plan(
        run_id="unit-moving",
        log_dir=tmp_path,
        bbox=(10, 20, 80, 60),
        execute_synthetic=True,
        duration_s=2.0,
        hz=15.0,
        min_found_ratio=0.8,
        max_loss_events=0,
        min_center_motion_px=15.0,
        max_yaw_axis=0.8,
        desired_target_width=120.0,
        real_duration_s=30.0,
        real_min_found_ratio=0.9,
        real_max_loss_events=2,
        real_min_target_motion_px=30.0,
        real_max_yaw_axis=0.8,
        real_tracking_report=None,
        ack_real_moving_target=False,
    )

    steps = {step.name: step for step in report.steps}
    synthetic = steps["synthetic_moving_target_yaw"]
    real = steps["real_pr0p_moving_target_yaw"]
    assert report.status == WAITING
    assert synthetic.status == PASS
    assert real.status == WAITING
    assert synthetic.metrics["enable_pitch"] is False
    assert synthetic.metrics["pitch_zero"] is True
    assert synthetic.metrics["center_motion_ok"] is True
    assert synthetic.metrics["found_ratio"] >= 0.8
    assert synthetic.metrics["loss_events"] == 0
    assert synthetic.metrics["real_input"] is False
    assert Path(synthetic.metrics["log_path"]).exists()
    assert real.metrics["enable_pitch"] is False
    assert "pitch/approach" in " ".join(real.notes)


def test_moving_target_yaw_plan_passes_with_real_moving_tracking_evidence(tmp_path):
    real_tracking = write_real_moving_tracking_evidence(tmp_path)

    report = build_moving_target_yaw_plan(
        run_id="unit-moving-real",
        log_dir=tmp_path,
        bbox=(10, 20, 80, 60),
        execute_synthetic=True,
        duration_s=2.0,
        hz=15.0,
        min_found_ratio=0.8,
        max_loss_events=0,
        min_center_motion_px=15.0,
        max_yaw_axis=0.8,
        desired_target_width=120.0,
        real_duration_s=30.0,
        real_min_found_ratio=0.9,
        real_max_loss_events=2,
        real_min_target_motion_px=30.0,
        real_max_yaw_axis=0.8,
        real_tracking_report=real_tracking,
        ack_real_moving_target=True,
    )

    steps = {step.name: step for step in report.steps}
    assert report.status == PASS
    assert steps["synthetic_moving_target_yaw"].status == PASS
    assert steps["real_pr0p_moving_target_yaw"].status == PASS
    assert steps["real_pr0p_moving_target_yaw"].metrics["target_center_span_px"] >= 30.0
    assert steps["real_pr0p_moving_target_yaw"].metrics["horizontal_error_reduction_ratio"] >= 0.5
    assert steps["real_pr0p_moving_target_yaw"].metrics["last_abs_horizontal_error"] <= 40.0
    assert steps["real_pr0p_moving_target_yaw"].metrics["probe_enable_pitch"] is False
    assert report.metrics["real_pr0p_status"] == PASS


def test_moving_target_yaw_plan_requires_real_moving_target_ack(tmp_path):
    real_tracking = write_real_moving_tracking_evidence(tmp_path)

    report = build_moving_target_yaw_plan(
        run_id="unit-moving-real-no-ack",
        log_dir=tmp_path,
        bbox=(10, 20, 80, 60),
        execute_synthetic=True,
        duration_s=2.0,
        hz=15.0,
        min_found_ratio=0.8,
        max_loss_events=0,
        min_center_motion_px=15.0,
        max_yaw_axis=0.8,
        desired_target_width=120.0,
        real_duration_s=30.0,
        real_min_found_ratio=0.9,
        real_max_loss_events=2,
        real_min_target_motion_px=30.0,
        real_max_yaw_axis=0.8,
        real_tracking_report=real_tracking,
        ack_real_moving_target=False,
    )

    steps = {step.name: step for step in report.steps}
    assert report.status == WAITING
    assert steps["real_pr0p_moving_target_yaw"].status == WAITING
    assert "ack_airborne_static_target" in steps["real_pr0p_moving_target_yaw"].notes


def test_moving_target_yaw_plan_rejects_screen_fixed_lock(tmp_path):
    real_tracking = write_real_moving_tracking_evidence(tmp_path, screen_fixed=True)

    report = build_moving_target_yaw_plan(
        run_id="unit-moving-screen-fixed",
        log_dir=tmp_path,
        bbox=(10, 20, 80, 60),
        execute_synthetic=True,
        duration_s=2.0,
        hz=15.0,
        min_found_ratio=0.8,
        max_loss_events=0,
        min_center_motion_px=15.0,
        max_yaw_axis=0.8,
        desired_target_width=120.0,
        real_duration_s=30.0,
        real_min_found_ratio=0.9,
        real_max_loss_events=2,
        real_min_target_motion_px=30.0,
        real_max_yaw_axis=0.8,
        real_tracking_report=real_tracking,
        ack_real_moving_target=True,
    )

    real = {step.name: step for step in report.steps}["real_pr0p_moving_target_yaw"]
    assert report.status == FAIL
    assert real.status == FAIL
    assert "real_screen_fixed_lock_suspected" in real.notes


def test_moving_target_yaw_plan_rejects_non_converging_static_target(tmp_path):
    real_tracking = write_real_moving_tracking_evidence(
        tmp_path,
        initial_error=100.0,
        final_error=80.0,
    )

    report = build_moving_target_yaw_plan(
        run_id="unit-moving-not-converged",
        log_dir=tmp_path,
        bbox=(10, 20, 80, 60),
        execute_synthetic=True,
        duration_s=2.0,
        hz=15.0,
        min_found_ratio=0.8,
        max_loss_events=0,
        min_center_motion_px=15.0,
        max_yaw_axis=0.8,
        desired_target_width=120.0,
        real_duration_s=30.0,
        real_min_found_ratio=0.9,
        real_max_loss_events=2,
        real_min_target_motion_px=30.0,
        real_max_yaw_axis=0.8,
        real_tracking_report=real_tracking,
        ack_real_moving_target=True,
    )

    real = {step.name: step for step in report.steps}["real_pr0p_moving_target_yaw"]
    assert report.status == FAIL
    assert "real_final_center_error_high" in real.notes
    assert "real_center_error_not_converged" in real.notes


def test_moving_target_yaw_cli_accepts_bbox_file(tmp_path):
    bbox_file = tmp_path / "target-bbox.json"
    bbox_file.write_text(json.dumps({"bbox": [10, 20, 80, 60]}), encoding="utf-8")

    args = parse_moving_target_yaw_args([
        "--bbox-file", str(bbox_file),
        "--ack-airborne-static-target",
    ])

    assert args.bbox == (10, 20, 80, 60)
    assert args.ack_airborne_static_target is True


def test_moving_target_acceptance_runner_plans_after_extended_follow_passes(tmp_path):
    write_tracking_acceptance(
        tmp_path / "20260707-unit-tracking-acceptance.json",
        duration_s=20.0,
        min_found_ratio=0.95,
        max_loss_events=0,
    )

    report = build_moving_target_acceptance_for_test(tmp_path)

    steps = {step.name: step for step in report.steps}
    assert report.status == WAITING
    assert steps["moving_target_bbox"].status == PASS
    assert steps["extended_follow_prerequisite"].status == PASS
    assert steps["moving_target_tracking_probe"].status == WAITING
    assert steps["real_pr0p_moving_target_yaw"].status == WAITING
    command = report.commands["real_pr0p_moving_target_yaw_live"]
    assert "pr0p_tracking_acceptance_runner.py" in command
    assert "--run-live-gates" in command
    assert "--ack-live-input" in command
    assert "--arm-first" in command
    assert "--takeoff-delay-frames 10" in command
    assert "--takeoff-boost-frames 10" in command
    assert "--settle-throttle -0.26" in command
    assert "--enable-pitch" not in command
    assert "--max-abs-pitch-axis 0.0" in command
    markdown = build_moving_target_acceptance_markdown(report)
    assert "pr0p Real Airborne Static Target Yaw Acceptance Runner" in markdown


def test_moving_target_acceptance_runner_builds_real_evidence_from_tracking(tmp_path):
    write_tracking_acceptance(
        tmp_path / "20260707-unit-tracking-acceptance.json",
        duration_s=20.0,
        min_found_ratio=0.95,
        max_loss_events=0,
    )
    tracking_report = write_real_moving_tracking_evidence(tmp_path)

    report = build_moving_target_acceptance_for_test(
        tmp_path,
        tracking_report=tracking_report,
        ack_real_moving_target=True,
    )

    steps = {step.name: step for step in report.steps}
    assert report.status == PASS
    assert steps["moving_target_tracking_probe"].status == PASS
    assert steps["real_pr0p_moving_target_yaw"].status == PASS
    assert steps["real_pr0p_moving_target_yaw"].metrics["target_center_span_px"] >= 30.0
    assert steps["real_pr0p_moving_target_yaw"].metrics["horizontal_error_reduction_ratio"] >= 0.5
    assert steps["real_pr0p_moving_target_yaw"].metrics["last_abs_horizontal_error"] <= 40.0
    assert steps["real_pr0p_moving_target_yaw"].metrics["probe_enable_pitch"] is False
    assert report.metrics["real_status"] == PASS


def test_moving_target_acceptance_runner_requires_ack(tmp_path):
    write_tracking_acceptance(
        tmp_path / "20260707-unit-tracking-acceptance.json",
        duration_s=20.0,
        min_found_ratio=0.95,
        max_loss_events=0,
    )
    tracking_report = write_real_moving_tracking_evidence(tmp_path)

    report = build_moving_target_acceptance_for_test(
        tmp_path,
        tracking_report=tracking_report,
        ack_real_moving_target=False,
    )

    steps = {step.name: step for step in report.steps}
    assert report.status == WAITING
    assert steps["real_pr0p_moving_target_yaw"].status == WAITING
    assert "ack_airborne_static_target" in steps["real_pr0p_moving_target_yaw"].notes


def test_moving_target_acceptance_runner_executes_live_command_with_ack(tmp_path):
    write_tracking_acceptance(
        tmp_path / "20260707-unit-tracking-acceptance.json",
        duration_s=20.0,
        min_found_ratio=0.95,
        max_loss_events=0,
    )
    tracking_report = write_real_moving_tracking_evidence(tmp_path)
    calls = []

    def fake_runner(argv, **_kwargs):
        text = " ".join(argv)
        calls.append(text)
        assert "pr0p_tracking_acceptance_runner.py" in text
        assert "--run-live-gates" in text
        assert "--ack-live-input" in text
        assert "--takeoff-delay-frames 10" in text
        assert "--settle-throttle -0.26" in text
        assert "--enable-pitch" not in text
        stdout = (
            "simitl-pr0p-tracking-acceptance PASS "
            "report=%s summary=tracking.md\n" % tracking_report
        )
        return subprocess.CompletedProcess(argv, 0, stdout=stdout, stderr="")

    report = build_moving_target_acceptance_for_test(
        tmp_path,
        run_live_gate=True,
        ack_live_input=True,
        ack_real_moving_target=True,
        command_runner=fake_runner,
    )

    steps = {step.name: step for step in report.steps}
    assert report.status == PASS
    assert steps["moving_target_tracking_probe"].status == PASS
    assert steps["real_pr0p_moving_target_yaw"].status == PASS
    assert report.metrics["tracking_report"] == str(tracking_report)
    assert calls


def test_moving_target_acceptance_cli_guards_live_input():
    with pytest.raises(SystemExit):
        parse_moving_target_acceptance_args(["--run-live-gate", "--ack-airborne-static-target"])

    with pytest.raises(SystemExit):
        parse_moving_target_acceptance_args(["--run-live-gate", "--ack-live-input"])


def test_moving_target_acceptance_cli_accepts_bbox_file(tmp_path):
    bbox_file = tmp_path / "target-bbox.json"
    bbox_file.write_text(json.dumps({"bbox": [10, 20, 80, 60]}), encoding="utf-8")

    args = parse_moving_target_acceptance_args(["--bbox-file", str(bbox_file)])

    assert args.bbox == (10, 20, 80, 60)

    ack_args = parse_moving_target_acceptance_args([
        "--bbox-file", str(bbox_file),
        "--ack-airborne-static-target",
    ])
    assert ack_args.ack_airborne_static_target is True


def test_approach_pitch_plan_waits_without_synthetic_execution(tmp_path):
    report = build_approach_pitch_plan(
        run_id="unit",
        log_dir=tmp_path,
        bbox=(10, 20, 80, 60),
        execute_synthetic=False,
        duration_s=1.0,
        hz=10.0,
        min_found_ratio=0.8,
        max_loss_events=0,
        initial_target_width=70.0,
        desired_target_width=120.0,
        max_final_width_error=8.0,
        min_width_error_reduction=0.65,
        max_yaw_axis=0.8,
        max_pitch_axis=0.8,
        real_duration_s=20.0,
        real_min_found_ratio=0.9,
        real_max_loss_events=2,
        real_max_final_width_error=12.0,
        real_min_width_error_reduction=0.5,
        real_min_initial_width_error=10.0,
        real_max_pitch_axis=0.8,
        real_min_pitch_axis=0.02,
        real_tracking_report=None,
        ack_real_approach_target=False,
    )

    steps = {step.name: step for step in report.steps}
    assert report.status == WAITING
    assert steps["synthetic_range_approach"].status == WAITING
    assert steps["real_pr0p_approach_pitch"].status == WAITING
    assert "--execute-synthetic" in report.commands["synthetic_range_approach"]
    assert "--enable-pitch" in report.commands["real_pr0p_approach_pitch_live"]
    assert report.metrics["real_input_sent"] is False
    markdown = build_approach_pitch_markdown(report)
    assert "pr0p Approach Pitch Plan" in markdown
    assert "real_pr0p_approach_pitch" in markdown


def test_approach_pitch_plan_runs_synthetic_range_regression(tmp_path):
    report = build_approach_pitch_plan(
        run_id="unit-approach",
        log_dir=tmp_path,
        bbox=(10, 20, 80, 60),
        execute_synthetic=True,
        duration_s=5.0,
        hz=20.0,
        min_found_ratio=0.9,
        max_loss_events=0,
        initial_target_width=70.0,
        desired_target_width=120.0,
        max_final_width_error=8.0,
        min_width_error_reduction=0.65,
        max_yaw_axis=0.8,
        max_pitch_axis=0.8,
        real_duration_s=20.0,
        real_min_found_ratio=0.9,
        real_max_loss_events=2,
        real_max_final_width_error=12.0,
        real_min_width_error_reduction=0.5,
        real_min_initial_width_error=10.0,
        real_max_pitch_axis=0.8,
        real_min_pitch_axis=0.02,
        real_tracking_report=None,
        ack_real_approach_target=False,
    )

    steps = {step.name: step for step in report.steps}
    synthetic = steps["synthetic_range_approach"]
    real = steps["real_pr0p_approach_pitch"]
    assert report.status == WAITING
    assert synthetic.status == PASS
    assert real.status == WAITING
    assert synthetic.metrics["range_control_applied_to_target_width"] is True
    assert synthetic.metrics["final_abs_width_error_px"] <= 8.0
    assert synthetic.metrics["width_error_reduction_ratio"] >= 0.65
    assert synthetic.metrics["max_abs_pitch_axis"] > 0.0
    assert synthetic.metrics["real_input"] is False
    assert Path(synthetic.metrics["log_path"]).exists()


def test_approach_pitch_plan_passes_with_real_approach_evidence(tmp_path):
    real_tracking = write_real_approach_tracking_evidence(tmp_path)

    report = build_approach_pitch_plan(
        run_id="unit-approach-real",
        log_dir=tmp_path,
        bbox=(10, 20, 80, 60),
        execute_synthetic=True,
        duration_s=5.0,
        hz=20.0,
        min_found_ratio=0.9,
        max_loss_events=0,
        initial_target_width=70.0,
        desired_target_width=120.0,
        max_final_width_error=8.0,
        min_width_error_reduction=0.65,
        max_yaw_axis=0.8,
        max_pitch_axis=0.8,
        real_duration_s=20.0,
        real_min_found_ratio=0.9,
        real_max_loss_events=2,
        real_max_final_width_error=12.0,
        real_min_width_error_reduction=0.5,
        real_min_initial_width_error=10.0,
        real_max_pitch_axis=0.8,
        real_min_pitch_axis=0.02,
        real_tracking_report=real_tracking,
        ack_real_approach_target=True,
    )

    steps = {step.name: step for step in report.steps}
    assert report.status == PASS
    assert steps["synthetic_range_approach"].status == PASS
    assert steps["real_pr0p_approach_pitch"].status == PASS
    assert steps["real_pr0p_approach_pitch"].metrics["width_error_reduction_ratio"] >= 0.5
    assert steps["real_pr0p_approach_pitch"].metrics["probe_enable_pitch"] is True


def test_approach_pitch_plan_requires_real_approach_ack(tmp_path):
    real_tracking = write_real_approach_tracking_evidence(tmp_path)

    report = build_approach_pitch_plan(
        run_id="unit-approach-no-ack",
        log_dir=tmp_path,
        bbox=(10, 20, 80, 60),
        execute_synthetic=True,
        duration_s=5.0,
        hz=20.0,
        min_found_ratio=0.9,
        max_loss_events=0,
        initial_target_width=70.0,
        desired_target_width=120.0,
        max_final_width_error=8.0,
        min_width_error_reduction=0.65,
        max_yaw_axis=0.8,
        max_pitch_axis=0.8,
        real_duration_s=20.0,
        real_min_found_ratio=0.9,
        real_max_loss_events=2,
        real_max_final_width_error=12.0,
        real_min_width_error_reduction=0.5,
        real_min_initial_width_error=10.0,
        real_max_pitch_axis=0.8,
        real_min_pitch_axis=0.02,
        real_tracking_report=real_tracking,
        ack_real_approach_target=False,
    )

    steps = {step.name: step for step in report.steps}
    assert report.status == WAITING
    assert steps["real_pr0p_approach_pitch"].status == WAITING
    assert "ack_real_approach_target" in steps["real_pr0p_approach_pitch"].notes


def test_approach_pitch_cli_accepts_bbox_file(tmp_path):
    bbox_file = tmp_path / "target-bbox.json"
    bbox_file.write_text(json.dumps({"bbox": [10, 20, 80, 60]}), encoding="utf-8")

    args = parse_approach_pitch_args(["--bbox-file", str(bbox_file)])

    assert args.bbox == (10, 20, 80, 60)


def test_handoff_plan_waits_without_synthetic_execution(tmp_path):
    report = build_handoff_plan_for_test(tmp_path, execute_synthetic=False)

    steps = {step.name: step for step in report.steps}
    assert report.status == WAITING
    assert steps["synthetic_manual_to_auto_handoff"].status == WAITING
    assert steps["real_pr0p_manual_to_auto_handoff"].status == WAITING
    assert "pr0p_handoff_acceptance_runner.py" in report.commands["real_pr0p_handoff_live"]
    assert "--execute-synthetic" in report.commands["synthetic_manual_to_auto_handoff"]
    assert "--ack-real-handoff" in report.commands["real_pr0p_handoff_refresh"]
    assert report.metrics["real_input_sent"] is False
    markdown = build_handoff_markdown(report)
    assert "pr0p Manual To Autonomous Handoff Plan" in markdown
    assert "real_pr0p_manual_to_auto_handoff" in markdown


def test_handoff_plan_runs_synthetic_handoff_regression(tmp_path):
    report = build_handoff_plan_for_test(
        tmp_path,
        run_id="unit-handoff-synthetic",
        execute_synthetic=True,
    )

    steps = {step.name: step for step in report.steps}
    synthetic = steps["synthetic_manual_to_auto_handoff"]
    real = steps["real_pr0p_manual_to_auto_handoff"]
    assert report.status == WAITING
    assert synthetic.status == PASS
    assert real.status == WAITING
    assert synthetic.metrics["manual_to_auto_handoff"] is True
    assert synthetic.metrics["pre_handoff_auto_control_samples"] == 0
    assert synthetic.metrics["follow_command_sent"] is True
    assert synthetic.metrics["tracker_initialized_after_follow"] is True
    assert synthetic.metrics["manual_final_abs_error_px"] <= 45.0
    assert synthetic.metrics["final_abs_error_px"] <= 35.0
    assert synthetic.metrics["final_abs_width_error_px"] <= 8.0
    assert synthetic.metrics["real_input"] is False
    assert Path(synthetic.metrics["log_path"]).exists()


def test_handoff_plan_passes_with_real_handoff_evidence(tmp_path):
    real_handoff = write_real_handoff_evidence(tmp_path)

    report = build_handoff_plan_for_test(
        tmp_path,
        run_id="unit-handoff-real",
        execute_synthetic=True,
        real_handoff_report=real_handoff,
        ack_real_handoff=True,
    )

    steps = {step.name: step for step in report.steps}
    assert report.status == PASS
    assert steps["synthetic_manual_to_auto_handoff"].status == PASS
    assert steps["real_pr0p_manual_to_auto_handoff"].status == PASS
    assert steps["real_pr0p_manual_to_auto_handoff"].metrics["follow_command_sent"] is True
    assert steps["real_pr0p_manual_to_auto_handoff"].metrics["pre_handoff_auto_control_samples"] == 0
    assert steps["real_pr0p_manual_to_auto_handoff"].metrics["real_input_sent"] is True


def test_handoff_plan_requires_real_handoff_ack(tmp_path):
    real_handoff = write_real_handoff_evidence(tmp_path)

    report = build_handoff_plan_for_test(
        tmp_path,
        run_id="unit-handoff-no-ack",
        execute_synthetic=True,
        real_handoff_report=real_handoff,
        ack_real_handoff=False,
    )

    steps = {step.name: step for step in report.steps}
    assert report.status == WAITING
    assert steps["real_pr0p_manual_to_auto_handoff"].status == WAITING
    assert "ack_real_handoff" in steps["real_pr0p_manual_to_auto_handoff"].notes


def test_handoff_cli_accepts_bbox_file(tmp_path):
    bbox_file = tmp_path / "target-bbox.json"
    bbox_file.write_text(json.dumps({"bbox": [10, 20, 80, 60]}), encoding="utf-8")

    args = parse_handoff_args(["--bbox-file", str(bbox_file)])

    assert args.bbox == (10, 20, 80, 60)


def test_handoff_acceptance_runner_plans_live_handoff_after_approach_passes(tmp_path):
    write_approach_pitch(
        tmp_path / "20260707-unit-approach-pitch.json",
        status=PASS,
        synthetic_status=PASS,
        real_status=PASS,
    )

    report = build_handoff_acceptance_for_test(tmp_path)

    steps = {step.name: step for step in report.steps}
    assert report.status == WAITING
    assert steps["handoff_bbox"].status == PASS
    assert steps["approach_pitch_prerequisite"].status == PASS
    assert steps["follow_command_tracking_probe"].status == WAITING
    assert steps["real_handoff_report"].status == WAITING
    command = report.commands["real_pr0p_handoff_live"]
    assert "pr0p_tracking_probe.py" in command
    assert "--enable-pitch" in command
    assert "--uinput" in command
    assert "--ack-live-input" in command
    assert "--arm-first" in command
    assert report.metrics["real_input_sent"] is False
    markdown = build_handoff_acceptance_markdown(report)
    assert "pr0p Real Handoff Acceptance Runner" in markdown


def test_handoff_acceptance_runner_builds_real_report_from_tracking_probe(tmp_path):
    write_approach_pitch(
        tmp_path / "20260707-unit-approach-pitch.json",
        status=PASS,
        synthetic_status=PASS,
        real_status=PASS,
    )
    tracking_report = write_handoff_tracking_probe(tmp_path / "tracking-probe.json")

    report = build_handoff_acceptance_for_test(
        tmp_path,
        tracking_report=tracking_report,
        ack_real_handoff=True,
    )

    steps = {step.name: step for step in report.steps}
    assert report.status == PASS
    assert steps["real_handoff_report"].status == PASS
    assert report.metrics["follow_command_sent"] is True
    assert report.metrics["tracker_initialized_before_follow"] is False
    assert report.metrics["tracker_initialized_after_follow"] is True
    assert report.metrics["pre_handoff_auto_control_samples"] == 0
    assert report.metrics["manual_to_auto_handoff"] is True
    assert report.metrics["real_input_sent"] is True
    assert report.metrics["manual_to_final_error_reduction_ratio"] >= 0.5
    assert report.metrics["width_error_reduction_ratio"] >= 0.45


def test_handoff_acceptance_runner_requires_real_handoff_ack(tmp_path):
    write_approach_pitch(
        tmp_path / "20260707-unit-approach-pitch.json",
        status=PASS,
        synthetic_status=PASS,
        real_status=PASS,
    )
    tracking_report = write_handoff_tracking_probe(tmp_path / "tracking-probe.json")

    report = build_handoff_acceptance_for_test(
        tmp_path,
        tracking_report=tracking_report,
        ack_real_handoff=False,
    )

    steps = {step.name: step for step in report.steps}
    assert report.status == WAITING
    assert steps["real_handoff_report"].status == WAITING
    assert "ACK_REAL_HANDOFF_REQUIRED" in steps["real_handoff_report"].notes


def test_handoff_acceptance_runner_executes_live_command_with_ack(tmp_path):
    write_approach_pitch(
        tmp_path / "20260707-unit-approach-pitch.json",
        status=PASS,
        synthetic_status=PASS,
        real_status=PASS,
    )
    tracking_report = write_handoff_tracking_probe(tmp_path / "tracking-probe.json")
    calls = []

    def fake_runner(argv, **_kwargs):
        text = " ".join(argv)
        calls.append(text)
        assert "pr0p_tracking_probe.py" in text
        assert "--uinput" in text
        assert "--ack-live-input" in text
        assert "--enable-pitch" in text
        stdout = "simitl-pr0p-tracking PASS report=%s summary=tracking.md\n" % tracking_report
        return subprocess.CompletedProcess(argv, 0, stdout=stdout, stderr="")

    report = build_handoff_acceptance_for_test(
        tmp_path,
        run_live_gate=True,
        ack_live_input=True,
        ack_real_handoff=True,
        command_runner=fake_runner,
    )

    steps = {step.name: step for step in report.steps}
    assert report.status == PASS
    assert steps["follow_command_tracking_probe"].status == PASS
    assert steps["real_handoff_report"].status == PASS
    assert report.metrics["tracking_probe_report"] == str(tracking_report)
    assert calls


def test_handoff_acceptance_cli_guards_live_input():
    with pytest.raises(SystemExit):
        parse_handoff_acceptance_args(["--run-live-gate", "--ack-real-handoff"])

    with pytest.raises(SystemExit):
        parse_handoff_acceptance_args(["--run-live-gate", "--ack-live-input"])


def test_handoff_acceptance_cli_accepts_bbox_file(tmp_path):
    bbox_file = tmp_path / "target-bbox.json"
    bbox_file.write_text(json.dumps({"bbox": [10, 20, 80, 60]}), encoding="utf-8")

    args = parse_handoff_acceptance_args(["--bbox-file", str(bbox_file)])

    assert args.bbox == (10, 20, 80, 60)


def write_airborne_static_live_waiting_prereqs(tmp_path):
    write_pr0p_suite(tmp_path / "20260707-pr0p-suite.json", readiness_promotion_steps())
    write_pr0p_manifest(
        tmp_path / "20260707-pr0p-live-manifest.json",
        readiness_promotion_steps(),
        suite_report=tmp_path / "20260707-pr0p-suite.json",
    )
    write_aux1_acceptance(tmp_path / "20260707-unit-aux1-acceptance.json")
    write_response_acceptance(tmp_path / "20260707-unit-response-acceptance.json")
    write_tracking_acceptance(
        tmp_path / "20260707-unit-tracking-acceptance.json",
        duration_s=20.0,
        min_found_ratio=0.95,
        max_loss_events=0,
    )
    write_moving_target_yaw(
        tmp_path / "20260707-unit-moving-target-yaw.json",
        status=WAITING,
        synthetic_status=PASS,
        real_status=WAITING,
    )


def test_core_next_runner_plan_only_does_not_execute_live_next(tmp_path):
    def forbidden_runner(*_args, **_kwargs):
        raise AssertionError("plan-only core next runner must not execute commands")

    report = run_core_next(
        run_id="unit",
        log_dir=tmp_path,
        bbox=(10, 20, 80, 60),
        max_evidence_age_s=60.0,
        execute_next=False,
        ack_live_command=False,
        timeout_s=1.0,
        command_runner=forbidden_runner,
    )

    assert report.status == "PLANNED"
    assert report.next_command_key == "rc_manual_flight_live"
    assert report.command_is_live is True
    assert report.executed is False
    assert report.core_report is not None
    assert report.metrics["core_completion_status"] == "IN_PROGRESS"
    assert "rc_manual_flight" in report.metrics["core_completion_missing"]
    assert report.metrics["isolation_status"] == PASS
    assert report.metrics["isolation_violation_count"] == 0
    assert report.metrics["isolation_report"].endswith("-isolation-precheck-isolation-check.json")
    assert report.metrics["next_objective_gate"] == "rc_manual_flight"
    assert report.metrics["next_objective_gate_status"] == WAITING
    assert "P2-websocket" in report.metrics["next_objective_gate_missing"]
    assert report.metrics["live_precheck_status"] == "NOT_CHECKED"
    assert report.metrics["safe_to_execute_live_with_ack"] is False
    assert "INDEPENDENT_READINESS_NOT_CHECKED" in report.metrics["safe_to_execute_live_blockers"]
    markdown = build_core_next_markdown(report)
    assert "pr0p Core Next Runner" in markdown
    assert "LIVE_COMMAND" in markdown


def test_core_next_runner_blocks_when_isolation_precheck_fails(monkeypatch, tmp_path):
    monkeypatch.setattr(
        pr0p_core_next_module,
        "isolation_precheck",
        lambda **_kwargs: {
            "status": FAIL,
            "summary": "isolated code references Gazebo",
            "report": "logs/isolation-fail.json",
            "summary_report": "logs/isolation-fail.md",
            "notes": ["GAZEBO_COUPLING_DETECTED"],
            "violation_count": 1,
        },
    )

    def forbidden_runner(*_args, **_kwargs):
        raise AssertionError("core-next must not run commands when isolation fails")

    report = run_core_next(
        run_id="unit",
        log_dir=tmp_path,
        bbox=(10, 20, 80, 60),
        max_evidence_age_s=60.0,
        execute_next=True,
        ack_live_command=True,
        timeout_s=1.0,
        command_runner=forbidden_runner,
    )

    assert report.status == FAIL
    assert report.executed is False
    assert report.notes == ["GAZEBO_INDEPENDENCE_FAILED"]
    assert report.metrics["isolation_status"] == FAIL
    assert report.metrics["isolation_violation_count"] == 1
    assert report.metrics["isolation_report"] == "logs/isolation-fail.json"
    assert report.next_action.startswith("Inspect the isolation report")


def test_core_next_runner_requires_ack_for_live_next(tmp_path):
    def forbidden_runner(*_args, **_kwargs):
        raise AssertionError("live next command must not run without ack")

    report = run_core_next(
        run_id="unit",
        log_dir=tmp_path,
        bbox=(10, 20, 80, 60),
        max_evidence_age_s=60.0,
        execute_next=True,
        ack_live_command=False,
        timeout_s=1.0,
        command_runner=forbidden_runner,
    )

    assert report.status == WAITING
    assert report.next_command_key == "rc_manual_flight_live"
    assert report.command_is_live is True
    assert report.executed is False
    assert "LIVE_COMMAND_REQUIRES_ACK" in report.notes


def test_core_next_runner_blocks_placeholder_command_before_live_ack(tmp_path):
    def forbidden_runner(*_args, **_kwargs):
        raise AssertionError("placeholder command must not execute")

    report = run_core_next(
        run_id="unit",
        log_dir=tmp_path,
        bbox=None,
        max_evidence_age_s=60.0,
        execute_next=True,
        ack_live_command=True,
        timeout_s=1.0,
        command_runner=forbidden_runner,
    )

    assert report.status == WAITING
    assert report.executed is False
    assert report.next_command_key == "rc_manual_flight_live"
    assert "x,y,w,h" in report.next_command
    assert "PLACEHOLDER_COMMAND" in report.notes
    assert report.metrics["command_has_placeholder"] is True


def test_core_next_runner_executes_dry_next_without_live_ack(tmp_path):
    write_rc_manual_flight(tmp_path / "20260707-unit-rc-manual-flight.json")
    calls = []

    def fake_runner(argv, **_kwargs):
        text = " ".join(argv)
        calls.append(text)
        assert "--execute-dry-run" in text
        assert "--run-live-gates" not in text
        stdout = "simitl-pr0p-tracking-acceptance PASS report=logs/tracking-dry.json summary=logs/tracking-dry.md\n"
        return subprocess.CompletedProcess(argv, 0, stdout=stdout, stderr="")

    report = run_core_next(
        run_id="unit",
        log_dir=tmp_path,
        bbox=(10, 20, 80, 60),
        max_evidence_age_s=600.0,
        execute_next=True,
        ack_live_command=False,
        timeout_s=1.0,
        command_runner=fake_runner,
    )

    assert report.status == PASS
    assert report.next_command_key == "tracking_acceptance_dry"
    assert report.command_is_live is False
    assert report.executed is True
    assert report.command_report == "logs/tracking-dry.json"
    assert calls


def test_core_next_runner_executes_moving_target_plan_after_extended_follow(tmp_path):
    write_pr0p_suite(tmp_path / "20260707-pr0p-suite.json", readiness_promotion_steps())
    write_aux1_acceptance(tmp_path / "20260707-unit-aux1-acceptance.json")
    write_response_acceptance(tmp_path / "20260707-unit-response-acceptance.json")
    write_tracking_acceptance(
        tmp_path / "20260707-unit-tracking-acceptance.json",
        duration_s=20.0,
        min_found_ratio=0.95,
        max_loss_events=0,
    )
    calls = []

    def fake_runner(argv, **_kwargs):
        text = " ".join(argv)
        calls.append(text)
        assert "pr0p_moving_target_yaw_plan.py" in text
        assert "--execute-synthetic" in text
        assert "--run-live-gates" not in text
        stdout = (
            "simitl-pr0p-moving-target-yaw WAITING "
            "report=logs/moving-target-yaw.json summary=logs/moving-target-yaw.md\n"
        )
        return subprocess.CompletedProcess(argv, 0, stdout=stdout, stderr="")

    report = run_core_next(
        run_id="unit",
        log_dir=tmp_path,
        bbox=(10, 20, 80, 60),
        max_evidence_age_s=600.0,
        execute_next=True,
        ack_live_command=False,
        timeout_s=1.0,
        command_runner=fake_runner,
    )

    assert report.status == WAITING
    assert report.next_command_key == "moving_target_yaw_plan"
    assert report.command_is_live is False
    assert report.executed is True
    assert report.command_report == "logs/moving-target-yaw.json"
    assert calls


def test_core_next_runner_requires_ack_for_airborne_static_live_after_synthetic_pass(tmp_path):
    write_pr0p_suite(tmp_path / "20260707-pr0p-suite.json", readiness_promotion_steps())
    write_aux1_acceptance(tmp_path / "20260707-unit-aux1-acceptance.json")
    write_response_acceptance(tmp_path / "20260707-unit-response-acceptance.json")
    write_tracking_acceptance(
        tmp_path / "20260707-unit-tracking-acceptance.json",
        duration_s=20.0,
        min_found_ratio=0.95,
        max_loss_events=0,
    )
    write_moving_target_yaw(
        tmp_path / "20260707-unit-moving-target-yaw.json",
        status=WAITING,
        synthetic_status=PASS,
        real_status=WAITING,
    )

    def forbidden_runner(*_args, **_kwargs):
        raise AssertionError("airborne static live command must require explicit ack")

    report = run_core_next(
        run_id="unit",
        log_dir=tmp_path,
        bbox=(10, 20, 80, 60),
        max_evidence_age_s=600.0,
        execute_next=True,
        ack_live_command=False,
        timeout_s=1.0,
        command_runner=forbidden_runner,
    )

    assert report.status == WAITING
    assert report.next_command_key == "airborne_static_yaw_live"
    assert report.command_is_live is True
    assert report.executed is False
    assert "LIVE_COMMAND_REQUIRES_ACK" in report.notes
    assert "pr0p_moving_target_acceptance_runner.py" in report.next_command
    assert "--ack-airborne-static-target" in report.next_command


def test_core_next_runner_blocks_live_execution_when_independent_readiness_missing(tmp_path):
    write_pr0p_suite(tmp_path / "20260707-pr0p-suite.json", readiness_promotion_steps())
    write_aux1_acceptance(tmp_path / "20260707-unit-aux1-acceptance.json")
    write_response_acceptance(tmp_path / "20260707-unit-response-acceptance.json")
    write_tracking_acceptance(
        tmp_path / "20260707-unit-tracking-acceptance.json",
        duration_s=20.0,
        min_found_ratio=0.95,
        max_loss_events=0,
    )
    write_moving_target_yaw(
        tmp_path / "20260707-unit-moving-target-yaw.json",
        status=WAITING,
        synthetic_status=PASS,
        real_status=WAITING,
    )

    def forbidden_runner(*_args, **_kwargs):
        raise AssertionError("live command must not run without independent readiness")

    report = run_core_next(
        run_id="unit",
        log_dir=tmp_path,
        game_log_dir=tmp_path / "missing-game-screen",
        bbox=(10, 20, 80, 60),
        max_evidence_age_s=600.0,
        execute_next=True,
        ack_live_command=True,
        timeout_s=1.0,
        command_runner=forbidden_runner,
    )

    assert report.status == WAITING
    assert report.next_command_key == "airborne_static_yaw_live"
    assert report.command_is_live is True
    assert report.executed is False
    assert "INDEPENDENT_READINESS_NOT_READY" in report.notes
    assert report.metrics["independent_readiness_precheck"]["status"] == WAITING
    assert report.metrics["independent_readiness_precheck"]["objective_completion_status"] == "IN_PROGRESS"
    assert report.metrics["live_precheck_status"] == WAITING
    assert report.metrics["safe_to_execute_live_with_ack"] is False
    assert report.metrics["safe_to_execute_live_blockers"] == ["INDEPENDENT_READINESS_NOT_READY"]
    assert report.metrics["safe_to_execute_live_with_all_prechecks"] is False
    assert report.metrics["safe_to_execute_live_required_blockers"] == [
        "INDEPENDENT_READINESS_NOT_READY",
        "OPERATOR_READINESS_NOT_CHECKED",
    ]
    assert "game-screen acceptance" in report.next_action


def test_core_next_runner_prechecks_live_readiness_without_executing(tmp_path):
    write_pr0p_suite(tmp_path / "20260707-pr0p-suite.json", readiness_promotion_steps())
    write_aux1_acceptance(tmp_path / "20260707-unit-aux1-acceptance.json")
    write_response_acceptance(tmp_path / "20260707-unit-response-acceptance.json")
    write_tracking_acceptance(
        tmp_path / "20260707-unit-tracking-acceptance.json",
        duration_s=20.0,
        min_found_ratio=0.95,
        max_loss_events=0,
    )
    write_moving_target_yaw(
        tmp_path / "20260707-unit-moving-target-yaw.json",
        status=WAITING,
        synthetic_status=PASS,
        real_status=WAITING,
    )

    def forbidden_runner(*_args, **_kwargs):
        raise AssertionError("plan precheck must not execute the live command")

    report = run_core_next(
        run_id="unit",
        log_dir=tmp_path,
        game_log_dir=tmp_path / "missing-game-screen",
        bbox=(10, 20, 80, 60),
        max_evidence_age_s=600.0,
        execute_next=False,
        ack_live_command=False,
        timeout_s=1.0,
        precheck_live_readiness=True,
        command_runner=forbidden_runner,
    )

    assert report.status == WAITING
    assert report.next_command_key == "airborne_static_yaw_live"
    assert report.command_is_live is True
    assert report.executed is False
    assert "PLAN_ONLY" in report.notes
    assert "INDEPENDENT_READINESS_NOT_READY" in report.notes
    assert report.metrics["precheck_live_readiness"] is True
    assert report.metrics["independent_readiness_precheck"]["status"] == WAITING
    assert report.metrics["live_precheck_status"] == WAITING
    assert report.metrics["safe_to_execute_live_with_ack"] is False
    assert report.metrics["safe_to_execute_live_blockers"] == ["INDEPENDENT_READINESS_NOT_READY"]
    assert report.metrics["safe_to_execute_live_with_all_prechecks"] is False
    assert report.metrics["safe_to_execute_live_required_blockers"] == [
        "INDEPENDENT_READINESS_NOT_READY",
        "OPERATOR_READINESS_NOT_CHECKED",
    ]
    assert "game-screen acceptance" in report.next_action


def test_core_next_runner_precheck_keeps_live_plan_when_readiness_ready(tmp_path):
    game_dir = tmp_path / "game"
    game_dir.mkdir()
    write_game_decision(game_dir / "20260705-game-decision.json")
    write_pr0p_suite(tmp_path / "20260707-pr0p-suite.json", readiness_promotion_steps())
    write_pr0p_manifest(
        tmp_path / "20260707-pr0p-live-manifest.json",
        readiness_promotion_steps(),
        suite_report=tmp_path / "20260707-pr0p-suite.json",
    )
    write_aux1_acceptance(tmp_path / "20260707-unit-aux1-acceptance.json")
    write_response_acceptance(tmp_path / "20260707-unit-response-acceptance.json")
    write_tracking_acceptance(
        tmp_path / "20260707-unit-tracking-acceptance.json",
        duration_s=20.0,
        min_found_ratio=0.95,
        max_loss_events=0,
    )
    write_moving_target_yaw(
        tmp_path / "20260707-unit-moving-target-yaw.json",
        status=WAITING,
        synthetic_status=PASS,
        real_status=WAITING,
    )

    def forbidden_runner(*_args, **_kwargs):
        raise AssertionError("plan precheck must not execute the live command")

    report = run_core_next(
        run_id="unit",
        log_dir=tmp_path,
        game_log_dir=game_dir,
        bbox=(10, 20, 80, 60),
        max_evidence_age_s=600.0,
        execute_next=False,
        ack_live_command=False,
        timeout_s=1.0,
        precheck_live_readiness=True,
        command_runner=forbidden_runner,
    )

    assert report.status == "PLANNED"
    assert report.next_command_key == "airborne_static_yaw_live"
    assert report.command_is_live is True
    assert report.executed is False
    assert "INDEPENDENT_READINESS_READY" in report.notes
    assert report.metrics["independent_readiness_precheck"]["status"] == INDEPENDENT_SIM_READY
    assert report.metrics["next_objective_gate"] == "moving_target_yaw"
    assert report.metrics["next_objective_gate_status"] == WAITING
    assert "real_pr0p_moving_target_yaw" in report.metrics["next_objective_gate_missing"]
    assert "moving_target_yaw_pass" in report.metrics["next_objective_gate_missing"]
    assert report.metrics["objective_requirement_statuses"]["airborne_static_yaw_centering"] == WAITING
    assert report.metrics["objective_requirement_missing"] == [
        "airborne_static_yaw_centering",
        "pitch_approach_control",
        "manual_to_auto_handoff",
    ]
    assert report.metrics["objective_requirement_failed"] == []
    assert report.metrics["live_precheck_status"] == INDEPENDENT_SIM_READY
    assert report.metrics["safe_to_execute_live_with_ack"] is True
    assert report.metrics["safe_to_execute_live_blockers"] == []
    assert report.metrics["safe_to_execute_live_with_all_prechecks"] is False
    assert report.metrics["safe_to_execute_live_required_blockers"] == [
        "OPERATOR_READINESS_NOT_CHECKED",
    ]


def test_core_next_runner_prechecks_operator_readiness_without_executing(monkeypatch, tmp_path):
    game_dir = tmp_path / "game"
    game_dir.mkdir()
    bbox_file = tmp_path / "target-bbox.json"
    bbox_file.write_text(json.dumps({"bbox": [10, 20, 80, 60]}), encoding="utf-8")
    write_game_decision(game_dir / "20260705-game-decision.json")
    write_pr0p_suite(tmp_path / "20260707-pr0p-suite.json", readiness_promotion_steps())
    write_pr0p_manifest(
        tmp_path / "20260707-pr0p-live-manifest.json",
        readiness_promotion_steps(),
        suite_report=tmp_path / "20260707-pr0p-suite.json",
    )
    write_aux1_acceptance(tmp_path / "20260707-unit-aux1-acceptance.json")
    write_response_acceptance(tmp_path / "20260707-unit-response-acceptance.json")
    write_tracking_acceptance(
        tmp_path / "20260707-unit-tracking-acceptance.json",
        duration_s=20.0,
        min_found_ratio=0.95,
        max_loss_events=0,
    )
    write_moving_target_yaw(
        tmp_path / "20260707-unit-moving-target-yaw.json",
        status=WAITING,
        synthetic_status=PASS,
        real_status=WAITING,
    )

    monkeypatch.setattr(
        pr0p_core_next_module,
        "operator_readiness_precheck",
        lambda **_kwargs: {
            "status": WAITING,
            "summary": "operator window is not ready",
            "next_actions": ["open pr0p and select the FPV window"],
            "evidence_paths": {},
            "decision": {"readiness_stage": "target_setup"},
        },
    )

    def forbidden_runner(*_args, **_kwargs):
        raise AssertionError("operator precheck must not execute the live command")

    report = run_core_next(
        run_id="unit",
        log_dir=tmp_path,
        game_log_dir=game_dir,
        bbox=(10, 20, 80, 60),
        bbox_file=bbox_file,
        max_evidence_age_s=600.0,
        execute_next=False,
        ack_live_command=False,
        timeout_s=1.0,
        precheck_live_readiness=True,
        precheck_operator_readiness=True,
        command_runner=forbidden_runner,
    )

    assert report.status == WAITING
    assert report.next_command_key == "airborne_static_yaw_live"
    assert report.command_is_live is True
    assert report.executed is False
    assert "INDEPENDENT_READINESS_READY" in report.notes
    assert "OPERATOR_READINESS_NOT_READY" in report.notes
    assert report.metrics["operator_precheck_status"] == WAITING
    assert report.metrics["safe_to_execute_live_with_operator_precheck"] is False
    assert report.metrics["safe_to_execute_live_operator_blockers"] == [
        "OPERATOR_READINESS_NOT_READY",
    ]
    assert report.metrics["safe_to_execute_live_with_all_prechecks"] is False
    assert report.metrics["safe_to_execute_live_required_blockers"] == [
        "OPERATOR_READINESS_NOT_READY",
    ]
    summary = report.metrics["live_blocker_summary"]
    assert summary["status"] == "BLOCKED"
    assert summary["safe_to_execute_live"] is False
    assert summary["next_command_key"] == "airborne_static_yaw_live"
    assert summary["next_objective_gate"] == "moving_target_yaw"
    assert summary["objective_requirement_missing"] == [
        "airborne_static_yaw_centering",
        "pitch_approach_control",
        "manual_to_auto_handoff",
    ]
    assert summary["objective_requirement_failed"] == []
    assert summary["operator_readiness_status"] == WAITING
    assert summary["operator_next_action"] == "open pr0p and select the FPV window"
    assert summary["operator_missing_evidence"] == []
    assert summary["operator_recommended_command_name"] is None
    assert summary["operator_recommended_command_sends_uinput"] is None
    assert summary["operator_recommended_candidate_next_step"] is None
    assert summary["operator_next_command_source"] is None
    assert summary["operator_next_command"] is None
    assert summary["operator_next_command_sends_uinput"] is None
    assert summary["operator_next_command_unmet_requires_pass"] == []
    assert report.next_action == "open pr0p and select the FPV window"


def test_core_next_runner_blocks_operator_bbox_mismatch_without_executing(tmp_path):
    game_dir = tmp_path / "game"
    game_dir.mkdir()
    command_bbox_file = tmp_path / "target-bbox.json"
    command_bbox_file.write_text(json.dumps({"bbox": [10, 20, 80, 60]}), encoding="utf-8")
    operator_bbox_file = tmp_path / "operator-bbox.json"
    operator_bbox_file.write_text(json.dumps({"bbox": [99, 20, 80, 60]}), encoding="utf-8")
    write_game_decision(game_dir / "20260705-game-decision.json")
    write_pr0p_suite(tmp_path / "20260707-pr0p-suite.json", readiness_promotion_steps())
    write_pr0p_manifest(
        tmp_path / "20260707-pr0p-live-manifest.json",
        readiness_promotion_steps(),
        suite_report=tmp_path / "20260707-pr0p-suite.json",
    )
    write_aux1_acceptance(tmp_path / "20260707-unit-aux1-acceptance.json")
    write_response_acceptance(tmp_path / "20260707-unit-response-acceptance.json")
    write_tracking_acceptance(
        tmp_path / "20260707-unit-tracking-acceptance.json",
        duration_s=20.0,
        min_found_ratio=0.95,
        max_loss_events=0,
    )
    write_moving_target_yaw(
        tmp_path / "20260707-unit-moving-target-yaw.json",
        status=WAITING,
        synthetic_status=PASS,
        real_status=WAITING,
    )

    def forbidden_runner(*_args, **_kwargs):
        raise AssertionError("bbox mismatch must not execute the live command")

    report = run_core_next(
        run_id="unit",
        log_dir=tmp_path,
        game_log_dir=game_dir,
        bbox=(10, 20, 80, 60),
        bbox_file=command_bbox_file,
        operator_bbox_file=operator_bbox_file,
        max_evidence_age_s=600.0,
        execute_next=False,
        ack_live_command=False,
        timeout_s=1.0,
        precheck_live_readiness=True,
        precheck_operator_readiness=True,
        command_runner=forbidden_runner,
    )

    assert report.status == WAITING
    assert report.next_command_key == "airborne_static_yaw_live"
    assert report.command_is_live is True
    assert report.executed is False
    assert "INDEPENDENT_READINESS_READY" in report.notes
    assert "OPERATOR_READINESS_NOT_READY" in report.notes
    precheck = report.metrics["operator_readiness_precheck"]
    assert precheck["status"] == WAITING
    assert precheck["decision"]["command_gate"] == "bbox_file_mismatch"
    assert precheck["selected_bbox"] == [10, 20, 80, 60]
    assert precheck["operator_bbox"] == [99, 20, 80, 60]
    assert report.metrics["safe_to_execute_live_with_operator_precheck"] is False
    assert report.metrics["safe_to_execute_live_operator_blockers"] == [
        "OPERATOR_READINESS_NOT_READY",
    ]
    assert report.metrics["safe_to_execute_live_with_all_prechecks"] is False
    assert report.metrics["safe_to_execute_live_required_blockers"] == [
        "OPERATOR_READINESS_NOT_READY",
    ]
    summary = report.metrics["live_blocker_summary"]
    assert summary["status"] == "BLOCKED"
    assert summary["safe_to_execute_live"] is False
    assert summary["next_command_key"] == "airborne_static_yaw_live"
    assert summary["next_objective_gate"] == "moving_target_yaw"
    assert summary["objective_requirement_missing"] == [
        "airborne_static_yaw_centering",
        "pitch_approach_control",
        "manual_to_auto_handoff",
    ]
    assert summary["objective_requirement_failed"] == []
    assert summary["operator_readiness_status"] == WAITING
    assert "same --bbox-file" in summary["operator_next_action"]
    assert summary["operator_missing_evidence"] == []
    assert summary["operator_recommended_command_name"] is None
    assert summary["operator_recommended_command_sends_uinput"] is None
    assert summary["operator_recommended_candidate_next_step"] is None
    assert summary["operator_next_command_source"] is None
    assert summary["operator_next_command"] is None
    assert summary["operator_next_command_sends_uinput"] is None
    assert summary["operator_next_command_unmet_requires_pass"] == []
    assert "same --bbox-file" in report.next_action


def test_core_next_runner_operator_precheck_keeps_live_plan_when_ready(monkeypatch, tmp_path):
    game_dir = tmp_path / "game"
    game_dir.mkdir()
    bbox_file = tmp_path / "target-bbox.json"
    bbox_file.write_text(json.dumps({"bbox": [10, 20, 80, 60]}), encoding="utf-8")
    write_game_decision(game_dir / "20260705-game-decision.json")
    write_pr0p_suite(tmp_path / "20260707-pr0p-suite.json", readiness_promotion_steps())
    write_pr0p_manifest(
        tmp_path / "20260707-pr0p-live-manifest.json",
        readiness_promotion_steps(),
        suite_report=tmp_path / "20260707-pr0p-suite.json",
    )
    write_aux1_acceptance(tmp_path / "20260707-unit-aux1-acceptance.json")
    write_response_acceptance(tmp_path / "20260707-unit-response-acceptance.json")
    write_tracking_acceptance(
        tmp_path / "20260707-unit-tracking-acceptance.json",
        duration_s=20.0,
        min_found_ratio=0.95,
        max_loss_events=0,
    )
    write_moving_target_yaw(
        tmp_path / "20260707-unit-moving-target-yaw.json",
        status=WAITING,
        synthetic_status=PASS,
        real_status=WAITING,
    )

    monkeypatch.setattr(
        pr0p_core_next_module,
        "operator_readiness_precheck",
        lambda **_kwargs: {
            "status": "OPERATOR_PREFLIGHT_READY",
            "summary": "operator preflight ready",
            "next_actions": [],
            "evidence_paths": {
                "operator_preflight_json": "logs/operator-ready.json",
            },
            "decision": {"readiness_stage": "ready"},
        },
    )

    def forbidden_runner(*_args, **_kwargs):
        raise AssertionError("plan precheck must not execute the live command")

    report = run_core_next(
        run_id="unit",
        log_dir=tmp_path,
        game_log_dir=game_dir,
        bbox=(10, 20, 80, 60),
        bbox_file=bbox_file,
        max_evidence_age_s=600.0,
        execute_next=False,
        ack_live_command=False,
        timeout_s=1.0,
        precheck_live_readiness=True,
        precheck_operator_readiness=True,
        command_runner=forbidden_runner,
    )

    assert report.status == "PLANNED"
    assert report.next_command_key == "airborne_static_yaw_live"
    assert report.executed is False
    assert "INDEPENDENT_READINESS_READY" in report.notes
    assert "OPERATOR_READINESS_READY" in report.notes
    assert report.metrics["operator_precheck_status"] == "OPERATOR_PREFLIGHT_READY"
    assert report.metrics["safe_to_execute_live_with_operator_precheck"] is True
    assert report.metrics["safe_to_execute_live_operator_blockers"] == []
    assert report.metrics["safe_to_execute_live_with_all_prechecks"] is True
    assert report.metrics["safe_to_execute_live_required_blockers"] == []


def test_core_next_runner_blocks_live_execution_when_operator_readiness_missing(monkeypatch, tmp_path):
    game_dir = tmp_path / "game"
    game_dir.mkdir()
    bbox_file = tmp_path / "target-bbox.json"
    bbox_file.write_text(json.dumps({"bbox": [10, 20, 80, 60]}), encoding="utf-8")
    write_game_decision(game_dir / "20260705-game-decision.json")
    write_pr0p_suite(tmp_path / "20260707-pr0p-suite.json", readiness_promotion_steps())
    write_pr0p_manifest(
        tmp_path / "20260707-pr0p-live-manifest.json",
        readiness_promotion_steps(),
        suite_report=tmp_path / "20260707-pr0p-suite.json",
    )
    write_aux1_acceptance(tmp_path / "20260707-unit-aux1-acceptance.json")
    write_response_acceptance(tmp_path / "20260707-unit-response-acceptance.json")
    write_tracking_acceptance(
        tmp_path / "20260707-unit-tracking-acceptance.json",
        duration_s=20.0,
        min_found_ratio=0.95,
        max_loss_events=0,
    )
    write_moving_target_yaw(
        tmp_path / "20260707-unit-moving-target-yaw.json",
        status=WAITING,
        synthetic_status=PASS,
        real_status=WAITING,
    )
    monkeypatch.setattr(
        pr0p_core_next_module,
        "operator_readiness_precheck",
        lambda **_kwargs: {
            "status": WAITING,
            "summary": "operator window is not ready",
            "next_actions": ["open pr0p FPV window"],
            "evidence_paths": {},
            "decision": {
                "readiness_stage": "target_setup",
                "command_gate": "blocked_by_dependency",
                "recommended_command": {
                    "name": "dry_sequence",
                    "safety_class": "dry_run_capture",
                    "sends_uinput": False,
                    "command": "fpv_env/bin/python dry-sequence.py",
                },
                "recommended_candidate_action": {
                    "candidate_index": 1,
                    "next_step": "dry_run_sequence_region",
                    "next_command": "fpv_env/bin/python external-dry-region.py",
                    "safety_class": "dry_run_capture",
                    "sends_uinput": False,
                    "requires_user_judgement": True,
                    "unblocks": ["external_window_preflight"],
                    "guard": "confirm this is the FPV window",
                },
                "safe_to_execute_now": False,
                "live_ack_required": False,
                "freshness_gate_status": WAITING,
                "window_discovery_status": WAITING,
                "window_discovery_reason": "NO_MATCH",
                "window_title_candidates": ["pr0p FPV"],
                "excluded_window_candidates": [
                    {
                        "title": "gz@gz: ~",
                        "window_id": "0x123",
                        "rejection_reason": "tooling_shell_window",
                    },
                ],
                "required_fresh_evidence": [
                    "latest_acceptance",
                    "matching_external_preflight",
                    "matching_external_follow",
                    "bbox_file",
                    "matching_external_live_follow_sequence",
                ],
                "stale_evidence": ["matching_external_follow"],
                "missing_evidence": ["matching_external_live_follow_sequence"],
            },
        },
    )

    def forbidden_runner(*_args, **_kwargs):
        raise AssertionError("live command must not run without operator readiness")

    report = run_core_next(
        run_id="unit",
        log_dir=tmp_path,
        game_log_dir=game_dir,
        bbox=(10, 20, 80, 60),
        bbox_file=bbox_file,
        max_evidence_age_s=600.0,
        execute_next=True,
        ack_live_command=True,
        timeout_s=1.0,
        command_runner=forbidden_runner,
    )

    assert report.status == WAITING
    assert report.next_command_key == "airborne_static_yaw_live"
    assert report.executed is False
    assert "OPERATOR_READINESS_NOT_READY" in report.notes
    assert report.metrics["independent_readiness_precheck"]["status"] == INDEPENDENT_SIM_READY
    assert report.metrics["operator_precheck_status"] == WAITING
    assert report.metrics["operator_readiness_stage"] == "target_setup"
    assert report.metrics["operator_readiness_command_gate"] == "blocked_by_dependency"
    assert report.metrics["operator_readiness_next_action"] == "open pr0p FPV window"
    assert report.metrics["operator_readiness_recommended_command"]["name"] == "dry_sequence"
    assert report.metrics["operator_readiness_recommended_command"]["sends_uinput"] is False
    assert (
        report.metrics["operator_readiness_recommended_candidate_action"]["next_step"]
        == "dry_run_sequence_region"
    )
    assert report.metrics["operator_readiness_next_command_packet"] == {
        "source": "recommended_candidate_action",
        "name": "dry_run_sequence_region",
        "command": "fpv_env/bin/python external-dry-region.py",
        "safety_class": "dry_run_capture",
        "sends_uinput": False,
        "requires_ack_live_input": False,
        "requires_user_judgement": True,
        "unmet_requires_pass": [],
        "unblocks": ["external_window_preflight"],
        "guard": "confirm this is the FPV window",
    }
    assert report.metrics["operator_readiness_safe_to_execute_now"] is False
    assert report.metrics["operator_readiness_live_ack_required"] is False
    assert report.metrics["operator_readiness_freshness_gate_status"] == WAITING
    assert report.metrics["operator_readiness_window_discovery_status"] == WAITING
    assert report.metrics["operator_readiness_window_discovery_reason"] == "NO_MATCH"
    assert report.metrics["operator_readiness_window_title_candidates"] == ["pr0p FPV"]
    assert report.metrics["operator_readiness_excluded_window_candidates"] == [
        {
            "title": "gz@gz: ~",
            "window_id": "0x123",
            "rejection_reason": "tooling_shell_window",
        },
    ]
    assert report.metrics["operator_readiness_stale_evidence"] == [
        "matching_external_follow",
    ]
    assert report.metrics["operator_readiness_missing_evidence"] == [
        "matching_external_live_follow_sequence",
    ]
    assert report.metrics["safe_to_execute_live_with_operator_precheck"] is False
    assert report.metrics["safe_to_execute_live_operator_blockers"] == [
        "OPERATOR_READINESS_NOT_READY",
    ]
    assert report.metrics["safe_to_execute_live_with_all_prechecks"] is False
    assert report.metrics["safe_to_execute_live_required_blockers"] == [
        "OPERATOR_READINESS_NOT_READY",
    ]
    summary = report.metrics["live_blocker_summary"]
    assert summary["operator_window_discovery_status"] == WAITING
    assert summary["operator_window_discovery_reason"] == "NO_MATCH"
    assert summary["operator_window_title_candidates"] == ["pr0p FPV"]
    assert summary["operator_excluded_window_candidates"] == [
        {
            "title": "gz@gz: ~",
            "window_id": "0x123",
            "rejection_reason": "tooling_shell_window",
        },
    ]
    assert summary["operator_next_command_source"] == "recommended_candidate_action"
    assert summary["operator_next_command_name"] == "dry_run_sequence_region"
    assert summary["operator_next_command"] == "fpv_env/bin/python external-dry-region.py"
    assert summary["operator_next_command_sends_uinput"] is False
    assert summary["operator_next_command_unmet_requires_pass"] == []
    assert summary["operator_next_command_guard"] == "confirm this is the FPV window"


def test_core_next_runner_execute_operator_next_blocks_unmet_requirements(monkeypatch, tmp_path):
    bbox_file = tmp_path / "target-bbox.json"
    bbox_file.write_text(json.dumps({"bbox": [10, 20, 80, 60]}), encoding="utf-8")
    write_airborne_static_live_waiting_prereqs(tmp_path)
    monkeypatch.setattr(
        pr0p_core_next_module,
        "operator_readiness_precheck",
        lambda **_kwargs: {
            "status": WAITING,
            "summary": "operator window is not ready",
            "next_actions": ["open pr0p FPV window"],
            "evidence_paths": {},
            "decision": {
                "readiness_stage": "target_setup",
                "command_gate": "blocked_by_dependency",
                "recommended_command": {
                    "name": "dry_sequence",
                    "safety_class": "dry_run_capture",
                    "sends_uinput": False,
                    "command": "fpv_env/bin/python dry-sequence.py",
                    "unmet_requires_pass": ["target_window=WAITING"],
                },
            },
        },
    )

    def forbidden_runner(*_args, **_kwargs):
        raise AssertionError("operator command with unmet requirements must not run")

    report = run_core_next(
        run_id="unit",
        log_dir=tmp_path,
        bbox=(10, 20, 80, 60),
        bbox_file=bbox_file,
        max_evidence_age_s=600.0,
        execute_next=False,
        ack_live_command=False,
        execute_operator_next=True,
        timeout_s=1.0,
        command_runner=forbidden_runner,
    )

    assert report.status == WAITING
    assert report.executed is False
    assert "OPERATOR_NEXT_COMMAND_UNMET_REQUIREMENTS" in report.notes
    assert report.metrics["operator_next_execution_requested"] is True
    assert report.metrics["operator_next_command_executed"] is False
    assert report.metrics["operator_next_execution_blockers"] == [
        "OPERATOR_NEXT_COMMAND_UNMET_REQUIREMENTS",
    ]


def test_core_next_runner_execute_operator_next_requires_candidate_ack(monkeypatch, tmp_path):
    bbox_file = tmp_path / "target-bbox.json"
    bbox_file.write_text(json.dumps({"bbox": [10, 20, 80, 60]}), encoding="utf-8")
    write_airborne_static_live_waiting_prereqs(tmp_path)
    monkeypatch.setattr(
        pr0p_core_next_module,
        "operator_readiness_precheck",
        lambda **_kwargs: {
            "status": WAITING,
            "summary": "candidate needs confirmation",
            "next_actions": ["confirm external FPV window candidate"],
            "evidence_paths": {},
            "decision": {
                "readiness_stage": "target_setup",
                "command_gate": "blocked_by_dependency",
                "recommended_candidate_action": {
                    "candidate_index": 1,
                    "next_step": "dry_run_sequence_region",
                    "next_command": "fpv_env/bin/python external-dry-region.py",
                    "safety_class": "dry_run_capture",
                    "sends_uinput": False,
                    "requires_user_judgement": True,
                    "unblocks": ["external_window_preflight"],
                    "guard": "confirm this is the FPV window",
                },
            },
        },
    )

    def forbidden_runner(*_args, **_kwargs):
        raise AssertionError("candidate command must require operator candidate ack")

    report = run_core_next(
        run_id="unit",
        log_dir=tmp_path,
        bbox=(10, 20, 80, 60),
        bbox_file=bbox_file,
        max_evidence_age_s=600.0,
        execute_next=False,
        ack_live_command=False,
        execute_operator_next=True,
        timeout_s=1.0,
        command_runner=forbidden_runner,
    )

    assert report.status == WAITING
    assert report.executed is False
    assert "OPERATOR_NEXT_COMMAND_REQUIRES_CANDIDATE_ACK" in report.notes
    assert report.metrics["operator_next_command_executed"] is False


def test_core_next_runner_execute_operator_next_runs_safe_non_uinput_candidate(
    monkeypatch,
    tmp_path,
):
    bbox_file = tmp_path / "target-bbox.json"
    bbox_file.write_text(json.dumps({"bbox": [10, 20, 80, 60]}), encoding="utf-8")
    write_airborne_static_live_waiting_prereqs(tmp_path)
    precheck_calls = []

    def fake_operator_precheck(**_kwargs):
        precheck_calls.append(_kwargs.get("run_id"))
        if len(precheck_calls) == 1:
            return {
                "status": WAITING,
                "summary": "candidate dry command available",
                "next_actions": ["run dry candidate command"],
                "evidence_paths": {},
                "decision": {
                    "readiness_stage": "target_setup",
                    "command_gate": "blocked_by_dependency",
                    "recommended_candidate_action": {
                        "candidate_index": 1,
                        "next_step": "dry_run_sequence_region",
                        "next_command": "fpv_env/bin/python external-dry-region.py --duration 2",
                        "safety_class": "dry_run_capture",
                        "sends_uinput": False,
                        "requires_user_judgement": True,
                        "unblocks": ["external_window_preflight"],
                        "guard": "confirm this is the FPV window",
                    },
                },
            }
        return {
            "status": "OPERATOR_PREFLIGHT_READY",
            "summary": "operator ready after dry command",
            "next_actions": ["operator ready for live follow evidence"],
            "evidence_paths": {},
            "decision": {
                "readiness_stage": "ready",
                "window_discovery_status": "MATCHED",
                "window_discovery_reason": "MATCHED",
                "missing_evidence": [],
                "stale_evidence": [],
            },
        }

    monkeypatch.setattr(
        pr0p_core_next_module,
        "operator_readiness_precheck",
        fake_operator_precheck,
    )
    calls = []

    def fake_runner(argv, **_kwargs):
        text = " ".join(argv)
        calls.append(text)
        assert text == "fpv_env/bin/python external-dry-region.py --duration 2"
        return subprocess.CompletedProcess(
            argv,
            0,
            stdout="external-dry-run-sequence PASS report=logs/operator-dry.json summary=logs/operator-dry.md\n",
            stderr="",
        )

    report = run_core_next(
        run_id="unit",
        log_dir=tmp_path,
        bbox=(10, 20, 80, 60),
        bbox_file=bbox_file,
        max_evidence_age_s=600.0,
        execute_next=False,
        ack_live_command=False,
        execute_operator_next=True,
        ack_operator_candidate=True,
        timeout_s=1.0,
        command_runner=fake_runner,
    )

    assert report.status == PASS
    assert report.executed is True
    assert report.command_report == "logs/operator-dry.json"
    assert report.notes == ["OPERATOR_NEXT_COMMAND_EXECUTED"]
    assert report.metrics["executed_command_role"] == "operator_next"
    assert report.metrics["operator_next_command_executed"] is True
    assert report.metrics["operator_next_execution_blockers"] == []
    assert report.metrics["post_operator_precheck_status"] == "OPERATOR_PREFLIGHT_READY"
    assert report.metrics["post_operator_readiness_stage"] == "ready"
    assert report.metrics["post_operator_window_discovery_status"] == "MATCHED"
    assert report.metrics["post_operator_missing_evidence"] == []
    assert report.next_action == (
        "Operator next command completed; post-operator readiness is "
        "OPERATOR_PREFLIGHT_READY."
    )
    assert len(precheck_calls) == 2
    assert calls


def test_core_next_runner_execute_operator_next_blocks_uinput_command(monkeypatch, tmp_path):
    bbox_file = tmp_path / "target-bbox.json"
    bbox_file.write_text(json.dumps({"bbox": [10, 20, 80, 60]}), encoding="utf-8")
    write_airborne_static_live_waiting_prereqs(tmp_path)
    monkeypatch.setattr(
        pr0p_core_next_module,
        "operator_readiness_precheck",
        lambda **_kwargs: {
            "status": WAITING,
            "summary": "live input command is not allowed here",
            "next_actions": ["run live input manually with explicit ack"],
            "evidence_paths": {},
            "decision": {
                "readiness_stage": "live_input",
                "recommended_command": {
                    "name": "live_input_readiness",
                    "safety_class": "live_input_ack_required",
                    "sends_uinput": True,
                    "command": "fpv_env/bin/python live-input.py --ack-live-input",
                },
            },
        },
    )

    def forbidden_runner(*_args, **_kwargs):
        raise AssertionError("operator-next must not run uinput/live commands")

    report = run_core_next(
        run_id="unit",
        log_dir=tmp_path,
        bbox=(10, 20, 80, 60),
        bbox_file=bbox_file,
        max_evidence_age_s=600.0,
        execute_next=False,
        ack_live_command=False,
        execute_operator_next=True,
        timeout_s=1.0,
        command_runner=forbidden_runner,
    )

    assert report.status == WAITING
    assert report.executed is False
    assert "OPERATOR_NEXT_COMMAND_UINPUT_OR_LIVE" in report.notes
    assert report.metrics["operator_next_command_executed"] is False


def test_core_next_runner_executes_live_when_operator_and_independent_ready(monkeypatch, tmp_path):
    game_dir = tmp_path / "game"
    game_dir.mkdir()
    bbox_file = tmp_path / "target-bbox.json"
    bbox_file.write_text(json.dumps({"bbox": [10, 20, 80, 60]}), encoding="utf-8")
    write_game_decision(game_dir / "20260705-game-decision.json")
    write_pr0p_suite(tmp_path / "20260707-pr0p-suite.json", readiness_promotion_steps())
    write_pr0p_manifest(
        tmp_path / "20260707-pr0p-live-manifest.json",
        readiness_promotion_steps(),
        suite_report=tmp_path / "20260707-pr0p-suite.json",
    )
    write_aux1_acceptance(tmp_path / "20260707-unit-aux1-acceptance.json")
    write_response_acceptance(tmp_path / "20260707-unit-response-acceptance.json")
    write_tracking_acceptance(
        tmp_path / "20260707-unit-tracking-acceptance.json",
        duration_s=20.0,
        min_found_ratio=0.95,
        max_loss_events=0,
    )
    write_moving_target_yaw(
        tmp_path / "20260707-unit-moving-target-yaw.json",
        status=WAITING,
        synthetic_status=PASS,
        real_status=WAITING,
    )
    monkeypatch.setattr(
        pr0p_core_next_module,
        "operator_readiness_precheck",
        lambda **_kwargs: {
            "status": "OPERATOR_PREFLIGHT_READY",
            "summary": "operator preflight ready",
            "next_actions": [],
            "evidence_paths": {},
            "decision": {"readiness_stage": "ready"},
        },
    )
    calls = []

    def fake_runner(argv, **_kwargs):
        calls.append(" ".join(argv))
        stdout = (
            "simitl-pr0p-moving-target-acceptance PASS "
            "report=logs/airborne-static-live.json "
            "summary=logs/airborne-static-live.md\n"
        )
        return subprocess.CompletedProcess(argv, 0, stdout=stdout, stderr="")

    report = run_core_next(
        run_id="unit",
        log_dir=tmp_path,
        game_log_dir=game_dir,
        bbox=(10, 20, 80, 60),
        bbox_file=bbox_file,
        max_evidence_age_s=600.0,
        execute_next=True,
        ack_live_command=True,
        timeout_s=1.0,
        command_runner=fake_runner,
    )

    assert report.status == PASS
    assert report.next_command_key == "airborne_static_yaw_live"
    assert report.command_is_live is True
    assert report.executed is True
    assert report.command_report == "logs/airborne-static-live.json"
    assert report.metrics["operator_precheck_status"] == "OPERATOR_PREFLIGHT_READY"
    assert report.metrics["safe_to_execute_live_with_operator_precheck"] is True
    assert report.metrics["safe_to_execute_live_with_all_prechecks"] is True
    assert report.metrics["safe_to_execute_live_required_blockers"] == []
    assert calls


def test_core_next_runner_executes_airborne_static_yaw_refresh_without_live_ack(tmp_path):
    write_pr0p_suite(tmp_path / "20260707-pr0p-suite.json", readiness_promotion_steps())
    write_aux1_acceptance(tmp_path / "20260707-unit-aux1-acceptance.json")
    write_response_acceptance(tmp_path / "20260707-unit-response-acceptance.json")
    write_tracking_acceptance(
        tmp_path / "20260707-unit-tracking-acceptance.json",
        duration_s=20.0,
        min_found_ratio=0.95,
        max_loss_events=0,
    )
    tracking_report = write_real_moving_tracking_evidence(tmp_path)
    write_moving_target_yaw(
        tmp_path / "20260707-unit-moving-target-yaw.json",
        status=WAITING,
        synthetic_status=PASS,
        real_status=WAITING,
    )
    write_moving_target_acceptance(
        tmp_path / "20260707-unit-moving-target-acceptance.json",
        tracking_report=tracking_report,
    )
    calls = []

    def fake_runner(argv, **_kwargs):
        text = " ".join(argv)
        calls.append(text)
        assert "pr0p_moving_target_yaw_plan.py" in text
        assert "--execute-synthetic" in text
        assert "--real-tracking-report %s" % tracking_report in text
        assert "--ack-airborne-static-target" in text
        assert "--run-live-gate" not in text
        assert "--ack-live-input" not in text
        stdout = (
            "simitl-pr0p-moving-target-yaw PASS "
            "report=logs/airborne-static-yaw-refresh.json "
            "summary=logs/airborne-static-yaw-refresh.md\n"
        )
        return subprocess.CompletedProcess(argv, 0, stdout=stdout, stderr="")

    report = run_core_next(
        run_id="unit",
        log_dir=tmp_path,
        bbox=(10, 20, 80, 60),
        max_evidence_age_s=600.0,
        execute_next=True,
        ack_live_command=False,
        timeout_s=1.0,
        command_runner=fake_runner,
    )

    assert report.status == PASS
    assert report.next_command_key == "airborne_static_yaw_refresh"
    assert report.command_is_live is False
    assert report.executed is True
    assert report.command_report == "logs/airborne-static-yaw-refresh.json"
    assert calls


def test_core_next_runner_executes_approach_plan_after_moving_target_passes(tmp_path):
    write_pr0p_suite(tmp_path / "20260707-pr0p-suite.json", readiness_promotion_steps())
    write_aux1_acceptance(tmp_path / "20260707-unit-aux1-acceptance.json")
    write_response_acceptance(tmp_path / "20260707-unit-response-acceptance.json")
    write_tracking_acceptance(
        tmp_path / "20260707-unit-tracking-acceptance.json",
        duration_s=20.0,
        min_found_ratio=0.95,
        max_loss_events=0,
    )
    write_moving_target_yaw(
        tmp_path / "20260707-unit-moving-target-yaw.json",
        status=PASS,
        synthetic_status=PASS,
        real_status=PASS,
    )
    calls = []

    def fake_runner(argv, **_kwargs):
        text = " ".join(argv)
        calls.append(text)
        assert "pr0p_approach_pitch_plan.py" in text
        assert "--execute-synthetic" in text
        assert "--run-live-gates" not in text
        stdout = (
            "simitl-pr0p-approach-pitch WAITING "
            "report=logs/approach-pitch.json summary=logs/approach-pitch.md\n"
        )
        return subprocess.CompletedProcess(argv, 0, stdout=stdout, stderr="")

    report = run_core_next(
        run_id="unit",
        log_dir=tmp_path,
        bbox=(10, 20, 80, 60),
        max_evidence_age_s=600.0,
        execute_next=True,
        ack_live_command=False,
        timeout_s=1.0,
        command_runner=fake_runner,
    )

    assert report.status == WAITING
    assert report.next_command_key == "approach_pitch_plan"
    assert report.command_is_live is False
    assert report.executed is True
    assert report.command_report == "logs/approach-pitch.json"
    assert calls


def test_core_next_runner_requires_ack_for_approach_pitch_live_after_synthetic_pass(tmp_path):
    write_pr0p_suite(tmp_path / "20260707-pr0p-suite.json", readiness_promotion_steps())
    write_aux1_acceptance(tmp_path / "20260707-unit-aux1-acceptance.json")
    write_response_acceptance(tmp_path / "20260707-unit-response-acceptance.json")
    write_tracking_acceptance(
        tmp_path / "20260707-unit-tracking-acceptance.json",
        duration_s=20.0,
        min_found_ratio=0.95,
        max_loss_events=0,
    )
    write_moving_target_yaw(
        tmp_path / "20260707-unit-moving-target-yaw.json",
        status=PASS,
        synthetic_status=PASS,
        real_status=PASS,
    )
    write_approach_pitch(
        tmp_path / "20260707-unit-approach-pitch.json",
        status=WAITING,
        synthetic_status=PASS,
        real_status=WAITING,
    )

    def forbidden_runner(*_args, **_kwargs):
        raise AssertionError("approach live command must require explicit ack")

    report = run_core_next(
        run_id="unit",
        log_dir=tmp_path,
        bbox=(10, 20, 80, 60),
        max_evidence_age_s=600.0,
        execute_next=True,
        ack_live_command=False,
        timeout_s=1.0,
        command_runner=forbidden_runner,
    )

    assert report.status == WAITING
    assert report.next_command_key == "approach_pitch_live"
    assert report.command_is_live is True
    assert report.executed is False
    assert "LIVE_COMMAND_REQUIRES_ACK" in report.notes
    assert "pr0p_tracking_acceptance_runner.py" in report.next_command
    assert "--enable-pitch" in report.next_command


def test_core_next_runner_executes_approach_pitch_refresh_without_live_ack(tmp_path):
    write_pr0p_suite(tmp_path / "20260707-pr0p-suite.json", readiness_promotion_steps())
    write_aux1_acceptance(tmp_path / "20260707-unit-aux1-acceptance.json")
    write_response_acceptance(tmp_path / "20260707-unit-response-acceptance.json")
    write_tracking_acceptance(
        tmp_path / "20260707-unit-tracking-acceptance.json",
        duration_s=20.0,
        min_found_ratio=0.95,
        max_loss_events=0,
    )
    write_moving_target_yaw(
        tmp_path / "20260707-unit-moving-target-yaw.json",
        status=PASS,
        synthetic_status=PASS,
        real_status=PASS,
    )
    write_approach_pitch(
        tmp_path / "20260707-unit-approach-pitch.json",
        status=WAITING,
        synthetic_status=PASS,
        real_status=WAITING,
    )
    approach_tracking = tmp_path / "20260707-unit-approach-tracking-acceptance.json"
    write_tracking_acceptance(
        approach_tracking,
        duration_s=20.0,
        min_found_ratio=0.9,
        max_loss_events=2,
        enable_pitch=True,
        desired_target_width=120.0,
        max_abs_pitch_axis=0.8,
    )
    calls = []

    def fake_runner(argv, **_kwargs):
        text = " ".join(argv)
        calls.append(text)
        assert "pr0p_approach_pitch_plan.py" in text
        assert "--execute-synthetic" in text
        assert "--real-tracking-report %s" % approach_tracking in text
        assert "--ack-real-approach-target" in text
        assert "--run-live-gates" not in text
        assert "--ack-live-input" not in text
        stdout = (
            "simitl-pr0p-approach-pitch PASS "
            "report=logs/approach-pitch-refresh.json "
            "summary=logs/approach-pitch-refresh.md\n"
        )
        return subprocess.CompletedProcess(argv, 0, stdout=stdout, stderr="")

    report = run_core_next(
        run_id="unit",
        log_dir=tmp_path,
        bbox=(10, 20, 80, 60),
        max_evidence_age_s=600.0,
        execute_next=True,
        ack_live_command=False,
        timeout_s=1.0,
        command_runner=fake_runner,
    )

    assert report.status == PASS
    assert report.next_command_key == "approach_pitch_refresh"
    assert report.command_is_live is False
    assert report.executed is True
    assert report.command_report == "logs/approach-pitch-refresh.json"
    assert calls


def test_core_next_runner_executes_handoff_plan_after_approach_passes(tmp_path):
    write_pr0p_suite(tmp_path / "20260707-pr0p-suite.json", readiness_promotion_steps())
    write_aux1_acceptance(tmp_path / "20260707-unit-aux1-acceptance.json")
    write_response_acceptance(tmp_path / "20260707-unit-response-acceptance.json")
    write_tracking_acceptance(
        tmp_path / "20260707-unit-tracking-acceptance.json",
        duration_s=20.0,
        min_found_ratio=0.95,
        max_loss_events=0,
    )
    write_moving_target_yaw(
        tmp_path / "20260707-unit-moving-target-yaw.json",
        status=PASS,
        synthetic_status=PASS,
        real_status=PASS,
    )
    write_approach_pitch(
        tmp_path / "20260707-unit-approach-pitch.json",
        status=PASS,
        synthetic_status=PASS,
        real_status=PASS,
    )
    calls = []

    def fake_runner(argv, **_kwargs):
        text = " ".join(argv)
        calls.append(text)
        assert "pr0p_handoff_plan.py" in text
        assert "--execute-synthetic" in text
        assert "--run-live-gates" not in text
        stdout = (
            "simitl-pr0p-handoff WAITING "
            "report=logs/handoff.json summary=logs/handoff.md\n"
        )
        return subprocess.CompletedProcess(argv, 0, stdout=stdout, stderr="")

    report = run_core_next(
        run_id="unit",
        log_dir=tmp_path,
        bbox=(10, 20, 80, 60),
        max_evidence_age_s=600.0,
        execute_next=True,
        ack_live_command=False,
        timeout_s=1.0,
        command_runner=fake_runner,
    )

    assert report.status == WAITING
    assert report.next_command_key == "handoff_plan"
    assert report.command_is_live is False
    assert report.executed is True
    assert report.command_report == "logs/handoff.json"
    assert calls


def test_core_next_runner_requires_ack_for_handoff_live_after_synthetic_pass(tmp_path):
    write_pr0p_suite(tmp_path / "20260707-pr0p-suite.json", readiness_promotion_steps())
    write_aux1_acceptance(tmp_path / "20260707-unit-aux1-acceptance.json")
    write_response_acceptance(tmp_path / "20260707-unit-response-acceptance.json")
    write_tracking_acceptance(
        tmp_path / "20260707-unit-tracking-acceptance.json",
        duration_s=20.0,
        min_found_ratio=0.95,
        max_loss_events=0,
    )
    write_moving_target_yaw(
        tmp_path / "20260707-unit-moving-target-yaw.json",
        status=PASS,
        synthetic_status=PASS,
        real_status=PASS,
    )
    write_approach_pitch(
        tmp_path / "20260707-unit-approach-pitch.json",
        status=PASS,
        synthetic_status=PASS,
        real_status=PASS,
    )
    write_handoff(
        tmp_path / "20260707-unit-handoff.json",
        status=WAITING,
        synthetic_status=PASS,
        real_status=WAITING,
    )

    def forbidden_runner(*_args, **_kwargs):
        raise AssertionError("handoff live command must require explicit ack")

    report = run_core_next(
        run_id="unit",
        log_dir=tmp_path,
        bbox=(10, 20, 80, 60),
        max_evidence_age_s=600.0,
        execute_next=True,
        ack_live_command=False,
        timeout_s=1.0,
        command_runner=forbidden_runner,
    )

    assert report.status == WAITING
    assert report.next_command_key == "handoff_live"
    assert report.command_is_live is True
    assert report.executed is False
    assert "LIVE_COMMAND_REQUIRES_ACK" in report.notes
    assert "pr0p_handoff_acceptance_runner.py" in report.next_command
    assert "--ack-real-handoff" in report.next_command


def test_core_next_runner_executes_handoff_refresh_without_live_ack(tmp_path):
    write_pr0p_suite(tmp_path / "20260707-pr0p-suite.json", readiness_promotion_steps())
    write_aux1_acceptance(tmp_path / "20260707-unit-aux1-acceptance.json")
    write_response_acceptance(tmp_path / "20260707-unit-response-acceptance.json")
    write_tracking_acceptance(
        tmp_path / "20260707-unit-tracking-acceptance.json",
        duration_s=20.0,
        min_found_ratio=0.95,
        max_loss_events=0,
    )
    write_moving_target_yaw(
        tmp_path / "20260707-unit-moving-target-yaw.json",
        status=PASS,
        synthetic_status=PASS,
        real_status=PASS,
    )
    write_approach_pitch(
        tmp_path / "20260707-unit-approach-pitch.json",
        status=PASS,
        synthetic_status=PASS,
        real_status=PASS,
    )
    write_handoff(
        tmp_path / "20260707-unit-handoff.json",
        status=WAITING,
        synthetic_status=PASS,
        real_status=WAITING,
    )
    real_handoff = tmp_path / "20260707-unit-real-handoff.json"
    write_real_handoff_acceptance(real_handoff)
    calls = []

    def fake_runner(argv, **_kwargs):
        text = " ".join(argv)
        calls.append(text)
        assert "pr0p_handoff_plan.py" in text
        assert "--execute-synthetic" in text
        assert "--real-handoff-report %s" % real_handoff in text
        assert "--ack-real-handoff" in text
        assert "--run-live-gate" not in text
        assert "--ack-live-input" not in text
        stdout = (
            "simitl-pr0p-handoff PASS "
            "report=logs/handoff-refresh.json "
            "summary=logs/handoff-refresh.md\n"
        )
        return subprocess.CompletedProcess(argv, 0, stdout=stdout, stderr="")

    report = run_core_next(
        run_id="unit",
        log_dir=tmp_path,
        bbox=(10, 20, 80, 60),
        max_evidence_age_s=600.0,
        execute_next=True,
        ack_live_command=False,
        timeout_s=1.0,
        command_runner=fake_runner,
    )

    assert report.status == PASS
    assert report.next_command_key == "handoff_refresh"
    assert report.command_is_live is False
    assert report.executed is True
    assert report.command_report == "logs/handoff-refresh.json"
    assert calls


def test_core_next_runner_requires_ack_flag_with_execute_next():
    with pytest.raises(SystemExit):
        parse_core_next_args(["--ack-live-command"])


def test_core_next_runner_operator_next_cli_guards():
    with pytest.raises(SystemExit):
        parse_core_next_args(["--execute-next", "--execute-operator-next"])

    with pytest.raises(SystemExit):
        parse_core_next_args(["--ack-operator-candidate"])


def test_core_next_runner_detects_singular_live_gate_marker():
    assert is_live_command(
        "fpv_env/bin/python experiments/simitl_pr0p_probe/example.py --run-live-gate"
    )
    assert not is_live_command(
        "fpv_env/bin/python experiments/simitl_pr0p_probe/example.py --execute-synthetic"
    )


def test_core_next_runner_cli_accepts_bbox_file(tmp_path):
    bbox_file = tmp_path / "target-bbox.json"
    bbox_file.write_text(json.dumps({"bbox": [10, 20, 80, 60]}), encoding="utf-8")

    args = parse_core_next_args(["--bbox-file", str(bbox_file)])

    assert args.bbox == (10, 20, 80, 60)


def test_core_next_runner_cli_accepts_physical_mapping_flag(tmp_path):
    args = parse_core_next_args(["--allow-physical-mapping"])

    assert args.allow_physical_mapping is True


def test_core_next_runner_cli_accepts_live_precheck_flag():
    args = parse_core_next_args(["--precheck-live-readiness"])

    assert args.precheck_live_readiness is True


def test_core_next_runner_cli_accepts_operator_precheck_flags(tmp_path):
    bbox_file = tmp_path / "operator-bbox.json"
    args = parse_core_next_args([
        "--precheck-operator-readiness",
        "--operator-bbox-file",
        str(bbox_file),
        "--operator-window-title",
        "pr0p",
    ])

    assert args.precheck_operator_readiness is True
    assert args.operator_bbox_file == bbox_file
    assert args.operator_window_title == "pr0p"


def test_core_next_runner_cli_accepts_operator_precheck_skip():
    args = parse_core_next_args(["--skip-operator-readiness-precheck"])

    assert args.skip_operator_readiness_precheck is True


def test_core_next_runner_cli_rejects_operator_precheck_and_skip():
    with pytest.raises(SystemExit):
        parse_core_next_args([
            "--precheck-operator-readiness",
            "--skip-operator-readiness-precheck",
        ])


def test_core_next_runner_cli_rejects_bbox_and_bbox_file(tmp_path):
    bbox_file = tmp_path / "target-bbox.json"
    bbox_file.write_text(json.dumps({"bbox": [10, 20, 80, 60]}), encoding="utf-8")

    with pytest.raises(SystemExit):
        parse_core_next_args(["--bbox", "1,2,30,40", "--bbox-file", str(bbox_file)])


def make_pr0p_client(root: Path) -> Path:
    root.mkdir(parents=True, exist_ok=True)
    client = root / "pr0p.x86_64"
    client.write_text("#!/bin/sh\n", encoding="utf-8")
    client.chmod(0o755)
    return client


def test_rc_manual_flight_gate_waits_at_aux1_without_live_evidence(tmp_path, monkeypatch):
    root = tmp_path / "install"
    make_pr0p_client(root)
    monkeypatch.setattr("pr0p_install_probe.ALLOWED_INSTALL_ROOTS", (root,))
    input_config = tmp_path / "input.json"
    write_generic_virtual_input_config(input_config)

    def forbidden_runner(*_args, **_kwargs):
        raise AssertionError("plan-only manual flight gate must not run external commands")

    report = run_manual_flight_gate(
        run_id="unit",
        log_dir=tmp_path,
        install_root=root,
        input_config=input_config,
        expected_device="Kenet Game Sandbox",
        bbox=(10, 20, 80, 60),
        execute_dry_patch=False,
        apply_config_patch=False,
        run_live_gates=False,
        manual_response_magnitude=0.05,
        manual_response_min_shift_px=2.0,
        manual_response_max_shift_px=200.0,
        manual_response_startup_wait_s=0.0,
        timeout_s=1.0,
        command_runner=forbidden_runner,
    )

    stages = {stage.name: stage for stage in report.stages}
    assert report.status == WAITING
    assert stages["pr0p_client"].status == PASS
    assert stages["primary_rc_mapping"].status == PASS
    assert stages["aux1_arm_acceptance"].status == WAITING
    assert stages["manual_yaw_response"].status == "SKIPPED"
    assert stages["manual_pitch_response"].status == "SKIPPED"
    assert stages["manual_roll_response"].status == "SKIPPED"
    assert report.next_action.startswith("Bind AUX1/CH5")
    markdown = build_rc_manual_flight_markdown(report)
    assert "pr0p RC Manual Flight Gate" in markdown
    assert "--response-axis yaw" in markdown
    assert "+  --response-axis" not in markdown


def test_rc_manual_flight_gate_accepts_physical_primary_mapping(tmp_path, monkeypatch):
    root = tmp_path / "install"
    make_pr0p_client(root)
    monkeypatch.setattr("pr0p_install_probe.ALLOWED_INSTALL_ROOTS", (root,))
    input_config = tmp_path / "input.json"
    input_config.write_text(json.dumps(physical_mix_input_config()), encoding="utf-8")

    def forbidden_runner(*_args, **_kwargs):
        raise AssertionError("plan-only manual flight gate must not run external commands")

    report = run_manual_flight_gate(
        run_id="unit",
        log_dir=tmp_path,
        install_root=root,
        input_config=input_config,
        expected_device="Kenet Game Sandbox",
        allow_physical_mapping=True,
        bbox=(10, 20, 80, 60),
        execute_dry_patch=False,
        apply_config_patch=False,
        run_live_gates=False,
        manual_response_magnitude=0.05,
        manual_response_min_shift_px=2.0,
        manual_response_max_shift_px=200.0,
        manual_response_startup_wait_s=0.0,
        timeout_s=1.0,
        command_runner=forbidden_runner,
    )

    stages = {stage.name: stage for stage in report.stages}
    assert report.status == WAITING
    assert stages["primary_rc_mapping"].status == PASS
    assert report.metrics["allow_physical_mapping"] is True
    assert stages["aux1_arm_acceptance"].status == WAITING


def test_rc_manual_flight_gate_passes_after_aux1_live_acceptance(tmp_path, monkeypatch):
    root = tmp_path / "install"
    make_pr0p_client(root)
    monkeypatch.setattr("pr0p_install_probe.ALLOWED_INSTALL_ROOTS", (root,))
    input_config = tmp_path / "input.json"
    write_generic_virtual_input_config(input_config)
    calls = []

    def fake_runner(argv, **_kwargs):
        text = " ".join(argv)
        calls.append(text)
        if "--run-rc-effect" in text:
            stdout = "simitl-pr0p-live-session PASS report=logs/aux-rc.json summary=logs/aux-rc.md\n"
        elif "--run-aux-arm-status" in text:
            stdout = "simitl-pr0p-live-session PASS report=logs/aux-arm.json summary=logs/aux-arm.md\n"
        elif "--run-live-response" in text:
            axis = "yaw"
            if "--response-axis pitch" in text:
                axis = "pitch"
            elif "--response-axis roll" in text:
                axis = "roll"
            stdout = (
                "simitl-pr0p-live-session PASS report=logs/%s-response.json "
                "summary=logs/%s-response.md\n"
            ) % (axis, axis)
        else:
            stdout = "simitl-pr0p-input-config-patch PASS report=logs/dry.json summary=logs/dry.md\n"
        return subprocess.CompletedProcess(argv, 0, stdout=stdout, stderr="")

    report = run_manual_flight_gate(
        run_id="unit",
        log_dir=tmp_path,
        install_root=root,
        input_config=input_config,
        expected_device="Kenet Game Sandbox",
        bbox=(10, 20, 80, 60),
        execute_dry_patch=True,
        apply_config_patch=False,
        run_live_gates=True,
        manual_response_magnitude=0.05,
        manual_response_min_shift_px=2.0,
        manual_response_max_shift_px=200.0,
        manual_response_startup_wait_s=0.0,
        timeout_s=1.0,
        command_runner=fake_runner,
    )

    stages = {stage.name: stage for stage in report.stages}
    assert report.status == PASS
    assert stages["aux1_arm_acceptance"].status == PASS
    assert stages["manual_yaw_response"].status == PASS
    assert stages["manual_pitch_response"].status == PASS
    assert stages["manual_roll_response"].status == PASS
    assert any("--response-axis yaw" in call for call in calls)
    assert any("--response-axis pitch" in call for call in calls)
    assert any("--response-axis roll" in call for call in calls)
    assert report.next_action.startswith("Proceed to camera/tracker")


def test_rc_manual_flight_gate_requires_ack_for_write_and_live():
    with pytest.raises(SystemExit):
        parse_rc_manual_flight_args(["--apply-config-patch"])
    with pytest.raises(SystemExit):
        parse_rc_manual_flight_args([
            "--run-live-gates",
            "--ack-live-launch",
            "--ack-live-ui",
        ])


def test_aux1_acceptance_runner_plan_only_does_not_execute(tmp_path):
    def forbidden_runner(*_args, **_kwargs):
        raise AssertionError("plan-only must not run external commands")

    report = run_aux1_acceptance(
        run_id="unit",
        log_dir=tmp_path,
        bbox=(10, 20, 80, 60),
        execute_dry_patch=False,
        apply_config_patch=False,
        run_live_gates=False,
        timeout_s=1.0,
        command_runner=forbidden_runner,
    )

    steps = {step.name: step for step in report.steps}
    assert report.status == WAITING
    assert steps["pr0p_aux1_config_patch_dry"].status == "PLANNED"
    assert steps["pr0p_aux1_config_patch_write"].status == WAITING
    assert steps["pr0p_aux1_rc_effect"].status == WAITING
    assert steps["pr0p_aux1_arm_status"].status == WAITING
    assert "--run-aux-arm-status" in steps["pr0p_aux1_arm_status"].command
    markdown = build_aux1_acceptance_markdown(report)
    assert "SimITL / pr0p AUX1 Acceptance Runner" in markdown


def test_aux1_acceptance_runner_requires_ack_for_writes_and_live_input():
    with pytest.raises(SystemExit):
        parse_aux1_acceptance_args(["--apply-config-patch"])
    with pytest.raises(SystemExit):
        parse_aux1_acceptance_args([
            "--run-live-gates",
            "--ack-live-launch",
            "--ack-live-ui",
        ])


def test_aux1_acceptance_runner_runs_live_gates_after_rc_effect_pass(tmp_path):
    calls = []

    def fake_runner(argv, **_kwargs):
        text = " ".join(argv)
        calls.append(text)
        if "--run-rc-effect" in text:
            stdout = "simitl-pr0p-live-session PASS report=logs/aux-rc.json summary=logs/aux-rc.md\n"
        elif "--run-aux-arm-status" in text:
            stdout = "simitl-pr0p-live-session PASS report=logs/aux-arm.json summary=logs/aux-arm.md\n"
        else:
            stdout = "simitl-pr0p-input-config-patch PASS report=logs/dry.json summary=logs/dry.md\n"
        return subprocess.CompletedProcess(argv, 0, stdout=stdout, stderr="")

    report = run_aux1_acceptance(
        run_id="unit",
        log_dir=tmp_path,
        bbox=None,
        execute_dry_patch=True,
        apply_config_patch=False,
        run_live_gates=True,
        timeout_s=1.0,
        command_runner=fake_runner,
    )

    steps = {step.name: step for step in report.steps}
    assert report.status == PASS
    assert steps["pr0p_aux1_config_patch_dry"].status == PASS
    assert steps["pr0p_aux1_config_patch_write"].status == WAITING
    assert steps["pr0p_aux1_rc_effect"].status == PASS
    assert steps["pr0p_aux1_arm_status"].status == PASS
    assert calls.index(next(item for item in calls if "--run-rc-effect" in item)) < calls.index(
        next(item for item in calls if "--run-aux-arm-status" in item)
    )


def test_aux1_acceptance_runner_skips_arm_status_when_rc_effect_waits(tmp_path):
    def fake_runner(argv, **_kwargs):
        text = " ".join(argv)
        if "--run-rc-effect" in text:
            stdout = "simitl-pr0p-live-session WAITING report=logs/aux-rc.json summary=logs/aux-rc.md\n"
        else:
            stdout = "simitl-pr0p-input-config-patch PASS report=logs/dry.json summary=logs/dry.md\n"
        return subprocess.CompletedProcess(argv, 0, stdout=stdout, stderr="")

    report = run_aux1_acceptance(
        run_id="unit",
        log_dir=tmp_path,
        bbox=None,
        execute_dry_patch=True,
        apply_config_patch=False,
        run_live_gates=True,
        timeout_s=1.0,
        command_runner=fake_runner,
    )

    steps = {step.name: step for step in report.steps}
    assert report.status == WAITING
    assert steps["pr0p_aux1_rc_effect"].status == WAITING
    assert steps["pr0p_aux1_arm_status"].status == "SKIPPED"


def response_prereq_steps(*, aux_status=PASS):
    return [
        {"phase": "P3-capture", "status": PASS},
        {"phase": "P4-input-readiness", "status": PASS},
        {"phase": "P4-input-mapping", "status": PASS},
        {"phase": "P5-uinput-rc-effect-throttle-low", "status": PASS},
        {"phase": "P5-uinput-status-effect-throttle-clear", "status": PASS},
        {"phase": "P5-uinput-rc-effect-aux1-high", "status": PASS},
        {"phase": "P5-uinput-aux1-arm-status", "status": aux_status},
    ]


def test_response_acceptance_runner_plan_only_does_not_execute(tmp_path):
    write_pr0p_manifest(tmp_path / "20260707-unit-live-manifest.json", response_prereq_steps())

    def forbidden_runner(*_args, **_kwargs):
        raise AssertionError("plan-only must not run external commands")

    report = run_response_acceptance(
        run_id="unit",
        log_dir=tmp_path,
        run_live_gates=False,
        yaw_expected_sign=1,
        pitch_expected_sign=-1,
        yaw_magnitude=0.03,
        pitch_magnitude=0.03,
        min_shift_px=2.0,
        max_shift_px=200.0,
        startup_wait_s=8.0,
        timeout_s=1.0,
        command_runner=forbidden_runner,
    )

    steps = {step.name: step for step in report.steps}
    assert report.status == WAITING
    assert steps["pr0p_response_prerequisites"].status == PASS
    assert steps["pr0p_yaw_response_live"].status == WAITING
    assert steps["pr0p_pitch_response_live"].status == WAITING
    assert "--response-axis yaw" in steps["pr0p_yaw_response_live"].command
    markdown = build_response_acceptance_markdown(report)
    assert "SimITL / pr0p Response Acceptance Runner" in markdown


def test_response_acceptance_runner_requires_ack_for_live_input():
    with pytest.raises(SystemExit):
        parse_response_acceptance_args([
            "--run-live-gates",
            "--ack-live-launch",
            "--ack-live-ui",
        ])


def test_response_acceptance_runner_skips_live_when_aux_arm_missing(tmp_path):
    write_pr0p_manifest(
        tmp_path / "20260707-unit-live-manifest.json",
        response_prereq_steps(aux_status=WAITING),
    )

    def forbidden_runner(*_args, **_kwargs):
        raise AssertionError("live gates must be skipped until AUX1 arm status passes")

    report = run_response_acceptance(
        run_id="unit",
        log_dir=tmp_path,
        run_live_gates=True,
        yaw_expected_sign=1,
        pitch_expected_sign=-1,
        yaw_magnitude=0.03,
        pitch_magnitude=0.03,
        min_shift_px=2.0,
        max_shift_px=200.0,
        startup_wait_s=8.0,
        timeout_s=1.0,
        command_runner=forbidden_runner,
    )

    steps = {step.name: step for step in report.steps}
    assert report.status == WAITING
    assert steps["pr0p_response_prerequisites"].status == WAITING
    assert steps["pr0p_yaw_response_live"].status == "SKIPPED"
    assert steps["pr0p_pitch_response_live"].status == "SKIPPED"
    assert "MISSING_OR_WAITING:P5-uinput-aux1-arm-status" in steps["pr0p_response_prerequisites"].notes


def test_response_acceptance_runner_runs_pitch_after_yaw_pass(tmp_path):
    write_pr0p_manifest(tmp_path / "20260707-unit-live-manifest.json", response_prereq_steps())
    calls = []

    def fake_runner(argv, **_kwargs):
        text = " ".join(argv)
        calls.append(text)
        stdout = "simitl-pr0p-live-session PASS report=logs/%s.json summary=logs/%s.md\n" % (
            "yaw" if "--response-axis yaw" in text else "pitch",
            "yaw" if "--response-axis yaw" in text else "pitch",
        )
        return subprocess.CompletedProcess(argv, 0, stdout=stdout, stderr="")

    report = run_response_acceptance(
        run_id="unit",
        log_dir=tmp_path,
        run_live_gates=True,
        yaw_expected_sign=1,
        pitch_expected_sign=-1,
        yaw_magnitude=0.03,
        pitch_magnitude=0.03,
        min_shift_px=2.0,
        max_shift_px=200.0,
        startup_wait_s=8.0,
        timeout_s=1.0,
        command_runner=fake_runner,
    )

    steps = {step.name: step for step in report.steps}
    assert report.status == PASS
    assert steps["pr0p_yaw_response_live"].status == PASS
    assert steps["pr0p_pitch_response_live"].status == PASS
    assert calls.index(next(item for item in calls if "--response-axis yaw" in item)) < calls.index(
        next(item for item in calls if "--response-axis pitch" in item)
    )


def test_response_acceptance_runner_skips_pitch_when_yaw_waits(tmp_path):
    write_pr0p_manifest(tmp_path / "20260707-unit-live-manifest.json", response_prereq_steps())

    def fake_runner(argv, **_kwargs):
        stdout = "simitl-pr0p-live-session WAITING report=logs/yaw.json summary=logs/yaw.md\n"
        return subprocess.CompletedProcess(argv, 0, stdout=stdout, stderr="")

    report = run_response_acceptance(
        run_id="unit",
        log_dir=tmp_path,
        run_live_gates=True,
        yaw_expected_sign=1,
        pitch_expected_sign=-1,
        yaw_magnitude=0.03,
        pitch_magnitude=0.03,
        min_shift_px=2.0,
        max_shift_px=200.0,
        startup_wait_s=8.0,
        timeout_s=1.0,
        command_runner=fake_runner,
    )

    steps = {step.name: step for step in report.steps}
    assert report.status == WAITING
    assert steps["pr0p_yaw_response_live"].status == WAITING
    assert steps["pr0p_pitch_response_live"].status == "SKIPPED"


def tracking_prereq_steps(*, p6_status=PASS):
    return [
        {"phase": "P3-capture", "status": PASS},
        {"phase": "P4-input-readiness", "status": PASS},
        {"phase": "P4-input-mapping", "status": PASS},
        {"phase": "P6-tracking-pid-dry-run", "status": p6_status},
    ]


def acceptance_chain_prereq_steps(*, p6_status=PASS):
    return [
        {"phase": "P3-capture", "status": PASS},
        {"phase": "P4-input-readiness", "status": PASS},
        {"phase": "P4-input-mapping", "status": PASS},
        {"phase": "P5-uinput-rc-effect-throttle-low", "status": PASS},
        {"phase": "P5-uinput-status-effect-throttle-clear", "status": PASS},
        {"phase": "P5-uinput-rc-effect-aux1-high", "status": PASS},
        {"phase": "P5-uinput-aux1-arm-status", "status": PASS},
        {"phase": "P6-tracking-pid-dry-run", "status": p6_status},
    ]


def acceptance_chain_kwargs(tmp_path, **overrides):
    install_root = Path("/tmp/fpv-test-simitl-pr0p") / ("pytest-%s" % tmp_path.name)
    make_pr0p_client(install_root)
    input_config = tmp_path / "input.json"
    write_generic_virtual_input_config(input_config)
    kwargs = {
        "run_id": "unit",
        "log_dir": tmp_path,
        "game_log_dir": tmp_path / "game",
        "install_root": install_root,
        "input_config": input_config,
        "expected_device": "Kenet Game Sandbox",
        "bbox": (10, 20, 80, 60),
        "execute_dry_patch": False,
        "apply_config_patch": False,
        "run_live_gates": False,
        "execute_tracking_dry_run": False,
        "window_title": "pr0p",
        "yaw_expected_sign": 1,
        "pitch_expected_sign": -1,
        "manual_response_magnitude": 0.05,
        "yaw_magnitude": 0.03,
        "pitch_magnitude": 0.03,
        "min_shift_px": 2.0,
        "max_shift_px": 200.0,
        "startup_wait_s": 8.0,
        "duration_s": 5.0,
        "hz": 10.0,
        "tracker": "CSRT",
        "enable_pitch": False,
        "min_found_ratio": 0.85,
        "max_loss_events": 0,
        "max_abs_yaw_axis": 0.6,
        "max_abs_pitch_axis": 0.6,
        "desired_target_width": 120.0,
        "timeout_s": 1.0,
    }
    kwargs.update(overrides)
    return kwargs


def test_tracking_acceptance_runner_plan_only_does_not_execute(tmp_path):
    write_pr0p_manifest(tmp_path / "20260707-unit-live-manifest.json", tracking_prereq_steps())
    write_response_acceptance(tmp_path / "20260707-unit-response-acceptance.json")

    def forbidden_runner(*_args, **_kwargs):
        raise AssertionError("plan-only must not run external commands")

    report = run_tracking_acceptance(
        run_id="unit",
        log_dir=tmp_path,
        bbox=(10, 20, 80, 60),
        execute_dry_run=False,
        run_live_gates=False,
        window_title="pr0p",
        duration_s=5.0,
        hz=10.0,
        tracker="CSRT",
        enable_pitch=False,
        min_found_ratio=0.85,
        max_loss_events=0,
        max_abs_yaw_axis=0.6,
        max_abs_pitch_axis=0.6,
        desired_target_width=120.0,
        timeout_s=1.0,
        command_runner=forbidden_runner,
    )

    steps = {step.name: step for step in report.steps}
    assert report.status == WAITING
    assert steps["pr0p_tracking_bbox"].status == PASS
    assert steps["pr0p_tracking_manifest_prerequisites"].status == PASS
    assert steps["pr0p_tracking_response_prerequisites"].status == PASS
    assert steps["pr0p_tracking_live"].status == WAITING
    assert "--uinput" in steps["pr0p_tracking_live"].command
    markdown = build_tracking_acceptance_markdown(report)
    assert "SimITL / pr0p Tracking Acceptance Runner" in markdown


def test_tracking_acceptance_runner_requires_ack_for_live_input():
    with pytest.raises(SystemExit):
        parse_tracking_acceptance_args(["--run-live-gates", "--bbox", "1,2,10,20"])


def test_tracking_acceptance_runner_skips_live_without_response_pass(tmp_path):
    write_pr0p_manifest(tmp_path / "20260707-unit-live-manifest.json", tracking_prereq_steps())
    write_response_acceptance(
        tmp_path / "20260707-unit-response-acceptance.json",
        status=WAITING,
        yaw_status=PASS,
        pitch_status=WAITING,
    )

    def forbidden_runner(*_args, **_kwargs):
        raise AssertionError("live tracking must be skipped until response acceptance passes")

    report = run_tracking_acceptance(
        run_id="unit",
        log_dir=tmp_path,
        bbox=(10, 20, 80, 60),
        execute_dry_run=False,
        run_live_gates=True,
        window_title="pr0p",
        duration_s=5.0,
        hz=10.0,
        tracker="CSRT",
        enable_pitch=False,
        min_found_ratio=0.85,
        max_loss_events=0,
        max_abs_yaw_axis=0.6,
        max_abs_pitch_axis=0.6,
        desired_target_width=120.0,
        timeout_s=1.0,
        command_runner=forbidden_runner,
    )

    steps = {step.name: step for step in report.steps}
    assert report.status == WAITING
    assert steps["pr0p_tracking_response_prerequisites"].status == WAITING
    assert steps["pr0p_tracking_live"].status == "SKIPPED"
    assert "MISSING_OR_WAITING:pr0p_pitch_response_live" in steps[
        "pr0p_tracking_response_prerequisites"
    ].notes


def test_tracking_acceptance_runner_runs_live_after_prereqs_pass(tmp_path):
    write_pr0p_manifest(tmp_path / "20260707-unit-live-manifest.json", tracking_prereq_steps())
    write_response_acceptance(tmp_path / "20260707-unit-response-acceptance.json")
    calls = []

    def fake_runner(argv, **_kwargs):
        text = " ".join(argv)
        calls.append(text)
        stdout = "simitl-pr0p-tracking PASS report=logs/live.json summary=logs/live.md real_input_sent=True\n"
        return subprocess.CompletedProcess(argv, 0, stdout=stdout, stderr="")

    report = run_tracking_acceptance(
        run_id="unit",
        log_dir=tmp_path,
        bbox=(10, 20, 80, 60),
        execute_dry_run=False,
        run_live_gates=True,
        window_title="pr0p",
        duration_s=5.0,
        hz=10.0,
        tracker="CSRT",
        enable_pitch=False,
        min_found_ratio=0.85,
        max_loss_events=0,
        max_abs_yaw_axis=0.6,
        max_abs_pitch_axis=0.6,
        desired_target_width=120.0,
        timeout_s=1.0,
        arm_first=True,
        command_runner=fake_runner,
    )

    steps = {step.name: step for step in report.steps}
    assert report.status == PASS
    assert steps["pr0p_tracking_live"].status == PASS
    assert "--bbox 10,20,80,60" in steps["pr0p_tracking_live"].command
    assert "--ack-live-input" in calls[0]
    assert "--takeoff-delay-frames 10" in calls[0]
    assert "--settle-throttle -0.26" in calls[0]


def test_tracking_acceptance_runner_executes_dry_run_without_live(tmp_path):
    write_pr0p_manifest(tmp_path / "20260707-unit-live-manifest.json", tracking_prereq_steps(p6_status=WAITING))
    write_response_acceptance(tmp_path / "20260707-unit-response-acceptance.json")

    def fake_runner(argv, **_kwargs):
        text = " ".join(argv)
        assert "--uinput" not in text
        stdout = "simitl-pr0p-tracking PASS report=logs/dry.json summary=logs/dry.md real_input_sent=False\n"
        return subprocess.CompletedProcess(argv, 0, stdout=stdout, stderr="")

    report = run_tracking_acceptance(
        run_id="unit",
        log_dir=tmp_path,
        bbox=(10, 20, 80, 60),
        execute_dry_run=True,
        run_live_gates=False,
        window_title="pr0p",
        duration_s=5.0,
        hz=10.0,
        tracker="CSRT",
        enable_pitch=False,
        min_found_ratio=0.85,
        max_loss_events=0,
        max_abs_yaw_axis=0.6,
        max_abs_pitch_axis=0.6,
        desired_target_width=120.0,
        timeout_s=1.0,
        command_runner=fake_runner,
    )

    steps = {step.name: step for step in report.steps}
    assert report.status == WAITING
    assert steps["pr0p_tracking_dry_run"].status == PASS
    assert steps["pr0p_tracking_live"].status == WAITING


def test_acceptance_chain_runner_plan_only_stops_at_rc_manual_flight(tmp_path):
    def forbidden_runner(*_args, **_kwargs):
        raise AssertionError("plan-only chain must not run external commands")

    report = run_acceptance_chain(
        **acceptance_chain_kwargs(
            tmp_path,
            command_runner=forbidden_runner,
        )
    )

    stages = {stage.name: stage for stage in report.stages}
    assert report.status == WAITING
    assert stages["rc_manual_flight"].status == WAITING
    assert stages["pr0p_live_manifest_after_rc_manual_flight"].status == "SKIPPED"
    assert stages["pr0p_response_acceptance"].status == "SKIPPED"
    assert stages["pr0p_tracking_acceptance"].status == "SKIPPED"
    assert report.next_action.startswith("Run pr0p_rc_manual_flight_runner.py")
    markdown = build_acceptance_chain_markdown(report)
    assert "SimITL / pr0p Acceptance Chain" in markdown


def test_acceptance_chain_runner_requires_ack_for_live_and_config():
    with pytest.raises(SystemExit):
        parse_acceptance_chain_args([
            "--run-live-gates",
            "--ack-live-launch",
            "--ack-live-ui",
        ])
    with pytest.raises(SystemExit):
        parse_acceptance_chain_args(["--apply-config-patch"])
    with pytest.raises(SystemExit):
        parse_acceptance_chain_args(["--manual-response-magnitude", "0"])


def test_acceptance_chain_runner_runs_response_only_after_rc_manual_flight_pass(tmp_path):
    write_pr0p_suite(tmp_path / "20260707-pr0p-suite.json", acceptance_chain_prereq_steps())
    calls = []

    def fake_runner(argv, **_kwargs):
        text = " ".join(argv)
        calls.append(text)
        if "--run-live-response" in text and "--response-expected-sign 0" in text:
            stdout = "simitl-pr0p-live-session PASS report=logs/manual-response.json summary=logs/manual-response.md\n"
        elif "--response-axis yaw" in text:
            stdout = "simitl-pr0p-live-session WAITING report=logs/yaw.json summary=logs/yaw.md\n"
        elif "--response-axis pitch" in text:
            raise AssertionError("pitch must not run until yaw passes")
        elif "pr0p_tracking_probe.py" in text:
            raise AssertionError("tracking must not run until response passes")
        elif "--run-rc-effect" in text:
            stdout = "simitl-pr0p-live-session PASS report=logs/aux-rc.json summary=logs/aux-rc.md\n"
        elif "--run-aux-arm-status" in text:
            stdout = "simitl-pr0p-live-session PASS report=logs/aux-arm.json summary=logs/aux-arm.md\n"
        else:
            stdout = "simitl-pr0p-input-config-patch PASS report=logs/dry.json summary=logs/dry.md\n"
        return subprocess.CompletedProcess(argv, 0, stdout=stdout, stderr="")

    report = run_acceptance_chain(
        **acceptance_chain_kwargs(
            tmp_path,
            execute_dry_patch=True,
            run_live_gates=True,
            arm_first=True,
            arm_throttle=-0.4,
            measure="attitude",
            attitude_min_delta_deg=12.5,
            command_runner=fake_runner,
        )
    )

    stages = {stage.name: stage for stage in report.stages}
    assert report.status == WAITING
    assert stages["rc_manual_flight"].status == PASS
    assert stages["pr0p_response_acceptance"].status == WAITING
    assert stages["pr0p_tracking_acceptance"].status == "SKIPPED"
    signed_yaw = next(
        item for item in calls
        if "--response-axis yaw" in item and "--response-expected-sign 1" in item
    )
    assert calls.index(next(item for item in calls if "--run-aux-arm-status" in item)) < calls.index(
        signed_yaw
    )
    assert calls.index(next(item for item in calls if "--response-axis roll" in item)) < calls.index(
        signed_yaw
    )


def test_acceptance_chain_runner_runs_tracking_after_response_pass(tmp_path):
    write_pr0p_suite(tmp_path / "20260707-pr0p-suite.json", readiness_promotion_steps())
    calls = []

    def fake_runner(argv, **_kwargs):
        text = " ".join(argv)
        calls.append(text)
        if "--run-live-response" in text and "--response-expected-sign 0" in text:
            stdout = "simitl-pr0p-live-session PASS report=logs/manual-response.json summary=logs/manual-response.md\n"
        elif "--response-axis yaw" in text:
            stdout = "simitl-pr0p-live-session PASS report=logs/yaw.json summary=logs/yaw.md\n"
        elif "--response-axis pitch" in text:
            stdout = "simitl-pr0p-live-session PASS report=logs/pitch.json summary=logs/pitch.md\n"
        elif "pr0p_tracking_probe.py" in text:
            stdout = "simitl-pr0p-tracking PASS report=logs/live.json summary=logs/live.md real_input_sent=True\n"
        elif "--run-rc-effect" in text:
            stdout = "simitl-pr0p-live-session PASS report=logs/aux-rc.json summary=logs/aux-rc.md\n"
        elif "--run-aux-arm-status" in text:
            stdout = "simitl-pr0p-live-session PASS report=logs/aux-arm.json summary=logs/aux-arm.md\n"
        else:
            stdout = "simitl-pr0p-input-config-patch PASS report=logs/dry.json summary=logs/dry.md\n"
        return subprocess.CompletedProcess(argv, 0, stdout=stdout, stderr="")

    report = run_acceptance_chain(
        **acceptance_chain_kwargs(
            tmp_path,
            execute_dry_patch=True,
            run_live_gates=True,
            arm_first=True,
            arm_throttle=-0.4,
            measure="attitude",
            attitude_min_delta_deg=12.5,
            command_runner=fake_runner,
        )
    )

    stages = {stage.name: stage for stage in report.stages}
    assert report.status == PASS
    assert stages["rc_manual_flight"].status == PASS
    assert stages["pr0p_response_acceptance"].status == PASS
    assert stages["pr0p_tracking_acceptance"].status == PASS
    assert stages["pr0p_live_manifest_refresh"].status == PASS
    assert stages["pr0p_decision_refresh"].status == PROMOTE_CANDIDATE
    assert stages["independent_sim_readiness_refresh"].status == WAITING
    assert stages["pr0p_decision_refresh"].report is not None
    assert stages["independent_sim_readiness_refresh"].report is not None
    assert report.next_action == (
        "Inspect the generated decision/readiness reports, then run "
        "pr0p_core_goal_report.py or pr0p_core_next_runner.py for the "
        "measured post-core gates."
    )
    assert calls.index(next(item for item in calls if "--response-axis pitch" in item)) < calls.index(
        next(item for item in calls if "pr0p_tracking_probe.py" in item)
    )
    signed_yaw = next(
        item for item in calls
        if "--response-axis yaw" in item and "--response-expected-sign 1" in item
    )
    tracking_live = next(item for item in calls if "pr0p_tracking_probe.py" in item)
    assert "--response-arm-first" in signed_yaw
    assert "--response-arm-throttle -0.4" in signed_yaw
    assert "--response-measure attitude" in signed_yaw
    assert "--response-attitude-min-delta 12.5" in signed_yaw
    assert "--arm-first" in tracking_live
    assert "--arm-throttle -0.4" in tracking_live


def test_pr0p_docs_cover_chain_refresh_and_promotion_guards():
    readme = (PROBE / "README.md").read_text(encoding="utf-8")
    plan = (PROBE / "PLAN.md").read_text(encoding="utf-8")

    for text in (readme, plan):
        assert "pr0p_acceptance_chain_runner.py" in text
        assert "pr0p_decision_refresh" in text
        assert "independent_sim_readiness_refresh" in text
        assert "--game-log-dir" in text
        assert "*-live-manifest.json" in text
        assert "latest suite" in text
        assert "safe suite" in text
    assert "pr0p_core_goal_report.py" in readme
    assert "pr0p_core_next_runner.py" in readme
    assert "objective_completion_status" in readme
    assert "objective_completion_missing" in readme
    assert "objective_completion_status" in plan
    assert "objective_completion_missing" in plan
    assert "objective_requirements" in readme
    assert "objective_requirement_statuses" in readme
    assert "gazebo_independent_path" in readme
    assert "manual_to_auto_handoff" in readme
    assert "objective_requirements" in plan
    assert "objective_requirement_statuses" in plan
    assert "gazebo_independent_path" in plan
    assert "manual_to_auto_handoff" in plan
    assert "independent_sim_readiness.py" in readme
    assert "S13-binding-dry" in readme
    assert "MISMATCH" in readme
    assert "S13-binding-dry" in plan
    assert "MISMATCH" in plan
    assert "precheck" in readme
    assert "--precheck-live-readiness" in readme
    assert "--precheck-live-readiness" in plan
    assert "safe_to_execute_live_with_ack" in readme
    assert "safe_to_execute_live_blockers" in readme
    assert "live_blocker_summary" in readme
    assert "isolation_status" in readme
    assert "GAZEBO_INDEPENDENCE_FAILED" in readme
    assert "safe_to_execute_live_with_ack" in plan
    assert "safe_to_execute_live_blockers" in plan
    assert "live_blocker_summary" in plan
    assert "isolation_status" in plan
    assert "GAZEBO_INDEPENDENCE_FAILED" in plan
    assert "next_objective_gate" in readme
    assert "next_objective_gate_snapshot" in readme
    assert "next_objective_gate" in plan
    assert "next_objective_gate_snapshot" in plan
    assert "--precheck-operator-readiness" in readme
    assert "--skip-operator-readiness-precheck" in readme
    assert "operator_precheck_status" in readme
    assert "safe_to_execute_live_with_operator_precheck" in readme
    assert "safe_to_execute_live_with_all_prechecks" in readme
    assert "safe_to_execute_live_required_blockers" in readme
    assert "operator_readiness_freshness_gate_status" in readme
    assert "operator_readiness_missing_evidence" in readme
    assert "operator_readiness_recommended_command" in readme
    assert "operator_readiness_next_command_packet" in readme
    assert "operator_next_command" in readme
    assert "operator_readiness_window_discovery_status" in readme
    assert "operator_window_title_candidates" in readme
    assert "--execute-operator-next" in readme
    assert "--ack-operator-candidate" in readme
    assert "post_operator_precheck_status" in readme
    assert "post_operator_readiness_summary" in readme
    assert "bbox_file_mismatch" in readme
    assert "--precheck-operator-readiness" in plan
    assert "--skip-operator-readiness-precheck" in plan
    assert "operator_precheck_status" in plan
    assert "safe_to_execute_live_with_operator_precheck" in plan
    assert "safe_to_execute_live_with_all_prechecks" in plan
    assert "safe_to_execute_live_required_blockers" in plan
    assert "operator_readiness_freshness_gate_status" in plan
    assert "operator_readiness_missing_evidence" in plan
    assert "operator_readiness_recommended_command" in plan
    assert "operator_readiness_next_command_packet" in plan
    assert "operator_next_command" in plan
    assert "operator_readiness_window_discovery_status" in plan
    assert "operator_window_title_candidates" in plan
    assert "--execute-operator-next" in plan
    assert "--ack-operator-candidate" in plan
    assert "post_operator_precheck_status" in plan
    assert "post_operator_readiness_summary" in plan
    assert "bbox_file_mismatch" in plan
    assert "INDEPENDENT_SIM_READY" in readme
    assert "readiness precheck" in plan
    assert "pr0p_rc_manual_flight_runner.py" in readme
    assert "rc_manual_flight" in readme
    assert "camera_tracker" in readme
    assert "autopilot_control" in readme
    assert "dependency-gated" in readme
    assert "Next command key" in readme
    assert "Commands" in readme
    assert "extended_follow_live" in readme
    assert "extended_follow_live" in plan
    assert "moving_target_yaw_plan" in readme
    assert "moving_target_yaw_plan" in plan
    assert "airborne_static_yaw_live" in readme
    assert "airborne_static_yaw_live" in plan
    assert "airborne_static_yaw_refresh" in readme
    assert "airborne_static_yaw_refresh" in plan
    assert "pr0p_moving_target_acceptance_runner.py" in readme
    assert "pr0p_moving_target_acceptance_runner.py" in plan
    assert "--real-tracking-report" in readme
    assert "--ack-airborne-static-target" in readme
    assert "--ack-airborne-static-target" in plan
    assert "approach_pitch_plan" in readme
    assert "approach_pitch_plan" in plan
    assert "approach_pitch_live" in readme
    assert "approach_pitch_live" in plan
    assert "approach_pitch_refresh" in readme
    assert "approach_pitch_refresh" in plan
    assert "--ack-real-approach-target" in readme
    assert "--ack-real-approach-target" in plan
    assert "handoff_plan" in readme
    assert "handoff_plan" in plan
    assert "handoff_live" in readme
    assert "handoff_live" in plan
    assert "handoff_refresh" in readme
    assert "handoff_refresh" in plan
    assert "pr0p_handoff_acceptance_runner.py" in readme
    assert "pr0p_handoff_acceptance_runner.py" in plan
    assert "--ack-real-handoff" in readme
    assert "--ack-real-handoff" in plan


def test_decision_report_waits_without_evidence():
    report = evaluate_decision(None, evidence_path=None)
    assert report.decision == WAITING
    assert "NO_EVIDENCE_REPORT" in report.reasons


def promotion_gate_steps():
    return [
        {"phase": "P2-websocket", "status": PASS},
        {"phase": "P2-msp-readonly", "status": PASS},
        {"phase": "P2-fc-status-readonly", "status": PASS},
        {"phase": "P2-mode-ranges-readonly", "status": PASS},
        {"phase": "P0-isolation", "status": PASS},
        {"phase": "P1-install-discovery", "status": PASS},
        {"phase": "P1-client-executable", "status": PASS},
        {"phase": "P3-capture", "status": PASS},
        {"phase": "P4-input-readiness", "status": PASS},
        {"phase": "P4-input-mapping", "status": PASS},
        {"phase": "P4-rc-channels-visual", "status": PASS},
        {"phase": "P5-yaw-live", "status": PASS},
        {"phase": "P5-pitch-live", "status": PASS},
        {"phase": "P5-uinput-rc-effect-throttle-low", "status": PASS},
        {"phase": "P5-uinput-status-effect-throttle-clear", "status": PASS},
        {"phase": "P5-uinput-arm-button-effect", "status": PASS},
        {"phase": "P5-uinput-rc-effect-aux1-high", "status": PASS},
        {"phase": "P5-uinput-aux1-arm-status", "status": PASS},
        {"phase": "P5-rc-loopback", "status": PASS},
        {"phase": "P5-persistent-uinput-live-response", "status": PASS},
        {"phase": "P5-response-acceptance", "status": PASS},
        {"phase": "P6-synthetic-e2e-dry-run", "status": PASS},
        {"phase": "P6-synthetic-log-check", "status": PASS},
        {"phase": "P6-tracking-pid-dry-run", "status": PASS},
        {"phase": "P6-tracking-log-check", "status": PASS},
        {"phase": "P6-tracking-live", "status": PASS},
        {"phase": "P6-tracking-acceptance", "status": PASS},
    ]


def promotion_manifest(*, status=PASS, steps=None):
    return {
        "status": status,
        "next_action": "ready for decision",
        "suite_report": "logs/pr0p-suite.json",
        "metadata": {"source": "unit"},
        "steps": promotion_gate_steps() if steps is None else steps,
    }


def test_decision_report_rejects_failed_phase(tmp_path):
    evidence = {
        "status": FAIL,
        "steps": [
            {"phase": "P3-capture", "status": FAIL},
            {"phase": "P4-input-readiness", "status": PASS},
        ],
    }
    report = evaluate_decision(evidence, evidence_path=tmp_path / "evidence.json")
    assert report.decision == REJECT
    assert "FAIL:P3-capture" in report.reasons


def test_decision_report_waits_when_source_status_is_not_pass(tmp_path):
    evidence = promotion_manifest(status=WAITING)

    report = evaluate_decision(evidence, evidence_path=tmp_path / "evidence.json")

    assert report.decision == WAITING
    assert "SOURCE_STATUS_NOT_PASS:WAITING" in report.reasons


def test_decision_report_waits_on_stale_evidence_path(tmp_path):
    evidence_path = tmp_path / "stale-live-manifest.json"
    evidence = {
        "status": PASS,
        "steps": promotion_gate_steps(),
    }
    evidence_path.write_text(json.dumps(evidence), encoding="utf-8")
    os.utime(evidence_path, ns=(1_000_000_000, 1_000_000_000))

    report = evaluate_decision(
        evidence,
        evidence_path=evidence_path,
        max_evidence_age_s=60.0,
        now_s=120.0,
    )

    assert report.decision == WAITING
    assert report.summary == "evidence is stale; rerun the live manifest or safe suite"
    assert "EVIDENCE_STALE" in report.reasons
    assert report.metrics["evidence_freshness"]["status"] == "STALE"


def test_decision_report_rejects_negative_evidence_age():
    with pytest.raises(SystemExit) as excinfo:
        parse_decision_args(["--max-evidence-age-s", "-1"])

    assert excinfo.value.code == 2


def test_decision_report_promotes_only_when_all_gates_pass(tmp_path):
    evidence = promotion_manifest()
    report = evaluate_decision(evidence, evidence_path=tmp_path / "evidence.json")
    assert report.decision == PROMOTE_CANDIDATE
    assert report.reasons == []
    assert report.metrics["source_kind"] == "live_manifest"


def test_decision_report_does_not_promote_suite_without_live_manifest(tmp_path):
    evidence = {
        "status": PASS,
        "steps": promotion_gate_steps(),
    }

    report = evaluate_decision(evidence, evidence_path=tmp_path / "20260707-pr0p-suite.json")

    assert report.decision == WAITING
    assert "MISSING_LIVE_MANIFEST_EVIDENCE" in report.reasons
    assert report.metrics["source_kind"] == "suite"


def test_decision_report_requires_acceptance_runner_gates(tmp_path):
    evidence = {
        "status": PASS,
        "steps": [
            {"phase": "P2-websocket", "status": PASS},
            {"phase": "P2-msp-readonly", "status": PASS},
            {"phase": "P2-fc-status-readonly", "status": PASS},
            {"phase": "P2-mode-ranges-readonly", "status": PASS},
            {"phase": "P0-isolation", "status": PASS},
            {"phase": "P1-install-discovery", "status": PASS},
            {"phase": "P1-client-executable", "status": PASS},
            {"phase": "P3-capture", "status": PASS},
            {"phase": "P4-input-readiness", "status": PASS},
            {"phase": "P4-input-mapping", "status": PASS},
            {"phase": "P4-rc-channels-visual", "status": PASS},
            {"phase": "P5-yaw-live", "status": PASS},
            {"phase": "P5-pitch-live", "status": PASS},
            {"phase": "P5-uinput-rc-effect-throttle-low", "status": PASS},
            {"phase": "P5-uinput-status-effect-throttle-clear", "status": PASS},
            {"phase": "P5-uinput-arm-button-effect", "status": PASS},
            {"phase": "P5-uinput-rc-effect-aux1-high", "status": PASS},
            {"phase": "P5-uinput-aux1-arm-status", "status": PASS},
            {"phase": "P5-rc-loopback", "status": PASS},
            {"phase": "P5-persistent-uinput-live-response", "status": PASS},
            {"phase": "P6-synthetic-e2e-dry-run", "status": PASS},
            {"phase": "P6-synthetic-log-check", "status": PASS},
            {"phase": "P6-tracking-pid-dry-run", "status": PASS},
            {"phase": "P6-tracking-log-check", "status": PASS},
            {"phase": "P6-tracking-live", "status": PASS},
        ],
    }
    report = evaluate_decision(evidence, evidence_path=tmp_path / "evidence.json")
    assert report.decision == WAITING
    assert "MISSING_RESPONSE_ACCEPTANCE" in report.reasons
    assert "MISSING_TRACKING_ACCEPTANCE" in report.reasons


def test_decision_report_blocks_promotion_when_fc_is_not_armed_without_aux_arm_status(tmp_path):
    evidence = {
        "status": PASS,
        "steps": [
            {"phase": "P2-websocket", "status": PASS},
            {"phase": "P2-msp-readonly", "status": PASS},
            {
                "phase": "P2-fc-status-readonly",
                "status": PASS,
                "notes": ["FC_NOT_ARMED", "ARMING_DISABLED:THROTTLE"],
            },
            {"phase": "P2-mode-ranges-readonly", "status": PASS},
            {"phase": "P0-isolation", "status": PASS},
            {"phase": "P1-install-discovery", "status": PASS},
            {"phase": "P1-client-executable", "status": PASS},
            {"phase": "P3-capture", "status": PASS},
            {"phase": "P4-input-readiness", "status": PASS},
            {"phase": "P4-input-mapping", "status": PASS},
            {"phase": "P4-rc-channels-visual", "status": PASS},
            {"phase": "P5-yaw-live", "status": PASS},
            {"phase": "P5-pitch-live", "status": PASS},
            {"phase": "P5-uinput-rc-effect-throttle-low", "status": PASS},
            {"phase": "P5-uinput-status-effect-throttle-clear", "status": PASS},
            {"phase": "P5-uinput-arm-button-effect", "status": PASS},
            {"phase": "P5-uinput-rc-effect-aux1-high", "status": PASS},
            {"phase": "P5-rc-loopback", "status": PASS},
            {"phase": "P5-persistent-uinput-live-response", "status": PASS},
            {"phase": "P5-response-acceptance", "status": PASS},
            {"phase": "P6-synthetic-e2e-dry-run", "status": PASS},
            {"phase": "P6-synthetic-log-check", "status": PASS},
            {"phase": "P6-tracking-pid-dry-run", "status": PASS},
            {"phase": "P6-tracking-log-check", "status": PASS},
            {"phase": "P6-tracking-live", "status": PASS},
            {"phase": "P6-tracking-acceptance", "status": PASS},
        ],
    }
    report = evaluate_decision(evidence, evidence_path=tmp_path / "evidence.json")
    assert report.decision == WAITING
    assert "FC_ARM_STATE_BLOCKED" in report.reasons
    assert "MISSING_UINPUT_AUX1_ARM_STATUS" in report.reasons


def test_decision_report_allows_baseline_fc_not_armed_when_aux_arm_status_passes(tmp_path):
    evidence = promotion_manifest(steps=[
            {"phase": "P2-websocket", "status": PASS},
            {"phase": "P2-msp-readonly", "status": PASS},
            {
                "phase": "P2-fc-status-readonly",
                "status": PASS,
                "notes": ["FC_NOT_ARMED", "ARMING_DISABLED:THROTTLE"],
            },
            {"phase": "P2-mode-ranges-readonly", "status": PASS},
            {"phase": "P0-isolation", "status": PASS},
            {"phase": "P1-install-discovery", "status": PASS},
            {"phase": "P1-client-executable", "status": PASS},
            {"phase": "P3-capture", "status": PASS},
            {"phase": "P4-input-readiness", "status": PASS},
            {"phase": "P4-input-mapping", "status": PASS},
            {"phase": "P4-rc-channels-visual", "status": PASS},
            {"phase": "P5-yaw-live", "status": PASS},
            {"phase": "P5-pitch-live", "status": PASS},
            {"phase": "P5-uinput-rc-effect-throttle-low", "status": PASS},
            {"phase": "P5-uinput-status-effect-throttle-clear", "status": PASS},
            {"phase": "P5-uinput-arm-button-effect", "status": PASS},
            {"phase": "P5-uinput-rc-effect-aux1-high", "status": PASS},
            {"phase": "P5-uinput-aux1-arm-status", "status": PASS},
            {"phase": "P5-rc-loopback", "status": PASS},
            {"phase": "P5-persistent-uinput-live-response", "status": PASS},
            {"phase": "P5-response-acceptance", "status": PASS},
            {"phase": "P6-synthetic-e2e-dry-run", "status": PASS},
            {"phase": "P6-synthetic-log-check", "status": PASS},
            {"phase": "P6-tracking-pid-dry-run", "status": PASS},
            {"phase": "P6-tracking-log-check", "status": PASS},
            {"phase": "P6-tracking-live", "status": PASS},
            {"phase": "P6-tracking-acceptance", "status": PASS},
    ])
    report = evaluate_decision(evidence, evidence_path=tmp_path / "evidence.json")
    assert report.decision == PROMOTE_CANDIDATE
    assert "FC_ARM_STATE_BLOCKED" not in report.reasons


def test_decision_report_waits_without_uinput_rc_effect(tmp_path):
    evidence = {
        "status": PASS,
        "steps": [
            {"phase": "P2-websocket", "status": PASS},
            {"phase": "P2-msp-readonly", "status": PASS},
            {"phase": "P2-fc-status-readonly", "status": PASS},
            {"phase": "P2-mode-ranges-readonly", "status": PASS},
            {"phase": "P0-isolation", "status": PASS},
            {"phase": "P1-install-discovery", "status": PASS},
            {"phase": "P1-client-executable", "status": PASS},
            {"phase": "P3-capture", "status": PASS},
            {"phase": "P4-input-readiness", "status": PASS},
            {"phase": "P4-input-mapping", "status": PASS},
            {"phase": "P4-rc-channels-visual", "status": PASS},
            {"phase": "P5-yaw-live", "status": PASS},
            {"phase": "P5-pitch-live", "status": PASS},
            {"phase": "P5-rc-loopback", "status": PASS},
            {"phase": "P5-persistent-uinput-live-response", "status": PASS},
            {"phase": "P5-response-acceptance", "status": PASS},
            {"phase": "P6-synthetic-e2e-dry-run", "status": PASS},
            {"phase": "P6-synthetic-log-check", "status": PASS},
            {"phase": "P6-tracking-pid-dry-run", "status": PASS},
            {"phase": "P6-tracking-log-check", "status": PASS},
            {"phase": "P6-tracking-live", "status": PASS},
            {"phase": "P6-tracking-acceptance", "status": PASS},
        ],
    }
    report = evaluate_decision(evidence, evidence_path=tmp_path / "evidence.json")
    assert report.decision == WAITING
    assert "MISSING_UINPUT_RC_EFFECT_THROTTLE_LOW" in report.reasons


def test_decision_report_waits_without_uinput_aux_arm_status(tmp_path):
    evidence = {
        "status": PASS,
        "steps": [
            {"phase": "P2-websocket", "status": PASS},
            {"phase": "P2-msp-readonly", "status": PASS},
            {"phase": "P2-fc-status-readonly", "status": PASS},
            {"phase": "P2-mode-ranges-readonly", "status": PASS},
            {"phase": "P0-isolation", "status": PASS},
            {"phase": "P1-install-discovery", "status": PASS},
            {"phase": "P1-client-executable", "status": PASS},
            {"phase": "P3-capture", "status": PASS},
            {"phase": "P4-input-readiness", "status": PASS},
            {"phase": "P4-input-mapping", "status": PASS},
            {"phase": "P4-rc-channels-visual", "status": PASS},
            {"phase": "P5-yaw-live", "status": PASS},
            {"phase": "P5-pitch-live", "status": PASS},
            {"phase": "P5-uinput-rc-effect-throttle-low", "status": PASS},
            {"phase": "P5-uinput-status-effect-throttle-clear", "status": PASS},
            {"phase": "P5-uinput-arm-button-effect", "status": PASS},
            {"phase": "P5-uinput-rc-effect-aux1-high", "status": PASS},
            {"phase": "P5-rc-loopback", "status": PASS},
            {"phase": "P5-persistent-uinput-live-response", "status": PASS},
            {"phase": "P5-response-acceptance", "status": PASS},
            {"phase": "P6-synthetic-e2e-dry-run", "status": PASS},
            {"phase": "P6-synthetic-log-check", "status": PASS},
            {"phase": "P6-tracking-pid-dry-run", "status": PASS},
            {"phase": "P6-tracking-log-check", "status": PASS},
            {"phase": "P6-tracking-live", "status": PASS},
            {"phase": "P6-tracking-acceptance", "status": PASS},
        ],
    }
    report = evaluate_decision(evidence, evidence_path=tmp_path / "evidence.json")
    assert report.decision == WAITING
    assert "MISSING_UINPUT_AUX1_ARM_STATUS" in report.reasons
    assert "MISSING_UINPUT_ARM_BUTTON_EFFECT" not in report.reasons


def test_decision_report_finds_latest_manifest(tmp_path):
    name_newer_but_stale = tmp_path / "20260705-999999-stale-live-manifest.json"
    name_older_but_fresh = tmp_path / "20260705-000000-fresh-live-manifest.json"
    name_newer_but_stale.write_text("{}", encoding="utf-8")
    name_older_but_fresh.write_text("{}", encoding="utf-8")
    os.utime(name_newer_but_stale, ns=(1_000_000_000, 1_000_000_000))
    os.utime(name_older_but_fresh, ns=(2_000_000_000, 2_000_000_000))

    assert latest_manifest_report(tmp_path) == name_older_but_fresh


def test_decision_report_prefers_live_manifest_over_fresher_suite(tmp_path):
    stale_manifest = tmp_path / "20260705-999999-stale-live-manifest.json"
    fresh_suite = tmp_path / "20260705-000000-fresh-suite.json"
    stale_manifest.write_text(json.dumps({
        "status": WAITING,
        "steps": [{"phase": "P2-websocket", "status": WAITING}],
    }), encoding="utf-8")
    fresh_suite.write_text(json.dumps({
        "status": WAITING,
        "steps": [{"phase": "P2-websocket", "status": PASS}],
    }), encoding="utf-8")
    os.utime(stale_manifest, ns=(1_000_000_000, 1_000_000_000))
    os.utime(fresh_suite, ns=(2_000_000_000, 2_000_000_000))

    evidence_path, evidence = choose_evidence(tmp_path, explicit=None)
    assert evidence_path == stale_manifest
    assert evidence["steps"][0]["status"] == WAITING


def test_decision_report_detects_live_manifest_source_kind(tmp_path):
    assert evidence_source_kind(promotion_manifest(), tmp_path / "evidence.json") == "live_manifest"
    assert evidence_source_kind({"status": PASS}, tmp_path / "20260707-pr0p-suite.json") == "suite"
    assert evidence_source_kind({"status": PASS}, tmp_path / "20260707-live-manifest.json") == "live_manifest"


def test_install_probe_discovers_linux_updater_link():
    html = """
    <a href="https:&#x2F;&#x2F;example.test&#x2F;build-current&#x2F;updater.exe">windows 0.9.9</a>
    <a href="https:&#x2F;&#x2F;example.test&#x2F;build-linux-current&#x2F;updater">linux 0.9.9</a>
    """
    result = discover_linux_updater(html, base_url="https://pr0p.dev/download")
    assert result is not None
    assert result["url"] == "https://example.test/build-linux-current/updater"
    assert result["version"] == "0.9.9"


def test_install_probe_allowed_root_can_be_monkeypatched(monkeypatch, tmp_path):
    root = tmp_path / "install"
    monkeypatch.setattr("pr0p_install_probe.ALLOWED_INSTALL_ROOTS", (root,))
    assert is_allowed_install_root(root)
    assert is_allowed_install_root(root / "child")
    assert not is_allowed_install_root(tmp_path / "other")


def test_install_probe_discovery_waits_without_download(monkeypatch, tmp_path):
    root = tmp_path / "install"
    monkeypatch.setattr("pr0p_install_probe.ALLOWED_INSTALL_ROOTS", (root,))

    def fake_request(url, *, method="GET", timeout=20.0):
        if method == "HEAD":
            return b"", {"content-length": "1234"}, url
        return (
            b'<a href="https://example.test/build-linux-current/updater">linux 0.9.9</a>',
            {"content-type": "text/html"},
            "https://pr0p.dev/download",
        )

    monkeypatch.setattr("pr0p_install_probe.request_url", fake_request)
    result = run_install_probe(
        install_root=root,
        download_page="https://pr0p.dev/download",
        download=False,
        force=False,
        timeout=1.0,
        run_id="unit",
    )
    assert result.status == WAITING
    assert result.metrics["candidate"]["version"] == "0.9.9"
    assert result.metrics["download_requested"] is False


def test_install_probe_download_writes_manifest_metrics(monkeypatch, tmp_path):
    root = tmp_path / "install"
    monkeypatch.setattr("pr0p_install_probe.ALLOWED_INSTALL_ROOTS", (root,))

    def fake_request(url, *, method="GET", timeout=20.0):
        if method == "HEAD":
            return b"", {"content-length": "4"}, url
        return (
            b'<a href="https://example.test/build-linux-current/updater">linux 0.9.9</a>',
            {"content-type": "text/html"},
            "https://pr0p.dev/download",
        )

    def fake_download(url, destination, *, timeout, force):
        destination.parent.mkdir(parents=True, exist_ok=True)
        destination.write_bytes(b"demo")
        destination.chmod(0o755)
        return {
            "path": str(destination),
            "downloaded": True,
            "size_bytes": 4,
            "sha256": "fake",
            "mode": "0o755",
        }

    monkeypatch.setattr("pr0p_install_probe.request_url", fake_request)
    monkeypatch.setattr("pr0p_install_probe.download_file", fake_download)
    monkeypatch.setattr("pr0p_install_probe.file_type", lambda _path: "fake executable")
    result = run_install_probe(
        install_root=root,
        download_page="https://pr0p.dev/download",
        download=True,
        force=False,
        timeout=1.0,
        run_id="unit",
    )
    assert result.status == PASS
    assert result.metrics["download"]["file_type"] == "fake executable"
    assert (root / "updater").exists()


def test_install_probe_passes_when_updater_already_present(monkeypatch, tmp_path):
    root = tmp_path / "install"
    root.mkdir()
    updater = root / "updater"
    updater.write_bytes(b"demo")
    updater.chmod(0o755)
    monkeypatch.setattr("pr0p_install_probe.ALLOWED_INSTALL_ROOTS", (root,))

    def fake_request(url, *, method="GET", timeout=20.0):
        if method == "HEAD":
            return b"", {"content-length": "4"}, url
        return (
            b'<a href="https://example.test/build-linux-current/updater">linux 0.9.9</a>',
            {"content-type": "text/html"},
            "https://pr0p.dev/download",
        )

    monkeypatch.setattr("pr0p_install_probe.request_url", fake_request)
    monkeypatch.setattr("pr0p_install_probe.file_type", lambda _path: "fake executable")
    result = run_install_probe(
        install_root=root,
        download_page="https://pr0p.dev/download",
        download=False,
        force=False,
        timeout=1.0,
        run_id="unit",
    )
    assert result.status == PASS
    assert result.metrics["download"]["reason"] == "already present in install root"
    assert result.metrics["install_scan"]["executables"][0]["name"] == "updater"


def test_install_probe_rejects_non_isolated_root(tmp_path):
    result = run_install_probe(
        install_root=tmp_path / "not-allowed",
        download_page="https://pr0p.dev/download",
        download=False,
        force=False,
        timeout=1.0,
        run_id="unit",
    )
    assert result.status == FAIL
    assert "REFUSE_NON_ISOLATED_INSTALL_ROOT" in result.notes


def test_install_probe_scans_client_candidates(tmp_path):
    root = tmp_path / "install"
    root.mkdir()
    updater = root / "updater"
    updater.write_bytes(b"demo")
    updater.chmod(0o755)
    client = root / "pr0p.x86_64"
    client.write_bytes(b"demo")
    client.chmod(0o755)
    scan = scan_install_root(root)
    assert [item["name"] for item in scan["executables"]] == ["pr0p.x86_64", "updater"]
    assert scan["client_candidates"][0]["name"] == "pr0p.x86_64"


def test_client_probe_waits_when_only_updater_is_present(monkeypatch, tmp_path):
    root = tmp_path / "install"
    root.mkdir()
    updater = root / "updater"
    updater.write_bytes(b"demo")
    updater.chmod(0o755)
    monkeypatch.setattr("pr0p_install_probe.ALLOWED_INSTALL_ROOTS", (root,))

    result = run_client_probe(
        install_root=root,
        max_depth=3,
        run_id="unit",
    )
    assert result.status == WAITING
    assert result.metrics["selected_client"] is None
    assert result.metrics["install_scan"]["executables"][0]["name"] == "updater"


def test_client_probe_finds_recursive_pr0p_executable(monkeypatch, tmp_path):
    root = tmp_path / "install"
    client_dir = root / "pr0p"
    client_dir.mkdir(parents=True)
    client = client_dir / "pr0p.x86_64"
    client.write_bytes(b"demo")
    client.chmod(0o755)
    monkeypatch.setattr("pr0p_install_probe.ALLOWED_INSTALL_ROOTS", (root,))

    candidates = find_client_candidates(root, max_depth=3)
    result = run_client_probe(
        install_root=root,
        max_depth=3,
        run_id="unit",
    )
    assert candidates[0]["relative_path"] == "pr0p/pr0p.x86_64"
    assert result.status == PASS
    assert result.metrics["selected_client"]["path"] == str(client)


def test_isolation_check_allows_shared_logging_helper(tmp_path):
    tool = tmp_path / "tool.py"
    tool.write_text("from sitl_log import JsonlLogger\nprint(JsonlLogger)\n", encoding="utf-8")
    result = run_isolation_check(roots=[tmp_path])
    assert result.status == PASS
    assert result.metrics["file_count"] == 1
    assert "GAZEBO_INDEPENDENCE_VERIFIED" in result.notes


def test_isolation_check_allows_rule_strings_in_audit_files(tmp_path):
    audit = tmp_path / "isolation_audit.py"
    audit.write_text(
        "pattern = 'tools/run_gazebo_betaflight.sh'\n",
        encoding="utf-8",
    )
    result = run_isolation_check(roots=[tmp_path])
    assert result.status == PASS
    assert result.metrics["violations"] == []
    assert "GAZEBO_INDEPENDENCE_VERIFIED" in result.notes


def test_isolation_check_default_pr0p_roots_stay_gazebo_independent():
    result = run_isolation_check(roots=list(DEFAULT_ISOLATION_ROOTS))

    assert result.status == PASS
    assert result.metrics["file_count"] > 0
    assert result.metrics["violations"] == []
    assert "GAZEBO_INDEPENDENCE_VERIFIED" in result.notes


def test_isolation_check_rejects_forbidden_gazebo_import(tmp_path):
    tool = tmp_path / "tool.py"
    tool.write_text("from gazebo_motor_moment_probe import main\n", encoding="utf-8")
    result = run_isolation_check(roots=[tmp_path])
    assert result.status == FAIL
    assert result.metrics["violations"][0]["kind"] == "forbidden_import"
    assert "GAZEBO_COUPLING_DETECTED" in result.notes


def test_isolation_check_rejects_forbidden_launcher_string(tmp_path):
    tool = tmp_path / "tool.py"
    tool.write_text("cmd = 'tools/run_gazebo_betaflight.sh'\n", encoding="utf-8")
    result = run_isolation_check(roots=[tmp_path])
    assert result.status == FAIL
    assert result.metrics["violations"][0]["kind"] == "forbidden_string"
