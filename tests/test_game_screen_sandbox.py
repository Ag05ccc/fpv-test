import json
import os
import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
SANDBOX = ROOT / "experiments" / "game_screen_sandbox"
if str(SANDBOX) not in sys.path:
    sys.path.insert(0, str(SANDBOX))
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import phase_runner as phase_runner_module  # noqa: E402
import sandbox_acceptance_runner as acceptance_runner_module  # noqa: E402
import external_window_preflight as external_window_preflight_module  # noqa: E402
import external_follow_session as external_follow_session_module  # noqa: E402
import external_dry_run_sequence as external_dry_run_sequence_module  # noqa: E402
import external_live_input_readiness as external_live_input_readiness_module  # noqa: E402
import external_live_follow_sequence as external_live_follow_sequence_module  # noqa: E402
import rc_binding_assistant as rc_binding_assistant_module  # noqa: E402
import sandbox_status_report as sandbox_status_report_module  # noqa: E402
import sandbox_resume_runner as sandbox_resume_runner_module  # noqa: E402
import external_target_acceptance as external_target_acceptance_module  # noqa: E402
import external_operator_preflight as external_operator_preflight_module  # noqa: E402
import external_candidate_action_runner as external_candidate_action_runner_module  # noqa: E402
import bbox_tool as bbox_tool_module  # noqa: E402
import screen_tracking_loop as screen_tracking_loop_module  # noqa: E402
import simple_window_smoke as simple_window_smoke_module  # noqa: E402
from isolation_audit import (  # noqa: E402
    PASS as AUDIT_PASS,
    audit_process_delta,
    audit_static_imports,
)
from decision_report import (  # noqa: E402
    REJECT,
    SIMPLE_SANDBOX_READY,
    build_markdown as build_decision_markdown,
    choose_evidence as choose_decision_evidence,
    evaluate_decision,
    latest_phase_report_with_phase,
    phase_statuses as decision_phase_statuses,
)
from external_window_preflight import (  # noqa: E402
    EXTERNAL_WINDOW_READY_DRY,
    EXTERNAL_WINDOW_READY_LIVE_INPUT,
    decide_preflight as decide_external_window_preflight,
    run_preflight as run_external_window_preflight,
)
from external_follow_session import (  # noqa: E402
    EXTERNAL_FOLLOW_DRY_RUN_READY,
    EXTERNAL_FOLLOW_LIVE_RUN_COMPLETE,
    ExternalFollowSessionReport,
    evaluate_session_metrics,
    run_external_follow_session,
)
from external_dry_run_sequence import (  # noqa: E402
    EXTERNAL_DRY_RUN_READY,
    ExternalDryRunSequenceReport,
    NOT_RUN,
    READY_FOR_OPTIONAL_LIVE_INPUT,
    WAITING_FOR_DRY_RUN,
    pre_sequence_reasons,
    run_sequence as run_external_dry_run_sequence,
)
from external_live_input_readiness import (  # noqa: E402
    ACK_LIVE_INPUT_REQUIRED,
    EXTERNAL_LIVE_INPUT_READY,
    ExternalLiveInputReadinessReport,
    run_live_input_readiness,
)
from external_live_follow_sequence import (  # noqa: E402
    EXTERNAL_LIVE_FOLLOW_COMPLETE,
    run_live_follow_sequence,
)
from external_window_preflight import ExternalWindowPreflightReport  # noqa: E402
from rc_binding_assistant import (  # noqa: E402
    build_binding_steps,
    run_binding_sequence,
)
from sandbox_status_report import (  # noqa: E402
    READY as SANDBOX_STATUS_READY,
    build_status_report,
    matching_report,
)
from sandbox_resume_runner import (  # noqa: E402
    BLOCKED,
    PLANNED,
    SANDBOX_RESUME_PROGRESS,
    SANDBOX_RESUME_READY,
    SKIPPED,
    CommandRunResult,
    run_resume,
)
from external_target_acceptance import (  # noqa: E402
    EXTERNAL_TARGET_READY,
    run_external_acceptance,
)
from external_operator_preflight import (  # noqa: E402
    OPERATOR_PREFLIGHT_READY,
    run_operator_preflight,
)
from external_candidate_action_runner import (  # noqa: E402
    OPERATOR_CANDIDATE_ACTION_PROGRESS,
    run_candidate_action,
)
from game_dynamics_loop import (  # noqa: E402
    GameDynamicsConfig,
    GameHandoffDynamicsConfig,
    GameRangeDynamicsConfig,
    evaluate_handoff_metrics,
    evaluate_metrics as evaluate_game_dynamics_metrics,
    evaluate_range_metrics,
    run_game_dynamics_loop,
    run_game_handoff_dynamics_loop,
    run_game_range_dynamics_loop,
)
from bbox_tool import (  # noqa: E402
    bbox_inside_frame as bbox_tool_inside_frame,
    draw_bbox,
    prepare_bbox_artifacts,
)
from capture_window import (  # noqa: E402
    CaptureFrame,
    FFmpegX11RegionFrameSource,
    SyntheticFrameSource,
    is_nonblank,
    make_region_source,
    parse_region,
    sample_source,
    save_first_frame,
)
from phase_runner import (  # noqa: E402
    FAIL,
    PASS,
    WAITING,
    PhaseRunReport,
    PhaseResult,
    build_markdown_report,
    check_s1_real_capture,
    check_s2_real_tracker,
    check_s3_real_input,
    check_s4_real_closed_loop,
    check_s5_real_latency,
    check_s6_real_approach,
    check_s7_real_combined,
    check_s8_moving_target,
    check_s9_game_dynamics,
    check_s10_range_dynamics,
    check_s11_handoff_dynamics,
    check_s12_adapter_dynamics,
    check_s13_binding_dry,
    check_s14_objective_loop,
    load_tracker_bbox_file,
    run_synthetic_gates,
    write_reports,
)
from latency_probe import (  # noqa: E402
    estimate_frame_shift,
    frame_diff,
    measure_axis_response_sweep,
    measure_visual_latency,
    parse_axis_shift_expectations,
)
from screen_tracking_loop import LoopConfig, run_loop  # noqa: E402
from simple_game_adapter_loop import (  # noqa: E402
    AdapterLoopConfig,
    evaluate_metrics as evaluate_adapter_metrics,
    run_adapter_closed_loop,
)
from simple_game_objective_loop import (  # noqa: E402
    ObjectiveLoopConfig,
    evaluate_objective_metrics,
    run_objective_closed_loop,
)
from simple_target_game import (  # noqa: E402
    GameCommand,
    GameConfig,
    bbox_intersects_frame,
    initial_state,
    render_frame,
    run_game,
    step_state,
    target_screen_bbox,
)
from simple_window_smoke import game_command, static_target_bbox, wait_for_window  # noqa: E402
from virtual_input import AxisCommand, DryRunInputAdapter, UInputProbe, command_from_channels  # noqa: E402
from x11_window import (  # noqa: E402
    X11Window,
    find_window_by_title,
    parse_size,
    parse_xwininfo_tree,
    visible_windows,
)
from kenet.pipeline import PipelineConfig  # noqa: E402
from kenet.tracker import TrackResult  # noqa: E402


class FixedTracker:
    def __init__(self, result):
        self.result = result
        self.initialized = False
        self.init_bbox = None

    @property
    def is_initialized(self):
        return self.initialized

    def init(self, _frame, bbox):
        self.initialized = True
        self.init_bbox = bbox

    def update(self, _frame):
        return self.result


class SequenceTracker:
    def __init__(self, results):
        self.results = list(results)
        self.initialized = False
        self.index = 0

    @property
    def is_initialized(self):
        return self.initialized

    def init(self, _frame, _bbox):
        self.initialized = True

    def update(self, _frame):
        result = self.results[min(self.index, len(self.results) - 1)]
        self.index += 1
        return result


class StepResponseSource:
    def __init__(self, *, width=32, height=24, fps=10, step_index=4):
        self.width = width
        self.height = height
        self.fps = fps
        self.step_index = step_index

    def frames(self, duration_s=None):
        total = int((duration_s or 1.0) * self.fps)
        start = 1000.0
        for index in range(total):
            frame = np.zeros((self.height, self.width, 3), dtype=np.uint8)
            if index >= self.step_index:
                frame[:, :] = 255
            yield CaptureFrame(
                frame=frame,
                timestamp=start + index / self.fps,
                index=index,
                source="step",
            )


class ShiftResponseSource:
    def __init__(self, *, width=64, height=48, fps=10, step_index=4, shift=(5, 0)):
        self.width = width
        self.height = height
        self.fps = fps
        self.step_index = step_index
        self.shift = shift
        self.base = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        self.base[:, :, 0] = np.linspace(20, 120, self.width, dtype=np.uint8)
        self.base[:, :, 1] = np.linspace(30, 160, self.height, dtype=np.uint8)[:, None]
        self.base[7:18, 9:24] = (220, 40, 90)
        self.base[27:38, 36:55] = (40, 230, 180)

    def frames(self, duration_s=None):
        total = int((duration_s or 1.0) * self.fps)
        start = 2000.0
        dx, dy = self.shift
        shifted = np.roll(self.base, shift=(dy, dx), axis=(0, 1))
        for index in range(total):
            frame = shifted.copy() if index >= self.step_index else self.base.copy()
            yield CaptureFrame(
                frame=frame,
                timestamp=start + index / self.fps,
                index=index,
                source="shift",
            )


def test_synthetic_capture_is_nonblank_and_sized():
    source = SyntheticFrameSource(width=320, height=240, fps=10)
    stats = sample_source(source, duration_s=0.5)
    assert stats.frames == 5
    assert stats.width == 320
    assert stats.height == 240
    assert stats.nonblank


def test_save_first_frame_writes_image(tmp_path):
    source = SyntheticFrameSource(width=64, height=48, fps=5)
    output = save_first_frame(source, tmp_path / "frame.png", duration_s=0.2)
    assert output.exists()
    assert output.stat().st_size > 0


def test_synthetic_source_reports_initial_target_bbox():
    source = SyntheticFrameSource(width=640, height=480, target_size=80)
    assert source.target_bbox_at(0.0) == (280, 248, 80, 80)


def test_blank_frame_detector_rejects_uniform_frame():
    assert not is_nonblank(np.zeros((16, 16, 3), dtype=np.uint8))
    frame = np.zeros((16, 16, 3), dtype=np.uint8)
    frame[4:8, 4:8] = 255
    assert is_nonblank(frame)


def test_parse_region_contract():
    assert parse_region("1,2,640,480") == {
        "left": 1,
        "top": 2,
        "width": 640,
        "height": 480,
    }


def test_bbox_tool_validates_frame_bounds():
    assert bbox_tool_inside_frame((1, 2, 10, 20), width=100, height=100)
    assert not bbox_tool_inside_frame((95, 2, 10, 20), width=100, height=100)
    assert not bbox_tool_inside_frame((0, 0, 0, 20), width=100, height=100)


def test_bbox_tool_waits_without_bbox_and_writes_frame(tmp_path):
    source = SyntheticFrameSource(width=80, height=60, fps=5)
    result = prepare_bbox_artifacts(
        source,
        bbox=None,
        frame_path=tmp_path / "frame.png",
        duration_s=0.2,
    )
    assert result.status == WAITING
    assert Path(result.frame_path).exists()
    assert result.overlay_path is None
    assert result.frame_width == 80
    assert result.frame_height == 60


def test_bbox_tool_interactive_select_writes_bbox_artifacts(tmp_path):
    source = SyntheticFrameSource(width=80, height=60, fps=5)
    result = prepare_bbox_artifacts(
        source,
        bbox=None,
        frame_path=tmp_path / "frame.png",
        duration_s=0.2,
        interactive_select=True,
        select_fn=lambda _frame: (5, 6, 20, 15),
    )
    assert result.status == PASS
    assert result.bbox == (5, 6, 20, 15)
    assert result.overlay_path is not None
    assert Path(result.overlay_path).exists()
    assert "bbox selected interactively" in result.notes


def test_bbox_tool_interactive_select_cancel_waits(tmp_path):
    source = SyntheticFrameSource(width=80, height=60, fps=5)
    result = prepare_bbox_artifacts(
        source,
        bbox=None,
        frame_path=tmp_path / "frame.png",
        duration_s=0.2,
        interactive_select=True,
        select_fn=lambda _frame: None,
    )
    assert result.status == WAITING
    assert result.bbox is None
    assert result.overlay_path is None
    assert "cancelled" in " ".join(result.notes)


def test_bbox_tool_writes_overlay_for_valid_bbox(tmp_path):
    source = SyntheticFrameSource(width=80, height=60, fps=5)
    result = prepare_bbox_artifacts(
        source,
        bbox=(10, 10, 20, 20),
        frame_path=tmp_path / "frame.png",
        duration_s=0.2,
    )
    assert result.status == PASS
    assert result.overlay_path is not None
    assert Path(result.overlay_path).exists()


def test_bbox_tool_missing_window_returns_waiting_report(monkeypatch, tmp_path):
    def raise_missing_window(_args):
        raise RuntimeError("no visible X11 window matched title: pr0p")

    monkeypatch.setattr(bbox_tool_module, "make_source", raise_missing_window)
    report_path = tmp_path / "bbox-report.json"
    code = bbox_tool_module.main([
        "--window-title",
        "pr0p",
        "--frame-path",
        str(tmp_path / "frame.png"),
        "--report-path",
        str(report_path),
    ])
    data = json.loads(report_path.read_text(encoding="utf-8"))
    assert code == 2
    assert data["status"] == WAITING
    assert data["frame_width"] == 0
    assert data["frame_height"] == 0
    assert not (tmp_path / "frame.png").exists()
    assert "open the window" in data["notes"][0]


def test_phase_runner_loads_tracker_bbox_file(tmp_path):
    path = tmp_path / "bbox.json"
    path.write_text('{"bbox": [1, 2, 30, 40]}\n', encoding="utf-8")
    assert load_tracker_bbox_file(path) == (1, 2, 30, 40)


def test_bbox_tool_draws_overlay_pixels():
    frame = np.zeros((40, 40, 3), dtype=np.uint8)
    overlay = draw_bbox(frame, (5, 5, 20, 20))
    assert overlay.shape == frame.shape
    assert int(overlay.sum()) > 0


def test_simple_target_game_renders_nonblank_frame_and_bbox():
    config = GameConfig(width=320, height=240)
    state = initial_state(config)
    frame = render_frame(state, config)
    bbox = target_screen_bbox(state, config)
    assert frame.shape == (240, 320, 3)
    assert is_nonblank(frame)
    assert bbox_intersects_frame(bbox, config)
    assert bbox[2] == config.target_size
    assert bbox[3] == config.target_size


def test_simple_target_game_camera_moves_with_command():
    config = GameConfig(width=320, height=240)
    state = initial_state(config)
    moved = step_state(state, GameCommand(yaw=1.0, pitch=0.5), 1.0, config)
    assert moved.camera_x > state.camera_x
    assert moved.camera_y < state.camera_y
    assert moved.command == GameCommand(yaw=1.0, pitch=0.5)


def test_simple_target_game_headless_smoke_writes_frame(tmp_path):
    frame_path = tmp_path / "simple-game.png"
    report = run_game(
        GameConfig(width=320, height=240),
        duration_s=0.5,
        fps=10.0,
        show=False,
        auto_input="orbit",
        save_frame=frame_path,
    )
    assert frame_path.exists()
    assert report["frames"] == 5
    assert report["show"] is False
    assert report["auto_input"] == "orbit"
    assert report["first_bbox"] is not None
    assert report["target_center_travel_px"] is not None


def test_simple_target_game_static_target_keeps_bbox_stable(tmp_path):
    report = run_game(
        GameConfig(width=320, height=240, target_motion_radius_x=0.0, target_motion_radius_y=0.0),
        duration_s=0.5,
        fps=10.0,
        show=False,
        auto_input="neutral",
        save_frame=tmp_path / "static.png",
    )
    assert report["static_target"] is True
    assert report["target_motion_radius_x"] == 0.0
    assert report["target_motion_radius_y"] == 0.0
    assert report["first_bbox"] == report["last_bbox"]
    assert report["target_center_travel_px"] == 0.0


def test_simple_window_smoke_static_bbox_matches_default_target():
    assert static_target_bbox() == (275, 195, 90, 90)


def test_simple_window_smoke_builds_static_game_command(tmp_path):
    command = game_command(
        duration_s=8.0,
        fps=20.0,
        width=640,
        height=480,
        report_path=tmp_path / "game.json",
    )
    assert "--show" in command
    assert "--static-target" in command
    assert "--report-path" in command
    assert str(tmp_path / "game.json") in command


def test_simple_window_smoke_wait_prefers_content_window(monkeypatch):
    windows = parse_xwininfo_tree("""
     0x1 "Kenet Simple Target Game": ("cv2" "Cv2")  668x546+10+20  +10+20
     0x2 "Kenet Simple Target Game": ("cv2" "Cv2")  640x480+24+69  +24+69
    """)
    monkeypatch.setattr(simple_window_smoke_module, "list_x11_windows", lambda: windows)
    region = wait_for_window(
        title="Kenet Simple Target Game",
        preferred_size=(640, 480),
        timeout_s=0.1,
        poll_s=0.01,
    )
    assert region == {"left": 24, "top": 69, "width": 640, "height": 480}


def test_latency_probe_detects_step_response():
    source = StepResponseSource(fps=10, step_index=4)
    adapter = DryRunInputAdapter()
    result = measure_visual_latency(
        source,
        adapter,
        command=AxisCommand(yaw=0.1),
        fps=10,
        pre_duration_s=0.3,
        post_duration_s=0.5,
        min_diff=10.0,
    )
    assert result.status == PASS
    assert result.latency_ms is not None
    assert result.max_diff >= 250
    assert adapter.commands[-1][1].is_neutral()


def test_latency_probe_waits_without_visual_response():
    source = StepResponseSource(fps=10, step_index=999)
    adapter = DryRunInputAdapter()
    result = measure_visual_latency(
        source,
        adapter,
        command=AxisCommand(yaw=0.1),
        fps=10,
        pre_duration_s=0.3,
        post_duration_s=0.3,
        min_diff=10.0,
    )
    assert result.status == WAITING
    assert result.latency_ms is None


def test_latency_probe_estimates_signed_frame_shift():
    source = ShiftResponseSource(fps=10, step_index=4, shift=(5, -2))
    adapter = DryRunInputAdapter()
    result = measure_visual_latency(
        source,
        adapter,
        command=AxisCommand(yaw=0.1),
        fps=10,
        pre_duration_s=0.3,
        post_duration_s=0.5,
        min_diff=3.0,
    )
    assert result.status == PASS
    assert result.response_shift_x_px is not None
    assert result.response_shift_y_px is not None
    assert result.response_shift_x_px > 4.0
    assert result.response_shift_y_px < -1.0
    direct_shift = estimate_frame_shift(source.base, np.roll(source.base, shift=(-2, 5), axis=(0, 1)))
    assert direct_shift[0] > 4.0
    assert direct_shift[1] < -1.0


def test_axis_response_sweep_reports_each_selected_axis():
    source = StepResponseSource(fps=10, step_index=4)
    adapter = DryRunInputAdapter()
    result = measure_axis_response_sweep(
        source,
        adapter,
        axes=("yaw", "pitch"),
        axis_value=0.1,
        fps=10,
        pre_duration_s=0.3,
        post_duration_s=0.5,
        min_diff=10.0,
    )
    assert result.status == PASS
    assert result.passed_axes == ("yaw", "pitch")
    assert result.as_dict()["axis_statuses"] == {"yaw": PASS, "pitch": PASS}
    assert adapter.commands[-1][1].is_neutral()


def test_axis_response_sweep_can_require_expected_signed_direction():
    source = ShiftResponseSource(fps=10, step_index=4, shift=(5, 0))
    adapter = DryRunInputAdapter()
    expected = parse_axis_shift_expectations("yaw:+x")
    result = measure_axis_response_sweep(
        source,
        adapter,
        axes=("yaw",),
        axis_value=0.1,
        fps=10,
        pre_duration_s=0.3,
        post_duration_s=0.5,
        min_diff=3.0,
        expected_shifts=expected,
        min_shift_px=1.0,
    )
    assert result.status == PASS
    assert result.as_dict()["expected_shifts"] == {"yaw": "+x"}
    yaw_result = result.as_dict()["axis_results"]["yaw"]
    assert yaw_result["direction_status"] == PASS
    assert yaw_result["response_shift_x_px"] > 4.0


def test_axis_response_sweep_fails_on_opposite_signed_direction():
    source = ShiftResponseSource(fps=10, step_index=4, shift=(5, 0))
    adapter = DryRunInputAdapter()
    result = measure_axis_response_sweep(
        source,
        adapter,
        axes=("yaw",),
        axis_value=0.1,
        fps=10,
        pre_duration_s=0.3,
        post_duration_s=0.5,
        min_diff=3.0,
        expected_shifts={"yaw": ("x", -1)},
        min_shift_px=1.0,
    )
    assert result.status == FAIL
    assert result.failed_axes == ("yaw",)
    yaw_result = result.as_dict()["axis_results"]["yaw"]
    assert yaw_result["direction_status"] == FAIL
    assert any("opposite expected" in note for note in yaw_result["notes"])


def test_axis_response_sweep_waits_when_axis_has_no_visual_response():
    source = StepResponseSource(fps=10, step_index=999)
    adapter = DryRunInputAdapter()
    result = measure_axis_response_sweep(
        source,
        adapter,
        axes=("yaw",),
        axis_value=0.1,
        fps=10,
        pre_duration_s=0.3,
        post_duration_s=0.3,
        min_diff=10.0,
    )
    assert result.status == WAITING
    assert result.waiting_axes == ("yaw",)
    assert "Controls -> RC Channels" in " ".join(result.notes)


def test_rc_binding_steps_cover_positive_negative_and_neutral():
    steps = build_binding_steps(
        axes=("yaw",),
        axis_value=0.6,
        hold_seconds=0.1,
        neutral_seconds=0.0,
        cycles=1,
        include_negative=True,
    )
    assert [step.phase for step in steps] == [
        "neutral-before-1",
        "positive-1",
        "neutral-middle-1",
        "negative-1",
        "neutral-after-1",
    ]
    assert steps[1].command.yaw == 0.6
    assert steps[3].command.yaw == -0.6
    assert steps[-1].command.is_neutral()


def test_rc_binding_assistant_dry_run_records_sequence():
    report = run_binding_sequence(
        run_id="unit-rc-binding-dry",
        axes=("yaw", "pitch"),
        axis_value=0.5,
        hold_seconds=0.0,
        neutral_seconds=0.0,
        cycles=1,
        include_negative=False,
        use_uinput=False,
        ack_live_input=False,
        countdown_seconds=0.0,
        sleep_fn=lambda _seconds: None,
    )
    assert report.status == PASS
    assert report.metrics["adapter"] == "DryRunInputAdapter"
    assert report.metrics["real_input"] is False
    assert report.metrics["command_count"] == len(report.steps) + 1
    assert "dry-run only" in " ".join(report.notes)


def test_rc_binding_assistant_records_window_and_bbox_context():
    report = run_binding_sequence(
        run_id="unit-rc-binding-context",
        axes=("yaw",),
        axis_value=0.5,
        hold_seconds=0.0,
        neutral_seconds=0.0,
        cycles=1,
        include_negative=False,
        use_uinput=False,
        ack_live_input=False,
        countdown_seconds=0.0,
        window_title="pr0p",
        tracker_bbox=(10, 20, 80, 80),
        sleep_fn=lambda _seconds: None,
    )
    assert report.status == PASS
    assert report.metrics["window_title"] == "pr0p"
    assert report.metrics["tracker_bbox"] == [10, 20, 80, 80]


def test_rc_binding_assistant_waits_without_live_ack():
    report = run_binding_sequence(
        run_id="unit-rc-binding-no-ack",
        axes=("yaw",),
        axis_value=0.5,
        hold_seconds=0.0,
        neutral_seconds=0.0,
        cycles=1,
        include_negative=True,
        use_uinput=True,
        ack_live_input=False,
        countdown_seconds=0.0,
        sleep_fn=lambda _seconds: None,
    )
    assert report.status == WAITING
    assert report.summary == "RC binding live input requires explicit acknowledgement"
    assert report.metrics["command_count"] == 0
    assert "--ack-live-input" in " ".join(report.notes)


def test_rc_binding_assistant_waits_when_uinput_is_unavailable(monkeypatch):
    monkeypatch.setattr(
        rc_binding_assistant_module,
        "probe_uinput_environment",
        lambda: UInputProbe(
            available=False,
            evdev_available=False,
            dev_uinput_exists=False,
            dev_uinput_readable=False,
            dev_uinput_writable=False,
            error="/dev/uinput does not exist",
        ),
    )
    report = run_binding_sequence(
        run_id="unit-rc-binding-no-uinput",
        axes=("yaw",),
        axis_value=0.5,
        hold_seconds=0.0,
        neutral_seconds=0.0,
        cycles=1,
        include_negative=True,
        use_uinput=True,
        ack_live_input=True,
        countdown_seconds=0.0,
        sleep_fn=lambda _seconds: None,
    )
    assert report.status == WAITING
    assert report.summary == "RC binding live input prerequisites are not ready"
    assert report.metrics["uinput_probe"]["available"] is False


def test_frame_diff_requires_same_shape():
    with pytest.raises(ValueError):
        frame_diff(
            np.zeros((10, 10, 3), dtype=np.uint8),
            np.zeros((12, 10, 3), dtype=np.uint8),
        )


def test_ffmpeg_x11_region_source_builds_expected_command():
    region = parse_region("10,20,320,240")
    source = FFmpegX11RegionFrameSource(region, fps=15.0, display=":99", ffmpeg_bin="ffmpeg")
    assert source.frame_bytes == 320 * 240 * 3
    command = source.command(frame_count=3)
    assert command[:5] == ["ffmpeg", "-nostdin", "-hide_banner", "-loglevel", "error"]
    assert "-f" in command
    assert "x11grab" in command
    assert "320x240" in command
    assert ":99+10,20" in command
    assert command[-3:] == ["-frames:v", "3", "pipe:1"]


def test_x11_window_parser_finds_visible_window_region():
    text = """
     0x2400004 "pr0p Simulator": ("pr0p" "Pr0p")  1280x720+66+32  +66+32
     0x460003b "hidden": ()  10x10+-100+-100  +-100+-100
     0x3c00004 "Visual Studio Code": ("code" "code")  1920x1080+1920+0  +1920+0
    """
    windows = parse_xwininfo_tree(text)
    assert len(windows) == 3
    visible = visible_windows(windows)
    assert [window.title for window in visible] == ["pr0p Simulator", "Visual Studio Code"]
    match = find_window_by_title("PR0P", windows)
    assert match is not None
    assert match.region() == {"left": 66, "top": 32, "width": 1280, "height": 720}


def test_x11_window_finder_prefers_largest_matching_window():
    text = """
     0x1 "Game": ("app" "App")  320x240+0+0  +0+0
     0x2 "Game - fullscreen": ("app" "App")  1920x1080+0+0  +0+0
    """
    match = find_window_by_title("game", parse_xwininfo_tree(text))
    assert match is not None
    assert match.window_id == "0x2"


def test_x11_window_finder_can_prefer_content_size():
    text = """
     0x1 "Kenet Simple Target Game": ("cv2" "Cv2")  668x546+10+20  +10+20
     0x2 "Kenet Simple Target Game": ("cv2" "Cv2")  640x480+24+69  +24+69
    """
    match = find_window_by_title(
        "Kenet Simple Target Game",
        parse_xwininfo_tree(text),
        preferred_size=(640, 480),
    )
    assert match is not None
    assert match.window_id == "0x2"
    assert parse_size("640x480") == (640, 480)


def test_make_region_source_rejects_unknown_backend():
    with pytest.raises(ValueError):
        make_region_source(parse_region("0,0,10,10"), fps=10.0, backend="nope")


def test_real_capture_gate_waits_without_region():
    result = check_s1_real_capture(
        None,
        backend="ffmpeg",
        duration_s=0.1,
        hz=5.0,
        min_fps=1.0,
    )
    assert result.status == WAITING
    assert result.phase == "S1-real"
    assert result.metrics["region"] is None


def test_real_capture_gate_waits_when_window_title_is_missing(monkeypatch):
    def fail_resolve(**_kwargs):
        raise RuntimeError("no visible X11 window matched title: pr0p")

    monkeypatch.setattr(phase_runner_module, "resolve_capture_region", fail_resolve)
    result = check_s1_real_capture(
        None,
        backend="ffmpeg",
        duration_s=0.1,
        hz=5.0,
        min_fps=1.0,
        window_title="pr0p",
    )
    assert result.status == WAITING
    assert result.summary == "real screen window capture is waiting for a matching window"


def test_real_tracker_gate_waits_without_bbox():
    result = check_s2_real_tracker(
        parse_region("0,0,320,240"),
        backend="ffmpeg",
        duration_s=0.1,
        hz=5.0,
        min_found_ratio=0.8,
        tracker_bbox=None,
    )
    assert result.status == WAITING
    assert result.phase == "S2-real"
    assert result.summary == "real tracker bbox has not been selected yet"


def test_real_tracker_gate_rejects_bbox_outside_frame(monkeypatch):
    class TinySource:
        def frames(self, _duration_s):
            frame = np.zeros((20, 20, 3), dtype=np.uint8)
            yield type("Frame", (), {
                "frame": frame,
                "timestamp": 0.0,
                "index": 0,
                "source": "tiny",
            })()

    monkeypatch.setattr(phase_runner_module, "resolve_capture_region",
                        lambda **_kwargs: parse_region("0,0,20,20"))
    monkeypatch.setattr(phase_runner_module, "make_region_source",
                        lambda *_args, **_kwargs: TinySource())
    result = check_s2_real_tracker(
        parse_region("0,0,20,20"),
        backend="ffmpeg",
        duration_s=0.1,
        hz=5.0,
        min_found_ratio=0.8,
        tracker_bbox=(15, 15, 10, 10),
    )
    assert result.status == "FAIL"
    assert result.summary == "real tracker bbox is outside the captured frame"


def test_real_input_gate_waits_when_smoke_is_not_requested():
    result = check_s3_real_input(
        run_smoke=False,
        command=AxisCommand(yaw=0.05),
        hold_seconds=0.0,
    )
    assert result.phase == "S3-real"
    assert result.status in {WAITING, "PASS"}
    if result.status == WAITING:
        assert result.summary in {
            "real uinput prerequisites are not ready",
            "real uinput is available but smoke was not requested",
        }


def test_real_loop_gate_waits_without_bbox(tmp_path):
    result = check_s4_real_closed_loop(
        parse_region("0,0,320,240"),
        backend="ffmpeg",
        duration_s=0.1,
        hz=5.0,
        min_found_ratio=0.8,
        tracker_bbox=None,
        log_dir=tmp_path,
        run_id="unit-real-loop",
    )
    assert result.status == WAITING
    assert result.phase == "S4-real"
    assert result.summary == "real closed-loop bbox has not been selected yet"


def test_real_loop_gate_can_run_dry_with_real_source_contract(monkeypatch, tmp_path):
    synthetic = SyntheticFrameSource(width=320, height=240, fps=10)

    monkeypatch.setattr(phase_runner_module, "resolve_capture_region",
                        lambda **_kwargs: parse_region("0,0,320,240"))
    monkeypatch.setattr(phase_runner_module, "make_region_source",
                        lambda *_args, **_kwargs: synthetic)
    result = check_s4_real_closed_loop(
        parse_region("0,0,320,240"),
        backend="ffmpeg",
        duration_s=0.5,
        hz=10.0,
        min_found_ratio=0.8,
        tracker_bbox=synthetic.target_bbox_at(0.0),
        log_dir=tmp_path,
        run_id="unit-real-loop-pass",
    )
    assert result.status == PASS
    assert result.metrics["adapter"] == "DryRunInputAdapter"
    assert result.metrics["real_input"] is False
    assert result.metrics["neutralized"] is True
    assert Path(result.metrics["log_path"]).exists()


def test_live_loop_gate_requires_explicit_ack(tmp_path):
    result = check_s4_real_closed_loop(
        parse_region("0,0,320,240"),
        backend="ffmpeg",
        duration_s=0.1,
        hz=5.0,
        min_found_ratio=0.8,
        tracker_bbox=(10, 10, 20, 20),
        log_dir=tmp_path,
        run_id="unit-live-loop",
        real_input=True,
        ack_live_input=False,
        phase="S4-live",
    )
    assert result.status == WAITING
    assert result.phase == "S4-live"
    assert result.summary == "real input live loop requires explicit acknowledgement"


def test_latency_gate_requires_ack_for_uinput():
    result = check_s5_real_latency(
        parse_region("0,0,320,240"),
        backend="ffmpeg",
        fps=10.0,
        pre_duration_s=0.2,
        post_duration_s=0.2,
        command=AxisCommand(yaw=0.1),
        min_diff=3.0,
        baseline_multiplier=3.0,
        use_uinput=True,
        ack_live_input=False,
    )
    assert result.status == WAITING
    assert result.phase == "S5-real"
    assert result.summary == "latency probe real input requires explicit acknowledgement"


def test_latency_gate_can_run_with_dry_adapter(monkeypatch):
    monkeypatch.setattr(phase_runner_module, "resolve_capture_region",
                        lambda **_kwargs: parse_region("0,0,32,24"))
    monkeypatch.setattr(phase_runner_module, "make_region_source",
                        lambda *_args, **_kwargs: StepResponseSource(fps=10, step_index=4))
    result = check_s5_real_latency(
        parse_region("0,0,32,24"),
        backend="ffmpeg",
        fps=10.0,
        pre_duration_s=0.3,
        post_duration_s=0.5,
        command=AxisCommand(yaw=0.1),
        min_diff=10.0,
        baseline_multiplier=3.0,
        use_uinput=False,
        ack_live_input=False,
    )
    assert result.status == PASS
    assert result.metrics["adapter"] == "DryRunInputAdapter"
    assert result.metrics["latency_ms"] is not None


def test_real_approach_gate_waits_without_bbox(tmp_path):
    result = check_s6_real_approach(
        parse_region("0,0,320,240"),
        backend="ffmpeg",
        duration_s=0.1,
        hz=5.0,
        min_found_ratio=0.8,
        tracker_bbox=None,
        log_dir=tmp_path,
        run_id="unit-approach-waiting",
        desired_target_width=120.0,
        max_pitch_axis=0.5,
    )
    assert result.status == WAITING
    assert result.phase == "S6-real"
    assert result.summary == "approach bbox has not been selected yet"


def test_real_approach_gate_can_run_dry_with_pitch(monkeypatch, tmp_path):
    synthetic = SyntheticFrameSource(width=320, height=240, fps=10, target_size=80)

    monkeypatch.setattr(phase_runner_module, "resolve_capture_region",
                        lambda **_kwargs: parse_region("0,0,320,240"))
    monkeypatch.setattr(phase_runner_module, "make_region_source",
                        lambda *_args, **_kwargs: synthetic)
    result = check_s6_real_approach(
        parse_region("0,0,320,240"),
        backend="ffmpeg",
        duration_s=0.5,
        hz=10.0,
        min_found_ratio=0.8,
        tracker_bbox=synthetic.target_bbox_at(0.0),
        log_dir=tmp_path,
        run_id="unit-approach-pass",
        desired_target_width=120.0,
        max_pitch_axis=0.5,
    )
    assert result.status == PASS
    assert result.metrics["adapter"] == "DryRunInputAdapter"
    assert result.metrics["real_input"] is False
    assert result.metrics["enable_pitch"] is True
    assert result.metrics["initial_target_width"] == 80.0
    assert result.metrics["initial_width_error"] == 40.0
    assert result.metrics["max_abs_pitch_axis"] > 0.0
    assert result.metrics["pitch_bounded"] is True
    assert result.metrics["neutralized"] is True
    assert Path(result.metrics["log_path"]).exists()


def test_real_combined_gate_waits_without_bbox(tmp_path):
    result = check_s7_real_combined(
        parse_region("0,0,320,240"),
        backend="ffmpeg",
        duration_s=0.1,
        hz=5.0,
        min_found_ratio=0.8,
        tracker_bbox=None,
        log_dir=tmp_path,
        run_id="unit-combined-waiting",
        desired_target_width=120.0,
        max_yaw_axis=0.5,
        max_pitch_axis=0.5,
    )
    assert result.status == WAITING
    assert result.phase == "S7-real"
    assert result.summary == "combined-loop bbox has not been selected yet"


def test_real_combined_gate_can_run_dry_with_yaw_and_pitch(monkeypatch, tmp_path):
    synthetic = SyntheticFrameSource(width=320, height=240, fps=10, target_size=80)

    monkeypatch.setattr(phase_runner_module, "resolve_capture_region",
                        lambda **_kwargs: parse_region("0,0,320,240"))
    monkeypatch.setattr(phase_runner_module, "make_region_source",
                        lambda *_args, **_kwargs: synthetic)
    result = check_s7_real_combined(
        parse_region("0,0,320,240"),
        backend="ffmpeg",
        duration_s=0.5,
        hz=10.0,
        min_found_ratio=0.8,
        tracker_bbox=synthetic.target_bbox_at(0.0),
        log_dir=tmp_path,
        run_id="unit-combined-pass",
        desired_target_width=120.0,
        max_yaw_axis=0.5,
        max_pitch_axis=0.5,
    )
    assert result.status == PASS
    assert result.metrics["adapter"] == "DryRunInputAdapter"
    assert result.metrics["real_input"] is False
    assert result.metrics["enable_pitch"] is True
    assert result.metrics["yaw_bounded"] is True
    assert result.metrics["pitch_bounded"] is True
    assert result.metrics["neutralized"] is True
    assert result.metrics["found_ratio"] == 1.0
    assert Path(result.metrics["log_path"]).exists()


def test_moving_target_gate_passes_on_synthetic_motion(tmp_path):
    result = check_s8_moving_target(
        duration_s=1.0,
        hz=10.0,
        min_found_ratio=0.8,
        min_center_motion_px=20.0,
        max_yaw_axis=0.5,
        max_pitch_axis=0.5,
        desired_target_width=120.0,
        log_dir=tmp_path,
        run_id="unit-moving-target",
    )
    assert result.status == PASS
    assert result.phase == "S8"
    assert result.metrics["center_motion_ok"] is True
    assert result.metrics["target_center_span_px"] >= 20.0
    assert result.metrics["yaw_bounded"] is True
    assert result.metrics["pitch_bounded"] is True
    assert result.metrics["neutralized"] is True
    assert result.metrics["real_input"] is False
    assert Path(result.metrics["log_path"]).exists()


def test_game_dynamics_loop_applies_pid_to_camera_and_centers_target(tmp_path):
    result = run_game_dynamics_loop(GameDynamicsConfig(
        duration_s=4.0,
        hz=20.0,
        initial_offset_x=180.0,
        log_dir=tmp_path,
        run_id="unit-game-dynamics",
    ))
    assert result.status == PASS
    assert result.metrics["control_applied_to_game_camera"] is True
    assert result.metrics["found_ratio"] >= 0.9
    assert result.metrics["initial_abs_error_px"] >= 80.0
    assert result.metrics["final_abs_error_px"] <= 35.0
    assert result.metrics["error_reduction_ratio"] >= 0.65
    assert result.metrics["yaw_initial_command_corrective"] is True
    assert result.metrics["yaw_initial_error_px"] > 0.0
    assert result.metrics["yaw_initial_command"] > 0.0
    assert result.metrics["real_input"] is False


def test_game_dynamics_evaluator_fails_without_convergence():
    status, summary, notes = evaluate_game_dynamics_metrics({
        "frames": 10,
        "found_ratio": 1.0,
        "loss_events": 0,
        "initial_abs_error_px": 180.0,
        "final_abs_error_px": 160.0,
        "error_reduction_ratio": 0.1,
        "max_abs_yaw_axis": 0.2,
        "yaw_initial_command_corrective": True,
        "yaw_corrective_command_ratio": 1.0,
    }, GameDynamicsConfig())
    assert status == FAIL
    assert "did not meet centering" in summary
    assert "FINAL_ERROR_TOO_HIGH" in notes
    assert "ERROR_REDUCTION_LOW" in notes


def test_game_dynamics_evaluator_fails_wrong_initial_yaw_direction():
    status, summary, notes = evaluate_game_dynamics_metrics({
        "frames": 10,
        "found_ratio": 1.0,
        "loss_events": 0,
        "initial_abs_error_px": 180.0,
        "final_abs_error_px": 8.0,
        "error_reduction_ratio": 0.9,
        "max_abs_yaw_axis": 0.2,
        "yaw_initial_command_corrective": False,
        "yaw_corrective_command_ratio": 0.9,
    }, GameDynamicsConfig())
    assert status == FAIL
    assert "did not meet centering" in summary
    assert "YAW_INITIAL_COMMAND_DIRECTION_WRONG" in notes


def test_game_dynamics_phase_writes_log(tmp_path):
    result = check_s9_game_dynamics(
        duration_s=4.0,
        hz=20.0,
        log_dir=tmp_path,
        run_id="unit-game-dynamics-phase",
        initial_offset_x=180.0,
        max_final_error_px=35.0,
        min_error_reduction_ratio=0.65,
        min_found_ratio=0.9,
        max_yaw_axis=0.8,
    )
    assert result.phase == "S9-game"
    assert result.status == PASS
    assert result.metrics["control_applied_to_game_camera"] is True
    assert Path(result.metrics["log_path"]).exists()


def test_range_dynamics_loop_applies_pitch_to_target_width(tmp_path):
    result = run_game_range_dynamics_loop(GameRangeDynamicsConfig(
        duration_s=5.0,
        hz=20.0,
        initial_offset_x=160.0,
        initial_target_width=70.0,
        desired_target_width=120.0,
        log_dir=tmp_path,
        run_id="unit-range-dynamics",
    ))
    assert result.status == PASS
    assert result.metrics["control_applied_to_game_camera"] is True
    assert result.metrics["range_control_applied_to_target_width"] is True
    assert result.metrics["found_ratio"] >= 0.9
    assert result.metrics["final_abs_error_px"] <= 35.0
    assert result.metrics["final_abs_width_error_px"] <= 8.0
    assert result.metrics["width_error_reduction_ratio"] >= 0.65
    assert result.metrics["yaw_initial_command_corrective"] is True
    assert result.metrics["pitch_initial_command_corrective"] is True
    assert result.metrics["pitch_initial_error_px"] > 0.0
    assert result.metrics["pitch_initial_command"] > 0.0
    assert result.metrics["max_abs_pitch_axis"] > 0.0
    assert result.metrics["real_input"] is False


def test_range_dynamics_evaluator_fails_without_width_convergence():
    config = GameRangeDynamicsConfig()
    status, summary, notes = evaluate_range_metrics({
        "frames": 10,
        "found_ratio": 1.0,
        "loss_events": 0,
        "initial_abs_error_px": 180.0,
        "final_abs_error_px": 8.0,
        "error_reduction_ratio": 0.9,
        "max_abs_yaw_axis": 0.2,
        "initial_abs_width_error_px": 50.0,
        "final_abs_width_error_px": 45.0,
        "width_error_reduction_ratio": 0.1,
        "max_abs_pitch_axis": 0.2,
        "yaw_initial_command_corrective": True,
        "yaw_corrective_command_ratio": 1.0,
        "pitch_initial_command_corrective": True,
        "pitch_corrective_command_ratio": 1.0,
    }, config)
    assert status == FAIL
    assert "center/approach" in summary
    assert "FINAL_WIDTH_ERROR_TOO_HIGH" in notes
    assert "WIDTH_ERROR_REDUCTION_LOW" in notes


def test_range_dynamics_evaluator_fails_wrong_initial_pitch_direction():
    config = GameRangeDynamicsConfig()
    status, summary, notes = evaluate_range_metrics({
        "frames": 10,
        "found_ratio": 1.0,
        "loss_events": 0,
        "initial_abs_error_px": 180.0,
        "final_abs_error_px": 8.0,
        "error_reduction_ratio": 0.9,
        "max_abs_yaw_axis": 0.2,
        "initial_abs_width_error_px": 50.0,
        "final_abs_width_error_px": 5.0,
        "width_error_reduction_ratio": 0.9,
        "max_abs_pitch_axis": 0.2,
        "yaw_initial_command_corrective": True,
        "yaw_corrective_command_ratio": 1.0,
        "pitch_initial_command_corrective": False,
        "pitch_corrective_command_ratio": 0.9,
    }, config)
    assert status == FAIL
    assert "center/approach" in summary
    assert "PITCH_INITIAL_COMMAND_DIRECTION_WRONG" in notes


def test_range_dynamics_phase_writes_log(tmp_path):
    result = check_s10_range_dynamics(
        duration_s=5.0,
        hz=20.0,
        log_dir=tmp_path,
        run_id="unit-range-dynamics-phase",
        initial_offset_x=160.0,
        max_final_error_px=35.0,
        min_error_reduction_ratio=0.65,
        min_found_ratio=0.9,
        max_yaw_axis=0.8,
        initial_target_width=70.0,
        desired_target_width=120.0,
        max_final_width_error_px=8.0,
        min_width_error_reduction_ratio=0.65,
        max_pitch_axis=0.8,
    )
    assert result.phase == "S10-range"
    assert result.status == PASS
    assert result.metrics["range_control_applied_to_target_width"] is True
    assert Path(result.metrics["log_path"]).exists()


def test_handoff_dynamics_loop_keeps_auto_off_until_follow(tmp_path):
    result = run_game_handoff_dynamics_loop(GameHandoffDynamicsConfig(
        duration_s=6.0,
        hz=20.0,
        initial_offset_x=180.0,
        initial_target_width=70.0,
        desired_target_width=120.0,
        log_dir=tmp_path,
        run_id="unit-handoff-dynamics",
    ))
    assert result.status == PASS
    assert result.metrics["follow_command_sent"] is True
    assert result.metrics["tracker_initialized_before_follow"] is False
    assert result.metrics["tracker_initialized_after_follow"] is True
    assert result.metrics["pre_handoff_auto_control_samples"] == 0
    assert result.metrics["manual_final_abs_error_px"] <= 45.0
    assert result.metrics["final_abs_error_px"] <= 35.0
    assert result.metrics["final_abs_width_error_px"] <= 8.0
    assert result.metrics["manual_to_final_error_reduction_ratio"] >= 0.80
    assert result.metrics["yaw_initial_command_corrective"] is True
    assert result.metrics["pitch_initial_command_corrective"] is True
    assert result.metrics["manual_to_auto_handoff"] is True
    assert result.metrics["real_input"] is False


def test_handoff_dynamics_evaluator_fails_when_auto_starts_before_follow():
    status, summary, notes = evaluate_handoff_metrics({
        "frames": 120,
        "found_ratio": 1.0,
        "loss_events": 0,
        "manual_initial_abs_error_px": 180.0,
        "manual_to_final_error_reduction_ratio": 0.9,
        "final_abs_error_px": 8.0,
        "initial_abs_width_error_px": 50.0,
        "final_abs_width_error_px": 5.0,
        "width_error_reduction_ratio": 0.9,
        "max_abs_yaw_axis": 0.2,
        "max_abs_pitch_axis": 0.2,
        "yaw_initial_command_corrective": True,
        "yaw_corrective_command_ratio": 1.0,
        "pitch_initial_command_corrective": True,
        "pitch_corrective_command_ratio": 1.0,
        "manual_frames": 32,
        "auto_frames": 88,
        "tracker_initialized_before_follow": True,
        "pre_handoff_auto_control_samples": 1,
        "manual_final_abs_error_px": 20.0,
        "follow_command_sent": True,
        "tracker_initialized_after_follow": True,
    }, GameHandoffDynamicsConfig())
    assert status == FAIL
    assert "handoff did not meet gates" in summary
    assert "TRACKER_INITIALIZED_BEFORE_FOLLOW" in notes
    assert "AUTO_CONTROL_BEFORE_FOLLOW" in notes


def test_handoff_dynamics_evaluator_fails_wrong_initial_auto_yaw_direction():
    status, summary, notes = evaluate_handoff_metrics({
        "frames": 120,
        "found_ratio": 1.0,
        "loss_events": 0,
        "manual_initial_abs_error_px": 180.0,
        "manual_to_final_error_reduction_ratio": 0.9,
        "final_abs_error_px": 8.0,
        "initial_abs_width_error_px": 50.0,
        "final_abs_width_error_px": 5.0,
        "width_error_reduction_ratio": 0.9,
        "max_abs_yaw_axis": 0.2,
        "max_abs_pitch_axis": 0.2,
        "yaw_initial_command_corrective": False,
        "yaw_corrective_command_ratio": 0.9,
        "pitch_initial_command_corrective": True,
        "pitch_corrective_command_ratio": 1.0,
        "manual_frames": 32,
        "auto_frames": 88,
        "tracker_initialized_before_follow": False,
        "pre_handoff_auto_control_samples": 0,
        "manual_final_abs_error_px": 20.0,
        "follow_command_sent": True,
        "tracker_initialized_after_follow": True,
    }, GameHandoffDynamicsConfig())
    assert status == FAIL
    assert "handoff did not meet gates" in summary
    assert "YAW_INITIAL_COMMAND_DIRECTION_WRONG" in notes


def test_handoff_dynamics_phase_writes_log(tmp_path):
    result = check_s11_handoff_dynamics(
        duration_s=6.0,
        hz=20.0,
        log_dir=tmp_path,
        run_id="unit-handoff-dynamics-phase",
        manual_duration_s=1.6,
        initial_offset_x=180.0,
        max_final_error_px=35.0,
        min_found_ratio=0.9,
        max_yaw_axis=0.8,
        initial_target_width=70.0,
        desired_target_width=120.0,
        max_final_width_error_px=8.0,
        min_width_error_reduction_ratio=0.65,
        max_pitch_axis=0.8,
        max_handoff_error_px=45.0,
    )
    assert result.phase == "S11-handoff"
    assert result.status == PASS
    assert result.metrics["manual_to_auto_handoff"] is True
    assert Path(result.metrics["log_path"]).exists()


def test_simple_game_adapter_loop_uses_input_adapter(tmp_path):
    result = run_adapter_closed_loop(AdapterLoopConfig(
        duration_s=5.0,
        hz=20.0,
        initial_offset_x=180.0,
        log_dir=tmp_path,
        run_id="unit-simple-game-adapter",
    ))
    assert result.status == PASS
    assert result.metrics["control_applied_through_input_adapter"] is True
    assert result.metrics["source_reads_adapter_command"] is True
    assert result.metrics["nonneutral_adapter_commands"] > 0
    assert result.metrics["applied_nonneutral_commands"] > 0
    assert result.metrics["neutralized"] is True
    assert result.metrics["camera_x_delta_px"] > 0
    assert result.metrics["final_abs_error_px"] <= 35.0


def test_simple_game_adapter_evaluator_fails_without_adapter_commands():
    status, summary, notes = evaluate_adapter_metrics({
        "frames": 100,
        "found_ratio": 1.0,
        "initial_abs_error_px": 180.0,
        "final_abs_error_px": 8.0,
        "error_reduction_ratio": 0.9,
        "max_abs_yaw_axis": 0.2,
        "nonneutral_adapter_commands": 0,
        "applied_nonneutral_commands": 0,
        "neutralized": True,
        "camera_x_delta_px": 120.0,
    }, AdapterLoopConfig())
    assert status == FAIL
    assert "adapter-driven simple game loop did not meet gates" in summary
    assert "NO_NONNEUTRAL_ADAPTER_COMMANDS" in notes
    assert "NO_APPLIED_GAME_COMMANDS" in notes


def test_simple_game_adapter_phase_writes_log(tmp_path):
    result = check_s12_adapter_dynamics(
        duration_s=5.0,
        hz=20.0,
        log_dir=tmp_path,
        run_id="unit-simple-game-adapter-phase",
        initial_offset_x=180.0,
        max_final_error_px=35.0,
        min_error_reduction_ratio=0.65,
        min_found_ratio=0.9,
        max_yaw_axis=0.8,
    )
    assert result.phase == "S12-adapter"
    assert result.status == PASS
    assert result.metrics["control_applied_through_input_adapter"] is True
    assert Path(result.metrics["log_path"]).exists()


def test_simple_game_objective_loop_runs_manual_follow_and_adapter_control(tmp_path):
    result = run_objective_closed_loop(ObjectiveLoopConfig(
        duration_s=6.0,
        hz=20.0,
        manual_duration_s=1.6,
        initial_offset_x=180.0,
        initial_target_width=70.0,
        desired_target_width=120.0,
        log_dir=tmp_path,
        run_id="unit-simple-game-objective",
    ))
    assert result.status == PASS
    assert result.metrics["manual_to_auto_handoff"] is True
    assert result.metrics["follow_command_sent"] is True
    assert result.metrics["follow_button_events"] >= 2
    assert result.metrics["tracker_initialized_before_follow"] is False
    assert result.metrics["tracker_initialized_after_follow"] is True
    assert result.metrics["pre_handoff_auto_control_samples"] == 0
    assert result.metrics["control_applied_through_input_adapter"] is True
    assert result.metrics["manual_commands_through_input_adapter"] is True
    assert result.metrics["autopilot_commands_through_input_adapter"] is True
    assert result.metrics["nonneutral_adapter_commands"] > 0
    assert result.metrics["applied_nonneutral_commands"] > 0
    assert result.metrics["range_control_applied_to_target_width"] is True
    assert result.metrics["final_abs_error_px"] <= 35.0
    assert result.metrics["final_abs_width_error_px"] <= 8.0
    assert result.metrics["neutralized"] is True
    assert result.metrics["gazebo_independent"] is True


def test_simple_game_objective_evaluator_fails_without_adapter_contract():
    config = ObjectiveLoopConfig()
    base_metrics = {
        "frames": 100,
        "found_ratio": 1.0,
        "loss_events": 0,
        "manual_initial_abs_error_px": 180.0,
        "manual_to_final_error_reduction_ratio": 0.9,
        "final_abs_error_px": 8.0,
        "initial_abs_width_error_px": 50.0,
        "final_abs_width_error_px": 4.0,
        "width_error_reduction_ratio": 0.9,
        "max_abs_yaw_axis": 0.2,
        "max_abs_pitch_axis": 0.2,
        "yaw_initial_command_corrective": True,
        "pitch_initial_command_corrective": True,
        "yaw_corrective_command_ratio": 1.0,
        "pitch_corrective_command_ratio": 1.0,
        "manual_frames": 30,
        "auto_frames": 70,
        "tracker_initialized_before_follow": False,
        "pre_handoff_auto_control_samples": 0,
        "manual_final_abs_error_px": 20.0,
        "follow_command_sent": True,
        "tracker_initialized_after_follow": True,
        "manual_commands_through_input_adapter": False,
        "autopilot_commands_through_input_adapter": False,
        "control_applied_through_input_adapter": False,
        "nonneutral_adapter_commands": 0,
        "applied_nonneutral_commands": 0,
        "follow_button_events": 0,
        "neutralized": False,
    }
    status, summary, notes = evaluate_objective_metrics(base_metrics, config)
    assert status == FAIL
    assert "objective-flow simple game loop did not meet gates" in summary
    assert "MANUAL_COMMANDS_NOT_THROUGH_ADAPTER" in notes
    assert "AUTOPILOT_COMMANDS_NOT_THROUGH_ADAPTER" in notes
    assert "CONTROL_NOT_APPLIED_THROUGH_ADAPTER" in notes
    assert "ADAPTER_NONNEUTRAL_COMMANDS_LOW" in notes
    assert "APPLIED_NONNEUTRAL_COMMANDS_LOW" in notes
    assert "FOLLOW_BUTTON_EVENTS_LOW" in notes
    assert "INPUT_NOT_NEUTRALIZED" in notes


def test_simple_game_objective_phase_writes_log(tmp_path):
    result = check_s14_objective_loop(
        duration_s=6.0,
        hz=20.0,
        log_dir=tmp_path,
        run_id="unit-simple-game-objective-phase",
        manual_duration_s=1.6,
        initial_offset_x=180.0,
        max_final_error_px=35.0,
        min_found_ratio=0.9,
        max_yaw_axis=0.8,
        initial_target_width=70.0,
        desired_target_width=120.0,
        max_final_width_error_px=8.0,
        min_width_error_reduction_ratio=0.65,
        max_pitch_axis=0.8,
        max_handoff_error_px=45.0,
    )
    assert result.phase == "S14-objective"
    assert result.status == PASS
    assert result.metrics["control_applied_through_input_adapter"] is True
    assert result.metrics["manual_to_auto_handoff"] is True
    assert Path(result.metrics["log_path"]).exists()


def test_live_loop_gate_can_use_uinput_adapter_when_acknowledged(monkeypatch, tmp_path):
    synthetic = SyntheticFrameSource(width=320, height=240, fps=10)

    class FakeProbe:
        available = True

        def as_dict(self):
            return {"available": True}

    monkeypatch.setattr(phase_runner_module, "resolve_capture_region",
                        lambda **_kwargs: parse_region("0,0,320,240"))
    monkeypatch.setattr(phase_runner_module, "make_region_source",
                        lambda *_args, **_kwargs: synthetic)
    monkeypatch.setattr(phase_runner_module, "probe_uinput_environment",
                        lambda: FakeProbe())
    monkeypatch.setattr(phase_runner_module, "UInputAdapter", DryRunInputAdapter)
    result = check_s4_real_closed_loop(
        parse_region("0,0,320,240"),
        backend="ffmpeg",
        duration_s=0.5,
        hz=10.0,
        min_found_ratio=0.8,
        tracker_bbox=synthetic.target_bbox_at(0.0),
        log_dir=tmp_path,
        run_id="unit-live-loop-pass",
        real_input=True,
        ack_live_input=True,
        phase="S4-live",
    )
    assert result.status == PASS
    assert result.phase == "S4-live"
    assert result.metrics["real_input"] is True
    assert result.metrics["adapter"] == "DryRunInputAdapter"


def test_dry_run_input_clamps_and_neutralizes_on_close():
    adapter = DryRunInputAdapter()
    adapter.send(AxisCommand(yaw=2.0, pitch=-2.0, roll=0.5, throttle=-0.5))
    assert adapter.last_command == AxisCommand(yaw=1.0, pitch=-1.0, roll=0.5, throttle=-0.5)
    adapter.close()
    assert adapter.commands[-1][1].is_neutral()
    assert adapter.closed


def test_command_from_channels_uses_pipeline_channel_mapping():
    cfg = PipelineConfig()
    channels = [1500] * cfg.num_channels
    channels[cfg.roll_ch] = 1600
    channels[cfg.pitch_ch] = 1400
    channels[cfg.throttle_ch] = 1750
    channels[cfg.yaw_ch] = 1250
    command = command_from_channels(channels, cfg)
    assert command.roll == 0.2
    assert command.pitch == -0.2
    assert command.throttle == 0.5
    assert command.yaw == -0.5


def test_tracking_loop_is_gazebo_free_and_drives_yaw_axis():
    source = SyntheticFrameSource(width=640, height=480, fps=30)
    tracker = FixedTracker(
        TrackResult(found=True, bbox=(420, 200, 100, 100), center=(470.0, 250.0))
    )
    adapter = DryRunInputAdapter()
    config = LoopConfig(
        duration_s=0.5,
        hz=0.0 + 10.0,
        initial_bbox=(270, 190, 100, 100),
        yaw_kp=0.8,
        yaw_ki=0.0,
        yaw_kd=0.0,
        yaw_output_limit=120.0,
    )
    summary = run_loop(source, tracker, adapter, config)
    assert summary.frames == 15
    assert summary.target_found == 15
    assert summary.found_ratio == 1.0
    assert summary.max_abs_horizontal_error > 100
    assert summary.as_dict()["horizontal_error_rms"] > 100
    assert summary.as_dict()["horizontal_error_p95"] > 100
    assert any(abs(command.yaw) > 0 for _, command in adapter.commands)
    assert adapter.commands[-1][1].is_neutral()


def test_tracking_loop_can_drive_pitch_axis_from_bbox_width():
    source = SyntheticFrameSource(width=640, height=480, fps=30)
    tracker = FixedTracker(
        TrackResult(found=True, bbox=(280, 200, 80, 80), center=(320.0, 240.0))
    )
    adapter = DryRunInputAdapter()
    config = LoopConfig(
        duration_s=0.5,
        hz=10.0,
        initial_bbox=(280, 200, 80, 80),
        enable_pitch=True,
        desired_target_width=120.0,
        forward_kp=0.4,
        forward_ki=0.0,
        forward_kd=0.0,
        forward_output_limit=80.0,
    )
    summary = run_loop(source, tracker, adapter, config)
    assert summary.frames == 15
    assert summary.found_ratio == 1.0
    assert summary.initial_target_width == 80.0
    assert summary.last_target_width == 80.0
    assert summary.max_abs_forward_error == 40.0
    assert summary.as_dict()["forward_error_rms"] == pytest.approx(40.0)
    assert summary.as_dict()["forward_error_p95"] == pytest.approx(40.0)
    assert any(abs(command.pitch) > 0 for _, command in adapter.commands)
    assert adapter.commands[-1][1].is_neutral()


def test_tracking_loop_pitch_delay_frames_holds_pitch_zero_then_releases():
    source = SyntheticFrameSource(width=640, height=480, fps=30)
    tracker = FixedTracker(
        TrackResult(found=True, bbox=(280, 200, 80, 80), center=(320.0, 240.0))
    )
    adapter = DryRunInputAdapter()
    config = LoopConfig(
        duration_s=0.5,
        hz=10.0,
        initial_bbox=(280, 200, 80, 80),
        enable_pitch=True,
        pitch_delay_frames=5,
        desired_target_width=120.0,
        forward_kp=0.4,
        forward_ki=0.0,
        forward_kd=0.0,
        forward_output_limit=80.0,
    )
    summary = run_loop(source, tracker, adapter, config)
    assert summary.frames == 15
    loop_commands = [command for _, command in adapter.commands[:summary.frames]]
    assert all(command.pitch == 0.0 for command in loop_commands[:5])
    assert any(abs(command.pitch) > 0 for command in loop_commands[5:])


def test_tracking_loop_reports_target_motion_and_loss_events():
    source = SyntheticFrameSource(width=160, height=120, fps=4)
    tracker = SequenceTracker([
        TrackResult(found=True, bbox=(0, 0, 20, 20), center=(10.0, 10.0)),
        TrackResult(found=True, bbox=(10, 0, 20, 20), center=(20.0, 10.0)),
        TrackResult(found=False),
        TrackResult(found=True, bbox=(30, 20, 20, 20), center=(40.0, 30.0)),
    ])
    adapter = DryRunInputAdapter()
    config = LoopConfig(
        duration_s=1.0,
        hz=4.0,
        initial_bbox=(0, 0, 20, 20),
        enable_pitch=True,
        frame_width=160,
        frame_height=120,
    )
    summary = run_loop(source, tracker, adapter, config)
    assert summary.frames == 4
    assert summary.target_found == 3
    assert summary.loss_events == 1
    assert summary.first_target_center == (10.0, 10.0)
    assert summary.last_target_center == (40.0, 30.0)
    assert summary.target_center_span_px == pytest.approx(36.0555, rel=1e-3)
    assert summary.target_center_travel_px == pytest.approx(36.0555, rel=1e-3)
    assert summary.as_dict()["target_center_span_px"] == pytest.approx(36.0555, rel=1e-3)
    assert summary.initial_horizontal_error == pytest.approx(-70.0)
    assert summary.last_horizontal_error == pytest.approx(-40.0)
    assert summary.horizontal_error_reduction_px == pytest.approx(30.0)
    assert summary.horizontal_error_reduction_ratio == pytest.approx(30.0 / 70.0)
    assert summary.as_dict()["last_abs_horizontal_error"] == pytest.approx(40.0)


def test_screen_tracking_loop_uinput_requires_ack():
    with pytest.raises(SystemExit):
        screen_tracking_loop_module.parse_args([
            "--synthetic",
            "--uinput",
            "--duration",
            "0.5",
        ])


def test_screen_tracking_loop_uinput_duration_is_capped():
    with pytest.raises(SystemExit):
        screen_tracking_loop_module.parse_args([
            "--synthetic",
            "--uinput",
            "--ack-live-input",
            "--duration",
            "5",
            "--live-max-duration",
            "1",
        ])


def test_phase_report_status_fails_on_any_failed_gate():
    report = PhaseRunReport(
        run_id="unit",
        mode="synthetic",
        started_at="now",
        results=[
            PhaseResult("S1", PASS, "ok"),
            PhaseResult("S2", "FAIL", "bad"),
        ],
    )
    assert report.status == "FAIL"


def test_phase_runner_synthetic_gates_write_pass_report(tmp_path):
    report = run_synthetic_gates(
        duration_s=0.5,
        hz=10.0,
        min_fps=8.0,
        min_tracker_found_ratio=0.80,
        min_loop_found_ratio=0.80,
        log_dir=tmp_path,
        run_id="unit-phase",
    )
    assert report.status == PASS
    assert [result.phase for result in report.results] == ["S0", "S1", "S2", "S3", "S4"]
    assert all(result.status == PASS for result in report.results)
    json_path, md_path = write_reports(report, tmp_path)
    assert json_path.exists()
    assert md_path.exists()
    text = md_path.read_text(encoding="utf-8")
    assert "Game Screen Sandbox Phase Report" in text
    assert "## Run Summary" in text
    assert "## Evidence Summary" in text
    assert "sandbox-only; no Gazebo/Betaflight launcher required" in text
    assert "| Phase | Status | Found | Loss | Motion px |" in text


def test_markdown_report_summarizes_key_s9_fields():
    report = PhaseRunReport(
        run_id="unit-s9",
        mode="synthetic",
        started_at="now",
        results=[
            PhaseResult(
                "S8",
                PASS,
                "moving target",
                metrics={
                    "source": "synthetic-moving-target",
                    "found_ratio": 1.0,
                    "loss_events": 0,
                    "target_center_span_px": 42.5,
                    "max_abs_yaw_axis": 0.2,
                    "max_abs_pitch_axis": 0.1,
                    "adapter": "DryRunInputAdapter",
                    "real_input": False,
                    "neutralized": True,
                    "log_path": "/tmp/s8.jsonl",
                },
            ),
        ],
    )
    text = build_markdown_report(report)
    assert "| Game/sim/source | synthetic-moving-target |" in text
    assert "| Input adapter(s) | DryRunInputAdapter |" in text
    assert "| S8 | PASS | 1 | 0 | 42.5 |" in text
    assert "| JSON details | full metrics are preserved in the paired phase-report JSON |" in text


def test_phase_runner_can_report_real_capture_as_waiting(tmp_path):
    report = run_synthetic_gates(
        duration_s=0.5,
        hz=10.0,
        min_fps=8.0,
        min_tracker_found_ratio=0.80,
        min_loop_found_ratio=0.80,
        log_dir=tmp_path,
        run_id="unit-phase-waiting",
        include_real_capture=True,
    )
    assert report.status == WAITING
    assert [result.phase for result in report.results] == [
        "S0", "S1", "S1-real", "S2", "S3", "S4",
    ]


def test_phase_runner_can_report_real_tracker_as_waiting(tmp_path):
    report = run_synthetic_gates(
        duration_s=0.5,
        hz=10.0,
        min_fps=8.0,
        min_tracker_found_ratio=0.80,
        min_loop_found_ratio=0.80,
        log_dir=tmp_path,
        run_id="unit-phase-tracker-waiting",
        include_real_tracker=True,
    )
    assert report.status == WAITING
    assert [result.phase for result in report.results] == [
        "S0", "S1", "S2", "S2-real", "S3", "S4",
    ]


def test_phase_runner_can_include_real_input_gate(tmp_path):
    report = run_synthetic_gates(
        duration_s=0.5,
        hz=10.0,
        min_fps=8.0,
        min_tracker_found_ratio=0.80,
        min_loop_found_ratio=0.80,
        log_dir=tmp_path,
        run_id="unit-phase-input",
        include_real_input=True,
    )
    assert "S3-real" in [result.phase for result in report.results]
    real_input = [result for result in report.results if result.phase == "S3-real"][0]
    assert real_input.status in {WAITING, "PASS"}


def test_phase_runner_can_include_real_loop_gate_as_waiting(tmp_path):
    report = run_synthetic_gates(
        duration_s=0.5,
        hz=10.0,
        min_fps=8.0,
        min_tracker_found_ratio=0.80,
        min_loop_found_ratio=0.80,
        log_dir=tmp_path,
        run_id="unit-phase-loop-waiting",
        include_real_loop=True,
    )
    assert report.status == WAITING
    assert report.results[-1].phase == "S4-real"


def test_phase_runner_can_include_live_loop_gate_as_waiting(tmp_path):
    report = run_synthetic_gates(
        duration_s=0.5,
        hz=10.0,
        min_fps=8.0,
        min_tracker_found_ratio=0.80,
        min_loop_found_ratio=0.80,
        log_dir=tmp_path,
        run_id="unit-phase-live-waiting",
        include_live_loop=True,
    )
    assert report.status == WAITING
    assert report.results[-1].phase == "S4-live"


def test_phase_runner_can_include_latency_probe_as_waiting(tmp_path):
    report = run_synthetic_gates(
        duration_s=0.5,
        hz=10.0,
        min_fps=8.0,
        min_tracker_found_ratio=0.80,
        min_loop_found_ratio=0.80,
        log_dir=tmp_path,
        run_id="unit-phase-latency-waiting",
        include_latency_probe=True,
    )
    assert report.status == WAITING
    assert report.results[-1].phase == "S5-real"


def test_phase_runner_cli_allows_short_latency_without_handoff():
    args = phase_runner_module.parse_args([
        "--duration", "1",
        "--include-latency-probe",
        "--latency-axis-sweep",
    ])
    assert args.include_latency_probe is True
    assert args.latency_axis_sweep is True
    assert args.include_handoff_dynamics is False
    with pytest.raises(SystemExit):
        phase_runner_module.parse_args([
            "--duration", "1",
            "--include-handoff-dynamics",
        ])


def test_latency_axis_sweep_phase_reports_axis_statuses(monkeypatch):
    source = StepResponseSource(fps=10, step_index=4)
    region = parse_region("0,0,32,24")
    monkeypatch.setattr(phase_runner_module, "resolve_capture_region",
                        lambda **_kwargs: region)
    monkeypatch.setattr(phase_runner_module, "make_region_source",
                        lambda *_args, **_kwargs: source)
    result = check_s5_real_latency(
        region,
        backend="ffmpeg",
        fps=10.0,
        pre_duration_s=0.3,
        post_duration_s=0.5,
        command=AxisCommand(yaw=0.05),
        min_diff=10.0,
        baseline_multiplier=3.0,
        use_uinput=False,
        ack_live_input=False,
        axis_sweep=True,
        axis_names=("yaw", "pitch"),
        axis_value=0.1,
    )
    assert result.phase == "S5-real"
    assert result.status == PASS
    assert result.metrics["axis_sweep"] is True
    assert result.metrics["axis_statuses"] == {"yaw": PASS, "pitch": PASS}
    assert result.metrics["passed_axes"] == ["yaw", "pitch"]


def test_latency_axis_sweep_phase_reports_signed_direction(monkeypatch):
    source = ShiftResponseSource(fps=10, step_index=4, shift=(5, 0))
    region = parse_region("0,0,64,48")
    monkeypatch.setattr(phase_runner_module, "resolve_capture_region",
                        lambda **_kwargs: region)
    monkeypatch.setattr(phase_runner_module, "make_region_source",
                        lambda *_args, **_kwargs: source)
    result = check_s5_real_latency(
        region,
        backend="ffmpeg",
        fps=10.0,
        pre_duration_s=0.3,
        post_duration_s=0.5,
        command=AxisCommand(yaw=0.05),
        min_diff=3.0,
        baseline_multiplier=3.0,
        use_uinput=False,
        ack_live_input=False,
        axis_sweep=True,
        axis_names=("yaw",),
        axis_value=0.1,
        axis_expected_shifts={"yaw": ("x", 1)},
        axis_min_shift_px=1.0,
    )
    assert result.status == PASS
    assert result.metrics["axis_expected_shifts"] == {"yaw": "+x"}
    assert result.metrics["axis_results"]["yaw"]["direction_status"] == PASS


def test_external_window_preflight_waits_and_writes_frame_without_bbox(monkeypatch, tmp_path):
    synthetic = SyntheticFrameSource(width=64, height=48, fps=5)
    region = parse_region("0,0,64,48")
    monkeypatch.setattr(
        external_window_preflight_module,
        "resolve_capture_region",
        lambda **_kwargs: region,
    )
    monkeypatch.setattr(
        external_window_preflight_module,
        "make_region_source",
        lambda *_args, **_kwargs: synthetic,
    )
    report = run_external_window_preflight(
        run_id="unit-external-window-waiting",
        log_dir=tmp_path,
        region=region,
        window_title=None,
        backend="ffmpeg",
        bbox=None,
        frame_path=tmp_path / "target.png",
        overlay_path=None,
        duration_s=0.5,
        hz=5.0,
        min_fps=1.0,
        min_found_ratio=0.8,
        desired_target_width=120.0,
        max_yaw_axis=0.5,
        max_pitch_axis=0.5,
        include_axis_sweep=False,
        ack_live_input=False,
        axis_names=("yaw", "pitch"),
        axis_value=0.05,
        window_exact=False,
        window_case_sensitive=False,
        window_min_width=32,
        window_min_height=32,
    )
    assert report.status == WAITING
    assert "BBOX_SELECTION_REQUIRED" in report.reasons
    assert Path(report.bbox_artifact["frame_path"]).exists()
    assert report.phases == []


def test_external_window_preflight_promotes_dry_when_window_gates_pass(monkeypatch, tmp_path):
    monkeypatch.setattr(
        external_window_preflight_module,
        "check_s1_real_capture",
        lambda *_args, **_kwargs: PhaseResult("S1-real", PASS, "capture pass"),
    )
    monkeypatch.setattr(
        external_window_preflight_module,
        "check_s2_real_tracker",
        lambda *_args, **_kwargs: PhaseResult("S2-real", PASS, "tracker pass"),
    )
    monkeypatch.setattr(
        external_window_preflight_module,
        "check_s7_real_combined",
        lambda *_args, **_kwargs: PhaseResult(
            "S7-real",
            PASS,
            "combined pass",
            metrics={"log_path": str(tmp_path / "s7.jsonl")},
        ),
    )
    report = run_external_window_preflight(
        run_id="unit-external-window-ready",
        log_dir=tmp_path,
        region=parse_region("0,0,640,480"),
        window_title=None,
        backend="ffmpeg",
        bbox=(100, 100, 80, 80),
        frame_path=None,
        overlay_path=None,
        duration_s=0.5,
        hz=5.0,
        min_fps=1.0,
        min_found_ratio=0.8,
        desired_target_width=120.0,
        max_yaw_axis=0.5,
        max_pitch_axis=0.5,
        include_axis_sweep=False,
        ack_live_input=False,
        axis_names=("yaw", "pitch"),
        axis_value=0.05,
        window_exact=False,
        window_case_sensitive=False,
        window_min_width=32,
        window_min_height=32,
    )
    assert report.status == EXTERNAL_WINDOW_READY_DRY
    assert report.reasons == []
    assert report.metrics["phase_statuses"] == {
        "S1-real": PASS,
        "S2-real": PASS,
        "S7-real": PASS,
    }
    assert report.evidence_paths["s7_log"] == str(tmp_path / "s7.jsonl")


def test_external_window_preflight_axis_sweep_requires_ack(monkeypatch, tmp_path):
    monkeypatch.setattr(
        external_window_preflight_module,
        "check_s1_real_capture",
        lambda *_args, **_kwargs: PhaseResult("S1-real", PASS, "capture pass"),
    )
    monkeypatch.setattr(
        external_window_preflight_module,
        "check_s2_real_tracker",
        lambda *_args, **_kwargs: PhaseResult("S2-real", PASS, "tracker pass"),
    )
    monkeypatch.setattr(
        external_window_preflight_module,
        "check_s7_real_combined",
        lambda *_args, **_kwargs: PhaseResult("S7-real", PASS, "combined pass"),
    )
    report = run_external_window_preflight(
        run_id="unit-external-window-axis-waiting",
        log_dir=tmp_path,
        region=parse_region("0,0,640,480"),
        window_title=None,
        backend="ffmpeg",
        bbox=(100, 100, 80, 80),
        frame_path=None,
        overlay_path=None,
        duration_s=0.5,
        hz=5.0,
        min_fps=1.0,
        min_found_ratio=0.8,
        desired_target_width=120.0,
        max_yaw_axis=0.5,
        max_pitch_axis=0.5,
        include_axis_sweep=True,
        ack_live_input=False,
        axis_names=("yaw", "pitch"),
        axis_value=0.05,
        window_exact=False,
        window_case_sensitive=False,
        window_min_width=32,
        window_min_height=32,
    )
    assert report.status == WAITING
    assert "PHASE_NOT_PASS:S5-real:WAITING" in report.reasons
    assert report.metrics["phase_statuses"]["S5-real"] == WAITING
    assert report.metrics["include_axis_sweep"] is True
    assert report.metrics["ack_live_input"] is False


def test_external_window_preflight_promotes_live_input_when_axis_sweep_passes(
    monkeypatch,
    tmp_path,
):
    monkeypatch.setattr(
        external_window_preflight_module,
        "check_s1_real_capture",
        lambda *_args, **_kwargs: PhaseResult("S1-real", PASS, "capture pass"),
    )
    monkeypatch.setattr(
        external_window_preflight_module,
        "check_s2_real_tracker",
        lambda *_args, **_kwargs: PhaseResult("S2-real", PASS, "tracker pass"),
    )
    monkeypatch.setattr(
        external_window_preflight_module,
        "check_s7_real_combined",
        lambda *_args, **_kwargs: PhaseResult("S7-real", PASS, "combined pass"),
    )
    monkeypatch.setattr(
        external_window_preflight_module,
        "check_s5_real_latency",
        lambda *_args, **_kwargs: PhaseResult(
            "S5-real",
            PASS,
            "axis sweep pass",
            metrics={
                "axis_sweep": True,
                "axis_statuses": {"yaw": PASS, "pitch": PASS},
            },
        ),
    )
    report = run_external_window_preflight(
        run_id="unit-external-window-live-ready",
        log_dir=tmp_path,
        region=parse_region("0,0,640,480"),
        window_title=None,
        backend="ffmpeg",
        bbox=(100, 100, 80, 80),
        frame_path=None,
        overlay_path=None,
        duration_s=0.5,
        hz=5.0,
        min_fps=1.0,
        min_found_ratio=0.8,
        desired_target_width=120.0,
        max_yaw_axis=0.5,
        max_pitch_axis=0.5,
        include_axis_sweep=True,
        ack_live_input=True,
        axis_names=("yaw", "pitch"),
        axis_value=0.05,
        window_exact=False,
        window_case_sensitive=False,
        window_min_width=32,
        window_min_height=32,
    )
    assert report.status == EXTERNAL_WINDOW_READY_LIVE_INPUT
    assert report.reasons == []
    assert report.metrics["phase_statuses"]["S5-real"] == PASS


def test_external_window_preflight_decider_rejects_failed_phase():
    status, summary, reasons = decide_external_window_preflight(
        [PhaseResult("S1-real", PASS, "ok"), PhaseResult("S2-real", FAIL, "bad")],
        bbox_artifact=None,
    )
    assert status == REJECT
    assert "external window preflight failed" in summary
    assert "PHASE_NOT_PASS:S2-real:FAIL" in reasons


def test_external_window_preflight_decider_waits_for_axis_gate_when_required():
    status, summary, reasons = decide_external_window_preflight(
        [
            PhaseResult("S1-real", PASS, "ok"),
            PhaseResult("S2-real", PASS, "ok"),
            PhaseResult("S7-real", PASS, "ok"),
        ],
        bbox_artifact=None,
        require_axis_response=True,
    )
    assert status == WAITING
    assert "waiting for required evidence" in summary
    assert "PHASE_MISSING:S5-real" in reasons


def external_follow_kwargs(tmp_path, **overrides):
    params = {
        "run_id": "unit-external-follow",
        "log_dir": tmp_path,
        "region": parse_region("0,0,320,240"),
        "window_title": None,
        "backend": "ffmpeg",
        "tracker_bbox": (110, 70, 100, 100),
        "duration_s": 0.5,
        "hz": 10.0,
        "tracker_type": "KCF",
        "enable_pitch": False,
        "desired_target_width": 120.0,
        "min_found_ratio": 0.8,
        "max_loss_events": 0,
        "max_yaw_axis": 0.8,
        "max_pitch_axis": 0.8,
        "use_uinput": False,
        "ack_live_input": False,
        "live_max_duration_s": 2.0,
        "window_exact": False,
        "window_case_sensitive": False,
        "window_min_width": 32,
        "window_min_height": 32,
    }
    params.update(overrides)
    return params


def test_external_follow_session_waits_without_bbox(tmp_path):
    report = run_external_follow_session(
        **external_follow_kwargs(tmp_path, tracker_bbox=None),
    )
    assert report.status == WAITING
    assert report.summary == "external follow session is waiting for a target bbox"
    assert "TRACKER_BBOX_REQUIRED" in report.reasons


def test_external_follow_session_live_input_requires_ack(tmp_path):
    report = run_external_follow_session(
        **external_follow_kwargs(tmp_path, use_uinput=True, ack_live_input=False),
    )
    assert report.status == WAITING
    assert report.summary == "external follow live input requires explicit acknowledgement"
    assert "ACK_LIVE_INPUT_REQUIRED" in report.reasons


def test_external_follow_session_dry_run_promotes_when_tracker_stays_found(
    monkeypatch,
    tmp_path,
):
    monkeypatch.setattr(external_follow_session_module, "process_snapshot", lambda: [])
    monkeypatch.setattr(
        external_follow_session_module,
        "resolve_capture_region",
        lambda **_kwargs: parse_region("0,0,320,240"),
    )
    monkeypatch.setattr(
        external_follow_session_module,
        "make_region_source",
        lambda *_args, **_kwargs: SyntheticFrameSource(width=320, height=240, fps=10),
    )
    monkeypatch.setattr(
        external_follow_session_module,
        "ObjectTracker",
        lambda _tracker_type: FixedTracker(
            TrackResult(found=True, bbox=(110, 70, 100, 100), center=(160.0, 120.0))
        ),
    )
    report = run_external_follow_session(**external_follow_kwargs(tmp_path))
    assert report.status == EXTERNAL_FOLLOW_DRY_RUN_READY
    assert report.reasons == []
    assert report.metrics["adapter"] == "DryRunInputAdapter"
    assert report.metrics["loop_summary"]["found_ratio"] == 1.0
    assert Path(report.evidence_paths["loop_log"]).exists()


def test_external_follow_session_rejects_when_tracker_is_lost(monkeypatch, tmp_path):
    monkeypatch.setattr(external_follow_session_module, "process_snapshot", lambda: [])
    monkeypatch.setattr(
        external_follow_session_module,
        "resolve_capture_region",
        lambda **_kwargs: parse_region("0,0,320,240"),
    )
    monkeypatch.setattr(
        external_follow_session_module,
        "make_region_source",
        lambda *_args, **_kwargs: SyntheticFrameSource(width=320, height=240, fps=10),
    )
    monkeypatch.setattr(
        external_follow_session_module,
        "ObjectTracker",
        lambda _tracker_type: FixedTracker(TrackResult(found=False)),
    )
    report = run_external_follow_session(**external_follow_kwargs(tmp_path))
    assert report.status == REJECT
    assert any(reason.startswith("FOUND_RATIO_LOW") for reason in report.reasons)


def test_external_follow_session_metric_evaluator_checks_axis_limits():
    reasons = evaluate_session_metrics(
        {
            "frames": 10,
            "found_ratio": 1.0,
            "loss_events": 0,
            "max_abs_yaw_axis": 0.9,
            "max_abs_pitch_axis": 0.7,
        },
        min_found_ratio=0.8,
        max_loss_events=0,
        max_yaw_axis=0.8,
        max_pitch_axis=0.5,
        enable_pitch=True,
    )
    assert "YAW_AXIS_LIMIT:0.900>0.800" in reasons
    assert "PITCH_AXIS_LIMIT:0.700>0.500" in reasons


def test_phase_runner_can_include_real_approach_gate_as_waiting(tmp_path):
    report = run_synthetic_gates(
        duration_s=0.5,
        hz=10.0,
        min_fps=8.0,
        min_tracker_found_ratio=0.80,
        min_loop_found_ratio=0.80,
        log_dir=tmp_path,
        run_id="unit-phase-approach-waiting",
        include_real_approach=True,
    )
    assert report.status == WAITING
    assert report.results[-1].phase == "S6-real"


def test_phase_runner_can_include_real_combined_gate_as_waiting(tmp_path):
    report = run_synthetic_gates(
        duration_s=0.5,
        hz=10.0,
        min_fps=8.0,
        min_tracker_found_ratio=0.80,
        min_loop_found_ratio=0.80,
        log_dir=tmp_path,
        run_id="unit-phase-combined-waiting",
        include_real_combined=True,
    )
    assert report.status == WAITING
    assert report.results[-1].phase == "S7-real"


def test_phase_runner_can_include_moving_target_gate(tmp_path):
    report = run_synthetic_gates(
        duration_s=1.0,
        hz=10.0,
        min_fps=8.0,
        min_tracker_found_ratio=0.80,
        min_loop_found_ratio=0.80,
        log_dir=tmp_path,
        run_id="unit-phase-moving-target",
        include_moving_target=True,
        min_moving_target_motion_px=20.0,
    )
    assert report.status == PASS
    assert report.results[-1].phase == "S8"
    assert report.results[-1].metrics["center_motion_ok"] is True


def test_phase_runner_can_include_game_dynamics_gate(tmp_path):
    report = run_synthetic_gates(
        duration_s=4.0,
        hz=20.0,
        min_fps=8.0,
        min_tracker_found_ratio=0.80,
        min_loop_found_ratio=0.90,
        log_dir=tmp_path,
        run_id="unit-phase-game-dynamics",
        include_game_dynamics=True,
    )
    assert report.status == PASS
    assert report.results[-1].phase == "S9-game"
    assert report.results[-1].metrics["control_applied_to_game_camera"] is True


def test_phase_runner_can_include_range_dynamics_gate(tmp_path):
    report = run_synthetic_gates(
        duration_s=5.0,
        hz=20.0,
        min_fps=8.0,
        min_tracker_found_ratio=0.80,
        min_loop_found_ratio=0.90,
        log_dir=tmp_path,
        run_id="unit-phase-range-dynamics",
        include_range_dynamics=True,
    )
    assert report.status == PASS
    assert report.results[-1].phase == "S10-range"
    assert report.results[-1].metrics["range_control_applied_to_target_width"] is True


def test_phase_runner_can_include_handoff_dynamics_gate(tmp_path):
    report = run_synthetic_gates(
        duration_s=6.0,
        hz=20.0,
        min_fps=8.0,
        min_tracker_found_ratio=0.80,
        min_loop_found_ratio=0.90,
        log_dir=tmp_path,
        run_id="unit-phase-handoff-dynamics",
        include_handoff_dynamics=True,
    )
    assert report.status == PASS
    assert report.results[-1].phase == "S11-handoff"
    assert report.results[-1].metrics["manual_to_auto_handoff"] is True


def test_phase_runner_can_include_adapter_dynamics_gate(tmp_path):
    report = run_synthetic_gates(
        duration_s=5.0,
        hz=20.0,
        min_fps=8.0,
        min_tracker_found_ratio=0.80,
        min_loop_found_ratio=0.90,
        log_dir=tmp_path,
        run_id="unit-phase-adapter-dynamics",
        include_adapter_dynamics=True,
    )
    assert report.status == PASS
    assert report.results[-1].phase == "S12-adapter"
    assert report.results[-1].metrics["control_applied_through_input_adapter"] is True


def test_binding_dry_gate_reports_virtual_rc_sequence():
    result = check_s13_binding_dry()
    assert result.phase == "S13-binding-dry"
    assert result.status == PASS
    assert result.metrics["adapter"] == "DryRunInputAdapter"
    assert result.metrics["no_os_input"] is True
    assert result.metrics["command_count"] == len(result.metrics["steps"]) + 1


def test_phase_runner_can_include_binding_dry_gate(tmp_path):
    report = run_synthetic_gates(
        duration_s=0.5,
        hz=10.0,
        min_fps=8.0,
        min_tracker_found_ratio=0.80,
        min_loop_found_ratio=0.80,
        log_dir=tmp_path,
        run_id="unit-phase-binding-dry",
        include_binding_dry=True,
    )
    assert report.status == PASS
    assert report.results[-1].phase == "S13-binding-dry"


def test_phase_runner_can_include_objective_loop_gate(tmp_path):
    report = run_synthetic_gates(
        duration_s=6.0,
        hz=20.0,
        min_fps=8.0,
        min_tracker_found_ratio=0.80,
        min_loop_found_ratio=0.90,
        log_dir=tmp_path,
        run_id="unit-phase-objective-loop",
        include_objective_loop=True,
    )
    assert report.status == PASS
    assert report.results[-1].phase == "S14-objective"
    assert report.results[-1].metrics["control_applied_through_input_adapter"] is True


def write_phase_report(path, phases):
    path.write_text(json.dumps({
        "status": PASS,
        "results": [
            {"phase": phase, "status": status, "summary": "%s %s" % (phase, status)}
            for phase, status in phases
        ],
    }), encoding="utf-8")


def full_synthetic_phases(status=PASS):
    return [
        ("S0", PASS), ("S1", PASS), ("S2", PASS), ("S3", PASS),
        ("S4", PASS), ("S8", status), ("S9-game", PASS), ("S10-range", PASS),
        ("S11-handoff", PASS), ("S12-adapter", PASS), ("S13-binding-dry", PASS),
        ("S14-objective", PASS),
    ]


def test_isolation_audit_static_imports_pass_for_sandbox():
    result = audit_static_imports(SANDBOX)
    assert result.status == AUDIT_PASS
    assert result.metrics["forbidden_imports"] == []


def test_isolation_audit_process_delta_detects_new_forbidden_process():
    result = audit_process_delta(
        before=["100 /usr/bin/gazebo --server"],
        after=[
            "100 /usr/bin/gazebo --server",
            "200 /tmp/betaflight --sitl",
        ],
    )
    assert result.status == FAIL
    assert result.metrics["new_forbidden_processes"] == ["200 /tmp/betaflight --sitl"]
    assert "FORBIDDEN_PROCESS_STARTED" in result.notes


def test_acceptance_runner_waits_when_simple_window_is_skipped(monkeypatch, tmp_path):
    fake_synthetic = PhaseRunReport(
        run_id="unit-acceptance",
        mode="synthetic",
        started_at="2026-07-05T00:00:00+0300",
        results=[
            PhaseResult(phase=phase, status=status, summary=phase)
            for phase, status in full_synthetic_phases()
        ],
    )
    monkeypatch.setattr(
        acceptance_runner_module,
        "run_synthetic_gates",
        lambda **_kwargs: fake_synthetic,
    )
    monkeypatch.setattr(acceptance_runner_module, "process_snapshot", lambda: [])
    report = acceptance_runner_module.run_acceptance(
        run_id="unit-acceptance",
        log_dir=tmp_path,
        duration_s=1.0,
        hz=10.0,
        min_fps=8.0,
        include_simple_window=False,
        simple_window_timeout_s=0.1,
        simple_window_game_duration_s=1.0,
        simple_window_gate_duration_s=0.1,
    )
    assert report.status == WAITING
    assert report.synthetic_phase_report_json is not None
    assert report.decision_report_json is not None
    assert "simple-window smoke skipped" in " ".join(report.notes)
    assert "MISSING_SIMPLE_WINDOW_SMOKE" in report.metrics["decision_reasons"]
    assert report.metrics["isolation_status"] == AUDIT_PASS


def test_acceptance_runner_promotes_when_all_evidence_exists(monkeypatch, tmp_path):
    fake_synthetic = PhaseRunReport(
        run_id="unit-acceptance-ready",
        mode="synthetic",
        started_at="2026-07-05T00:00:00+0300",
        results=[
            PhaseResult(phase=phase, status=status, summary=phase)
            for phase, status in full_synthetic_phases()
        ],
    )
    simple_phase = tmp_path / "simple-phase-report.json"
    write_phase_report(simple_phase, [
        ("S1-real", PASS), ("S2-real", PASS), ("S7-real", PASS),
    ])
    monkeypatch.setattr(
        acceptance_runner_module,
        "run_synthetic_gates",
        lambda **_kwargs: fake_synthetic,
    )
    monkeypatch.setattr(acceptance_runner_module, "process_snapshot", lambda: [])
    monkeypatch.setattr(
        acceptance_runner_module,
        "run_simple_window_smoke",
        lambda **_kwargs: {
            "status": PASS,
            "phase_report_json": str(simple_phase),
            "phase_report_md": str(tmp_path / "simple-phase-report.md"),
        },
    )
    report = acceptance_runner_module.run_acceptance(
        run_id="unit-acceptance-ready",
        log_dir=tmp_path,
        duration_s=1.0,
        hz=10.0,
        min_fps=8.0,
        include_simple_window=True,
        simple_window_timeout_s=0.1,
        simple_window_game_duration_s=1.0,
        simple_window_gate_duration_s=0.1,
    )
    assert report.status == SIMPLE_SANDBOX_READY
    assert report.simple_window_smoke_report is not None
    assert report.simple_window_phase_report_json == str(simple_phase)
    assert report.metrics["decision_reasons"] == []
    assert report.metrics["isolation_status"] == AUDIT_PASS


def test_acceptance_runner_rejects_when_forbidden_process_starts(monkeypatch, tmp_path):
    fake_synthetic = PhaseRunReport(
        run_id="unit-acceptance-isolation-fail",
        mode="synthetic",
        started_at="2026-07-05T00:00:00+0300",
        results=[
            PhaseResult(phase=phase, status=status, summary=phase)
            for phase, status in full_synthetic_phases()
        ],
    )
    simple_phase = tmp_path / "simple-phase-report.json"
    write_phase_report(simple_phase, [
        ("S1-real", PASS), ("S2-real", PASS), ("S7-real", PASS),
    ])
    snapshots = iter([
        [],
        ["200 /tmp/betaflight --sitl"],
    ])
    monkeypatch.setattr(
        acceptance_runner_module,
        "run_synthetic_gates",
        lambda **_kwargs: fake_synthetic,
    )
    monkeypatch.setattr(
        acceptance_runner_module,
        "run_simple_window_smoke",
        lambda **_kwargs: {
            "status": PASS,
            "phase_report_json": str(simple_phase),
        },
    )
    monkeypatch.setattr(acceptance_runner_module, "process_snapshot", lambda: next(snapshots))
    report = acceptance_runner_module.run_acceptance(
        run_id="unit-acceptance-isolation-fail",
        log_dir=tmp_path,
        duration_s=1.0,
        hz=10.0,
        min_fps=8.0,
        include_simple_window=True,
        simple_window_timeout_s=0.1,
        simple_window_game_duration_s=1.0,
        simple_window_gate_duration_s=0.1,
    )
    assert report.status == REJECT
    assert report.metrics["decision"] == SIMPLE_SANDBOX_READY
    assert report.metrics["isolation_status"] == FAIL
    assert "FORBIDDEN_PROCESS_STARTED" in report.notes


def test_sandbox_decision_waits_without_evidence(tmp_path):
    paths, reports = choose_decision_evidence(
        log_dir=tmp_path,
        synthetic_report=None,
        simple_window_report=None,
        simple_window_phase_report=None,
    )
    report = evaluate_decision(reports, paths=paths, run_id="unit-decision")
    assert report.decision == WAITING
    assert "MISSING_SYNTHETIC_MOVING_TARGET_REPORT" in report.reasons
    assert "MISSING_SIMPLE_WINDOW_SMOKE" in report.reasons
    assert "MISSING_SIMPLE_WINDOW_PHASE_REPORT" in report.reasons
    assert report.metrics["scope"] == "isolated game/screen sandbox only; not pr0p/Betaflight promotion"


def test_sandbox_decision_rejects_failed_phase(tmp_path):
    synthetic = tmp_path / "synthetic-phase-report.json"
    simple_phase = tmp_path / "simple-phase-report.json"
    simple_smoke = tmp_path / "simple-window-smoke.json"
    write_phase_report(synthetic, [
        ("S0", PASS), ("S1", PASS), ("S2", PASS), ("S3", PASS),
        ("S4", PASS), ("S8", FAIL), ("S9-game", PASS), ("S10-range", PASS),
        ("S11-handoff", PASS), ("S12-adapter", PASS), ("S13-binding-dry", PASS),
        ("S14-objective", PASS),
    ])
    write_phase_report(simple_phase, [
        ("S1-real", PASS), ("S2-real", PASS), ("S7-real", PASS),
    ])
    simple_smoke.write_text(json.dumps({
        "status": PASS,
        "phase_report_json": str(simple_phase),
    }), encoding="utf-8")
    paths, reports = choose_decision_evidence(
        log_dir=tmp_path,
        synthetic_report=synthetic,
        simple_window_report=simple_smoke,
        simple_window_phase_report=simple_phase,
    )
    report = evaluate_decision(reports, paths=paths, run_id="unit-decision")
    assert report.decision == REJECT
    assert "FAIL:S8" in report.reasons


def test_sandbox_decision_promotes_ready_dry_run_evidence(tmp_path):
    synthetic = tmp_path / "20260705-000000-synthetic-phase-report.json"
    simple_phase = tmp_path / "20260705-000001-simple-phase-report.json"
    simple_smoke = tmp_path / "20260705-000002-simple-window-smoke.json"
    write_phase_report(synthetic, [
        ("S0", PASS), ("S1", PASS), ("S2", PASS), ("S3", PASS),
        ("S4", PASS), ("S8", PASS), ("S9-game", PASS), ("S10-range", PASS),
        ("S11-handoff", PASS), ("S12-adapter", PASS), ("S13-binding-dry", PASS),
        ("S14-objective", PASS),
    ])
    write_phase_report(simple_phase, [
        ("S1-real", PASS), ("S2-real", PASS), ("S7-real", PASS),
    ])
    simple_smoke.write_text(json.dumps({
        "status": PASS,
        "phase_report_json": str(simple_phase),
    }), encoding="utf-8")
    paths, reports = choose_decision_evidence(
        log_dir=tmp_path,
        synthetic_report=None,
        simple_window_report=None,
        simple_window_phase_report=None,
    )
    report = evaluate_decision(reports, paths=paths, run_id="unit-decision")
    assert latest_phase_report_with_phase(tmp_path, "S8") == synthetic
    assert decision_phase_statuses(reports["synthetic_report"])["S8"] == PASS
    assert decision_phase_statuses(reports["synthetic_report"])["S9-game"] == PASS
    assert decision_phase_statuses(reports["synthetic_report"])["S10-range"] == PASS
    assert decision_phase_statuses(reports["synthetic_report"])["S11-handoff"] == PASS
    assert decision_phase_statuses(reports["synthetic_report"])["S12-adapter"] == PASS
    assert decision_phase_statuses(reports["synthetic_report"])["S13-binding-dry"] == PASS
    assert decision_phase_statuses(reports["synthetic_report"])["S14-objective"] == PASS
    assert report.decision == SIMPLE_SANDBOX_READY
    assert report.reasons == []
    assert report.evidence_paths["simple_window_phase_report"] == str(simple_phase)
    text = build_decision_markdown(report)
    assert "Game Screen Sandbox Decision Report" in text
    assert "does not promote pr0p" in text


def write_status_json(path, data):
    path.write_text(json.dumps(data), encoding="utf-8")


def write_dry_ready_status_evidence(tmp_path):
    bbox_file = tmp_path / "pr0p-target-bbox.json"
    acceptance_path = tmp_path / "20260705-000000-acceptance-run.json"
    preflight_path = tmp_path / "20260705-000001-pr0p-external-window-preflight.json"
    follow_path = tmp_path / "20260705-000002-pr0p-external-follow-session.json"
    write_status_json(bbox_file, {"bbox": [10, 20, 80, 80]})
    write_status_json(acceptance_path, {
        "status": SIMPLE_SANDBOX_READY,
    })
    write_status_json(preflight_path, {
        "status": "EXTERNAL_WINDOW_READY_DRY",
        "metrics": {"window_title": "pr0p", "tracker_bbox": [10, 20, 80, 80]},
    })
    write_status_json(follow_path, {
        "status": "EXTERNAL_FOLLOW_DRY_RUN_READY",
        "metrics": {
            "window_title": "pr0p",
            "tracker_bbox": [10, 20, 80, 80],
            "loop_summary": {"found_ratio": 1.0, "loss_events": 0},
        },
    })
    return {
        "bbox_file": bbox_file,
        "latest_acceptance": acceptance_path,
        "matching_external_preflight": preflight_path,
        "matching_external_follow": follow_path,
    }


def readiness_ladder_by_name(report):
    return {
        stage["name"]: stage
        for stage in report.metrics["readiness_ladder"]
    }


def fake_sandbox_status_report(status, *, next_actions=None, resume_commands=None):
    return sandbox_status_report_module.SandboxStatusReport(
        run_id="nested-status",
        started_at="2026-07-06T00:00:00+0300",
        status=status,
        summary="nested status %s" % status,
        items=[],
        next_actions=next_actions or [],
        resume_commands=resume_commands or [],
        evidence_paths={"latest_acceptance": "acceptance.json"},
        metrics={
            "readiness_ladder": [
                {
                    "stage": "S1",
                    "name": "target_setup",
                    "status": status,
                    "unmet_requires_pass": (
                        ["target_window=WAITING"] if status == WAITING else []
                    ),
                    "unmet_requires_any_pass": [],
                },
            ],
            "freshness_gate": {
                "status": PASS if status == SANDBOX_STATUS_READY else WAITING,
                "required_keys": ["latest_acceptance"],
                "stale_keys": [],
                "missing_keys": [],
            },
            "first_blocking_stage": (
                {"stage": "S1", "name": "target_setup"}
                if status == WAITING else
                None
            ),
        },
    )


def fake_operator_preflight_report(
    *,
    candidate_action_plan,
    run_id="nested-operator-preflight",
):
    return external_operator_preflight_module.ExternalOperatorPreflightReport(
        run_id=run_id,
        started_at="2026-07-07T00:00:00+0300",
        status=WAITING,
        summary="nested operator preflight waiting",
        next_actions=["select the intended simulator candidate"],
        metrics={
            "candidate_action_plan": candidate_action_plan,
            "operator_preflight_decision": {
                "recommended_candidate_action": (
                    candidate_action_plan[0] if candidate_action_plan else None
                ),
            },
        },
    )


def test_status_report_evidence_freshness_marks_fresh_stale_and_missing(tmp_path):
    fresh = tmp_path / "fresh.json"
    stale = tmp_path / "stale.json"
    fresh.write_text("{}", encoding="utf-8")
    stale.write_text("{}", encoding="utf-8")
    os.utime(fresh, (90.0, 90.0))
    os.utime(stale, (10.0, 10.0))
    report = sandbox_status_report_module.build_evidence_freshness(
        {
            "fresh": str(fresh),
            "stale": str(stale),
            "missing": str(tmp_path / "missing.json"),
            "none": None,
        },
        now_s=100.0,
        stale_after_s=30.0,
    )
    assert report["fresh_keys"] == ["fresh"]
    assert report["stale_keys"] == ["stale"]
    assert report["missing_keys"] == ["missing", "none"]
    assert report["artifacts"]["fresh"]["freshness"] == "FRESH"
    assert report["artifacts"]["fresh"]["age_seconds"] == 10.0
    assert report["artifacts"]["stale"]["freshness"] == "STALE"
    assert report["artifacts"]["stale"]["age_seconds"] == 90.0
    assert report["artifacts"]["missing"]["freshness"] == "MISSING"
    assert report["artifacts"]["none"]["path"] is None


def test_external_target_acceptance_defaults_to_live_follow_and_fresh_evidence(
    monkeypatch,
    tmp_path,
):
    seen = {}

    def fake_build_status_report(**kwargs):
        seen.update(kwargs)
        return fake_sandbox_status_report(SANDBOX_STATUS_READY)

    monkeypatch.setattr(
        external_target_acceptance_module,
        "build_status_report",
        fake_build_status_report,
    )
    monkeypatch.setattr(
        external_target_acceptance_module,
        "write_status_reports",
        lambda _report, _log_dir: (
            tmp_path / "nested-status.json",
            tmp_path / "nested-status.md",
        ),
    )
    monkeypatch.setattr(
        external_target_acceptance_module,
        "process_snapshot",
        lambda: [],
    )
    report = run_external_acceptance(
        run_id="unit-external-acceptance-ready",
        log_dir=tmp_path,
        window_title="pr0p",
        bbox_file=tmp_path / "bbox.json",
        mode="live-follow",
        window_exact=False,
        window_case_sensitive=False,
        window_min_width=32,
        window_min_height=32,
        require_fresh_evidence=True,
        evidence_stale_after_s=3600.0,
        axis_sweep_axes=("yaw", "pitch"),
        axis_expected_shifts={"yaw": ("x", 1)},
        axis_min_shift_px=1.0,
    )
    assert report.status == EXTERNAL_TARGET_READY
    assert seen["require_live_input"] is False
    assert seen["require_live_follow"] is True
    assert seen["require_fresh_evidence"] is True
    assert seen["evidence_stale_after_s"] == 3600.0
    assert seen["axis_expected_shifts"] == {"yaw": ("x", 1)}
    assert report.metrics["acceptance_mode"] == "live-follow"
    assert report.metrics["require_live_follow"] is True
    assert report.metrics["require_fresh_evidence"] is True
    assert report.metrics["isolation_status"] == PASS
    decision = report.metrics["acceptance_decision"]
    assert decision["decision"] == EXTERNAL_TARGET_READY
    assert decision["next_operator_action"] is None
    assert decision["first_blocking_stage"] is None
    assert decision["freshness_gate_status"] == PASS
    assert report.metrics["operator_command_queue"] == []
    assert report.metrics["operator_command_summary"]["total_count"] == 0
    assert report.metrics["operator_command_summary"]["state_counts"] == {}


def test_external_target_acceptance_waits_with_next_actions(monkeypatch, tmp_path):
    monkeypatch.setattr(
        external_target_acceptance_module,
        "build_status_report",
        lambda **_kwargs: fake_sandbox_status_report(
            WAITING,
            next_actions=["open the external game/sim window"],
            resume_commands=[
                {
                    "name": "status",
                    "safety_class": "status_only",
                    "sends_uinput": False,
                    "command": "status command",
                },
                {
                    "name": "capture_bbox_frame",
                    "purpose": "capture one frame for manual bbox selection",
                    "safety_class": "capture_only",
                    "sends_uinput": False,
                    "requires_pass": ["target_window"],
                    "expected_status": WAITING,
                    "unblocks": ["target_bbox"],
                    "command": "bbox command",
                },
            ],
        ),
    )
    monkeypatch.setattr(
        external_target_acceptance_module,
        "write_status_reports",
        lambda _report, _log_dir: (
            tmp_path / "nested-status.json",
            tmp_path / "nested-status.md",
        ),
    )
    monkeypatch.setattr(
        external_target_acceptance_module,
        "process_snapshot",
        lambda: [],
    )
    report = run_external_acceptance(
        run_id="unit-external-acceptance-waiting",
        log_dir=tmp_path,
        window_title="pr0p",
        bbox_file=tmp_path / "bbox.json",
        mode="live-follow",
        window_exact=False,
        window_case_sensitive=False,
        window_min_width=32,
        window_min_height=32,
        require_fresh_evidence=True,
        evidence_stale_after_s=3600.0,
        axis_sweep_axes=("yaw", "pitch"),
        axis_expected_shifts=None,
        axis_min_shift_px=1.0,
    )
    assert report.status == WAITING
    assert report.next_actions == ["open the external game/sim window"]
    assert report.metrics["status_report"]["status"] == WAITING
    assert report.metrics["isolation_status"] == PASS
    decision = report.metrics["acceptance_decision"]
    assert decision["decision"] == WAITING
    assert decision["next_operator_action"] == "open the external game/sim window"
    assert decision["first_blocking_stage"]["name"] == "target_setup"
    assert decision["freshness_gate_status"] == WAITING
    assert decision["next_operator_command"]["name"] == "capture_bbox_frame"
    assert decision["next_operator_command"]["command_state"] == "BLOCKED"
    assert decision["first_blocked_command"]["name"] == "capture_bbox_frame"
    assert decision["first_available_command"] is None
    assert decision["first_ack_required_command"] is None
    queue = report.metrics["operator_command_queue"]
    assert len(queue) == 1
    assert queue[0]["name"] == "capture_bbox_frame"
    assert queue[0]["command_state"] == "BLOCKED"
    assert queue[0]["safety_class"] == "capture_only"
    assert queue[0]["sends_uinput"] is False
    assert queue[0]["requires_pass"] == ["target_window"]
    assert queue[0]["unmet_requires_pass"] == ["target_window=UNKNOWN"]
    assert queue[0]["command"] == "bbox command"
    summary = report.metrics["operator_command_summary"]
    assert summary["total_count"] == 1
    assert summary["state_counts"] == {"BLOCKED": 1}
    assert summary["sends_uinput_count"] == 0
    assert summary["first_blocked_command"]["name"] == "capture_bbox_frame"
    assert summary["first_available_command"] is None
    assert summary["first_ack_required_command"] is None


def test_operator_preflight_promotes_ready_status(monkeypatch, tmp_path):
    seen = {}

    def fake_build_status_report(**kwargs):
        seen.update(kwargs)
        return fake_sandbox_status_report(SANDBOX_STATUS_READY)

    monkeypatch.setattr(
        external_operator_preflight_module,
        "build_status_report",
        fake_build_status_report,
    )
    monkeypatch.setattr(
        external_operator_preflight_module,
        "write_status_reports",
        lambda _report, _log_dir: (
            tmp_path / "nested-status.json",
            tmp_path / "nested-status.md",
        ),
    )
    report = run_operator_preflight(
        run_id="unit-operator-preflight-ready",
        log_dir=tmp_path,
        window_title="pr0p",
        bbox_file=tmp_path / "bbox.json",
        window_exact=False,
        window_case_sensitive=False,
        window_min_width=32,
        window_min_height=32,
        require_live_input=False,
        require_live_follow=True,
        require_fresh_evidence=True,
        evidence_stale_after_s=3600.0,
        axis_sweep_axes=("yaw", "pitch"),
        axis_expected_shifts={"yaw": ("x", 1)},
        axis_min_shift_px=1.0,
    )
    assert report.status == OPERATOR_PREFLIGHT_READY
    assert seen["require_live_follow"] is True
    assert seen["require_fresh_evidence"] is True
    assert seen["axis_expected_shifts"] == {"yaw": ("x", 1)}
    decision = report.metrics["operator_preflight_decision"]
    assert decision["decision"] == OPERATOR_PREFLIGHT_READY
    assert decision["readiness_stage"] == "ready"
    assert decision["freshness_gate_status"] == PASS
    assert decision["required_fresh_evidence"] == ["latest_acceptance"]
    assert decision["stale_evidence"] == []
    assert decision["missing_evidence"] == []
    assert decision["command_gate"] == "no_operator_command"
    assert decision["recommended_command"] is None
    assert report.metrics["operator_command_summary"]["total_count"] == 0


def test_operator_preflight_reports_blocked_next_command(monkeypatch, tmp_path):
    monkeypatch.setattr(
        external_operator_preflight_module,
        "build_status_report",
        lambda **_kwargs: fake_sandbox_status_report(
            WAITING,
            next_actions=["open the external game/sim window"],
            resume_commands=[
                {
                    "name": "status",
                    "safety_class": "status_only",
                    "sends_uinput": False,
                    "command": "status command",
                },
                {
                    "name": "capture_bbox_frame",
                    "purpose": "capture one frame for manual bbox selection",
                    "safety_class": "capture_only",
                    "sends_uinput": False,
                    "requires_pass": ["target_window"],
                    "expected_status": WAITING,
                    "unblocks": ["target_bbox"],
                    "command": "bbox command",
                },
            ],
        ),
    )
    monkeypatch.setattr(
        external_operator_preflight_module,
        "write_status_reports",
        lambda _report, _log_dir: (
            tmp_path / "nested-status.json",
            tmp_path / "nested-status.md",
        ),
    )
    report = run_operator_preflight(
        run_id="unit-operator-preflight-waiting",
        log_dir=tmp_path,
        window_title="pr0p",
        bbox_file=tmp_path / "bbox.json",
        window_exact=False,
        window_case_sensitive=False,
        window_min_width=32,
        window_min_height=32,
        require_live_input=False,
        require_live_follow=True,
        require_fresh_evidence=True,
        evidence_stale_after_s=3600.0,
        axis_sweep_axes=("yaw", "pitch"),
        axis_expected_shifts=None,
        axis_min_shift_px=1.0,
    )
    assert report.status == WAITING
    assert report.next_actions == ["open the external game/sim window"]
    decision = report.metrics["operator_preflight_decision"]
    assert decision["decision"] == WAITING
    assert decision["readiness_stage"] == "target_setup"
    assert decision["freshness_gate_status"] == WAITING
    assert decision["command_gate"] == "blocked_by_dependency"
    assert decision["next_operator_action"] == "open the external game/sim window"
    assert decision["recommended_command"]["name"] == "capture_bbox_frame"
    assert decision["recommended_command"]["unmet_requires_pass"] == [
        "target_window=UNKNOWN",
    ]
    summary = report.metrics["operator_command_summary"]
    assert summary["total_count"] == 1
    assert summary["state_counts"] == {"BLOCKED": 1}
    assert summary["first_blocked_command"]["name"] == "capture_bbox_frame"


def test_operator_preflight_reports_window_title_candidates(monkeypatch, tmp_path):
    monkeypatch.setenv("DISPLAY", ":99")
    monkeypatch.setattr(
        external_operator_preflight_module,
        "list_x11_windows",
        lambda: [
            X11Window("0x1", "Visual Studio Code", "code", 900, 700, 0, 0, 0, 0),
            X11Window(
                "0x2",
                "SITL Forge - FPV Laboratory",
                "game",
                1280,
                720,
                0,
                0,
                10,
                20,
            ),
            X11Window("0x3", "tiny utility", "tiny", 20, 20, 0, 0, 0, 0),
        ],
    )
    monkeypatch.setattr(
        external_operator_preflight_module,
        "build_status_report",
        lambda **_kwargs: fake_sandbox_status_report(
            WAITING,
            next_actions=["open the external game/sim window"],
            resume_commands=[],
        ),
    )
    monkeypatch.setattr(
        external_operator_preflight_module,
        "write_status_reports",
        lambda _report, _log_dir: (
            tmp_path / "nested-status.json",
            tmp_path / "nested-status.md",
        ),
    )
    report = run_operator_preflight(
        run_id="unit-operator-preflight-window-candidates",
        log_dir=tmp_path,
        window_title="pr0p",
        bbox_file=tmp_path / "bbox.json",
        window_exact=False,
        window_case_sensitive=False,
        window_min_width=32,
        window_min_height=32,
        require_live_input=False,
        require_live_follow=True,
        require_fresh_evidence=True,
        evidence_stale_after_s=3600.0,
        max_window_candidates=2,
        axis_sweep_axes=("yaw", "pitch"),
        axis_expected_shifts=None,
        axis_min_shift_px=1.0,
    )
    discovery = report.metrics["window_discovery"]
    assert discovery["reason"] == "NO_MATCH"
    assert discovery["visible_count"] == 2
    assert discovery["basic_candidate_pool_count"] == 2
    assert discovery["candidate_pool_count"] == 1
    assert discovery["excluded_candidate_count"] == 1
    assert discovery["candidate_count"] == 1
    assert discovery["excluded_candidates"][0]["title"] == "Visual Studio Code"
    assert discovery["excluded_candidates"][0]["rejection_reason"] == (
        "candidate_window_is_tooling:0x1"
    )
    assert [
        candidate["title"] for candidate in discovery["candidates"]
    ] == [
        "SITL Forge - FPV Laboratory",
    ]
    first_candidate = discovery["candidates"][0]
    assert first_candidate["use_window_title_arg"] == "SITL Forge - FPV Laboratory"
    assert "--window-exact" in first_candidate["rerun_operator_preflight_command"]
    assert (
        "--window-title 'SITL Forge - FPV Laboratory'"
        in first_candidate["rerun_operator_preflight_command"]
    )
    assert (
        "--window-title 'SITL Forge - FPV Laboratory'"
        in first_candidate["capture_bbox_frame_command"]
    )
    assert (
        "--region 10,20,1280,720"
        in first_candidate["capture_bbox_frame_region_command"]
    )
    assert "--window-title" not in first_candidate["capture_bbox_frame_region_command"]
    assert (
        "--region 10,20,1280,720"
        in first_candidate["write_bbox_file_region_command"]
    )
    assert (
        "--bbox X,Y,W,H"
        in first_candidate["write_bbox_file_region_command"]
    )
    assert (
        "--report-path %s" % (tmp_path / "bbox.json")
        in first_candidate["write_bbox_file_region_command"]
    )
    assert (
        "--interactive-select"
        in first_candidate["interactive_bbox_select_region_command"]
    )
    assert (
        "experiments/game_screen_sandbox/external_dry_run_sequence.py"
        in first_candidate["dry_run_sequence_region_command"]
    )
    assert (
        "--capture-region 10,20,1280,720"
        in first_candidate["dry_run_sequence_region_command"]
    )
    assert (
        "--capture-window-title 'SITL Forge - FPV Laboratory'"
        in first_candidate["dry_run_sequence_region_command"]
    )
    action_plan = report.metrics["candidate_action_plan"]
    assert action_plan[0]["candidate_index"] == 1
    assert action_plan[0]["title"] == "SITL Forge - FPV Laboratory"
    assert action_plan[0]["next_step"] == "capture_bbox_frame_region"
    assert action_plan[0]["next_command"] == first_candidate[
        "capture_bbox_frame_region_command"
    ]
    assert action_plan[0]["write_bbox_file_region_command"] == (
        first_candidate["write_bbox_file_region_command"]
    )
    assert action_plan[0]["interactive_bbox_select_region_command"] == (
        first_candidate["interactive_bbox_select_region_command"]
    )
    assert action_plan[0]["sends_uinput"] is False
    assert action_plan[0]["requires_user_judgement"] is True
    decision = report.metrics["operator_preflight_decision"]
    assert decision["window_discovery_reason"] == "NO_MATCH"
    assert decision["candidate_action_gate"] == "manual_candidate_selection"
    assert (
        decision["recommended_candidate_action"]["next_step"]
        == "capture_bbox_frame_region"
    )
    assert decision["window_title_candidates"] == [
        "SITL Forge - FPV Laboratory",
    ]
    assert decision["excluded_window_candidates"] == [
        {
            "title": "Visual Studio Code",
            "window_id": "0x1",
            "rejection_reason": "candidate_window_is_tooling:0x1",
        },
    ]


def test_operator_preflight_excludes_tooling_title_match_from_candidate_plan(
    monkeypatch,
    tmp_path,
):
    monkeypatch.setenv("DISPLAY", ":99")
    monkeypatch.setattr(
        external_operator_preflight_module,
        "list_x11_windows",
        lambda: [
            X11Window(
                "0x2c00004",
                "Compare SITL Forge v1 - fpv-test - Visual Studio Code",
                "code",
                1920,
                1080,
                0,
                0,
                0,
                0,
            ),
        ],
    )
    monkeypatch.setattr(
        external_operator_preflight_module,
        "build_status_report",
        lambda **_kwargs: fake_sandbox_status_report(
            WAITING,
            next_actions=["open the external game/sim window"],
            resume_commands=[],
        ),
    )
    monkeypatch.setattr(
        external_operator_preflight_module,
        "write_status_reports",
        lambda _report, _log_dir: (
            tmp_path / "nested-status.json",
            tmp_path / "nested-status.md",
        ),
    )
    report = run_operator_preflight(
        run_id="unit-operator-preflight-tooling-window-match",
        log_dir=tmp_path,
        window_title="SITL Forge",
        bbox_file=tmp_path / "bbox.json",
        window_exact=False,
        window_case_sensitive=False,
        window_min_width=32,
        window_min_height=32,
        require_live_input=False,
        require_live_follow=True,
        require_fresh_evidence=True,
        evidence_stale_after_s=3600.0,
        max_window_candidates=2,
        axis_sweep_axes=("yaw", "pitch"),
        axis_expected_shifts=None,
        axis_min_shift_px=1.0,
    )
    discovery = report.metrics["window_discovery"]
    assert discovery["reason"] == "NO_MATCH"
    assert discovery["visible_count"] == 1
    assert discovery["basic_candidate_pool_count"] == 1
    assert discovery["candidate_pool_count"] == 0
    assert discovery["excluded_candidate_count"] == 1
    assert discovery["candidates"] == []
    assert discovery["excluded_candidates"][0]["rejection_reason"] == (
        "candidate_window_is_tooling:0x2c00004"
    )
    assert report.metrics["candidate_action_plan"] == []
    decision = report.metrics["operator_preflight_decision"]
    assert decision["candidate_action_gate"] == "no_candidate_action"
    assert decision["recommended_candidate_action"] is None
    assert decision["window_title_candidates"] == []
    assert decision["excluded_window_candidates"] == [
        {
            "title": "Compare SITL Forge v1 - fpv-test - Visual Studio Code",
            "window_id": "0x2c00004",
            "rejection_reason": "candidate_window_is_tooling:0x2c00004",
        },
    ]


def test_operator_preflight_excludes_shell_title_from_candidate_plan(
    monkeypatch,
    tmp_path,
):
    monkeypatch.setenv("DISPLAY", ":99")
    monkeypatch.setattr(
        external_operator_preflight_module,
        "list_x11_windows",
        lambda: [
            X11Window(
                "0x6000af",
                "gz@gz: ~",
                "",
                1854,
                1048,
                0,
                0,
                66,
                32,
            ),
        ],
    )
    monkeypatch.setattr(
        external_operator_preflight_module,
        "build_status_report",
        lambda **_kwargs: fake_sandbox_status_report(
            WAITING,
            next_actions=["open the external game/sim window"],
            resume_commands=[],
        ),
    )
    monkeypatch.setattr(
        external_operator_preflight_module,
        "write_status_reports",
        lambda _report, _log_dir: (
            tmp_path / "nested-status.json",
            tmp_path / "nested-status.md",
        ),
    )
    report = run_operator_preflight(
        run_id="unit-operator-preflight-shell-window",
        log_dir=tmp_path,
        window_title="pr0p",
        bbox_file=tmp_path / "bbox.json",
        window_exact=False,
        window_case_sensitive=False,
        window_min_width=32,
        window_min_height=32,
        require_live_input=False,
        require_live_follow=True,
        require_fresh_evidence=True,
        evidence_stale_after_s=3600.0,
        max_window_candidates=2,
        axis_sweep_axes=("yaw", "pitch"),
        axis_expected_shifts=None,
        axis_min_shift_px=1.0,
    )
    discovery = report.metrics["window_discovery"]
    assert discovery["candidate_count"] == 0
    assert discovery["excluded_candidate_count"] == 1
    assert discovery["excluded_candidates"][0]["title"] == "gz@gz: ~"
    assert discovery["excluded_candidates"][0]["rejection_reason"] == (
        "candidate_window_is_tooling:0x6000af"
    )
    assert report.metrics["candidate_action_plan"] == []
    decision = report.metrics["operator_preflight_decision"]
    assert decision["candidate_action_gate"] == "no_candidate_action"
    assert decision["recommended_candidate_action"] is None


def test_operator_preflight_candidate_action_plan_uses_region_dry_run_after_bbox(
    monkeypatch,
    tmp_path,
):
    monkeypatch.setenv("DISPLAY", ":99")
    monkeypatch.setattr(
        external_operator_preflight_module,
        "list_x11_windows",
        lambda: [
            X11Window(
                "0x2",
                "SITL Forge - FPV Laboratory",
                "game",
                1280,
                720,
                0,
                0,
                10,
                20,
            ),
        ],
    )

    def status_with_bbox(**_kwargs):
        return sandbox_status_report_module.SandboxStatusReport(
            run_id="nested-status",
            started_at="2026-07-07T00:00:00+0300",
            status=WAITING,
            summary="nested status waiting",
            items=[
                sandbox_status_report_module.StatusItem(
                    "target_window", WAITING, "title missing",
                ),
                sandbox_status_report_module.StatusItem(
                    "target_bbox", PASS, "bbox ready",
                ),
                sandbox_status_report_module.StatusItem(
                    "gazebo_betaflight_process_boundary", PASS, "clean",
                ),
                sandbox_status_report_module.StatusItem(
                    "external_window_preflight", WAITING, "preflight missing",
                ),
                sandbox_status_report_module.StatusItem(
                    "external_follow_session", WAITING, "follow missing",
                ),
            ],
            next_actions=["run external dry-run sequence"],
            resume_commands=[],
            evidence_paths={},
            metrics={
                "first_blocking_stage": {
                    "stage": "S2",
                    "name": "external_dry_run",
                },
                "freshness_gate": {"status": WAITING},
            },
        )

    monkeypatch.setattr(
        external_operator_preflight_module,
        "build_status_report",
        status_with_bbox,
    )
    monkeypatch.setattr(
        external_operator_preflight_module,
        "write_status_reports",
        lambda _report, _log_dir: (
            tmp_path / "nested-status.json",
            tmp_path / "nested-status.md",
        ),
    )
    report = run_operator_preflight(
        run_id="unit-operator-preflight-region-dry-run-action",
        log_dir=tmp_path,
        window_title="pr0p",
        bbox_file=tmp_path / "bbox.json",
        window_exact=False,
        window_case_sensitive=False,
        window_min_width=32,
        window_min_height=32,
        require_live_input=False,
        require_live_follow=True,
        require_fresh_evidence=True,
        evidence_stale_after_s=3600.0,
        max_window_candidates=1,
        axis_sweep_axes=("yaw", "pitch"),
        axis_expected_shifts=None,
        axis_min_shift_px=1.0,
    )
    action = report.metrics["candidate_action_plan"][0]
    assert action["next_step"] == "dry_run_sequence_region"
    assert action["expected_status"] == EXTERNAL_DRY_RUN_READY
    assert action["safety_class"] == "dry_run_capture"
    assert action["sends_uinput"] is False
    assert "--capture-region 10,20,1280,720" in action["next_command"]
    assert (
        "experiments/game_screen_sandbox/external_dry_run_sequence.py"
        in action["next_command"]
    )
    decision = report.metrics["operator_preflight_decision"]
    assert (
        decision["recommended_candidate_action"]["next_step"]
        == "dry_run_sequence_region"
    )


def test_candidate_action_runner_waits_for_candidate_index(monkeypatch, tmp_path):
    action = {
        "candidate_index": 1,
        "title": "SITL Forge - FPV Laboratory",
        "window_id": "0x2",
        "next_step": "capture_bbox_frame_region",
        "next_command": "bbox region command",
        "expected_status": WAITING,
        "safety_class": "capture_only",
        "sends_uinput": False,
        "requires_user_judgement": True,
        "unblocks": ["target_bbox"],
    }
    calls = []
    monkeypatch.setattr(
        external_candidate_action_runner_module,
        "run_operator_preflight",
        lambda **_kwargs: fake_operator_preflight_report(
            candidate_action_plan=[action],
        ),
    )
    monkeypatch.setattr(
        external_candidate_action_runner_module,
        "write_operator_reports",
        lambda _report, _log_dir: (
            tmp_path / "nested-operator.json",
            tmp_path / "nested-operator.md",
        ),
    )
    monkeypatch.setattr(
        external_candidate_action_runner_module,
        "run_shell_command",
        lambda command, *, timeout_s: calls.append(command) or CommandRunResult(0),
    )
    report = run_candidate_action(
        run_id="unit-candidate-action-no-index",
        log_dir=tmp_path,
        window_title="pr0p",
        bbox_file=tmp_path / "bbox.json",
        window_exact=False,
        window_case_sensitive=False,
        window_min_width=32,
        window_min_height=32,
        require_live_input=False,
        require_live_follow=True,
        require_fresh_evidence=True,
        evidence_stale_after_s=3600.0,
        max_window_candidates=1,
        axis_sweep_axes=("yaw", "pitch"),
        axis_expected_shifts=None,
        axis_min_shift_px=1.0,
        candidate_index=None,
        execute_safe=True,
        ack_candidate=True,
        timeout_s=5.0,
    )
    assert report.status == WAITING
    assert calls == []
    step = report.steps[0]
    assert step["status"] == WAITING
    assert step["reasons"] == ["candidate_index_required"]
    assert report.metrics["executed_count"] == 0
    assert report.metrics["post_operator_preflight_report"] is None
    assert report.evidence_paths["post_operator_preflight_json"] is None


def test_candidate_action_runner_requires_candidate_ack(monkeypatch, tmp_path):
    action = {
        "candidate_index": 1,
        "title": "SITL Forge - FPV Laboratory",
        "window_id": "0x2",
        "next_step": "capture_bbox_frame_region",
        "next_command": "bbox region command",
        "expected_status": WAITING,
        "safety_class": "capture_only",
        "sends_uinput": False,
        "requires_user_judgement": True,
        "unblocks": ["target_bbox"],
    }
    calls = []
    monkeypatch.setattr(
        external_candidate_action_runner_module,
        "run_operator_preflight",
        lambda **_kwargs: fake_operator_preflight_report(
            candidate_action_plan=[action],
        ),
    )
    monkeypatch.setattr(
        external_candidate_action_runner_module,
        "write_operator_reports",
        lambda _report, _log_dir: (
            tmp_path / "nested-operator.json",
            tmp_path / "nested-operator.md",
        ),
    )
    monkeypatch.setattr(
        external_candidate_action_runner_module,
        "run_shell_command",
        lambda command, *, timeout_s: calls.append(command) or CommandRunResult(0),
    )
    report = run_candidate_action(
        run_id="unit-candidate-action-no-ack",
        log_dir=tmp_path,
        window_title="pr0p",
        bbox_file=tmp_path / "bbox.json",
        window_exact=False,
        window_case_sensitive=False,
        window_min_width=32,
        window_min_height=32,
        require_live_input=False,
        require_live_follow=True,
        require_fresh_evidence=True,
        evidence_stale_after_s=3600.0,
        max_window_candidates=1,
        axis_sweep_axes=("yaw", "pitch"),
        axis_expected_shifts=None,
        axis_min_shift_px=1.0,
        candidate_index=1,
        execute_safe=True,
        ack_candidate=False,
        timeout_s=5.0,
    )
    assert report.status == WAITING
    assert calls == []
    step = report.steps[0]
    assert step["status"] == BLOCKED
    assert "--ack-candidate" in step["summary"]
    assert report.metrics["post_operator_preflight_report"] is None
    assert report.evidence_paths["post_operator_preflight_json"] is None


def test_candidate_action_runner_selects_candidate_by_window_id(
    monkeypatch,
    tmp_path,
):
    actions = [
        {
            "candidate_index": 1,
            "title": "Visual Studio Code",
            "window_id": "0x1",
            "next_step": "capture_bbox_frame_region",
            "next_command": "wrong window command",
            "expected_status": WAITING,
            "safety_class": "capture_only",
            "sends_uinput": False,
            "requires_user_judgement": True,
            "unblocks": ["target_bbox"],
        },
        {
            "candidate_index": 2,
            "title": "SITL Forge - FPV Laboratory",
            "window_id": "0x2",
            "next_step": "capture_bbox_frame_region",
            "next_command": "sim window command",
            "expected_status": WAITING,
            "safety_class": "capture_only",
            "sends_uinput": False,
            "requires_user_judgement": True,
            "unblocks": ["target_bbox"],
        },
    ]
    calls = []
    monkeypatch.setattr(
        external_candidate_action_runner_module,
        "run_operator_preflight",
        lambda **_kwargs: fake_operator_preflight_report(
            candidate_action_plan=actions,
        ),
    )
    monkeypatch.setattr(
        external_candidate_action_runner_module,
        "write_operator_reports",
        lambda _report, _log_dir: (
            tmp_path / "nested-operator.json",
            tmp_path / "nested-operator.md",
        ),
    )
    monkeypatch.setattr(
        external_candidate_action_runner_module,
        "run_shell_command",
        lambda command, *, timeout_s: calls.append(command) or CommandRunResult(0),
    )
    report = run_candidate_action(
        run_id="unit-candidate-action-window-id",
        log_dir=tmp_path,
        window_title="pr0p",
        bbox_file=tmp_path / "bbox.json",
        window_exact=False,
        window_case_sensitive=False,
        window_min_width=32,
        window_min_height=32,
        require_live_input=False,
        require_live_follow=True,
        require_fresh_evidence=True,
        evidence_stale_after_s=3600.0,
        max_window_candidates=2,
        axis_sweep_axes=("yaw", "pitch"),
        axis_expected_shifts=None,
        axis_min_shift_px=1.0,
        candidate_window_id="0x2",
        execute_safe=False,
        ack_candidate=True,
        timeout_s=5.0,
    )
    assert report.status == WAITING
    assert calls == []
    step = report.steps[0]
    assert step["candidate_index"] == 2
    assert step["title"] == "SITL Forge - FPV Laboratory"
    assert step["command"] == "sim window command"
    assert step["status"] == PLANNED
    assert report.metrics["candidate_action_decision"]["selector"] == (
        "candidate_window_id"
    )
    assert report.metrics["candidate_action_decision"]["selector_value"] == "0x2"
    assert report.metrics["candidate_action_decision"]["selected_candidate_index"] == 2


def test_candidate_action_runner_writes_region_bbox_for_selected_candidate(
    monkeypatch,
    tmp_path,
):
    actions = [
        {
            "candidate_index": 1,
            "title": "SITL Forge - FPV Laboratory",
            "window_id": "0x2",
            "region": {"left": 10, "top": 20, "width": 1280, "height": 720},
            "next_step": "capture_bbox_frame_region",
            "next_command": "bbox region command",
            "write_bbox_file_region_command": (
                "fpv_env/bin/python experiments/game_screen_sandbox/bbox_tool.py "
                "--region 10,20,1280,720 --bbox X,Y,W,H "
                "--report-path %s"
            ) % (tmp_path / "bbox.json"),
            "expected_status": WAITING,
            "safety_class": "capture_only",
            "sends_uinput": False,
            "requires_user_judgement": True,
            "unblocks": ["target_bbox"],
        },
    ]
    calls = []
    preflight_run_ids = []

    def fake_run_operator_preflight(**kwargs):
        preflight_run_ids.append(kwargs["run_id"])
        return fake_operator_preflight_report(
            candidate_action_plan=actions,
            run_id=kwargs["run_id"],
        )

    monkeypatch.setattr(
        external_candidate_action_runner_module,
        "run_operator_preflight",
        fake_run_operator_preflight,
    )
    monkeypatch.setattr(
        external_candidate_action_runner_module,
        "write_operator_reports",
        lambda report, _log_dir: (
            tmp_path / ("%s.json" % report.run_id),
            tmp_path / ("%s.md" % report.run_id),
        ),
    )
    monkeypatch.setattr(
        external_candidate_action_runner_module,
        "run_shell_command",
        lambda command, *, timeout_s: (
            calls.append((command, timeout_s))
            or CommandRunResult(returncode=0, stdout="bbox PASS\n")
        ),
    )
    report = run_candidate_action(
        run_id="unit-candidate-action-write-bbox",
        log_dir=tmp_path,
        window_title="pr0p",
        bbox_file=tmp_path / "bbox.json",
        window_exact=False,
        window_case_sensitive=False,
        window_min_width=32,
        window_min_height=32,
        require_live_input=False,
        require_live_follow=True,
        require_fresh_evidence=True,
        evidence_stale_after_s=3600.0,
        max_window_candidates=1,
        axis_sweep_axes=("yaw", "pitch"),
        axis_expected_shifts=None,
        axis_min_shift_px=1.0,
        candidate_window_id="0x2",
        candidate_bbox=(100, 120, 220, 140),
        execute_safe=True,
        ack_candidate=True,
        timeout_s=5.0,
    )
    assert report.status == OPERATOR_CANDIDATE_ACTION_PROGRESS
    assert len(calls) == 1
    assert "--bbox 100,120,220,140" in calls[0][0]
    assert "X,Y,W,H" not in calls[0][0]
    step = report.steps[0]
    assert step["next_step"] == "write_bbox_file_region"
    assert step["candidate_bbox"] == [100, 120, 220, 140]
    assert step["expected_status"] == PASS
    assert step["status"] == PASS
    assert report.metrics["candidate_bbox"] == [100, 120, 220, 140]
    assert preflight_run_ids == [
        "unit-candidate-action-write-bbox-operator-preflight",
        "unit-candidate-action-write-bbox-post-operator-preflight",
    ]


def test_candidate_action_runner_rejects_candidate_bbox_outside_region(
    monkeypatch,
    tmp_path,
):
    actions = [
        {
            "candidate_index": 1,
            "title": "SITL Forge - FPV Laboratory",
            "window_id": "0x2",
            "region": {"left": 10, "top": 20, "width": 320, "height": 240},
            "next_step": "capture_bbox_frame_region",
            "next_command": "bbox region command",
            "write_bbox_file_region_command": (
                "fpv_env/bin/python experiments/game_screen_sandbox/bbox_tool.py "
                "--region 10,20,320,240 --bbox X,Y,W,H "
                "--report-path %s"
            ) % (tmp_path / "bbox.json"),
            "expected_status": WAITING,
            "safety_class": "capture_only",
            "sends_uinput": False,
            "requires_user_judgement": True,
            "unblocks": ["target_bbox"],
        },
    ]
    calls = []

    monkeypatch.setattr(
        external_candidate_action_runner_module,
        "run_operator_preflight",
        lambda **kwargs: fake_operator_preflight_report(
            candidate_action_plan=actions,
            run_id=kwargs["run_id"],
        ),
    )
    monkeypatch.setattr(
        external_candidate_action_runner_module,
        "write_operator_reports",
        lambda report, _log_dir: (
            tmp_path / ("%s.json" % report.run_id),
            tmp_path / ("%s.md" % report.run_id),
        ),
    )
    monkeypatch.setattr(
        external_candidate_action_runner_module,
        "run_shell_command",
        lambda command, *, timeout_s: (
            calls.append((command, timeout_s)) or CommandRunResult(returncode=0)
        ),
    )
    report = run_candidate_action(
        run_id="unit-candidate-action-bbox-outside-region",
        log_dir=tmp_path,
        window_title="pr0p",
        bbox_file=tmp_path / "bbox.json",
        window_exact=False,
        window_case_sensitive=False,
        window_min_width=32,
        window_min_height=32,
        require_live_input=False,
        require_live_follow=True,
        require_fresh_evidence=True,
        evidence_stale_after_s=3600.0,
        max_window_candidates=1,
        axis_sweep_axes=("yaw", "pitch"),
        axis_expected_shifts=None,
        axis_min_shift_px=1.0,
        candidate_window_id="0x2",
        candidate_bbox=(300, 200, 80, 60),
        execute_safe=True,
        ack_candidate=True,
        timeout_s=5.0,
    )
    assert report.status == REJECT
    assert calls == []
    step = report.steps[0]
    assert step["status"] == REJECT
    assert step["summary"] == "candidate bbox is outside the selected candidate region"
    assert step["candidate_index"] == 1
    assert step["title"] == "SITL Forge - FPV Laboratory"
    assert step["window_id"] == "0x2"
    assert step["command"] is None
    assert step["reasons"] == ["candidate_bbox_outside_region:300,200,80,60"]
    decision = report.metrics["candidate_action_decision"]
    assert decision["selected_candidate_index"] == 1
    assert decision["selected_title"] == "SITL Forge - FPV Laboratory"
    assert decision["reasons"] == ["candidate_bbox_outside_region:300,200,80,60"]
    assert decision["post_operator_status"] is None
    assert report.metrics["post_operator_preflight_report"] is None


def test_candidate_action_runner_rejects_tooling_window_title_match(
    monkeypatch,
    tmp_path,
):
    actions = [
        {
            "candidate_index": 1,
            "title": "Compare SITL Forge v1 - fpv-test - Visual Studio Code",
            "class_text": '("code" "code")',
            "window_id": "0x2c00004",
            "region": {"left": 0, "top": 0, "width": 1920, "height": 1080},
            "next_step": "capture_bbox_frame_region",
            "next_command": "wrong editor capture command",
            "write_bbox_file_region_command": (
                "fpv_env/bin/python experiments/game_screen_sandbox/bbox_tool.py "
                "--region 0,0,1920,1080 --bbox X,Y,W,H "
                "--report-path %s"
            ) % (tmp_path / "bbox.json"),
            "expected_status": WAITING,
            "safety_class": "capture_only",
            "sends_uinput": False,
            "requires_user_judgement": True,
            "unblocks": ["target_bbox"],
        },
    ]
    calls = []

    monkeypatch.setattr(
        external_candidate_action_runner_module,
        "run_operator_preflight",
        lambda **kwargs: fake_operator_preflight_report(
            candidate_action_plan=actions,
            run_id=kwargs["run_id"],
        ),
    )
    monkeypatch.setattr(
        external_candidate_action_runner_module,
        "write_operator_reports",
        lambda report, _log_dir: (
            tmp_path / ("%s.json" % report.run_id),
            tmp_path / ("%s.md" % report.run_id),
        ),
    )
    monkeypatch.setattr(
        external_candidate_action_runner_module,
        "run_shell_command",
        lambda command, *, timeout_s: (
            calls.append((command, timeout_s)) or CommandRunResult(returncode=0)
        ),
    )
    report = run_candidate_action(
        run_id="unit-candidate-action-tooling-window",
        log_dir=tmp_path,
        window_title="pr0p",
        bbox_file=tmp_path / "bbox.json",
        window_exact=False,
        window_case_sensitive=False,
        window_min_width=32,
        window_min_height=32,
        require_live_input=False,
        require_live_follow=True,
        require_fresh_evidence=True,
        evidence_stale_after_s=3600.0,
        max_window_candidates=1,
        axis_sweep_axes=("yaw", "pitch"),
        axis_expected_shifts=None,
        axis_min_shift_px=1.0,
        candidate_title_contains="SITL Forge",
        candidate_bbox=(100, 100, 80, 60),
        execute_safe=True,
        ack_candidate=True,
        timeout_s=5.0,
    )
    assert report.status == REJECT
    assert calls == []
    step = report.steps[0]
    assert step["candidate_index"] == 1
    assert step["window_id"] == "0x2c00004"
    assert step["command"] is None
    assert step["summary"] == (
        "selected candidate is a tooling/editor window, not a simulator view"
    )
    assert step["reasons"] == ["candidate_window_is_tooling:0x2c00004"]
    decision = report.metrics["candidate_action_decision"]
    assert decision["selected_candidate_index"] == 1
    assert decision["reasons"] == ["candidate_window_is_tooling:0x2c00004"]
    assert report.metrics["post_operator_preflight_report"] is None


def test_candidate_action_runner_rejects_ambiguous_title_contains(
    monkeypatch,
    tmp_path,
):
    actions = [
        {
            "candidate_index": 1,
            "title": "SITL Forge - FPV Laboratory",
            "window_id": "0x1",
            "next_step": "capture_bbox_frame_region",
            "next_command": "first command",
            "expected_status": WAITING,
            "safety_class": "capture_only",
            "sends_uinput": False,
            "requires_user_judgement": True,
            "unblocks": ["target_bbox"],
        },
        {
            "candidate_index": 2,
            "title": "SITL Forge - External FC Betaflight",
            "window_id": "0x2",
            "next_step": "capture_bbox_frame_region",
            "next_command": "second command",
            "expected_status": WAITING,
            "safety_class": "capture_only",
            "sends_uinput": False,
            "requires_user_judgement": True,
            "unblocks": ["target_bbox"],
        },
    ]
    calls = []
    monkeypatch.setattr(
        external_candidate_action_runner_module,
        "run_operator_preflight",
        lambda **_kwargs: fake_operator_preflight_report(
            candidate_action_plan=actions,
        ),
    )
    monkeypatch.setattr(
        external_candidate_action_runner_module,
        "write_operator_reports",
        lambda _report, _log_dir: (
            tmp_path / "nested-operator.json",
            tmp_path / "nested-operator.md",
        ),
    )
    monkeypatch.setattr(
        external_candidate_action_runner_module,
        "run_shell_command",
        lambda command, *, timeout_s: calls.append(command) or CommandRunResult(0),
    )
    report = run_candidate_action(
        run_id="unit-candidate-action-ambiguous-title",
        log_dir=tmp_path,
        window_title="pr0p",
        bbox_file=tmp_path / "bbox.json",
        window_exact=False,
        window_case_sensitive=False,
        window_min_width=32,
        window_min_height=32,
        require_live_input=False,
        require_live_follow=True,
        require_fresh_evidence=True,
        evidence_stale_after_s=3600.0,
        max_window_candidates=2,
        axis_sweep_axes=("yaw", "pitch"),
        axis_expected_shifts=None,
        axis_min_shift_px=1.0,
        candidate_title_contains="SITL Forge",
        execute_safe=True,
        ack_candidate=True,
        timeout_s=5.0,
    )
    assert report.status == REJECT
    assert calls == []
    step = report.steps[0]
    assert step["status"] == REJECT
    assert step["summary"] == "candidate title substring matched multiple candidates"
    assert step["reasons"] == ["candidate_title_contains_ambiguous:SITL Forge:2"]


def test_candidate_action_runner_executes_selected_safe_candidate(
    monkeypatch,
    tmp_path,
):
    action = {
        "candidate_index": 2,
        "title": "SITL Forge - FPV Laboratory",
        "window_id": "0x2",
        "next_step": "dry_run_sequence_region",
        "next_command": "dry sequence region command",
        "expected_status": EXTERNAL_DRY_RUN_READY,
        "safety_class": "dry_run_capture",
        "sends_uinput": False,
        "requires_user_judgement": True,
        "unblocks": ["external_window_preflight", "external_follow_session"],
    }
    calls = []
    preflight_run_ids = []

    def fake_run_operator_preflight(**kwargs):
        preflight_run_ids.append(kwargs["run_id"])
        return fake_operator_preflight_report(
            candidate_action_plan=[action],
            run_id=kwargs["run_id"],
        )

    monkeypatch.setattr(
        external_candidate_action_runner_module,
        "run_operator_preflight",
        fake_run_operator_preflight,
    )
    monkeypatch.setattr(
        external_candidate_action_runner_module,
        "write_operator_reports",
        lambda report, _log_dir: (
            tmp_path / ("%s.json" % report.run_id),
            tmp_path / ("%s.md" % report.run_id),
        ),
    )
    monkeypatch.setattr(
        external_candidate_action_runner_module,
        "run_shell_command",
        lambda command, *, timeout_s: (
            calls.append((command, timeout_s))
            or CommandRunResult(returncode=2, stdout="waiting\n")
        ),
    )
    report = run_candidate_action(
        run_id="unit-candidate-action-exec",
        log_dir=tmp_path,
        window_title="pr0p",
        bbox_file=tmp_path / "bbox.json",
        window_exact=False,
        window_case_sensitive=False,
        window_min_width=32,
        window_min_height=32,
        require_live_input=False,
        require_live_follow=True,
        require_fresh_evidence=True,
        evidence_stale_after_s=3600.0,
        max_window_candidates=2,
        axis_sweep_axes=("yaw", "pitch"),
        axis_expected_shifts=None,
        axis_min_shift_px=1.0,
        candidate_index=2,
        execute_safe=True,
        ack_candidate=True,
        timeout_s=5.0,
    )
    assert report.status == OPERATOR_CANDIDATE_ACTION_PROGRESS
    assert calls == [("dry sequence region command", 5.0)]
    step = report.steps[0]
    assert step["status"] == WAITING
    assert step["returncode"] == 2
    assert step["sends_uinput"] is False
    assert report.metrics["candidate_action_decision"]["selected_next_step"] == (
        "dry_run_sequence_region"
    )
    assert report.metrics["executed_count"] == 1
    assert preflight_run_ids == [
        "unit-candidate-action-exec-operator-preflight",
        "unit-candidate-action-exec-post-operator-preflight",
    ]
    assert (
        report.metrics["candidate_action_decision"]["post_operator_status"]
        == WAITING
    )
    assert report.metrics["post_operator_preflight_report"]["status"] == WAITING
    assert report.evidence_paths["post_operator_preflight_json"].endswith(
        "unit-candidate-action-exec-post-operator-preflight.json"
    )


def test_external_target_acceptance_rejects_nested_status_failure(
    monkeypatch,
    tmp_path,
):
    monkeypatch.setattr(
        external_target_acceptance_module,
        "build_status_report",
        lambda **_kwargs: fake_sandbox_status_report(REJECT),
    )
    monkeypatch.setattr(
        external_target_acceptance_module,
        "write_status_reports",
        lambda _report, _log_dir: (
            tmp_path / "nested-status.json",
            tmp_path / "nested-status.md",
        ),
    )
    monkeypatch.setattr(
        external_target_acceptance_module,
        "process_snapshot",
        lambda: [],
    )
    report = run_external_acceptance(
        run_id="unit-external-acceptance-reject",
        log_dir=tmp_path,
        window_title="pr0p",
        bbox_file=tmp_path / "bbox.json",
        mode="dry-run",
        window_exact=False,
        window_case_sensitive=False,
        window_min_width=32,
        window_min_height=32,
        require_fresh_evidence=True,
        evidence_stale_after_s=3600.0,
        axis_sweep_axes=("yaw", "pitch"),
        axis_expected_shifts=None,
        axis_min_shift_px=1.0,
    )
    assert report.status == REJECT
    assert report.metrics["acceptance_mode"] == "dry-run"
    assert report.metrics["require_live_input"] is False
    assert report.metrics["require_live_follow"] is False
    assert report.metrics["isolation_status"] == PASS
    assert report.metrics["acceptance_decision"]["decision"] == REJECT


def test_external_target_acceptance_rejects_forbidden_process_delta(
    monkeypatch,
    tmp_path,
):
    snapshots = iter([
        [],
        ["200 /usr/bin/gazebo --server"],
    ])
    monkeypatch.setattr(
        external_target_acceptance_module,
        "build_status_report",
        lambda **_kwargs: fake_sandbox_status_report(SANDBOX_STATUS_READY),
    )
    monkeypatch.setattr(
        external_target_acceptance_module,
        "write_status_reports",
        lambda _report, _log_dir: (
            tmp_path / "nested-status.json",
            tmp_path / "nested-status.md",
        ),
    )
    monkeypatch.setattr(
        external_target_acceptance_module,
        "process_snapshot",
        lambda: next(snapshots),
    )
    report = run_external_acceptance(
        run_id="unit-external-acceptance-process-delta",
        log_dir=tmp_path,
        window_title="pr0p",
        bbox_file=tmp_path / "bbox.json",
        mode="live-follow",
        window_exact=False,
        window_case_sensitive=False,
        window_min_width=32,
        window_min_height=32,
        require_fresh_evidence=True,
        evidence_stale_after_s=3600.0,
        axis_sweep_axes=("yaw", "pitch"),
        axis_expected_shifts=None,
        axis_min_shift_px=1.0,
    )
    assert report.status == REJECT
    assert report.metrics["isolation_status"] == FAIL
    assert report.metrics["isolation_audit"]["notes"] == [
        "FORBIDDEN_PROCESS_STARTED"
    ]
    assert report.next_actions == [
        "stop forbidden Gazebo/Betaflight processes before external acceptance"
    ]
    decision = report.metrics["acceptance_decision"]
    assert decision["decision"] == REJECT
    assert decision["isolation_status"] == FAIL
    assert decision["next_operator_action"] == (
        "stop forbidden Gazebo/Betaflight processes before external acceptance"
    )


def test_status_report_require_fresh_evidence_keeps_ready_with_fresh_evidence(
    monkeypatch,
    tmp_path,
):
    paths = write_dry_ready_status_evidence(tmp_path)
    monkeypatch.setattr(sandbox_status_report_module, "process_snapshot", lambda: [])
    monkeypatch.setattr(
        sandbox_status_report_module,
        "check_window",
        lambda *_args, **_kwargs: sandbox_status_report_module.StatusItem(
            "target_window", PASS, "target window is visible",
        ),
    )
    report = build_status_report(
        run_id="unit-status-fresh-gate-ready",
        log_dir=tmp_path,
        window_title="pr0p",
        bbox_file=paths["bbox_file"],
        window_exact=False,
        window_case_sensitive=False,
        window_min_width=32,
        window_min_height=32,
        require_fresh_evidence=True,
        evidence_stale_after_s=3600.0,
    )
    assert report.status == SANDBOX_STATUS_READY
    assert report.metrics["base_status_without_freshness_gate"] == SANDBOX_STATUS_READY
    assert report.metrics["freshness_gate"]["status"] == PASS
    assert report.metrics["freshness_gate"]["required_keys"] == [
        "latest_acceptance",
        "matching_external_preflight",
        "matching_external_follow",
        "bbox_file",
    ]
    commands = {command["name"]: command for command in report.resume_commands}
    assert "--require-fresh-evidence" in commands["status"]["command"]
    assert "--evidence-stale-after-s 3600.0" in commands["status"]["command"]


def test_status_report_require_fresh_evidence_waits_when_ready_evidence_is_stale(
    monkeypatch,
    tmp_path,
):
    paths = write_dry_ready_status_evidence(tmp_path)
    for path in paths.values():
        os.utime(path, (10.0, 10.0))
    monkeypatch.setattr(sandbox_status_report_module, "process_snapshot", lambda: [])
    monkeypatch.setattr(
        sandbox_status_report_module,
        "check_window",
        lambda *_args, **_kwargs: sandbox_status_report_module.StatusItem(
            "target_window", PASS, "target window is visible",
        ),
    )
    report = build_status_report(
        run_id="unit-status-fresh-gate-stale",
        log_dir=tmp_path,
        window_title="pr0p",
        bbox_file=paths["bbox_file"],
        window_exact=False,
        window_case_sensitive=False,
        window_min_width=32,
        window_min_height=32,
        require_fresh_evidence=True,
        evidence_stale_after_s=1.0,
    )
    assert report.status == WAITING
    assert report.summary == (
        "external game/screen sandbox is waiting for fresh setup evidence"
    )
    assert report.metrics["base_status_without_freshness_gate"] == SANDBOX_STATUS_READY
    gate = report.metrics["freshness_gate"]
    assert gate["status"] == WAITING
    assert gate["missing_keys"] == []
    assert set(gate["stale_keys"]) == {
        "latest_acceptance",
        "matching_external_preflight",
        "matching_external_follow",
        "bbox_file",
    }
    assert report.next_actions[0].startswith(
        "refresh stale/missing evidence for freshness gate:"
    )


def test_status_report_matching_report_filters_by_window_title(tmp_path):
    other = tmp_path / "20260705-000000-other-external-follow-session.json"
    pr0p = tmp_path / "20260705-000001-pr0p-external-follow-session.json"
    write_status_json(other, {
        "status": "EXTERNAL_FOLLOW_DRY_RUN_READY",
        "metrics": {"window_title": "Other Window"},
    })
    write_status_json(pr0p, {
        "status": "EXTERNAL_FOLLOW_DRY_RUN_READY",
        "metrics": {"window_title": "pr0p", "tracker_bbox": [10, 20, 80, 80]},
    })
    path, data = matching_report(
        tmp_path,
        "*-external-follow-session.json",
        window_title="pr0p",
        tracker_bbox=[10, 20, 80, 80],
    )
    assert path == pr0p
    assert data["metrics"]["window_title"] == "pr0p"


def test_status_report_matching_report_filters_by_tracker_bbox(tmp_path):
    stale = tmp_path / "20260705-000000-stale-external-follow-session.json"
    current = tmp_path / "20260705-000001-current-external-follow-session.json"
    write_status_json(stale, {
        "status": "EXTERNAL_FOLLOW_DRY_RUN_READY",
        "metrics": {"window_title": "pr0p", "tracker_bbox": [99, 99, 80, 80]},
    })
    write_status_json(current, {
        "status": "EXTERNAL_FOLLOW_DRY_RUN_READY",
        "metrics": {"window_title": "pr0p", "tracker_bbox": [10, 20, 80, 80]},
    })
    path, data = matching_report(
        tmp_path,
        "*-external-follow-session.json",
        window_title="pr0p",
        tracker_bbox=[10, 20, 80, 80],
    )
    assert path == current
    assert data["metrics"]["tracker_bbox"] == [10, 20, 80, 80]


def test_status_report_waits_when_target_setup_is_missing(monkeypatch, tmp_path):
    monkeypatch.setattr(sandbox_status_report_module, "process_snapshot", lambda: [])
    monkeypatch.setattr(
        sandbox_status_report_module,
        "check_window",
        lambda *_args, **_kwargs: sandbox_status_report_module.StatusItem(
            "target_window", WAITING, "no visible window matched the target title",
        ),
    )
    report = build_status_report(
        run_id="unit-status-waiting",
        log_dir=tmp_path,
        window_title="pr0p",
        bbox_file=tmp_path / "missing-bbox.json",
        window_exact=False,
        window_case_sensitive=False,
        window_min_width=32,
        window_min_height=32,
    )
    assert report.status == WAITING
    assert "create bbox file" in " ".join(report.next_actions)
    assert "open the external game/sim window" in " ".join(report.next_actions)
    commands = {command["name"]: command for command in report.resume_commands}
    ladder = readiness_ladder_by_name(report)
    assert ladder["target_setup"]["status"] == WAITING
    assert "target_window=WAITING" in ladder["target_setup"]["unmet_requires_pass"]
    assert "target_bbox=WAITING" in ladder["target_setup"]["unmet_requires_pass"]
    assert report.metrics["first_blocking_stage"]["name"] == "local_sandbox_isolation"
    assert "status" in commands
    assert "capture_bbox_frame" in commands
    assert "write_bbox_file_template" in commands
    assert "interactive_bbox_select" in commands
    assert commands["status"]["safety_class"] == "status_only"
    assert commands["status"]["expected_status"] == "READY_OR_WAITING_STATUS_REPORT"
    assert commands["capture_bbox_frame"]["sends_uinput"] is False
    assert commands["capture_bbox_frame"]["safety_class"] == "capture_only"
    assert commands["capture_bbox_frame"]["requires_pass"] == ["target_window"]
    assert commands["capture_bbox_frame"]["expected_status"] == WAITING
    assert commands["capture_bbox_frame"]["unblocks"] == ["target_bbox"]
    assert commands["write_bbox_file_template"]["requires_edit"] is True
    assert commands["write_bbox_file_template"]["safety_class"] == "manual_bbox_write"
    assert commands["write_bbox_file_template"]["requires_pass"] == ["target_window"]
    assert commands["write_bbox_file_template"]["expected_status"] == PASS
    assert commands["write_bbox_file_template"]["expected_report_path"] == str(
        tmp_path / "missing-bbox.json"
    )
    assert "--bbox X,Y,W,H" in commands["write_bbox_file_template"]["command"]
    assert commands["interactive_bbox_select"]["requires_user_interaction"] is True
    assert commands["interactive_bbox_select"]["safety_class"] == "manual_bbox_select"
    assert commands["interactive_bbox_select"]["requires_pass"] == ["target_window"]
    assert "--interactive-select" in commands["interactive_bbox_select"]["command"]
    assert commands["interactive_bbox_select"]["expected_report_path"] == str(
        tmp_path / "missing-bbox.json"
    )


def test_status_report_promotes_ready_with_matching_evidence(monkeypatch, tmp_path):
    bbox_file = tmp_path / "pr0p-target-bbox.json"
    write_status_json(bbox_file, {"bbox": [10, 20, 80, 80]})
    write_status_json(tmp_path / "20260705-000000-acceptance-run.json", {
        "status": SIMPLE_SANDBOX_READY,
        "summary": "ready",
        "decision_report_md": str(tmp_path / "decision.md"),
    })
    write_status_json(tmp_path / "20260705-000001-pr0p-external-window-preflight.json", {
        "status": "EXTERNAL_WINDOW_READY_DRY",
        "reasons": [],
        "metrics": {
            "window_title": "pr0p",
            "tracker_bbox": [10, 20, 80, 80],
            "phase_statuses": {"S1-real": PASS, "S2-real": PASS, "S7-real": PASS},
        },
    })
    write_status_json(tmp_path / "20260705-000002-pr0p-external-follow-session.json", {
        "status": "EXTERNAL_FOLLOW_DRY_RUN_READY",
        "reasons": [],
        "metrics": {
            "window_title": "pr0p",
            "tracker_bbox": [10, 20, 80, 80],
            "loop_summary": {"found_ratio": 1.0, "loss_events": 0},
        },
    })
    monkeypatch.setattr(sandbox_status_report_module, "process_snapshot", lambda: [])
    monkeypatch.setattr(
        sandbox_status_report_module,
        "check_window",
        lambda *_args, **_kwargs: sandbox_status_report_module.StatusItem(
            "target_window",
            PASS,
            "target window is visible",
            metrics={"window": X11Window("0x1", "pr0p", "game", 640, 480, 0, 0, 0, 0).as_dict()},
        ),
    )
    report = build_status_report(
        run_id="unit-status-ready",
        log_dir=tmp_path,
        window_title="pr0p",
        bbox_file=bbox_file,
        window_exact=False,
        window_case_sensitive=False,
        window_min_width=32,
        window_min_height=32,
    )
    assert report.status == SANDBOX_STATUS_READY
    live_item = next(
        item for item in report.items
        if item.name == "external_live_input_readiness"
    )
    live_follow_item = next(
        item for item in report.items
        if item.name == "external_live_follow_sequence"
    )
    assert live_item.status == WAITING
    assert live_follow_item.status == WAITING
    assert report.next_actions == [
        "external dry-run path is ready; optional live input requires "
        "external_live_input_readiness.py --ack-live-input"
    ]
    ladder = readiness_ladder_by_name(report)
    assert ladder["local_sandbox_isolation"]["status"] == PASS
    assert ladder["target_setup"]["status"] == PASS
    assert ladder["dry_perception_control"]["status"] == PASS
    assert ladder["control_binding_evidence"]["status"] == WAITING
    assert report.metrics["first_blocking_stage"]["name"] == "control_binding_evidence"
    freshness = report.metrics["evidence_freshness"]
    assert freshness["artifacts"]["bbox_file"]["freshness"] == "FRESH"
    assert freshness["artifacts"]["latest_acceptance"]["freshness"] == "FRESH"
    assert freshness["artifacts"]["matching_external_preflight"]["freshness"] == "FRESH"
    assert freshness["artifacts"]["matching_external_follow"]["freshness"] == "FRESH"
    markdown = sandbox_status_report_module.build_markdown(report)
    assert "## Readiness Ladder" in markdown
    assert "## Evidence Freshness" in markdown
    assert "| S2 | dry_perception_control | PASS | none |" in markdown


def test_status_report_waits_when_matching_reports_use_stale_bbox(
    monkeypatch,
    tmp_path,
):
    bbox_file = tmp_path / "pr0p-target-bbox.json"
    write_status_json(bbox_file, {"bbox": [10, 20, 80, 80]})
    write_status_json(tmp_path / "20260705-000000-acceptance-run.json", {
        "status": SIMPLE_SANDBOX_READY,
    })
    write_status_json(tmp_path / "20260705-000001-pr0p-external-window-preflight.json", {
        "status": "EXTERNAL_WINDOW_READY_DRY",
        "metrics": {"window_title": "pr0p", "tracker_bbox": [99, 20, 80, 80]},
    })
    write_status_json(tmp_path / "20260705-000002-pr0p-external-follow-session.json", {
        "status": "EXTERNAL_FOLLOW_DRY_RUN_READY",
        "metrics": {
            "window_title": "pr0p",
            "tracker_bbox": [99, 20, 80, 80],
            "loop_summary": {"found_ratio": 1.0, "loss_events": 0},
        },
    })
    monkeypatch.setattr(sandbox_status_report_module, "process_snapshot", lambda: [])
    monkeypatch.setattr(
        sandbox_status_report_module,
        "check_window",
        lambda *_args, **_kwargs: sandbox_status_report_module.StatusItem(
            "target_window", PASS, "target window is visible",
        ),
    )
    report = build_status_report(
        run_id="unit-status-stale-bbox",
        log_dir=tmp_path,
        window_title="pr0p",
        bbox_file=bbox_file,
        window_exact=False,
        window_case_sensitive=False,
        window_min_width=32,
        window_min_height=32,
    )
    assert report.status == WAITING
    assert report.evidence_paths["matching_external_preflight"] is None
    assert report.evidence_paths["matching_external_follow"] is None
    assert "run external_window_preflight.py" in " ".join(report.next_actions)


def test_status_report_require_live_input_waits_without_readiness_evidence(
    monkeypatch,
    tmp_path,
):
    bbox_file = tmp_path / "pr0p-target-bbox.json"
    write_status_json(bbox_file, {"bbox": [10, 20, 80, 80]})
    write_status_json(tmp_path / "20260705-000000-acceptance-run.json", {
        "status": SIMPLE_SANDBOX_READY,
    })
    write_status_json(tmp_path / "20260705-000001-pr0p-external-window-preflight.json", {
        "status": "EXTERNAL_WINDOW_READY_DRY",
        "metrics": {"window_title": "pr0p", "tracker_bbox": [10, 20, 80, 80]},
    })
    write_status_json(tmp_path / "20260705-000002-pr0p-external-follow-session.json", {
        "status": "EXTERNAL_FOLLOW_DRY_RUN_READY",
        "metrics": {
            "window_title": "pr0p",
            "tracker_bbox": [10, 20, 80, 80],
            "loop_summary": {"found_ratio": 1.0, "loss_events": 0},
        },
    })
    monkeypatch.setattr(sandbox_status_report_module, "process_snapshot", lambda: [])
    monkeypatch.setattr(
        sandbox_status_report_module,
        "check_window",
        lambda *_args, **_kwargs: sandbox_status_report_module.StatusItem(
            "target_window", PASS, "target window is visible",
        ),
    )
    report = build_status_report(
        run_id="unit-status-live-waiting",
        log_dir=tmp_path,
        window_title="pr0p",
        bbox_file=bbox_file,
        window_exact=False,
        window_case_sensitive=False,
        window_min_width=32,
        window_min_height=32,
        require_live_input=True,
    )
    assert report.status == WAITING
    assert "run external_live_input_readiness.py --ack-live-input" in " ".join(
        report.next_actions
    )
    assert report.metrics["require_live_input"] is True
    commands = {command["name"]: command for command in report.resume_commands}
    assert commands["rc_binding_assistant"]["sends_uinput"] is True
    assert commands["rc_binding_assistant"]["requires_ack_live_input"] is True
    assert commands["rc_binding_assistant"]["safety_class"] == "live_binding_ack_required"
    assert commands["rc_binding_assistant"]["requires_pass"] == [
        "target_window",
        "target_bbox",
        "external_window_preflight",
        "external_follow_session",
        "uinput_environment",
        "gazebo_betaflight_process_boundary",
    ]
    assert commands["rc_binding_assistant"]["expected_status"] == PASS
    assert commands["rc_binding_assistant"]["unblocks"] == ["rc_channel_binding"]
    assert (
        commands["rc_binding_assistant"]["expected_report_glob"]
        == "*-rc-binding.json"
    )
    assert "rc_binding_assistant.py" in commands["rc_binding_assistant"]["command"]
    assert "--window-title pr0p" in commands["rc_binding_assistant"]["command"]
    assert "--tracker-bbox-file" in commands["rc_binding_assistant"]["command"]
    assert "--uinput --ack-live-input" in commands["rc_binding_assistant"]["command"]
    assert "--axes yaw,pitch,roll,throttle" in commands["rc_binding_assistant"]["command"]
    assert commands["live_input_readiness"]["sends_uinput"] is True
    assert commands["live_input_readiness"]["requires_ack_live_input"] is True
    assert commands["live_input_readiness"]["safety_class"] == "live_input_ack_required"
    assert commands["live_input_readiness"]["requires_pass"] == [
        "target_window",
        "target_bbox",
        "external_window_preflight",
        "external_follow_session",
        "uinput_environment",
        "gazebo_betaflight_process_boundary",
    ]
    assert (
        commands["live_input_readiness"]["expected_status"]
        == EXTERNAL_LIVE_INPUT_READY
    )
    assert commands["live_input_readiness"]["unblocks"] == [
        "external_live_input_readiness"
    ]
    assert (
        commands["live_input_readiness"]["expected_report_glob"]
        == "*-external-live-input-readiness.json"
    )
    assert "--ack-live-input" in commands["live_input_readiness"]["command"]


def test_status_report_matches_rc_binding_by_current_bbox(monkeypatch, tmp_path):
    bbox_file = tmp_path / "pr0p-target-bbox.json"
    write_status_json(bbox_file, {"bbox": [10, 20, 80, 80]})
    write_status_json(tmp_path / "20260705-000000-acceptance-run.json", {
        "status": SIMPLE_SANDBOX_READY,
    })
    write_status_json(tmp_path / "20260705-000001-pr0p-external-window-preflight.json", {
        "status": "EXTERNAL_WINDOW_READY_DRY",
        "metrics": {"window_title": "pr0p", "tracker_bbox": [10, 20, 80, 80]},
    })
    write_status_json(tmp_path / "20260705-000002-pr0p-external-follow-session.json", {
        "status": "EXTERNAL_FOLLOW_DRY_RUN_READY",
        "metrics": {
            "window_title": "pr0p",
            "tracker_bbox": [10, 20, 80, 80],
            "loop_summary": {"found_ratio": 1.0, "loss_events": 0},
        },
    })
    write_status_json(tmp_path / "20260705-000003-pr0p-stale-rc-binding.json", {
        "status": PASS,
        "metrics": {
            "window_title": "pr0p",
            "tracker_bbox": [99, 20, 80, 80],
            "axes": ["yaw"],
            "real_input": True,
            "ack_live_input": True,
            "neutralized": True,
            "command_count": 4,
        },
    })
    binding_path = tmp_path / "20260705-000004-pr0p-current-rc-binding.json"
    write_status_json(binding_path, {
        "status": PASS,
        "metrics": {
            "window_title": "pr0p",
            "tracker_bbox": [10, 20, 80, 80],
            "axes": ["yaw", "pitch", "roll", "throttle"],
            "real_input": True,
            "ack_live_input": True,
            "neutralized": True,
            "command_count": 13,
        },
    })
    monkeypatch.setattr(sandbox_status_report_module, "process_snapshot", lambda: [])
    monkeypatch.setattr(
        sandbox_status_report_module,
        "check_window",
        lambda *_args, **_kwargs: sandbox_status_report_module.StatusItem(
            "target_window", PASS, "target window is visible",
        ),
    )
    report = build_status_report(
        run_id="unit-status-rc-binding-ready",
        log_dir=tmp_path,
        window_title="pr0p",
        bbox_file=bbox_file,
        window_exact=False,
        window_case_sensitive=False,
        window_min_width=32,
        window_min_height=32,
        require_live_input=True,
    )
    item = next(
        item for item in report.items
        if item.name == "rc_binding_assistant"
    )
    assert item.status == PASS
    assert item.metrics["tracker_bbox"] == [10, 20, 80, 80]
    assert item.metrics["command_count"] == 13
    assert item.metrics["real_input"] is True
    assert item.metrics["ack_live_input"] is True
    assert item.metrics["neutralized"] is True
    assert report.evidence_paths["matching_rc_binding"] == str(binding_path)
    commands = {command["name"]: command for command in report.resume_commands}
    assert "rc_binding_assistant" not in commands
    assert "live_input_readiness" in commands


def test_status_report_does_not_accept_dry_run_rc_binding_as_live_evidence(
    monkeypatch,
    tmp_path,
):
    bbox_file = tmp_path / "pr0p-target-bbox.json"
    write_status_json(bbox_file, {"bbox": [10, 20, 80, 80]})
    write_status_json(tmp_path / "20260705-000000-acceptance-run.json", {
        "status": SIMPLE_SANDBOX_READY,
    })
    write_status_json(tmp_path / "20260705-000001-pr0p-external-window-preflight.json", {
        "status": "EXTERNAL_WINDOW_READY_DRY",
        "metrics": {"window_title": "pr0p", "tracker_bbox": [10, 20, 80, 80]},
    })
    write_status_json(tmp_path / "20260705-000002-pr0p-external-follow-session.json", {
        "status": "EXTERNAL_FOLLOW_DRY_RUN_READY",
        "metrics": {
            "window_title": "pr0p",
            "tracker_bbox": [10, 20, 80, 80],
            "loop_summary": {"found_ratio": 1.0, "loss_events": 0},
        },
    })
    binding_path = tmp_path / "20260705-000003-pr0p-dry-rc-binding.json"
    write_status_json(binding_path, {
        "status": PASS,
        "metrics": {
            "window_title": "pr0p",
            "tracker_bbox": [10, 20, 80, 80],
            "axes": ["yaw", "pitch"],
            "real_input": False,
            "ack_live_input": False,
            "neutralized": True,
            "command_count": 7,
        },
    })
    monkeypatch.setattr(sandbox_status_report_module, "process_snapshot", lambda: [])
    monkeypatch.setattr(
        sandbox_status_report_module,
        "check_window",
        lambda *_args, **_kwargs: sandbox_status_report_module.StatusItem(
            "target_window", PASS, "target window is visible",
        ),
    )
    report = build_status_report(
        run_id="unit-status-rc-binding-dry-only",
        log_dir=tmp_path,
        window_title="pr0p",
        bbox_file=bbox_file,
        window_exact=False,
        window_case_sensitive=False,
        window_min_width=32,
        window_min_height=32,
        require_live_input=True,
    )
    item = next(
        item for item in report.items
        if item.name == "rc_binding_assistant"
    )
    assert item.status == WAITING
    assert "dry-run or incomplete" in item.summary
    assert item.metrics["real_input"] is False
    assert report.evidence_paths["matching_rc_binding"] == str(binding_path)
    commands = {command["name"]: command for command in report.resume_commands}
    assert "rc_binding_assistant" in commands


def test_status_report_require_live_input_promotes_ready_with_matching_evidence(
    monkeypatch,
    tmp_path,
):
    bbox_file = tmp_path / "pr0p-target-bbox.json"
    write_status_json(bbox_file, {"bbox": [10, 20, 80, 80]})
    write_status_json(tmp_path / "20260705-000000-acceptance-run.json", {
        "status": SIMPLE_SANDBOX_READY,
    })
    write_status_json(tmp_path / "20260705-000001-pr0p-external-window-preflight.json", {
        "status": "EXTERNAL_WINDOW_READY_DRY",
        "metrics": {"window_title": "pr0p", "tracker_bbox": [10, 20, 80, 80]},
    })
    write_status_json(tmp_path / "20260705-000002-pr0p-external-follow-session.json", {
        "status": "EXTERNAL_FOLLOW_DRY_RUN_READY",
        "metrics": {
            "window_title": "pr0p",
            "tracker_bbox": [10, 20, 80, 80],
            "loop_summary": {"found_ratio": 1.0, "loss_events": 0},
        },
    })
    live_input_path = tmp_path / "20260705-000003-pr0p-external-live-input-readiness.json"
    write_status_json(live_input_path, {
        "status": "EXTERNAL_LIVE_INPUT_READY",
        "reasons": [],
        "metrics": {
            "window_title": "pr0p",
            "tracker_bbox": [10, 20, 80, 80],
            "readiness_steps": [
                {"step": "dry_sequence", "status": "EXTERNAL_DRY_RUN_READY"},
                {"step": "ack_live_input", "status": PASS},
                {"step": "axis_response", "status": "EXTERNAL_WINDOW_READY_LIVE_INPUT"},
            ],
            "axis_preflight": {
                "status": "EXTERNAL_WINDOW_READY_LIVE_INPUT",
                "phases": [
                    {
                        "phase": "S5-real",
                        "status": PASS,
                        "metrics": {
                            "real_input": True,
                            "adapter": "UInputAdapter",
                            "axis_sweep": True,
                        },
                    },
                ],
            },
        },
    })
    monkeypatch.setattr(sandbox_status_report_module, "process_snapshot", lambda: [])
    monkeypatch.setattr(
        sandbox_status_report_module,
        "check_window",
        lambda *_args, **_kwargs: sandbox_status_report_module.StatusItem(
            "target_window", PASS, "target window is visible",
        ),
    )
    report = build_status_report(
        run_id="unit-status-live-ready",
        log_dir=tmp_path,
        window_title="pr0p",
        bbox_file=bbox_file,
        window_exact=False,
        window_case_sensitive=False,
        window_min_width=32,
        window_min_height=32,
        require_live_input=True,
    )
    assert report.status == SANDBOX_STATUS_READY
    assert report.summary == "external game/screen sandbox live-input readiness is ready"
    assert report.evidence_paths["matching_external_live_input_readiness"] == str(
        live_input_path
    )
    live_input_item = next(
        item for item in report.items
        if item.name == "external_live_input_readiness"
    )
    assert live_input_item.metrics["real_input"] is True
    assert live_input_item.metrics["adapter"] == "UInputAdapter"
    assert live_input_item.metrics["axis_sweep"] is True
    assert live_input_item.metrics["proof_gaps"] == []
    assert report.next_actions == [
        "external dry-run and live-input readiness evidence are ready"
    ]


def test_status_report_does_not_accept_live_input_without_live_uinput_proof(
    monkeypatch,
    tmp_path,
):
    bbox_file = tmp_path / "pr0p-target-bbox.json"
    write_status_json(bbox_file, {"bbox": [10, 20, 80, 80]})
    write_status_json(tmp_path / "20260705-000000-acceptance-run.json", {
        "status": SIMPLE_SANDBOX_READY,
    })
    write_status_json(tmp_path / "20260705-000001-pr0p-external-window-preflight.json", {
        "status": "EXTERNAL_WINDOW_READY_DRY",
        "metrics": {"window_title": "pr0p", "tracker_bbox": [10, 20, 80, 80]},
    })
    write_status_json(tmp_path / "20260705-000002-pr0p-external-follow-session.json", {
        "status": "EXTERNAL_FOLLOW_DRY_RUN_READY",
        "metrics": {
            "window_title": "pr0p",
            "tracker_bbox": [10, 20, 80, 80],
            "loop_summary": {"found_ratio": 1.0, "loss_events": 0},
        },
    })
    write_status_json(tmp_path / "20260705-000003-pr0p-external-live-input-readiness.json", {
        "status": "EXTERNAL_LIVE_INPUT_READY",
        "reasons": [],
        "metrics": {
            "window_title": "pr0p",
            "tracker_bbox": [10, 20, 80, 80],
            "readiness_steps": [
                {"step": "dry_sequence", "status": "EXTERNAL_DRY_RUN_READY"},
                {"step": "ack_live_input", "status": PASS},
                {"step": "axis_response", "status": "EXTERNAL_WINDOW_READY_LIVE_INPUT"},
            ],
            "axis_preflight": {
                "status": "EXTERNAL_WINDOW_READY_LIVE_INPUT",
                "phases": [
                    {
                        "phase": "S5-real",
                        "status": PASS,
                        "metrics": {
                            "real_input": False,
                            "adapter": "DryRunInputAdapter",
                            "axis_sweep": True,
                        },
                    },
                ],
            },
        },
    })
    monkeypatch.setattr(sandbox_status_report_module, "process_snapshot", lambda: [])
    monkeypatch.setattr(
        sandbox_status_report_module,
        "check_window",
        lambda *_args, **_kwargs: sandbox_status_report_module.StatusItem(
            "target_window", PASS, "target window is visible",
        ),
    )
    report = build_status_report(
        run_id="unit-status-live-input-incomplete",
        log_dir=tmp_path,
        window_title="pr0p",
        bbox_file=bbox_file,
        window_exact=False,
        window_case_sensitive=False,
        window_min_width=32,
        window_min_height=32,
        require_live_input=True,
    )
    assert report.status == WAITING
    item = next(
        item for item in report.items
        if item.name == "external_live_input_readiness"
    )
    assert "live uinput proof" in item.summary
    assert "S5_REAL_INPUT_NOT_TRUE" in item.metrics["proof_gaps"]
    assert "S5_ADAPTER_NOT_UINPUT" in item.metrics["proof_gaps"]


def test_status_report_require_live_follow_waits_without_sequence_evidence(
    monkeypatch,
    tmp_path,
):
    bbox_file = tmp_path / "pr0p-target-bbox.json"
    write_status_json(bbox_file, {"bbox": [10, 20, 80, 80]})
    write_status_json(tmp_path / "20260705-000000-acceptance-run.json", {
        "status": SIMPLE_SANDBOX_READY,
    })
    write_status_json(tmp_path / "20260705-000001-pr0p-external-window-preflight.json", {
        "status": "EXTERNAL_WINDOW_READY_DRY",
        "metrics": {"window_title": "pr0p", "tracker_bbox": [10, 20, 80, 80]},
    })
    write_status_json(tmp_path / "20260705-000002-pr0p-external-follow-session.json", {
        "status": "EXTERNAL_FOLLOW_DRY_RUN_READY",
        "metrics": {
            "window_title": "pr0p",
            "tracker_bbox": [10, 20, 80, 80],
            "loop_summary": {"found_ratio": 1.0, "loss_events": 0},
        },
    })
    monkeypatch.setattr(sandbox_status_report_module, "process_snapshot", lambda: [])
    monkeypatch.setattr(
        sandbox_status_report_module,
        "check_window",
        lambda *_args, **_kwargs: sandbox_status_report_module.StatusItem(
            "target_window", PASS, "target window is visible",
        ),
    )
    report = build_status_report(
        run_id="unit-status-live-follow-waiting",
        log_dir=tmp_path,
        window_title="pr0p",
        bbox_file=bbox_file,
        window_exact=False,
        window_case_sensitive=False,
        window_min_width=32,
        window_min_height=32,
        require_live_follow=True,
    )
    assert report.status == WAITING
    assert "run external_live_follow_sequence.py --ack-live-input" in " ".join(
        report.next_actions
    )
    assert report.metrics["require_live_follow"] is True
    commands = {command["name"]: command for command in report.resume_commands}
    assert "live_follow_sequence" in commands
    assert commands["live_follow_sequence"]["sends_uinput"] is True
    assert commands["live_follow_sequence"]["requires_ack_live_input"] is True
    assert commands["live_follow_sequence"]["safety_class"] == "live_follow_ack_required"
    assert (
        commands["live_follow_sequence"]["expected_status"]
        == EXTERNAL_LIVE_FOLLOW_COMPLETE
    )
    assert commands["live_follow_sequence"]["unblocks"] == [
        "external_live_follow_sequence"
    ]
    assert (
        commands["live_follow_sequence"]["expected_report_glob"]
        == "*-external-live-follow-sequence.json"
    )
    assert "external_live_follow_sequence.py" in commands["live_follow_sequence"]["command"]
    assert "--ack-live-input" in commands["live_follow_sequence"]["command"]


def test_status_report_resume_commands_preserve_signed_axis_gate(
    monkeypatch,
    tmp_path,
):
    bbox_file = tmp_path / "pr0p-target-bbox.json"
    write_status_json(bbox_file, {"bbox": [10, 20, 80, 80]})
    write_status_json(tmp_path / "20260705-000000-acceptance-run.json", {
        "status": SIMPLE_SANDBOX_READY,
    })
    write_status_json(tmp_path / "20260705-000001-pr0p-external-window-preflight.json", {
        "status": "EXTERNAL_WINDOW_READY_DRY",
        "metrics": {"window_title": "pr0p", "tracker_bbox": [10, 20, 80, 80]},
    })
    write_status_json(tmp_path / "20260705-000002-pr0p-external-follow-session.json", {
        "status": "EXTERNAL_FOLLOW_DRY_RUN_READY",
        "metrics": {
            "window_title": "pr0p",
            "tracker_bbox": [10, 20, 80, 80],
            "loop_summary": {"found_ratio": 1.0, "loss_events": 0},
        },
    })
    monkeypatch.setattr(sandbox_status_report_module, "process_snapshot", lambda: [])
    monkeypatch.setattr(
        sandbox_status_report_module,
        "check_window",
        lambda *_args, **_kwargs: sandbox_status_report_module.StatusItem(
            "target_window", PASS, "target window is visible",
        ),
    )
    report = build_status_report(
        run_id="unit-status-signed-axis-resume",
        log_dir=tmp_path,
        window_title="pr0p",
        bbox_file=bbox_file,
        window_exact=False,
        window_case_sensitive=False,
        window_min_width=32,
        window_min_height=32,
        require_live_follow=True,
        axis_sweep_axes=("yaw", "pitch"),
        axis_expected_shifts={"yaw": ("x", 1), "pitch": ("y", -1)},
        axis_min_shift_px=1.5,
    )
    commands = {command["name"]: command for command in report.resume_commands}
    assert report.metrics["axis_expected_shifts"] == {
        "yaw": "+x",
        "pitch": "-y",
    }
    assert "--axis-expected-shifts" in commands["status"]["command"]
    assert "pitch:-y,yaw:+x" in commands["status"]["command"]
    assert "--axis-min-shift-px 1.5" in commands["live_input_readiness"]["command"]
    assert "pitch:-y,yaw:+x" in commands["live_input_readiness"]["command"]
    assert "--axis-min-shift-px 1.5" in commands["live_follow_sequence"]["command"]
    assert "pitch:-y,yaw:+x" in commands["live_follow_sequence"]["command"]


def test_status_report_require_live_follow_promotes_ready_with_matching_evidence(
    monkeypatch,
    tmp_path,
):
    bbox_file = tmp_path / "pr0p-target-bbox.json"
    write_status_json(bbox_file, {"bbox": [10, 20, 80, 80]})
    write_status_json(tmp_path / "20260705-000000-acceptance-run.json", {
        "status": SIMPLE_SANDBOX_READY,
    })
    write_status_json(tmp_path / "20260705-000001-pr0p-external-window-preflight.json", {
        "status": "EXTERNAL_WINDOW_READY_DRY",
        "metrics": {"window_title": "pr0p", "tracker_bbox": [10, 20, 80, 80]},
    })
    write_status_json(tmp_path / "20260705-000002-pr0p-external-follow-session.json", {
        "status": "EXTERNAL_FOLLOW_DRY_RUN_READY",
        "metrics": {
            "window_title": "pr0p",
            "tracker_bbox": [10, 20, 80, 80],
            "loop_summary": {"found_ratio": 1.0, "loss_events": 0},
        },
    })
    live_follow_path = tmp_path / "20260705-000003-pr0p-external-live-follow-sequence.json"
    write_status_json(live_follow_path, {
        "status": "EXTERNAL_LIVE_FOLLOW_COMPLETE",
        "reasons": [],
        "metrics": {
            "window_title": "pr0p",
            "tracker_bbox": [10, 20, 80, 80],
            "sequence_steps": [
                {"step": "ack_live_input", "status": PASS},
                {"step": "live_input_readiness", "status": "EXTERNAL_LIVE_INPUT_READY"},
                {"step": "live_follow", "status": "EXTERNAL_FOLLOW_LIVE_RUN_COMPLETE"},
            ],
                "follow": {
                    "status": "EXTERNAL_FOLLOW_LIVE_RUN_COMPLETE",
                    "metrics": {
                        "real_input": True,
                        "ack_live_input": True,
                        "adapter": "UInputAdapter",
                        "loop_summary": {"found_ratio": 1.0, "loss_events": 0},
                    },
                },
        },
    })
    monkeypatch.setattr(sandbox_status_report_module, "process_snapshot", lambda: [])
    monkeypatch.setattr(
        sandbox_status_report_module,
        "check_window",
        lambda *_args, **_kwargs: sandbox_status_report_module.StatusItem(
            "target_window", PASS, "target window is visible",
        ),
    )
    report = build_status_report(
        run_id="unit-status-live-follow-ready",
        log_dir=tmp_path,
        window_title="pr0p",
        bbox_file=bbox_file,
        window_exact=False,
        window_case_sensitive=False,
        window_min_width=32,
        window_min_height=32,
        require_live_follow=True,
    )
    assert report.status == SANDBOX_STATUS_READY
    assert report.summary == "external game/screen sandbox bounded live follow is ready"
    assert report.evidence_paths["matching_external_live_follow_sequence"] == str(
        live_follow_path
    )
    live_follow_item = next(
        item for item in report.items
        if item.name == "external_live_follow_sequence"
    )
    assert live_follow_item.metrics["found_ratio"] == 1.0
    assert live_follow_item.metrics["real_input"] is True
    assert live_follow_item.metrics["ack_live_input"] is True
    assert live_follow_item.metrics["adapter"] == "UInputAdapter"
    assert live_follow_item.metrics["proof_gaps"] == []
    assert report.next_actions == [
        "external dry-run, live-input, and live-follow evidence are ready"
    ]


def test_status_report_requires_matching_signed_axis_for_live_follow(
    monkeypatch,
    tmp_path,
):
    bbox_file = tmp_path / "pr0p-target-bbox.json"
    write_status_json(bbox_file, {"bbox": [10, 20, 80, 80]})
    write_status_json(tmp_path / "20260705-000000-acceptance-run.json", {
        "status": SIMPLE_SANDBOX_READY,
    })
    write_status_json(tmp_path / "20260705-000001-pr0p-external-window-preflight.json", {
        "status": "EXTERNAL_WINDOW_READY_DRY",
        "metrics": {"window_title": "pr0p", "tracker_bbox": [10, 20, 80, 80]},
    })
    write_status_json(tmp_path / "20260705-000002-pr0p-external-follow-session.json", {
        "status": "EXTERNAL_FOLLOW_DRY_RUN_READY",
        "metrics": {
            "window_title": "pr0p",
            "tracker_bbox": [10, 20, 80, 80],
            "loop_summary": {"found_ratio": 1.0, "loss_events": 0},
        },
    })
    write_status_json(tmp_path / "20260705-000003-pr0p-external-live-follow-sequence.json", {
        "status": "EXTERNAL_LIVE_FOLLOW_COMPLETE",
        "reasons": [],
        "metrics": {
            "window_title": "pr0p",
            "tracker_bbox": [10, 20, 80, 80],
            "sequence_steps": [
                {"step": "ack_live_input", "status": PASS},
                {"step": "live_input_readiness", "status": "EXTERNAL_LIVE_INPUT_READY"},
                {"step": "live_follow", "status": "EXTERNAL_FOLLOW_LIVE_RUN_COMPLETE"},
            ],
            "follow": {
                "status": "EXTERNAL_FOLLOW_LIVE_RUN_COMPLETE",
                "metrics": {
                    "real_input": True,
                    "ack_live_input": True,
                    "adapter": "UInputAdapter",
                    "loop_summary": {"found_ratio": 1.0, "loss_events": 0},
                },
            },
        },
    })
    monkeypatch.setattr(sandbox_status_report_module, "process_snapshot", lambda: [])
    monkeypatch.setattr(
        sandbox_status_report_module,
        "check_window",
        lambda *_args, **_kwargs: sandbox_status_report_module.StatusItem(
            "target_window", PASS, "target window is visible",
        ),
    )
    report = build_status_report(
        run_id="unit-status-live-follow-signed-missing",
        log_dir=tmp_path,
        window_title="pr0p",
        bbox_file=bbox_file,
        window_exact=False,
        window_case_sensitive=False,
        window_min_width=32,
        window_min_height=32,
        require_live_follow=True,
        axis_expected_shifts={"yaw": ("x", 1)},
        axis_min_shift_px=1.0,
    )
    assert report.status == WAITING
    item = next(
        item for item in report.items
        if item.name == "external_live_follow_sequence"
    )
    assert "AXIS_EXPECTED_SHIFTS_MISMATCH" in item.metrics["proof_gaps"]
    assert "AXIS_MIN_SHIFT_MISMATCH" in item.metrics["proof_gaps"]


def test_status_report_does_not_accept_live_follow_without_live_uinput_proof(
    monkeypatch,
    tmp_path,
):
    bbox_file = tmp_path / "pr0p-target-bbox.json"
    write_status_json(bbox_file, {"bbox": [10, 20, 80, 80]})
    write_status_json(tmp_path / "20260705-000000-acceptance-run.json", {
        "status": SIMPLE_SANDBOX_READY,
    })
    write_status_json(tmp_path / "20260705-000001-pr0p-external-window-preflight.json", {
        "status": "EXTERNAL_WINDOW_READY_DRY",
        "metrics": {"window_title": "pr0p", "tracker_bbox": [10, 20, 80, 80]},
    })
    write_status_json(tmp_path / "20260705-000002-pr0p-external-follow-session.json", {
        "status": "EXTERNAL_FOLLOW_DRY_RUN_READY",
        "metrics": {
            "window_title": "pr0p",
            "tracker_bbox": [10, 20, 80, 80],
            "loop_summary": {"found_ratio": 1.0, "loss_events": 0},
        },
    })
    live_follow_path = tmp_path / "20260705-000003-pr0p-external-live-follow-sequence.json"
    write_status_json(live_follow_path, {
        "status": "EXTERNAL_LIVE_FOLLOW_COMPLETE",
        "reasons": [],
        "metrics": {
            "window_title": "pr0p",
            "tracker_bbox": [10, 20, 80, 80],
            "sequence_steps": [
                {"step": "ack_live_input", "status": PASS},
                {"step": "live_input_readiness", "status": "EXTERNAL_LIVE_INPUT_READY"},
                {"step": "live_follow", "status": "EXTERNAL_FOLLOW_LIVE_RUN_COMPLETE"},
            ],
            "follow": {
                "status": "EXTERNAL_FOLLOW_LIVE_RUN_COMPLETE",
                "metrics": {
                    "real_input": False,
                    "ack_live_input": False,
                    "adapter": "DryRunInputAdapter",
                    "loop_summary": {"found_ratio": 1.0, "loss_events": 0},
                },
            },
        },
    })
    monkeypatch.setattr(sandbox_status_report_module, "process_snapshot", lambda: [])
    monkeypatch.setattr(
        sandbox_status_report_module,
        "check_window",
        lambda *_args, **_kwargs: sandbox_status_report_module.StatusItem(
            "target_window", PASS, "target window is visible",
        ),
    )
    report = build_status_report(
        run_id="unit-status-live-follow-incomplete",
        log_dir=tmp_path,
        window_title="pr0p",
        bbox_file=bbox_file,
        window_exact=False,
        window_case_sensitive=False,
        window_min_width=32,
        window_min_height=32,
        require_live_follow=True,
    )
    assert report.status == WAITING
    assert report.evidence_paths["matching_external_live_follow_sequence"] == str(
        live_follow_path
    )
    item = next(
        item for item in report.items
        if item.name == "external_live_follow_sequence"
    )
    assert item.status == WAITING
    assert "live uinput proof" in item.summary
    assert "FOLLOW_REAL_INPUT_NOT_TRUE" in item.metrics["proof_gaps"]
    assert "FOLLOW_ACK_NOT_TRUE" in item.metrics["proof_gaps"]
    assert "FOLLOW_ADAPTER_NOT_UINPUT" in item.metrics["proof_gaps"]


def test_status_report_rejects_forbidden_process(monkeypatch, tmp_path):
    bbox_file = tmp_path / "pr0p-target-bbox.json"
    write_status_json(bbox_file, {"bbox": [10, 20, 80, 80]})
    write_status_json(tmp_path / "20260705-000000-acceptance-run.json", {
        "status": SIMPLE_SANDBOX_READY,
    })
    monkeypatch.setattr(
        sandbox_status_report_module,
        "process_snapshot",
        lambda: ["200 /tmp/betaflight --sitl"],
    )
    monkeypatch.setattr(
        sandbox_status_report_module,
        "check_window",
        lambda *_args, **_kwargs: sandbox_status_report_module.StatusItem(
            "target_window", PASS, "target window is visible",
        ),
    )
    report = build_status_report(
        run_id="unit-status-reject",
        log_dir=tmp_path,
        window_title="pr0p",
        bbox_file=bbox_file,
        window_exact=False,
        window_case_sensitive=False,
        window_min_width=32,
        window_min_height=32,
    )
    assert report.status == REJECT
    assert "stop forbidden Gazebo/Betaflight processes" in report.next_actions[0]


def resume_status_report(status=WAITING, commands=None, items=None):
    return sandbox_status_report_module.SandboxStatusReport(
        run_id="unit-resume-status",
        started_at="2026-07-06T00:00:00+0300",
        status=status,
        summary="unit resume status",
        items=items or [],
        resume_commands=commands or [],
    )


def test_resume_runner_plan_only_does_not_execute_commands(monkeypatch, tmp_path):
    commands = [
        {
            "name": "status",
            "safety_class": "status_only",
            "sends_uinput": False,
            "command": "fpv_env/bin/python status.py",
        },
        {
            "name": "capture_bbox_frame",
            "safety_class": "capture_only",
            "sends_uinput": False,
            "command": "fpv_env/bin/python capture.py",
        },
        {
            "name": "write_bbox_file_template",
            "safety_class": "manual_bbox_write",
            "sends_uinput": False,
            "requires_edit": True,
            "command": "fpv_env/bin/python bbox.py --bbox X,Y,W,H",
        },
        {
            "name": "interactive_bbox_select",
            "safety_class": "manual_bbox_select",
            "sends_uinput": False,
            "requires_user_interaction": True,
            "command": "fpv_env/bin/python bbox.py --interactive-select",
        },
        {
            "name": "live_input_readiness",
            "safety_class": "live_input_ack_required",
            "sends_uinput": True,
            "requires_ack_live_input": True,
            "command": "fpv_env/bin/python live.py --ack-live-input",
        },
    ]
    monkeypatch.setattr(
        sandbox_resume_runner_module,
        "build_status_report",
        lambda **_kwargs: resume_status_report(commands=commands),
    )
    calls = []
    monkeypatch.setattr(
        sandbox_resume_runner_module,
        "run_shell_command",
        lambda command, *, timeout_s: calls.append(command) or CommandRunResult(0),
    )
    report = run_resume(
        run_id="unit-resume-plan",
        log_dir=tmp_path,
        window_title="pr0p",
        bbox_file=tmp_path / "bbox.json",
        window_exact=False,
        window_case_sensitive=False,
        window_min_width=32,
        window_min_height=32,
        require_live_input=False,
        require_live_follow=False,
        execute_safe=False,
        execute_live_input=False,
        ack_live_input=False,
        max_commands=1,
        timeout_s=1.0,
    )
    steps = {step["name"]: step for step in report.steps}
    assert report.status == WAITING
    assert calls == []
    assert steps["status"]["status"] == SKIPPED
    assert steps["capture_bbox_frame"]["status"] == PLANNED
    assert steps["write_bbox_file_template"]["status"] == BLOCKED
    assert steps["interactive_bbox_select"]["status"] == BLOCKED
    assert "manual UI interaction" in steps["interactive_bbox_select"]["summary"]
    assert steps["live_input_readiness"]["status"] == BLOCKED
    assert "requires --execute-live-input" in steps["live_input_readiness"]["summary"]
    decision = report.metrics["resume_decision"]
    assert decision["decision"] == "command_available"
    assert decision["first_planned_step"]["name"] == "capture_bbox_frame"
    assert decision["first_blocked_step"]["name"] == "write_bbox_file_template"
    assert decision["next_operator_action"] is None
    summary = report.metrics["resume_step_summary"]
    assert summary["total_count"] == 5
    assert summary["status_counts"] == {
        SKIPPED: 1,
        PLANNED: 1,
        BLOCKED: 3,
    }
    assert summary["sends_uinput_count"] == 1
    assert summary["executed_count"] == 0
    assert summary["first_planned_step"]["name"] == "capture_bbox_frame"
    assert summary["first_blocked_step"]["name"] == "write_bbox_file_template"
    assert summary["first_unmet_requirement_step"] is None


def test_resume_runner_passes_signed_axis_gate_to_status_report(monkeypatch, tmp_path):
    seen = []

    def fake_status_report(**kwargs):
        seen.append(kwargs)
        return resume_status_report(commands=[])

    monkeypatch.setattr(
        sandbox_resume_runner_module,
        "build_status_report",
        fake_status_report,
    )
    report = run_resume(
        run_id="unit-resume-signed-axis",
        log_dir=tmp_path,
        window_title="pr0p",
        bbox_file=tmp_path / "bbox.json",
        window_exact=False,
        window_case_sensitive=False,
        window_min_width=32,
        window_min_height=32,
        require_live_input=False,
        require_live_follow=True,
        execute_safe=False,
        execute_live_input=False,
        ack_live_input=False,
        max_commands=1,
        timeout_s=1.0,
        axis_sweep_axes=("yaw", "pitch"),
        axis_expected_shifts={"yaw": ("x", 1)},
        axis_min_shift_px=2.0,
        require_fresh_evidence=True,
        evidence_stale_after_s=12.0,
    )
    assert report.status == WAITING
    assert seen[0]["axis_sweep_axes"] == ("yaw", "pitch")
    assert seen[0]["axis_expected_shifts"] == {"yaw": ("x", 1)}
    assert seen[0]["axis_min_shift_px"] == 2.0
    assert seen[0]["require_fresh_evidence"] is True
    assert seen[0]["evidence_stale_after_s"] == 12.0
    assert report.metrics["axis_expected_shifts"] == {"yaw": "+x"}
    assert report.metrics["require_fresh_evidence"] is True
    assert report.metrics["evidence_stale_after_s"] == 12.0
    assert report.metrics["axis_min_shift_px"] == 2.0


def test_resume_runner_execute_safe_runs_only_safe_command(monkeypatch, tmp_path):
    commands = [
        {
            "name": "capture_bbox_frame",
            "safety_class": "capture_only",
            "sends_uinput": False,
            "command": "fpv_env/bin/python capture.py",
        },
        {
            "name": "dry_sequence",
            "safety_class": "dry_run_capture",
            "sends_uinput": False,
            "command": "fpv_env/bin/python dry.py",
        },
    ]
    monkeypatch.setattr(
        sandbox_resume_runner_module,
        "build_status_report",
        lambda **_kwargs: resume_status_report(commands=commands),
    )
    calls = []

    def fake_run(command, *, timeout_s):
        calls.append((command, timeout_s))
        return CommandRunResult(returncode=0, stdout="ok\n")

    monkeypatch.setattr(sandbox_resume_runner_module, "run_shell_command", fake_run)
    report = run_resume(
        run_id="unit-resume-execute-safe",
        log_dir=tmp_path,
        window_title="pr0p",
        bbox_file=tmp_path / "bbox.json",
        window_exact=False,
        window_case_sensitive=False,
        window_min_width=32,
        window_min_height=32,
        require_live_input=False,
        require_live_follow=False,
        execute_safe=True,
        execute_live_input=False,
        ack_live_input=False,
        max_commands=1,
        timeout_s=3.0,
    )
    steps = {step["name"]: step for step in report.steps}
    assert report.status == SANDBOX_RESUME_PROGRESS
    assert calls == [("fpv_env/bin/python capture.py", 3.0)]
    assert report.evidence_paths["post_status_report_json"] is not None
    assert report.metrics["post_status_report"]["status"] == WAITING
    assert steps["capture_bbox_frame"]["status"] == PASS
    assert steps["capture_bbox_frame"]["stdout_tail"] == "ok\n"
    assert steps["dry_sequence"]["status"] == PLANNED
    assert "execution limit reached" in steps["dry_sequence"]["summary"]


def test_resume_runner_blocks_command_when_required_status_is_not_pass(
    monkeypatch,
    tmp_path,
):
    commands = [
        {
            "name": "dry_sequence",
            "safety_class": "dry_run_capture",
            "sends_uinput": False,
            "requires_pass": ["target_window", "target_bbox"],
            "command": "fpv_env/bin/python dry.py",
        },
        {
            "name": "live_input_readiness",
            "safety_class": "live_input_ack_required",
            "sends_uinput": True,
            "requires_ack_live_input": True,
            "requires_pass": [
                "target_window",
                "target_bbox",
                "external_window_preflight",
            ],
            "command": "fpv_env/bin/python live.py --ack-live-input",
        },
    ]
    items = [
        sandbox_status_report_module.StatusItem(
            "target_window", WAITING, "target window is missing",
        ),
        sandbox_status_report_module.StatusItem(
            "target_bbox", PASS, "target bbox exists",
        ),
        sandbox_status_report_module.StatusItem(
            "external_window_preflight", WAITING, "preflight missing",
        ),
    ]
    monkeypatch.setattr(
        sandbox_resume_runner_module,
        "build_status_report",
        lambda **_kwargs: resume_status_report(commands=commands, items=items),
    )
    calls = []
    monkeypatch.setattr(
        sandbox_resume_runner_module,
        "run_shell_command",
        lambda command, *, timeout_s: calls.append(command) or CommandRunResult(0),
    )
    report = run_resume(
        run_id="unit-resume-dependency-block",
        log_dir=tmp_path,
        window_title="pr0p",
        bbox_file=tmp_path / "bbox.json",
        window_exact=False,
        window_case_sensitive=False,
        window_min_width=32,
        window_min_height=32,
        require_live_input=True,
        require_live_follow=False,
        execute_safe=True,
        execute_live_input=True,
        ack_live_input=True,
        max_commands=2,
        timeout_s=1.0,
    )
    steps = {step["name"]: step for step in report.steps}
    assert report.status == WAITING
    assert calls == []
    assert steps["dry_sequence"]["status"] == BLOCKED
    assert steps["dry_sequence"]["unmet_requires_pass"] == ["target_window=WAITING"]
    assert "target_window=WAITING" in steps["dry_sequence"]["summary"]
    assert steps["live_input_readiness"]["status"] == BLOCKED
    assert steps["live_input_readiness"]["unmet_requires_pass"] == [
        "target_window=WAITING",
        "external_window_preflight=WAITING",
    ]
    decision = report.metrics["resume_decision"]
    assert decision["decision"] == "missing_required_status"
    assert decision["first_unmet_requirement_step"]["name"] == "dry_sequence"
    assert decision["first_unmet_requirement_step"]["unmet_requires_pass"] == [
        "target_window=WAITING",
    ]
    assert decision["executed_count"] == 0
    assert decision["executed_steps"] == []
    summary = report.metrics["resume_step_summary"]
    assert summary["status_counts"] == {BLOCKED: 2}
    assert summary["first_unmet_requirement_step"]["name"] == "dry_sequence"
    assert summary["first_unmet_requirement_step"]["unmet_requires_pass"] == [
        "target_window=WAITING",
    ]


def test_resume_runner_promotes_ready_after_post_status(monkeypatch, tmp_path):
    command = {
        "name": "dry_sequence",
        "safety_class": "dry_run_capture",
        "sends_uinput": False,
        "command": "fpv_env/bin/python dry.py",
    }
    reports = iter([
        resume_status_report(commands=[command]),
        resume_status_report(status=SANDBOX_STATUS_READY, commands=[]),
    ])
    monkeypatch.setattr(
        sandbox_resume_runner_module,
        "build_status_report",
        lambda **_kwargs: next(reports),
    )
    monkeypatch.setattr(
        sandbox_resume_runner_module,
        "run_shell_command",
        lambda command, *, timeout_s: CommandRunResult(returncode=0),
    )
    report = run_resume(
        run_id="unit-resume-post-ready",
        log_dir=tmp_path,
        window_title="pr0p",
        bbox_file=tmp_path / "bbox.json",
        window_exact=False,
        window_case_sensitive=False,
        window_min_width=32,
        window_min_height=32,
        require_live_input=False,
        require_live_follow=False,
        execute_safe=True,
        execute_live_input=False,
        ack_live_input=False,
        max_commands=1,
        timeout_s=3.0,
    )
    assert report.status == SANDBOX_RESUME_READY
    assert report.metrics["post_status_report"]["status"] == SANDBOX_STATUS_READY
    assert report.evidence_paths["post_status_report_md"] is not None


def test_resume_runner_live_execution_requires_ack(monkeypatch, tmp_path):
    commands = [
        {
            "name": "live_follow_sequence",
            "safety_class": "live_follow_ack_required",
            "sends_uinput": True,
            "requires_ack_live_input": True,
            "command": "fpv_env/bin/python live_follow.py --ack-live-input",
        },
    ]
    monkeypatch.setattr(
        sandbox_resume_runner_module,
        "build_status_report",
        lambda **_kwargs: resume_status_report(commands=commands),
    )
    calls = []
    monkeypatch.setattr(
        sandbox_resume_runner_module,
        "run_shell_command",
        lambda command, *, timeout_s: calls.append(command) or CommandRunResult(0),
    )
    report = run_resume(
        run_id="unit-resume-live-noack",
        log_dir=tmp_path,
        window_title="pr0p",
        bbox_file=tmp_path / "bbox.json",
        window_exact=False,
        window_case_sensitive=False,
        window_min_width=32,
        window_min_height=32,
        require_live_input=False,
        require_live_follow=True,
        execute_safe=True,
        execute_live_input=False,
        ack_live_input=False,
        max_commands=1,
        timeout_s=1.0,
    )
    assert report.steps[0]["status"] == BLOCKED
    assert calls == []

    report = run_resume(
        run_id="unit-resume-live-ack",
        log_dir=tmp_path,
        window_title="pr0p",
        bbox_file=tmp_path / "bbox.json",
        window_exact=False,
        window_case_sensitive=False,
        window_min_width=32,
        window_min_height=32,
        require_live_input=False,
        require_live_follow=True,
        execute_safe=True,
        execute_live_input=True,
        ack_live_input=True,
        max_commands=1,
        timeout_s=1.0,
    )
    assert report.status == SANDBOX_RESUME_PROGRESS
    assert report.steps[0]["status"] == PASS
    assert calls == ["fpv_env/bin/python live_follow.py --ack-live-input"]


def sequence_status_report(*, window=PASS, bbox=PASS, process=PASS):
    return sandbox_status_report_module.SandboxStatusReport(
        run_id="unit-sequence-status",
        started_at="2026-07-06T00:00:00+0300",
        status=WAITING,
        summary="unit status",
        items=[
            sandbox_status_report_module.StatusItem(
                "target_window", window, "window status"
            ),
            sandbox_status_report_module.StatusItem(
                "target_bbox", bbox, "bbox status"
            ),
            sandbox_status_report_module.StatusItem(
                "gazebo_betaflight_process_boundary", process, "process status"
            ),
        ],
    )


def test_external_dry_run_sequence_waits_before_running_gates(monkeypatch, tmp_path):
    bbox_file = tmp_path / "missing-bbox.json"
    monkeypatch.setattr(
        external_dry_run_sequence_module,
        "build_status_report",
        lambda **_kwargs: sequence_status_report(bbox=WAITING),
    )
    report = run_external_dry_run_sequence(
        run_id="unit-sequence-waiting",
        log_dir=tmp_path,
        region=None,
        window_title="pr0p",
        backend="ffmpeg",
        bbox_file=bbox_file,
        duration_s=0.5,
        hz=10.0,
        tracker_type="KCF",
        enable_pitch=False,
        desired_target_width=120.0,
        min_found_ratio=0.8,
        max_loss_events=0,
        max_yaw_axis=0.8,
        max_pitch_axis=0.8,
        window_exact=False,
        window_case_sensitive=False,
        window_min_width=32,
        window_min_height=32,
    )
    assert report.status == WAITING
    assert any(reason.startswith("target_bbox:WAITING") for reason in report.reasons)
    assert report.evidence_paths["status_report_json"] is not None
    assert report.evidence_paths["preflight_report_json"] is None
    steps = {step["step"]: step for step in report.metrics["sequence_steps"]}
    assert steps["status"]["status"] == WAITING
    assert steps["preflight"]["status"] == NOT_RUN
    assert steps["follow"]["status"] == NOT_RUN
    assert report.metrics["live_readiness"] == WAITING_FOR_DRY_RUN


def test_external_dry_run_sequence_promotes_ready_when_gates_pass(monkeypatch, tmp_path):
    bbox_file = tmp_path / "bbox.json"
    write_status_json(bbox_file, {"bbox": [10, 20, 80, 80]})
    monkeypatch.setattr(
        external_dry_run_sequence_module,
        "build_status_report",
        lambda **_kwargs: sequence_status_report(),
    )
    monkeypatch.setattr(
        external_dry_run_sequence_module,
        "run_external_preflight",
        lambda **_kwargs: ExternalWindowPreflightReport(
            run_id="unit-preflight",
            started_at="now",
            status=EXTERNAL_WINDOW_READY_DRY,
            summary="preflight pass",
        ),
    )
    monkeypatch.setattr(
        external_dry_run_sequence_module,
        "run_external_follow_session",
        lambda **_kwargs: ExternalFollowSessionReport(
            run_id="unit-follow",
            started_at="now",
            status=EXTERNAL_FOLLOW_DRY_RUN_READY,
            summary="follow pass",
        ),
    )
    report = run_external_dry_run_sequence(
        run_id="unit-sequence-ready",
        log_dir=tmp_path,
        region=None,
        window_title="pr0p",
        backend="ffmpeg",
        bbox_file=bbox_file,
        duration_s=0.5,
        hz=10.0,
        tracker_type="KCF",
        enable_pitch=True,
        desired_target_width=120.0,
        min_found_ratio=0.8,
        max_loss_events=0,
        max_yaw_axis=0.8,
        max_pitch_axis=0.8,
        window_exact=False,
        window_case_sensitive=False,
        window_min_width=32,
        window_min_height=32,
    )
    assert report.status == EXTERNAL_DRY_RUN_READY
    assert report.reasons == []
    assert report.evidence_paths["preflight_report_json"] is not None
    assert report.evidence_paths["follow_report_json"] is not None
    steps = {step["step"]: step for step in report.metrics["sequence_steps"]}
    assert steps["status"]["status"] == PASS
    assert steps["status"]["summary"] == "required external dry-run setup checks passed"
    assert steps["status"]["source_status"] == WAITING
    assert steps["preflight"]["status"] == EXTERNAL_WINDOW_READY_DRY
    assert steps["follow"]["status"] == EXTERNAL_FOLLOW_DRY_RUN_READY
    assert report.metrics["live_readiness"] == READY_FOR_OPTIONAL_LIVE_INPUT
    assert any(
        "--ack-live-input" in note
        for note in report.metrics["live_readiness_notes"]
    )


def test_external_dry_run_sequence_region_bypasses_window_title_gate(
    monkeypatch,
    tmp_path,
):
    bbox_file = tmp_path / "bbox.json"
    write_status_json(bbox_file, {"bbox": [10, 20, 80, 80]})
    region = {"left": 10, "top": 20, "width": 320, "height": 240}
    seen = {"preflight": None, "follow": None}
    monkeypatch.setattr(
        external_dry_run_sequence_module,
        "build_status_report",
        lambda **_kwargs: sequence_status_report(window=WAITING),
    )

    def fake_preflight(**kwargs):
        seen["preflight"] = kwargs
        return ExternalWindowPreflightReport(
            run_id="unit-preflight",
            started_at="now",
            status=EXTERNAL_WINDOW_READY_DRY,
            summary="preflight pass",
        )

    def fake_follow(**kwargs):
        seen["follow"] = kwargs
        return ExternalFollowSessionReport(
            run_id="unit-follow",
            started_at="now",
            status=EXTERNAL_FOLLOW_DRY_RUN_READY,
            summary="follow pass",
        )

    monkeypatch.setattr(
        external_dry_run_sequence_module,
        "run_external_preflight",
        fake_preflight,
    )
    monkeypatch.setattr(
        external_dry_run_sequence_module,
        "run_external_follow_session",
        fake_follow,
    )
    report = run_external_dry_run_sequence(
        run_id="unit-sequence-region-ready",
        log_dir=tmp_path,
        region=region,
        window_title="pr0p",
        backend="ffmpeg",
        bbox_file=bbox_file,
        duration_s=0.5,
        hz=10.0,
        tracker_type="KCF",
        enable_pitch=True,
        desired_target_width=120.0,
        min_found_ratio=0.8,
        max_loss_events=0,
        max_yaw_axis=0.8,
        max_pitch_axis=0.8,
        window_exact=False,
        window_case_sensitive=False,
        window_min_width=32,
        window_min_height=32,
    )
    assert report.status == EXTERNAL_DRY_RUN_READY
    assert report.reasons == []
    assert report.metrics["required_status_items"] == [
        "target_bbox",
        "gazebo_betaflight_process_boundary",
    ]
    steps = {step["step"]: step for step in report.metrics["sequence_steps"]}
    assert steps["status"]["status"] == PASS
    assert not any(
        reason.startswith("target_window:")
        for reason in steps["status"]["reasons"]
    )
    assert seen["preflight"]["region"] == region
    assert seen["follow"]["region"] == region


def test_external_dry_run_sequence_stops_on_preflight_waiting(monkeypatch, tmp_path):
    bbox_file = tmp_path / "bbox.json"
    write_status_json(bbox_file, {"bbox": [10, 20, 80, 80]})
    monkeypatch.setattr(
        external_dry_run_sequence_module,
        "build_status_report",
        lambda **_kwargs: sequence_status_report(),
    )
    monkeypatch.setattr(
        external_dry_run_sequence_module,
        "run_external_preflight",
        lambda **_kwargs: ExternalWindowPreflightReport(
            run_id="unit-preflight",
            started_at="now",
            status=WAITING,
            summary="preflight waiting",
            reasons=["PHASE_NOT_PASS:S2-real:WAITING"],
        ),
    )
    report = run_external_dry_run_sequence(
        run_id="unit-sequence-preflight-waiting",
        log_dir=tmp_path,
        region=None,
        window_title="pr0p",
        backend="ffmpeg",
        bbox_file=bbox_file,
        duration_s=0.5,
        hz=10.0,
        tracker_type="KCF",
        enable_pitch=False,
        desired_target_width=120.0,
        min_found_ratio=0.8,
        max_loss_events=0,
        max_yaw_axis=0.8,
        max_pitch_axis=0.8,
        window_exact=False,
        window_case_sensitive=False,
        window_min_width=32,
        window_min_height=32,
    )
    assert report.status == WAITING
    assert "PREFLIGHT:WAITING:PHASE_NOT_PASS:S2-real:WAITING" in report.reasons
    steps = {step["step"]: step for step in report.metrics["sequence_steps"]}
    assert steps["status"]["status"] == PASS
    assert steps["preflight"]["status"] == WAITING
    assert steps["follow"]["status"] == NOT_RUN
    assert report.metrics["live_readiness"] == WAITING_FOR_DRY_RUN


def test_external_dry_run_sequence_pre_sequence_reasons():
    report = sequence_status_report(window=WAITING, process=REJECT)
    reasons = pre_sequence_reasons(report)
    assert any(reason.startswith("target_window:WAITING") for reason in reasons)
    assert any(
        reason.startswith("gazebo_betaflight_process_boundary:REJECT")
        for reason in reasons
    )


def live_readiness_dry_report(
    *,
    status=EXTERNAL_DRY_RUN_READY,
    summary="dry pass",
    reasons=None,
):
    return ExternalDryRunSequenceReport(
        run_id="unit-dry",
        started_at="now",
        status=status,
        summary=summary,
        reasons=list(reasons or []),
    )


def run_live_readiness_test(
    tmp_path,
    *,
    bbox_file,
    ack_live_input,
    axis_expected_shifts=None,
    axis_min_shift_px=0.5,
):
    return run_live_input_readiness(
        run_id="unit-live-readiness",
        log_dir=tmp_path,
        region=None,
        window_title="pr0p",
        backend="ffmpeg",
        bbox_file=bbox_file,
        duration_s=0.5,
        hz=10.0,
        tracker_type="KCF",
        enable_pitch=True,
        desired_target_width=120.0,
        min_fps=0.0,
        min_found_ratio=0.8,
        max_loss_events=0,
        max_yaw_axis=0.8,
        max_pitch_axis=0.8,
        ack_live_input=ack_live_input,
        axis_names=("yaw", "pitch"),
        axis_value=0.05,
        window_exact=False,
        window_case_sensitive=False,
        window_min_width=32,
        window_min_height=32,
        axis_expected_shifts=axis_expected_shifts,
        axis_min_shift_px=axis_min_shift_px,
    )


def test_external_live_input_readiness_waits_when_dry_sequence_waits(
    monkeypatch,
    tmp_path,
):
    bbox_file = tmp_path / "missing-bbox.json"
    monkeypatch.setattr(
        external_live_input_readiness_module,
        "run_external_dry_run_sequence",
        lambda **_kwargs: live_readiness_dry_report(
            status=WAITING,
            summary="dry waiting",
            reasons=["target_bbox:WAITING:missing"],
        ),
    )
    monkeypatch.setattr(
        external_live_input_readiness_module,
        "run_external_preflight",
        lambda **_kwargs: pytest.fail("axis preflight should not run"),
    )
    report = run_live_readiness_test(
        tmp_path,
        bbox_file=bbox_file,
        ack_live_input=True,
    )
    assert report.status == WAITING
    assert "DRY_SEQUENCE:WAITING:target_bbox:WAITING:missing" in report.reasons
    steps = {step["step"]: step for step in report.metrics["readiness_steps"]}
    assert steps["dry_sequence"]["status"] == WAITING
    assert steps["ack_live_input"]["status"] == PASS
    assert steps["axis_response"]["status"] == NOT_RUN


def test_external_live_input_readiness_requires_ack_before_axis_response(
    monkeypatch,
    tmp_path,
):
    bbox_file = tmp_path / "bbox.json"
    write_status_json(bbox_file, {"bbox": [10, 20, 80, 80]})
    monkeypatch.setattr(
        external_live_input_readiness_module,
        "run_external_dry_run_sequence",
        lambda **_kwargs: live_readiness_dry_report(),
    )
    monkeypatch.setattr(
        external_live_input_readiness_module,
        "run_external_preflight",
        lambda **_kwargs: pytest.fail("axis preflight should require ack"),
    )
    report = run_live_readiness_test(
        tmp_path,
        bbox_file=bbox_file,
        ack_live_input=False,
    )
    assert report.status == WAITING
    assert report.reasons == [ACK_LIVE_INPUT_REQUIRED]
    assert report.evidence_paths["dry_sequence_report_json"] is not None
    assert report.evidence_paths["axis_preflight_report_json"] is None
    steps = {step["step"]: step for step in report.metrics["readiness_steps"]}
    assert steps["dry_sequence"]["status"] == EXTERNAL_DRY_RUN_READY
    assert steps["ack_live_input"]["status"] == WAITING
    assert steps["axis_response"]["status"] == NOT_RUN


def test_external_live_input_readiness_promotes_ready_after_axis_response(
    monkeypatch,
    tmp_path,
):
    bbox_file = tmp_path / "bbox.json"
    write_status_json(bbox_file, {"bbox": [10, 20, 80, 80]})
    monkeypatch.setattr(
        external_live_input_readiness_module,
        "run_external_dry_run_sequence",
        lambda **_kwargs: live_readiness_dry_report(),
    )
    monkeypatch.setattr(
        external_live_input_readiness_module,
        "run_external_preflight",
        lambda **_kwargs: ExternalWindowPreflightReport(
            run_id="unit-axis",
            started_at="now",
            status=EXTERNAL_WINDOW_READY_LIVE_INPUT,
            summary="axis pass",
        ),
    )
    report = run_live_readiness_test(
        tmp_path,
        bbox_file=bbox_file,
        ack_live_input=True,
    )
    assert report.status == EXTERNAL_LIVE_INPUT_READY
    assert report.reasons == []
    assert report.evidence_paths["axis_preflight_report_json"] is not None
    steps = {step["step"]: step for step in report.metrics["readiness_steps"]}
    assert steps["dry_sequence"]["status"] == EXTERNAL_DRY_RUN_READY
    assert steps["ack_live_input"]["status"] == PASS
    assert steps["axis_response"]["status"] == EXTERNAL_WINDOW_READY_LIVE_INPUT


def test_external_live_input_readiness_passes_signed_axis_expectation(
    monkeypatch,
    tmp_path,
):
    bbox_file = tmp_path / "bbox.json"
    write_status_json(bbox_file, {"bbox": [10, 20, 80, 80]})
    seen = {}
    monkeypatch.setattr(
        external_live_input_readiness_module,
        "run_external_dry_run_sequence",
        lambda **_kwargs: live_readiness_dry_report(),
    )

    def fake_preflight(**kwargs):
        seen.update(kwargs)
        return ExternalWindowPreflightReport(
            run_id="unit-axis",
            started_at="now",
            status=EXTERNAL_WINDOW_READY_LIVE_INPUT,
            summary="axis pass",
        )

    monkeypatch.setattr(
        external_live_input_readiness_module,
        "run_external_preflight",
        fake_preflight,
    )
    report = run_live_readiness_test(
        tmp_path,
        bbox_file=bbox_file,
        ack_live_input=True,
        axis_expected_shifts={"yaw": ("x", 1)},
        axis_min_shift_px=2.0,
    )
    assert report.status == EXTERNAL_LIVE_INPUT_READY
    assert seen["axis_expected_shifts"] == {"yaw": ("x", 1)}
    assert seen["axis_min_shift_px"] == 2.0
    assert report.metrics["axis_expected_shifts"] == {"yaw": "+x"}


def live_follow_readiness_report(
    *,
    status=EXTERNAL_LIVE_INPUT_READY,
    summary="readiness pass",
    reasons=None,
):
    return ExternalLiveInputReadinessReport(
        run_id="unit-readiness",
        started_at="now",
        status=status,
        summary=summary,
        reasons=list(reasons or []),
    )


def run_live_follow_sequence_test(
    tmp_path,
    *,
    bbox_file,
    ack_live_input,
):
    return run_live_follow_sequence(
        run_id="unit-live-follow-sequence",
        log_dir=tmp_path,
        region=None,
        window_title="pr0p",
        backend="ffmpeg",
        bbox_file=bbox_file,
        duration_s=0.5,
        hz=10.0,
        tracker_type="KCF",
        enable_pitch=True,
        desired_target_width=120.0,
        min_fps=0.0,
        min_found_ratio=0.8,
        max_loss_events=0,
        max_yaw_axis=0.8,
        max_pitch_axis=0.8,
        ack_live_input=ack_live_input,
        live_max_duration_s=2.0,
        axis_names=("yaw", "pitch"),
        axis_value=0.05,
        window_exact=False,
        window_case_sensitive=False,
        window_min_width=32,
        window_min_height=32,
    )


def test_external_live_follow_sequence_requires_ack_before_any_live_gate(
    monkeypatch,
    tmp_path,
):
    bbox_file = tmp_path / "bbox.json"
    write_status_json(bbox_file, {"bbox": [10, 20, 80, 80]})
    monkeypatch.setattr(
        external_live_follow_sequence_module,
        "run_live_input_readiness",
        lambda **_kwargs: pytest.fail("readiness should require live ack first"),
    )
    monkeypatch.setattr(
        external_live_follow_sequence_module,
        "run_external_follow_session",
        lambda **_kwargs: pytest.fail("follow should require live ack first"),
    )
    report = run_live_follow_sequence_test(
        tmp_path,
        bbox_file=bbox_file,
        ack_live_input=False,
    )
    assert report.status == WAITING
    assert report.reasons == [ACK_LIVE_INPUT_REQUIRED]
    steps = {step["step"]: step for step in report.metrics["sequence_steps"]}
    assert steps["ack_live_input"]["status"] == WAITING
    assert steps["live_input_readiness"]["status"] == "NOT_RUN"
    assert steps["live_follow"]["status"] == "NOT_RUN"


def test_external_live_follow_sequence_waits_when_readiness_waits(
    monkeypatch,
    tmp_path,
):
    bbox_file = tmp_path / "bbox.json"
    write_status_json(bbox_file, {"bbox": [10, 20, 80, 80]})
    monkeypatch.setattr(
        external_live_follow_sequence_module,
        "run_live_input_readiness",
        lambda **_kwargs: live_follow_readiness_report(
            status=WAITING,
            summary="readiness waiting",
            reasons=["ACK_LIVE_INPUT_REQUIRED"],
        ),
    )
    monkeypatch.setattr(
        external_live_follow_sequence_module,
        "run_external_follow_session",
        lambda **_kwargs: pytest.fail("follow should wait for readiness"),
    )
    report = run_live_follow_sequence_test(
        tmp_path,
        bbox_file=bbox_file,
        ack_live_input=True,
    )
    assert report.status == WAITING
    assert "LIVE_INPUT_READINESS:WAITING:ACK_LIVE_INPUT_REQUIRED" in report.reasons
    steps = {step["step"]: step for step in report.metrics["sequence_steps"]}
    assert steps["ack_live_input"]["status"] == PASS
    assert steps["live_input_readiness"]["status"] == WAITING
    assert steps["live_follow"]["status"] == "NOT_RUN"


def test_external_live_follow_sequence_promotes_complete_after_live_follow(
    monkeypatch,
    tmp_path,
):
    bbox_file = tmp_path / "bbox.json"
    write_status_json(bbox_file, {"bbox": [10, 20, 80, 80]})
    monkeypatch.setattr(
        external_live_follow_sequence_module,
        "run_live_input_readiness",
        lambda **_kwargs: live_follow_readiness_report(),
    )
    monkeypatch.setattr(
        external_live_follow_sequence_module,
        "run_external_follow_session",
        lambda **_kwargs: ExternalFollowSessionReport(
            run_id="unit-live-follow",
            started_at="now",
            status=EXTERNAL_FOLLOW_LIVE_RUN_COMPLETE,
            summary="live follow pass",
        ),
    )
    report = run_live_follow_sequence_test(
        tmp_path,
        bbox_file=bbox_file,
        ack_live_input=True,
    )
    assert report.status == EXTERNAL_LIVE_FOLLOW_COMPLETE
    assert report.reasons == []
    assert report.evidence_paths["readiness_report_json"] is not None
    assert report.evidence_paths["follow_report_json"] is not None
    steps = {step["step"]: step for step in report.metrics["sequence_steps"]}
    assert steps["ack_live_input"]["status"] == PASS
    assert steps["live_input_readiness"]["status"] == EXTERNAL_LIVE_INPUT_READY
    assert steps["live_follow"]["status"] == EXTERNAL_FOLLOW_LIVE_RUN_COMPLETE
