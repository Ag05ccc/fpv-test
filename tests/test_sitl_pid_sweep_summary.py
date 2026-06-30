import json
import math
import sys
from pathlib import Path


TOOLS_DIR = Path(__file__).resolve().parents[1] / "tools"
if str(TOOLS_DIR) not in sys.path:
    sys.path.insert(0, str(TOOLS_DIR))

from sitl_pid_sweep_summary import (  # noqa: E402
    active_flight_phase,
    angle_delta_deg,
    diagnostic_snapshot,
    diagnostics_events,
    first_virtual_nudge_time,
    raw_motor_events,
    summarize_case,
)


def write_jsonl(path, records):
    with path.open("w", encoding="utf-8") as handle:
        for record in records:
            handle.write(json.dumps(record) + "\n")


def test_first_virtual_nudge_time_detects_step_or_yaw_channel(tmp_path):
    path = tmp_path / "rc.jsonl"
    write_jsonl(path, [
        {"event": "virtual_rc_frame", "time": 10.0, "step": "takeoff-hold", "channels": [1500, 1500, 1750, 1500]},
        {"event": "virtual_rc_step_start", "time": 12.5, "step": "takeoff-hold-nudge", "channels": [1500, 1500, 1750, 1504]},
    ])

    assert first_virtual_nudge_time(path) == 12.5


def test_angle_delta_deg_wraps_across_180():
    assert angle_delta_deg(-179.0, 179.0) == 2.0
    assert angle_delta_deg(179.0, -179.0) == -2.0


def test_active_flight_phase_requires_arm_and_high_throttle():
    record = {
        "msp": {
            "rc_channels": [1500, 1500, 1500, 1600],
            "status": {"active_modes": ["ARM", "ANGLE"]},
        }
    }

    assert active_flight_phase(record, 1500) is True
    record["msp"]["rc_channels"][3] = 1400
    assert active_flight_phase(record, 1500) is False


def test_diagnostic_snapshot_extracts_nudge_state():
    record = {
        "time": 10.5,
        "sample_index": 3,
        "msp": {
            "rc_channels": [1500, 1500, 1500, 1750],
            "status": {"active_modes": ["ARM", "ANGLE"]},
            "debug_mode": 5,
            "debug": [1, -2, 3, -4, 5, -6, 7, -8],
            "attitude": {"roll": 1.0, "pitch": 2.0, "yaw": 270},
            "motor": [1200, 1210, 1190, 1205],
        },
        "gazebo_pose": {
            "imu": {"angular_velocity": {"z": 0.25}},
            "pose": {"position": {"z": 3.5}, "euler_deg": {"yaw": 91.0}},
        },
    }

    result = diagnostic_snapshot(record, active_min_throttle=1500)

    assert result["active"] is True
    assert result["debug_mode"] == 5
    assert result["debug"] == [1.0, -2.0, 3.0, -4.0, 5.0, -6.0, 7.0, -8.0]
    assert result["z"] == 3.5
    assert result["roll"] == 1.0
    assert result["pitch"] == 2.0
    assert result["fc_yaw"] == 270.0
    assert result["pose_yaw"] == 91.0
    assert result["imu_yaw_rate"] == 0.25
    assert result["throttle"] == 1750
    assert result["motor_spread"] == 20.0


def test_diagnostics_events_reports_threshold_times(tmp_path):
    path = tmp_path / "diag.jsonl"
    write_jsonl(path, [
        {
            "event": "diagnostic_sample",
            "time": 10.0,
            "sample_index": 1,
            "msp": {
                "rc_channels": [1500, 1500, 1500, 1600],
                "status": {"active_modes": ["ARM", "ANGLE"]},
                "debug_mode": 5,
                "debug": [1, -2, 3, -4, 5, -6, 7, -8],
                "attitude": {"roll": 0.0, "pitch": 0.0},
                "motor": [1200, 1200, 1200, 1200],
            },
            "gazebo_pose": {
                "imu": {"angular_velocity": {"z": 0.1}},
                "pose": {"position": {"z": 0.2}, "euler_deg": {"yaw": 90.0}},
            },
        },
        {
            "event": "diagnostic_sample",
            "time": 11.0,
            "sample_index": 2,
            "msp": {
                "rc_channels": [1500, 1500, 1500, 1600],
                "status": {"active_modes": ["ARM", "ANGLE"]},
                "debug_mode": 5,
                "debug": [-10, 20, -30, 40, -50, 60, -70, 80],
                "attitude": {"roll": 36.0, "pitch": 1.0},
                "motor": [1000, 1500, 1600, 1700],
            },
            "gazebo_pose": {
                "imu": {"angular_velocity": {"z": 0.7}},
                "pose": {"position": {"z": 2.2}, "euler_deg": {"yaw": 96.0}},
            },
        },
        {
            "event": "diagnostic_sample",
            "time": 12.0,
            "sample_index": 3,
            "msp": {
                "rc_channels": [1500, 1500, 1500, 1000],
                "status": {"active_modes": []},
                "debug_mode": 5,
                "debug": [100, -200, 300, -400, 500, -600, 700, -800],
                "attitude": {"roll": 90.0, "pitch": 1.0},
                "motor": [1000, 1000, 1000, 1000],
            },
            "gazebo_pose": {
                "imu": {"angular_velocity": {"z": 0.0}},
                "pose": {"position": {"z": 1.5}, "euler_deg": {"yaw": 100.0}},
            },
        },
    ])

    result = diagnostics_events(
        path,
        attitude_threshold=35.0,
        motor_spread_threshold=400.0,
        yaw_rate_threshold=0.5,
        yaw_delta_threshold=5.0,
        active_min_throttle=1500,
        nudge_time=10.6,
    )

    assert result["altitude_gain"] == 2.0
    assert result["max_abs_roll"] == 90.0
    assert result["max_abs_active_roll"] == 36.0
    assert result["first_attitude"]["time"] == 11.0
    assert result["first_active_attitude"]["time"] == 11.0
    assert result["first_motor_spread"]["spread"] == 700.0
    assert result["first_imu_yaw_rate"]["time"] == 11.0
    assert result["first_pose_yaw_delta"]["yaw_delta"] == 6.0
    assert result["debug_mode"] == 5
    assert result["max_abs_debug"][:4] == [100.0, 200.0, 300.0, 400.0]
    assert result["max_abs_debug"][4:] == [500.0, 600.0, 700.0, 800.0]
    assert result["max_abs_active_debug"][:4] == [10.0, 20.0, 30.0, 40.0]
    assert result["max_abs_active_debug"][4:] == [50.0, 60.0, 70.0, 80.0]
    assert result["sample_near_nudge"]["sample_index"] == 2
    assert result["sample_near_nudge"]["debug"][:4] == [-10.0, 20.0, -30.0, 40.0]
    assert math.isclose(result["sample_near_nudge"]["dt_from_nudge"], 0.4)
    assert result["sample_near_nudge"]["z"] == 2.2
    assert result["sample_before_nudge"]["sample_index"] == 1
    assert math.isclose(result["sample_before_nudge"]["dt_from_nudge"], -0.6)
    assert result["sample_after_nudge"]["sample_index"] == 2
    assert math.isclose(result["sample_after_nudge"]["dt_from_nudge"], 0.4)


def test_raw_motor_events_reports_axis_bias_for_first_large_spread(tmp_path):
    path = tmp_path / "motor.jsonl"
    write_jsonl(path, [
        {
            "event": "motor_udp_sample",
            "time": 20.0,
            "received_at": 20.0,
            "motor_spread_raw": 10.0,
            "axis_bias_raw": {"yaw_cw_minus_ccw": 10.0},
        },
        {
            "event": "motor_udp_sample",
            "time": 21.0,
            "received_at": 21.0,
            "motor_spread_raw": 410.0,
            "axis_bias_raw": {
                "roll_left_minus_right": 1.0,
                "pitch_rear_minus_front": 2.0,
                "yaw_cw_minus_ccw": 407.0,
            },
        },
    ])

    result = raw_motor_events(path, spread_threshold=400.0, axis_thresholds=(5.0, 100.0, 400.0))

    assert result["max_raw_spread"] == 410.0
    assert result["first_large_spread"]["time"] == 21.0
    assert result["first_large_spread"]["axis_bias_raw"]["yaw_cw_minus_ccw"] == 407.0
    assert result["first_any_axis_thresholds"]["5.0"]["time"] == 20.0
    assert result["first_any_axis_thresholds"]["5.0"]["axis"] == "yaw_cw_minus_ccw"
    assert result["first_axis_thresholds"]["yaw_cw_minus_ccw"]["400.0"]["value"] == 407.0


def test_summarize_case_offsets_events_from_virtual_nudge(tmp_path):
    rc = tmp_path / "rc.jsonl"
    diag = tmp_path / "diag.jsonl"
    motor = tmp_path / "motor.jsonl"
    write_jsonl(rc, [
        {"event": "virtual_rc_step_start", "time": 100.0, "step": "takeoff-hold-nudge", "channels": [1500, 1500, 1750, 1504]},
    ])
    write_jsonl(diag, [
        {
            "event": "diagnostic_sample",
            "time": 101.0,
            "sample_index": 1,
            "msp": {
                "rc_channels": [1500, 1500, 1500, 1600],
                "status": {"active_modes": ["ARM", "ANGLE"]},
                "debug_mode": 7,
                "debug": [11, 22, -33, -44, 55, -66, 77, -88],
                "attitude": {"roll": 40.0, "pitch": 0.0},
                "motor": [1000, 1000, 1000, 1000],
            },
            "gazebo_pose": {"pose": {"position": {"z": 0.2}}},
        },
    ])
    write_jsonl(motor, [
        {
            "event": "motor_udp_sample",
            "time": 100.5,
            "received_at": 100.5,
            "motor_spread_raw": 450.0,
            "axis_bias_raw": {"yaw_cw_minus_ccw": 430.0},
        },
    ])

    result = summarize_case(
        "case",
        diag,
        motor,
        rc,
        attitude_threshold=35.0,
        spread_threshold=400.0,
        yaw_rate_threshold=0.5,
        yaw_delta_threshold=5.0,
        active_min_throttle=1500,
        axis_thresholds=(25.0, 100.0, 400.0),
    )

    assert result["first_raw_spread_after_nudge_s"] == 0.5
    assert result["first_attitude_after_nudge_s"] == 1.0
    assert result["first_active_attitude_after_nudge_s"] == 1.0
    assert result["nudge_z"] == 0.2
    assert result["nudge_roll"] == 40.0
    assert result["nudge_throttle"] == 1600
    assert result["debug_mode"] == 7
    assert result["nudge_debug0"] == 11.0
    assert result["nudge_debug3"] == -44.0
    assert result["nudge_debug7"] == -88.0
    assert result["max_abs_debug2"] == 33.0
    assert result["max_abs_debug7"] == 88.0
    assert result["max_abs_active_debug2"] == 33.0
    assert result["max_abs_active_debug7"] == 88.0
    assert result["after_nudge_z"] == 0.2
    assert result["after_nudge_roll"] == 40.0
    assert result["first_raw_axis_25_after_nudge_s"] == 0.5
    assert result["first_raw_axis_100_after_nudge_s"] == 0.5
    assert result["first_raw_axis_400_after_nudge_s"] == 0.5
    assert result["first_raw_axis_400_name"] == "yaw_cw_minus_ccw"
