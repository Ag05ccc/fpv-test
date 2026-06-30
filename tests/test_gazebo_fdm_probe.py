import struct
import sys
from pathlib import Path


TOOLS_DIR = Path(__file__).resolve().parents[1] / "tools"
if str(TOOLS_DIR) not in sys.path:
    sys.path.insert(0, str(TOOLS_DIR))

from gazebo_fdm_probe import (  # noqa: E402
    FDM_DOUBLE_COUNT,
    FDM_PACKET_SIZE,
    angular_sign_summary,
    parse_fdm_packet,
    select_axis_pattern,
)


def test_parse_fdm_packet_extracts_named_fields():
    values = [float(index) for index in range(FDM_DOUBLE_COUNT)]
    packet = struct.pack("<%dd" % FDM_DOUBLE_COUNT, *values)

    parsed = parse_fdm_packet(packet)

    assert len(packet) == FDM_PACKET_SIZE
    assert parsed["timestamp"] == 0.0
    assert parsed["imu_angular_velocity"] == {"x": 1.0, "y": 2.0, "z": 3.0}
    assert parsed["imu_linear_acceleration"] == {"x": 4.0, "y": 5.0, "z": 6.0}
    assert parsed["imu_orientation_quat"] == {"w": 7.0, "x": 8.0, "y": 9.0, "z": 10.0}
    assert parsed["velocity"] == {"x": 11.0, "y": 12.0, "z": 13.0}
    assert parsed["position"] == {"x": 14.0, "y": 15.0, "z": 16.0}
    assert parsed["esc_rpm"] == [33.0, 34.0, 35.0, 36.0]


def test_parse_fdm_packet_rejects_short_packet():
    try:
        parse_fdm_packet(b"\x00" * (FDM_PACKET_SIZE - 1))
    except ValueError as exc:
        assert "too short" in str(exc)
    else:
        raise AssertionError("short FDM packet should fail")


def test_angular_sign_summary_expects_yaw_inversion():
    gazebo = {"angular_velocity": {"x": 0.2, "y": -0.3, "z": 0.4}}
    fdm = {"imu_angular_velocity": {"x": 0.21, "y": -0.31, "z": -0.39}}

    summary = angular_sign_summary(gazebo, fdm, min_abs=0.02)

    assert summary["x"]["status"] == "ok"
    assert summary["y"]["status"] == "ok"
    assert summary["z"]["expected_relation"] == "inverted"
    assert summary["z"]["observed_relation"] == "inverted"
    assert summary["z"]["status"] == "ok"


def test_angular_sign_summary_flags_yaw_mismatch():
    gazebo = {"angular_velocity": {"x": 0.2, "y": -0.3, "z": 0.4}}
    fdm = {"imu_angular_velocity": {"x": 0.21, "y": -0.31, "z": 0.39}}

    summary = angular_sign_summary(gazebo, fdm, min_abs=0.02)

    assert summary["z"]["observed_relation"] == "same"
    assert summary["z"]["status"] == "mismatch"


def test_angular_sign_summary_marks_small_motion_inconclusive():
    gazebo = {"angular_velocity": {"x": 0.001, "y": 0.0, "z": 0.4}}
    fdm = {"imu_angular_velocity": {"x": 0.001, "y": 0.0, "z": -0.4}}

    summary = angular_sign_summary(gazebo, fdm, min_abs=0.02)

    assert summary["x"]["status"] == "insufficient-motion"
    assert summary["y"]["status"] == "insufficient-motion"
    assert summary["z"]["status"] == "ok"


def test_select_axis_pattern_uses_existing_bf_patterns():
    assert select_axis_pattern("bf_cw_pair_high", 0.45, 0.04) == [
        0.49,
        0.41000000000000003,
        0.41000000000000003,
        0.49,
    ]
