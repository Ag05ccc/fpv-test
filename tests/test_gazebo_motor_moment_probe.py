import math
import struct
import sys
from pathlib import Path


TOOLS_DIR = Path(__file__).resolve().parents[1] / "tools"
if str(TOOLS_DIR) not in sys.path:
    sys.path.insert(0, str(TOOLS_DIR))

from gazebo_motor_moment_probe import (  # noqa: E402
    bf_axis_patterns,
    build_patterns,
    delta_pose,
    delta_vector,
    pack_motor_packet,
    parse_motor_map,
    parse_imu_text,
    parse_pose_text,
    quat_to_euler_deg,
    remap_speeds,
)


def test_pack_motor_packet_uses_four_little_endian_floats():
    packet = pack_motor_packet([0.0, 0.25, 0.5, 1.2])

    assert len(packet) == 16
    assert struct.unpack("<4f", packet) == (0.0, 0.25, 0.5, 1.0)


def test_quaternion_to_euler_roll_90_deg():
    half = math.sqrt(0.5)

    euler = quat_to_euler_deg(half, half, 0.0, 0.0)

    assert round(euler["roll"], 3) == 90.0
    assert round(euler["pitch"], 3) == 0.0
    assert round(euler["yaw"], 3) == 0.0


def test_parse_dynamic_pose_text_extracts_named_entity_pose():
    text = """
pose {
  name: "ground"
  id: 1
  position { z: 0 }
  orientation { w: 1 }
}
pose {
  name: "iris"
  id: 8
  position {
    x: 1.25
    y: -0.5
    z: 0.75
  }
  orientation {
    x: 0.7071068
    y: 0
    z: 0
    w: 0.7071068
  }
}
"""

    pose = parse_pose_text(text, "iris")

    assert pose is not None
    assert pose["position"] == {"x": 1.25, "y": -0.5, "z": 0.75}
    assert round(pose["euler_deg"]["roll"], 1) == 90.0


def test_parse_imu_text_extracts_orientation_and_vectors():
    text = """
orientation {
  x: 0
  y: 0
  z: 0.7071068
  w: 0.7071068
}
angular_velocity {
  x: 0.1
  y: -0.2
  z: 0.3
}
linear_acceleration {
  x: 1
  y: 2
  z: -9.8
}
"""

    imu = parse_imu_text(text)

    assert round(imu["euler_deg"]["yaw"], 1) == 90.0
    assert imu["angular_velocity"] == {"x": 0.1, "y": -0.2, "z": 0.3}
    assert imu["linear_acceleration"] == {"x": 1.0, "y": 2.0, "z": -9.8}


def test_delta_pose_reports_euler_difference():
    before = {"euler_deg": {"roll": 1.0, "pitch": -2.0, "yaw": 3.0}}
    after = {"euler_deg": {"roll": 4.5, "pitch": -1.0, "yaw": 1.0}}

    assert delta_pose(before, after) == {"roll": 3.5, "pitch": 1.0, "yaw": -2.0}


def test_delta_vector_reports_imu_angular_difference():
    before = {"angular_velocity": {"x": 0.1, "y": -0.2, "z": 0.3}}
    after = {"angular_velocity": {"x": 0.3, "y": -0.1, "z": -0.2}}

    assert delta_vector(before, after, "angular_velocity") == {
        "x": 0.19999999999999998,
        "y": 0.1,
        "z": -0.5,
    }


def test_bf_axis_patterns_use_patched_iris_motor_order():
    patterns = dict(bf_axis_patterns(0.45, 0.04))

    assert patterns["bf_right_pair_high"] == [0.49, 0.49, 0.41000000000000003, 0.41000000000000003]
    assert patterns["bf_front_pair_high"] == [0.41000000000000003, 0.49, 0.41000000000000003, 0.49]
    assert patterns["bf_cw_pair_high"] == [0.49, 0.41000000000000003, 0.41000000000000003, 0.49]


def test_build_patterns_can_select_only_axis_patterns():
    class Args:
        base_speed = 0.45
        pulse_speed = 0.62
        axis_delta = 0.04
        pattern_set = "axis"

    names = [name for name, _speeds in build_patterns(Args())]

    assert names == [
        "bf_right_pair_high",
        "bf_left_pair_high",
        "bf_front_pair_high",
        "bf_rear_pair_high",
        "bf_cw_pair_high",
        "bf_ccw_pair_high",
    ]


def test_parse_motor_map_requires_permutation():
    assert parse_motor_map("0,3,2,1") == [0, 3, 2, 1]
    assert parse_motor_map("identity") == [0, 1, 2, 3]
    assert parse_motor_map("bf-sitl") == [3, 0, 1, 2]


def test_remap_speeds_maps_logical_bf_indexes_to_physical_packet_indexes():
    assert remap_speeds([0.1, 0.2, 0.3, 0.4], [0, 3, 2, 1]) == [0.1, 0.4, 0.3, 0.2]
