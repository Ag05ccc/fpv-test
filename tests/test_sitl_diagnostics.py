import sys
import struct
from pathlib import Path

TOOLS_DIR = Path(__file__).resolve().parents[1] / "tools"
if str(TOOLS_DIR) not in sys.path:
    sys.path.insert(0, str(TOOLS_DIR))

from gazebo_stats_monitor import parse_stats  # noqa: E402
from sitl_diagnostics import (  # noqa: E402
    ExternalRcSampler,
    MSP_ADVANCED_CONFIG,
    MSP_ATTITUDE,
    MSP_DEBUG,
    MSP_MOTOR,
    MSP_RC,
    MSP_STATUS_EX,
    VirtualRcSampler,
    analyze_motor_mapping,
    channel_deltas,
    sample_gazebo_pose,
    sample_msp,
    summarize,
)


def write_model(root, mapping):
    model_dir = root / "models" / "betaloop_iris_with_standoffs"
    model_dir.mkdir(parents=True)
    rotors = "\n".join(
        """
        <rotor id="{rotor_id}">
          <jointName>{joint}</jointName>
          <turningDirection>{direction}</turningDirection>
        </rotor>
        """.format(rotor_id=rotor_id, joint=joint, direction=direction)
        for rotor_id, joint, direction in mapping
    )
    (model_dir / "model.sdf").write_text(
        """<sdf version="1.9">
  <model name="iris">
    <link name="rotor_0"><pose>0.13 -0.22 0.023 0 0 0</pose></link>
    <link name="rotor_1"><pose>-0.13 0.20 0.023 0 0 0</pose></link>
    <link name="rotor_2"><pose>0.13 0.22 0.023 0 0 0</pose></link>
    <link name="rotor_3"><pose>-0.13 -0.20 0.023 0 0 0</pose></link>
    <plugin name="gz::sim::systems::BetaflightPlugin" filename="BetaflightPlugin">
      {rotors}
    </plugin>
  </model>
</sdf>
""".format(rotors=rotors),
        encoding="utf-8",
    )


def write_runtime_model(root, mapping):
    model_dir = root / "betaloop_iris_with_standoffs"
    model_dir.mkdir(parents=True)
    rotors = "\n".join(
        """
        <rotor id="{rotor_id}">
          <jointName>{joint}</jointName>
          <turningDirection>{direction}</turningDirection>
        </rotor>
        """.format(rotor_id=rotor_id, joint=joint, direction=direction)
        for rotor_id, joint, direction in mapping
    )
    (model_dir / "model.sdf").write_text(
        """<sdf version="1.9">
  <model name="iris">
    <link name="rotor_0"><pose>0.13 -0.22 0.023 0 0 0</pose></link>
    <link name="rotor_1"><pose>-0.13 0.20 0.023 0 0 0</pose></link>
    <link name="rotor_2"><pose>0.13 0.22 0.023 0 0 0</pose></link>
    <link name="rotor_3"><pose>-0.13 -0.20 0.023 0 0 0</pose></link>
    <plugin name="gz::sim::systems::BetaflightPlugin" filename="BetaflightPlugin">
      {rotors}
    </plugin>
  </model>
</sdf>
""".format(rotors=rotors),
        encoding="utf-8",
    )


def test_parse_gazebo_stats_extracts_world_fields():
    stats = parse_stats(
        """
        sim_time { sec: 21 nsec: 778000000 }
        real_time { sec: 154 nsec: 842390768 }
        iterations: 21778
        real_time_factor: 0.72078565636543834
        step_size { nsec: 1000000 }
        """
    )

    assert stats["rtf"] == 0.72078565636543834
    assert stats["iterations"] == 21778
    assert stats["sim_time"] == 21.778
    assert stats["step_size"] == 0.001


def test_motor_mapping_identity_plugin_matches_betaflight_sitl_remap(tmp_path):
    write_model(tmp_path, [
        (0, "rotor_0_joint", "ccw"),
        (1, "rotor_1_joint", "ccw"),
        (2, "rotor_2_joint", "cw"),
        (3, "rotor_3_joint", "cw"),
    ])

    result = analyze_motor_mapping(tmp_path)

    assert result["ok"] is True
    assert result["double_remap_risk"] is False
    assert result["mismatched_bf_motors"] == []


def test_motor_mapping_current_double_remap_pattern_is_flagged(tmp_path):
    write_model(tmp_path, [
        (0, "rotor_3_joint", "cw"),
        (1, "rotor_0_joint", "ccw"),
        (2, "rotor_1_joint", "ccw"),
        (3, "rotor_2_joint", "cw"),
    ])

    result = analyze_motor_mapping(tmp_path)

    assert result["ok"] is False
    assert result["double_remap_risk"] is True
    assert result["mismatched_bf_motors"] == [0, 1, 2, 3]
    assert result["bf_to_physical"]["0"]["actual_joint"] == "rotor_2_joint"


def test_channel_deltas_use_msp_rpyt_order_for_throttle_and_yaw():
    rows = channel_deltas(
        [1500, 1501, 1000, 1503, 2000, 1000, 1500, 1500],
        [1500, 1501, 1503, 1000, 2000, 1000, 1500, 1500],
    )

    assert [row["delta"] for row in rows[:4]] == [0, 0, 0, 0]
    assert rows[4]["label"] == "AUX1 ARM"
    assert rows[5]["label"] == "AUX2 Kenet"


def test_sample_msp_includes_debug_mode_and_values(monkeypatch):
    advanced = bytearray()
    advanced += bytes([1, 4, 1, 5])
    advanced += struct.pack("<H", 480)
    advanced += struct.pack("<H", 550)
    advanced += bytes([0, 1, 0, 0, 32])
    advanced += struct.pack("<H", 125)
    advanced += struct.pack("<H", 7)
    advanced += bytes([1, 37, 135])
    debug = struct.pack("<8h", 1, -2, 3, -4, 5, -6, 7, -8)
    requested_codes = []

    def fake_request_many(_host, _port, codes, _timeout):
        requested_codes.extend(codes)
        return {
            MSP_STATUS_EX: b"",
            MSP_RC: struct.pack("<4H", 1500, 1500, 1500, 1000),
            MSP_MOTOR: struct.pack("<4H", 1000, 1000, 1000, 1000),
            MSP_ATTITUDE: struct.pack("<hhH", 0, 0, 90),
            MSP_ADVANCED_CONFIG: bytes(advanced),
            MSP_DEBUG: debug,
        }

    monkeypatch.setattr("sitl_diagnostics.msp_request_many", fake_request_many)

    sample = sample_msp("127.0.0.1", 5761, 1.0, [], [])

    assert MSP_ADVANCED_CONFIG in requested_codes
    assert MSP_DEBUG in requested_codes
    assert sample["connected"] is True
    assert sample["debug_mode"] == 37
    assert sample["advanced_config"]["debug_mode_count"] == 135
    assert sample["debug"] == [1, -2, 3, -4, 5, -6, 7, -8]


def test_virtual_rc_sampler_provides_expected_pilot_channels():
    class Args:
        virtual_roll = 1500
        virtual_pitch = 1500
        virtual_throttle = 1600
        virtual_yaw = 1500
        virtual_arm_pwm = 2000
        virtual_kenet_pwm = 1000
        virtual_mode_pwm = 1500
        force_mode_pwm = None

    sample = VirtualRcSampler(Args()).sample()

    assert sample["connected"] is True
    assert sample["source"] == "virtual"
    assert sample["pilot_channels"][:7] == [1500, 1500, 1600, 1500, 2000, 1000, 1500]


def test_external_rc_sampler_does_not_invent_expected_pilot_channels():
    sample = ExternalRcSampler().sample()

    assert sample["connected"] is True
    assert sample["source"] == "external"
    assert sample["pilot_channels"] == []


def test_sample_gazebo_pose_combines_dynamic_pose_and_imu(monkeypatch):
    def fake_topic_once(topic, timeout):
        assert timeout == 0.25
        if topic.endswith("/dynamic_pose/info"):
            return """
pose {
  name: "iris"
  position { z: 1.5 }
  orientation {
    x: 0.7071068
    y: 0
    z: 0
    w: 0.7071068
  }
}
"""
        return """
orientation {
  x: 0
  y: 0.7071068
  z: 0
  w: 0.7071068
}
"""

    monkeypatch.setattr("sitl_diagnostics.gz_topic_once", fake_topic_once)

    sample = sample_gazebo_pose("betaloop_demo", "iris", "/imu", 0.25)

    assert sample["ok"] is True
    assert sample["pose_topic"] == "/world/betaloop_demo/dynamic_pose/info"
    assert round(sample["pose"]["euler_deg"]["roll"], 1) == 90.0
    assert round(sample["imu"]["euler_deg"]["pitch"], 1) == 90.0


def test_summary_warns_on_arm_blockers_and_mapping_mismatch(tmp_path):
    write_model(tmp_path, [
        (0, "rotor_3_joint", "cw"),
        (1, "rotor_0_joint", "ccw"),
        (2, "rotor_1_joint", "ccw"),
        (3, "rotor_2_joint", "cw"),
    ])
    mapping = analyze_motor_mapping(tmp_path)
    summary = summarize([
        {
            "gazebo": {"rtf": 0.5},
            "dashboard": {"ok": True, "elapsed_ms": 10, "state_time_age_ms": 20},
            "joystick": {"connected": True, "elapsed_ms": 1},
            "msp": {
                "connected": True,
                "elapsed_ms": 2,
                "attitude": {"roll": 180.0, "pitch": 0.0},
                "status": {
                    "active_modes": [],
                    "arming_disable_names": ["ANGLE"],
                },
            },
            "channels": [{"delta": 0}],
        }
    ], mapping)

    assert "ANGLE" in summary["arming_disable_names"]
    assert summary["max_abs_roll"] == 180.0
    assert any("real_time_factor" in warning for warning in summary["warnings"])
    assert any("Motor mapping" in warning for warning in summary["warnings"])


def test_summary_detects_runtime_motor_map_fix_from_gazebo_env(tmp_path):
    source_root = tmp_path / "source"
    write_model(source_root, [
        (0, "rotor_3_joint", "cw"),
        (1, "rotor_0_joint", "ccw"),
        (2, "rotor_1_joint", "ccw"),
        (3, "rotor_2_joint", "cw"),
    ])
    runtime_root = tmp_path / "runtime"
    write_runtime_model(runtime_root, [
        (0, "rotor_0_joint", "ccw"),
        (1, "rotor_1_joint", "ccw"),
        (2, "rotor_2_joint", "cw"),
        (3, "rotor_3_joint", "cw"),
    ])
    mapping = analyze_motor_mapping(source_root)

    summary = summarize([
        {
            "gazebo": {"rtf": 1.0},
            "dashboard": {"ok": False},
            "joystick": {"connected": True, "elapsed_ms": 1},
            "msp": {
                "connected": True,
                "elapsed_ms": 2,
                "attitude": {"roll": 0.0, "pitch": 0.0},
                "status": {
                    "active_modes": [],
                    "arming_disable_names": [],
                },
            },
            "channels": [{"delta": 0}],
            "processes": [
                {
                    "cmd": "gz sim -r -s /tmp/world.sdf",
                    "env": {"SDF_PATH": str(runtime_root)},
                }
            ],
        }
    ], mapping)

    assert summary["runtime_motor_fix_active"] is True
    assert summary["runtime_motor_fix_path"] == str(
        runtime_root / "betaloop_iris_with_standoffs" / "model.sdf"
    )
    assert any("Source SDF motor mapping bozuk" in warning for warning in summary["warnings"])
