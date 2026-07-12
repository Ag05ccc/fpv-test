import json
import sys
from argparse import Namespace
from pathlib import Path


TOOLS_DIR = Path(__file__).resolve().parents[1] / "tools"
if str(TOOLS_DIR) not in sys.path:
    sys.path.insert(0, str(TOOLS_DIR))

from sitl_virtual_takeoff_check import (  # noqa: E402
    SAFE_MANUAL_RATE,
    SAFE_MANUAL_RATE_LIMIT,
    SAFE_MANUAL_RC_RATE,
    SAFE_YAW_RATE,
    SAFE_YAW_RATE_LIMIT,
    SAFE_YAW_RC_RATE,
    analyze_diagnostics,
    analyze_motor_udp_log,
    apply_safe_manual_authority,
    apply_safe_yaw_authority,
    betaflight_work_dir,
    bootstrap_gazebo_state,
    build_betaflight_config_import_command,
    build_debug_config_command,
    build_diagnostics_command,
    build_gazebo_command,
    build_mixer_config_command,
    build_motor_udp_command,
    build_pid_config_command,
    build_rate_config_command,
    build_virtual_rc_command,
    estimate_virtual_rc_timeout,
    estimate_motor_udp_duration,
    lockstep_wait_env_value,
    looptime_us_for_step,
    parse_pid_triplet,
    resolve_betaflight_config_file,
    wait_for_msp_ready,
)


def write_jsonl(path, records):
    with path.open("w", encoding="utf-8") as handle:
        for record in records:
            handle.write(json.dumps(record) + "\n")


def args(**overrides):
    values = {
        "rc_driver": "virtual",
        "diagnostic_samples": 75,
        "diagnostic_interval": 0.1,
        "dashboard_timeout": 0.1,
        "msp_host": "127.0.0.1",
        "msp_port": 5761,
        "msp_timeout": 1.0,
        "world_name": "betaloop_demo",
        "gazebo_timeout": 0.5,
        "throttle": 1750,
        "mode_pwm": 1500,
        "virtual_roll": 1500,
        "virtual_pitch": 1500,
        "virtual_yaw": 1500,
        "nudge_delay_seconds": None,
        "nudge_roll": None,
        "nudge_pitch": None,
        "nudge_yaw": None,
        "hold_seconds": 8.0,
        "virtual_print_hz": 1.0,
        "virtual_rc_timeout": None,
        "yaw_motors_reversed": "keep",
        "zero_yaw_pid": False,
        "yaw_pid": None,
        "zero_pitch_pid": False,
        "pitch_pid": None,
        "yaw_rc_rate": None,
        "yaw_rate": None,
        "yaw_rate_limit": None,
        "roll_rc_rate": None,
        "roll_rate": None,
        "roll_rate_limit": None,
        "pitch_rc_rate": None,
        "pitch_rate": None,
        "pitch_rate_limit": None,
        "safe_yaw_authority": False,
        "safe_manual_authority": False,
        "debug_mode": None,
        "no_debug_restart": False,
        "motor_udp_duration": None,
        "pre_takeoff_seconds": 1.0,
        "betaflight_cwd": "temp",
        "betaflight_config_file": None,
        "betaflight_config_import_timeout": 20.0,
        "betaflight_ready_timeout": 5.0,
        "fix_iris_imu_pose": True,
        "fix_iris_motor_map": True,
        "iris_yaw_gyro_scale": None,
        "iris_rotor_vel_p_gain": None,
        "iris_velocity_control": False,
        "iris_motor_time_constant": None,
        "iris_rotor_damping": None,
        "iris_forward_camera": False,
        "world": "betaloop_iris_betaflight_demo_harmonic.sdf",
        "max_step_size": "0.001",
        "gazebo_gui": False,
    }
    values.update(overrides)
    return Namespace(**values)


def sample(index, *, z, roll=0.0, pitch=0.0, throttle=1000, arm=1000, modes=None, motors=None):
    modes = modes or ["ANGLE"]
    motors = motors or [1000, 1000, 1000, 1000]
    return {
        "event": "diagnostic_sample",
        "sample_index": index,
        "msp": {
            "connected": True,
            "rc_channels": [1500, 1500, 1500, throttle, arm, 1000, 1500, 1500],
            "motor": motors,
            "attitude": {"roll": roll, "pitch": pitch, "yaw": 270},
            "status": {
                "armed": "ARM" in modes,
                "active_modes": modes,
            },
        },
        "gazebo_pose": {
            "ok": True,
            "pose": {"position": {"z": z}},
        },
    }


def test_takeoff_check_passes_clean_virtual_takeoff(tmp_path):
    path = tmp_path / "diagnostics.jsonl"
    write_jsonl(path, [
        sample(1, z=0.2),
        sample(2, z=0.4, throttle=1200, arm=2000, modes=["ARM", "ANGLE"], motors=[1200] * 4),
        sample(3, z=2.2, throttle=1750, arm=2000, modes=["ARM", "ANGLE"], motors=[1750] * 4),
    ])

    summary = analyze_diagnostics(
        path,
        target_throttle=1750,
        max_abs_attitude=35.0,
        min_altitude_gain=1.0,
    )

    assert summary["passed"] is True
    assert summary["altitude_gain"] == 2.0
    assert summary["armed_angle_samples"] == 2
    assert summary["high_throttle_samples"] == 1
    assert summary["max_motor_spread"] == 0


def test_takeoff_check_fails_on_flip_attitude(tmp_path):
    path = tmp_path / "diagnostics.jsonl"
    write_jsonl(path, [
        sample(1, z=0.2),
        sample(2, z=2.0, roll=72.0, throttle=1750, arm=2000, modes=["ARM", "ANGLE"]),
    ])

    summary = analyze_diagnostics(
        path,
        target_throttle=1750,
        max_abs_attitude=35.0,
        min_altitude_gain=1.0,
    )

    assert summary["passed"] is False
    assert "roll/pitch 35.0 derece ustune cikti" in summary["failures"]


def test_takeoff_check_fails_without_altitude_gain(tmp_path):
    path = tmp_path / "diagnostics.jsonl"
    write_jsonl(path, [
        sample(1, z=0.20),
        sample(2, z=0.25, throttle=1750, arm=2000, modes=["ARM", "ANGLE"]),
    ])

    summary = analyze_diagnostics(
        path,
        target_throttle=1750,
        max_abs_attitude=35.0,
        min_altitude_gain=1.0,
    )

    assert summary["passed"] is False
    assert "irtifa artisi 1.00 m altinda" in summary["failures"]


def test_takeoff_check_fails_without_pose_when_required(tmp_path):
    path = tmp_path / "diagnostics.jsonl"
    record = sample(1, z=2.0, throttle=1750, arm=2000, modes=["ARM", "ANGLE"])
    record["gazebo_pose"] = {"ok": False}
    write_jsonl(path, [record])

    summary = analyze_diagnostics(
        path,
        target_throttle=1750,
        max_abs_attitude=35.0,
        min_altitude_gain=1.0,
    )

    assert summary["passed"] is False
    assert "Gazebo pose/IMU ornekleri yok" in summary["failures"]


def test_virtual_driver_diagnostics_declares_expected_virtual_rc(tmp_path):
    command = build_diagnostics_command(args(msp_host="127.0.0.2", msp_port=5762), tmp_path / "diag.jsonl")

    assert "--rc-source" in command
    assert command[command.index("--rc-source") + 1] == "virtual"
    assert command[command.index("--msp-host") + 1] == "127.0.0.2"
    assert command[command.index("--msp-port") + 1] == "5762"
    assert "--virtual-throttle" in command
    assert command[command.index("--virtual-throttle") + 1] == "1750"


def test_external_driver_diagnostics_observes_without_expected_virtual_rc(tmp_path):
    command = build_diagnostics_command(args(rc_driver="external"), tmp_path / "diag.jsonl")

    assert command[command.index("--rc-source") + 1] == "external"
    assert "--virtual-throttle" not in command
    assert "--virtual-arm-pwm" not in command
    assert "--virtual-mode-pwm" not in command


def test_gazebo_command_can_set_temporary_yaw_gyro_scale():
    command = build_gazebo_command(args(iris_yaw_gyro_scale=0.5))

    assert "--iris-yaw-gyro-scale" in command
    assert command[command.index("--iris-yaw-gyro-scale") + 1] == "0.5"


def test_gazebo_command_can_enable_rotor_velocity_control():
    command = build_gazebo_command(args(iris_velocity_control=True, iris_motor_time_constant=0.02))

    assert "--iris-velocity-control" in command
    assert command[command.index("--iris-motor-time-constant") + 1] == "0.02"


def test_gazebo_command_can_start_gui():
    command = build_gazebo_command(args(gazebo_gui=True))

    assert "--headless" not in command


def test_gazebo_command_defaults_to_headless():
    command = build_gazebo_command(args())

    assert "--headless" in command


def test_gazebo_command_omits_velocity_control_by_default():
    command = build_gazebo_command(args())

    assert "--iris-velocity-control" not in command
    assert "--iris-motor-time-constant" not in command


def test_gazebo_command_can_enable_forward_camera():
    command = build_gazebo_command(args(iris_forward_camera=True))

    assert "--iris-forward-camera" in command


def test_gazebo_command_omits_forward_camera_by_default():
    command = build_gazebo_command(args())

    assert "--iris-forward-camera" not in command


def test_gazebo_command_can_set_temporary_rotor_velocity_p_gain():
    command = build_gazebo_command(args(iris_rotor_vel_p_gain=0.01))

    assert "--iris-rotor-vel-p-gain" in command
    assert command[command.index("--iris-rotor-vel-p-gain") + 1] == "0.01"


def test_virtual_rc_command_uses_takeoff_script():
    command = build_virtual_rc_command(args(throttle=1600, hold_seconds=3.5))

    assert "--script" in command
    assert command[command.index("--script") + 1] == "takeoff"
    assert command[command.index("--roll") + 1] == "1500"
    assert command[command.index("--pitch") + 1] == "1500"
    assert command[command.index("--throttle") + 1] == "1600"
    assert command[command.index("--yaw") + 1] == "1500"
    assert command[command.index("--hold-seconds") + 1] == "3.5"


def test_virtual_rc_command_can_delay_nudge():
    command = build_virtual_rc_command(args(
        nudge_delay_seconds=18.0,
        nudge_pitch=1510,
        nudge_yaw=1504,
    ))

    assert command[command.index("--nudge-delay-seconds") + 1] == "18.0"
    assert command[command.index("--nudge-pitch") + 1] == "1510"
    assert command[command.index("--nudge-yaw") + 1] == "1504"


def test_virtual_rc_command_can_write_log(tmp_path):
    log = tmp_path / "virtual.jsonl"
    command = build_virtual_rc_command(args(), log)

    assert command[command.index("--log-file") + 1] == str(log)


def test_mixer_config_command_can_set_yaw_motors_reversed():
    command = build_mixer_config_command(args(
        msp_host="127.0.0.2",
        msp_port=5762,
        yaw_motors_reversed="on",
        msp_timeout=2.5,
    ))

    assert command[1].endswith("tools/sitl_mixer_config.py")
    assert command[command.index("--host") + 1] == "127.0.0.2"
    assert command[command.index("--port") + 1] == "5762"
    assert command[command.index("--timeout") + 1] == "2.5"
    assert command[command.index("--yaw-motors-reversed") + 1] == "on"


def test_pid_config_command_can_zero_yaw_pid():
    command = build_pid_config_command(args(msp_host="127.0.0.2", msp_port=5762, msp_timeout=2.5, zero_yaw_pid=True))

    assert command[1].endswith("tools/sitl_pid_config.py")
    assert command[command.index("--host") + 1] == "127.0.0.2"
    assert command[command.index("--port") + 1] == "5762"
    assert command[command.index("--timeout") + 1] == "2.5"
    assert "--zero-yaw" in command


def test_pid_config_command_can_set_custom_yaw_pid():
    command = build_pid_config_command(args(msp_timeout=2.5, yaw_pid=(45, 0, 0)))

    assert "--zero-yaw" not in command
    assert command[command.index("--yaw-p") + 1] == "45"
    assert command[command.index("--yaw-i") + 1] == "0"
    assert command[command.index("--yaw-d") + 1] == "0"


def test_pid_config_command_can_set_custom_pitch_pid():
    command = build_pid_config_command(args(msp_timeout=2.5, pitch_pid=(23, 0, 0)))

    assert "--zero-pitch" not in command
    assert command[command.index("--pitch-p") + 1] == "23"
    assert command[command.index("--pitch-i") + 1] == "0"
    assert command[command.index("--pitch-d") + 1] == "0"


def test_rate_config_command_can_set_yaw_authority():
    command = build_rate_config_command(args(msp_timeout=2.5, yaw_rc_rate=5, yaw_rate=30, yaw_rate_limit=120))

    assert command[1].endswith("tools/sitl_rate_config.py")
    assert command[command.index("--timeout") + 1] == "2.5"
    assert command[command.index("--yaw-rc-rate") + 1] == "5"
    assert command[command.index("--yaw-rate") + 1] == "30"
    assert command[command.index("--yaw-rate-limit") + 1] == "120"


def test_rate_config_command_can_set_pitch_authority():
    command = build_rate_config_command(args(msp_timeout=2.5, pitch_rc_rate=5, pitch_rate=30, pitch_rate_limit=120))

    assert command[command.index("--pitch-rc-rate") + 1] == "5"
    assert command[command.index("--pitch-rate") + 1] == "30"
    assert command[command.index("--pitch-rate-limit") + 1] == "120"


def test_rate_config_command_can_set_roll_authority():
    command = build_rate_config_command(args(msp_timeout=2.5, roll_rc_rate=5, roll_rate=30, roll_rate_limit=120))

    assert command[command.index("--roll-rc-rate") + 1] == "5"
    assert command[command.index("--roll-rate") + 1] == "30"
    assert command[command.index("--roll-rate-limit") + 1] == "120"


def test_safe_yaw_authority_fills_default_rate_profile():
    configured = apply_safe_yaw_authority(args(safe_yaw_authority=True))

    assert configured.yaw_rc_rate == SAFE_YAW_RC_RATE
    assert configured.yaw_rate == SAFE_YAW_RATE
    assert configured.yaw_rate_limit == SAFE_YAW_RATE_LIMIT


def test_safe_yaw_authority_does_not_override_explicit_rate_values():
    configured = apply_safe_yaw_authority(args(
        safe_yaw_authority=True,
        yaw_rc_rate=6,
        yaw_rate=40,
        yaw_rate_limit=180,
    ))

    assert configured.yaw_rc_rate == 6
    assert configured.yaw_rate == 40
    assert configured.yaw_rate_limit == 180


def test_safe_manual_authority_fills_default_rate_profile():
    configured = apply_safe_manual_authority(args(safe_manual_authority=True))

    for axis in ("roll", "pitch", "yaw"):
        assert getattr(configured, "%s_rc_rate" % axis) == SAFE_MANUAL_RC_RATE
        assert getattr(configured, "%s_rate" % axis) == SAFE_MANUAL_RATE
        assert getattr(configured, "%s_rate_limit" % axis) == SAFE_MANUAL_RATE_LIMIT


def test_safe_manual_authority_does_not_override_explicit_rate_values():
    configured = apply_safe_manual_authority(args(
        safe_manual_authority=True,
        pitch_rc_rate=6,
        pitch_rate=40,
        pitch_rate_limit=180,
    ))

    assert configured.pitch_rc_rate == 6
    assert configured.pitch_rate == 40
    assert configured.pitch_rate_limit == 180
    assert configured.roll_rc_rate == SAFE_MANUAL_RC_RATE
    assert configured.yaw_rate_limit == SAFE_MANUAL_RATE_LIMIT


def test_debug_config_command_can_set_debug_mode():
    command = build_debug_config_command(args(msp_host="127.0.0.2", msp_port=5762, msp_timeout=2.5, debug_mode=7))

    assert command[1].endswith("tools/sitl_debug_config.py")
    assert command[command.index("--host") + 1] == "127.0.0.2"
    assert command[command.index("--port") + 1] == "5762"
    assert command[command.index("--timeout") + 1] == "2.5"
    assert command[command.index("--debug-mode") + 1] == "7"
    assert "--save" not in command


def test_debug_config_command_can_save_for_restart():
    command = build_debug_config_command(args(msp_timeout=2.5, debug_mode=7), save=True)

    assert "--save" in command


def test_wait_for_msp_ready_retries_until_probe_succeeds(monkeypatch):
    calls = {"count": 0}

    def fake_request(host, port, code, timeout):
        calls["count"] += 1
        if calls["count"] < 2:
            raise RuntimeError("not ready")
        return b"ok"

    monkeypatch.setattr("sitl_virtual_takeoff_check.msp_request", fake_request)
    wait_for_msp_ready("127.0.0.1", 5761, 0.1, 1.0)

    assert calls["count"] == 2


def test_parse_pid_triplet_validates_input():
    assert parse_pid_triplet("45,0,1") == (45, 0, 1)
    for value in ("45,0", "45,no,0", "45,0,256"):
        try:
            parse_pid_triplet(value)
        except Exception:
            pass
        else:
            raise AssertionError("invalid PID triplet should fail")


def test_motor_udp_duration_covers_virtual_rc_script():
    duration = estimate_motor_udp_duration(args(hold_seconds=30.0, diagnostic_samples=120, diagnostic_interval=0.25))

    assert duration == 48.0


def test_virtual_rc_timeout_covers_long_takeoff_script():
    assert estimate_virtual_rc_timeout(args(hold_seconds=40.0)) == 62.0
    assert estimate_virtual_rc_timeout(args(virtual_rc_timeout=12.5)) == 12.5


def test_betaflight_work_dir_defaults_to_temp_directory(tmp_path):
    work_dir = betaflight_work_dir(args(), tmp_path)

    assert work_dir == tmp_path / "betaflight-cwd"
    assert work_dir.is_dir()


def test_betaflight_work_dir_can_use_repo_directory(tmp_path):
    work_dir = betaflight_work_dir(args(betaflight_cwd="repo"), tmp_path)

    assert work_dir == Path(__file__).resolve().parents[1]


def test_betaflight_config_file_resolves_relative_to_repo():
    path = resolve_betaflight_config_file("config/sitl.txt")

    assert path == Path(__file__).resolve().parents[1] / "config/sitl.txt"


def test_betaflight_config_import_command_uses_sitl_binary(tmp_path):
    config = tmp_path / "sitl_config.txt"
    command = build_betaflight_config_import_command(
        {"BETAFLIGHT_ROOT": "/tmp/betaflight"},
        config,
    )

    assert command == [
        "/tmp/betaflight/obj/main/betaflight_SITL.elf",
        "--config",
        str(config),
    ]


def test_bootstrap_gazebo_state_sends_zero_motor_packets(monkeypatch):
    calls = []

    def fake_send_motor_speeds(speeds, duration, host, port, hz):
        calls.append((speeds, duration, host, port, hz))
        return 12

    monkeypatch.setattr("sitl_virtual_takeoff_check.send_motor_speeds", fake_send_motor_speeds)
    result = bootstrap_gazebo_state(Namespace(
        bootstrap_seconds=0.75,
        bootstrap_hz=50.0,
        motor_host="127.0.0.2",
        motor_port=9003,
    ))

    assert result == 12
    assert calls == [([0.0, 0.0, 0.0, 0.0], 0.75, "127.0.0.2", 9003, 50.0)]


def test_bootstrap_gazebo_state_can_be_disabled(monkeypatch):
    def fake_send_motor_speeds(*args, **kwargs):
        raise AssertionError("disabled bootstrap should not send packets")

    monkeypatch.setattr("sitl_virtual_takeoff_check.send_motor_speeds", fake_send_motor_speeds)

    assert bootstrap_gazebo_state(Namespace(bootstrap_seconds=0)) == 0


def test_motor_udp_command_uses_explicit_log_and_duration(tmp_path):
    log = tmp_path / "motor.jsonl"
    command = build_motor_udp_command(args(motor_udp_duration=12.5), log)

    assert command[1].endswith("tools/sitl_motor_udp_probe.py")
    assert command[command.index("--duration") + 1] == "12.5"
    assert command[command.index("--log-file") + 1] == str(log)


def test_analyze_motor_udp_log_reads_last_summary(tmp_path):
    path = tmp_path / "motor.jsonl"
    write_jsonl(path, [
        {"event": "motor_udp_sample", "ok": True},
        {
            "event": "motor_udp_summary",
            "summary": {
                "samples": 3,
                "max_raw_spread": 945.0,
                "max_abs_axis_bias_raw": {"yaw_cw_minus_ccw": 756.0},
            },
        },
    ])

    summary = analyze_motor_udp_log(path)

    assert summary["path"] == str(path)
    assert summary["samples"] == 3
    assert summary["max_raw_spread"] == 945.0
    assert summary["max_abs_axis_bias_raw"]["yaw_cw_minus_ccw"] == 756.0


def test_looptime_us_for_step_maps_step_to_microseconds():
    assert looptime_us_for_step("0.001") == "1000"
    assert looptime_us_for_step("0.0025") == "2500"


def test_looptime_us_for_step_rejects_out_of_range_steps():
    import pytest

    with pytest.raises(RuntimeError):
        looptime_us_for_step("0.02")
    with pytest.raises(RuntimeError):
        looptime_us_for_step("0.00005")


def test_lockstep_wait_env_value_passes_valid_microseconds():
    assert lockstep_wait_env_value(4000) == "4000"
    assert lockstep_wait_env_value(1) == "1"
    assert lockstep_wait_env_value(1_000_000) == "1000000"


def test_lockstep_wait_env_value_rejects_out_of_range_values():
    import pytest

    with pytest.raises(RuntimeError):
        lockstep_wait_env_value(0)
    with pytest.raises(RuntimeError):
        lockstep_wait_env_value(1_000_001)
    with pytest.raises(RuntimeError):
        lockstep_wait_env_value(-5)
