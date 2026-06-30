import sys
from argparse import Namespace
from pathlib import Path


TOOLS_DIR = Path(__file__).resolve().parents[1] / "tools"
if str(TOOLS_DIR) not in sys.path:
    sys.path.insert(0, str(TOOLS_DIR))

from sitl_dashboard import (  # noqa: E402
    CHANNEL_ROWS,
    DashboardHandler,
    ProcessManager,
    betaflight_work_dir,
    is_loopback_client,
    list_active_rc_senders,
)


def args(**overrides):
    values = {
        "betaflight_cwd": "temp",
        "log_dir": None,
        "device": "/dev/input/js0",
        "msp_host": "127.0.0.1",
        "msp_port": 5761,
        "allow_remote_control": False,
        "gazebo_world": "test_betaflight.sdf",
        "gazebo_headless": True,
        "gazebo_max_step_size": None,
        "gazebo_fix_iris_imu_pose": True,
        "gazebo_fix_iris_motor_map": True,
        "kenet_camera": "test-2.mp4",
        "kenet_print_hz": 5.0,
        "kenet_flight_log_hz": 30.0,
        "force_mode_pwm": None,
        "kenet_no_vision": False,
        "kenet_preview": False,
    }
    values.update(overrides)
    return Namespace(**values)


def test_betaflight_work_dir_defaults_to_dashboard_log_directory(tmp_path):
    work_dir = betaflight_work_dir(
        args(),
        tmp_path / "repo",
        tmp_path / "logs",
        "session",
        2,
        create=True,
    )

    assert work_dir == tmp_path / "logs" / "session-betaflight-cwd-2"
    assert work_dir.is_dir()


def test_dashboard_channel_rows_use_shared_aetr_to_msp_mapping():
    assert CHANNEL_ROWS[2]["label"] == "Throttle"
    assert CHANNEL_ROWS[2]["pilot_index"] == 2
    assert CHANNEL_ROWS[2]["fc_index"] == 3
    assert CHANNEL_ROWS[3]["label"] == "Yaw"
    assert CHANNEL_ROWS[3]["pilot_index"] == 3
    assert CHANNEL_ROWS[3]["fc_index"] == 2


def test_betaflight_work_dir_can_use_repo_directory(tmp_path):
    repo_root = tmp_path / "repo"

    work_dir = betaflight_work_dir(
        args(betaflight_cwd="repo"),
        repo_root,
        tmp_path / "logs",
        "session",
        1,
        create=True,
    )

    assert work_dir == repo_root
    assert not (tmp_path / "logs" / "session-betaflight-cwd-1").exists()


def test_process_snapshot_reports_betaflight_cwd_and_eeprom_path(tmp_path):
    manager = ProcessManager(args(log_dir=str(tmp_path / "logs"), msp_port=65530), tmp_path / "repo")
    manager.session_id = "dash"
    manager.start_counts["betaflight"] = 3

    snapshot = manager.snapshot()

    assert snapshot["betaflight"]["working_dir"] == str(tmp_path / "logs" / "dash-betaflight-cwd-3")
    assert snapshot["betaflight"]["eeprom_path"] == str(tmp_path / "logs" / "dash-betaflight-cwd-3" / "eeprom.bin")


def test_start_betaflight_uses_temp_working_directory(monkeypatch, tmp_path):
    calls = []

    class FakePopen:
        pid = 4242
        returncode = None

        def __init__(self, command, **kwargs):
            calls.append({"command": command, **kwargs})

        def poll(self):
            return None

    monkeypatch.setattr("sitl_dashboard.subprocess.Popen", FakePopen)
    manager = ProcessManager(args(log_dir=str(tmp_path / "logs")), tmp_path / "repo")
    manager.session_id = "dash"

    result = manager.start("betaflight")

    expected_cwd = tmp_path / "logs" / "dash-betaflight-cwd-1"
    assert result["ok"] is True
    assert result["working_dir"] == str(expected_cwd)
    assert result["eeprom_path"] == str(expected_cwd / "eeprom.bin")
    assert calls[0]["cwd"] == str(expected_cwd)


def test_loopback_client_detection():
    assert is_loopback_client("127.0.0.1") is True
    assert is_loopback_client("localhost") is True
    assert is_loopback_client("192.168.1.20") is False


def test_remote_process_control_requires_opt_in():
    handler = object.__new__(DashboardHandler)
    handler.client_address = ("192.168.1.20", 12345)
    handler.allow_remote_control = False

    assert handler._process_control_allowed() is False

    handler.allow_remote_control = True
    assert handler._process_control_allowed() is True


def test_loopback_process_control_is_allowed_by_default():
    handler = object.__new__(DashboardHandler)
    handler.client_address = ("127.0.0.1", 12345)
    handler.allow_remote_control = False

    assert handler._process_control_allowed() is True


def test_list_active_rc_senders_detects_send_processes(monkeypatch):
    class Result:
        stdout = """
          100 python tools/sitl_rc_bridge.py --send --verbose
          101 python tools/sitl_virtual_rc.py --script takeoff
          102 python tools/kenet_sitl_mixer.py --send
          103 python unrelated.py --send
        """

    monkeypatch.setattr("sitl_dashboard.subprocess.run", lambda *args, **kwargs: Result())

    senders = list_active_rc_senders(current_pid=999)

    assert senders == [
        {"pid": 100, "command": "python tools/sitl_rc_bridge.py --send --verbose"},
        {"pid": 102, "command": "python tools/kenet_sitl_mixer.py --send"},
    ]


def test_start_kenet_refuses_duplicate_external_rc_sender(monkeypatch, tmp_path):
    popen_calls = []

    class FakePopen:
        pid = 4242
        returncode = None

        def __init__(self, command, **kwargs):
            popen_calls.append({"command": command, **kwargs})

        def poll(self):
            return None

    monkeypatch.setattr("sitl_dashboard.subprocess.Popen", FakePopen)
    monkeypatch.setattr(
        "sitl_dashboard.list_active_rc_senders",
        lambda: [{"pid": 100, "command": "python tools/sitl_rc_bridge.py --send"}],
    )
    manager = ProcessManager(args(log_dir=str(tmp_path / "logs")), tmp_path / "repo")

    result = manager.start("kenet")

    assert result["ok"] is True
    assert "RC sender already active" in result["message"]
    assert popen_calls == []
