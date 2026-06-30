import sys
from argparse import Namespace
from pathlib import Path

TOOLS_DIR = Path(__file__).resolve().parents[1] / "tools"
if str(TOOLS_DIR) not in sys.path:
    sys.path.insert(0, str(TOOLS_DIR))

from sitl_rc_bridge import AXIS_MAX, make_channels  # noqa: E402
from state_monitor import (  # noqa: E402
    StateSampler,
    build_channel_map,
    build_dashboard_command,
    main,
    parse_args,
)


class FakeJoystick:
    def __init__(self, axes=None, buttons=None):
        self.axes = axes or {}
        self.buttons = buttons or {}


def args(**overrides):
    values = {
        "device": "/dev/input/js0",
        "rate_hz": 50.0,
        "algorithm_axis": 6,
        "algorithm_ch": 5,
        "algorithm_invert": False,
        "algorithm_arm_threshold": 1300,
        "algorithm_track_threshold": 1700,
        "autopilot_axis": 4,
        "autopilot_ch": 4,
        "autopilot_invert": False,
        "autopilot_arm_threshold": 1700,
        "autopilot_mode_axis": 5,
        "autopilot_mode_ch": 6,
        "autopilot_mode_invert": False,
    }
    values.update(overrides)
    return Namespace(**values)


def test_state_monitor_channel_map_matches_bridge_conversions():
    joystick = FakeJoystick(axes={
        0: AXIS_MAX,
        1: AXIS_MAX,
        2: -AXIS_MAX,
        3: 0,
        4: AXIS_MAX,
        5: 0,
        6: AXIS_MAX,
    })

    channels = make_channels(joystick, build_channel_map(args()))

    assert channels[:7] == [2000, 1000, 1000, 1500, 2000, 2000, 1500]


def test_state_monitor_snapshot_uses_shared_channel_labels():
    sampler = StateSampler(args())
    sampler.channels = [1500, 1500, 1000, 1500, 2000, 2000, 1500, 1500]

    snapshot = sampler.snapshot()

    assert snapshot["algorithm"]["channel_label"] == "AUX2 Kenet CH6"
    assert snapshot["autopilot"]["channel_label"] == "AUX1 ARM CH5"
    assert snapshot["autopilot_mode"]["channel_label"] == "AUX3 Mode CH7"
    assert [item["label"] for item in snapshot["channels"][:7]] == [
        "Roll",
        "Pitch",
        "Throttle",
        "Yaw",
        "AUX1 ARM",
        "AUX2 Kenet",
        "AUX3 Mode",
    ]


def test_state_monitor_redirects_to_dashboard_command():
    parsed = parse_args([
        "--device", "/dev/input/js1",
        "--host", "127.0.0.1",
        "--port", "8765",
        "--autopilot-ch", "4",
        "--algorithm-ch", "5",
        "--autopilot-mode-ch", "6",
        "--open",
    ])

    command = build_dashboard_command(parsed)

    assert command[1].endswith("sitl_dashboard.py")
    assert command[command.index("--device") + 1] == "/dev/input/js1"
    assert command[command.index("--port") + 1] == "8765"
    assert command[command.index("--arm-ch") + 1] == "4"
    assert command[command.index("--kenet-ch") + 1] == "5"
    assert command[command.index("--mode-ch") + 1] == "6"
    assert "--open" in command


def test_state_monitor_main_uses_dashboard_redirect_by_default(monkeypatch):
    calls = []

    monkeypatch.setattr("state_monitor.subprocess.call", lambda command: calls.append(command) or 0)

    assert main(["--port", "8765"]) == 0
    assert calls
    assert calls[0][1].endswith("sitl_dashboard.py")
