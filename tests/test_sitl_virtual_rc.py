import sys
from argparse import Namespace
from pathlib import Path

TOOLS_DIR = Path(__file__).resolve().parents[1] / "tools"
if str(TOOLS_DIR) not in sys.path:
    sys.path.insert(0, str(TOOLS_DIR))

from sitl_virtual_rc import (  # noqa: E402
    ARM_CH,
    KENET_CH,
    MODE_CH,
    THROTTLE_CH,
    build_steps,
    make_virtual_channels,
    run,
)


def args(**overrides):
    values = {
        "script": "takeoff",
        "roll": 1500,
        "pitch": 1500,
        "throttle": 1550,
        "yaw": 1500,
        "arm": False,
        "arm_pwm": 1000,
        "kenet_pwm": 1000,
        "mode_pwm": 1500,
        "nudge_delay_seconds": None,
        "nudge_roll": None,
        "nudge_pitch": None,
        "nudge_yaw": None,
        "duration": 0.0,
        "low_seconds": 5.0,
        "arm_seconds": 3.0,
        "ramp_seconds": 2.0,
        "hold_seconds": 4.0,
        "disarm_seconds": 1.0,
        "host": "127.0.0.1",
        "port": 9004,
        "rate_hz": 50.0,
        "print_hz": 5.0,
        "send": False,
        "once": False,
        "log_file": None,
    }
    values.update(overrides)
    return Namespace(**values)


def test_make_virtual_channels_defaults_to_safe_disarmed_rc():
    channels = make_virtual_channels()

    assert channels[THROTTLE_CH] == 1000
    assert channels[ARM_CH] == 1000
    assert channels[KENET_CH] == 1000
    assert channels[MODE_CH] == 1000
    assert len(channels) == 16


def test_takeoff_script_arms_then_ramps_then_disarms():
    steps = build_steps(args())

    assert [step.name for step in steps] == [
        "boot-low",
        "arm-low-throttle",
        "throttle-ramp",
        "takeoff-hold",
        "disarm-low",
    ]
    assert steps[0].channels[ARM_CH] == 1000
    assert steps[1].channels[ARM_CH] == 2000
    assert steps[2].channels_at(0.0)[THROTTLE_CH] == 1000
    assert steps[2].channels_at(1.0)[THROTTLE_CH] == 1275
    assert steps[2].channels_at(2.0)[THROTTLE_CH] == 1550
    assert steps[3].channels[THROTTLE_CH] == 1550
    assert steps[4].channels[ARM_CH] == 1000


def test_takeoff_script_can_delay_nudge_until_hold():
    steps = build_steps(args(
        nudge_delay_seconds=11.0,
        nudge_pitch=1510,
        nudge_yaw=1504,
    ))

    assert [step.name for step in steps] == [
        "boot-low",
        "arm-low-throttle",
        "throttle-ramp",
        "takeoff-hold",
        "takeoff-hold-nudge",
        "disarm-low",
    ]
    assert steps[3].seconds == 1.0
    assert steps[4].seconds == 3.0
    assert steps[3].channels[1] == 1500
    assert steps[3].channels[3] == 1500
    assert steps[4].channels[1] == 1510
    assert steps[4].channels[3] == 1504


def test_manual_script_can_hold_fixed_takeoff_like_frame():
    steps = build_steps(args(script="manual", arm=True, throttle=1600, duration=1.0))

    assert len(steps) == 1
    assert steps[0].channels[ARM_CH] == 2000
    assert steps[0].channels[THROTTLE_CH] == 1600
    assert steps[0].channels[MODE_CH] == 1500


def test_run_can_write_jsonl_log_for_one_frame(tmp_path):
    path = tmp_path / "virtual-rc.jsonl"

    assert run(args(script="manual", duration=1.0, once=True, log_file=str(path))) == 0

    text = path.read_text(encoding="utf-8")
    assert '"event":"session_start"' in text
    assert '"event":"virtual_rc_step_start"' in text
    assert '"event":"virtual_rc_frame"' in text
    assert '"event":"virtual_rc_summary"' in text
