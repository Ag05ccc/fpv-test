import sys
from argparse import Namespace
from pathlib import Path


TOOLS_DIR = Path(__file__).resolve().parents[1] / "tools"
if str(TOOLS_DIR) not in sys.path:
    sys.path.insert(0, str(TOOLS_DIR))

from sitl_debug_calibration import (  # noqa: E402
    build_debug_config_command,
    debug_abs_max,
    first_nonzero_debug,
    summarize_records,
)


def args(**overrides):
    values = {
        "msp_host": "127.0.0.1",
        "msp_port": 5761,
        "msp_timeout": 2.0,
        "debug_mode": 7,
    }
    values.update(overrides)
    return Namespace(**values)


def test_build_debug_config_command_can_save_mode():
    command = build_debug_config_command(args(debug_mode=5, msp_timeout=3.5), save=True)

    assert command[1].endswith("tools/sitl_debug_config.py")
    assert command[command.index("--debug-mode") + 1] == "5"
    assert command[command.index("--timeout") + 1] == "3.5"
    assert "--save" in command


def test_build_debug_config_command_can_skip_save():
    command = build_debug_config_command(args(debug_mode=82), save=False)

    assert command[command.index("--debug-mode") + 1] == "82"
    assert "--save" not in command


def test_debug_abs_max_uses_first_eight_signed_channels():
    records = [
        {"debug": [0, -3, 4, 0, 11, -12, 0, 1, 999]},
        {"debug": [2, 1, -9, 0, -4, 7, 0, -8, 888]},
    ]

    assert debug_abs_max(records) == [2, 3, 9, 0, 11, 12, 0, 8]


def test_first_nonzero_debug_returns_first_record_with_signal():
    first = {"debug": [0, 0, 0]}
    second = {"debug": [0, -1, 0]}

    assert first_nonzero_debug([first, second]) is second
    assert first_nonzero_debug([first]) is None


def test_summarize_records_flags_zero_debug_as_not_calibrated():
    summary = summarize_records([
        {"debug_mode": 7, "debug": [0, 0, 0, 0], "rc_channels": [1500, 1500, 1700, 1000]},
        {"debug_mode": 7, "debug": [0, 0, 0, 0], "rc_channels": [1500, 1500, 1700, 1000]},
    ], expected_debug_mode=7)

    assert summary["debug_mode_ok"] is True
    assert summary["any_nonzero_debug"] is False
    assert summary["min_fc_yaw"] == 1700
    assert summary["max_fc_yaw"] == 1700


def test_summarize_records_reports_nonzero_debug():
    second = {"debug_mode": 7, "debug": [0, 25, 0, -3], "rc_channels": [1500, 1500, 1800, 1000]}

    summary = summarize_records([
        {"debug_mode": 7, "debug": [0, 0, 0, 0], "rc_channels": [1500, 1500, 1500, 1000]},
        second,
    ], expected_debug_mode=7)

    assert summary["any_nonzero_debug"] is True
    assert summary["first_nonzero_debug"] is second
    assert summary["max_abs_debug"][:4] == [0, 25, 0, 3]
