import argparse
import sys
from pathlib import Path


TOOLS_DIR = Path(__file__).resolve().parents[1] / "tools"
if str(TOOLS_DIR) not in sys.path:
    sys.path.insert(0, str(TOOLS_DIR))

from sitl_debug_config import debug_mode_name, format_config, parse_debug_mode_value  # noqa: E402


def test_parse_debug_mode_value_accepts_names_and_numbers():
    assert parse_debug_mode_value("PIDLOOP") == 5
    assert parse_debug_mode_value("debug-anglerate") == 7
    assert parse_debug_mode_value("ANGLE_TARGET") == 82
    assert parse_debug_mode_value("0x07") == 7


def test_parse_debug_mode_value_rejects_unknown_or_out_of_range():
    for value in ("NO_SUCH_MODE", "-1", "256"):
        try:
            parse_debug_mode_value(value)
        except argparse.ArgumentTypeError:
            pass
        else:
            raise AssertionError("invalid debug mode should fail")


def test_debug_mode_name_formats_known_and_unknown_values():
    assert debug_mode_name(5) == "PIDLOOP"
    assert debug_mode_name(999) == "mode999"
    assert debug_mode_name(None) == "unknown"


def test_format_config_prints_mode_name_and_count():
    assert format_config("before", {"debug_mode": 7, "debug_mode_count": 106}) == (
        "before debug_mode=7/ANGLERATE debug_mode_count=106"
    )
