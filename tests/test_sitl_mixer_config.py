import argparse
import sys
from pathlib import Path


TOOLS_DIR = Path(__file__).resolve().parents[1] / "tools"
if str(TOOLS_DIR) not in sys.path:
    sys.path.insert(0, str(TOOLS_DIR))

from sitl_mixer_config import format_config, parse_on_off  # noqa: E402


def test_parse_on_off_accepts_common_values():
    assert parse_on_off("on") is True
    assert parse_on_off("true") is True
    assert parse_on_off("1") is True
    assert parse_on_off("off") is False
    assert parse_on_off("false") is False
    assert parse_on_off("0") is False


def test_parse_on_off_rejects_unknown_value():
    try:
        parse_on_off("maybe")
    except argparse.ArgumentTypeError as exc:
        assert "expected" in str(exc)
    else:
        raise AssertionError("unknown on/off value should fail")


def test_format_config_prints_yaw_state():
    assert format_config("before", {"mixer_mode": 3, "yaw_motors_reversed": True}) == (
        "before mixer_mode=3 yaw_motors_reversed=ON"
    )
