import argparse
import sys
from pathlib import Path


TOOLS_DIR = Path(__file__).resolve().parents[1] / "tools"
if str(TOOLS_DIR) not in sys.path:
    sys.path.insert(0, str(TOOLS_DIR))

from sitl_pid_config import (  # noqa: E402
    encode_pid_payload,
    format_pid,
    parse_pid_payload,
    parse_pid_value,
    update_axis_pid,
)


def sample_items():
    return [
        {"index": 0, "name": "roll", "p": 45, "i": 80, "d": 30},
        {"index": 1, "name": "pitch", "p": 47, "i": 84, "d": 34},
        {"index": 2, "name": "yaw", "p": 45, "i": 80, "d": 0},
        {"index": 3, "name": "level", "p": 50, "i": 50, "d": 75},
        {"index": 4, "name": "mag", "p": 40, "i": 0, "d": 0},
    ]


def test_parse_and_encode_pid_payload():
    payload = bytes([45, 80, 30, 47, 84, 34, 45, 80, 0, 50, 50, 75, 40, 0, 0])

    items = parse_pid_payload(payload)

    assert items == sample_items()
    assert encode_pid_payload(items) == payload


def test_update_axis_pid_preserves_other_items():
    updated = update_axis_pid(sample_items(), "yaw", p=0, i=0, d=0)

    assert updated[0] == sample_items()[0]
    assert updated[2] == {"index": 2, "name": "yaw", "p": 0, "i": 0, "d": 0}


def test_update_axis_pid_can_update_pitch():
    updated = update_axis_pid(sample_items(), "pitch", p=23, i=0, d=0)

    assert updated[1] == {"index": 1, "name": "pitch", "p": 23, "i": 0, "d": 0}
    assert updated[2] == sample_items()[2]


def test_parse_pid_value_validates_byte_range():
    assert parse_pid_value("42") == 42
    for value in ("-1", "256", "nope"):
        try:
            parse_pid_value(value)
        except argparse.ArgumentTypeError:
            pass
        else:
            raise AssertionError("invalid PID value should fail")


def test_format_pid_is_compact():
    assert format_pid("before", sample_items()[:2]) == "before roll=45/80/30 pitch=47/84/34"
