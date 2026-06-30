import argparse
import struct
import sys
from pathlib import Path


TOOLS_DIR = Path(__file__).resolve().parents[1] / "tools"
if str(TOOLS_DIR) not in sys.path:
    sys.path.insert(0, str(TOOLS_DIR))

from sitl_rate_config import (  # noqa: E402
    encode_rate_payload,
    format_rate_config,
    parse_byte_value,
    parse_rate_payload,
    parse_u16_value,
    update_rate_config,
)


def sample_payload():
    payload = bytearray()
    payload += bytes([7, 0, 67, 67, 67, 0, 50, 0])
    payload += struct.pack("<H", 0)
    payload += bytes([0, 7, 7, 0, 0, 100])
    payload += struct.pack("<HHH", 1998, 1998, 1998)
    payload += bytes([0, 50])
    return bytes(payload)


def test_parse_and_encode_rate_payload_preserves_layout():
    parsed = parse_rate_payload(sample_payload())

    assert parsed["payload_len"] == 24
    assert parsed["roll_rc_rate"] == 7
    assert parsed["yaw_rate"] == 67
    assert parsed["yaw_rc_rate"] == 7
    assert parsed["yaw_rate_limit"] == 1998
    assert parsed["thr_hover"] == 50
    assert encode_rate_payload(parsed) == sample_payload()


def test_update_rate_config_changes_only_requested_yaw_fields():
    parsed = parse_rate_payload(sample_payload())
    updated = update_rate_config(parsed, yaw_rc_rate=5, yaw_rate=30, yaw_rate_limit=120)

    assert updated["yaw_rc_rate"] == 5
    assert updated["yaw_rate"] == 30
    assert updated["yaw_rate_limit"] == 120
    assert updated["roll_rate_limit"] == parsed["roll_rate_limit"]
    assert updated["pitch_rate_limit"] == parsed["pitch_rate_limit"]


def test_update_rate_config_can_change_pitch_fields():
    parsed = parse_rate_payload(sample_payload())
    updated = update_rate_config(parsed, pitch_rc_rate=5, pitch_rate=30, pitch_rate_limit=120)

    assert updated["pitch_rc_rate"] == 5
    assert updated["pitch_rate"] == 30
    assert updated["pitch_rate_limit"] == 120
    assert updated["yaw_rc_rate"] == parsed["yaw_rc_rate"]
    assert updated["yaw_rate_limit"] == parsed["yaw_rate_limit"]


def test_rate_value_parsers_validate_ranges():
    assert parse_byte_value("255") == 255
    assert parse_u16_value("65535") == 65535
    for parser, value in ((parse_byte_value, "256"), (parse_u16_value, "65536"), (parse_u16_value, "nope")):
        try:
            parser(value)
        except argparse.ArgumentTypeError:
            pass
        else:
            raise AssertionError("invalid rate value should fail")


def test_format_rate_config_is_compact():
    parsed = parse_rate_payload(sample_payload())

    assert format_rate_config("before", parsed) == (
        "before yaw_rc_rate=7 yaw_rate=67 yaw_rate_limit=1998 "
        "pitch_rc_rate=7 pitch_rate=67 pitch_rate_limit=1998 "
        "roll_limit=1998 pitch_limit=1998 rates_type=0"
    )
