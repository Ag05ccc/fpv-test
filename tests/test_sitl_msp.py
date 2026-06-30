import struct
import sys
from pathlib import Path

TOOLS_DIR = Path(__file__).resolve().parents[1] / "tools"
if str(TOOLS_DIR) not in sys.path:
    sys.path.insert(0, str(TOOLS_DIR))

from sitl_msp import (  # noqa: E402
    encode_advanced_config,
    encode_mixer_config,
    parse_advanced_config,
    parse_box_ids,
    parse_box_names,
    parse_debug,
    parse_mixer_config,
    parse_status_ex,
)


def status_ex_payload(first_flags=0, extra=b"", arming_flags=0):
    payload = bytearray()
    payload += struct.pack("<HHH", 250, 0, 0)
    payload += struct.pack("<I", first_flags)
    payload += bytes([0])
    payload += struct.pack("<H", 12)
    payload += bytes([3, 0])
    payload += bytes([len(extra)])
    payload += extra
    payload += bytes([29])
    payload += struct.pack("<I", arming_flags)
    payload += bytes([0, 0, 0, 0])
    return bytes(payload)


def test_parse_box_names_and_ids():
    assert parse_box_names(b"ARM;ANGLE;HORIZON;") == ["ARM", "ANGLE", "HORIZON"]
    assert parse_box_ids(bytes([0, 1, 2])) == [0, 1, 2]


def test_parse_and_encode_mixer_config():
    assert parse_mixer_config(bytes([3, 1])) == {
        "valid": True,
        "mixer_mode": 3,
        "yaw_motors_reversed": True,
    }
    assert parse_mixer_config(b"\x03") == {
        "valid": False,
        "mixer_mode": None,
        "yaw_motors_reversed": None,
    }
    assert encode_mixer_config(3, False) == bytes([3, 0])


def test_parse_advanced_config_reads_debug_mode_tail_fields():
    payload = bytearray()
    payload += bytes([1, 4, 1, 5])
    payload += struct.pack("<H", 480)
    payload += struct.pack("<H", 550)
    payload += bytes([0, 1, 0, 0, 32])
    payload += struct.pack("<H", 125)
    payload += struct.pack("<H", 7)
    payload += bytes([1, 37, 135])

    parsed = parse_advanced_config(bytes(payload))

    assert parsed["valid"] is True
    assert parsed["gyro_sync_denom"] == 1
    assert parsed["pid_process_denom"] == 4
    assert parsed["motor_continuous_update"] is True
    assert parsed["motor_pwm_rate"] == 480
    assert parsed["motor_idle"] == 550
    assert parsed["motor_inversion"] is True
    assert parsed["gyro_calibration_duration"] == 125
    assert parsed["gyro_offset_yaw"] == 7
    assert parsed["gyro_check_overflow"] == 1
    assert parsed["debug_mode"] == 37
    assert parsed["debug_mode_count"] == 135


def test_encode_advanced_config_preserves_fields_and_changes_debug_mode():
    payload = bytearray()
    payload += bytes([1, 4, 1, 5])
    payload += struct.pack("<H", 480)
    payload += struct.pack("<H", 550)
    payload += bytes([0, 1, 0, 0, 32])
    payload += struct.pack("<H", 125)
    payload += struct.pack("<H", 7)
    payload += bytes([1, 37, 135])
    parsed = parse_advanced_config(bytes(payload))

    encoded = encode_advanced_config(parsed, debug_mode=7)

    assert encoded == bytes(payload[:18] + bytes([7]))


def test_encode_advanced_config_rejects_missing_preserved_fields():
    try:
        encode_advanced_config({"debug_mode": 0}, debug_mode=1)
    except ValueError as exc:
        assert "pid_process_denom" in str(exc)
    else:
        raise AssertionError("missing advanced config field should fail")


def test_parse_debug_reads_signed_int16_values():
    payload = struct.pack("<8h", -1, 0, 123, -456, 32767, -32768, 42, -42)

    assert parse_debug(payload) == [-1, 0, 123, -456, 32767, -32768, 42, -42]


def test_parse_status_ex_maps_flags_by_box_order():
    status = parse_status_ex(
        status_ex_payload(first_flags=0b011),
        box_names=["ARM", "ANGLE", "HORIZON"],
        box_ids=[0, 1, 2],
    )

    assert status["valid"] is True
    assert status["armed"] is True
    assert status["active_modes_valid"] is True
    assert status["active_modes"] == ["ARM", "ANGLE"]
    assert status["active_mode_ids"] == [0, 1]


def test_parse_status_ex_does_not_guess_armed_without_box_metadata():
    status = parse_status_ex(status_ex_payload(first_flags=0b1))

    assert status["valid"] is True
    assert status["armed"] is None
    assert status["active_modes_valid"] is False
    assert status["active_modes"] == []


def test_parse_status_ex_uses_structural_arming_flags_offset():
    status = parse_status_ex(status_ex_payload(first_flags=0, extra=b"\x00\x00", arming_flags=1 << 7))

    assert status["arming_disable_flags"] == 1 << 7
    assert status["arming_disable_names"] == ["THROTTLE"]
