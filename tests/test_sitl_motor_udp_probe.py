import struct
import sys
from pathlib import Path


TOOLS_DIR = Path(__file__).resolve().parents[1] / "tools"
if str(TOOLS_DIR) not in sys.path:
    sys.path.insert(0, str(TOOLS_DIR))

from sitl_motor_udp_probe import (  # noqa: E402
    motor_axis_biases,
    parse_raw_servo_packet,
    remap_logical_to_packet_slots,
    summarize,
)


def test_parse_raw_servo_packet_reads_padded_motor_count_and_floats():
    outputs = [1000.0, 1055.0, 1500.0, 1749.0] + [0.0] * 12
    packet = struct.pack("<H2x16f", 4, *outputs)

    parsed = parse_raw_servo_packet(packet)

    assert parsed["motor_count"] == 4
    assert parsed["motors_raw"] == [1000.0, 1055.0, 1500.0, 1749.0]
    assert [round(value, 3) for value in parsed["motors_normalized_1000_idle"]] == [0.0, 0.055, 0.5, 0.749]
    assert [round(value or 0.0, 3) for value in parsed["motors_packet_order_normalized"]] == [0.055, 0.5, 0.749, 0.0]
    assert parsed["motor_spread_raw"] == 749.0


def test_remap_logical_to_packet_slots_uses_betaflight_sitl_order():
    assert remap_logical_to_packet_slots([0.1, 0.2, 0.3, 0.4]) == [0.2, 0.3, 0.4, 0.1]


def test_motor_axis_biases_report_logical_bf_pairs():
    biases = motor_axis_biases([1800.0, 1400.0, 1200.0, 1600.0])

    assert biases == {
        "roll_left_minus_right": -200.0,
        "pitch_rear_minus_front": 0.0,
        "yaw_cw_minus_ccw": 400.0,
    }


def test_summarize_reports_motor_ranges():
    summary = summarize([
        {"motor_count": 2, "motors_raw": [1000.0, 1100.0], "motors_normalized_1000_idle": [0.0, 0.1]},
        {"motor_count": 2, "motors_raw": [1200.0, 1300.0], "motors_normalized_1000_idle": [0.2, 0.3]},
    ])

    assert summary["samples"] == 2
    assert summary["motor_counts"] == [2]
    assert summary["min_raw_by_motor"] == [1000.0, 1100.0]
    assert summary["max_raw_by_motor"] == [1200.0, 1300.0]
    assert summary["max_normalized_by_motor"] == [0.2, 0.3]
    assert summary["max_raw_spread"] is None


def test_summarize_reports_large_spread_and_axis_bias():
    first = parse_raw_servo_packet(struct.pack("<H2x16f", 4, *([1000.0, 1000.0, 1000.0, 1000.0] + [0.0] * 12)))
    first["received_at"] = 1.0
    second = parse_raw_servo_packet(struct.pack("<H2x16f", 4, *([2000.0, 1055.0, 1421.0, 1316.0] + [0.0] * 12)))
    second["received_at"] = 2.0

    summary = summarize([first, second])

    assert summary["max_raw_spread"] == 945.0
    assert summary["max_abs_axis_bias_raw"]["roll_left_minus_right"] == 159.0
    assert summary["max_abs_axis_bias_raw"]["pitch_rear_minus_front"] == 525.0
    assert summary["max_abs_axis_bias_raw"]["yaw_cw_minus_ccw"] == 420.0
    assert summary["first_large_spread"]["received_at"] == 2.0
