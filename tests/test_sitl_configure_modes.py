import sys
from pathlib import Path


TOOLS_DIR = Path(__file__).resolve().parents[1] / "tools"
if str(TOOLS_DIR) not in sys.path:
    sys.path.insert(0, str(TOOLS_DIR))

from sitl_configure_modes import ModeRange, default_ranges, step_to_us, us_to_step  # noqa: E402


def test_mode_range_us_step_conversion():
    assert us_to_step(900) == 0
    assert us_to_step(1300) == 16
    assert us_to_step(1600) == 28
    assert us_to_step(1700) == 32
    assert us_to_step(2100) == 48
    assert step_to_us(32) == 1700


def test_mode_range_payload_matches_betaflight_msp_format():
    item = ModeRange(
        index=2,
        permanent_id=1,
        aux_channel_index=2,
        start_us=1300,
        end_us=1700,
    )

    assert item.payload() == bytes([2, 1, 2, 16, 32])
    assert item.is_usable() is True


def test_disabled_mode_range_is_not_usable():
    item = ModeRange(
        index=4,
        permanent_id=0,
        aux_channel_index=0,
        start_us=900,
        end_us=900,
    )

    assert item.payload() == bytes([4, 0, 0, 0, 0])
    assert item.is_usable() is False


def test_default_ranges_cover_arm_msp_override_angle_and_horizon():
    class Args:
        arm_start = 1600
        arm_end = 2100
        msp_override_start = 1700
        msp_override_end = 2100
        angle_start = 1300
        angle_end = 1700
        horizon_start = 1700
        horizon_end = 2100

    ranges = default_ranges(Args())

    assert [(item.permanent_id, item.aux_channel_index, item.start_us, item.end_us) for item in ranges] == [
        (0, 0, 1600, 2100),
        (50, 1, 1700, 2100),
        (1, 2, 1300, 1700),
        (2, 2, 1700, 2100),
    ]
