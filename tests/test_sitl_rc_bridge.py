import struct
import sys
from pathlib import Path

TOOLS_DIR = Path(__file__).resolve().parents[1] / "tools"
if str(TOOLS_DIR) not in sys.path:
    sys.path.insert(0, str(TOOLS_DIR))

from sitl_rc_bridge import (  # noqa: E402
    AUTOPILOT_MODE_CHANNEL,
    AXIS_MAX,
    apply_forced_mode_pwm,
    axis_to_rc,
    axis_to_three_pos_rc,
    axis_to_two_pos_rc,
    clamp_rc,
    make_channels,
    pack_rc_packet,
)


class FakeJoystick:
    def __init__(self, axes=None, buttons=None):
        self.axes = axes or {}
        self.buttons = buttons or {}


def test_axis_to_rc_boundaries_and_invert():
    assert axis_to_rc(-AXIS_MAX) == 1000
    assert axis_to_rc(0) == 1500
    assert axis_to_rc(AXIS_MAX) == 2000
    assert axis_to_rc(AXIS_MAX, invert=True) == 1000


def test_switch_axis_conversions():
    assert axis_to_two_pos_rc(-1) == 1000
    assert axis_to_two_pos_rc(1) == 2000
    assert axis_to_three_pos_rc(-AXIS_MAX) == 1000
    assert axis_to_three_pos_rc(0) == 1500
    assert axis_to_three_pos_rc(AXIS_MAX) == 2000


def test_clamp_rc():
    assert clamp_rc(900) == 1000
    assert clamp_rc(1500.4) == 1500
    assert clamp_rc(2100) == 2000


def test_make_channels_uses_mapping_sources():
    mapping = {
        "roll": {"channel": 0, "source": "axis", "index": 0},
        "pitch": {"channel": 1, "source": "axis", "index": 1, "invert": True},
        "arm": {"channel": 4, "source": "axis", "index": 4, "two_pos": True},
        "mode": {"channel": 5, "source": "axis", "index": 6, "three_pos": True},
        "fixed": {"channel": 7, "source": "fixed", "value": 1750},
    }
    joystick = FakeJoystick(axes={0: AXIS_MAX, 1: AXIS_MAX, 4: AXIS_MAX, 6: 0})

    channels = make_channels(joystick, mapping)

    assert channels[0] == 2000
    assert channels[1] == 1000
    assert channels[4] == 2000
    assert channels[5] == 1500
    assert channels[7] == 1750


def test_apply_forced_mode_pwm_only_changes_autopilot_mode_channel():
    channels = [1500] * 16
    channels[AUTOPILOT_MODE_CHANNEL] = 1000

    result = apply_forced_mode_pwm(channels, 1500)

    assert result is channels
    assert channels[AUTOPILOT_MODE_CHANNEL] == 1500
    for index, value in enumerate(channels):
        if index != AUTOPILOT_MODE_CHANNEL:
            assert value == 1500


def test_pack_rc_packet_contract():
    channels = [1500] * 16
    packet = pack_rc_packet(channels)

    assert len(packet) == 40
    timestamp, *decoded_channels = struct.unpack("<d16H", packet)
    assert timestamp > 0
    assert decoded_channels == channels
