import struct
import sys
from pathlib import Path

TOOLS_DIR = Path(__file__).resolve().parents[1] / "tools"
if str(TOOLS_DIR) not in sys.path:
    sys.path.insert(0, str(TOOLS_DIR))

import sitl_rc_bridge  # noqa: E402
from sitl_keyboard_rc import (  # noqa: E402
    ARM_CH,
    AUTOPILOT_MODE_CHANNEL,
    KENET_STATE_CH,
    PITCH_CH,
    ROLL_CH,
    SAFE_EXIT_PACKET_COUNT,
    THROTTLE_CH,
    YAW_CH,
    KeyboardRcState,
    format_status,
    pack_rc_packet,
    safe_exit_channels,
)


def test_startup_channels_are_safe():
    state = KeyboardRcState()
    channels = state.channels(0.0)

    assert len(channels) == 16
    assert channels[THROTTLE_CH] == 1000
    assert channels[ARM_CH] == 1000
    assert channels[KENET_STATE_CH] == 1000
    assert channels[AUTOPILOT_MODE_CHANNEL] == 1500
    assert channels[ROLL_CH] == 1500
    assert channels[PITCH_CH] == 1500
    assert channels[YAW_CH] == 1500
    assert not state.armed
    assert not state.quit_requested


def test_throttle_is_latched_and_steps():
    state = KeyboardRcState(throttle_step=20)

    state.apply_key("w", 0.0)
    state.apply_key("w", 0.1)
    assert state.channels(0.1)[THROTTLE_CH] == 1040
    # Latched: does not decay with time.
    assert state.channels(100.0)[THROTTLE_CH] == 1040

    state.apply_key("s", 0.2)
    assert state.channels(0.2)[THROTTLE_CH] == 1020


def test_throttle_clamps_at_both_ends():
    state = KeyboardRcState(throttle_step=300)

    for _ in range(10):
        state.apply_key("w", 0.0)
    assert state.channels(0.0)[THROTTLE_CH] == 2000

    for _ in range(10):
        state.apply_key("s", 0.0)
    assert state.channels(0.0)[THROTTLE_CH] == 1000


def test_x_snaps_throttle_to_min():
    state = KeyboardRcState()
    for _ in range(5):
        state.apply_key("w", 0.0)
    assert state.channels(0.0)[THROTTLE_CH] == 1100

    state.apply_key("x", 0.1)
    assert state.channels(0.1)[THROTTLE_CH] == 1000


def test_momentary_stick_deflects_and_auto_returns():
    state = KeyboardRcState(stick_step=150, stick_hold=0.30)

    state.apply_key("d", 10.0)
    assert state.channels(10.0)[YAW_CH] == 1650
    assert state.channels(10.29)[YAW_CH] == 1650
    assert state.channels(10.31)[YAW_CH] == 1500


def test_yaw_authority_clamps_yaw_but_not_roll_pitch():
    # plant spins on large yaw; the clamp keeps yaw in the stable window while
    # roll/pitch stay at full stick_step authority.
    state = KeyboardRcState(stick_step=150, stick_hold=0.30, yaw_authority=10)
    state.apply_key("d", 10.0)   # yaw right
    assert state.channels(10.0)[YAW_CH] == 1510   # clamped to +/-10, not 1650
    state.apply_key("a", 11.0)   # yaw left
    assert state.channels(11.0)[YAW_CH] == 1490
    state.apply_key("l", 12.0)   # roll — unaffected by yaw clamp
    assert state.channels(12.0)[ROLL_CH] == 1650


def test_yaw_authority_zero_disables_clamp():
    state = KeyboardRcState(stick_step=150, stick_hold=0.30, yaw_authority=0)
    state.apply_key("d", 0.0)
    assert state.channels(0.0)[YAW_CH] == 1650


def test_repeat_press_refreshes_hold_without_stacking():
    state = KeyboardRcState(stick_step=150, stick_hold=0.30)

    state.apply_key("d", 0.0)
    state.apply_key("d", 0.2)
    # Not stacked beyond one step.
    assert state.channels(0.25)[YAW_CH] == 1650
    # Hold window refreshed by the second press.
    assert state.channels(0.45)[YAW_CH] == 1650
    assert state.channels(0.55)[YAW_CH] == 1500


def test_opposite_key_crosses_immediately():
    state = KeyboardRcState(stick_step=150, stick_hold=0.30)

    state.apply_key("d", 0.0)
    assert state.channels(0.05)[YAW_CH] == 1650
    state.apply_key("a", 0.1)
    assert state.channels(0.1)[YAW_CH] == 1350


def test_all_stick_axes_map_to_expected_channels():
    hold = 0.30
    for key, channel, expected in (
        ("a", YAW_CH, 1350),
        ("d", YAW_CH, 1650),
        ("j", ROLL_CH, 1350),
        ("l", ROLL_CH, 1650),
        ("i", PITCH_CH, 1650),  # i = nose forward
        ("k", PITCH_CH, 1350),
    ):
        state = KeyboardRcState(stick_step=150, stick_hold=hold)
        state.apply_key(key, 0.0)
        channels = state.channels(0.1)
        assert channels[channel] == expected
        for other in (ROLL_CH, PITCH_CH, YAW_CH):
            if other != channel:
                assert channels[other] == 1500


def test_stick_deflection_is_clamped():
    state = KeyboardRcState(stick_step=800, stick_hold=0.30)

    state.apply_key("d", 0.0)
    assert state.channels(0.1)[YAW_CH] == 2000
    state.apply_key("a", 0.2)
    assert state.channels(0.25)[YAW_CH] == 1000


def test_arm_toggle():
    state = KeyboardRcState()

    state.apply_key("e", 0.0)
    assert state.armed
    assert state.channels(0.0)[ARM_CH] == 2000

    state.apply_key("e", 0.1)
    assert not state.armed
    assert state.channels(0.1)[ARM_CH] == 1000


def test_panic_disarms_and_kills_throttle_and_sticks():
    state = KeyboardRcState()
    state.apply_key("e", 0.0)
    for _ in range(10):
        state.apply_key("w", 0.0)
    state.apply_key("d", 0.0)

    state.apply_key(" ", 0.05)

    channels = state.channels(0.05)
    assert channels[ARM_CH] == 1000
    assert channels[THROTTLE_CH] == 1000
    assert channels[YAW_CH] == 1500
    assert not state.armed


def test_kenet_state_keys():
    state = KeyboardRcState()
    assert state.channels(0.0)[KENET_STATE_CH] == 1000

    state.apply_key("2", 0.0)
    assert state.channels(0.0)[KENET_STATE_CH] == 1500
    state.apply_key("3", 0.1)
    assert state.channels(0.1)[KENET_STATE_CH] == 2000
    state.apply_key("1", 0.2)
    assert state.channels(0.2)[KENET_STATE_CH] == 1000


def test_mode_channel_is_forced_and_not_key_controlled():
    state = KeyboardRcState(mode_pwm=1700)
    assert state.channels(0.0)[AUTOPILOT_MODE_CHANNEL] == 1700

    for key in "wsxadjlike 123":
        state.apply_key(key, 0.0)
    assert state.channels(0.0)[AUTOPILOT_MODE_CHANNEL] == 1700


def test_quit_keys_request_quit():
    for key in ("q", "\x1b"):
        state = KeyboardRcState()
        state.apply_key(key, 0.0)
        assert state.quit_requested


def test_unknown_key_is_ignored():
    state = KeyboardRcState()
    before = state.channels(0.0)

    assert state.apply_key("z", 0.0) is False
    assert state.channels(0.0) == before
    assert not state.quit_requested


def test_packet_bytes_match_sitl_rc_bridge(monkeypatch):
    monkeypatch.setattr(sitl_rc_bridge.time, "time", lambda: 123.456)
    state = KeyboardRcState()
    state.apply_key("e", 0.0)
    state.apply_key("w", 0.0)
    channels = state.channels(0.0)

    packet = pack_rc_packet(channels)

    assert packet == sitl_rc_bridge.pack_rc_packet(channels)
    assert packet == struct.pack("<d16H", 123.456, *channels)
    assert len(packet) == 40


def test_safe_exit_burst_channels():
    channels = safe_exit_channels(mode_pwm=1500)

    assert len(channels) == 16
    assert channels[ARM_CH] == 1000
    assert channels[THROTTLE_CH] == 1000
    assert channels[KENET_STATE_CH] == 1000
    assert channels[AUTOPILOT_MODE_CHANNEL] == 1500
    assert channels[ROLL_CH] == 1500
    assert channels[PITCH_CH] == 1500
    assert channels[YAW_CH] == 1500
    assert SAFE_EXIT_PACKET_COUNT == 10

    forced = safe_exit_channels(mode_pwm=2000)
    assert forced[AUTOPILOT_MODE_CHANNEL] == 2000
    assert forced[ARM_CH] == 1000
    assert forced[THROTTLE_CH] == 1000


def test_format_status_reports_channels_and_arm_state():
    state = KeyboardRcState()
    state.apply_key("e", 0.0)
    channels = state.channels(0.0)

    line = format_status(channels, state.armed, 7)
    assert "ARMED" in line
    assert "sent=7" in line
    assert ",".join(str(v) for v in channels[:8]) in line

    verbose_line = format_status(channels, False, 8, verbose=True)
    assert "DISARMED" in verbose_line
    assert ",".join(str(v) for v in channels) in verbose_line


def test_extract_keys_filters_arrow_and_function_sequences():
    from sitl_keyboard_rc import extract_keys

    assert extract_keys("\x1b[A") == []
    assert extract_keys("\x1b[15~") == []
    assert extract_keys("\x1bOP") == []
    assert extract_keys("a\x1b[Bd") == ["a", "d"]


def test_extract_keys_keeps_lone_escape_as_quit():
    from sitl_keyboard_rc import extract_keys

    assert extract_keys("\x1b") == ["\x1b"]
    assert extract_keys("w\x1b") == ["w", "\x1b"]
