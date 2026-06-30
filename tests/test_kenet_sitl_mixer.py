import struct
import sys
import time
from pathlib import Path
from types import SimpleNamespace

TOOLS_DIR = Path(__file__).resolve().parents[1] / "tools"
if str(TOOLS_DIR) not in sys.path:
    sys.path.insert(0, str(TOOLS_DIR))

from kenet_sitl_mixer import KenetSitlMixer  # noqa: E402
from kenet.pipeline import AI_ARMED, IDLE, TRACKING  # noqa: E402
from kenet.tracker import TrackResult  # noqa: E402


def make_args(**overrides):
    values = {
        "camera": "0",
        "frame_width": 640,
        "frame_height": 480,
        "camera_fps": 30,
        "tracker": "CSRT",
        "loop_hz": 10.0,
        "preview": False,
        "aux_ch": 5,
        "force_mode_pwm": None,
        "track_size": 100,
        "desired_target_width": 120.0,
        "yaw_kp": 0.8,
        "yaw_ki": 0.05,
        "yaw_kd": 0.15,
        "yaw_limit": 300.0,
        "forward_kp": 0.4,
        "forward_ki": 0.02,
        "forward_kd": 0.1,
        "forward_limit": 250.0,
        "device": "/dev/input/js0",
        "pilot_source": "joystick",
        "virtual_script": "manual",
        "virtual_roll": 1500,
        "virtual_pitch": 1500,
        "virtual_throttle": 1000,
        "virtual_yaw": 1500,
        "virtual_arm_pwm": 1000,
        "virtual_kenet_pwm": 1000,
        "virtual_kenet_pre_pwm": 1000,
        "virtual_kenet_delay_seconds": 0.0,
        "virtual_mode_pwm": 1500,
        "virtual_low_seconds": 5.0,
        "virtual_arm_seconds": 3.0,
        "virtual_ramp_seconds": 3.0,
        "virtual_hold_seconds": 8.0,
        "virtual_disarm_seconds": 1.0,
        "send": False,
        "no_vision": False,
        "synthetic_target": False,
        "synthetic_target_x": 340.0,
        "synthetic_target_y": 240.0,
        "synthetic_target_width": 110.0,
        "synthetic_target_height": 110.0,
        "synthetic_target_delay_seconds": 0.0,
        "synthetic_target_loss_after_seconds": None,
        "synthetic_elapsed_seconds": 0.0,
        "aux_arm_threshold": 1300,
        "aux_track_threshold": 1700,
        "lost_seconds": 0.1,
        "print_hz": 5.0,
        "flight_log_hz": 30.0,
        "flight_log_flush_every": 10,
        "flight_log_flush_seconds": 1.0,
        "flight_log_max_mb": 50.0,
        "flight_log": None,
        "log_dir": None,
        "no_flight_log": True,
        "host": "127.0.0.1",
        "port": 9004,
        "duration": 0.0,
        "log_level": "INFO",
    }
    values.update(overrides)
    return SimpleNamespace(**values)


def make_mixer(**overrides):
    return KenetSitlMixer(make_args(**overrides))


def pilot_channels():
    return [1500, 1500, 1000, 1500, 1000, 1500, 2000, 1500]


def packet_channels():
    return pilot_channels() + [1500] * 8


class FakeSocket:
    def __init__(self):
        self.packets = []

    def sendto(self, packet, address):
        self.packets.append((packet, address))
        return len(packet)


class FakeFlightLogger:
    def __init__(self):
        self.events = []

    def write(self, event, **payload):
        self.events.append((event, payload))


def decode_packet(packet):
    _timestamp, *channels = struct.unpack("<d16H", packet)
    return channels


def test_forced_mode_pwm_is_applied_to_pilot_channels():
    mixer = make_mixer(force_mode_pwm=1500)
    mixer.joystick.axes = {6: -32767, 5: -32767}
    mixer.joystick.poll = lambda timeout=0: []

    channels = mixer._read_pilot_channels()

    assert channels[6] == 1500
    assert channels[5] == 1000


def test_virtual_pilot_source_does_not_require_joystick():
    mixer = make_mixer(
        pilot_source="virtual",
        virtual_throttle=1450,
        virtual_arm_pwm=2000,
        virtual_kenet_pwm=2000,
        virtual_mode_pwm=1500,
    )

    channels = mixer._read_pilot_channels()

    assert mixer.joystick is None
    assert channels[:8] == [1500, 1500, 1450, 1500, 2000, 2000, 1500, 1500]


def test_virtual_pilot_source_honors_forced_mode_pwm():
    mixer = make_mixer(
        pilot_source="virtual",
        force_mode_pwm=1700,
        virtual_mode_pwm=1000,
    )

    channels = mixer._read_pilot_channels()

    assert channels[6] == 1700


def test_virtual_takeoff_script_throttle_and_arm_sequence():
    mixer = make_mixer(
        pilot_source="virtual",
        virtual_script="takeoff",
        virtual_throttle=1600,
        virtual_low_seconds=2.0,
        virtual_arm_seconds=1.0,
        virtual_ramp_seconds=2.0,
        virtual_hold_seconds=3.0,
        virtual_disarm_seconds=1.0,
    )

    assert mixer._virtual_takeoff_throttle_arm(0.5) == (1000, 1000)
    assert mixer._virtual_takeoff_throttle_arm(2.5) == (1000, 2000)
    assert mixer._virtual_takeoff_throttle_arm(4.0) == (1300, 2000)
    assert mixer._virtual_takeoff_throttle_arm(5.5) == (1600, 2000)
    assert mixer._virtual_takeoff_throttle_arm(8.2) == (1000, 1000)
    assert mixer._virtual_takeoff_throttle_arm(10.0) == (1000, 1000)


def test_virtual_kenet_pwm_can_be_delayed():
    mixer = make_mixer(
        pilot_source="virtual",
        virtual_kenet_pre_pwm=1000,
        virtual_kenet_pwm=2000,
        virtual_kenet_delay_seconds=18.0,
    )

    assert mixer._virtual_kenet_pwm(17.9) == 1000
    assert mixer._virtual_kenet_pwm(18.0) == 2000


def test_synthetic_target_result_uses_configured_bbox():
    mixer = make_mixer(
        synthetic_target=True,
        synthetic_target_x=350.0,
        synthetic_target_y=245.0,
        synthetic_target_width=80.0,
        synthetic_target_height=60.0,
    )

    result = mixer._synthetic_track_result()

    assert result.found is True
    assert result.center == (350.0, 245.0)
    assert result.bbox == (310, 215, 80, 60)


def test_synthetic_target_delay_starts_centered():
    mixer = make_mixer(
        synthetic_target=True,
        synthetic_target_delay_seconds=10.0,
        synthetic_target_x=420.0,
        synthetic_target_width=80.0,
    )

    result = mixer._synthetic_track_result()

    assert result.found is True
    assert result.center == (320.0, 240.0)
    assert result.bbox == (260, 180, 120, 120)


def test_synthetic_target_updates_controller_in_tracking():
    mixer = make_mixer(
        synthetic_target=True,
        synthetic_target_x=420.0,
        synthetic_target_width=80.0,
        virtual_kenet_pwm=2000,
    )
    pilot = pilot_channels()
    pilot[mixer.args.aux_ch] = 2000

    first = mixer._update_vision_state(None, pilot)
    mixer.controller._prev_time -= 0.1
    mixer.controller.yaw_pid._prev_time -= 0.1
    mixer.controller.forward_pid._prev_time -= 0.1
    second = mixer._update_vision_state(None, pilot)
    final = mixer._mix_channels(pilot, second)

    assert first.found is True
    assert second.found is True
    assert mixer.state == TRACKING
    assert mixer.last_source == "kenet"
    assert final[mixer.cfg.yaw_ch] != pilot[mixer.cfg.yaw_ch]
    assert final[mixer.cfg.pitch_ch] != pilot[mixer.cfg.pitch_ch]
    assert final[mixer.cfg.roll_ch] == pilot[mixer.cfg.roll_ch]
    assert final[mixer.cfg.throttle_ch] == pilot[mixer.cfg.throttle_ch]


def test_synthetic_target_loss_passthrough_then_drops_to_ai_armed():
    mixer = make_mixer(
        synthetic_target=True,
        synthetic_target_loss_after_seconds=1.0,
        lost_seconds=0.2,
        loop_hz=10.0,
    )
    mixer.virtual_started = time.monotonic() - 1.1
    mixer.state = TRACKING
    mixer.prev_state = TRACKING
    pilot = pilot_channels()
    pilot[mixer.args.aux_ch] = 2000

    first = mixer._update_vision_state(None, pilot)
    first_final = mixer._mix_channels(pilot, first)
    first_source = mixer.last_source
    second = mixer._update_vision_state(None, pilot)
    second_final = mixer._mix_channels(pilot, second)

    assert first.found is False
    assert first_final == pilot
    assert first_source == "pilot-target-lost"
    assert mixer.lost_count >= 2
    assert second.found is False
    assert second_final == pilot
    assert mixer.state == AI_ARMED
    assert mixer.prev_state == AI_ARMED
    assert mixer.tracking_reentry_blocked is True

    third = mixer._update_vision_state(None, pilot)
    third_final = mixer._mix_channels(pilot, third)

    assert third.found is False
    assert third_final == pilot
    assert mixer.state == AI_ARMED
    assert mixer.prev_state == AI_ARMED

    pilot[mixer.args.aux_ch] = 1500
    mixer._update_vision_state(None, pilot)

    assert mixer.state == AI_ARMED
    assert mixer.tracking_reentry_blocked is False


def test_mix_passthrough_when_not_tracking():
    mixer = make_mixer()
    pilot = pilot_channels()
    mixer.state = IDLE

    assert mixer._mix_channels(pilot, TrackResult(found=True)) == pilot
    assert mixer.last_source == "pilot"

    mixer.state = AI_ARMED
    assert mixer._mix_channels(pilot, TrackResult(found=True)) == pilot
    assert mixer.last_source == "pilot"


def test_mix_overrides_only_pitch_and_yaw_when_tracking_found():
    mixer = make_mixer()
    pilot = pilot_channels()
    mixer.state = TRACKING
    mixer.controller._channels = list(pilot)
    mixer.controller._channels[mixer.cfg.pitch_ch] = 1600
    mixer.controller._channels[mixer.cfg.yaw_ch] = 1400

    final = mixer._mix_channels(pilot, TrackResult(found=True))

    assert final[mixer.cfg.pitch_ch] == 1600
    assert final[mixer.cfg.yaw_ch] == 1400
    for index, value in enumerate(final):
        if index not in (mixer.cfg.pitch_ch, mixer.cfg.yaw_ch):
            assert value == pilot[index]
    assert mixer.last_source == "kenet"


def test_mix_falls_back_on_target_lost():
    mixer = make_mixer()
    pilot = pilot_channels()
    mixer.state = TRACKING

    assert mixer._mix_channels(pilot, TrackResult(found=False)) == pilot
    assert mixer.last_source == "pilot-target-lost"


def test_mix_falls_back_on_tracker_unavailable():
    mixer = make_mixer()
    pilot = pilot_channels()
    mixer.state = TRACKING
    mixer.tracker_error = "missing tracker"

    assert mixer._mix_channels(pilot, TrackResult(found=False)) == pilot
    assert mixer.last_source == "pilot-tracker-unavailable"


class LostTracker:
    is_initialized = True

    def __init__(self):
        self.reset_count = 0

    def update(self, _frame):
        return TrackResult(found=False)

    def reset(self):
        self.reset_count += 1


class FakeFrame:
    shape = (480, 640, 3)


def test_target_loss_drops_mixer_to_ai_armed():
    mixer = make_mixer(lost_seconds=0.1, loop_hz=10.0)
    mixer.tracker = LostTracker()
    mixer.state = TRACKING
    mixer.prev_state = TRACKING
    pilot = pilot_channels()
    pilot[mixer.args.aux_ch] = 2000

    result = mixer._update_vision_state(FakeFrame(), pilot)

    assert result.found is False
    assert mixer.state == AI_ARMED
    assert mixer.prev_state == AI_ARMED
    assert mixer.tracker.reset_count == 1


def test_safe_exit_frame_sends_throttle_low_centered_channels():
    mixer = make_mixer(send=True, force_mode_pwm=1500)
    mixer.sock = FakeSocket()

    assert mixer._send_safe_exit_frame() is True

    packet, address = mixer.sock.packets[-1]
    channels = decode_packet(packet)
    assert address == ("127.0.0.1", 9004)
    assert channels[:8] == [1500, 1500, 1000, 1500, 1000, 1500, 1500, 1500]


def test_rc_send_watchdog_records_warning_event(caplog):
    mixer = make_mixer(send=True, loop_hz=10.0)
    mixer.sock = FakeSocket()
    mixer.flight_logger = FakeFlightLogger()
    mixer.last_send_time = 1.0

    with caplog.at_level("WARNING", logger="kenet_sitl_mixer"):
        mixer._send_rc_frame(packet_channels(), now=1.25, reason="loop")

    assert mixer.tx_warning_count == 1
    assert "RC send interval 0.250s exceeded watchdog 0.200s" in caplog.text
    assert mixer.flight_logger.events[0][0] == "tx_warning"
    assert mixer.flight_logger.events[0][1]["reason"] == "loop"


def test_rc_send_watchdog_allows_nominal_period():
    mixer = make_mixer(send=True, loop_hz=10.0)
    mixer.sock = FakeSocket()
    mixer.last_send_time = 1.0

    mixer._send_rc_frame(packet_channels(), now=1.19, reason="loop")

    assert mixer.tx_warning_count == 0
    assert len(mixer.sock.packets) == 1
