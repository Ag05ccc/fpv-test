from kenet.gcs import GCSCommand
from kenet.pipeline import AI_ARMED, IDLE, TRACKING, PipelineConfig, TrackingPipeline
from kenet.tracker import TrackResult


class FakeGCS:
    def __init__(self, commands=None):
        self.commands = list(commands or [])
        self.sent = []

    def recv_command(self):
        return self.commands.pop(0) if self.commands else None

    def ready_to_send(self):
        return True

    def send_telemetry(self, packet):
        self.sent.append(packet)
        return True


class FakeMSP:
    def get_attitude(self):
        return {"roll": 1.5, "pitch": -2.5, "yaw": 123}


class FakeRcMSP:
    def __init__(self, channels):
        self.channels = channels

    def get_rc_channels(self):
        return self.channels


class FakeSendMSP:
    def __init__(self):
        self.sent = []

    def send_rc(self, channels):
        self.sent.append(list(channels))


class SequenceTracker:
    def __init__(self, results):
        self.results = list(results)
        self.reset_count = 0

    def update(self, frame):
        return self.results.pop(0)

    def reset(self):
        self.reset_count += 1


def test_gcs_stop_command_is_state_independent():
    pipeline = TrackingPipeline(PipelineConfig(gcs_enabled=False))
    pipeline.gcs = FakeGCS([GCSCommand(command="stop")])
    pipeline._state = IDLE
    pipeline._running = True

    pipeline._handle_gcs_commands()

    assert pipeline._running is False


def test_send_telemetry_populates_attitude_when_msp_connected():
    pipeline = TrackingPipeline(PipelineConfig(gcs_enabled=False))
    pipeline.gcs = FakeGCS()
    pipeline.msp = FakeMSP()
    pipeline._msp_connected = True

    assert pipeline._send_telemetry(TrackResult()) is True

    packet = pipeline.gcs.sent[0]
    assert packet.attitude_valid is True
    assert packet.roll == 1.5
    assert packet.pitch == -2.5
    assert packet.yaw == 123


def test_override_channels_keep_pilot_roll_throttle_and_aux():
    cfg = PipelineConfig(gcs_enabled=False)
    pipeline = TrackingPipeline(cfg)
    pilot = [1510, 1520, 1100, 1490, 2000, 2000, 1500, 1000]
    pipeline._last_rc = pilot
    pipeline.controller._channels[cfg.pitch_ch] = 1430
    pipeline.controller._channels[cfg.yaw_ch] = 1580
    pipeline.controller._channels[cfg.throttle_ch] = 1500

    channels = pipeline._build_override_channels()

    assert channels == [1510, 1430, 1100, 1580, 2000, 2000, 1500, 1000]
    assert pilot == [1510, 1520, 1100, 1490, 2000, 2000, 1500, 1000]


def test_override_channels_skip_without_complete_pilot_rc():
    pipeline = TrackingPipeline(PipelineConfig(gcs_enabled=False))

    pipeline._last_rc = [1500, 1500]

    assert pipeline._build_override_channels() is None


def test_poll_aux_stores_msp_rc_as_pilot_ordered_channels():
    cfg = PipelineConfig(gcs_enabled=False)
    pipeline = TrackingPipeline(cfg)
    pipeline.msp = FakeRcMSP([1500, 1500, 1400, 1120, 1000, 1500, 1500, 1500])

    pipeline._poll_aux_state()

    assert pipeline._state == AI_ARMED
    assert pipeline._last_rc[:4] == [1500, 1500, 1120, 1400]


def test_tracking_step_sends_only_when_target_found():
    cfg = PipelineConfig(gcs_enabled=False, max_rc_rate=100000.0)
    pipeline = TrackingPipeline(cfg)
    pipeline._state = TRACKING
    pipeline._last_rc = [1510, 1520, 1100, 1490, 1000, 2000, 1500, 1000]
    pipeline.tracker = SequenceTracker([
        TrackResult(found=True, bbox=(260, 200, 80, 80), center=(400, 240)),
        TrackResult(found=True, bbox=(260, 200, 80, 80), center=(400, 240)),
    ])
    pipeline.msp = FakeSendMSP()

    pipeline._track_control_step(object())
    pipeline._track_control_step(object())

    assert len(pipeline.msp.sent) == 2
    sent = pipeline.msp.sent[-1]
    assert sent[cfg.roll_ch] == 1510
    assert sent[cfg.throttle_ch] == 1100
    assert sent[cfg.pitch_ch] != 1520
    assert sent[cfg.yaw_ch] != 1490


def test_tracking_step_does_not_send_override_when_target_lost():
    cfg = PipelineConfig(gcs_enabled=False, loop_hz=2)
    pipeline = TrackingPipeline(cfg)
    pipeline._state = TRACKING
    pipeline._last_rc = [1510, 1520, 1100, 1490, 1000, 2000, 1500, 1000]
    pipeline._last_override_channels = [1500, 1600, 1100, 1400, 1000, 2000, 1500, 1000]
    pipeline.tracker = SequenceTracker([TrackResult(found=False)])
    pipeline.msp = FakeSendMSP()

    pipeline._track_control_step(object())

    assert pipeline.msp.sent == []
    assert pipeline._last_override_channels is None
    assert pipeline._state == TRACKING


def test_tracking_step_drops_to_ai_armed_after_target_loss_threshold():
    cfg = PipelineConfig(gcs_enabled=False, loop_hz=2)
    pipeline = TrackingPipeline(cfg)
    pipeline._state = TRACKING
    pipeline._last_rc = [1510, 1520, 1100, 1490, 1000, 2000, 1500, 1000]
    pipeline.tracker = SequenceTracker([TrackResult(found=False) for _ in range(5)])
    pipeline.msp = FakeSendMSP()

    for _ in range(5):
        pipeline._track_control_step(object())

    assert pipeline.msp.sent == []
    assert pipeline._state == AI_ARMED
    assert pipeline._lost_count == 0
    assert pipeline.tracker.reset_count == 1


def test_target_loss_blocks_tracking_reentry_until_aux_cycles_below_track_threshold():
    cfg = PipelineConfig(gcs_enabled=False, loop_hz=2)
    pipeline = TrackingPipeline(cfg)
    pipeline._state = TRACKING
    pipeline._last_rc = [1510, 1520, 1100, 1490, 1000, 2000, 1500, 1000]
    pipeline.tracker = SequenceTracker([TrackResult(found=False) for _ in range(5)])
    pipeline.msp = FakeSendMSP()

    for _ in range(5):
        pipeline._track_control_step(object())

    assert pipeline._state == AI_ARMED
    assert pipeline._tracking_reentry_blocked is True

    pipeline.msp = FakeRcMSP([1500, 1500, 2000, 1100, 1000, 2000, 1500, 1500])
    pipeline._poll_aux_state()

    assert pipeline._state == AI_ARMED
    assert pipeline._tracking_reentry_blocked is True

    pipeline.msp = FakeRcMSP([1500, 1500, 1500, 1100, 1000, 1500, 1500, 1500])
    pipeline._poll_aux_state()

    assert pipeline._state == AI_ARMED
    assert pipeline._tracking_reentry_blocked is False
