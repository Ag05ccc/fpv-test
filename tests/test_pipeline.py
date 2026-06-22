from kenet.gcs import GCSCommand
from kenet.pipeline import IDLE, PipelineConfig, TrackingPipeline
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
