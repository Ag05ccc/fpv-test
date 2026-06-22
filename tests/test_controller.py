import time

from kenet.controller import FlightController, PIDGains
from kenet.pipeline import PipelineConfig
from kenet.tracker import TrackResult


def test_flight_controller_only_drives_pitch_and_yaw():
    cfg = PipelineConfig(
        yaw_pid=PIDGains(kp=1.0, ki=0.0, kd=0.0),
        forward_pid=PIDGains(kp=1.0, ki=0.0, kd=0.0),
        max_rc_rate=100000.0,
    )
    controller = FlightController(cfg)
    result = TrackResult(found=True, bbox=(260, 200, 80, 80), center=(400, 240))

    controller.update(result)
    time.sleep(0.001)
    controller.update(result)

    channels = controller.channels
    assert channels[cfg.roll_ch] == cfg.rc_center
    assert channels[cfg.throttle_ch] == cfg.throttle_neutral
    assert channels[cfg.pitch_ch] != cfg.rc_center
    assert channels[cfg.yaw_ch] != cfg.rc_center
