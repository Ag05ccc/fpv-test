import time

from kenet.controller import FlightController, PIDController, PIDGains
from kenet.pipeline import PipelineConfig
from kenet.tracker import TrackResult


def test_pid_set_gains_updates_live():
    pid = PIDController(PIDGains(kp=1.0, ki=0.0, kd=0.0))
    pid.set_gains(kp=2.5, ki=0.1)
    assert pid.gains.kp == 2.5
    assert pid.gains.ki == 0.1
    assert pid.gains.kd == 0.0  # unchanged when not provided


def test_flight_controller_set_gains_changes_output():
    cfg = PipelineConfig(max_rc_rate=100000.0)
    controller = FlightController(cfg)
    result = TrackResult(found=True, bbox=(260, 200, 80, 80), center=(400, 240))

    controller.yaw_pid.set_gains(kp=1.0, ki=0.0, kd=0.0)
    controller.update(result)
    time.sleep(0.001)
    controller.update(result)
    strong = controller.yaw_output

    controller.reset()
    controller.yaw_pid.set_gains(kp=0.1)
    controller.update(result)
    time.sleep(0.001)
    controller.update(result)
    weak = controller.yaw_output

    assert abs(weak) < abs(strong)


def _load_kenet_cli():
    import importlib.util
    from pathlib import Path
    path = Path(__file__).resolve().parents[1] / "kenet.py"
    spec = importlib.util.spec_from_file_location("kenet_cli_module", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_cli_pid_override_helper():
    kenet_cli = _load_kenet_cli()
    gains = PIDGains(kp=0.8, ki=0.05, kd=0.15)
    kenet_cli._apply_pid_override(gains, 1.2, None, None)
    assert gains.kp == 1.2
    assert gains.ki == 0.05  # untouched
    assert gains.kd == 0.15
