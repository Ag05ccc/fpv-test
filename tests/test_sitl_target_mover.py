import math
import sys
from pathlib import Path

import pytest

TOOLS_DIR = Path(__file__).resolve().parents[1] / "tools"
if str(TOOLS_DIR) not in sys.path:
    sys.path.insert(0, str(TOOLS_DIR))

from sitl_target_mover import (  # noqa: E402
    TargetMover,
    circle_offset,
    evaluate_motion,
    line_offset,
    parse_args,
    trajectory_offset,
    yaw_to_quaternion,
)


def test_line_ping_pongs():
    speed, distance = 2.0, 10.0
    dx0, dy0, _ = line_offset(0.0, speed, distance)
    assert (dx0, dy0) == (0.0, 0.0)
    dx_mid, _, yaw_mid = line_offset(2.5, speed, distance)  # 5 m out
    assert dx_mid == pytest.approx(5.0)
    assert yaw_mid == 0.0
    dx_end, _, _ = line_offset(5.0, speed, distance)  # at the far end
    assert dx_end == pytest.approx(10.0)
    dx_back, _, yaw_back = line_offset(7.5, speed, distance)  # returning
    assert dx_back == pytest.approx(5.0)
    assert yaw_back == pytest.approx(math.pi)
    dx_home, _, _ = line_offset(10.0, speed, distance)  # full cycle
    assert dx_home == pytest.approx(0.0)


def test_line_zero_speed_is_stationary():
    assert line_offset(100.0, 0.0, 10.0) == (0.0, 0.0, 0.0)


def test_circle_stays_on_radius():
    speed, radius = 1.0, 5.0
    for elapsed in (0.0, 3.0, 7.0, 20.0):
        dx, dy, _ = circle_offset(elapsed, speed, radius)
        # circle is centered at (0, radius) relative to the start point
        distance_from_center = math.hypot(dx - 0.0, dy - radius)
        assert distance_from_center == pytest.approx(radius, abs=1e-9)


def test_trajectory_dispatch_and_unknown():
    assert trajectory_offset("line", 1.0, speed=1.0, distance=5.0, radius=1.0)[0] == 1.0
    with pytest.raises(ValueError):
        trajectory_offset("zigzag", 0.0, speed=1.0, distance=1.0, radius=1.0)


def test_yaw_quaternion_roundtrip():
    for yaw in (0.0, math.pi / 2, -math.pi / 3):
        w, x, y, z = yaw_to_quaternion(yaw)
        assert x == 0.0 and y == 0.0
        recovered = 2.0 * math.atan2(z, w)
        assert recovered == pytest.approx(yaw)


def make_track(n, dx_per_step=0.1, t0=0.0, dt=0.05, x0=0.0, y0=0.0):
    return [{"t": t0 + i * dt, "x": x0 + i * dx_per_step, "y": y0, "z": 0.0}
            for i in range(n)]


def test_evaluate_motion_pass():
    commanded = make_track(40)
    measured = make_track(40)
    verdict = evaluate_motion(commanded, measured,
                              min_displacement=1.0, max_path_error=0.5)
    assert verdict["ok"], verdict["failures"]
    assert verdict["measured_displacement"] == pytest.approx(3.9)
    assert verdict["mean_path_error"] == pytest.approx(0.0)


def test_evaluate_motion_no_measured_samples_fails():
    verdict = evaluate_motion(make_track(10), [],
                              min_displacement=0.1, max_path_error=0.5)
    assert not verdict["ok"]
    assert any("no measured pose samples" in f for f in verdict["failures"])


def test_evaluate_motion_static_model_fails():
    commanded = make_track(40)
    measured = make_track(40, dx_per_step=0.0)  # model never moved
    verdict = evaluate_motion(commanded, measured,
                              min_displacement=1.0, max_path_error=10.0)
    assert not verdict["ok"]
    assert any("displacement" in f for f in verdict["failures"])


def test_evaluate_motion_path_error_fails():
    commanded = make_track(40)
    measured = make_track(40, y0=2.0)  # moved, but 2 m off the commanded path
    verdict = evaluate_motion(commanded, measured,
                              min_displacement=1.0, max_path_error=0.5)
    assert not verdict["ok"]
    assert any("error" in f for f in verdict["failures"])


class FakeTransport:
    def __init__(self, track_commands=True, fail_after=None,
                 start_pose=(3.0, 4.0, 0.5)):
        self.track_commands = track_commands
        self.fail_after = fail_after
        self.calls = 0
        x, y, z = start_pose
        self.pose = {"t": 0.0, "x": x, "y": y, "z": z}

    def get_model_pose(self, name, timeout=3.0):
        import time
        return dict(self.pose, t=time.monotonic())

    def set_pose(self, name, x, y, z, yaw):
        self.calls += 1
        if self.track_commands:
            # commands are applied even when the reply is lost (measured
            # live behavior)
            self.pose = {"t": 0.0, "x": x, "y": y, "z": z}
        if self.fail_after is not None and self.calls > self.fail_after:
            return False
        return True


def mover_args(**overrides):
    argv = ["--model", "kenet_car", "--world", "betaloop_demo",
            "--trajectory", "line", "--speed", "50.0", "--distance", "5",
            "--rate-hz", "200", "--duration", "0.2",
            "--min-displacement", "1.0", "--max-path-error", "0.5"]
    for key, value in overrides.items():
        argv += [key, str(value)]
    return parse_args(argv)


def test_mover_runs_and_passes_with_faithful_transport():
    transport = FakeTransport()
    mover = TargetMover(mover_args(), transport)
    verdict = mover.run()
    assert verdict["ok"], verdict["failures"]
    assert verdict["commanded_samples"] > 5
    assert verdict["measured_samples"] > 5
    # one extra set_pose from the warm-up step
    assert transport.calls == verdict["commanded_samples"] + 1


def test_mover_fails_when_model_does_not_follow():
    transport = FakeTransport(track_commands=False)
    verdict = TargetMover(mover_args(), transport).run()
    assert not verdict["ok"]


def test_mover_reports_reply_timeouts_without_failing():
    # lost replies are a diagnostic; the verdict rests on measured motion
    transport = FakeTransport(fail_after=3)
    verdict = TargetMover(mover_args(), transport).run()
    assert verdict["reply_timeouts"] > 0
    assert verdict["ok"], verdict["failures"]


def test_resolve_start_prefers_cli_override():
    args = mover_args(**{"--start-x": 10.0, "--start-y": -2.0, "--z": 1.5})
    mover = TargetMover(args, FakeTransport())
    start = mover.resolve_start()
    assert start == {"x": 10.0, "y": -2.0, "z": 1.5}


def test_resolve_start_reads_model_pose():
    mover = TargetMover(mover_args(), FakeTransport(start_pose=(3.0, 4.0, 0.5)))
    start = mover.resolve_start()
    assert start["x"] == 3.0 and start["y"] == 4.0 and start["z"] == 0.5


class BlindTransport:
    def get_model_pose(self, name, timeout=3.0):
        return None

    def set_pose(self, *args):
        return True


def test_resolve_start_fails_when_model_missing():
    with pytest.raises(RuntimeError):
        TargetMover(mover_args(), BlindTransport()).resolve_start()
