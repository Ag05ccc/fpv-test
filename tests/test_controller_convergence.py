"""Offline PID convergence gates (roadmap-v2 A6, metric 6).

These tests close the "is the tracking PID reasonable and how do I verify it"
gap: instead of only checking output signs, they close the loop against a
simple deterministic plant model and assert that the controller actually
centers the target (settling) and keeps a moving target bounded (following).

Plant model: yaw stick offset produces a proportional yaw rate, which moves
the target's horizontal pixel position back toward the frame center. The
constants approximate the SITL setup (640 px wide, ~60 deg HFOV -> ~10.7
px/deg; safe-yaw-authority-scale rates), but the gate is about closed-loop
behavior (converges, no blow-up), not about exact plant fidelity.
"""

import pytest

import kenet.controller as controller_module
from kenet.controller import FlightController
from kenet.pipeline import PipelineConfig
from kenet.tracker import TrackResult


class FakeTime:
    def __init__(self):
        self.now = 100.0

    def monotonic(self):
        return self.now

    def advance(self, dt):
        self.now += dt


@pytest.fixture
def fake_time(monkeypatch):
    clock = FakeTime()
    monkeypatch.setattr(controller_module, "time", clock)
    return clock


PX_PER_DEG = 640 / 60.0          # horizontal pixels per degree of yaw
YAW_RATE_PER_UNIT = 0.35         # deg/s of yaw rate per RC unit off center
WIDTH_RATE_PER_UNIT = 0.25       # px/s of bbox-width growth per pitch unit


def simulate_yaw_tracking(fake_time, *, initial_error_px, target_drift_px_s,
                          seconds, hz=30):
    """Close the loop: pixel error -> controller -> yaw stick -> plant."""
    cfg = PipelineConfig()
    controller = FlightController(cfg)
    dt = 1.0 / hz
    cx = cfg.frame_width / 2.0
    target_x = cx + initial_error_px
    errors = []
    for _ in range(int(seconds * hz)):
        result = TrackResult(found=True, bbox=(int(target_x) - 60, 200, 120, 120),
                             center=(target_x, 240.0))
        controller.update(result)
        yaw_offset = controller.channels[cfg.yaw_ch] - cfg.rc_center
        fake_time.advance(dt)
        # plant: positive yaw offset rotates toward the target
        target_x -= yaw_offset * YAW_RATE_PER_UNIT * PX_PER_DEG * dt
        # independent target motion (0 for a static target)
        target_x += target_drift_px_s * dt
        errors.append(target_x - cx)
    return errors, cfg


def test_static_target_settles_to_center(fake_time):
    errors, cfg = simulate_yaw_tracking(
        fake_time, initial_error_px=200.0, target_drift_px_s=0.0, seconds=10.0)
    settle_band = cfg.deadband + 10.0  # deadband (10 px) + margin
    # converged: the last two seconds stay inside the band
    tail = errors[-60:]
    assert max(abs(e) for e in tail) < settle_band, \
        "controller did not settle: tail error %.1f px" % max(abs(e) for e in tail)
    # settled reasonably fast: inside the band within 6 simulated seconds
    first_inside = next(i for i, e in enumerate(errors) if abs(e) < settle_band)
    assert first_inside < 6 * 30, \
        "settling took %.1f s" % (first_inside / 30.0)
    # no divergence/overshoot blow-up on the way
    assert max(abs(e) for e in errors) <= 200.0 * 1.25


def test_moving_target_error_stays_bounded(fake_time):
    """A slowly crossing target (metric 7 scenario) must not run away."""
    errors, _ = simulate_yaw_tracking(
        fake_time, initial_error_px=0.0, target_drift_px_s=40.0, seconds=10.0)
    tail = errors[-90:]
    assert max(abs(e) for e in tail) < 120.0, \
        "steady-state error %.1f px against a moving target" % max(abs(e) for e in tail)


def test_forward_axis_converges_on_desired_width(fake_time):
    """Approach loop: bbox width converges to desired_target_width."""
    cfg = PipelineConfig()
    controller = FlightController(cfg)
    dt = 1.0 / 30
    width = 40.0  # far away: bbox much smaller than desired 120 px
    widths = []
    for _ in range(int(20.0 * 30)):
        cx = cfg.frame_width / 2.0
        result = TrackResult(found=True,
                             bbox=(int(cx) - int(width) // 2, 220,
                                   int(width), int(width)),
                             center=(cx, 240.0))
        controller.update(result)
        pitch_offset = controller.channels[cfg.pitch_ch] - cfg.rc_center
        fake_time.advance(dt)
        # plant: pitching forward (positive offset) grows the apparent width
        width += pitch_offset * WIDTH_RATE_PER_UNIT * dt
        width = max(4.0, width)
        widths.append(width)
    tail = widths[-60:]
    band = cfg.size_deadband + 15.0
    assert all(abs(cfg.desired_target_width - w) < band for w in tail), \
        "width did not converge: tail %.1f px vs desired %.1f" % (
            tail[-1], cfg.desired_target_width)


def test_lost_target_returns_sticks_to_center(fake_time):
    cfg = PipelineConfig()
    controller = FlightController(cfg)
    dt = 1.0 / 30
    result = TrackResult(found=True, bbox=(500, 200, 120, 120),
                         center=(560.0, 240.0))
    for _ in range(30):
        controller.update(result)
        fake_time.advance(dt)
    assert controller.channels[cfg.yaw_ch] != cfg.rc_center
    lost = TrackResult(found=False)
    for _ in range(90):
        controller.update(lost)
        fake_time.advance(dt)
    assert controller.channels[cfg.yaw_ch] == cfg.rc_center
    assert controller.channels[cfg.pitch_ch] == cfg.rc_center
