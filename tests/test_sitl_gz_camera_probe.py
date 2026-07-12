import sys
from pathlib import Path

import numpy as np

TOOLS_DIR = Path(__file__).resolve().parents[1] / "tools"
if str(TOOLS_DIR) not in sys.path:
    sys.path.insert(0, str(TOOLS_DIR))

from sitl_gz_camera_probe import evaluate_probe, main, parse_args, run_probe  # noqa: E402


def verdict(**overrides):
    base = dict(
        frame_count=100,
        duration=5.0,
        frame_shape=(480, 640),
        decode_errors=0,
        last_frame_age=0.05,
        min_fps=15.0,
        expect_width=640,
        expect_height=480,
        max_frame_age=1.0,
    )
    base.update(overrides)
    return evaluate_probe(**base)


def test_healthy_stream_passes():
    result = verdict()
    assert result["ok"]
    assert result["measured_fps"] == 20.0
    assert not result["failures"]


def test_no_frames_fails():
    result = verdict(frame_count=0, frame_shape=None, last_frame_age=None)
    assert not result["ok"]
    assert any("no frames" in f for f in result["failures"])


def test_low_fps_fails():
    result = verdict(frame_count=10)
    assert not result["ok"]
    assert any("fps" in f for f in result["failures"])


def test_wrong_resolution_fails():
    result = verdict(frame_shape=(240, 320))
    assert not result["ok"]
    assert any("width" in f for f in result["failures"])
    assert any("height" in f for f in result["failures"])


def test_resolution_check_disabled_with_zero():
    result = verdict(frame_shape=(240, 320), expect_width=0, expect_height=0)
    assert result["ok"]


def test_decode_errors_fail():
    result = verdict(decode_errors=3)
    assert not result["ok"]


def test_stale_last_frame_fails():
    result = verdict(last_frame_age=2.5)
    assert not result["ok"]
    assert any("stale" in f for f in result["failures"])


class FakeCapture:
    def __init__(self, frames=30):
        self.frames = frames
        self.frame_count = 0
        self.decode_errors = 0
        self.started = False
        self.stopped = False

    def start(self):
        self.started = True

    def read(self):
        if self.frame_count < self.frames:
            self.frame_count += 1
        return np.zeros((480, 640, 3), dtype=np.uint8)

    def last_frame_age(self):
        return 0.01

    def stop(self):
        self.stopped = True


def test_run_probe_uses_bridge_and_stops(monkeypatch):
    captures = []

    def factory(source):
        assert source == "gz:/kenet/fpv_camera"
        capture = FakeCapture()
        captures.append(capture)
        return capture

    args = parse_args(["--duration", "0.1", "--min-fps", "1"])
    result = run_probe(args, capture_factory=factory)
    assert captures[0].started and captures[0].stopped
    assert result["ok"], result["failures"]
