import sys
from pathlib import Path

import numpy as np
import pytest

TOOLS_DIR = Path(__file__).resolve().parents[1] / "tools"
if str(TOOLS_DIR) not in sys.path:
    sys.path.insert(0, str(TOOLS_DIR))

from tracker_motion_benchmark import (  # noqa: E402
    bbox_center,
    bbox_iou,
    evaluate_benchmark,
    generate_sequence,
    parse_args,
    run_benchmark,
)


def test_bbox_iou_basics():
    assert bbox_iou((0, 0, 10, 10), (0, 0, 10, 10)) == pytest.approx(1.0)
    assert bbox_iou((0, 0, 10, 10), (20, 20, 10, 10)) == 0.0
    assert bbox_iou((0, 0, 10, 10), (5, 0, 10, 10)) == pytest.approx(1.0 / 3.0)


def test_bbox_center():
    assert bbox_center((10, 20, 4, 8)) == (12.0, 24.0)


def test_generate_sequence_is_deterministic_and_moves():
    kwargs = dict(width=320, height=240, frames=10, base_size=40,
                  speed_px=2.0, scale_per_frame=1.0, seed=3)
    seq_a = list(generate_sequence(**kwargs))
    seq_b = list(generate_sequence(**kwargs))
    assert len(seq_a) == 10
    assert all(np.array_equal(fa, fb) for (fa, _), (fb, _) in zip(seq_a, seq_b))
    xs = [bbox[0] for _, bbox in seq_a]
    assert xs == sorted(xs) and xs[-1] > xs[0]


def test_generate_sequence_stops_at_frame_edge():
    seq = list(generate_sequence(width=160, height=120, frames=1000,
                                 base_size=40, speed_px=10.0,
                                 scale_per_frame=1.0))
    assert 0 < len(seq) < 1000
    for _, (x, y, w, h) in seq:
        assert x + w <= 160 and y + h <= 120


def test_evaluate_benchmark_thresholds():
    good = [{"frame": i, "found": True, "center_error": 2.0, "iou": 0.9}
            for i in range(20)]
    verdict = evaluate_benchmark(good, min_found_ratio=1.0,
                                 max_mean_center_error=20.0,
                                 max_center_error=40.0, min_mean_iou=0.5)
    assert verdict["ok"], verdict["failures"]

    lost = list(good)
    lost[10] = {"frame": 10, "found": False, "center_error": None, "iou": None}
    verdict = evaluate_benchmark(lost, min_found_ratio=1.0,
                                 max_mean_center_error=20.0,
                                 max_center_error=40.0, min_mean_iou=0.5)
    assert not verdict["ok"]
    assert any("found ratio" in f for f in verdict["failures"])

    drifted = [{"frame": i, "found": True, "center_error": 80.0, "iou": 0.1}
               for i in range(20)]
    verdict = evaluate_benchmark(drifted, min_found_ratio=1.0,
                                 max_mean_center_error=20.0,
                                 max_center_error=40.0, min_mean_iou=0.5)
    assert not verdict["ok"]
    assert any("center error" in f for f in verdict["failures"])
    assert any("IoU" in f for f in verdict["failures"])


def test_kcf_tracks_smooth_motion():
    """The metric-7 offline gate: smooth drift must not break the tracker."""
    args = parse_args(["--tracker", "KCF", "--width", "320", "--height", "240",
                       "--frames", "40", "--target-size", "48",
                       "--speed-px", "2"])
    verdict = run_benchmark(args)
    assert verdict["ok"], verdict["failures"]
    assert verdict["found_ratio"] == 1.0
    assert verdict["mean_center_error"] < 10.0


def test_csrt_tracks_motion_with_scale_change():
    args = parse_args(["--tracker", "CSRT", "--width", "320", "--height", "240",
                       "--frames", "30", "--target-size", "48",
                       "--speed-px", "2", "--scale-per-frame", "1.004"])
    verdict = run_benchmark(args)
    assert verdict["ok"], verdict["failures"]
