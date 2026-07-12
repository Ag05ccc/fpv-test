import math
import sys
from pathlib import Path

TOOLS_DIR = Path(__file__).resolve().parents[1] / "tools"
if str(TOOLS_DIR) not in sys.path:
    sys.path.insert(0, str(TOOLS_DIR))

from sitl_track_truth import (  # noqa: E402
    evaluate_convergence,
    horizontal_distance,
    quat_to_yaw_deg,
)


def sample(t, drone, target):
    return {"t": t, "drone": drone, "target": target,
            "dist": horizontal_distance(drone, target)}


DEFAULTS = dict(min_approach_m=2.0, max_distance_growth_m=1.0,
                max_yaw_rate=90.0, min_translation_m=1.0)


def test_horizontal_distance_and_yaw():
    assert horizontal_distance((0, 0, 5), (3, 4, 0)) == 5.0
    assert round(quat_to_yaw_deg(math.cos(math.pi / 4), 0, 0, math.sin(math.pi / 4)), 1) == 90.0


def test_approaching_static_target_passes():
    # target at (0,10); drone flies from (0,0) to (0,8), yaw steady
    samples = [sample(i * 0.1, (0, i * 0.4, 2, 90.0), (0, 10, 0)) for i in range(20)]
    v = evaluate_convergence(samples, **DEFAULTS)
    assert v["ok"], v["failures"]
    assert v["distance_closed_m"] > 2.0
    assert v["drone_path_m"] > 1.0


def test_spin_in_place_fails():
    # drone sits at (0,0) and spins on yaw; target at (0,10) never approached
    samples = [sample(i * 0.05, (0, 0, 2, (i * 40) % 360 - 180), (0, 10, 0))
               for i in range(40)]
    v = evaluate_convergence(samples, **DEFAULTS)
    assert not v["ok"]
    joined = " ".join(v["failures"])
    assert "spin" in joined
    assert "did not close" in joined
    assert "move through space" in joined


def test_following_moving_target_passes():
    # target moves +x, drone chases and keeps distance bounded, translates along
    samples = []
    for i in range(30):
        tx = i * 0.2
        dx = i * 0.19            # drone stays ~just behind the target in x
        samples.append(sample(i * 0.1, (dx, 9.0, 2, 90.0), (tx, 10.0, 0)))
    v = evaluate_convergence(samples, **DEFAULTS)
    assert v["ok"], v["failures"]


def test_drifting_away_fails():
    # drone drifts away from a static target: distance grows, no approach
    samples = [sample(i * 0.1, (0, -i * 0.3, 2, 90.0), (0, 10, 0)) for i in range(20)]
    v = evaluate_convergence(samples, **DEFAULTS)
    assert not v["ok"]
    assert any("did not close" in f for f in v["failures"])
