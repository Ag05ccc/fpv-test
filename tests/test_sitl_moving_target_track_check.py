import json
import sys
from pathlib import Path

TOOLS_DIR = Path(__file__).resolve().parents[1] / "tools"
if str(TOOLS_DIR) not in sys.path:
    sys.path.insert(0, str(TOOLS_DIR))

from sitl_moving_target_track_check import evaluate, _QualityArgs  # noqa: E402


class _Args:
    min_target_motion = 1.0
    min_found_ratio = 0.5
    max_step_size = "0.0025"


def _write(path, records):
    with path.open("w", encoding="utf-8") as f:
        for r in records:
            f.write(json.dumps(r) + "\n")


def _mixer_records(found, tracking, yaw_delta):
    recs = []
    for i in range(tracking):
        recs.append({
            "event": "kenet_mixer_sample", "state": "TRACKING",
            "target_found": i < found,
            "first8": {"delta": [0, 0, 0, yaw_delta if i else 0, 0, 0, 0, 0]},
            "controller": {}, "target_center": [320 + i, 240],
        })
    return recs


def _diag_records(n=20, armed_from=2, roll=1.0, spread_motors=(1600, 1600, 1600, 1600)):
    recs = []
    for i in range(n):
        armed = i >= armed_from
        recs.append({
            "event": "diagnostic_sample", "sample_index": i + 1,
            "time": 1000.0 + i * 0.5,
            "msp": {"connected": True,
                    "status": {"armed": armed,
                               "active_modes": ["ARM", "ANGLE"] if armed else ["ANGLE"]},
                    "attitude": {"roll": roll if armed else 0.0, "pitch": 0.5, "yaw": 0},
                    "motor": list(spread_motors) if armed else [1000, 1000, 1000, 1000]},
            "gazebo": {"rtf": 1.0, "step_size": 0.0025},
            "gazebo_pose": {"pose": {"position": {"x": 0, "y": 0, "z": 0.5 + 0.1 * i},
                                     "euler_deg": {"roll": 0, "pitch": 0, "yaw": 90 - i}}},
        })
    return recs


def _mover_records(lateral_range):
    return [{"event": "target_mover_summary",
             "measured_lateral_range": lateral_range, "samples": 100}]


def test_a4_pass_when_tracks_moving_target_and_stays_level(tmp_path):
    mixer = tmp_path / "mixer.jsonl"; _write(mixer, _mixer_records(400, 400, 5))
    diag = tmp_path / "diag.jsonl"; _write(diag, _diag_records())
    mover = tmp_path / "mover.jsonl"; _write(mover, _mover_records(6.0))
    motor = tmp_path / "missing.jsonl"
    v = evaluate(mixer, diag, motor, mover, _Args())
    assert v["ok"], v["failures"]
    assert v["target_lateral_range_m"] == 6.0
    assert v["max_yaw_cmd_delta"] == 5
    assert v["found_ratio"] == 1.0


def test_a4_fail_when_target_did_not_move(tmp_path):
    mixer = tmp_path / "mixer.jsonl"; _write(mixer, _mixer_records(400, 400, 5))
    diag = tmp_path / "diag.jsonl"; _write(diag, _diag_records())
    mover = tmp_path / "mover.jsonl"; _write(mover, _mover_records(0.1))
    v = evaluate(mixer, diag, tmp_path / "m.jsonl", mover, _Args())
    assert not v["ok"]
    assert any("didn't move" in f for f in v["failures"])


def test_a4_fail_when_no_yaw_command(tmp_path):
    mixer = tmp_path / "mixer.jsonl"; _write(mixer, _mixer_records(400, 400, 0))
    diag = tmp_path / "diag.jsonl"; _write(diag, _diag_records())
    mover = tmp_path / "mover.jsonl"; _write(mover, _mover_records(6.0))
    v = evaluate(mixer, diag, tmp_path / "m.jsonl", mover, _Args())
    assert not v["ok"]
    assert any("never commanded yaw" in f for f in v["failures"])


def test_a4_fail_when_flight_flips(tmp_path):
    mixer = tmp_path / "mixer.jsonl"; _write(mixer, _mixer_records(400, 400, 5))
    diag = tmp_path / "diag.jsonl"
    _write(diag, _diag_records(roll=180.0, spread_motors=(2000, 1000, 1000, 2000)))
    mover = tmp_path / "mover.jsonl"; _write(mover, _mover_records(6.0))
    v = evaluate(mixer, diag, tmp_path / "m.jsonl", mover, _Args())
    assert not v["ok"]
    assert any("not level" in f for f in v["failures"])


def test_a4_quality_args_are_tracking_tolerant():
    a = _QualityArgs()
    assert a.max_motor_spread < 945  # below flip signature
    assert a.msp_min_connected_fraction <= 0.5  # camera MSP contention
    assert a.min_climb_rate < 0  # low-hover tracking, no climb required
