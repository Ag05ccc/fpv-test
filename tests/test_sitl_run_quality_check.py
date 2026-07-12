import json
import sys
from pathlib import Path

TOOLS_DIR = Path(__file__).resolve().parents[1] / "tools"
if str(TOOLS_DIR) not in sys.path:
    sys.path.insert(0, str(TOOLS_DIR))

from sitl_run_quality_check import (  # noqa: E402
    check_attitude,
    check_yaw_spin,
    check_climb,
    check_disarm,
    check_health,
    check_signed_yaw_response,
    check_validity,
    evaluate_run,
    extract_flight_samples,
    main,
    parse_args,
    unwrapped_yaw_delta,
)


def diagnostic_record(index, *, t=None, armed=True, modes=("ARM", "ANGLE"),
                      roll=0.0, pitch=0.0, motors=(1500, 1500, 1500, 1500),
                      connected=True, rtf=1.0, step=0.0025, z=0.0,
                      pose_roll=0.0, pose_pitch=0.0, pose_yaw=90.0):
    return {
        "event": "diagnostic_sample",
        "sample_index": index,
        "time": 1000.0 + index * 0.5 if t is None else t,
        "msp": {
            "connected": connected,
            "status": {"armed": armed, "active_modes": list(modes)},
            "attitude": {"roll": roll, "pitch": pitch, "yaw": 0},
            "motor": list(motors),
        },
        "gazebo": {"rtf": rtf, "step_size": step},
        "gazebo_pose": {
            "pose": {
                "position": {"x": 0.0, "y": 0.0, "z": z},
                "euler_deg": {"roll": pose_roll, "pitch": pose_pitch,
                              "yaw": pose_yaw},
            },
        },
    }


def motor_record(monotonic, motors=(1500, 1500, 1500, 1500)):
    return {"event": "motor_udp_sample", "ok": True, "monotonic": monotonic,
            "motors_raw": list(motors)}


def clean_run_records(n=20):
    records = []
    for i in range(n):
        armed = 1 <= i < n - 2
        records.append(diagnostic_record(
            i + 1,
            armed=armed,
            modes=("ARM", "ANGLE") if armed else ("ANGLE",),
            motors=(1600, 1600, 1600, 1600) if armed else (1000, 1000, 1000, 1000),
            z=0.5 * i if armed else (0.0 if i == 0 else 0.5 * (n - 3)),
        ))
    return records


def test_extract_flight_samples_pulls_fields():
    samples = extract_flight_samples([
        {"event": "session_start"},
        diagnostic_record(1, rtf=1.001, z=2.5, pose_yaw=45.0),
        {"event": "session_end"},
    ])
    assert len(samples) == 1
    sample = samples[0]
    assert sample["armed"] is True
    assert sample["angle_active"] is True
    assert sample["rtf"] == 1.001
    assert sample["pos_z"] == 2.5
    assert sample["pose_yaw"] == 45.0


def test_validity_clean_run_is_valid():
    samples = extract_flight_samples(clean_run_records())
    result = check_validity(samples)
    assert result["valid"], result["reasons"]
    assert result["armed_angle_samples"] > 0


def test_validity_never_armed_is_invalid():
    records = [diagnostic_record(i + 1, armed=False, modes=("ANGLE",),
                                 motors=(1000,) * 4) for i in range(5)]
    result = check_validity(extract_flight_samples(records))
    assert not result["valid"]
    assert any("armed_angle_samples == 0" in r for r in result["reasons"])


def test_validity_dirty_first_sample_is_invalid():
    records = clean_run_records()
    records[0] = diagnostic_record(1, armed=False, modes=(), roll=-180.0,
                                   pose_roll=-180.0, motors=(1000,) * 4)
    result = check_validity(extract_flight_samples(records))
    assert not result["valid"]
    assert any("dirty first sample" in r for r in result["reasons"])


def test_validity_msp_drop_is_invalid():
    records = clean_run_records()
    records[5]["msp"]["connected"] = False
    result = check_validity(extract_flight_samples(records))
    assert not result["valid"]
    assert any("MSP connected" in r for r in result["reasons"])


def test_validity_msp_drop_tolerated_when_relaxed():
    # camera/mixer runs: MSP polling contention drops samples but the flight
    # is still valid; a relaxed fraction accepts it.
    records = clean_run_records()
    for i in range(4):
        records[5 + i]["msp"]["connected"] = False
    strict = check_validity(extract_flight_samples(records))
    assert not strict["valid"]
    relaxed = check_validity(extract_flight_samples(records),
                             msp_min_connected_fraction=0.5)
    assert relaxed["valid"], relaxed["reasons"]


def test_health_rtf_band():
    samples = extract_flight_samples(clean_run_records())
    ok = check_health(samples, [], rtf_min=0.99, rtf_max=1.01)
    assert ok["ok"], ok["reasons"]
    bad = extract_flight_samples(
        [diagnostic_record(1, rtf=0.9), diagnostic_record(2, rtf=1.3)])
    result = check_health(bad, [])
    assert not result["ok"]
    assert any("RTF min" in r for r in result["reasons"])
    assert any("RTF max" in r for r in result["reasons"])


def test_health_step_pinning():
    samples = extract_flight_samples(clean_run_records())
    result = check_health(samples, [], expected_step=0.001)
    assert not result["ok"]
    assert any("step size" in r for r in result["reasons"])
    assert check_health(samples, [], expected_step=0.0025)["ok"]


def test_health_cadence_and_gap():
    samples = extract_flight_samples(clean_run_records())
    # metronomic packets at exactly the step: PASS
    good = [motor_record(10.0 + 0.0025 * i) for i in range(400)]
    assert check_health(samples, good)["ok"]
    # a 50 ms hole in the active window: FAIL
    gappy = list(good)
    gappy[200] = motor_record(gappy[199]["monotonic"] + 0.050)
    result = check_health(samples, gappy)
    assert not result["ok"]
    assert any("gap" in r for r in result["reasons"])
    # cadence inconsistent with the physics step: FAIL
    slow = [motor_record(10.0 + 0.010 * i) for i in range(100)]
    result = check_health(samples, slow)
    assert not result["ok"]
    assert any("cadence" in r for r in result["reasons"])


def test_attitude_gate_passes_level_flight():
    samples = extract_flight_samples(clean_run_records())
    result = check_attitude(samples)
    assert result["ok"], result["reasons"]
    assert result["max_roll"] == 0.0


def test_attitude_gate_catches_flip():
    records = [diagnostic_record(1, armed=True, roll=0.0)]
    records += [diagnostic_record(i + 2, armed=True, roll=180.0,
                                  motors=(2000, 1000, 1000, 2000)) for i in range(5)]
    result = check_attitude(extract_flight_samples(records))
    assert not result["ok"]
    assert any("flip signature" in r for r in result["reasons"])
    assert any("spread" in r for r in result["reasons"])


def test_yaw_spin_gate_catches_level_spin():
    # a drone spinning fast on yaw but staying LEVEL (roll/pitch ~0): the case
    # a roll/pitch-only gate misses. yaw jumps 40 deg every 0.1 s = 400 deg/s.
    records = []
    for i in range(20):
        records.append(diagnostic_record(i + 1, t=1000.0 + i * 0.1, armed=True,
                                         roll=0.5, pitch=0.5,
                                         pose_yaw=(i * 40) % 360 - 180))
    samples = extract_flight_samples(records)
    attitude = check_attitude(samples)
    spin = check_yaw_spin(samples, max_yaw_rate_dps=90.0)
    assert attitude["ok"]          # roll/pitch gate is fooled (level)
    assert not spin["ok"]          # yaw-rate gate catches it
    assert spin["max_yaw_rate_dps"] > 300
    assert any("spin" in r for r in spin["reasons"])


def test_yaw_spin_gate_passes_gentle_yaw():
    # gentle reactive tracking yaw: ~10 deg/s, not a spin
    records = [diagnostic_record(i + 1, t=1000.0 + i * 0.1, armed=True,
                                 pose_yaw=90.0 + i * 1.0) for i in range(20)]
    spin = check_yaw_spin(extract_flight_samples(records), max_yaw_rate_dps=90.0)
    assert spin["ok"], spin["reasons"]


def test_disarm_check():
    samples = extract_flight_samples(clean_run_records())
    assert check_disarm(samples)["ok"]
    still_armed = extract_flight_samples(
        [diagnostic_record(1, armed=True, motors=(1700,) * 4)] * 3)
    result = check_disarm(still_armed)
    assert not result["ok"]
    assert any("still armed" in r for r in result["reasons"])


def test_climb_rate_normalizes_by_window():
    # 9.5 m over 9.5 s = 1 m/s
    records = [diagnostic_record(i + 1, z=0.5 * i) for i in range(20)]
    result = check_climb(extract_flight_samples(records), min_climb_rate=0.3)
    assert result["ok"]
    assert result["climb_rate"] > 0.9
    flat = [diagnostic_record(i + 1, z=0.0) for i in range(20)]
    result = check_climb(extract_flight_samples(flat), min_climb_rate=0.3)
    assert not result["ok"]


def test_unwrapped_yaw_handles_wraparound():
    # rotating ~+30 deg/sample through the 180/-180 seam: 3 steps = ~+90
    yaws = [150.0, 179.0, -152.0, -120.0]
    assert unwrapped_yaw_delta(yaws) > 0
    assert abs(unwrapped_yaw_delta(yaws) - 90.0) < 1.0


def test_signed_yaw_response_measurement_and_gate():
    records = [diagnostic_record(i + 1, pose_yaw=90.0 + 5.0 * i)
               for i in range(10)]
    samples = extract_flight_samples(records)
    measured = check_signed_yaw_response(samples)
    assert measured["ok"] and measured["yaw_delta_deg"] > 0
    gated = check_signed_yaw_response(samples, expect_sign=1,
                                      min_delta_deg=10.0)
    assert gated["ok"]
    wrong_sign = check_signed_yaw_response(samples, expect_sign=-1)
    assert not wrong_sign["ok"]
    too_small = check_signed_yaw_response(samples, expect_sign=1,
                                          min_delta_deg=500.0)
    assert not too_small["ok"]
    runaway = check_signed_yaw_response(samples, expect_sign=1,
                                        max_delta_deg=10.0)
    assert not runaway["ok"]


def write_jsonl(path, records):
    with path.open("w", encoding="utf-8") as handle:
        for record in records:
            handle.write(json.dumps(record) + "\n")


def test_evaluate_run_and_chain_end_to_end(tmp_path):
    good = tmp_path / "good.jsonl"
    write_jsonl(good, clean_run_records())
    invalid = tmp_path / "invalid.jsonl"
    write_jsonl(invalid, [diagnostic_record(i + 1, armed=False, modes=(),
                                            motors=(1000,) * 4)
                          for i in range(5)])
    args = parse_args(["--diagnostics", str(good)])
    run = evaluate_run(good, None, args)
    assert run["verdict"] == "PASS"

    assert main(["--diagnostics", str(good)]) == 0
    assert main(["--diagnostics", str(invalid)]) == 2
    # chain with one invalid run cannot pass
    assert main(["--diagnostics", str(good), str(invalid)]) == 2


def test_require_disarm_gates(tmp_path):
    # valid run (clean disarmed first sample) whose window ends still armed
    records = [diagnostic_record(1, armed=False, modes=("ANGLE",),
                                 motors=(1000,) * 4, z=0.0)]
    records += [diagnostic_record(i + 1, armed=True, motors=(1700,) * 4,
                                  z=0.5 * i) for i in range(1, 20)]
    path = tmp_path / "armed-at-end.jsonl"
    write_jsonl(path, records)
    assert main(["--diagnostics", str(path)]) == 0
    assert main(["--diagnostics", str(path), "--require-disarm"]) == 1
