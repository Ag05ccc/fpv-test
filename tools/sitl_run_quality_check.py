#!/usr/bin/env python3
"""Automated run-validity, sim-health, and flight-quality verdicts for one or
more SITL runs (roadmap-v2 A0; layers 2-4 of
docs/sitl-flight-readiness-criteria.md, previously checked by hand).

Input: the diagnostics JSONL of a takeoff-runner run (and optionally its motor
UDP JSONL). Output: a structured verdict.

  VALIDITY  (INVALID beats PASS/FAIL — an invalid run is never evidence)
    - armed_angle_samples > 0 (ARM + ANGLE co-active over MSP)
    - clean first sample: |roll|,|pitch| < threshold on both MSP attitude and
      Gazebo pose, motors at idle, disarmed (catches state carried over from
      a previous run, e.g. the roll -180 case)
    - MSP connected on every sample (no mid-window drop)
  HEALTH    (sim itself ran honestly)
    - RTF inside [--rtf-min, --rtf-max] (measured band 0.99-1.01)
    - physics step matches --expected-step when given (evidence labeling)
    - motor UDP cadence consistent with the step; no gap > --max-gap-ms in
      the active (motors spinning) window
  FLIGHT    (quality metrics)
    - disarm: run ends disarmed with motors back at idle (metric 1's missing
      half)
    - climb rate: altitude gain / window seconds >= --min-climb-rate
      (window-normalized version of the old raw altitude-gain gate)
    - signed yaw response (optional): unwrapped Gazebo pose yaw delta; gated
      only when --expect-yaw-sign is given, otherwise reported as
      measurement. Magnitude gated by --min-yaw-delta-deg / capped by
      --max-yaw-delta-deg when provided.

Pass multiple --diagnostics files (with matching order of --motor-udp files,
or none) to get a repeatability chain verdict: chain PASS requires every run
VALID and PASS (the 5/5 rule).

Exit codes: 0 chain PASS, 1 any FAIL, 2 any INVALID.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any


def iter_records(path: Path):
    with path.open(encoding="utf-8") as handle:
        for line in handle:
            line = line.strip()
            if not line:
                continue
            try:
                yield json.loads(line)
            except json.JSONDecodeError:
                continue


def numeric_or_none(value):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    return float(value)


# ── extraction ───────────────────────────────────────────────────────

def extract_flight_samples(records) -> list[dict[str, Any]]:
    """Flatten diagnostic_sample records into the fields the checks need."""
    samples = []
    for record in records:
        if record.get("event") != "diagnostic_sample":
            continue
        msp = record.get("msp") or {}
        status = msp.get("status") or {}
        attitude = msp.get("attitude") or {}
        gazebo = record.get("gazebo") or {}
        gazebo_pose = record.get("gazebo_pose") or {}
        pose = gazebo_pose.get("pose") or {}
        position = pose.get("position") or {}
        euler = pose.get("euler_deg") or {}
        active_modes = status.get("active_modes") or []
        samples.append({
            "index": record.get("sample_index"),
            "time": numeric_or_none(record.get("time")),
            "msp_connected": bool(msp.get("connected")),
            "armed": status.get("armed"),
            "angle_active": "ANGLE" in active_modes,
            "roll": numeric_or_none(attitude.get("roll")),
            "pitch": numeric_or_none(attitude.get("pitch")),
            "motors": [m for m in (msp.get("motor") or [])
                       if numeric_or_none(m) is not None],
            "rtf": numeric_or_none(gazebo.get("rtf")),
            "step_size": numeric_or_none(gazebo.get("step_size")),
            "pos_x": numeric_or_none(position.get("x")),
            "pos_y": numeric_or_none(position.get("y")),
            "pos_z": numeric_or_none(position.get("z")),
            "pose_roll": numeric_or_none(euler.get("roll")),
            "pose_pitch": numeric_or_none(euler.get("pitch")),
            "pose_yaw": numeric_or_none(euler.get("yaw")),
        })
    return samples


def extract_motor_packets(records) -> list[dict[str, Any]]:
    packets = []
    for record in records:
        if record.get("event") != "motor_udp_sample" or not record.get("ok", True):
            continue
        monotonic = numeric_or_none(record.get("monotonic"))
        if monotonic is None:
            continue
        packets.append({
            "monotonic": monotonic,
            "motors_raw": [m for m in (record.get("motors_raw") or [])
                           if numeric_or_none(m) is not None],
        })
    return packets


# ── checks (pure) ────────────────────────────────────────────────────

def check_validity(samples: list[dict], *, max_first_attitude: float = 5.0,
                   idle_motor_max: float = 1050.0,
                   msp_min_connected_fraction: float = 1.0) -> dict:
    reasons = []
    armed_angle = sum(1 for s in samples if s["armed"] and s["angle_active"])
    if not samples:
        return {"valid": False, "reasons": ["no diagnostic samples"],
                "armed_angle_samples": 0}
    if armed_angle == 0:
        reasons.append("armed_angle_samples == 0 (run never armed; INVALID, "
                       "not FAIL)")
    first = samples[0]
    for label, roll, pitch in (
            ("msp attitude", first["roll"], first["pitch"]),
            ("gazebo pose", first["pose_roll"], first["pose_pitch"])):
        for axis, value in (("roll", roll), ("pitch", pitch)):
            if value is not None and abs(value) >= max_first_attitude:
                reasons.append(
                    "dirty first sample: %s %s %.1f deg (carried-over state?)"
                    % (label, axis, value))
    if first["armed"]:
        reasons.append("dirty first sample: already armed")
    if first["motors"] and max(first["motors"]) > idle_motor_max:
        reasons.append("dirty first sample: motors above idle (%s)"
                       % first["motors"])
    disconnected = sum(1 for s in samples if not s["msp_connected"])
    connected_fraction = 1.0 - disconnected / len(samples)
    if connected_fraction < msp_min_connected_fraction:
        reasons.append(
            "MSP connected on %.0f%% of samples < required %.0f%% "
            "(diagnostics polling dropped; raise --msp-min-connected-fraction "
            "for camera/mixer runs where MSP contention is expected)"
            % (connected_fraction * 100, msp_min_connected_fraction * 100))
    return {"valid": not reasons, "reasons": reasons,
            "armed_angle_samples": armed_angle, "samples": len(samples)}


def check_health(samples: list[dict], motor_packets: list[dict], *,
                 rtf_min: float = 0.99, rtf_max: float = 1.01,
                 expected_step: float | None = None,
                 max_gap_ms: float = 30.0,
                 cadence_tolerance: float = 0.25,
                 spin_threshold: float = 1050.0) -> dict:
    reasons = []
    rtf_values = [s["rtf"] for s in samples if s["rtf"] is not None]
    rtf_lo = min(rtf_values) if rtf_values else None
    rtf_hi = max(rtf_values) if rtf_values else None
    if not rtf_values:
        reasons.append("no RTF samples")
    else:
        if rtf_lo < rtf_min:
            reasons.append("RTF min %.4f < %.2f" % (rtf_lo, rtf_min))
        if rtf_hi > rtf_max:
            reasons.append("RTF max %.4f > %.2f" % (rtf_hi, rtf_max))
    steps = {s["step_size"] for s in samples if s["step_size"] is not None}
    observed_step = steps.pop() if len(steps) == 1 else (None if not steps else sorted(steps))
    if expected_step is not None:
        if observed_step is None or not isinstance(observed_step, float):
            reasons.append("step size missing or inconsistent: %s" % observed_step)
        elif abs(observed_step - expected_step) > 1e-9:
            reasons.append("step size %.4g != expected %.4g"
                           % (observed_step, expected_step))

    cadence = {"packets": len(motor_packets)}
    if motor_packets:
        active = [p for p in motor_packets
                  if p["motors_raw"] and max(p["motors_raw"]) > spin_threshold]
        window = active if len(active) >= 2 else motor_packets
        times = [p["monotonic"] for p in window]
        deltas = [b - a for a, b in zip(times, times[1:])]
        if deltas:
            mean_delta = sum(deltas) / len(deltas)
            max_delta = max(deltas)
            cadence.update({
                "active_packets": len(window),
                "mean_interval_ms": mean_delta * 1000.0,
                "max_interval_ms": max_delta * 1000.0,
            })
            if max_delta * 1000.0 > max_gap_ms:
                reasons.append("motor UDP gap %.1f ms > %.1f ms in active window"
                               % (max_delta * 1000.0, max_gap_ms))
            if isinstance(observed_step, float) and observed_step > 0:
                if abs(mean_delta - observed_step) > cadence_tolerance * observed_step:
                    reasons.append(
                        "motor UDP cadence %.2f ms inconsistent with step %.2f ms"
                        % (mean_delta * 1000.0, observed_step * 1000.0))
    return {"ok": not reasons, "reasons": reasons,
            "rtf_min": rtf_lo, "rtf_max": rtf_hi,
            "step_size": observed_step, "cadence": cadence}


def check_attitude(samples: list[dict], *, max_abs_attitude: float = 35.0,
                   max_motor_spread: float = 400.0) -> dict:
    """The real flip discriminator: max |roll|/|pitch| and motor spread while
    armed. A flip pins roll/pitch to ~180 and spread to ~945."""
    reasons = []
    armed = [s for s in samples if s["armed"]]
    window = armed if armed else samples
    rolls = [abs(s["roll"]) for s in window if s["roll"] is not None]
    pitches = [abs(s["pitch"]) for s in window if s["pitch"] is not None]
    # A quad reports 8 motor slots with the last 4 zero-padded; spread must use
    # only the 4 active motors or the padding zeros dominate max-min.
    spreads = [max(s["motors"][:4]) - min(s["motors"][:4])
               for s in window if len(s["motors"]) >= 4]
    max_roll = max(rolls) if rolls else None
    max_pitch = max(pitches) if pitches else None
    max_spread = max(spreads) if spreads else None
    if max_roll is None and max_pitch is None:
        reasons.append("no attitude samples")
    else:
        worst = max(v for v in (max_roll, max_pitch) if v is not None)
        if worst > max_abs_attitude:
            reasons.append("max |roll|/|pitch| %.1f deg > %.1f (flip signature)"
                           % (worst, max_abs_attitude))
    if max_spread is not None and max_spread > max_motor_spread:
        reasons.append("max motor spread %.0f us > %.0f (saturation/flip)"
                       % (max_spread, max_motor_spread))
    return {"ok": not reasons, "reasons": reasons, "max_roll": max_roll,
            "max_pitch": max_pitch, "max_motor_spread": max_spread}


def check_yaw_spin(samples: list[dict], *, max_yaw_rate_dps: float = 90.0) -> dict:
    """Detect a yaw spin: a level drone rotating fast on yaw. Uses Gazebo pose
    yaw (independent of the flaky MSP feed). This is the check whose absence
    let a 436 deg/s spin pass a roll/pitch-only gate.

    NOTE: diagnostics poses are sampled slowly (~2-3 Hz), so a fast spin is
    ALIASED and this is a lower bound; the high-rate truth comes from
    tools/sitl_yaw_monitor.py at ~50 Hz. A rate over the threshold here is
    still a definite spin."""
    reasons = []
    armed = [(s["time"], s["pose_yaw"]) for s in samples
             if s["armed"] and s["pose_yaw"] is not None and s["time"] is not None]
    rates = []
    for (t0, y0), (t1, y1) in zip(armed, armed[1:]):
        dt = t1 - t0
        if dt <= 0:
            continue
        rates.append(abs((y1 - y0 + 180.0) % 360.0 - 180.0) / dt)
    max_rate = max(rates) if rates else 0.0
    if max_rate > max_yaw_rate_dps:
        reasons.append(
            "yaw rate %.0f deg/s > %.0f (spin; aliased lower bound from ~2-3 Hz "
            "diagnostics, real rate likely higher — see sitl_yaw_monitor.py)"
            % (max_rate, max_yaw_rate_dps))
    return {"ok": not reasons, "reasons": reasons, "max_yaw_rate_dps": max_rate}


def check_disarm(samples: list[dict], *, idle_motor_max: float = 1100.0) -> dict:
    reasons = []
    if not samples:
        return {"ok": False, "reasons": ["no samples"]}
    was_armed = any(s["armed"] for s in samples)
    last = samples[-1]
    if not was_armed:
        reasons.append("run never armed (disarm check meaningless)")
    if last["armed"]:
        reasons.append("final sample still armed")
    if last["motors"] and max(last["motors"]) > idle_motor_max:
        reasons.append("final motors not at idle: %s" % last["motors"])
    return {"ok": not reasons, "reasons": reasons,
            "final_armed": last["armed"], "final_motors": last["motors"]}


def check_climb(samples: list[dict], *, min_climb_rate: float = 0.3) -> dict:
    reasons = []
    # Normalize over the ARMED window only: pre-arm ground idle (drone sitting
    # at ~0 m for many seconds) otherwise drags the average rate far below the
    # true climb rate and fails clean runs.
    armed = [s for s in samples
             if s["armed"] and s["pos_z"] is not None and s["time"] is not None]
    window = armed if len(armed) >= 2 else [
        s for s in samples if s["pos_z"] is not None and s["time"] is not None]
    if len(window) < 2:
        return {"ok": False, "reasons": ["not enough altitude samples"]}
    altitudes = [s["pos_z"] for s in window]
    times = [s["time"] for s in window]
    gain = max(altitudes) - min(altitudes)
    window_seconds = times[-1] - times[0]
    rate = gain / window_seconds if window_seconds > 0 else 0.0
    if rate < min_climb_rate:
        reasons.append("climb rate %.3f m/s < %.3f m/s (gain %.2f m over %.1f s armed)"
                       % (rate, min_climb_rate, gain, window_seconds))
    return {"ok": not reasons, "reasons": reasons, "altitude_gain": gain,
            "window_seconds": window_seconds, "climb_rate": rate}


def unwrapped_yaw_delta(yaws: list[float]) -> float:
    """Sum of smallest-angle successive differences, in degrees."""
    total = 0.0
    for a, b in zip(yaws, yaws[1:]):
        diff = (b - a + 180.0) % 360.0 - 180.0
        total += diff
    return total


def check_signed_yaw_response(samples: list[dict], *,
                              expect_sign: int | None = None,
                              min_delta_deg: float = 0.0,
                              max_delta_deg: float | None = None) -> dict:
    yaws = [s["pose_yaw"] for s in samples
            if s["pose_yaw"] is not None and s["armed"]]
    if len(yaws) < 2:
        return {"ok": expect_sign is None, "reasons": ["no armed yaw samples"],
                "yaw_delta_deg": None}
    delta = unwrapped_yaw_delta(yaws)
    reasons = []
    if expect_sign is not None:
        if expect_sign > 0 and delta <= 0 or expect_sign < 0 and delta >= 0:
            reasons.append("yaw delta %.1f deg has wrong sign (expected %+d)"
                           % (delta, expect_sign))
        if abs(delta) < min_delta_deg:
            reasons.append("yaw delta %.1f deg below minimum %.1f"
                           % (delta, min_delta_deg))
        if max_delta_deg is not None and abs(delta) > max_delta_deg:
            reasons.append("yaw delta %.1f deg above maximum %.1f"
                           % (delta, max_delta_deg))
    return {"ok": not reasons, "reasons": reasons, "yaw_delta_deg": delta,
            "gated": expect_sign is not None}


# ── per-run and chain verdicts ───────────────────────────────────────

def evaluate_run(diagnostics_path: Path, motor_udp_path: Path | None,
                 args) -> dict:
    samples = extract_flight_samples(iter_records(diagnostics_path))
    motor_packets = (extract_motor_packets(iter_records(motor_udp_path))
                     if motor_udp_path else [])
    validity = check_validity(
        samples, max_first_attitude=args.first_sample_max_attitude,
        msp_min_connected_fraction=getattr(args, "msp_min_connected_fraction", 1.0))
    health = check_health(samples, motor_packets,
                          rtf_min=args.rtf_min, rtf_max=args.rtf_max,
                          expected_step=args.expected_step,
                          max_gap_ms=args.max_gap_ms)
    attitude = check_attitude(samples, max_abs_attitude=args.max_abs_attitude,
                              max_motor_spread=args.max_motor_spread)
    yaw_spin = check_yaw_spin(
        samples, max_yaw_rate_dps=getattr(args, "max_yaw_rate_dps", 90.0))
    disarm = check_disarm(samples, idle_motor_max=args.disarm_motor_max)
    disarm["gated"] = args.require_disarm
    if not args.require_disarm:
        # Report-only until runner diagnostics windows cover the disarm
        # phase; historical 75-sample windows end while still armed.
        disarm = dict(disarm, ok=True)
    climb = check_climb(samples, min_climb_rate=args.min_climb_rate)
    signed = check_signed_yaw_response(
        samples, expect_sign=args.expect_yaw_sign,
        min_delta_deg=args.min_yaw_delta_deg,
        max_delta_deg=args.max_yaw_delta_deg)
    if not validity["valid"]:
        verdict = "INVALID"
    elif all(part["ok"] for part in (attitude, yaw_spin, health, disarm, climb, signed)):
        verdict = "PASS"
    else:
        verdict = "FAIL"
    return {
        "diagnostics": str(diagnostics_path),
        "motor_udp": str(motor_udp_path) if motor_udp_path else None,
        "verdict": verdict,
        "validity": validity,
        "attitude": attitude,
        "yaw_spin": yaw_spin,
        "health": health,
        "disarm": disarm,
        "climb": climb,
        "signed_yaw_response": signed,
    }


def evaluate_chain(runs: list[dict]) -> dict:
    passes = sum(1 for r in runs if r["verdict"] == "PASS")
    invalids = sum(1 for r in runs if r["verdict"] == "INVALID")
    return {
        "runs": len(runs),
        "pass": passes,
        "fail": sum(1 for r in runs if r["verdict"] == "FAIL"),
        "invalid": invalids,
        "chain_pass": passes == len(runs) and len(runs) > 0,
    }


def parse_args(argv=None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--diagnostics", nargs="+", required=True, type=Path,
                        help="Diagnostics JSONL file(s); several = chain")
    parser.add_argument("--motor-udp", nargs="*", type=Path, default=[],
                        help="Motor UDP JSONL file(s), same order as --diagnostics")
    parser.add_argument("--expected-step", type=float, default=None,
                        help="Pin the physics step this evidence claims")
    parser.add_argument("--rtf-min", type=float, default=0.99)
    parser.add_argument("--rtf-max", type=float, default=1.01)
    parser.add_argument("--max-gap-ms", type=float, default=30.0)
    parser.add_argument("--first-sample-max-attitude", type=float, default=5.0)
    parser.add_argument("--msp-min-connected-fraction", type=float, default=1.0,
                        help="Min fraction of samples with MSP connected "
                             "(lower for camera/mixer runs; MSP contention "
                             "drops diagnostics polling but not the flight)")
    parser.add_argument("--disarm-motor-max", type=float, default=1100.0)
    parser.add_argument("--require-disarm", action="store_true",
                        help="Gate on end-of-window disarm (needs a "
                             "diagnostics window that covers the disarm "
                             "phase); default is report-only")
    parser.add_argument("--max-abs-attitude", type=float, default=35.0,
                        help="Max |roll|/|pitch| deg while armed (flip gate)")
    parser.add_argument("--max-motor-spread", type=float, default=400.0,
                        help="Max raw motor spread us while armed (saturation gate)")
    parser.add_argument("--max-yaw-rate-dps", type=float, default=90.0,
                        help="Max |yaw rate| deg/s while armed (spin gate; "
                             "diagnostics are aliased, so this is a lower bound)")
    parser.add_argument("--min-climb-rate", type=float, default=0.1,
                        help="m/s over the armed window (throttle 1500 ~0.13, "
                             "1750 ~0.6); catches 'raised throttle, no climb'")
    parser.add_argument("--expect-yaw-sign", type=int, choices=(-1, 1),
                        default=None,
                        help="Gate the signed yaw response; omit to just measure")
    parser.add_argument("--min-yaw-delta-deg", type=float, default=0.0)
    parser.add_argument("--max-yaw-delta-deg", type=float, default=None)
    parser.add_argument("--json", action="store_true")
    args = parser.parse_args(argv)
    if args.motor_udp and len(args.motor_udp) != len(args.diagnostics):
        parser.error("--motor-udp count must match --diagnostics count")
    return args


def print_run(run: dict) -> None:
    print("%s: %s" % (run["verdict"], run["diagnostics"]))
    validity = run["validity"]
    print("  validity: %s (armed_angle_samples=%d, samples=%d)" % (
        "VALID" if validity["valid"] else "INVALID",
        validity["armed_angle_samples"], validity.get("samples", 0)))
    attitude = run["attitude"]
    print("  attitude: %s (max|roll|=%s max|pitch|=%s max_spread=%s)" % (
        "PASS" if attitude["ok"] else "FAIL",
        "%.1f" % attitude["max_roll"] if attitude["max_roll"] is not None else "n/a",
        "%.1f" % attitude["max_pitch"] if attitude["max_pitch"] is not None else "n/a",
        "%.0f" % attitude["max_motor_spread"] if attitude["max_motor_spread"] is not None else "n/a"))
    health = run["health"]
    print("  health: %s (rtf %s..%s, step=%s, cadence=%s)" % (
        "PASS" if health["ok"] else "FAIL",
        "%.4f" % health["rtf_min"] if health["rtf_min"] is not None else "n/a",
        "%.4f" % health["rtf_max"] if health["rtf_max"] is not None else "n/a",
        health["step_size"],
        ("%.2f ms mean / %.1f ms max" % (
            health["cadence"].get("mean_interval_ms", float("nan")),
            health["cadence"].get("max_interval_ms", float("nan")))
         if health["cadence"].get("mean_interval_ms") is not None else
         "no motor log")))
    disarm = run["disarm"]
    if disarm.get("gated"):
        print("  disarm: %s" % ("PASS" if disarm["ok"] else "FAIL"))
    else:
        print("  disarm: %s (measurement only; gate with --require-disarm)"
              % ("clean" if not disarm.get("reasons") else
                 "; ".join(disarm["reasons"])))
    climb = run["climb"]
    if climb.get("climb_rate") is not None:
        print("  climb: %s (%.2f m over %.1f s = %.3f m/s)" % (
            "PASS" if climb["ok"] else "FAIL", climb["altitude_gain"],
            climb["window_seconds"], climb["climb_rate"]))
    else:
        print("  climb: FAIL (%s)" % "; ".join(climb["reasons"]))
    signed = run["signed_yaw_response"]
    if signed["yaw_delta_deg"] is not None:
        print("  signed yaw response: %s (delta %.1f deg%s)" % (
            "PASS" if signed["ok"] else "FAIL", signed["yaw_delta_deg"],
            "" if signed.get("gated") else ", measurement only"))
    yaw_spin = run["yaw_spin"]
    print("  yaw spin: %s (max yaw rate %.0f deg/s)" % (
        "PASS" if yaw_spin["ok"] else "SPIN", yaw_spin["max_yaw_rate_dps"]))
    for section in ("validity", "attitude", "yaw_spin", "health", "disarm",
                    "climb", "signed_yaw_response"):
        for reason in run[section].get("reasons", []):
            print("    - [%s] %s" % (section, reason))


def main(argv=None) -> int:
    args = parse_args(argv)
    runs = []
    for i, diag in enumerate(args.diagnostics):
        motor = args.motor_udp[i] if args.motor_udp else None
        runs.append(evaluate_run(diag, motor, args))
    chain = evaluate_chain(runs)
    report = {"runs": runs, "chain": chain}
    if args.json:
        print(json.dumps(report, indent=2, sort_keys=True))
    else:
        for run in runs:
            print_run(run)
        if len(runs) > 1:
            print("chain: %s (%d/%d PASS, %d INVALID)" % (
                "PASS" if chain["chain_pass"] else "FAIL",
                chain["pass"], chain["runs"], chain["invalid"]))
    if chain["invalid"]:
        return 2
    return 0 if chain["chain_pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
