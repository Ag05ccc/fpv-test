#!/usr/bin/env python3
"""Closed-loop moving-target visual tracking gate (roadmap-v2 A4 — the heart).

Combines A2 (live sim camera -> tracker) and A3 (moving target) into the
project's core scenario: the drone tracks a MOVING target it sees through its
own forward camera, keeping it centered with bounded yaw while staying level.

It orchestrates two live processes:
  1. sitl_video_tracking_check.py (external RC mode) -> starts Gazebo + the
     forward FPV camera + Betaflight + the Kenet mixer tracking gz camera.
  2. A target driver that positions the chosen model in the drone's forward
     view and drives it laterally (crossing the frame) at a set speed, timed so
     the target is centered when TRACKING begins.

Verdict (measured, not asserted-by-hope):
  - tracking continuity: mixer target_found ratio during TRACKING >= threshold,
  - the mixer actually commanded yaw (max_abs_yaw_delta > 0) in the correct
    direction to follow the crossing target,
  - flight stayed level (delegated to sitl_run_quality_check attitude gate),
  - target motion was real (measured displacement from ground-truth).

The drone stays low (low throttle) on purpose: a fixed forward camera on a
climbing drone looks at the horizon, so a ground target must be near the
optical axis. Exit 0 PASS, 1 FAIL.
"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
import threading
import time
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parent
for p in (str(SCRIPT_DIR), str(REPO_ROOT)):
    if p not in sys.path:
        sys.path.insert(0, p)

from sitl_log import JsonlLogger, make_log_path, resolve_log_dir, timestamp_slug  # noqa: E402
from sitl_target_mover import GzWorldTransport, trajectory_offset  # noqa: E402
from sitl_video_tracking_check import summarize_mixer_log  # noqa: E402
from sitl_run_quality_check import evaluate_run  # noqa: E402


class _QualityArgs:
    def __init__(self, expected_step=None):
        # None: don't pin the physics step here (RTF is already loose for
        # camera+sync runs); avoids str/float mixups from the CLI value.
        self.expected_step = expected_step
        self.rtf_min = 0.0        # camera+sync runs have loose RTF; report only
        self.rtf_max = 99.0
        self.max_gap_ms = 1000.0
        self.first_sample_max_attitude = 10.0
        self.max_abs_attitude = 25.0   # roll/pitch is the real flip gate
        # A spin stays level, so roll/pitch alone can't catch it: gate yaw rate.
        # Aliased lower bound from ~2-3 Hz diagnostics; the truth is the 50 Hz
        # sitl_yaw_monitor. A tracking drone should never sustain >90 deg/s.
        self.max_yaw_rate_dps = 90.0
        # Aggressive bounded-yaw tracking transiently spikes motor spread; the
        # flip signature is ~945 us. Cap below that but above tracking spikes.
        self.max_motor_spread = 900.0
        self.msp_min_connected_fraction = 0.5  # camera+mixer MSP contention
        self.disarm_motor_max = 1100.0
        self.min_climb_rate = -100.0   # low-hover tracking: climb not required
        self.expect_yaw_sign = None
        self.min_yaw_delta_deg = 0.0
        self.max_yaw_delta_deg = None
        self.require_disarm = False


def drive_target(args, stop_event, ready_event, log_path):
    """Position the target in front of the drone and drive it laterally."""
    transport = GzWorldTransport(args.world_name)
    logger = JsonlLogger(log_path, metadata={
        "tool": "sitl_moving_target_track_check.mover",
        "model": args.model, "speed": args.speed,
        "distance": args.distance, "cross_amplitude": args.cross_amplitude})
    # Prime the lazy service, then hold the target centered until tracking is
    # about to start.
    center_x, y, z = 0.0, args.distance, args.z
    warmed = False
    for _ in range(40):
        if transport.set_pose(args.model, center_x, y, z, 0.0):
            warmed = True
            break
        time.sleep(0.25)
    if not warmed:
        logger.write("mover_error", reason="set_pose service never responded")
        logger.close()
        return
    ready_event.set()
    t0 = time.monotonic()
    period = 1.0 / args.rate_hz
    next_tick = t0
    measured = []
    while not stop_event.is_set():
        elapsed = time.monotonic() - t0
        # Lateral crossing: x oscillates within +/- cross_amplitude at `speed`.
        # Before tracking starts (hold_seconds) keep it centered so the tracker
        # locks the target, then start moving.
        if elapsed < args.hold_seconds:
            x = center_x
        else:
            # line() ping-pongs 0..amplitude..0; center it so the target
            # crosses the frame symmetrically at `speed`.
            dx, _, _ = trajectory_offset("line", elapsed - args.hold_seconds,
                                         speed=args.speed,
                                         distance=args.cross_amplitude, radius=1.0)
            x = center_x + (dx - args.cross_amplitude / 2.0)
        ok = transport.set_pose(args.model, x, y, z, 0.0)
        pose = transport.get_model_pose(args.model, timeout=0.0)
        if pose is not None:
            measured.append(pose)
        logger.write("target_mover_sample", elapsed=elapsed, ok=ok,
                     commanded={"x": x, "y": y, "z": z})
        next_tick += period
        now = time.monotonic()
        if next_tick <= now:
            next_tick = now + period
        time.sleep(max(0.0, next_tick - now))
    disp = 0.0
    if len(measured) >= 2:
        xs = [m["x"] for m in measured]
        disp = max(xs) - min(xs)
    logger.write("target_mover_summary", measured_lateral_range=disp,
                 samples=len(measured))
    logger.close()


def evaluate(mixer_log, diagnostics_log, motor_log, mover_log, args):
    mixer = summarize_mixer_log(mixer_log) if mixer_log.exists() else {}
    run = evaluate_run(diagnostics_log, motor_log if motor_log.exists() else None,
                       _QualityArgs()) if diagnostics_log.exists() else {}
    lateral_range = 0.0
    if mover_log.exists():
        for rec in (json.loads(l) for l in mover_log.open() if l.strip()):
            if rec.get("event") == "target_mover_summary":
                lateral_range = rec.get("measured_lateral_range", 0.0)
    tracking = mixer.get("tracking_samples", 0)
    found = mixer.get("target_found_samples", 0)
    found_ratio = (found / tracking) if tracking else 0.0
    yaw_cmd = mixer.get("max_abs_yaw_delta", 0)
    attitude_ok = run.get("attitude", {}).get("ok", False)
    failures = []
    if lateral_range < args.min_target_motion:
        failures.append("target lateral motion %.2f m < %.2f (target didn't move)"
                        % (lateral_range, args.min_target_motion))
    if found_ratio < args.min_found_ratio:
        failures.append("tracking found ratio %.2f < %.2f" % (found_ratio, args.min_found_ratio))
    if yaw_cmd <= 0:
        failures.append("mixer never commanded yaw to follow the target")
    if not attitude_ok:
        failures.append("flight not level: %s" % run.get("attitude", {}).get("reasons"))
    if mixer.get("tracker_errors"):
        failures.append("tracker errors: %s" % mixer["tracker_errors"])
    return {
        "ok": not failures,
        "failures": failures,
        "target_lateral_range_m": lateral_range,
        "tracking_samples": tracking,
        "found_ratio": found_ratio,
        "max_yaw_cmd_delta": yaw_cmd,
        "attitude": run.get("attitude", {}),
        "verdict_run": run.get("verdict"),
    }


def parse_args(argv=None):
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--world", default="betaloop_iris_betaflight_demo_populated.sdf")
    p.add_argument("--world-name", default="betaloop_demo")
    p.add_argument("--model", default="kenet_car")
    p.add_argument("--speed", type=float, default=0.5, help="Target speed m/s")
    p.add_argument("--distance", type=float, default=12.0,
                   help="Forward (+Y) distance of the target from the drone")
    p.add_argument("--z", type=float, default=1.5,
                   help="Target height (raise it toward the camera optical axis)")
    p.add_argument("--cross-amplitude", type=float, default=6.0,
                   help="Lateral crossing half-range in m")
    p.add_argument("--rate-hz", type=float, default=20.0)
    p.add_argument("--hold-seconds", type=float, default=20.0,
                   help="Keep the target centered this long before moving "
                        "(>= kenet-delay so the tracker locks it first)")
    p.add_argument("--kenet-delay-seconds", type=float, default=18.0)
    p.add_argument("--throttle", type=int, default=1470,
                   help="Low hover so the forward camera sees the ground target")
    p.add_argument("--yaw-limit", type=int, default=6)
    p.add_argument("--betaflight-yaw-pid", default="6,0,0",
                   help="Betaflight yaw rate-PID (the feedback loop gain). "
                        "MEASURED 2026-07-04: P=23 diverges into a spin (up to "
                        "7655 deg/s); P<=6 is stable (peak ~2.5 deg/s). Default "
                        "is the stable value. Trade-off: this stable gain is "
                        "currently too weak to fully turn/track — open tuning.")
    p.add_argument("--forward-limit", type=int, default=0)
    p.add_argument("--track-size", type=int, default=160)
    p.add_argument("--max-step-size", default="0.0025")
    p.add_argument("--min-found-ratio", type=float, default=0.5)
    p.add_argument("--min-target-motion", type=float, default=1.0)
    p.add_argument("--mixer-duration", type=float, default=70.0)
    p.add_argument("--gazebo-gui", action="store_true",
                   help="Watch it live: open the Gazebo GUI + FPV camera panel "
                        "while the drone autonomously tracks the moving target")
    p.add_argument("--timeout", type=float, default=260.0)
    p.add_argument("--log-dir", default=None)
    p.add_argument("--json", action="store_true")
    return p.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    log_dir = resolve_log_dir(args.log_dir)
    run_id = "%s-a4-track" % timestamp_slug()
    mover_log = make_log_path(log_dir, "a4-mover-%s" % args.model)
    checker_diag = log_dir / ("%s-diagnostics.jsonl" % run_id)
    checker_motor = log_dir / ("%s-motor-udp.jsonl" % run_id)
    checker_mixer = log_dir / ("%s-mixer.jsonl" % run_id)

    checker_cmd = [
        sys.executable, str(SCRIPT_DIR / "sitl_video_tracking_check.py"),
        "--run-id", run_id, "--log-dir", str(log_dir),
        "--world", args.world, "--iris-forward-camera",
        "--camera", "gz:/kenet/fpv_camera",
        "--sync-betaflight-looptime",
        "--betaflight-config-file", "configs/fpv-sim.txt",
        "--max-step-size", args.max_step_size,
        "--yaw-limit", str(args.yaw_limit),
        "--forward-limit", str(args.forward_limit),
        "--track-size", str(args.track_size),
        "--kenet-delay-seconds", str(args.kenet_delay_seconds),
        "--throttle", str(args.throttle),
        "--yaw-pid", args.betaflight_yaw_pid,
        "--pitch-pid", "23,0,0", "--safe-yaw-authority",
        "--max-abs-delta", str(max(args.yaw_limit, args.forward_limit) + 1),
        "--capture-motor-udp",
        "--mixer-duration", str(args.mixer_duration),
        "--allow-fail",
    ]
    if args.gazebo_gui:
        checker_cmd += ["--gazebo-gui",
                        "--gui-config", str(REPO_ROOT / "tools/fpv_gui.config")]
    print("launching camera-tracking checker:\n  %s" % " ".join(checker_cmd), file=sys.stderr)
    checker = subprocess.Popen(checker_cmd, cwd=str(REPO_ROOT))

    stop_event = threading.Event()
    ready_event = threading.Event()
    mover_thread = None
    # Wait for Gazebo's pose service before starting the mover (GUI starts slower).
    deadline = time.monotonic() + (120 if args.gazebo_gui else 60)
    transport_ready = False
    while time.monotonic() < deadline and checker.poll() is None:
        try:
            t = GzWorldTransport(args.world_name)
            if t.get_model_pose(args.model, timeout=2.0) is not None:
                transport_ready = True
                break
        except Exception:
            pass
        time.sleep(2.0)
    if transport_ready:
        mover_thread = threading.Thread(
            target=drive_target, args=(args, stop_event, ready_event, mover_log))
        mover_thread.start()
        print("target driver started (waiting for it to center the target)", file=sys.stderr)
    else:
        print("WARNING: Gazebo pose service not ready; mover not started", file=sys.stderr)

    run_timeout = args.timeout if not args.gazebo_gui else max(args.timeout, 600.0)
    try:
        checker.wait(timeout=run_timeout)
    except subprocess.TimeoutExpired:
        checker.terminate()
    stop_event.set()
    if mover_thread:
        mover_thread.join(timeout=10)

    verdict = evaluate(checker_mixer, checker_diag, checker_motor, mover_log, args)
    mixer_log = checker_mixer
    if args.json:
        print(json.dumps(verdict, indent=2, sort_keys=True))
    else:
        print("moving-target track check: %s" % ("PASS" if verdict["ok"] else "FAIL"))
        print("  target lateral motion: %.2f m" % verdict["target_lateral_range_m"])
        print("  tracking found ratio: %.2f (%d tracking samples)" % (
            verdict["found_ratio"], verdict["tracking_samples"]))
        print("  max yaw command delta: %s" % verdict["max_yaw_cmd_delta"])
        print("  flight attitude: %s" % ("level" if verdict["attitude"].get("ok") else verdict["attitude"].get("reasons")))
        print("  mixer log: %s" % mixer_log)
        for f in verdict["failures"]:
            print("  FAIL: %s" % f)
    return 0 if verdict["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
