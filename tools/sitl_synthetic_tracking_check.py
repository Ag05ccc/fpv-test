#!/usr/bin/env python3
"""Run Gazebo takeoff acceptance with Kenet mixer synthetic target RC."""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
import time
from pathlib import Path
from typing import Any


SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from gazebo_motor_moment_probe import terminate_process
from sitl_log import resolve_log_dir, timestamp_slug


PROFILE_DEFAULTS = {
    "centered": {"x": 320.0, "width": 120.0, "yaw_limit": 300.0, "forward_limit": 250.0},
    "yaw-positive": {"x": 340.0, "width": 120.0, "yaw_limit": 300.0, "forward_limit": 0.0},
    "yaw-negative": {"x": 300.0, "width": 120.0, "yaw_limit": 300.0, "forward_limit": 0.0},
    "pitch-positive": {"x": 320.0, "width": 80.0, "yaw_limit": 0.0, "forward_limit": 250.0},
    "combined": {"x": 340.0, "width": 80.0, "yaw_limit": 300.0, "forward_limit": 250.0},
    "yaw-micro-positive": {
        "x": 335.0, "width": 120.0, "yaw_limit": 12.0, "forward_limit": 0.0,
        "yaw_ki": 0.0, "yaw_kd": 0.0, "forward_ki": 0.0, "forward_kd": 0.0,
    },
    "yaw-micro-negative": {
        "x": 305.0, "width": 120.0, "yaw_limit": 12.0, "forward_limit": 0.0,
        "yaw_ki": 0.0, "yaw_kd": 0.0, "forward_ki": 0.0, "forward_kd": 0.0,
    },
    "pitch-micro-positive": {
        "x": 320.0, "width": 95.0, "yaw_limit": 0.0, "forward_limit": 10.0,
        "yaw_ki": 0.0, "yaw_kd": 0.0, "forward_ki": 0.0, "forward_kd": 0.0,
    },
    "pitch-micro-negative": {
        "x": 320.0, "width": 145.0, "yaw_limit": 0.0, "forward_limit": 10.0,
        "yaw_ki": 0.0, "yaw_kd": 0.0, "forward_ki": 0.0, "forward_kd": 0.0,
    },
    "combined-micro": {
        "x": 335.0, "width": 95.0, "yaw_limit": 12.0, "forward_limit": 10.0,
        "yaw_ki": 0.0, "yaw_kd": 0.0, "forward_ki": 0.0, "forward_kd": 0.0,
    },
}

CONTROL_DEFAULTS = {
    "yaw_kp": 0.8,
    "yaw_ki": 0.05,
    "yaw_kd": 0.15,
    "forward_kp": 0.4,
    "forward_ki": 0.02,
    "forward_kd": 0.1,
}


def apply_profile_defaults(args: argparse.Namespace) -> argparse.Namespace:
    defaults = PROFILE_DEFAULTS[args.profile]
    if args.synthetic_target_x is None:
        args.synthetic_target_x = defaults["x"]
    if args.synthetic_target_width is None:
        args.synthetic_target_width = defaults["width"]
    if args.synthetic_target_height is None:
        args.synthetic_target_height = args.synthetic_target_width
    if args.yaw_limit is None:
        args.yaw_limit = defaults["yaw_limit"]
    if args.forward_limit is None:
        args.forward_limit = defaults["forward_limit"]
    for name, value in CONTROL_DEFAULTS.items():
        if getattr(args, name) is None:
            setattr(args, name, defaults.get(name, value))
    return args


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def estimated_command(args: argparse.Namespace) -> dict[str, float]:
    yaw_error = float(args.synthetic_target_x) - float(args.frame_width) / 2.0
    if abs(yaw_error) < args.yaw_deadband:
        yaw_error = 0.0
    forward_error = float(args.desired_target_width) - float(args.synthetic_target_width)
    if abs(forward_error) < args.size_deadband:
        forward_error = 0.0
    yaw_delta = clamp(args.yaw_kp * yaw_error, -args.yaw_limit, args.yaw_limit)
    pitch_delta = clamp(args.forward_kp * forward_error, -args.forward_limit, args.forward_limit)
    return {
        "yaw_error": yaw_error,
        "forward_error": forward_error,
        "yaw_delta": yaw_delta,
        "pitch_delta": pitch_delta,
    }


def iter_jsonl(path: Path):
    with path.open("r", encoding="utf-8") as handle:
        for line in handle:
            line = line.strip()
            if line:
                yield json.loads(line)


def build_checker_command(args: argparse.Namespace, diagnostics_log: Path) -> list[str]:
    command = [
        sys.executable,
        str(SCRIPT_DIR / "sitl_virtual_takeoff_check.py"),
        "--rc-driver", "external",
        "--throttle", str(args.throttle),
        "--mode-pwm", str(args.mode_pwm),
        "--diagnostic-samples", str(args.diagnostic_samples),
        "--diagnostic-interval", str(args.diagnostic_interval),
        "--gazebo-timeout", str(args.gazebo_timeout),
        "--diagnostics-timeout", str(args.diagnostics_timeout),
        "--max-abs-attitude", str(args.max_abs_attitude),
        "--min-altitude-gain", str(args.min_altitude_gain),
        "--diagnostics-log-file", str(diagnostics_log),
    ]
    if args.no_fix_iris_imu_pose:
        command.append("--no-fix-iris-imu-pose")
    if args.no_fix_iris_motor_map:
        command.append("--no-fix-iris-motor-map")
    if args.safe_yaw_authority:
        command.append("--safe-yaw-authority")
    if args.yaw_pid is not None:
        command.extend(["--yaw-pid", args.yaw_pid])
    if args.pitch_pid is not None:
        command.extend(["--pitch-pid", args.pitch_pid])
    if args.pitch_rc_rate is not None:
        command.extend(["--pitch-rc-rate", str(args.pitch_rc_rate)])
    if args.pitch_rate is not None:
        command.extend(["--pitch-rate", str(args.pitch_rate)])
    if args.pitch_rate_limit is not None:
        command.extend(["--pitch-rate-limit", str(args.pitch_rate_limit)])
    return command


def build_mixer_command(args: argparse.Namespace, mixer_log: Path) -> list[str]:
    duration = args.mixer_duration
    if duration is None:
        duration = (
            args.virtual_low_seconds
            + args.virtual_arm_seconds
            + args.virtual_ramp_seconds
            + args.virtual_hold_seconds
            + args.virtual_disarm_seconds
        )
    command = [
        sys.executable,
        str(SCRIPT_DIR / "kenet_sitl_mixer.py"),
        "--pilot-source", "virtual",
        "--virtual-script", "takeoff",
        "--virtual-throttle", str(args.throttle),
        "--virtual-kenet-pwm", str(args.kenet_pwm),
        "--virtual-mode-pwm", str(args.mode_pwm),
        "--virtual-low-seconds", str(args.virtual_low_seconds),
        "--virtual-arm-seconds", str(args.virtual_arm_seconds),
        "--virtual-ramp-seconds", str(args.virtual_ramp_seconds),
        "--virtual-hold-seconds", str(args.virtual_hold_seconds),
        "--virtual-disarm-seconds", str(args.virtual_disarm_seconds),
        "--synthetic-target",
        "--frame-width", str(args.frame_width),
        "--frame-height", str(args.frame_height),
        "--desired-target-width", str(args.desired_target_width),
        "--synthetic-target-x", str(args.synthetic_target_x),
        "--synthetic-target-y", str(args.synthetic_target_y),
        "--synthetic-target-width", str(args.synthetic_target_width),
        "--synthetic-target-height", str(args.synthetic_target_height),
        "--synthetic-target-delay-seconds", str(args.synthetic_target_delay_seconds),
        "--lost-seconds", str(args.lost_seconds),
        "--yaw-kp", str(args.yaw_kp),
        "--yaw-ki", str(args.yaw_ki),
        "--yaw-kd", str(args.yaw_kd),
        "--yaw-limit", str(args.yaw_limit),
        "--forward-kp", str(args.forward_kp),
        "--forward-ki", str(args.forward_ki),
        "--forward-kd", str(args.forward_kd),
        "--forward-limit", str(args.forward_limit),
        "--send",
        "--duration", str(duration),
        "--print-hz", str(args.mixer_print_hz),
        "--flight-log", str(mixer_log),
    ]
    if args.synthetic_target_loss_after_seconds is not None:
        command.extend([
            "--synthetic-target-loss-after-seconds",
            str(args.synthetic_target_loss_after_seconds),
        ])
    return command


def communicate_or_terminate(proc: subprocess.Popen[str], timeout: float, label: str) -> tuple[int, str]:
    try:
        output, _ = proc.communicate(timeout=timeout)
    except subprocess.TimeoutExpired:
        terminate_process(proc)
        output, _ = proc.communicate(timeout=5)
        return 124, "%s timed out\n%s" % (label, output or "")
    return proc.returncode or 0, output or ""


def wait_or_terminate(proc: subprocess.Popen[str], timeout: float, label: str) -> int:
    try:
        proc.wait(timeout=timeout)
    except subprocess.TimeoutExpired:
        terminate_process(proc)
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            pass
        return 124
    return proc.returncode or 0


def read_process_output(path: Path, max_chars: int = 12000) -> str:
    if not path.exists():
        return ""
    output = path.read_text(encoding="utf-8", errors="replace")
    if len(output) <= max_chars:
        return output
    return "... output truncated; full log: %s ...\n%s" % (path, output[-max_chars:])


def empty_mixer_summary() -> dict[str, Any]:
    return {
        "samples": 0,
        "tracking_samples": 0,
        "target_found_samples": 0,
        "target_lost_samples": 0,
        "kenet_source_samples": 0,
        "target_lost_source_samples": 0,
        "ai_armed_samples": 0,
        "max_abs_delta": 0,
        "max_abs_pitch_delta": 0,
        "max_abs_yaw_delta": 0,
        "tracker_errors": [],
    }


def summarize_mixer_log(path: Path) -> dict[str, Any]:
    summary = empty_mixer_summary()
    for record in iter_jsonl(path):
        if record.get("event") != "kenet_mixer_sample":
            continue
        summary["samples"] += 1
        state = record.get("state")
        source = record.get("source")
        target_found = bool(record.get("target_found"))
        if state == "TRACKING":
            summary["tracking_samples"] += 1
        if state == "AI-ARMED":
            summary["ai_armed_samples"] += 1
        if target_found:
            summary["target_found_samples"] += 1
        else:
            summary["target_lost_samples"] += 1
        if source == "kenet":
            summary["kenet_source_samples"] += 1
        if source == "pilot-target-lost":
            summary["target_lost_source_samples"] += 1
        delta = ((record.get("first8") or {}).get("delta") or [])[:8]
        if delta:
            summary["max_abs_delta"] = max(summary["max_abs_delta"], max(abs(int(value)) for value in delta))
            if len(delta) > 1:
                summary["max_abs_pitch_delta"] = max(summary["max_abs_pitch_delta"], abs(int(delta[1])))
            if len(delta) > 3:
                summary["max_abs_yaw_delta"] = max(summary["max_abs_yaw_delta"], abs(int(delta[3])))
        tracker_error = record.get("tracker_error")
        if tracker_error and tracker_error not in summary["tracker_errors"]:
            summary["tracker_errors"].append(tracker_error)
    return summary


def validate_mixer_summary(summary: dict[str, Any], args: argparse.Namespace) -> list[str]:
    failures: list[str] = []
    if summary["samples"] <= 0:
        failures.append("mixer log sample yok")
    if summary["tracking_samples"] < args.min_tracking_samples:
        failures.append("TRACKING sample sayisi yetersiz: %s" % summary["tracking_samples"])
    if summary["target_found_samples"] < args.min_target_found_samples:
        failures.append("target_found sample sayisi yetersiz: %s" % summary["target_found_samples"])
    if summary["kenet_source_samples"] < args.min_kenet_source_samples:
        failures.append("source=kenet sample sayisi yetersiz: %s" % summary["kenet_source_samples"])
    if summary["target_lost_source_samples"] < args.min_target_lost_samples:
        failures.append("source=pilot-target-lost sample sayisi yetersiz: %s" % summary["target_lost_source_samples"])
    if summary["ai_armed_samples"] < args.min_ai_armed_samples:
        failures.append("AI-ARMED sample sayisi yetersiz: %s" % summary["ai_armed_samples"])
    if summary["tracker_errors"]:
        failures.append("tracker error: %s" % "; ".join(summary["tracker_errors"]))
    if args.max_abs_delta is not None and summary["max_abs_delta"] > args.max_abs_delta:
        failures.append("mixer delta limiti asildi: %s > %s" % (summary["max_abs_delta"], args.max_abs_delta))
    return failures


def main() -> int:
    args = apply_profile_defaults(parse_args())
    log_dir = resolve_log_dir(args.log_dir, REPO_ROOT)
    log_dir.mkdir(parents=True, exist_ok=True)
    run_id = args.run_id or ("%s-synthetic-%s" % (timestamp_slug(), args.profile))
    diagnostics_log = log_dir / ("%s-diagnostics.jsonl" % run_id)
    mixer_log = log_dir / ("%s-mixer.jsonl" % run_id)
    checker_stdout_log = log_dir / ("%s-checker.stdout.log" % run_id)
    mixer_stdout_log = log_dir / ("%s-mixer.stdout.log" % run_id)

    checker_command = build_checker_command(args, diagnostics_log)
    mixer_command = build_mixer_command(args, mixer_log)

    print("diagnostics_log=%s" % diagnostics_log)
    print("mixer_log=%s" % mixer_log)
    print("profile=%s target_x=%s target_width=%s delay=%s" % (
        args.profile,
        args.synthetic_target_x,
        args.synthetic_target_width,
        args.synthetic_target_delay_seconds,
    ))
    estimate = estimated_command(args)
    print("estimated_errors yaw_px=%.1f forward_px=%.1f" % (
        estimate["yaw_error"],
        estimate["forward_error"],
    ))
    print("estimated_deltas pitch_pwm=%+.1f yaw_pwm=%+.1f limits pitch/yaw=%.1f/%.1f" % (
        estimate["pitch_delta"],
        estimate["yaw_delta"],
        args.forward_limit,
        args.yaw_limit,
    ))

    checker = subprocess.Popen(
        checker_command,
        cwd=str(REPO_ROOT),
        stdout=checker_stdout_log.open("w", encoding="utf-8"),
        stderr=subprocess.STDOUT,
        text=True,
    )
    mixer = None
    try:
        time.sleep(args.mixer_start_delay)
        mixer = subprocess.Popen(
            mixer_command,
            cwd=str(REPO_ROOT),
            stdout=mixer_stdout_log.open("w", encoding="utf-8"),
            stderr=subprocess.STDOUT,
            text=True,
        )
        checker_rc = wait_or_terminate(checker, args.checker_timeout, "checker")
        mixer_terminated_after_checker = False
        if mixer.poll() is None:
            terminate_process(mixer)
            mixer_terminated_after_checker = True
            mixer_rc = 0
        else:
            mixer_rc = mixer.returncode or 0
    finally:
        if mixer is not None and mixer.poll() is None:
            terminate_process(mixer)
        if checker.poll() is None:
            terminate_process(checker)

    mixer_output = read_process_output(mixer_stdout_log)
    if mixer_output:
        print(mixer_output.rstrip())
    checker_output = read_process_output(checker_stdout_log)
    if checker_output:
        print(checker_output.rstrip())

    summary = summarize_mixer_log(mixer_log) if mixer_log.exists() else empty_mixer_summary()
    print("mixer_summary=%s" % json.dumps(summary, sort_keys=True))
    failures = validate_mixer_summary(summary, args)
    if mixer_rc != 0:
        failures.append("mixer_rc=%d" % mixer_rc)
    if 'mixer_terminated_after_checker' in locals() and mixer_terminated_after_checker:
        print("mixer stopped after checker completion")
    if checker_rc != 0:
        failures.append("checker_rc=%d" % checker_rc)
    if failures:
        print("result=FAIL %s" % "; ".join(failures))
        return 0 if args.allow_fail else 1
    print("result=PASS")
    if args.allow_fail:
        return 0
    return 0


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--profile", choices=sorted(PROFILE_DEFAULTS), default="combined")
    parser.add_argument("--run-id", default=None)
    parser.add_argument("--log-dir", default=None)
    parser.add_argument("--allow-fail", action="store_true",
                        help="Return success if the run completed, even when the acceptance gate fails")

    parser.add_argument("--throttle", type=int, default=1750)
    parser.add_argument("--mode-pwm", type=int, default=1500)
    parser.add_argument("--kenet-pwm", type=int, default=2000)
    parser.add_argument("--virtual-low-seconds", type=float, default=7.0)
    parser.add_argument("--virtual-arm-seconds", type=float, default=3.0)
    parser.add_argument("--virtual-ramp-seconds", type=float, default=3.0)
    parser.add_argument("--virtual-hold-seconds", type=float, default=30.0)
    parser.add_argument("--virtual-disarm-seconds", type=float, default=1.0)
    parser.add_argument("--mixer-duration", type=float, default=None)
    parser.add_argument("--mixer-start-delay", type=float, default=8.0)
    parser.add_argument("--mixer-timeout", type=float, default=90.0)
    parser.add_argument("--mixer-print-hz", type=float, default=1.0)

    parser.add_argument("--synthetic-target-x", type=float, default=None)
    parser.add_argument("--synthetic-target-y", type=float, default=240.0)
    parser.add_argument("--synthetic-target-width", type=float, default=None)
    parser.add_argument("--synthetic-target-height", type=float, default=None)
    parser.add_argument("--synthetic-target-delay-seconds", type=float, default=18.0)
    parser.add_argument("--synthetic-target-loss-after-seconds", type=float, default=None)
    parser.add_argument("--frame-width", type=int, default=640)
    parser.add_argument("--frame-height", type=int, default=480)
    parser.add_argument("--desired-target-width", type=float, default=120.0)
    parser.add_argument("--yaw-deadband", type=float, default=10.0)
    parser.add_argument("--size-deadband", type=float, default=15.0)
    parser.add_argument("--yaw-kp", type=float, default=None)
    parser.add_argument("--yaw-ki", type=float, default=None)
    parser.add_argument("--yaw-kd", type=float, default=None)
    parser.add_argument("--yaw-limit", type=float, default=None)
    parser.add_argument("--forward-kp", type=float, default=None)
    parser.add_argument("--forward-ki", type=float, default=None)
    parser.add_argument("--forward-kd", type=float, default=None)
    parser.add_argument("--forward-limit", type=float, default=None)
    parser.add_argument("--lost-seconds", type=float, default=2.0)

    parser.add_argument("--diagnostic-samples", type=int, default=120)
    parser.add_argument("--diagnostic-interval", type=float, default=0.1)
    parser.add_argument("--gazebo-timeout", type=float, default=0.25)
    parser.add_argument("--diagnostics-timeout", type=float, default=240.0)
    parser.add_argument("--checker-timeout", type=float, default=300.0)
    parser.add_argument("--max-abs-attitude", type=float, default=35.0)
    parser.add_argument("--min-altitude-gain", type=float, default=1.0)
    parser.add_argument("--safe-yaw-authority", action="store_true", default=True)
    parser.add_argument("--no-safe-yaw-authority", action="store_false", dest="safe_yaw_authority")
    parser.add_argument("--yaw-pid", default=None,
                        help="Optional Betaflight yaw PID triplet passed to the external checker, e.g. 23,0,0")
    parser.add_argument("--pitch-pid", default=None,
                        help="Optional Betaflight pitch PID triplet passed to the external checker, e.g. 23,0,0")
    parser.add_argument("--pitch-rc-rate", type=int, default=None,
                        help="Optional Betaflight pitch_rc_rate passed to the external checker")
    parser.add_argument("--pitch-rate", type=int, default=None,
                        help="Optional Betaflight pitch super-rate passed to the external checker")
    parser.add_argument("--pitch-rate-limit", type=int, default=None,
                        help="Optional Betaflight pitch rate_limit passed to the external checker")
    parser.add_argument("--no-fix-iris-imu-pose", action="store_true")
    parser.add_argument("--no-fix-iris-motor-map", action="store_true")
    parser.add_argument("--min-tracking-samples", type=int, default=5)
    parser.add_argument("--min-target-found-samples", type=int, default=5)
    parser.add_argument("--min-kenet-source-samples", type=int, default=5)
    parser.add_argument("--min-target-lost-samples", type=int, default=0)
    parser.add_argument("--min-ai-armed-samples", type=int, default=0)
    parser.add_argument("--max-abs-delta", type=int, default=None,
                        help="Optional mixer delta limit; use 0 for centered no-command acceptance")

    args = parser.parse_args(argv)
    if not 1000 <= args.throttle <= 2000:
        parser.error("--throttle must be between 1000 and 2000")
    if not 1000 <= args.mode_pwm <= 2000:
        parser.error("--mode-pwm must be between 1000 and 2000")
    if not 1000 <= args.kenet_pwm <= 2000:
        parser.error("--kenet-pwm must be between 1000 and 2000")
    for name in (
        "virtual_low_seconds",
        "virtual_arm_seconds",
        "virtual_ramp_seconds",
        "virtual_hold_seconds",
        "virtual_disarm_seconds",
        "mixer_start_delay",
        "synthetic_target_delay_seconds",
    ):
        if getattr(args, name) < 0:
            parser.error("--%s must be non-negative" % name.replace("_", "-"))
    if args.synthetic_target_loss_after_seconds is not None and args.synthetic_target_loss_after_seconds < 0:
        parser.error("--synthetic-target-loss-after-seconds must be non-negative")
    for name in ("diagnostic_samples",):
        if getattr(args, name) <= 0:
            parser.error("--%s must be positive" % name.replace("_", "-"))
    for name in (
        "frame_width",
        "frame_height",
        "desired_target_width",
        "yaw_deadband",
        "size_deadband",
        "lost_seconds",
    ):
        if getattr(args, name) <= 0:
            parser.error("--%s must be positive" % name.replace("_", "-"))
    for name in ("yaw_kp", "forward_kp"):
        value = getattr(args, name)
        if value is not None and value <= 0:
            parser.error("--%s must be positive" % name.replace("_", "-"))
    for name in ("yaw_ki", "yaw_kd", "forward_ki", "forward_kd"):
        value = getattr(args, name)
        if value is not None and value < 0:
            parser.error("--%s must be non-negative" % name.replace("_", "-"))
    for name in ("yaw_limit", "forward_limit"):
        value = getattr(args, name)
        if value is not None and value < 0:
            parser.error("--%s must be non-negative" % name.replace("_", "-"))
    for name in (
        "min_tracking_samples",
        "min_target_found_samples",
        "min_kenet_source_samples",
        "min_target_lost_samples",
        "min_ai_armed_samples",
    ):
        if getattr(args, name) < 0:
            parser.error("--%s must be non-negative" % name.replace("_", "-"))
    if args.max_abs_delta is not None and args.max_abs_delta < 0:
        parser.error("--max-abs-delta must be non-negative")
    for name in ("pitch_rc_rate", "pitch_rate"):
        value = getattr(args, name)
        if value is not None and not 0 <= value <= 255:
            parser.error("--%s must be 0..255" % name.replace("_", "-"))
    if args.pitch_rate_limit is not None and not 0 <= args.pitch_rate_limit <= 65535:
        parser.error("--pitch-rate-limit must be 0..65535")
    return args


if __name__ == "__main__":
    raise SystemExit(main())
