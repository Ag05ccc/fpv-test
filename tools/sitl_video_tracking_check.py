#!/usr/bin/env python3
"""Run Gazebo takeoff acceptance with Kenet mixer tracking a real camera/video."""

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


def iter_jsonl(path: Path):
    with path.open("r", encoding="utf-8") as handle:
        for line in handle:
            line = line.strip()
            if line:
                yield json.loads(line)


def build_checker_command(args: argparse.Namespace, diagnostics_log: Path, motor_udp_log: Path | None) -> list[str]:
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
    if args.max_step_size is not None:
        command.extend(["--max-step-size", args.max_step_size])
    if args.sync_betaflight_looptime:
        command.append("--sync-betaflight-looptime")
    if args.iris_rotor_damping is not None:
        command.extend(["--iris-rotor-damping", str(args.iris_rotor_damping)])
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
    if args.yaw_rc_rate is not None:
        command.extend(["--yaw-rc-rate", str(args.yaw_rc_rate)])
    if args.yaw_rate is not None:
        command.extend(["--yaw-rate", str(args.yaw_rate)])
    if args.yaw_rate_limit is not None:
        command.extend(["--yaw-rate-limit", str(args.yaw_rate_limit)])
    if args.betaflight_config_file is not None:
        command.extend(["--betaflight-config-file", args.betaflight_config_file])
    if args.capture_motor_udp:
        command.append("--capture-motor-udp")
        if motor_udp_log is not None:
            command.extend(["--motor-udp-log-file", str(motor_udp_log)])
    if args.no_fix_iris_imu_pose:
        command.append("--no-fix-iris-imu-pose")
    if args.no_fix_iris_motor_map:
        command.append("--no-fix-iris-motor-map")
    if args.world is not None:
        command.extend(["--world", args.world])
    if args.iris_forward_camera:
        command.append("--iris-forward-camera")
    if getattr(args, "gazebo_gui", False):
        command.append("--gazebo-gui")
        if getattr(args, "gui_config", None):
            command.extend(["--gui-config", args.gui_config])
    return command


def mixer_duration(args: argparse.Namespace) -> float:
    if args.mixer_duration is not None:
        return args.mixer_duration
    return (
        args.virtual_low_seconds
        + args.virtual_arm_seconds
        + args.virtual_ramp_seconds
        + args.virtual_hold_seconds
        + args.virtual_disarm_seconds
    )


def build_mixer_command(args: argparse.Namespace, mixer_log: Path) -> list[str]:
    command = [
        sys.executable,
        str(SCRIPT_DIR / "kenet_sitl_mixer.py"),
        "--pilot-source", "virtual",
        "--virtual-script", "takeoff",
        "--virtual-throttle", str(args.throttle),
        "--virtual-kenet-pwm", str(args.kenet_pwm),
        "--virtual-kenet-pre-pwm", str(args.kenet_pre_pwm),
        "--virtual-kenet-delay-seconds", str(args.kenet_delay_seconds),
        "--virtual-mode-pwm", str(args.mode_pwm),
        "--virtual-low-seconds", str(args.virtual_low_seconds),
        "--virtual-arm-seconds", str(args.virtual_arm_seconds),
        "--virtual-ramp-seconds", str(args.virtual_ramp_seconds),
        "--virtual-hold-seconds", str(args.virtual_hold_seconds),
        "--virtual-disarm-seconds", str(args.virtual_disarm_seconds),
        "--camera", str(args.camera),
        "--tracker", args.tracker,
        "--track-size", str(args.track_size),
        "--desired-target-width", str(args.desired_target_width),
        "--yaw-kp", str(args.yaw_kp),
        "--yaw-ki", str(args.yaw_ki),
        "--yaw-kd", str(args.yaw_kd),
        "--yaw-limit", str(args.yaw_limit),
        "--forward-kp", str(args.forward_kp),
        "--forward-ki", str(args.forward_ki),
        "--forward-kd", str(args.forward_kd),
        "--forward-limit", str(args.forward_limit),
        "--send",
        "--duration", str(mixer_duration(args)),
        "--print-hz", str(args.mixer_print_hz),
        "--flight-log", str(mixer_log),
    ]
    if args.target_loss_after_seconds is not None:
        command.extend(["--target-loss-after-seconds", str(args.target_loss_after_seconds)])
    return command


def communicate_or_terminate(proc: subprocess.Popen[str], timeout: float, label: str) -> tuple[int, str]:
    try:
        output, _ = proc.communicate(timeout=timeout)
    except subprocess.TimeoutExpired:
        terminate_process(proc)
        output, _ = proc.communicate(timeout=5)
        return 124, "%s timed out\n%s" % (label, output or "")
    return proc.returncode or 0, output or ""


def summarize_mixer_log(path: Path) -> dict[str, Any]:
    summary: dict[str, Any] = {
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
        "frame_shapes": set(),
    }
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
        frame_shape = record.get("frame_shape")
        if frame_shape:
            summary["frame_shapes"].add(tuple(frame_shape))
    summary["frame_shapes"] = [list(shape) for shape in sorted(summary["frame_shapes"])]
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
    if args.min_abs_pitch_delta is not None and summary["max_abs_pitch_delta"] < args.min_abs_pitch_delta:
        failures.append(
            "pitch delta yetersiz: %s < %s" % (summary["max_abs_pitch_delta"], args.min_abs_pitch_delta)
        )
    if args.min_abs_yaw_delta is not None and summary["max_abs_yaw_delta"] < args.min_abs_yaw_delta:
        failures.append("yaw delta yetersiz: %s < %s" % (summary["max_abs_yaw_delta"], args.min_abs_yaw_delta))
    if args.max_abs_pitch_delta is not None and summary["max_abs_pitch_delta"] > args.max_abs_pitch_delta:
        failures.append(
            "pitch delta limiti asildi: %s > %s" % (summary["max_abs_pitch_delta"], args.max_abs_pitch_delta)
        )
    if args.max_abs_yaw_delta is not None and summary["max_abs_yaw_delta"] > args.max_abs_yaw_delta:
        failures.append("yaw delta limiti asildi: %s > %s" % (summary["max_abs_yaw_delta"], args.max_abs_yaw_delta))
    return failures


def main() -> int:
    args = parse_args()
    log_dir = resolve_log_dir(args.log_dir, REPO_ROOT)
    log_dir.mkdir(parents=True, exist_ok=True)
    run_id = args.run_id or ("%s-video-tracking" % timestamp_slug())
    diagnostics_log = log_dir / ("%s-diagnostics.jsonl" % run_id)
    mixer_log = log_dir / ("%s-mixer.jsonl" % run_id)
    motor_udp_log = log_dir / ("%s-motor-udp.jsonl" % run_id) if args.capture_motor_udp else None

    checker_command = build_checker_command(args, diagnostics_log, motor_udp_log)
    mixer_command = build_mixer_command(args, mixer_log)

    print("diagnostics_log=%s" % diagnostics_log)
    print("mixer_log=%s" % mixer_log)
    if motor_udp_log is not None:
        print("motor_udp_log=%s" % motor_udp_log)
    print("camera=%s tracker=%s" % (args.camera, args.tracker))
    print("control_limits pitch/yaw=%.1f/%.1f" % (args.forward_limit, args.yaw_limit))

    checker = subprocess.Popen(
        checker_command,
        cwd=str(REPO_ROOT),
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    )
    try:
        time.sleep(args.mixer_start_delay)
        mixer = subprocess.Popen(
            mixer_command,
            cwd=str(REPO_ROOT),
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
        )
        mixer_rc, mixer_output = communicate_or_terminate(mixer, args.mixer_timeout, "mixer")
        if mixer_output:
            print(mixer_output.rstrip())
        checker_rc, checker_output = communicate_or_terminate(checker, args.checker_timeout, "checker")
        if checker_output:
            print(checker_output.rstrip())
    finally:
        if checker.poll() is None:
            terminate_process(checker)

    summary = summarize_mixer_log(mixer_log) if mixer_log.exists() else {"samples": 0}
    print("mixer_summary=%s" % json.dumps(summary, sort_keys=True))
    failures = validate_mixer_summary(summary, args)

    if mixer_rc != 0:
        failures.append("mixer_rc=%d" % mixer_rc)
    if checker_rc != 0:
        failures.append("checker_rc=%d" % checker_rc)
    if failures:
        print("result=FAIL %s" % "; ".join(failures))
        return 0 if args.allow_fail else 1
    print("result=PASS")
    return 0


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-id", default=None)
    parser.add_argument("--log-dir", default=None)
    parser.add_argument("--allow-fail", action="store_true",
                        help="Return success if the run completed, even when the acceptance gate fails")

    parser.add_argument("--camera", default="test-2.mp4")
    parser.add_argument("--tracker", default="CSRT", choices=["CSRT", "KCF"])
    parser.add_argument("--throttle", type=int, default=1750)
    parser.add_argument("--mode-pwm", type=int, default=1500)
    parser.add_argument("--kenet-pwm", type=int, default=2000)
    parser.add_argument("--kenet-pre-pwm", type=int, default=1000)
    parser.add_argument("--kenet-delay-seconds", type=float, default=18.0,
                        help="Delay TRACKING state so video commands arrive after takeoff settles")
    parser.add_argument("--virtual-low-seconds", type=float, default=7.0)
    parser.add_argument("--virtual-arm-seconds", type=float, default=3.0)
    parser.add_argument("--virtual-ramp-seconds", type=float, default=3.0)
    parser.add_argument("--virtual-hold-seconds", type=float, default=30.0)
    parser.add_argument("--virtual-disarm-seconds", type=float, default=1.0)
    parser.add_argument("--mixer-duration", type=float, default=None)
    parser.add_argument("--mixer-start-delay", type=float, default=8.0)
    parser.add_argument("--mixer-timeout", type=float, default=90.0)
    parser.add_argument("--mixer-print-hz", type=float, default=1.0)
    parser.add_argument("--target-loss-after-seconds", type=float, default=None,
                        help="Force real camera/video target-loss after this virtual-pilot elapsed time")

    parser.add_argument("--track-size", type=int, default=100)
    parser.add_argument("--desired-target-width", type=float, default=120.0)
    parser.add_argument("--yaw-kp", type=float, default=0.8)
    parser.add_argument("--yaw-ki", type=float, default=0.05)
    parser.add_argument("--yaw-kd", type=float, default=0.15)
    parser.add_argument("--yaw-limit", type=float, default=0.0,
                        help="Default 0 keeps this first real-video gate neutral; raise for command-response tests.")
    parser.add_argument("--forward-kp", type=float, default=0.4)
    parser.add_argument("--forward-ki", type=float, default=0.02)
    parser.add_argument("--forward-kd", type=float, default=0.1)
    parser.add_argument("--forward-limit", type=float, default=0.0,
                        help="Default 0 keeps this first real-video gate neutral; raise for command-response tests.")

    parser.add_argument("--diagnostic-samples", type=int, default=120)
    parser.add_argument("--diagnostic-interval", type=float, default=0.1)
    parser.add_argument("--gazebo-timeout", type=float, default=0.25)
    parser.add_argument("--diagnostics-timeout", type=float, default=240.0)
    parser.add_argument("--checker-timeout", type=float, default=300.0)
    parser.add_argument("--max-abs-attitude", type=float, default=35.0)
    parser.add_argument("--min-altitude-gain", type=float, default=1.0)
    parser.add_argument("--max-step-size", default=None,
                        help="Optional Gazebo physics step passed to the external checker, e.g. 0.001 or 0.0025. "
                             "Default keeps the external checker's own default; command-response brackets are "
                             "step-dependent, so record the effective step with the evidence.")
    parser.add_argument("--sync-betaflight-looptime", action="store_true",
                        help="Pass --sync-betaflight-looptime to the external checker (requires the "
                             "looptime-sync patched Betaflight SITL build).")
    parser.add_argument("--iris-rotor-damping", type=float, default=None,
                        help="Pass --iris-rotor-damping to the external checker (temporary Iris rotor "
                             "joint damping override).")
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
    parser.add_argument("--yaw-rc-rate", type=int, default=None,
                        help="Optional Betaflight yaw_rc_rate passed to the external checker "
                             "(overrides the --safe-yaw-authority value for that field)")
    parser.add_argument("--yaw-rate", type=int, default=None,
                        help="Optional Betaflight yaw super-rate passed to the external checker")
    parser.add_argument("--yaw-rate-limit", type=int, default=None,
                        help="Optional Betaflight yaw rate_limit passed to the external checker")
    parser.add_argument("--betaflight-config-file", default=None,
                        help="Optional Betaflight CLI config file imported into the SITL eeprom "
                             "before the run (passed to the external checker); rate/PID flags "
                             "still apply over MSP afterwards")
    parser.add_argument("--capture-motor-udp", action="store_true", default=True)
    parser.add_argument("--no-capture-motor-udp", action="store_false", dest="capture_motor_udp")
    parser.add_argument("--no-fix-iris-imu-pose", action="store_true")
    parser.add_argument("--no-fix-iris-motor-map", action="store_true")
    parser.add_argument("--world", default=None,
                        help="World name/path forwarded to the external checker "
                             "(e.g. betaloop_iris_betaflight_demo_populated.sdf)")
    parser.add_argument("--iris-forward-camera", action="store_true",
                        help="Inject the forward FPV camera into the Iris and let "
                             "the mixer consume it live via --camera gz:/kenet/fpv_camera")
    parser.add_argument("--gazebo-gui", action="store_true",
                        help="Watch the run in the Gazebo GUI instead of headless")
    parser.add_argument("--gui-config", default=None,
                        help="Gazebo GUI config (e.g. tools/fpv_gui.config)")

    parser.add_argument("--min-tracking-samples", type=int, default=5)
    parser.add_argument("--min-target-found-samples", type=int, default=5)
    parser.add_argument("--min-kenet-source-samples", type=int, default=5)
    parser.add_argument("--min-target-lost-samples", type=int, default=0)
    parser.add_argument("--min-ai-armed-samples", type=int, default=0)
    parser.add_argument("--max-abs-delta", type=int, default=0,
                        help="Default 0 proves real video target-found without commanding pitch/yaw.")
    parser.add_argument("--min-abs-pitch-delta", type=int, default=None,
                        help="Require the mixer to command at least this pitch delta")
    parser.add_argument("--min-abs-yaw-delta", type=int, default=None,
                        help="Require the mixer to command at least this yaw delta")
    parser.add_argument("--max-abs-pitch-delta", type=int, default=None,
                        help="Optional pitch-axis command ceiling")
    parser.add_argument("--max-abs-yaw-delta", type=int, default=None,
                        help="Optional yaw-axis command ceiling")

    args = parser.parse_args(argv)
    if not 1000 <= args.throttle <= 2000:
        parser.error("--throttle must be between 1000 and 2000")
    if not 1000 <= args.mode_pwm <= 2000:
        parser.error("--mode-pwm must be between 1000 and 2000")
    if not 1000 <= args.kenet_pwm <= 2000:
        parser.error("--kenet-pwm must be between 1000 and 2000")
    if not 1000 <= args.kenet_pre_pwm <= 2000:
        parser.error("--kenet-pre-pwm must be between 1000 and 2000")
    for name in (
        "virtual_low_seconds",
        "virtual_arm_seconds",
        "virtual_ramp_seconds",
        "virtual_hold_seconds",
        "virtual_disarm_seconds",
        "mixer_start_delay",
        "kenet_delay_seconds",
    ):
        if getattr(args, name) < 0:
            parser.error("--%s must be non-negative" % name.replace("_", "-"))
    if args.target_loss_after_seconds is not None and args.target_loss_after_seconds < 0:
        parser.error("--target-loss-after-seconds must be non-negative")
    for name in ("diagnostic_samples", "track_size"):
        if getattr(args, name) <= 0:
            parser.error("--%s must be positive" % name.replace("_", "-"))
    for name in ("desired_target_width", "yaw_kp", "forward_kp"):
        if getattr(args, name) <= 0:
            parser.error("--%s must be positive" % name.replace("_", "-"))
    for name in ("yaw_ki", "yaw_kd", "forward_ki", "forward_kd", "yaw_limit", "forward_limit"):
        if getattr(args, name) < 0:
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
    for name in ("min_abs_pitch_delta", "min_abs_yaw_delta", "max_abs_pitch_delta", "max_abs_yaw_delta"):
        value = getattr(args, name)
        if value is not None and value < 0:
            parser.error("--%s must be non-negative" % name.replace("_", "-"))
    for name in ("pitch_rc_rate", "pitch_rate", "yaw_rc_rate", "yaw_rate"):
        value = getattr(args, name)
        if value is not None and not 0 <= value <= 255:
            parser.error("--%s must be 0..255" % name.replace("_", "-"))
    for name in ("pitch_rate_limit", "yaw_rate_limit"):
        value = getattr(args, name)
        if value is not None and not 0 <= value <= 65535:
            parser.error("--%s must be 0..65535" % name.replace("_", "-"))
    return args


if __name__ == "__main__":
    raise SystemExit(main())
