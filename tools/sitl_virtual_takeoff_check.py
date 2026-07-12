#!/usr/bin/env python3
"""Run a Gazebo/Betaflight takeoff check end-to-end."""

from __future__ import annotations

import argparse
import json
import os
import shutil
import subprocess
import sys
import tempfile
import time
from pathlib import Path
from typing import Any


SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from gazebo_motor_moment_probe import send_motor_speeds, terminate_process
from sitl_debug_config import parse_debug_mode_value
from sitl_log import make_log_path, resolve_log_dir
from sitl_msp import MSP_ADVANCED_CONFIG, MSP_STATUS_EX, msp_request, parse_status_ex


FC_THROTTLE_INDEX = 3
FC_ARM_INDEX = 4
FC_MODE_INDEX = 6
SAFE_YAW_RC_RATE = 5
SAFE_YAW_RATE = 30
SAFE_YAW_RATE_LIMIT = 120
SAFE_MANUAL_RC_RATE = 5
SAFE_MANUAL_RATE = 30
SAFE_MANUAL_RATE_LIMIT = 120


def iter_jsonl(path: Path):
    with path.open("r", encoding="utf-8") as handle:
        for line in handle:
            line = line.strip()
            if line:
                yield json.loads(line)


def numeric(value: Any) -> float | None:
    if isinstance(value, bool):
        return None
    if isinstance(value, (int, float)):
        return float(value)
    return None


def analyze_diagnostics(
    path: Path,
    *,
    target_throttle: int,
    max_abs_attitude: float,
    min_altitude_gain: float,
    require_pose: bool = True,
) -> dict[str, Any]:
    records = [record for record in iter_jsonl(path) if record.get("event") == "diagnostic_sample"]
    z_values: list[float] = []
    roll_values: list[float] = []
    pitch_values: list[float] = []
    motor_spreads: list[float] = []
    msp_connected = 0
    pose_ok = 0
    armed_samples = 0
    armed_angle_samples = 0
    high_throttle_samples = 0
    max_fc_throttle: int | None = None
    max_fc_arm: int | None = None
    max_fc_mode: int | None = None

    for record in records:
        msp = record.get("msp") or {}
        if msp.get("connected"):
            msp_connected += 1
        status = msp.get("status") or {}
        active_modes = set(status.get("active_modes") or [])
        if status.get("armed") or "ARM" in active_modes:
            armed_samples += 1
        if ("ARM" in active_modes or status.get("armed")) and "ANGLE" in active_modes:
            armed_angle_samples += 1

        attitude = msp.get("attitude") or {}
        roll = numeric(attitude.get("roll"))
        pitch = numeric(attitude.get("pitch"))
        if roll is not None:
            roll_values.append(roll)
        if pitch is not None:
            pitch_values.append(pitch)

        motors = [numeric(value) for value in (msp.get("motor") or [])[:4]]
        if len(motors) == 4 and all(value is not None for value in motors):
            motor_spreads.append(max(motors) - min(motors))  # type: ignore[arg-type]

        fc_channels = msp.get("rc_channels") or []
        if len(fc_channels) > FC_THROTTLE_INDEX:
            throttle = int(fc_channels[FC_THROTTLE_INDEX])
            max_fc_throttle = throttle if max_fc_throttle is None else max(max_fc_throttle, throttle)
            if throttle >= target_throttle - 50:
                high_throttle_samples += 1
        if len(fc_channels) > FC_ARM_INDEX:
            arm = int(fc_channels[FC_ARM_INDEX])
            max_fc_arm = arm if max_fc_arm is None else max(max_fc_arm, arm)
        if len(fc_channels) > FC_MODE_INDEX:
            mode = int(fc_channels[FC_MODE_INDEX])
            max_fc_mode = mode if max_fc_mode is None else max(max_fc_mode, mode)

        gazebo_pose = record.get("gazebo_pose") or {}
        if gazebo_pose.get("ok"):
            pose_ok += 1
        position = ((gazebo_pose.get("pose") or {}).get("position") or {})
        z = numeric(position.get("z"))
        if z is not None:
            z_values.append(z)

    max_abs_roll = max((abs(value) for value in roll_values), default=None)
    max_abs_pitch = max((abs(value) for value in pitch_values), default=None)
    max_abs_seen = max(
        value for value in (max_abs_roll, max_abs_pitch)
        if value is not None
    ) if (max_abs_roll is not None or max_abs_pitch is not None) else None
    altitude_gain = (max(z_values) - min(z_values)) if z_values else None
    failures: list[str] = []

    if not records:
        failures.append("diagnostic_sample yok")
    if msp_connected == 0:
        failures.append("MSP baglantisi dogrulanamadi")
    if require_pose and pose_ok == 0:
        failures.append("Gazebo pose/IMU ornekleri yok")
    if armed_angle_samples == 0:
        failures.append("ARM + ANGLE ayni anda gorulmedi")
    if high_throttle_samples == 0:
        failures.append("FC tarafinda hedef throttle gorulmedi")
    if altitude_gain is None or altitude_gain < min_altitude_gain:
        failures.append("irtifa artisi %.2f m altinda" % min_altitude_gain)
    if max_abs_seen is None:
        failures.append("attitude ornegi yok")
    elif max_abs_seen > max_abs_attitude:
        failures.append("roll/pitch %.1f derece ustune cikti" % max_abs_attitude)

    return {
        "path": str(path),
        "samples": len(records),
        "msp_connected_samples": msp_connected,
        "pose_ok_samples": pose_ok,
        "armed_samples": armed_samples,
        "armed_angle_samples": armed_angle_samples,
        "high_throttle_samples": high_throttle_samples,
        "max_fc_throttle": max_fc_throttle,
        "max_fc_arm": max_fc_arm,
        "max_fc_mode": max_fc_mode,
        "min_z": min(z_values) if z_values else None,
        "max_z": max(z_values) if z_values else None,
        "altitude_gain": altitude_gain,
        "max_abs_roll": max_abs_roll,
        "max_abs_pitch": max_abs_pitch,
        "max_motor_spread": max(motor_spreads) if motor_spreads else None,
        "failures": failures,
        "passed": not failures,
    }


def print_check_summary(summary: dict[str, Any]) -> None:
    print("SITL takeoff check: %s" % ("PASS" if summary["passed"] else "FAIL"))
    print("diagnostics log: %s" % summary["path"])
    print("samples: %s, MSP connected: %s, pose ok: %s" % (
        summary["samples"],
        summary["msp_connected_samples"],
        summary["pose_ok_samples"],
    ))
    print("ARM+ANGLE samples: %s, high throttle samples: %s" % (
        summary["armed_angle_samples"],
        summary["high_throttle_samples"],
    ))
    print("altitude: min=%s max=%s gain=%s" % (
        _fmt(summary["min_z"]),
        _fmt(summary["max_z"]),
        _fmt(summary["altitude_gain"]),
    ))
    print("max attitude abs: roll=%s pitch=%s" % (
        _fmt(summary["max_abs_roll"]),
        _fmt(summary["max_abs_pitch"]),
    ))
    print("max motor spread: %s" % _fmt(summary["max_motor_spread"]))
    if summary["failures"]:
        print("failures:")
        for item in summary["failures"]:
            print("  - %s" % item)


def analyze_motor_udp_log(path: Path) -> dict[str, Any]:
    summary = None
    for record in iter_jsonl(path):
        if record.get("event") == "motor_udp_summary":
            summary = record.get("summary") or {}
    if summary is None:
        summary = {}
    result = {"path": str(path)}
    result.update(summary)
    return result


def print_motor_udp_summary(summary: dict[str, Any]) -> None:
    print("motor UDP summary: %s" % summary["path"])
    print("motor UDP samples: %s" % summary.get("samples", 0))
    print("motor UDP max raw spread: %s" % _fmt(summary.get("max_raw_spread")))
    axis = summary.get("max_abs_axis_bias_raw") or {}
    print("motor UDP max abs axis bias raw: roll=%s pitch=%s yaw=%s" % (
        _fmt(axis.get("roll_left_minus_right")),
        _fmt(axis.get("pitch_rear_minus_front")),
        _fmt(axis.get("yaw_cw_minus_ccw")),
    ))
    first = summary.get("first_large_spread") or {}
    if first:
        print("motor UDP first large spread motors: %s" % ",".join(_fmt(value) for value in (first.get("motors_raw") or [])))
        print("motor UDP first large spread packet norm: %s" % ",".join(
            _fmt(value) for value in (first.get("motors_packet_order_normalized") or [])
        ))
        first_axis = first.get("axis_bias_raw") or {}
        print("motor UDP first large spread axis raw: roll=%s pitch=%s yaw=%s" % (
            _fmt(first_axis.get("roll_left_minus_right")),
            _fmt(first_axis.get("pitch_rear_minus_front")),
            _fmt(first_axis.get("yaw_cw_minus_ccw")),
        ))


def _fmt(value: Any) -> str:
    if value is None:
        return "-"
    if isinstance(value, float):
        return "%.3f" % value
    return str(value)


def start_logged_process(command: list[str], log_path: Path, cwd: Path, env: dict[str, str]) -> subprocess.Popen[str]:
    log_handle = log_path.open("w", encoding="utf-8", errors="replace")
    try:
        return subprocess.Popen(
            command,
            cwd=str(cwd),
            env=env,
            stdout=log_handle,
            stderr=subprocess.STDOUT,
            text=True,
            start_new_session=True,
        )
    finally:
        log_handle.close()


def run_checked(command: list[str], *, cwd: Path, timeout: float | None = None) -> subprocess.CompletedProcess[str]:
    proc = subprocess.run(
        command,
        cwd=str(cwd),
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        timeout=timeout,
        check=False,
    )
    if proc.stdout:
        print(proc.stdout.rstrip())
    return proc


def build_gazebo_command(args: argparse.Namespace) -> list[str]:
    command = [
        str(REPO_ROOT / "tools/run_gazebo_betaflight.sh"),
        "--world", args.world,
        "--max-step-size", args.max_step_size,
    ]
    if not args.gazebo_gui:
        command.append("--headless")
    elif getattr(args, "gui_config", None):
        command.extend(["--gui-config", args.gui_config])
    if args.fix_iris_imu_pose:
        command.append("--fix-iris-imu-pose")
    if args.fix_iris_motor_map:
        command.append("--fix-iris-motor-map")
    if args.iris_yaw_gyro_scale is not None:
        command.extend(["--iris-yaw-gyro-scale", str(args.iris_yaw_gyro_scale)])
    if args.iris_rotor_vel_p_gain is not None:
        command.extend(["--iris-rotor-vel-p-gain", str(args.iris_rotor_vel_p_gain)])
    if args.iris_velocity_control:
        command.append("--iris-velocity-control")
    if args.iris_motor_time_constant is not None:
        command.extend(["--iris-motor-time-constant", str(args.iris_motor_time_constant)])
    if args.iris_rotor_damping is not None:
        command.extend(["--iris-rotor-damping", str(args.iris_rotor_damping)])
    if args.iris_forward_camera:
        command.append("--iris-forward-camera")
    return command


def looptime_us_for_step(max_step_size: str) -> str:
    """Map a Gazebo max_step_size (seconds, string) to whole microseconds.

    The patched Betaflight SITL reads KENET_SITL_LOOPTIME_US (valid 100..10000)
    and makes gyro/PID looptime match the lockstep FDM step instead of the
    virtual gyro's claimed 8 kHz, so dT, I/D scaling, and filter inits are
    truthful for the step actually being run.
    """
    step_seconds = float(max_step_size)
    looptime_us = int(round(step_seconds * 1_000_000))
    if not 100 <= looptime_us <= 10000:
        raise RuntimeError(
            "--sync-betaflight-looptime needs a step between 100us and 10000us, got %s" % max_step_size
        )
    return str(looptime_us)


def lockstep_wait_env_value(lockstep_wait_us: int) -> str:
    """Validate --lockstep-wait-us and return the KENET_SITL_LOCKSTEP_WAIT_US value.

    The semaphore-patched Betaflight SITL blocks up to this many microseconds
    for the next FDM packet inside lockMainPID instead of a pure trylock,
    removing the scheduler/FDM phase race at fine steps.
    """
    if not 1 <= lockstep_wait_us <= 1_000_000:
        raise RuntimeError(
            "--lockstep-wait-us must be 1..1000000 microseconds, got %s" % lockstep_wait_us
        )
    return str(lockstep_wait_us)


def betaflight_work_dir(args: argparse.Namespace, process_dir: Path) -> Path:
    if args.betaflight_cwd == "repo":
        return REPO_ROOT
    work_dir = process_dir / "betaflight-cwd"
    work_dir.mkdir(parents=True, exist_ok=True)
    return work_dir


def resolve_betaflight_config_file(value: str) -> Path:
    path = Path(value).expanduser()
    if not path.is_absolute():
        path = REPO_ROOT / path
    return path


def build_betaflight_config_import_command(env: dict[str, str], config_file: Path) -> list[str]:
    betaflight_bin = Path(env["BETAFLIGHT_ROOT"]) / "obj/main/betaflight_SITL.elf"
    return [str(betaflight_bin), "--config", str(config_file)]


def import_betaflight_config(
    args: argparse.Namespace,
    process_dir: Path,
    env: dict[str, str],
    work_dir: Path,
) -> None:
    if not args.betaflight_config_file:
        return
    config_file = resolve_betaflight_config_file(args.betaflight_config_file)
    if not config_file.is_file():
        raise RuntimeError("Betaflight config file not found: %s" % config_file)
    log_path = process_dir / "betaflight-config-import.log"
    command = build_betaflight_config_import_command(env, config_file)
    with log_path.open("w", encoding="utf-8", errors="replace") as log_handle:
        try:
            proc = subprocess.run(
                command,
                cwd=str(work_dir),
                env=env,
                stdout=log_handle,
                stderr=subprocess.STDOUT,
                text=True,
                timeout=args.betaflight_config_import_timeout,
                check=False,
            )
        except subprocess.TimeoutExpired as exc:
            raise RuntimeError("Betaflight config import timed out; see %s" % log_path) from exc
    print("Betaflight config import: %s" % config_file)
    print("Betaflight config import log: %s" % log_path)
    if proc.returncode != 0:
        raise RuntimeError("Betaflight config import failed with rc=%d; see %s" % (proc.returncode, log_path))


def start_betaflight_process(
    args: argparse.Namespace,
    process_dir: Path,
    env: dict[str, str],
    work_dir: Path,
    *,
    log_name: str = "betaflight.log",
    wait_ready: bool = False,
) -> subprocess.Popen[str]:
    betaflight_bin = Path(env["BETAFLIGHT_ROOT"]) / "obj/main/betaflight_SITL.elf"
    proc = start_logged_process([str(betaflight_bin)], process_dir / log_name, work_dir, env)
    time.sleep(args.betaflight_startup_seconds)
    if proc.poll() is not None:
        raise RuntimeError("Betaflight exited early; see %s" % (process_dir / log_name))
    if wait_ready:
        wait_for_msp_ready(args.msp_host, args.msp_port, args.msp_timeout, args.betaflight_ready_timeout, proc)
    return proc


def bootstrap_gazebo_state(args: argparse.Namespace) -> int:
    if args.bootstrap_seconds <= 0:
        return 0
    return send_motor_speeds(
        [0.0, 0.0, 0.0, 0.0],
        args.bootstrap_seconds,
        args.motor_host,
        args.motor_port,
        args.bootstrap_hz,
    )


def wait_for_msp_ready(
    host: str,
    port: int,
    msp_timeout: float,
    ready_timeout: float,
    proc: subprocess.Popen[str] | None = None,
) -> None:
    deadline = time.monotonic() + ready_timeout
    last_error: Exception | None = None
    while time.monotonic() < deadline:
        if proc is not None and proc.poll() is not None:
            raise RuntimeError("Betaflight exited before MSP became ready")
        try:
            msp_request(host, port, MSP_ADVANCED_CONFIG, msp_timeout)
            return
        except Exception as exc:
            last_error = exc
            time.sleep(0.1)
    raise RuntimeError("Betaflight MSP not ready after %.1fs: %s" % (ready_timeout, last_error))


def wait_for_arming_grace(
    host: str,
    port: int,
    msp_timeout: float,
    ready_timeout: float,
) -> None:
    """Block until Betaflight's boot-time arming grace (BOOTGRACE) clears.

    If the ARM channel goes high while BOOTGRACE is still active, Betaflight
    latches ARM_SWITCH and the fixed virtual RC script never re-toggles the
    switch, so the run silently never arms (measured 2026-07-02 with the
    looptime-sync build, whose boot takes longer). RXLOSS is expected here:
    it only clears once RC packets start flowing.
    """
    deadline = time.monotonic() + ready_timeout
    last_names: list[str] = []
    while time.monotonic() < deadline:
        try:
            payload = msp_request(host, port, MSP_STATUS_EX, msp_timeout)
            status = parse_status_ex(payload)
            last_names = status.get("arming_disable_names") or []
            if "BOOTGRACE" not in last_names:
                return
        except Exception:
            pass
        time.sleep(0.2)
    raise RuntimeError(
        "Betaflight arming grace did not clear after %.1fs (arming_disable=%s)"
        % (ready_timeout, ",".join(last_names) or "unknown")
    )


def build_diagnostics_command(args: argparse.Namespace, diagnostics_log: Path) -> list[str]:
    command = [
        sys.executable,
        str(REPO_ROOT / "tools/sitl_diagnostics.py"),
        "--rc-source", "virtual" if args.rc_driver == "virtual" else "external",
        "--samples", str(args.diagnostic_samples),
        "--interval", str(args.diagnostic_interval),
        "--dashboard-timeout", str(args.dashboard_timeout),
        "--direct-msp",
        "--msp-host", args.msp_host,
        "--msp-port", str(args.msp_port),
        "--msp-timeout", str(args.msp_timeout),
        "--include-gazebo-pose",
        "--gazebo-world-name", args.world_name,
        "--gazebo-timeout", str(args.gazebo_timeout),
        "--log-file", str(diagnostics_log),
    ]
    if args.rc_driver == "virtual":
        command.extend([
            "--virtual-throttle", str(args.throttle),
            "--virtual-arm-pwm", "2000",
            "--virtual-mode-pwm", str(args.mode_pwm),
        ])
    return command


def build_virtual_rc_command(args: argparse.Namespace, virtual_rc_log: Path | None = None) -> list[str]:
    command = [
        sys.executable,
        str(REPO_ROOT / "tools/sitl_virtual_rc.py"),
        "--script", "takeoff",
        "--roll", str(args.virtual_roll),
        "--pitch", str(args.virtual_pitch),
        "--throttle", str(args.throttle),
        "--yaw", str(args.virtual_yaw),
        "--mode-pwm", str(args.mode_pwm),
        "--hold-seconds", str(args.hold_seconds),
        "--send",
        "--print-hz", str(args.virtual_print_hz),
    ]
    if args.nudge_delay_seconds is not None:
        command.extend(["--nudge-delay-seconds", str(args.nudge_delay_seconds)])
    for name in ("roll", "pitch", "yaw"):
        value = getattr(args, "nudge_%s" % name)
        if value is not None:
            command.extend(["--nudge-%s" % name, str(value)])
    if virtual_rc_log is not None:
        command.extend(["--log-file", str(virtual_rc_log)])
    return command


def estimate_virtual_rc_timeout(args: argparse.Namespace) -> float:
    if args.virtual_rc_timeout is not None:
        return args.virtual_rc_timeout
    # Matches tools/sitl_virtual_rc.py takeoff defaults: boot-low, arm-low,
    # throttle-ramp, takeoff-hold, disarm-low. Add slack for process startup and
    # scheduler jitter during long Gazebo acceptance runs.
    return 5.0 + 3.0 + 3.0 + args.hold_seconds + 1.0 + 10.0


def build_mixer_config_command(args: argparse.Namespace) -> list[str]:
    command = [
        sys.executable,
        str(REPO_ROOT / "tools/sitl_mixer_config.py"),
        "--host", args.msp_host,
        "--port", str(args.msp_port),
        "--timeout", str(args.msp_timeout),
        "--yaw-motors-reversed", args.yaw_motors_reversed,
    ]
    return command


def build_pid_config_command(args: argparse.Namespace) -> list[str]:
    command = [
        sys.executable,
        str(REPO_ROOT / "tools/sitl_pid_config.py"),
        "--host", args.msp_host,
        "--port", str(args.msp_port),
        "--timeout", str(args.msp_timeout),
    ]
    if args.zero_yaw_pid:
        command.append("--zero-yaw")
    elif args.yaw_pid is not None:
        p, i, d = args.yaw_pid
        command.extend(["--yaw-p", str(p), "--yaw-i", str(i), "--yaw-d", str(d)])
    if args.zero_pitch_pid:
        command.append("--zero-pitch")
    elif args.pitch_pid is not None:
        p, i, d = args.pitch_pid
        command.extend(["--pitch-p", str(p), "--pitch-i", str(i), "--pitch-d", str(d)])
    return command


def build_rate_config_command(args: argparse.Namespace) -> list[str]:
    command = [
        sys.executable,
        str(REPO_ROOT / "tools/sitl_rate_config.py"),
        "--host", args.msp_host,
        "--port", str(args.msp_port),
        "--timeout", str(args.msp_timeout),
    ]
    if args.yaw_rc_rate is not None:
        command.extend(["--yaw-rc-rate", str(args.yaw_rc_rate)])
    if args.yaw_rate is not None:
        command.extend(["--yaw-rate", str(args.yaw_rate)])
    if args.yaw_rate_limit is not None:
        command.extend(["--yaw-rate-limit", str(args.yaw_rate_limit)])
    if args.roll_rc_rate is not None:
        command.extend(["--roll-rc-rate", str(args.roll_rc_rate)])
    if args.roll_rate is not None:
        command.extend(["--roll-rate", str(args.roll_rate)])
    if args.roll_rate_limit is not None:
        command.extend(["--roll-rate-limit", str(args.roll_rate_limit)])
    if args.pitch_rc_rate is not None:
        command.extend(["--pitch-rc-rate", str(args.pitch_rc_rate)])
    if args.pitch_rate is not None:
        command.extend(["--pitch-rate", str(args.pitch_rate)])
    if args.pitch_rate_limit is not None:
        command.extend(["--pitch-rate-limit", str(args.pitch_rate_limit)])
    return command


def apply_safe_yaw_authority(args: argparse.Namespace) -> argparse.Namespace:
    if not getattr(args, "safe_yaw_authority", False):
        return args
    if args.yaw_rc_rate is None:
        args.yaw_rc_rate = SAFE_YAW_RC_RATE
    if args.yaw_rate is None:
        args.yaw_rate = SAFE_YAW_RATE
    if args.yaw_rate_limit is None:
        args.yaw_rate_limit = SAFE_YAW_RATE_LIMIT
    return args


def apply_safe_manual_authority(args: argparse.Namespace) -> argparse.Namespace:
    if not getattr(args, "safe_manual_authority", False):
        return args
    for axis in ("roll", "pitch", "yaw"):
        rc_key = "%s_rc_rate" % axis
        rate_key = "%s_rate" % axis
        limit_key = "%s_rate_limit" % axis
        if getattr(args, rc_key) is None:
            setattr(args, rc_key, SAFE_MANUAL_RC_RATE)
        if getattr(args, rate_key) is None:
            setattr(args, rate_key, SAFE_MANUAL_RATE)
        if getattr(args, limit_key) is None:
            setattr(args, limit_key, SAFE_MANUAL_RATE_LIMIT)
    return args


def build_debug_config_command(args: argparse.Namespace, *, save: bool = False) -> list[str]:
    command = [
        sys.executable,
        str(REPO_ROOT / "tools/sitl_debug_config.py"),
        "--host", args.msp_host,
        "--port", str(args.msp_port),
        "--timeout", str(args.msp_timeout),
        "--debug-mode", str(args.debug_mode),
    ]
    if save:
        command.append("--save")
    return command


def estimate_motor_udp_duration(args: argparse.Namespace) -> float:
    if args.motor_udp_duration is not None:
        return args.motor_udp_duration
    diagnostic_duration = args.diagnostic_samples * args.diagnostic_interval
    if args.rc_driver == "virtual":
        virtual_rc_duration = 5.0 + 3.0 + 3.0 + args.hold_seconds + 1.0 + args.pre_takeoff_seconds
    else:
        virtual_rc_duration = diagnostic_duration + args.pre_takeoff_seconds
    return max(diagnostic_duration + args.pre_takeoff_seconds + 5.0, virtual_rc_duration + 5.0)


def build_motor_udp_command(args: argparse.Namespace, motor_udp_log: Path) -> list[str]:
    return [
        sys.executable,
        str(REPO_ROOT / "tools/sitl_motor_udp_probe.py"),
        "--duration", str(estimate_motor_udp_duration(args)),
        "--log-file", str(motor_udp_log),
    ]


def main() -> int:
    args = parse_args()
    apply_safe_yaw_authority(args)
    apply_safe_manual_authority(args)
    log_dir = resolve_log_dir(args.log_dir, REPO_ROOT)
    diagnostics_log = Path(args.diagnostics_log_file) if args.diagnostics_log_file else make_log_path(log_dir, "takeoff-diagnostics")
    motor_udp_log = (
        Path(args.motor_udp_log_file)
        if args.motor_udp_log_file
        else make_log_path(log_dir, "takeoff-motor-udp")
    ) if args.capture_motor_udp else None
    virtual_rc_log = (
        Path(args.virtual_rc_log_file)
        if args.virtual_rc_log_file
        else make_log_path(log_dir, "takeoff-virtual-rc")
    ) if args.capture_motor_udp and args.rc_driver == "virtual" else None
    process_dir = Path(tempfile.mkdtemp(prefix="kenet-takeoff-check-"))
    env = os.environ.copy()
    env.setdefault("FPV_ROOT", str(REPO_ROOT))
    env.setdefault("AEROLOOP_GAZEBO", str(REPO_ROOT / "../aeroloop_gazebo"))
    env.setdefault("BETAFLIGHT_ROOT", str(REPO_ROOT / "../betaflight"))
    if args.sync_betaflight_looptime:
        env["KENET_SITL_LOOPTIME_US"] = looptime_us_for_step(args.max_step_size)
        print("Betaflight looptime sync: KENET_SITL_LOOPTIME_US=%s" % env["KENET_SITL_LOOPTIME_US"])
    if args.lockstep_wait_us is not None:
        env["KENET_SITL_LOCKSTEP_WAIT_US"] = lockstep_wait_env_value(args.lockstep_wait_us)
        print("Betaflight lockstep bounded wait: KENET_SITL_LOCKSTEP_WAIT_US=%s"
              % env["KENET_SITL_LOCKSTEP_WAIT_US"])
    betaflight_cwd = betaflight_work_dir(args, process_dir)

    gazebo_proc: subprocess.Popen[str] | None = None
    betaflight_proc: subprocess.Popen[str] | None = None
    motor_udp_proc: subprocess.Popen[str] | None = None
    return_code = 1
    process_logs_reported = False

    try:
        if not args.no_start_gazebo:
            gazebo_proc = start_logged_process(build_gazebo_command(args), process_dir / "gazebo.log", REPO_ROOT, env)
            time.sleep(args.gazebo_startup_seconds)
            if gazebo_proc.poll() is not None:
                raise RuntimeError("Gazebo exited early; see %s" % (process_dir / "gazebo.log"))

        if not args.no_start_betaflight:
            import_betaflight_config(args, process_dir, env, betaflight_cwd)
            betaflight_proc = start_betaflight_process(args, process_dir, env, betaflight_cwd)

        if not args.no_start_betaflight:
            sent = bootstrap_gazebo_state(args)
            print("bootstrap motor packets sent: %d" % sent)
            wait_for_msp_ready(args.msp_host, args.msp_port, args.msp_timeout, args.betaflight_ready_timeout, betaflight_proc)

        debug_configured_with_restart = False
        if args.debug_mode is not None and not args.no_start_betaflight and not args.no_debug_restart:
            proc = run_checked(
                build_debug_config_command(args, save=True),
                cwd=REPO_ROOT,
                timeout=args.configure_timeout,
            )
            if proc.returncode != 0:
                raise RuntimeError("debug configuration failed with rc=%d" % proc.returncode)
            if betaflight_proc is not None:
                terminate_process(betaflight_proc)
                betaflight_proc = None
                time.sleep(args.betaflight_restart_settle_seconds)
            betaflight_proc = start_betaflight_process(
                args,
                process_dir,
                env,
                betaflight_cwd,
                log_name="betaflight-after-debug-restart.log",
            )
            sent = bootstrap_gazebo_state(args)
            print("post-restart bootstrap motor packets sent: %d" % sent)
            wait_for_msp_ready(args.msp_host, args.msp_port, args.msp_timeout, args.betaflight_ready_timeout, betaflight_proc)
            debug_configured_with_restart = True

        if not args.skip_configure_modes:
            proc = run_checked(
                [
                    sys.executable,
                    str(REPO_ROOT / "tools/sitl_configure_modes.py"),
                    "--host", args.msp_host,
                    "--port", str(args.msp_port),
                    "--timeout", str(args.msp_timeout),
                ],
                cwd=REPO_ROOT,
                timeout=args.configure_timeout,
            )
            if proc.returncode != 0:
                raise RuntimeError("mode configuration failed with rc=%d" % proc.returncode)

        if args.yaw_motors_reversed != "keep":
            proc = run_checked(
                build_mixer_config_command(args),
                cwd=REPO_ROOT,
                timeout=args.configure_timeout,
            )
            if proc.returncode != 0:
                raise RuntimeError("mixer configuration failed with rc=%d" % proc.returncode)

        if (
            args.yaw_rc_rate is not None
            or args.yaw_rate is not None
            or args.yaw_rate_limit is not None
            or args.roll_rc_rate is not None
            or args.roll_rate is not None
            or args.roll_rate_limit is not None
            or args.pitch_rc_rate is not None
            or args.pitch_rate is not None
            or args.pitch_rate_limit is not None
        ):
            proc = run_checked(
                build_rate_config_command(args),
                cwd=REPO_ROOT,
                timeout=args.configure_timeout,
            )
            if proc.returncode != 0:
                raise RuntimeError("rate configuration failed with rc=%d" % proc.returncode)

        if args.zero_yaw_pid or args.yaw_pid is not None or args.zero_pitch_pid or args.pitch_pid is not None:
            proc = run_checked(
                build_pid_config_command(args),
                cwd=REPO_ROOT,
                timeout=args.configure_timeout,
            )
            if proc.returncode != 0:
                raise RuntimeError("PID configuration failed with rc=%d" % proc.returncode)

        if args.debug_mode is not None and not debug_configured_with_restart:
            proc = run_checked(
                build_debug_config_command(args),
                cwd=REPO_ROOT,
                timeout=args.configure_timeout,
            )
            if proc.returncode != 0:
                raise RuntimeError("debug configuration failed with rc=%d" % proc.returncode)

        if args.capture_motor_udp:
            assert motor_udp_log is not None
            motor_udp_proc = start_logged_process(
                build_motor_udp_command(args, motor_udp_log),
                process_dir / "motor_udp.stdout.log",
                REPO_ROOT,
                env,
            )
            time.sleep(0.2)
            if motor_udp_proc.poll() is not None:
                raise RuntimeError("motor UDP probe exited early; stdout log %s" % (process_dir / "motor_udp.stdout.log"))

        # Must run before the diagnostics poller starts: the SITL MSP TCP port
        # accepts one client at a time, so a later poll would fight the
        # diagnostics connection and time out. Applies to both RC drivers:
        # external senders (kenet mixer) also use fixed scripts that latch
        # ARM_SWITCH if the ARM channel rises during BOOTGRACE.
        if not args.no_start_betaflight:
            wait_for_arming_grace(args.msp_host, args.msp_port, args.msp_timeout, args.arming_grace_timeout)

        diagnostics_cmd = build_diagnostics_command(args, diagnostics_log)
        diagnostics_proc = start_logged_process(diagnostics_cmd, process_dir / "diagnostics.stdout.log", REPO_ROOT, env)
        time.sleep(args.pre_takeoff_seconds)

        if args.rc_driver == "virtual":
            rc_proc = run_checked(
                build_virtual_rc_command(args, virtual_rc_log),
                cwd=REPO_ROOT,
                timeout=estimate_virtual_rc_timeout(args),
            )
            if rc_proc.returncode != 0:
                raise RuntimeError("virtual RC failed with rc=%d" % rc_proc.returncode)
            if virtual_rc_log is not None:
                print("virtual RC log: %s" % virtual_rc_log)
        else:
            print(
                "external RC mode: send RC from sitl_rc_bridge.py, kenet_sitl_mixer.py, "
                "or a physical transmitter during this diagnostics window"
            )

        try:
            diagnostics_rc = diagnostics_proc.wait(timeout=args.diagnostics_timeout)
        except subprocess.TimeoutExpired:
            terminate_process(diagnostics_proc)
            raise RuntimeError("diagnostics timed out; stdout log %s" % (process_dir / "diagnostics.stdout.log"))
        if diagnostics_rc != 0:
            raise RuntimeError("diagnostics failed with rc=%d; stdout log %s" % (
                diagnostics_rc,
                process_dir / "diagnostics.stdout.log",
            ))

        if motor_udp_proc is not None:
            try:
                motor_udp_rc = motor_udp_proc.wait(timeout=args.motor_udp_wait_timeout)
            except subprocess.TimeoutExpired:
                terminate_process(motor_udp_proc)
                raise RuntimeError("motor UDP probe timed out; stdout log %s" % (process_dir / "motor_udp.stdout.log"))
            if motor_udp_rc != 0:
                raise RuntimeError("motor UDP probe failed with rc=%d; stdout log %s" % (
                    motor_udp_rc,
                    process_dir / "motor_udp.stdout.log",
                ))
            print("motor UDP log: %s" % motor_udp_log)
            print_motor_udp_summary(analyze_motor_udp_log(motor_udp_log))

        summary = analyze_diagnostics(
            diagnostics_log,
            target_throttle=args.throttle,
            max_abs_attitude=args.max_abs_attitude,
            min_altitude_gain=args.min_altitude_gain,
        )
        print_check_summary(summary)
        return_code = 0 if summary["passed"] else 1
    except Exception as exc:
        print("result=FAIL %s" % exc, file=sys.stderr)
        print("process_logs=%s" % process_dir, file=sys.stderr)
        process_logs_reported = True
        return_code = 1
    finally:
        if motor_udp_proc is not None:
            terminate_process(motor_udp_proc)
        if betaflight_proc is not None:
            terminate_process(betaflight_proc)
        if gazebo_proc is not None:
            terminate_process(gazebo_proc)
        if return_code == 0 and not args.keep_process_logs:
            shutil.rmtree(process_dir, ignore_errors=True)
        elif return_code == 0:
            print("process_logs=%s" % process_dir)
        elif not process_logs_reported:
            print("process_logs=%s" % process_dir)

    return return_code


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--rc-driver", choices=["virtual", "external"], default="virtual",
                        help="virtual sends the built-in RC takeoff script; external only observes RC already sent to Betaflight")
    parser.add_argument("--world", default="betaloop_iris_betaflight_demo_harmonic.sdf")
    parser.add_argument("--world-name", default="betaloop_demo")
    # 0.0025 matches the bulk of the measured acceptance evidence (video/yaw
    # gates). The coupled Gazebo iris plant is step-dependent per axis: yaw is
    # stable at 0.0025 but unstable at 0.001 (P>=20 nudge / video yaw>=4),
    # while the pitch nudge bracket is the opposite. Gates that need 0.001
    # (pitch nudge profile) must pin it explicitly and record it as evidence.
    parser.add_argument("--max-step-size", default="0.0025")
    parser.add_argument("--sync-betaflight-looptime", action="store_true",
                        help="Export KENET_SITL_LOOPTIME_US=<step in us> so the patched Betaflight SITL "
                             "initializes gyro/PID looptime and filters for the real lockstep step instead "
                             "of the virtual gyro's claimed 8 kHz. Requires the looptime-sync patched build. "
                             "Boundary evidence measured with and without this flag is not comparable.")
    parser.add_argument("--lockstep-wait-us", type=int, default=None,
                        help="Export KENET_SITL_LOCKSTEP_WAIT_US=<us> so the patched Betaflight SITL "
                             "blocks up to this long for the next FDM packet inside lockMainPID instead "
                             "of a pure trylock. Removes the scheduler/FDM phase race at fine steps "
                             "(e.g. 4000 with --max-step-size 0.001). Requires the semaphore-patched build; "
                             "unset keeps stock trylock behavior.")
    parser.add_argument("--gazebo-gui", action="store_true",
                        help="Start Gazebo with its GUI instead of the runner's default headless acceptance mode.")
    parser.add_argument("--gui-config", default=None,
                        help="Gazebo GUI config (e.g. tools/fpv_gui.config for the "
                             "FPV camera panel); only used with --gazebo-gui.")
    parser.add_argument("--no-fix-iris-imu-pose", dest="fix_iris_imu_pose", action="store_false")
    parser.add_argument("--no-fix-iris-motor-map", dest="fix_iris_motor_map", action="store_false")
    parser.add_argument("--iris-forward-camera", action="store_true",
                        help="Inject a forward FPV camera into the temporary Iris model "
                             "(topic /kenet/fpv_camera); matches the launch-fpv-sim.sh setup.")
    parser.set_defaults(fix_iris_imu_pose=True, fix_iris_motor_map=True)
    parser.add_argument("--iris-yaw-gyro-scale", type=float, default=None,
                        help="Temporary BetaflightPlugin yawGyroScale for Iris model diagnostics")
    parser.add_argument("--iris-rotor-vel-p-gain", type=float, default=None,
                        help="Temporary Iris rotor vel_p_gain for motor response diagnostics")
    parser.add_argument("--iris-velocity-control", action="store_true",
                        help="Drive Iris rotors with plugin velocity commands (fast ESC/motor servo) "
                             "instead of the weak force PID; requires the velocity-control plugin build")
    parser.add_argument("--iris-motor-time-constant", type=float, default=None,
                        help="First-order rotor response time constant in seconds for --iris-velocity-control")
    parser.add_argument("--iris-rotor-damping", type=float, default=None,
                        help="Temporary Iris rotor joint damping (stock 0.004 makes differential yaw authority "
                             "unrealistically hot; ~0.001 is closer to real-quad scale)")
    parser.add_argument("--no-start-gazebo", action="store_true")
    parser.add_argument("--no-start-betaflight", action="store_true")
    parser.add_argument("--betaflight-cwd", choices=["temp", "repo"], default="temp",
                        help="Working directory for Betaflight SITL; temp avoids persistent eeprom.bin state")
    parser.add_argument("--betaflight-config-file", default=None,
                        help="Optional Betaflight CLI config file to import into the runner-owned eeprom.bin before start")
    parser.add_argument("--betaflight-config-import-timeout", type=float, default=20.0)
    parser.add_argument("--gazebo-startup-seconds", type=float, default=5.0)
    parser.add_argument("--betaflight-startup-seconds", type=float, default=1.0)
    parser.add_argument("--betaflight-restart-settle-seconds", type=float, default=2.0,
                        help="Delay after stopping Betaflight before a runner-owned restart")
    parser.add_argument("--bootstrap-seconds", type=float, default=1.0)
    parser.add_argument("--bootstrap-hz", type=float, default=100.0)
    parser.add_argument("--motor-host", default="127.0.0.1")
    parser.add_argument("--motor-port", type=int, default=9002)
    parser.add_argument("--skip-configure-modes", action="store_true")
    parser.add_argument("--configure-timeout", type=float, default=10.0)
    parser.add_argument("--msp-host", default="127.0.0.1")
    parser.add_argument("--msp-port", type=int, default=5761)
    parser.add_argument("--msp-timeout", type=float, default=1.0)
    parser.add_argument("--betaflight-ready-timeout", type=float, default=5.0)
    parser.add_argument("--arming-grace-timeout", type=float, default=20.0,
                        help="Seconds to wait for Betaflight's BOOTGRACE arming-disable flag to clear "
                             "before the virtual RC script raises the ARM switch. Prevents the silent "
                             "never-arms latch when boot (e.g. gyro calibration with --sync-betaflight-"
                             "looptime) outlasts the RC script's fixed boot-low phase.")
    parser.add_argument("--yaw-motors-reversed", choices=["keep", "on", "off"], default="keep",
                        help="Optionally set Betaflight yaw_motors_reversed before the takeoff script")
    parser.add_argument("--zero-yaw-pid", action="store_true",
                        help="Set yaw P/I/D to 0/0/0 over MSP before the takeoff script")
    parser.add_argument("--yaw-pid", type=parse_pid_triplet, default=None,
                        help="Set yaw P/I/D over MSP before the takeoff script, e.g. 45,0,0")
    parser.add_argument("--zero-pitch-pid", action="store_true",
                        help="Set pitch P/I/D to 0/0/0 over MSP before the takeoff script")
    parser.add_argument("--pitch-pid", type=parse_pid_triplet, default=None,
                        help="Set pitch P/I/D over MSP before the takeoff script, e.g. 23,0,0")
    parser.add_argument("--yaw-rc-rate", type=parse_byte_value, default=None,
                        help="Set Betaflight yaw_rc_rate over MSP before the takeoff script")
    parser.add_argument("--yaw-rate", type=parse_byte_value, default=None,
                        help="Set Betaflight yaw super-rate over MSP before the takeoff script")
    parser.add_argument("--yaw-rate-limit", type=parse_u16_value, default=None,
                        help="Set Betaflight yaw rate_limit over MSP before the takeoff script")
    parser.add_argument("--roll-rc-rate", type=parse_byte_value, default=None,
                        help="Set Betaflight roll_rc_rate over MSP before the takeoff script")
    parser.add_argument("--roll-rate", type=parse_byte_value, default=None,
                        help="Set Betaflight roll super-rate over MSP before the takeoff script")
    parser.add_argument("--roll-rate-limit", type=parse_u16_value, default=None,
                        help="Set Betaflight roll rate_limit over MSP before the takeoff script")
    parser.add_argument("--pitch-rc-rate", type=parse_byte_value, default=None,
                        help="Set Betaflight pitch_rc_rate over MSP before the takeoff script")
    parser.add_argument("--pitch-rate", type=parse_byte_value, default=None,
                        help="Set Betaflight pitch super-rate over MSP before the takeoff script")
    parser.add_argument("--pitch-rate-limit", type=parse_u16_value, default=None,
                        help="Set Betaflight pitch rate_limit over MSP before the takeoff script")
    parser.add_argument("--safe-yaw-authority", action="store_true",
                        help="Apply the measured Iris SITL safe yaw profile: yaw_rc_rate=5 yaw_rate=30 yaw_rate_limit=120")
    parser.add_argument("--safe-manual-authority", action="store_true",
                        help="Apply the measured Iris SITL safe roll/pitch/yaw profile: rc_rate=5 rate=30 rate_limit=120")
    parser.add_argument("--debug-mode", type=parse_debug_mode_value, default=None,
                        help=(
                            "Set Betaflight debug_mode before the takeoff script, "
                            "e.g. PIDLOOP, ANGLERATE, or 7. Runner-owned Betaflight "
                            "is saved and restarted so the runtime debugMode reloads."
                        ))
    parser.add_argument("--no-debug-restart", action="store_true",
                        help="Do not save/restart runner-owned Betaflight after setting debug_mode")
    parser.add_argument("--throttle", type=int, default=1750)
    parser.add_argument("--mode-pwm", type=int, default=1500)
    parser.add_argument("--virtual-roll", type=int, default=1500)
    parser.add_argument("--virtual-pitch", type=int, default=1500)
    parser.add_argument("--virtual-yaw", type=int, default=1500)
    parser.add_argument("--nudge-delay-seconds", type=float, default=None)
    parser.add_argument("--nudge-roll", type=int, default=None)
    parser.add_argument("--nudge-pitch", type=int, default=None)
    parser.add_argument("--nudge-yaw", type=int, default=None)
    parser.add_argument("--hold-seconds", type=float, default=8.0)
    parser.add_argument("--pre-takeoff-seconds", type=float, default=1.0)
    parser.add_argument("--virtual-print-hz", type=float, default=1.0)
    parser.add_argument("--virtual-rc-timeout", type=float, default=None,
                        help="Timeout for the virtual RC subprocess. Defaults to the estimated takeoff script duration plus slack.")
    parser.add_argument("--diagnostic-samples", type=int, default=75)
    parser.add_argument("--diagnostic-interval", type=float, default=0.1)
    parser.add_argument("--diagnostics-timeout", type=float, default=180.0)
    parser.add_argument("--dashboard-timeout", type=float, default=0.1)
    parser.add_argument("--gazebo-timeout", type=float, default=0.5)
    parser.add_argument("--capture-motor-udp", action="store_true",
                        help="Capture Betaflight raw motor UDP output on :9001 during the check")
    parser.add_argument("--motor-udp-log-file", default=None)
    parser.add_argument("--virtual-rc-log-file", default=None,
                        help="Optional virtual RC JSONL path; used automatically with --capture-motor-udp")
    parser.add_argument("--motor-udp-duration", type=float, default=None,
                        help="Override raw motor UDP capture duration in seconds")
    parser.add_argument("--motor-udp-wait-timeout", type=float, default=20.0)
    parser.add_argument("--max-abs-attitude", type=float, default=35.0)
    parser.add_argument("--min-altitude-gain", type=float, default=1.0)
    parser.add_argument("--log-dir", default=os.environ.get("KENET_SITL_LOG_DIR"))
    parser.add_argument("--diagnostics-log-file", default=None)
    parser.add_argument("--keep-process-logs", action="store_true")
    args = parser.parse_args()
    if args.bootstrap_seconds < 0:
        parser.error("--bootstrap-seconds must be non-negative")
    if args.bootstrap_hz <= 0:
        parser.error("--bootstrap-hz must be positive")
    if args.diagnostic_samples <= 0:
        parser.error("--diagnostic-samples must be positive")
    if args.zero_yaw_pid and args.yaw_pid is not None:
        parser.error("--zero-yaw-pid and --yaw-pid are mutually exclusive")
    if args.zero_pitch_pid and args.pitch_pid is not None:
        parser.error("--zero-pitch-pid and --pitch-pid are mutually exclusive")
    if args.motor_udp_duration is not None and args.motor_udp_duration <= 0:
        parser.error("--motor-udp-duration must be positive")
    if args.motor_udp_wait_timeout <= 0:
        parser.error("--motor-udp-wait-timeout must be positive")
    if args.virtual_rc_timeout is not None and args.virtual_rc_timeout <= 0:
        parser.error("--virtual-rc-timeout must be positive")
    if args.msp_port <= 0:
        parser.error("--msp-port must be positive")
    if args.betaflight_ready_timeout <= 0:
        parser.error("--betaflight-ready-timeout must be positive")
    if args.betaflight_restart_settle_seconds < 0:
        parser.error("--betaflight-restart-settle-seconds must be non-negative")
    if args.betaflight_config_import_timeout <= 0:
        parser.error("--betaflight-config-import-timeout must be positive")
    if args.iris_yaw_gyro_scale is not None and args.iris_yaw_gyro_scale <= 0:
        parser.error("--iris-yaw-gyro-scale must be positive")
    if args.iris_rotor_vel_p_gain is not None and args.iris_rotor_vel_p_gain <= 0:
        parser.error("--iris-rotor-vel-p-gain must be positive")
    if not 1000 <= args.throttle <= 2000:
        parser.error("--throttle must be between 1000 and 2000")
    if not 1000 <= args.mode_pwm <= 2000:
        parser.error("--mode-pwm must be between 1000 and 2000")
    for name in ("virtual_roll", "virtual_pitch", "virtual_yaw"):
        if not 1000 <= getattr(args, name) <= 2000:
            parser.error("--%s must be between 1000 and 2000" % name.replace("_", "-"))
    if args.nudge_delay_seconds is not None and args.nudge_delay_seconds < 0:
        parser.error("--nudge-delay-seconds must be non-negative")
    for name in ("nudge_roll", "nudge_pitch", "nudge_yaw"):
        value = getattr(args, name)
        if value is not None and not 1000 <= value <= 2000:
            parser.error("--%s must be between 1000 and 2000" % name.replace("_", "-"))
    return args


def parse_pid_triplet(value: str) -> tuple[int, int, int]:
    parts = value.split(",")
    if len(parts) != 3:
        raise argparse.ArgumentTypeError("expected P,I,D")
    try:
        parsed = tuple(int(part.strip()) for part in parts)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("PID values must be integers") from exc
    if any(part < 0 or part > 255 for part in parsed):
        raise argparse.ArgumentTypeError("PID values must be 0..255")
    return parsed  # type: ignore[return-value]


def parse_byte_value(value: str) -> int:
    try:
        parsed = int(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("value must be an integer") from exc
    if not 0 <= parsed <= 255:
        raise argparse.ArgumentTypeError("value must be 0..255")
    return parsed


def parse_u16_value(value: str) -> int:
    try:
        parsed = int(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("value must be an integer") from exc
    if not 0 <= parsed <= 65535:
        raise argparse.ArgumentTypeError("value must be 0..65535")
    return parsed


if __name__ == "__main__":
    raise SystemExit(main())
