#!/usr/bin/env python3
"""Calibrate whether Betaflight SITL MSP_DEBUG changes under virtual RC input."""

from __future__ import annotations

import argparse
import os
import shutil
import socket
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

from gazebo_motor_moment_probe import terminate_process
from sitl_debug_config import debug_mode_name, parse_debug_mode_value
from sitl_log import JsonlLogger, make_log_path, resolve_log_dir
from sitl_msp import (
    MSP_ADVANCED_CONFIG,
    MSP_DEBUG,
    MSP_RC,
    msp_request_many,
    parse_advanced_config,
    parse_debug,
    parse_u16_list,
)
from sitl_virtual_rc import make_virtual_channels, send_channels


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
        )
    finally:
        log_handle.close()


def run_checked(command: list[str], *, cwd: Path, timeout: float) -> subprocess.CompletedProcess[str]:
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


def start_betaflight(args: argparse.Namespace, process_dir: Path, work_dir: Path, env: dict[str, str], log_name: str) -> subprocess.Popen[str]:
    betaflight_bin = Path(env["BETAFLIGHT_ROOT"]) / "obj/main/betaflight_SITL.elf"
    proc = start_logged_process([str(betaflight_bin)], process_dir / log_name, work_dir, env)
    time.sleep(args.betaflight_startup_seconds)
    if proc.poll() is not None:
        raise RuntimeError("Betaflight exited early; see %s" % (process_dir / log_name))
    return proc


def build_debug_config_command(args: argparse.Namespace, *, save: bool) -> list[str]:
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


def sample_msp(host: str, port: int, timeout: float) -> dict[str, Any]:
    responses = msp_request_many(host, port, [MSP_ADVANCED_CONFIG, MSP_RC, MSP_DEBUG], timeout)
    advanced_config = parse_advanced_config(responses[MSP_ADVANCED_CONFIG])
    return {
        "advanced_config": advanced_config,
        "debug_mode": advanced_config.get("debug_mode"),
        "rc_channels": parse_u16_list(responses[MSP_RC]),
        "debug": parse_debug(responses[MSP_DEBUG]),
    }


def debug_abs_max(records: list[dict[str, Any]], channels: int = 8) -> list[int | None]:
    maxima: list[int | None] = [None] * channels
    for record in records:
        values = record.get("debug") or []
        for index, value in enumerate(values[:channels]):
            if not isinstance(value, int):
                continue
            current = maxima[index]
            maxima[index] = abs(value) if current is None else max(current, abs(value))
    return maxima


def first_nonzero_debug(records: list[dict[str, Any]]) -> dict[str, Any] | None:
    for record in records:
        values = record.get("debug") or []
        if any(isinstance(value, int) and value != 0 for value in values):
            return record
    return None


def summarize_records(records: list[dict[str, Any]], expected_debug_mode: int) -> dict[str, Any]:
    max_abs_debug = debug_abs_max(records)
    debug_modes = sorted({
        record.get("debug_mode")
        for record in records
        if record.get("debug_mode") is not None
    })
    rc_yaw_values = [
        (record.get("rc_channels") or [None, None, None])[2]
        for record in records
        if len(record.get("rc_channels") or []) > 2
    ]
    nonzero = first_nonzero_debug(records)
    return {
        "samples": len(records),
        "expected_debug_mode": expected_debug_mode,
        "debug_mode_name": debug_mode_name(expected_debug_mode),
        "debug_modes_seen": debug_modes,
        "debug_mode_ok": expected_debug_mode in debug_modes,
        "max_abs_debug": max_abs_debug,
        "any_nonzero_debug": any(value not in (None, 0) for value in max_abs_debug),
        "first_nonzero_debug": nonzero,
        "min_fc_yaw": min(rc_yaw_values) if rc_yaw_values else None,
        "max_fc_yaw": max(rc_yaw_values) if rc_yaw_values else None,
    }


def print_summary(summary: dict[str, Any]) -> None:
    print("debug calibration: %s/%s" % (summary["expected_debug_mode"], summary["debug_mode_name"]))
    print("samples: %s" % summary["samples"])
    print("debug modes seen: %s" % summary["debug_modes_seen"])
    print("FC yaw range: %s..%s" % (_fmt(summary.get("min_fc_yaw")), _fmt(summary.get("max_fc_yaw"))))
    print("max abs debug[0:8]: %s" % ",".join(_fmt(value) for value in summary["max_abs_debug"]))
    print("any nonzero debug: %s" % summary["any_nonzero_debug"])


def _fmt(value: Any) -> str:
    return "-" if value is None else str(value)


def run_calibration(args: argparse.Namespace) -> int:
    log_dir = resolve_log_dir(args.log_dir, REPO_ROOT)
    log_path = Path(args.log_file) if args.log_file else make_log_path(log_dir, "debug-calibration")
    process_dir = Path(tempfile.mkdtemp(prefix="kenet-debug-calibration-"))
    work_dir = process_dir / "betaflight-cwd"
    work_dir.mkdir(parents=True, exist_ok=True)
    env = os.environ.copy()
    env.setdefault("BETAFLIGHT_ROOT", str(REPO_ROOT / "../betaflight"))

    proc: subprocess.Popen[str] | None = None
    records: list[dict[str, Any]] = []
    return_code = 1
    process_logs_reported = False

    try:
        proc = start_betaflight(args, process_dir, work_dir, env, "betaflight-before-debug.log")
        config = run_checked(build_debug_config_command(args, save=True), cwd=REPO_ROOT, timeout=args.configure_timeout)
        if config.returncode != 0:
            raise RuntimeError("debug configuration failed with rc=%d" % config.returncode)
        terminate_process(proc)
        proc = None
        proc = start_betaflight(args, process_dir, work_dir, env, "betaflight-after-debug-restart.log")

        channels = make_virtual_channels(
            roll=args.roll,
            pitch=args.pitch,
            throttle=args.throttle,
            yaw=args.yaw,
            arm_pwm=args.arm_pwm,
            kenet_pwm=args.kenet_pwm,
            mode_pwm=args.mode_pwm,
        )
        with JsonlLogger(log_path, metadata={
            "tool": "sitl_debug_calibration",
            "debug_mode": args.debug_mode,
            "debug_mode_name": debug_mode_name(args.debug_mode),
            "channels": channels,
            "sample_hz": args.sample_hz,
            "duration": args.duration,
        }) as logger:
            period = 1.0 / args.sample_hz
            deadline = time.monotonic() + args.duration
            while time.monotonic() < deadline:
                sent = send_channels(None if args.no_send_rc else args._sock, channels, args.rc_host, args.rc_port)
                sample = sample_msp(args.msp_host, args.msp_port, args.msp_timeout)
                sample["sent_bytes"] = sent
                sample["channels_sent"] = channels
                logger.write("debug_calibration_sample", **sample)
                records.append(sample)
                time.sleep(period)
            summary = summarize_records(records, args.debug_mode)
            logger.write("debug_calibration_summary", summary=summary)
        print("debug calibration log: %s" % log_path)
        print_summary(summary)
        return_code = 0 if summary["debug_mode_ok"] else 1
    except Exception as exc:
        print("result=FAIL %s" % exc, file=sys.stderr)
        print("process_logs=%s" % process_dir, file=sys.stderr)
        process_logs_reported = True
        return_code = 1
    finally:
        if getattr(args, "_sock", None) is not None:
            args._sock.close()
        if proc is not None:
            terminate_process(proc)
        if return_code == 0 and not args.keep_process_logs:
            shutil.rmtree(process_dir, ignore_errors=True)
        elif return_code == 0:
            print("process_logs=%s" % process_dir)
        elif not process_logs_reported:
            print("process_logs=%s" % process_dir)
    return return_code


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--debug-mode", type=parse_debug_mode_value, default=parse_debug_mode_value("ANGLERATE"),
                        help="Debug mode name or number, e.g. ANGLERATE, PIDLOOP, ANGLE_TARGET")
    parser.add_argument("--duration", type=float, default=3.0)
    parser.add_argument("--sample-hz", type=float, default=20.0)
    parser.add_argument("--roll", type=int, default=1500)
    parser.add_argument("--pitch", type=int, default=1500)
    parser.add_argument("--throttle", type=int, default=1000)
    parser.add_argument("--yaw", type=int, default=1700)
    parser.add_argument("--arm-pwm", type=int, default=1000)
    parser.add_argument("--kenet-pwm", type=int, default=1000)
    parser.add_argument("--mode-pwm", type=int, default=1500)
    parser.add_argument("--no-send-rc", action="store_true")
    parser.add_argument("--rc-host", default="127.0.0.1")
    parser.add_argument("--rc-port", type=int, default=9004)
    parser.add_argument("--msp-host", default="127.0.0.1")
    parser.add_argument("--msp-port", type=int, default=5761)
    parser.add_argument("--msp-timeout", type=float, default=2.0)
    parser.add_argument("--configure-timeout", type=float, default=10.0)
    parser.add_argument("--betaflight-startup-seconds", type=float, default=1.0)
    parser.add_argument("--log-dir", default=os.environ.get("KENET_SITL_LOG_DIR"))
    parser.add_argument("--log-file", default=None)
    parser.add_argument("--keep-process-logs", action="store_true")
    args = parser.parse_args()
    if args.duration <= 0:
        parser.error("--duration must be positive")
    if args.sample_hz <= 0:
        parser.error("--sample-hz must be positive")
    for name in ("roll", "pitch", "throttle", "yaw", "arm_pwm", "kenet_pwm", "mode_pwm"):
        value = getattr(args, name)
        if not 1000 <= value <= 2000:
            parser.error("--%s must be between 1000 and 2000" % name.replace("_", "-"))
    if args.msp_timeout <= 0:
        parser.error("--msp-timeout must be positive")
    if args.configure_timeout <= 0:
        parser.error("--configure-timeout must be positive")
    if args.betaflight_startup_seconds <= 0:
        parser.error("--betaflight-startup-seconds must be positive")
    args._sock = None if args.no_send_rc else socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    return args


if __name__ == "__main__":
    raise SystemExit(run_calibration(parse_args()))
