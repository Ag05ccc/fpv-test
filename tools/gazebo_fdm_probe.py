#!/usr/bin/env python3
"""Capture Gazebo BetaflightPlugin FDM packets and compare IMU signs."""

from __future__ import annotations

import argparse
import os
import shutil
import socket
import struct
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

from gazebo_motor_moment_probe import (  # noqa: E402
    bf_axis_patterns,
    choose_imu_topic,
    pack_motor_packet,
    parse_motor_map,
    remap_speeds,
    reset_world,
    sample_imu,
    send_motor_speeds,
    start_process,
    terminate_process,
)
from sitl_log import JsonlLogger, make_log_path, resolve_log_dir  # noqa: E402


FDM_DOUBLE_COUNT = 37
FDM_PACKET_SIZE = struct.calcsize("<%dd" % FDM_DOUBLE_COUNT)
AXES = ("x", "y", "z")
EXPECTED_ANGULAR_SIGNS = {"x": 1.0, "y": 1.0, "z": -1.0}


def _vector3(values: tuple[float, ...] | list[float], offset: int) -> dict[str, float]:
    return {
        "x": float(values[offset]),
        "y": float(values[offset + 1]),
        "z": float(values[offset + 2]),
    }


def _vector4(values: tuple[float, ...] | list[float], offset: int) -> dict[str, float]:
    return {
        "w": float(values[offset]),
        "x": float(values[offset + 1]),
        "y": float(values[offset + 2]),
        "z": float(values[offset + 3]),
    }


def parse_fdm_packet(data: bytes) -> dict[str, Any]:
    if len(data) < FDM_PACKET_SIZE:
        raise ValueError("FDM packet too short: %d < %d" % (len(data), FDM_PACKET_SIZE))
    values = struct.unpack("<%dd" % FDM_DOUBLE_COUNT, data[:FDM_PACKET_SIZE])
    return {
        "timestamp": values[0],
        "imu_angular_velocity": _vector3(values, 1),
        "imu_linear_acceleration": _vector3(values, 4),
        "imu_orientation_quat": _vector4(values, 7),
        "velocity": _vector3(values, 11),
        "position": _vector3(values, 14),
        "esc_temperature": list(values[17:21]),
        "esc_voltage": list(values[21:25]),
        "esc_current": list(values[25:29]),
        "esc_consumption": list(values[29:33]),
        "esc_rpm": list(values[33:37]),
    }


def angular_sign_summary(
    gazebo_imu: dict[str, Any] | None,
    fdm_packet: dict[str, Any] | None,
    *,
    min_abs: float,
) -> dict[str, dict[str, Any]]:
    gazebo_angular = ((gazebo_imu or {}).get("angular_velocity") or {})
    fdm_angular = ((fdm_packet or {}).get("imu_angular_velocity") or {})
    summary: dict[str, dict[str, Any]] = {}
    for axis in AXES:
        gz_value = gazebo_angular.get(axis)
        fdm_value = fdm_angular.get(axis)
        expected_sign = EXPECTED_ANGULAR_SIGNS[axis]
        item: dict[str, Any] = {
            "gazebo": gz_value,
            "fdm": fdm_value,
            "expected_relation": "same" if expected_sign > 0 else "inverted",
            "status": "missing",
        }
        if not isinstance(gz_value, (int, float)) or not isinstance(fdm_value, (int, float)):
            summary[axis] = item
            continue
        if abs(gz_value) < min_abs or abs(fdm_value) < min_abs:
            item["status"] = "insufficient-motion"
            summary[axis] = item
            continue
        product = float(gz_value) * float(fdm_value) * expected_sign
        item["observed_relation"] = "same" if float(gz_value) * float(fdm_value) > 0 else "inverted"
        item["status"] = "ok" if product > 0 else "mismatch"
        summary[axis] = item
    return summary


def select_axis_pattern(name: str, base_speed: float, axis_delta: float) -> list[float]:
    patterns = dict(bf_axis_patterns(base_speed, axis_delta))
    if name not in patterns:
        raise ValueError("unknown axis pattern %r; expected one of %s" % (name, ", ".join(sorted(patterns))))
    return patterns[name]


def bind_fdm_socket(host: str, port: int, timeout: float) -> socket.socket:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((host, port))
    sock.settimeout(timeout)
    return sock


def collect_fdm_samples(
    *,
    fdm_sock: socket.socket,
    motor_speeds: list[float],
    seconds: float,
    motor_host: str,
    motor_port: int,
    hz: float,
) -> tuple[list[dict[str, Any]], int]:
    motor_packet = pack_motor_packet(motor_speeds)
    motor_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    period = 1.0 / hz
    next_send = time.monotonic()
    deadline = time.monotonic() + seconds
    samples: list[dict[str, Any]] = []
    sent_packets = 0
    try:
        fdm_sock.setblocking(False)
        while time.monotonic() < deadline:
            now = time.monotonic()
            if now >= next_send:
                motor_sock.sendto(motor_packet, (motor_host, motor_port))
                sent_packets += 1
                next_send += period
            while True:
                try:
                    data, address = fdm_sock.recvfrom(4096)
                except BlockingIOError:
                    break
                if len(data) >= FDM_PACKET_SIZE:
                    samples.append({
                        "recv_monotonic": time.monotonic(),
                        "address": list(address),
                        "bytes": len(data),
                        "packet": parse_fdm_packet(data),
                    })
            time.sleep(0.002)
    finally:
        motor_sock.close()
        fdm_sock.settimeout(1.0)
    return samples, sent_packets


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--world", default="betaloop_iris_betaflight_demo_harmonic.sdf")
    parser.add_argument("--world-name", default="betaloop_demo")
    parser.add_argument("--motor-host", default="127.0.0.1")
    parser.add_argument("--motor-port", type=int, default=9002)
    parser.add_argument("--fdm-host", default="127.0.0.1")
    parser.add_argument("--fdm-port", type=int, default=9003)
    parser.add_argument("--hz", type=float, default=100.0)
    parser.add_argument("--base-speed", type=float, default=0.45)
    parser.add_argument("--axis-delta", type=float, default=0.04)
    parser.add_argument("--pattern", default="bf_cw_pair_high")
    parser.add_argument("--motor-map", type=parse_motor_map, default=parse_motor_map("bf-sitl"),
                        help="Logical motor index to packet index map. Use bf-sitl, identity, or e.g. 0,3,2,1")
    parser.add_argument("--settle-seconds", type=float, default=0.5)
    parser.add_argument("--sample-seconds", type=float, default=1.0)
    parser.add_argument("--startup-seconds", type=float, default=5.0)
    parser.add_argument("--sample-timeout", type=float, default=2.0)
    parser.add_argument("--sign-min-abs", type=float, default=0.02)
    parser.add_argument("--require-axis", action="append", choices=list(AXES), default=[])
    parser.add_argument("--no-start-gazebo", action="store_true")
    parser.add_argument("--keep-process-logs", action="store_true")
    parser.add_argument("--log-dir", default=os.environ.get("KENET_SITL_LOG_DIR"))
    parser.add_argument("--log-file", default=None)
    args = parser.parse_args()
    if args.hz <= 0:
        parser.error("--hz must be positive")
    if args.sample_seconds <= 0 or args.settle_seconds < 0:
        parser.error("--sample-seconds must be positive and --settle-seconds must be non-negative")
    if args.axis_delta < 0:
        parser.error("--axis-delta must be non-negative")
    if args.sign_min_abs < 0:
        parser.error("--sign-min-abs must be non-negative")
    try:
        select_axis_pattern(args.pattern, args.base_speed, args.axis_delta)
    except ValueError as exc:
        parser.error(str(exc))
    return args


def main() -> int:
    args = parse_args()
    env = os.environ.copy()
    env.setdefault("FPV_ROOT", str(REPO_ROOT))
    env.setdefault("AEROLOOP_GAZEBO", str(REPO_ROOT / "../aeroloop_gazebo"))
    env.setdefault("BETAFLIGHT_ROOT", str(REPO_ROOT / "../betaflight"))

    log_dir = resolve_log_dir(args.log_dir, REPO_ROOT)
    log_path = Path(args.log_file) if args.log_file else make_log_path(log_dir, "fdm-probe")
    process_dir = Path(tempfile.mkdtemp(prefix="kenet-fdm-probe-"))
    gazebo_proc: subprocess.Popen[str] | None = None
    failed = False

    metadata = {
        "tool": "gazebo_fdm_probe",
        "world": args.world,
        "world_name": args.world_name,
        "pattern": args.pattern,
        "base_speed": args.base_speed,
        "axis_delta": args.axis_delta,
        "motor_map": args.motor_map,
        "fdm_port": args.fdm_port,
        "expected_angular_signs": EXPECTED_ANGULAR_SIGNS,
    }

    fdm_sock: socket.socket | None = None
    try:
        if not args.no_start_gazebo:
            gazebo_proc = start_process(
                [
                    str(REPO_ROOT / "tools/run_gazebo_betaflight.sh"),
                    "--world", args.world,
                    "--headless",
                    "--max-step-size", "0.0025",
                    "--fix-iris-imu-pose",
                    "--fix-iris-motor-map",
                ],
                process_dir / "gazebo.log",
                REPO_ROOT,
                env,
            )
            time.sleep(args.startup_seconds)
            if gazebo_proc.poll() is not None:
                raise RuntimeError("Gazebo exited early; see %s" % (process_dir / "gazebo.log"))

        fdm_sock = bind_fdm_socket(args.fdm_host, args.fdm_port, args.sample_timeout)
        imu_topic = choose_imu_topic(args.world_name, args.sample_timeout)
        metadata["imu_topic"] = imu_topic

        reset_ok = reset_world(args.world_name, args.sample_timeout)
        if args.settle_seconds:
            send_motor_speeds([0.0] * 4, args.settle_seconds, args.motor_host, args.motor_port, args.hz)
        before_imu = sample_imu(imu_topic, args.sample_timeout)
        logical_speeds = select_axis_pattern(args.pattern, args.base_speed, args.axis_delta)
        speeds = remap_speeds(logical_speeds, args.motor_map)
        samples, sent_packets = collect_fdm_samples(
            fdm_sock=fdm_sock,
            motor_speeds=speeds,
            seconds=args.sample_seconds,
            motor_host=args.motor_host,
            motor_port=args.motor_port,
            hz=args.hz,
        )
        after_imu = sample_imu(imu_topic, args.sample_timeout)
        latest_fdm = samples[-1]["packet"] if samples else None
        sign_summary = angular_sign_summary(after_imu, latest_fdm, min_abs=args.sign_min_abs)
        required_failures = [
            axis for axis in args.require_axis
            if (sign_summary.get(axis) or {}).get("status") != "ok"
        ]

        with JsonlLogger(log_path, metadata=metadata) as logger:
            logger.write(
                "fdm_probe_sample",
                reset_ok=reset_ok,
                imu_topic=imu_topic,
                logical_speeds=logical_speeds,
                speeds=speeds,
                motor_map=args.motor_map,
                sent_packets=sent_packets,
                fdm_sample_count=len(samples),
                first_fdm=samples[0] if samples else None,
                latest_fdm=samples[-1] if samples else None,
                before_imu=before_imu,
                after_imu=after_imu,
                angular_sign_summary=sign_summary,
                required_axes=args.require_axis,
                required_failures=required_failures,
            )

        print("pattern=%s speeds=%s sent_packets=%d fdm_samples=%d reset=%s" % (
            args.pattern,
            ",".join("%.2f" % value for value in speeds),
            sent_packets,
            len(samples),
            reset_ok,
        ))
        for axis in AXES:
            item = sign_summary[axis]
            print(
                "angular_%s gazebo=%s fdm=%s expected=%s status=%s" %
                (
                    axis,
                    _fmt(item.get("gazebo")),
                    _fmt(item.get("fdm")),
                    item.get("expected_relation"),
                    item.get("status"),
                )
            )
        print("log=%s" % log_path)

        if not samples:
            print("result=FAIL no FDM packets captured", file=sys.stderr)
            return 1
        if required_failures:
            print("result=FAIL required axes did not match: %s" % ",".join(required_failures), file=sys.stderr)
            return 1
        return 0
    except Exception as exc:
        failed = True
        print("result=FAIL %s" % exc, file=sys.stderr)
        print("process_logs=%s" % process_dir, file=sys.stderr)
        return 1
    finally:
        if fdm_sock is not None:
            fdm_sock.close()
        if gazebo_proc is not None:
            terminate_process(gazebo_proc)
        if not args.keep_process_logs and not failed:
            shutil.rmtree(process_dir, ignore_errors=True)


def _fmt(value: Any) -> str:
    if value is None:
        return "-"
    if isinstance(value, float):
        return "%.3f" % value
    return str(value)


if __name__ == "__main__":
    raise SystemExit(main())
