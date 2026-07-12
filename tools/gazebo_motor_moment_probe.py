#!/usr/bin/env python3
"""Probe Gazebo BetaflightPlugin direct motor-to-body moment response."""

from __future__ import annotations

import argparse
import json
import math
import os
import signal
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

from sitl_log import JsonlLogger, make_log_path, resolve_log_dir


MOTOR_MAP_ALIASES = {
    "identity": [0, 1, 2, 3],
    "bf-sitl": [3, 0, 1, 2],
}


def pack_motor_packet(speeds: list[float]) -> bytes:
    if len(speeds) != 4:
        raise ValueError("exactly four motor speeds are required")
    return struct.pack("<4f", *[max(0.0, min(1.0, float(value))) for value in speeds])


def send_motor_speeds(speeds: list[float], seconds: float, host: str, port: int, hz: float) -> int:
    packet = pack_motor_packet(speeds)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    deadline = time.monotonic() + seconds
    period = 1.0 / hz
    count = 0
    try:
        while time.monotonic() < deadline:
            sock.sendto(packet, (host, port))
            count += 1
            time.sleep(period)
    finally:
        sock.close()
    return count


def remap_speeds(speeds: list[float], motor_map: list[int]) -> list[float]:
    if len(speeds) != len(motor_map):
        raise ValueError("motor map length must match speed count")
    remapped = [0.0] * len(speeds)
    for logical_index, physical_index in enumerate(motor_map):
        remapped[physical_index] = speeds[logical_index]
    return remapped


def run_text(command: list[str], timeout: float) -> tuple[str, str, int | None]:
    try:
        proc = subprocess.run(
            command,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            timeout=timeout,
            check=False,
        )
        return proc.stdout, proc.stderr, proc.returncode
    except subprocess.TimeoutExpired as exc:
        stdout = exc.stdout.decode("utf-8", errors="replace") if isinstance(exc.stdout, bytes) else (exc.stdout or "")
        stderr = exc.stderr.decode("utf-8", errors="replace") if isinstance(exc.stderr, bytes) else (exc.stderr or "")
        return stdout, stderr, None


def gz_topic_once(topic: str, timeout: float) -> str:
    stdout, _stderr, _rc = run_text(
        ["timeout", f"{timeout}s", "gz", "topic", "-e", "-t", topic, "-n", "1"],
        timeout + 1.0,
    )
    return stdout


def list_topics(timeout: float) -> list[str]:
    stdout, _stderr, _rc = run_text(["gz", "topic", "-l"], timeout)
    return [line.strip() for line in stdout.splitlines() if line.strip()]


def choose_imu_topic(world_name: str, timeout: float) -> str:
    topics = list_topics(timeout)
    preferred = [
        topic for topic in topics
        if world_name in topic and "imu" in topic.lower() and topic.endswith("/imu")
    ]
    if preferred:
        return sorted(preferred)[0]
    any_imu = [topic for topic in topics if "imu" in topic.lower()]
    if any_imu:
        return sorted(any_imu)[0]
    return f"/world/{world_name}/model/iris/link/iris/imu_link/sensor/imu_sensor/imu"


def _scalar(block: str, name: str) -> float | None:
    marker = f"{name}:"
    for line in block.splitlines():
        line = line.strip()
        if not line.startswith(marker):
            continue
        try:
            return float(line[len(marker):].strip())
        except ValueError:
            return None
    return None


def _block_after(text: str, marker: str) -> str | None:
    start = text.find(marker)
    if start < 0:
        return None
    brace = text.find("{", start)
    if brace < 0:
        return None
    depth = 0
    for index in range(brace, len(text)):
        char = text[index]
        if char == "{":
            depth += 1
        elif char == "}":
            depth -= 1
            if depth == 0:
                return text[brace + 1:index]
    return None


def _message_chunk_for_name(text: str, entity_name: str) -> str | None:
    marker = f'name: "{entity_name}"'
    start = text.find(marker)
    if start < 0:
        return None
    next_start = text.find('\nname: "', start + len(marker))
    if next_start < 0:
        next_start = text.find('\npose {', start + len(marker))
    if next_start < 0:
        next_start = len(text)
    return text[start:next_start]


def quat_to_euler_deg(w: float, x: float, y: float, z: float) -> dict[str, float]:
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return {
        "roll": math.degrees(roll),
        "pitch": math.degrees(pitch),
        "yaw": math.degrees(yaw),
    }


def parse_pose_text(text: str, entity_name: str) -> dict[str, Any] | None:
    chunk = _message_chunk_for_name(text, entity_name)
    if chunk is None:
        return None
    position = _block_after(chunk, "position")
    orientation = _block_after(chunk, "orientation")
    if orientation is None:
        return None
    qx = _scalar(orientation, "x") or 0.0
    qy = _scalar(orientation, "y") or 0.0
    qz = _scalar(orientation, "z") or 0.0
    qw = _scalar(orientation, "w")
    if qw is None:
        qw = 1.0
    pose = {
        "entity": entity_name,
        "quaternion": {"w": qw, "x": qx, "y": qy, "z": qz},
        "euler_deg": quat_to_euler_deg(qw, qx, qy, qz),
    }
    if position is not None:
        pose["position"] = {
            "x": _scalar(position, "x") or 0.0,
            "y": _scalar(position, "y") or 0.0,
            "z": _scalar(position, "z") or 0.0,
        }
    return pose


def parse_imu_text(text: str) -> dict[str, Any]:
    orientation = _block_after(text, "orientation")
    angular = _block_after(text, "angular_velocity")
    linear = _block_after(text, "linear_acceleration")
    result: dict[str, Any] = {}
    if orientation is not None:
        qx = _scalar(orientation, "x") or 0.0
        qy = _scalar(orientation, "y") or 0.0
        qz = _scalar(orientation, "z") or 0.0
        qw = _scalar(orientation, "w")
        if qw is None:
            qw = 1.0
        result["quaternion"] = {"w": qw, "x": qx, "y": qy, "z": qz}
        result["euler_deg"] = quat_to_euler_deg(qw, qx, qy, qz)
    if angular is not None:
        result["angular_velocity"] = {
            "x": _scalar(angular, "x") or 0.0,
            "y": _scalar(angular, "y") or 0.0,
            "z": _scalar(angular, "z") or 0.0,
        }
    if linear is not None:
        result["linear_acceleration"] = {
            "x": _scalar(linear, "x") or 0.0,
            "y": _scalar(linear, "y") or 0.0,
            "z": _scalar(linear, "z") or 0.0,
        }
    return result


def sample_pose(world_name: str, entity_name: str, timeout: float) -> dict[str, Any] | None:
    text = gz_topic_once(f"/world/{world_name}/dynamic_pose/info", timeout)
    return parse_pose_text(text, entity_name)


def sample_imu(topic: str, timeout: float) -> dict[str, Any]:
    return parse_imu_text(gz_topic_once(topic, timeout))


def reset_world(world_name: str, timeout: float) -> bool:
    _stdout, _stderr, rc = run_text(
        [
            "gz", "service",
            "-s", f"/world/{world_name}/control",
            "--reqtype", "gz.msgs.WorldControl",
            "--reptype", "gz.msgs.Boolean",
            "--timeout", str(int(timeout * 1000)),
            "--req", "reset: { all: true }",
        ],
        timeout + 1.0,
    )
    return rc == 0


def delta_pose(before: dict[str, Any] | None, after: dict[str, Any] | None) -> dict[str, float | None]:
    if before is None or after is None:
        return {"roll": None, "pitch": None, "yaw": None}
    b = before.get("euler_deg") or {}
    a = after.get("euler_deg") or {}
    return {
        axis: (a.get(axis) - b.get(axis)) if a.get(axis) is not None and b.get(axis) is not None else None
        for axis in ("roll", "pitch", "yaw")
    }


def delta_vector(
    before: dict[str, Any] | None,
    after: dict[str, Any] | None,
    field: str,
) -> dict[str, float | None]:
    if before is None or after is None:
        return {"x": None, "y": None, "z": None}
    b = before.get(field) or {}
    a = after.get(field) or {}
    return {
        axis: (a.get(axis) - b.get(axis)) if a.get(axis) is not None and b.get(axis) is not None else None
        for axis in ("x", "y", "z")
    }


def start_process(command: list[str], log_path: Path, cwd: Path, env: dict[str, str]) -> subprocess.Popen[str]:
    log_file = log_path.open("w", encoding="utf-8", errors="replace")
    try:
        return subprocess.Popen(
            command,
            cwd=str(cwd),
            env=env,
            stdout=log_file,
            stderr=subprocess.STDOUT,
            text=True,
            start_new_session=True,
        )
    finally:
        log_file.close()


def terminate_process(proc: subprocess.Popen[str]) -> None:
    if proc.poll() is not None:
        return
    try:
        pgid = os.getpgid(proc.pid)
    except ProcessLookupError:
        return

    use_process_group = pgid != os.getpgrp()
    if use_process_group:
        os.killpg(pgid, signal.SIGTERM)
    else:
        proc.terminate()
    try:
        proc.wait(timeout=3)
    except subprocess.TimeoutExpired:
        if use_process_group:
            os.killpg(pgid, signal.SIGKILL)
        else:
            proc.kill()
        proc.wait(timeout=3)


def motor_patterns(base: float, pulse: float) -> list[tuple[str, list[float]]]:
    patterns = []
    for index in range(4):
        speeds = [base] * 4
        speeds[index] = pulse
        patterns.append((f"motor{index}", speeds))
    patterns.append(("all_equal", [pulse] * 4))
    return patterns


def bf_axis_patterns(base: float, delta: float) -> list[tuple[str, list[float]]]:
    high = max(0.0, min(1.0, base + delta))
    low = max(0.0, min(1.0, base - delta))
    # Betaflight QUADX order in the patched iris model:
    # BF0 rear-right CW, BF1 front-right CCW, BF2 rear-left CCW, BF3 front-left CW.
    return [
        ("bf_right_pair_high", [high, high, low, low]),
        ("bf_left_pair_high", [low, low, high, high]),
        ("bf_front_pair_high", [low, high, low, high]),
        ("bf_rear_pair_high", [high, low, high, low]),
        ("bf_cw_pair_high", [high, low, low, high]),
        ("bf_ccw_pair_high", [low, high, high, low]),
    ]


def build_patterns(args: argparse.Namespace) -> list[tuple[str, list[float]]]:
    patterns: list[tuple[str, list[float]]] = []
    if args.pattern_set in ("motors", "all"):
        patterns.extend(motor_patterns(args.base_speed, args.pulse_speed))
    if args.pattern_set in ("axis", "all"):
        patterns.extend(bf_axis_patterns(args.base_speed, args.axis_delta))
    return patterns


def parse_motor_map(value: str) -> list[int]:
    if value in MOTOR_MAP_ALIASES:
        return list(MOTOR_MAP_ALIASES[value])
    try:
        items = [int(part.strip()) for part in value.split(",")]
    except ValueError as exc:
        raise argparse.ArgumentTypeError("motor map must be comma-separated integers") from exc
    if len(items) != 4:
        raise argparse.ArgumentTypeError("motor map must contain exactly four indexes")
    if sorted(items) != [0, 1, 2, 3]:
        raise argparse.ArgumentTypeError("motor map must be a permutation of 0,1,2,3")
    return items


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--world", default="betaloop_iris_betaflight_demo_harmonic.sdf")
    parser.add_argument("--world-name", default="betaloop_demo")
    parser.add_argument("--entity", default="iris")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=9002)
    parser.add_argument("--hz", type=float, default=100.0)
    parser.add_argument("--base-speed", type=float, default=0.45)
    parser.add_argument("--pulse-speed", type=float, default=0.62)
    parser.add_argument("--axis-delta", type=float, default=0.04,
                        help="Balanced +/- motor speed delta for BF axis pair patterns")
    parser.add_argument("--pattern-set", choices=["motors", "axis", "all"], default="motors")
    parser.add_argument("--motor-map", type=parse_motor_map, default=parse_motor_map("bf-sitl"),
                        help="Logical motor index to packet index map. Use bf-sitl, identity, or e.g. 0,3,2,1")
    parser.add_argument("--settle-seconds", type=float, default=0.5)
    parser.add_argument("--pulse-seconds", type=float, default=0.5)
    parser.add_argument("--sample-timeout", type=float, default=2.0)
    parser.add_argument("--startup-seconds", type=float, default=5.0)
    parser.add_argument("--no-start-gazebo", action="store_true")
    parser.add_argument("--keep-process-logs", action="store_true")
    parser.add_argument("--log-dir", default=os.environ.get("KENET_SITL_LOG_DIR"))
    parser.add_argument("--log-file", default=None)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.hz <= 0:
        raise SystemExit("--hz must be positive")
    if args.settle_seconds < 0 or args.pulse_seconds <= 0:
        raise SystemExit("--settle-seconds must be non-negative and --pulse-seconds must be positive")
    if not 0.0 <= args.base_speed <= 1.0:
        raise SystemExit("--base-speed must be between 0 and 1")
    if not 0.0 <= args.pulse_speed <= 1.0:
        raise SystemExit("--pulse-speed must be between 0 and 1")
    if args.axis_delta < 0:
        raise SystemExit("--axis-delta must be non-negative")

    env = os.environ.copy()
    env.setdefault("FPV_ROOT", str(REPO_ROOT))
    env.setdefault("AEROLOOP_GAZEBO", str(REPO_ROOT / "../aeroloop_gazebo"))
    env.setdefault("BETAFLIGHT_ROOT", str(REPO_ROOT / "../betaflight"))

    log_dir = resolve_log_dir(args.log_dir, REPO_ROOT)
    log_path = Path(args.log_file) if args.log_file else make_log_path(log_dir, "motor-moment")
    process_dir = Path(tempfile.mkdtemp(prefix="kenet-motor-moment-"))
    gazebo_proc: subprocess.Popen[str] | None = None
    failed = False

    metadata = {
        "tool": "gazebo_motor_moment_probe",
        "world": args.world,
        "world_name": args.world_name,
        "entity": args.entity,
        "base_speed": args.base_speed,
        "pulse_speed": args.pulse_speed,
        "axis_delta": args.axis_delta,
        "pattern_set": args.pattern_set,
        "motor_map": args.motor_map,
        "settle_seconds": args.settle_seconds,
        "pulse_seconds": args.pulse_seconds,
        "direct_motor_port": args.port,
    }

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

        imu_topic = choose_imu_topic(args.world_name, args.sample_timeout)
        metadata["imu_topic"] = imu_topic
        results: list[dict[str, Any]] = []
        with JsonlLogger(log_path, metadata=metadata) as logger:
            for name, logical_speeds in build_patterns(args):
                speeds = remap_speeds(logical_speeds, args.motor_map)
                reset_ok = reset_world(args.world_name, args.sample_timeout)
                if args.settle_seconds:
                    send_motor_speeds([0.0] * 4, args.settle_seconds, args.host, args.port, args.hz)
                before_pose = sample_pose(args.world_name, args.entity, args.sample_timeout)
                before_imu = sample_imu(imu_topic, args.sample_timeout)
                sent_packets = send_motor_speeds(speeds, args.pulse_seconds, args.host, args.port, args.hz)
                after_pose = sample_pose(args.world_name, args.entity, args.sample_timeout)
                after_imu = sample_imu(imu_topic, args.sample_timeout)
                send_motor_speeds([0.0] * 4, 0.2, args.host, args.port, args.hz)
                record = {
                    "name": name,
                    "logical_speeds": logical_speeds,
                    "speeds": speeds,
                    "motor_map": args.motor_map,
                    "sent_packets": sent_packets,
                    "reset_ok": reset_ok,
                    "before_pose": before_pose,
                    "after_pose": after_pose,
                    "delta_pose_deg": delta_pose(before_pose, after_pose),
                    "before_imu": before_imu,
                    "after_imu": after_imu,
                    "delta_imu_angular_velocity": delta_vector(before_imu, after_imu, "angular_velocity"),
                    "delta_imu_linear_acceleration": delta_vector(before_imu, after_imu, "linear_acceleration"),
                }
                logger.write("motor_moment_sample", **record)
                results.append(record)
                delta = record["delta_pose_deg"]
                print(
                    "%s speeds=%s packets=%d reset=%s delta_roll=%s delta_pitch=%s delta_yaw=%s imu_domega_z=%s" %
                    (
                        name,
                        ",".join("%.2f" % value for value in speeds),
                        sent_packets,
                        reset_ok,
                        _fmt(delta.get("roll")),
                        _fmt(delta.get("pitch")),
                        _fmt(delta.get("yaw")),
                        _fmt(record["delta_imu_angular_velocity"].get("z")),
                    )
                )

        print("log=%s" % log_path)
        return 0
    except Exception as exc:
        failed = True
        print("result=FAIL %s" % exc, file=sys.stderr)
        print("process_logs=%s" % process_dir, file=sys.stderr)
        return 1
    finally:
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
