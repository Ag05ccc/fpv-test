#!/usr/bin/env python3
"""
Run a small Gazebo + Betaflight SITL motor-output smoke test.

The test starts Gazebo headless, starts Betaflight SITL, sends synthetic RC
packets with a safe arming sequence, checks MSP_MOTOR, and samples Gazebo
dynamic pose before and after throttle to confirm rotor joints moved.
"""

from __future__ import annotations

import argparse
import os
import re
import shutil
import socket
import struct
import subprocess
import sys
import tempfile
import time
from pathlib import Path

from sitl_msp import MSP_MOTOR, MSP_STATUS_EX, msp_request, parse_status_ex


class SmokeError(RuntimeError):
    """Expected smoke-test failure."""


def pack_rc(channels: list[int]) -> bytes:
    return struct.pack("<d16H", time.time(), *channels)


def send_rc(channels: list[int], seconds: float, host: str, port: int, hz: float) -> int:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    deadline = time.monotonic() + seconds
    period = 1.0 / hz
    count = 0
    try:
        while time.monotonic() < deadline:
            sock.sendto(pack_rc(channels), (host, port))
            count += 1
            time.sleep(period)
    finally:
        sock.close()
    return count


def wait_for_tcp(host: str, port: int, timeout: float) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            with socket.create_connection((host, port), timeout=0.5):
                return
        except OSError:
            time.sleep(0.2)
    raise SmokeError("timeout waiting for TCP %s:%d" % (host, port))


def run_command(command: list[str], timeout: float) -> str:
    proc = subprocess.run(
        command,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        timeout=timeout,
        check=False,
    )
    return proc.stdout


def sample_dynamic_pose(world_name: str, timeout: float) -> str:
    # `gz topic -n 1` may print a sample but not exit promptly on some builds.
    # Use the system timeout wrapper so partial stdout is still available.
    proc = subprocess.run(
        ["timeout", f"{timeout}s", "gz", "topic", "-e", "-t", f"/world/{world_name}/dynamic_pose/info", "-n", "1"],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        check=False,
    )
    return proc.stdout


def extract_orientation(text: str, entity_name: str) -> tuple[float | None, float | None]:
    marker = f'name: "{entity_name}"'
    start = text.find(marker)
    if start < 0:
        return None, None
    chunk = text[start:start + 700]
    orient_match = re.search(r"orientation\s*\{(?P<body>.*?)\n\}", chunk, re.S)
    if not orient_match:
        return None, None
    body = orient_match.group("body")
    z_match = re.search(r"\bz:\s*([^\n]+)", body)
    w_match = re.search(r"\bw:\s*([^\n]+)", body)
    z = float(z_match.group(1)) if z_match else None
    w = float(w_match.group(1)) if w_match else None
    return z, w


def terminate_process(proc: subprocess.Popen[str]) -> None:
    if proc.poll() is not None:
        return
    proc.terminate()
    try:
        proc.wait(timeout=3)
    except subprocess.TimeoutExpired:
        proc.kill()
        proc.wait(timeout=3)


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
        )
    finally:
        log_file.close()


def betaflight_work_dir(args: argparse.Namespace, fpv_root: Path, temp_dir: Path) -> Path:
    if args.betaflight_cwd == "repo":
        return fpv_root
    work_dir = temp_dir / "betaflight-cwd"
    work_dir.mkdir(parents=True, exist_ok=True)
    return work_dir


def evaluate_smoke_result(
    motors: tuple[int, ...],
    changed_rotors: list[str],
    *,
    require_rotor_motion: bool,
) -> str | None:
    if max(motors[:4]) <= 1000:
        raise SmokeError("motors did not rise above idle")
    if len(changed_rotors) < 4:
        message = "not all rotor joints moved: %s" % ",".join(changed_rotors)
        if require_rotor_motion:
            raise SmokeError(message)
        return message
    return None


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--world", default="test_betaflight.sdf")
    parser.add_argument("--world-name", default="betaflight_test")
    parser.add_argument("--rc-host", default="127.0.0.1")
    parser.add_argument("--rc-port", type=int, default=9004)
    parser.add_argument("--msp-host", default="127.0.0.1")
    parser.add_argument("--msp-port", type=int, default=5761)
    parser.add_argument("--low-seconds", type=float, default=10.0)
    parser.add_argument("--arm-seconds", type=float, default=3.0)
    parser.add_argument("--throttle-seconds", type=float, default=5.0)
    parser.add_argument("--throttle", type=int, default=1400)
    parser.add_argument("--rate-hz", type=float, default=50.0)
    parser.add_argument("--startup-timeout", type=float, default=12.0)
    parser.add_argument("--pose-timeout", type=float, default=8.0)
    parser.add_argument("--betaflight-cwd", choices=["temp", "repo"], default="temp",
                        help="Working directory for Betaflight SITL; temp avoids persistent eeprom.bin state")
    parser.add_argument("--require-rotor-motion", action="store_true",
                        help="Fail when Gazebo rotor joint pose samples do not all change")
    parser.add_argument("--keep-logs", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    fpv_root = Path(__file__).resolve().parents[1]
    betaflight_root = Path(os.environ.get("BETAFLIGHT_ROOT", fpv_root / "../betaflight"))
    betaflight_bin = betaflight_root / "obj/main/betaflight_SITL.elf"
    if not betaflight_bin.exists():
        raise SmokeError("Betaflight SITL binary not found: %s" % betaflight_bin)

    env = os.environ.copy()
    env.setdefault("FPV_ROOT", str(fpv_root))
    env.setdefault("AEROLOOP_GAZEBO", str(fpv_root / "../aeroloop_gazebo"))
    env.setdefault("BETAFLIGHT_ROOT", str(betaflight_root))

    temp_dir = Path(tempfile.mkdtemp(prefix="kenet-gazebo-smoke-"))
    gazebo_log = temp_dir / "gazebo.log"
    betaflight_log = temp_dir / "betaflight.log"
    betaflight_cwd = betaflight_work_dir(args, fpv_root, temp_dir)
    gazebo_proc: subprocess.Popen[str] | None = None
    betaflight_proc: subprocess.Popen[str] | None = None
    failed = False

    try:
        print("run_dir=%s" % temp_dir)
        print("betaflight_cwd=%s" % betaflight_cwd)
        print("eeprom_path=%s" % (betaflight_cwd / "eeprom.bin"))
        gazebo_proc = start_process(
            [str(fpv_root / "tools/run_gazebo_betaflight.sh"), "--world", args.world, "--headless"],
            gazebo_log,
            fpv_root,
            env,
        )
        time.sleep(5.0)
        if gazebo_proc.poll() is not None:
            raise SmokeError("Gazebo exited early; see %s" % gazebo_log)

        before_pose = sample_dynamic_pose(args.world_name, timeout=args.pose_timeout)

        betaflight_proc = start_process([str(betaflight_bin)], betaflight_log, betaflight_cwd, env)
        wait_for_tcp(args.msp_host, args.msp_port, args.startup_timeout)

        low = [1500] * 16
        low[2] = 1000
        low[4] = 1000
        armed = low[:]
        armed[4] = 2000
        throttle = armed[:]
        throttle[2] = args.throttle

        low_packets = 0
        deadline = time.monotonic() + args.low_seconds
        flags = 0
        active: list[str] = []
        while time.monotonic() < deadline:
            low_packets += send_rc(low, 1.0, args.rc_host, args.rc_port, args.rate_hz)
            status = parse_status_ex(msp_request(args.msp_host, args.msp_port, MSP_STATUS_EX, 2.0))
            flags = int(status.get("arming_disable_flags") or 0)
            active = list(status.get("arming_disable_names") or [])
            if not active:
                break
        print("low_boot_packets=%d" % low_packets)
        print("flags_after_low=0x%08x %s" % (flags, ",".join(active) if active else "none"))
        if active:
            raise SmokeError("arming flags did not clear while AUX low: %s" % ",".join(active))

        print("arm_packets=%d" % send_rc(armed, args.arm_seconds, args.rc_host, args.rc_port, args.rate_hz))
        status = parse_status_ex(msp_request(args.msp_host, args.msp_port, MSP_STATUS_EX, 2.0))
        flags = int(status.get("arming_disable_flags") or 0)
        active = list(status.get("arming_disable_names") or [])
        print("flags_after_arm=0x%08x %s" % (flags, ",".join(active) if active else "none"))

        print("throttle_packets=%d" % send_rc(throttle, args.throttle_seconds, args.rc_host, args.rc_port, args.rate_hz))
        status = parse_status_ex(msp_request(args.msp_host, args.msp_port, MSP_STATUS_EX, 2.0))
        flags = int(status.get("arming_disable_flags") or 0)
        active = list(status.get("arming_disable_names") or [])
        print("flags_after_throttle=0x%08x %s" % (flags, ",".join(active) if active else "none"))

        motor_payload = msp_request(args.msp_host, args.msp_port, MSP_MOTOR, 2.0)
        motors = struct.unpack("<%dH" % (len(motor_payload) // 2), motor_payload)
        print("msp_motor=%s" % ",".join(str(value) for value in motors[:4]))

        after_pose = sample_dynamic_pose(args.world_name, timeout=args.pose_timeout)
        changed_rotors = []
        for rotor in ("rotor_0", "rotor_1", "rotor_2", "rotor_3"):
            before_z, before_w = extract_orientation(before_pose, rotor)
            after_z, after_w = extract_orientation(after_pose, rotor)
            print(
                "%s_before=(z=%s,w=%s) after=(z=%s,w=%s)"
                % (rotor, before_z, before_w, after_z, after_w)
            )
            if before_z is not None and after_z is not None and abs(after_z - before_z) > 0.01:
                changed_rotors.append(rotor)

        rotor_warning = evaluate_smoke_result(
            motors[:4],
            changed_rotors,
            require_rotor_motion=args.require_rotor_motion,
        )
        if rotor_warning:
            print("rotor_motion_warning=%s" % rotor_warning)
        else:
            print("rotor_motion=all")

        print("result=PASS motor output rose above idle")
        if args.keep_logs:
            print("logs=%s" % temp_dir)
        return 0
    except Exception as exc:
        failed = True
        print("result=FAIL %s" % exc, file=sys.stderr)
        print("logs=%s" % temp_dir, file=sys.stderr)
        return 1
    finally:
        if betaflight_proc is not None:
            terminate_process(betaflight_proc)
        if gazebo_proc is not None:
            terminate_process(gazebo_proc)
        if not args.keep_logs and not failed:
            shutil.rmtree(temp_dir, ignore_errors=True)


if __name__ == "__main__":
    raise SystemExit(main())
