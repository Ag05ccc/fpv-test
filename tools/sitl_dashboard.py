#!/usr/bin/env python3
"""
Local web dashboard for Kenet + Betaflight SITL state.

It reads the joystick mapping used by sitl_rc_bridge.py and polls Betaflight SITL
over MSP/TCP. The dashboard is read-only: it does not send RC commands.
"""

from __future__ import annotations

import argparse
import json
import os
import signal
import socket
import subprocess
import sys
import threading
import time
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlparse


SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from sitl_rc_bridge import CHANNEL_MAP, LinuxJoystick, apply_forced_mode_pwm, make_channels
from sitl_rc_channels import PILOT_TO_MSP_INDEX, channel_label
from sitl_log import JsonlLogger, make_log_path, resolve_log_dir, timestamp_slug
from sitl_msp import (
    MSP_ADVANCED_CONFIG,
    MSP_ATTITUDE,
    MSP_BOXIDS,
    MSP_BOXNAMES,
    MSP_DEBUG,
    MSP_MOTOR,
    MSP_RC,
    MSP_STATUS_EX,
    msp_request_many,
    parse_advanced_config,
    parse_attitude,
    parse_box_ids,
    parse_box_names,
    parse_debug,
    parse_status_ex,
    parse_u16_list,
)

CHANNEL_ROWS = [
    {
        "channel": index + 1,
        "label": channel_label(index),
        "pilot_index": index,
        "fc_index": PILOT_TO_MSP_INDEX.get(index, index),
    }
    for index in range(8)
]

def clamp(value: int, low: int, high: int) -> int:
    return max(low, min(high, int(value)))


def three_position(value: int) -> str:
    if value < 1300:
        return "LOW"
    if value > 1700:
        return "HIGH"
    return "MID"


def kenet_state(value: int) -> str:
    if value >= 1700:
        return "TRACKING"
    if value >= 1300:
        return "AI-ARMED"
    return "IDLE"


def arm_command(value: int) -> str:
    return "ARM HIGH" if value >= 1700 else "ARM LOW"


def command_status(command: list[str]) -> str:
    try:
        proc = subprocess.run(
            command,
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            text=True,
            timeout=0.8,
            check=False,
        )
    except Exception:
        return "unknown"
    return "ok" if proc.returncode == 0 and proc.stdout.strip() else "missing"


def tail_text(path: Path, max_bytes: int = 2500) -> str:
    if not path.exists():
        return ""
    try:
        with path.open("rb") as handle:
            handle.seek(0, os.SEEK_END)
            size = handle.tell()
            handle.seek(max(0, size - max_bytes), os.SEEK_SET)
            data = handle.read()
    except OSError as exc:
        return "failed to read log: %s" % exc
    return data.decode("utf-8", errors="replace")


def betaflight_work_dir(
    args: argparse.Namespace,
    repo_root: Path,
    log_dir: Path,
    session_id: str,
    start_count: int,
    *,
    create: bool = False,
) -> Path:
    if args.betaflight_cwd == "repo":
        return repo_root
    work_dir = log_dir / ("%s-betaflight-cwd-%d" % (session_id, start_count))
    if create:
        work_dir.mkdir(parents=True, exist_ok=True)
    return work_dir


def is_loopback_client(host: str) -> bool:
    try:
        ip = socket.gethostbyname(host)
    except OSError:
        return False
    return ip.startswith("127.") or ip == "::1"


RC_SENDER_SCRIPTS = (
    "kenet_sitl_mixer.py",
    "sitl_rc_bridge.py",
    "sitl_virtual_rc.py",
)


def list_active_rc_senders(current_pid: int | None = None) -> list[dict[str, str | int]]:
    current_pid = os.getpid() if current_pid is None else current_pid
    try:
        proc = subprocess.run(
            ["ps", "-eo", "pid=,args="],
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            text=True,
            timeout=0.8,
            check=False,
        )
    except Exception:
        return []
    senders: list[dict[str, str | int]] = []
    for line in proc.stdout.splitlines():
        stripped = line.strip()
        if not stripped:
            continue
        pid_text, _space, command = stripped.partition(" ")
        try:
            pid = int(pid_text)
        except ValueError:
            continue
        if pid == current_pid:
            continue
        if "--send" not in command:
            continue
        if not any(script in command for script in RC_SENDER_SCRIPTS):
            continue
        senders.append({"pid": pid, "command": command})
    return senders


class ProcessManager:
    def __init__(self, args: argparse.Namespace, repo_root: Path):
        self.args = args
        self.repo_root = repo_root
        self.lock = threading.Lock()
        self.processes: dict[str, subprocess.Popen[str]] = {}
        self.log_dir = resolve_log_dir(args.log_dir, repo_root)
        self.log_dir.mkdir(parents=True, exist_ok=True)
        self.session_id = timestamp_slug()
        self.start_counts = {"gazebo": 0, "betaflight": 0, "kenet": 0}
        self.log_paths: dict[str, Path] = {}

    def start(self, name: str) -> dict:
        with self.lock:
            proc = self.processes.get(name)
            if proc is not None and proc.poll() is None:
                return {"ok": True, "message": "%s already running" % name}
            external_message = self._external_running_message(name)
            if external_message:
                return {"ok": True, "message": external_message}
            log_path = self._new_log_path(name)
            command = self._command_for(name)
            work_dir = self._work_dir_for(name, create=True)
            log_file = log_path.open("w", encoding="utf-8", errors="replace")
            try:
                proc = subprocess.Popen(
                    command,
                    cwd=str(work_dir),
                    env=self._env(),
                    stdout=log_file,
                    stderr=subprocess.STDOUT,
                    text=True,
                    start_new_session=True,
                )
            finally:
                log_file.close()
            self.processes[name] = proc
            self.log_paths[name] = log_path
            return {
                "ok": True,
                "message": "started %s pid=%d" % (name, proc.pid),
                "pid": proc.pid,
                "command": command,
                "working_dir": str(work_dir),
                "eeprom_path": str(work_dir / "eeprom.bin") if name == "betaflight" else None,
                "log_path": str(log_path),
            }

    def stop(self, name: str) -> dict:
        with self.lock:
            proc = self.processes.get(name)
            if proc is None:
                return {"ok": True, "message": "%s not started by dashboard" % name}
            if proc.poll() is not None:
                return {
                    "ok": True,
                    "message": "%s already exited rc=%s" % (name, proc.returncode),
                }
            try:
                os.killpg(proc.pid, signal.SIGTERM)
            except ProcessLookupError:
                return {"ok": True, "message": "%s process group already gone" % name}
        deadline = time.monotonic() + 4.0
        while time.monotonic() < deadline:
            if proc.poll() is not None:
                return {"ok": True, "message": "stopped %s" % name}
            time.sleep(0.1)
        try:
            os.killpg(proc.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        return {"ok": True, "message": "killed %s" % name}

    def stop_all(self) -> None:
        for name in ("kenet", "betaflight", "gazebo"):
            self.stop(name)

    def handle_action(self, action: str) -> dict:
        starts = {
            "start_gazebo": "gazebo",
            "start_betaflight": "betaflight",
            "start_kenet": "kenet",
        }
        stops = {
            "stop_gazebo": "gazebo",
            "stop_betaflight": "betaflight",
            "stop_kenet": "kenet",
        }
        if action in starts:
            return self.start(starts[action])
        if action in stops:
            return self.stop(stops[action])
        return {"ok": False, "message": "unknown action: %s" % action}

    def snapshot(self) -> dict:
        with self.lock:
            names = ("gazebo", "betaflight", "kenet")
            result = {}
            for name in names:
                proc = self.processes.get(name)
                running = proc is not None and proc.poll() is None
                returncode = None if proc is None else proc.poll()
                result[name] = {
                    "running": running,
                    "pid": None if proc is None else proc.pid,
                    "returncode": returncode,
                    "log_path": str(self._log_path(name)),
                    "log_tail": tail_text(self._log_path(name)),
                    "command": self._command_for(name),
                    "working_dir": str(self._work_dir_for(name, create=False)),
                    "eeprom_path": str(self._work_dir_for(name, create=False) / "eeprom.bin") if name == "betaflight" else None,
                }
            return result

    def _new_log_path(self, name: str) -> Path:
        self.start_counts[name] = self.start_counts.get(name, 0) + 1
        return self.log_dir / ("%s-%s-%d.log" % (
            self.session_id,
            name,
            self.start_counts[name],
        ))

    def _log_path(self, name: str) -> Path:
        return self.log_paths.get(name, self.log_dir / ("%s-%s.log" % (self.session_id, name)))

    def _work_dir_for(self, name: str, *, create: bool = False) -> Path:
        if name == "betaflight":
            start_count = max(1, self.start_counts.get(name, 0))
            return betaflight_work_dir(
                self.args,
                self.repo_root,
                self.log_dir,
                self.session_id,
                start_count,
                create=create,
            )
        return self.repo_root

    def _env(self) -> dict[str, str]:
        env = os.environ.copy()
        env.setdefault("FPV_ROOT", str(self.repo_root))
        env.setdefault("AEROLOOP_GAZEBO", str(self.repo_root / "../aeroloop_gazebo"))
        env.setdefault("BETAFLIGHT_ROOT", str(self.repo_root / "../betaflight"))
        env.setdefault("JOY_DEV", self.args.device)
        env.setdefault("KENET_SITL_LOG_DIR", str(self.log_dir))
        return env

    def _external_running_message(self, name: str) -> str | None:
        if name == "gazebo" and command_status(["gz", "topic", "-l"]) == "ok":
            return "Gazebo already appears active; not starting a duplicate"
        if name == "betaflight":
            try:
                with socket.create_connection((self.args.msp_host, self.args.msp_port), timeout=0.3):
                    return "Betaflight SITL TCP is already active; not starting a duplicate"
            except OSError:
                return None
        if name == "kenet":
            senders = list_active_rc_senders()
            if senders:
                summary = "; ".join("%s:%s" % (sender["pid"], sender["command"]) for sender in senders[:3])
                return "RC sender already active; not starting duplicate Kenet sender: %s" % summary
        return None

    def _betaflight_bin(self) -> str:
        root = Path(os.environ.get("BETAFLIGHT_ROOT", self.repo_root / "../betaflight"))
        return str(root / "obj/main/betaflight_SITL.elf")

    def _python_bin(self) -> str:
        project_python = self.repo_root / "fpv_env/bin/python"
        if project_python.exists():
            return str(project_python)
        return sys.executable

    def _command_for(self, name: str) -> list[str]:
        if name == "gazebo":
            command = [
                str(self.repo_root / "tools/run_gazebo_betaflight.sh"),
                "--world",
                self.args.gazebo_world,
            ]
            if self.args.gazebo_headless:
                command.append("--headless")
            if self.args.gazebo_max_step_size is not None:
                command.extend(["--max-step-size", str(self.args.gazebo_max_step_size)])
            if self.args.gazebo_fix_iris_imu_pose:
                command.append("--fix-iris-imu-pose")
            if self.args.gazebo_fix_iris_motor_map:
                command.append("--fix-iris-motor-map")
            return command
        if name == "betaflight":
            return [self._betaflight_bin()]
        if name == "kenet":
            command = [
                self._python_bin(),
                str(self.repo_root / "tools/kenet_sitl_mixer.py"),
                "--device",
                self.args.device,
                "--camera",
                self.args.kenet_camera,
                "--send",
                "--print-hz",
                str(self.args.kenet_print_hz),
                "--log-dir",
                str(self.log_dir),
                "--flight-log-hz",
                str(self.args.kenet_flight_log_hz),
            ]
            if self.args.force_mode_pwm is not None:
                command.extend(["--force-mode-pwm", str(self.args.force_mode_pwm)])
            if self.args.kenet_no_vision:
                command.append("--no-vision")
            if self.args.kenet_preview:
                command.append("--preview")
            return command
        raise ValueError("unknown process: %s" % name)


class DashboardSampler:
    def __init__(self, args: argparse.Namespace, process_manager: ProcessManager | None = None):
        self.args = args
        self.process_manager = process_manager
        self.log_dir = process_manager.log_dir if process_manager else resolve_log_dir(args.log_dir, REPO_ROOT)
        self.log_dir.mkdir(parents=True, exist_ok=True)
        self.flight_logger = self._create_flight_logger()
        self.joystick = LinuxJoystick(args.device)
        self.lock = threading.Lock()
        self.running = False
        self.thread = None
        self.started_at = time.monotonic()
        self.last_joystick_open_attempt = -9999.0
        self.last_msp_poll = 0.0
        self.last_gazebo_poll = 0.0
        self.last_process_poll = -9999.0
        self.last_flight_log = 0.0
        self.last_box_poll = -9999.0
        self.box_names: list[str] = []
        self.box_ids: list[int] = []
        self.snapshot_data = self._empty_snapshot()

    def start(self) -> None:
        self.running = True
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def stop(self) -> None:
        self.running = False
        if self.thread:
            self.thread.join(timeout=1.0)
        self.joystick.close()
        if self.flight_logger:
            self.flight_logger.close()
            self.flight_logger = None

    def snapshot(self) -> dict:
        with self.lock:
            return json.loads(json.dumps(self.snapshot_data))

    def sample_once(self) -> dict:
        now = time.monotonic()
        snapshot = self.snapshot()
        snapshot["time"] = time.time()
        snapshot["uptime"] = now - self.started_at
        self._sample_joystick(snapshot, now)
        self._sample_msp(snapshot, now)
        self._sample_links(snapshot)
        self._build_channel_rows(snapshot)
        self._sample_processes(snapshot)
        self._update_logs(snapshot)
        self._write_flight_log(snapshot, "dashboard_sample", now, force=True)
        with self.lock:
            self.snapshot_data = snapshot
        return self.snapshot()

    def mark_event(self, label: str = "manual") -> dict:
        snapshot = self.snapshot()
        self._write_flight_log(snapshot, "manual_marker", time.monotonic(), force=True, label=label)
        if self.flight_logger:
            return {
                "ok": True,
                "message": "marked %s in %s" % (label, self.flight_logger.path),
                "log_path": str(self.flight_logger.path),
            }
        return {"ok": False, "message": "flight logging is disabled"}

    def _empty_snapshot(self) -> dict:
        return {
            "time": time.time(),
            "uptime": 0.0,
            "joystick": {
                "device": self.args.device,
                "connected": False,
                "error": None,
                "axes": {},
                "buttons": {},
                "pilot_channels": [1500] * 16,
            },
            "kenet": {
                "state": "UNKNOWN",
                "aux_channel": self.args.kenet_ch + 1,
                "aux_value": None,
            },
            "autopilot_command": {
                "state": "UNKNOWN",
                "channel": self.args.arm_ch + 1,
                "value": None,
            },
            "autopilot_mode": {
                "state": "UNKNOWN",
                "channel": self.args.mode_ch + 1,
                "value": None,
            },
            "msp": {
                "host": self.args.msp_host,
                "port": self.args.msp_port,
                "connected": False,
                "error": None,
                "rc_channels": [],
                "motor": [],
                "attitude": None,
                "advanced_config": parse_advanced_config(b""),
                "debug_mode": None,
                "debug": [],
                "status": parse_status_ex(b""),
                "box_error": None,
                "last_update_age": None,
            },
            "links": {
                "gazebo": "unknown",
                "betaflight_tcp": "unknown",
            },
            "logs": {
                "dir": str(self.log_dir),
                "dashboard_flight_log": None if self.flight_logger is None else str(self.flight_logger.path),
            },
            "processes": {},
            "channels": [],
        }

    def _run(self) -> None:
        period = 1.0 / self.args.rate_hz
        while self.running:
            now = time.monotonic()
            snapshot = self.snapshot()
            snapshot["time"] = time.time()
            snapshot["uptime"] = now - self.started_at
            self._sample_joystick(snapshot, now)
            if now - self.last_msp_poll >= 1.0 / self.args.msp_hz:
                self._sample_msp(snapshot, now)
                self.last_msp_poll = now
            if now - self.last_gazebo_poll >= 1.0 / self.args.link_hz:
                self._sample_links(snapshot)
                self.last_gazebo_poll = now
            self._build_channel_rows(snapshot)
            if now - self.last_process_poll >= 1.0 / self.args.process_hz:
                self._sample_processes(snapshot)
                self.last_process_poll = now
            self._update_logs(snapshot)
            self._write_flight_log(snapshot, "dashboard_sample", now)
            with self.lock:
                self.snapshot_data = snapshot
            time.sleep(period)

    def _create_flight_logger(self) -> JsonlLogger | None:
        if self.args.no_flight_log:
            return None
        path = Path(self.args.flight_log) if self.args.flight_log else make_log_path(self.log_dir, "dashboard")
        max_bytes = None if self.args.flight_log_max_mb <= 0 else int(self.args.flight_log_max_mb * 1024 * 1024)
        return JsonlLogger(path, metadata={
            "tool": "sitl_dashboard",
            "device": self.args.device,
            "msp_host": self.args.msp_host,
            "msp_port": self.args.msp_port,
            "sample_hz": self.args.flight_log_hz,
            "gazebo_world": self.args.gazebo_world,
            "gazebo_headless": self.args.gazebo_headless,
            "gazebo_max_step_size": self.args.gazebo_max_step_size,
            "gazebo_fix_iris_motor_map": self.args.gazebo_fix_iris_motor_map,
            "betaflight_cwd": self.args.betaflight_cwd,
            "force_mode_pwm": self.args.force_mode_pwm,
        },
            flush_every=self.args.flight_log_flush_every,
            flush_interval=self.args.flight_log_flush_seconds,
            max_bytes=max_bytes,
        )

    def _update_logs(self, snapshot: dict) -> None:
        snapshot["logs"] = {
            "dir": str(self.log_dir),
            "dashboard_flight_log": None if self.flight_logger is None else str(self.flight_logger.path),
        }

    def _write_flight_log(self, snapshot: dict, event: str, now: float,
                          force: bool = False, label: str | None = None) -> None:
        if not self.flight_logger:
            return
        if not force and now - self.last_flight_log < 1.0 / self.args.flight_log_hz:
            return
        if event == "dashboard_sample":
            self.last_flight_log = now
        msp = snapshot.get("msp") or {}
        status = msp.get("status") or {}
        record = {
            "uptime": snapshot.get("uptime"),
            "joystick": {
                "device": snapshot.get("joystick", {}).get("device"),
                "connected": snapshot.get("joystick", {}).get("connected"),
                "error": snapshot.get("joystick", {}).get("error"),
                "axes": snapshot.get("joystick", {}).get("axes"),
                "buttons": snapshot.get("joystick", {}).get("buttons"),
                "pilot_channels": snapshot.get("joystick", {}).get("pilot_channels"),
            },
            "kenet": snapshot.get("kenet"),
            "autopilot_command": snapshot.get("autopilot_command"),
            "autopilot_mode": snapshot.get("autopilot_mode"),
            "msp": {
                "connected": msp.get("connected"),
                "error": msp.get("error"),
                "rc_channels": msp.get("rc_channels"),
                "motor": msp.get("motor"),
                "attitude": msp.get("attitude"),
                "box_error": msp.get("box_error"),
                "armed": status.get("armed"),
                "active_modes_valid": status.get("active_modes_valid"),
                "active_modes": status.get("active_modes"),
                "active_mode_ids": status.get("active_mode_ids"),
                "flight_mode_flags": status.get("flight_mode_flags"),
                "flight_mode_extra_flags": status.get("flight_mode_extra_flags"),
                "arming_disable_flags": status.get("arming_disable_flags"),
                "arming_disable_names": status.get("arming_disable_names"),
                "last_update_age": msp.get("last_update_age"),
            },
            "channels": [
                {
                    "channel": row.get("channel"),
                    "label": row.get("label"),
                    "pilot": row.get("pilot"),
                    "fc": row.get("fc"),
                    "delta": row.get("delta"),
                    "source": row.get("source"),
                }
                for row in snapshot.get("channels", [])
            ],
            "links": snapshot.get("links"),
        }
        if label is not None:
            record["label"] = label
        self.flight_logger.write(event, **record)

    def _sample_joystick(self, snapshot: dict, now: float) -> None:
        joystick = snapshot["joystick"]
        if self.joystick.fd is None:
            if now - self.last_joystick_open_attempt > 1.0:
                self.last_joystick_open_attempt = now
                try:
                    self.joystick.open()
                except OSError as exc:
                    joystick.update({
                        "connected": False,
                        "error": str(exc),
                        "axes": {},
                        "buttons": {},
                        "pilot_channels": [1500] * 16,
                    })
                    self._update_switch_states(snapshot, joystick["pilot_channels"], connected=False)
                    return
            else:
                joystick.update({
                    "connected": False,
                    "error": joystick.get("error") or "joystick not open",
                    "axes": {},
                    "buttons": {},
                    "pilot_channels": [1500] * 16,
                })
                self._update_switch_states(snapshot, joystick["pilot_channels"], connected=False)
                return
        try:
            self.joystick.poll(timeout=0)
            channels = make_channels(self.joystick, CHANNEL_MAP)
            apply_forced_mode_pwm(channels, self.args.force_mode_pwm)
            joystick.update({
                "connected": self.joystick.fd is not None,
                "error": None,
                "axes": {str(k): v for k, v in sorted(self.joystick.axes.items())},
                "buttons": {str(k): v for k, v in sorted(self.joystick.buttons.items())},
                "pilot_channels": channels,
            })
            self._update_switch_states(snapshot, channels, connected=True)
        except Exception as exc:
            self.joystick.close()
            joystick.update({
                "connected": False,
                "error": str(exc),
                "axes": {},
                "buttons": {},
                "pilot_channels": [1500] * 16,
            })
            self._update_switch_states(snapshot, joystick["pilot_channels"], connected=False)

    def _update_switch_states(self, snapshot: dict, channels: list[int], connected: bool) -> None:
        kenet_value = channels[self.args.kenet_ch] if 0 <= self.args.kenet_ch < len(channels) else 1500
        arm_value = channels[self.args.arm_ch] if 0 <= self.args.arm_ch < len(channels) else 1500
        mode_value = channels[self.args.mode_ch] if 0 <= self.args.mode_ch < len(channels) else 1500
        snapshot["kenet"].update({
            "state": kenet_state(kenet_value) if connected else "NO JOYSTICK",
            "aux_value": kenet_value,
        })
        snapshot["autopilot_command"].update({
            "state": arm_command(arm_value) if connected else "NO JOYSTICK",
            "value": arm_value,
        })
        snapshot["autopilot_mode"].update({
            "state": three_position(mode_value) if connected else "NO JOYSTICK",
            "value": mode_value,
        })

    def _sample_msp(self, snapshot: dict, now: float) -> None:
        msp = snapshot["msp"]
        try:
            if now - self.last_box_poll >= self.args.box_refresh_seconds:
                try:
                    self._sample_msp_boxes()
                    msp["box_error"] = None
                except Exception as exc:
                    msp["box_error"] = str(exc)
                self.last_box_poll = now
            responses = msp_request_many(
                self.args.msp_host,
                self.args.msp_port,
                [MSP_STATUS_EX, MSP_RC, MSP_MOTOR, MSP_ATTITUDE, MSP_ADVANCED_CONFIG, MSP_DEBUG],
                self.args.msp_timeout,
            )
            status_payload = responses[MSP_STATUS_EX]
            rc_payload = responses[MSP_RC]
            motor_payload = responses[MSP_MOTOR]
            advanced_config = parse_advanced_config(responses[MSP_ADVANCED_CONFIG])
            try:
                attitude = parse_attitude(responses[MSP_ATTITUDE])
            except Exception:
                attitude = None
            msp.update({
                "connected": True,
                "error": None,
                "rc_channels": parse_u16_list(rc_payload),
                "motor": parse_u16_list(motor_payload),
                "attitude": attitude,
                "advanced_config": advanced_config,
                "debug_mode": advanced_config.get("debug_mode"),
                "debug": parse_debug(responses[MSP_DEBUG]),
                "status": parse_status_ex(
                    status_payload,
                    box_names=self.box_names,
                    box_ids=self.box_ids,
                ),
                "last_update_age": 0.0,
                "last_update_time": now,
            })
        except Exception as exc:
            last_update_time = msp.get("last_update_time")
            age = None if last_update_time is None else now - last_update_time
            msp.update({
                "connected": False,
                "error": str(exc),
                "rc_channels": [],
                "motor": [],
                "attitude": None,
                "status": parse_status_ex(b""),
                "last_update_age": age,
            })

    def _sample_msp_boxes(self) -> None:
        responses = msp_request_many(
            self.args.msp_host,
            self.args.msp_port,
            [MSP_BOXNAMES, MSP_BOXIDS],
            self.args.msp_timeout,
        )
        self.box_names = parse_box_names(responses[MSP_BOXNAMES])
        self.box_ids = parse_box_ids(responses[MSP_BOXIDS])

    def _sample_links(self, snapshot: dict) -> None:
        snapshot["links"]["gazebo"] = command_status(["pgrep", "-f", "gz sim"])
        if (snapshot.get("msp") or {}).get("connected"):
            snapshot["links"]["betaflight_tcp"] = "ok"
            return
        try:
            with socket.create_connection((self.args.msp_host, self.args.msp_port), timeout=0.3):
                snapshot["links"]["betaflight_tcp"] = "ok"
        except OSError:
            snapshot["links"]["betaflight_tcp"] = "missing"

    def _sample_processes(self, snapshot: dict) -> None:
        if self.process_manager is None:
            snapshot["processes"] = {}
        else:
            snapshot["processes"] = self.process_manager.snapshot()

    def _build_channel_rows(self, snapshot: dict) -> None:
        pilot = snapshot["joystick"].get("pilot_channels") or []
        fc = snapshot["msp"].get("rc_channels") or []
        rows = []
        for index, row in enumerate(CHANNEL_ROWS):
            pilot_index = row["pilot_index"]
            fc_index = row["fc_index"]
            pilot_value = pilot[pilot_index] if pilot_index < len(pilot) else None
            fc_value = fc[fc_index] if fc_index < len(fc) else None
            delta = None if pilot_value is None or fc_value is None else fc_value - pilot_value
            if fc_value is None:
                source = "no FC"
            elif pilot_value is None:
                source = "FC"
            elif abs(delta) > self.args.delta_threshold:
                source = "mixer/FC"
            else:
                source = "pilot"
            rows.append({
                "index": index,
                "channel": row["channel"],
                "label": row["label"],
                "pilot_index": pilot_index,
                "fc_index": fc_index,
                "pilot": pilot_value,
                "fc": fc_value,
                "delta": delta,
                "source": source,
            })
        snapshot["channels"] = rows


INDEX_HTML = r"""<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Kenet SITL Dashboard</title>
  <style>
    :root {
      color-scheme: dark;
      --bg: #0f1115;
      --panel: #171b22;
      --panel-2: #1f2630;
      --text: #e6edf3;
      --muted: #8b949e;
      --line: #30363d;
      --blue: #58a6ff;
      --green: #3fb950;
      --yellow: #d29922;
      --red: #f85149;
      --cyan: #39c5cf;
    }
    * { box-sizing: border-box; }
    body {
      margin: 0;
      background: var(--bg);
      color: var(--text);
      font-family: Inter, ui-sans-serif, system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
      font-size: 14px;
      letter-spacing: 0;
    }
    main {
      max-width: 1420px;
      margin: 0 auto;
      padding: 18px;
    }
    header {
      display: flex;
      justify-content: space-between;
      gap: 16px;
      align-items: flex-start;
      margin-bottom: 14px;
    }
    h1 {
      margin: 0 0 4px;
      font-size: 22px;
      line-height: 1.2;
      font-weight: 700;
    }
    .sub {
      color: var(--muted);
      font-size: 13px;
    }
    .grid {
      display: grid;
      grid-template-columns: repeat(4, minmax(0, 1fr));
      gap: 12px;
    }
    .panel {
      background: var(--panel);
      border: 1px solid var(--line);
      border-radius: 6px;
      padding: 12px;
      min-width: 0;
    }
    .panel h2 {
      margin: 0 0 10px;
      font-size: 13px;
      color: var(--muted);
      font-weight: 700;
      text-transform: uppercase;
    }
    .status {
      min-height: 82px;
      display: flex;
      flex-direction: column;
      justify-content: space-between;
    }
    .status .value {
      font-size: 24px;
      line-height: 1.1;
      font-weight: 800;
      word-break: break-word;
    }
    .status .detail {
      color: var(--muted);
      margin-top: 8px;
      font-size: 12px;
    }
    .ok { color: var(--green); }
    .warn { color: var(--yellow); }
    .bad { color: var(--red); }
    .info { color: var(--blue); }
    .cyan { color: var(--cyan); }
    .wide {
      grid-column: span 2;
    }
    .full {
      grid-column: 1 / -1;
    }
    table {
      width: 100%;
      border-collapse: collapse;
      table-layout: fixed;
    }
    th, td {
      border-bottom: 1px solid var(--line);
      padding: 8px 6px;
      text-align: left;
      vertical-align: middle;
      white-space: nowrap;
      overflow: hidden;
      text-overflow: ellipsis;
    }
    th {
      color: var(--muted);
      font-weight: 700;
      font-size: 12px;
    }
    tr:last-child td { border-bottom: none; }
    .num { font-variant-numeric: tabular-nums; }
    .bar {
      height: 12px;
      width: 100%;
      min-width: 80px;
      background: #2b313b;
      border-radius: 4px;
      overflow: hidden;
      position: relative;
    }
    .bar > span {
      display: block;
      height: 100%;
      width: 0;
      background: var(--blue);
    }
    .bar.motor > span { background: var(--cyan); }
    .kv {
      display: grid;
      grid-template-columns: minmax(80px, 1fr) minmax(0, 2fr);
      gap: 6px 10px;
    }
    .kv .key {
      color: var(--muted);
    }
    .raw {
      display: grid;
      grid-template-columns: repeat(4, minmax(0, 1fr));
      gap: 8px;
    }
    .raw-item {
      background: var(--panel-2);
      border-radius: 5px;
      padding: 8px;
      min-height: 42px;
    }
    .raw-name {
      color: var(--muted);
      font-size: 12px;
    }
    .raw-value {
      margin-top: 2px;
      font-variant-numeric: tabular-nums;
      font-weight: 700;
    }
    .small {
      font-size: 12px;
      color: var(--muted);
    }
    .controls {
      display: grid;
      grid-template-columns: repeat(3, minmax(0, 1fr));
      gap: 10px;
    }
    .process-card {
      background: var(--panel-2);
      border: 1px solid var(--line);
      border-radius: 6px;
      padding: 10px;
      min-width: 0;
    }
    .process-head {
      display: flex;
      align-items: center;
      justify-content: space-between;
      gap: 8px;
      margin-bottom: 8px;
    }
    .process-title {
      font-weight: 800;
      min-width: 0;
      overflow: hidden;
      text-overflow: ellipsis;
      white-space: nowrap;
    }
    .badge {
      font-size: 12px;
      font-weight: 700;
      padding: 2px 7px;
      border-radius: 999px;
      background: #2b313b;
      color: var(--muted);
      white-space: nowrap;
    }
    .badge.running {
      background: rgba(63, 185, 80, 0.16);
      color: var(--green);
    }
    .actions {
      display: flex;
      gap: 8px;
      flex-wrap: wrap;
      margin-bottom: 8px;
    }
    button {
      border: 1px solid var(--line);
      border-radius: 5px;
      background: #263040;
      color: var(--text);
      padding: 7px 10px;
      font: inherit;
      font-weight: 700;
      min-height: 34px;
      cursor: pointer;
    }
    button:hover { background: #303b4d; }
    button.stop { background: #3a2528; }
    button.stop:hover { background: #4a2b31; }
    pre.log {
      margin: 0;
      max-height: 160px;
      min-height: 70px;
      overflow: auto;
      white-space: pre-wrap;
      word-break: break-word;
      background: #0b0d12;
      border: 1px solid var(--line);
      border-radius: 5px;
      padding: 8px;
      color: #c9d1d9;
      font-size: 12px;
      line-height: 1.35;
    }
    #actionStatus {
      margin-top: 10px;
      min-height: 18px;
    }
    @media (max-width: 1040px) {
      .grid { grid-template-columns: repeat(2, minmax(0, 1fr)); }
      .wide { grid-column: span 2; }
      .raw { grid-template-columns: repeat(2, minmax(0, 1fr)); }
      .controls { grid-template-columns: 1fr; }
    }
    @media (max-width: 640px) {
      main { padding: 12px; }
      header { flex-direction: column; }
      .grid { grid-template-columns: 1fr; }
      .wide, .full { grid-column: 1; }
      th, td { padding: 7px 4px; }
      .status .value { font-size: 20px; }
    }
  </style>
</head>
<body>
<main>
  <header>
    <div>
      <h1>Kenet SITL Dashboard</h1>
      <div class="sub" id="subtitle">waiting...</div>
    </div>
    <div class="sub" id="links">links...</div>
  </header>

  <section class="grid">
    <div class="panel full">
      <h2>Process Controls</h2>
      <div class="controls">
        <div class="process-card">
          <div class="process-head">
            <div class="process-title">Gazebo</div>
            <div class="badge" id="gazeboBadge">...</div>
          </div>
          <div class="actions">
            <button onclick="sendAction('start_gazebo')">Start Gazebo</button>
            <button class="stop" onclick="sendAction('stop_gazebo')">Stop</button>
          </div>
          <div class="small" id="gazeboCommand"></div>
          <pre class="log" id="gazeboLog"></pre>
        </div>
        <div class="process-card">
          <div class="process-head">
            <div class="process-title">Betaflight SITL</div>
            <div class="badge" id="betaflightBadge">...</div>
          </div>
          <div class="actions">
            <button onclick="sendAction('start_betaflight')">Start Betaflight SITL</button>
            <button class="stop" onclick="sendAction('stop_betaflight')">Stop</button>
          </div>
          <div class="small" id="betaflightCommand"></div>
          <pre class="log" id="betaflightLog"></pre>
        </div>
        <div class="process-card">
          <div class="process-head">
            <div class="process-title">Kenet Mixer</div>
            <div class="badge" id="kenetProcBadge">...</div>
          </div>
          <div class="actions">
            <button onclick="sendAction('start_kenet')">Start Kenet</button>
            <button class="stop" onclick="sendAction('stop_kenet')">Stop</button>
          </div>
          <div class="small" id="kenetCommand"></div>
          <pre class="log" id="kenetLog"></pre>
        </div>
      </div>
      <div class="actions" style="margin-top: 10px">
        <button onclick="sendAction('mark_event')">Mark Event</button>
      </div>
      <div class="small" id="logPaths"></div>
      <div class="small" id="actionStatus"></div>
    </div>

    <div class="panel status">
      <h2>Kenet</h2>
      <div class="value" id="kenetState">...</div>
      <div class="detail" id="kenetDetail"></div>
    </div>
    <div class="panel status">
      <h2>Arm Command</h2>
      <div class="value" id="armCommand">...</div>
      <div class="detail" id="armCommandDetail"></div>
    </div>
    <div class="panel status">
      <h2>Betaflight</h2>
      <div class="value" id="fcArmed">...</div>
      <div class="detail" id="fcDetail"></div>
    </div>
    <div class="panel status">
      <h2>Autopilot Mode</h2>
      <div class="value" id="apMode">...</div>
      <div class="detail" id="apModeDetail"></div>
    </div>

    <div class="panel full">
      <h2>RC Channels</h2>
      <table>
        <thead>
          <tr>
            <th style="width: 70px">CH</th>
            <th>Name</th>
            <th>Pilot</th>
            <th>FC</th>
            <th>Delta</th>
            <th>Source</th>
            <th style="width: 22%">Pilot Bar</th>
            <th style="width: 22%">FC Bar</th>
          </tr>
        </thead>
        <tbody id="channelRows"></tbody>
      </table>
    </div>

    <div class="panel wide">
      <h2>Motors</h2>
      <table>
        <tbody id="motorRows"></tbody>
      </table>
    </div>

    <div class="panel wide">
      <h2>MSP / Attitude</h2>
      <div class="kv" id="mspKv"></div>
    </div>

    <div class="panel wide">
      <h2>Raw Axes</h2>
      <div class="raw" id="axes"></div>
    </div>

    <div class="panel wide">
      <h2>Raw Buttons</h2>
      <div class="raw" id="buttons"></div>
    </div>
  </section>
</main>

<script>
const channelRows = document.getElementById('channelRows');
const motorRows = document.getElementById('motorRows');
const axesEl = document.getElementById('axes');
const buttonsEl = document.getElementById('buttons');
const mspKv = document.getElementById('mspKv');

function clsForState(text) {
  if (!text) return 'warn';
  if (text.includes('NO') || text.includes('FAIL')) return 'bad';
  if (text === 'LOW') return 'warn';
  if (text.includes('DISARMED') || text.includes('ARM LOW')) return 'warn';
  if (text.includes('TRACKING') || text.includes('ARMED') || text.includes('ARM HIGH')) return 'ok';
  if (text.includes('AI-ARMED') || text.includes('MID')) return 'warn';
  return 'info';
}

function bar(value, min=1000, max=2000, klass='') {
  const v = Number.isFinite(value) ? value : min;
  const pct = Math.max(0, Math.min(100, ((v - min) / (max - min)) * 100));
  return `<div class="bar ${klass}"><span style="width:${pct}%"></span></div>`;
}

function fmt(value) {
  if (value === null || value === undefined) return '-';
  if (typeof value === 'number') return Math.round(value * 100) / 100;
  return String(value);
}

function setStatus(id, text, detail) {
  const el = document.getElementById(id);
  el.className = 'value ' + clsForState(String(text));
  el.textContent = text || 'UNKNOWN';
  const detailEl = document.getElementById(id + 'Detail');
  if (detailEl) detailEl.textContent = detail || '';
}

function renderChannels(rows) {
  channelRows.innerHTML = rows.map(row => {
    const deltaClass = Math.abs(row.delta || 0) > 8 ? 'warn' : '';
    return `<tr>
      <td class="num">CH${row.channel}</td>
      <td>${row.label}</td>
      <td class="num">${fmt(row.pilot)}</td>
      <td class="num">${fmt(row.fc)}</td>
      <td class="num ${deltaClass}">${fmt(row.delta)}</td>
      <td>${row.source}</td>
      <td>${bar(row.pilot)}</td>
      <td>${bar(row.fc)}</td>
    </tr>`;
  }).join('');
}

function renderMotors(values) {
  const motors = values && values.length ? values.slice(0, 8) : [];
  motorRows.innerHTML = motors.map((value, index) => `<tr>
    <td style="width: 80px">M${index + 1}</td>
    <td class="num" style="width: 90px">${fmt(value)}</td>
    <td>${bar(value, 1000, 2000, 'motor')}</td>
  </tr>`).join('') || '<tr><td class="small">No motor data</td></tr>';
}

function renderRaw(el, values, prefix) {
  const keys = Object.keys(values || {}).sort((a, b) => Number(a) - Number(b));
  el.innerHTML = keys.map(key => `<div class="raw-item">
    <div class="raw-name">${prefix}${key}</div>
    <div class="raw-value">${values[key]}</div>
  </div>`).join('') || '<div class="small">No data</div>';
}

function renderKv(data) {
  const status = data.msp.status || {};
  const attitude = data.msp.attitude || {};
  const flags = status.arming_disable_names && status.arming_disable_names.length
    ? status.arming_disable_names.join(', ')
    : 'none';
  const activeModes = status.active_modes && status.active_modes.length
    ? status.active_modes.join(', ')
    : (status.active_modes_valid ? 'none' : 'unknown');
  const rows = [
    ['MSP', data.msp.connected ? 'connected' : 'offline'],
    ['MSP error', data.msp.error || '-'],
    ['Box decode', data.msp.box_error || (status.active_modes_valid ? 'ok' : 'unknown')],
    ['Active modes', activeModes],
    ['Armed', status.armed === null || status.armed === undefined ? 'unknown' : status.armed],
    ['Arming flags', flags],
    ['Flight flags', '0x' + (status.flight_mode_flags || 0).toString(16).padStart(8, '0')],
    ['Roll', attitude.roll ?? '-'],
    ['Pitch', attitude.pitch ?? '-'],
    ['Yaw', attitude.yaw ?? '-'],
  ];
  mspKv.innerHTML = rows.map(([k, v]) => `<div class="key">${k}</div><div>${fmt(v)}</div>`).join('');
}

function renderProcess(name, proc) {
  const badge = document.getElementById(`${name}Badge`) || document.getElementById(`${name}ProcBadge`);
  const command = document.getElementById(`${name}Command`);
  const log = document.getElementById(`${name}Log`);
  if (!proc) return;
  const status = proc.running ? `running pid ${proc.pid}` : (proc.pid ? `stopped rc ${proc.returncode}` : 'not started');
  if (badge) {
    badge.textContent = status;
    badge.className = 'badge' + (proc.running ? ' running' : '');
  }
  if (command) {
    const commandText = (proc.command || []).join(' ');
    command.textContent = proc.working_dir ? `${commandText} | cwd ${proc.working_dir}` : commandText;
  }
  if (log) {
    const text = proc.log_tail || '';
    log.textContent = text ? text.slice(-5000) : 'No log yet.';
    log.scrollTop = log.scrollHeight;
  }
}

function renderProcesses(data) {
  const procs = data.processes || {};
  renderProcess('gazebo', procs.gazebo);
  renderProcess('betaflight', procs.betaflight);
  renderProcess('kenet', procs.kenet);
}

async function sendAction(action) {
  const status = document.getElementById('actionStatus');
  status.textContent = `${action}...`;
  try {
    const response = await fetch('/api/action', {
      method: 'POST',
      headers: {'Content-Type': 'application/json'},
      body: JSON.stringify({action}),
    });
    const result = await response.json();
    status.className = 'small ' + (result.ok ? 'ok' : 'bad');
    status.textContent = result.message || JSON.stringify(result);
    await refresh();
  } catch (err) {
    status.className = 'small bad';
    status.textContent = String(err);
  }
}

let refreshInFlight = false;

async function refresh() {
  if (refreshInFlight) return;
  refreshInFlight = true;
  const started = performance.now();
  try {
    const response = await fetch('/api/state', {cache: 'no-store'});
    const data = await response.json();
    const uptime = Math.round(data.uptime || 0);
    const fetchMs = Math.round(performance.now() - started);
    const stateAgeMs = data.time ? Math.max(0, Math.round((Date.now() / 1000 - data.time) * 1000)) : null;
    document.getElementById('subtitle').textContent =
      `uptime ${uptime}s | joystick ${data.joystick.connected ? 'connected' : 'offline'} | API ${fetchMs}ms | age ${fmt(stateAgeMs)}ms | ${data.joystick.device}`;
    document.getElementById('links').textContent =
      `Gazebo ${data.links.gazebo} | Betaflight TCP ${data.links.betaflight_tcp}`;
    const logs = data.logs || {};
    document.getElementById('logPaths').textContent =
      `Log dir: ${logs.dir || '-'} | Dashboard JSONL: ${logs.dashboard_flight_log || 'disabled'}`;

    setStatus('kenetState', data.kenet.state, `CH${data.kenet.aux_channel} = ${fmt(data.kenet.aux_value)}`);
    setStatus('armCommand', data.autopilot_command.state, `CH${data.autopilot_command.channel} = ${fmt(data.autopilot_command.value)}`);
    setStatus('apMode', data.autopilot_mode.state, `CH${data.autopilot_mode.channel} = ${fmt(data.autopilot_mode.value)}`);

    const armed = data.msp.status && data.msp.status.armed;
    const flags = data.msp.status && data.msp.status.arming_disable_names || [];
    const fcState = data.msp.connected
      ? (armed === null || armed === undefined ? 'UNKNOWN' : (armed ? 'ARMED' : 'DISARMED'))
      : 'OFFLINE';
    const fcDetail = data.msp.connected
      ? (flags.length ? flags.join(', ') : 'arming flags clear')
      : (data.msp.error || 'MSP offline');
    setStatus('fcArmed', fcState, fcDetail);

    renderChannels(data.channels || []);
    renderMotors(data.msp.motor || []);
    renderRaw(axesEl, data.joystick.axes, 'A');
    renderRaw(buttonsEl, data.joystick.buttons, 'B');
    renderKv(data);
    renderProcesses(data);
  } catch (err) {
    document.getElementById('subtitle').textContent = 'dashboard fetch error: ' + err;
  } finally {
    refreshInFlight = false;
  }
}

refresh();
setInterval(refresh, 100);
</script>
</body>
</html>
"""


class DashboardHandler(BaseHTTPRequestHandler):
    sampler: DashboardSampler | None = None
    process_manager: ProcessManager | None = None
    allow_remote_control: bool = False

    def do_GET(self) -> None:
        path = urlparse(self.path).path
        if path == "/api/state":
            self._send_json(self.sampler.snapshot() if self.sampler else {})
            return
        if path in ("/", "/index.html"):
            self._send_html(INDEX_HTML)
            return
        self.send_error(404)

    def do_POST(self) -> None:
        path = urlparse(self.path).path
        if path != "/api/action":
            self.send_error(404)
            return
        length = int(self.headers.get("Content-Length", "0") or "0")
        body = self.rfile.read(length) if length else b"{}"
        try:
            payload = json.loads(body.decode("utf-8"))
            action = str(payload.get("action", ""))
            if action == "mark_event":
                if self.sampler is None:
                    result = {"ok": False, "message": "sampler is not available"}
                else:
                    result = self.sampler.mark_event(str(payload.get("label", "manual")))
            elif self.process_manager is None:
                result = {"ok": False, "message": "process manager is not available"}
            elif not self._process_control_allowed():
                result = {
                    "ok": False,
                    "message": "remote process control disabled; bind/use loopback or pass --allow-remote-control",
                }
            else:
                result = self.process_manager.handle_action(action)
        except Exception as exc:
            result = {"ok": False, "message": str(exc)}
        self._send_json(result)

    def _process_control_allowed(self) -> bool:
        return self.allow_remote_control or is_loopback_client(str(self.client_address[0]))

    def log_message(self, fmt: str, *args) -> None:
        return

    def _send_html(self, html: str) -> None:
        body = html.encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _send_json(self, data: dict) -> None:
        body = json.dumps(data, separators=(",", ":")).encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--device", default=os.environ.get("JOY_DEV", "/dev/input/js0"))
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument("--allow-remote-control", action="store_true",
                        help="Allow /api/action process start/stop from non-loopback clients")
    parser.add_argument("--msp-host", default="127.0.0.1")
    parser.add_argument("--msp-port", type=int, default=5761)
    parser.add_argument("--msp-timeout", type=float, default=0.5)
    parser.add_argument("--rate-hz", type=float, default=30.0)
    parser.add_argument("--msp-hz", type=float, default=2.0)
    parser.add_argument("--box-refresh-seconds", type=float, default=5.0,
                        help="Refresh MSP_BOXNAMES/MSP_BOXIDS metadata at this interval")
    parser.add_argument("--link-hz", type=float, default=1.0)
    parser.add_argument("--process-hz", type=float, default=0.5,
                        help="Refresh process status/log tails at this rate")
    parser.add_argument("--arm-ch", type=int, default=4, help="0-based ARM channel index")
    parser.add_argument("--kenet-ch", type=int, default=5, help="0-based Kenet state channel index")
    parser.add_argument("--mode-ch", type=int, default=6, help="0-based autopilot mode channel index")
    parser.add_argument("--force-mode-pwm", type=int, default=None,
                        help="Force CH7/AUX3 to this PWM value, e.g. 1500 for ANGLE mode tests")
    parser.add_argument("--delta-threshold", type=int, default=8)
    parser.add_argument("--gazebo-world", default="betaloop_iris_betaflight_demo_harmonic.sdf")
    parser.add_argument("--gazebo-headless", action="store_true")
    parser.add_argument("--gazebo-max-step-size", type=float, default=None,
                        help="Run Gazebo through a temporary world with this max_step_size")
    parser.add_argument("--gazebo-no-fix-iris-imu-pose", dest="gazebo_fix_iris_imu_pose",
                        action="store_false",
                        help="Disable the temporary Iris IMU pose correction used for Betaflight SITL")
    parser.add_argument("--gazebo-no-fix-iris-motor-map", dest="gazebo_fix_iris_motor_map",
                        action="store_false",
                        help="Disable the temporary Iris Betaflight motor mapping correction")
    parser.set_defaults(gazebo_fix_iris_imu_pose=True)
    parser.set_defaults(gazebo_fix_iris_motor_map=True)
    parser.add_argument("--betaflight-cwd", choices=["temp", "repo"], default="temp",
                        help="Working directory for dashboard-started Betaflight SITL; temp avoids persistent eeprom.bin state")
    parser.add_argument("--kenet-camera", default="test-2.mp4")
    parser.add_argument("--kenet-print-hz", type=float, default=5.0)
    parser.add_argument("--kenet-flight-log-hz", type=float, default=10.0)
    parser.add_argument("--kenet-no-vision", action="store_true")
    parser.add_argument("--kenet-preview", action="store_true")
    parser.add_argument("--log-dir", default=os.environ.get("KENET_SITL_LOG_DIR"),
                        help="Directory for process and JSONL logs; default logs/sitl")
    parser.add_argument("--flight-log", default=None,
                        help="Exact JSONL path for dashboard MSP/autopilot samples")
    parser.add_argument("--no-flight-log", action="store_true",
                        help="Disable dashboard MSP/autopilot JSONL logging")
    parser.add_argument("--flight-log-hz", type=float, default=5.0,
                        help="Dashboard MSP/autopilot JSONL sample rate")
    parser.add_argument("--flight-log-flush-every", type=int, default=10,
                        help="Flush dashboard JSONL logs every N records")
    parser.add_argument("--flight-log-flush-seconds", type=float, default=1.0,
                        help="Flush dashboard JSONL logs at least this often; 0 disables time-based flushing")
    parser.add_argument("--flight-log-max-mb", type=float, default=50.0,
                        help="Rotate dashboard JSONL logs after this many MiB; 0 disables rotation")
    parser.add_argument("--open", action="store_true")
    parser.add_argument("--once", action="store_true", help="Print one JSON snapshot and exit")
    args = parser.parse_args()
    if args.rate_hz <= 0 or args.msp_hz <= 0 or args.link_hz <= 0 or args.process_hz <= 0:
        parser.error("rates must be positive")
    if args.box_refresh_seconds <= 0:
        parser.error("--box-refresh-seconds must be positive")
    if args.kenet_print_hz <= 0:
        parser.error("--kenet-print-hz must be positive")
    if args.kenet_flight_log_hz <= 0 or args.flight_log_hz <= 0:
        parser.error("flight log rates must be positive")
    if args.flight_log_flush_every <= 0:
        parser.error("--flight-log-flush-every must be positive")
    if args.flight_log_flush_seconds < 0:
        parser.error("--flight-log-flush-seconds must be non-negative")
    if args.flight_log_max_mb < 0:
        parser.error("--flight-log-max-mb must be non-negative")
    if args.force_mode_pwm is not None and not 1000 <= args.force_mode_pwm <= 2000:
        parser.error("--force-mode-pwm must be between 1000 and 2000")
    if args.gazebo_max_step_size is not None and args.gazebo_max_step_size <= 0:
        parser.error("--gazebo-max-step-size must be positive")
    return args


def main() -> int:
    args = parse_args()
    repo_root = Path(__file__).resolve().parents[1]
    process_manager = ProcessManager(args, repo_root)
    sampler = DashboardSampler(args, process_manager)
    try:
        if args.once:
            print(json.dumps(sampler.sample_once(), indent=2, sort_keys=True))
            return 0

        sampler.start()
        DashboardHandler.sampler = sampler
        DashboardHandler.process_manager = process_manager
        DashboardHandler.allow_remote_control = args.allow_remote_control
        server = ThreadingHTTPServer((args.host, args.port), DashboardHandler)
        url = "http://%s:%d" % (args.host, args.port)
        print("Kenet SITL dashboard: %s" % url, flush=True)
        print("Close Betaflight Configurator if MSP data stays offline.", flush=True)
        if args.open:
            webbrowser.open(url)
        try:
            server.serve_forever()
        except KeyboardInterrupt:
            pass
        finally:
            server.server_close()
        return 0
    finally:
        sampler.stop()
        process_manager.stop_all()


if __name__ == "__main__":
    raise SystemExit(main())
