#!/usr/bin/env python3
"""Collect structural diagnostics for Kenet + Betaflight + Gazebo SITL."""

from __future__ import annotations

import argparse
import json
import os
import re
import socket
import statistics
import subprocess
import sys
import time
import urllib.error
import urllib.request
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any


SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from gazebo_stats_monitor import choose_stats_topic, parse_stats, read_stats_once
from gazebo_motor_moment_probe import choose_imu_topic, gz_topic_once, parse_imu_text, parse_pose_text
from sitl_log import JsonlLogger, make_log_path, resolve_log_dir
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
from sitl_rc_bridge import CHANNEL_MAP, LinuxJoystick, apply_forced_mode_pwm, make_channels
from sitl_rc_channels import PILOT_TO_MSP_INDEX, first_n_channel_labels
from sitl_virtual_rc import make_virtual_channels


PORTS = (8080, 5761, 6761, 9002, 9003, 9004)
PROCESS_NEEDLES = (
    "sitl_dashboard.py",
    "kenet_sitl_mixer.py",
    "sitl_rc_bridge.py",
    "betaflight_SITL",
    "gz sim",
    "ruby",
)
PROCESS_ENV_KEYS = (
    "SDF_PATH",
    "GZ_SIM_RESOURCE_PATH",
    "GZ_SIM_SYSTEM_PLUGIN_PATH",
)
FC_INDEX_BY_PILOT_INDEX = {index: PILOT_TO_MSP_INDEX.get(index, index) for index in range(8)}
PILOT_LABELS = tuple(first_n_channel_labels(8))
BETAFIGHT_EXPECTED_BY_MOTOR = {
    0: "rotor_3_joint",  # rear-right
    1: "rotor_0_joint",  # front-right
    2: "rotor_1_joint",  # rear-left
    3: "rotor_2_joint",  # front-left
}
SITL_PWM_SLOT_BY_BF_MOTOR = {
    0: 3,
    1: 0,
    2: 1,
    3: 2,
}


def run_text(command: list[str], timeout: float = 1.0) -> tuple[str, str, int | None, float]:
    started = time.monotonic()
    try:
        proc = subprocess.run(
            command,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            timeout=timeout,
            check=False,
        )
        return proc.stdout, proc.stderr, proc.returncode, (time.monotonic() - started) * 1000
    except subprocess.TimeoutExpired as exc:
        stdout = exc.stdout.decode("utf-8", errors="replace") if isinstance(exc.stdout, bytes) else (exc.stdout or "")
        stderr = exc.stderr.decode("utf-8", errors="replace") if isinstance(exc.stderr, bytes) else (exc.stderr or "")
        return stdout, stderr, None, (time.monotonic() - started) * 1000


def read_process_env(pid: int) -> dict[str, str]:
    try:
        raw = Path("/proc") / str(pid) / "environ"
        env_bytes = raw.read_bytes()
    except OSError:
        return {}
    env: dict[str, str] = {}
    for item in env_bytes.split(b"\0"):
        if b"=" not in item:
            continue
        key, value = item.split(b"=", 1)
        text_key = key.decode("utf-8", errors="replace")
        if text_key in PROCESS_ENV_KEYS:
            env[text_key] = value.decode("utf-8", errors="replace")
    return env


def sample_processes() -> list[dict[str, Any]]:
    stdout, _stderr, _rc, _ms = run_text(
        ["ps", "-eo", "pid,ppid,pgid,stat,pcpu,pmem,etime,cmd", "--no-headers"],
        timeout=1.0,
    )
    rows: list[dict[str, Any]] = []
    for line in stdout.splitlines():
        if not any(needle in line for needle in PROCESS_NEEDLES):
            continue
        parts = line.split(None, 7)
        if len(parts) < 8:
            continue
        pid, ppid, pgid, stat, pcpu, pmem, etime, cmd = parts
        row = {
            "pid": int(pid),
            "ppid": int(ppid),
            "pgid": int(pgid),
            "stat": stat,
            "cpu_percent": float(pcpu),
            "mem_percent": float(pmem),
            "etime": etime,
            "cmd": cmd,
        }
        if "gz sim" in cmd:
            row["env"] = read_process_env(int(pid))
        rows.append(row)
    return rows


def sample_ports() -> dict[str, list[str]]:
    stdout, _stderr, _rc, _ms = run_text(["ss", "-H", "-lntup"], timeout=1.0)
    result = {str(port): [] for port in PORTS}
    for line in stdout.splitlines():
        for port in PORTS:
            if re.search(r":%d\b" % port, line):
                result[str(port)].append(line)
    return result


def sample_dashboard(url: str, timeout: float) -> dict[str, Any]:
    started = time.monotonic()
    try:
        with urllib.request.urlopen(url, timeout=timeout) as response:
            body = response.read()
        elapsed_ms = (time.monotonic() - started) * 1000
        data = json.loads(body.decode("utf-8"))
        return {
            "ok": True,
            "elapsed_ms": elapsed_ms,
            "bytes": len(body),
            "state_time_age_ms": max(0.0, (time.time() - float(data.get("time", time.time()))) * 1000),
            "joystick_connected": (data.get("joystick") or {}).get("connected"),
            "msp_connected": (data.get("msp") or {}).get("connected"),
            "links": data.get("links"),
            "kenet": data.get("kenet"),
            "autopilot_command": data.get("autopilot_command"),
            "autopilot_mode": data.get("autopilot_mode"),
            "msp": data.get("msp"),
            "channels": data.get("channels"),
            "processes": data.get("processes"),
        }
    except (OSError, urllib.error.URLError, json.JSONDecodeError) as exc:
        return {
            "ok": False,
            "elapsed_ms": (time.monotonic() - started) * 1000,
            "error": str(exc),
        }


def sample_gazebo(topic: str, timeout: float) -> dict[str, Any]:
    started = time.monotonic()
    text = read_stats_once(topic, timeout)
    elapsed_ms = (time.monotonic() - started) * 1000
    stats = parse_stats(text)
    stats["topic"] = topic
    stats["elapsed_ms"] = elapsed_ms
    stats["ok"] = stats.get("rtf") is not None
    return stats


def sample_gazebo_pose(
    world_name: str,
    entity_name: str,
    imu_topic: str,
    timeout: float,
) -> dict[str, Any]:
    started = time.monotonic()
    pose_topic = f"/world/{world_name}/dynamic_pose/info"
    pose_text = gz_topic_once(pose_topic, timeout)
    imu_text = gz_topic_once(imu_topic, timeout)
    pose = parse_pose_text(pose_text, entity_name)
    imu = parse_imu_text(imu_text)
    return {
        "enabled": True,
        "elapsed_ms": (time.monotonic() - started) * 1000,
        "world_name": world_name,
        "entity": entity_name,
        "pose_topic": pose_topic,
        "imu_topic": imu_topic,
        "pose": pose,
        "imu": imu,
        "ok": pose is not None and bool(imu),
    }


def get_msp_box_metadata(host: str, port: int, timeout: float) -> tuple[list[str], list[int], str | None]:
    try:
        responses = msp_request_many(host, port, [MSP_BOXNAMES, MSP_BOXIDS], timeout)
        return parse_box_names(responses[MSP_BOXNAMES]), parse_box_ids(responses[MSP_BOXIDS]), None
    except Exception as exc:
        return [], [], str(exc)


def sample_msp(
    host: str,
    port: int,
    timeout: float,
    box_names: list[str],
    box_ids: list[int],
) -> dict[str, Any]:
    started = time.monotonic()
    try:
        responses = msp_request_many(
            host,
            port,
            [MSP_STATUS_EX, MSP_RC, MSP_MOTOR, MSP_ATTITUDE, MSP_ADVANCED_CONFIG, MSP_DEBUG],
            timeout,
        )
        elapsed_ms = (time.monotonic() - started) * 1000
        status = parse_status_ex(responses[MSP_STATUS_EX], box_names=box_names, box_ids=box_ids)
        advanced_config = parse_advanced_config(responses[MSP_ADVANCED_CONFIG])
        return {
            "connected": True,
            "elapsed_ms": elapsed_ms,
            "error": None,
            "status": status,
            "rc_channels": parse_u16_list(responses[MSP_RC]),
            "motor": parse_u16_list(responses[MSP_MOTOR]),
            "attitude": parse_attitude(responses[MSP_ATTITUDE]),
            "advanced_config": advanced_config,
            "debug_mode": advanced_config.get("debug_mode"),
            "debug": parse_debug(responses[MSP_DEBUG]),
        }
    except Exception as exc:
        return {
            "connected": False,
            "elapsed_ms": (time.monotonic() - started) * 1000,
            "error": str(exc),
            "status": parse_status_ex(b"", box_names=box_names, box_ids=box_ids),
            "rc_channels": [],
            "motor": [],
            "attitude": None,
            "advanced_config": parse_advanced_config(b""),
            "debug_mode": None,
            "debug": [],
        }


class JoystickSampler:
    def __init__(self, device: str, force_mode_pwm: int | None):
        self.device = device
        self.force_mode_pwm = force_mode_pwm
        self.joystick = LinuxJoystick(device)
        self.error: str | None = None
        try:
            self.joystick.open()
        except OSError as exc:
            self.error = str(exc)

    def close(self) -> None:
        self.joystick.close()

    def sample(self) -> dict[str, Any]:
        started = time.monotonic()
        if self.error:
            return {
                "connected": False,
                "elapsed_ms": (time.monotonic() - started) * 1000,
                "error": self.error,
                "axes": {},
                "buttons": {},
                "pilot_channels": [1500] * 16,
            }
        try:
            self.joystick.poll(timeout=0)
            channels = make_channels(self.joystick, CHANNEL_MAP)
            channels = apply_forced_mode_pwm(channels, self.force_mode_pwm)
            return {
                "connected": True,
                "elapsed_ms": (time.monotonic() - started) * 1000,
                "error": None,
                "axes": {str(k): v for k, v in sorted(self.joystick.axes.items())},
                "buttons": {str(k): v for k, v in sorted(self.joystick.buttons.items())},
                "pilot_channels": channels,
            }
        except Exception as exc:
            self.error = str(exc)
            self.joystick.close()
            return {
                "connected": False,
                "elapsed_ms": (time.monotonic() - started) * 1000,
                "error": self.error,
                "axes": {},
                "buttons": {},
                "pilot_channels": [1500] * 16,
            }


class VirtualRcSampler:
    def __init__(self, args: argparse.Namespace):
        self.args = args

    def close(self) -> None:
        pass

    def sample(self) -> dict[str, Any]:
        started = time.monotonic()
        mode_pwm = self.args.virtual_mode_pwm
        if self.args.force_mode_pwm is not None:
            mode_pwm = self.args.force_mode_pwm
        channels = make_virtual_channels(
            roll=self.args.virtual_roll,
            pitch=self.args.virtual_pitch,
            throttle=self.args.virtual_throttle,
            yaw=self.args.virtual_yaw,
            arm_pwm=self.args.virtual_arm_pwm,
            kenet_pwm=self.args.virtual_kenet_pwm,
            mode_pwm=mode_pwm,
        )
        return {
            "connected": True,
            "source": "virtual",
            "elapsed_ms": (time.monotonic() - started) * 1000,
            "error": None,
            "axes": {},
            "buttons": {},
            "pilot_channels": channels,
        }


class ExternalRcSampler:
    def close(self) -> None:
        pass

    def sample(self) -> dict[str, Any]:
        return {
            "connected": True,
            "source": "external",
            "elapsed_ms": 0.0,
            "error": None,
            "axes": {},
            "buttons": {},
            "pilot_channels": [],
        }


def channel_deltas(pilot: list[int], fc: list[int]) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for pilot_index in range(8):
        fc_index = FC_INDEX_BY_PILOT_INDEX[pilot_index]
        pilot_value = pilot[pilot_index] if pilot_index < len(pilot) else None
        fc_value = fc[fc_index] if fc_index < len(fc) else None
        rows.append({
            "channel": pilot_index + 1,
            "label": PILOT_LABELS[pilot_index],
            "pilot": pilot_value,
            "fc": fc_value,
            "delta": None if pilot_value is None or fc_value is None else fc_value - pilot_value,
        })
    return rows


def _text_or_none(elem: ET.Element, path: str) -> str | None:
    found = elem.find(path)
    if found is None or found.text is None:
        return None
    return found.text.strip()


def _rotor_location_from_pose(pose: str | None) -> str:
    if not pose:
        return "unknown"
    values = pose.split()
    if len(values) < 2:
        return "unknown"
    try:
        x = float(values[0])
        y = float(values[1])
    except ValueError:
        return "unknown"
    fore = "front" if x >= 0 else "rear"
    side = "left" if y >= 0 else "right"
    return "%s-%s" % (fore, side)


def analyze_motor_mapping_model(model_path: Path) -> dict[str, Any]:
    result: dict[str, Any] = {
        "model_path": str(model_path),
        "exists": model_path.exists(),
        "ok": False,
        "error": None,
        "rotor_links": {},
        "plugin_rotors": {},
        "bf_to_physical": {},
        "expected_identity_plugin_mapping": {},
        "double_remap_risk": None,
    }
    if not model_path.exists():
        result["error"] = "model.sdf not found"
        return result
    try:
        root = ET.parse(model_path).getroot()
    except ET.ParseError as exc:
        result["error"] = "xml parse failed: %s" % exc
        return result

    rotor_links: dict[str, dict[str, Any]] = {}
    for link in root.findall(".//link"):
        name = link.attrib.get("name", "")
        if not re.fullmatch(r"rotor_[0-9]+", name):
            continue
        pose = _text_or_none(link, "pose")
        rotor_links[name] = {
            "pose": pose,
            "location": _rotor_location_from_pose(pose),
        }
    plugin_rotors: dict[int, dict[str, Any]] = {}
    for plugin in root.findall(".//plugin"):
        if plugin.attrib.get("filename") != "BetaflightPlugin":
            continue
        for rotor in plugin.findall("rotor"):
            try:
                rotor_id = int(rotor.attrib.get("id", ""))
            except ValueError:
                continue
            joint = _text_or_none(rotor, "jointName")
            link = joint[:-6] if joint and joint.endswith("_joint") else None
            plugin_rotors[rotor_id] = {
                "jointName": joint,
                "turningDirection": _text_or_none(rotor, "turningDirection"),
                "link": link,
                "location": rotor_links.get(link or "", {}).get("location"),
            }

    bf_to_physical = {}
    mismatches = []
    for bf_motor, slot in SITL_PWM_SLOT_BY_BF_MOTOR.items():
        physical = plugin_rotors.get(slot, {})
        expected_joint = BETAFIGHT_EXPECTED_BY_MOTOR[bf_motor]
        actual_joint = physical.get("jointName")
        ok = actual_joint == expected_joint
        if not ok:
            mismatches.append(bf_motor)
        bf_to_physical[str(bf_motor)] = {
            "sitl_packet_slot": slot,
            "expected_joint_after_betaflight_remap": expected_joint,
            "actual_joint": actual_joint,
            "actual_location": physical.get("location"),
            "ok": ok,
        }

    expected_identity = {
        str(rotor_id): "rotor_%d_joint" % rotor_id
        for rotor_id in sorted(plugin_rotors)
    }
    plugin_identity = all(
        plugin_rotors[rotor_id].get("jointName") == "rotor_%d_joint" % rotor_id
        for rotor_id in plugin_rotors
    )
    result.update({
        "ok": not mismatches,
        "rotor_links": rotor_links,
        "plugin_rotors": {str(k): v for k, v in sorted(plugin_rotors.items())},
        "bf_to_physical": bf_to_physical,
        "expected_identity_plugin_mapping": expected_identity,
        "double_remap_risk": not plugin_identity,
        "mismatched_bf_motors": mismatches,
    })
    return result


def analyze_motor_mapping(aeroloop_gazebo: Path) -> dict[str, Any]:
    model_path = aeroloop_gazebo / "models" / "betaloop_iris_with_standoffs" / "model.sdf"
    return analyze_motor_mapping_model(model_path)


def runtime_model_paths_from_process(process: dict[str, Any]) -> list[Path]:
    env = process.get("env") or {}
    paths: list[Path] = []
    seen: set[Path] = set()
    for key in ("SDF_PATH", "GZ_SIM_RESOURCE_PATH"):
        for raw_path in str(env.get(key, "")).split(":"):
            if not raw_path:
                continue
            candidate = Path(raw_path) / "betaloop_iris_with_standoffs" / "model.sdf"
            if candidate in seen:
                continue
            seen.add(candidate)
            paths.append(candidate)
    return paths


def runtime_motor_fix_from_processes(processes: list[dict[str, Any]]) -> tuple[bool, str | None]:
    for process in processes:
        cmd = process.get("cmd") or ""
        if "--fix-iris-motor-map" in cmd:
            return True, None
        if "gz sim" not in cmd:
            continue
        for model_path in runtime_model_paths_from_process(process):
            if not model_path.exists():
                continue
            mapping = analyze_motor_mapping_model(model_path)
            if mapping.get("ok") and not mapping.get("double_remap_risk"):
                return True, str(model_path)
    return False, None


def summarize(records: list[dict[str, Any]], mapping: dict[str, Any]) -> dict[str, Any]:
    rtf = [record["gazebo"]["rtf"] for record in records if record.get("gazebo", {}).get("rtf") is not None]
    dashboard_ms = [record["dashboard"]["elapsed_ms"] for record in records if record.get("dashboard", {}).get("ok")]
    dashboard_age = [record["dashboard"]["state_time_age_ms"] for record in records if record.get("dashboard", {}).get("ok") and record["dashboard"].get("state_time_age_ms") is not None]
    msp_ms = [
        record["msp"]["elapsed_ms"]
        for record in records
        if record.get("msp", {}).get("connected") and isinstance(record["msp"].get("elapsed_ms"), (int, float))
    ]
    joystick_ms = [record["joystick"]["elapsed_ms"] for record in records if record.get("joystick", {}).get("connected")]
    attitude_roll = [
        abs(record["msp"]["attitude"]["roll"])
        for record in records
        if record.get("msp", {}).get("attitude") and record["msp"]["attitude"].get("roll") is not None
    ]
    attitude_pitch = [
        abs(record["msp"]["attitude"]["pitch"])
        for record in records
        if record.get("msp", {}).get("attitude") and record["msp"]["attitude"].get("pitch") is not None
    ]
    channel_delta_abs = [
        abs(row["delta"])
        for record in records
        for row in record.get("channels", [])
        if row.get("delta") is not None
    ]
    arming_flags = sorted({
        flag
        for record in records
        for flag in (record.get("msp", {}).get("status", {}).get("arming_disable_names") or [])
    })
    active_modes = sorted({
        mode
        for record in records
        for mode in (record.get("msp", {}).get("status", {}).get("active_modes") or [])
    })
    dashboard_runtime_fix_active = any(
        "--fix-iris-motor-map" in (record.get("dashboard", {}).get("processes", {}).get("gazebo", {}).get("command") or [])
        for record in records
    )
    process_runtime_fix_path = None
    process_runtime_fix_active = False
    for record in records:
        active, path = runtime_motor_fix_from_processes(record.get("processes", []))
        if active:
            process_runtime_fix_active = True
            process_runtime_fix_path = path
            break
    runtime_motor_fix_active = dashboard_runtime_fix_active or process_runtime_fix_active

    def stats(values: list[float]) -> dict[str, float | None]:
        if not values:
            return {"min": None, "mean": None, "max": None, "spread": None}
        return {
            "min": min(values),
            "mean": statistics.fmean(values),
            "max": max(values),
            "spread": max(values) - min(values),
        }

    warnings: list[str] = []
    rtf_stats = stats(rtf)
    if rtf_stats["mean"] is None:
        warnings.append("Gazebo stats okunamadı; Gazebo açık değil ya da topic seçilemedi.")
    elif rtf_stats["mean"] < 0.8:
        warnings.append("Gazebo real_time_factor ortalaması 0.8 altında; sim zamanlaması yavaş.")
    elif rtf_stats["spread"] is not None and rtf_stats["spread"] > 0.2:
        warnings.append("Gazebo real_time_factor dalgalanması yüksek; FC/sim zaman uyumu bozulabilir.")
    if dashboard_age and max(dashboard_age) > 800:
        warnings.append("Dashboard state yaşı 800 ms üstünde; ekrandaki değerler stale olabilir.")
    if dashboard_ms and max(dashboard_ms) > 250:
        warnings.append("Dashboard API yanıtı 250 ms üstünde; UI gecikmesi ölçüldü.")
    if not any(record.get("joystick", {}).get("connected") for record in records):
        warnings.append("Joystick bağlı görünmüyor; ARM ve RC komutları FC'ye gitmez.")
    if not any(record.get("msp", {}).get("connected") for record in records):
        warnings.append("MSP bağlı değil; Betaflight arm/mode/attitude doğrulanamıyor.")
    if "ANGLE" in arming_flags:
        warnings.append("Betaflight ANGLE arming blocker aktif; FC craft'ı dik/upright görmüyor olabilir.")
    if "THROTTLE" in arming_flags:
        warnings.append("Betaflight THROTTLE arming blocker aktif; gaz low değil ya da kalibrasyon hatalı.")
    if attitude_roll and max(attitude_roll) >= 60:
        warnings.append("Roll attitude 60 derece üstünde; flip/tumble veya orientation problemi var.")
    if attitude_pitch and max(attitude_pitch) >= 60:
        warnings.append("Pitch attitude 60 derece üstünde; flip/tumble veya orientation problemi var.")
    if channel_delta_abs and max(channel_delta_abs) > 25:
        warnings.append("Pilot RC ile FC RC arasında 25us üstü fark var; sender/mixer/FC mapping kontrol edilmeli.")
    if mapping.get("double_remap_risk") and not runtime_motor_fix_active:
        warnings.append("SDF BetaflightPlugin rotor id->joint eşlemesi identity değil; Betaflight SITL remap ile çift remap riski var.")
    if not mapping.get("ok") and mapping.get("mismatched_bf_motors") and not runtime_motor_fix_active:
        warnings.append("Motor mapping analizi BF motorlarının beklenen fiziksel rotorlara gitmediğini gösteriyor.")
    if runtime_motor_fix_active and not mapping.get("ok"):
        warnings.append("Source SDF motor mapping bozuk, ancak çalışan Gazebo komutunda --fix-iris-motor-map aktif görünüyor.")

    return {
        "samples": len(records),
        "rtf": rtf_stats,
        "dashboard_elapsed_ms": stats(dashboard_ms),
        "dashboard_state_age_ms": stats(dashboard_age),
        "msp_elapsed_ms": stats(msp_ms),
        "joystick_elapsed_ms": stats(joystick_ms),
        "max_abs_roll": max(attitude_roll) if attitude_roll else None,
        "max_abs_pitch": max(attitude_pitch) if attitude_pitch else None,
        "max_abs_channel_delta": max(channel_delta_abs) if channel_delta_abs else None,
        "active_modes": active_modes,
        "arming_disable_names": arming_flags,
        "runtime_motor_fix_active": runtime_motor_fix_active,
        "runtime_motor_fix_path": process_runtime_fix_path,
        "warnings": warnings,
    }


def print_summary(summary: dict[str, Any], log_path: Path, mapping: dict[str, Any]) -> None:
    print("SITL diagnostics log: %s" % log_path)
    print("samples: %d" % summary["samples"])
    print("Gazebo RTF: %s" % _format_stats(summary["rtf"]))
    print("Dashboard API ms: %s" % _format_stats(summary["dashboard_elapsed_ms"]))
    print("Dashboard state age ms: %s" % _format_stats(summary["dashboard_state_age_ms"]))
    print("MSP poll ms: %s" % _format_stats(summary["msp_elapsed_ms"]))
    print("RC source poll ms: %s" % _format_stats(summary["joystick_elapsed_ms"]))
    print("max attitude abs: roll=%s pitch=%s" % (
        _fmt(summary["max_abs_roll"]),
        _fmt(summary["max_abs_pitch"]),
    ))
    print("active modes: %s" % (", ".join(summary["active_modes"]) or "-"))
    print("arming blockers: %s" % (", ".join(summary["arming_disable_names"]) or "-"))
    print("max pilot/FC RC delta: %s" % _fmt(summary["max_abs_channel_delta"]))
    print("motor mapping ok: %s" % mapping.get("ok"))
    print("runtime motor map fix active: %s" % summary.get("runtime_motor_fix_active"))
    if summary.get("runtime_motor_fix_path"):
        print("runtime motor map model: %s" % summary["runtime_motor_fix_path"])
    if mapping.get("mismatched_bf_motors"):
        print("mismatched BF motors: %s" % ",".join(str(v) for v in mapping["mismatched_bf_motors"]))
    if summary["warnings"]:
        print("\nWarnings")
        for warning in summary["warnings"]:
            print("  - %s" % warning)
    else:
        print("\nWarnings: none")


def _fmt(value: Any) -> str:
    if value is None:
        return "-"
    if isinstance(value, float):
        return "%.3f" % value
    return str(value)


def _format_stats(values: dict[str, Any]) -> str:
    if values.get("mean") is None:
        return "-"
    return "min=%s mean=%s max=%s spread=%s" % (
        _fmt(values["min"]),
        _fmt(values["mean"]),
        _fmt(values["max"]),
        _fmt(values["spread"]),
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--samples", type=int, default=20)
    parser.add_argument("--interval", type=float, default=0.25)
    parser.add_argument("--rc-source", choices=["joystick", "virtual", "external"], default="joystick",
                        help="Expected pilot RC source for diagnostics")
    parser.add_argument("--device", default=os.environ.get("JOY_DEV", "/dev/input/js0"))
    parser.add_argument("--force-mode-pwm", type=int, default=None)
    parser.add_argument("--virtual-roll", type=int, default=1500)
    parser.add_argument("--virtual-pitch", type=int, default=1500)
    parser.add_argument("--virtual-throttle", type=int, default=1000)
    parser.add_argument("--virtual-yaw", type=int, default=1500)
    parser.add_argument("--virtual-arm-pwm", type=int, default=1000)
    parser.add_argument("--virtual-kenet-pwm", type=int, default=1000)
    parser.add_argument("--virtual-mode-pwm", type=int, default=1000)
    parser.add_argument("--msp-host", default="127.0.0.1")
    parser.add_argument("--msp-port", type=int, default=5761)
    parser.add_argument("--msp-timeout", type=float, default=0.5)
    parser.add_argument("--direct-msp", action="store_true",
                        help="Poll Betaflight MSP directly instead of reusing dashboard MSP snapshot")
    parser.add_argument("--dashboard-url", default="http://127.0.0.1:8080/api/state")
    parser.add_argument("--dashboard-timeout", type=float, default=0.4)
    parser.add_argument("--gazebo-topic", default="auto")
    parser.add_argument("--gazebo-timeout", type=float, default=0.8)
    parser.add_argument("--include-gazebo-pose", action="store_true",
                        help="Also log Gazebo dynamic pose and IMU orientation for frame/sign checks")
    parser.add_argument("--gazebo-world-name", default="betaloop_demo")
    parser.add_argument("--pose-entity", default="iris")
    parser.add_argument("--imu-topic", default="auto")
    parser.add_argument("--aeroloop-gazebo", default=os.environ.get("AEROLOOP_GAZEBO", "../aeroloop_gazebo"))
    parser.add_argument("--log-dir", default=os.environ.get("KENET_SITL_LOG_DIR"))
    parser.add_argument("--log-file", default=None)
    args = parser.parse_args()
    if args.samples <= 0:
        parser.error("--samples must be positive")
    if args.interval < 0:
        parser.error("--interval must be non-negative")
    if args.force_mode_pwm is not None and not 1000 <= args.force_mode_pwm <= 2000:
        parser.error("--force-mode-pwm must be between 1000 and 2000")
    for name in (
        "virtual_roll",
        "virtual_pitch",
        "virtual_throttle",
        "virtual_yaw",
        "virtual_arm_pwm",
        "virtual_kenet_pwm",
        "virtual_mode_pwm",
    ):
        if not 1000 <= getattr(args, name) <= 2000:
            parser.error("--%s must be between 1000 and 2000" % name.replace("_", "-"))
    return args


def main() -> int:
    args = parse_args()
    log_dir = resolve_log_dir(args.log_dir, REPO_ROOT)
    log_path = Path(args.log_file) if args.log_file else make_log_path(log_dir, "diagnostics")
    mapping = analyze_motor_mapping((REPO_ROOT / args.aeroloop_gazebo).resolve())
    topic = choose_stats_topic(args.gazebo_topic, args.gazebo_timeout)
    imu_topic = None
    if args.include_gazebo_pose:
        imu_topic = (
            choose_imu_topic(args.gazebo_world_name, args.gazebo_timeout)
            if args.imu_topic == "auto"
            else args.imu_topic
        )
    if args.direct_msp:
        box_names, box_ids, box_error = get_msp_box_metadata(args.msp_host, args.msp_port, args.msp_timeout)
    else:
        box_names, box_ids, box_error = [], [], None
    if args.rc_source == "virtual":
        rc_sampler = VirtualRcSampler(args)
    elif args.rc_source == "external":
        rc_sampler = ExternalRcSampler()
    else:
        rc_sampler = JoystickSampler(args.device, args.force_mode_pwm)
    records: list[dict[str, Any]] = []

    metadata = {
        "tool": "sitl_diagnostics",
        "samples": args.samples,
        "interval": args.interval,
        "rc_source": args.rc_source,
        "device": args.device,
        "virtual_rc": {
            "roll": args.virtual_roll,
            "pitch": args.virtual_pitch,
            "throttle": args.virtual_throttle,
            "yaw": args.virtual_yaw,
            "arm_pwm": args.virtual_arm_pwm,
            "kenet_pwm": args.virtual_kenet_pwm,
            "mode_pwm": args.force_mode_pwm if args.force_mode_pwm is not None else args.virtual_mode_pwm,
        },
        "msp_host": args.msp_host,
        "msp_port": args.msp_port,
        "dashboard_url": args.dashboard_url,
        "gazebo_topic": topic,
        "gazebo_pose": {
            "enabled": args.include_gazebo_pose,
            "world_name": args.gazebo_world_name,
            "entity": args.pose_entity,
            "imu_topic": imu_topic,
        },
        "box_error": box_error,
        "box_names": box_names,
        "box_ids": box_ids,
        "motor_mapping": mapping,
    }
    with JsonlLogger(log_path, metadata=metadata) as logger:
        try:
            for index in range(args.samples):
                started = time.monotonic()
                dashboard_sample = sample_dashboard(args.dashboard_url, args.dashboard_timeout)
                joystick_sample = rc_sampler.sample()
                if not args.direct_msp and dashboard_sample.get("ok") and isinstance(dashboard_sample.get("msp"), dict):
                    msp_sample = dict(dashboard_sample["msp"])
                    msp_sample.setdefault("elapsed_ms", None)
                    msp_sample["source"] = "dashboard"
                else:
                    msp_sample = sample_msp(args.msp_host, args.msp_port, args.msp_timeout, box_names, box_ids)
                    msp_sample["source"] = "direct"
                gazebo_sample = sample_gazebo(topic, args.gazebo_timeout)
                gazebo_pose_sample = (
                    sample_gazebo_pose(
                        args.gazebo_world_name,
                        args.pose_entity,
                        imu_topic or "",
                        args.gazebo_timeout,
                    )
                    if args.include_gazebo_pose and imu_topic
                    else {"enabled": False}
                )
                pilot_channels = joystick_sample.get("pilot_channels") or []
                fc_channels = msp_sample.get("rc_channels") or []
                record = {
                    "event": "diagnostic_sample",
                    "sample_index": index + 1,
                    "sample_elapsed_ms": (time.monotonic() - started) * 1000,
                    "joystick": joystick_sample,
                    "msp": msp_sample,
                    "gazebo": gazebo_sample,
                    "gazebo_pose": gazebo_pose_sample,
                    "dashboard": dashboard_sample,
                    "ports": sample_ports(),
                    "processes": sample_processes(),
                    "channels": channel_deltas(pilot_channels, fc_channels),
                }
                logger.write(**record)
                records.append(record)
                if index + 1 < args.samples:
                    time.sleep(max(0.0, args.interval - (time.monotonic() - started)))
        finally:
            rc_sampler.close()

        summary = summarize(records, mapping)
        logger.write("diagnostic_summary", summary=summary)

    print_summary(summary, log_path, mapping)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
