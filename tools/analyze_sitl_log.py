#!/usr/bin/env python3
"""Summarize Kenet / Betaflight SITL JSONL logs."""

from __future__ import annotations

import argparse
import json
import os
from collections import Counter, defaultdict
from numbers import Real
from pathlib import Path

from sitl_rc_channels import channel_label, channel_neutral
from sitl_log import resolve_log_dir


def iter_records(path: Path):
    with path.open("r", encoding="utf-8") as handle:
        for line_no, line in enumerate(handle, 1):
            line = line.strip()
            if not line:
                continue
            try:
                yield json.loads(line)
            except json.JSONDecodeError as exc:
                print("warning: %s:%d invalid json: %s" % (path, line_no, exc))


def latest_logs(log_dir: Path, limit: int) -> list[Path]:
    if not log_dir.exists():
        return []
    paths = [p for p in log_dir.glob("*.jsonl") if p.is_file()]
    paths.sort(key=lambda p: p.stat().st_mtime)
    return paths[-limit:]


def absmax_update(current, value, record):
    if value is None:
        return current
    if current is None or abs(value) > abs(current["value"]):
        return {"value": value, "time_iso": record.get("time_iso"), "event": record.get("event")}
    return current


def numeric_or_none(value):
    if isinstance(value, bool):
        return None
    if isinstance(value, Real):
        return float(value)
    return None


def msp_field(msp: dict, key: str, default=None):
    status = msp.get("status") or {}
    if key in msp:
        return msp.get(key)
    return status.get(key, default)


def summarize(paths: list[Path]) -> dict:
    stats = {
        "paths": [str(path) for path in paths],
        "events": Counter(),
        "first_time": None,
        "last_time": None,
        "markers": [],
        "dashboard": {
            "samples": 0,
            "attitude_samples": 0,
            "motor_samples": 0,
            "msp_offline": 0,
            "armed": Counter(),
            "active_modes": Counter(),
            "kenet_states": Counter(),
            "autopilot_modes": Counter(),
            "arming_flags": Counter(),
            "max_roll": None,
            "max_pitch": None,
            "max_motor_spread": None,
            "max_channel_delta": defaultdict(lambda: {"value": 0, "time_iso": None}),
        },
        "mixer": {
            "samples": 0,
            "states": Counter(),
            "sources": Counter(),
            "target_found": Counter(),
            "max_channel_delta": defaultdict(lambda: {"value": 0, "time_iso": None}),
            "max_command_deviation": defaultdict(lambda: {"value": 0, "time_iso": None}),
            "schema_warnings": Counter(),
        },
    }

    for path in paths:
        for record in iter_records(path):
            event = record.get("event", "unknown")
            stats["events"][event] += 1
            if record.get("time_iso"):
                stats["first_time"] = stats["first_time"] or record["time_iso"]
                stats["last_time"] = record["time_iso"]

            if event == "manual_marker":
                stats["markers"].append({
                    "time_iso": record.get("time_iso"),
                    "label": record.get("label"),
                    "path": str(path),
                })

            if event in ("dashboard_sample", "diagnostic_sample", "manual_marker"):
                dash = stats["dashboard"]
                dash["samples"] += 1
                msp = record.get("msp") or {}
                if not msp.get("connected"):
                    dash["msp_offline"] += 1
                armed = msp_field(msp, "armed")
                if armed is None:
                    dash["armed"]["unknown"] += 1
                else:
                    dash["armed"][str(bool(armed))] += 1
                active_modes = msp_field(msp, "active_modes")
                if active_modes:
                    for active_mode in active_modes:
                        dash["active_modes"][str(active_mode)] += 1
                elif msp_field(msp, "active_modes_valid"):
                    dash["active_modes"]["none"] += 1
                else:
                    dash["active_modes"]["unknown"] += 1
                kenet = record.get("kenet") or {}
                mode = record.get("autopilot_mode") or {}
                dash["kenet_states"][str(kenet.get("state"))] += 1
                dash["autopilot_modes"][str(mode.get("state"))] += 1
                for flag in msp_field(msp, "arming_disable_names", []) or []:
                    dash["arming_flags"][flag] += 1
                attitude = msp.get("attitude") or {}
                roll = numeric_or_none(attitude.get("roll"))
                pitch = numeric_or_none(attitude.get("pitch"))
                if roll is not None or pitch is not None:
                    dash["attitude_samples"] += 1
                dash["max_roll"] = absmax_update(dash["max_roll"], roll, record)
                dash["max_pitch"] = absmax_update(dash["max_pitch"], pitch, record)
                motor = (msp.get("motor") or [])[:4]
                motor_values = [numeric_or_none(value) for value in motor]
                if len(motor_values) >= 2 and all(value is not None for value in motor_values):
                    dash["motor_samples"] += 1
                    spread = max(motor_values) - min(motor_values)
                    current = dash["max_motor_spread"]
                    if current is None or spread > current["value"]:
                        dash["max_motor_spread"] = {
                            "value": spread,
                            "motors": motor,
                            "time_iso": record.get("time_iso"),
                        }
                for row in record.get("channels") or []:
                    label = row.get("label") or ("CH%s" % row.get("channel"))
                    delta = row.get("delta")
                    if delta is None:
                        continue
                    current = dash["max_channel_delta"][label]
                    if abs(delta) > abs(current["value"]):
                        dash["max_channel_delta"][label] = {
                            "value": delta,
                            "time_iso": record.get("time_iso"),
                        }

            if event == "kenet_mixer_sample":
                mix = stats["mixer"]
                mix["samples"] += 1
                mix["states"][str(record.get("state"))] += 1
                mix["sources"][str(record.get("source"))] += 1
                mix["target_found"][str(bool(record.get("target_found")))] += 1
                first8 = record.get("first8") or {}
                pilot_channels = record.get("pilot_channels")
                final_channels = record.get("final_channels")
                if isinstance(pilot_channels, list) and isinstance(final_channels, list):
                    deltas = [
                        final_channels[index] - pilot_channels[index]
                        for index in range(min(len(pilot_channels), len(final_channels)))
                        if numeric_or_none(final_channels[index]) is not None
                        and numeric_or_none(pilot_channels[index]) is not None
                    ]
                else:
                    deltas = first8.get("delta") or []
                    if not deltas:
                        mix["schema_warnings"]["missing pilot/final channel fields"] += 1
                    else:
                        mix["schema_warnings"]["using legacy first8 channel fields"] += 1
                for index, delta in enumerate(deltas):
                    label = channel_label(index)
                    current = mix["max_channel_delta"][label]
                    if abs(delta) > abs(current["value"]):
                        mix["max_channel_delta"][label] = {
                            "value": delta,
                            "time_iso": record.get("time_iso"),
                        }
                final = final_channels if isinstance(final_channels, list) else (first8.get("final") or [])
                if not final:
                    mix["schema_warnings"]["missing final channel fields"] += 1
                for index, value in enumerate(final):
                    if numeric_or_none(value) is None:
                        continue
                    deviation = value - channel_neutral(index)
                    label = channel_label(index)
                    current = mix["max_command_deviation"][label]
                    if abs(deviation) > abs(current["value"]):
                        mix["max_command_deviation"][label] = {
                            "value": deviation,
                            "time_iso": record.get("time_iso"),
                            "rc": value,
                        }
    return stats


def print_counter(title: str, counter: Counter) -> None:
    print(title)
    if not counter:
        print("  -")
        return
    for key, value in counter.most_common():
        print("  %s: %s" % (key, value))


def print_top_map(title: str, values: dict, limit: int = 8) -> None:
    print(title)
    items = sorted(values.items(), key=lambda item: abs(item[1]["value"]), reverse=True)
    if not items:
        print("  -")
        return
    for label, data in items[:limit]:
        extra = " rc=%s" % data.get("rc") if data.get("rc") is not None else ""
        print("  %s: %+d at %s%s" % (
            label,
            int(data["value"]),
            data.get("time_iso") or "-",
            extra,
        ))


def print_summary(stats: dict) -> None:
    print("SITL log summary")
    print("paths:")
    for path in stats["paths"]:
        print("  %s" % path)
    print("time: %s -> %s" % (stats["first_time"] or "-", stats["last_time"] or "-"))
    print_counter("events:", stats["events"])

    dash = stats["dashboard"]
    print("\nDashboard / Betaflight")
    print("  samples: %d" % dash["samples"])
    print("  attitude samples: %d" % dash["attitude_samples"])
    print("  motor samples: %d" % dash["motor_samples"])
    print("  msp_offline: %d" % dash["msp_offline"])
    print_counter("  armed:", dash["armed"])
    print_counter("  active modes:", dash["active_modes"])
    print_counter("  kenet states:", dash["kenet_states"])
    print_counter("  autopilot modes:", dash["autopilot_modes"])
    print_counter("  arming flags:", dash["arming_flags"])
    for name, entry in (("max roll", dash["max_roll"]), ("max pitch", dash["max_pitch"])):
        if entry:
            print("  %s: %.1f deg at %s" % (name, entry["value"], entry["time_iso"]))
    if dash["max_motor_spread"]:
        entry = dash["max_motor_spread"]
        print("  max motor spread: %d at %s motors=%s" % (
            int(entry["value"]),
            entry["time_iso"],
            ",".join(str(v) for v in entry["motors"]),
        ))
    print_top_map("  max FC-pilot channel deltas:", dash["max_channel_delta"])

    mix = stats["mixer"]
    print("\nKenet Mixer")
    print("  samples: %d" % mix["samples"])
    print_counter("  states:", mix["states"])
    print_counter("  sources:", mix["sources"])
    print_counter("  target found:", mix["target_found"])
    print_top_map("  max final-pilot deltas:", mix["max_channel_delta"])
    print_top_map("  max command deviations:", mix["max_command_deviation"])
    print_counter("  schema warnings:", mix["schema_warnings"])

    if stats["markers"]:
        print("\nMarkers")
        for marker in stats["markers"]:
            print("  %s label=%s path=%s" % (
                marker.get("time_iso"),
                marker.get("label"),
                marker.get("path"),
            ))

    print("\nHeuristics")
    if dash["attitude_samples"]:
        roll = dash["max_roll"]["value"] if dash["max_roll"] else 0
        pitch = dash["max_pitch"]["value"] if dash["max_pitch"] else 0
        attitude_abs = max(abs(roll), abs(pitch))
        if attitude_abs >= 60:
            print("  WARNING: attitude exceeded 60 deg; this matches a flip/tumble event.")
        elif attitude_abs >= 35:
            print("  WARNING: attitude exceeded 35 deg; close to loss-of-control territory.")
        else:
            print("  attitude stayed below 35 deg in available MSP samples.")
    else:
        print("  attitude data unavailable; no MSP attitude samples were found.")
    if dash["motor_samples"]:
        motor_spread = dash["max_motor_spread"]["value"] if dash["max_motor_spread"] else 0
        if motor_spread >= 400:
            print("  WARNING: motor spread exceeded 400 us; check mixer, motor order/direction, or aggressive PID correction.")
        elif motor_spread >= 250:
            print("  motor spread exceeded 250 us; correction demand is significant.")
        else:
            print("  motor spread did not show a large correction in available samples.")
    else:
        print("  motor data unavailable; no MSP motor samples were found.")
    if dash["msp_offline"] and dash["samples"] and dash["msp_offline"] == dash["samples"]:
        print("  MSP was offline for all dashboard samples; close Betaflight Configurator while logging.")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("paths", nargs="*", type=Path,
                        help="JSONL paths. If omitted, latest logs are used.")
    parser.add_argument("--log-dir", default=os.environ.get("KENET_SITL_LOG_DIR"),
                        help="Log directory; default logs/sitl")
    parser.add_argument("--latest", type=int, default=5,
                        help="When no paths are supplied, analyze latest N JSONL files")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    paths = args.paths
    if not paths:
        paths = latest_logs(resolve_log_dir(args.log_dir), args.latest)
    if not paths:
        print("No JSONL logs found. Run tools/sitl_dashboard.py or tools/kenet_sitl_mixer.py first.")
        return 1
    missing = [str(path) for path in paths if not path.exists()]
    if missing:
        print("missing logs: %s" % ", ".join(missing))
        return 2
    print_summary(summarize(paths))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
