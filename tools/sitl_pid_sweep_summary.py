#!/usr/bin/env python3
"""Summarize yaw PID sweep logs from the virtual takeoff runner."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any


AXIS_KEYS = ("roll_left_minus_right", "pitch_rear_minus_front", "yaw_cw_minus_ccw")
DEFAULT_AXIS_THRESHOLDS = (25.0, 50.0, 100.0, 200.0, 400.0)


def iter_jsonl(path: Path):
    with path.open("r", encoding="utf-8") as handle:
        for line in handle:
            line = line.strip()
            if line:
                yield json.loads(line)


def number(value: Any) -> float | None:
    if isinstance(value, bool):
        return None
    if isinstance(value, (int, float)):
        return float(value)
    return None


def angle_delta_deg(value: float, origin: float) -> float:
    return (value - origin + 180.0) % 360.0 - 180.0


def first_virtual_nudge_time(path: Path, neutral_yaw: int = 1500) -> float | None:
    for record in iter_jsonl(path):
        if record.get("event") not in ("virtual_rc_step_start", "virtual_rc_frame"):
            continue
        step = str(record.get("step") or "")
        channels = record.get("channels") or []
        yaw = channels[3] if len(channels) > 3 else None
        if "nudge" in step or (isinstance(yaw, int) and yaw != neutral_yaw):
            return number(record.get("time"))
    return None


def active_flight_phase(record: dict[str, Any], min_throttle: int) -> bool:
    msp = record.get("msp") or {}
    status = msp.get("status") or {}
    active_modes = set(status.get("active_modes") or [])
    armed = bool(status.get("armed")) or "ARM" in active_modes
    rc = msp.get("rc_channels") or []
    throttle = rc[3] if len(rc) > 3 else None
    return armed and isinstance(throttle, int) and throttle >= min_throttle


def diagnostic_snapshot(record: dict[str, Any], *, active_min_throttle: int) -> dict[str, Any]:
    msp = record.get("msp") or {}
    debug = [number(value) for value in (msp.get("debug") or [])[:8]]
    attitude = msp.get("attitude") or {}
    rc = msp.get("rc_channels") or []
    motors = [number(value) for value in (msp.get("motor") or [])[:4]]
    motor_spread = None
    if len(motors) == 4 and all(value is not None for value in motors):
        motor_spread = max(motors) - min(motors)  # type: ignore[arg-type]
    pose = record.get("gazebo_pose") or {}
    pose_data = pose.get("pose") or {}
    position = pose_data.get("position") or {}
    pose_euler = pose_data.get("euler_deg") or {}
    imu = pose.get("imu") or {}
    imu_angular = imu.get("angular_velocity") or {}
    throttle = rc[3] if len(rc) > 3 else None
    return {
        "time": number(record.get("time")),
        "sample_index": record.get("sample_index"),
        "active": active_flight_phase(record, active_min_throttle),
        "debug_mode": msp.get("debug_mode"),
        "debug": debug,
        "roll": number(attitude.get("roll")),
        "pitch": number(attitude.get("pitch")),
        "fc_yaw": number(attitude.get("yaw")),
        "pose_yaw": number(pose_euler.get("yaw")),
        "z": number(position.get("z")),
        "imu_yaw_rate": number(imu_angular.get("z")),
        "throttle": throttle if isinstance(throttle, int) else None,
        "motor_spread": motor_spread,
    }


def diagnostics_events(
    path: Path,
    *,
    attitude_threshold: float,
    motor_spread_threshold: float,
    yaw_rate_threshold: float,
    yaw_delta_threshold: float,
    active_min_throttle: int,
    nudge_time: float | None = None,
) -> dict[str, Any]:
    result: dict[str, Any] = {
        "samples": 0,
        "max_abs_roll": None,
        "max_abs_pitch": None,
        "max_abs_active_roll": None,
        "max_abs_active_pitch": None,
        "max_abs_imu_yaw_rate": None,
        "max_abs_pose_yaw_delta": None,
        "max_motor_spread": None,
        "altitude_gain": None,
        "first_attitude": None,
        "first_active_attitude": None,
        "first_imu_yaw_rate": None,
        "first_pose_yaw_delta": None,
        "first_motor_spread": None,
        "sample_near_nudge": None,
        "sample_before_nudge": None,
        "sample_after_nudge": None,
        "debug_mode": None,
        "max_abs_debug": [None] * 8,
        "max_abs_active_debug": [None] * 8,
    }
    z_values: list[float] = []
    pose_yaw_origin: float | None = None
    best_nudge_abs_dt: float | None = None
    for record in iter_jsonl(path):
        if record.get("event") != "diagnostic_sample":
            continue
        result["samples"] += 1
        t = number(record.get("time"))
        if nudge_time is not None and t is not None:
            nudge_abs_dt = abs(t - nudge_time)
            snapshot = diagnostic_snapshot(record, active_min_throttle=active_min_throttle)
            snapshot["dt_from_nudge"] = t - nudge_time
            if best_nudge_abs_dt is None or nudge_abs_dt < best_nudge_abs_dt:
                best_nudge_abs_dt = nudge_abs_dt
                result["sample_near_nudge"] = snapshot
            if t <= nudge_time:
                result["sample_before_nudge"] = snapshot
            if t >= nudge_time and result["sample_after_nudge"] is None:
                result["sample_after_nudge"] = snapshot
        active_phase = active_flight_phase(record, active_min_throttle)
        msp = record.get("msp") or {}
        debug_mode = msp.get("debug_mode")
        if result["debug_mode"] is None and debug_mode is not None:
            result["debug_mode"] = debug_mode
        for index, value in enumerate((msp.get("debug") or [])[:8]):
            parsed = number(value)
            if parsed is None:
                continue
            current = result["max_abs_debug"][index]
            if current is None or abs(parsed) > current:
                result["max_abs_debug"][index] = abs(parsed)
            if active_phase:
                active_current = result["max_abs_active_debug"][index]
                if active_current is None or abs(parsed) > active_current:
                    result["max_abs_active_debug"][index] = abs(parsed)
        attitude = msp.get("attitude") or {}
        roll = number(attitude.get("roll"))
        pitch = number(attitude.get("pitch"))
        if roll is not None:
            result["max_abs_roll"] = max(result["max_abs_roll"] or 0.0, abs(roll))
        if pitch is not None:
            result["max_abs_pitch"] = max(result["max_abs_pitch"] or 0.0, abs(pitch))
        if active_phase:
            if roll is not None:
                result["max_abs_active_roll"] = max(result["max_abs_active_roll"] or 0.0, abs(roll))
            if pitch is not None:
                result["max_abs_active_pitch"] = max(result["max_abs_active_pitch"] or 0.0, abs(pitch))
        if result["first_attitude"] is None and (
            (roll is not None and abs(roll) >= attitude_threshold)
            or (pitch is not None and abs(pitch) >= attitude_threshold)
        ):
            result["first_attitude"] = {
                "time": t,
                "roll": roll,
                "pitch": pitch,
                "sample_index": record.get("sample_index"),
            }
        if active_phase and result["first_active_attitude"] is None and (
            (roll is not None and abs(roll) >= attitude_threshold)
            or (pitch is not None and abs(pitch) >= attitude_threshold)
        ):
            result["first_active_attitude"] = {
                "time": t,
                "roll": roll,
                "pitch": pitch,
                "sample_index": record.get("sample_index"),
            }

        motors = [number(value) for value in (msp.get("motor") or [])[:4]]
        if len(motors) == 4 and all(value is not None for value in motors):
            spread = max(motors) - min(motors)  # type: ignore[arg-type]
            result["max_motor_spread"] = max(result["max_motor_spread"] or 0.0, spread)
            if result["first_motor_spread"] is None and spread >= motor_spread_threshold:
                result["first_motor_spread"] = {
                    "time": t,
                    "spread": spread,
                    "motors": motors,
                    "sample_index": record.get("sample_index"),
                }

        pose = record.get("gazebo_pose") or {}
        pose_euler = ((pose.get("pose") or {}).get("euler_deg") or {})
        pose_yaw = number(pose_euler.get("yaw"))
        if pose_yaw is not None:
            if pose_yaw_origin is None:
                pose_yaw_origin = pose_yaw

        if active_phase:
            imu = pose.get("imu") or {}
            imu_angular = imu.get("angular_velocity") or {}
            yaw_rate = number(imu_angular.get("z"))
            if yaw_rate is not None:
                abs_yaw_rate = abs(yaw_rate)
                result["max_abs_imu_yaw_rate"] = max(result["max_abs_imu_yaw_rate"] or 0.0, abs_yaw_rate)
                if result["first_imu_yaw_rate"] is None and abs_yaw_rate >= yaw_rate_threshold:
                    result["first_imu_yaw_rate"] = {
                        "time": t,
                        "yaw_rate": yaw_rate,
                        "sample_index": record.get("sample_index"),
                    }

        if active_phase and pose_yaw is not None:
            yaw_delta = angle_delta_deg(pose_yaw, pose_yaw_origin)
            abs_yaw_delta = abs(yaw_delta)
            result["max_abs_pose_yaw_delta"] = max(result["max_abs_pose_yaw_delta"] or 0.0, abs_yaw_delta)
            if result["first_pose_yaw_delta"] is None and abs_yaw_delta >= yaw_delta_threshold:
                result["first_pose_yaw_delta"] = {
                    "time": t,
                    "yaw_delta": yaw_delta,
                    "sample_index": record.get("sample_index"),
                }

        position = ((pose.get("pose") or {}).get("position") or {})
        z = number(position.get("z"))
        if z is not None:
            z_values.append(z)

    if z_values:
        result["altitude_gain"] = max(z_values) - min(z_values)
    return result


def raw_motor_events(
    path: Path,
    *,
    spread_threshold: float,
    axis_thresholds: tuple[float, ...] = DEFAULT_AXIS_THRESHOLDS,
) -> dict[str, Any]:
    result: dict[str, Any] = {
        "samples": 0,
        "max_raw_spread": None,
        "max_abs_axis_bias_raw": {key: None for key in AXIS_KEYS},
        "first_large_spread": None,
        "first_axis_thresholds": {
            key: {str(threshold): None for threshold in axis_thresholds}
            for key in AXIS_KEYS
        },
        "first_any_axis_thresholds": {str(threshold): None for threshold in axis_thresholds},
    }
    for record in iter_jsonl(path):
        if record.get("event") != "motor_udp_sample":
            continue
        result["samples"] += 1
        t = number(record.get("received_at")) or number(record.get("time"))
        spread = number(record.get("motor_spread_raw"))
        if spread is not None:
            result["max_raw_spread"] = max(result["max_raw_spread"] or 0.0, spread)
            if result["first_large_spread"] is None and spread >= spread_threshold:
                result["first_large_spread"] = {
                    "time": t,
                    "spread": spread,
                    "motors_raw": record.get("motors_raw"),
                    "axis_bias_raw": record.get("axis_bias_raw") or {},
                }
        axis = record.get("axis_bias_raw") or {}
        for key in AXIS_KEYS:
            value = number(axis.get(key))
            if value is None:
                continue
            current = result["max_abs_axis_bias_raw"][key]
            if current is None or abs(value) > current:
                result["max_abs_axis_bias_raw"][key] = abs(value)
            for threshold in axis_thresholds:
                threshold_key = str(threshold)
                if abs(value) < threshold:
                    continue
                axis_threshold = result["first_axis_thresholds"][key]
                if axis_threshold[threshold_key] is None:
                    axis_threshold[threshold_key] = {
                        "time": t,
                        "axis": key,
                        "value": value,
                    }
                any_threshold = result["first_any_axis_thresholds"]
                if any_threshold[threshold_key] is None:
                    any_threshold[threshold_key] = {
                        "time": t,
                        "axis": key,
                        "value": value,
                    }
    return result


def offset(time_value: float | None, base: float | None) -> float | None:
    if time_value is None or base is None:
        return None
    return time_value - base


def summarize_case(
    label: str,
    diagnostics: Path,
    motor_udp: Path,
    virtual_rc: Path,
    *,
    attitude_threshold: float,
    spread_threshold: float,
    yaw_rate_threshold: float,
    yaw_delta_threshold: float,
    active_min_throttle: int,
    axis_thresholds: tuple[float, ...] = DEFAULT_AXIS_THRESHOLDS,
) -> dict[str, Any]:
    nudge_time = first_virtual_nudge_time(virtual_rc)
    diag = diagnostics_events(
        diagnostics,
        attitude_threshold=attitude_threshold,
        motor_spread_threshold=spread_threshold,
        yaw_rate_threshold=yaw_rate_threshold,
        yaw_delta_threshold=yaw_delta_threshold,
        active_min_throttle=active_min_throttle,
        nudge_time=nudge_time,
    )
    motor = raw_motor_events(motor_udp, spread_threshold=spread_threshold, axis_thresholds=axis_thresholds)
    first_attitude = diag.get("first_attitude") or {}
    first_active_attitude = diag.get("first_active_attitude") or {}
    first_yaw_rate = diag.get("first_imu_yaw_rate") or {}
    first_yaw_delta = diag.get("first_pose_yaw_delta") or {}
    first_diag_motor = diag.get("first_motor_spread") or {}
    first_raw = motor.get("first_large_spread") or {}
    first_any_axis = motor.get("first_any_axis_thresholds") or {}
    nudge_sample = diag.get("sample_near_nudge") or {}
    before_nudge = diag.get("sample_before_nudge") or {}
    after_nudge = diag.get("sample_after_nudge") or {}
    nudge_debug = nudge_sample.get("debug") or []
    before_debug = before_nudge.get("debug") or []
    after_debug = after_nudge.get("debug") or []
    max_abs_debug = diag.get("max_abs_debug") or []
    max_abs_active_debug = diag.get("max_abs_active_debug") or []
    axis = first_raw.get("axis_bias_raw") or {}
    row = {
        "label": label,
        "diagnostics": str(diagnostics),
        "motor_udp": str(motor_udp),
        "virtual_rc": str(virtual_rc),
        "nudge_time": nudge_time,
        "samples": diag.get("samples"),
        "altitude_gain": diag.get("altitude_gain"),
        "max_abs_roll": diag.get("max_abs_roll"),
        "max_abs_pitch": diag.get("max_abs_pitch"),
        "max_abs_active_roll": diag.get("max_abs_active_roll"),
        "max_abs_active_pitch": diag.get("max_abs_active_pitch"),
        "max_abs_imu_yaw_rate": diag.get("max_abs_imu_yaw_rate"),
        "max_abs_pose_yaw_delta": diag.get("max_abs_pose_yaw_delta"),
        "max_diag_motor_spread": diag.get("max_motor_spread"),
        "max_raw_spread": motor.get("max_raw_spread"),
        "debug_mode": diag.get("debug_mode"),
        "first_attitude_after_nudge_s": offset(first_attitude.get("time"), nudge_time),
        "first_imu_yaw_rate_after_nudge_s": offset(first_yaw_rate.get("time"), nudge_time),
        "first_pose_yaw_delta_after_nudge_s": offset(first_yaw_delta.get("time"), nudge_time),
        "first_diag_motor_spread_after_nudge_s": offset(first_diag_motor.get("time"), nudge_time),
        "first_raw_spread_after_nudge_s": offset(first_raw.get("time"), nudge_time),
        "first_raw_roll_bias": axis.get("roll_left_minus_right"),
        "first_raw_pitch_bias": axis.get("pitch_rear_minus_front"),
        "first_raw_yaw_bias": axis.get("yaw_cw_minus_ccw"),
        "first_active_attitude_after_nudge_s": offset(first_active_attitude.get("time"), nudge_time),
        "nudge_sample_dt_s": nudge_sample.get("dt_from_nudge"),
        "nudge_active": nudge_sample.get("active"),
        "nudge_z": nudge_sample.get("z"),
        "nudge_roll": nudge_sample.get("roll"),
        "nudge_pitch": nudge_sample.get("pitch"),
        "nudge_fc_yaw": nudge_sample.get("fc_yaw"),
        "nudge_pose_yaw": nudge_sample.get("pose_yaw"),
        "nudge_imu_yaw_rate": nudge_sample.get("imu_yaw_rate"),
        "nudge_throttle": nudge_sample.get("throttle"),
        "nudge_motor_spread": nudge_sample.get("motor_spread"),
        "before_nudge_dt_s": before_nudge.get("dt_from_nudge"),
        "before_nudge_z": before_nudge.get("z"),
        "before_nudge_roll": before_nudge.get("roll"),
        "before_nudge_pitch": before_nudge.get("pitch"),
        "before_nudge_pose_yaw": before_nudge.get("pose_yaw"),
        "after_nudge_dt_s": after_nudge.get("dt_from_nudge"),
        "after_nudge_z": after_nudge.get("z"),
        "after_nudge_roll": after_nudge.get("roll"),
        "after_nudge_pitch": after_nudge.get("pitch"),
        "after_nudge_pose_yaw": after_nudge.get("pose_yaw"),
        "first_axis_thresholds": motor.get("first_axis_thresholds"),
        "first_any_axis_thresholds": first_any_axis,
    }
    for index in range(8):
        row[f"nudge_debug{index}"] = nudge_debug[index] if len(nudge_debug) > index else None
        row[f"before_debug{index}"] = before_debug[index] if len(before_debug) > index else None
        row[f"after_debug{index}"] = after_debug[index] if len(after_debug) > index else None
        row[f"max_abs_debug{index}"] = max_abs_debug[index] if len(max_abs_debug) > index else None
        row[f"max_abs_active_debug{index}"] = (
            max_abs_active_debug[index] if len(max_abs_active_debug) > index else None
        )
    for threshold in axis_thresholds:
        threshold_key = str(threshold)
        first_axis = first_any_axis.get(threshold_key) or {}
        column_key = str(int(threshold)) if threshold.is_integer() else threshold_key
        row[f"first_raw_axis_{column_key}_after_nudge_s"] = offset(first_axis.get("time"), nudge_time)
        row[f"first_raw_axis_{column_key}_name"] = first_axis.get("axis")
        row[f"first_raw_axis_{column_key}_value"] = first_axis.get("value")
    return row


def format_value(value: Any) -> str:
    if value is None:
        return "-"
    if isinstance(value, float):
        return "%.3f" % value
    return str(value)


def print_markdown(rows: list[dict[str, Any]]) -> None:
    columns = [
        ("label", "case"),
        ("altitude_gain", "alt_gain"),
        ("max_abs_roll", "max_roll"),
        ("max_abs_pitch", "max_pitch"),
        ("max_abs_active_roll", "active_roll"),
        ("max_abs_active_pitch", "active_pitch"),
        ("max_abs_imu_yaw_rate", "max_yaw_rate"),
        ("max_abs_pose_yaw_delta", "max_yaw_delta"),
        ("max_raw_spread", "raw_spread"),
        ("debug_mode", "dbg_mode"),
        ("nudge_z", "nudge_z"),
        ("nudge_roll", "nudge_roll"),
        ("nudge_pitch", "nudge_pitch"),
        ("nudge_pose_yaw", "nudge_yaw"),
        ("nudge_sample_dt_s", "nudge_dt"),
        ("nudge_debug0", "dbg0"),
        ("nudge_debug1", "dbg1"),
        ("nudge_debug2", "dbg2"),
        ("nudge_debug3", "dbg3"),
        ("nudge_debug4", "dbg4"),
        ("nudge_debug5", "dbg5"),
        ("nudge_debug6", "dbg6"),
        ("nudge_debug7", "dbg7"),
        ("max_abs_debug0", "max_dbg0"),
        ("max_abs_debug1", "max_dbg1"),
        ("max_abs_debug2", "max_dbg2"),
        ("max_abs_debug3", "max_dbg3"),
        ("max_abs_debug4", "max_dbg4"),
        ("max_abs_debug5", "max_dbg5"),
        ("max_abs_debug6", "max_dbg6"),
        ("max_abs_debug7", "max_dbg7"),
        ("max_abs_active_debug0", "active_dbg0"),
        ("max_abs_active_debug1", "active_dbg1"),
        ("max_abs_active_debug2", "active_dbg2"),
        ("max_abs_active_debug3", "active_dbg3"),
        ("max_abs_active_debug4", "active_dbg4"),
        ("max_abs_active_debug5", "active_dbg5"),
        ("max_abs_active_debug6", "active_dbg6"),
        ("max_abs_active_debug7", "active_dbg7"),
        ("before_nudge_z", "before_z"),
        ("after_nudge_z", "after_z"),
        ("after_nudge_dt_s", "after_dt"),
        ("first_raw_axis_25_after_nudge_s", "axis25_dt"),
        ("first_raw_axis_100_after_nudge_s", "axis100_dt"),
        ("first_raw_axis_200_after_nudge_s", "axis200_dt"),
        ("first_raw_axis_400_after_nudge_s", "axis400_dt"),
        ("first_imu_yaw_rate_after_nudge_s", "yaw_rate_dt"),
        ("first_pose_yaw_delta_after_nudge_s", "yaw_delta_dt"),
        ("first_raw_spread_after_nudge_s", "raw_dt"),
        ("first_attitude_after_nudge_s", "att_dt"),
        ("first_active_attitude_after_nudge_s", "active_att_dt"),
        ("first_raw_axis_400_name", "axis400"),
        ("first_raw_roll_bias", "raw_roll"),
        ("first_raw_pitch_bias", "raw_pitch"),
        ("first_raw_yaw_bias", "raw_yaw"),
    ]
    print("| " + " | ".join(header for _, header in columns) + " |")
    print("| " + " | ".join("---" for _ in columns) + " |")
    for row in rows:
        print("| " + " | ".join(format_value(row.get(key)) for key, _ in columns) + " |")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--case",
        action="append",
        nargs=4,
        metavar=("LABEL", "DIAGNOSTICS", "MOTOR_UDP", "VIRTUAL_RC"),
        help="Add one sweep case",
    )
    parser.add_argument("--attitude-threshold", type=float, default=35.0)
    parser.add_argument("--spread-threshold", type=float, default=400.0)
    parser.add_argument("--yaw-rate-threshold", type=float, default=0.5,
                        help="Absolute Gazebo IMU yaw-rate threshold in rad/s")
    parser.add_argument("--yaw-delta-threshold", type=float, default=5.0,
                        help="Absolute pose yaw delta threshold in degrees")
    parser.add_argument("--active-min-throttle", type=int, default=1500,
                        help="Minimum FC throttle for active yaw-rate/yaw-delta metrics")
    parser.add_argument(
        "--axis-thresholds",
        default="25,50,100,200,400",
        help="Comma-separated raw motor axis-bias thresholds in microseconds",
    )
    parser.add_argument("--json", action="store_true", help="Print JSON instead of markdown")
    args = parser.parse_args()
    if not args.case:
        parser.error("at least one --case is required")
    try:
        args.axis_thresholds = tuple(float(value) for value in args.axis_thresholds.split(",") if value.strip())
    except ValueError as exc:
        parser.error(f"invalid --axis-thresholds: {exc}")
    if not args.axis_thresholds:
        parser.error("--axis-thresholds cannot be empty")
    return args


def main() -> int:
    args = parse_args()
    rows = [
        summarize_case(
            label,
            Path(diagnostics),
            Path(motor_udp),
            Path(virtual_rc),
            attitude_threshold=args.attitude_threshold,
            spread_threshold=args.spread_threshold,
            yaw_rate_threshold=args.yaw_rate_threshold,
            yaw_delta_threshold=args.yaw_delta_threshold,
            active_min_throttle=args.active_min_throttle,
            axis_thresholds=args.axis_thresholds,
        )
        for label, diagnostics, motor_udp, virtual_rc in args.case
    ]
    if args.json:
        print(json.dumps(rows, indent=2, sort_keys=True))
    else:
        print_markdown(rows)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
