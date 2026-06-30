#!/usr/bin/env python3
"""Capture Betaflight SITL raw motor UDP output packets."""

from __future__ import annotations

import argparse
import os
import socket
import statistics
import struct
import time
from pathlib import Path
from typing import Any


SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parent

from sitl_log import JsonlLogger, make_log_path, resolve_log_dir


RAW_SERVO_PACKET_SIZE = 68
RAW_SERVO_PACKET_FORMAT = "<H2x16f"
BF_SITL_PACKET_SLOT_BY_MOTOR = [3, 0, 1, 2]
AXIS_BIAS_LABELS = (
    "roll_left_minus_right",
    "pitch_rear_minus_front",
    "yaw_cw_minus_ccw",
)


def remap_logical_to_packet_slots(values: list[float],
                                  motor_map: list[int] | None = None) -> list[float | None]:
    motor_map = motor_map or BF_SITL_PACKET_SLOT_BY_MOTOR
    remapped: list[float | None] = [None] * len(motor_map)
    for logical_index, packet_slot in enumerate(motor_map):
        if logical_index < len(values) and packet_slot < len(remapped):
            remapped[packet_slot] = values[logical_index]
    return remapped


def motor_axis_biases(motors: list[float]) -> dict[str, float | None]:
    if len(motors) < 4:
        return {label: None for label in AXIS_BIAS_LABELS}
    # Betaflight QUADX logical order: 0=rear-right, 1=front-right,
    # 2=rear-left, 3=front-left.
    right = (motors[0] + motors[1]) / 2.0
    left = (motors[2] + motors[3]) / 2.0
    rear = (motors[0] + motors[2]) / 2.0
    front = (motors[1] + motors[3]) / 2.0
    cw = (motors[0] + motors[3]) / 2.0
    ccw = (motors[1] + motors[2]) / 2.0
    return {
        "roll_left_minus_right": left - right,
        "pitch_rear_minus_front": rear - front,
        "yaw_cw_minus_ccw": cw - ccw,
    }


def parse_raw_servo_packet(data: bytes) -> dict[str, Any]:
    if len(data) < RAW_SERVO_PACKET_SIZE:
        raise ValueError("raw servo packet too short: %d < %d" % (len(data), RAW_SERVO_PACKET_SIZE))
    unpacked = struct.unpack(RAW_SERVO_PACKET_FORMAT, data[:RAW_SERVO_PACKET_SIZE])
    motor_count = int(unpacked[0])
    outputs = list(unpacked[1:])
    motors = outputs[:motor_count]
    normalized = [(value - 1000.0) / 1000.0 for value in motors]
    return {
        "motor_count": motor_count,
        "pwm_output_raw": outputs,
        "motors_raw": motors,
        "motors_normalized_1000_idle": normalized,
        "motors_packet_order_normalized": remap_logical_to_packet_slots(normalized),
        "motor_spread_raw": (max(motors) - min(motors)) if motors else None,
        "axis_bias_raw": motor_axis_biases(motors),
        "axis_bias_normalized": motor_axis_biases(normalized),
    }


def summarize(samples: list[dict[str, Any]]) -> dict[str, Any]:
    motor_counts = sorted({sample["motor_count"] for sample in samples})
    max_by_motor = []
    min_by_motor = []
    mean_by_motor = []
    max_norm_by_motor = []
    spreads = []
    axis_values: dict[str, list[float]] = {label: [] for label in AXIS_BIAS_LABELS}
    first_large_spread = None
    count = max(motor_counts) if motor_counts else 0
    for index in range(count):
        values = [
            sample["motors_raw"][index]
            for sample in samples
            if index < len(sample["motors_raw"])
        ]
        norm_values = [
            sample["motors_normalized_1000_idle"][index]
            for sample in samples
            if index < len(sample["motors_normalized_1000_idle"])
        ]
        max_by_motor.append(max(values) if values else None)
        min_by_motor.append(min(values) if values else None)
        mean_by_motor.append(statistics.fmean(values) if values else None)
        max_norm_by_motor.append(max(norm_values) if norm_values else None)
    for sample in samples:
        spread = sample.get("motor_spread_raw")
        if isinstance(spread, (int, float)):
            spreads.append(float(spread))
            if first_large_spread is None and spread >= 400:
                first_large_spread = {
                    "received_at": sample.get("received_at"),
                    "motors_raw": sample.get("motors_raw"),
                    "motor_spread_raw": spread,
                    "axis_bias_raw": sample.get("axis_bias_raw"),
                    "motors_packet_order_normalized": sample.get("motors_packet_order_normalized"),
                }
        for label in AXIS_BIAS_LABELS:
            value = (sample.get("axis_bias_raw") or {}).get(label)
            if isinstance(value, (int, float)):
                axis_values[label].append(float(value))
    return {
        "samples": len(samples),
        "motor_counts": motor_counts,
        "min_raw_by_motor": min_by_motor,
        "mean_raw_by_motor": mean_by_motor,
        "max_raw_by_motor": max_by_motor,
        "max_normalized_by_motor": max_norm_by_motor,
        "max_raw_spread": max(spreads) if spreads else None,
        "max_abs_axis_bias_raw": {
            label: max((abs(value) for value in values), default=None)
            for label, values in axis_values.items()
        },
        "first_large_spread": first_large_spread,
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=9001)
    parser.add_argument("--duration", type=float, default=15.0)
    parser.add_argument("--socket-timeout", type=float, default=0.5)
    parser.add_argument("--log-dir", default=os.environ.get("KENET_SITL_LOG_DIR"))
    parser.add_argument("--log-file", default=None)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.duration <= 0:
        raise SystemExit("--duration must be positive")
    if args.socket_timeout <= 0:
        raise SystemExit("--socket-timeout must be positive")

    log_dir = resolve_log_dir(args.log_dir, REPO_ROOT)
    log_path = Path(args.log_file) if args.log_file else make_log_path(log_dir, "motor-udp")
    deadline = time.monotonic() + args.duration
    samples: list[dict[str, Any]] = []

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.settimeout(args.socket_timeout)
        sock.bind((args.host, args.port))
        metadata = {
            "tool": "sitl_motor_udp_probe",
            "host": args.host,
            "port": args.port,
            "duration": args.duration,
        }
        with JsonlLogger(log_path, metadata=metadata) as logger:
            while time.monotonic() < deadline:
                try:
                    data, addr = sock.recvfrom(4096)
                except socket.timeout:
                    continue
                received_at = time.time()
                try:
                    sample = parse_raw_servo_packet(data)
                    sample.update({
                        "ok": True,
                        "error": None,
                        "bytes": len(data),
                        "source": "%s:%d" % addr,
                        "received_at": received_at,
                    })
                except ValueError as exc:
                    sample = {
                        "ok": False,
                        "error": str(exc),
                        "bytes": len(data),
                        "source": "%s:%d" % addr,
                        "received_at": received_at,
                    }
                logger.write("motor_udp_sample", **sample)
                if sample.get("ok"):
                    samples.append(sample)
            summary = summarize(samples)
            logger.write("motor_udp_summary", summary=summary)
    finally:
        sock.close()

    print("motor UDP log: %s" % log_path)
    print("samples: %d" % len(samples))
    if samples:
        summary = summarize(samples)
        print("motor counts: %s" % (",".join(str(v) for v in summary["motor_counts"]) or "-"))
        print("max raw by motor: %s" % ",".join(_fmt(v) for v in summary["max_raw_by_motor"]))
        print("max normalized by motor: %s" % ",".join(_fmt(v) for v in summary["max_normalized_by_motor"]))
        print("max raw spread: %s" % _fmt(summary["max_raw_spread"]))
        print("max abs axis bias raw: %s" % ",".join(
            "%s=%s" % (label, _fmt(summary["max_abs_axis_bias_raw"].get(label)))
            for label in AXIS_BIAS_LABELS
        ))
    return 0


def _fmt(value: Any) -> str:
    if value is None:
        return "-"
    if isinstance(value, float):
        return "%.3f" % value
    return str(value)


if __name__ == "__main__":
    raise SystemExit(main())
