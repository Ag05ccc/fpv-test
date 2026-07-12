#!/usr/bin/env python3
"""Measure signed vehicle attitude response to a uinput axis pulse over read-only MSP.

The FPV camera uptilt makes pad/climb views sky-dominated, so pixel-shift
response estimates collapse there. MSP_ATTITUDE reads the vehicle response
directly and is render-independent.
"""

from __future__ import annotations

import struct
import time
from dataclasses import dataclass, field
from pathlib import Path
from statistics import median
from typing import Any, Callable

import sys

PROBE_DIR = Path(__file__).resolve().parent
if str(PROBE_DIR) not in sys.path:
    sys.path.insert(0, str(PROBE_DIR))
SANDBOX_DIR = PROBE_DIR.parent / "game_screen_sandbox"
if str(SANDBOX_DIR) not in sys.path:
    sys.path.insert(0, str(SANDBOX_DIR))

from msp_ws_rc_probe import request_msp_over_websocket  # noqa: E402
from virtual_input import AxisCommand  # noqa: E402


PASS = "PASS"
WAITING = "WAITING"
FAIL = "FAIL"
MSP_ATTITUDE = 108
MSP_ALTITUDE = 109
ATTITUDE_AXES = ("yaw", "pitch", "roll")
AXIS_TO_FIELD = {"yaw": "yaw_deg", "pitch": "pitch_deg", "roll": "roll_deg"}


def read_altitude_m(
    *,
    host: str,
    port: int,
    path: str = "/",
    timeout: float = 1.0,
    max_frames: int = 8,
) -> float | None:
    payload, _metrics = request_msp_over_websocket(
        host=host,
        port=port,
        path=path,
        code=MSP_ALTITUDE,
        timeout=timeout,
        max_frames=max_frames,
    )
    if payload is None or len(payload) < 4:
        return None
    alt_cm = struct.unpack("<i", payload[:4])[0]
    return alt_cm / 100.0


@dataclass
class UinputAttitudeResponseResult:
    status: str
    summary: str
    metrics: dict[str, Any] = field(default_factory=dict)
    notes: list[str] = field(default_factory=list)

    def as_dict(self) -> dict[str, Any]:
        return {
            "status": self.status,
            "summary": self.summary,
            "metrics": self.metrics,
            "notes": self.notes,
        }


def parse_attitude_payload(payload: bytes | None) -> dict[str, float] | None:
    if payload is None or len(payload) < 6:
        return None
    roll_raw, pitch_raw, yaw_raw = struct.unpack("<hhh", payload[:6])
    return {
        "roll_deg": roll_raw / 10.0,
        "pitch_deg": pitch_raw / 10.0,
        "yaw_deg": float(yaw_raw),
    }


def wrap_deg(value: float) -> float:
    while value > 180.0:
        value -= 360.0
    while value < -180.0:
        value += 360.0
    return value


def accumulated_delta(values: list[float], *, wrap: bool) -> float:
    """Sum of successive differences; wrapped per step for heading so multi-turn
    yaw does not alias."""
    if len(values) < 2:
        return 0.0
    total = 0.0
    for previous, current in zip(values, values[1:]):
        step = current - previous
        if wrap:
            step = wrap_deg(step)
        total += step
    return total


def sample_attitude(
    *,
    host: str,
    port: int,
    path: str,
    timeout: float,
    max_frames: int,
    samples: int,
    interval_s: float,
) -> tuple[list[dict[str, float]], list[dict[str, Any]]]:
    if samples <= 0:
        raise ValueError("samples must be positive")
    parsed_samples: list[dict[str, float]] = []
    request_metrics: list[dict[str, Any]] = []
    for index in range(samples):
        payload, metrics = request_msp_over_websocket(
            host=host,
            port=port,
            path=path,
            code=MSP_ATTITUDE,
            timeout=timeout,
            max_frames=max_frames,
        )
        parsed = parse_attitude_payload(payload)
        request_metrics.append({"index": index, "attitude": parsed, **metrics})
        if parsed is not None:
            parsed_samples.append(parsed)
        if index + 1 < samples and interval_s > 0:
            time.sleep(interval_s)
    return parsed_samples, request_metrics


def evaluate_attitude_response(
    *,
    axis: str,
    delta_deg: float,
    expected_sign: int,
    min_delta_deg: float,
    max_delta_deg: float,
) -> tuple[str, str, list[str]]:
    notes = ["ATTITUDE_DELTA_DEG:%.1f" % delta_deg]
    if abs(delta_deg) < min_delta_deg:
        return (
            WAITING,
            "attitude %s response did not cross the minimum delta" % axis,
            notes + ["NO_ATTITUDE_RESPONSE"],
        )
    if abs(delta_deg) > max_delta_deg:
        return (
            FAIL,
            "attitude %s response exceeded the maximum bounded delta" % axis,
            notes + ["ATTITUDE_RESPONSE_UNBOUNDED"],
        )
    observed_sign = 1 if delta_deg > 0 else -1
    if expected_sign != 0 and observed_sign != expected_sign:
        return (
            FAIL,
            "attitude %s response moved against the expected sign" % axis,
            notes + ["ATTITUDE_SIGN_MISMATCH"],
        )
    return (
        PASS,
        "attitude %s response moved %.1f deg in the expected direction" % (axis, delta_deg),
        notes + ["ATTITUDE_RESPONSE_DETECTED"],
    )


def measure_uinput_attitude_response(
    adapter: Any,
    *,
    axis: str,
    pulse_command: AxisCommand,
    release_command: AxisCommand | None,
    host: str,
    port: int,
    path: str,
    timeout: float,
    max_frames: int,
    baseline_samples: int,
    during_samples: int,
    interval_s: float,
    settle_s: float,
    expected_sign: int,
    min_delta_deg: float,
    max_delta_deg: float,
    sleeper: Callable[[float], None] = time.sleep,
) -> UinputAttitudeResponseResult:
    if axis not in ATTITUDE_AXES:
        raise ValueError("axis must be one of %s" % (ATTITUDE_AXES,))
    field_name = AXIS_TO_FIELD[axis]
    metrics: dict[str, Any] = {
        "axis": axis,
        "attitude_field": field_name,
        "pulse_command": pulse_command.as_dict(),
        "expected_sign": expected_sign,
        "min_delta_deg": min_delta_deg,
        "max_delta_deg": max_delta_deg,
        "host": host,
        "port": port,
        "path": path,
    }
    baseline, baseline_requests = sample_attitude(
        host=host,
        port=port,
        path=path,
        timeout=timeout,
        max_frames=max_frames,
        samples=baseline_samples,
        interval_s=interval_s,
    )
    metrics["baseline_samples"] = baseline
    metrics["baseline_request_metrics"] = baseline_requests
    if not baseline:
        return UinputAttitudeResponseResult(
            status=WAITING,
            summary="MSP_ATTITUDE baseline could not be read",
            metrics=metrics,
            notes=["ATTITUDE_BASELINE_UNREADABLE"],
        )
    try:
        adapter.send(pulse_command)
        if settle_s > 0:
            sleeper(settle_s)
        during, during_requests = sample_attitude(
            host=host,
            port=port,
            path=path,
            timeout=timeout,
            max_frames=max_frames,
            samples=during_samples,
            interval_s=interval_s,
        )
    finally:
        if release_command is not None:
            adapter.send(release_command)
        else:
            adapter.neutral()
    metrics["during_samples"] = during
    metrics["during_request_metrics"] = during_requests
    if not during:
        return UinputAttitudeResponseResult(
            status=WAITING,
            summary="MSP_ATTITUDE could not be read during the pulse",
            metrics=metrics,
            notes=["ATTITUDE_DURING_UNREADABLE"],
        )
    wrap = axis == "yaw"
    series = [baseline[-1][field_name]] + [item[field_name] for item in during]
    delta = accumulated_delta(series, wrap=wrap)
    metrics["baseline_median_deg"] = float(median(item[field_name] for item in baseline))
    metrics["delta_deg"] = delta
    metrics["observed_sign"] = 0 if delta == 0 else (1 if delta > 0 else -1)
    status, summary, notes = evaluate_attitude_response(
        axis=axis,
        delta_deg=delta,
        expected_sign=expected_sign,
        min_delta_deg=min_delta_deg,
        max_delta_deg=max_delta_deg,
    )
    return UinputAttitudeResponseResult(
        status=status,
        summary=summary,
        metrics=metrics,
        notes=notes,
    )
