#!/usr/bin/env python3
"""Shared arm/hover sequence helpers for live pr0p control gates.

Betaflight latches ARM_SWITCH if AUX1 goes high while boot-grace/calibration
blockers are still active, so the sequence holds throttle-low first, verifies
the armed state over read-only MSP, then raises hover throttle.
"""

from __future__ import annotations

import sys
import time
from pathlib import Path
from typing import Any, Callable

PROBE_DIR = Path(__file__).resolve().parent
if str(PROBE_DIR) not in sys.path:
    sys.path.insert(0, str(PROBE_DIR))
SANDBOX_DIR = PROBE_DIR.parent / "game_screen_sandbox"
if str(SANDBOX_DIR) not in sys.path:
    sys.path.insert(0, str(SANDBOX_DIR))

from msp_ws_status_probe import run_msp_ws_status_once  # noqa: E402
from pr0p_response_probe import command_for_axis as response_command_for_axis  # noqa: E402
from virtual_input import AxisCommand  # noqa: E402


def arm_and_hover_for_response(
    adapter: Any,
    *,
    ws_host: str,
    ws_port: int,
    ws_path: str = "/",
    timeout: float = 1.0,
    max_frames: int = 8,
    arm_wait_s: float = 12.0,
    arm_settle_s: float = 2.0,
    arm_throttle: float = -0.35,
    hover_wait_s: float = 2.0,
    status_reader: Callable[..., Any] = run_msp_ws_status_once,
    sleeper: Callable[[float], None] = time.sleep,
) -> tuple[bool, dict[str, Any], list[str]]:
    """Arm via throttle-low then AUX1-high, verify read-only FC status, hold hover throttle."""
    metrics: dict[str, Any] = {
        "arm_wait_s": arm_wait_s,
        "arm_settle_s": arm_settle_s,
        "arm_throttle": arm_throttle,
        "hover_wait_s": hover_wait_s,
    }
    adapter.send(AxisCommand(throttle=1.0))
    sleeper(arm_wait_s)
    adapter.send(AxisCommand(throttle=1.0, aux1=1.0))
    sleeper(arm_settle_s)
    try:
        status = status_reader(
            host=ws_host,
            port=ws_port,
            path=ws_path,
            timeout=timeout,
            max_frames=max_frames,
        )
        fc_status = (status.metrics or {}).get("fc_status") or {}
    except Exception as exc:
        adapter.neutral()
        metrics["error"] = str(exc)
        return False, metrics, ["ARM_FIRST_STATUS_UNREADABLE"]
    metrics["armed_before_pulse"] = bool(fc_status.get("armed"))
    metrics["active_modes"] = fc_status.get("active_modes")
    metrics["arming_disable_names"] = fc_status.get("arming_disable_names")
    if not fc_status.get("armed"):
        adapter.neutral()
        blockers = ",".join(fc_status.get("arming_disable_names") or []) or "UNKNOWN"
        return False, metrics, ["ARM_FIRST_NOT_ARMED", "ARM_BLOCKERS:%s" % blockers]
    adapter.send(AxisCommand(throttle=arm_throttle, aux1=1.0))
    sleeper(hover_wait_s)
    return True, metrics, ["ARM_FIRST_ARMED"]


def response_pulse_command(
    axis: str,
    magnitude: float,
    *,
    arm_first: bool,
    arm_throttle: float,
) -> AxisCommand:
    if not arm_first:
        return response_command_for_axis(axis, magnitude)
    values = {"throttle": arm_throttle, "aux1": 1.0}
    values[axis] = magnitude
    return AxisCommand(**values)


class HoldAxesAdapter:
    """Adapter wrapper that keeps the arm/hover axes held under every command.

    neutral() returns to the hold state instead of a full release so a tracker
    stop command does not disarm the vehicle mid-run; close() releases
    everything through the wrapped adapter.

    initial_throttle/initial_sends implement an in-loop takeoff: the first
    initial_sends commands hold initial_throttle (e.g. +1.0 = low, grounded)
    so the tracker can lock the target on the pad while capture spins up,
    then the hold switches to the flight throttle and the vehicle lifts on
    camera.
    """

    def __init__(
        self,
        adapter: Any,
        *,
        throttle: float,
        aux1: float = 1.0,
        initial_throttle: float | None = None,
        initial_sends: int = 0,
        throttle_schedule: list[tuple[int, float]] | None = None,
        throttle_provider: Any = None,
    ):
        self._adapter = adapter
        self._throttle = throttle
        self._aux1 = aux1
        self._sends = 0
        if throttle_schedule is None:
            throttle_schedule = []
            if initial_throttle is not None and initial_sends > 0:
                throttle_schedule.append((initial_sends, initial_throttle))
        self._schedule = list(throttle_schedule)
        self._throttle_provider = throttle_provider

    def _current_throttle(self) -> float:
        remaining = self._sends
        for count, value in self._schedule:
            if remaining < count:
                return value
            remaining -= count
        if self._throttle_provider is not None:
            return float(self._throttle_provider())
        return self._throttle

    def send(self, command: AxisCommand) -> None:
        throttle = self._current_throttle()
        self._sends += 1
        self._adapter.send(AxisCommand(
            yaw=command.yaw,
            pitch=command.pitch,
            roll=command.roll,
            throttle=throttle,
            aux1=self._aux1,
        ))

    def press_button(self, button: str, pressed: bool = True) -> None:
        self._adapter.press_button(button, pressed)

    def neutral(self) -> None:
        self._adapter.send(AxisCommand(throttle=self._current_throttle(), aux1=self._aux1))

    def close(self) -> None:
        try:
            self._adapter.neutral()
        finally:
            self._adapter.close()
