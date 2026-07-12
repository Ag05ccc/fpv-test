#!/usr/bin/env python3
"""Virtual input adapters for the game/screen sandbox.

The default adapter is DryRunInputAdapter. It records commands but does not send
anything to the OS, which keeps the first smoke tests safe and repeatable.
"""

from __future__ import annotations

import argparse
import importlib.util
import os
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Protocol


@dataclass(frozen=True)
class AxisCommand:
    yaw: float = 0.0
    pitch: float = 0.0
    roll: float = 0.0
    throttle: float = 0.0
    aux1: float = 0.0

    def clamped(self) -> "AxisCommand":
        return AxisCommand(
            yaw=clamp_axis(self.yaw),
            pitch=clamp_axis(self.pitch),
            roll=clamp_axis(self.roll),
            throttle=clamp_axis(self.throttle),
            aux1=clamp_axis(self.aux1),
        )

    def is_neutral(self, *, eps: float = 1e-6) -> bool:
        cmd = self.clamped()
        return all(abs(v) <= eps for v in (cmd.yaw, cmd.pitch, cmd.roll, cmd.throttle, cmd.aux1))

    def as_dict(self) -> dict[str, float]:
        cmd = self.clamped()
        return {
            "yaw": cmd.yaw,
            "pitch": cmd.pitch,
            "roll": cmd.roll,
            "throttle": cmd.throttle,
            "aux1": cmd.aux1,
        }


class InputAdapter(Protocol):
    def send(self, command: AxisCommand) -> None:
        ...

    def press_button(self, button: str, pressed: bool = True) -> None:
        ...

    def neutral(self) -> None:
        ...

    def close(self) -> None:
        ...


def clamp_axis(value: float) -> float:
    return max(-1.0, min(1.0, float(value)))


def rc_to_axis(value: int, *, center: int = 1500, span: int = 500) -> float:
    if span <= 0:
        raise ValueError("span must be positive")
    return clamp_axis((int(value) - center) / float(span))


def command_from_channels(channels: list[int], cfg) -> AxisCommand:
    return AxisCommand(
        roll=rc_to_axis(channels[cfg.roll_ch], center=cfg.rc_center),
        pitch=rc_to_axis(channels[cfg.pitch_ch], center=cfg.rc_center),
        throttle=rc_to_axis(channels[cfg.throttle_ch], center=cfg.throttle_neutral),
        yaw=rc_to_axis(channels[cfg.yaw_ch], center=cfg.rc_center),
    ).clamped()


BUTTON_ALIASES = {
    "south": "south",
    "btn_south": "south",
    "BTN_SOUTH": "south",
    "east": "east",
    "btn_east": "east",
    "BTN_EAST": "east",
}


def normalize_button_name(button: str) -> str:
    key = str(button).strip()
    normalized = BUTTON_ALIASES.get(key) or BUTTON_ALIASES.get(key.lower())
    if normalized is None:
        raise ValueError("unknown button: %s" % button)
    return normalized


class DryRunInputAdapter:
    def __init__(self):
        self.commands: list[tuple[float, AxisCommand]] = []
        self.button_events: list[tuple[float, str, bool]] = []
        self.closed = False

    @property
    def last_command(self) -> AxisCommand | None:
        return self.commands[-1][1] if self.commands else None

    def send(self, command: AxisCommand) -> None:
        if self.closed:
            raise RuntimeError("adapter is closed")
        self.commands.append((time.monotonic(), command.clamped()))

    def press_button(self, button: str, pressed: bool = True) -> None:
        if self.closed:
            raise RuntimeError("adapter is closed")
        self.button_events.append((time.monotonic(), normalize_button_name(button), bool(pressed)))

    def neutral(self) -> None:
        for button in ("south", "east"):
            self.press_button(button, False)
        self.send(AxisCommand())

    def close(self) -> None:
        if not self.closed:
            self.neutral()
            self.closed = True


class UInputUnavailable(RuntimeError):
    pass


@dataclass(frozen=True)
class UInputProbe:
    available: bool
    evdev_available: bool
    dev_uinput_exists: bool
    dev_uinput_readable: bool
    dev_uinput_writable: bool
    error: str | None = None

    def as_dict(self) -> dict[str, bool | str | None]:
        return {
            "available": self.available,
            "evdev_available": self.evdev_available,
            "dev_uinput_exists": self.dev_uinput_exists,
            "dev_uinput_readable": self.dev_uinput_readable,
            "dev_uinput_writable": self.dev_uinput_writable,
            "error": self.error,
        }


def probe_uinput_environment() -> UInputProbe:
    evdev_available = importlib.util.find_spec("evdev") is not None
    path = Path("/dev/uinput")
    exists = path.exists()
    readable = os.access(path, os.R_OK) if exists else False
    writable = os.access(path, os.W_OK) if exists else False
    error = None
    if not evdev_available:
        error = "evdev is not installed in this Python environment"
    elif not exists:
        error = "/dev/uinput does not exist"
    elif not writable:
        error = "/dev/uinput is not writable by this user"
    return UInputProbe(
        available=evdev_available and exists and writable,
        evdev_available=evdev_available,
        dev_uinput_exists=exists,
        dev_uinput_readable=readable,
        dev_uinput_writable=writable,
        error=error,
    )


class UInputAdapter:
    """Small optional uinput adapter.

    This is intentionally conservative. It is not used by default and should be
    enabled only after manual neutral/kill-switch validation.
    """

    def __init__(self, *, name: str = "Kenet Game Sandbox"):
        try:
            from evdev import UInput, ecodes as e  # type: ignore
        except ImportError as exc:
            raise UInputUnavailable("evdev is required for uinput output") from exc

        self._e = e
        capabilities = {
            e.EV_ABS: [
                (e.ABS_X, (0, -32768, 32767, 0, 0, 0)),
                (e.ABS_Y, (0, -32768, 32767, 0, 0, 0)),
                (e.ABS_Z, (0, -32768, 32767, 0, 0, 0)),
                (e.ABS_RX, (0, -32768, 32767, 0, 0, 0)),
                (e.ABS_RY, (0, -32768, 32767, 0, 0, 0)),
            ],
            e.EV_KEY: [e.BTN_SOUTH, e.BTN_EAST],
        }
        self._ui = UInput(capabilities, name=name)
        self._button_codes = {
            "south": e.BTN_SOUTH,
            "east": e.BTN_EAST,
        }
        self.closed = False
        self.neutral()

    def _write_abs(self, code: int, value: float) -> None:
        scaled = int(clamp_axis(value) * 32767)
        self._ui.write(self._e.EV_ABS, code, scaled)

    def send(self, command: AxisCommand) -> None:
        if self.closed:
            raise RuntimeError("adapter is closed")
        cmd = command.clamped()
        self._write_abs(self._e.ABS_X, cmd.roll)
        self._write_abs(self._e.ABS_Y, -cmd.pitch)
        self._write_abs(self._e.ABS_Z, cmd.aux1)
        self._write_abs(self._e.ABS_RX, cmd.yaw)
        self._write_abs(self._e.ABS_RY, -cmd.throttle)
        self._ui.syn()

    def press_button(self, button: str, pressed: bool = True) -> None:
        if self.closed:
            raise RuntimeError("adapter is closed")
        normalized = normalize_button_name(button)
        self._ui.write(self._e.EV_KEY, self._button_codes[normalized], 1 if pressed else 0)
        self._ui.syn()

    def neutral(self) -> None:
        for button in ("south", "east"):
            self.press_button(button, False)
        self.send(AxisCommand())

    def close(self) -> None:
        if not self.closed:
            self.neutral()
            self._ui.close()
            self.closed = True


def run_uinput_smoke(
    command: AxisCommand | None = None,
    *,
    hold_seconds: float = 0.0,
) -> dict[str, object]:
    probe = probe_uinput_environment()
    if not probe.available:
        raise UInputUnavailable(probe.error or "uinput is unavailable")
    adapter = UInputAdapter()
    try:
        cmd = command or AxisCommand(yaw=0.05)
        adapter.send(cmd)
        if hold_seconds:
            time.sleep(hold_seconds)
        adapter.neutral()
        return {
            "probe": probe.as_dict(),
            "command": cmd.clamped().as_dict(),
            "last_command": AxisCommand().as_dict(),
            "adapter": adapter.__class__.__name__,
        }
    finally:
        adapter.close()


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--uinput", action="store_true")
    parser.add_argument("--probe-uinput", action="store_true")
    parser.add_argument("--uinput-smoke", action="store_true")
    parser.add_argument("--yaw", type=float, default=0.0)
    parser.add_argument("--pitch", type=float, default=0.0)
    parser.add_argument("--roll", type=float, default=0.0)
    parser.add_argument("--throttle", type=float, default=0.0)
    parser.add_argument("--aux1", type=float, default=0.0)
    parser.add_argument("--hold-seconds", type=float, default=0.5)
    args = parser.parse_args(argv)
    if args.hold_seconds < 0:
        parser.error("--hold-seconds must be non-negative")
    return args


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    if args.probe_uinput:
        probe = probe_uinput_environment()
        verdict = "PASS" if probe.available else "WAITING"
        print("uinput-probe %s %s" % (verdict, probe.as_dict()))
        return 0 if probe.available else 2
    if args.uinput_smoke:
        command = AxisCommand(args.yaw, args.pitch, args.roll, args.throttle, args.aux1).clamped()
        try:
            result = run_uinput_smoke(command, hold_seconds=args.hold_seconds)
        except UInputUnavailable as exc:
            probe = probe_uinput_environment()
            print("uinput-smoke WAITING error=%s probe=%s" % (exc, probe.as_dict()))
            return 2
        print("uinput-smoke PASS %s" % result)
        return 0

    adapter: InputAdapter
    if args.uinput:
        adapter = UInputAdapter()
    else:
        adapter = DryRunInputAdapter()
    try:
        command = AxisCommand(args.yaw, args.pitch, args.roll, args.throttle, args.aux1).clamped()
        adapter.send(command)
        time.sleep(args.hold_seconds)
        adapter.neutral()
        print("input PASS command=%s adapter=%s" % (
            command.as_dict(), adapter.__class__.__name__))
    finally:
        adapter.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
