#!/usr/bin/env python3
"""Pulse Kenet virtual RC axes while binding pr0p Controls -> RC Channels."""

from __future__ import annotations

import argparse
import json
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

PROBE_DIR = Path(__file__).resolve().parent
SANDBOX_DIR = PROBE_DIR.parent / "game_screen_sandbox"
if str(SANDBOX_DIR) not in sys.path:
    sys.path.insert(0, str(SANDBOX_DIR))

from virtual_input import (  # noqa: E402
    AxisCommand,
    DryRunInputAdapter,
    UInputAdapter,
    UInputUnavailable,
    probe_uinput_environment,
)


PASS = "PASS"
WAITING = "WAITING"
FAIL = "FAIL"
DEFAULT_LOG_DIR = Path("logs/simitl_pr0p")
ROLE_ORDER = ("roll", "pitch", "throttle", "yaw", "aux1")


@dataclass
class MappingAssistantResult:
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


def expand_roles(roles: list[str]) -> list[str]:
    expanded: list[str] = []
    for role in roles:
        values = list(ROLE_ORDER) if role == "all" else [role]
        for value in values:
            if value not in expanded:
                expanded.append(value)
    return expanded


def command_for_role(role: str, amplitude: float) -> AxisCommand:
    kwargs = {name: 0.0 for name in ROLE_ORDER}
    if role not in kwargs:
        raise ValueError("unknown role: %s" % role)
    kwargs[role] = amplitude
    return AxisCommand(**kwargs).clamped()


def build_pulse_events(
    roles: list[str],
    *,
    amplitude: float,
    pulse_seconds: float,
    neutral_seconds: float,
    between_roles_seconds: float,
) -> list[dict[str, Any]]:
    events: list[dict[str, Any]] = []
    for role in roles:
        for direction, value in (("positive", amplitude), ("negative", -amplitude)):
            command = command_for_role(role, value)
            events.append({
                "role": role,
                "direction": direction,
                "command": command.as_dict(),
                "duration_s": pulse_seconds,
            })
            events.append({
                "role": role,
                "direction": "neutral",
                "command": AxisCommand().as_dict(),
                "duration_s": neutral_seconds,
            })
        if between_roles_seconds:
            events.append({
                "role": role,
                "direction": "between_roles_wait",
                "command": AxisCommand().as_dict(),
                "duration_s": between_roles_seconds,
            })
    return events


def sleep_if_live(seconds: float, *, live: bool) -> None:
    if live and seconds > 0:
        time.sleep(seconds)


def run_mapping_assistant(
    *,
    roles: list[str],
    use_uinput: bool,
    device_name: str,
    amplitude: float,
    setup_delay_s: float,
    pulse_seconds: float,
    neutral_seconds: float,
    between_roles_seconds: float,
    hold_neutral_seconds: float,
) -> MappingAssistantResult:
    role_sequence = expand_roles(roles)
    probe = probe_uinput_environment()
    metrics: dict[str, Any] = {
        "roles": role_sequence,
        "device_name": device_name,
        "amplitude": amplitude,
        "setup_delay_s": setup_delay_s,
        "pulse_seconds": pulse_seconds,
        "neutral_seconds": neutral_seconds,
        "between_roles_seconds": between_roles_seconds,
        "hold_neutral_seconds": hold_neutral_seconds,
        "real_input_sent": use_uinput,
        "uinput_probe": probe.as_dict(),
        "events": build_pulse_events(
            role_sequence,
            amplitude=amplitude,
            pulse_seconds=pulse_seconds,
            neutral_seconds=neutral_seconds,
            between_roles_seconds=between_roles_seconds,
        ),
    }
    if use_uinput and not probe.available:
        return MappingAssistantResult(
            status=WAITING,
            summary="uinput is not ready, so the virtual RC mapping assistant cannot send live pulses",
            metrics=metrics,
            notes=["UINPUT_PERMISSION_FAIL"],
        )

    adapter = None
    try:
        adapter = UInputAdapter(name=device_name) if use_uinput else DryRunInputAdapter()
        adapter.neutral()
        sleep_if_live(setup_delay_s, live=use_uinput)
        for event in metrics["events"]:
            command = AxisCommand(**event["command"])
            adapter.send(command)
            sleep_if_live(float(event["duration_s"]), live=use_uinput)
        adapter.neutral()
        sleep_if_live(hold_neutral_seconds, live=use_uinput)
        if isinstance(adapter, DryRunInputAdapter):
            metrics["commands"] = [cmd.as_dict() for _ts, cmd in adapter.commands]
        return MappingAssistantResult(
            status=PASS,
            summary=(
                "live virtual RC mapping pulses were sent"
                if use_uinput else
                "dry-run virtual RC mapping pulse plan was generated"
            ),
            metrics=metrics,
            notes=[
                "open pr0p Controls -> RC Channels and bind the listed roles in order",
                "dry-run only; no OS input event was sent" if not use_uinput else "real OS input was sent with explicit ack",
            ],
        )
    except UInputUnavailable as exc:
        metrics["error"] = str(exc)
        return MappingAssistantResult(
            status=WAITING,
            summary="uinput became unavailable while creating the virtual controller",
            metrics=metrics,
            notes=["UINPUT_PERMISSION_FAIL"],
        )
    except Exception as exc:
        metrics["error"] = str(exc)
        return MappingAssistantResult(
            status=FAIL,
            summary="virtual RC mapping assistant failed",
            metrics=metrics,
            notes=["PR0P_RC_MAPPING_ASSISTANT_FAIL"],
        )
    finally:
        if adapter is not None:
            try:
                adapter.close()
            except Exception:
                pass


def build_markdown(result: MappingAssistantResult, *, run_id: str) -> str:
    lines = [
        "# SimITL / pr0p RC Channel Mapping Assistant",
        "",
        "Run: `%s`" % run_id,
        "Verdict: `%s`" % result.status,
        "",
        result.summary,
        "",
        "## Controls -> RC Channels Pulse Order",
        "",
        "| Role | Pulse | Duration |",
        "| --- | --- | --- |",
    ]
    for event in result.metrics.get("events", []):
        if event.get("direction") in ("positive", "negative"):
            lines.append("| %s | `%s` | `%s` |" % (
                event["role"],
                event["direction"],
                event["duration_s"],
            ))
    lines.extend([
        "",
        "```json",
        json.dumps(result.metrics, indent=2, sort_keys=True),
        "```",
    ])
    for note in result.notes:
        lines.append("- %s" % note)
    lines.append("")
    return "\n".join(lines)


def write_reports(result: MappingAssistantResult, log_dir: Path, *, run_id: str) -> tuple[Path, Path]:
    log_dir.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime("%Y%m%d-%H%M%S", time.localtime())
    json_path = log_dir / ("%s-%s-rc-channel-mapping.json" % (stamp, run_id))
    md_path = log_dir / ("%s-%s-rc-channel-mapping.md" % (stamp, run_id))
    json_path.write_text(json.dumps(result.as_dict(), indent=2, sort_keys=True) + "\n",
                         encoding="utf-8")
    md_path.write_text(build_markdown(result, run_id=run_id), encoding="utf-8")
    return json_path, md_path


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-id", default="pr0p-rc-channel-mapping")
    parser.add_argument("--log-dir", type=Path, default=DEFAULT_LOG_DIR)
    parser.add_argument("--role", action="append", choices=("all",) + ROLE_ORDER)
    parser.add_argument("--device-name", default="Kenet Game Sandbox")
    parser.add_argument("--amplitude", type=float, default=0.6)
    parser.add_argument("--setup-delay", type=float, default=3.0)
    parser.add_argument("--pulse-seconds", type=float, default=1.0)
    parser.add_argument("--neutral-seconds", type=float, default=0.35)
    parser.add_argument("--between-roles-seconds", type=float, default=2.0)
    parser.add_argument("--hold-neutral-seconds", type=float, default=3.0)
    parser.add_argument("--uinput", action="store_true",
                        help="Create a real uinput virtual controller and send pulses")
    parser.add_argument("--ack-live-input", action="store_true",
                        help="Required before --uinput sends real OS input")
    args = parser.parse_args(argv)
    for name in (
        "setup_delay",
        "pulse_seconds",
        "neutral_seconds",
        "between_roles_seconds",
        "hold_neutral_seconds",
    ):
        if getattr(args, name) < 0:
            parser.error("--%s must be non-negative" % name.replace("_", "-"))
    if not 0 < args.amplitude <= 1.0:
        parser.error("--amplitude must be in (0, 1]")
    if args.uinput and not args.ack_live_input:
        parser.error("--uinput requires --ack-live-input")
    if args.role is None:
        args.role = ["all"]
    return args


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    result = run_mapping_assistant(
        roles=args.role,
        use_uinput=args.uinput,
        device_name=args.device_name,
        amplitude=args.amplitude,
        setup_delay_s=args.setup_delay,
        pulse_seconds=args.pulse_seconds,
        neutral_seconds=args.neutral_seconds,
        between_roles_seconds=args.between_roles_seconds,
        hold_neutral_seconds=args.hold_neutral_seconds,
    )
    json_path, md_path = write_reports(result, args.log_dir, run_id=args.run_id)
    print("simitl-pr0p-rc-channel-mapping %s report=%s summary=%s real_input_sent=%s" % (
        result.status,
        json_path,
        md_path,
        result.metrics.get("real_input_sent"),
    ))
    print(result.summary)
    return 1 if result.status == FAIL else 0


if __name__ == "__main__":
    raise SystemExit(main())
