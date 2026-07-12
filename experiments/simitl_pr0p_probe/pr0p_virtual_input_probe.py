#!/usr/bin/env python3
"""Virtual input probe for the isolated SimITL/pr0p experiment."""

from __future__ import annotations

import argparse
import json
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

REPO_ROOT = Path(__file__).resolve().parents[2]
SANDBOX_DIR = REPO_ROOT / "experiments" / "game_screen_sandbox"
if str(SANDBOX_DIR) not in sys.path:
    sys.path.insert(0, str(SANDBOX_DIR))

from virtual_input import (  # noqa: E402
    AxisCommand,
    DryRunInputAdapter,
    UInputUnavailable,
    probe_uinput_environment,
    run_uinput_smoke,
)


PASS = "PASS"
WAITING = "WAITING"
FAIL = "FAIL"
DEFAULT_LOG_DIR = Path("logs/simitl_pr0p")


@dataclass
class InputProbeResult:
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


def command_from_args(yaw: float, pitch: float, roll: float, throttle: float) -> AxisCommand:
    return AxisCommand(yaw=yaw, pitch=pitch, roll=roll, throttle=throttle).clamped()


def dry_run_smoke(command: AxisCommand, *, hold_seconds: float) -> InputProbeResult:
    adapter = DryRunInputAdapter()
    try:
        adapter.send(command)
        if hold_seconds:
            time.sleep(hold_seconds)
        adapter.neutral()
    finally:
        adapter.close()
    commands = [cmd.as_dict() for _ts, cmd in adapter.commands]
    last = adapter.last_command
    neutralized = bool(last and last.is_neutral())
    return InputProbeResult(
        status=PASS if neutralized else FAIL,
        summary=(
            "dry-run virtual input command was generated and neutralized"
            if neutralized else
            "dry-run virtual input did not return to neutral"
        ),
        metrics={
            "adapter": adapter.__class__.__name__,
            "real_input_sent": False,
            "requested_command": command.as_dict(),
            "commands": commands,
            "neutralized": neutralized,
            "closed": adapter.closed,
            "hold_seconds": hold_seconds,
        },
        notes=["dry-run only; no OS input event was sent"],
    )


def run_input_probe(
    *,
    command: AxisCommand,
    hold_seconds: float,
    require_uinput: bool,
    uinput_smoke: bool,
) -> InputProbeResult:
    probe = probe_uinput_environment()
    if uinput_smoke:
        try:
            smoke = run_uinput_smoke(command, hold_seconds=hold_seconds)
        except UInputUnavailable as exc:
            return InputProbeResult(
                status=WAITING,
                summary="real uinput smoke is not available yet",
                metrics={
                    "uinput_probe": probe.as_dict(),
                    "requested_command": command.as_dict(),
                    "error": str(exc),
                    "real_input_sent": False,
                },
                notes=["UINPUT_PERMISSION_FAIL"],
            )
        except Exception as exc:
            return InputProbeResult(
                status=FAIL,
                summary="real uinput smoke failed",
                metrics={
                    "uinput_probe": probe.as_dict(),
                    "requested_command": command.as_dict(),
                    "error": str(exc),
                    "real_input_sent": True,
                },
                notes=["UINPUT_SMOKE_FAIL"],
            )
        return InputProbeResult(
            status=PASS,
            summary="real uinput smoke created a virtual controller and neutralized",
            metrics={
                "uinput_probe": probe.as_dict(),
                "smoke": smoke,
                "requested_command": command.as_dict(),
                "real_input_sent": True,
            },
            notes=["verify pr0p axis mapping visually before using PID output"],
        )

    result = dry_run_smoke(command, hold_seconds=hold_seconds)
    result.metrics["uinput_probe"] = probe.as_dict()
    result.metrics["require_uinput"] = require_uinput
    if require_uinput and not probe.available:
        result.status = WAITING
        result.summary = "dry-run passed, but real uinput prerequisites are not ready"
        result.notes.append("UINPUT_PERMISSION_FAIL")
    elif require_uinput:
        result.summary = "dry-run passed and real uinput prerequisites look ready"
        result.notes.append("real uinput prerequisites look ready; live smoke still requires ack")
    elif probe.available:
        result.notes.append("real uinput prerequisites look ready; live smoke still requires ack")
    else:
        result.notes.append("real uinput prerequisites are incomplete")
    return result


def build_markdown(result: InputProbeResult, *, run_id: str) -> str:
    lines = [
        "# SimITL / pr0p Virtual Input Probe",
        "",
        "Run: `%s`" % run_id,
        "Verdict: `%s`" % result.status,
        "",
        result.summary,
        "",
        "| Metric | Value |",
        "| --- | --- |",
    ]
    for key in ("real_input_sent", "neutralized", "adapter", "require_uinput"):
        if key in result.metrics:
            lines.append("| %s | `%s` |" % (key, result.metrics[key]))
    lines.extend([
        "",
        "## Metrics",
        "",
        "```json",
        json.dumps(result.metrics, indent=2, sort_keys=True),
        "```",
    ])
    for note in result.notes:
        lines.append("- %s" % note)
    lines.append("")
    return "\n".join(lines)


def write_reports(result: InputProbeResult, log_dir: Path, *, run_id: str) -> tuple[Path, Path]:
    log_dir.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime("%Y%m%d-%H%M%S", time.localtime())
    json_path = log_dir / ("%s-%s-input-probe.json" % (stamp, run_id))
    md_path = log_dir / ("%s-%s-input-probe.md" % (stamp, run_id))
    json_path.write_text(
        json.dumps(result.as_dict(), indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    md_path.write_text(build_markdown(result, run_id=run_id), encoding="utf-8")
    return json_path, md_path


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-id", default="pr0p-input")
    parser.add_argument("--log-dir", type=Path, default=DEFAULT_LOG_DIR)
    parser.add_argument("--yaw", type=float, default=0.05)
    parser.add_argument("--pitch", type=float, default=0.0)
    parser.add_argument("--roll", type=float, default=0.0)
    parser.add_argument("--throttle", type=float, default=0.0)
    parser.add_argument("--hold-seconds", type=float, default=0.0)
    parser.add_argument("--require-uinput", action="store_true",
                        help="Return WAITING unless /dev/uinput is writable")
    parser.add_argument("--uinput-smoke", action="store_true",
                        help="Create a real uinput virtual controller")
    parser.add_argument("--ack-live-input", action="store_true",
                        help="Required before --uinput-smoke sends real OS input")
    args = parser.parse_args(argv)
    if args.hold_seconds < 0:
        parser.error("--hold-seconds must be non-negative")
    if args.uinput_smoke and not args.ack_live_input:
        parser.error("--uinput-smoke requires --ack-live-input")
    return args


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    command = command_from_args(args.yaw, args.pitch, args.roll, args.throttle)
    result = run_input_probe(
        command=command,
        hold_seconds=args.hold_seconds,
        require_uinput=args.require_uinput,
        uinput_smoke=args.uinput_smoke,
    )
    json_path, md_path = write_reports(result, args.log_dir, run_id=args.run_id)
    print("simitl-pr0p-input %s report=%s summary=%s real_input_sent=%s" % (
        result.status,
        json_path,
        md_path,
        result.metrics.get("real_input_sent"),
    ))
    print(result.summary)
    return 1 if result.status == FAIL else 0


if __name__ == "__main__":
    raise SystemExit(main())
