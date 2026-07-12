#!/usr/bin/env python3
"""Help bind simulator RC channels to the sandbox virtual joystick."""

from __future__ import annotations

import argparse
import json
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from virtual_input import (  # noqa: E402
    AxisCommand,
    DryRunInputAdapter,
    InputAdapter,
    UInputAdapter,
    UInputUnavailable,
    probe_uinput_environment,
)


PASS = "PASS"
FAIL = "FAIL"
WAITING = "WAITING"
AXIS_NAMES = ("yaw", "pitch", "roll", "throttle")
AXIS_BINDING_HINTS = {
    "roll": "ABS_X",
    "pitch": "ABS_Y inverted",
    "yaw": "ABS_RX",
    "throttle": "ABS_RY inverted",
}
DEFAULT_LOG_DIR = Path("logs/game_screen_sandbox")


@dataclass(frozen=True)
class BindingStep:
    axis: str
    phase: str
    command: AxisCommand
    duration_s: float

    def as_dict(self) -> dict[str, Any]:
        return {
            "axis": self.axis,
            "phase": self.phase,
            "command": self.command.as_dict(),
            "duration_s": self.duration_s,
        }


@dataclass
class BindingRunReport:
    run_id: str
    started_at: str
    status: str
    summary: str
    steps: list[BindingStep] = field(default_factory=list)
    metrics: dict[str, Any] = field(default_factory=dict)
    notes: list[str] = field(default_factory=list)

    def as_dict(self) -> dict[str, Any]:
        return {
            "run_id": self.run_id,
            "started_at": self.started_at,
            "status": self.status,
            "summary": self.summary,
            "steps": [step.as_dict() for step in self.steps],
            "metrics": self.metrics,
            "notes": self.notes,
        }


def timestamp_slug() -> str:
    return time.strftime("%Y%m%d-%H%M%S", time.localtime())


def normalize_axes(axes: list[str] | tuple[str, ...] | str) -> tuple[str, ...]:
    if isinstance(axes, str):
        raw_axes = [part.strip() for part in axes.split(",")]
    else:
        raw_axes = [str(part).strip() for part in axes]
    selected = tuple(axis for axis in raw_axes if axis)
    if not selected:
        raise ValueError("at least one axis must be selected")
    unknown = [axis for axis in selected if axis not in AXIS_NAMES]
    if unknown:
        raise ValueError("unknown axis name(s): %s" % ",".join(unknown))
    return selected


def axis_command(axis: str, value: float) -> AxisCommand:
    if axis not in AXIS_NAMES:
        raise ValueError("unknown axis: %s" % axis)
    kwargs = {name: 0.0 for name in AXIS_NAMES}
    kwargs[axis] = value
    return AxisCommand(**kwargs).clamped()


def normalize_bbox(value: Any) -> tuple[int, int, int, int] | None:
    if not isinstance(value, (list, tuple)) or len(value) != 4:
        return None
    try:
        x, y, w, h = [int(part) for part in value]
    except (TypeError, ValueError):
        return None
    if w <= 0 or h <= 0:
        return None
    return x, y, w, h


def load_context_bbox_file(path: Path) -> tuple[int, int, int, int]:
    data = json.loads(path.read_text(encoding="utf-8"))
    for key in ("bbox", "tracker_bbox", "initial_bbox"):
        bbox = normalize_bbox(data.get(key))
        if bbox is not None:
            return bbox
    metrics = data.get("metrics")
    if isinstance(metrics, dict):
        bbox = normalize_bbox(metrics.get("tracker_bbox"))
        if bbox is not None:
            return bbox
    raise ValueError("bbox file does not contain bbox/tracker_bbox: %s" % path)


def build_binding_steps(
    *,
    axes: tuple[str, ...],
    axis_value: float,
    hold_seconds: float,
    neutral_seconds: float,
    cycles: int,
    include_negative: bool,
) -> list[BindingStep]:
    if not 0 < abs(axis_value) <= 1:
        raise ValueError("axis_value magnitude must be in (0, 1]")
    if hold_seconds < 0 or neutral_seconds < 0:
        raise ValueError("hold_seconds and neutral_seconds must be non-negative")
    if cycles <= 0:
        raise ValueError("cycles must be positive")
    selected_axes = normalize_axes(axes)
    steps: list[BindingStep] = []
    for axis in selected_axes:
        for cycle in range(cycles):
            suffix = str(cycle + 1)
            steps.append(BindingStep(
                axis=axis,
                phase="neutral-before-%s" % suffix,
                command=AxisCommand(),
                duration_s=neutral_seconds,
            ))
            steps.append(BindingStep(
                axis=axis,
                phase="positive-%s" % suffix,
                command=axis_command(axis, axis_value),
                duration_s=hold_seconds,
            ))
            steps.append(BindingStep(
                axis=axis,
                phase="neutral-middle-%s" % suffix,
                command=AxisCommand(),
                duration_s=neutral_seconds,
            ))
            if include_negative:
                steps.append(BindingStep(
                    axis=axis,
                    phase="negative-%s" % suffix,
                    command=axis_command(axis, -axis_value),
                    duration_s=hold_seconds,
                ))
                steps.append(BindingStep(
                    axis=axis,
                    phase="neutral-after-%s" % suffix,
                    command=AxisCommand(),
                    duration_s=neutral_seconds,
                ))
    return steps


def build_markdown(report: BindingRunReport) -> str:
    lines = [
        "# RC Binding Assistant",
        "",
        "Run: `%s`" % report.run_id,
        "Started: `%s`" % report.started_at,
        "Status: `%s`" % report.status,
        "",
        report.summary,
        "",
        "## Axis Hints",
        "",
        "| Sandbox axis | Virtual joystick code |",
        "| --- | --- |",
    ]
    for axis, hint in AXIS_BINDING_HINTS.items():
        lines.append("| %s | `%s` |" % (axis, hint))
    lines.extend([
        "",
        "## Steps",
        "",
        "| Axis | Phase | Duration s | Command |",
        "| --- | --- | ---: | --- |",
    ])
    for step in report.steps:
        lines.append("| %s | %s | %.3f | `%s` |" % (
            step.axis,
            step.phase,
            step.duration_s,
            json.dumps(step.command.as_dict(), sort_keys=True),
        ))
    lines.extend([
        "",
        "## Notes",
        "",
    ])
    if report.notes:
        lines.extend("- %s" % note for note in report.notes)
    else:
        lines.append("- no extra notes")
    lines.extend([
        "",
        "## Metrics",
        "",
        "```json",
        json.dumps(report.metrics, indent=2, sort_keys=True),
        "```",
        "",
    ])
    return "\n".join(lines)


def write_reports(report: BindingRunReport, log_dir: Path) -> tuple[Path, Path]:
    log_dir.mkdir(parents=True, exist_ok=True)
    slug = "%s-%s" % (timestamp_slug(), report.run_id)
    json_path = log_dir / ("%s-rc-binding.json" % slug)
    md_path = log_dir / ("%s-rc-binding.md" % slug)
    json_path.write_text(json.dumps(report.as_dict(), indent=2, sort_keys=True) + "\n",
                         encoding="utf-8")
    md_path.write_text(build_markdown(report), encoding="utf-8")
    return json_path, md_path


def execute_steps(
    adapter: InputAdapter,
    steps: list[BindingStep],
    *,
    sleep_fn: Callable[[float], None] = time.sleep,
) -> int:
    command_count = 0
    for step in steps:
        adapter.send(step.command)
        command_count += 1
        if step.duration_s > 0:
            sleep_fn(step.duration_s)
    adapter.neutral()
    command_count += 1
    return command_count


def run_binding_sequence(
    *,
    run_id: str,
    axes: tuple[str, ...],
    axis_value: float,
    hold_seconds: float,
    neutral_seconds: float,
    cycles: int,
    include_negative: bool,
    use_uinput: bool,
    ack_live_input: bool,
    countdown_seconds: float,
    window_title: str | None = None,
    tracker_bbox: tuple[int, int, int, int] | None = None,
    sleep_fn: Callable[[float], None] = time.sleep,
) -> BindingRunReport:
    started_at = time.strftime("%Y-%m-%dT%H:%M:%S%z", time.localtime())
    steps = build_binding_steps(
        axes=axes,
        axis_value=axis_value,
        hold_seconds=hold_seconds,
        neutral_seconds=neutral_seconds,
        cycles=cycles,
        include_negative=include_negative,
    )
    base_metrics: dict[str, Any] = {
        "axes": list(normalize_axes(axes)),
        "axis_value": axis_value,
        "axis_binding_hints": AXIS_BINDING_HINTS,
        "cycles": cycles,
        "include_negative": include_negative,
        "hold_seconds": hold_seconds,
        "neutral_seconds": neutral_seconds,
        "countdown_seconds": countdown_seconds,
        "real_input": use_uinput,
        "ack_live_input": ack_live_input,
        "window_title": window_title,
        "tracker_bbox": list(tracker_bbox) if tracker_bbox else None,
        "planned_step_count": len(steps),
    }
    if countdown_seconds < 0:
        raise ValueError("countdown_seconds must be non-negative")
    if use_uinput and not ack_live_input:
        return BindingRunReport(
            run_id=run_id,
            started_at=started_at,
            status=WAITING,
            summary="RC binding live input requires explicit acknowledgement",
            steps=steps,
            metrics={
                **base_metrics,
                "command_count": 0,
            },
            notes=["rerun with --uinput --ack-live-input after opening Controls -> RC Channels"],
        )
    probe = probe_uinput_environment() if use_uinput else None
    if use_uinput and probe and not probe.available:
        return BindingRunReport(
            run_id=run_id,
            started_at=started_at,
            status=WAITING,
            summary="RC binding live input prerequisites are not ready",
            steps=steps,
            metrics={
                **base_metrics,
                "command_count": 0,
                "uinput_probe": probe.as_dict(),
            },
            notes=[probe.error or "uinput is unavailable"],
        )
    if countdown_seconds > 0:
        sleep_fn(countdown_seconds)
    adapter: InputAdapter | None = None
    command_count = 0
    try:
        adapter = UInputAdapter() if use_uinput else DryRunInputAdapter()
        command_count = execute_steps(adapter, steps, sleep_fn=sleep_fn)
    except UInputUnavailable as exc:
        return BindingRunReport(
            run_id=run_id,
            started_at=started_at,
            status=WAITING,
            summary="RC binding live input is unavailable",
            steps=steps,
            metrics={
                **base_metrics,
                "command_count": command_count,
            },
            notes=[str(exc)],
        )
    finally:
        if adapter is not None:
            adapter.close()
    if adapter is None:
        return BindingRunReport(
            run_id=run_id,
            started_at=started_at,
            status=FAIL,
            summary="RC binding adapter was not created",
            steps=steps,
            metrics={
                **base_metrics,
                "command_count": command_count,
            },
            notes=["adapter construction ended without a usable adapter"],
        )
    return BindingRunReport(
        run_id=run_id,
        started_at=started_at,
        status=PASS,
        summary=(
            "RC binding live input sequence completed and neutralized"
            if use_uinput else
            "RC binding dry-run sequence completed and neutralized"
        ),
        steps=steps,
        metrics={
            **base_metrics,
            "adapter": adapter.__class__.__name__,
            "command_count": command_count,
            "neutralized": True,
            "uinput_probe": probe.as_dict() if probe else None,
        },
        notes=[
            "open the simulator Controls -> RC Channels page and bind one axis at a time"
            if use_uinput else
            "dry-run only; no OS joystick commands were sent",
        ],
    )


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-id", default="rc-binding")
    parser.add_argument("--log-dir", type=Path, default=DEFAULT_LOG_DIR)
    parser.add_argument("--axes", default="yaw,pitch,roll,throttle")
    parser.add_argument("--axis-value", type=float, default=0.6)
    parser.add_argument("--hold-seconds", type=float, default=0.8)
    parser.add_argument("--neutral-seconds", type=float, default=0.3)
    parser.add_argument("--cycles", type=int, default=1)
    parser.add_argument("--positive-only", action="store_true")
    parser.add_argument("--uinput", action="store_true")
    parser.add_argument("--ack-live-input", action="store_true")
    parser.add_argument("--countdown-seconds", type=float, default=0.0)
    parser.add_argument("--window-title",
                        help="Optional simulator window title for report matching")
    parser.add_argument("--tracker-bbox-file", type=Path,
                        help="Optional bbox report path for report matching")
    args = parser.parse_args(argv)
    try:
        args.axes = normalize_axes(args.axes)
    except ValueError as exc:
        parser.error(str(exc))
    if not 0 < abs(args.axis_value) <= 1:
        parser.error("--axis-value magnitude must be in (0, 1]")
    if args.hold_seconds < 0 or args.neutral_seconds < 0:
        parser.error("--hold-seconds/--neutral-seconds must be non-negative")
    if args.countdown_seconds < 0:
        parser.error("--countdown-seconds must be non-negative")
    if args.cycles <= 0:
        parser.error("--cycles must be positive")
    if args.tracker_bbox_file:
        try:
            args.tracker_bbox = load_context_bbox_file(args.tracker_bbox_file)
        except (OSError, ValueError, json.JSONDecodeError) as exc:
            parser.error(str(exc))
    else:
        args.tracker_bbox = None
    return args


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    report = run_binding_sequence(
        run_id=args.run_id,
        axes=args.axes,
        axis_value=args.axis_value,
        hold_seconds=args.hold_seconds,
        neutral_seconds=args.neutral_seconds,
        cycles=args.cycles,
        include_negative=not args.positive_only,
        use_uinput=args.uinput,
        ack_live_input=args.ack_live_input,
        countdown_seconds=args.countdown_seconds,
        window_title=args.window_title,
        tracker_bbox=args.tracker_bbox,
    )
    json_path, md_path = write_reports(report, args.log_dir)
    print("rc-binding-assistant %s report=%s summary=%s" % (
        report.status,
        json_path,
        md_path,
    ))
    if report.status == PASS:
        return 0
    if report.status == WAITING:
        return 2
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
