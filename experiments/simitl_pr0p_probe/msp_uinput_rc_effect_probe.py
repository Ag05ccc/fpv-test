#!/usr/bin/env python3
"""Measure whether Kenet uinput pulses change pr0p/Betaflight MSP_RC."""

from __future__ import annotations

import argparse
import json
import socket
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from statistics import median
from typing import Any

PROBE_DIR = Path(__file__).resolve().parent
if str(PROBE_DIR) not in sys.path:
    sys.path.insert(0, str(PROBE_DIR))
SANDBOX_DIR = PROBE_DIR.parent / "game_screen_sandbox"
if str(SANDBOX_DIR) not in sys.path:
    sys.path.insert(0, str(SANDBOX_DIR))

from msp_ws_rc_probe import (  # noqa: E402
    MSP_RC,
    WebSocketProtocolError,
    parse_rc_payload,
    rc_channel_deltas,
    request_msp_over_websocket,
)
from msp_ws_semantic_probe import MSPProtocolError  # noqa: E402
from pr0p_capture_probe import DEFAULT_LOG_DIR  # noqa: E402
from virtual_input import AxisCommand, DryRunInputAdapter, UInputAdapter  # noqa: E402


PASS = "PASS"
WAITING = "WAITING"
FAIL = "FAIL"
AXES = ("yaw", "pitch", "roll", "throttle", "aux1")
EXPECTED_CHANNELS = ("any", "roll", "pitch", "yaw", "throttle", "aux1")
EXPECTED_DIRECTIONS = ("any", "higher", "lower")
MSP_RC_LABELS = {
    0: "roll",
    1: "pitch",
    2: "yaw",
    3: "throttle",
    4: "aux1",
}
CHANNEL_TO_INDEX = {label: index for index, label in MSP_RC_LABELS.items()}


@dataclass
class UinputRcEffectResult:
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


def command_for_axis(axis: str, magnitude: float) -> AxisCommand:
    if axis not in AXES:
        raise ValueError("unknown axis: %s" % axis)
    values = {name: 0.0 for name in AXES}
    values[axis] = magnitude
    return AxisCommand(**values).clamped()


def sample_msp_rc(
    *,
    host: str,
    port: int,
    path: str,
    timeout: float,
    max_frames: int,
    samples: int,
    interval_s: float,
) -> tuple[list[list[int]], list[dict[str, Any]]]:
    if samples <= 0:
        raise ValueError("samples must be positive")
    parsed_samples: list[list[int]] = []
    request_metrics: list[dict[str, Any]] = []
    for index in range(samples):
        payload, metrics = request_msp_over_websocket(
            host=host,
            port=port,
            path=path,
            code=MSP_RC,
            timeout=timeout,
            max_frames=max_frames,
        )
        parsed = parse_rc_payload(payload)
        request_metrics.append({"index": index, "rc": parsed, **metrics})
        if parsed is not None:
            parsed_samples.append(parsed)
        if index + 1 < samples and interval_s > 0:
            time.sleep(interval_s)
    return parsed_samples, request_metrics


def channel_medians(samples: list[list[int]]) -> list[float]:
    if not samples:
        return []
    count = min(len(sample) for sample in samples)
    return [
        float(median(sample[index] for sample in samples))
        for index in range(count)
    ]


def changed_channels(
    baseline_medians: list[float],
    during_medians: list[float],
    *,
    min_delta: int,
) -> list[dict[str, Any]]:
    count = min(len(baseline_medians), len(during_medians))
    changes = []
    for index in range(count):
        delta = during_medians[index] - baseline_medians[index]
        if abs(delta) >= min_delta:
            changes.append({
                "index": index,
                "label": MSP_RC_LABELS.get(index, "ch%d" % (index + 1)),
                "baseline": baseline_medians[index],
                "during": during_medians[index],
                "delta": delta,
            })
    return changes


def direction_matches(delta: float, expected_direction: str, *, min_delta: int) -> bool:
    if expected_direction == "any":
        return abs(delta) >= min_delta
    if expected_direction == "higher":
        return delta >= min_delta
    if expected_direction == "lower":
        return delta <= -min_delta
    raise ValueError("unknown expected direction: %s" % expected_direction)


def evaluate_rc_effect(
    *,
    baseline_samples: list[list[int]],
    during_samples: list[list[int]],
    after_samples: list[list[int]],
    expected_channel: str,
    expected_direction: str,
    min_delta: int,
    max_baseline_delta: int,
    throttle_low_threshold: int,
) -> tuple[str, str, dict[str, Any], list[str]]:
    notes: list[str] = []
    if not baseline_samples:
        return WAITING, "MSP_RC baseline could not be read yet", {}, ["MSP_RC_BASELINE_MISSING"]
    if not during_samples:
        return WAITING, "MSP_RC could not be read while uinput pulse was active", {}, ["MSP_RC_DURING_MISSING"]

    baseline_deltas = rc_channel_deltas(baseline_samples)
    if baseline_deltas and max(baseline_deltas) > max_baseline_delta:
        return (
            WAITING,
            "MSP_RC baseline is moving before the uinput pulse",
            {"baseline_deltas": baseline_deltas},
            ["RC_INPUT_CONTENTION"],
        )

    baseline_medians = channel_medians(baseline_samples)
    during_medians = channel_medians(during_samples)
    after_medians = channel_medians(after_samples)
    changes = changed_channels(
        baseline_medians,
        during_medians,
        min_delta=min_delta,
    )
    metrics = {
        "baseline_medians": baseline_medians,
        "during_medians": during_medians,
        "after_medians": after_medians,
        "baseline_deltas": baseline_deltas,
        "changed_channels": changes,
        "min_delta": min_delta,
        "max_baseline_delta": max_baseline_delta,
        "throttle_low_threshold": throttle_low_threshold,
    }

    if not changes:
        return (
            WAITING,
            "uinput pulse did not produce a measurable MSP_RC change",
            metrics,
            ["NO_UINPUT_MSP_RC_EFFECT"],
        )

    if expected_channel == "any":
        notes.append("UINPUT_MSP_RC_EFFECT_DETECTED")
        return PASS, "uinput pulse changed one or more MSP_RC channels", metrics, notes

    expected_index = CHANNEL_TO_INDEX[expected_channel]
    expected_delta = (
        during_medians[expected_index] - baseline_medians[expected_index]
        if expected_index < min(len(baseline_medians), len(during_medians))
        else 0.0
    )
    metrics["expected_channel_index"] = expected_index
    metrics["expected_channel_delta"] = expected_delta
    if abs(expected_delta) < min_delta:
        return (
            WAITING,
            "uinput pulse changed MSP_RC, but not the expected channel",
            metrics,
            ["EXPECTED_CHANNEL_UNCHANGED", "changed channels: %s" % changes],
        )
    if not direction_matches(expected_delta, expected_direction, min_delta=min_delta):
        return (
            FAIL,
            "expected MSP_RC channel moved in the wrong direction",
            metrics,
            ["EXPECTED_CHANNEL_DIRECTION_FAIL"],
        )

    notes.append("EXPECTED_CHANNEL_EFFECT_DETECTED")
    if expected_channel == "throttle":
        throttle_values = [
            sample[expected_index]
            for sample in during_samples
            if len(sample) > expected_index
        ]
        metrics["during_throttle_min"] = min(throttle_values) if throttle_values else None
        if throttle_values and min(throttle_values) <= throttle_low_threshold:
            notes.append("THROTTLE_LOW_REACHED")
        else:
            notes.append("THROTTLE_STILL_NOT_LOW")
    return PASS, "uinput pulse moved the expected MSP_RC %s channel" % expected_channel, metrics, notes


def measure_uinput_rc_effect(
    adapter,
    *,
    command: AxisCommand,
    host: str,
    port: int,
    path: str,
    timeout: float,
    max_frames: int,
    baseline_samples: int,
    during_samples: int,
    after_samples: int,
    interval_s: float,
    settle_s: float,
    expected_channel: str,
    expected_direction: str,
    min_delta: int,
    max_baseline_delta: int,
    throttle_low_threshold: int,
) -> UinputRcEffectResult:
    try:
        baseline, baseline_metrics = sample_msp_rc(
            host=host,
            port=port,
            path=path,
            timeout=timeout,
            max_frames=max_frames,
            samples=baseline_samples,
            interval_s=interval_s,
        )
        adapter.send(command)
        if settle_s > 0:
            time.sleep(settle_s)
        during, during_metrics = sample_msp_rc(
            host=host,
            port=port,
            path=path,
            timeout=timeout,
            max_frames=max_frames,
            samples=during_samples,
            interval_s=interval_s,
        )
    finally:
        adapter.neutral()
    after, after_metrics = sample_msp_rc(
        host=host,
        port=port,
        path=path,
        timeout=timeout,
        max_frames=max_frames,
        samples=after_samples,
        interval_s=interval_s,
    )
    status, summary, effect_metrics, notes = evaluate_rc_effect(
        baseline_samples=baseline,
        during_samples=during,
        after_samples=after,
        expected_channel=expected_channel,
        expected_direction=expected_direction,
        min_delta=min_delta,
        max_baseline_delta=max_baseline_delta,
        throttle_low_threshold=throttle_low_threshold,
    )
    return UinputRcEffectResult(
        status=status,
        summary=summary,
        metrics={
            "host": host,
            "port": port,
            "path": path,
            "command": command.as_dict(),
            "expected_channel": expected_channel,
            "expected_direction": expected_direction,
            "baseline_samples": baseline,
            "during_samples": during,
            "after_samples": after,
            "baseline_request_metrics": baseline_metrics,
            "during_request_metrics": during_metrics,
            "after_request_metrics": after_metrics,
            **effect_metrics,
        },
        notes=notes,
    )


def run_uinput_rc_effect_probe(
    *,
    run_id: str,
    host: str,
    port: int,
    path: str,
    timeout: float,
    max_frames: int,
    axis: str,
    magnitude: float,
    baseline_samples: int,
    during_samples: int,
    after_samples: int,
    interval_s: float,
    settle_s: float,
    expected_channel: str,
    expected_direction: str,
    min_delta: int,
    max_baseline_delta: int,
    throttle_low_threshold: int,
    use_uinput: bool,
    device_name: str,
) -> UinputRcEffectResult:
    command = command_for_axis(axis, magnitude)
    adapter = UInputAdapter(name=device_name) if use_uinput else DryRunInputAdapter()
    try:
        result = measure_uinput_rc_effect(
            adapter,
            command=command,
            host=host,
            port=port,
            path=path,
            timeout=timeout,
            max_frames=max_frames,
            baseline_samples=baseline_samples,
            during_samples=during_samples,
            after_samples=after_samples,
            interval_s=interval_s,
            settle_s=settle_s,
            expected_channel=expected_channel,
            expected_direction=expected_direction,
            min_delta=min_delta,
            max_baseline_delta=max_baseline_delta,
            throttle_low_threshold=throttle_low_threshold,
        )
    except (ConnectionRefusedError, TimeoutError, socket.timeout, WebSocketProtocolError, MSPProtocolError, OSError) as exc:
        return UinputRcEffectResult(
            status=WAITING,
            summary="MSP_RC endpoint is not reachable for uinput effect probing",
            metrics={
                "run_id": run_id,
                "host": host,
                "port": port,
                "path": path,
                "command": command.as_dict(),
                "real_input_sent": False,
                "error": str(exc),
            },
            notes=["start pr0p local race and rerun the uinput RC effect probe"],
        )
    finally:
        adapter.close()
    result.metrics["run_id"] = run_id
    result.metrics["real_input_sent"] = use_uinput
    result.metrics["adapter"] = adapter.__class__.__name__
    result.metrics["device_name"] = device_name
    if not use_uinput:
        result.status = WAITING
        result.summary = "uinput RC effect metric is informational because no real input was sent"
        result.notes.append("DRY_RUN_ONLY")
    return result


def build_markdown(result: UinputRcEffectResult, *, run_id: str) -> str:
    lines = [
        "# SimITL / pr0p UInput -> MSP_RC Effect Probe",
        "",
        "Run: `%s`" % run_id,
        "Verdict: `%s`" % result.status,
        "",
        result.summary,
        "",
        "| Metric | Value |",
        "| --- | --- |",
    ]
    for key in (
        "real_input_sent",
        "adapter",
        "expected_channel",
        "expected_direction",
        "expected_channel_delta",
        "during_throttle_min",
    ):
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


def write_reports(result: UinputRcEffectResult, log_dir: Path, *, run_id: str) -> tuple[Path, Path]:
    log_dir.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime("%Y%m%d-%H%M%S", time.localtime())
    json_path = log_dir / ("%s-%s-uinput-rc-effect.json" % (stamp, run_id))
    md_path = log_dir / ("%s-%s-uinput-rc-effect.md" % (stamp, run_id))
    json_path.write_text(
        json.dumps(result.as_dict(), indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    md_path.write_text(build_markdown(result, run_id=run_id), encoding="utf-8")
    return json_path, md_path


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-id", default="pr0p-uinput-rc-effect")
    parser.add_argument("--log-dir", type=Path, default=DEFAULT_LOG_DIR)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=5761)
    parser.add_argument("--path", default="/")
    parser.add_argument("--timeout", type=float, default=1.0)
    parser.add_argument("--max-frames", type=int, default=8)
    parser.add_argument("--axis", choices=AXES, default="throttle")
    parser.add_argument("--magnitude", type=float, default=1.0)
    parser.add_argument("--baseline-samples", type=int, default=3)
    parser.add_argument("--during-samples", type=int, default=3)
    parser.add_argument("--after-samples", type=int, default=2)
    parser.add_argument("--interval", type=float, default=0.1)
    parser.add_argument("--settle", type=float, default=0.2)
    parser.add_argument("--expected-channel", choices=EXPECTED_CHANNELS, default="throttle")
    parser.add_argument("--expected-direction", choices=EXPECTED_DIRECTIONS, default="lower")
    parser.add_argument("--min-delta", type=int, default=25)
    parser.add_argument("--max-baseline-delta", type=int, default=4)
    parser.add_argument("--throttle-low-threshold", type=int, default=1050)
    parser.add_argument("--device-name", default="Kenet Game Sandbox")
    parser.add_argument("--uinput", action="store_true",
                        help="Send a real uinput pulse")
    parser.add_argument("--ack-live-input", action="store_true",
                        help="Required before --uinput sends real OS input")
    args = parser.parse_args(argv)
    if args.port <= 0:
        parser.error("--port must be positive")
    if args.timeout <= 0:
        parser.error("--timeout must be positive")
    if args.max_frames <= 0:
        parser.error("--max-frames must be positive")
    if args.baseline_samples <= 0 or args.during_samples <= 0 or args.after_samples <= 0:
        parser.error("sample counts must be positive")
    if args.interval < 0 or args.settle < 0:
        parser.error("--interval/--settle must be non-negative")
    if args.min_delta <= 0 or args.max_baseline_delta < 0:
        parser.error("--min-delta must be positive and --max-baseline-delta non-negative")
    if args.throttle_low_threshold <= 0:
        parser.error("--throttle-low-threshold must be positive")
    if args.uinput and not args.ack_live_input:
        parser.error("--uinput requires --ack-live-input")
    return args


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    result = run_uinput_rc_effect_probe(
        run_id=args.run_id,
        host=args.host,
        port=args.port,
        path=args.path,
        timeout=args.timeout,
        max_frames=args.max_frames,
        axis=args.axis,
        magnitude=args.magnitude,
        baseline_samples=args.baseline_samples,
        during_samples=args.during_samples,
        after_samples=args.after_samples,
        interval_s=args.interval,
        settle_s=args.settle,
        expected_channel=args.expected_channel,
        expected_direction=args.expected_direction,
        min_delta=args.min_delta,
        max_baseline_delta=args.max_baseline_delta,
        throttle_low_threshold=args.throttle_low_threshold,
        use_uinput=args.uinput,
        device_name=args.device_name,
    )
    json_path, md_path = write_reports(result, args.log_dir, run_id=args.run_id)
    print("simitl-pr0p-uinput-rc-effect %s report=%s summary=%s real_input_sent=%s" % (
        result.status,
        json_path,
        md_path,
        result.metrics.get("real_input_sent"),
    ))
    print(result.summary)
    return 1 if result.status == FAIL else 0


if __name__ == "__main__":
    raise SystemExit(main())
