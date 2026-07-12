#!/usr/bin/env python3
"""Discover whether a uinput button activates pr0p/Betaflight ARM mode."""

from __future__ import annotations

import argparse
import json
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

PROBE_DIR = Path(__file__).resolve().parent
if str(PROBE_DIR) not in sys.path:
    sys.path.insert(0, str(PROBE_DIR))
SANDBOX_DIR = PROBE_DIR.parent / "game_screen_sandbox"
if str(SANDBOX_DIR) not in sys.path:
    sys.path.insert(0, str(SANDBOX_DIR))

from msp_uinput_rc_effect_probe import command_for_axis  # noqa: E402
from msp_uinput_status_probe import blockers_by_sample, sample_status  # noqa: E402
from pr0p_capture_probe import DEFAULT_LOG_DIR  # noqa: E402
from virtual_input import DryRunInputAdapter, UInputAdapter, normalize_button_name  # noqa: E402


PASS = "PASS"
WAITING = "WAITING"
FAIL = "FAIL"
BUTTONS = ("south", "east")


@dataclass
class UinputArmResult:
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


def fc_statuses_by_sample(status_result: dict[str, Any]) -> list[dict[str, Any]]:
    metrics = status_result.get("metrics", {}) if isinstance(status_result, dict) else {}
    raw_samples = metrics.get("samples")
    if not isinstance(raw_samples, list):
        fc_status = metrics.get("fc_status")
        return [fc_status] if isinstance(fc_status, dict) else []
    statuses: list[dict[str, Any]] = []
    for sample in raw_samples:
        if not isinstance(sample, dict):
            continue
        fc_status = sample.get("metrics", {}).get("fc_status")
        if isinstance(fc_status, dict):
            statuses.append(fc_status)
    return statuses


def arm_active_by_sample(status_result: dict[str, Any]) -> list[bool]:
    values: list[bool] = []
    for fc_status in fc_statuses_by_sample(status_result):
        active_modes = fc_status.get("active_modes") or []
        values.append(bool(fc_status.get("armed") is True or "ARM" in active_modes))
    return values


def active_modes_by_sample(status_result: dict[str, Any]) -> list[list[str]]:
    return [
        [str(mode) for mode in (fc_status.get("active_modes") or [])]
        for fc_status in fc_statuses_by_sample(status_result)
    ]


def evaluate_arm_effect(
    *,
    baseline_status: dict[str, Any],
    candidate_results: list[dict[str, Any]],
    after_status: dict[str, Any],
) -> tuple[str, str, dict[str, Any], list[str]]:
    metrics: dict[str, Any] = {
        "baseline_arm_active_by_sample": arm_active_by_sample(baseline_status),
        "baseline_blockers_by_sample": blockers_by_sample(baseline_status),
        "after_arm_active_by_sample": arm_active_by_sample(after_status),
        "after_blockers_by_sample": blockers_by_sample(after_status),
        "candidates": candidate_results,
    }
    notes: list[str] = []

    if not candidate_results:
        return WAITING, "no ARM button candidates were tested", metrics, ["NO_ARM_BUTTON_CANDIDATES"]

    for candidate in candidate_results:
        if candidate.get("status") != PASS:
            continue
        button = str(candidate.get("button"))
        if any(candidate.get("arm_active_by_sample") or []):
            notes.append("UINPUT_ARM_BUTTON_DETECTED:%s" % button)
            notes.append("FC_ARMED")
            if any(metrics["after_arm_active_by_sample"]):
                notes.append("FC_ARMED_LATCHED_AFTER_RELEASE")
            else:
                notes.append("ARM_MODE_ONLY_WHILE_BUTTON_HELD")
            return (
                PASS,
                "uinput button %s activated ARM mode/status" % button,
                metrics,
                notes,
            )

    unreachable = [
        candidate
        for candidate in candidate_results
        if candidate.get("status") != PASS
    ]
    if len(unreachable) == len(candidate_results):
        return (
            WAITING,
            "FC status could not be read while testing ARM button candidates",
            metrics,
            ["UINPUT_ARM_STATUS_MISSING"],
        )

    notes.append("NO_UINPUT_ARM_BUTTON_EFFECT")
    blockers_seen = sorted({
        blocker
        for candidate in candidate_results
        for sample in candidate.get("blockers_by_sample", [])
        for blocker in sample
    })
    if blockers_seen:
        notes.append("ARM_BLOCKERS_DURING_BUTTON:%s" % ",".join(blockers_seen))
    return (
        WAITING,
        "tested uinput buttons did not activate ARM mode/status",
        metrics,
        notes,
    )


def measure_uinput_arm_effect(
    adapter,
    *,
    buttons: list[str],
    throttle_magnitude: float,
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
    button_hold_s: float,
) -> UinputArmResult:
    normalized_buttons = [normalize_button_name(button) for button in buttons]
    throttle_command = command_for_axis("throttle", throttle_magnitude)
    baseline = sample_status(
        host=host,
        port=port,
        path=path,
        timeout=timeout,
        max_frames=max_frames,
        samples=baseline_samples,
        interval_s=interval_s,
    )

    candidate_results: list[dict[str, Any]] = []
    try:
        for button in normalized_buttons:
            adapter.neutral()
            adapter.send(throttle_command)
            if settle_s > 0:
                time.sleep(settle_s)
            adapter.press_button(button, True)
            if button_hold_s > 0:
                time.sleep(button_hold_s)
            during = sample_status(
                host=host,
                port=port,
                path=path,
                timeout=timeout,
                max_frames=max_frames,
                samples=during_samples,
                interval_s=interval_s,
            )
            adapter.press_button(button, False)
            candidate_results.append({
                "button": button,
                "status": during.get("status"),
                "summary": during.get("summary"),
                "arm_active_by_sample": arm_active_by_sample(during),
                "active_modes_by_sample": active_modes_by_sample(during),
                "blockers_by_sample": blockers_by_sample(during),
                "status_result": during,
            })
    finally:
        adapter.neutral()

    after = sample_status(
        host=host,
        port=port,
        path=path,
        timeout=timeout,
        max_frames=max_frames,
        samples=after_samples,
        interval_s=interval_s,
    )
    status, summary, effect_metrics, notes = evaluate_arm_effect(
        baseline_status=baseline,
        candidate_results=candidate_results,
        after_status=after,
    )
    return UinputArmResult(
        status=status,
        summary=summary,
        metrics={
            "host": host,
            "port": port,
            "path": path,
            "buttons": normalized_buttons,
            "throttle_command": throttle_command.as_dict(),
            "baseline_status": baseline,
            "after_status": after,
            **effect_metrics,
        },
        notes=notes,
    )


def run_uinput_arm_probe(
    *,
    run_id: str,
    host: str,
    port: int,
    path: str,
    timeout: float,
    max_frames: int,
    buttons: list[str],
    throttle_magnitude: float,
    baseline_samples: int,
    during_samples: int,
    after_samples: int,
    interval_s: float,
    settle_s: float,
    button_hold_s: float,
    use_uinput: bool,
    device_name: str,
) -> UinputArmResult:
    adapter = UInputAdapter(name=device_name) if use_uinput else DryRunInputAdapter()
    try:
        result = measure_uinput_arm_effect(
            adapter,
            buttons=buttons,
            throttle_magnitude=throttle_magnitude,
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
            button_hold_s=button_hold_s,
        )
    except Exception as exc:
        return UinputArmResult(
            status=WAITING,
            summary="FC status endpoint is not reachable for ARM button probing",
            metrics={
                "run_id": run_id,
                "host": host,
                "port": port,
                "path": path,
                "buttons": buttons,
                "real_input_sent": False,
                "error": str(exc),
            },
            notes=["start pr0p local race and rerun the ARM button probe"],
        )
    finally:
        adapter.close()
    result.metrics["run_id"] = run_id
    result.metrics["real_input_sent"] = use_uinput
    result.metrics["adapter"] = adapter.__class__.__name__
    result.metrics["device_name"] = device_name
    if not use_uinput:
        result.status = WAITING
        result.summary = "uinput ARM button metric is informational because no real input was sent"
        result.notes.append("DRY_RUN_ONLY")
    return result


def build_markdown(result: UinputArmResult, *, run_id: str) -> str:
    lines = [
        "# SimITL / pr0p UInput -> ARM Button Probe",
        "",
        "Run: `%s`" % run_id,
        "Verdict: `%s`" % result.status,
        "",
        result.summary,
        "",
        "| Metric | Value |",
        "| --- | --- |",
    ]
    for key in ("real_input_sent", "adapter", "buttons"):
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


def write_reports(result: UinputArmResult, log_dir: Path, *, run_id: str) -> tuple[Path, Path]:
    log_dir.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime("%Y%m%d-%H%M%S", time.localtime())
    json_path = log_dir / ("%s-%s-uinput-arm.json" % (stamp, run_id))
    md_path = log_dir / ("%s-%s-uinput-arm.md" % (stamp, run_id))
    json_path.write_text(json.dumps(result.as_dict(), indent=2, sort_keys=True) + "\n",
                         encoding="utf-8")
    md_path.write_text(build_markdown(result, run_id=run_id), encoding="utf-8")
    return json_path, md_path


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-id", default="pr0p-uinput-arm")
    parser.add_argument("--log-dir", type=Path, default=DEFAULT_LOG_DIR)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=5761)
    parser.add_argument("--path", default="/")
    parser.add_argument("--timeout", type=float, default=1.0)
    parser.add_argument("--max-frames", type=int, default=8)
    parser.add_argument("--button", action="append", choices=BUTTONS,
                        help="Candidate button to test. Repeat to test multiple buttons.")
    parser.add_argument("--throttle-magnitude", type=float, default=1.0)
    parser.add_argument("--baseline-samples", type=int, default=2)
    parser.add_argument("--during-samples", type=int, default=3)
    parser.add_argument("--after-samples", type=int, default=2)
    parser.add_argument("--interval", type=float, default=0.1)
    parser.add_argument("--settle", type=float, default=0.2)
    parser.add_argument("--button-hold", type=float, default=0.2)
    parser.add_argument("--device-name", default="Kenet Game Sandbox")
    parser.add_argument("--uinput", action="store_true",
                        help="Send real uinput axis/button events")
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
    if args.interval < 0 or args.settle < 0 or args.button_hold < 0:
        parser.error("--interval/--settle/--button-hold must be non-negative")
    if args.uinput and not args.ack_live_input:
        parser.error("--uinput requires --ack-live-input")
    args.button = args.button or list(BUTTONS)
    return args


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    result = run_uinput_arm_probe(
        run_id=args.run_id,
        host=args.host,
        port=args.port,
        path=args.path,
        timeout=args.timeout,
        max_frames=args.max_frames,
        buttons=args.button,
        throttle_magnitude=args.throttle_magnitude,
        baseline_samples=args.baseline_samples,
        during_samples=args.during_samples,
        after_samples=args.after_samples,
        interval_s=args.interval,
        settle_s=args.settle,
        button_hold_s=args.button_hold,
        use_uinput=args.uinput,
        device_name=args.device_name,
    )
    json_path, md_path = write_reports(result, args.log_dir, run_id=args.run_id)
    print("simitl-pr0p-uinput-arm %s report=%s summary=%s real_input_sent=%s" % (
        result.status,
        json_path,
        md_path,
        result.metrics.get("real_input_sent"),
    ))
    print(result.summary)
    return 1 if result.status == FAIL else 0


if __name__ == "__main__":
    raise SystemExit(main())
