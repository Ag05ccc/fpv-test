#!/usr/bin/env python3
"""Measure FC ARM status while throttle-low and AUX1-high are held by uinput."""

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

from msp_uinput_arm_probe import active_modes_by_sample, arm_active_by_sample  # noqa: E402
from msp_uinput_status_probe import blockers_by_sample, sample_status  # noqa: E402
from pr0p_capture_probe import DEFAULT_LOG_DIR  # noqa: E402
from virtual_input import AxisCommand, DryRunInputAdapter, UInputAdapter  # noqa: E402


PASS = "PASS"
WAITING = "WAITING"
FAIL = "FAIL"


@dataclass
class UinputAuxArmStatusResult:
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


def combined_arm_command(*, throttle_magnitude: float, aux1_magnitude: float) -> AxisCommand:
    return AxisCommand(throttle=throttle_magnitude, aux1=aux1_magnitude).clamped()


def _all_or_any(values: list[bool], *, require_all: bool) -> bool:
    if not values:
        return False
    return all(values) if require_all else any(values)


def evaluate_aux_arm_status(
    *,
    baseline_status: dict[str, Any],
    during_status: dict[str, Any],
    after_status: dict[str, Any],
    require_all_armed: bool,
) -> tuple[str, str, dict[str, Any], list[str]]:
    baseline_arm = arm_active_by_sample(baseline_status)
    during_arm = arm_active_by_sample(during_status)
    after_arm = arm_active_by_sample(after_status)
    baseline_blockers = blockers_by_sample(baseline_status)
    during_blockers = blockers_by_sample(during_status)
    after_blockers = blockers_by_sample(after_status)
    metrics = {
        "require_all_armed": require_all_armed,
        "baseline_arm_active_by_sample": baseline_arm,
        "during_arm_active_by_sample": during_arm,
        "after_arm_active_by_sample": after_arm,
        "baseline_active_modes_by_sample": active_modes_by_sample(baseline_status),
        "during_active_modes_by_sample": active_modes_by_sample(during_status),
        "after_active_modes_by_sample": active_modes_by_sample(after_status),
        "baseline_blockers_by_sample": baseline_blockers,
        "during_blockers_by_sample": during_blockers,
        "after_blockers_by_sample": after_blockers,
    }
    if during_status.get("status") != PASS:
        return (
            WAITING,
            "FC status could not be read while throttle-low and AUX1-high were active",
            metrics,
            ["UINPUT_AUX_ARM_STATUS_MISSING"],
        )

    notes: list[str] = []
    armed = _all_or_any(during_arm, require_all=require_all_armed)
    blockers_seen = sorted({
        str(blocker)
        for sample in during_blockers
        for blocker in sample
    })
    throttle_clear = bool(during_blockers) and not any("THROTTLE" in sample for sample in during_blockers)
    if armed:
        notes.extend(["UINPUT_AUX1_ARMED", "FC_ARMED"])
        notes.append("UINPUT_CLEARED_THROTTLE" if throttle_clear else "THROTTLE_STILL_BLOCKING")
        if any(after_arm):
            notes.append("FC_ARMED_LATCHED_AFTER_RELEASE")
        else:
            notes.append("ARM_MODE_ONLY_WHILE_COMMAND_HELD")
        return (
            PASS,
            "throttle-low plus AUX1-high activated ARM mode/status",
            metrics,
            notes,
        )

    notes.append("UINPUT_AUX1_DID_NOT_ARM")
    if throttle_clear:
        notes.append("THROTTLE_CLEAR_BUT_ARM_INACTIVE")
    if blockers_seen:
        notes.append("ARM_BLOCKERS_DURING_AUX:%s" % ",".join(blockers_seen))
    return (
        WAITING,
        "throttle-low plus AUX1-high did not activate ARM mode/status",
        metrics,
        notes,
    )


def measure_uinput_aux_arm_status(
    adapter,
    *,
    throttle_magnitude: float,
    aux1_magnitude: float,
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
    require_all_armed: bool,
) -> UinputAuxArmStatusResult:
    command = combined_arm_command(
        throttle_magnitude=throttle_magnitude,
        aux1_magnitude=aux1_magnitude,
    )
    baseline = sample_status(
        host=host,
        port=port,
        path=path,
        timeout=timeout,
        max_frames=max_frames,
        samples=baseline_samples,
        interval_s=interval_s,
    )
    try:
        adapter.send(command)
        if settle_s > 0:
            time.sleep(settle_s)
        during = sample_status(
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
    after = sample_status(
        host=host,
        port=port,
        path=path,
        timeout=timeout,
        max_frames=max_frames,
        samples=after_samples,
        interval_s=interval_s,
    )
    status, summary, effect_metrics, notes = evaluate_aux_arm_status(
        baseline_status=baseline,
        during_status=during,
        after_status=after,
        require_all_armed=require_all_armed,
    )
    return UinputAuxArmStatusResult(
        status=status,
        summary=summary,
        metrics={
            "host": host,
            "port": port,
            "path": path,
            "command": command.as_dict(),
            "baseline_status": baseline,
            "during_status": during,
            "after_status": after,
            **effect_metrics,
        },
        notes=notes,
    )


def run_uinput_aux_arm_status_probe(
    *,
    run_id: str,
    host: str,
    port: int,
    path: str,
    timeout: float,
    max_frames: int,
    throttle_magnitude: float,
    aux1_magnitude: float,
    baseline_samples: int,
    during_samples: int,
    after_samples: int,
    interval_s: float,
    settle_s: float,
    require_all_armed: bool,
    use_uinput: bool,
    device_name: str,
) -> UinputAuxArmStatusResult:
    adapter = UInputAdapter(name=device_name) if use_uinput else DryRunInputAdapter()
    try:
        result = measure_uinput_aux_arm_status(
            adapter,
            throttle_magnitude=throttle_magnitude,
            aux1_magnitude=aux1_magnitude,
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
            require_all_armed=require_all_armed,
        )
    except Exception as exc:
        return UinputAuxArmStatusResult(
            status=WAITING,
            summary="FC status endpoint is not reachable for AUX1 ARM status probing",
            metrics={
                "run_id": run_id,
                "host": host,
                "port": port,
                "path": path,
                "command": combined_arm_command(
                    throttle_magnitude=throttle_magnitude,
                    aux1_magnitude=aux1_magnitude,
                ).as_dict(),
                "real_input_sent": False,
                "error": str(exc),
            },
            notes=["start pr0p local race and rerun the AUX1 ARM status probe"],
        )
    finally:
        adapter.close()
    result.metrics["run_id"] = run_id
    result.metrics["real_input_sent"] = use_uinput
    result.metrics["adapter"] = adapter.__class__.__name__
    result.metrics["device_name"] = device_name
    if not use_uinput:
        result.status = WAITING
        result.summary = "uinput AUX1 ARM status metric is informational because no real input was sent"
        result.notes.append("DRY_RUN_ONLY")
    return result


def build_markdown(result: UinputAuxArmStatusResult, *, run_id: str) -> str:
    lines = [
        "# SimITL / pr0p UInput -> AUX1 ARM Status Probe",
        "",
        "Run: `%s`" % run_id,
        "Verdict: `%s`" % result.status,
        "",
        result.summary,
        "",
        "| Metric | Value |",
        "| --- | --- |",
    ]
    for key in ("real_input_sent", "adapter", "require_all_armed"):
        if key in result.metrics:
            lines.append("| %s | `%s` |" % (key, result.metrics[key]))
    command = result.metrics.get("command")
    if isinstance(command, dict):
        lines.append("| command.throttle | `%s` |" % command.get("throttle"))
        lines.append("| command.aux1 | `%s` |" % command.get("aux1"))
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


def write_reports(
    result: UinputAuxArmStatusResult,
    log_dir: Path,
    *,
    run_id: str,
) -> tuple[Path, Path]:
    log_dir.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime("%Y%m%d-%H%M%S", time.localtime())
    json_path = log_dir / ("%s-%s-uinput-aux-arm-status.json" % (stamp, run_id))
    md_path = log_dir / ("%s-%s-uinput-aux-arm-status.md" % (stamp, run_id))
    json_path.write_text(json.dumps(result.as_dict(), indent=2, sort_keys=True) + "\n",
                         encoding="utf-8")
    md_path.write_text(build_markdown(result, run_id=run_id), encoding="utf-8")
    return json_path, md_path


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-id", default="pr0p-uinput-aux-arm-status")
    parser.add_argument("--log-dir", type=Path, default=DEFAULT_LOG_DIR)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=5761)
    parser.add_argument("--path", default="/")
    parser.add_argument("--timeout", type=float, default=1.0)
    parser.add_argument("--max-frames", type=int, default=8)
    parser.add_argument("--throttle-magnitude", type=float, default=1.0)
    parser.add_argument("--aux1-magnitude", type=float, default=1.0)
    parser.add_argument("--baseline-samples", type=int, default=2)
    parser.add_argument("--during-samples", type=int, default=4)
    parser.add_argument("--after-samples", type=int, default=2)
    parser.add_argument("--interval", type=float, default=0.1)
    parser.add_argument("--settle", type=float, default=0.2)
    parser.add_argument("--require-all-armed", action="store_true")
    parser.add_argument("--device-name", default="Kenet Game Sandbox")
    parser.add_argument("--uinput", action="store_true",
                        help="Send real uinput axis events")
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
    if args.uinput and not args.ack_live_input:
        parser.error("--uinput requires --ack-live-input")
    return args


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    result = run_uinput_aux_arm_status_probe(
        run_id=args.run_id,
        host=args.host,
        port=args.port,
        path=args.path,
        timeout=args.timeout,
        max_frames=args.max_frames,
        throttle_magnitude=args.throttle_magnitude,
        aux1_magnitude=args.aux1_magnitude,
        baseline_samples=args.baseline_samples,
        during_samples=args.during_samples,
        after_samples=args.after_samples,
        interval_s=args.interval,
        settle_s=args.settle,
        require_all_armed=args.require_all_armed,
        use_uinput=args.uinput,
        device_name=args.device_name,
    )
    json_path, md_path = write_reports(result, args.log_dir, run_id=args.run_id)
    print("simitl-pr0p-uinput-aux-arm-status %s report=%s summary=%s real_input_sent=%s" % (
        result.status,
        json_path,
        md_path,
        result.metrics.get("real_input_sent"),
    ))
    print(result.summary)
    return 1 if result.status == FAIL else 0


if __name__ == "__main__":
    raise SystemExit(main())
