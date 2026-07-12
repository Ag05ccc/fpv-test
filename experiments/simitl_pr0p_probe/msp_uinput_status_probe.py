#!/usr/bin/env python3
"""Measure FC arming/status flags while a Kenet uinput command is held."""

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

from msp_uinput_rc_effect_probe import AXES, command_for_axis  # noqa: E402
from msp_ws_status_probe import run_msp_ws_status_probe  # noqa: E402
from pr0p_capture_probe import DEFAULT_LOG_DIR  # noqa: E402
from virtual_input import DryRunInputAdapter, UInputAdapter  # noqa: E402


PASS = "PASS"
WAITING = "WAITING"
FAIL = "FAIL"


@dataclass
class UinputStatusResult:
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


def sample_status(
    *,
    host: str,
    port: int,
    path: str,
    timeout: float,
    max_frames: int,
    samples: int,
    interval_s: float,
) -> dict[str, Any]:
    return run_msp_ws_status_probe(
        host=host,
        port=port,
        path=path,
        timeout=timeout,
        max_frames=max_frames,
        samples=samples,
        interval_s=interval_s,
    ).as_dict()


def blockers_by_sample(status_result: dict[str, Any]) -> list[list[str]]:
    metrics = status_result.get("metrics", {})
    raw_samples = metrics.get("samples")
    if not isinstance(raw_samples, list):
        fc_status = metrics.get("fc_status")
        if isinstance(fc_status, dict):
            return [list(fc_status.get("arming_disable_names") or [])]
        return []
    blockers: list[list[str]] = []
    for sample in raw_samples:
        if not isinstance(sample, dict):
            continue
        fc_status = sample.get("metrics", {}).get("fc_status", {})
        if isinstance(fc_status, dict):
            blockers.append(list(fc_status.get("arming_disable_names") or []))
    return blockers


def blocker_clear(blocker_samples: list[list[str]], blocker: str, *, require_all_clear: bool) -> bool:
    if not blocker_samples:
        return False
    present = [blocker in sample for sample in blocker_samples]
    return not any(present) if require_all_clear else not present[-1]


def evaluate_status_effect(
    *,
    baseline_status: dict[str, Any],
    during_status: dict[str, Any],
    after_status: dict[str, Any],
    blocker: str,
    require_all_clear: bool,
) -> tuple[str, str, dict[str, Any], list[str]]:
    notes: list[str] = []
    baseline_blockers = blockers_by_sample(baseline_status)
    during_blockers = blockers_by_sample(during_status)
    after_blockers = blockers_by_sample(after_status)
    metrics = {
        "blocker": blocker,
        "require_all_clear": require_all_clear,
        "baseline_blockers_by_sample": baseline_blockers,
        "during_blockers_by_sample": during_blockers,
        "after_blockers_by_sample": after_blockers,
    }
    if during_status.get("status") != PASS:
        return (
            WAITING,
            "FC status could not be read while uinput command was active",
            metrics,
            ["UINPUT_STATUS_DURING_MISSING"],
        )
    if blocker_clear(during_blockers, blocker, require_all_clear=require_all_clear):
        notes.append("UINPUT_CLEARED_%s" % blocker)
        return PASS, "uinput command cleared %s arming blocker" % blocker, metrics, notes
    notes.append("UINPUT_DID_NOT_CLEAR_%s" % blocker)
    return (
        WAITING,
        "uinput command did not clear %s arming blocker" % blocker,
        metrics,
        notes,
    )


def measure_uinput_status_effect(
    adapter,
    *,
    command,
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
    blocker: str,
    require_all_clear: bool,
) -> UinputStatusResult:
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
    status, summary, effect_metrics, notes = evaluate_status_effect(
        baseline_status=baseline,
        during_status=during,
        after_status=after,
        blocker=blocker,
        require_all_clear=require_all_clear,
    )
    return UinputStatusResult(
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


def run_uinput_status_probe(
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
    blocker: str,
    require_all_clear: bool,
    use_uinput: bool,
    device_name: str,
) -> UinputStatusResult:
    command = command_for_axis(axis, magnitude)
    adapter = UInputAdapter(name=device_name) if use_uinput else DryRunInputAdapter()
    try:
        result = measure_uinput_status_effect(
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
            blocker=blocker,
            require_all_clear=require_all_clear,
        )
    except Exception as exc:
        return UinputStatusResult(
            status=WAITING,
            summary="FC status endpoint is not reachable for uinput status probing",
            metrics={
                "run_id": run_id,
                "host": host,
                "port": port,
                "path": path,
                "command": command.as_dict(),
                "real_input_sent": False,
                "error": str(exc),
            },
            notes=["start pr0p local race and rerun the uinput status probe"],
        )
    finally:
        adapter.close()
    result.metrics["run_id"] = run_id
    result.metrics["real_input_sent"] = use_uinput
    result.metrics["adapter"] = adapter.__class__.__name__
    result.metrics["device_name"] = device_name
    if not use_uinput:
        result.status = WAITING
        result.summary = "uinput status metric is informational because no real input was sent"
        result.notes.append("DRY_RUN_ONLY")
    return result


def build_markdown(result: UinputStatusResult, *, run_id: str) -> str:
    lines = [
        "# SimITL / pr0p UInput -> FC Status Probe",
        "",
        "Run: `%s`" % run_id,
        "Verdict: `%s`" % result.status,
        "",
        result.summary,
        "",
        "| Metric | Value |",
        "| --- | --- |",
    ]
    for key in ("real_input_sent", "adapter", "blocker", "require_all_clear"):
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


def write_reports(result: UinputStatusResult, log_dir: Path, *, run_id: str) -> tuple[Path, Path]:
    log_dir.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime("%Y%m%d-%H%M%S", time.localtime())
    json_path = log_dir / ("%s-%s-uinput-status.json" % (stamp, run_id))
    md_path = log_dir / ("%s-%s-uinput-status.md" % (stamp, run_id))
    json_path.write_text(json.dumps(result.as_dict(), indent=2, sort_keys=True) + "\n",
                         encoding="utf-8")
    md_path.write_text(build_markdown(result, run_id=run_id), encoding="utf-8")
    return json_path, md_path


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-id", default="pr0p-uinput-status")
    parser.add_argument("--log-dir", type=Path, default=DEFAULT_LOG_DIR)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=5761)
    parser.add_argument("--path", default="/")
    parser.add_argument("--timeout", type=float, default=1.0)
    parser.add_argument("--max-frames", type=int, default=8)
    parser.add_argument("--axis", choices=AXES, default="throttle")
    parser.add_argument("--magnitude", type=float, default=1.0)
    parser.add_argument("--baseline-samples", type=int, default=2)
    parser.add_argument("--during-samples", type=int, default=4)
    parser.add_argument("--after-samples", type=int, default=2)
    parser.add_argument("--interval", type=float, default=0.1)
    parser.add_argument("--settle", type=float, default=0.2)
    parser.add_argument("--blocker", default="THROTTLE")
    parser.add_argument("--allow-last-sample-clear", action="store_true",
                        help="Pass if the last during sample clears the blocker")
    parser.add_argument("--device-name", default="Kenet Game Sandbox")
    parser.add_argument("--uinput", action="store_true",
                        help="Send a real uinput command")
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
    result = run_uinput_status_probe(
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
        blocker=args.blocker,
        require_all_clear=not args.allow_last_sample_clear,
        use_uinput=args.uinput,
        device_name=args.device_name,
    )
    json_path, md_path = write_reports(result, args.log_dir, run_id=args.run_id)
    print("simitl-pr0p-uinput-status %s report=%s summary=%s real_input_sent=%s" % (
        result.status,
        json_path,
        md_path,
        result.metrics.get("real_input_sent"),
    ))
    print(result.summary)
    return 1 if result.status == FAIL else 0


if __name__ == "__main__":
    raise SystemExit(main())
