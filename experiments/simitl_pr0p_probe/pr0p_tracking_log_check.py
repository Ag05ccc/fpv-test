#!/usr/bin/env python3
"""Verify pr0p tracking/PID JSONL logs independently of the probe summary."""

from __future__ import annotations

import argparse
import json
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

from pr0p_capture_probe import DEFAULT_LOG_DIR


PASS = "PASS"
WAITING = "WAITING"
FAIL = "FAIL"


@dataclass
class TrackingLogCheckResult:
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


def report_sort_key(path: Path) -> tuple[int, str]:
    try:
        mtime_ns = path.stat().st_mtime_ns
    except OSError:
        mtime_ns = -1
    return mtime_ns, path.name


def latest_tracking_log(log_dir: Path) -> Path | None:
    reports = list(log_dir.glob("*-p6-tracking.jsonl"))
    return max(reports, key=report_sort_key) if reports else None


def load_jsonl(path: Path) -> tuple[list[dict[str, Any]], list[str]]:
    records: list[dict[str, Any]] = []
    errors: list[str] = []
    for index, line in enumerate(path.read_text(encoding="utf-8").splitlines(), start=1):
        if not line.strip():
            continue
        try:
            record = json.loads(line)
        except json.JSONDecodeError as exc:
            errors.append("line %d: %s" % (index, exc))
            continue
        if isinstance(record, dict):
            records.append(record)
        else:
            errors.append("line %d: JSON record is not an object" % index)
    return records, errors


def command_abs_max(samples: list[dict[str, Any]], key: str) -> float:
    values = []
    for sample in samples:
        command = sample.get("command")
        if isinstance(command, dict) and key in command:
            try:
                values.append(abs(float(command[key])))
            except (TypeError, ValueError):
                continue
    return max(values) if values else 0.0


def find_summary(records: list[dict[str, Any]]) -> dict[str, Any] | None:
    for record in reversed(records):
        if record.get("event") in (
            "simitl_pr0p_tracking_summary",
            "game_screen_summary",
            "game_screen_phase_s4_summary",
        ):
            return record
    return None


def find_metadata(records: list[dict[str, Any]]) -> dict[str, Any]:
    for record in records:
        if record.get("event") == "session_start":
            metadata = record.get("metadata")
            return metadata if isinstance(metadata, dict) else {}
    return {}


def metadata_is_dry_run(metadata: dict[str, Any]) -> bool:
    if metadata.get("real_input_sent") is False:
        return True
    adapter = metadata.get("adapter")
    return isinstance(adapter, str) and adapter == "DryRunInputAdapter"


def run_tracking_log_check(
    *,
    log_path: Path | None,
    log_dir: Path,
    allow_latest: bool,
    min_samples: int,
    min_found_ratio: float,
    max_loss_events: int,
    max_abs_yaw_axis: float,
    max_abs_pitch_axis: float,
    require_dry_run: bool,
) -> TrackingLogCheckResult:
    selected_log = log_path or (latest_tracking_log(log_dir) if allow_latest else None)
    if selected_log is None:
        return TrackingLogCheckResult(
            status=WAITING,
            summary="no pr0p tracking JSONL log is available yet",
        metrics={"log_dir": str(log_dir), "log_path": None},
            notes=["TRACKING_LOG_MISSING"],
        )
    if not selected_log.exists():
        return TrackingLogCheckResult(
            status=WAITING,
            summary="selected pr0p tracking JSONL log does not exist",
            metrics={"log_path": str(selected_log)},
            notes=["TRACKING_LOG_MISSING"],
        )
    records, errors = load_jsonl(selected_log)
    metadata = find_metadata(records)
    samples = [record for record in records if record.get("event") == "game_screen_sample"]
    summary = find_summary(records)
    found_samples = [sample for sample in samples if sample.get("target_found")]
    sample_found_ratio = len(found_samples) / len(samples) if samples else 0.0
    summary_frames = int(summary.get("frames", -1)) if summary and "frames" in summary else None
    summary_found_ratio = float(summary.get("found_ratio", sample_found_ratio)) if summary else sample_found_ratio
    summary_loss_events = int(summary.get("loss_events", 0)) if summary else None
    max_yaw = max(
        command_abs_max(samples, "yaw"),
        float(summary.get("max_abs_yaw_axis", 0.0)) if summary else 0.0,
    )
    max_pitch = max(
        command_abs_max(samples, "pitch"),
        float(summary.get("max_abs_pitch_axis", 0.0)) if summary else 0.0,
    )
    metrics = {
        "log_path": str(selected_log),
        "records": len(records),
        "parse_errors": errors,
        "metadata": metadata,
        "sample_count": len(samples),
        "summary_present": summary is not None,
        "summary_frames": summary_frames,
        "sample_found_ratio": sample_found_ratio,
        "summary_found_ratio": summary_found_ratio,
        "summary_loss_events": summary_loss_events,
        "summary_horizontal_error_rms": summary.get("horizontal_error_rms") if summary else None,
        "summary_horizontal_error_p95": summary.get("horizontal_error_p95") if summary else None,
        "summary_forward_error_rms": summary.get("forward_error_rms") if summary else None,
        "summary_forward_error_p95": summary.get("forward_error_p95") if summary else None,
        "max_abs_yaw_axis": max_yaw,
        "max_abs_pitch_axis": max_pitch,
        "min_samples": min_samples,
        "min_found_ratio": min_found_ratio,
        "max_loss_events": max_loss_events,
        "max_abs_yaw_axis_limit": max_abs_yaw_axis,
        "max_abs_pitch_axis_limit": max_abs_pitch_axis,
        "require_dry_run": require_dry_run,
    }
    if errors:
        return TrackingLogCheckResult(
            status=FAIL,
            summary="tracking JSONL log contains parse errors",
            metrics=metrics,
            notes=["TRACKING_LOG_PARSE_FAIL"],
        )
    if summary is None:
        return TrackingLogCheckResult(
            status=FAIL,
            summary="tracking JSONL log has no summary event",
            metrics=metrics,
            notes=["TRACKING_LOG_SUMMARY_MISSING"],
        )
    if len(samples) < min_samples:
        return TrackingLogCheckResult(
            status=FAIL,
            summary="tracking JSONL log has too few sample events",
            metrics=metrics,
            notes=["TRACKING_LOG_TOO_FEW_SAMPLES"],
        )
    if summary_frames != len(samples):
        return TrackingLogCheckResult(
            status=FAIL,
            summary="tracking JSONL summary frame count does not match samples",
            metrics=metrics,
            notes=["TRACKING_LOG_FRAME_COUNT_MISMATCH"],
        )
    if summary_found_ratio < min_found_ratio or sample_found_ratio < min_found_ratio:
        return TrackingLogCheckResult(
            status=FAIL,
            summary="tracking JSONL found ratio is below threshold",
            metrics=metrics,
            notes=["TRACKER_LOSS"],
        )
    if summary_loss_events is None or summary_loss_events > max_loss_events:
        return TrackingLogCheckResult(
            status=FAIL,
            summary="tracking JSONL loss events exceeded threshold",
            metrics=metrics,
            notes=["TRACKER_LOSS"],
        )
    if max_yaw > max_abs_yaw_axis or max_pitch > max_abs_pitch_axis:
        return TrackingLogCheckResult(
            status=FAIL,
            summary="tracking JSONL PID command exceeded configured bounds",
            metrics=metrics,
            notes=["PID_COMMAND_UNBOUNDED"],
        )
    if require_dry_run and not metadata_is_dry_run(metadata):
        return TrackingLogCheckResult(
            status=FAIL,
            summary="tracking JSONL was not produced by a dry-run input adapter",
            metrics=metrics,
            notes=["TRACKING_LOG_NOT_DRY_RUN"],
        )
    return TrackingLogCheckResult(
        status=PASS,
        summary="tracking JSONL confirms bounded tracker/PID dry-run behavior",
        metrics=metrics,
        notes=["TRACKING_LOG_VERIFIED"],
    )


def build_markdown(result: TrackingLogCheckResult, *, run_id: str) -> str:
    lines = [
        "# SimITL / pr0p Tracking Log Check",
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
        "log_path",
        "sample_count",
        "summary_frames",
        "summary_found_ratio",
        "summary_loss_events",
        "summary_horizontal_error_rms",
        "summary_horizontal_error_p95",
        "summary_forward_error_rms",
        "summary_forward_error_p95",
        "max_abs_yaw_axis",
        "max_abs_pitch_axis",
        "require_dry_run",
    ):
        if key in result.metrics:
            lines.append("| %s | `%s` |" % (key, result.metrics[key]))
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


def write_reports(result: TrackingLogCheckResult, log_dir: Path, *, run_id: str) -> tuple[Path, Path]:
    log_dir.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime("%Y%m%d-%H%M%S", time.localtime())
    json_path = log_dir / ("%s-%s-tracking-log-check.json" % (stamp, run_id))
    md_path = log_dir / ("%s-%s-tracking-log-check.md" % (stamp, run_id))
    json_path.write_text(json.dumps(result.as_dict(), indent=2, sort_keys=True) + "\n",
                         encoding="utf-8")
    md_path.write_text(build_markdown(result, run_id=run_id), encoding="utf-8")
    return json_path, md_path


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-id", default="pr0p-tracking-log-check")
    parser.add_argument("--log-dir", type=Path, default=DEFAULT_LOG_DIR)
    parser.add_argument("--tracking-log", type=Path)
    parser.add_argument("--no-latest", action="store_true",
                        help="Do not fall back to the latest *-p6-tracking.jsonl in --log-dir")
    parser.add_argument("--min-samples", type=int, default=3)
    parser.add_argument("--min-found-ratio", type=float, default=0.85)
    parser.add_argument("--max-loss-events", type=int, default=0)
    parser.add_argument("--max-abs-yaw-axis", type=float, default=0.6)
    parser.add_argument("--max-abs-pitch-axis", type=float, default=0.6)
    parser.add_argument("--require-dry-run", action="store_true", default=True)
    parser.add_argument("--allow-live-input-log", action="store_true",
                        help="Do not require metadata.real_input_sent=false")
    args = parser.parse_args(argv)
    if args.min_samples <= 0:
        parser.error("--min-samples must be positive")
    if not 0 <= args.min_found_ratio <= 1:
        parser.error("--min-found-ratio must be between 0 and 1")
    if args.max_loss_events < 0:
        parser.error("--max-loss-events must be non-negative")
    if args.max_abs_yaw_axis < 0 or args.max_abs_pitch_axis < 0:
        parser.error("--max-abs-* limits must be non-negative")
    if args.allow_live_input_log:
        args.require_dry_run = False
    return args


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    result = run_tracking_log_check(
        log_path=args.tracking_log,
        log_dir=args.log_dir,
        allow_latest=not args.no_latest,
        min_samples=args.min_samples,
        min_found_ratio=args.min_found_ratio,
        max_loss_events=args.max_loss_events,
        max_abs_yaw_axis=args.max_abs_yaw_axis,
        max_abs_pitch_axis=args.max_abs_pitch_axis,
        require_dry_run=args.require_dry_run,
    )
    json_path, md_path = write_reports(result, args.log_dir, run_id=args.run_id)
    print("simitl-pr0p-tracking-log-check %s report=%s summary=%s" % (
        result.status,
        json_path,
        md_path,
    ))
    print(result.summary)
    return 1 if result.status == FAIL else 0


if __name__ == "__main__":
    raise SystemExit(main())
