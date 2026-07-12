#!/usr/bin/env python3
"""Run status -> preflight -> follow dry-run for an external game window."""

from __future__ import annotations

import argparse
import json
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))
TOOLS_DIR = REPO_ROOT / "tools"
if str(TOOLS_DIR) not in sys.path:
    sys.path.insert(0, str(TOOLS_DIR))

from capture_window import parse_region  # noqa: E402
from external_follow_session import (  # noqa: E402
    EXTERNAL_FOLLOW_DRY_RUN_READY,
    run_external_follow_session,
    write_reports as write_follow_reports,
)
from external_window_preflight import (  # noqa: E402
    EXTERNAL_WINDOW_READY_DRY,
    EXTERNAL_WINDOW_READY_LIVE_INPUT,
    run_preflight as run_external_preflight,
    write_reports as write_preflight_reports,
)
from phase_runner import load_tracker_bbox_file  # noqa: E402
from sandbox_status_report import (  # noqa: E402
    PASS,
    READY,
    REJECT,
    WAITING,
    DEFAULT_BBOX_FILE,
    DEFAULT_LOG_DIR,
    SandboxStatusReport,
    build_status_report,
    write_reports as write_status_reports,
)
from sitl_log import resolve_log_dir, timestamp_slug  # noqa: E402
from kenet.tracker import TrackerType  # noqa: E402


EXTERNAL_DRY_RUN_READY = "EXTERNAL_DRY_RUN_READY"
NOT_RUN = "NOT_RUN"
READY_FOR_OPTIONAL_LIVE_INPUT = "READY_FOR_OPTIONAL_LIVE_INPUT"
WAITING_FOR_DRY_RUN = "WAITING_FOR_DRY_RUN"
REQUIRED_STATUS_ITEMS = ("target_window", "target_bbox", "gazebo_betaflight_process_boundary")
REGION_REQUIRED_STATUS_ITEMS = ("target_bbox", "gazebo_betaflight_process_boundary")


@dataclass
class ExternalDryRunSequenceReport:
    run_id: str
    started_at: str
    status: str
    summary: str
    reasons: list[str] = field(default_factory=list)
    evidence_paths: dict[str, str | None] = field(default_factory=dict)
    metrics: dict[str, Any] = field(default_factory=dict)

    def as_dict(self) -> dict[str, Any]:
        return {
            "run_id": self.run_id,
            "started_at": self.started_at,
            "status": self.status,
            "summary": self.summary,
            "reasons": self.reasons,
            "evidence_paths": self.evidence_paths,
            "metrics": self.metrics,
        }


def item_status(report: SandboxStatusReport, name: str) -> str | None:
    for item in report.items:
        if item.name == name:
            return item.status
    return None


def item_summary(report: SandboxStatusReport, name: str) -> str | None:
    for item in report.items:
        if item.name == name:
            return item.summary
    return None


def required_status_items_for(region: dict[str, int] | None) -> tuple[str, ...]:
    return REGION_REQUIRED_STATUS_ITEMS if region is not None else REQUIRED_STATUS_ITEMS


def pre_sequence_reasons(
    report: SandboxStatusReport,
    required_items: tuple[str, ...] = REQUIRED_STATUS_ITEMS,
) -> list[str]:
    reasons: list[str] = []
    for name in required_items:
        status = item_status(report, name)
        if status != PASS:
            reasons.append("%s:%s:%s" % (
                name,
                status or "MISSING",
                item_summary(report, name) or "missing status item",
            ))
    return reasons


def setup_gate_status(
    report: SandboxStatusReport,
    required_items: tuple[str, ...] = REQUIRED_STATUS_ITEMS,
) -> str:
    statuses = [item_status(report, name) for name in required_items]
    if all(status == PASS for status in statuses):
        return PASS
    if any(status == REJECT for status in statuses):
        return REJECT
    return WAITING


def report_evidence(evidence_paths: dict[str, str | None], prefix: str) -> dict[str, str | None]:
    return {
        "json": evidence_paths.get("%s_report_json" % prefix),
        "markdown": evidence_paths.get("%s_report_md" % prefix),
    }


def build_sequence_steps(
    *,
    status_report: SandboxStatusReport,
    evidence_paths: dict[str, str | None],
    required_items: tuple[str, ...] = REQUIRED_STATUS_ITEMS,
    preflight: Any | None = None,
    follow: Any | None = None,
) -> list[dict[str, Any]]:
    status_reasons = pre_sequence_reasons(status_report, required_items)
    status_gate = setup_gate_status(status_report, required_items)
    status_summary = (
        "required external dry-run setup checks passed"
        if status_gate == PASS
        else status_report.summary
    )
    steps: list[dict[str, Any]] = [
        {
            "step": "status",
            "status": status_gate,
            "source_status": status_report.status,
            "source_summary": status_report.summary,
            "summary": status_summary,
            "reasons": status_reasons,
            "evidence_paths": report_evidence(evidence_paths, "status"),
        },
        {
            "step": "preflight",
            "status": preflight.status if preflight else NOT_RUN,
            "summary": preflight.summary if preflight else "not run before status setup passed",
            "reasons": list(preflight.reasons) if preflight else [],
            "evidence_paths": report_evidence(evidence_paths, "preflight"),
        },
        {
            "step": "follow",
            "status": follow.status if follow else NOT_RUN,
            "summary": follow.summary if follow else "not run before preflight passed",
            "reasons": list(follow.reasons) if follow else [],
            "evidence_paths": report_evidence(evidence_paths, "follow"),
        },
    ]
    return steps


def live_readiness(status: str) -> str:
    if status == EXTERNAL_DRY_RUN_READY:
        return READY_FOR_OPTIONAL_LIVE_INPUT
    return WAITING_FOR_DRY_RUN


def live_readiness_notes(status: str) -> list[str]:
    if status == EXTERNAL_DRY_RUN_READY:
        return [
            "dry sequence passed",
            "run axis-response preflight with --include-axis-sweep --ack-live-input if RC channel response is unproven",
            "live follow still requires --uinput --ack-live-input --live-max-duration",
        ]
    return [
        "do not run live input until status, preflight, and follow dry-run pass",
    ]


def sequence_metrics(
    *,
    base_metrics: dict[str, Any],
    final_status: str,
    status_report: SandboxStatusReport,
    evidence_paths: dict[str, str | None],
    required_items: tuple[str, ...] = REQUIRED_STATUS_ITEMS,
    preflight: Any | None = None,
    follow: Any | None = None,
) -> dict[str, Any]:
    metrics = {
        **base_metrics,
        "sequence_steps": build_sequence_steps(
            status_report=status_report,
            evidence_paths=evidence_paths,
            required_items=required_items,
            preflight=preflight,
            follow=follow,
        ),
        "required_status_items": list(required_items),
        "live_readiness": live_readiness(final_status),
        "live_readiness_notes": live_readiness_notes(final_status),
    }
    if preflight is not None:
        metrics["preflight"] = preflight.as_dict()
    if follow is not None:
        metrics["follow"] = follow.as_dict()
    return metrics


def build_markdown(report: ExternalDryRunSequenceReport) -> str:
    lines = [
        "# External Dry-Run Sequence",
        "",
        "Run: `%s`" % report.run_id,
        "Started: `%s`" % report.started_at,
        "Status: `%s`" % report.status,
        "",
        report.summary,
        "",
        "## Sequence Steps",
        "",
        "| Step | Status | Summary | Evidence |",
        "| --- | --- | --- | --- |",
    ]
    for step in report.metrics.get("sequence_steps", []):
        evidence = step.get("evidence_paths", {})
        if isinstance(evidence, dict):
            evidence_text = ", ".join(
                "%s=%s" % (key, value)
                for key, value in evidence.items()
                if value
            ) or "none"
        else:
            evidence_text = "none"
        lines.append("| %s | `%s` | %s | `%s` |" % (
            step.get("step", "unknown"),
            step.get("status", "UNKNOWN"),
            step.get("summary", ""),
            evidence_text,
        ))
    readiness = report.metrics.get("live_readiness")
    if readiness:
        lines.extend([
            "",
            "## Live Readiness",
            "",
            "Status: `%s`" % readiness,
            "",
        ])
        for note in report.metrics.get("live_readiness_notes", []):
            lines.append("- %s" % note)
    lines.extend([
        "",
        "## Evidence",
        "",
        "| Artifact | Path |",
        "| --- | --- |",
    ])
    for key, value in report.evidence_paths.items():
        lines.append("| %s | `%s` |" % (key, value or "none"))
    lines.extend(["", "## Reasons", ""])
    if report.reasons:
        lines.extend("- %s" % reason for reason in report.reasons)
    else:
        lines.append("- status, preflight, and follow dry-run passed")
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


def write_reports(
    report: ExternalDryRunSequenceReport,
    log_dir: Path,
) -> tuple[Path, Path]:
    slug = "%s-%s" % (timestamp_slug(), report.run_id)
    json_path = log_dir / ("%s-external-dry-run-sequence.json" % slug)
    md_path = log_dir / ("%s-external-dry-run-sequence.md" % slug)
    json_path.write_text(json.dumps(report.as_dict(), indent=2, sort_keys=True) + "\n",
                         encoding="utf-8")
    md_path.write_text(build_markdown(report), encoding="utf-8")
    return json_path, md_path


def run_sequence(
    *,
    run_id: str,
    log_dir: Path,
    region: dict[str, int] | None,
    window_title: str | None,
    backend: str,
    bbox_file: Path,
    duration_s: float,
    hz: float,
    tracker_type: str,
    enable_pitch: bool,
    desired_target_width: float,
    min_found_ratio: float,
    max_loss_events: int,
    max_yaw_axis: float,
    max_pitch_axis: float,
    window_exact: bool,
    window_case_sensitive: bool,
    window_min_width: int,
    window_min_height: int,
) -> ExternalDryRunSequenceReport:
    log_dir.mkdir(parents=True, exist_ok=True)
    started_at = time.strftime("%Y-%m-%dT%H:%M:%S%z", time.localtime())
    status_report = build_status_report(
        run_id="%s-status" % run_id,
        log_dir=log_dir,
        window_title=window_title,
        bbox_file=bbox_file,
        window_exact=window_exact,
        window_case_sensitive=window_case_sensitive,
        window_min_width=window_min_width,
        window_min_height=window_min_height,
    )
    status_json, status_md = write_status_reports(status_report, log_dir)
    required_items = required_status_items_for(region)
    reasons = pre_sequence_reasons(status_report, required_items)
    evidence_paths: dict[str, str | None] = {
        "status_report_json": str(status_json),
        "status_report_md": str(status_md),
        "preflight_report_json": None,
        "preflight_report_md": None,
        "follow_report_json": None,
        "follow_report_md": None,
    }
    base_metrics: dict[str, Any] = {
        "window_title": window_title,
        "region": region,
        "backend": backend,
        "bbox_file": str(bbox_file),
        "duration_s": duration_s,
        "hz": hz,
        "tracker_type": tracker_type,
        "enable_pitch": enable_pitch,
        "desired_target_width": desired_target_width,
        "status_report": status_report.as_dict(),
        "scope": "external dry-run sequence only; no Gazebo, Betaflight, or uinput",
    }
    if reasons:
        return ExternalDryRunSequenceReport(
            run_id=run_id,
            started_at=started_at,
            status=WAITING,
            summary="external dry-run sequence is waiting for setup",
            reasons=reasons,
            evidence_paths=evidence_paths,
            metrics=sequence_metrics(
                base_metrics=base_metrics,
                final_status=WAITING,
                status_report=status_report,
                evidence_paths=evidence_paths,
                required_items=required_items,
            ),
        )

    try:
        bbox = load_tracker_bbox_file(bbox_file)
    except Exception as exc:
        return ExternalDryRunSequenceReport(
            run_id=run_id,
            started_at=started_at,
            status=REJECT,
            summary="external dry-run sequence bbox is invalid",
            reasons=["BBOX_INVALID:%s" % exc],
            evidence_paths=evidence_paths,
            metrics=sequence_metrics(
                base_metrics=base_metrics,
                final_status=REJECT,
                status_report=status_report,
                evidence_paths=evidence_paths,
                required_items=required_items,
            ),
        )

    preflight = run_external_preflight(
        run_id="%s-preflight" % run_id,
        log_dir=log_dir,
        region=region,
        window_title=window_title,
        backend=backend,
        bbox=bbox,
        frame_path=None,
        overlay_path=None,
        duration_s=duration_s,
        hz=hz,
        min_fps=0.0,
        min_found_ratio=min_found_ratio,
        desired_target_width=desired_target_width,
        max_yaw_axis=max_yaw_axis,
        max_pitch_axis=max_pitch_axis,
        include_axis_sweep=False,
        ack_live_input=False,
        axis_names=("yaw", "pitch"),
        axis_value=0.05,
        window_exact=window_exact,
        window_case_sensitive=window_case_sensitive,
        window_min_width=window_min_width,
        window_min_height=window_min_height,
    )
    preflight_json, preflight_md = write_preflight_reports(preflight, log_dir)
    evidence_paths["preflight_report_json"] = str(preflight_json)
    evidence_paths["preflight_report_md"] = str(preflight_md)
    if preflight.status not in {EXTERNAL_WINDOW_READY_DRY, EXTERNAL_WINDOW_READY_LIVE_INPUT}:
        return ExternalDryRunSequenceReport(
            run_id=run_id,
            started_at=started_at,
            status=REJECT if preflight.status == REJECT else WAITING,
            summary="external dry-run sequence stopped at preflight",
            reasons=["PREFLIGHT:%s:%s" % (preflight.status, reason)
                     for reason in (preflight.reasons or ["NO_REASON"])],
            evidence_paths=evidence_paths,
            metrics=sequence_metrics(
                base_metrics=base_metrics,
                final_status=REJECT if preflight.status == REJECT else WAITING,
                status_report=status_report,
                evidence_paths=evidence_paths,
                required_items=required_items,
                preflight=preflight,
            ),
        )

    follow = run_external_follow_session(
        run_id="%s-follow" % run_id,
        log_dir=log_dir,
        region=region,
        window_title=window_title,
        backend=backend,
        tracker_bbox=bbox,
        duration_s=duration_s,
        hz=hz,
        tracker_type=tracker_type,
        enable_pitch=enable_pitch,
        desired_target_width=desired_target_width,
        min_found_ratio=min_found_ratio,
        max_loss_events=max_loss_events,
        max_yaw_axis=max_yaw_axis,
        max_pitch_axis=max_pitch_axis,
        use_uinput=False,
        ack_live_input=False,
        live_max_duration_s=2.0,
        window_exact=window_exact,
        window_case_sensitive=window_case_sensitive,
        window_min_width=window_min_width,
        window_min_height=window_min_height,
    )
    follow_json, follow_md = write_follow_reports(follow, log_dir)
    evidence_paths["follow_report_json"] = str(follow_json)
    evidence_paths["follow_report_md"] = str(follow_md)
    if follow.status != EXTERNAL_FOLLOW_DRY_RUN_READY:
        return ExternalDryRunSequenceReport(
            run_id=run_id,
            started_at=started_at,
            status=REJECT if follow.status == REJECT else WAITING,
            summary="external dry-run sequence stopped at follow session",
            reasons=["FOLLOW:%s:%s" % (follow.status, reason)
                     for reason in (follow.reasons or ["NO_REASON"])],
            evidence_paths=evidence_paths,
            metrics=sequence_metrics(
                base_metrics=base_metrics,
                final_status=REJECT if follow.status == REJECT else WAITING,
                status_report=status_report,
                evidence_paths=evidence_paths,
                required_items=required_items,
                preflight=preflight,
                follow=follow,
            ),
        )

    return ExternalDryRunSequenceReport(
        run_id=run_id,
        started_at=started_at,
        status=EXTERNAL_DRY_RUN_READY,
        summary="external status, preflight, and follow dry-run passed",
        reasons=[],
        evidence_paths=evidence_paths,
        metrics=sequence_metrics(
            base_metrics=base_metrics,
            final_status=EXTERNAL_DRY_RUN_READY,
            status_report=status_report,
            evidence_paths=evidence_paths,
            required_items=required_items,
            preflight=preflight,
            follow=follow,
        ),
    )


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-id", default="external-dry-run-sequence")
    parser.add_argument("--log-dir", type=Path, default=DEFAULT_LOG_DIR)
    parser.add_argument("--capture-region", type=parse_region)
    parser.add_argument("--capture-window-title", default="pr0p")
    parser.add_argument("--capture-backend", choices=["auto", "mss", "ffmpeg"], default="ffmpeg")
    parser.add_argument("--tracker-bbox-file", type=Path, default=DEFAULT_BBOX_FILE)
    parser.add_argument("--duration", type=float, default=2.0)
    parser.add_argument("--hz", type=float, default=10.0)
    parser.add_argument("--tracker", choices=[TrackerType.CSRT, TrackerType.KCF],
                        default=TrackerType.KCF)
    parser.add_argument("--enable-pitch", action="store_true")
    parser.add_argument("--desired-target-width", type=float, default=120.0)
    parser.add_argument("--min-found-ratio", type=float, default=0.80)
    parser.add_argument("--max-loss-events", type=int, default=0)
    parser.add_argument("--max-yaw-axis", type=float, default=0.8)
    parser.add_argument("--max-pitch-axis", type=float, default=0.8)
    parser.add_argument("--window-exact", action="store_true")
    parser.add_argument("--window-case-sensitive", action="store_true")
    parser.add_argument("--window-min-width", type=int, default=32)
    parser.add_argument("--window-min-height", type=int, default=32)
    args = parser.parse_args(argv)
    if args.duration <= 0:
        parser.error("--duration must be positive")
    if args.hz <= 0:
        parser.error("--hz must be positive")
    if args.desired_target_width <= 0:
        parser.error("--desired-target-width must be positive")
    if not 0 <= args.min_found_ratio <= 1:
        parser.error("--min-found-ratio must be in [0, 1]")
    if args.max_loss_events < 0:
        parser.error("--max-loss-events must be non-negative")
    if not 0 < args.max_yaw_axis <= 1:
        parser.error("--max-yaw-axis must be in (0, 1]")
    if not 0 < args.max_pitch_axis <= 1:
        parser.error("--max-pitch-axis must be in (0, 1]")
    if args.window_min_width <= 0 or args.window_min_height <= 0:
        parser.error("--window-min-width/--window-min-height must be positive")
    return args


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    log_dir = resolve_log_dir(str(args.log_dir), repo_root=REPO_ROOT)
    report = run_sequence(
        run_id=args.run_id,
        log_dir=log_dir,
        region=args.capture_region,
        window_title=args.capture_window_title,
        backend=args.capture_backend,
        bbox_file=args.tracker_bbox_file,
        duration_s=args.duration,
        hz=args.hz,
        tracker_type=args.tracker,
        enable_pitch=args.enable_pitch,
        desired_target_width=args.desired_target_width,
        min_found_ratio=args.min_found_ratio,
        max_loss_events=args.max_loss_events,
        max_yaw_axis=args.max_yaw_axis,
        max_pitch_axis=args.max_pitch_axis,
        window_exact=args.window_exact,
        window_case_sensitive=args.window_case_sensitive,
        window_min_width=args.window_min_width,
        window_min_height=args.window_min_height,
    )
    json_path, md_path = write_reports(report, log_dir)
    print("external-dry-run-sequence %s report=%s summary=%s" % (
        report.status,
        json_path,
        md_path,
    ))
    if report.status == EXTERNAL_DRY_RUN_READY:
        return 0
    if report.status == WAITING:
        return 2
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
