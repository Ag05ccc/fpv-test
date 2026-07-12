#!/usr/bin/env python3
"""Run bounded live follow only after live-input readiness passes."""

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
    EXTERNAL_FOLLOW_LIVE_RUN_COMPLETE,
    run_external_follow_session,
    write_reports as write_follow_reports,
)
from external_live_input_readiness import (  # noqa: E402
    ACK_LIVE_INPUT_REQUIRED,
    EXTERNAL_LIVE_INPUT_READY,
    run_live_input_readiness,
    write_reports as write_readiness_reports,
)
from latency_probe import normalize_axes, parse_axis_shift_expectations  # noqa: E402
from phase_runner import load_tracker_bbox_file  # noqa: E402
from sandbox_status_report import DEFAULT_BBOX_FILE, DEFAULT_LOG_DIR, REJECT, WAITING  # noqa: E402
from sitl_log import resolve_log_dir, timestamp_slug  # noqa: E402
from kenet.tracker import TrackerType  # noqa: E402


EXTERNAL_LIVE_FOLLOW_COMPLETE = "EXTERNAL_LIVE_FOLLOW_COMPLETE"
NOT_RUN = "NOT_RUN"


@dataclass
class ExternalLiveFollowSequenceReport:
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


def report_evidence(evidence_paths: dict[str, str | None], prefix: str) -> dict[str, str | None]:
    return {
        "json": evidence_paths.get("%s_report_json" % prefix),
        "markdown": evidence_paths.get("%s_report_md" % prefix),
    }


def sequence_steps(
    *,
    ack_live_input: bool,
    evidence_paths: dict[str, str | None],
    readiness: Any | None = None,
    follow: Any | None = None,
) -> list[dict[str, Any]]:
    return [
        {
            "step": "ack_live_input",
            "status": "PASS" if ack_live_input else WAITING,
            "summary": (
                "explicit live-input acknowledgement provided"
                if ack_live_input else
                "live follow is not allowed without --ack-live-input"
            ),
            "evidence_paths": {"json": None, "markdown": None},
        },
        {
            "step": "live_input_readiness",
            "status": readiness.status if readiness else NOT_RUN,
            "summary": (
                readiness.summary
                if readiness else
                "not run before live-input acknowledgement"
            ),
            "reasons": list(readiness.reasons) if readiness else [],
            "evidence_paths": report_evidence(evidence_paths, "readiness"),
        },
        {
            "step": "live_follow",
            "status": follow.status if follow else NOT_RUN,
            "summary": (
                follow.summary
                if follow else
                "not run before live-input readiness passes"
            ),
            "reasons": list(follow.reasons) if follow else [],
            "evidence_paths": report_evidence(evidence_paths, "follow"),
        },
    ]


def build_metrics(
    *,
    base_metrics: dict[str, Any],
    evidence_paths: dict[str, str | None],
    ack_live_input: bool,
    readiness: Any | None = None,
    follow: Any | None = None,
) -> dict[str, Any]:
    metrics = {
        **base_metrics,
        "sequence_steps": sequence_steps(
            ack_live_input=ack_live_input,
            evidence_paths=evidence_paths,
            readiness=readiness,
            follow=follow,
        ),
    }
    if readiness is not None:
        metrics["readiness"] = readiness.as_dict()
    if follow is not None:
        metrics["follow"] = follow.as_dict()
    return metrics


def build_markdown(report: ExternalLiveFollowSequenceReport) -> str:
    lines = [
        "# External Live Follow Sequence",
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
        lines.append("- live-input readiness and bounded live follow passed")
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
    report: ExternalLiveFollowSequenceReport,
    log_dir: Path,
) -> tuple[Path, Path]:
    slug = "%s-%s" % (timestamp_slug(), report.run_id)
    json_path = log_dir / ("%s-external-live-follow-sequence.json" % slug)
    md_path = log_dir / ("%s-external-live-follow-sequence.md" % slug)
    json_path.write_text(json.dumps(report.as_dict(), indent=2, sort_keys=True) + "\n",
                         encoding="utf-8")
    md_path.write_text(build_markdown(report), encoding="utf-8")
    return json_path, md_path


def run_live_follow_sequence(
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
    min_fps: float,
    min_found_ratio: float,
    max_loss_events: int,
    max_yaw_axis: float,
    max_pitch_axis: float,
    ack_live_input: bool,
    live_max_duration_s: float,
    axis_names: tuple[str, ...],
    axis_value: float,
    window_exact: bool,
    window_case_sensitive: bool,
    window_min_width: int,
    window_min_height: int,
    axis_expected_shifts: dict[str, tuple[str, int]] | None = None,
    axis_min_shift_px: float = 0.5,
) -> ExternalLiveFollowSequenceReport:
    log_dir.mkdir(parents=True, exist_ok=True)
    started_at = time.strftime("%Y-%m-%dT%H:%M:%S%z", time.localtime())
    evidence_paths: dict[str, str | None] = {
        "readiness_report_json": None,
        "readiness_report_md": None,
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
        "min_fps": min_fps,
        "min_found_ratio": min_found_ratio,
        "max_loss_events": max_loss_events,
        "max_yaw_axis": max_yaw_axis,
        "max_pitch_axis": max_pitch_axis,
        "ack_live_input": ack_live_input,
        "live_max_duration_s": live_max_duration_s,
        "axis_names": list(axis_names),
        "axis_value": axis_value,
        "axis_expected_shifts": {
            axis: "%s%s" % ("+" if sign > 0 else "-", component)
            for axis, (component, sign) in (axis_expected_shifts or {}).items()
        } or None,
        "axis_min_shift_px": axis_min_shift_px if axis_expected_shifts else None,
        "scope": "external bounded live follow only; no Gazebo or Betaflight",
    }
    if not ack_live_input:
        return ExternalLiveFollowSequenceReport(
            run_id=run_id,
            started_at=started_at,
            status=WAITING,
            summary="external live follow sequence requires explicit acknowledgement",
            reasons=[ACK_LIVE_INPUT_REQUIRED],
            evidence_paths=evidence_paths,
            metrics=build_metrics(
                base_metrics=base_metrics,
                evidence_paths=evidence_paths,
                ack_live_input=False,
            ),
        )

    readiness = run_live_input_readiness(
        run_id="%s-readiness" % run_id,
        log_dir=log_dir,
        region=region,
        window_title=window_title,
        backend=backend,
        bbox_file=bbox_file,
        duration_s=duration_s,
        hz=hz,
        tracker_type=tracker_type,
        enable_pitch=enable_pitch,
        desired_target_width=desired_target_width,
        min_fps=min_fps,
        min_found_ratio=min_found_ratio,
        max_loss_events=max_loss_events,
        max_yaw_axis=max_yaw_axis,
        max_pitch_axis=max_pitch_axis,
        ack_live_input=True,
        axis_names=axis_names,
        axis_value=axis_value,
        axis_expected_shifts=axis_expected_shifts,
        axis_min_shift_px=axis_min_shift_px,
        window_exact=window_exact,
        window_case_sensitive=window_case_sensitive,
        window_min_width=window_min_width,
        window_min_height=window_min_height,
    )
    readiness_json, readiness_md = write_readiness_reports(readiness, log_dir)
    evidence_paths["readiness_report_json"] = str(readiness_json)
    evidence_paths["readiness_report_md"] = str(readiness_md)
    if readiness.status != EXTERNAL_LIVE_INPUT_READY:
        status = REJECT if readiness.status == REJECT else WAITING
        reasons = [
            "LIVE_INPUT_READINESS:%s:%s" % (readiness.status, reason)
            for reason in (readiness.reasons or ["NO_REASON"])
        ]
        return ExternalLiveFollowSequenceReport(
            run_id=run_id,
            started_at=started_at,
            status=status,
            summary="external live follow sequence stopped before live follow",
            reasons=reasons,
            evidence_paths=evidence_paths,
            metrics=build_metrics(
                base_metrics=base_metrics,
                evidence_paths=evidence_paths,
                ack_live_input=True,
                readiness=readiness,
            ),
        )

    try:
        bbox = load_tracker_bbox_file(bbox_file)
    except Exception as exc:
        return ExternalLiveFollowSequenceReport(
            run_id=run_id,
            started_at=started_at,
            status=REJECT,
            summary="external live follow sequence bbox is invalid after readiness",
            reasons=["BBOX_INVALID:%s" % exc],
            evidence_paths=evidence_paths,
            metrics=build_metrics(
                base_metrics=base_metrics,
                evidence_paths=evidence_paths,
                ack_live_input=True,
                readiness=readiness,
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
        use_uinput=True,
        ack_live_input=True,
        live_max_duration_s=live_max_duration_s,
        window_exact=window_exact,
        window_case_sensitive=window_case_sensitive,
        window_min_width=window_min_width,
        window_min_height=window_min_height,
    )
    follow_json, follow_md = write_follow_reports(follow, log_dir)
    evidence_paths["follow_report_json"] = str(follow_json)
    evidence_paths["follow_report_md"] = str(follow_md)
    if follow.status != EXTERNAL_FOLLOW_LIVE_RUN_COMPLETE:
        status = REJECT if follow.status == REJECT else WAITING
        reasons = [
            "LIVE_FOLLOW:%s:%s" % (follow.status, reason)
            for reason in (follow.reasons or ["NO_REASON"])
        ]
        return ExternalLiveFollowSequenceReport(
            run_id=run_id,
            started_at=started_at,
            status=status,
            summary="external live follow sequence stopped at live follow",
            reasons=reasons,
            evidence_paths=evidence_paths,
            metrics=build_metrics(
                base_metrics=base_metrics,
                evidence_paths=evidence_paths,
                ack_live_input=True,
                readiness=readiness,
                follow=follow,
            ),
        )

    return ExternalLiveFollowSequenceReport(
        run_id=run_id,
        started_at=started_at,
        status=EXTERNAL_LIVE_FOLLOW_COMPLETE,
        summary="external live-input readiness and bounded live follow passed",
        reasons=[],
        evidence_paths=evidence_paths,
        metrics=build_metrics(
            base_metrics=base_metrics,
            evidence_paths=evidence_paths,
            ack_live_input=True,
            readiness=readiness,
            follow=follow,
        ),
    )


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-id", default="external-live-follow-sequence")
    parser.add_argument("--log-dir", type=Path, default=DEFAULT_LOG_DIR)
    parser.add_argument("--capture-region", type=parse_region)
    parser.add_argument("--capture-window-title", default="pr0p")
    parser.add_argument("--capture-backend", choices=["auto", "mss", "ffmpeg"], default="ffmpeg")
    parser.add_argument("--tracker-bbox-file", type=Path, default=DEFAULT_BBOX_FILE)
    parser.add_argument("--duration", type=float, default=1.0)
    parser.add_argument("--hz", type=float, default=10.0)
    parser.add_argument("--tracker", choices=[TrackerType.CSRT, TrackerType.KCF],
                        default=TrackerType.KCF)
    parser.add_argument("--enable-pitch", action="store_true")
    parser.add_argument("--desired-target-width", type=float, default=120.0)
    parser.add_argument("--min-fps", type=float, default=0.0)
    parser.add_argument("--min-found-ratio", type=float, default=0.80)
    parser.add_argument("--max-loss-events", type=int, default=0)
    parser.add_argument("--max-yaw-axis", type=float, default=0.8)
    parser.add_argument("--max-pitch-axis", type=float, default=0.8)
    parser.add_argument("--ack-live-input", action="store_true",
                        help="Required before readiness axis sweep and live follow send uinput")
    parser.add_argument("--live-max-duration", type=float, default=2.0)
    parser.add_argument("--axis-sweep-axes", default="yaw,pitch")
    parser.add_argument("--axis-sweep-value", type=float, default=0.05)
    parser.add_argument("--axis-expected-shifts", default="",
                        help="Optional signed visual shift checks, e.g. yaw:+x,pitch:-y")
    parser.add_argument("--axis-min-shift-px", type=float, default=0.5)
    parser.add_argument("--window-exact", action="store_true")
    parser.add_argument("--window-case-sensitive", action="store_true")
    parser.add_argument("--window-min-width", type=int, default=32)
    parser.add_argument("--window-min-height", type=int, default=32)
    args = parser.parse_args(argv)
    if args.duration <= 0:
        parser.error("--duration must be positive")
    if args.hz <= 0:
        parser.error("--hz must be positive")
    if args.min_fps < 0:
        parser.error("--min-fps must be non-negative")
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
    if args.live_max_duration <= 0:
        parser.error("--live-max-duration must be positive")
    if not 0 < abs(args.axis_sweep_value) <= 1:
        parser.error("--axis-sweep-value magnitude must be in (0, 1]")
    try:
        args.axis_sweep_axes = normalize_axes(args.axis_sweep_axes)
    except ValueError as exc:
        parser.error(str(exc))
    try:
        args.axis_expected_shifts = parse_axis_shift_expectations(args.axis_expected_shifts)
    except ValueError as exc:
        parser.error(str(exc))
    if args.axis_min_shift_px < 0:
        parser.error("--axis-min-shift-px must be non-negative")
    if args.window_min_width <= 0 or args.window_min_height <= 0:
        parser.error("--window-min-width/--window-min-height must be positive")
    return args


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    log_dir = resolve_log_dir(str(args.log_dir), repo_root=REPO_ROOT)
    report = run_live_follow_sequence(
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
        min_fps=args.min_fps,
        min_found_ratio=args.min_found_ratio,
        max_loss_events=args.max_loss_events,
        max_yaw_axis=args.max_yaw_axis,
        max_pitch_axis=args.max_pitch_axis,
        ack_live_input=args.ack_live_input,
        live_max_duration_s=args.live_max_duration,
        axis_names=args.axis_sweep_axes,
        axis_value=args.axis_sweep_value,
        window_exact=args.window_exact,
        window_case_sensitive=args.window_case_sensitive,
        window_min_width=args.window_min_width,
        window_min_height=args.window_min_height,
        axis_expected_shifts=args.axis_expected_shifts,
        axis_min_shift_px=args.axis_min_shift_px,
    )
    json_path, md_path = write_reports(report, log_dir)
    print("external-live-follow-sequence %s report=%s summary=%s" % (
        report.status,
        json_path,
        md_path,
    ))
    if report.status == EXTERNAL_LIVE_FOLLOW_COMPLETE:
        return 0
    if report.status == WAITING:
        return 2
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
