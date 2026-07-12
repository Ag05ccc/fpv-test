#!/usr/bin/env python3
"""Run a bounded tracker/PID follow session on an external game window."""

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

from capture_window import make_region_source, parse_region, resolve_capture_region  # noqa: E402
from isolation_audit import (  # noqa: E402
    FAIL,
    PASS,
    audit_process_delta,
    audit_static_imports,
    combine_audits,
    process_snapshot,
)
from phase_runner import load_tracker_bbox_file  # noqa: E402
from screen_tracking_loop import LoopConfig, parse_bbox, run_loop  # noqa: E402
from sitl_log import JsonlLogger, make_log_path, resolve_log_dir, timestamp_slug  # noqa: E402
from virtual_input import (  # noqa: E402
    DryRunInputAdapter,
    UInputAdapter,
    UInputUnavailable,
    probe_uinput_environment,
)
from kenet.tracker import ObjectTracker, TrackerType  # noqa: E402


WAITING = "WAITING"
REJECT = "REJECT"
EXTERNAL_FOLLOW_DRY_RUN_READY = "EXTERNAL_FOLLOW_DRY_RUN_READY"
EXTERNAL_FOLLOW_LIVE_RUN_COMPLETE = "EXTERNAL_FOLLOW_LIVE_RUN_COMPLETE"
DEFAULT_LOG_DIR = Path("logs/game_screen_sandbox")


@dataclass
class ExternalFollowSessionReport:
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


def build_markdown(report: ExternalFollowSessionReport) -> str:
    lines = [
        "# External Follow Session",
        "",
        "Run: `%s`" % report.run_id,
        "Started: `%s`" % report.started_at,
        "Status: `%s`" % report.status,
        "",
        report.summary,
        "",
        "## Evidence",
        "",
        "| Artifact | Path |",
        "| --- | --- |",
    ]
    for key, value in report.evidence_paths.items():
        lines.append("| %s | `%s` |" % (key, value or "none"))
    lines.extend([
        "",
        "## Reasons",
        "",
    ])
    if report.reasons:
        lines.extend("- %s" % reason for reason in report.reasons)
    else:
        lines.append("- all external follow session gates passed")
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
    report: ExternalFollowSessionReport,
    log_dir: Path,
) -> tuple[Path, Path]:
    slug = "%s-%s" % (timestamp_slug(), report.run_id)
    json_path = log_dir / ("%s-external-follow-session.json" % slug)
    md_path = log_dir / ("%s-external-follow-session.md" % slug)
    json_path.write_text(json.dumps(report.as_dict(), indent=2, sort_keys=True) + "\n",
                         encoding="utf-8")
    md_path.write_text(build_markdown(report), encoding="utf-8")
    return json_path, md_path


def evaluate_session_metrics(
    metrics: dict[str, Any],
    *,
    min_found_ratio: float,
    max_loss_events: int,
    max_yaw_axis: float,
    max_pitch_axis: float,
    enable_pitch: bool,
) -> list[str]:
    reasons: list[str] = []
    if int(metrics.get("frames") or 0) <= 0:
        reasons.append("NO_FRAMES")
    found_ratio = float(metrics.get("found_ratio") or 0.0)
    if found_ratio < min_found_ratio:
        reasons.append("FOUND_RATIO_LOW:%.3f<%.3f" % (found_ratio, min_found_ratio))
    loss_events = int(metrics.get("loss_events") or 0)
    if loss_events > max_loss_events:
        reasons.append("LOSS_EVENTS_HIGH:%d>%d" % (loss_events, max_loss_events))
    yaw_axis = float(metrics.get("max_abs_yaw_axis") or 0.0)
    if yaw_axis > max_yaw_axis:
        reasons.append("YAW_AXIS_LIMIT:%.3f>%.3f" % (yaw_axis, max_yaw_axis))
    pitch_axis = float(metrics.get("max_abs_pitch_axis") or 0.0)
    if enable_pitch and pitch_axis > max_pitch_axis:
        reasons.append("PITCH_AXIS_LIMIT:%.3f>%.3f" % (pitch_axis, max_pitch_axis))
    return reasons


def waiting_report(
    *,
    run_id: str,
    started_at: str,
    summary: str,
    reasons: list[str],
    metrics: dict[str, Any],
) -> ExternalFollowSessionReport:
    return ExternalFollowSessionReport(
        run_id=run_id,
        started_at=started_at,
        status=WAITING,
        summary=summary,
        reasons=reasons,
        metrics=metrics,
    )


def run_external_follow_session(
    *,
    run_id: str,
    log_dir: Path,
    region: dict[str, int] | None,
    window_title: str | None,
    backend: str,
    tracker_bbox: tuple[int, int, int, int] | None,
    duration_s: float,
    hz: float,
    tracker_type: str,
    enable_pitch: bool,
    desired_target_width: float,
    min_found_ratio: float,
    max_loss_events: int,
    max_yaw_axis: float,
    max_pitch_axis: float,
    use_uinput: bool,
    ack_live_input: bool,
    live_max_duration_s: float,
    window_exact: bool,
    window_case_sensitive: bool,
    window_min_width: int,
    window_min_height: int,
) -> ExternalFollowSessionReport:
    log_dir.mkdir(parents=True, exist_ok=True)
    started_at = time.strftime("%Y-%m-%dT%H:%M:%S%z", time.localtime())
    base_metrics: dict[str, Any] = {
        "window_title": window_title,
        "region": region,
        "backend": backend,
        "tracker_bbox": list(tracker_bbox) if tracker_bbox else None,
        "duration_s": duration_s,
        "hz": hz,
        "tracker_type": tracker_type,
        "enable_pitch": enable_pitch,
        "desired_target_width": desired_target_width,
        "min_found_ratio": min_found_ratio,
        "max_loss_events": max_loss_events,
        "max_yaw_axis": max_yaw_axis,
        "max_pitch_axis": max_pitch_axis,
        "real_input": use_uinput,
        "ack_live_input": ack_live_input,
        "live_max_duration_s": live_max_duration_s,
        "scope": "external game/sim window follow session; no Gazebo/Betaflight promotion",
    }
    if region is None and not window_title:
        return waiting_report(
            run_id=run_id,
            started_at=started_at,
            summary="external follow session is waiting for a capture target",
            reasons=["CAPTURE_TARGET_REQUIRED"],
            metrics=base_metrics,
        )
    if tracker_bbox is None:
        return waiting_report(
            run_id=run_id,
            started_at=started_at,
            summary="external follow session is waiting for a target bbox",
            reasons=["TRACKER_BBOX_REQUIRED"],
            metrics=base_metrics,
        )
    if use_uinput and not ack_live_input:
        return waiting_report(
            run_id=run_id,
            started_at=started_at,
            summary="external follow live input requires explicit acknowledgement",
            reasons=["ACK_LIVE_INPUT_REQUIRED"],
            metrics=base_metrics,
        )
    if use_uinput and duration_s > live_max_duration_s:
        return waiting_report(
            run_id=run_id,
            started_at=started_at,
            summary="external follow live input duration exceeds safety limit",
            reasons=["LIVE_DURATION_LIMIT:%.3f>%.3f" % (duration_s, live_max_duration_s)],
            metrics=base_metrics,
        )
    uinput_probe = probe_uinput_environment() if use_uinput else None
    if use_uinput and uinput_probe and not uinput_probe.available:
        return waiting_report(
            run_id=run_id,
            started_at=started_at,
            summary="external follow live input prerequisites are not ready",
            reasons=["UINPUT_UNAVAILABLE"],
            metrics={**base_metrics, "uinput_probe": uinput_probe.as_dict()},
        )

    forbidden_processes_before = process_snapshot()
    try:
        resolved_region = resolve_capture_region(
            region=region,
            window_title=window_title,
            window_exact=window_exact,
            window_case_sensitive=window_case_sensitive,
            window_min_width=window_min_width,
            window_min_height=window_min_height,
        )
        source = make_region_source(resolved_region, fps=hz, backend=backend)
    except Exception as exc:
        status = WAITING if window_title and region is None else REJECT
        return ExternalFollowSessionReport(
            run_id=run_id,
            started_at=started_at,
            status=status,
            summary=(
                "external follow capture is waiting for a matching window"
                if status == WAITING else
                "external follow capture setup failed"
            ),
            reasons=["CAPTURE_SETUP:%s" % exc],
            metrics=base_metrics,
        )

    adapter = None
    log_path = make_log_path(log_dir, "%s-external-follow-loop" % run_id)
    try:
        adapter = UInputAdapter() if use_uinput else DryRunInputAdapter()
        tracker = ObjectTracker(tracker_type)
        config = LoopConfig(
            duration_s=duration_s,
            hz=hz,
            initial_bbox=tracker_bbox,
            enable_pitch=enable_pitch,
            log_dir=log_dir,
            run_id=run_id,
            tracker_type=tracker_type,
            frame_width=resolved_region["width"],
            frame_height=resolved_region["height"],
            desired_target_width=desired_target_width,
        )
        with JsonlLogger(log_path, metadata={
            "tool": "external_follow_session",
            "run_id": run_id,
            "source": "window:%s:%s" % (window_title, backend) if window_title else "region:%s" % backend,
            "adapter": adapter.__class__.__name__,
            "real_input": use_uinput,
        }) as logger:
            loop_summary = run_loop(source, tracker, adapter, config, logger=logger)
            logger.write("external_follow_session_summary", **loop_summary.as_dict())
    except UInputUnavailable as exc:
        return waiting_report(
            run_id=run_id,
            started_at=started_at,
            summary="external follow live input is unavailable",
            reasons=["UINPUT_UNAVAILABLE:%s" % exc],
            metrics={**base_metrics, "uinput_probe": uinput_probe.as_dict() if uinput_probe else None},
        )
    except Exception as exc:
        return ExternalFollowSessionReport(
            run_id=run_id,
            started_at=started_at,
            status=REJECT,
            summary="external follow session failed",
            reasons=["SESSION_ERROR:%s" % exc],
            evidence_paths={"loop_log": str(log_path)},
            metrics=base_metrics,
        )
    finally:
        if adapter is not None:
            adapter.close()

    loop_metrics = loop_summary.as_dict()
    gate_reasons = evaluate_session_metrics(
        loop_metrics,
        min_found_ratio=min_found_ratio,
        max_loss_events=max_loss_events,
        max_yaw_axis=max_yaw_axis,
        max_pitch_axis=max_pitch_axis,
        enable_pitch=enable_pitch,
    )
    isolation = combine_audits(
        audit_static_imports(Path(__file__).resolve().parent),
        audit_process_delta(forbidden_processes_before, process_snapshot()),
    )
    if isolation.status == FAIL:
        gate_reasons.extend(isolation.notes)
    status = (
        EXTERNAL_FOLLOW_LIVE_RUN_COMPLETE
        if use_uinput and not gate_reasons else
        EXTERNAL_FOLLOW_DRY_RUN_READY
        if not use_uinput and not gate_reasons else
        REJECT
    )
    summary = (
        "external follow live session completed within gates"
        if status == EXTERNAL_FOLLOW_LIVE_RUN_COMPLETE else
        "external follow dry-run session completed within gates"
        if status == EXTERNAL_FOLLOW_DRY_RUN_READY else
        "external follow session did not meet gates"
    )
    return ExternalFollowSessionReport(
        run_id=run_id,
        started_at=started_at,
        status=status,
        summary=summary,
        reasons=gate_reasons,
        evidence_paths={"loop_log": str(log_path)},
        metrics={
            **base_metrics,
            "resolved_region": resolved_region,
            "adapter": adapter.__class__.__name__ if adapter else None,
            "loop_summary": loop_metrics,
            "uinput_probe": uinput_probe.as_dict() if uinput_probe else None,
            "isolation_status": isolation.status,
            "isolation_audit": isolation.as_dict(),
        },
    )


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-id", default="external-follow-session")
    parser.add_argument("--log-dir", type=Path, default=DEFAULT_LOG_DIR)
    parser.add_argument("--capture-region", type=parse_region)
    parser.add_argument("--capture-window-title")
    parser.add_argument("--capture-backend", choices=["auto", "mss", "ffmpeg"], default="auto")
    parser.add_argument("--tracker-bbox", type=parse_bbox)
    parser.add_argument("--tracker-bbox-file", type=Path)
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
    parser.add_argument("--uinput", action="store_true")
    parser.add_argument("--ack-live-input", action="store_true")
    parser.add_argument("--live-max-duration", type=float, default=2.0)
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
    if args.live_max_duration <= 0:
        parser.error("--live-max-duration must be positive")
    if args.window_min_width <= 0 or args.window_min_height <= 0:
        parser.error("--window-min-width/--window-min-height must be positive")
    if args.tracker_bbox and args.tracker_bbox_file:
        parser.error("use --tracker-bbox or --tracker-bbox-file, not both")
    if args.tracker_bbox_file:
        try:
            args.tracker_bbox = load_tracker_bbox_file(args.tracker_bbox_file)
        except Exception as exc:
            parser.error(str(exc))
    return args


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    log_dir = resolve_log_dir(str(args.log_dir), repo_root=REPO_ROOT)
    report = run_external_follow_session(
        run_id=args.run_id,
        log_dir=log_dir,
        region=args.capture_region,
        window_title=args.capture_window_title,
        backend=args.capture_backend,
        tracker_bbox=args.tracker_bbox,
        duration_s=args.duration,
        hz=args.hz,
        tracker_type=args.tracker,
        enable_pitch=args.enable_pitch,
        desired_target_width=args.desired_target_width,
        min_found_ratio=args.min_found_ratio,
        max_loss_events=args.max_loss_events,
        max_yaw_axis=args.max_yaw_axis,
        max_pitch_axis=args.max_pitch_axis,
        use_uinput=args.uinput,
        ack_live_input=args.ack_live_input,
        live_max_duration_s=args.live_max_duration,
        window_exact=args.window_exact,
        window_case_sensitive=args.window_case_sensitive,
        window_min_width=args.window_min_width,
        window_min_height=args.window_min_height,
    )
    json_path, md_path = write_reports(report, log_dir)
    print("external-follow-session %s report=%s summary=%s" % (
        report.status,
        json_path,
        md_path,
    ))
    if report.status in (EXTERNAL_FOLLOW_DRY_RUN_READY, EXTERNAL_FOLLOW_LIVE_RUN_COMPLETE):
        return 0
    if report.status == WAITING:
        return 2
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
