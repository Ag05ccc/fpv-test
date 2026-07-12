#!/usr/bin/env python3
"""Preflight a real game/sim window for tracker/PID sandbox work."""

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

from bbox_tool import prepare_bbox_artifacts  # noqa: E402
from capture_window import make_region_source, parse_region, resolve_capture_region  # noqa: E402
from isolation_audit import audit_static_imports  # noqa: E402
from phase_runner import (  # noqa: E402
    FAIL,
    PASS,
    WAITING,
    PhaseResult,
    check_s1_real_capture,
    check_s2_real_tracker,
    check_s5_real_latency,
    check_s7_real_combined,
    load_tracker_bbox_file,
)
from latency_probe import normalize_axes, parse_axis_shift_expectations  # noqa: E402
from screen_tracking_loop import parse_bbox  # noqa: E402
from sitl_log import resolve_log_dir, timestamp_slug  # noqa: E402
from virtual_input import AxisCommand  # noqa: E402


EXTERNAL_WINDOW_READY_DRY = "EXTERNAL_WINDOW_READY_DRY"
EXTERNAL_WINDOW_READY_LIVE_INPUT = "EXTERNAL_WINDOW_READY_LIVE_INPUT"
REJECT = "REJECT"
DEFAULT_LOG_DIR = Path("logs/game_screen_sandbox")


@dataclass
class ExternalWindowPreflightReport:
    run_id: str
    started_at: str
    status: str
    summary: str
    phases: list[PhaseResult] = field(default_factory=list)
    bbox_artifact: dict[str, Any] | None = None
    evidence_paths: dict[str, str | None] = field(default_factory=dict)
    reasons: list[str] = field(default_factory=list)
    metrics: dict[str, Any] = field(default_factory=dict)

    def as_dict(self) -> dict[str, Any]:
        return {
            "run_id": self.run_id,
            "started_at": self.started_at,
            "status": self.status,
            "summary": self.summary,
            "phases": [phase.as_dict() for phase in self.phases],
            "bbox_artifact": self.bbox_artifact,
            "evidence_paths": self.evidence_paths,
            "reasons": self.reasons,
            "metrics": self.metrics,
        }


def phase_statuses(phases: list[PhaseResult]) -> dict[str, str]:
    return {phase.phase: phase.status for phase in phases}


def decide_preflight(
    phases: list[PhaseResult],
    *,
    bbox_artifact: dict[str, Any] | None,
    require_axis_response: bool = False,
) -> tuple[str, str, list[str]]:
    reasons: list[str] = []
    if bbox_artifact and bbox_artifact.get("status") == WAITING:
        reasons.append("BBOX_SELECTION_REQUIRED")
    if bbox_artifact and bbox_artifact.get("status") == FAIL:
        reasons.append("BBOX_INVALID")
    statuses = phase_statuses(phases)
    for phase in ("S1-real", "S2-real", "S7-real"):
        status = statuses.get(phase)
        if status is None:
            if not bbox_artifact or bbox_artifact.get("status") != WAITING:
                reasons.append("PHASE_MISSING:%s" % phase)
        elif status != PASS:
            reasons.append("PHASE_NOT_PASS:%s:%s" % (phase, status))
    if require_axis_response:
        status = statuses.get("S5-real")
        if status is None:
            reasons.append("PHASE_MISSING:S5-real")
        elif status != PASS:
            reasons.append("PHASE_NOT_PASS:S5-real:%s" % status)
    if any(reason.startswith("PHASE_NOT_PASS") and reason.endswith("FAIL") for reason in reasons):
        return REJECT, "external window preflight failed", reasons
    if "BBOX_INVALID" in reasons:
        return REJECT, "external window bbox is invalid", reasons
    if reasons:
        return WAITING, "external window preflight is waiting for required evidence", reasons
    if require_axis_response:
        return (
            EXTERNAL_WINDOW_READY_LIVE_INPUT,
            "external window is ready for tracker/PID dry-run and live axis-response checks",
            reasons,
        )
    return EXTERNAL_WINDOW_READY_DRY, "external window is ready for tracker/PID dry-run development", reasons


def build_markdown(report: ExternalWindowPreflightReport) -> str:
    lines = [
        "# External Window Preflight",
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
        "## Phase Statuses",
        "",
        "| Phase | Status | Summary |",
        "| --- | --- | --- |",
    ])
    for phase in report.phases:
        lines.append("| %s | %s | %s |" % (phase.phase, phase.status, phase.summary))
    lines.extend([
        "",
        "## Reasons",
        "",
    ])
    if report.reasons:
        lines.extend("- %s" % reason for reason in report.reasons)
    else:
        lines.append("- all required external-window dry-run gates passed")
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
    report: ExternalWindowPreflightReport,
    log_dir: Path,
) -> tuple[Path, Path]:
    slug = "%s-%s" % (timestamp_slug(), report.run_id)
    json_path = log_dir / ("%s-external-window-preflight.json" % slug)
    md_path = log_dir / ("%s-external-window-preflight.md" % slug)
    json_path.write_text(json.dumps(report.as_dict(), indent=2, sort_keys=True) + "\n",
                         encoding="utf-8")
    md_path.write_text(build_markdown(report), encoding="utf-8")
    return json_path, md_path


def prepare_bbox_if_needed(
    *,
    region: dict[str, int] | None,
    window_title: str | None,
    backend: str,
    bbox: tuple[int, int, int, int] | None,
    frame_path: Path,
    overlay_path: Path | None,
    fps: float,
    window_exact: bool,
    window_case_sensitive: bool,
    window_min_width: int,
    window_min_height: int,
) -> tuple[dict[str, Any] | None, dict[str, int] | None]:
    if bbox is not None:
        return None, None
    resolved_region = resolve_capture_region(
        region=region,
        window_title=window_title,
        window_exact=window_exact,
        window_case_sensitive=window_case_sensitive,
        window_min_width=window_min_width,
        window_min_height=window_min_height,
    )
    source = make_region_source(resolved_region, fps=fps, backend=backend)
    check = prepare_bbox_artifacts(
        source,
        bbox=None,
        frame_path=frame_path,
        overlay_path=overlay_path,
        duration_s=1.0,
    )
    return check.as_dict(), resolved_region


def run_preflight(
    *,
    run_id: str,
    log_dir: Path,
    region: dict[str, int] | None,
    window_title: str | None,
    backend: str,
    bbox: tuple[int, int, int, int] | None,
    frame_path: Path | None,
    overlay_path: Path | None,
    duration_s: float,
    hz: float,
    min_fps: float,
    min_found_ratio: float,
    desired_target_width: float,
    max_yaw_axis: float,
    max_pitch_axis: float,
    include_axis_sweep: bool,
    ack_live_input: bool,
    axis_names: tuple[str, ...],
    axis_value: float,
    window_exact: bool,
    window_case_sensitive: bool,
    window_min_width: int,
    window_min_height: int,
    axis_expected_shifts: dict[str, tuple[str, int]] | None = None,
    axis_min_shift_px: float = 0.5,
) -> ExternalWindowPreflightReport:
    log_dir.mkdir(parents=True, exist_ok=True)
    started_at = time.strftime("%Y-%m-%dT%H:%M:%S%z", time.localtime())
    phases: list[PhaseResult] = []
    bbox_artifact: dict[str, Any] | None = None
    resolved_bbox_frame = frame_path or log_dir / ("%s-%s-target-frame.png" % (
        timestamp_slug(),
        run_id,
    ))

    if region is None and not window_title:
        bbox_artifact = {
            "status": WAITING,
            "notes": ["provide --capture-region or --capture-window-title"],
        }
    else:
        try:
            bbox_artifact, _resolved = prepare_bbox_if_needed(
                region=region,
                window_title=window_title,
                backend=backend,
                bbox=bbox,
                frame_path=resolved_bbox_frame,
                overlay_path=overlay_path,
                fps=hz,
                window_exact=window_exact,
                window_case_sensitive=window_case_sensitive,
                window_min_width=window_min_width,
                window_min_height=window_min_height,
            )
        except Exception as exc:
            bbox_artifact = {
                "status": WAITING if window_title and region is None else FAIL,
                "notes": [str(exc)],
            }

    if bbox is not None:
        phases.append(check_s1_real_capture(
            region,
            backend=backend,
            duration_s=duration_s,
            hz=hz,
            min_fps=min_fps,
            window_title=window_title,
            window_exact=window_exact,
            window_case_sensitive=window_case_sensitive,
            window_min_width=window_min_width,
            window_min_height=window_min_height,
        ))
        phases.append(check_s2_real_tracker(
            region,
            backend=backend,
            duration_s=duration_s,
            hz=hz,
            min_found_ratio=min_found_ratio,
            tracker_bbox=bbox,
            window_title=window_title,
            window_exact=window_exact,
            window_case_sensitive=window_case_sensitive,
            window_min_width=window_min_width,
            window_min_height=window_min_height,
        ))
        phases.append(check_s7_real_combined(
            region,
            backend=backend,
            duration_s=duration_s,
            hz=hz,
            min_found_ratio=min_found_ratio,
            tracker_bbox=bbox,
            log_dir=log_dir,
            run_id=run_id,
            desired_target_width=desired_target_width,
            max_yaw_axis=max_yaw_axis,
            max_pitch_axis=max_pitch_axis,
            window_title=window_title,
            window_exact=window_exact,
            window_case_sensitive=window_case_sensitive,
            window_min_width=window_min_width,
            window_min_height=window_min_height,
        ))
        if include_axis_sweep:
            phases.append(check_s5_real_latency(
                region,
                backend=backend,
                fps=hz,
                pre_duration_s=0.5,
                post_duration_s=1.0,
                command=AxisCommand(yaw=axis_value),
                min_diff=3.0,
                baseline_multiplier=3.0,
                use_uinput=True,
                ack_live_input=ack_live_input,
                axis_sweep=True,
                axis_names=axis_names,
                axis_value=axis_value,
                axis_expected_shifts=axis_expected_shifts,
                axis_min_shift_px=axis_min_shift_px,
                window_title=window_title,
                window_exact=window_exact,
                window_case_sensitive=window_case_sensitive,
                window_min_width=window_min_width,
                window_min_height=window_min_height,
            ))

    status, summary, reasons = decide_preflight(
        phases,
        bbox_artifact=bbox_artifact,
        require_axis_response=include_axis_sweep,
    )
    isolation = audit_static_imports(Path(__file__).resolve().parent)
    if isolation.status == FAIL:
        status = REJECT
        reasons.extend(isolation.notes)
    return ExternalWindowPreflightReport(
        run_id=run_id,
        started_at=started_at,
        status=status,
        summary=summary,
        phases=phases,
        bbox_artifact=bbox_artifact,
        reasons=reasons,
        evidence_paths={
            "bbox_frame": (
                bbox_artifact.get("frame_path")
                if bbox_artifact and bbox_artifact.get("frame_path") else None
            ),
            "bbox_overlay": (
                bbox_artifact.get("overlay_path")
                if bbox_artifact and bbox_artifact.get("overlay_path") else None
            ),
            "s7_log": next(
                (
                    phase.metrics.get("log_path")
                    for phase in phases
                    if phase.phase == "S7-real"
                ),
                None,
            ),
        },
        metrics={
            "window_title": window_title,
            "region": region,
            "backend": backend,
            "tracker_bbox": list(bbox) if bbox else None,
            "phase_statuses": phase_statuses(phases),
            "include_axis_sweep": include_axis_sweep,
            "axis_names": list(axis_names) if include_axis_sweep else None,
            "axis_value": axis_value if include_axis_sweep else None,
            "axis_expected_shifts": (
                {
                    axis: "%s%s" % ("+" if sign > 0 else "-", component)
                    for axis, (component, sign) in (axis_expected_shifts or {}).items()
                } if include_axis_sweep and axis_expected_shifts else None
            ),
            "axis_min_shift_px": (
                axis_min_shift_px if include_axis_sweep and axis_expected_shifts else None
            ),
            "ack_live_input": ack_live_input,
            "isolation_status": isolation.status,
            "isolation_audit": isolation.as_dict(),
            "scope": "external game/sim window dry-run only; no Gazebo/Betaflight promotion",
        },
    )


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-id", default="external-window-preflight")
    parser.add_argument("--log-dir", type=Path, default=DEFAULT_LOG_DIR)
    parser.add_argument("--capture-region", type=parse_region)
    parser.add_argument("--capture-window-title")
    parser.add_argument("--capture-backend", choices=["auto", "mss", "ffmpeg"], default="auto")
    parser.add_argument("--tracker-bbox", type=parse_bbox)
    parser.add_argument("--tracker-bbox-file", type=Path)
    parser.add_argument("--frame-path", type=Path)
    parser.add_argument("--overlay-path", type=Path)
    parser.add_argument("--duration", type=float, default=2.0)
    parser.add_argument("--hz", type=float, default=10.0)
    parser.add_argument("--min-fps", type=float, default=8.0)
    parser.add_argument("--min-found-ratio", type=float, default=0.85)
    parser.add_argument("--desired-target-width", type=float, default=120.0)
    parser.add_argument("--max-yaw-axis", type=float, default=0.5)
    parser.add_argument("--max-pitch-axis", type=float, default=0.5)
    parser.add_argument("--include-axis-sweep", action="store_true",
                        help="Add S5-real live uinput axis-response probe")
    parser.add_argument("--ack-live-input", action="store_true",
                        help="Required before --include-axis-sweep sends uinput")
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
    if not 0 <= args.min_found_ratio <= 1:
        parser.error("--min-found-ratio must be in [0, 1]")
    if args.desired_target_width <= 0:
        parser.error("--desired-target-width must be positive")
    if not 0 < args.max_yaw_axis <= 1:
        parser.error("--max-yaw-axis must be in (0, 1]")
    if not 0 < args.max_pitch_axis <= 1:
        parser.error("--max-pitch-axis must be in (0, 1]")
    if not 0 < abs(args.axis_sweep_value) <= 1:
        parser.error("--axis-sweep-value magnitude must be in (0, 1]")
    try:
        args.axis_sweep_axes = normalize_axes(args.axis_sweep_axes)
    except ValueError as exc:
        parser.error(str(exc))
    if args.axis_expected_shifts and not args.include_axis_sweep:
        parser.error("--axis-expected-shifts requires --include-axis-sweep")
    try:
        args.axis_expected_shifts = parse_axis_shift_expectations(args.axis_expected_shifts)
    except ValueError as exc:
        parser.error(str(exc))
    if args.axis_min_shift_px < 0:
        parser.error("--axis-min-shift-px must be non-negative")
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
    report = run_preflight(
        run_id=args.run_id,
        log_dir=log_dir,
        region=args.capture_region,
        window_title=args.capture_window_title,
        backend=args.capture_backend,
        bbox=args.tracker_bbox,
        frame_path=args.frame_path,
        overlay_path=args.overlay_path,
        duration_s=args.duration,
        hz=args.hz,
        min_fps=args.min_fps,
        min_found_ratio=args.min_found_ratio,
        desired_target_width=args.desired_target_width,
        max_yaw_axis=args.max_yaw_axis,
        max_pitch_axis=args.max_pitch_axis,
        include_axis_sweep=args.include_axis_sweep,
        ack_live_input=args.ack_live_input,
        axis_names=args.axis_sweep_axes,
        axis_value=args.axis_sweep_value,
        axis_expected_shifts=args.axis_expected_shifts,
        axis_min_shift_px=args.axis_min_shift_px,
        window_exact=args.window_exact,
        window_case_sensitive=args.window_case_sensitive,
        window_min_width=args.window_min_width,
        window_min_height=args.window_min_height,
    )
    json_path, md_path = write_reports(report, log_dir)
    print("external-window-preflight %s report=%s summary=%s" % (
        report.status,
        json_path,
        md_path,
    ))
    if report.status in (EXTERNAL_WINDOW_READY_DRY, EXTERNAL_WINDOW_READY_LIVE_INPUT):
        return 0
    if report.status == WAITING:
        return 2
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
