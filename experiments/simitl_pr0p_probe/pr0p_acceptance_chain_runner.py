#!/usr/bin/env python3
"""Plan or run the ordered pr0p acceptance chain.

The chain is deliberately conservative:

1. RC/manual flight must pass before response acceptance can run.
2. Response acceptance must pass before tracking acceptance can run.
3. Live mode requires explicit launch/UI/input acknowledgement flags.

By default this is a plan-only report and sends no real OS input.
"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable

PROBE_DIR = Path(__file__).resolve().parent
if str(PROBE_DIR) not in sys.path:
    sys.path.insert(0, str(PROBE_DIR))

from pr0p_capture_probe import DEFAULT_LOG_DIR, DEFAULT_WINDOW_TITLES  # noqa: E402
from pr0p_decision_report import (  # noqa: E402
    evaluate_decision,
    write_reports as write_decision_reports,
)
from preflight_probe import DEFAULT_INSTALL_ROOT  # noqa: E402
from independent_sim_readiness import (  # noqa: E402
    GAME_SCREEN_LOG_DIR,
    build_readiness,
    write_reports as write_readiness_reports,
)
from pr0p_input_config_probe import (  # noqa: E402
    DEFAULT_EXPECTED_DEVICE,
    DEFAULT_INPUT_CONFIG,
)
from pr0p_live_run_manifest import (  # noqa: E402
    build_manifest,
    write_reports as write_manifest_reports,
)
from pr0p_rc_manual_flight_runner import (  # noqa: E402
    run_manual_flight_gate,
    write_reports as write_manual_flight_reports,
)
from pr0p_response_acceptance_runner import (  # noqa: E402
    run_response_acceptance,
    write_reports as write_response_reports,
)
from pr0p_tracking_acceptance_runner import (  # noqa: E402
    run_tracking_acceptance,
    write_reports as write_tracking_reports,
)
from screen_tracking_loop import parse_bbox  # noqa: E402


PASS = "PASS"
WAITING = "WAITING"
FAIL = "FAIL"
SKIPPED = "SKIPPED"


@dataclass
class ChainStageResult:
    name: str
    status: str
    summary: str
    report: str | None = None
    notes: list[str] = field(default_factory=list)

    def as_dict(self) -> dict[str, Any]:
        return {
            "name": self.name,
            "status": self.status,
            "summary": self.summary,
            "report": self.report,
            "notes": self.notes,
        }


@dataclass
class AcceptanceChainReport:
    run_id: str
    started_at: str
    status: str
    summary: str
    next_action: str
    stages: list[ChainStageResult] = field(default_factory=list)
    metrics: dict[str, Any] = field(default_factory=dict)

    def as_dict(self) -> dict[str, Any]:
        return {
            "run_id": self.run_id,
            "started_at": self.started_at,
            "status": self.status,
            "summary": self.summary,
            "next_action": self.next_action,
            "stages": [stage.as_dict() for stage in self.stages],
            "metrics": self.metrics,
        }


def stage_from_report(name: str, report: Any, json_path: Path | None) -> ChainStageResult:
    return ChainStageResult(
        name=name,
        status=str(report.status),
        summary=str(report.summary),
        report=str(json_path) if json_path else None,
    )


def skipped_stage(name: str, *, summary: str, notes: list[str]) -> ChainStageResult:
    return ChainStageResult(
        name=name,
        status=SKIPPED,
        summary=summary,
        notes=notes,
    )


def refresh_manifest(
    *,
    run_id: str,
    log_dir: Path,
    bbox: tuple[int, int, int, int] | None,
    window_title: str,
    yaw_expected_sign: int,
    pitch_expected_sign: int,
) -> ChainStageResult:
    manifest = build_manifest(
        run_id=run_id,
        log_dir=log_dir,
        suite_report=None,
        bbox=bbox,
        window_title=window_title,
        yaw_expected_sign=yaw_expected_sign,
        pitch_expected_sign=pitch_expected_sign,
    )
    json_path, _md_path = write_manifest_reports(manifest, log_dir)
    return ChainStageResult(
        name="pr0p_live_manifest_refresh",
        status=manifest.status,
        summary=manifest.next_action,
        report=str(json_path),
    )


def write_decision_stage(
    *,
    run_id: str,
    log_dir: Path,
    manifest_path: str | None,
) -> ChainStageResult:
    if manifest_path is None:
        return ChainStageResult(
            name="pr0p_decision_refresh",
            status=WAITING,
            summary="skipped because live manifest refresh did not produce a report",
        )
    path = Path(manifest_path)
    try:
        manifest_report = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        return ChainStageResult(
            name="pr0p_decision_refresh",
            status=FAIL,
            summary="live manifest report could not be read",
            report=str(path),
            notes=[str(exc)],
        )
    decision = evaluate_decision(
        manifest_report,
        evidence_path=path,
    )
    json_path, _md_path = write_decision_reports(
        decision,
        log_dir,
        run_id="%s-decision" % run_id,
    )
    return ChainStageResult(
        name="pr0p_decision_refresh",
        status=decision.decision,
        summary=decision.summary,
        report=str(json_path),
        notes=decision.reasons,
    )


def write_readiness_stage(
    *,
    run_id: str,
    log_dir: Path,
    game_log_dir: Path,
    bbox: tuple[int, int, int, int] | None,
    window_title: str,
    yaw_expected_sign: int,
    pitch_expected_sign: int,
) -> ChainStageResult:
    readiness = build_readiness(
        run_id="%s-readiness" % run_id,
        game_log_dir=game_log_dir,
        pr0p_log_dir=log_dir,
        bbox=bbox,
        window_title=window_title,
        yaw_expected_sign=yaw_expected_sign,
        pitch_expected_sign=pitch_expected_sign,
    )
    json_path, _md_path = write_readiness_reports(readiness, log_dir)
    return ChainStageResult(
        name="independent_sim_readiness_refresh",
        status=readiness.status,
        summary=readiness.summary,
        report=str(json_path),
        notes=readiness.reasons,
    )


def summarize_chain(stages: list[ChainStageResult]) -> tuple[str, str, str]:
    status_by_name = {stage.name: stage.status for stage in stages}
    if any(stage.status == FAIL for stage in stages):
        return FAIL, "one or more ordered pr0p acceptance stages failed", (
            "Inspect the failed stage report before running later gates."
        )
    manual_status = status_by_name.get("rc_manual_flight")
    response_status = status_by_name.get("pr0p_response_acceptance")
    tracking_status = status_by_name.get("pr0p_tracking_acceptance")
    if manual_status == PASS and response_status == PASS and tracking_status == PASS:
        decision_status = status_by_name.get("pr0p_decision_refresh")
        readiness_status = status_by_name.get("independent_sim_readiness_refresh")
        if decision_status or readiness_status:
            return PASS, "RC/manual flight, signed response, and live tracking acceptance passed", (
                "Inspect the generated decision/readiness reports, then run "
                "pr0p_core_goal_report.py or pr0p_core_next_runner.py for the "
                "measured post-core gates."
            )
        return PASS, "RC/manual flight, signed response, and live tracking acceptance passed", (
            "Generate the live manifest and decision report, then continue with "
            "pr0p_core_goal_report.py or pr0p_core_next_runner.py."
        )
    if manual_status != PASS:
        return WAITING, "acceptance chain is waiting at RC/manual flight", (
            "Run pr0p_rc_manual_flight_runner.py until AUX1/ARM and bounded "
            "yaw/pitch/roll response evidence pass."
        )
    if response_status != PASS:
        return WAITING, "acceptance chain is waiting at signed response", (
            "Run response acceptance until signed yaw and pitch gates pass."
        )
    return WAITING, "acceptance chain is waiting at live tracking", (
        "Run tracking acceptance after bbox, P6 dry-run, and response evidence are PASS."
    )


def run_acceptance_chain(
    *,
    run_id: str,
    log_dir: Path,
    game_log_dir: Path,
    install_root: Path,
    input_config: Path,
    expected_device: str,
    allow_physical_mapping: bool = False,
    bbox: tuple[int, int, int, int] | None,
    execute_dry_patch: bool,
    apply_config_patch: bool,
    run_live_gates: bool,
    execute_tracking_dry_run: bool,
    window_title: str,
    yaw_expected_sign: int,
    pitch_expected_sign: int,
    manual_response_magnitude: float,
    arm_first: bool = False,
    arm_throttle: float = -0.4,
    measure: str = "visual",
    attitude_min_delta_deg: float = 10.0,
    yaw_magnitude: float,
    pitch_magnitude: float,
    min_shift_px: float,
    max_shift_px: float,
    startup_wait_s: float,
    duration_s: float,
    hz: float,
    tracker: str,
    enable_pitch: bool,
    min_found_ratio: float,
    max_loss_events: int,
    max_abs_yaw_axis: float,
    max_abs_pitch_axis: float,
    desired_target_width: float,
    timeout_s: float,
    command_runner: Callable[..., subprocess.CompletedProcess[str]] = subprocess.run,
) -> AcceptanceChainReport:
    stages: list[ChainStageResult] = []

    manual_report = run_manual_flight_gate(
        run_id="%s-rc-manual" % run_id,
        log_dir=log_dir,
        install_root=install_root,
        input_config=input_config,
        expected_device=expected_device,
        allow_physical_mapping=allow_physical_mapping,
        bbox=bbox,
        execute_dry_patch=execute_dry_patch,
        apply_config_patch=apply_config_patch,
        run_live_gates=run_live_gates,
        manual_response_magnitude=manual_response_magnitude,
        manual_response_min_shift_px=min_shift_px,
        manual_response_max_shift_px=max_shift_px,
        manual_response_startup_wait_s=startup_wait_s,
        timeout_s=timeout_s,
        arm_first=arm_first,
        arm_throttle=arm_throttle,
        measure=measure,
        attitude_min_delta_deg=attitude_min_delta_deg,
        command_runner=command_runner,
    )
    manual_json, _manual_md = write_manual_flight_reports(manual_report, log_dir)
    stages.append(stage_from_report("rc_manual_flight", manual_report, manual_json))

    if manual_report.status != PASS:
        stages.append(skipped_stage(
            "pr0p_live_manifest_after_rc_manual_flight",
            summary="skipped because RC/manual flight is not PASS",
            notes=["later gates are not evaluated until arm and basic stick response are proven"],
        ))
        stages.append(skipped_stage(
            "pr0p_response_acceptance",
            summary="skipped because RC/manual flight is not PASS",
            notes=["signed response requires RC/manual flight acceptance first"],
        ))
        stages.append(skipped_stage(
            "pr0p_tracking_acceptance",
            summary="skipped because response acceptance is not PASS",
            notes=["live tracking requires signed yaw and pitch response first"],
        ))
        status, summary, next_action = summarize_chain(stages)
        return AcceptanceChainReport(
            run_id=run_id,
            started_at=time.strftime("%Y-%m-%dT%H:%M:%S%z", time.localtime()),
            status=status,
            summary=summary,
            next_action=next_action,
            stages=stages,
            metrics={
                "bbox": list(bbox) if bbox else None,
                "log_dir": str(log_dir),
                "game_log_dir": str(game_log_dir),
                "install_root": str(install_root),
                "input_config": str(input_config),
                "expected_device": expected_device,
                "allow_physical_mapping": allow_physical_mapping,
                "run_live_gates": run_live_gates,
                "execute_dry_patch": execute_dry_patch,
                "apply_config_patch": apply_config_patch,
                "execute_tracking_dry_run": execute_tracking_dry_run,
                "manual_response_magnitude": manual_response_magnitude,
                "arm_first": arm_first,
                "arm_throttle": arm_throttle,
                "measure": measure,
                "attitude_min_delta_deg": attitude_min_delta_deg,
            },
        )

    stages.append(refresh_manifest(
        run_id="%s-after-rc-manual-flight" % run_id,
        log_dir=log_dir,
        bbox=bbox,
        window_title=window_title,
        yaw_expected_sign=yaw_expected_sign,
        pitch_expected_sign=pitch_expected_sign,
    ))

    response_report = run_response_acceptance(
        run_id="%s-response" % run_id,
        log_dir=log_dir,
        run_live_gates=run_live_gates,
        yaw_expected_sign=yaw_expected_sign,
        pitch_expected_sign=pitch_expected_sign,
        yaw_magnitude=yaw_magnitude,
        pitch_magnitude=pitch_magnitude,
        min_shift_px=min_shift_px,
        max_shift_px=max_shift_px,
        startup_wait_s=startup_wait_s,
        timeout_s=timeout_s,
        arm_first=arm_first,
        arm_throttle=arm_throttle,
        measure=measure,
        attitude_min_delta_deg=attitude_min_delta_deg,
        command_runner=command_runner,
    )
    response_json, _response_md = write_response_reports(response_report, log_dir)
    stages.append(stage_from_report("pr0p_response_acceptance", response_report, response_json))

    if response_report.status != PASS:
        stages.append(skipped_stage(
            "pr0p_live_manifest_after_response",
            summary="skipped because response acceptance is not PASS",
            notes=["tracking prerequisites are not refreshed until response is proven"],
        ))
        stages.append(skipped_stage(
            "pr0p_tracking_acceptance",
            summary="skipped because response acceptance is not PASS",
            notes=["live tracking requires signed yaw and pitch response first"],
        ))
        status, summary, next_action = summarize_chain(stages)
        return AcceptanceChainReport(
            run_id=run_id,
            started_at=time.strftime("%Y-%m-%dT%H:%M:%S%z", time.localtime()),
            status=status,
            summary=summary,
            next_action=next_action,
            stages=stages,
            metrics={
                "bbox": list(bbox) if bbox else None,
                "log_dir": str(log_dir),
                "game_log_dir": str(game_log_dir),
                "install_root": str(install_root),
                "input_config": str(input_config),
                "expected_device": expected_device,
                "allow_physical_mapping": allow_physical_mapping,
                "run_live_gates": run_live_gates,
                "execute_dry_patch": execute_dry_patch,
                "apply_config_patch": apply_config_patch,
                "execute_tracking_dry_run": execute_tracking_dry_run,
                "manual_response_magnitude": manual_response_magnitude,
                "arm_first": arm_first,
                "arm_throttle": arm_throttle,
                "measure": measure,
                "attitude_min_delta_deg": attitude_min_delta_deg,
            },
        )

    stages.append(refresh_manifest(
        run_id="%s-after-response" % run_id,
        log_dir=log_dir,
        bbox=bbox,
        window_title=window_title,
        yaw_expected_sign=yaw_expected_sign,
        pitch_expected_sign=pitch_expected_sign,
    ))

    tracking_report = run_tracking_acceptance(
        run_id="%s-tracking" % run_id,
        log_dir=log_dir,
        bbox=bbox,
        execute_dry_run=execute_tracking_dry_run,
        run_live_gates=run_live_gates,
        window_title=window_title,
        duration_s=duration_s,
        hz=hz,
        tracker=tracker,
        enable_pitch=enable_pitch,
        min_found_ratio=min_found_ratio,
        max_loss_events=max_loss_events,
        max_abs_yaw_axis=max_abs_yaw_axis,
        max_abs_pitch_axis=max_abs_pitch_axis,
        desired_target_width=desired_target_width,
        timeout_s=timeout_s,
        arm_first=arm_first,
        arm_throttle=arm_throttle,
        command_runner=command_runner,
    )
    tracking_json, _tracking_md = write_tracking_reports(tracking_report, log_dir)
    stages.append(stage_from_report("pr0p_tracking_acceptance", tracking_report, tracking_json))

    if tracking_report.status == PASS:
        final_manifest_stage = refresh_manifest(
            run_id="%s-after-tracking" % run_id,
            log_dir=log_dir,
            bbox=bbox,
            window_title=window_title,
            yaw_expected_sign=yaw_expected_sign,
            pitch_expected_sign=pitch_expected_sign,
        )
        stages.append(final_manifest_stage)
        stages.append(write_decision_stage(
            run_id=run_id,
            log_dir=log_dir,
            manifest_path=final_manifest_stage.report,
        ))
        stages.append(write_readiness_stage(
            run_id=run_id,
            log_dir=log_dir,
            game_log_dir=game_log_dir,
            bbox=bbox,
            window_title=window_title,
            yaw_expected_sign=yaw_expected_sign,
            pitch_expected_sign=pitch_expected_sign,
        ))

    status, summary, next_action = summarize_chain(stages)
    return AcceptanceChainReport(
        run_id=run_id,
        started_at=time.strftime("%Y-%m-%dT%H:%M:%S%z", time.localtime()),
        status=status,
        summary=summary,
        next_action=next_action,
        stages=stages,
        metrics={
            "bbox": list(bbox) if bbox else None,
            "log_dir": str(log_dir),
            "game_log_dir": str(game_log_dir),
            "install_root": str(install_root),
            "input_config": str(input_config),
            "expected_device": expected_device,
            "allow_physical_mapping": allow_physical_mapping,
            "run_live_gates": run_live_gates,
            "execute_dry_patch": execute_dry_patch,
            "apply_config_patch": apply_config_patch,
            "execute_tracking_dry_run": execute_tracking_dry_run,
            "window_title": window_title,
            "yaw_expected_sign": yaw_expected_sign,
            "pitch_expected_sign": pitch_expected_sign,
            "manual_response_magnitude": manual_response_magnitude,
            "arm_first": arm_first,
            "arm_throttle": arm_throttle,
            "measure": measure,
            "attitude_min_delta_deg": attitude_min_delta_deg,
            "yaw_magnitude": yaw_magnitude,
            "pitch_magnitude": pitch_magnitude,
            "min_shift_px": min_shift_px,
            "max_shift_px": max_shift_px,
            "startup_wait_s": startup_wait_s,
            "duration_s": duration_s,
            "hz": hz,
            "tracker": tracker,
            "enable_pitch": enable_pitch,
            "min_found_ratio": min_found_ratio,
            "max_loss_events": max_loss_events,
            "max_abs_yaw_axis": max_abs_yaw_axis,
            "max_abs_pitch_axis": max_abs_pitch_axis,
            "desired_target_width": desired_target_width,
        },
    )


def build_markdown(report: AcceptanceChainReport) -> str:
    lines = [
        "# SimITL / pr0p Acceptance Chain",
        "",
        "Run: `%s`" % report.run_id,
        "Started: `%s`" % report.started_at,
        "Verdict: `%s`" % report.status,
        "",
        report.summary,
        "",
        "Next action: %s" % report.next_action,
        "",
        "| Stage | Status | Report |",
        "| --- | --- | --- |",
    ]
    for stage in report.stages:
        lines.append("| %s | `%s` | `%s` |" % (
            stage.name,
            stage.status,
            stage.report or "-",
        ))
    lines.extend(["", "## Stages", ""])
    for stage in report.stages:
        lines.extend(["### %s" % stage.name, "", stage.summary, ""])
        for note in stage.notes:
            lines.append("- %s" % note)
        if stage.notes:
            lines.append("")
    lines.extend([
        "## Metrics",
        "",
        "```json",
        json.dumps(report.metrics, indent=2, sort_keys=True),
        "```",
        "",
    ])
    return "\n".join(lines)


def write_reports(report: AcceptanceChainReport, log_dir: Path) -> tuple[Path, Path]:
    log_dir.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime("%Y%m%d-%H%M%S", time.localtime())
    json_path = log_dir / ("%s-%s-acceptance-chain.json" % (stamp, report.run_id))
    md_path = log_dir / ("%s-%s-acceptance-chain.md" % (stamp, report.run_id))
    json_path.write_text(json.dumps(report.as_dict(), indent=2, sort_keys=True) + "\n",
                         encoding="utf-8")
    md_path.write_text(build_markdown(report), encoding="utf-8")
    return json_path, md_path


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-id", default="pr0p-acceptance-chain")
    parser.add_argument("--log-dir", type=Path, default=DEFAULT_LOG_DIR)
    parser.add_argument("--game-log-dir", type=Path, default=GAME_SCREEN_LOG_DIR)
    parser.add_argument("--install-root", type=Path, default=DEFAULT_INSTALL_ROOT)
    parser.add_argument("--input-config", type=Path, default=DEFAULT_INPUT_CONFIG)
    parser.add_argument("--expected-device", default=DEFAULT_EXPECTED_DEVICE)
    parser.add_argument("--allow-physical-mapping", action="store_true",
                        help="Accept primary RC axes mapped to a physical transmitter")
    parser.add_argument("--bbox", type=parse_bbox)
    parser.add_argument("--execute-dry-patch", action="store_true")
    parser.add_argument("--apply-config-patch", action="store_true")
    parser.add_argument("--ack-config-write", action="store_true")
    parser.add_argument("--run-live-gates", action="store_true")
    parser.add_argument("--ack-live-launch", action="store_true")
    parser.add_argument("--ack-live-ui", action="store_true")
    parser.add_argument("--ack-live-input", action="store_true")
    parser.add_argument("--execute-tracking-dry-run", action="store_true")
    parser.add_argument("--window-title", default=DEFAULT_WINDOW_TITLES[0])
    parser.add_argument("--yaw-expected-sign", type=int, choices=(-1, 0, 1), default=1)
    parser.add_argument("--pitch-expected-sign", type=int, choices=(-1, 0, 1), default=-1)
    parser.add_argument("--manual-response-magnitude", type=float, default=0.05)
    parser.add_argument("--arm-first", action="store_true",
                        help="Arm the FC and hold hover throttle during response/tracking gates")
    parser.add_argument("--arm-throttle", type=float, default=-0.4)
    parser.add_argument("--measure", default="visual", choices=("visual", "attitude"),
                        help="Response measurement mode forwarded to manual/response gates")
    parser.add_argument("--attitude-min-delta", type=float, default=10.0)
    parser.add_argument("--yaw-magnitude", type=float, default=0.03)
    parser.add_argument("--pitch-magnitude", type=float, default=0.03)
    parser.add_argument("--min-shift-px", type=float, default=2.0)
    parser.add_argument("--max-shift-px", type=float, default=200.0)
    parser.add_argument("--startup-wait", type=float, default=8.0)
    parser.add_argument("--duration", type=float, default=5.0)
    parser.add_argument("--hz", type=float, default=10.0)
    parser.add_argument("--tracker", choices=("CSRT", "KCF"), default="CSRT")
    parser.add_argument("--enable-pitch", action="store_true")
    parser.add_argument("--min-found-ratio", type=float, default=0.85)
    parser.add_argument("--max-loss-events", type=int, default=0)
    parser.add_argument("--max-abs-yaw-axis", type=float, default=0.6)
    parser.add_argument("--max-abs-pitch-axis", type=float, default=0.6)
    parser.add_argument("--desired-target-width", type=float, default=120.0)
    parser.add_argument("--timeout", type=float, default=240.0)
    args = parser.parse_args(argv)
    if args.apply_config_patch and not args.ack_config_write:
        parser.error("--apply-config-patch requires --ack-config-write")
    if args.run_live_gates and not (
        args.ack_live_launch and args.ack_live_ui and args.ack_live_input
    ):
        parser.error("--run-live-gates requires --ack-live-launch --ack-live-ui --ack-live-input")
    if args.manual_response_magnitude <= 0:
        parser.error("--manual-response-magnitude must be positive")
    if not -1.0 <= args.arm_throttle <= 1.0:
        parser.error("--arm-throttle must be within [-1.0, 1.0]")
    if args.attitude_min_delta <= 0:
        parser.error("--attitude-min-delta must be positive")
    if args.yaw_magnitude <= 0 or args.pitch_magnitude <= 0:
        parser.error("--yaw-magnitude/--pitch-magnitude must be positive")
    if args.min_shift_px < 0:
        parser.error("--min-shift-px must be non-negative")
    if args.max_shift_px <= 0:
        parser.error("--max-shift-px must be positive")
    if args.min_shift_px > args.max_shift_px:
        parser.error("--min-shift-px must be <= --max-shift-px")
    if args.startup_wait < 0:
        parser.error("--startup-wait must be non-negative")
    if args.duration <= 0:
        parser.error("--duration must be positive")
    if args.hz <= 0:
        parser.error("--hz must be positive")
    if not 0 <= args.min_found_ratio <= 1:
        parser.error("--min-found-ratio must be between 0 and 1")
    if args.max_loss_events < 0:
        parser.error("--max-loss-events must be non-negative")
    if args.max_abs_yaw_axis < 0 or args.max_abs_pitch_axis < 0:
        parser.error("--max-abs-* limits must be non-negative")
    if args.desired_target_width <= 0:
        parser.error("--desired-target-width must be positive")
    if args.timeout <= 0:
        parser.error("--timeout must be positive")
    return args


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    report = run_acceptance_chain(
        run_id=args.run_id,
        log_dir=args.log_dir,
        game_log_dir=args.game_log_dir,
        install_root=args.install_root,
        input_config=args.input_config,
        expected_device=args.expected_device,
        allow_physical_mapping=args.allow_physical_mapping,
        bbox=args.bbox,
        execute_dry_patch=args.execute_dry_patch,
        apply_config_patch=args.apply_config_patch,
        run_live_gates=args.run_live_gates,
        execute_tracking_dry_run=args.execute_tracking_dry_run,
        window_title=args.window_title,
        yaw_expected_sign=args.yaw_expected_sign,
        pitch_expected_sign=args.pitch_expected_sign,
        manual_response_magnitude=args.manual_response_magnitude,
        arm_first=args.arm_first,
        arm_throttle=args.arm_throttle,
        measure=args.measure,
        attitude_min_delta_deg=args.attitude_min_delta,
        yaw_magnitude=args.yaw_magnitude,
        pitch_magnitude=args.pitch_magnitude,
        min_shift_px=args.min_shift_px,
        max_shift_px=args.max_shift_px,
        startup_wait_s=args.startup_wait,
        duration_s=args.duration,
        hz=args.hz,
        tracker=args.tracker,
        enable_pitch=args.enable_pitch,
        min_found_ratio=args.min_found_ratio,
        max_loss_events=args.max_loss_events,
        max_abs_yaw_axis=args.max_abs_yaw_axis,
        max_abs_pitch_axis=args.max_abs_pitch_axis,
        desired_target_width=args.desired_target_width,
        timeout_s=args.timeout,
    )
    json_path, md_path = write_reports(report, args.log_dir)
    print("simitl-pr0p-acceptance-chain %s report=%s summary=%s" % (
        report.status,
        json_path,
        md_path,
    ))
    print(report.next_action)
    return 1 if report.status == FAIL else 0


if __name__ == "__main__":
    raise SystemExit(main())
