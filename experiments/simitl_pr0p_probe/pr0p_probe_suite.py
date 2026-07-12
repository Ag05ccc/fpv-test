#!/usr/bin/env python3
"""Run the safe SimITL/pr0p probe gates as one report."""

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

from msp_ws_probe import probe_websocket, write_reports as write_ws_reports  # noqa: E402
from msp_ws_semantic_probe import (  # noqa: E402
    run_msp_ws_semantic_probe,
    write_reports as write_msp_ws_semantic_reports,
)
from msp_ws_status_probe import (  # noqa: E402
    run_msp_ws_status_probe,
    write_reports as write_msp_ws_status_reports,
)
from msp_ws_mode_ranges_probe import (  # noqa: E402
    run_msp_ws_mode_ranges_probe,
    write_reports as write_msp_ws_mode_ranges_reports,
)
from msp_ws_rc_probe import (  # noqa: E402
    run_msp_ws_rc_probe,
    write_reports as write_msp_ws_rc_reports,
)
from preflight_probe import (  # noqa: E402
    DEFAULT_INSTALL_ROOT,
    DEFAULT_WS_HOST,
    DEFAULT_WS_PORT,
    run_preflight,
    write_reports as write_preflight_reports,
)
from pr0p_install_probe import (  # noqa: E402
    run_install_probe,
    write_reports as write_install_reports,
)
from pr0p_client_probe import (  # noqa: E402
    run_client_probe,
    write_reports as write_client_reports,
)
from pr0p_isolation_check import (  # noqa: E402
    DEFAULT_ROOTS as ISOLATION_ROOTS,
    run_isolation_check,
    write_reports as write_isolation_reports,
)
from pr0p_input_config_probe import (  # noqa: E402
    run_input_config_probe,
    write_reports as write_input_config_reports,
)
from pr0p_input_config_patch import (  # noqa: E402
    run_input_config_patch,
    write_reports as write_input_config_patch_reports,
)
from pr0p_capture_probe import (  # noqa: E402
    DEFAULT_LOG_DIR,
    DEFAULT_WINDOW_TITLES,
    run_capture_probe,
    write_reports as write_capture_reports,
)
from pr0p_response_probe import (  # noqa: E402
    run_response_probe,
    write_reports as write_response_reports,
)
from pr0p_tracking_probe import (  # noqa: E402
    load_bbox_file,
    parse_bbox,
    run_tracking_probe,
    write_reports as write_tracking_reports,
)
from pr0p_tracking_log_check import (  # noqa: E402
    run_tracking_log_check,
    write_reports as write_tracking_log_check_reports,
)
from pr0p_virtual_input_probe import (  # noqa: E402
    command_from_args,
    run_input_probe,
    write_reports as write_input_reports,
)
from phase_runner import (  # noqa: E402
    run_synthetic_gates,
    write_reports as write_synthetic_phase_reports,
)


PASS = "PASS"
WAITING = "WAITING"
FAIL = "FAIL"


@dataclass
class SuiteStep:
    phase: str
    status: str
    summary: str
    report_json: str | None = None
    report_md: str | None = None
    metrics: dict[str, Any] = field(default_factory=dict)
    notes: list[str] = field(default_factory=list)

    def as_dict(self) -> dict[str, Any]:
        return {
            "phase": self.phase,
            "status": self.status,
            "summary": self.summary,
            "report_json": self.report_json,
            "report_md": self.report_md,
            "metrics": self.metrics,
            "notes": self.notes,
        }


@dataclass
class SuiteReport:
    run_id: str
    started_at: str
    steps: list[SuiteStep]

    @property
    def status(self) -> str:
        return suite_status([step.status for step in self.steps])

    def as_dict(self) -> dict[str, Any]:
        return {
            "run_id": self.run_id,
            "started_at": self.started_at,
            "status": self.status,
            "steps": [step.as_dict() for step in self.steps],
        }


def suite_status(statuses: list[str]) -> str:
    if any(status == FAIL for status in statuses):
        return FAIL
    if any(status == WAITING for status in statuses):
        return WAITING
    return PASS


def run_suite(
    *,
    run_id: str,
    log_dir: Path,
    install_root: Path,
    ws_host: str,
    ws_port: int,
    timeout: float,
    window_titles: list[str],
    capture_duration_s: float,
    capture_fps: float,
    capture_min_fps: float,
    response_axis: str,
    response_magnitude: float,
    response_image_axis: str,
    response_expected_sign: int,
    tracking_bbox: tuple[int, int, int, int] | None,
) -> SuiteReport:
    log_dir.mkdir(parents=True, exist_ok=True)
    steps: list[SuiteStep] = []

    isolation = run_isolation_check(roots=list(ISOLATION_ROOTS))
    isolation_json, isolation_md = write_isolation_reports(
        isolation,
        log_dir,
        run_id="%s-p0-isolation" % run_id,
    )
    steps.append(SuiteStep(
        phase="P0-isolation",
        status=isolation.status,
        summary=isolation.summary,
        report_json=str(isolation_json),
        report_md=str(isolation_md),
        metrics=isolation.metrics,
        notes=isolation.notes,
    ))

    preflight = run_preflight(
        install_root=install_root,
        ws_host=ws_host,
        ws_port=ws_port,
        timeout=timeout,
        run_id="%s-p0" % run_id,
    )
    pre_json, pre_md = write_preflight_reports(preflight, log_dir)
    steps.append(SuiteStep(
        phase="P0-preflight",
        status=preflight.status,
        summary="environment preflight completed",
        report_json=str(pre_json),
        report_md=str(pre_md),
        metrics={"checks": [check.as_dict() for check in preflight.checks]},
    ))

    install = run_install_probe(
        install_root=install_root,
        download_page="https://pr0p.dev/download",
        download=False,
        force=False,
        timeout=max(2.0, timeout),
        run_id="%s-p1" % run_id,
    )
    install_json, install_md = write_install_reports(install, log_dir, run_id="%s-p1" % run_id)
    steps.append(SuiteStep(
        phase="P1-install-discovery",
        status=install.status,
        summary=install.summary,
        report_json=str(install_json),
        report_md=str(install_md),
        metrics=install.metrics,
        notes=install.notes,
    ))

    client = run_client_probe(
        install_root=install_root,
        max_depth=3,
        run_id="%s-p1-client" % run_id,
    )
    client_json, client_md = write_client_reports(
        client,
        log_dir,
        run_id="%s-p1-client" % run_id,
    )
    steps.append(SuiteStep(
        phase="P1-client-executable",
        status=client.status,
        summary=client.summary,
        report_json=str(client_json),
        report_md=str(client_md),
        metrics=client.metrics,
        notes=client.notes,
    ))

    ws = probe_websocket(ws_host, ws_port, "/", timeout=timeout)
    ws_json, ws_md = write_ws_reports(ws, log_dir, run_id="%s-p2" % run_id)
    steps.append(SuiteStep(
        phase="P2-websocket",
        status=ws.status,
        summary=ws.summary,
        report_json=str(ws_json),
        report_md=str(ws_md),
        metrics=ws.metrics,
        notes=ws.notes,
    ))

    msp_ws = run_msp_ws_semantic_probe(
        host=ws_host,
        port=ws_port,
        path="/",
        timeout=timeout,
        max_frames=8,
    )
    msp_ws_json, msp_ws_md = write_msp_ws_semantic_reports(
        msp_ws,
        log_dir,
        run_id="%s-p2-msp" % run_id,
    )
    steps.append(SuiteStep(
        phase="P2-msp-readonly",
        status=msp_ws.status,
        summary=msp_ws.summary,
        report_json=str(msp_ws_json),
        report_md=str(msp_ws_md),
        metrics=msp_ws.metrics,
        notes=msp_ws.notes,
    ))

    fc_status = run_msp_ws_status_probe(
        host=ws_host,
        port=ws_port,
        path="/",
        timeout=timeout,
        max_frames=8,
    )
    fc_status_json, fc_status_md = write_msp_ws_status_reports(
        fc_status,
        log_dir,
        run_id="%s-p2-fc-status" % run_id,
    )
    steps.append(SuiteStep(
        phase="P2-fc-status-readonly",
        status=fc_status.status,
        summary=fc_status.summary,
        report_json=str(fc_status_json),
        report_md=str(fc_status_md),
        metrics=fc_status.metrics,
        notes=fc_status.notes,
    ))

    mode_ranges = run_msp_ws_mode_ranges_probe(
        host=ws_host,
        port=ws_port,
        path="/",
        timeout=timeout,
        max_frames=8,
    )
    mode_ranges_json, mode_ranges_md = write_msp_ws_mode_ranges_reports(
        mode_ranges,
        log_dir,
        run_id="%s-p2-mode-ranges" % run_id,
    )
    steps.append(SuiteStep(
        phase="P2-mode-ranges-readonly",
        status=mode_ranges.status,
        summary=mode_ranges.summary,
        report_json=str(mode_ranges_json),
        report_md=str(mode_ranges_md),
        metrics=mode_ranges.metrics,
        notes=mode_ranges.notes,
    ))

    rc_baseline = run_msp_ws_rc_probe(
        host=ws_host,
        port=ws_port,
        path="/",
        timeout=timeout,
        channels=[1450, 1620, 1120, 1380, 1000, 1000, 1000, 1000],
        write=False,
        settle_s=0.2,
        max_frames=8,
        baseline_samples=3,
        baseline_interval_s=0.1,
        max_baseline_delta=4,
        allow_unstable_baseline=False,
    )
    rc_baseline_json, rc_baseline_md = write_msp_ws_rc_reports(
        rc_baseline,
        log_dir,
        run_id="%s-p5-rc-baseline" % run_id,
    )
    steps.append(SuiteStep(
        phase="P5-rc-baseline",
        status=rc_baseline.status,
        summary=rc_baseline.summary,
        report_json=str(rc_baseline_json),
        report_md=str(rc_baseline_md),
        metrics=rc_baseline.metrics,
        notes=rc_baseline.notes,
    ))

    capture = run_capture_probe(
        run_id="%s-p3" % run_id,
        log_dir=log_dir,
        region=None,
        crop=None,
        window_titles=window_titles,
        exact=False,
        case_sensitive=False,
        min_width=160,
        min_height=120,
        preferred_size=None,
        allow_common_non_fpv_windows=False,
        backend="auto",
        fps=capture_fps,
        duration_s=capture_duration_s,
        min_fps=capture_min_fps,
        min_std=1.0,
    )
    capture_json, capture_md = write_capture_reports(capture, log_dir, run_id="%s-p3" % run_id)
    steps.append(SuiteStep(
        phase="P3-capture",
        status=capture.status,
        summary=capture.summary,
        report_json=str(capture_json),
        report_md=str(capture_md),
        metrics=capture.metrics,
        notes=capture.notes,
    ))

    input_result = run_input_probe(
        command=command_from_args(yaw=0.05, pitch=0.0, roll=0.0, throttle=0.0),
        hold_seconds=0.0,
        require_uinput=True,
        uinput_smoke=False,
    )
    input_json, input_md = write_input_reports(input_result, log_dir, run_id="%s-p4" % run_id)
    steps.append(SuiteStep(
        phase="P4-input-readiness",
        status=input_result.status,
        summary=input_result.summary,
        report_json=str(input_json),
        report_md=str(input_md),
        metrics=input_result.metrics,
        notes=input_result.notes,
    ))

    input_config = run_input_config_probe(
        input_config=Path.home() / ".config" / "unity3d" / "sigsegowl" / "pr0p" / "input.json",
        expected_device="Kenet Game Sandbox",
        require_virtual_mapping=True,
        allow_generic_uinput_profile=True,
    )
    input_config_json, input_config_md = write_input_config_reports(
        input_config,
        log_dir,
        run_id="%s-p4-config" % run_id,
    )
    steps.append(SuiteStep(
        phase="P4-input-mapping",
        status=input_config.status,
        summary=input_config.summary,
        report_json=str(input_config_json),
        report_md=str(input_config_md),
        metrics=input_config.metrics,
        notes=input_config.notes,
    ))

    input_config_patch = run_input_config_patch(
        input_config=Path.home() / ".config" / "unity3d" / "sigsegowl" / "pr0p" / "input.json",
        log_dir=log_dir,
        roles=["all"],
        expected_device="Kenet Game Sandbox",
        target_device="Joystick",
        write=False,
    )
    input_config_patch_json, input_config_patch_md = write_input_config_patch_reports(
        input_config_patch,
        log_dir,
        run_id="%s-p4-config-patch-dry" % run_id,
    )
    steps.append(SuiteStep(
        phase="P4-input-config-patch-dry",
        status=input_config_patch.status,
        summary=input_config_patch.summary,
        report_json=str(input_config_patch_json),
        report_md=str(input_config_patch_md),
        metrics=input_config_patch.metrics,
        notes=input_config_patch.notes,
    ))

    response = run_response_probe(
        run_id="%s-p5" % run_id,
        region=None,
        crop=None,
        window_titles=window_titles,
        exact=False,
        case_sensitive=False,
        min_width=160,
        min_height=120,
        preferred_size=None,
        allow_common_non_fpv_windows=False,
        backend="auto",
        fps=max(5.0, min(15.0, capture_fps)),
        axis=response_axis,
        magnitude=response_magnitude,
        image_axis=response_image_axis,
        expected_sign=response_expected_sign,
        pre_duration_s=0.2,
        post_duration_s=0.2,
        min_shift_px=2.0,
        max_shift_px=200.0,
        use_uinput=False,
    )
    response_json, response_md = write_response_reports(response, log_dir, run_id="%s-p5" % run_id)
    steps.append(SuiteStep(
        phase="P5-response-dry-run",
        status=response.status,
        summary=response.summary,
        report_json=str(response_json),
        report_md=str(response_md),
        metrics=response.metrics,
        notes=response.notes,
    ))

    synthetic_e2e = run_synthetic_gates(
        duration_s=1.0,
        hz=max(5.0, min(10.0, capture_fps)),
        min_fps=1.0,
        min_tracker_found_ratio=0.8,
        min_loop_found_ratio=0.8,
        log_dir=log_dir,
        run_id="%s-p6-synthetic-e2e" % run_id,
    )
    synthetic_json, synthetic_md = write_synthetic_phase_reports(synthetic_e2e, log_dir)
    steps.append(SuiteStep(
        phase="P6-synthetic-e2e-dry-run",
        status=synthetic_e2e.status,
        summary="synthetic capture/tracker/PID dry-run gates",
        report_json=str(synthetic_json),
        report_md=str(synthetic_md),
        metrics={
            "mode": synthetic_e2e.mode,
            "log_path": synthetic_e2e.log_path,
            "results": [result.as_dict() for result in synthetic_e2e.results],
        },
        notes=["sandbox-only; no pr0p/Gazebo window required"],
    ))

    synthetic_s4_log = None
    for result in synthetic_e2e.results:
        if result.phase == "S4":
            synthetic_s4_log = result.metrics.get("log_path")
            break
    synthetic_log_check = run_tracking_log_check(
        log_path=Path(synthetic_s4_log) if synthetic_s4_log else None,
        log_dir=log_dir,
        allow_latest=False,
        min_samples=3,
        min_found_ratio=0.8,
        max_loss_events=0,
        max_abs_yaw_axis=0.6,
        max_abs_pitch_axis=0.6,
        require_dry_run=True,
    )
    synthetic_log_json, synthetic_log_md = write_tracking_log_check_reports(
        synthetic_log_check,
        log_dir,
        run_id="%s-p6-synthetic-log" % run_id,
    )
    steps.append(SuiteStep(
        phase="P6-synthetic-log-check",
        status=synthetic_log_check.status,
        summary=synthetic_log_check.summary,
        report_json=str(synthetic_log_json),
        report_md=str(synthetic_log_md),
        metrics=synthetic_log_check.metrics,
        notes=synthetic_log_check.notes,
    ))

    tracking = run_tracking_probe(
        run_id="%s-p6" % run_id,
        log_dir=log_dir,
        region=None,
        crop=None,
        window_titles=window_titles,
        exact=False,
        case_sensitive=False,
        min_width=160,
        min_height=120,
        preferred_size=None,
        allow_common_non_fpv_windows=False,
        backend="auto",
        bbox=tracking_bbox,
        duration_s=0.5,
        hz=max(5.0, min(10.0, capture_fps)),
        tracker_type="CSRT",
        enable_pitch=False,
        use_uinput=False,
        min_found_ratio=0.85,
        max_loss_events=0,
        max_abs_yaw_axis=0.6,
        max_abs_pitch_axis=0.6,
        desired_target_width=120.0,
    )
    tracking_json, tracking_md = write_tracking_reports(
        tracking,
        log_dir,
        run_id="%s-p6" % run_id,
    )
    steps.append(SuiteStep(
        phase="P6-tracking-pid-dry-run",
        status=tracking.status,
        summary=tracking.summary,
        report_json=str(tracking_json),
        report_md=str(tracking_md),
        metrics=tracking.metrics,
        notes=tracking.notes,
    ))

    tracking_log_path = tracking.metrics.get("log_path")
    tracking_log = run_tracking_log_check(
        log_path=Path(tracking_log_path) if tracking_log_path else None,
        log_dir=log_dir,
        allow_latest=False,
        min_samples=3,
        min_found_ratio=0.85,
        max_loss_events=0,
        max_abs_yaw_axis=0.6,
        max_abs_pitch_axis=0.6,
        require_dry_run=True,
    )
    tracking_log_json, tracking_log_md = write_tracking_log_check_reports(
        tracking_log,
        log_dir,
        run_id="%s-p6-log" % run_id,
    )
    steps.append(SuiteStep(
        phase="P6-tracking-log-check",
        status=tracking_log.status,
        summary=tracking_log.summary,
        report_json=str(tracking_log_json),
        report_md=str(tracking_log_md),
        metrics=tracking_log.metrics,
        notes=tracking_log.notes,
    ))

    return SuiteReport(
        run_id=run_id,
        started_at=time.strftime("%Y-%m-%dT%H:%M:%S%z", time.localtime()),
        steps=steps,
    )


def build_markdown(report: SuiteReport) -> str:
    lines = [
        "# SimITL / pr0p Probe Suite",
        "",
        "Run: `%s`" % report.run_id,
        "Started: `%s`" % report.started_at,
        "Verdict: `%s`" % report.status,
        "",
        "| Phase | Status | Summary | Report |",
        "| --- | --- | --- | --- |",
    ]
    for step in report.steps:
        report_ref = step.report_md or step.report_json or "-"
        lines.append("| %s | %s | %s | `%s` |" % (
            step.phase,
            step.status,
            step.summary,
            report_ref,
        ))
    lines.extend([
        "",
        "## Notes",
        "",
        "- Safe suite defaults never send real OS input.",
        "- P5 and P6 remain dry-run only here; live control requires dedicated probes with --uinput --ack-live-input.",
        "- WAITING is expected until pr0p is running with a visible local race.",
        "",
        "## Raw Steps",
        "",
        "```json",
        json.dumps([step.as_dict() for step in report.steps], indent=2, sort_keys=True),
        "```",
        "",
    ])
    return "\n".join(lines)


def write_reports(report: SuiteReport, log_dir: Path) -> tuple[Path, Path]:
    log_dir.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime("%Y%m%d-%H%M%S", time.localtime())
    json_path = log_dir / ("%s-%s-suite.json" % (stamp, report.run_id))
    md_path = log_dir / ("%s-%s-suite.md" % (stamp, report.run_id))
    json_path.write_text(
        json.dumps(report.as_dict(), indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    md_path.write_text(build_markdown(report), encoding="utf-8")
    return json_path, md_path


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-id", default="pr0p-suite")
    parser.add_argument("--log-dir", type=Path, default=DEFAULT_LOG_DIR)
    parser.add_argument("--install-root", type=Path, default=DEFAULT_INSTALL_ROOT)
    parser.add_argument("--ws-host", default=DEFAULT_WS_HOST)
    parser.add_argument("--ws-port", type=int, default=DEFAULT_WS_PORT)
    parser.add_argument("--timeout", type=float, default=0.5)
    parser.add_argument("--window-title", action="append", dest="window_titles")
    parser.add_argument("--capture-duration", type=float, default=0.5)
    parser.add_argument("--capture-fps", type=float, default=5.0)
    parser.add_argument("--capture-min-fps", type=float, default=1.0)
    parser.add_argument("--response-axis", default="yaw")
    parser.add_argument("--response-magnitude", type=float, default=0.05)
    parser.add_argument("--response-image-axis", default="x")
    parser.add_argument("--response-expected-sign", type=int, choices=(-1, 0, 1), default=0)
    parser.add_argument("--tracking-bbox", type=parse_bbox)
    parser.add_argument("--tracking-bbox-file", type=Path)
    args = parser.parse_args(argv)
    if args.ws_port <= 0:
        parser.error("--ws-port must be positive")
    if args.timeout <= 0:
        parser.error("--timeout must be positive")
    if args.capture_duration <= 0:
        parser.error("--capture-duration must be positive")
    if args.capture_fps <= 0:
        parser.error("--capture-fps must be positive")
    if args.capture_min_fps < 0:
        parser.error("--capture-min-fps must be non-negative")
    if args.window_titles is None:
        args.window_titles = list(DEFAULT_WINDOW_TITLES)
    if args.tracking_bbox and args.tracking_bbox_file:
        parser.error("use either --tracking-bbox or --tracking-bbox-file, not both")
    if args.tracking_bbox_file:
        args.tracking_bbox = load_bbox_file(args.tracking_bbox_file)
    return args


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    report = run_suite(
        run_id=args.run_id,
        log_dir=args.log_dir,
        install_root=args.install_root,
        ws_host=args.ws_host,
        ws_port=args.ws_port,
        timeout=args.timeout,
        window_titles=args.window_titles,
        capture_duration_s=args.capture_duration,
        capture_fps=args.capture_fps,
        capture_min_fps=args.capture_min_fps,
        response_axis=args.response_axis,
        response_magnitude=args.response_magnitude,
        response_image_axis=args.response_image_axis,
        response_expected_sign=args.response_expected_sign,
        tracking_bbox=args.tracking_bbox,
    )
    json_path, md_path = write_reports(report, args.log_dir)
    print("simitl-pr0p-suite %s report=%s summary=%s" % (
        report.status,
        json_path,
        md_path,
    ))
    for step in report.steps:
        print("%s %s - %s" % (step.phase, step.status, step.summary))
    return 1 if report.status == FAIL else 0


if __name__ == "__main__":
    raise SystemExit(main())
