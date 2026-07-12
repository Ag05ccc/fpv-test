#!/usr/bin/env python3
"""One-command acceptance runner for the isolated game/screen sandbox."""

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

from decision_report import (  # noqa: E402
    REJECT,
    SIMPLE_SANDBOX_READY,
    WAITING,
    build_markdown as build_decision_markdown,
    choose_evidence,
    evaluate_decision,
)
from isolation_audit import (  # noqa: E402
    audit_process_delta,
    audit_static_imports,
    combine_audits,
    process_snapshot,
)
from phase_runner import run_synthetic_gates, write_reports as write_phase_reports  # noqa: E402
from simple_window_smoke import run_smoke as run_simple_window_smoke  # noqa: E402
from simple_window_smoke import write_smoke_report  # noqa: E402
from sitl_log import resolve_log_dir, timestamp_slug  # noqa: E402


PASS = "PASS"
FAIL = "FAIL"
DEFAULT_LOG_DIR = Path("logs/game_screen_sandbox")


@dataclass
class AcceptanceRunReport:
    run_id: str
    started_at: str
    status: str
    summary: str
    synthetic_phase_report_json: str | None = None
    synthetic_phase_report_md: str | None = None
    simple_window_smoke_report: str | None = None
    simple_window_phase_report_json: str | None = None
    decision_report_json: str | None = None
    decision_report_md: str | None = None
    notes: list[str] = field(default_factory=list)
    metrics: dict[str, Any] = field(default_factory=dict)

    def as_dict(self) -> dict[str, Any]:
        return {
            "run_id": self.run_id,
            "started_at": self.started_at,
            "status": self.status,
            "summary": self.summary,
            "synthetic_phase_report_json": self.synthetic_phase_report_json,
            "synthetic_phase_report_md": self.synthetic_phase_report_md,
            "simple_window_smoke_report": self.simple_window_smoke_report,
            "simple_window_phase_report_json": self.simple_window_phase_report_json,
            "decision_report_json": self.decision_report_json,
            "decision_report_md": self.decision_report_md,
            "notes": self.notes,
            "metrics": self.metrics,
        }


def build_acceptance_markdown(report: AcceptanceRunReport) -> str:
    lines = [
        "# Game Screen Sandbox Acceptance Run",
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
    for key in (
        "synthetic_phase_report_json",
        "synthetic_phase_report_md",
        "simple_window_smoke_report",
        "simple_window_phase_report_json",
        "decision_report_json",
        "decision_report_md",
    ):
        lines.append("| %s | `%s` |" % (key, getattr(report, key) or "none"))
    lines.extend([
        "",
        "## Notes",
        "",
    ])
    if report.notes:
        lines.extend("- %s" % note for note in report.notes)
    else:
        lines.append("- no additional notes")
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


def write_acceptance_reports(
    report: AcceptanceRunReport,
    log_dir: Path,
) -> tuple[Path, Path]:
    slug = "%s-%s" % (timestamp_slug(), report.run_id)
    json_path = log_dir / ("%s-acceptance-run.json" % slug)
    md_path = log_dir / ("%s-acceptance-run.md" % slug)
    json_path.write_text(json.dumps(report.as_dict(), indent=2, sort_keys=True) + "\n",
                         encoding="utf-8")
    md_path.write_text(build_acceptance_markdown(report), encoding="utf-8")
    return json_path, md_path


def write_decision_reports(decision, log_dir: Path, *, run_id: str) -> tuple[Path, Path]:
    slug = "%s-%s" % (timestamp_slug(), run_id)
    json_path = log_dir / ("%s-decision.json" % slug)
    md_path = log_dir / ("%s-decision.md" % slug)
    json_path.write_text(json.dumps(decision.as_dict(), indent=2, sort_keys=True) + "\n",
                         encoding="utf-8")
    md_path.write_text(build_decision_markdown(decision), encoding="utf-8")
    return json_path, md_path


def run_acceptance(
    *,
    run_id: str,
    log_dir: Path,
    duration_s: float,
    hz: float,
    min_fps: float,
    include_simple_window: bool,
    simple_window_timeout_s: float,
    simple_window_game_duration_s: float,
    simple_window_gate_duration_s: float,
) -> AcceptanceRunReport:
    log_dir.mkdir(parents=True, exist_ok=True)
    started_at = time.strftime("%Y-%m-%dT%H:%M:%S%z", time.localtime())
    forbidden_processes_before = process_snapshot()
    notes = [
        "sandbox-only: no Gazebo, Betaflight, pr0p, or physical RC process is launched",
    ]
    synthetic_duration_s = max(duration_s, 6.0)
    synthetic_hz = max(hz, 20.0)
    if synthetic_duration_s != duration_s or synthetic_hz != hz:
        notes.append(
            "synthetic objective gates normalized to %.1fs at %.1f Hz" % (
                synthetic_duration_s,
                synthetic_hz,
            )
        )

    synthetic_report = run_synthetic_gates(
        duration_s=synthetic_duration_s,
        hz=synthetic_hz,
        min_fps=min_fps,
        min_tracker_found_ratio=0.80,
        min_loop_found_ratio=0.90,
        log_dir=log_dir,
        run_id=run_id,
        include_moving_target=True,
        min_moving_target_motion_px=20.0,
        include_game_dynamics=True,
        include_range_dynamics=True,
        include_handoff_dynamics=True,
        include_adapter_dynamics=True,
        include_binding_dry=True,
        include_objective_loop=True,
    )
    synthetic_json, synthetic_md = write_phase_reports(synthetic_report, log_dir)

    simple_window_report_path: Path | None = None
    simple_window_phase_path: Path | None = None
    simple_window_report: dict[str, Any] | None = None
    if include_simple_window:
        simple_window_report = run_simple_window_smoke(
            run_id=run_id,
            log_dir=log_dir,
            width=640,
            height=480,
            game_duration_s=simple_window_game_duration_s,
            game_fps=20.0,
            window_timeout_s=simple_window_timeout_s,
            gate_duration_s=simple_window_gate_duration_s,
            gate_hz=10.0,
            min_fps=8.0,
        )
        simple_window_report_path = (
            log_dir / ("%s-%s-simple-window-smoke.json" % (timestamp_slug(), run_id))
        )
        write_smoke_report(simple_window_report_path, simple_window_report)
        raw_phase_path = simple_window_report.get("phase_report_json")
        if isinstance(raw_phase_path, str) and raw_phase_path:
            simple_window_phase_path = Path(raw_phase_path)
    else:
        notes.append("simple-window smoke skipped; same-run decision will remain WAITING")

    if include_simple_window:
        paths, reports = choose_evidence(
            log_dir=log_dir,
            synthetic_report=synthetic_json,
            simple_window_report=simple_window_report_path,
            simple_window_phase_report=simple_window_phase_path,
        )
    else:
        paths = {
            "synthetic_report": synthetic_json,
            "simple_window_report": None,
            "simple_window_phase_report": None,
        }
        reports = {
            "synthetic_report": synthetic_report.as_dict(),
            "simple_window_report": None,
            "simple_window_phase_report": None,
        }
    decision = evaluate_decision(reports, paths=paths, run_id=run_id)
    decision_json, decision_md = write_decision_reports(decision, log_dir, run_id=run_id)
    isolation_audit = combine_audits(
        audit_static_imports(Path(__file__).resolve().parent),
        audit_process_delta(forbidden_processes_before, process_snapshot()),
    )

    status = decision.decision
    if synthetic_report.status == FAIL:
        status = REJECT
    if isolation_audit.status == FAIL:
        status = REJECT
        notes.extend(isolation_audit.notes)
    summary = (
        "independent sandbox acceptance evidence is ready"
        if status == SIMPLE_SANDBOX_READY else
        "independent sandbox acceptance evidence is incomplete"
        if status == WAITING else
        "independent sandbox acceptance evidence failed"
    )
    return AcceptanceRunReport(
        run_id=run_id,
        started_at=started_at,
        status=status,
        summary=summary,
        synthetic_phase_report_json=str(synthetic_json),
        synthetic_phase_report_md=str(synthetic_md),
        simple_window_smoke_report=str(simple_window_report_path) if simple_window_report_path else None,
        simple_window_phase_report_json=str(simple_window_phase_path) if simple_window_phase_path else None,
        decision_report_json=str(decision_json),
        decision_report_md=str(decision_md),
        notes=notes,
        metrics={
            "synthetic_status": synthetic_report.status,
            "simple_window_status": simple_window_report.get("status") if simple_window_report else None,
            "decision": decision.decision,
            "decision_reasons": decision.reasons,
            "include_simple_window": include_simple_window,
            "requested_duration_s": duration_s,
            "requested_hz": hz,
            "synthetic_duration_s": synthetic_duration_s,
            "synthetic_hz": synthetic_hz,
            "isolation_status": isolation_audit.status,
            "isolation_audit": isolation_audit.as_dict(),
        },
    )


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-id", default="sandbox-acceptance")
    parser.add_argument("--log-dir", type=Path, default=DEFAULT_LOG_DIR)
    parser.add_argument("--duration", type=float, default=6.0)
    parser.add_argument("--hz", type=float, default=20.0)
    parser.add_argument("--min-fps", type=float, default=8.0)
    parser.add_argument("--include-simple-window", action="store_true",
                        help="Open the local OpenCV target window and run S1/S2/S7-real smoke")
    parser.add_argument("--simple-window-timeout", type=float, default=5.0)
    parser.add_argument("--simple-window-game-duration", type=float, default=8.0)
    parser.add_argument("--simple-window-gate-duration", type=float, default=2.0)
    args = parser.parse_args(argv)
    if args.duration <= 0:
        parser.error("--duration must be positive")
    if args.hz <= 0:
        parser.error("--hz must be positive")
    if args.min_fps < 0:
        parser.error("--min-fps must be non-negative")
    if args.simple_window_timeout <= 0:
        parser.error("--simple-window-timeout must be positive")
    if args.simple_window_game_duration <= 0:
        parser.error("--simple-window-game-duration must be positive")
    if args.simple_window_gate_duration <= 0:
        parser.error("--simple-window-gate-duration must be positive")
    if (
        args.include_simple_window
        and args.simple_window_game_duration
        <= args.simple_window_timeout + args.simple_window_gate_duration
    ):
        parser.error(
            "--simple-window-game-duration must exceed "
            "--simple-window-timeout + --simple-window-gate-duration"
        )
    return args


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    log_dir = resolve_log_dir(str(args.log_dir), repo_root=REPO_ROOT)
    report = run_acceptance(
        run_id=args.run_id,
        log_dir=log_dir,
        duration_s=args.duration,
        hz=args.hz,
        min_fps=args.min_fps,
        include_simple_window=args.include_simple_window,
        simple_window_timeout_s=args.simple_window_timeout,
        simple_window_game_duration_s=args.simple_window_game_duration,
        simple_window_gate_duration_s=args.simple_window_gate_duration,
    )
    json_path, md_path = write_acceptance_reports(report, log_dir)
    print("sandbox-acceptance %s report=%s summary=%s decision=%s" % (
        report.status,
        json_path,
        md_path,
        report.decision_report_md,
    ))
    if report.status == SIMPLE_SANDBOX_READY:
        return 0
    if report.status == WAITING:
        return 2
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
