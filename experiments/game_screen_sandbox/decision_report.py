#!/usr/bin/env python3
"""Evidence-based readiness decision for the isolated game/screen sandbox."""

from __future__ import annotations

import argparse
import json
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any


PASS = "PASS"
WAITING = "WAITING"
FAIL = "FAIL"
UNKNOWN = "UNKNOWN"
SIMPLE_SANDBOX_READY = "SIMPLE_SANDBOX_READY"
REJECT = "REJECT"
DEFAULT_LOG_DIR = Path("logs/game_screen_sandbox")
SYNTHETIC_REQUIRED_PHASES = (
    "S0", "S1", "S2", "S3", "S4", "S8", "S9-game", "S10-range", "S11-handoff",
    "S12-adapter", "S13-binding-dry", "S14-objective",
)
SIMPLE_WINDOW_REQUIRED_PHASES = ("S1-real", "S2-real", "S7-real")


@dataclass
class SandboxDecisionReport:
    run_id: str
    started_at: str
    decision: str
    summary: str
    evidence_paths: dict[str, str | None]
    reasons: list[str] = field(default_factory=list)
    metrics: dict[str, Any] = field(default_factory=dict)

    def as_dict(self) -> dict[str, Any]:
        return {
            "run_id": self.run_id,
            "started_at": self.started_at,
            "decision": self.decision,
            "summary": self.summary,
            "evidence_paths": self.evidence_paths,
            "reasons": self.reasons,
            "metrics": self.metrics,
        }


def report_sort_key(path: Path) -> tuple[int, str]:
    try:
        mtime_ns = path.stat().st_mtime_ns
    except OSError:
        mtime_ns = -1
    return mtime_ns, path.name


def load_json_report(path: Path | None) -> dict[str, Any] | None:
    if path is None:
        return None
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return None
    return data if isinstance(data, dict) else None


def phase_statuses(report: dict[str, Any] | None) -> dict[str, str]:
    if not report:
        return {}
    results = report.get("results", [])
    if not isinstance(results, list):
        return {}
    return {
        str(result.get("phase")): str(result.get("status", UNKNOWN))
        for result in results
        if isinstance(result, dict) and result.get("phase")
    }


def contains_phase(path: Path, phase: str) -> bool:
    return phase in phase_statuses(load_json_report(path))


def latest_phase_report_with_phase(log_dir: Path, phase: str) -> Path | None:
    reports = [
        path
        for path in log_dir.glob("*-phase-report.json")
        if contains_phase(path, phase)
    ]
    return max(reports, key=report_sort_key) if reports else None


def latest_report(log_dir: Path, pattern: str) -> Path | None:
    reports = list(log_dir.glob(pattern))
    return max(reports, key=report_sort_key) if reports else None


def resolve_simple_window_phase_report(
    *,
    log_dir: Path,
    simple_window_report: dict[str, Any] | None,
    explicit_path: Path | None,
) -> Path | None:
    if explicit_path is not None:
        return explicit_path
    if simple_window_report:
        raw = simple_window_report.get("phase_report_json")
        if isinstance(raw, str) and raw:
            path = Path(raw)
            if path.exists():
                return path
    return latest_phase_report_with_phase(log_dir, "S7-real")


def choose_evidence(
    *,
    log_dir: Path,
    synthetic_report: Path | None,
    simple_window_report: Path | None,
    simple_window_phase_report: Path | None,
) -> tuple[dict[str, Path | None], dict[str, dict[str, Any] | None]]:
    synthetic_path = synthetic_report or latest_phase_report_with_phase(log_dir, "S8")
    simple_window_path = simple_window_report or latest_report(log_dir, "*-simple-window-smoke.json")
    simple_window_data = load_json_report(simple_window_path)
    simple_phase_path = resolve_simple_window_phase_report(
        log_dir=log_dir,
        simple_window_report=simple_window_data,
        explicit_path=simple_window_phase_report,
    )
    paths = {
        "synthetic_report": synthetic_path,
        "simple_window_report": simple_window_path,
        "simple_window_phase_report": simple_phase_path,
    }
    reports = {
        "synthetic_report": load_json_report(synthetic_path),
        "simple_window_report": simple_window_data,
        "simple_window_phase_report": load_json_report(simple_phase_path),
    }
    return paths, reports


def missing_phase_reasons(
    statuses: dict[str, str],
    required: tuple[str, ...],
    *,
    prefix: str,
) -> list[str]:
    reasons = []
    for phase in required:
        status = statuses.get(phase)
        if status is None:
            reasons.append("%s_MISSING:%s" % (prefix, phase))
        elif status != PASS:
            reasons.append("%s_NOT_PASS:%s:%s" % (prefix, phase, status))
    return reasons


def any_failed_status(*status_maps: dict[str, str]) -> list[str]:
    failed = []
    for statuses in status_maps:
        failed.extend(phase for phase, status in statuses.items() if status == FAIL)
    return sorted(set(failed))


def evaluate_decision(
    reports: dict[str, dict[str, Any] | None],
    *,
    paths: dict[str, Path | None],
    run_id: str,
) -> SandboxDecisionReport:
    synthetic = reports.get("synthetic_report")
    simple_window = reports.get("simple_window_report")
    simple_phase = reports.get("simple_window_phase_report")
    synthetic_statuses = phase_statuses(synthetic)
    simple_phase_statuses = phase_statuses(simple_phase)
    reasons: list[str] = []
    decision = WAITING
    summary = "waiting for independent sandbox evidence"

    if synthetic is None:
        reasons.append("MISSING_SYNTHETIC_MOVING_TARGET_REPORT")
    else:
        reasons.extend(missing_phase_reasons(
            synthetic_statuses,
            SYNTHETIC_REQUIRED_PHASES,
            prefix="SYNTHETIC",
        ))

    if simple_window is None:
        reasons.append("MISSING_SIMPLE_WINDOW_SMOKE")
    else:
        simple_window_status = str(simple_window.get("status", UNKNOWN))
        if simple_window_status != PASS:
            reasons.append("SIMPLE_WINDOW_SMOKE_NOT_PASS:%s" % simple_window_status)

    if simple_phase is None:
        reasons.append("MISSING_SIMPLE_WINDOW_PHASE_REPORT")
    else:
        reasons.extend(missing_phase_reasons(
            simple_phase_statuses,
            SIMPLE_WINDOW_REQUIRED_PHASES,
            prefix="SIMPLE_WINDOW",
        ))

    failed_phases = any_failed_status(synthetic_statuses, simple_phase_statuses)
    if simple_window and str(simple_window.get("status", UNKNOWN)) == FAIL:
        failed_phases.append("simple-window-smoke")
    if failed_phases:
        decision = REJECT
        summary = "one or more independent sandbox evidence gates failed"
        reasons.extend("FAIL:%s" % phase for phase in sorted(set(failed_phases)))
    elif not reasons:
        decision = SIMPLE_SANDBOX_READY
        summary = "independent game/screen sandbox is ready for tracker/PID dry-run development"
    else:
        summary = "independent game/screen sandbox is not fully proven yet"

    return SandboxDecisionReport(
        run_id=run_id,
        started_at=time.strftime("%Y-%m-%dT%H:%M:%S%z", time.localtime()),
        decision=decision,
        summary=summary,
        evidence_paths={
            key: str(path) if path else None
            for key, path in paths.items()
        },
        reasons=reasons,
        metrics={
            "synthetic_required_phases": list(SYNTHETIC_REQUIRED_PHASES),
            "simple_window_required_phases": list(SIMPLE_WINDOW_REQUIRED_PHASES),
            "synthetic_statuses": synthetic_statuses,
            "simple_window_status": (
                simple_window.get("status") if simple_window else None
            ),
            "simple_window_phase_statuses": simple_phase_statuses,
            "scope": "isolated game/screen sandbox only; not pr0p/Betaflight promotion",
        },
    )


def build_markdown(report: SandboxDecisionReport) -> str:
    lines = [
        "# Game Screen Sandbox Decision Report",
        "",
        "Run: `%s`" % report.run_id,
        "Started: `%s`" % report.started_at,
        "Decision: `%s`" % report.decision,
        "",
        report.summary,
        "",
        "## Scope",
        "",
        "- This decision covers only the isolated game/screen sandbox.",
        "- It does not promote pr0p, SimITL, Betaflight, Gazebo, or physical RC readiness.",
        "",
        "## Evidence",
        "",
        "| Evidence | Path |",
        "| --- | --- |",
    ]
    for key, path in report.evidence_paths.items():
        lines.append("| %s | `%s` |" % (key, path or "none"))
    lines.extend([
        "",
        "## Reasons",
        "",
    ])
    if report.reasons:
        for reason in report.reasons:
            lines.append("- %s" % reason)
    else:
        lines.append("- all independent sandbox readiness gates are present")
    lines.extend([
        "",
        "## Phase Statuses",
        "",
        "| Group | Phase | Status |",
        "| --- | --- | --- |",
    ])
    for phase, status in sorted(report.metrics.get("synthetic_statuses", {}).items()):
        lines.append("| synthetic | %s | %s |" % (phase, status))
    for phase, status in sorted(report.metrics.get("simple_window_phase_statuses", {}).items()):
        lines.append("| simple-window | %s | %s |" % (phase, status))
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


def write_reports(report: SandboxDecisionReport, log_dir: Path) -> tuple[Path, Path]:
    log_dir.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime("%Y%m%d-%H%M%S", time.localtime())
    json_path = log_dir / ("%s-%s-decision.json" % (stamp, report.run_id))
    md_path = log_dir / ("%s-%s-decision.md" % (stamp, report.run_id))
    json_path.write_text(json.dumps(report.as_dict(), indent=2, sort_keys=True) + "\n",
                         encoding="utf-8")
    md_path.write_text(build_markdown(report), encoding="utf-8")
    return json_path, md_path


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-id", default="sandbox-decision")
    parser.add_argument("--log-dir", type=Path, default=DEFAULT_LOG_DIR)
    parser.add_argument("--synthetic-report", type=Path)
    parser.add_argument("--simple-window-report", type=Path)
    parser.add_argument("--simple-window-phase-report", type=Path)
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    paths, reports = choose_evidence(
        log_dir=args.log_dir,
        synthetic_report=args.synthetic_report,
        simple_window_report=args.simple_window_report,
        simple_window_phase_report=args.simple_window_phase_report,
    )
    report = evaluate_decision(reports, paths=paths, run_id=args.run_id)
    json_path, md_path = write_reports(report, args.log_dir)
    print("game-screen-sandbox-decision %s report=%s summary=%s" % (
        report.decision,
        json_path,
        md_path,
    ))
    print(report.summary)
    return 1 if report.decision == REJECT else 0


if __name__ == "__main__":
    raise SystemExit(main())
