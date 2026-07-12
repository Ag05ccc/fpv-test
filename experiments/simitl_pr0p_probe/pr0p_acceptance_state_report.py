#!/usr/bin/env python3
"""Read-only state report for the ordered pr0p acceptance gates.

This report does not launch pr0p, send OS input, edit config, or run any
acceptance command. It only reads the latest acceptance JSON reports and tells
which gate should be run next.
"""

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

from independent_sim_readiness import build_commands  # noqa: E402
from pr0p_capture_probe import DEFAULT_LOG_DIR  # noqa: E402
from pr0p_live_run_manifest import report_sort_key  # noqa: E402
from screen_tracking_loop import parse_bbox  # noqa: E402


PASS = "PASS"
WAITING = "WAITING"
FAIL = "FAIL"
MISSING = "MISSING"
STALE = "STALE"
UNKNOWN = "UNKNOWN"
DEFAULT_MAX_EVIDENCE_AGE_S = 24.0 * 60.0 * 60.0

STAGES = (
    ("pr0p_aux1_acceptance", "aux1-acceptance"),
    ("pr0p_response_acceptance", "response-acceptance"),
    ("pr0p_tracking_acceptance", "tracking-acceptance"),
)


@dataclass
class AcceptanceStateStage:
    name: str
    status: str
    summary: str
    report: str | None = None
    raw_status: str | None = None
    age_s: float | None = None
    max_age_s: float = DEFAULT_MAX_EVIDENCE_AGE_S
    step_statuses: dict[str, str] = field(default_factory=dict)
    notes: list[str] = field(default_factory=list)

    def as_dict(self) -> dict[str, Any]:
        return {
            "name": self.name,
            "status": self.status,
            "summary": self.summary,
            "report": self.report,
            "raw_status": self.raw_status,
            "age_s": self.age_s,
            "max_age_s": self.max_age_s,
            "step_statuses": self.step_statuses,
            "notes": self.notes,
        }


@dataclass
class AcceptanceStateReport:
    run_id: str
    started_at: str
    status: str
    summary: str
    next_action: str
    stages: list[AcceptanceStateStage] = field(default_factory=list)
    commands: dict[str, str] = field(default_factory=dict)
    metrics: dict[str, Any] = field(default_factory=dict)

    def as_dict(self) -> dict[str, Any]:
        return {
            "run_id": self.run_id,
            "started_at": self.started_at,
            "status": self.status,
            "summary": self.summary,
            "next_action": self.next_action,
            "stages": [stage.as_dict() for stage in self.stages],
            "commands": self.commands,
            "metrics": self.metrics,
        }


def latest_report(log_dir: Path, suffix: str) -> Path | None:
    reports = list(log_dir.glob("*-%s.json" % suffix))
    return max(reports, key=report_sort_key) if reports else None


def load_json(path: Path | None) -> dict[str, Any] | None:
    if path is None:
        return None
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return None
    return data if isinstance(data, dict) else None


def step_statuses(report: dict[str, Any] | None) -> dict[str, str]:
    if not report or not isinstance(report.get("steps"), list):
        return {}
    statuses: dict[str, str] = {}
    for step in report["steps"]:
        if not isinstance(step, dict):
            continue
        key = step.get("name") or step.get("phase")
        if key:
            statuses[str(key)] = str(step.get("status", UNKNOWN))
    return statuses


def evidence_age(path: Path | None, *, now_s: float) -> float | None:
    if path is None:
        return None
    try:
        return max(0.0, now_s - path.stat().st_mtime)
    except OSError:
        return None


def stage_from_latest(
    *,
    name: str,
    suffix: str,
    log_dir: Path,
    now_s: float,
    max_age_s: float,
) -> AcceptanceStateStage:
    path = latest_report(log_dir, suffix)
    if path is None:
        return AcceptanceStateStage(
            name=name,
            status=MISSING,
            summary="no %s report is available" % name,
            max_age_s=max_age_s,
            notes=["MISSING_REPORT:%s" % suffix],
        )
    data = load_json(path)
    if data is None:
        return AcceptanceStateStage(
            name=name,
            status=FAIL,
            summary="latest %s report could not be parsed" % name,
            report=str(path),
            max_age_s=max_age_s,
            notes=["REPORT_PARSE_FAIL"],
        )
    age_s = evidence_age(path, now_s=now_s)
    raw_status = str(data.get("status", UNKNOWN))
    if age_s is not None and age_s > max_age_s:
        return AcceptanceStateStage(
            name=name,
            status=STALE,
            summary="latest %s evidence is stale" % name,
            report=str(path),
            raw_status=raw_status,
            age_s=age_s,
            max_age_s=max_age_s,
            step_statuses=step_statuses(data),
            notes=["STALE_REPORT:%s" % suffix],
        )
    return AcceptanceStateStage(
        name=name,
        status=raw_status if raw_status in {PASS, WAITING, FAIL} else UNKNOWN,
        summary=str(data.get("summary") or "latest %s report loaded" % name),
        report=str(path),
        raw_status=raw_status,
        age_s=age_s,
        max_age_s=max_age_s,
        step_statuses=step_statuses(data),
    )


def summarize_state(
    stages: list[AcceptanceStateStage],
    *,
    bbox: tuple[int, int, int, int] | None,
) -> tuple[str, str, str]:
    if any(stage.status == FAIL for stage in stages):
        return FAIL, "one or more acceptance evidence reports failed", (
            "Inspect the failed acceptance report before running later gates."
        )
    stale = next((stage for stage in stages if stage.status == STALE), None)
    if stale is not None:
        return WAITING, "acceptance evidence is stale", (
            "Rerun %s because its latest evidence is stale." % stale.name
        )
    missing_or_waiting = next(
        (stage for stage in stages if stage.status in {MISSING, WAITING, UNKNOWN}),
        None,
    )
    if missing_or_waiting is None:
        return PASS, "AUX1, signed response, and tracking acceptance evidence are present", (
            "Inspect the chain-generated decision/readiness reports, or rerun the "
            "acceptance chain to refresh manifest, decision, and independent readiness."
        )
    if missing_or_waiting.name == "pr0p_aux1_acceptance":
        return WAITING, "acceptance state is waiting at AUX1/ARM", (
            "Bind AUX1/CH5 or apply the backup-backed patch, then run pr0p_aux1_acceptance_live."
        )
    if missing_or_waiting.name == "pr0p_response_acceptance":
        return WAITING, "acceptance state is waiting at signed response", (
            "Run pr0p_response_acceptance_live after AUX1/ARM acceptance is PASS."
        )
    if bbox is None:
        return WAITING, "acceptance state is waiting at tracking bbox/live gate", (
            "Select a bbox, prove P6 dry-run, then run pr0p_tracking_acceptance_live."
        )
    return WAITING, "acceptance state is waiting at live tracking", (
        "Run pr0p_tracking_acceptance_live after response acceptance is PASS."
    )


def build_acceptance_state(
    *,
    run_id: str,
    log_dir: Path,
    bbox: tuple[int, int, int, int] | None,
    max_evidence_age_s: float,
    now_s: float | None = None,
) -> AcceptanceStateReport:
    if now_s is None:
        now_s = time.time()
    stages = [
        stage_from_latest(
            name=name,
            suffix=suffix,
            log_dir=log_dir,
            now_s=now_s,
            max_age_s=max_evidence_age_s,
        )
        for name, suffix in STAGES
    ]
    status, summary, next_action = summarize_state(stages, bbox=bbox)
    return AcceptanceStateReport(
        run_id=run_id,
        started_at=time.strftime("%Y-%m-%dT%H:%M:%S%z", time.localtime()),
        status=status,
        summary=summary,
        next_action=next_action,
        stages=stages,
        commands=build_commands(run_id, bbox),
        metrics={
            "bbox": list(bbox) if bbox else None,
            "log_dir": str(log_dir),
            "max_evidence_age_s": max_evidence_age_s,
            "evidence_only": True,
            "real_input_sent": False,
            "stage_statuses": {stage.name: stage.status for stage in stages},
            "stage_reports": {stage.name: stage.report for stage in stages},
        },
    )


def build_markdown(report: AcceptanceStateReport) -> str:
    lines = [
        "# SimITL / pr0p Acceptance State",
        "",
        "Run: `%s`" % report.run_id,
        "Started: `%s`" % report.started_at,
        "Status: `%s`" % report.status,
        "",
        report.summary,
        "",
        "Next action: %s" % report.next_action,
        "",
        "## Evidence",
        "",
        "| Stage | Status | Raw | Age s | Report |",
        "| --- | --- | --- | --- | --- |",
    ]
    for stage in report.stages:
        age = "-" if stage.age_s is None else "%.3f" % stage.age_s
        lines.append("| %s | `%s` | `%s` | `%s` | `%s` |" % (
            stage.name,
            stage.status,
            stage.raw_status or "-",
            age,
            stage.report or "-",
        ))
    lines.extend(["", "## Commands", ""])
    for name, command in report.commands.items():
        if "acceptance" not in name and "aux1" not in name and name not in {
            "pr0p_live_manifest",
            "pr0p_decision",
        }:
            continue
        lines.extend(["### %s" % name, "", "```bash", command, "```", ""])
    lines.extend([
        "## Metrics",
        "",
        "```json",
        json.dumps(report.metrics, indent=2, sort_keys=True),
        "```",
        "",
    ])
    return "\n".join(lines)


def write_reports(report: AcceptanceStateReport, log_dir: Path) -> tuple[Path, Path]:
    log_dir.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime("%Y%m%d-%H%M%S", time.localtime())
    json_path = log_dir / ("%s-%s-acceptance-state.json" % (stamp, report.run_id))
    md_path = log_dir / ("%s-%s-acceptance-state.md" % (stamp, report.run_id))
    json_path.write_text(json.dumps(report.as_dict(), indent=2, sort_keys=True) + "\n",
                         encoding="utf-8")
    md_path.write_text(build_markdown(report), encoding="utf-8")
    return json_path, md_path


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-id", default="pr0p-acceptance-state")
    parser.add_argument("--log-dir", type=Path, default=DEFAULT_LOG_DIR)
    parser.add_argument("--bbox", type=parse_bbox)
    parser.add_argument("--max-evidence-age-s", type=float,
                        default=DEFAULT_MAX_EVIDENCE_AGE_S)
    args = parser.parse_args(argv)
    if args.max_evidence_age_s < 0:
        parser.error("--max-evidence-age-s must be non-negative")
    return args


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    report = build_acceptance_state(
        run_id=args.run_id,
        log_dir=args.log_dir,
        bbox=args.bbox,
        max_evidence_age_s=args.max_evidence_age_s,
    )
    json_path, md_path = write_reports(report, args.log_dir)
    print("simitl-pr0p-acceptance-state %s report=%s summary=%s" % (
        report.status,
        json_path,
        md_path,
    ))
    print(report.next_action)
    return 1 if report.status == FAIL else 0


if __name__ == "__main__":
    raise SystemExit(main())
