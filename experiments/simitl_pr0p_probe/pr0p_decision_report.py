#!/usr/bin/env python3
"""Evidence-based promote/wait/reject report for the SimITL/pr0p spike."""

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

from pr0p_capture_probe import DEFAULT_LOG_DIR  # noqa: E402
from pr0p_live_run_manifest import latest_suite_report, load_suite_report, report_sort_key  # noqa: E402


PROMOTE_CANDIDATE = "PROMOTE_CANDIDATE"
WAITING = "WAITING"
REJECT = "REJECT"
UNKNOWN = "UNKNOWN"
PASS = "PASS"
FAIL = "FAIL"
STALE = "STALE"
DEFAULT_MAX_EVIDENCE_AGE_S = 24.0 * 60.0 * 60.0


@dataclass
class DecisionReport:
    run_id: str
    started_at: str
    decision: str
    summary: str
    evidence_path: str | None
    reasons: list[str] = field(default_factory=list)
    metrics: dict[str, Any] = field(default_factory=dict)

    def as_dict(self) -> dict[str, Any]:
        return {
            "run_id": self.run_id,
            "started_at": self.started_at,
            "decision": self.decision,
            "summary": self.summary,
            "evidence_path": self.evidence_path,
            "reasons": self.reasons,
            "metrics": self.metrics,
        }


def latest_manifest_report(log_dir: Path) -> Path | None:
    reports = list(log_dir.glob("*-live-manifest.json"))
    return max(reports, key=report_sort_key) if reports else None


def evidence_source_kind(report: dict[str, Any] | None, path: Path | None) -> str:
    if isinstance(report, dict) and (
        "suite_report" in report
        or {"next_action", "metadata"}.issubset(report.keys())
    ):
        return "live_manifest"
    if path is not None:
        if path.name.endswith("-live-manifest.json"):
            return "live_manifest"
        if path.name.endswith("-suite.json"):
            return "suite"
    return UNKNOWN


def load_json_report(path: Path | None) -> dict[str, Any] | None:
    if path is None:
        return None
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return None
    return data if isinstance(data, dict) else None


def evidence_freshness(
    path: Path | None,
    *,
    now_s: float,
    max_age_s: float,
) -> dict[str, Any]:
    if path is None:
        return {"status": UNKNOWN, "age_s": None, "max_age_s": max_age_s}
    try:
        age_s = max(0.0, now_s - path.stat().st_mtime)
    except OSError:
        return {"status": UNKNOWN, "age_s": None, "max_age_s": max_age_s}
    return {
        "status": STALE if age_s > max_age_s else PASS,
        "age_s": age_s,
        "max_age_s": max_age_s,
    }


def step_statuses(report: dict[str, Any] | None) -> dict[str, str]:
    if not report:
        return {}
    steps = report.get("steps", [])
    if not isinstance(steps, list):
        return {}
    return {
        str(step.get("phase")): str(step.get("status", UNKNOWN))
        for step in steps
        if isinstance(step, dict) and step.get("phase")
    }


def step_notes(report: dict[str, Any] | None) -> dict[str, list[str]]:
    if not report:
        return {}
    steps = report.get("steps", [])
    if not isinstance(steps, list):
        return {}
    return {
        str(step.get("phase")): [
            str(note)
            for note in (step.get("notes") or [])
        ]
        for step in steps
        if isinstance(step, dict) and step.get("phase")
    }


def choose_evidence(log_dir: Path, explicit: Path | None) -> tuple[Path | None, dict[str, Any] | None]:
    if explicit is not None:
        return explicit, load_json_report(explicit)
    manifest = latest_manifest_report(log_dir)
    if manifest is not None:
        return manifest, load_json_report(manifest)
    suite = latest_suite_report(log_dir)
    if suite is not None:
        return suite, load_suite_report(suite)
    return None, None


def evaluate_decision(
    report: dict[str, Any] | None,
    *,
    evidence_path: Path | None,
    max_evidence_age_s: float | None = None,
    now_s: float | None = None,
) -> DecisionReport:
    statuses = step_statuses(report)
    notes_by_phase = step_notes(report)
    reasons: list[str] = []
    decision = WAITING
    summary = "waiting for live pr0p evidence"
    source_kind = evidence_source_kind(report, evidence_path)
    freshness: dict[str, Any] | None = None
    if max_evidence_age_s is not None:
        freshness = evidence_freshness(
            evidence_path,
            now_s=time.time() if now_s is None else now_s,
            max_age_s=max_evidence_age_s,
        )

    if report is None:
        reasons.append("NO_EVIDENCE_REPORT")
    elif freshness and freshness["status"] == STALE:
        summary = "evidence is stale; rerun the live manifest or safe suite"
        reasons.append("EVIDENCE_STALE")
    elif report.get("status") == FAIL or any(status == FAIL for status in statuses.values()):
        decision = REJECT
        summary = "one or more core probe phases failed"
        failed = [phase for phase, status in statuses.items() if status == FAIL]
        if report.get("status") == FAIL:
            reasons.append("SOURCE_STATUS_FAIL")
        reasons.extend("FAIL:%s" % phase for phase in failed)
    else:
        source_status = report.get("status")
        p2 = statuses.get("P2-websocket")
        p2_msp = statuses.get("P2-msp-readonly")
        p2_fc_status = statuses.get("P2-fc-status-readonly")
        p2_mode_ranges = statuses.get("P2-mode-ranges-readonly")
        p0_isolation = statuses.get("P0-isolation")
        p1 = statuses.get("P1-install-discovery")
        p1_client = statuses.get("P1-client-executable")
        p3 = statuses.get("P3-capture")
        p4 = statuses.get("P4-input-readiness")
        p4_mapping = statuses.get("P4-input-mapping")
        p4_rc_visual = statuses.get("P4-rc-channels-visual")
        p5_live_yaw = statuses.get("P5-yaw-live")
        p5_live_pitch = statuses.get("P5-pitch-live")
        p5_uinput_rc_effect = statuses.get("P5-uinput-rc-effect-throttle-low")
        p5_uinput_status_effect = statuses.get("P5-uinput-status-effect-throttle-clear")
        p5_uinput_aux_effect = statuses.get("P5-uinput-rc-effect-aux1-high")
        p5_uinput_aux_arm_status = statuses.get("P5-uinput-aux1-arm-status")
        p5_rc_loopback = statuses.get("P5-rc-loopback")
        p5_persistent_uinput = statuses.get("P5-persistent-uinput-live-response")
        p5_response_acceptance = statuses.get("P5-response-acceptance")
        p6_synthetic = statuses.get("P6-synthetic-e2e-dry-run")
        p6_synthetic_log = statuses.get("P6-synthetic-log-check")
        p6 = statuses.get("P6-tracking-pid-dry-run")
        p6_log = statuses.get("P6-tracking-log-check")
        p6_live = statuses.get("P6-tracking-live")
        p6_tracking_acceptance = statuses.get("P6-tracking-acceptance")

        if source_kind != "live_manifest":
            reasons.append("MISSING_LIVE_MANIFEST_EVIDENCE")
        if source_status != PASS:
            reasons.append("SOURCE_STATUS_NOT_PASS:%s" % source_status)
        if p0_isolation != PASS:
            reasons.append("MISSING_GAZEBO_ISOLATION_CHECK")
        if p1 not in (PASS, "WAITING"):
            reasons.append("MISSING_INSTALL_DISCOVERY")
        if p1_client != PASS:
            reasons.append("MISSING_PR0P_CLIENT_EXECUTABLE")
        if p2 != PASS:
            reasons.append("MISSING_LIVE_PR0P_WEBSOCKET")
        if p2_msp != PASS:
            reasons.append("MISSING_READONLY_MSP_OVER_WEBSOCKET")
        if p2_fc_status != PASS:
            reasons.append("MISSING_READONLY_FC_STATUS")
        elif p5_uinput_aux_arm_status != PASS and any(
            note == "FC_NOT_ARMED" or note.startswith("ARMING_DISABLED:")
            for note in notes_by_phase.get("P2-fc-status-readonly", [])
        ):
            reasons.append("FC_ARM_STATE_BLOCKED")
        if p2_mode_ranges != PASS:
            reasons.append("MISSING_READONLY_MODE_RANGES")
        if p3 != PASS:
            reasons.append("MISSING_FPV_CAPTURE")
        if p4 != PASS:
            reasons.append("MISSING_INPUT_READINESS")
        if p4_mapping != PASS:
            reasons.append("MISSING_PR0P_INPUT_MAPPING")
        if p4_rc_visual != PASS:
            reasons.append("MISSING_RUNTIME_RC_CHANNEL_VISUAL")
        if p6 != PASS:
            reasons.append("MISSING_TRACKING_DRY_RUN")
        if p6_log != PASS:
            reasons.append("MISSING_TRACKING_LOG_CHECK")
        if p5_response_acceptance != PASS and p5_live_yaw != PASS:
            reasons.append("MISSING_LIVE_YAW_RESPONSE")
        if p5_response_acceptance != PASS and p5_live_pitch != PASS:
            reasons.append("MISSING_LIVE_PITCH_RESPONSE")
        if p5_uinput_rc_effect != PASS:
            reasons.append("MISSING_UINPUT_RC_EFFECT_THROTTLE_LOW")
        if p5_uinput_status_effect != PASS:
            reasons.append("MISSING_UINPUT_STATUS_THROTTLE_CLEAR")
        if p5_uinput_aux_effect != PASS:
            reasons.append("MISSING_UINPUT_AUX1_RC_EFFECT")
        if p5_uinput_aux_arm_status != PASS:
            reasons.append("MISSING_UINPUT_AUX1_ARM_STATUS")
        virtual_rc_source_proven = (
            p5_uinput_rc_effect == PASS
            and p5_uinput_aux_effect == PASS
            and p5_uinput_aux_arm_status == PASS
        )
        if p5_rc_loopback != PASS and not virtual_rc_source_proven:
            # MSP_SET_RAW_RC cannot become the RC source while the sim reads a
            # joystick receiver; a fully proven virtual-joystick chain (RC
            # effect + AUX1 effect + arm status) is the accepted RC source.
            reasons.append("MISSING_RC_SOURCE_LOOPBACK")
        if p5_response_acceptance != PASS and p5_persistent_uinput != PASS:
            reasons.append("MISSING_PERSISTENT_UINPUT_RESPONSE")
        if p5_response_acceptance != PASS:
            reasons.append("MISSING_RESPONSE_ACCEPTANCE")
        if p6_synthetic != PASS:
            reasons.append("MISSING_SYNTHETIC_E2E_DRY_RUN")
        if p6_synthetic_log != PASS:
            reasons.append("MISSING_SYNTHETIC_LOG_CHECK")
        if p6_tracking_acceptance != PASS and p6_live != PASS:
            reasons.append("MISSING_LIVE_TRACKING_CONTROL")
        if p6_tracking_acceptance != PASS:
            reasons.append("MISSING_TRACKING_ACCEPTANCE")

        if not reasons:
            decision = PROMOTE_CANDIDATE
            summary = "all promotion gates are present"
        else:
            summary = "promotion is not justified yet"

    return DecisionReport(
        run_id="decision",
        started_at=time.strftime("%Y-%m-%dT%H:%M:%S%z", time.localtime()),
        decision=decision,
        summary=summary,
        evidence_path=str(evidence_path) if evidence_path else None,
        reasons=reasons,
        metrics={
            "phase_statuses": statuses,
            "phase_notes": notes_by_phase,
            "source_status": report.get("status") if report else None,
            "source_kind": source_kind,
            "source_next_action": report.get("next_action") if report else None,
            "evidence_freshness": freshness,
        },
    )


def build_markdown(report: DecisionReport) -> str:
    lines = [
        "# SimITL / pr0p Decision Report",
        "",
        "Run: `%s`" % report.run_id,
        "Started: `%s`" % report.started_at,
        "Decision: `%s`" % report.decision,
        "Evidence: `%s`" % (report.evidence_path or "none"),
        "",
        report.summary,
        "",
        "## Reasons",
        "",
    ]
    if report.reasons:
        for reason in report.reasons:
            lines.append("- %s" % reason)
    else:
        lines.append("- all required promotion gates are present")
    lines.extend([
        "",
        "## Phase Statuses",
        "",
        "| Phase | Status |",
        "| --- | --- |",
    ])
    for phase, status in sorted(report.metrics.get("phase_statuses", {}).items()):
        lines.append("| %s | %s |" % (phase, status))
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


def write_reports(report: DecisionReport, log_dir: Path, *, run_id: str) -> tuple[Path, Path]:
    log_dir.mkdir(parents=True, exist_ok=True)
    report.run_id = run_id
    stamp = time.strftime("%Y%m%d-%H%M%S", time.localtime())
    json_path = log_dir / ("%s-%s-decision.json" % (stamp, run_id))
    md_path = log_dir / ("%s-%s-decision.md" % (stamp, run_id))
    json_path.write_text(
        json.dumps(report.as_dict(), indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    md_path.write_text(build_markdown(report), encoding="utf-8")
    return json_path, md_path


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-id", default="pr0p-decision")
    parser.add_argument("--log-dir", type=Path, default=DEFAULT_LOG_DIR)
    parser.add_argument("--evidence", type=Path,
                        help="Explicit live-manifest or suite JSON report")
    parser.add_argument(
        "--max-evidence-age-s",
        type=float,
        default=DEFAULT_MAX_EVIDENCE_AGE_S,
        help="Maximum age for decision evidence before it is treated as stale.",
    )
    args = parser.parse_args(argv)
    if args.max_evidence_age_s < 0:
        parser.error("--max-evidence-age-s must be non-negative")
    return args


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    evidence_path, evidence = choose_evidence(args.log_dir, args.evidence)
    report = evaluate_decision(
        evidence,
        evidence_path=evidence_path,
        max_evidence_age_s=args.max_evidence_age_s,
    )
    json_path, md_path = write_reports(report, args.log_dir, run_id=args.run_id)
    print("simitl-pr0p-decision %s report=%s summary=%s" % (
        report.decision,
        json_path,
        md_path,
    ))
    print(report.summary)
    return 1 if report.decision == REJECT else 0


if __name__ == "__main__":
    raise SystemExit(main())
