#!/usr/bin/env python3
"""Plan or run one safe candidate action from external_operator_preflight."""

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

from external_operator_preflight import (  # noqa: E402
    ExternalOperatorPreflightReport,
    run_operator_preflight,
    write_reports as write_operator_reports,
)
from latency_probe import normalize_axes, parse_axis_shift_expectations  # noqa: E402
from sandbox_resume_runner import (  # noqa: E402
    BLOCKED,
    CommandRunResult,
    PASS,
    PLANNED,
    REJECT,
    TIMEOUT,
    command_status_from_returncode,
    output_tail,
    run_shell_command,
)
from sandbox_status_report import (  # noqa: E402
    DEFAULT_BBOX_FILE,
    DEFAULT_EVIDENCE_STALE_AFTER_S,
    DEFAULT_LOG_DIR,
    WAITING,
)
from screen_tracking_loop import parse_bbox  # noqa: E402
from sitl_log import resolve_log_dir, timestamp_slug  # noqa: E402
from window_safety import tooling_window_rejection_reason  # noqa: E402


OPERATOR_CANDIDATE_ACTION_PROGRESS = "OPERATOR_CANDIDATE_ACTION_PROGRESS"


@dataclass
class ExternalCandidateActionRunnerReport:
    run_id: str
    started_at: str
    status: str
    summary: str
    steps: list[dict[str, Any]] = field(default_factory=list)
    evidence_paths: dict[str, str | None] = field(default_factory=dict)
    metrics: dict[str, Any] = field(default_factory=dict)

    def as_dict(self) -> dict[str, Any]:
        return {
            "run_id": self.run_id,
            "started_at": self.started_at,
            "status": self.status,
            "summary": self.summary,
            "steps": self.steps,
            "evidence_paths": self.evidence_paths,
            "metrics": self.metrics,
        }


def candidate_actions(operator_report: ExternalOperatorPreflightReport) -> list[dict[str, Any]]:
    actions = operator_report.metrics.get("candidate_action_plan", [])
    if not isinstance(actions, list):
        return []
    return [action for action in actions if isinstance(action, dict)]


def select_candidate_action(
    actions: list[dict[str, Any]],
    candidate_index: int | None,
    candidate_window_id: str | None = None,
    candidate_title: str | None = None,
    candidate_title_contains: str | None = None,
) -> tuple[dict[str, Any] | None, list[str]]:
    if candidate_window_id:
        for action in actions:
            if str(action.get("window_id", "")) == candidate_window_id:
                return action, []
        return None, ["candidate_window_id_not_found:%s" % candidate_window_id]
    if candidate_title:
        for action in actions:
            if str(action.get("title", "")) == candidate_title:
                return action, []
        return None, ["candidate_title_not_found:%s" % candidate_title]
    if candidate_title_contains:
        needle = candidate_title_contains.lower()
        matches = [
            action for action in actions
            if needle in str(action.get("title", "")).lower()
        ]
        if len(matches) == 1:
            return matches[0], []
        if not matches:
            return None, [
                "candidate_title_contains_not_found:%s" % candidate_title_contains
            ]
        return None, [
            "candidate_title_contains_ambiguous:%s:%d"
            % (candidate_title_contains, len(matches))
        ]
    if candidate_index is None:
        return None, ["candidate_index_required"]
    for action in actions:
        if int(action.get("candidate_index", -1)) == candidate_index:
            return action, []
    return None, ["candidate_index_not_found:%s" % candidate_index]


def bbox_to_arg(bbox: tuple[int, int, int, int] | None) -> str | None:
    if bbox is None:
        return None
    return "%d,%d,%d,%d" % bbox


def bbox_inside_candidate_region(
    bbox: tuple[int, int, int, int],
    region: dict[str, Any] | None,
) -> bool:
    if not isinstance(region, dict):
        return False
    try:
        width = int(region["width"])
        height = int(region["height"])
        x, y, w, h = (int(value) for value in bbox)
    except (KeyError, TypeError, ValueError):
        return False
    return x >= 0 and y >= 0 and w > 0 and h > 0 and x + w <= width and y + h <= height


def action_with_candidate_bbox(
    action: dict[str, Any] | None,
    candidate_bbox: tuple[int, int, int, int] | None,
) -> tuple[dict[str, Any] | None, list[str]]:
    if action is None or candidate_bbox is None:
        return action, []
    if not bbox_inside_candidate_region(candidate_bbox, action.get("region")):
        return action, [
            "candidate_bbox_outside_region:%s" % bbox_to_arg(candidate_bbox)
        ]
    command_template = action.get("write_bbox_file_region_command")
    if not command_template:
        return action, ["candidate_bbox_write_command_missing"]
    updated = dict(action)
    updated["next_step"] = "write_bbox_file_region"
    updated["next_command"] = str(command_template).replace(
        "X,Y,W,H",
        str(bbox_to_arg(candidate_bbox)),
    )
    updated["expected_status"] = PASS
    updated["safety_class"] = "manual_bbox_write"
    updated["candidate_bbox"] = list(candidate_bbox)
    updated["unblocks"] = ["target_bbox"]
    return updated, []


def candidate_rejection_summary(reasons: list[str]) -> str | None:
    if any(reason.startswith("candidate_window_is_tooling:") for reason in reasons):
        return "selected candidate is a tooling/editor window, not a simulator view"
    if any(reason.startswith("candidate_bbox_outside_region:") for reason in reasons):
        return "candidate bbox is outside the selected candidate region"
    if "candidate_bbox_write_command_missing" in reasons:
        return "selected candidate cannot write a bbox file"
    return None


def rejection_summary_for_reasons(reasons: list[str]) -> str:
    candidate_summary = candidate_rejection_summary(reasons)
    if candidate_summary:
        return candidate_summary
    if any(
        reason.startswith("candidate_title_contains_not_found:")
        for reason in reasons
    ):
        return "candidate title substring did not match any eligible candidate"
    if any(
        reason.startswith("candidate_title_contains_ambiguous:")
        for reason in reasons
    ):
        return "candidate title substring matched multiple candidates"
    if any(reason.startswith("candidate_window_id_not_found:") for reason in reasons):
        return "candidate window id did not match any eligible candidate"
    if any(reason.startswith("candidate_title_not_found:") for reason in reasons):
        return "candidate title did not match any eligible candidate"
    if any(reason.startswith("candidate_index_not_found:") for reason in reasons):
        return "candidate index did not match any eligible candidate"
    return "selected candidate index was not found"


def base_step(
    *,
    action: dict[str, Any] | None,
    candidate_index: int | None,
    reasons: list[str],
) -> dict[str, Any]:
    return {
        "candidate_index": (
            action.get("candidate_index")
            if isinstance(action, dict) else
            candidate_index
        ),
        "title": action.get("title") if isinstance(action, dict) else None,
        "window_id": action.get("window_id") if isinstance(action, dict) else None,
        "next_step": action.get("next_step") if isinstance(action, dict) else None,
        "status": PLANNED,
        "summary": "not evaluated yet",
        "safety_class": (
            action.get("safety_class") if isinstance(action, dict) else "unknown"
        ),
        "sends_uinput": bool(action.get("sends_uinput")) if isinstance(action, dict) else False,
        "requires_user_judgement": (
            bool(action.get("requires_user_judgement")) if isinstance(action, dict) else False
        ),
        "expected_status": (
            action.get("expected_status") if isinstance(action, dict) else None
        ),
        "candidate_bbox": (
            action.get("candidate_bbox") if isinstance(action, dict) else None
        ),
        "unblocks": action.get("unblocks", []) if isinstance(action, dict) else [],
        "command": action.get("next_command") if isinstance(action, dict) else None,
        "reasons": reasons,
    }


def evaluate_candidate_action(
    *,
    action: dict[str, Any] | None,
    candidate_index: int | None,
    reasons: list[str],
    execute_safe: bool = False,
    ack_candidate: bool = False,
    timeout_s: float = 30.0,
) -> tuple[dict[str, Any], int]:
    step = base_step(
        action=action,
        candidate_index=candidate_index,
        reasons=reasons,
    )
    candidate_summary = candidate_rejection_summary(reasons)
    if candidate_summary:
        step["status"] = REJECT
        step["summary"] = candidate_summary
        step["command"] = None
        return step, 0
    if action is None:
        step["status"] = WAITING if "candidate_index_required" in reasons else REJECT
        step["summary"] = (
            "select a candidate index from candidate_action_plan"
            if step["status"] == WAITING else
            rejection_summary_for_reasons(reasons)
        )
        return step, 0
    if action.get("sends_uinput"):
        step["status"] = REJECT
        step["summary"] = "candidate actions are not allowed to send uinput"
        return step, 0
    if not action.get("next_command"):
        step["status"] = REJECT
        step["summary"] = "candidate action has no command"
        return step, 0
    if not ack_candidate:
        step["status"] = BLOCKED
        step["summary"] = "candidate execution requires --ack-candidate"
        return step, 0
    if not execute_safe:
        step["status"] = PLANNED
        step["summary"] = "candidate action is available; rerun with --execute-safe"
        return step, 0

    result = run_shell_command(str(action.get("next_command")), timeout_s=timeout_s)
    step["status"] = command_status_from_returncode(result)
    step["returncode"] = result.returncode
    step["stdout_tail"] = output_tail(result.stdout)
    step["stderr_tail"] = output_tail(result.stderr)
    if step["status"] == PASS:
        step["summary"] = "candidate action completed successfully"
    elif step["status"] == WAITING:
        step["summary"] = "candidate action ran and reported WAITING"
    elif step["status"] == TIMEOUT:
        step["summary"] = "candidate action timed out"
    else:
        step["summary"] = "candidate action failed"
    return step, 1


def decide_status(step: dict[str, Any], executed_count: int) -> str:
    if step.get("status") in {REJECT, TIMEOUT}:
        return REJECT
    if executed_count > 0:
        return OPERATOR_CANDIDATE_ACTION_PROGRESS
    if step.get("status") in {PLANNED, BLOCKED, WAITING}:
        return WAITING
    return WAITING


def report_summary(status: str) -> str:
    if status == OPERATOR_CANDIDATE_ACTION_PROGRESS:
        return "candidate action executed; inspect the generated evidence"
    if status == REJECT:
        return "candidate action runner found a blocking failure"
    return "candidate action runner is waiting for candidate selection or execution ack"


def build_markdown(report: ExternalCandidateActionRunnerReport) -> str:
    decision = report.metrics.get("candidate_action_decision", {})
    operator_report = report.metrics.get("operator_preflight_report", {})
    post_operator_report = report.metrics.get("post_operator_preflight_report")
    actions = report.metrics.get("candidate_action_plan", [])
    lines = [
        "# External Candidate Action Runner",
        "",
        "Run: `%s`" % report.run_id,
        "Started: `%s`" % report.started_at,
        "Status: `%s`" % report.status,
        "",
        report.summary,
        "",
        "## Decision",
        "",
        "| Field | Value |",
        "| --- | --- |",
        "| selected_candidate_index | `%s` |" % (
            decision.get("selected_candidate_index") or "none"
        ),
        "| selected_next_step | `%s` |" % (
            decision.get("selected_next_step") or "none"
        ),
        "| executed_count | `%s` |" % decision.get("executed_count", 0),
        "| operator_status | `%s` |" % operator_report.get("status", "unknown"),
        "| post_operator_status | `%s` |" % (
            post_operator_report.get("status", "none")
            if isinstance(post_operator_report, dict) else
            "none"
        ),
        "",
        "## Steps",
        "",
        "| Candidate | Step | Status | Sends uinput | Summary |",
        "| --- | --- | --- | --- | --- |",
    ]
    for step in report.steps:
        lines.append("| %s | `%s` | `%s` | %s | %s |" % (
            step.get("candidate_index") or "none",
            step.get("next_step") or "none",
            step.get("status", "UNKNOWN"),
            "yes" if step.get("sends_uinput") else "no",
            str(step.get("summary", "")).replace("|", "\\|"),
        ))
    lines.extend([
        "",
        "## Candidate Action Plan",
        "",
        "| # | Title | Next step | Expected | Sends uinput |",
        "| --- | --- | --- | --- | --- |",
    ])
    for action in actions if isinstance(actions, list) else []:
        if not isinstance(action, dict):
            continue
        lines.append("| %s | %s | `%s` | `%s` | %s |" % (
            action.get("candidate_index", "?"),
            str(action.get("title", "")).replace("|", "\\|"),
            action.get("next_step", "unknown"),
            action.get("expected_status", "unknown"),
            "yes" if action.get("sends_uinput") else "no",
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
    report: ExternalCandidateActionRunnerReport,
    log_dir: Path,
) -> tuple[Path, Path]:
    slug = "%s-%s" % (timestamp_slug(), report.run_id)
    json_path = log_dir / ("%s-external-candidate-action-runner.json" % slug)
    md_path = log_dir / ("%s-external-candidate-action-runner.md" % slug)
    report.evidence_paths["candidate_action_runner_json"] = str(json_path)
    report.evidence_paths["candidate_action_runner_md"] = str(md_path)
    json_path.write_text(json.dumps(report.as_dict(), indent=2, sort_keys=True) + "\n",
                         encoding="utf-8")
    md_path.write_text(build_markdown(report), encoding="utf-8")
    return json_path, md_path


def run_candidate_action(
    *,
    run_id: str,
    log_dir: Path,
    window_title: str | None,
    bbox_file: Path,
    window_exact: bool,
    window_case_sensitive: bool,
    window_min_width: int,
    window_min_height: int,
    require_live_input: bool,
    require_live_follow: bool,
    require_fresh_evidence: bool,
    evidence_stale_after_s: float,
    max_window_candidates: int,
    axis_sweep_axes: tuple[str, ...],
    axis_expected_shifts: dict[str, tuple[str, int]] | None,
    axis_min_shift_px: float,
    candidate_index: int | None = None,
    candidate_window_id: str | None = None,
    candidate_title: str | None = None,
    candidate_title_contains: str | None = None,
    candidate_bbox: tuple[int, int, int, int] | None = None,
    execute_safe: bool = False,
    ack_candidate: bool = False,
    timeout_s: float = 30.0,
) -> ExternalCandidateActionRunnerReport:
    log_dir.mkdir(parents=True, exist_ok=True)
    started_at = time.strftime("%Y-%m-%dT%H:%M:%S%z", time.localtime())
    operator_report = run_operator_preflight(
        run_id="%s-operator-preflight" % run_id,
        log_dir=log_dir,
        window_title=window_title,
        bbox_file=bbox_file,
        window_exact=window_exact,
        window_case_sensitive=window_case_sensitive,
        window_min_width=window_min_width,
        window_min_height=window_min_height,
        require_live_input=require_live_input,
        require_live_follow=require_live_follow,
        require_fresh_evidence=require_fresh_evidence,
        evidence_stale_after_s=evidence_stale_after_s,
        max_window_candidates=max_window_candidates,
        axis_sweep_axes=axis_sweep_axes,
        axis_expected_shifts=axis_expected_shifts,
        axis_min_shift_px=axis_min_shift_px,
    )
    operator_json, operator_md = write_operator_reports(operator_report, log_dir)
    actions = candidate_actions(operator_report)
    selected, reasons = select_candidate_action(
        actions,
        candidate_index,
        candidate_window_id=candidate_window_id,
        candidate_title=candidate_title,
        candidate_title_contains=candidate_title_contains,
    )
    selected, bbox_reasons = action_with_candidate_bbox(selected, candidate_bbox)
    tooling_reason = tooling_window_rejection_reason(selected)
    reasons = reasons + bbox_reasons + ([tooling_reason] if tooling_reason else [])
    step, executed_count = evaluate_candidate_action(
        action=selected,
        candidate_index=candidate_index,
        reasons=reasons,
        execute_safe=execute_safe,
        ack_candidate=ack_candidate,
        timeout_s=timeout_s,
    )
    post_operator_report: ExternalOperatorPreflightReport | None = None
    post_operator_json: Path | None = None
    post_operator_md: Path | None = None
    if executed_count > 0:
        post_operator_report = run_operator_preflight(
            run_id="%s-post-operator-preflight" % run_id,
            log_dir=log_dir,
            window_title=window_title,
            bbox_file=bbox_file,
            window_exact=window_exact,
            window_case_sensitive=window_case_sensitive,
            window_min_width=window_min_width,
            window_min_height=window_min_height,
            require_live_input=require_live_input,
            require_live_follow=require_live_follow,
            require_fresh_evidence=require_fresh_evidence,
            evidence_stale_after_s=evidence_stale_after_s,
            max_window_candidates=max_window_candidates,
            axis_sweep_axes=axis_sweep_axes,
            axis_expected_shifts=axis_expected_shifts,
            axis_min_shift_px=axis_min_shift_px,
        )
        post_operator_json, post_operator_md = write_operator_reports(
            post_operator_report,
            log_dir,
        )
    status = decide_status(step, executed_count)
    return ExternalCandidateActionRunnerReport(
        run_id=run_id,
        started_at=started_at,
        status=status,
        summary=report_summary(status),
        steps=[step],
        evidence_paths={
            "operator_preflight_json": str(operator_json),
            "operator_preflight_md": str(operator_md),
            "post_operator_preflight_json": (
                str(post_operator_json) if post_operator_json is not None else None
            ),
            "post_operator_preflight_md": (
                str(post_operator_md) if post_operator_md is not None else None
            ),
        },
        metrics={
            "window_title": window_title,
            "bbox_file": str(bbox_file),
            "candidate_index": candidate_index,
            "candidate_window_id": candidate_window_id,
            "candidate_title": candidate_title,
            "candidate_title_contains": candidate_title_contains,
            "candidate_bbox": (
                list(candidate_bbox) if candidate_bbox is not None else None
            ),
            "execute_safe": execute_safe,
            "ack_candidate": ack_candidate,
            "executed_count": executed_count,
            "axis_sweep_axes": list(axis_sweep_axes),
            "axis_expected_shifts": {
                axis: "%s%s" % ("+" if sign > 0 else "-", component)
                for axis, (component, sign) in (axis_expected_shifts or {}).items()
            } or None,
            "axis_min_shift_px": axis_min_shift_px if axis_expected_shifts else None,
            "candidate_action_decision": {
                "selected_candidate_index": (
                    selected.get("candidate_index")
                    if isinstance(selected, dict) else
                    candidate_index
                ),
                "selector": (
                    "candidate_window_id"
                    if candidate_window_id else
                    "candidate_title"
                    if candidate_title else
                    "candidate_title_contains"
                    if candidate_title_contains else
                    "candidate_index"
                    if candidate_index is not None else
                    None
                ),
                "selector_value": (
                    candidate_window_id
                    or candidate_title
                    or candidate_title_contains
                    or candidate_index
                ),
                "selected_next_step": (
                    selected.get("next_step") if isinstance(selected, dict) else None
                ),
                "selected_title": (
                    selected.get("title") if isinstance(selected, dict) else None
                ),
                "reasons": reasons,
                "executed_count": executed_count,
                "post_operator_status": (
                    post_operator_report.status
                    if post_operator_report is not None else
                    None
                ),
            },
            "candidate_action_plan": actions,
            "operator_preflight_report": operator_report.as_dict(),
            "post_operator_preflight_report": (
                post_operator_report.as_dict()
                if post_operator_report is not None else
                None
            ),
            "scope": (
                "candidate action runner; executes only one selected non-uinput "
                "candidate action after --ack-candidate and --execute-safe"
            ),
        },
    )


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-id", default="external-candidate-action")
    parser.add_argument("--log-dir", type=Path, default=DEFAULT_LOG_DIR)
    parser.add_argument("--window-title", default="pr0p")
    parser.add_argument("--bbox-file", type=Path, default=DEFAULT_BBOX_FILE)
    parser.add_argument("--window-exact", action="store_true")
    parser.add_argument("--window-case-sensitive", action="store_true")
    parser.add_argument("--window-min-width", type=int, default=32)
    parser.add_argument("--window-min-height", type=int, default=32)
    parser.add_argument("--require-live-input", action="store_true")
    parser.add_argument("--require-live-follow", action="store_true")
    parser.add_argument("--require-fresh-evidence", action="store_true")
    parser.add_argument(
        "--evidence-stale-after-s",
        type=float,
        default=DEFAULT_EVIDENCE_STALE_AFTER_S,
    )
    parser.add_argument("--axis-sweep-axes", default="yaw,pitch")
    parser.add_argument("--axis-expected-shifts", default="")
    parser.add_argument("--axis-min-shift-px", type=float, default=0.5)
    parser.add_argument("--max-window-candidates", type=int, default=8)
    parser.add_argument("--candidate-index", type=int)
    parser.add_argument("--candidate-window-id")
    parser.add_argument("--candidate-title")
    parser.add_argument("--candidate-title-contains")
    parser.add_argument("--candidate-bbox", type=parse_bbox,
                        help="Write bbox for the selected region candidate as X,Y,W,H")
    parser.add_argument("--execute-safe", action="store_true")
    parser.add_argument("--ack-candidate", action="store_true")
    parser.add_argument("--timeout", type=float, default=30.0)
    parser.add_argument("--json", action="store_true")
    args = parser.parse_args(argv)
    if args.window_min_width <= 0 or args.window_min_height <= 0:
        parser.error("--window-min-width/--window-min-height must be positive")
    if args.evidence_stale_after_s < 0:
        parser.error("--evidence-stale-after-s must be non-negative")
    if args.axis_min_shift_px < 0:
        parser.error("--axis-min-shift-px must be non-negative")
    if args.max_window_candidates <= 0:
        parser.error("--max-window-candidates must be positive")
    if args.candidate_index is not None and args.candidate_index <= 0:
        parser.error("--candidate-index must be positive")
    if args.timeout <= 0:
        parser.error("--timeout must be positive")
    selectors = [
        args.candidate_index is not None,
        bool(args.candidate_window_id),
        bool(args.candidate_title),
        bool(args.candidate_title_contains),
    ]
    if sum(1 for selected in selectors if selected) > 1:
        parser.error(
            "use only one of --candidate-index, --candidate-window-id, "
            "--candidate-title, or --candidate-title-contains"
        )
    try:
        args.axis_sweep_axes = normalize_axes(args.axis_sweep_axes)
    except ValueError as exc:
        parser.error(str(exc))
    try:
        args.axis_expected_shifts = parse_axis_shift_expectations(
            args.axis_expected_shifts
        )
    except ValueError as exc:
        parser.error(str(exc))
    unknown_expected = [
        axis for axis in args.axis_expected_shifts
        if axis not in args.axis_sweep_axes
    ]
    if unknown_expected:
        parser.error(
            "--axis-expected-shifts contains unselected axis/axes: %s"
            % ",".join(sorted(unknown_expected))
        )
    return args


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    log_dir = resolve_log_dir(str(args.log_dir), repo_root=REPO_ROOT)
    report = run_candidate_action(
        run_id=args.run_id,
        log_dir=log_dir,
        window_title=args.window_title,
        bbox_file=args.bbox_file,
        window_exact=args.window_exact,
        window_case_sensitive=args.window_case_sensitive,
        window_min_width=args.window_min_width,
        window_min_height=args.window_min_height,
        require_live_input=args.require_live_input,
        require_live_follow=args.require_live_follow,
        require_fresh_evidence=args.require_fresh_evidence,
        evidence_stale_after_s=args.evidence_stale_after_s,
        max_window_candidates=args.max_window_candidates,
        axis_sweep_axes=args.axis_sweep_axes,
        axis_expected_shifts=args.axis_expected_shifts,
        axis_min_shift_px=args.axis_min_shift_px,
        candidate_index=args.candidate_index,
        candidate_window_id=args.candidate_window_id,
        candidate_title=args.candidate_title,
        candidate_title_contains=args.candidate_title_contains,
        candidate_bbox=args.candidate_bbox,
        execute_safe=args.execute_safe,
        ack_candidate=args.ack_candidate,
        timeout_s=args.timeout,
    )
    json_path, md_path = write_reports(report, log_dir)
    if args.json:
        print(json.dumps(report.as_dict(), indent=2, sort_keys=True))
    else:
        print("external-candidate-action %s report=%s summary=%s" % (
            report.status,
            json_path,
            md_path,
        ))
    if report.status == OPERATOR_CANDIDATE_ACTION_PROGRESS:
        return 0
    if report.status == WAITING:
        return 2
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
