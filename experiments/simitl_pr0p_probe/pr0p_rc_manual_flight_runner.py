#!/usr/bin/env python3
"""Plan or run the first pr0p RC/manual-flight readiness gate."""

from __future__ import annotations

import argparse
import json
import shlex
import subprocess
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable

PROBE_DIR = Path(__file__).resolve().parent
if str(PROBE_DIR) not in sys.path:
    sys.path.insert(0, str(PROBE_DIR))

from preflight_probe import DEFAULT_INSTALL_ROOT  # noqa: E402
from pr0p_aux1_acceptance_runner import (  # noqa: E402
    run_aux1_acceptance,
    write_reports as write_aux1_reports,
)
from pr0p_capture_probe import DEFAULT_LOG_DIR  # noqa: E402
from pr0p_client_probe import (  # noqa: E402
    run_client_probe,
    write_reports as write_client_reports,
)
from pr0p_input_config_probe import (  # noqa: E402
    DEFAULT_EXPECTED_DEVICE,
    DEFAULT_INPUT_CONFIG,
    run_input_config_probe,
    write_reports as write_input_config_reports,
)
from screen_tracking_loop import parse_bbox  # noqa: E402


PASS = "PASS"
WAITING = "WAITING"
FAIL = "FAIL"
SKIPPED = "SKIPPED"
MANUAL_RESPONSE_AXES = (
    ("manual_yaw_response", "yaw", "x"),
    ("manual_pitch_response", "pitch", "y"),
    ("manual_roll_response", "roll", "x"),
)


@dataclass
class ManualFlightStage:
    name: str
    status: str
    summary: str
    report: str | None = None
    command: str = ""
    returncode: int | None = None
    stdout: str = ""
    stderr: str = ""
    notes: list[str] = field(default_factory=list)

    def as_dict(self) -> dict[str, Any]:
        return {
            "name": self.name,
            "status": self.status,
            "summary": self.summary,
            "report": self.report,
            "command": self.command,
            "returncode": self.returncode,
            "stdout": self.stdout,
            "stderr": self.stderr,
            "notes": self.notes,
        }


@dataclass
class ManualFlightReport:
    run_id: str
    started_at: str
    status: str
    summary: str
    next_action: str
    stages: list[ManualFlightStage] = field(default_factory=list)
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


def summarize(stages: list[ManualFlightStage]) -> tuple[str, str, str]:
    by_name = {stage.name: stage for stage in stages}
    if any(stage.status == FAIL for stage in stages):
        return FAIL, "RC/manual-flight readiness has a failed prerequisite", (
            "Fix the failed prerequisite before launching a live manual flight attempt."
        )
    client = by_name.get("pr0p_client")
    if client is None or client.status != PASS:
        return WAITING, "pr0p client is not ready for manual flight", (
            "Install or repair the isolated pr0p client before opening a local race."
        )
    mapping = by_name.get("primary_rc_mapping")
    if mapping is None or mapping.status != PASS:
        return WAITING, "primary RC axes are not ready", (
            "Bind roll, pitch, throttle, and yaw in pr0p Controls -> RC Channels."
        )
    aux = by_name.get("aux1_arm_acceptance")
    if aux is None or aux.status != PASS:
        return WAITING, "AUX1/CH5 arm channel is not ready", (
            "Bind AUX1/CH5 or apply the backup-backed AUX1 patch, then run this "
            "gate with live acknowledgements."
        )
    missing_response = [
        name
        for name, _axis, _image_axis in MANUAL_RESPONSE_AXES
        if by_name.get(name) is None or by_name[name].status != PASS
    ]
    if missing_response:
        return WAITING, "manual stick response is not proven yet", (
            "Run this gate with live acknowledgements until yaw, pitch, and roll "
            "bounded response checks pass."
        )
    return PASS, "RC/manual-flight gate is proven", (
        "Proceed to camera/tracker with a known-good manual RC control baseline."
    )


def build_stage(
    name: str,
    status: str,
    summary: str,
    report: Path | None,
    notes: list[str],
    *,
    command: str = "",
    returncode: int | None = None,
    stdout: str = "",
    stderr: str = "",
) -> ManualFlightStage:
    return ManualFlightStage(
        name=name,
        status=status,
        summary=summary,
        report=str(report) if report else None,
        command=command,
        returncode=returncode,
        stdout=stdout,
        stderr=stderr,
        notes=notes,
    )


def command_block(*parts: str) -> str:
    return " \\\n  ".join(parts)


def manual_response_command(
    *,
    run_id: str,
    axis: str,
    image_axis: str,
    magnitude: float,
    min_shift_px: float,
    max_shift_px: float,
    startup_wait_s: float,
    arm_first: bool = False,
    arm_throttle: float = -0.4,
    measure: str = "visual",
    attitude_min_delta_deg: float = 10.0,
) -> str:
    parts = [
        "fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_live_session_runner.py",
        "--launch-pr0p",
        "--ack-live-launch",
        "--send-ui",
        "--ack-live-ui",
        "--hold-uinput",
        "--ack-live-input",
        "--run-live-response",
        "--response-axis %s" % axis,
        "--response-magnitude %s" % magnitude,
        "--response-image-axis %s" % image_axis,
        "--response-expected-sign 0",
        "--response-min-shift-px %s" % min_shift_px,
        "--response-max-shift-px %s" % max_shift_px,
        "--startup-wait %s" % startup_wait_s,
    ]
    if arm_first:
        parts.extend([
            "--response-arm-first",
            "--response-arm-throttle %s" % arm_throttle,
        ])
    if measure != "visual":
        parts.extend([
            "--response-measure %s" % measure,
            "--response-attitude-min-delta %s" % attitude_min_delta_deg,
        ])
    parts.append("--run-id %s-manual-%s" % (run_id, axis))
    return command_block(*parts)


def parsed_status(stdout: str, *, returncode: int) -> str:
    first_line = stdout.splitlines()[0] if stdout.splitlines() else ""
    for token in first_line.split():
        if token in {PASS, WAITING, FAIL}:
            return token
    return PASS if returncode == 0 else FAIL


def parsed_report_path(stdout: str) -> str | None:
    for token in stdout.replace("\n", " ").split():
        if token.startswith("report="):
            return token.split("=", 1)[1]
    return None


def run_command_stage(
    name: str,
    command: str,
    *,
    command_runner: Callable[..., subprocess.CompletedProcess[str]],
    timeout_s: float,
) -> ManualFlightStage:
    try:
        completed = command_runner(
            shlex.split(command.replace("\\\n", " ")),
            capture_output=True,
            text=True,
            timeout=timeout_s,
        )
    except Exception as exc:  # pragma: no cover - subprocess failures vary
        return build_stage(
            name,
            FAIL,
            "command could not be executed",
            None,
            [str(exc)],
            command=command,
        )
    status = parsed_status(completed.stdout or "", returncode=completed.returncode)
    if completed.returncode != 0 and status != WAITING:
        status = FAIL
    return build_stage(
        name,
        status,
        "command returned %s" % status,
        Path(parsed_report_path(completed.stdout or "")) if parsed_report_path(completed.stdout or "") else None,
        [],
        command=command,
        returncode=completed.returncode,
        stdout=completed.stdout or "",
        stderr=completed.stderr or "",
    )


def run_manual_flight_gate(
    *,
    run_id: str,
    log_dir: Path,
    install_root: Path,
    input_config: Path,
    expected_device: str,
    allow_physical_mapping: bool = False,
    bbox: tuple[int, int, int, int] | None,
    execute_dry_patch: bool,
    apply_config_patch: bool,
    run_live_gates: bool,
    manual_response_magnitude: float,
    manual_response_min_shift_px: float,
    manual_response_max_shift_px: float,
    manual_response_startup_wait_s: float,
    timeout_s: float,
    arm_first: bool = False,
    arm_throttle: float = -0.4,
    measure: str = "visual",
    attitude_min_delta_deg: float = 10.0,
    command_runner: Callable[..., subprocess.CompletedProcess[str]] = subprocess.run,
) -> ManualFlightReport:
    stages: list[ManualFlightStage] = []

    client = run_client_probe(
        install_root=install_root,
        max_depth=3,
        run_id="%s-client" % run_id,
    )
    client_json, _client_md = write_client_reports(
        client,
        log_dir,
        run_id="%s-client" % run_id,
    )
    stages.append(build_stage(
        "pr0p_client",
        client.status,
        client.summary,
        client_json,
        client.notes,
    ))

    input_result = run_input_config_probe(
        input_config=input_config,
        expected_device=expected_device,
        require_virtual_mapping=not allow_physical_mapping,
        allow_generic_uinput_profile=True,
    )
    input_json, _input_md = write_input_config_reports(
        input_result,
        log_dir,
        run_id="%s-primary-rc" % run_id,
    )
    stages.append(build_stage(
        "primary_rc_mapping",
        input_result.status,
        input_result.summary,
        input_json,
        input_result.notes,
    ))

    aux = run_aux1_acceptance(
        run_id="%s-aux1" % run_id,
        log_dir=log_dir,
        bbox=bbox,
        execute_dry_patch=execute_dry_patch,
        apply_config_patch=apply_config_patch,
        run_live_gates=run_live_gates,
        timeout_s=timeout_s,
        command_runner=command_runner,
    )
    aux_json, _aux_md = write_aux1_reports(aux, log_dir)
    stages.append(build_stage(
        "aux1_arm_acceptance",
        aux.status,
        aux.summary,
        aux_json,
        [],
    ))

    if aux.status == PASS and run_live_gates:
        for stage_name, axis, image_axis in MANUAL_RESPONSE_AXES:
            command = manual_response_command(
                run_id=run_id,
                axis=axis,
                image_axis=image_axis,
                magnitude=manual_response_magnitude,
                min_shift_px=manual_response_min_shift_px,
                max_shift_px=manual_response_max_shift_px,
                startup_wait_s=manual_response_startup_wait_s,
                arm_first=arm_first,
                arm_throttle=arm_throttle,
                measure=measure,
                attitude_min_delta_deg=attitude_min_delta_deg,
            )
            stages.append(run_command_stage(
                stage_name,
                command,
                command_runner=command_runner,
                timeout_s=timeout_s,
            ))
    else:
        for stage_name, axis, image_axis in MANUAL_RESPONSE_AXES:
            command = manual_response_command(
                run_id=run_id,
                axis=axis,
                image_axis=image_axis,
                magnitude=manual_response_magnitude,
                min_shift_px=manual_response_min_shift_px,
                max_shift_px=manual_response_max_shift_px,
                startup_wait_s=manual_response_startup_wait_s,
                arm_first=arm_first,
                arm_throttle=arm_throttle,
                measure=measure,
                attitude_min_delta_deg=attitude_min_delta_deg,
            )
            stages.append(build_stage(
                stage_name,
                SKIPPED if aux.status != PASS else WAITING,
                (
                    "skipped until AUX1/ARM acceptance passes"
                    if aux.status != PASS else
                    "requires --run-live-gates and live ack flags"
                ),
                None,
                ["bounded manual stick response check"],
                command=command,
            ))

    status, summary, next_action = summarize(stages)
    return ManualFlightReport(
        run_id=run_id,
        started_at=time.strftime("%Y-%m-%dT%H:%M:%S%z", time.localtime()),
        status=status,
        summary=summary,
        next_action=next_action,
        stages=stages,
        metrics={
            "bbox": list(bbox) if bbox else None,
            "install_root": str(install_root),
            "input_config": str(input_config),
            "allow_physical_mapping": allow_physical_mapping,
            "execute_dry_patch": execute_dry_patch,
            "apply_config_patch": apply_config_patch,
            "run_live_gates": run_live_gates,
            "manual_response_axes": [axis for _name, axis, _image_axis in MANUAL_RESPONSE_AXES],
            "manual_response_magnitude": manual_response_magnitude,
            "manual_response_min_shift_px": manual_response_min_shift_px,
            "manual_response_max_shift_px": manual_response_max_shift_px,
            "manual_response_startup_wait_s": manual_response_startup_wait_s,
            "arm_first": arm_first,
            "arm_throttle": arm_throttle,
            "measure": measure,
            "attitude_min_delta_deg": attitude_min_delta_deg,
            "evidence_scope": "rc_manual_flight",
            "real_config_write_requested": apply_config_patch,
            "real_input_requested": run_live_gates,
        },
    )


def build_markdown(report: ManualFlightReport) -> str:
    lines = [
        "# pr0p RC Manual Flight Gate",
        "",
        "Run: `%s`" % report.run_id,
        "Started: `%s`" % report.started_at,
        "Status: `%s`" % report.status,
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
    lines.extend(["", "## Commands", ""])
    for stage in report.stages:
        if stage.command:
            lines.extend(["### %s" % stage.name, "", "```bash", stage.command, "```", ""])
    lines.extend(["", "## Metrics", "", "```json"])
    lines.append(json.dumps(report.metrics, indent=2, sort_keys=True))
    lines.extend(["```", ""])
    return "\n".join(lines)


def write_reports(report: ManualFlightReport, log_dir: Path) -> tuple[Path, Path]:
    log_dir.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime("%Y%m%d-%H%M%S", time.localtime())
    json_path = log_dir / ("%s-%s-rc-manual-flight.json" % (stamp, report.run_id))
    md_path = log_dir / ("%s-%s-rc-manual-flight.md" % (stamp, report.run_id))
    json_path.write_text(json.dumps(report.as_dict(), indent=2, sort_keys=True) + "\n",
                         encoding="utf-8")
    md_path.write_text(build_markdown(report), encoding="utf-8")
    return json_path, md_path


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-id", default="pr0p-rc-manual-flight")
    parser.add_argument("--log-dir", type=Path, default=DEFAULT_LOG_DIR)
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
    parser.add_argument("--manual-response-magnitude", type=float, default=0.05)
    parser.add_argument("--manual-response-min-shift-px", type=float, default=2.0)
    parser.add_argument("--manual-response-max-shift-px", type=float, default=200.0)
    parser.add_argument("--manual-response-startup-wait", type=float, default=8.0)
    parser.add_argument("--timeout", type=float, default=180.0)
    parser.add_argument("--arm-first", action="store_true",
                        help="Arm the FC and hold hover throttle during each manual "
                             "response pulse")
    parser.add_argument("--arm-throttle", type=float, default=-0.4)
    parser.add_argument("--measure", default="visual", choices=("visual", "attitude"),
                        help="attitude reads signed MSP_ATTITUDE deltas; use it when the "
                             "FPV view is sky-dominated")
    parser.add_argument("--attitude-min-delta", type=float, default=10.0)
    args = parser.parse_args(argv)
    if args.timeout <= 0:
        parser.error("--timeout must be positive")
    if args.manual_response_magnitude <= 0:
        parser.error("--manual-response-magnitude must be positive")
    if args.manual_response_min_shift_px < 0:
        parser.error("--manual-response-min-shift-px must be non-negative")
    if args.manual_response_max_shift_px <= 0:
        parser.error("--manual-response-max-shift-px must be positive")
    if args.manual_response_min_shift_px > args.manual_response_max_shift_px:
        parser.error("--manual-response-min-shift-px must be <= --manual-response-max-shift-px")
    if args.manual_response_startup_wait < 0:
        parser.error("--manual-response-startup-wait must be non-negative")
    if not -1.0 <= args.arm_throttle <= 1.0:
        parser.error("--arm-throttle must be within [-1.0, 1.0]")
    if args.attitude_min_delta <= 0:
        parser.error("--attitude-min-delta must be positive")
    if args.apply_config_patch and not args.ack_config_write:
        parser.error("--apply-config-patch requires --ack-config-write")
    if args.run_live_gates and not (
        args.ack_live_launch and args.ack_live_ui and args.ack_live_input
    ):
        parser.error("--run-live-gates requires --ack-live-launch --ack-live-ui --ack-live-input")
    return args


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    report = run_manual_flight_gate(
        run_id=args.run_id,
        log_dir=args.log_dir,
        install_root=args.install_root,
        input_config=args.input_config,
        expected_device=args.expected_device,
        allow_physical_mapping=args.allow_physical_mapping,
        bbox=args.bbox,
        execute_dry_patch=args.execute_dry_patch,
        apply_config_patch=args.apply_config_patch,
        run_live_gates=args.run_live_gates,
        manual_response_magnitude=args.manual_response_magnitude,
        manual_response_min_shift_px=args.manual_response_min_shift_px,
        manual_response_max_shift_px=args.manual_response_max_shift_px,
        manual_response_startup_wait_s=args.manual_response_startup_wait,
        timeout_s=args.timeout,
        arm_first=args.arm_first,
        arm_throttle=args.arm_throttle,
        measure=args.measure,
        attitude_min_delta_deg=args.attitude_min_delta,
    )
    json_path, md_path = write_reports(report, args.log_dir)
    print("simitl-pr0p-rc-manual-flight %s report=%s summary=%s" % (
        report.status,
        json_path,
        md_path,
    ))
    print(report.next_action)
    return 1 if report.status == FAIL else 0


if __name__ == "__main__":
    raise SystemExit(main())
