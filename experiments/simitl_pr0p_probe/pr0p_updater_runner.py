#!/usr/bin/env python3
"""Plan or bound a pr0p updater launch from the isolated install root."""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable

PROBE_DIR = Path(__file__).resolve().parent
if str(PROBE_DIR) not in sys.path:
    sys.path.insert(0, str(PROBE_DIR))

from preflight_probe import DEFAULT_INSTALL_ROOT, DEFAULT_LOG_DIR  # noqa: E402
from pr0p_client_probe import find_client_candidates  # noqa: E402
from pr0p_install_probe import is_allowed_install_root, scan_install_root  # noqa: E402


PASS = "PASS"
WAITING = "WAITING"
FAIL = "FAIL"
UPDATER_NAME = "updater"
PROCESS_PATTERN = r"/tmp/fpv-test-simitl-pr0p/updater|(^|/)updater($| )"


@dataclass
class UpdaterRunResult:
    status: str
    summary: str
    metrics: dict[str, Any] = field(default_factory=dict)
    notes: list[str] = field(default_factory=list)

    def as_dict(self) -> dict[str, Any]:
        return {
            "status": self.status,
            "summary": self.summary,
            "metrics": self.metrics,
            "notes": self.notes,
        }


def find_updater(install_root: Path) -> Path | None:
    candidate = install_root / UPDATER_NAME
    if candidate.is_file() and os.access(candidate, os.X_OK):
        return candidate
    return None


def list_updater_processes(pattern: str = PROCESS_PATTERN) -> list[str]:
    proc = subprocess.run(
        ["pgrep", "-af", pattern],
        check=False,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        timeout=2.0,
    )
    if proc.returncode not in (0, 1):
        return ["PGREP_ERROR: %s" % (proc.stderr.strip() or proc.stdout.strip())]
    return [line for line in proc.stdout.splitlines() if line.strip()]


def list_updater_windows(pattern: str = "updater") -> list[dict[str, Any]]:
    try:
        proc = subprocess.run(
            ["xwininfo", "-root", "-tree"],
            check=False,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            timeout=3.0,
        )
    except (OSError, subprocess.SubprocessError) as exc:
        return [{"error": str(exc)}]
    if proc.returncode != 0:
        return [{"error": proc.stderr.strip() or proc.stdout.strip()}]
    matches: list[dict[str, Any]] = []
    lowered_pattern = pattern.lower()
    for line in proc.stdout.splitlines():
        if lowered_pattern not in line.lower():
            continue
        stripped = line.strip()
        parts = stripped.split(maxsplit=1)
        matches.append({
            "window_id": parts[0] if parts else None,
            "line": stripped,
        })
    return matches


def cleanup_process(process: Any, *, timeout: float) -> bool:
    if process is None or process.poll() is not None:
        return True
    try:
        process.terminate()
        process.wait(timeout=timeout)
        return process.poll() is not None
    except subprocess.TimeoutExpired:
        process.kill()
        try:
            process.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            return False
        return process.poll() is not None


def new_process_lines(before: list[str], after: list[str]) -> list[str]:
    before_set = set(before)
    return [line for line in after if line not in before_set]


def install_snapshot(install_root: Path) -> dict[str, Any]:
    return {
        "install_scan": scan_install_root(install_root),
        "client_candidates": find_client_candidates(install_root, max_depth=3),
    }


def run_updater(
    *,
    run_id: str,
    log_dir: Path,
    install_root: Path,
    launch_updater: bool,
    leave_running: bool,
    startup_wait_s: float,
    cleanup_timeout_s: float,
    popen_factory: Callable[..., Any] = subprocess.Popen,
    process_lister: Callable[[], list[str]] = list_updater_processes,
    window_lister: Callable[[], list[dict[str, Any]]] = list_updater_windows,
) -> UpdaterRunResult:
    log_dir.mkdir(parents=True, exist_ok=True)
    before_processes = process_lister()
    before_windows = window_lister()
    before_snapshot = install_snapshot(install_root) if install_root.exists() else {
        "install_scan": {"exists": False},
        "client_candidates": [],
    }
    metrics: dict[str, Any] = {
        "run_id": run_id,
        "install_root": str(install_root),
        "updater": None,
        "planned_command": None,
        "real_launch": False,
        "leave_running": leave_running,
        "cleanup_performed": False,
        "cleanup_ok": True,
        "processes_before": before_processes,
        "windows_before": before_windows,
        "before": before_snapshot,
        "after": None,
    }
    notes: list[str] = []

    if not is_allowed_install_root(install_root):
        metrics["processes_after"] = process_lister()
        metrics["windows_after"] = window_lister()
        return UpdaterRunResult(
            status=FAIL,
            summary="install root is outside the allowed isolated roots",
            metrics=metrics,
            notes=["REFUSE_NON_ISOLATED_INSTALL_ROOT"],
        )

    updater = find_updater(install_root)
    if updater is None:
        metrics["processes_after"] = process_lister()
        metrics["windows_after"] = window_lister()
        return UpdaterRunResult(
            status=WAITING,
            summary="Linux updater is not present in the isolated root",
            metrics=metrics,
            notes=["RUN_INSTALL_DOWNLOAD_FIRST"],
        )

    metrics["updater"] = str(updater)
    metrics["planned_command"] = [str(updater)]
    if not launch_updater:
        metrics["processes_after"] = process_lister()
        metrics["windows_after"] = window_lister()
        metrics["after"] = install_snapshot(install_root)
        return UpdaterRunResult(
            status=PASS,
            summary="bounded updater launch plan generated without executing external binary",
            metrics=metrics,
            notes=["DRY_RUN_ONLY", "rerun with --launch-updater --ack-external-binary to execute"],
        )

    stdout_path = log_dir / ("%s-updater-stdout.log" % run_id)
    stderr_path = log_dir / ("%s-updater-stderr.log" % run_id)
    process = None
    status = PASS
    try:
        with stdout_path.open("wb") as stdout, stderr_path.open("wb") as stderr:
            process = popen_factory(
                [str(updater)],
                cwd=str(install_root),
                stdout=stdout,
                stderr=stderr,
                start_new_session=True,
            )
            metrics["real_launch"] = True
            metrics["pid"] = getattr(process, "pid", None)
            metrics["stdout_log"] = str(stdout_path)
            metrics["stderr_log"] = str(stderr_path)
            time.sleep(startup_wait_s)
            metrics["returncode_after_startup"] = process.poll()
            if process.poll() not in (None, 0):
                status = FAIL
                notes.append("UPDATER_EXITED_WITH_ERROR")
    except OSError as exc:
        status = FAIL
        metrics["launch_error"] = str(exc)
        notes.append("UPDATER_LAUNCH_FAIL")
    finally:
        if process is not None and not leave_running:
            metrics["cleanup_performed"] = True
            metrics["cleanup_ok"] = cleanup_process(process, timeout=cleanup_timeout_s)
            if not metrics["cleanup_ok"]:
                status = FAIL
                notes.append("PROCESS_CLEANUP_FAIL")
        elif process is not None:
            notes.append("PROCESS_LEFT_RUNNING_BY_ACK")
        after_processes = process_lister()
        metrics["processes_after"] = after_processes
        metrics["windows_after"] = window_lister()
        metrics["new_processes_after"] = new_process_lines(before_processes, after_processes)
        metrics["after"] = install_snapshot(install_root)

    if status == PASS:
        summary = (
            "updater was launched and left running by explicit acknowledgement"
            if leave_running else
            "bounded updater launch completed and cleanup was attempted"
        )
    else:
        summary = "bounded updater launch failed"
    return UpdaterRunResult(status=status, summary=summary, metrics=metrics, notes=notes)


def build_markdown(result: UpdaterRunResult, *, run_id: str) -> str:
    lines = [
        "# SimITL / pr0p Updater Runner",
        "",
        "Run: `%s`" % run_id,
        "Verdict: `%s`" % result.status,
        "",
        result.summary,
        "",
        "| Metric | Value |",
        "| --- | --- |",
    ]
    for key in ("updater", "real_launch", "leave_running", "cleanup_performed", "cleanup_ok"):
        lines.append("| %s | `%s` |" % (key, result.metrics.get(key)))
    after = result.metrics.get("after") or {}
    lines.append("| client_candidate_count_after | `%s` |" % len(after.get("client_candidates") or []))
    lines.append("| updater_window_count_after | `%s` |" % len(result.metrics.get("windows_after") or []))
    lines.extend([
        "",
        "## Metrics",
        "",
        "```json",
        json.dumps(result.metrics, indent=2, sort_keys=True),
        "```",
    ])
    for note in result.notes:
        lines.append("- %s" % note)
    lines.append("")
    return "\n".join(lines)


def write_reports(result: UpdaterRunResult, log_dir: Path, *, run_id: str) -> tuple[Path, Path]:
    log_dir.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime("%Y%m%d-%H%M%S", time.localtime())
    json_path = log_dir / ("%s-%s-updater-runner.json" % (stamp, run_id))
    md_path = log_dir / ("%s-%s-updater-runner.md" % (stamp, run_id))
    json_path.write_text(
        json.dumps(result.as_dict(), indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    md_path.write_text(build_markdown(result, run_id=run_id), encoding="utf-8")
    return json_path, md_path


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-id", default="pr0p-updater")
    parser.add_argument("--install-root", type=Path, default=DEFAULT_INSTALL_ROOT)
    parser.add_argument("--log-dir", type=Path, default=DEFAULT_LOG_DIR)
    parser.add_argument("--launch-updater", action="store_true",
                        help="Execute the updater binary from the isolated install root")
    parser.add_argument("--ack-external-binary", action="store_true",
                        help="Required before --launch-updater can execute the downloaded binary")
    parser.add_argument("--leave-running", action="store_true",
                        help="Do not terminate the started updater process after startup wait")
    parser.add_argument("--ack-leave-running", action="store_true",
                        help="Required with --leave-running")
    parser.add_argument("--startup-wait", type=float, default=3.0)
    parser.add_argument("--cleanup-timeout", type=float, default=3.0)
    args = parser.parse_args(argv)
    if args.launch_updater and not args.ack_external_binary:
        parser.error("--launch-updater requires --ack-external-binary")
    if args.leave_running and not args.launch_updater:
        parser.error("--leave-running requires --launch-updater")
    if args.leave_running and not args.ack_leave_running:
        parser.error("--leave-running requires --ack-leave-running")
    if args.startup_wait < 0:
        parser.error("--startup-wait must be non-negative")
    if args.cleanup_timeout <= 0:
        parser.error("--cleanup-timeout must be positive")
    return args


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    result = run_updater(
        run_id=args.run_id,
        log_dir=args.log_dir,
        install_root=args.install_root,
        launch_updater=args.launch_updater,
        leave_running=args.leave_running,
        startup_wait_s=args.startup_wait,
        cleanup_timeout_s=args.cleanup_timeout,
    )
    json_path, md_path = write_reports(result, args.log_dir, run_id=args.run_id)
    print("simitl-pr0p-updater %s report=%s summary=%s" % (
        result.status,
        json_path,
        md_path,
    ))
    print(result.summary)
    return 1 if result.status == FAIL else 0


if __name__ == "__main__":
    raise SystemExit(main())
