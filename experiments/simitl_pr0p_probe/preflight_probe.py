#!/usr/bin/env python3
"""Preflight checks for the isolated SimITL/pr0p probe."""

from __future__ import annotations

import argparse
import importlib.util
import json
import os
import platform
import shutil
import socket
import subprocess
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any


PASS = "PASS"
WAITING = "WAITING"
FAIL = "FAIL"
DEFAULT_INSTALL_ROOT = Path("/tmp/fpv-test-simitl-pr0p")
DEFAULT_LOG_DIR = Path("logs/simitl_pr0p")
DEFAULT_WS_HOST = "127.0.0.1"
DEFAULT_WS_PORT = 5761


@dataclass
class ProbeCheck:
    name: str
    status: str
    summary: str
    metrics: dict[str, Any] = field(default_factory=dict)
    notes: list[str] = field(default_factory=list)

    def as_dict(self) -> dict[str, Any]:
        return {
            "name": self.name,
            "status": self.status,
            "summary": self.summary,
            "metrics": self.metrics,
            "notes": self.notes,
        }


@dataclass
class PreflightReport:
    run_id: str
    started_at: str
    install_root: str
    checks: list[ProbeCheck]

    @property
    def status(self) -> str:
        if any(check.status == FAIL for check in self.checks):
            return FAIL
        if any(check.status == WAITING for check in self.checks):
            return WAITING
        return PASS

    def as_dict(self) -> dict[str, Any]:
        return {
            "run_id": self.run_id,
            "started_at": self.started_at,
            "install_root": self.install_root,
            "status": self.status,
            "checks": [check.as_dict() for check in self.checks],
        }


def module_available(name: str) -> bool:
    return importlib.util.find_spec(name) is not None


def command_output(argv: list[str], *, timeout: float = 2.0) -> tuple[int, str, str]:
    try:
        proc = subprocess.run(
            argv,
            check=False,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            timeout=timeout,
        )
    except FileNotFoundError as exc:
        return 127, "", str(exc)
    except subprocess.TimeoutExpired as exc:
        return 124, exc.stdout or "", exc.stderr or "timeout"
    return proc.returncode, proc.stdout, proc.stderr


def probe_display() -> ProbeCheck:
    xdg_session = os.environ.get("XDG_SESSION_TYPE")
    display = os.environ.get("DISPLAY")
    wayland_display = os.environ.get("WAYLAND_DISPLAY")
    ffmpeg = shutil.which("ffmpeg")
    xwininfo = shutil.which("xwininfo")
    glxinfo = shutil.which("glxinfo")
    glx_summary = None
    if glxinfo:
        code, stdout, stderr = command_output([glxinfo, "-B"], timeout=3.0)
        glx_summary = stdout.splitlines()[:12] if code == 0 else (stderr or stdout).splitlines()[:8]
    has_capture_candidate = bool((display and ffmpeg) or module_available("mss"))
    status = PASS if (display or wayland_display) and has_capture_candidate else WAITING
    return ProbeCheck(
        name="display_capture",
        status=status,
        summary=(
            "display and capture prerequisites look usable"
            if status == PASS else
            "display/capture prerequisites are incomplete"
        ),
        metrics={
            "platform": platform.platform(),
            "python": sys.executable,
            "xdg_session_type": xdg_session,
            "display": display,
            "wayland_display": wayland_display,
            "ffmpeg": ffmpeg,
            "xwininfo": xwininfo,
            "mss_available": module_available("mss"),
            "glxinfo": glxinfo,
            "glx_summary": glx_summary,
        },
        notes=["X11 + ffmpeg/x11grab is the preferred first capture path"],
    )


def probe_input() -> ProbeCheck:
    dev_uinput = Path("/dev/uinput")
    exists = dev_uinput.exists()
    writable = os.access(dev_uinput, os.W_OK) if exists else False
    evdev_available = module_available("evdev")
    status = PASS if exists and writable and evdev_available else WAITING
    return ProbeCheck(
        name="virtual_input",
        status=status,
        summary=(
            "uinput virtual joystick prerequisites look usable"
            if status == PASS else
            "uinput virtual joystick prerequisites are incomplete"
        ),
        metrics={
            "evdev_available": evdev_available,
            "dev_uinput_exists": exists,
            "dev_uinput_writable": writable,
        },
        notes=["physical joystick is optional; uinput is preferred for repeatable probes"],
    )


def probe_install_root(path: Path) -> ProbeCheck:
    try:
        path.mkdir(parents=True, exist_ok=True)
        writable = os.access(path, os.W_OK)
    except OSError as exc:
        return ProbeCheck(
            name="install_root",
            status=FAIL,
            summary="isolated install root could not be created",
            metrics={"path": str(path), "error": str(exc)},
        )
    contents = sorted(item.name for item in path.iterdir())[:20]
    return ProbeCheck(
        name="install_root",
        status=PASS if writable else FAIL,
        summary="isolated install root is ready" if writable else "isolated install root is not writable",
        metrics={
            "path": str(path),
            "writable": writable,
            "contents_sample": contents,
        },
        notes=["do not commit downloaded pr0p/SimITL binaries into the repo"],
    )


def probe_processes() -> ProbeCheck:
    code, stdout, stderr = command_output(["pgrep", "-af", "pr0p|SimITL|simitl"], timeout=2.0)
    lines = [line for line in stdout.splitlines() if line.strip()]
    status = PASS
    return ProbeCheck(
        name="processes",
        status=status,
        summary="process probe completed",
        metrics={
            "matches": lines,
            "pgrep_code": code,
            "stderr": stderr.strip(),
        },
        notes=["matches are informational; stale processes should be stopped before a measured run"],
    )


def probe_ws_port(host: str, port: int, *, timeout: float) -> ProbeCheck:
    try:
        with socket.create_connection((host, port), timeout=timeout):
            reachable = True
            error = None
    except OSError as exc:
        reachable = False
        error = str(exc)
    return ProbeCheck(
        name="msp_websocket_port",
        status=PASS if reachable else WAITING,
        summary=(
            "pr0p/virtual-FC websocket port is reachable"
            if reachable else
            "pr0p/virtual-FC websocket port is not reachable yet"
        ),
        metrics={
            "host": host,
            "port": port,
            "reachable": reachable,
            "error": error,
        },
        notes=["expected only while a pr0p local race is running"],
    )


def run_preflight(
    *,
    install_root: Path,
    ws_host: str,
    ws_port: int,
    timeout: float,
    run_id: str,
) -> PreflightReport:
    checks = [
        probe_display(),
        probe_input(),
        probe_install_root(install_root),
        probe_processes(),
        probe_ws_port(ws_host, ws_port, timeout=timeout),
    ]
    return PreflightReport(
        run_id=run_id,
        started_at=time.strftime("%Y-%m-%dT%H:%M:%S%z", time.localtime()),
        install_root=str(install_root),
        checks=checks,
    )


def build_markdown(report: PreflightReport) -> str:
    lines = [
        "# SimITL / pr0p Preflight Report",
        "",
        "Run: `%s`" % report.run_id,
        "Started: `%s`" % report.started_at,
        "Install root: `%s`" % report.install_root,
        "Verdict: `%s`" % report.status,
        "",
        "| Check | Status | Summary |",
        "| --- | --- | --- |",
    ]
    for check in report.checks:
        lines.append("| %s | %s | %s |" % (check.name, check.status, check.summary))
    lines.append("")
    lines.append("## Metrics")
    for check in report.checks:
        lines.extend([
            "",
            "### %s" % check.name,
            "",
            "```json",
            json.dumps(check.metrics, indent=2, sort_keys=True),
            "```",
        ])
        for note in check.notes:
            lines.append("- %s" % note)
    lines.append("")
    return "\n".join(lines)


def write_reports(report: PreflightReport, log_dir: Path) -> tuple[Path, Path]:
    log_dir.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime("%Y%m%d-%H%M%S", time.localtime())
    json_path = log_dir / ("%s-%s-preflight.json" % (stamp, report.run_id))
    md_path = log_dir / ("%s-%s-preflight.md" % (stamp, report.run_id))
    json_path.write_text(json.dumps(report.as_dict(), indent=2, sort_keys=True) + "\n",
                         encoding="utf-8")
    md_path.write_text(build_markdown(report), encoding="utf-8")
    return json_path, md_path


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--install-root", type=Path, default=DEFAULT_INSTALL_ROOT)
    parser.add_argument("--ws-host", default=DEFAULT_WS_HOST)
    parser.add_argument("--ws-port", type=int, default=DEFAULT_WS_PORT)
    parser.add_argument("--timeout", type=float, default=0.5)
    parser.add_argument("--log-dir", type=Path, default=DEFAULT_LOG_DIR)
    parser.add_argument("--run-id", default="pr0p-preflight")
    args = parser.parse_args(argv)
    if args.ws_port <= 0:
        parser.error("--ws-port must be positive")
    if args.timeout <= 0:
        parser.error("--timeout must be positive")
    return args


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    report = run_preflight(
        install_root=args.install_root,
        ws_host=args.ws_host,
        ws_port=args.ws_port,
        timeout=args.timeout,
        run_id=args.run_id,
    )
    json_path, md_path = write_reports(report, args.log_dir)
    print("simitl-pr0p-preflight %s report=%s summary=%s" % (
        report.status, json_path, md_path))
    for check in report.checks:
        print("%s %s - %s" % (check.name, check.status, check.summary))
    return 1 if report.status == FAIL else 0


if __name__ == "__main__":
    raise SystemExit(main())
