#!/usr/bin/env python3
"""Drive the minimal pr0p local time-attack menu path with XTEST clicks."""

from __future__ import annotations

import argparse
import ctypes
import ctypes.util
import json
import re
import subprocess
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any


PASS = "PASS"
WAITING = "WAITING"
FAIL = "FAIL"
DEFAULT_LOG_DIR = Path("logs/simitl_pr0p")
BASE_WIDTH = 828
BASE_HEIGHT = 666


@dataclass(frozen=True)
class WindowGeometry:
    x: int
    y: int
    width: int
    height: int

    def scale_point(self, base_x: int, base_y: int) -> tuple[int, int]:
        return (
            self.x + round(base_x * self.width / BASE_WIDTH),
            self.y + round(base_y * self.height / BASE_HEIGHT),
        )


@dataclass
class UiClick:
    label: str
    base_x: int
    base_y: int
    wait_s: float

    def as_dict(self, geometry: WindowGeometry) -> dict[str, Any]:
        x, y = geometry.scale_point(self.base_x, self.base_y)
        return {
            "label": self.label,
            "base_x": self.base_x,
            "base_y": self.base_y,
            "screen_x": x,
            "screen_y": y,
            "wait_s": self.wait_s,
        }


@dataclass
class UiSmokeResult:
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


DEFAULT_LOCAL_TIME_ATTACK_CLICKS = (
    UiClick("skip-login", 450, 355, 1.2),
    UiClick("local-menu", 55, 69, 0.7),
    UiClick("time-attack", 260, 181, 0.7),
    UiClick("first-quad", 75, 120, 0.4),
    UiClick("quad-ok", 254, 626, 0.8),
    UiClick("first-scene", 52, 120, 0.4),
    UiClick("scene-ok", 254, 626, 0.8),
    UiClick("first-track", 80, 120, 0.4),
    UiClick("track-ok", 254, 626, 1.0),
)


def parse_xwininfo_geometry(text: str) -> WindowGeometry | None:
    x_match = re.search(r"Absolute upper-left X:\s*(-?\d+)", text)
    y_match = re.search(r"Absolute upper-left Y:\s*(-?\d+)", text)
    width_match = re.search(r"Width:\s*(\d+)", text)
    height_match = re.search(r"Height:\s*(\d+)", text)
    if not (x_match and y_match and width_match and height_match):
        return None
    return WindowGeometry(
        x=int(x_match.group(1)),
        y=int(y_match.group(1)),
        width=int(width_match.group(1)),
        height=int(height_match.group(1)),
    )


def query_window_geometry(window_title: str, *, timeout: float) -> WindowGeometry | None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        proc = subprocess.run(
            ["xwininfo", "-name", window_title],
            check=False,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            timeout=2.0,
        )
        if proc.returncode == 0:
            geometry = parse_xwininfo_geometry(proc.stdout)
            if geometry is not None:
                return geometry
        time.sleep(0.25)
    return None


def planned_clicks(geometry: WindowGeometry) -> list[dict[str, Any]]:
    return [click.as_dict(geometry) for click in DEFAULT_LOCAL_TIME_ATTACK_CLICKS]


def send_xtest_clicks(clicks: list[dict[str, Any]]) -> None:
    x11 = ctypes.CDLL(ctypes.util.find_library("X11"))
    xtst = ctypes.CDLL(ctypes.util.find_library("Xtst"))
    x11.XOpenDisplay.argtypes = [ctypes.c_char_p]
    x11.XOpenDisplay.restype = ctypes.c_void_p
    x11.XFlush.argtypes = [ctypes.c_void_p]
    xtst.XTestFakeMotionEvent.argtypes = [
        ctypes.c_void_p,
        ctypes.c_int,
        ctypes.c_int,
        ctypes.c_int,
        ctypes.c_ulong,
    ]
    xtst.XTestFakeButtonEvent.argtypes = [
        ctypes.c_void_p,
        ctypes.c_uint,
        ctypes.c_int,
        ctypes.c_ulong,
    ]
    display = x11.XOpenDisplay(None)
    if not display:
        raise RuntimeError("XOpenDisplay failed")
    for click in clicks:
        x = int(click["screen_x"])
        y = int(click["screen_y"])
        xtst.XTestFakeMotionEvent(display, -1, x, y, 0)
        x11.XFlush(display)
        time.sleep(0.08)
        xtst.XTestFakeButtonEvent(display, 1, 1, 0)
        x11.XFlush(display)
        time.sleep(0.05)
        xtst.XTestFakeButtonEvent(display, 1, 0, 0)
        x11.XFlush(display)
        time.sleep(float(click["wait_s"]))


def run_ui_smoke(
    *,
    window_title: str,
    timeout: float,
    send_ui: bool,
) -> UiSmokeResult:
    geometry = query_window_geometry(window_title, timeout=timeout)
    if geometry is None:
        return UiSmokeResult(
            status=WAITING,
            summary="pr0p window is not visible yet",
            metrics={"window_title": window_title},
            notes=["PR0P_WINDOW_NOT_FOUND"],
        )
    clicks = planned_clicks(geometry)
    metrics = {
        "window_title": window_title,
        "geometry": {
            "x": geometry.x,
            "y": geometry.y,
            "width": geometry.width,
            "height": geometry.height,
        },
        "clicks": clicks,
        "real_ui_input_sent": send_ui,
    }
    if not send_ui:
        return UiSmokeResult(
            status=WAITING,
            summary="local time-attack UI path is planned but not sent",
            metrics=metrics,
            notes=["DRY_RUN_ONLY", "rerun with --send-ui --ack-live-ui to click pr0p"],
        )
    try:
        send_xtest_clicks(clicks)
    except Exception as exc:
        return UiSmokeResult(
            status=FAIL,
            summary="failed to send XTEST UI clicks",
            metrics={**metrics, "error": str(exc)},
            notes=["XTEST_CLICK_FAIL"],
        )
    return UiSmokeResult(
        status=PASS,
        summary="local time-attack UI click sequence was sent",
        metrics=metrics,
        notes=[
            "this only navigates menus; run P2-msp-readonly to prove the virtual FC opened",
            "real UI input was sent to the desktop",
        ],
    )


def build_markdown(result: UiSmokeResult, *, run_id: str) -> str:
    lines = [
        "# SimITL / pr0p UI Smoke",
        "",
        "Run: `%s`" % run_id,
        "Verdict: `%s`" % result.status,
        "",
        result.summary,
        "",
        "```json",
        json.dumps(result.metrics, indent=2, sort_keys=True),
        "```",
    ]
    for note in result.notes:
        lines.append("- %s" % note)
    lines.append("")
    return "\n".join(lines)


def write_reports(result: UiSmokeResult, log_dir: Path, *, run_id: str) -> tuple[Path, Path]:
    log_dir.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime("%Y%m%d-%H%M%S", time.localtime())
    json_path = log_dir / ("%s-%s-ui-smoke.json" % (stamp, run_id))
    md_path = log_dir / ("%s-%s-ui-smoke.md" % (stamp, run_id))
    json_path.write_text(json.dumps(result.as_dict(), indent=2, sort_keys=True) + "\n",
                         encoding="utf-8")
    md_path.write_text(build_markdown(result, run_id=run_id), encoding="utf-8")
    return json_path, md_path


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--window-title", default="pr0p")
    parser.add_argument("--timeout", type=float, default=5.0)
    parser.add_argument("--send-ui", action="store_true",
                        help="Send real XTEST clicks to the desktop")
    parser.add_argument("--ack-live-ui", action="store_true",
                        help="Required before --send-ui can send real UI input")
    parser.add_argument("--log-dir", type=Path, default=DEFAULT_LOG_DIR)
    parser.add_argument("--run-id", default="pr0p-ui-smoke")
    args = parser.parse_args(argv)
    if args.timeout <= 0:
        parser.error("--timeout must be positive")
    if args.send_ui and not args.ack_live_ui:
        parser.error("--send-ui requires --ack-live-ui")
    return args


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    result = run_ui_smoke(
        window_title=args.window_title,
        timeout=args.timeout,
        send_ui=args.send_ui,
    )
    json_path, md_path = write_reports(result, args.log_dir, run_id=args.run_id)
    print("simitl-pr0p-ui-smoke %s report=%s summary=%s" % (
        result.status,
        json_path,
        md_path,
    ))
    print(result.summary)
    return 1 if result.status == FAIL else 0


if __name__ == "__main__":
    raise SystemExit(main())
