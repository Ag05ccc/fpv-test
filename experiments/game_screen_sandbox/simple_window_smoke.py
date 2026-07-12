#!/usr/bin/env python3
"""One-command real-window smoke for the isolated game/screen sandbox."""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
import time
from pathlib import Path
from typing import Any

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))
TOOLS_DIR = REPO_ROOT / "tools"
if str(TOOLS_DIR) not in sys.path:
    sys.path.insert(0, str(TOOLS_DIR))

from phase_runner import run_synthetic_gates, write_reports  # noqa: E402
from simple_target_game import GameConfig, initial_state, target_screen_bbox  # noqa: E402
from sitl_log import resolve_log_dir, timestamp_slug  # noqa: E402
from x11_window import find_window_by_title, list_x11_windows  # noqa: E402

PASS = "PASS"
FAIL = "FAIL"
WAITING = "WAITING"
WINDOW_TITLE = "Kenet Simple Target Game"


def static_target_bbox(width: int = 640, height: int = 480) -> tuple[int, int, int, int]:
    config = GameConfig(
        width=width,
        height=height,
        target_motion_radius_x=0.0,
        target_motion_radius_y=0.0,
    )
    return target_screen_bbox(initial_state(config), config)


def game_command(
    *,
    duration_s: float,
    fps: float,
    width: int,
    height: int,
    report_path: Path,
) -> list[str]:
    return [
        sys.executable,
        str(Path(__file__).with_name("simple_target_game.py")),
        "--show",
        "--duration", str(duration_s),
        "--fps", str(fps),
        "--width", str(width),
        "--height", str(height),
        "--static-target",
        "--report-path", str(report_path),
    ]


def wait_for_window(
    *,
    title: str,
    preferred_size: tuple[int, int],
    timeout_s: float,
    poll_s: float = 0.2,
) -> dict[str, int] | None:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        try:
            match = find_window_by_title(
                title,
                list_x11_windows(),
                preferred_size=preferred_size,
            )
        except Exception:
            match = None
        if match is not None:
            return match.region()
        time.sleep(poll_s)
    return None


def finish_process(
    proc: subprocess.Popen[str],
    *,
    terminate: bool,
    timeout_s: float = 3.0,
) -> dict[str, Any]:
    if terminate and proc.poll() is None:
        proc.terminate()
    try:
        stdout, stderr = proc.communicate(timeout=timeout_s)
    except subprocess.TimeoutExpired:
        proc.kill()
        stdout, stderr = proc.communicate(timeout=3.0)
    return {
        "returncode": proc.returncode,
        "stdout_tail": "\n".join(stdout.splitlines()[-5:]),
        "stderr_tail": "\n".join(stderr.splitlines()[-5:]),
    }


def run_smoke(
    *,
    run_id: str,
    log_dir: Path,
    width: int,
    height: int,
    game_duration_s: float,
    game_fps: float,
    window_timeout_s: float,
    gate_duration_s: float,
    gate_hz: float,
    min_fps: float,
) -> dict[str, Any]:
    log_dir.mkdir(parents=True, exist_ok=True)
    slug = timestamp_slug()
    game_report_path = log_dir / ("%s-%s-simple-window-game.json" % (slug, run_id))
    proc = subprocess.Popen(
        game_command(
            duration_s=game_duration_s,
            fps=game_fps,
            width=width,
            height=height,
            report_path=game_report_path,
        ),
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )
    report: dict[str, Any]
    terminate_game = False
    try:
        region = wait_for_window(
            title=WINDOW_TITLE,
            preferred_size=(width, height),
            timeout_s=window_timeout_s,
        )
        if region is None:
            terminate_game = True
            report = {
                "status": WAITING,
                "run_id": run_id,
                "summary": "simple target window did not appear",
                "window_title": WINDOW_TITLE,
                "preferred_size": [width, height],
                "game_report_path": str(game_report_path),
            }
            return report

        bbox = static_target_bbox(width, height)
        phase_report = run_synthetic_gates(
            duration_s=gate_duration_s,
            hz=gate_hz,
            min_fps=min_fps,
            min_tracker_found_ratio=0.90,
            min_loop_found_ratio=0.85,
            log_dir=log_dir,
            run_id=run_id,
            capture_region=region,
            capture_backend="ffmpeg",
            tracker_bbox=bbox,
            include_real_combined=True,
            desired_target_width=120.0,
        )
        phase_json_path, phase_md_path = write_reports(phase_report, log_dir)
        status = phase_report.status
        report = {
            "status": status,
            "run_id": run_id,
            "summary": "simple target window capture smoke %s" % status,
            "window_title": WINDOW_TITLE,
            "window_region": region,
            "tracker_bbox": list(bbox),
            "game_report_path": str(game_report_path),
            "phase_report_json": str(phase_json_path),
            "phase_report_md": str(phase_md_path),
        }
        return report
    finally:
        process = finish_process(
            proc,
            terminate=terminate_game,
            timeout_s=max(3.0, game_duration_s + 1.0),
        )
        if "report" in locals():
            report["game_process"] = process


def write_smoke_report(path: Path, report: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-id", default="simple-window-smoke")
    parser.add_argument("--log-dir")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--game-duration", type=float, default=8.0)
    parser.add_argument("--game-fps", type=float, default=20.0)
    parser.add_argument("--window-timeout", type=float, default=5.0)
    parser.add_argument("--gate-duration", type=float, default=2.0)
    parser.add_argument("--gate-hz", type=float, default=10.0)
    parser.add_argument("--min-fps", type=float, default=8.0)
    args = parser.parse_args(argv)
    for name in ("width", "height"):
        if getattr(args, name) <= 0:
            parser.error("--%s must be positive" % name.replace("_", "-"))
    for name in ("game_duration", "game_fps", "window_timeout", "gate_duration", "gate_hz"):
        if getattr(args, name) <= 0:
            parser.error("--%s must be positive" % name.replace("_", "-"))
    if args.min_fps < 0:
        parser.error("--min-fps must be non-negative")
    if args.game_duration <= args.window_timeout + args.gate_duration:
        parser.error("--game-duration must exceed --window-timeout + --gate-duration")
    return args


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    base_log_dir = resolve_log_dir(args.log_dir, repo_root=REPO_ROOT).parent
    log_dir = base_log_dir / "game_screen_sandbox"
    report = run_smoke(
        run_id=args.run_id,
        log_dir=log_dir,
        width=args.width,
        height=args.height,
        game_duration_s=args.game_duration,
        game_fps=args.game_fps,
        window_timeout_s=args.window_timeout,
        gate_duration_s=args.gate_duration,
        gate_hz=args.gate_hz,
        min_fps=args.min_fps,
    )
    report_path = log_dir / ("%s-%s-simple-window-smoke.json" % (timestamp_slug(), args.run_id))
    write_smoke_report(report_path, report)
    print("simple-window-smoke %s report=%s phase_summary=%s" % (
        report["status"],
        report_path,
        report.get("phase_report_md", "-"),
    ))
    if report["status"] == PASS:
        return 0
    if report["status"] == WAITING:
        return 2
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
