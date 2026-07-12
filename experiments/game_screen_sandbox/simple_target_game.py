#!/usr/bin/env python3
"""Tiny OpenCV game window for the isolated game/screen sandbox.

This is deliberately simple: a textured camera view, a moving target, and
optional keyboard/manual camera motion. It has no Gazebo or Betaflight
dependency and exists only to give the screen-capture/tracker loop a local
window to look at.
"""

from __future__ import annotations

import argparse
import json
import math
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import cv2
import numpy as np

WINDOW_TITLE = "Kenet Simple Target Game"


@dataclass(frozen=True)
class GameConfig:
    width: int = 640
    height: int = 480
    world_width: int = 1800
    world_height: int = 1200
    target_size: int = 90
    target_motion_radius_x: float = 360.0
    target_motion_radius_y: float = 180.0
    target_period_s: float = 8.0
    camera_speed_px_s: float = 260.0
    grid_px: int = 80


@dataclass(frozen=True)
class GameCommand:
    yaw: float = 0.0
    pitch: float = 0.0

    def clamped(self) -> "GameCommand":
        return GameCommand(
            yaw=max(-1.0, min(1.0, float(self.yaw))),
            pitch=max(-1.0, min(1.0, float(self.pitch))),
        )


@dataclass(frozen=True)
class GameState:
    t: float
    camera_x: float
    camera_y: float
    command: GameCommand


def initial_state(config: GameConfig) -> GameState:
    return GameState(
        t=0.0,
        camera_x=config.world_width / 2.0,
        camera_y=config.world_height / 2.0,
        command=GameCommand(),
    )


def target_world_center(t: float, config: GameConfig) -> tuple[float, float]:
    phase = 2.0 * math.pi * t / config.target_period_s
    cx = config.world_width / 2.0 + math.sin(phase) * config.target_motion_radius_x
    cy = config.world_height / 2.0 + math.sin(phase * 0.7 + 0.6) * config.target_motion_radius_y
    return cx, cy


def step_state(state: GameState, command: GameCommand, dt: float, config: GameConfig) -> GameState:
    if dt < 0:
        raise ValueError("dt must be non-negative")
    cmd = command.clamped()
    next_x = state.camera_x + cmd.yaw * config.camera_speed_px_s * dt
    next_y = state.camera_y - cmd.pitch * config.camera_speed_px_s * dt
    half_w = config.width / 2.0
    half_h = config.height / 2.0
    next_x = max(half_w, min(config.world_width - half_w, next_x))
    next_y = max(half_h, min(config.world_height - half_h, next_y))
    return GameState(
        t=state.t + dt,
        camera_x=next_x,
        camera_y=next_y,
        command=cmd,
    )


def target_screen_bbox(state: GameState, config: GameConfig) -> tuple[int, int, int, int]:
    target_x, target_y = target_world_center(state.t, config)
    screen_x = target_x - state.camera_x + config.width / 2.0
    screen_y = target_y - state.camera_y + config.height / 2.0
    half = config.target_size / 2.0
    return (
        int(round(screen_x - half)),
        int(round(screen_y - half)),
        config.target_size,
        config.target_size,
    )


def bbox_intersects_frame(bbox: tuple[int, int, int, int], config: GameConfig) -> bool:
    x, y, w, h = bbox
    return x + w > 0 and y + h > 0 and x < config.width and y < config.height


def render_frame(state: GameState, config: GameConfig) -> np.ndarray:
    frame = np.zeros((config.height, config.width, 3), dtype=np.uint8)
    frame[:, :, 0] = np.linspace(25, 90, config.width, dtype=np.uint8)
    frame[:, :, 1] = np.linspace(35, 125, config.height, dtype=np.uint8)[:, None]
    frame[:, :, 2] = 48

    offset_x = int(round(config.width / 2.0 - state.camera_x))
    offset_y = int(round(config.height / 2.0 - state.camera_y))
    for world_x in range(0, config.world_width + config.grid_px, config.grid_px):
        x = world_x + offset_x
        if 0 <= x < config.width:
            cv2.line(frame, (x, 0), (x, config.height - 1), (45, 80, 90), 1)
    for world_y in range(0, config.world_height + config.grid_px, config.grid_px):
        y = world_y + offset_y
        if 0 <= y < config.height:
            cv2.line(frame, (0, y), (config.width - 1, y), (45, 80, 90), 1)

    cross_x = config.width // 2
    cross_y = config.height // 2
    cv2.line(frame, (cross_x - 18, cross_y), (cross_x + 18, cross_y), (220, 220, 220), 1)
    cv2.line(frame, (cross_x, cross_y - 18), (cross_x, cross_y + 18), (220, 220, 220), 1)

    bbox = target_screen_bbox(state, config)
    if bbox_intersects_frame(bbox, config):
        x, y, w, h = bbox
        x1 = max(0, x)
        y1 = max(0, y)
        x2 = min(config.width - 1, x + w)
        y2 = min(config.height - 1, y + h)
        cv2.rectangle(frame, (x1, y1), (x2, y2), (40, 230, 245), -1)
        cv2.rectangle(frame, (x1 + 8, y1 + 8), (max(x1 + 8, x2 - 8), max(y1 + 8, y2 - 8)),
                      (15, 70, 115), 3)
        cv2.line(frame, (x1, y1), (x2, y2), (255, 255, 255), 2)
        cv2.circle(frame, ((x1 + x2) // 2, (y1 + y2) // 2), max(6, w // 7), (5, 20, 30), -1)
        cv2.circle(frame, (x1 + max(8, w // 5), y1 + max(8, h // 5)), max(4, w // 12),
                   (255, 255, 255), -1)
        cv2.circle(frame, (x2 - max(8, w // 5), y2 - max(8, h // 5)), max(4, w // 12),
                   (0, 0, 0), -1)

    return frame


def keyboard_command(key: int, current: GameCommand) -> tuple[GameCommand, bool]:
    if key < 0:
        return current, False
    char = chr(key & 0xFF).lower()
    if char == "q":
        return current, True
    if char in {"a", "j"}:
        return GameCommand(yaw=-0.7, pitch=current.pitch), False
    if char in {"d", "l"}:
        return GameCommand(yaw=0.7, pitch=current.pitch), False
    if char in {"w", "i"}:
        return GameCommand(yaw=current.yaw, pitch=0.7), False
    if char in {"s", "k"}:
        return GameCommand(yaw=current.yaw, pitch=-0.7), False
    if char in {" ", "x"}:
        return GameCommand(), False
    return current, False


def auto_command(t: float, mode: str) -> GameCommand:
    if mode == "orbit":
        return GameCommand(yaw=0.45 * math.sin(t * 0.8), pitch=0.35 * math.cos(t * 0.6))
    if mode == "sweep":
        return GameCommand(yaw=0.55 * math.sin(t * 1.1), pitch=0.0)
    return GameCommand()


def run_game(
    config: GameConfig,
    *,
    duration_s: float,
    fps: float,
    show: bool = False,
    auto_input: str = "neutral",
    save_frame: Path | None = None,
) -> dict[str, Any]:
    if duration_s <= 0:
        raise ValueError("duration_s must be positive")
    if fps <= 0:
        raise ValueError("fps must be positive")
    if auto_input not in {"neutral", "orbit", "sweep"}:
        raise ValueError("unknown auto_input mode")

    state = initial_state(config)
    dt = 1.0 / fps
    frame_count = int(duration_s * fps)
    first_bbox: tuple[int, int, int, int] | None = None
    last_bbox: tuple[int, int, int, int] | None = None
    first_center: tuple[float, float] | None = None
    last_center: tuple[float, float] | None = None
    saved = False

    if show:
        cv2.namedWindow(WINDOW_TITLE, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(WINDOW_TITLE, config.width, config.height)

    start = time.monotonic()
    try:
        for index in range(frame_count):
            command = auto_command(state.t, auto_input) if auto_input != "neutral" else state.command
            state = step_state(state, command, dt, config)
            frame = render_frame(state, config)
            bbox = target_screen_bbox(state, config)
            center = (bbox[0] + bbox[2] / 2.0, bbox[1] + bbox[3] / 2.0)
            if first_bbox is None:
                first_bbox = bbox
                first_center = center
            last_bbox = bbox
            last_center = center
            if save_frame is not None and not saved:
                save_frame.parent.mkdir(parents=True, exist_ok=True)
                cv2.imwrite(str(save_frame), frame)
                saved = True
            if show:
                cv2.imshow(WINDOW_TITLE, frame)
                key = cv2.waitKey(max(1, int(dt * 1000)))
                state_command, should_quit = keyboard_command(key, state.command)
                if auto_input == "neutral":
                    state = GameState(
                        t=state.t,
                        camera_x=state.camera_x,
                        camera_y=state.camera_y,
                        command=state_command,
                    )
                if should_quit:
                    break
            elif index == 0:
                # Keep headless smoke fast while still producing deterministic frames.
                pass
    finally:
        if show:
            cv2.destroyWindow(WINDOW_TITLE)

    elapsed = max(time.monotonic() - start, 1e-9)
    travel_px = None
    if first_center is not None and last_center is not None:
        travel_px = math.hypot(last_center[0] - first_center[0], last_center[1] - first_center[1])
    return {
        "window_title": WINDOW_TITLE,
        "frames": frame_count,
        "fps_target": fps,
        "fps_wall": frame_count / elapsed,
        "width": config.width,
        "height": config.height,
        "show": show,
        "auto_input": auto_input,
        "target_motion_radius_x": config.target_motion_radius_x,
        "target_motion_radius_y": config.target_motion_radius_y,
        "static_target": config.target_motion_radius_x == 0.0 and config.target_motion_radius_y == 0.0,
        "first_bbox": list(first_bbox) if first_bbox else None,
        "last_bbox": list(last_bbox) if last_bbox else None,
        "target_center_travel_px": travel_px,
        "save_frame": str(save_frame) if save_frame else None,
    }


def write_report(path: Path, report: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--show", action="store_true", help="Open an OpenCV window")
    parser.add_argument("--duration", type=float, default=10.0)
    parser.add_argument("--fps", type=float, default=30.0)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--auto-input", choices=["neutral", "orbit", "sweep"], default="neutral")
    parser.add_argument("--static-target", action="store_true",
                        help="Keep the target centered for stable capture smoke")
    parser.add_argument("--save-frame", type=Path)
    parser.add_argument("--report-path", type=Path)
    args = parser.parse_args(argv)
    if args.duration <= 0:
        parser.error("--duration must be positive")
    if args.fps <= 0:
        parser.error("--fps must be positive")
    if args.width <= 0 or args.height <= 0:
        parser.error("--width/--height must be positive")
    return args


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    config = GameConfig(
        width=args.width,
        height=args.height,
        target_motion_radius_x=0.0 if args.static_target else GameConfig.target_motion_radius_x,
        target_motion_radius_y=0.0 if args.static_target else GameConfig.target_motion_radius_y,
    )
    report = run_game(
        config,
        duration_s=args.duration,
        fps=args.fps,
        show=args.show,
        auto_input=args.auto_input,
        save_frame=args.save_frame,
    )
    if args.report_path:
        write_report(args.report_path, report)
    print("simple-target-game PASS %s" % json.dumps(report, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
