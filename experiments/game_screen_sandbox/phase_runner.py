#!/usr/bin/env python3
"""Run measurable phase gates for the isolated game/screen sandbox."""

from __future__ import annotations

import argparse
import importlib.util
import json
import os
import platform
import shutil
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

from capture_window import (  # noqa: E402
    SyntheticFrameSource,
    make_region_source,
    parse_region,
    resolve_capture_region,
    sample_source,
)
from game_dynamics_loop import (  # noqa: E402
    GameDynamicsConfig,
    GameHandoffDynamicsConfig,
    GameRangeDynamicsConfig,
    run_game_dynamics_loop,
    run_game_handoff_dynamics_loop,
    run_game_range_dynamics_loop,
)
from latency_probe import (  # noqa: E402
    measure_axis_response_sweep,
    measure_visual_latency,
    normalize_axes,
    parse_axis_shift_expectations,
)
from rc_binding_assistant import run_binding_sequence as run_rc_binding_sequence  # noqa: E402
from screen_tracking_loop import LoopConfig, parse_bbox, run_loop  # noqa: E402
from simple_game_adapter_loop import AdapterLoopConfig, run_adapter_closed_loop  # noqa: E402
from simple_game_objective_loop import (  # noqa: E402
    ObjectiveLoopConfig,
    run_objective_closed_loop,
)
from virtual_input import (  # noqa: E402
    AxisCommand,
    DryRunInputAdapter,
    UInputAdapter,
    UInputUnavailable,
    probe_uinput_environment,
    run_uinput_smoke,
)
from kenet.tracker import ObjectTracker, TrackerType  # noqa: E402
from sitl_log import JsonlLogger, make_log_path, resolve_log_dir, timestamp_slug  # noqa: E402


PASS = "PASS"
FAIL = "FAIL"
WAITING = "WAITING"


@dataclass
class PhaseResult:
    phase: str
    status: str
    summary: str
    metrics: dict[str, Any] = field(default_factory=dict)
    notes: list[str] = field(default_factory=list)

    def as_dict(self) -> dict[str, Any]:
        return {
            "phase": self.phase,
            "status": self.status,
            "summary": self.summary,
            "metrics": self.metrics,
            "notes": self.notes,
        }


@dataclass
class PhaseRunReport:
    run_id: str
    mode: str
    started_at: str
    results: list[PhaseResult]
    log_path: str | None = None

    @property
    def status(self) -> str:
        if any(result.status == FAIL for result in self.results):
            return FAIL
        if any(result.status == WAITING for result in self.results):
            return WAITING
        return PASS

    def as_dict(self) -> dict[str, Any]:
        return {
            "run_id": self.run_id,
            "mode": self.mode,
            "started_at": self.started_at,
            "status": self.status,
            "log_path": self.log_path,
            "results": [result.as_dict() for result in self.results],
        }


def module_available(name: str) -> bool:
    return importlib.util.find_spec(name) is not None


def load_tracker_bbox_file(path: Path) -> tuple[int, int, int, int]:
    data = json.loads(path.read_text(encoding="utf-8"))
    raw = data.get("bbox")
    if not isinstance(raw, list) or len(raw) != 4:
        raise ValueError("tracker bbox file must contain a bbox list with 4 values")
    x, y, w, h = [int(value) for value in raw]
    if w <= 0 or h <= 0:
        raise ValueError("tracker bbox width/height must be positive")
    return (x, y, w, h)


def environment_probe() -> dict[str, Any]:
    display = os.environ.get("DISPLAY")
    ffmpeg_available = shutil.which("ffmpeg") is not None
    return {
        "python": sys.executable,
        "platform": platform.platform(),
        "xdg_session_type": os.environ.get("XDG_SESSION_TYPE"),
        "display": display,
        "wayland_display": os.environ.get("WAYLAND_DISPLAY"),
        "mss_available": module_available("mss"),
        "evdev_available": module_available("evdev"),
        "ffmpeg_available": ffmpeg_available,
        "ffmpeg_x11grab_candidate": bool(display and ffmpeg_available),
        "xwininfo_available": shutil.which("xwininfo") is not None,
        "dev_uinput_exists": Path("/dev/uinput").exists(),
    }


def check_s0_environment() -> PhaseResult:
    probe = environment_probe()
    notes = [
        "synthetic capture and dry-run input do not need a game window",
        "real game capture remains waiting until a simulator/window is selected",
    ]
    real_capture_ready = bool(probe["display"] or probe["wayland_display"])
    real_input_ready = bool(probe["evdev_available"] and probe["dev_uinput_exists"])
    return PhaseResult(
        phase="S0",
        status=PASS,
        summary="core sandbox environment is usable; real game/window selection is separate",
        metrics={
            **probe,
            "real_capture_candidate": real_capture_ready,
            "real_uinput_candidate": real_input_ready,
        },
        notes=notes,
    )


def check_s1_capture(duration_s: float, hz: float, min_fps: float) -> PhaseResult:
    source = SyntheticFrameSource(fps=hz)
    stats = sample_source(source, duration_s=duration_s)
    ok = stats.frames > 0 and stats.nonblank and stats.fps >= min_fps
    return PhaseResult(
        phase="S1",
        status=PASS if ok else FAIL,
        summary="synthetic capture smoke",
        metrics={
            "frames": stats.frames,
            "fps": stats.fps,
            "width": stats.width,
            "height": stats.height,
            "nonblank": stats.nonblank,
            "min_fps": min_fps,
        },
    )


def check_s1_real_capture(
    region: dict[str, int] | None,
    *,
    backend: str,
    duration_s: float,
    hz: float,
    min_fps: float,
    window_title: str | None = None,
    window_exact: bool = False,
    window_case_sensitive: bool = False,
    window_min_width: int = 32,
    window_min_height: int = 32,
) -> PhaseResult:
    if region is None and not window_title:
        return PhaseResult(
            phase="S1-real",
            status=WAITING,
            summary="real screen capture region has not been selected yet",
            metrics={
                "backend": backend,
                "region": None,
                "window_title": None,
                "min_fps": min_fps,
            },
            notes=["provide --capture-region or --capture-window-title after opening the game/sim"],
        )
    try:
        resolved_region = resolve_capture_region(
            region=region,
            window_title=window_title,
            window_exact=window_exact,
            window_case_sensitive=window_case_sensitive,
            window_min_width=window_min_width,
            window_min_height=window_min_height,
        )
        source = make_region_source(resolved_region, fps=hz, backend=backend)
        stats = sample_source(source, duration_s=duration_s)
        ok = stats.frames > 0 and stats.nonblank and stats.fps >= min_fps
        return PhaseResult(
            phase="S1-real",
            status=PASS if ok else FAIL,
            summary="real screen region capture smoke",
            metrics={
                "backend": backend,
                "region": resolved_region,
                "window_title": window_title,
                "frames": stats.frames,
                "fps": stats.fps,
                "width": stats.width,
                "height": stats.height,
                "nonblank": stats.nonblank,
                "min_fps": min_fps,
            },
        )
    except Exception as exc:
        status = WAITING if window_title and region is None else FAIL
        return PhaseResult(
            phase="S1-real",
            status=status,
            summary=(
                "real screen window capture is waiting for a matching window"
                if status == WAITING else "real screen region capture failed"
            ),
            metrics={
                "backend": backend,
                "region": region,
                "window_title": window_title,
                "error": str(exc),
            },
        )


def check_s2_tracker(duration_s: float, hz: float, min_found_ratio: float) -> PhaseResult:
    source = SyntheticFrameSource(fps=hz)
    tracker = ObjectTracker(TrackerType.KCF)
    bbox = source.target_bbox_at(0.0)
    frames = 0
    found = 0
    loss_events = 0
    was_found = True
    centers: list[tuple[float, float]] = []
    for item in source.frames(duration_s):
        if not tracker.is_initialized:
            tracker.init(item.frame, bbox)
        result = tracker.update(item.frame)
        frames += 1
        if result.found:
            found += 1
            centers.append(tuple(result.center))
        elif was_found:
            loss_events += 1
        was_found = result.found
    found_ratio = found / frames if frames else 0.0
    ok = frames > 0 and found_ratio >= min_found_ratio
    return PhaseResult(
        phase="S2",
        status=PASS if ok else FAIL,
        summary="tracker follows the synthetic target with no input",
        metrics={
            "frames": frames,
            "target_found": found,
            "found_ratio": found_ratio,
            "min_found_ratio": min_found_ratio,
            "loss_events": loss_events,
            "initial_bbox": list(bbox),
            "last_center": list(centers[-1]) if centers else None,
        },
    )


def bbox_inside_frame(bbox: tuple[int, int, int, int], *, width: int, height: int) -> bool:
    x, y, w, h = bbox
    return x >= 0 and y >= 0 and w > 0 and h > 0 and x + w <= width and y + h <= height


def check_s2_real_tracker(
    region: dict[str, int] | None,
    *,
    backend: str,
    duration_s: float,
    hz: float,
    min_found_ratio: float,
    tracker_bbox: tuple[int, int, int, int] | None,
    window_title: str | None = None,
    window_exact: bool = False,
    window_case_sensitive: bool = False,
    window_min_width: int = 32,
    window_min_height: int = 32,
) -> PhaseResult:
    if region is None and not window_title:
        return PhaseResult(
            phase="S2-real",
            status=WAITING,
            summary="real tracker capture target has not been selected yet",
            metrics={
                "backend": backend,
                "region": None,
                "window_title": None,
                "tracker_bbox": list(tracker_bbox) if tracker_bbox else None,
            },
            notes=["provide --capture-region or --capture-window-title"],
        )
    if tracker_bbox is None:
        return PhaseResult(
            phase="S2-real",
            status=WAITING,
            summary="real tracker bbox has not been selected yet",
            metrics={
                "backend": backend,
                "region": region,
                "window_title": window_title,
                "tracker_bbox": None,
            },
            notes=["provide --tracker-bbox x,y,w,h after selecting the target in the captured frame"],
        )
    try:
        resolved_region = resolve_capture_region(
            region=region,
            window_title=window_title,
            window_exact=window_exact,
            window_case_sensitive=window_case_sensitive,
            window_min_width=window_min_width,
            window_min_height=window_min_height,
        )
        source = make_region_source(resolved_region, fps=hz, backend=backend)
        tracker = ObjectTracker(TrackerType.KCF)
        frames = 0
        found = 0
        loss_events = 0
        was_found = True
        last_center: tuple[float, float] | None = None
        first_shape: tuple[int, int] | None = None
        for item in source.frames(duration_s):
            frame_height, frame_width = item.frame.shape[:2]
            if first_shape is None:
                first_shape = (frame_width, frame_height)
                if not bbox_inside_frame(tracker_bbox, width=frame_width, height=frame_height):
                    return PhaseResult(
                        phase="S2-real",
                        status=FAIL,
                        summary="real tracker bbox is outside the captured frame",
                        metrics={
                            "backend": backend,
                            "region": resolved_region,
                            "window_title": window_title,
                            "tracker_bbox": list(tracker_bbox),
                            "frame_width": frame_width,
                            "frame_height": frame_height,
                        },
                    )
            if not tracker.is_initialized:
                tracker.init(item.frame, tracker_bbox)
            result = tracker.update(item.frame)
            frames += 1
            if result.found:
                found += 1
                last_center = tuple(result.center)
            elif was_found:
                loss_events += 1
            was_found = result.found
        found_ratio = found / frames if frames else 0.0
        ok = frames > 0 and found_ratio >= min_found_ratio
        frame_width, frame_height = first_shape or (0, 0)
        return PhaseResult(
            phase="S2-real",
            status=PASS if ok else FAIL,
            summary="tracker follows the real captured target with no input",
            metrics={
                "backend": backend,
                "region": resolved_region,
                "window_title": window_title,
                "tracker_bbox": list(tracker_bbox),
                "frames": frames,
                "target_found": found,
                "found_ratio": found_ratio,
                "min_found_ratio": min_found_ratio,
                "loss_events": loss_events,
                "last_center": list(last_center) if last_center else None,
                "frame_width": frame_width,
                "frame_height": frame_height,
            },
        )
    except Exception as exc:
        status = WAITING if window_title and region is None else FAIL
        return PhaseResult(
            phase="S2-real",
            status=status,
            summary=(
                "real tracker window capture is waiting for a matching window"
                if status == WAITING else "real tracker smoke failed"
            ),
            metrics={
                "backend": backend,
                "region": region,
                "window_title": window_title,
                "tracker_bbox": list(tracker_bbox) if tracker_bbox else None,
                "error": str(exc),
            },
        )


def check_s3_input() -> PhaseResult:
    adapter = DryRunInputAdapter()
    command = AxisCommand(yaw=0.10, pitch=-0.05, roll=0.0, throttle=0.0)
    adapter.send(command)
    adapter.close()
    neutral = adapter.commands[-1][1].is_neutral()
    ok = neutral and adapter.closed and adapter.commands[0][1] == command
    return PhaseResult(
        phase="S3",
        status=PASS if ok else FAIL,
        summary="dry-run virtual input sends a small command and returns neutral",
        metrics={
            "commands": len(adapter.commands),
            "first_command": adapter.commands[0][1].as_dict(),
            "last_command": adapter.commands[-1][1].as_dict(),
            "closed": adapter.closed,
            "adapter": adapter.__class__.__name__,
            "real_input": False,
            "neutralized": neutral,
        },
        notes=["real uinput remains disabled until manual neutral/kill-switch smoke"],
    )


def check_s3_real_input(
    *,
    run_smoke: bool,
    command: AxisCommand,
    hold_seconds: float,
) -> PhaseResult:
    probe = probe_uinput_environment()
    if not probe.available:
        return PhaseResult(
            phase="S3-real",
            status=WAITING,
            summary="real uinput prerequisites are not ready",
            metrics=probe.as_dict(),
            notes=["install sandbox optional dependency: fpv_env/bin/python -m pip install -r experiments/game_screen_sandbox/requirements-input.txt"],
        )
    if not run_smoke:
        return PhaseResult(
            phase="S3-real",
            status=WAITING,
            summary="real uinput is available but smoke was not requested",
            metrics=probe.as_dict(),
            notes=["rerun with --include-real-input --uinput-smoke after confirming a neutral/stop plan"],
        )
    try:
        result = run_uinput_smoke(command, hold_seconds=hold_seconds)
    except UInputUnavailable as exc:
        return PhaseResult(
            phase="S3-real",
            status=WAITING,
            summary="real uinput smoke is not available",
            metrics={
                **probe.as_dict(),
                "error": str(exc),
            },
        )
    except Exception as exc:
        return PhaseResult(
            phase="S3-real",
            status=FAIL,
            summary="real uinput smoke failed unexpectedly",
            metrics={
                **probe.as_dict(),
                "error": str(exc),
            },
        )
    return PhaseResult(
        phase="S3-real",
        status=PASS,
        summary="real uinput smoke created a virtual controller and neutralized",
        metrics=result,
    )


def check_s4_closed_loop(
    duration_s: float,
    hz: float,
    min_found_ratio: float,
    log_dir: Path,
    run_id: str,
) -> PhaseResult:
    source = SyntheticFrameSource(fps=hz)
    tracker = ObjectTracker(TrackerType.KCF)
    adapter = DryRunInputAdapter()
    log_path = make_log_path(log_dir, "%s-s4-loop" % run_id)
    config = LoopConfig(
        duration_s=duration_s,
        hz=hz,
        initial_bbox=source.target_bbox_at(0.0),
        run_id=run_id,
        tracker_type=TrackerType.KCF,
        yaw_kp=0.8,
        yaw_ki=0.0,
        yaw_kd=0.0,
        yaw_output_limit=120.0,
    )
    with JsonlLogger(log_path, metadata={
        "tool": "game_screen_phase_runner",
        "phase": "S4",
        "run_id": run_id,
        "source": "synthetic",
        "adapter": adapter.__class__.__name__,
    }) as logger:
        summary = run_loop(source, tracker, adapter, config, logger=logger)
        logger.write("game_screen_phase_s4_summary", **summary.as_dict())
    adapter.close()
    neutral = adapter.commands[-1][1].is_neutral() if adapter.commands else False
    ok = summary.frames > 0 and summary.found_ratio >= min_found_ratio and neutral
    return PhaseResult(
        phase="S4",
        status=PASS if ok else FAIL,
        summary="yaw-only synthetic closed loop stays bounded and neutralizes",
        metrics={
            **summary.as_dict(),
            "min_found_ratio": min_found_ratio,
            "neutralized": neutral,
            "adapter": adapter.__class__.__name__,
            "real_input": False,
            "log_path": str(log_path),
        },
    )


def check_s4_real_closed_loop(
    region: dict[str, int] | None,
    *,
    backend: str,
    duration_s: float,
    hz: float,
    min_found_ratio: float,
    tracker_bbox: tuple[int, int, int, int] | None,
    log_dir: Path,
    run_id: str,
    window_title: str | None = None,
    window_exact: bool = False,
    window_case_sensitive: bool = False,
    window_min_width: int = 32,
    window_min_height: int = 32,
    real_input: bool = False,
    ack_live_input: bool = False,
    live_max_duration_s: float = 2.0,
    phase: str = "S4-real",
) -> PhaseResult:
    if real_input and not ack_live_input:
        return PhaseResult(
            phase=phase,
            status=WAITING,
            summary="real input live loop requires explicit acknowledgement",
            metrics={
                "backend": backend,
                "region": region,
                "window_title": window_title,
                "tracker_bbox": list(tracker_bbox) if tracker_bbox else None,
                "real_input": True,
            },
            notes=["rerun with --include-live-loop --ack-live-input after confirming neutral/stop"],
        )
    if real_input and duration_s > live_max_duration_s:
        return PhaseResult(
            phase=phase,
            status=WAITING,
            summary="real input live loop duration exceeds safety limit",
            metrics={
                "duration_s": duration_s,
                "live_max_duration_s": live_max_duration_s,
                "real_input": True,
            },
            notes=["increase --live-max-duration deliberately if this is expected"],
        )
    if real_input:
        probe = probe_uinput_environment()
        if not probe.available:
            return PhaseResult(
                phase=phase,
                status=WAITING,
                summary="real input live loop prerequisites are not ready",
                metrics=probe.as_dict(),
                notes=["install/enable uinput before live-loop testing"],
            )
    if region is None and not window_title:
        return PhaseResult(
            phase=phase,
            status=WAITING,
            summary="real closed-loop capture target has not been selected yet",
            metrics={
                "backend": backend,
                "region": None,
                "window_title": None,
                "tracker_bbox": list(tracker_bbox) if tracker_bbox else None,
            },
            notes=["provide --capture-region or --capture-window-title"],
        )
    if tracker_bbox is None:
        return PhaseResult(
            phase=phase,
            status=WAITING,
            summary="real closed-loop bbox has not been selected yet",
            metrics={
                "backend": backend,
                "region": region,
                "window_title": window_title,
                "tracker_bbox": None,
            },
            notes=["provide --tracker-bbox x,y,w,h; S4-real is dry-run by default"],
        )
    try:
        resolved_region = resolve_capture_region(
            region=region,
            window_title=window_title,
            window_exact=window_exact,
            window_case_sensitive=window_case_sensitive,
            window_min_width=window_min_width,
            window_min_height=window_min_height,
        )
        frame_width = resolved_region["width"]
        frame_height = resolved_region["height"]
        if not bbox_inside_frame(tracker_bbox, width=frame_width, height=frame_height):
            return PhaseResult(
                phase=phase,
                status=FAIL,
                summary="real closed-loop bbox is outside the captured frame",
                metrics={
                    "backend": backend,
                    "region": resolved_region,
                    "window_title": window_title,
                    "tracker_bbox": list(tracker_bbox),
                    "frame_width": frame_width,
                    "frame_height": frame_height,
                },
            )
        source = make_region_source(resolved_region, fps=hz, backend=backend)
        tracker = ObjectTracker(TrackerType.KCF)
        adapter = UInputAdapter() if real_input else DryRunInputAdapter()
        log_suffix = "s4-live-loop" if real_input else "s4-real-loop"
        log_path = make_log_path(log_dir, "%s-%s" % (run_id, log_suffix))
        config = LoopConfig(
            duration_s=duration_s,
            hz=hz,
            initial_bbox=tracker_bbox,
            run_id=run_id,
            tracker_type=TrackerType.KCF,
            frame_width=frame_width,
            frame_height=frame_height,
            yaw_kp=0.8,
            yaw_ki=0.0,
            yaw_kd=0.0,
            yaw_output_limit=120.0,
        )
        with JsonlLogger(log_path, metadata={
            "tool": "game_screen_phase_runner",
            "phase": phase,
            "run_id": run_id,
            "source": "window:%s:%s" % (window_title, backend) if window_title else "region:%s" % backend,
            "adapter": adapter.__class__.__name__,
            "real_input": real_input,
        }) as logger:
            summary = run_loop(source, tracker, adapter, config, logger=logger)
            logger.write("game_screen_phase_%s_summary" % phase.lower().replace("-", "_"),
                         **summary.as_dict())
        adapter.close()
        neutral = (
            adapter.commands[-1][1].is_neutral()
            if isinstance(adapter, DryRunInputAdapter) and adapter.commands
            else True
        )
        ok = summary.frames > 0 and summary.found_ratio >= min_found_ratio and neutral
        return PhaseResult(
            phase=phase,
            status=PASS if ok else FAIL,
            summary=(
                "real-window yaw-only live loop stays bounded and neutralizes"
                if real_input else
                "real-window yaw-only dry-run loop stays bounded and neutralizes"
            ),
            metrics={
                **summary.as_dict(),
                "backend": backend,
                "region": resolved_region,
                "window_title": window_title,
                "tracker_bbox": list(tracker_bbox),
                "min_found_ratio": min_found_ratio,
                "neutralized": neutral,
                "adapter": adapter.__class__.__name__,
                "real_input": real_input,
                "log_path": str(log_path),
            },
            notes=[
                "real joystick commands were sent through uinput"
                if real_input else
                "dry-run only: no joystick command is sent to the game in S4-real"
            ],
        )
    except Exception as exc:
        status = WAITING if window_title and region is None else FAIL
        return PhaseResult(
            phase=phase,
            status=status,
            summary=(
                "real closed-loop window capture is waiting for a matching window"
                if status == WAITING else (
                    "real closed-loop live input failed" if real_input
                    else "real closed-loop dry-run failed"
                )
            ),
            metrics={
                "backend": backend,
                "region": region,
                "window_title": window_title,
                "tracker_bbox": list(tracker_bbox) if tracker_bbox else None,
                "error": str(exc),
            },
        )


def check_s5_real_latency(
    region: dict[str, int] | None,
    *,
    backend: str,
    fps: float,
    pre_duration_s: float,
    post_duration_s: float,
    command: AxisCommand,
    min_diff: float,
    baseline_multiplier: float,
    use_uinput: bool,
    ack_live_input: bool,
    axis_sweep: bool = False,
    axis_names: tuple[str, ...] = ("yaw", "pitch"),
    axis_value: float = 0.05,
    axis_expected_shifts: dict[str, tuple[str, int]] | None = None,
    axis_min_shift_px: float = 0.5,
    window_title: str | None = None,
    window_exact: bool = False,
    window_case_sensitive: bool = False,
    window_min_width: int = 32,
    window_min_height: int = 32,
) -> PhaseResult:
    if region is None and not window_title:
        return PhaseResult(
            phase="S5-real",
            status=WAITING,
            summary="latency capture target has not been selected yet",
            metrics={"region": None, "window_title": None},
        )
    if use_uinput and not ack_live_input:
        return PhaseResult(
            phase="S5-real",
            status=WAITING,
            summary="latency probe real input requires explicit acknowledgement",
            metrics={"real_input": True, "command": command.as_dict()},
            notes=["rerun with --include-latency-probe --latency-uinput --ack-live-input"],
        )
    try:
        resolved_region = resolve_capture_region(
            region=region,
            window_title=window_title,
            window_exact=window_exact,
            window_case_sensitive=window_case_sensitive,
            window_min_width=window_min_width,
            window_min_height=window_min_height,
        )
        source = make_region_source(resolved_region, fps=fps, backend=backend)
        adapter = UInputAdapter() if use_uinput else DryRunInputAdapter()
        try:
            if axis_sweep:
                result = measure_axis_response_sweep(
                    source,
                    adapter,
                    axes=axis_names,
                    axis_value=axis_value,
                    fps=fps,
                    pre_duration_s=pre_duration_s,
                    post_duration_s=post_duration_s,
                    min_diff=min_diff,
                    baseline_multiplier=baseline_multiplier,
                    expected_shifts=axis_expected_shifts,
                    min_shift_px=axis_min_shift_px,
                )
            else:
                result = measure_visual_latency(
                    source,
                    adapter,
                    command=command,
                    fps=fps,
                    pre_duration_s=pre_duration_s,
                    post_duration_s=post_duration_s,
                    min_diff=min_diff,
                    baseline_multiplier=baseline_multiplier,
                )
        finally:
            adapter.close()
        return PhaseResult(
            phase="S5-real",
            status=result.status,
            summary=(
                "RC channel axis visual response measured"
                if axis_sweep and result.status == PASS else
                "RC channel axis visual response not fully observed"
                if axis_sweep else
                "visual response latency measured"
                if result.status == PASS else
                "visual response latency not observed"
            ),
            metrics={
                **result.as_dict(),
                "backend": backend,
                "region": resolved_region,
                "window_title": window_title,
                "adapter": adapter.__class__.__name__,
                "real_input": use_uinput,
                "axis_sweep": axis_sweep,
                "axis_names": list(axis_names) if axis_sweep else None,
                "axis_value": axis_value if axis_sweep else None,
                "axis_expected_shifts": (
                    {
                        axis: "%s%s" % ("+" if sign > 0 else "-", component)
                        for axis, (component, sign) in (axis_expected_shifts or {}).items()
                    } if axis_sweep and axis_expected_shifts else None
                ),
                "axis_min_shift_px": (
                    axis_min_shift_px if axis_sweep and axis_expected_shifts else None
                ),
            },
            notes=result.notes,
        )
    except Exception as exc:
        status = WAITING if window_title and region is None else FAIL
        return PhaseResult(
            phase="S5-real",
            status=status,
            summary=(
                "latency window capture is waiting for a matching window"
                if status == WAITING else "latency probe failed"
            ),
            metrics={
                "backend": backend,
                "region": region,
                "window_title": window_title,
                "error": str(exc),
            },
        )


def check_s6_real_approach(
    region: dict[str, int] | None,
    *,
    backend: str,
    duration_s: float,
    hz: float,
    min_found_ratio: float,
    tracker_bbox: tuple[int, int, int, int] | None,
    log_dir: Path,
    run_id: str,
    desired_target_width: float,
    max_pitch_axis: float,
    window_title: str | None = None,
    window_exact: bool = False,
    window_case_sensitive: bool = False,
    window_min_width: int = 32,
    window_min_height: int = 32,
) -> PhaseResult:
    if region is None and not window_title:
        return PhaseResult(
            phase="S6-real",
            status=WAITING,
            summary="approach capture target has not been selected yet",
            metrics={
                "backend": backend,
                "region": None,
                "window_title": None,
                "tracker_bbox": list(tracker_bbox) if tracker_bbox else None,
                "desired_target_width": desired_target_width,
            },
            notes=["provide --capture-region or --capture-window-title"],
        )
    if tracker_bbox is None:
        return PhaseResult(
            phase="S6-real",
            status=WAITING,
            summary="approach bbox has not been selected yet",
            metrics={
                "backend": backend,
                "region": region,
                "window_title": window_title,
                "tracker_bbox": None,
                "desired_target_width": desired_target_width,
            },
            notes=["provide --tracker-bbox x,y,w,h; S6-real remains dry-run"],
        )
    try:
        resolved_region = resolve_capture_region(
            region=region,
            window_title=window_title,
            window_exact=window_exact,
            window_case_sensitive=window_case_sensitive,
            window_min_width=window_min_width,
            window_min_height=window_min_height,
        )
        frame_width = resolved_region["width"]
        frame_height = resolved_region["height"]
        if not bbox_inside_frame(tracker_bbox, width=frame_width, height=frame_height):
            return PhaseResult(
                phase="S6-real",
                status=FAIL,
                summary="approach bbox is outside the captured frame",
                metrics={
                    "backend": backend,
                    "region": resolved_region,
                    "window_title": window_title,
                    "tracker_bbox": list(tracker_bbox),
                    "frame_width": frame_width,
                    "frame_height": frame_height,
                    "desired_target_width": desired_target_width,
                },
            )
        source = make_region_source(resolved_region, fps=hz, backend=backend)
        tracker = ObjectTracker(TrackerType.KCF)
        adapter = DryRunInputAdapter()
        log_path = make_log_path(log_dir, "%s-s6-real-approach" % run_id)
        config = LoopConfig(
            duration_s=duration_s,
            hz=hz,
            initial_bbox=tracker_bbox,
            enable_pitch=True,
            run_id=run_id,
            tracker_type=TrackerType.KCF,
            frame_width=frame_width,
            frame_height=frame_height,
            yaw_kp=0.8,
            yaw_ki=0.0,
            yaw_kd=0.0,
            forward_kp=0.4,
            forward_ki=0.0,
            forward_kd=0.0,
            yaw_output_limit=120.0,
            forward_output_limit=80.0,
            desired_target_width=desired_target_width,
        )
        with JsonlLogger(log_path, metadata={
            "tool": "game_screen_phase_runner",
            "phase": "S6-real",
            "run_id": run_id,
            "source": "window:%s:%s" % (window_title, backend) if window_title else "region:%s" % backend,
            "adapter": adapter.__class__.__name__,
            "real_input": False,
            "desired_target_width": desired_target_width,
        }) as logger:
            summary = run_loop(source, tracker, adapter, config, logger=logger)
            logger.write("game_screen_phase_s6_real_summary", **summary.as_dict())
        adapter.close()
        neutral = adapter.commands[-1][1].is_neutral() if adapter.commands else False
        initial_width_error = (
            desired_target_width - summary.initial_target_width
            if summary.initial_target_width is not None else None
        )
        last_width_error = (
            desired_target_width - summary.last_target_width
            if summary.last_target_width is not None else None
        )
        pitch_bounded = summary.max_abs_pitch_axis <= max_pitch_axis
        ok = (
            summary.frames > 0
            and summary.found_ratio >= min_found_ratio
            and neutral
            and pitch_bounded
        )
        return PhaseResult(
            phase="S6-real",
            status=PASS if ok else FAIL,
            summary="real-window approach dry-run produces bounded pitch commands",
            metrics={
                **summary.as_dict(),
                "backend": backend,
                "region": resolved_region,
                "window_title": window_title,
                "tracker_bbox": list(tracker_bbox),
                "desired_target_width": desired_target_width,
                "initial_width_error": initial_width_error,
                "last_width_error": last_width_error,
                "min_found_ratio": min_found_ratio,
                "max_pitch_axis": max_pitch_axis,
                "pitch_bounded": pitch_bounded,
                "neutralized": neutral,
                "adapter": adapter.__class__.__name__,
                "real_input": False,
                "enable_pitch": True,
                "log_path": str(log_path),
            },
            notes=[
                "dry-run only: no joystick command is sent to the game in S6-real",
                "bbox width is the first-pass approach signal; use S6b only if it proves noisy or misleading",
            ],
        )
    except Exception as exc:
        status = WAITING if window_title and region is None else FAIL
        return PhaseResult(
            phase="S6-real",
            status=status,
            summary=(
                "approach window capture is waiting for a matching window"
                if status == WAITING else "approach dry-run failed"
            ),
            metrics={
                "backend": backend,
                "region": region,
                "window_title": window_title,
                "tracker_bbox": list(tracker_bbox) if tracker_bbox else None,
                "desired_target_width": desired_target_width,
                "error": str(exc),
            },
        )


def check_s7_real_combined(
    region: dict[str, int] | None,
    *,
    backend: str,
    duration_s: float,
    hz: float,
    min_found_ratio: float,
    tracker_bbox: tuple[int, int, int, int] | None,
    log_dir: Path,
    run_id: str,
    desired_target_width: float,
    max_yaw_axis: float,
    max_pitch_axis: float,
    window_title: str | None = None,
    window_exact: bool = False,
    window_case_sensitive: bool = False,
    window_min_width: int = 32,
    window_min_height: int = 32,
) -> PhaseResult:
    if region is None and not window_title:
        return PhaseResult(
            phase="S7-real",
            status=WAITING,
            summary="combined-loop capture target has not been selected yet",
            metrics={
                "backend": backend,
                "region": None,
                "window_title": None,
                "tracker_bbox": list(tracker_bbox) if tracker_bbox else None,
                "desired_target_width": desired_target_width,
            },
            notes=["provide --capture-region or --capture-window-title"],
        )
    if tracker_bbox is None:
        return PhaseResult(
            phase="S7-real",
            status=WAITING,
            summary="combined-loop bbox has not been selected yet",
            metrics={
                "backend": backend,
                "region": region,
                "window_title": window_title,
                "tracker_bbox": None,
                "desired_target_width": desired_target_width,
            },
            notes=["provide --tracker-bbox x,y,w,h; S7-real remains dry-run"],
        )
    try:
        resolved_region = resolve_capture_region(
            region=region,
            window_title=window_title,
            window_exact=window_exact,
            window_case_sensitive=window_case_sensitive,
            window_min_width=window_min_width,
            window_min_height=window_min_height,
        )
        frame_width = resolved_region["width"]
        frame_height = resolved_region["height"]
        if not bbox_inside_frame(tracker_bbox, width=frame_width, height=frame_height):
            return PhaseResult(
                phase="S7-real",
                status=FAIL,
                summary="combined-loop bbox is outside the captured frame",
                metrics={
                    "backend": backend,
                    "region": resolved_region,
                    "window_title": window_title,
                    "tracker_bbox": list(tracker_bbox),
                    "frame_width": frame_width,
                    "frame_height": frame_height,
                    "desired_target_width": desired_target_width,
                },
            )
        source = make_region_source(resolved_region, fps=hz, backend=backend)
        tracker = ObjectTracker(TrackerType.KCF)
        adapter = DryRunInputAdapter()
        log_path = make_log_path(log_dir, "%s-s7-real-combined" % run_id)
        config = LoopConfig(
            duration_s=duration_s,
            hz=hz,
            initial_bbox=tracker_bbox,
            enable_pitch=True,
            run_id=run_id,
            tracker_type=TrackerType.KCF,
            frame_width=frame_width,
            frame_height=frame_height,
            yaw_kp=0.8,
            yaw_ki=0.0,
            yaw_kd=0.0,
            forward_kp=0.4,
            forward_ki=0.0,
            forward_kd=0.0,
            yaw_output_limit=120.0,
            forward_output_limit=80.0,
            desired_target_width=desired_target_width,
        )
        with JsonlLogger(log_path, metadata={
            "tool": "game_screen_phase_runner",
            "phase": "S7-real",
            "run_id": run_id,
            "source": "window:%s:%s" % (window_title, backend) if window_title else "region:%s" % backend,
            "adapter": adapter.__class__.__name__,
            "real_input": False,
            "desired_target_width": desired_target_width,
        }) as logger:
            summary = run_loop(source, tracker, adapter, config, logger=logger)
            logger.write("game_screen_phase_s7_real_summary", **summary.as_dict())
        adapter.close()
        neutral = adapter.commands[-1][1].is_neutral() if adapter.commands else False
        yaw_bounded = summary.max_abs_yaw_axis <= max_yaw_axis
        pitch_bounded = summary.max_abs_pitch_axis <= max_pitch_axis
        yaw_nonzero_samples = sum(1 for _, command in adapter.commands if abs(command.yaw) > 0.0)
        pitch_nonzero_samples = sum(1 for _, command in adapter.commands if abs(command.pitch) > 0.0)
        initial_width_error = (
            desired_target_width - summary.initial_target_width
            if summary.initial_target_width is not None else None
        )
        last_width_error = (
            desired_target_width - summary.last_target_width
            if summary.last_target_width is not None else None
        )
        ok = (
            summary.frames > 0
            and summary.found_ratio >= min_found_ratio
            and neutral
            and yaw_bounded
            and pitch_bounded
        )
        return PhaseResult(
            phase="S7-real",
            status=PASS if ok else FAIL,
            summary="real-window combined yaw+pitch dry-run stays bounded",
            metrics={
                **summary.as_dict(),
                "backend": backend,
                "region": resolved_region,
                "window_title": window_title,
                "tracker_bbox": list(tracker_bbox),
                "desired_target_width": desired_target_width,
                "initial_width_error": initial_width_error,
                "last_width_error": last_width_error,
                "min_found_ratio": min_found_ratio,
                "max_yaw_axis": max_yaw_axis,
                "max_pitch_axis": max_pitch_axis,
                "yaw_bounded": yaw_bounded,
                "pitch_bounded": pitch_bounded,
                "yaw_nonzero_samples": yaw_nonzero_samples,
                "pitch_nonzero_samples": pitch_nonzero_samples,
                "neutralized": neutral,
                "adapter": adapter.__class__.__name__,
                "real_input": False,
                "enable_pitch": True,
                "log_path": str(log_path),
            },
            notes=[
                "dry-run only: no joystick command is sent to the game in S7-real",
                "live game-response S7 should verify center/width trends after this gate passes",
            ],
        )
    except Exception as exc:
        status = WAITING if window_title and region is None else FAIL
        return PhaseResult(
            phase="S7-real",
            status=status,
            summary=(
                "combined-loop window capture is waiting for a matching window"
                if status == WAITING else "combined-loop dry-run failed"
            ),
            metrics={
                "backend": backend,
                "region": region,
                "window_title": window_title,
                "tracker_bbox": list(tracker_bbox) if tracker_bbox else None,
                "desired_target_width": desired_target_width,
                "error": str(exc),
            },
        )


def check_s8_moving_target(
    *,
    duration_s: float,
    hz: float,
    min_found_ratio: float,
    min_center_motion_px: float,
    max_yaw_axis: float,
    max_pitch_axis: float,
    desired_target_width: float,
    log_dir: Path,
    run_id: str,
) -> PhaseResult:
    source = SyntheticFrameSource(width=640, height=480, fps=hz, target_size=80)
    tracker = ObjectTracker(TrackerType.KCF)
    adapter = DryRunInputAdapter()
    log_path = make_log_path(log_dir, "%s-s8-moving-target" % run_id)
    config = LoopConfig(
        duration_s=duration_s,
        hz=hz,
        initial_bbox=source.target_bbox_at(0.0),
        enable_pitch=True,
        run_id=run_id,
        tracker_type=TrackerType.KCF,
        frame_width=source.width,
        frame_height=source.height,
        yaw_kp=0.8,
        yaw_ki=0.0,
        yaw_kd=0.0,
        forward_kp=0.4,
        forward_ki=0.0,
        forward_kd=0.0,
        yaw_output_limit=120.0,
        forward_output_limit=80.0,
        desired_target_width=desired_target_width,
    )
    with JsonlLogger(log_path, metadata={
        "tool": "game_screen_phase_runner",
        "phase": "S8",
        "run_id": run_id,
        "source": "synthetic-moving-target",
        "adapter": adapter.__class__.__name__,
        "real_input": False,
        "desired_target_width": desired_target_width,
    }) as logger:
        summary = run_loop(source, tracker, adapter, config, logger=logger)
        logger.write("game_screen_phase_s8_summary", **summary.as_dict())
    adapter.close()
    neutral = adapter.commands[-1][1].is_neutral() if adapter.commands else False
    yaw_bounded = summary.max_abs_yaw_axis <= max_yaw_axis
    pitch_bounded = summary.max_abs_pitch_axis <= max_pitch_axis
    center_motion_px = summary.target_center_span_px or 0.0
    motion_ok = center_motion_px >= min_center_motion_px
    ok = (
        summary.frames > 0
        and summary.found_ratio >= min_found_ratio
        and motion_ok
        and neutral
        and yaw_bounded
        and pitch_bounded
    )
    return PhaseResult(
        phase="S8",
        status=PASS if ok else FAIL,
        summary="synthetic moving-target combined dry-run stays bounded",
        metrics={
            **summary.as_dict(),
            "source": "synthetic-moving-target",
            "initial_bbox": list(source.target_bbox_at(0.0)),
            "desired_target_width": desired_target_width,
            "min_found_ratio": min_found_ratio,
            "min_center_motion_px": min_center_motion_px,
            "center_motion_ok": motion_ok,
            "max_yaw_axis": max_yaw_axis,
            "max_pitch_axis": max_pitch_axis,
            "yaw_bounded": yaw_bounded,
            "pitch_bounded": pitch_bounded,
            "neutralized": neutral,
            "adapter": adapter.__class__.__name__,
            "real_input": False,
            "enable_pitch": True,
            "log_path": str(log_path),
        },
        notes=[
            "dry-run only: no joystick command is sent in S8",
            "this is a deterministic pre-game moving-target gate; run game-specific S8 after a simulator target is selected",
        ],
    )


def check_s9_game_dynamics(
    *,
    duration_s: float,
    hz: float,
    log_dir: Path,
    run_id: str,
    initial_offset_x: float,
    max_final_error_px: float,
    min_error_reduction_ratio: float,
    min_found_ratio: float,
    max_yaw_axis: float,
) -> PhaseResult:
    log_path = make_log_path(log_dir, "%s-s9-game-dynamics" % run_id)
    config = GameDynamicsConfig(
        duration_s=duration_s,
        hz=hz,
        initial_offset_x=initial_offset_x,
        max_final_abs_error_px=max_final_error_px,
        min_error_reduction_ratio=min_error_reduction_ratio,
        min_found_ratio=min_found_ratio,
        max_yaw_axis=max_yaw_axis,
        run_id=run_id,
        log_dir=log_dir,
    )
    with JsonlLogger(log_path, metadata={
        "tool": "game_screen_phase_runner",
        "phase": "S9-game",
        "run_id": run_id,
        "source": "in-process-simple-target-game",
        "adapter": "in-process-game-camera",
        "real_input": False,
        "control_applied_to_game_camera": True,
    }) as logger:
        result = run_game_dynamics_loop(config, logger=logger)
        logger.write("game_screen_phase_s9_game_summary", **result.metrics)
    return PhaseResult(
        phase="S9-game",
        status=result.status,
        summary=result.summary,
        metrics={**result.metrics, "log_path": str(log_path)},
        notes=result.notes + [
            "PID output is applied to the in-process simple game camera",
            "sandbox-only: no Gazebo, pr0p, Betaflight, or OS input is used",
        ],
    )


def check_s10_range_dynamics(
    *,
    duration_s: float,
    hz: float,
    log_dir: Path,
    run_id: str,
    initial_offset_x: float,
    max_final_error_px: float,
    min_error_reduction_ratio: float,
    min_found_ratio: float,
    max_yaw_axis: float,
    initial_target_width: float,
    desired_target_width: float,
    max_final_width_error_px: float,
    min_width_error_reduction_ratio: float,
    max_pitch_axis: float,
) -> PhaseResult:
    log_path = make_log_path(log_dir, "%s-s10-range-dynamics" % run_id)
    config = GameRangeDynamicsConfig(
        duration_s=duration_s,
        hz=hz,
        initial_offset_x=initial_offset_x,
        max_final_abs_error_px=max_final_error_px,
        min_error_reduction_ratio=min_error_reduction_ratio,
        min_found_ratio=min_found_ratio,
        max_yaw_axis=max_yaw_axis,
        initial_target_width=initial_target_width,
        desired_target_width=desired_target_width,
        max_final_abs_width_error_px=max_final_width_error_px,
        min_width_error_reduction_ratio=min_width_error_reduction_ratio,
        max_pitch_axis=max_pitch_axis,
        run_id=run_id,
        log_dir=log_dir,
    )
    with JsonlLogger(log_path, metadata={
        "tool": "game_screen_phase_runner",
        "phase": "S10-range",
        "run_id": run_id,
        "source": "in-process-simple-target-game-range",
        "adapter": "in-process-game-camera-and-range",
        "real_input": False,
        "control_applied_to_game_camera": True,
        "range_control_applied_to_target_width": True,
    }) as logger:
        result = run_game_range_dynamics_loop(config, logger=logger)
        logger.write("game_screen_phase_s10_range_summary", **result.metrics)
    return PhaseResult(
        phase="S10-range",
        status=result.status,
        summary=result.summary,
        metrics={**result.metrics, "log_path": str(log_path)},
        notes=result.notes + [
            "PID yaw is applied to the in-process game camera",
            "PID forward/pitch is applied to synthetic apparent target width",
            "sandbox-only: no Gazebo, pr0p, Betaflight, or OS input is used",
        ],
    )


def check_s11_handoff_dynamics(
    *,
    duration_s: float,
    hz: float,
    log_dir: Path,
    run_id: str,
    manual_duration_s: float,
    initial_offset_x: float,
    max_final_error_px: float,
    min_found_ratio: float,
    max_yaw_axis: float,
    initial_target_width: float,
    desired_target_width: float,
    max_final_width_error_px: float,
    min_width_error_reduction_ratio: float,
    max_pitch_axis: float,
    max_handoff_error_px: float,
) -> PhaseResult:
    log_path = make_log_path(log_dir, "%s-s11-handoff-dynamics" % run_id)
    config = GameHandoffDynamicsConfig(
        duration_s=duration_s,
        hz=hz,
        manual_duration_s=manual_duration_s,
        initial_offset_x=initial_offset_x,
        max_final_abs_error_px=max_final_error_px,
        min_found_ratio=min_found_ratio,
        max_yaw_axis=max_yaw_axis,
        initial_target_width=initial_target_width,
        desired_target_width=desired_target_width,
        max_final_abs_width_error_px=max_final_width_error_px,
        min_width_error_reduction_ratio=min_width_error_reduction_ratio,
        max_pitch_axis=max_pitch_axis,
        max_handoff_abs_error_px=max_handoff_error_px,
        run_id=run_id,
        log_dir=log_dir,
    )
    with JsonlLogger(log_path, metadata={
        "tool": "game_screen_phase_runner",
        "phase": "S11-handoff",
        "run_id": run_id,
        "source": "in-process-simple-target-game-handoff",
        "adapter": "scripted-manual-then-in-process-autopilot",
        "real_input": False,
        "manual_to_auto_handoff": True,
        "control_applied_to_game_camera": True,
        "range_control_applied_to_target_width": True,
    }) as logger:
        result = run_game_handoff_dynamics_loop(config, logger=logger)
        logger.write("game_screen_phase_s11_handoff_summary", **result.metrics)
    return PhaseResult(
        phase="S11-handoff",
        status=result.status,
        summary=result.summary,
        metrics={**result.metrics, "log_path": str(log_path)},
        notes=result.notes + [
            "scripted manual centering runs before the follow command",
            "autonomous tracker/PID control starts only after the follow command",
            "sandbox-only: no Gazebo, pr0p, Betaflight, or OS input is used",
        ],
    )


def check_s12_adapter_dynamics(
    *,
    duration_s: float,
    hz: float,
    log_dir: Path,
    run_id: str,
    initial_offset_x: float,
    max_final_error_px: float,
    min_error_reduction_ratio: float,
    min_found_ratio: float,
    max_yaw_axis: float,
) -> PhaseResult:
    log_path = make_log_path(log_dir, "%s-s12-adapter-dynamics" % run_id)
    config = AdapterLoopConfig(
        duration_s=duration_s,
        hz=hz,
        initial_offset_x=initial_offset_x,
        max_final_abs_error_px=max_final_error_px,
        min_error_reduction_ratio=min_error_reduction_ratio,
        min_found_ratio=min_found_ratio,
        max_yaw_axis=max_yaw_axis,
        run_id=run_id,
        log_dir=log_dir,
    )
    with JsonlLogger(log_path, metadata={
        "tool": "game_screen_phase_runner",
        "phase": "S12-adapter",
        "run_id": run_id,
        "source": "in-process-simple-target-game-adapter",
        "adapter": "SimpleGameInputAdapter",
        "real_input": False,
        "control_applied_through_input_adapter": True,
    }) as logger:
        result = run_adapter_closed_loop(config, logger=logger)
        logger.write("game_screen_phase_s12_adapter_summary", **result.metrics)
    return PhaseResult(
        phase="S12-adapter",
        status=result.status,
        summary=result.summary,
        metrics={**result.metrics, "log_path": str(log_path)},
        notes=result.notes + [
            "screen_tracking_loop sends commands through an InputAdapter",
            "the simple game frame source applies the adapter command to the next frame",
            "sandbox-only: no Gazebo, pr0p, Betaflight, screen capture, or OS input is used",
        ],
    )


def check_s13_binding_dry() -> PhaseResult:
    report = run_rc_binding_sequence(
        run_id="s13-binding-dry",
        axes=("yaw", "pitch", "roll", "throttle"),
        axis_value=0.6,
        hold_seconds=0.0,
        neutral_seconds=0.0,
        cycles=1,
        include_negative=True,
        use_uinput=False,
        ack_live_input=False,
        countdown_seconds=0.0,
        sleep_fn=lambda _seconds: None,
    )
    return PhaseResult(
        phase="S13-binding-dry",
        status=report.status,
        summary=report.summary,
        metrics={
            **report.metrics,
            "steps": [step.as_dict() for step in report.steps],
            "gazebo_independent": True,
            "no_os_input": True,
        },
        notes=report.notes + [
            "dry gate only: use rc_binding_assistant.py --uinput --ack-live-input for real simulator binding",
            "sandbox-only: no Gazebo, Betaflight, pr0p, screen capture, or OS input is used",
        ],
    )


def check_s14_objective_loop(
    *,
    duration_s: float,
    hz: float,
    log_dir: Path,
    run_id: str,
    manual_duration_s: float,
    initial_offset_x: float,
    max_final_error_px: float,
    min_found_ratio: float,
    max_yaw_axis: float,
    initial_target_width: float,
    desired_target_width: float,
    max_final_width_error_px: float,
    min_width_error_reduction_ratio: float,
    max_pitch_axis: float,
    max_handoff_error_px: float,
) -> PhaseResult:
    log_path = make_log_path(log_dir, "%s-s14-objective-loop" % run_id)
    config = ObjectiveLoopConfig(
        duration_s=duration_s,
        hz=hz,
        manual_duration_s=manual_duration_s,
        initial_offset_x=initial_offset_x,
        max_final_abs_error_px=max_final_error_px,
        min_found_ratio=min_found_ratio,
        max_yaw_axis=max_yaw_axis,
        initial_target_width=initial_target_width,
        desired_target_width=desired_target_width,
        max_final_abs_width_error_px=max_final_width_error_px,
        min_width_error_reduction_ratio=min_width_error_reduction_ratio,
        max_pitch_axis=max_pitch_axis,
        max_handoff_abs_error_px=max_handoff_error_px,
        run_id=run_id,
        log_dir=log_dir,
    )
    with JsonlLogger(log_path, metadata={
        "tool": "game_screen_phase_runner",
        "phase": "S14-objective",
        "run_id": run_id,
        "source": "in-process-simple-target-game-objective",
        "adapter": "ObjectiveInputAdapter",
        "real_input": False,
        "manual_to_auto_handoff": True,
        "control_applied_through_input_adapter": True,
        "range_control_applied_to_target_width": True,
    }) as logger:
        result = run_objective_closed_loop(config, logger=logger)
        logger.write("game_screen_phase_s14_objective_summary", **result.metrics)
    return PhaseResult(
        phase="S14-objective",
        status=result.status,
        summary=result.summary,
        metrics={**result.metrics, "log_path": str(log_path)},
        notes=result.notes + [
            "manual phase, follow event, tracker init, PID yaw/pitch, and adapter-applied control are measured in one flow",
            "sandbox-only: no Gazebo, pr0p, Betaflight, screen capture, OS input, or physical RC is used",
        ],
    )


def markdown_value(value: Any) -> str:
    if value is None:
        return "-"
    if isinstance(value, bool):
        return "true" if value else "false"
    if isinstance(value, float):
        text = "%.3f" % value
        return text.rstrip("0").rstrip(".")
    if isinstance(value, (list, dict)):
        text = json.dumps(value, sort_keys=True)
    else:
        text = str(value)
    return text.replace("\n", " ").replace("|", "\\|")


def first_metric(metrics: dict[str, Any], *names: str) -> Any:
    for name in names:
        value = metrics.get(name)
        if value is not None:
            return value
    return None


def evidence_row(result: PhaseResult) -> list[Any]:
    metrics = result.metrics
    return [
        result.phase,
        result.status,
        first_metric(metrics, "found_ratio"),
        first_metric(metrics, "loss_events"),
        first_metric(metrics, "target_center_span_px"),
        first_metric(metrics, "max_abs_horizontal_error"),
        first_metric(metrics, "max_abs_forward_error", "latency_ms"),
        first_metric(metrics, "max_abs_yaw_axis"),
        first_metric(metrics, "max_abs_pitch_axis"),
        first_metric(metrics, "adapter"),
        first_metric(metrics, "real_input"),
        first_metric(metrics, "neutralized"),
        first_metric(metrics, "log_path"),
    ]


def report_sources(report: PhaseRunReport) -> str:
    sources: list[str] = []
    for result in report.results:
        metrics = result.metrics
        source = metrics.get("source")
        if not source and metrics.get("window_title"):
            source = "window:%s" % metrics["window_title"]
        if not source and metrics.get("region") is not None:
            source = "region:%s" % metrics.get("backend", "unknown")
        if source and source not in sources:
            sources.append(str(source))
    return ", ".join(sources) if sources else report.mode


def report_adapters(report: PhaseRunReport) -> str:
    adapters: list[str] = []
    for result in report.results:
        adapter = result.metrics.get("adapter")
        if adapter and adapter not in adapters:
            adapters.append(str(adapter))
    return ", ".join(adapters) if adapters else "-"


def build_markdown_report(report: PhaseRunReport) -> str:
    lines = [
        "# Game Screen Sandbox Phase Report",
        "",
        "Run: `%s`" % report.run_id,
        "Mode: `%s`" % report.mode,
        "Started: `%s`" % report.started_at,
        "Verdict: `%s`" % report.status,
        "",
        "## Run Summary",
        "",
        "| Field | Value |",
        "| --- | --- |",
        "| Game/sim/source | %s |" % markdown_value(report_sources(report)),
        "| Input adapter(s) | %s |" % markdown_value(report_adapters(report)),
        "| Isolation | sandbox-only; no Gazebo/Betaflight launcher required |",
        "| JSON details | full metrics are preserved in the paired phase-report JSON |",
        "",
        "## Phase Verdicts",
        "",
        "| Phase | Status | Summary |",
        "| --- | --- | --- |",
    ]
    for result in report.results:
        lines.append("| %s | %s | %s |" % (
            markdown_value(result.phase),
            markdown_value(result.status),
            markdown_value(result.summary),
        ))
    lines.extend([
        "",
        "## Evidence Summary",
        "",
        "| Phase | Status | Found | Loss | Motion px | H error | Forward/latency | Yaw axis | Pitch axis | Adapter | Real input | Neutral | Log |",
        "| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |",
    ])
    for result in report.results:
        lines.append("| %s |" % " | ".join(markdown_value(value) for value in evidence_row(result)))
    lines.append("")
    lines.append("## Raw Metrics")
    for result in report.results:
        lines.append("")
        lines.append("### %s" % result.phase)
        lines.append("")
        lines.append("```json")
        lines.append(json.dumps(result.metrics, indent=2, sort_keys=True))
        lines.append("```")
        if result.notes:
            lines.append("")
            for note in result.notes:
                lines.append("- %s" % note)
    lines.append("")
    return "\n".join(lines)


def write_reports(report: PhaseRunReport, log_dir: Path) -> tuple[Path, Path]:
    slug = "%s-%s" % (timestamp_slug(), report.run_id)
    json_path = log_dir / ("%s-phase-report.json" % slug)
    md_path = log_dir / ("%s-phase-report.md" % slug)
    json_path.write_text(json.dumps(report.as_dict(), indent=2, sort_keys=True) + "\n",
                         encoding="utf-8")
    md_path.write_text(build_markdown_report(report), encoding="utf-8")
    return json_path, md_path


def run_synthetic_gates(
    *,
    duration_s: float,
    hz: float,
    min_fps: float,
    min_tracker_found_ratio: float,
    min_loop_found_ratio: float,
    log_dir: Path,
    run_id: str,
    capture_region: dict[str, int] | None = None,
    capture_window_title: str | None = None,
    capture_backend: str = "auto",
    include_real_capture: bool = False,
    include_real_tracker: bool = False,
    include_real_loop: bool = False,
    include_live_loop: bool = False,
    include_latency_probe: bool = False,
    latency_axis_sweep: bool = False,
    include_real_approach: bool = False,
    include_real_combined: bool = False,
    include_moving_target: bool = False,
    include_game_dynamics: bool = False,
    include_range_dynamics: bool = False,
    include_handoff_dynamics: bool = False,
    include_adapter_dynamics: bool = False,
    include_binding_dry: bool = False,
    include_objective_loop: bool = False,
    latency_uinput: bool = False,
    latency_pre_duration_s: float = 0.5,
    latency_post_duration_s: float = 1.0,
    latency_min_diff: float = 3.0,
    latency_baseline_multiplier: float = 3.0,
    latency_axis_names: tuple[str, ...] = ("yaw", "pitch"),
    latency_axis_value: float = 0.05,
    latency_axis_expected_shifts: dict[str, tuple[str, int]] | None = None,
    latency_axis_min_shift_px: float = 0.5,
    desired_target_width: float = 120.0,
    max_approach_pitch_axis: float = 0.5,
    max_combined_yaw_axis: float = 0.5,
    max_combined_pitch_axis: float = 0.5,
    min_moving_target_motion_px: float = 30.0,
    max_moving_yaw_axis: float = 0.5,
    max_moving_pitch_axis: float = 0.5,
    game_initial_offset_x: float = 180.0,
    game_max_final_error_px: float = 35.0,
    game_min_error_reduction_ratio: float = 0.65,
    game_max_yaw_axis: float = 0.8,
    range_initial_target_width: float = 70.0,
    range_desired_target_width: float = 120.0,
    range_max_final_width_error_px: float = 8.0,
    range_min_width_error_reduction_ratio: float = 0.65,
    range_max_pitch_axis: float = 0.8,
    handoff_manual_duration_s: float = 1.6,
    handoff_max_error_px: float = 45.0,
    ack_live_input: bool = False,
    live_max_duration_s: float = 2.0,
    tracker_bbox: tuple[int, int, int, int] | None = None,
    window_exact: bool = False,
    window_case_sensitive: bool = False,
    window_min_width: int = 32,
    window_min_height: int = 32,
    include_real_input: bool = False,
    run_uinput_smoke_gate: bool = False,
    input_command: AxisCommand | None = None,
    input_hold_seconds: float = 0.0,
) -> PhaseRunReport:
    results = [
        check_s0_environment(),
        check_s1_capture(duration_s, hz, min_fps),
    ]
    if include_real_capture or capture_region is not None or capture_window_title:
        results.append(check_s1_real_capture(
            capture_region,
            backend=capture_backend,
            duration_s=duration_s,
            hz=hz,
            min_fps=min_fps,
            window_title=capture_window_title,
            window_exact=window_exact,
            window_case_sensitive=window_case_sensitive,
            window_min_width=window_min_width,
            window_min_height=window_min_height,
        ))
    results.extend([
        check_s2_tracker(duration_s, hz, min_tracker_found_ratio),
    ])
    if include_real_tracker or tracker_bbox is not None:
        results.append(check_s2_real_tracker(
            capture_region,
            backend=capture_backend,
            duration_s=duration_s,
            hz=hz,
            min_found_ratio=min_tracker_found_ratio,
            tracker_bbox=tracker_bbox,
            window_title=capture_window_title,
            window_exact=window_exact,
            window_case_sensitive=window_case_sensitive,
            window_min_width=window_min_width,
            window_min_height=window_min_height,
        ))
    results.append(check_s3_input())
    if include_real_input or run_uinput_smoke_gate:
        results.append(check_s3_real_input(
            run_smoke=run_uinput_smoke_gate,
            command=input_command or AxisCommand(yaw=0.05),
            hold_seconds=input_hold_seconds,
        ))
    results.append(check_s4_closed_loop(
        duration_s, hz, min_loop_found_ratio, log_dir, run_id))
    if include_real_loop:
        results.append(check_s4_real_closed_loop(
            capture_region,
            backend=capture_backend,
            duration_s=duration_s,
            hz=hz,
            min_found_ratio=min_loop_found_ratio,
            tracker_bbox=tracker_bbox,
            log_dir=log_dir,
            run_id=run_id,
            window_title=capture_window_title,
            window_exact=window_exact,
            window_case_sensitive=window_case_sensitive,
            window_min_width=window_min_width,
            window_min_height=window_min_height,
        ))
    if include_live_loop:
        results.append(check_s4_real_closed_loop(
            capture_region,
            backend=capture_backend,
            duration_s=duration_s,
            hz=hz,
            min_found_ratio=min_loop_found_ratio,
            tracker_bbox=tracker_bbox,
            log_dir=log_dir,
            run_id=run_id,
            window_title=capture_window_title,
            window_exact=window_exact,
            window_case_sensitive=window_case_sensitive,
            window_min_width=window_min_width,
            window_min_height=window_min_height,
            real_input=True,
            ack_live_input=ack_live_input,
            live_max_duration_s=live_max_duration_s,
            phase="S4-live",
        ))
    if include_latency_probe:
        results.append(check_s5_real_latency(
            capture_region,
            backend=capture_backend,
            fps=hz,
            pre_duration_s=latency_pre_duration_s,
            post_duration_s=latency_post_duration_s,
            command=input_command or AxisCommand(yaw=0.05),
            min_diff=latency_min_diff,
            baseline_multiplier=latency_baseline_multiplier,
            use_uinput=latency_uinput,
            ack_live_input=ack_live_input,
            axis_sweep=latency_axis_sweep,
            axis_names=latency_axis_names,
            axis_value=latency_axis_value,
            axis_expected_shifts=latency_axis_expected_shifts,
            axis_min_shift_px=latency_axis_min_shift_px,
            window_title=capture_window_title,
            window_exact=window_exact,
            window_case_sensitive=window_case_sensitive,
            window_min_width=window_min_width,
            window_min_height=window_min_height,
        ))
    if include_real_approach:
        results.append(check_s6_real_approach(
            capture_region,
            backend=capture_backend,
            duration_s=duration_s,
            hz=hz,
            min_found_ratio=min_loop_found_ratio,
            tracker_bbox=tracker_bbox,
            log_dir=log_dir,
            run_id=run_id,
            desired_target_width=desired_target_width,
            max_pitch_axis=max_approach_pitch_axis,
            window_title=capture_window_title,
            window_exact=window_exact,
            window_case_sensitive=window_case_sensitive,
            window_min_width=window_min_width,
            window_min_height=window_min_height,
        ))
    if include_real_combined:
        results.append(check_s7_real_combined(
            capture_region,
            backend=capture_backend,
            duration_s=duration_s,
            hz=hz,
            min_found_ratio=min_loop_found_ratio,
            tracker_bbox=tracker_bbox,
            log_dir=log_dir,
            run_id=run_id,
            desired_target_width=desired_target_width,
            max_yaw_axis=max_combined_yaw_axis,
            max_pitch_axis=max_combined_pitch_axis,
            window_title=capture_window_title,
            window_exact=window_exact,
            window_case_sensitive=window_case_sensitive,
            window_min_width=window_min_width,
            window_min_height=window_min_height,
        ))
    if include_moving_target:
        results.append(check_s8_moving_target(
            duration_s=duration_s,
            hz=hz,
            min_found_ratio=min_loop_found_ratio,
            min_center_motion_px=min_moving_target_motion_px,
            max_yaw_axis=max_moving_yaw_axis,
            max_pitch_axis=max_moving_pitch_axis,
            desired_target_width=desired_target_width,
            log_dir=log_dir,
            run_id=run_id,
        ))
    if include_game_dynamics:
        results.append(check_s9_game_dynamics(
            duration_s=duration_s,
            hz=hz,
            log_dir=log_dir,
            run_id=run_id,
            initial_offset_x=game_initial_offset_x,
            max_final_error_px=game_max_final_error_px,
            min_error_reduction_ratio=game_min_error_reduction_ratio,
            min_found_ratio=min_loop_found_ratio,
            max_yaw_axis=game_max_yaw_axis,
        ))
    if include_range_dynamics:
        results.append(check_s10_range_dynamics(
            duration_s=duration_s,
            hz=hz,
            log_dir=log_dir,
            run_id=run_id,
            initial_offset_x=game_initial_offset_x,
            max_final_error_px=game_max_final_error_px,
            min_error_reduction_ratio=game_min_error_reduction_ratio,
            min_found_ratio=min_loop_found_ratio,
            max_yaw_axis=game_max_yaw_axis,
            initial_target_width=range_initial_target_width,
            desired_target_width=range_desired_target_width,
            max_final_width_error_px=range_max_final_width_error_px,
            min_width_error_reduction_ratio=range_min_width_error_reduction_ratio,
            max_pitch_axis=range_max_pitch_axis,
        ))
    if include_handoff_dynamics:
        results.append(check_s11_handoff_dynamics(
            duration_s=duration_s,
            hz=hz,
            log_dir=log_dir,
            run_id=run_id,
            manual_duration_s=handoff_manual_duration_s,
            initial_offset_x=game_initial_offset_x,
            max_final_error_px=game_max_final_error_px,
            min_found_ratio=min_loop_found_ratio,
            max_yaw_axis=game_max_yaw_axis,
            initial_target_width=range_initial_target_width,
            desired_target_width=range_desired_target_width,
            max_final_width_error_px=range_max_final_width_error_px,
            min_width_error_reduction_ratio=range_min_width_error_reduction_ratio,
            max_pitch_axis=range_max_pitch_axis,
            max_handoff_error_px=handoff_max_error_px,
        ))
    if include_adapter_dynamics:
        results.append(check_s12_adapter_dynamics(
            duration_s=duration_s,
            hz=hz,
            log_dir=log_dir,
            run_id=run_id,
            initial_offset_x=game_initial_offset_x,
            max_final_error_px=game_max_final_error_px,
            min_error_reduction_ratio=game_min_error_reduction_ratio,
            min_found_ratio=min_loop_found_ratio,
            max_yaw_axis=game_max_yaw_axis,
        ))
    if include_binding_dry:
        results.append(check_s13_binding_dry())
    if include_objective_loop:
        results.append(check_s14_objective_loop(
            duration_s=max(duration_s, 6.0),
            hz=max(hz, 20.0),
            log_dir=log_dir,
            run_id=run_id,
            manual_duration_s=handoff_manual_duration_s,
            initial_offset_x=game_initial_offset_x,
            max_final_error_px=game_max_final_error_px,
            min_found_ratio=min_loop_found_ratio,
            max_yaw_axis=game_max_yaw_axis,
            initial_target_width=range_initial_target_width,
            desired_target_width=range_desired_target_width,
            max_final_width_error_px=range_max_final_width_error_px,
            min_width_error_reduction_ratio=range_min_width_error_reduction_ratio,
            max_pitch_axis=range_max_pitch_axis,
            max_handoff_error_px=handoff_max_error_px,
        ))
    return PhaseRunReport(
        run_id=run_id,
        mode=(
            "synthetic+real-capture"
            if capture_region is not None or capture_window_title
            else "synthetic"
        ),
        started_at=time.strftime("%Y-%m-%dT%H:%M:%S%z", time.localtime()),
        results=results,
    )


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=2.0)
    parser.add_argument("--hz", type=float, default=10.0)
    parser.add_argument("--min-fps", type=float, default=8.0)
    parser.add_argument("--min-tracker-found-ratio", type=float, default=0.90)
    parser.add_argument("--min-loop-found-ratio", type=float, default=0.85)
    parser.add_argument("--capture-region", type=parse_region,
                        help="Optional real screen region: left,top,width,height")
    parser.add_argument("--capture-window-title",
                        help="Optional X11 window title substring for real capture")
    parser.add_argument("--capture-backend", choices=["auto", "mss", "ffmpeg"], default="auto")
    parser.add_argument("--include-real-tracker", action="store_true",
                        help="Add S2-real as WAITING when --tracker-bbox is omitted")
    parser.add_argument("--include-real-loop", action="store_true",
                        help="Add S4-real dry-run visual-servo gate")
    parser.add_argument("--include-live-loop", action="store_true",
                        help="Add S4-live real-uinput visual-servo gate")
    parser.add_argument("--ack-live-input", action="store_true",
                        help="Required before S4-live sends real uinput commands")
    parser.add_argument("--live-max-duration", type=float, default=2.0,
                        help="Safety cap for S4-live duration")
    parser.add_argument("--include-latency-probe", action="store_true",
                        help="Add S5-real visual response latency probe")
    parser.add_argument("--include-real-approach", action="store_true",
                        help="Add S6-real dry-run pitch/approach gate")
    parser.add_argument("--include-real-combined", action="store_true",
                        help="Add S7-real dry-run combined yaw+pitch gate")
    parser.add_argument("--include-moving-target", action="store_true",
                        help="Add S8 synthetic moving-target dry-run gate")
    parser.add_argument("--include-game-dynamics", action="store_true",
                        help="Add S9 in-process game-camera closed-loop gate")
    parser.add_argument("--include-range-dynamics", action="store_true",
                        help="Add S10 in-process target-width/approach closed-loop gate")
    parser.add_argument("--include-handoff-dynamics", action="store_true",
                        help="Add S11 scripted manual-to-autonomous handoff gate")
    parser.add_argument("--include-adapter-dynamics", action="store_true",
                        help="Add S12 in-process game loop driven through InputAdapter")
    parser.add_argument("--include-binding-dry", action="store_true",
                        help="Add S13 dry RC channel binding sequence gate")
    parser.add_argument("--include-objective-loop", action="store_true",
                        help="Add S14 manual-follow-tracker-PID-adapter objective gate")
    parser.add_argument("--latency-uinput", action="store_true",
                        help="Use real uinput for S5-real; requires --ack-live-input")
    parser.add_argument("--latency-axis-sweep", action="store_true",
                        help="Probe selected RC/game axes for visual response in S5-real")
    parser.add_argument("--latency-axes", default="yaw,pitch",
                        help="Comma-separated axes for --latency-axis-sweep")
    parser.add_argument("--latency-axis-value", type=float, default=0.05)
    parser.add_argument("--latency-axis-expected-shifts", default="",
                        help="Optional signed shift checks for S5, e.g. yaw:+x,pitch:-y")
    parser.add_argument("--latency-axis-min-shift-px", type=float, default=0.5)
    parser.add_argument("--latency-pre-duration", type=float, default=0.5)
    parser.add_argument("--latency-post-duration", type=float, default=1.0)
    parser.add_argument("--latency-min-diff", type=float, default=3.0)
    parser.add_argument("--latency-baseline-multiplier", type=float, default=3.0)
    parser.add_argument("--desired-target-width", type=float, default=120.0)
    parser.add_argument("--max-approach-pitch-axis", type=float, default=0.5)
    parser.add_argument("--max-combined-yaw-axis", type=float, default=0.5)
    parser.add_argument("--max-combined-pitch-axis", type=float, default=0.5)
    parser.add_argument("--min-moving-target-motion", type=float, default=30.0)
    parser.add_argument("--max-moving-yaw-axis", type=float, default=0.5)
    parser.add_argument("--max-moving-pitch-axis", type=float, default=0.5)
    parser.add_argument("--game-initial-offset-x", type=float, default=180.0)
    parser.add_argument("--game-max-final-error", type=float, default=35.0)
    parser.add_argument("--game-min-error-reduction", type=float, default=0.65)
    parser.add_argument("--game-max-yaw-axis", type=float, default=0.8)
    parser.add_argument("--range-initial-target-width", type=float, default=70.0)
    parser.add_argument("--range-desired-target-width", type=float, default=120.0)
    parser.add_argument("--range-max-final-width-error", type=float, default=8.0)
    parser.add_argument("--range-min-width-error-reduction", type=float, default=0.65)
    parser.add_argument("--range-max-pitch-axis", type=float, default=0.8)
    parser.add_argument("--handoff-manual-duration", type=float, default=1.6)
    parser.add_argument("--handoff-max-error", type=float, default=45.0)
    parser.add_argument("--tracker-bbox", type=parse_bbox,
                        help="Initial target bbox for S2-real: x,y,w,h")
    parser.add_argument("--tracker-bbox-file", type=Path,
                        help="JSON report from bbox_tool.py")
    parser.add_argument("--window-exact", action="store_true")
    parser.add_argument("--window-case-sensitive", action="store_true")
    parser.add_argument("--window-min-width", type=int, default=32)
    parser.add_argument("--window-min-height", type=int, default=32)
    parser.add_argument("--include-real-capture", action="store_true",
                        help="Add S1-real as WAITING when no capture target is selected")
    parser.add_argument("--include-real-input", action="store_true",
                        help="Add S3-real uinput environment gate")
    parser.add_argument("--uinput-smoke", action="store_true",
                        help="Actually create a uinput virtual controller for S3-real")
    parser.add_argument("--input-yaw", type=float, default=0.05)
    parser.add_argument("--input-pitch", type=float, default=0.0)
    parser.add_argument("--input-roll", type=float, default=0.0)
    parser.add_argument("--input-throttle", type=float, default=0.0)
    parser.add_argument("--input-hold-seconds", type=float, default=0.0)
    parser.add_argument("--log-dir", default=None)
    parser.add_argument("--run-id", default="sandbox-phase")
    args = parser.parse_args(argv)
    if args.duration <= 0:
        parser.error("--duration must be positive")
    if args.hz <= 0:
        parser.error("--hz must be positive")
    if args.min_fps < 0:
        parser.error("--min-fps must be non-negative")
    if not 0 <= args.min_tracker_found_ratio <= 1:
        parser.error("--min-tracker-found-ratio must be in [0, 1]")
    if not 0 <= args.min_loop_found_ratio <= 1:
        parser.error("--min-loop-found-ratio must be in [0, 1]")
    if args.input_hold_seconds < 0:
        parser.error("--input-hold-seconds must be non-negative")
    if args.live_max_duration <= 0:
        parser.error("--live-max-duration must be positive")
    if args.latency_pre_duration <= 0 or args.latency_post_duration <= 0:
        parser.error("--latency-pre-duration/--latency-post-duration must be positive")
    if args.latency_min_diff < 0:
        parser.error("--latency-min-diff must be non-negative")
    if args.latency_baseline_multiplier < 1:
        parser.error("--latency-baseline-multiplier must be >= 1")
    if not 0 < abs(args.latency_axis_value) <= 1:
        parser.error("--latency-axis-value magnitude must be in (0, 1]")
    try:
        args.latency_axes = normalize_axes(args.latency_axes)
    except ValueError as exc:
        parser.error(str(exc))
    if args.latency_axis_expected_shifts and not args.latency_axis_sweep:
        parser.error("--latency-axis-expected-shifts requires --latency-axis-sweep")
    try:
        args.latency_axis_expected_shifts = parse_axis_shift_expectations(
            args.latency_axis_expected_shifts
        )
    except ValueError as exc:
        parser.error(str(exc))
    if args.latency_axis_min_shift_px < 0:
        parser.error("--latency-axis-min-shift-px must be non-negative")
    if args.desired_target_width <= 0:
        parser.error("--desired-target-width must be positive")
    if not 0 < args.max_approach_pitch_axis <= 1:
        parser.error("--max-approach-pitch-axis must be in (0, 1]")
    if not 0 < args.max_combined_yaw_axis <= 1:
        parser.error("--max-combined-yaw-axis must be in (0, 1]")
    if not 0 < args.max_combined_pitch_axis <= 1:
        parser.error("--max-combined-pitch-axis must be in (0, 1]")
    if args.min_moving_target_motion < 0:
        parser.error("--min-moving-target-motion must be non-negative")
    if not 0 < args.max_moving_yaw_axis <= 1:
        parser.error("--max-moving-yaw-axis must be in (0, 1]")
    if not 0 < args.max_moving_pitch_axis <= 1:
        parser.error("--max-moving-pitch-axis must be in (0, 1]")
    if args.game_max_final_error < 0:
        parser.error("--game-max-final-error must be non-negative")
    if not 0 <= args.game_min_error_reduction <= 1:
        parser.error("--game-min-error-reduction must be in [0, 1]")
    if not 0 < args.game_max_yaw_axis <= 1:
        parser.error("--game-max-yaw-axis must be in (0, 1]")
    if args.range_initial_target_width <= 0:
        parser.error("--range-initial-target-width must be positive")
    if args.range_desired_target_width <= 0:
        parser.error("--range-desired-target-width must be positive")
    if args.range_max_final_width_error < 0:
        parser.error("--range-max-final-width-error must be non-negative")
    if not 0 <= args.range_min_width_error_reduction <= 1:
        parser.error("--range-min-width-error-reduction must be in [0, 1]")
    if not 0 < args.range_max_pitch_axis <= 1:
        parser.error("--range-max-pitch-axis must be in (0, 1]")
    if args.handoff_manual_duration <= 0:
        parser.error("--handoff-manual-duration must be positive")
    if args.include_handoff_dynamics and args.handoff_manual_duration >= args.duration:
        parser.error("--handoff-manual-duration must be shorter than --duration")
    if (
        args.include_objective_loop
        and args.handoff_manual_duration >= max(args.duration, 6.0)
    ):
        parser.error("--handoff-manual-duration must be shorter than objective duration")
    if args.handoff_max_error < 0:
        parser.error("--handoff-max-error must be non-negative")
    if args.latency_uinput and not args.ack_live_input:
        parser.error("--latency-uinput requires --ack-live-input")
    if args.window_min_width <= 0 or args.window_min_height <= 0:
        parser.error("--window-min-width/--window-min-height must be positive")
    if args.tracker_bbox and args.tracker_bbox_file:
        parser.error("use --tracker-bbox or --tracker-bbox-file, not both")
    if args.tracker_bbox_file:
        try:
            args.tracker_bbox = load_tracker_bbox_file(args.tracker_bbox_file)
        except Exception as exc:
            parser.error(str(exc))
    return args


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    base_log_dir = resolve_log_dir(args.log_dir, repo_root=REPO_ROOT).parent
    log_dir = base_log_dir / "game_screen_sandbox"
    log_dir.mkdir(parents=True, exist_ok=True)
    report = run_synthetic_gates(
        duration_s=args.duration,
        hz=args.hz,
        min_fps=args.min_fps,
        min_tracker_found_ratio=args.min_tracker_found_ratio,
        min_loop_found_ratio=args.min_loop_found_ratio,
        log_dir=log_dir,
        run_id=args.run_id,
        capture_region=args.capture_region,
        capture_window_title=args.capture_window_title,
        capture_backend=args.capture_backend,
        include_real_capture=args.include_real_capture,
        include_real_tracker=args.include_real_tracker,
        include_real_loop=args.include_real_loop,
        include_live_loop=args.include_live_loop,
        include_latency_probe=args.include_latency_probe,
        latency_axis_sweep=args.latency_axis_sweep,
        include_real_approach=args.include_real_approach,
        include_real_combined=args.include_real_combined,
        include_moving_target=args.include_moving_target,
        include_game_dynamics=args.include_game_dynamics,
        include_range_dynamics=args.include_range_dynamics,
        include_handoff_dynamics=args.include_handoff_dynamics,
        include_adapter_dynamics=args.include_adapter_dynamics,
        include_binding_dry=args.include_binding_dry,
        include_objective_loop=args.include_objective_loop,
        latency_uinput=args.latency_uinput,
        latency_pre_duration_s=args.latency_pre_duration,
        latency_post_duration_s=args.latency_post_duration,
        latency_min_diff=args.latency_min_diff,
        latency_baseline_multiplier=args.latency_baseline_multiplier,
        latency_axis_names=args.latency_axes,
        latency_axis_value=args.latency_axis_value,
        latency_axis_expected_shifts=args.latency_axis_expected_shifts,
        latency_axis_min_shift_px=args.latency_axis_min_shift_px,
        desired_target_width=args.desired_target_width,
        max_approach_pitch_axis=args.max_approach_pitch_axis,
        max_combined_yaw_axis=args.max_combined_yaw_axis,
        max_combined_pitch_axis=args.max_combined_pitch_axis,
        min_moving_target_motion_px=args.min_moving_target_motion,
        max_moving_yaw_axis=args.max_moving_yaw_axis,
        max_moving_pitch_axis=args.max_moving_pitch_axis,
        game_initial_offset_x=args.game_initial_offset_x,
        game_max_final_error_px=args.game_max_final_error,
        game_min_error_reduction_ratio=args.game_min_error_reduction,
        game_max_yaw_axis=args.game_max_yaw_axis,
        range_initial_target_width=args.range_initial_target_width,
        range_desired_target_width=args.range_desired_target_width,
        range_max_final_width_error_px=args.range_max_final_width_error,
        range_min_width_error_reduction_ratio=args.range_min_width_error_reduction,
        range_max_pitch_axis=args.range_max_pitch_axis,
        handoff_manual_duration_s=args.handoff_manual_duration,
        handoff_max_error_px=args.handoff_max_error,
        ack_live_input=args.ack_live_input,
        live_max_duration_s=args.live_max_duration,
        tracker_bbox=args.tracker_bbox,
        window_exact=args.window_exact,
        window_case_sensitive=args.window_case_sensitive,
        window_min_width=args.window_min_width,
        window_min_height=args.window_min_height,
        include_real_input=args.include_real_input,
        run_uinput_smoke_gate=args.uinput_smoke,
        input_command=AxisCommand(
            yaw=args.input_yaw,
            pitch=args.input_pitch,
            roll=args.input_roll,
            throttle=args.input_throttle,
        ).clamped(),
        input_hold_seconds=args.input_hold_seconds,
    )
    json_path, md_path = write_reports(report, log_dir)
    print("game-screen-phases %s run_id=%s report=%s summary=%s" % (
        report.status, report.run_id, json_path, md_path))
    for result in report.results:
        print("%s %s - %s" % (result.phase, result.status, result.summary))
    return 1 if report.status == FAIL else 0


if __name__ == "__main__":
    raise SystemExit(main())
