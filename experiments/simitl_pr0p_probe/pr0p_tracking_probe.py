#!/usr/bin/env python3
"""Tracker/PID loop probe for the isolated SimITL/pr0p experiment."""

from __future__ import annotations

import argparse
import json
import sys
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

REPO_ROOT = Path(__file__).resolve().parents[2]
SANDBOX_DIR = REPO_ROOT / "experiments" / "game_screen_sandbox"
TOOLS_DIR = REPO_ROOT / "tools"
for item in (SANDBOX_DIR, TOOLS_DIR):
    if str(item) not in sys.path:
        sys.path.insert(0, str(item))
PROBE_DIR = Path(__file__).resolve().parent
if str(PROBE_DIR) not in sys.path:
    sys.path.insert(0, str(PROBE_DIR))

from capture_window import make_region_source, parse_region  # noqa: E402
from pr0p_capture_probe import (  # noqa: E402
    DEFAULT_LOG_DIR,
    DEFAULT_WINDOW_TITLES,
    apply_relative_crop,
    find_pr0p_window,
)
from msp_uinput_attitude_response_probe import read_altitude_m  # noqa: E402
from msp_uinput_rc_effect_probe import sample_msp_rc  # noqa: E402
from msp_ws_status_probe import run_msp_ws_status_once  # noqa: E402
from pr0p_arm_sequence import HoldAxesAdapter, arm_and_hover_for_response  # noqa: E402
from screen_tracking_loop import LoopConfig, parse_bbox, run_loop  # noqa: E402
from virtual_input import DryRunInputAdapter, UInputAdapter  # noqa: E402
from x11_window import parse_size  # noqa: E402
from kenet.tracker import ObjectTracker, TrackerType  # noqa: E402
from sitl_log import JsonlLogger, make_log_path  # noqa: E402


PASS = "PASS"
WAITING = "WAITING"
FAIL = "FAIL"


@dataclass
class TrackingProbeResult:
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


def load_bbox_file(path: Path) -> tuple[int, int, int, int]:
    data = json.loads(path.read_text(encoding="utf-8"))
    value = data.get("bbox") if isinstance(data, dict) else data
    if not isinstance(value, list) or len(value) != 4:
        raise ValueError("bbox file must contain [x, y, w, h] or {'bbox': [...]}")
    x, y, w, h = [int(item) for item in value]
    if w <= 0 or h <= 0:
        raise ValueError("bbox width/height must be positive")
    return (x, y, w, h)


def bbox_inside_region(bbox: tuple[int, int, int, int], region: dict[str, int]) -> bool:
    x, y, w, h = bbox
    return x >= 0 and y >= 0 and w > 0 and h > 0 and x + w <= region["width"] and y + h <= region["height"]


def takeoff_ramp_schedule(
    *,
    delay_frames: int,
    boost_frames: int,
    boost_throttle: float,
    ramp_frames: int,
    settle_throttle: float,
) -> list[tuple[int, float]]:
    """Build the HoldAxesAdapter throttle schedule for the in-loop takeoff.

    A direct step from throttle-low (+1.0) to the hover value launches this
    overpowered quad at tens of km/h within 0.3 s (measured on video: 0 -> 27
    km/h in one frame, 84 km/h by the next), so ramp_frames inserts a linear
    per-frame ramp from ground to settle_throttle that crosses the lift
    threshold with minimal excess thrust."""
    schedule: list[tuple[int, float]] = []
    if delay_frames > 0:
        schedule.append((delay_frames, 1.0))
    if boost_frames > 0:
        schedule.append((boost_frames, boost_throttle))
    if ramp_frames > 0:
        for step in range(1, ramp_frames + 1):
            value = 1.0 + (settle_throttle - 1.0) * step / (ramp_frames + 1)
            schedule.append((1, value))
    return schedule


def apply_axis_clamps(
    config: LoopConfig,
    *,
    max_abs_yaw_axis: float,
    max_abs_pitch_axis: float,
) -> LoopConfig:
    """Turn the acceptance axis limits into real PID output clamps.

    The loop's own defaults (yaw_output_limit/yaw_scale) allow a 0.4-axis
    command regardless of the acceptance limits, and an unclamped
    max-authority yaw sweep moves the image fast enough to blur the target
    out of CSRT during takeoff (measured: three identical f17 losses).
    """
    if max_abs_yaw_axis > 0:
        config.yaw_output_limit = min(
            config.yaw_output_limit, max_abs_yaw_axis / config.yaw_scale)
    if config.enable_pitch and max_abs_pitch_axis > 0:
        config.forward_output_limit = min(
            config.forward_output_limit, max_abs_pitch_axis / config.pitch_scale)
    return config


def make_source_from_selection(
    *,
    region: dict[str, int] | None,
    crop: dict[str, int] | None,
    window_titles: list[str],
    exact: bool,
    case_sensitive: bool,
    min_width: int,
    min_height: int,
    preferred_size: tuple[int, int] | None,
    allow_common_non_fpv_windows: bool,
    backend: str,
    hz: float,
) -> tuple[Any | None, dict[str, Any], TrackingProbeResult | None]:
    selected_region = region
    selected_window = None
    visible_sample: list[dict[str, Any]] = []
    if selected_region is None:
        kwargs: dict[str, Any] = {}
        if allow_common_non_fpv_windows:
            kwargs["excluded_terms"] = ()
        match, windows, window_error = find_pr0p_window(
            window_titles,
            exact=exact,
            case_sensitive=case_sensitive,
            min_width=min_width,
            min_height=min_height,
            preferred_size=preferred_size,
            **kwargs,
        )
        visible_sample = [
            {
                "window_id": window.window_id,
                "title": window.title,
                "class_text": window.class_text,
                "region": window.region(),
            }
            for window in windows[:12]
        ]
        if match is None:
            return None, {}, TrackingProbeResult(
                status=WAITING,
                summary="no matching pr0p/FPV window is visible yet",
                metrics={
                    "window_titles": list(window_titles),
                    "window_error": window_error,
                    "visible_windows_sample": visible_sample,
                    "real_input_sent": False,
                },
                notes=["CAPTURE_NO_WINDOW", "start pr0p/local race or pass --region"],
            )
        selected_window = match.as_dict()
        selected_region = match.region()

    try:
        capture_region = apply_relative_crop(selected_region, crop)
    except ValueError as exc:
        return None, {}, TrackingProbeResult(
            status=FAIL,
            summary="capture crop is invalid",
            metrics={
                "selected_region": selected_region,
                "crop": crop,
                "error": str(exc),
                "real_input_sent": False,
            },
            notes=["CAPTURE_CROP_INVALID"],
        )

    try:
        source = make_region_source(capture_region, fps=hz, backend=backend)
    except Exception as exc:
        return None, {}, TrackingProbeResult(
            status=WAITING,
            summary="no usable real screen capture backend is available",
            metrics={
                "backend": backend,
                "capture_region": capture_region,
                "error": str(exc),
                "real_input_sent": False,
            },
            notes=["CAPTURE_BACKEND_UNAVAILABLE"],
        )

    return source, {
        "selected_region": selected_region,
        "capture_region": capture_region,
        "crop": crop,
        "selected_window": selected_window,
        "visible_windows_sample": visible_sample,
    }, None


def evaluate_loop_summary(
    summary: Any,
    *,
    min_found_ratio: float,
    max_loss_events: int,
    max_abs_yaw_axis: float,
    max_abs_pitch_axis: float,
) -> tuple[str, str, list[str]]:
    if summary.frames <= 0:
        return FAIL, "tracking loop produced no frames", ["TRACKING_NO_FRAMES"]
    if summary.found_ratio < min_found_ratio:
        return FAIL, "tracker found ratio is below threshold", ["TRACKER_LOSS"]
    if summary.loss_events > max_loss_events:
        return FAIL, "tracker loss events exceeded threshold", ["TRACKER_LOSS"]
    if summary.max_abs_yaw_axis > max_abs_yaw_axis:
        return FAIL, "yaw command exceeded configured bound", ["PID_COMMAND_UNBOUNDED"]
    if summary.max_abs_pitch_axis > max_abs_pitch_axis:
        return FAIL, "pitch command exceeded configured bound", ["PID_COMMAND_UNBOUNDED"]
    return PASS, "tracker/PID loop stayed bounded", ["tracker found target and PID commands stayed bounded"]


class LastResultTrackerProxy:
    """Wraps a tracker and remembers the last found bbox center so the
    throttle provider can see where the target sits vertically. The baro
    saturates at 0.3 m in this build, so the only usable altitude cue is the
    target's vertical position in the frame."""

    def __init__(self, tracker: Any):
        self._tracker = tracker
        self.last_center_y: float | None = None
        self.last_found: bool = False

    @property
    def is_initialized(self) -> bool:
        return self._tracker.is_initialized

    def init(self, frame, bbox) -> None:
        self._tracker.init(frame, bbox)

    def update(self, frame):
        result = self._tracker.update(frame)
        self.last_found = bool(result.found)
        if result.found and result.center is not None:
            self.last_center_y = float(result.center[1])
        return result


def vertical_hold_throttle(
    *,
    center_y: float | None,
    found: bool,
    frame_center_y: float,
    base: float,
    kp: float,
    min_axis: float,
    max_axis: float,
    lost_bias: float,
) -> float:
    """Vision-based throttle hold: keep the target near the vertical frame
    center. Positive vertical error (target below center) means the vehicle
    is too high, so push the axis toward +1 (less thrust). On loss, bias
    slightly above base so the vehicle descends gently instead of climbing
    away or free-falling."""
    if not found or center_y is None:
        value = base + lost_bias
    else:
        error = (center_y - frame_center_y) / max(frame_center_y, 1.0)
        value = base + kp * error
    return max(min_axis, min(max_axis, value))


class AltitudeHoldController:
    """Small P controller: reads baro altitude over MSP and provides a
    throttle axis value that holds the target altitude. Negative axis values
    mean more thrust. Open-loop hover is infeasible here: the measured hover
    band is a few PWM wide (grounded at 1625, strong climb at 1640)."""

    def __init__(
        self,
        *,
        ws_host: str,
        ws_port: int,
        target_alt_m: float,
        base_throttle: float,
        kp: float,
        min_throttle: float = -0.5,
        max_throttle: float = 0.6,
        interval_s: float = 0.25,
    ):
        self.ws_host = ws_host
        self.ws_port = ws_port
        self.target_alt_m = target_alt_m
        self.base_throttle = base_throttle
        self.kp = kp
        self.min_throttle = min_throttle
        self.max_throttle = max_throttle
        self.interval_s = interval_s
        self._value = base_throttle
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self.samples: list[dict[str, float]] = []

    def value(self) -> float:
        return self._value

    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                alt = read_altitude_m(host=self.ws_host, port=self.ws_port)
            except Exception:
                alt = None
            if alt is not None:
                throttle = self.base_throttle - self.kp * (self.target_alt_m - alt)
                throttle = max(self.min_throttle, min(self.max_throttle, throttle))
                self._value = throttle
                self.samples.append({"alt_m": alt, "throttle": round(throttle, 3)})
            self._stop.wait(self.interval_s)

    def start(self) -> None:
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)


def target_center_range_from_log(log_path: Path) -> tuple[float, float] | None:
    xs: list[float] = []
    ys: list[float] = []
    try:
        with log_path.open(encoding="utf-8") as handle:
            for line in handle:
                record = json.loads(line)
                if record.get("event") != "game_screen_sample":
                    continue
                center = record.get("target_center")
                if center and record.get("target_found"):
                    xs.append(float(center[0]))
                    ys.append(float(center[1]))
    except OSError:
        return None
    if not xs:
        return None
    return (max(xs) - min(xs), max(ys) - min(ys))


def run_tracking_probe(
    *,
    run_id: str,
    log_dir: Path,
    region: dict[str, int] | None,
    crop: dict[str, int] | None,
    window_titles: list[str],
    exact: bool,
    case_sensitive: bool,
    min_width: int,
    min_height: int,
    preferred_size: tuple[int, int] | None,
    allow_common_non_fpv_windows: bool,
    backend: str,
    bbox: tuple[int, int, int, int] | None,
    duration_s: float,
    hz: float,
    tracker_type: str,
    enable_pitch: bool,
    use_uinput: bool,
    min_found_ratio: float,
    max_loss_events: int,
    max_abs_yaw_axis: float,
    max_abs_pitch_axis: float,
    desired_target_width: float,
    pitch_delay_frames: int = 0,
    arm_first: bool = False,
    arm_wait_s: float = 12.0,
    arm_throttle: float = -0.4,
    arm_hover_wait_s: float = 0.5,
    takeoff_delay_frames: int = 10,
    takeoff_boost_frames: int = 10,
    takeoff_ramp_frames: int = 0,
    settle_throttle: float = -0.26,
    alt_hold: bool = False,
    alt_target_m: float = 2.5,
    alt_kp: float = 0.06,
    track_throttle_hold: bool = False,
    track_throttle_kp: float = 0.10,
    ws_host: str = "127.0.0.1",
    ws_port: int = 5761,
) -> TrackingProbeResult:
    log_dir.mkdir(parents=True, exist_ok=True)
    if bbox is None:
        return TrackingProbeResult(
            status=WAITING,
            summary="tracking bbox has not been selected yet",
            metrics={
                "run_id": run_id,
                "real_input_sent": False,
                "window_titles": list(window_titles),
            },
            notes=["TRACKER_NO_TARGET", "pass --bbox x,y,w,h after selecting a target in the FPV view"],
        )

    source, selection_metrics, early = make_source_from_selection(
        region=region,
        crop=crop,
        window_titles=window_titles,
        exact=exact,
        case_sensitive=case_sensitive,
        min_width=min_width,
        min_height=min_height,
        preferred_size=preferred_size,
        allow_common_non_fpv_windows=allow_common_non_fpv_windows,
        backend=backend,
        hz=hz,
    )
    if early is not None:
        early.metrics["run_id"] = run_id
        early.metrics["bbox"] = list(bbox)
        return early
    assert source is not None

    capture_region = selection_metrics["capture_region"]
    if not bbox_inside_region(bbox, capture_region):
        return TrackingProbeResult(
            status=FAIL,
            summary="tracking bbox is outside the captured frame",
            metrics={
                "run_id": run_id,
                "bbox": list(bbox),
                "capture_region": capture_region,
                "real_input_sent": False,
            },
            notes=["TRACKER_BBOX_OUTSIDE_FRAME"],
        )

    adapter = UInputAdapter() if use_uinput else DryRunInputAdapter()
    arm_metrics = None
    alt_controller: AltitudeHoldController | None = None
    loop_adapter = adapter
    tracker = ObjectTracker(tracker_type)
    if arm_first:
        if not use_uinput:
            adapter.close()
            return TrackingProbeResult(
                status=WAITING,
                summary="--arm-first requires real uinput input",
                metrics={"run_id": run_id, "bbox": list(bbox), "real_input_sent": False},
                notes=["ARM_FIRST_REQUIRES_UINPUT"],
            )
        # Initialize the tracker on the pre-takeoff scene. The vehicle starts
        # rising as soon as hover throttle is applied, and a tracker that
        # initializes after liftoff can lock onto screen-fixed OSD elements
        # (measured 2026-07-08: CSRT held the OSD battery text with
        # found_ratio 1.0 while the vehicle flew away).
        init_frame = None
        for item in source.frames(max(0.4, 2.0 / hz)):
            init_frame = item.frame
        if init_frame is None:
            adapter.close()
            return TrackingProbeResult(
                status=FAIL,
                summary="could not capture a pre-takeoff frame for tracker init",
                metrics={"run_id": run_id, "bbox": list(bbox), "real_input_sent": False},
                notes=["ARM_FIRST_INIT_FRAME_MISSING"],
            )
        tracker.init(init_frame, bbox)
        # Arm on the ground (hover hold stays at throttle-low) and take off
        # inside the loop: capture spin-up takes ~1s and any pre-loop climb
        # puts the target out of view before the first tracked frame.
        armed, arm_metrics, arm_notes = arm_and_hover_for_response(
            adapter,
            ws_host=ws_host,
            ws_port=ws_port,
            arm_wait_s=arm_wait_s,
            arm_throttle=1.0,
            hover_wait_s=arm_hover_wait_s,
        )
        if not armed:
            adapter.close()
            return TrackingProbeResult(
                status=WAITING,
                summary="arm-first tracking could not arm the FC before the loop",
                metrics={
                    "run_id": run_id,
                    "bbox": list(bbox),
                    "arm_first": arm_metrics,
                    "real_input_sent": True,
                },
                notes=arm_notes,
            )
        arm_metrics["flight_throttle"] = arm_throttle
        arm_metrics["takeoff_delay_frames"] = takeoff_delay_frames
        arm_metrics["takeoff_boost_frames"] = takeoff_boost_frames
        arm_metrics["settle_throttle"] = settle_throttle
        arm_metrics["alt_hold"] = alt_hold
        arm_metrics["track_throttle_hold"] = track_throttle_hold
        if alt_hold:
            alt_controller = AltitudeHoldController(
                ws_host=ws_host,
                ws_port=ws_port,
                target_alt_m=alt_target_m,
                base_throttle=settle_throttle,
                kp=alt_kp,
            )
            alt_controller.start()
            throttle_provider = alt_controller.value
        elif track_throttle_hold:
            # Open-loop settle throttle is metastable: the same profile that
            # quasi-hovered in one run climbs away in the next (measured: the
            # target left the frame bottom at ~120 px/frame at 1.76 s). The
            # baro is saturated, so hold height with the only cue available:
            # the target's vertical position in the frame.
            tracker_proxy = LastResultTrackerProxy(tracker)
            tracker = tracker_proxy
            frame_center_y = capture_region["height"] / 2.0

            def throttle_provider() -> float:
                return vertical_hold_throttle(
                    center_y=tracker_proxy.last_center_y,
                    found=tracker_proxy.last_found,
                    frame_center_y=frame_center_y,
                    base=settle_throttle,
                    kp=track_throttle_kp,
                    min_axis=settle_throttle - 0.06,
                    max_axis=settle_throttle + 0.12,
                    lost_bias=0.04,
                )
        else:
            throttle_provider = None
        # Throttle stages: grounded lock -> short climb -> altitude-held (or
        # just-below-hover) hold so the vehicle stays where the uptilted
        # camera can still see a ground-level target.
        arm_metrics["takeoff_ramp_frames"] = takeoff_ramp_frames
        loop_adapter = HoldAxesAdapter(
            adapter,
            throttle=settle_throttle,
            throttle_schedule=takeoff_ramp_schedule(
                delay_frames=takeoff_delay_frames,
                boost_frames=takeoff_boost_frames,
                boost_throttle=arm_throttle,
                ramp_frames=takeoff_ramp_frames,
                settle_throttle=settle_throttle,
            ),
            throttle_provider=throttle_provider,
        )
    log_path = make_log_path(log_dir, "%s-p6-tracking" % run_id)
    config = LoopConfig(
        duration_s=duration_s,
        hz=hz,
        initial_bbox=bbox,
        enable_pitch=enable_pitch,
        pitch_delay_frames=pitch_delay_frames,
        run_id=run_id,
        tracker_type=tracker_type,
        frame_width=capture_region["width"],
        frame_height=capture_region["height"],
        desired_target_width=desired_target_width,
    )
    apply_axis_clamps(
        config,
        max_abs_yaw_axis=max_abs_yaw_axis,
        max_abs_pitch_axis=max_abs_pitch_axis,
    )
    post_loop_fc: dict[str, Any] | None = None
    try:
        with JsonlLogger(log_path, metadata={
            "tool": "simitl_pr0p_tracking_probe",
            "run_id": run_id,
            "adapter": adapter.__class__.__name__,
            "real_input_sent": use_uinput,
            "tracker_type": tracker_type,
            "capture_region": capture_region,
            "bbox": list(bbox),
        }) as logger:
            summary = run_loop(source, tracker, loop_adapter, config, logger=logger)
            if arm_first:
                # Read FC state before release: a silent mid-loop disarm (e.g.
                # runaway-takeoff prevention) must be visible in the evidence.
                try:
                    status = run_msp_ws_status_once(
                        host=ws_host, port=ws_port, path="/", timeout=1.0, max_frames=8,
                    )
                    fc = (status.metrics or {}).get("fc_status") or {}
                    rc_samples, _rc_metrics = sample_msp_rc(
                        host=ws_host, port=ws_port, path="/", timeout=1.0,
                        max_frames=8, samples=1, interval_s=0.0,
                    )
                    post_loop_fc = {
                        "armed": fc.get("armed"),
                        "active_modes": fc.get("active_modes"),
                        "arming_disable_names": fc.get("arming_disable_names"),
                        "rc_first6": rc_samples[0][:6] if rc_samples else None,
                    }
                except Exception as exc:
                    post_loop_fc = {"error": str(exc)}
                logger.write("simitl_pr0p_post_loop_fc", **post_loop_fc)
            logger.write("simitl_pr0p_tracking_summary", **summary.as_dict())
    except Exception as exc:
        return TrackingProbeResult(
            status=FAIL,
            summary="tracking loop raised an error",
            metrics={
                "run_id": run_id,
                "bbox": list(bbox),
                "error": str(exc),
                "real_input_sent": use_uinput,
                "log_path": str(log_path),
            },
            notes=["TRACKING_LOOP_ERROR"],
        )
    finally:
        if alt_controller is not None:
            alt_controller.stop()
        loop_adapter.close()

    status, summary_text, notes = evaluate_loop_summary(
        summary,
        min_found_ratio=min_found_ratio,
        max_loss_events=max_loss_events,
        max_abs_yaw_axis=max_abs_yaw_axis,
        max_abs_pitch_axis=max_abs_pitch_axis,
    )
    metrics = summary.as_dict()
    metrics.update(selection_metrics)
    metrics.update({
        "run_id": run_id,
        "bbox": list(bbox),
        "adapter": adapter.__class__.__name__,
        "real_input_sent": use_uinput,
        "tracker_type": tracker_type,
        "enable_pitch": enable_pitch,
        "pitch_delay_frames": pitch_delay_frames,
        "min_found_ratio": min_found_ratio,
        "max_loss_events": max_loss_events,
        "max_abs_yaw_axis_limit": max_abs_yaw_axis,
        "max_abs_pitch_axis_limit": max_abs_pitch_axis,
        "desired_target_width": desired_target_width,
        "log_path": str(log_path),
    })
    if arm_metrics is not None:
        metrics["arm_first"] = arm_metrics
        notes.append("ARM_FIRST_ARMED")
    if alt_controller is not None and alt_controller.samples:
        alts = [s["alt_m"] for s in alt_controller.samples]
        metrics["alt_hold_summary"] = {
            "samples": len(alts),
            "alt_min_m": round(min(alts), 2),
            "alt_max_m": round(max(alts), 2),
            "alt_last_m": round(alts[-1], 2),
            "target_alt_m": alt_target_m,
        }
    if post_loop_fc is not None:
        metrics["post_loop_fc"] = post_loop_fc
        if post_loop_fc.get("armed") is False:
            status = WAITING if status == PASS else status
            notes.append("FC_DISARMED_DURING_LOOP")
            summary_text = (
                "tracker loop ran but the FC disarmed mid-loop; "
                "vehicle was not under control"
            )
    if arm_first:
        center_range = target_center_range_from_log(log_path)
        metrics["target_center_range_px"] = center_range
        screen_fixed_lock_suspected = False
        if (
            center_range is not None
            and summary.frames >= 10
            and max(center_range) < 5.0
        ):
            # An armed vehicle moves; a bbox that never moves on screen means
            # the tracker locked a screen-fixed element (e.g. OSD text).
            status = WAITING if status == PASS else status
            screen_fixed_lock_suspected = True
            notes.append("SCREEN_FIXED_LOCK_SUSPECTED")
            summary_text = (
                "tracker bbox never moved during an armed run; "
                "likely locked onto a screen-fixed OSD element"
            )
        metrics["screen_fixed_lock_suspected"] = screen_fixed_lock_suspected
    if not use_uinput:
        notes.append("DRY_RUN_ONLY")
    return TrackingProbeResult(
        status=status,
        summary=summary_text if use_uinput else "%s (dry-run input)" % summary_text,
        metrics=metrics,
        notes=notes,
    )


def build_markdown(result: TrackingProbeResult, *, run_id: str) -> str:
    lines = [
        "# SimITL / pr0p Tracking PID Probe",
        "",
        "Run: `%s`" % run_id,
        "Verdict: `%s`" % result.status,
        "",
        result.summary,
        "",
        "| Metric | Value |",
        "| --- | --- |",
    ]
    for key in (
        "frames",
        "found_ratio",
        "loss_events",
        "max_abs_yaw_axis",
        "max_abs_pitch_axis",
        "horizontal_error_rms",
        "horizontal_error_p95",
        "initial_abs_horizontal_error",
        "last_abs_horizontal_error",
        "horizontal_error_reduction_ratio",
        "forward_error_rms",
        "forward_error_p95",
        "target_center_span_px",
        "real_input_sent",
        "log_path",
    ):
        if key in result.metrics:
            lines.append("| %s | `%s` |" % (key, result.metrics[key]))
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


def write_reports(result: TrackingProbeResult, log_dir: Path, *, run_id: str) -> tuple[Path, Path]:
    log_dir.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime("%Y%m%d-%H%M%S", time.localtime())
    json_path = log_dir / ("%s-%s-tracking-probe.json" % (stamp, run_id))
    md_path = log_dir / ("%s-%s-tracking-probe.md" % (stamp, run_id))
    json_path.write_text(
        json.dumps(result.as_dict(), indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    md_path.write_text(build_markdown(result, run_id=run_id), encoding="utf-8")
    return json_path, md_path


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-id", default="pr0p-tracking")
    parser.add_argument("--log-dir", type=Path, default=DEFAULT_LOG_DIR)
    parser.add_argument("--region", type=parse_region)
    parser.add_argument("--crop", type=parse_region)
    parser.add_argument("--window-title", action="append", dest="window_titles")
    parser.add_argument("--window-exact", action="store_true")
    parser.add_argument("--window-case-sensitive", action="store_true")
    parser.add_argument("--allow-common-non-fpv-windows", action="store_true")
    parser.add_argument("--window-min-width", type=int, default=160)
    parser.add_argument("--window-min-height", type=int, default=120)
    parser.add_argument("--preferred-size", type=parse_size)
    parser.add_argument("--backend", choices=["auto", "mss", "ffmpeg"], default="auto")
    parser.add_argument("--bbox", type=parse_bbox)
    parser.add_argument("--bbox-file", type=Path)
    parser.add_argument("--duration", type=float, default=3.0)
    parser.add_argument("--hz", type=float, default=10.0)
    parser.add_argument("--tracker", choices=[TrackerType.CSRT, TrackerType.KCF],
                        default=TrackerType.CSRT)
    parser.add_argument("--enable-pitch", action="store_true")
    parser.add_argument("--pitch-delay-frames", type=int, default=0,
                        help="Hold pitch at zero for the first N loop frames so the "
                             "approach starts after yaw centering settles")
    parser.add_argument("--desired-target-width", type=float, default=120.0)
    parser.add_argument("--min-found-ratio", type=float, default=0.85)
    parser.add_argument("--max-loss-events", type=int, default=0)
    parser.add_argument("--max-abs-yaw-axis", type=float, default=0.6)
    parser.add_argument("--max-abs-pitch-axis", type=float, default=0.6)
    parser.add_argument("--uinput", action="store_true",
                        help="Send real uinput commands")
    parser.add_argument("--ack-live-input", action="store_true",
                        help="Required before --uinput sends real OS input")
    parser.add_argument("--arm-first", action="store_true",
                        help="Arm the FC (throttle-low then AUX1-high) and hold hover "
                             "throttle during the tracking loop")
    parser.add_argument("--arm-wait", type=float, default=12.0)
    parser.add_argument("--arm-throttle", type=float, default=-0.4)
    parser.add_argument("--arm-hover-wait", type=float, default=0.5,
                        help="Seconds between throttle-up and loop start; keep small "
                             "so the target is still in view at the first frames")
    parser.add_argument("--takeoff-delay-frames", type=int, default=10,
                        help="Loop frames held at throttle-low before switching to "
                             "--arm-throttle, so the tracker locks on the pad first")
    parser.add_argument("--takeoff-boost-frames", type=int, default=10,
                        help="Loop frames held at --arm-throttle for the climb before "
                             "settling to --settle-throttle")
    parser.add_argument("--takeoff-ramp-frames", type=int, default=0,
                        help="Linear per-frame throttle ramp from ground to "
                             "--settle-throttle; a direct step launches the quad at "
                             "tens of km/h within 0.3 s")
    parser.add_argument("--settle-throttle", type=float, default=-0.26,
                        help="Throttle held after the boost; keep slightly below hover "
                             "so the vehicle sinks slowly instead of climbing away")
    parser.add_argument("--alt-hold", action="store_true",
                        help="Hold altitude with a baro P controller after the boost; "
                             "open-loop hover is infeasible (hover band is a few PWM wide)")
    parser.add_argument("--alt-target", type=float, default=2.5)
    parser.add_argument("--alt-kp", type=float, default=0.06)
    parser.add_argument("--track-throttle-hold", action="store_true",
                        help="After the takeoff schedule, steer throttle to keep the "
                             "tracked target near the vertical frame center (the baro "
                             "saturates at 0.3 m, so this is the usable height cue)")
    parser.add_argument("--track-throttle-kp", type=float, default=0.10)
    parser.add_argument("--ws-host", default="127.0.0.1")
    parser.add_argument("--ws-port", type=int, default=5761)
    args = parser.parse_args(argv)
    if args.window_titles is None:
        args.window_titles = list(DEFAULT_WINDOW_TITLES)
    if args.window_min_width <= 0 or args.window_min_height <= 0:
        parser.error("--window-min-width/--window-min-height must be positive")
    if args.duration <= 0:
        parser.error("--duration must be positive")
    if args.hz <= 0:
        parser.error("--hz must be positive")
    if args.desired_target_width <= 0:
        parser.error("--desired-target-width must be positive")
    if args.pitch_delay_frames < 0:
        parser.error("--pitch-delay-frames must be non-negative")
    if args.takeoff_ramp_frames < 0:
        parser.error("--takeoff-ramp-frames must be non-negative")
    if not 0 <= args.min_found_ratio <= 1:
        parser.error("--min-found-ratio must be between 0 and 1")
    if args.max_loss_events < 0:
        parser.error("--max-loss-events must be non-negative")
    if args.max_abs_yaw_axis < 0 or args.max_abs_pitch_axis < 0:
        parser.error("--max-abs-* limits must be non-negative")
    if args.uinput and not args.ack_live_input:
        parser.error("--uinput requires --ack-live-input")
    if args.arm_first and not args.uinput:
        parser.error("--arm-first requires --uinput")
    if args.arm_wait < 0:
        parser.error("--arm-wait must be non-negative")
    if args.arm_hover_wait < 0:
        parser.error("--arm-hover-wait must be non-negative")
    if args.takeoff_delay_frames < 0:
        parser.error("--takeoff-delay-frames must be non-negative")
    if args.takeoff_boost_frames < 0:
        parser.error("--takeoff-boost-frames must be non-negative")
    if not -1.0 <= args.settle_throttle <= 1.0:
        parser.error("--settle-throttle must be within [-1.0, 1.0]")
    if args.alt_target <= 0:
        parser.error("--alt-target must be positive")
    if args.alt_kp <= 0:
        parser.error("--alt-kp must be positive")
    if not -1.0 <= args.arm_throttle <= 1.0:
        parser.error("--arm-throttle must be within [-1.0, 1.0]")
    if args.bbox and args.bbox_file:
        parser.error("use either --bbox or --bbox-file, not both")
    if args.bbox_file:
        args.bbox = load_bbox_file(args.bbox_file)
    return args


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    result = run_tracking_probe(
        run_id=args.run_id,
        log_dir=args.log_dir,
        region=args.region,
        crop=args.crop,
        window_titles=args.window_titles,
        exact=args.window_exact,
        case_sensitive=args.window_case_sensitive,
        min_width=args.window_min_width,
        min_height=args.window_min_height,
        preferred_size=args.preferred_size,
        allow_common_non_fpv_windows=args.allow_common_non_fpv_windows,
        backend=args.backend,
        bbox=args.bbox,
        duration_s=args.duration,
        hz=args.hz,
        tracker_type=args.tracker,
        enable_pitch=args.enable_pitch,
        pitch_delay_frames=args.pitch_delay_frames,
        use_uinput=args.uinput,
        min_found_ratio=args.min_found_ratio,
        max_loss_events=args.max_loss_events,
        max_abs_yaw_axis=args.max_abs_yaw_axis,
        max_abs_pitch_axis=args.max_abs_pitch_axis,
        desired_target_width=args.desired_target_width,
        arm_first=args.arm_first,
        arm_wait_s=args.arm_wait,
        arm_throttle=args.arm_throttle,
        arm_hover_wait_s=args.arm_hover_wait,
        takeoff_delay_frames=args.takeoff_delay_frames,
        takeoff_boost_frames=args.takeoff_boost_frames,
        takeoff_ramp_frames=args.takeoff_ramp_frames,
        settle_throttle=args.settle_throttle,
        alt_hold=args.alt_hold,
        alt_target_m=args.alt_target,
        alt_kp=args.alt_kp,
        track_throttle_hold=args.track_throttle_hold,
        track_throttle_kp=args.track_throttle_kp,
        ws_host=args.ws_host,
        ws_port=args.ws_port,
    )
    json_path, md_path = write_reports(result, args.log_dir, run_id=args.run_id)
    print("simitl-pr0p-tracking %s report=%s summary=%s real_input_sent=%s" % (
        result.status,
        json_path,
        md_path,
        result.metrics.get("real_input_sent"),
    ))
    print(result.summary)
    return 1 if result.status == FAIL else 0


if __name__ == "__main__":
    raise SystemExit(main())
