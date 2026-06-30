#!/usr/bin/env python3
"""Run the no-hardware P6 mixer safety matrix for Kenet SITL."""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path
from types import SimpleNamespace


SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from kenet.pipeline import AI_ARMED, IDLE, TRACKING  # noqa: E402
from kenet.state_machine import STATE_NAMES  # noqa: E402
from kenet.tracker import TrackResult  # noqa: E402
from kenet_sitl_mixer import KenetSitlMixer  # noqa: E402


ROLL_CH = 0
PITCH_CH = 1
THROTTLE_CH = 2
YAW_CH = 3
ARM_CH = 4
KENET_CH = 5
MODE_CH = 6


def make_args(**overrides) -> SimpleNamespace:
    values = {
        "camera": "0",
        "frame_width": 640,
        "frame_height": 480,
        "camera_fps": 30,
        "tracker": "CSRT",
        "loop_hz": 10.0,
        "preview": False,
        "aux_ch": KENET_CH,
        "force_mode_pwm": None,
        "track_size": 100,
        "desired_target_width": 120.0,
        "yaw_kp": 0.8,
        "yaw_ki": 0.05,
        "yaw_kd": 0.15,
        "yaw_limit": 300.0,
        "forward_kp": 0.4,
        "forward_ki": 0.02,
        "forward_kd": 0.1,
        "forward_limit": 250.0,
        "device": "/dev/input/js0",
        "pilot_source": "virtual",
        "virtual_script": "manual",
        "virtual_roll": 1500,
        "virtual_pitch": 1500,
        "virtual_throttle": 1000,
        "virtual_yaw": 1500,
        "virtual_arm_pwm": 1000,
        "virtual_kenet_pwm": 1000,
        "virtual_kenet_pre_pwm": 1000,
        "virtual_kenet_delay_seconds": 0.0,
        "virtual_mode_pwm": 1500,
        "virtual_low_seconds": 5.0,
        "virtual_arm_seconds": 3.0,
        "virtual_ramp_seconds": 3.0,
        "virtual_hold_seconds": 8.0,
        "virtual_disarm_seconds": 1.0,
        "send": False,
        "no_vision": True,
        "synthetic_target": False,
        "synthetic_target_x": 420.0,
        "synthetic_target_y": 240.0,
        "synthetic_target_width": 80.0,
        "synthetic_target_height": 80.0,
        "synthetic_target_delay_seconds": 0.0,
        "synthetic_elapsed_seconds": 0.0,
        "aux_arm_threshold": 1300,
        "aux_track_threshold": 1700,
        "lost_seconds": 0.1,
        "print_hz": 5.0,
        "flight_log_hz": 10.0,
        "flight_log_flush_every": 10,
        "flight_log_flush_seconds": 1.0,
        "flight_log_max_mb": 50.0,
        "flight_log": None,
        "log_dir": None,
        "no_flight_log": True,
        "host": "127.0.0.1",
        "port": 9004,
        "duration": 0.0,
        "log_level": "INFO",
    }
    values.update(overrides)
    return SimpleNamespace(**values)


def pilot_channels(*, kenet_pwm: int, roll: int = 1500, throttle: int = 1000, arm: int = 1000) -> list[int]:
    channels = [1500] * 16
    channels[ROLL_CH] = roll
    channels[PITCH_CH] = 1500
    channels[THROTTLE_CH] = throttle
    channels[YAW_CH] = 1500
    channels[ARM_CH] = arm
    channels[KENET_CH] = kenet_pwm
    channels[MODE_CH] = 1500
    return channels


def changed_channels(pilot: list[int], final: list[int]) -> list[int]:
    return [index for index, value in enumerate(final[:8]) if value != pilot[index]]


def run_tracking_found_case() -> tuple[KenetSitlMixer, list[int], list[int], TrackResult]:
    mixer = KenetSitlMixer(make_args(synthetic_target=True, no_vision=False))
    pilot = pilot_channels(kenet_pwm=2000, roll=1650, throttle=1425, arm=2000)
    first = mixer._update_vision_state(None, pilot)
    mixer.controller._prev_time = time.monotonic() - 0.1
    mixer.controller.yaw_pid._prev_time = time.monotonic() - 0.1
    mixer.controller.forward_pid._prev_time = time.monotonic() - 0.1
    result = mixer._update_vision_state(None, pilot)
    final = mixer._mix_channels(pilot, result)
    if not first.found:
        raise AssertionError("synthetic target first sample not found")
    return mixer, pilot, final, result


def matrix_results() -> list[dict[str, object]]:
    rows: list[dict[str, object]] = []

    for name, kenet_pwm, expected_state in (
        ("P6.1 AUX2 LOW", 1000, IDLE),
        ("P6.2 AUX2 MID", 1500, AI_ARMED),
    ):
        mixer = KenetSitlMixer(make_args(no_vision=True))
        pilot = pilot_channels(kenet_pwm=kenet_pwm, roll=1640, throttle=1410)
        result = mixer._update_vision_state(None, pilot)
        final = mixer._mix_channels(pilot, result)
        rows.append({
            "name": name,
            "ok": mixer.state == expected_state and final == pilot and mixer.last_source == "pilot",
            "detail": "state=%s source=%s changed=%s" % (
                STATE_NAMES[mixer.state],
                mixer.last_source,
                changed_channels(pilot, final),
            ),
        })

    mixer = KenetSitlMixer(make_args(no_vision=True))
    pilot = pilot_channels(kenet_pwm=2000, roll=1660, throttle=1430)
    result = mixer._update_vision_state(None, pilot)
    final = mixer._mix_channels(pilot, result)
    rows.append({
        "name": "P6.3 AUX2 HIGH target lost",
        "ok": mixer.state == TRACKING and not result.found and final == pilot and mixer.last_source == "pilot-target-lost",
        "detail": "state=%s source=%s changed=%s" % (
            STATE_NAMES[mixer.state],
            mixer.last_source,
            changed_channels(pilot, final),
        ),
    })

    mixer, pilot, final, result = run_tracking_found_case()
    rows.append({
        "name": "P6.4 AUX2 HIGH target found",
        "ok": (
            mixer.state == TRACKING
            and result.found
            and mixer.last_source == "kenet"
            and set(changed_channels(pilot, final)) == {PITCH_CH, YAW_CH}
        ),
        "detail": "state=%s source=%s changed=%s pitch=%d yaw=%d" % (
            STATE_NAMES[mixer.state],
            mixer.last_source,
            changed_channels(pilot, final),
            final[PITCH_CH] - pilot[PITCH_CH],
            final[YAW_CH] - pilot[YAW_CH],
        ),
    })

    rows.append({
        "name": "P6.5 ARM low/high channel contract",
        "ok": pilot_channels(kenet_pwm=1000, arm=1000)[ARM_CH] == 1000
        and pilot_channels(kenet_pwm=1000, arm=2000)[ARM_CH] == 2000,
        "detail": "CH5 low=%d high=%d" % (
            pilot_channels(kenet_pwm=1000, arm=1000)[ARM_CH],
            pilot_channels(kenet_pwm=1000, arm=2000)[ARM_CH],
        ),
    })

    rows.append({
        "name": "P6.6 throttle remains pilot in TRACKING",
        "ok": final[THROTTLE_CH] == pilot[THROTTLE_CH],
        "detail": "pilot_throttle=%d final_throttle=%d" % (pilot[THROTTLE_CH], final[THROTTLE_CH]),
    })
    rows.append({
        "name": "P6.7 roll remains pilot in TRACKING",
        "ok": final[ROLL_CH] == pilot[ROLL_CH],
        "detail": "pilot_roll=%d final_roll=%d" % (pilot[ROLL_CH], final[ROLL_CH]),
    })

    mixer = KenetSitlMixer(make_args(no_vision=True))
    mixer.state = TRACKING
    pilot = pilot_channels(kenet_pwm=2000, roll=1675, throttle=1450)
    final = mixer._mix_channels(pilot, TrackResult(found=False))
    rows.append({
        "name": "P6.8 target loss releases pitch/yaw",
        "ok": final == pilot and mixer.last_source == "pilot-target-lost",
        "detail": "source=%s changed=%s" % (mixer.last_source, changed_channels(pilot, final)),
    })

    return rows


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    return argparse.ArgumentParser(description=__doc__).parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    parse_args(argv)
    rows = matrix_results()
    for row in rows:
        print("%s %s %s" % ("PASS" if row["ok"] else "FAIL", row["name"], row["detail"]))
    return 0 if all(row["ok"] for row in rows) else 1


if __name__ == "__main__":
    raise SystemExit(main())
