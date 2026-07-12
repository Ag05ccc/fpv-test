#!/usr/bin/env python3
"""Measure the live Gazebo FPV camera topic and gate it (roadmap-v2 A2).

Subscribes to the camera topic injected by run_gazebo_betaflight.sh
--iris-forward-camera (default /kenet/fpv_camera) through the same
kenet.gz_camera bridge that the pipeline and the SITL mixer use, samples for a
fixed window, and produces a PASS/FAIL verdict:

- frames arrive at all,
- measured frame rate >= --min-fps,
- resolution matches --expect-width/--expect-height (when nonzero),
- no decode errors,
- the last frame is fresher than --max-frame-age at window end.

Exit code 0 on PASS, 1 on FAIL. This is the metric-9 gate: "the sim camera
image is consumable by the tracker side".
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parent
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from kenet.camera import GZ_SOURCE_PREFIX, create_camera_capture  # noqa: E402


def evaluate_probe(
    *,
    frame_count: int,
    duration: float,
    frame_shape: tuple[int, int] | None,
    decode_errors: int,
    last_frame_age: float | None,
    min_fps: float,
    expect_width: int,
    expect_height: int,
    max_frame_age: float,
) -> dict:
    """Pure verdict logic so the gate itself is unit-testable offline."""
    measured_fps = frame_count / duration if duration > 0 else 0.0
    failures = []
    if frame_count <= 0:
        failures.append("no frames received")
    if measured_fps < min_fps:
        failures.append("fps %.2f < min %.2f" % (measured_fps, min_fps))
    if frame_count > 0 and frame_shape is not None:
        height, width = frame_shape
        if expect_width and width != expect_width:
            failures.append("width %d != expected %d" % (width, expect_width))
        if expect_height and height != expect_height:
            failures.append("height %d != expected %d" % (height, expect_height))
    if decode_errors:
        failures.append("%d frame decode errors" % decode_errors)
    if frame_count > 0 and last_frame_age is not None and last_frame_age > max_frame_age:
        failures.append(
            "last frame is stale: %.2fs > %.2fs" % (last_frame_age, max_frame_age))
    return {
        "ok": not failures,
        "failures": failures,
        "frame_count": frame_count,
        "duration": duration,
        "measured_fps": measured_fps,
        "frame_shape": list(frame_shape) if frame_shape else None,
        "decode_errors": decode_errors,
        "last_frame_age": last_frame_age,
    }


def run_probe(args, capture_factory=None) -> dict:
    source = args.topic if args.topic.startswith(GZ_SOURCE_PREFIX) else (
        GZ_SOURCE_PREFIX + args.topic)
    factory = capture_factory or create_camera_capture
    capture = factory(source)
    capture.start()
    frame_shape = None
    try:
        deadline = time.monotonic() + args.duration
        start = time.monotonic()
        while time.monotonic() < deadline:
            frame = capture.read()
            if frame is not None and frame_shape is None:
                frame_shape = frame.shape[:2]
            time.sleep(0.02)
        duration = time.monotonic() - start
        return evaluate_probe(
            frame_count=capture.frame_count,
            duration=duration,
            frame_shape=frame_shape,
            decode_errors=capture.decode_errors,
            last_frame_age=capture.last_frame_age(),
            min_fps=args.min_fps,
            expect_width=args.expect_width,
            expect_height=args.expect_height,
            max_frame_age=args.max_frame_age,
        )
    finally:
        capture.stop()


def parse_args(argv=None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--topic", default="/kenet/fpv_camera",
                        help="Gazebo camera topic (default /kenet/fpv_camera)")
    parser.add_argument("--duration", type=float, default=5.0,
                        help="Sampling window in seconds (default 5)")
    parser.add_argument("--min-fps", type=float, default=20.0,
                        help="Minimum acceptable frame rate (default 20)")
    parser.add_argument("--expect-width", type=int, default=640,
                        help="Expected frame width; 0 disables the check")
    parser.add_argument("--expect-height", type=int, default=480,
                        help="Expected frame height; 0 disables the check")
    parser.add_argument("--max-frame-age", type=float, default=1.0,
                        help="Max staleness of the last frame at window end (s)")
    parser.add_argument("--json", action="store_true",
                        help="Print the verdict as JSON")
    return parser.parse_args(argv)


def main(argv=None) -> int:
    args = parse_args(argv)
    verdict = run_probe(args)
    if args.json:
        print(json.dumps(verdict, indent=2, sort_keys=True))
    else:
        print("gz camera probe: %s" % ("PASS" if verdict["ok"] else "FAIL"))
        print("  topic=%s duration=%.1fs" % (args.topic, verdict["duration"]))
        print("  frames=%d fps=%.2f shape=%s decode_errors=%d last_age=%s" % (
            verdict["frame_count"], verdict["measured_fps"],
            verdict["frame_shape"], verdict["decode_errors"],
            "%.2fs" % verdict["last_frame_age"] if verdict["last_frame_age"] is not None else "n/a"))
        for failure in verdict["failures"]:
            print("  FAIL: %s" % failure)
    return 0 if verdict["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
