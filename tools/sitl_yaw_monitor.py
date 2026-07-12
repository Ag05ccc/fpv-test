#!/usr/bin/env python3
"""Real-time high-rate yaw monitor for the SITL drone (observability fix).

Why this exists: the diagnostics poller reads attitude over MSP at ~2-3 Hz, so
a fast yaw spin (measured up to 436 deg/s) is aliased into a slow/flat signal
and slips past roll/pitch-only flip gates (a spin stays level). This tool
subscribes DIRECTLY to Gazebo's pose stream (/world/<world>/pose/info, ~50 Hz)
for the drone, unwraps yaw, and computes yaw rate every sample — the rate at
which the drone is actually rotating.

It prints a live line, logs JSONL (event yaw_sample / yaw_summary), and returns
a verdict: SPIN if the max sustained |yaw rate| exceeds --max-yaw-rate. Run it
alongside any flight to see, in real time, whether the drone is spinning.

    /usr/bin/python3 tools/sitl_yaw_monitor.py --world betaloop_demo --model iris

(Uses the system python for gz bindings, or fpv_env with dist-packages on path.)
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from sitl_log import JsonlLogger, make_log_path, resolve_log_dir  # noqa: E402

GZ_DIST = "/usr/lib/python3/dist-packages"


def quat_to_yaw_deg(w, x, y, z):
    """Yaw (deg) about world +Z from a quaternion, pure math (unit-testable)."""
    import math
    siny = 2.0 * (w * z + x * y)
    cosy = 1.0 - 2.0 * (y * y + z * z)
    return math.degrees(math.atan2(siny, cosy))


def wrap180(delta):
    return (delta + 180.0) % 360.0 - 180.0


def summarize_rates(samples):
    """samples: list of (t, yaw_deg). Return rate stats + spin verdict inputs."""
    rates = []
    total = 0.0
    for (t0, y0), (t1, y1) in zip(samples, samples[1:]):
        dt = t1 - t0
        if dt <= 0:
            continue
        dy = wrap180(y1 - y0)
        total += dy
        rates.append(dy / dt)
    if not rates:
        return {"samples": len(samples), "max_abs_rate": 0.0,
                "mean_abs_rate": 0.0, "total_yaw_deg": total}
    return {
        "samples": len(samples),
        "max_abs_rate": max(abs(r) for r in rates),
        "mean_abs_rate": sum(abs(r) for r in rates) / len(rates),
        "total_yaw_deg": total,
    }


def _import_gz():
    import os
    try:
        from gz.transport13 import Node
        from gz.msgs10.pose_v_pb2 import Pose_V
    except ImportError:
        if GZ_DIST not in sys.path and os.path.isdir(GZ_DIST):
            sys.path.append(GZ_DIST)
        from gz.transport13 import Node
        from gz.msgs10.pose_v_pb2 import Pose_V
    return Node, Pose_V


def run_monitor(args):
    Node, Pose_V = _import_gz()
    node = Node()
    samples = []
    state = {"last_print": 0.0, "max_rate": 0.0}
    logger = None
    if not args.no_log:
        path = Path(args.log_file) if args.log_file else make_log_path(
            resolve_log_dir(args.log_dir), "yaw-monitor-%s" % args.model)
        logger = JsonlLogger(path, metadata={"tool": "sitl_yaw_monitor",
                             "world": args.world, "model": args.model})
        print("yaw log: %s" % path, file=sys.stderr)

    def on_pose(msg):
        now = time.monotonic()
        for pose in msg.pose:
            if pose.name != args.model:
                continue
            q = pose.orientation
            yaw = quat_to_yaw_deg(q.w, q.x, q.y, q.z)
            if samples:
                dt = now - samples[-1][0]
                rate = wrap180(yaw - samples[-1][1]) / dt if dt > 0 else 0.0
            else:
                rate = 0.0
            samples.append((now, yaw))
            state["max_rate"] = max(state["max_rate"], abs(rate))
            if logger:
                logger.write("yaw_sample", yaw_deg=yaw, yaw_rate_dps=rate)
            if now - state["last_print"] >= args.print_interval:
                state["last_print"] = now
                flag = "  <-- SPIN" if abs(rate) > args.max_yaw_rate else ""
                print("yaw=%+7.1f deg   rate=%+8.1f deg/s   peak=%.0f deg/s%s"
                      % (yaw, rate, state["max_rate"], flag), flush=True)

    node.subscribe(Pose_V, "/world/%s/pose/info" % args.world, on_pose)
    deadline = time.monotonic() + args.duration if args.duration > 0 else None
    try:
        while deadline is None or time.monotonic() < deadline:
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    summary = summarize_rates(samples)
    summary["spin"] = summary["max_abs_rate"] > args.max_yaw_rate
    summary["max_yaw_rate_threshold"] = args.max_yaw_rate
    if logger:
        logger.write("yaw_summary", summary=summary)
        logger.close()
    return summary


def parse_args(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--world", default="betaloop_demo")
    p.add_argument("--model", default="iris")
    p.add_argument("--duration", type=float, default=0.0, help="0 = until Ctrl-C")
    p.add_argument("--max-yaw-rate", type=float, default=60.0,
                   help="deg/s above which a sustained rate is called a SPIN")
    p.add_argument("--print-interval", type=float, default=0.3)
    p.add_argument("--log-dir", default=None)
    p.add_argument("--log-file", default=None)
    p.add_argument("--no-log", action="store_true")
    return p.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    summary = run_monitor(args)
    print("\nyaw monitor: %s" % ("SPIN DETECTED" if summary["spin"] else "no spin"))
    print("  samples=%d  peak yaw rate=%.1f deg/s  mean=%.1f deg/s  total yaw=%.0f deg"
          % (summary["samples"], summary["max_abs_rate"],
             summary["mean_abs_rate"], summary["total_yaw_deg"]))
    print("  (threshold %.0f deg/s)" % summary["max_yaw_rate_threshold"])
    return 1 if summary["spin"] else 0


if __name__ == "__main__":
    raise SystemExit(main())
