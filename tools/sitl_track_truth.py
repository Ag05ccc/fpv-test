#!/usr/bin/env python3
"""Ground-truth track/approach monitor: does the DRONE POSITION converge on the
target? (the correct track/control test).

Rotating toward a target ("moving a direction") is not tracking — a spinning
drone yaws through every heading and proves nothing. The real test is whether
the gap between drone and target closes for a static target (approach) or stays
bounded while the target moves (follow). Both the drone (iris) and the target
pose are ground truth on Gazebo's /world/<world>/pose/info at ~50 Hz, so we
measure position directly instead of inferring from heading.

Per sample it records drone xyz + yaw and target xyz, and computes:
  - horizontal distance drone->target,
  - yaw rate (to flag a spin, which is the failure that masqueraded as tracking),
  - closing speed = -d(distance)/dt projected sanity.

Verdict inputs:
  - approach: min distance < initial distance - --min-approach-m (drone got
    meaningfully closer),  OR for a moving target, final distance <=
    initial + --max-distance-growth-m (kept up),
  - no spin: peak |yaw rate| <= --max-yaw-rate,
  - the drone actually TRANSLATED toward the target (net displacement of the
    drone projected on the initial drone->target direction > --min-translation-m),
    so yawing/circling in place cannot pass.

    /usr/bin/python3 tools/sitl_track_truth.py --world betaloop_demo \
        --drone iris --target car_front_1 --duration 60
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from sitl_log import JsonlLogger, make_log_path, resolve_log_dir  # noqa: E402

GZ_DIST = "/usr/lib/python3/dist-packages"


def horizontal_distance(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def quat_to_yaw_deg(w, x, y, z):
    return math.degrees(math.atan2(2.0 * (w * z + x * y),
                                   1.0 - 2.0 * (y * y + z * z)))


def _path_length(points):
    """Sum of consecutive horizontal step lengths (0 for a stationary body)."""
    total = 0.0
    for a, b in zip(points, points[1:]):
        total += math.hypot(b[0] - a[0], b[1] - a[1])
    return total


def evaluate_convergence(samples, *, min_approach_m, max_distance_growth_m,
                         max_yaw_rate, min_translation_m, static_target_m=0.5,
                         min_target_motion_m=1.0):
    """samples: list of dicts with keys t, drone(x,y,z,yaw), target(x,y,z), dist.
    Pure verdict logic (unit-testable).

    The drone must move THROUGH SPACE toward/with the target, not just rotate:
      - static target -> the drone must genuinely approach (distance closes),
      - moving target -> the drone must keep the distance bounded while the
        target actually moves,
      - either way the drone must translate (path length), so spinning or
        hovering in place is a FAIL, and yaw rate must stay sane."""
    failures = []
    if len(samples) < 2:
        return {"ok": False, "failures": ["not enough samples"], "samples": len(samples)}
    dists = [s["dist"] for s in samples]
    initial, final, minimum = dists[0], dists[-1], min(dists)
    peak_rate = 0.0
    for a, b in zip(samples, samples[1:]):
        dt = b["t"] - a["t"]
        if dt <= 0:
            continue
        dy = (b["drone"][3] - a["drone"][3] + 180.0) % 360.0 - 180.0
        peak_rate = max(peak_rate, abs(dy / dt))
    drone_path = _path_length([s["drone"] for s in samples])
    target_path = _path_length([s["target"] for s in samples])
    target_moving = target_path >= min_target_motion_m

    if target_moving:
        kept_up = final <= initial + max_distance_growth_m
        if not kept_up:
            failures.append(
                "did not keep up with moving target: initial %.2f m -> final "
                "%.2f m (grew > %.2f); target moved %.2f m"
                % (initial, final, max_distance_growth_m, target_path))
    else:
        approached = (initial - minimum) >= min_approach_m
        if not approached:
            failures.append(
                "distance to static target did not close: initial %.2f m, min "
                "%.2f m (needed to drop >= %.2f m)"
                % (initial, minimum, min_approach_m))
    if peak_rate > max_yaw_rate:
        failures.append("yaw spin: peak %.0f deg/s > %.0f (rotating, not tracking)"
                        % (peak_rate, max_yaw_rate))
    if drone_path < min_translation_m:
        failures.append(
            "drone did not move through space: path length %.2f m < %.2f "
            "(hovering/spinning in place is not tracking)"
            % (drone_path, min_translation_m))
    return {
        "ok": not failures,
        "failures": failures,
        "samples": len(samples),
        "initial_distance_m": initial,
        "min_distance_m": minimum,
        "final_distance_m": final,
        "distance_closed_m": initial - minimum,
        "drone_path_m": drone_path,
        "target_path_m": target_path,
        "target_moving": target_moving,
        "peak_yaw_rate_dps": peak_rate,
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
    latest = {}
    samples = []
    logger = None
    if not args.no_log:
        path = Path(args.log_file) if args.log_file else make_log_path(
            resolve_log_dir(args.log_dir), "track-truth-%s" % args.target)
        logger = JsonlLogger(path, metadata={"tool": "sitl_track_truth",
                             "world": args.world, "drone": args.drone,
                             "target": args.target})
        print("track-truth log: %s" % path, file=sys.stderr)
    state = {"last_print": 0.0}

    def on_pose(msg):
        now = time.monotonic()
        for pose in msg.pose:
            if pose.name in (args.drone, args.target):
                p = pose.position
                if pose.name == args.drone:
                    q = pose.orientation
                    latest["drone"] = (p.x, p.y, p.z, quat_to_yaw_deg(q.w, q.x, q.y, q.z))
                else:
                    latest["target"] = (p.x, p.y, p.z)
        if "drone" in latest and "target" in latest:
            dist = horizontal_distance(latest["drone"], latest["target"])
            samples.append({"t": now, "drone": latest["drone"],
                            "target": latest["target"], "dist": dist})
            if logger:
                logger.write("track_truth_sample", drone=latest["drone"],
                             target=latest["target"], distance_m=dist)
            if now - state["last_print"] >= args.print_interval:
                state["last_print"] = now
                d = latest["drone"]
                print("dist=%6.2f m  drone=(%.1f,%.1f,%.1f) yaw=%+.0f  target=(%.1f,%.1f)"
                      % (dist, d[0], d[1], d[2], d[3],
                         latest["target"][0], latest["target"][1]), flush=True)

    node.subscribe(Pose_V, "/world/%s/pose/info" % args.world, on_pose)
    deadline = time.monotonic() + args.duration if args.duration > 0 else None
    try:
        while deadline is None or time.monotonic() < deadline:
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    verdict = evaluate_convergence(
        samples, min_approach_m=args.min_approach_m,
        max_distance_growth_m=args.max_distance_growth_m,
        max_yaw_rate=args.max_yaw_rate, min_translation_m=args.min_translation_m)
    if logger:
        logger.write("track_truth_summary", summary=verdict)
        logger.close()
    return verdict


def parse_args(argv=None):
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--world", default="betaloop_demo")
    p.add_argument("--drone", default="iris")
    p.add_argument("--target", default="car_front_1")
    p.add_argument("--duration", type=float, default=0.0)
    p.add_argument("--min-approach-m", type=float, default=2.0,
                   help="Static target: min distance must drop this far below initial")
    p.add_argument("--max-distance-growth-m", type=float, default=1.0,
                   help="Moving target: final distance may exceed initial by at most this")
    p.add_argument("--min-translation-m", type=float, default=1.0,
                   help="Net drone translation toward target required (rules out spin-in-place)")
    p.add_argument("--max-yaw-rate", type=float, default=90.0)
    p.add_argument("--print-interval", type=float, default=2.0)
    p.add_argument("--log-dir", default=None)
    p.add_argument("--log-file", default=None)
    p.add_argument("--no-log", action="store_true")
    return p.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    v = run_monitor(args)
    print("\ntrack/approach truth: %s" % ("PASS" if v["ok"] else "FAIL"))
    if v.get("samples", 0) >= 2:
        print("  distance to target: initial %.2f -> min %.2f -> final %.2f m (closed %.2f m)"
              % (v["initial_distance_m"], v["min_distance_m"], v["final_distance_m"],
                 v["distance_closed_m"]))
        print("  drone path length: %.2f m | target moved: %.2f m | peak yaw rate: %.0f deg/s"
              % (v["drone_path_m"], v["target_path_m"], v["peak_yaw_rate_dps"]))
    for f in v["failures"]:
        print("  FAIL: %s" % f)
    return 0 if v["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
