#!/usr/bin/env python3
"""Drive a scenery model (kenet_car, kenet_person, ...) along a scripted
trajectory in a running Gazebo world, and verify the motion actually happened
(roadmap-v2 A3, metric 8: "targets in the sim really move").

Motion is commanded through the world's /world/<world>/set_pose service
(UserCommands system, already loaded by every betaloop world), so no world or
model edits are needed and any model can be moved. Ground truth is written to
a JSONL log (event target_mover_sample) so tracker gates can compare pixel
tracks against the commanded trajectory.

Verification does not trust our own commands: the mover subscribes to
/world/<world>/pose/info and measures the model's actual displacement and the
error between commanded and observed positions. The verdict FAILs if the model
did not move at least --min-displacement, or if the mean command-vs-observed
error exceeds --max-path-error.

Example (populated world, slow car for first closed-loop gates):

    /usr/bin/python3 tools/sitl_target_mover.py \
        --world betaloop_demo --model kenet_car \
        --trajectory line --speed 0.5 --distance 10 --duration 30

Exit code 0 on PASS, 1 on FAIL.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import threading
import time
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from sitl_log import JsonlLogger, make_log_path, resolve_log_dir  # noqa: E402


# ── trajectory math (pure, unit-tested) ──────────────────────────────

def line_offset(elapsed: float, speed: float, distance: float) -> tuple[float, float, float]:
    """Ping-pong along +x: 0 -> distance -> 0 -> ... Returns (dx, dy, yaw)."""
    if distance <= 0 or speed <= 0:
        return 0.0, 0.0, 0.0
    s = (elapsed * speed) % (2.0 * distance)
    if s <= distance:
        return s, 0.0, 0.0
    return 2.0 * distance - s, 0.0, math.pi


def circle_offset(elapsed: float, speed: float, radius: float) -> tuple[float, float, float]:
    """Counter-clockwise circle starting at the model's pose, initial heading
    +x. Returns (dx, dy, yaw) with yaw = instantaneous heading."""
    if radius <= 0 or speed <= 0:
        return 0.0, 0.0, 0.0
    angle = (elapsed * speed) / radius
    dx = radius * math.sin(angle)
    dy = radius * (1.0 - math.cos(angle))
    return dx, dy, angle % (2.0 * math.pi)


def trajectory_offset(kind: str, elapsed: float, *, speed: float,
                      distance: float, radius: float) -> tuple[float, float, float]:
    if kind == "line":
        return line_offset(elapsed, speed, distance)
    if kind == "circle":
        return circle_offset(elapsed, speed, radius)
    raise ValueError("unknown trajectory kind: %s" % kind)


def yaw_to_quaternion(yaw: float) -> tuple[float, float, float, float]:
    """Return (w, x, y, z) for a pure-Z rotation."""
    return math.cos(yaw / 2.0), 0.0, 0.0, math.sin(yaw / 2.0)


# ── verdict (pure, unit-tested) ──────────────────────────────────────

def evaluate_motion(
    commanded: list[dict],
    measured: list[dict],
    *,
    min_displacement: float,
    max_path_error: float,
) -> dict:
    """commanded/measured entries: {"t": monotonic, "x": .., "y": .., "z": ..}."""
    failures = []
    if not commanded:
        failures.append("no commands were sent")
    displacement = 0.0
    mean_error = None
    if not measured:
        failures.append("no measured pose samples (pose/info silent for model)")
    else:
        first = measured[0]
        for sample in measured:
            d = math.hypot(sample["x"] - first["x"], sample["y"] - first["y"])
            displacement = max(displacement, d)
        if displacement < min_displacement:
            failures.append(
                "measured displacement %.3f m < required %.3f m"
                % (displacement, min_displacement))
        if commanded:
            errors = []
            for cmd in commanded:
                nearest = min(measured, key=lambda m: abs(m["t"] - cmd["t"]))
                if abs(nearest["t"] - cmd["t"]) > 1.0:
                    continue
                errors.append(math.hypot(nearest["x"] - cmd["x"],
                                         nearest["y"] - cmd["y"]))
            if errors:
                mean_error = sum(errors) / len(errors)
                if mean_error > max_path_error:
                    failures.append(
                        "mean command-vs-observed error %.3f m > %.3f m"
                        % (mean_error, max_path_error))
            else:
                failures.append("no time-aligned measured samples to compare")
    return {
        "ok": not failures,
        "failures": failures,
        "commanded_samples": len(commanded),
        "measured_samples": len(measured),
        "measured_displacement": displacement,
        "mean_path_error": mean_error,
    }


# ── gz transport (thin, injectable) ──────────────────────────────────

GZ_DIST_PACKAGES = "/usr/lib/python3/dist-packages"


def _import_gz():
    import os
    try:
        from gz.transport13 import Node
    except ImportError:
        if GZ_DIST_PACKAGES not in sys.path and os.path.isdir(GZ_DIST_PACKAGES):
            sys.path.append(GZ_DIST_PACKAGES)
        from gz.transport13 import Node
    from gz.msgs10.pose_pb2 import Pose
    from gz.msgs10.pose_v_pb2 import Pose_V
    from gz.msgs10.boolean_pb2 import Boolean
    return Node, Pose, Pose_V, Boolean


class GzWorldTransport:
    """set_pose requests + pose/info subscription for one world."""

    def __init__(self, world: str, request_timeout_ms: int = 250):
        node_cls, pose_cls, pose_v_cls, boolean_cls = _import_gz()
        self._node = node_cls()
        self._pose_cls = pose_cls
        self._pose_v_cls = pose_v_cls
        self._boolean_cls = boolean_cls
        self._world = world
        self._timeout_ms = request_timeout_ms
        self._lock = threading.Lock()
        self._latest = {}  # name -> (monotonic, x, y, z)
        self._node.subscribe(pose_v_cls, "/world/%s/pose/info" % world,
                             self._on_pose_v)

    def _on_pose_v(self, msg):
        now = time.monotonic()
        with self._lock:
            for pose in msg.pose:
                self._latest[pose.name] = (
                    now, pose.position.x, pose.position.y, pose.position.z)

    def get_model_pose(self, name: str, timeout: float = 3.0):
        deadline = time.monotonic() + timeout
        while True:
            with self._lock:
                entry = self._latest.get(name)
            if entry is not None:
                return {"t": entry[0], "x": entry[1], "y": entry[2], "z": entry[3]}
            if time.monotonic() >= deadline:
                return None
            time.sleep(0.05)

    def set_pose(self, name: str, x: float, y: float, z: float, yaw: float) -> bool:
        pose = self._pose_cls()
        pose.name = name
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        qw, qx, qy, qz = yaw_to_quaternion(yaw)
        pose.orientation.w = qw
        pose.orientation.x = qx
        pose.orientation.y = qy
        pose.orientation.z = qz
        ok, reply = self._node.request(
            "/world/%s/set_pose" % self._world, pose,
            self._pose_cls, self._boolean_cls, self._timeout_ms)
        return bool(ok) and bool(reply.data)


# ── mover ────────────────────────────────────────────────────────────

class TargetMover:
    """Command loop + ground-truth log + measured verification.

    transport only needs set_pose() and get_model_pose(); tests inject a fake.
    """

    def __init__(self, args, transport, logger: JsonlLogger | None = None):
        self.args = args
        self.transport = transport
        self.logger = logger
        self.commanded: list[dict] = []
        self.measured: list[dict] = []
        self.reply_timeouts = 0

    def resolve_start(self):
        if self.args.start_x is not None and self.args.start_y is not None:
            z = self.args.z if self.args.z is not None else 0.0
            return {"x": self.args.start_x, "y": self.args.start_y, "z": z}
        pose = self.transport.get_model_pose(self.args.model)
        if pose is None:
            raise RuntimeError(
                "Model %r not visible on /world/%s/pose/info; is the sim "
                "running and the model name correct?"
                % (self.args.model, self.args.world))
        if self.args.z is not None:
            pose = dict(pose, z=self.args.z)
        return pose

    def warm_up(self, start: dict, attempts: int = 20) -> None:
        """gz-transport service discovery is lazy: the first request(s) after
        node creation can time out before the channel is established (measured
        live: two 1 s timeouts, then 1 ms responses). Prime the service with
        a no-op set_pose to the start position before the timed loop."""
        yaw0 = trajectory_offset(self.args.trajectory, 0.0,
                                 speed=self.args.speed,
                                 distance=self.args.distance,
                                 radius=self.args.radius)[2]
        for _ in range(attempts):
            if self.transport.set_pose(self.args.model, start["x"], start["y"],
                                       start["z"], yaw0):
                return
            time.sleep(0.2)
        raise RuntimeError(
            "set_pose service for world %r not responding after %d attempts"
            % (self.args.world, attempts))

    def run(self) -> dict:
        start = self.resolve_start()
        self.warm_up(start)
        if self.logger:
            self.logger.write("target_mover_start", model=self.args.model,
                              world=self.args.world, start=start,
                              trajectory=self.args.trajectory,
                              speed=self.args.speed,
                              distance=self.args.distance,
                              radius=self.args.radius,
                              rate_hz=self.args.rate_hz,
                              duration=self.args.duration)
        period = 1.0 / self.args.rate_hz
        t0 = time.monotonic()
        next_tick = t0
        while True:
            elapsed = time.monotonic() - t0
            if elapsed >= self.args.duration:
                break
            dx, dy, yaw = trajectory_offset(
                self.args.trajectory, elapsed, speed=self.args.speed,
                distance=self.args.distance, radius=self.args.radius)
            x, y, z = start["x"] + dx, start["y"] + dy, start["z"]
            ok = self.transport.set_pose(self.args.model, x, y, z, yaw)
            now = time.monotonic()
            if not ok:
                # Measured live: a few % of replies get lost even on a warm
                # connection while the command itself is still applied, so a
                # lost reply is a diagnostic, not a failure. The verdict rests
                # on the MEASURED motion below.
                self.reply_timeouts += 1
            self.commanded.append({"t": now, "x": x, "y": y, "z": z})
            if self.logger:
                self.logger.write("target_mover_sample", model=self.args.model,
                                  elapsed=elapsed, ok=ok,
                                  commanded={"x": x, "y": y, "z": z, "yaw": yaw})
            measured = self.transport.get_model_pose(self.args.model, timeout=0.0) \
                if hasattr(self.transport, "get_model_pose") else None
            if measured is not None:
                self.measured.append(measured)
            # Absolute-tick pacing; skip missed ticks instead of catching up.
            # Firing bursts after a slow reply cascades into more lost
            # replies (measured live: 6% loss paced vs 56% when hammering).
            next_tick += period
            now = time.monotonic()
            if next_tick <= now:
                next_tick = now + period
            time.sleep(next_tick - now)
        verdict = evaluate_motion(
            self.commanded, self.measured,
            min_displacement=self.args.min_displacement,
            max_path_error=self.args.max_path_error)
        verdict["reply_timeouts"] = self.reply_timeouts
        if self.logger:
            self.logger.write("target_mover_summary", summary=verdict)
        return verdict


def parse_args(argv=None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--world", default="betaloop_demo",
                        help="Gazebo world name (default betaloop_demo)")
    parser.add_argument("--model", default="kenet_car",
                        help="Model name in the world (e.g. car_front_1)")
    parser.add_argument("--trajectory", choices=("line", "circle"), default="line")
    parser.add_argument("--speed", type=float, default=0.5,
                        help="Target speed in m/s (default 0.5)")
    parser.add_argument("--distance", type=float, default=10.0,
                        help="Line trajectory one-way length in m")
    parser.add_argument("--radius", type=float, default=8.0,
                        help="Circle trajectory radius in m")
    parser.add_argument("--rate-hz", type=float, default=20.0,
                        help="set_pose command rate (default 20)")
    parser.add_argument("--duration", type=float, default=30.0,
                        help="How long to drive the target (s)")
    parser.add_argument("--start-x", type=float, default=None,
                        help="Start x; default = model's current pose")
    parser.add_argument("--start-y", type=float, default=None)
    parser.add_argument("--z", type=float, default=None,
                        help="Fixed z; default = model's current z")
    parser.add_argument("--min-displacement", type=float, default=1.0,
                        help="Verification: required measured displacement (m)")
    parser.add_argument("--max-path-error", type=float, default=0.5,
                        help="Verification: mean command-vs-observed error (m)")
    parser.add_argument("--log-dir", default=None)
    parser.add_argument("--log-file", default=None,
                        help="Ground-truth JSONL path (default: auto under logs/sitl)")
    parser.add_argument("--no-log", action="store_true")
    parser.add_argument("--json", action="store_true")
    return parser.parse_args(argv)


def main(argv=None) -> int:
    args = parse_args(argv)
    transport = GzWorldTransport(args.world)
    logger = None
    if not args.no_log:
        if args.log_file:
            path = Path(args.log_file)
        else:
            path = make_log_path(resolve_log_dir(args.log_dir),
                                 "target-mover-%s" % args.model)
        logger = JsonlLogger(path, metadata={
            "tool": "sitl_target_mover",
            "world": args.world,
            "model": args.model,
            "trajectory": args.trajectory,
            "speed": args.speed,
        })
        print("ground-truth log: %s" % path, file=sys.stderr)
    try:
        verdict = TargetMover(args, transport, logger).run()
    finally:
        if logger:
            logger.close()
    if args.json:
        print(json.dumps(verdict, indent=2, sort_keys=True))
    else:
        print("target mover: %s" % ("PASS" if verdict["ok"] else "FAIL"))
        print("  model=%s world=%s trajectory=%s speed=%.2f m/s" % (
            args.model, args.world, args.trajectory, args.speed))
        print("  commanded=%d measured=%d displacement=%.3f m mean_error=%s "
              "reply_timeouts=%d" % (
                  verdict["commanded_samples"], verdict["measured_samples"],
                  verdict["measured_displacement"],
                  "%.3f m" % verdict["mean_path_error"]
                  if verdict["mean_path_error"] is not None else "n/a",
                  verdict["reply_timeouts"]))
        for failure in verdict["failures"]:
            print("  FAIL: %s" % failure)
    return 0 if verdict["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
