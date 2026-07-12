#!/usr/bin/env python3
"""Offline tracker motion benchmark (roadmap-v2 A2, metric 7).

Answers "does the tracker stay locked under smooth motion, or does it break on
simple movement?" with numbers instead of anecdotes: renders a deterministic
synthetic sequence (textured target over a textured background, known
trajectory), runs the real OpenCV tracker (CSRT/KCF) on it, and gates on

- found ratio (tracker reported success on >= --min-found-ratio of frames),
- mean/max center error in pixels against ground truth,
- mean IoU between the tracked bbox and the ground-truth bbox.

The sequence generator is seeded, so a FAIL is reproducible bit-for-bit.
Smooth horizontal drift plus optional per-frame scale change approximates a
slow crossing/approaching vehicle; speeds are in px/frame, so the same
physical speed can be mapped from any camera model.

Example:

    fpv_env/bin/python tools/tracker_motion_benchmark.py \
        --tracker CSRT --frames 90 --speed-px 3 --scale-per-frame 1.002

Exit code 0 on PASS, 1 on FAIL.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from kenet.tracker import ObjectTracker, TrackerType  # noqa: E402


def make_background(width: int, height: int, rng: np.random.Generator) -> np.ndarray:
    """Low-frequency textured background (so KCF/CSRT have context, but the
    target patch stays the most distinctive structure)."""
    coarse = rng.integers(40, 120, size=(height // 16 + 1, width // 16 + 1, 3),
                          dtype=np.uint8)
    background = np.kron(coarse, np.ones((16, 16, 1), dtype=np.uint8))
    return background[:height, :width].copy()


def make_target_patch(size: int) -> np.ndarray:
    """High-contrast checkerboard with a colored border: trackable texture."""
    patch = np.zeros((size, size, 3), dtype=np.uint8)
    cell = max(2, size // 8)
    for y in range(0, size, cell):
        for x in range(0, size, cell):
            if ((x // cell) + (y // cell)) % 2 == 0:
                patch[y:y + cell, x:x + cell] = (230, 230, 230)
            else:
                patch[y:y + cell, x:x + cell] = (30, 30, 30)
    border = max(1, size // 12)
    patch[:border, :] = (0, 0, 220)
    patch[-border:, :] = (0, 0, 220)
    patch[:, :border] = (0, 0, 220)
    patch[:, -border:] = (0, 0, 220)
    return patch


def ground_truth_bbox(frame_index: int, *, start_x: int, start_y: int,
                      base_size: int, speed_px: float,
                      scale_per_frame: float) -> tuple[int, int, int, int]:
    size = int(round(base_size * (scale_per_frame ** frame_index)))
    x = int(round(start_x + speed_px * frame_index))
    return x, start_y, size, size


def generate_sequence(*, width: int, height: int, frames: int, base_size: int,
                      speed_px: float, scale_per_frame: float, seed: int = 7):
    """Yield (frame, gt_bbox) pairs. Deterministic for a given seed."""
    import cv2

    rng = np.random.default_rng(seed)
    background = make_background(width, height, rng)
    start_x = width // 6
    start_y = height // 2 - base_size // 2
    for i in range(frames):
        x, y, w, h = ground_truth_bbox(
            i, start_x=start_x, start_y=start_y, base_size=base_size,
            speed_px=speed_px, scale_per_frame=scale_per_frame)
        if x < 0 or y < 0 or x + w > width or y + h > height:
            return
        frame = background.copy()
        patch = make_target_patch(base_size)
        if (w, h) != (base_size, base_size):
            patch = cv2.resize(patch, (w, h), interpolation=cv2.INTER_LINEAR)
        frame[y:y + h, x:x + w] = patch
        yield frame, (x, y, w, h)


def bbox_center(bbox) -> tuple[float, float]:
    x, y, w, h = bbox
    return x + w / 2.0, y + h / 2.0


def bbox_iou(a, b) -> float:
    ax, ay, aw, ah = a
    bx, by, bw, bh = b
    inter_w = max(0.0, min(ax + aw, bx + bw) - max(ax, bx))
    inter_h = max(0.0, min(ay + ah, by + bh) - max(ay, by))
    inter = inter_w * inter_h
    union = aw * ah + bw * bh - inter
    return inter / union if union > 0 else 0.0


def evaluate_benchmark(results: list[dict], *, min_found_ratio: float,
                       max_mean_center_error: float, max_center_error: float,
                       min_mean_iou: float) -> dict:
    failures = []
    if not results:
        return {"ok": False, "failures": ["no frames were produced"],
                "frames": 0}
    found = [r for r in results if r["found"]]
    found_ratio = len(found) / len(results)
    center_errors = [r["center_error"] for r in found]
    ious = [r["iou"] for r in found]
    mean_error = float(np.mean(center_errors)) if center_errors else None
    peak_error = float(np.max(center_errors)) if center_errors else None
    mean_iou = float(np.mean(ious)) if ious else None
    if found_ratio < min_found_ratio:
        failures.append("found ratio %.2f < %.2f (tracker broke lock)"
                        % (found_ratio, min_found_ratio))
    if mean_error is None:
        failures.append("tracker never reported a target")
    else:
        if mean_error > max_mean_center_error:
            failures.append("mean center error %.1f px > %.1f px"
                            % (mean_error, max_mean_center_error))
        if peak_error > max_center_error:
            failures.append("max center error %.1f px > %.1f px"
                            % (peak_error, max_center_error))
        if mean_iou < min_mean_iou:
            failures.append("mean IoU %.2f < %.2f" % (mean_iou, min_mean_iou))
    return {
        "ok": not failures,
        "failures": failures,
        "frames": len(results),
        "found_ratio": found_ratio,
        "mean_center_error": mean_error,
        "max_center_error": peak_error,
        "mean_iou": mean_iou,
    }


def run_benchmark(args) -> dict:
    tracker = ObjectTracker(args.tracker)
    results = []
    for i, (frame, gt_bbox) in enumerate(generate_sequence(
            width=args.width, height=args.height, frames=args.frames,
            base_size=args.target_size, speed_px=args.speed_px,
            scale_per_frame=args.scale_per_frame, seed=args.seed)):
        if i == 0:
            tracker.init(frame, gt_bbox)
            continue
        result = tracker.update(frame)
        entry = {"frame": i, "found": result.found,
                 "center_error": None, "iou": None}
        if result.found:
            gx, gy = bbox_center(gt_bbox)
            tx, ty = result.center
            entry["center_error"] = float(np.hypot(tx - gx, ty - gy))
            entry["iou"] = bbox_iou(result.bbox, gt_bbox)
        results.append(entry)
    verdict = evaluate_benchmark(
        results, min_found_ratio=args.min_found_ratio,
        max_mean_center_error=args.max_mean_center_error,
        max_center_error=args.max_center_error,
        min_mean_iou=args.min_mean_iou)
    verdict["tracker"] = args.tracker
    verdict["speed_px"] = args.speed_px
    verdict["scale_per_frame"] = args.scale_per_frame
    return verdict


def parse_args(argv=None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--tracker", choices=(TrackerType.CSRT, TrackerType.KCF),
                        default=TrackerType.CSRT)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--frames", type=int, default=90)
    parser.add_argument("--target-size", type=int, default=80)
    parser.add_argument("--speed-px", type=float, default=3.0,
                        help="Horizontal drift per frame (px)")
    parser.add_argument("--scale-per-frame", type=float, default=1.0,
                        help="Per-frame scale factor (approach = >1)")
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--min-found-ratio", type=float, default=1.0)
    parser.add_argument("--max-mean-center-error", type=float, default=20.0)
    parser.add_argument("--max-center-error", type=float, default=40.0)
    parser.add_argument("--min-mean-iou", type=float, default=0.5)
    parser.add_argument("--json", action="store_true")
    return parser.parse_args(argv)


def main(argv=None) -> int:
    args = parse_args(argv)
    verdict = run_benchmark(args)
    if args.json:
        print(json.dumps(verdict, indent=2, sort_keys=True))
    else:
        print("tracker motion benchmark: %s" % ("PASS" if verdict["ok"] else "FAIL"))
        print("  tracker=%s frames=%d speed=%.1f px/f scale=%.4f" % (
            verdict["tracker"], verdict["frames"], verdict["speed_px"],
            verdict["scale_per_frame"]))
        print("  found_ratio=%.2f mean_err=%s max_err=%s mean_iou=%s" % (
            verdict["found_ratio"],
            "%.1f px" % verdict["mean_center_error"]
            if verdict["mean_center_error"] is not None else "n/a",
            "%.1f px" % verdict["max_center_error"]
            if verdict["max_center_error"] is not None else "n/a",
            "%.2f" % verdict["mean_iou"]
            if verdict["mean_iou"] is not None else "n/a"))
        for failure in verdict["failures"]:
            print("  FAIL: %s" % failure)
    return 0 if verdict["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
