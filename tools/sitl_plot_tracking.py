#!/usr/bin/env python3
"""Render a moving-target tracking run to a PNG so you can *see* the result
without matplotlib (uses OpenCV, already a dependency).

Three stacked panels from an A4 run's logs:
  1. tracked target center X (px)  -> the box sweeping the frame = following the
     moving car,
  2. drone heading / pose yaw (deg) -> the drone rotating to follow,
  3. roll & pitch (deg)             -> staying level (no flip).

Usage:
    fpv_env/bin/python tools/sitl_plot_tracking.py \
        --mixer logs/sitl/<run>-a4-track-mixer.jsonl \
        --diagnostics logs/sitl/<run>-a4-track-diagnostics.jsonl \
        --out /tmp/tracking.png
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import cv2
import numpy as np


def read_jsonl(path):
    for line in Path(path).open(encoding="utf-8"):
        line = line.strip()
        if line:
            try:
                yield json.loads(line)
            except json.JSONDecodeError:
                pass


def series_from_mixer(path):
    t, cx = [], []
    for d in read_jsonl(path):
        if d.get("event") == "kenet_mixer_sample" and d.get("state") == "TRACKING" \
                and d.get("target_found") and d.get("target_center"):
            t.append(d.get("monotonic", len(t)))
            cx.append(d["target_center"][0])
    return t, cx


def series_from_diag(path):
    t, yaw, roll, pitch = [], [], [], []
    for d in read_jsonl(path):
        if d.get("event") != "diagnostic_sample":
            continue
        st = (d.get("msp") or {}).get("status") or {}
        if not st.get("armed"):
            continue
        euler = (((d.get("gazebo_pose") or {}).get("pose") or {}).get("euler_deg") or {})
        att = (d.get("msp") or {}).get("attitude") or {}
        t.append(d.get("time", len(t)))
        yaw.append(euler.get("yaw", 0.0))
        roll.append(att.get("roll", 0.0))
        pitch.append(att.get("pitch", 0.0))
    return t, yaw, roll, pitch


def draw_panel(canvas, y0, h, title, serieses, width, ymin=None, ymax=None):
    pad = 55
    plot_w = width - pad - 15
    cv2.putText(canvas, title, (10, y0 + 16), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                (230, 230, 230), 1, cv2.LINE_AA)
    all_vals = [v for _, ys, _ in serieses for v in ys if v is not None]
    if not all_vals:
        return
    lo = ymin if ymin is not None else min(all_vals)
    hi = ymax if ymax is not None else max(all_vals)
    if hi - lo < 1e-6:
        hi, lo = hi + 1, lo - 1
    base = y0 + h - 12
    top = y0 + 24
    # zero / mid gridline
    for gv in (lo, (lo + hi) / 2, hi):
        gy = int(base - (gv - lo) / (hi - lo) * (base - top))
        cv2.line(canvas, (pad, gy), (pad + plot_w, gy), (60, 60, 60), 1)
        cv2.putText(canvas, "%.0f" % gv, (4, gy + 4), cv2.FONT_HERSHEY_SIMPLEX,
                    0.35, (150, 150, 150), 1, cv2.LINE_AA)
    for label, ys, color in serieses:
        pts = []
        n = len(ys)
        for i, v in enumerate(ys):
            if v is None:
                continue
            x = pad + int(i / max(1, n - 1) * plot_w)
            yy = int(base - (v - lo) / (hi - lo) * (base - top))
            pts.append((x, yy))
        if len(pts) > 1:
            cv2.polylines(canvas, [np.array(pts, np.int32)], False, color, 2, cv2.LINE_AA)
        if label:
            cv2.putText(canvas, label, (pad + plot_w - 120, y0 + 16),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1, cv2.LINE_AA)


def render(mixer, diagnostics, out, title):
    width, panel_h = 900, 200
    canvas = np.full((panel_h * 3 + 40, width, 3), 24, np.uint8)
    cv2.putText(canvas, title, (10, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                (255, 255, 255), 2, cv2.LINE_AA)

    _, cx = series_from_mixer(mixer)
    _, yaw, roll, pitch = series_from_diag(diagnostics)

    draw_panel(canvas, 40, panel_h,
               "Tracked target X in frame (px) - sweeps 0..640 = following the moving car",
               [("target_x", cx, (80, 200, 255))], width, ymin=0, ymax=640)
    draw_panel(canvas, 40 + panel_h, panel_h,
               "Drone heading / pose yaw (deg) - rotating to follow the target",
               [("pose_yaw", yaw, (120, 255, 120))], width)
    draw_panel(canvas, 40 + panel_h * 2, panel_h,
               "Roll & pitch (deg) - stays near 0 = level, no flip (flip = +/-180)",
               [("roll", roll, (120, 160, 255)), ("pitch", pitch, (255, 180, 120))],
               width, ymin=-30, ymax=30)
    cv2.imwrite(str(out), canvas)
    return len(cx), len(yaw)


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--mixer", required=True)
    p.add_argument("--diagnostics", required=True)
    p.add_argument("--out", default="/tmp/tracking.png")
    p.add_argument("--title", default="Moving-target visual tracking")
    a = p.parse_args(argv)
    n_cx, n_yaw = render(a.mixer, a.diagnostics, a.out, a.title)
    print("wrote %s (%d tracked samples, %d flight samples)" % (a.out, n_cx, n_yaw))


if __name__ == "__main__":
    main()
