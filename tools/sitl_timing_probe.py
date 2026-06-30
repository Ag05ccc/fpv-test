#!/usr/bin/env python3
"""Correlated SITL timing probe.

Samples Gazebo's /stats real-time factor (RTF) over a window, logs every sample
to JSONL, and classifies whether the sim is healthy, degraded, or in the
structural RTF-collapse that makes Betaflight SITL run slow and refuse to arm.

Why this exists
---------------
The Gazebo BetaflightPlugin blocks the physics thread on a UDP motor-packet
recv every step (BetaflightPlugin.cc ReceiveMotorCommand, waitMs=1000 once the
FC is online). Betaflight SITL paces its own loop with delayMicroseconds(us /
simRate) (sitl.c), where simRate is the measured sim/real ratio. Under load
these couple into a feedback collapse: RTF drops -> simRate drops -> the FC
sleeps longer -> motor packets arrive slower -> physics blocks longer -> RTF
drops further, settling at a low fixed point (a few percent). That is NOT a
compute limit, so a capable PC still crawls.

How to isolate the cause (the decisive test)
---------------------------------------------
Run this probe with --label in two scenarios and compare the JSONL:

  1) Gazebo ALONE (Betaflight not running) at the target step size:
         python tools/sitl_timing_probe.py --label gazebo-only
     A capable PC should report RTF >> 1 (healthy). If so, physics compute is
     not the bottleneck.

  2) Gazebo + Betaflight SITL (FC online):
         python tools/sitl_timing_probe.py --label with-betaflight
     If RTF now collapses (e.g. < 0.4), the blocking motor-recv coupling is the
     structural bottleneck -- not the PC.

Repeat per --max-step-size you launched Gazebo with to find the knee where it
collapses.
"""

from __future__ import annotations

import argparse
import os
import sys
import time
from pathlib import Path


SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from gazebo_stats_monitor import choose_stats_topic, read_stats_once, parse_stats
from sitl_log import JsonlLogger, make_log_path, resolve_log_dir


def classify_rtf(rtf, collapse_rtf=0.4, degraded_rtf=0.8):
    """Bucket an RTF value. healthy >= degraded_rtf, collapsed < collapse_rtf."""
    if rtf is None:
        return "unknown"
    if rtf < collapse_rtf:
        return "collapsed"
    if rtf < degraded_rtf:
        return "degraded"
    return "healthy"


def delta_rtf(prev, cur):
    """RTF derived from the change in cumulative sim_time vs real_time between
    two /stats samples. More robust than the instantaneous real_time_factor
    field because it averages over the sample interval. Returns None if it
    cannot be computed."""
    if not prev or not cur:
        return None
    for key in ("sim_time", "real_time"):
        if prev.get(key) is None or cur.get(key) is None:
            return None
    d_real = cur["real_time"] - prev["real_time"]
    d_sim = cur["sim_time"] - prev["sim_time"]
    if d_real <= 0:
        return None
    return d_sim / d_real


def summarize(rtfs):
    if not rtfs:
        return None
    ordered = sorted(rtfs)
    n = len(ordered)
    mean = sum(ordered) / n
    return {
        "count": n,
        "min": ordered[0],
        "max": ordered[-1],
        "mean": mean,
        "median": ordered[n // 2],
        "spread": ordered[-1] - ordered[0],
    }


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("--label", default="probe",
                        help="Scenario tag stored on every JSONL row (e.g. gazebo-only, with-betaflight)")
    parser.add_argument("--topic", default="auto",
                        help="Gazebo stats topic, or 'auto' to prefer /world/*/stats")
    parser.add_argument("--samples", type=int, default=30)
    parser.add_argument("--interval", type=float, default=0.5)
    parser.add_argument("--timeout", type=float, default=2.0)
    parser.add_argument("--collapse-rtf", type=float, default=0.4,
                        help="Mean RTF below this is reported as a structural collapse")
    parser.add_argument("--degraded-rtf", type=float, default=0.8,
                        help="Mean RTF below this (but >= collapse) is reported as degraded")
    parser.add_argument("--log-dir", default=os.environ.get("KENET_SITL_LOG_DIR"),
                        help="Directory for the JSONL log; default logs/sitl")
    parser.add_argument("--no-log", action="store_true", help="Do not write a JSONL log")
    args = parser.parse_args(argv)
    if args.samples <= 0:
        parser.error("--samples must be positive")
    if args.interval < 0 or args.timeout <= 0:
        parser.error("--interval must be non-negative and --timeout positive")
    return args


def main(argv=None):
    args = parse_args(argv)
    topic = choose_stats_topic(args.topic, args.timeout)
    print("topic=%s label=%s" % (topic, args.label))

    logger = None
    if not args.no_log:
        log_dir = resolve_log_dir(args.log_dir, REPO_ROOT)
        logger = JsonlLogger(make_log_path(log_dir, "timing-probe"), metadata={
            "tool": "sitl_timing_probe",
            "label": args.label,
            "topic": topic,
            "samples": args.samples,
            "interval": args.interval,
            "collapse_rtf": args.collapse_rtf,
        })
        print("log: %s" % logger.path)

    rtfs = []
    misses = 0
    prev = None
    for index in range(args.samples):
        stats = parse_stats(read_stats_once(topic, args.timeout))
        drtf = delta_rtf(prev, stats)
        prev = stats
        rtf = stats["rtf"]
        if rtf is None:
            misses += 1
            print("sample %02d: rtf=missing (Gazebo çalışıyor mu / topic doğru mu?)" % (index + 1))
        else:
            rtfs.append(rtf)
            print("sample %02d: rtf=%.3f delta_rtf=%s step=%s iter=%s [%s]" % (
                index + 1,
                rtf,
                "-" if drtf is None else "%.3f" % drtf,
                "-" if stats["step_size"] is None else "%.6f" % stats["step_size"],
                stats["iterations"] if stats["iterations"] is not None else "-",
                classify_rtf(rtf, args.collapse_rtf, args.degraded_rtf),
            ))
        if logger:
            logger.write("timing_sample", label=args.label, delta_rtf=drtf, **stats)
        if index + 1 < args.samples:
            time.sleep(args.interval)

    summary = summarize(rtfs)
    if summary is None:
        print("result=FAIL no real_time_factor samples; is Gazebo running on this topic?")
        if logger:
            logger.write("timing_summary", label=args.label, result="FAIL", misses=misses)
            logger.close()
        return 1

    state = classify_rtf(summary["mean"], args.collapse_rtf, args.degraded_rtf)
    print("summary label=%s count=%d missing=%d min=%.3f mean=%.3f max=%.3f spread=%.3f state=%s" % (
        args.label, summary["count"], misses,
        summary["min"], summary["mean"], summary["max"], summary["spread"], state,
    ))
    if state == "collapsed":
        print("result=COLLAPSE mean RTF %.3f < %.2f. Bu compute limiti DEĞİL: aynı step ile "
              "Gazebo'yu Betaflight OLMADAN koşup karşılaştır (--label gazebo-only). Tek başına "
              "hızlıysa darboğaz BF motor-recv kuplajıdır." % (summary["mean"], args.collapse_rtf))
        rc = 2
    elif state == "degraded":
        print("result=DEGRADED mean RTF %.3f; sim gerçek zamanın altında, zamanlama hassas." % summary["mean"])
        rc = 3
    else:
        print("result=OK mean RTF %.3f" % summary["mean"])
        rc = 0
    if logger:
        logger.write("timing_summary", label=args.label, result=state,
                     misses=misses, **summary)
        logger.close()
    return rc


if __name__ == "__main__":
    raise SystemExit(main())
