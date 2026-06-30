#!/usr/bin/env python3
"""Sample Gazebo stats and summarize real-time factor stability."""

from __future__ import annotations

import argparse
import re
import statistics
import subprocess
import time


RTF_RE = re.compile(r"\breal_time_factor:\s*([-+0-9.eE]+)")
ITER_RE = re.compile(r"\biterations:\s*([0-9]+)")


def run_text(command: list[str], timeout: float) -> str:
    try:
        proc = subprocess.run(
            command,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            timeout=timeout,
            check=False,
        )
        return proc.stdout
    except subprocess.TimeoutExpired as exc:
        if isinstance(exc.stdout, bytes):
            return exc.stdout.decode("utf-8", errors="replace")
        return exc.stdout or ""


def list_topics(timeout: float) -> list[str]:
    text = run_text(["gz", "topic", "-l"], timeout)
    return [line.strip() for line in text.splitlines() if line.strip()]


def choose_stats_topic(requested: str, timeout: float) -> str:
    if requested != "auto":
        return requested
    topics = list_topics(timeout)
    world_stats = sorted(topic for topic in topics if topic.startswith("/world/") and topic.endswith("/stats"))
    if world_stats:
        return world_stats[0]
    return "/stats"


def read_stats_once(topic: str, timeout: float) -> str:
    return run_text(["gz", "topic", "-e", "-t", topic, "-n", "1"], timeout)


def parse_rtf(text: str) -> float | None:
    match = RTF_RE.search(text)
    if not match:
        return None
    try:
        return float(match.group(1))
    except ValueError:
        return None


def parse_iterations(text: str) -> int | None:
    match = ITER_RE.search(text)
    if not match:
        return None
    return int(match.group(1))


def _parse_time_block(text: str, name: str) -> float | None:
    match = re.search(r"\b%s\s*\{([^}]*)\}" % re.escape(name), text, flags=re.S)
    if not match:
        return None
    body = match.group(1)
    sec_match = re.search(r"\bsec:\s*([-0-9]+)", body)
    nsec_match = re.search(r"\bnsec:\s*([0-9]+)", body)
    if not sec_match and not nsec_match:
        return None
    sec = int(sec_match.group(1)) if sec_match else 0
    nsec = int(nsec_match.group(1)) if nsec_match else 0
    return sec + nsec / 1e9


def parse_step_size(text: str) -> float | None:
    return _parse_time_block(text, "step_size")


def parse_stats(text: str) -> dict:
    return {
        "rtf": parse_rtf(text),
        "iterations": parse_iterations(text),
        "sim_time": _parse_time_block(text, "sim_time"),
        "real_time": _parse_time_block(text, "real_time"),
        "step_size": parse_step_size(text),
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--topic", default="auto",
                        help="Gazebo stats topic, or 'auto' to prefer /world/*/stats")
    parser.add_argument("--samples", type=int, default=20)
    parser.add_argument("--interval", type=float, default=0.5)
    parser.add_argument("--timeout", type=float, default=2.0)
    parser.add_argument("--warn-spread", type=float, default=0.15,
                        help="Warn when max-min RTF spread exceeds this value")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.samples <= 0:
        raise SystemExit("--samples must be positive")
    if args.interval < 0 or args.timeout <= 0:
        raise SystemExit("--interval must be non-negative and --timeout must be positive")

    topic = choose_stats_topic(args.topic, args.timeout)
    print("topic=%s" % topic)

    values: list[float] = []
    misses = 0
    for index in range(args.samples):
        stats = parse_stats(read_stats_once(topic, args.timeout))
        rtf = stats["rtf"]
        if rtf is None:
            misses += 1
            print("sample %02d: rtf=missing" % (index + 1))
        else:
            values.append(rtf)
            step = "-" if stats["step_size"] is None else "%.6f" % stats["step_size"]
            print("sample %02d: rtf=%.3f step=%s iter=%s sim=%.3f real=%.3f" % (
                index + 1,
                rtf,
                step,
                stats["iterations"] if stats["iterations"] is not None else "-",
                stats["sim_time"] or 0.0,
                stats["real_time"] or 0.0,
            ))
        if index + 1 < args.samples:
            time.sleep(args.interval)

    if not values:
        print("result=FAIL no real_time_factor samples read from /stats")
        return 1

    spread = max(values) - min(values)
    mean = statistics.fmean(values)
    stdev = statistics.pstdev(values) if len(values) > 1 else 0.0
    print("summary count=%d missing=%d min=%.3f mean=%.3f max=%.3f spread=%.3f stdev=%.3f" % (
        len(values),
        misses,
        min(values),
        mean,
        max(values),
        spread,
        stdev,
    ))
    if spread > args.warn_spread:
        print("result=WARN RTF spread exceeds %.3f; timing jitter may affect SITL stability" % args.warn_spread)
        return 2
    print("result=OK RTF spread within %.3f" % args.warn_spread)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
