#!/usr/bin/env python3
"""Summarize SITL readiness evidence and remaining external gates."""

from __future__ import annotations

import argparse
import glob
import json
import sys
from pathlib import Path
from typing import Any


SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from analyze_sitl_log import summarize  # noqa: E402
from sitl_video_tracking_check import summarize_mixer_log  # noqa: E402
from sitl_run_quality_check import evaluate_run as _evaluate_run  # noqa: E402


class _QualityArgs:
    """Minimal args object for delegating to sitl_run_quality_check."""
    def __init__(self, expected_step=None):
        self.expected_step = expected_step
        self.rtf_min = 0.99
        self.rtf_max = 1.01
        self.max_gap_ms = 30.0
        self.first_sample_max_attitude = 5.0
        self.max_abs_attitude = 35.0
        self.max_motor_spread = 400.0
        self.max_yaw_rate_dps = 90.0
        self.disarm_motor_max = 1100.0
        self.min_climb_rate = 0.1
        self.expect_yaw_sign = None
        self.min_yaw_delta_deg = 0.0
        self.max_yaw_delta_deg = None
        self.require_disarm = False


def quality_gate(diagnostics: Path, motor_udp: Path | None = None,
                 expected_step: float | None = None) -> dict[str, Any]:
    """Delegate a run's validity/health/climb/signed verdict to the shared
    run_quality_check logic (A0)."""
    if not diagnostics.exists():
        return {"ok": False, "reason": "missing", "path": str(diagnostics)}
    run = _evaluate_run(diagnostics, motor_udp, _QualityArgs(expected_step))
    return {
        "ok": run["verdict"] == "PASS",
        "verdict": run["verdict"],
        "path": str(diagnostics),
        "validity": run["validity"]["valid"],
        "armed_angle_samples": run["validity"]["armed_angle_samples"],
        "climb_rate": run["climb"].get("climb_rate"),
        "rtf_min": run["health"].get("rtf_min"),
        "rtf_max": run["health"].get("rtf_max"),
    }


DEFAULT_EVIDENCE = {
    "real_video_target_loss": {
        "diagnostics": "logs/sitl/20260630-video-target-loss-p23-delay18-final-diagnostics.jsonl",
        "mixer": "logs/sitl/20260630-video-target-loss-p23-delay18-final-mixer.jsonl",
        "motor_udp": "logs/sitl/20260630-video-target-loss-p23-delay18-final-motor-udp.jsonl",
    },
    "safe_yaw_virtual_takeoff": {
        "diagnostics": "logs/sitl/20260630-064350-takeoff-diagnostics.jsonl",
        "motor_udp": "logs/sitl/20260630-064350-takeoff-motor-udp.jsonl",
        "virtual_rc": "logs/sitl/20260630-064350-takeoff-virtual-rc.jsonl",
    },
    "quickstart": {
        "doc": "docs/sitl-quickstart.md",
    },
}


def resolve(root: Path, value: str) -> Path:
    path = Path(value)
    return path if path.is_absolute() else root / path


def diagnostic_pass(path: Path, *, max_attitude: float = 35.0, max_spread: float = 400.0) -> dict[str, Any]:
    if not path.exists():
        return {"ok": False, "reason": "missing", "path": str(path)}
    stats = summarize([path])
    dash = stats["dashboard"]
    if not dash["attitude_samples"] or not dash["motor_samples"]:
        return {"ok": False, "reason": "missing attitude/motor samples", "path": str(path)}
    roll = abs(dash["max_roll"]["value"]) if dash["max_roll"] else 0.0
    pitch = abs(dash["max_pitch"]["value"]) if dash["max_pitch"] else 0.0
    spread = dash["max_motor_spread"]["value"] if dash["max_motor_spread"] else 0.0
    ok = max(roll, pitch) <= max_attitude and spread <= max_spread
    return {
        "ok": ok,
        "reason": "ok" if ok else "threshold exceeded",
        "path": str(path),
        "samples": dash["samples"],
        "armed_samples": dash["armed"].get("True", 0),
        "max_roll": roll,
        "max_pitch": pitch,
        "max_motor_spread": spread,
    }


def mixer_target_loss_pass(path: Path) -> dict[str, Any]:
    if not path.exists():
        return {"ok": False, "reason": "missing", "path": str(path)}
    summary = summarize_mixer_log(path)
    ok = (
        summary["target_found_samples"] > 0
        and summary["kenet_source_samples"] > 0
        and summary["target_lost_source_samples"] > 0
        and summary["ai_armed_samples"] > 0
        and summary["max_abs_delta"] == 0
        and not summary["tracker_errors"]
    )
    return {
        "ok": ok,
        "reason": "ok" if ok else "target-loss criteria not met",
        "path": str(path),
        "samples": summary["samples"],
        "target_found_samples": summary["target_found_samples"],
        "kenet_source_samples": summary["kenet_source_samples"],
        "target_lost_source_samples": summary["target_lost_source_samples"],
        "ai_armed_samples": summary["ai_armed_samples"],
        "max_abs_delta": summary["max_abs_delta"],
        "tracker_errors": summary["tracker_errors"],
    }


def file_exists(path: Path) -> dict[str, Any]:
    return {"ok": path.exists(), "reason": "ok" if path.exists() else "missing", "path": str(path)}


def build_report(root: Path, joystick_glob: str = "/dev/input/js*") -> dict[str, Any]:
    evidence = DEFAULT_EVIDENCE
    safe_diag = diagnostic_pass(resolve(root, evidence["safe_yaw_virtual_takeoff"]["diagnostics"]))
    target_diag = diagnostic_pass(resolve(root, evidence["real_video_target_loss"]["diagnostics"]))
    target_mixer = mixer_target_loss_pass(resolve(root, evidence["real_video_target_loss"]["mixer"]))
    quickstart = file_exists(resolve(root, evidence["quickstart"]["doc"]))
    joystick_devices = sorted(glob.glob(joystick_glob))

    no_hardware_items = {
        "quickstart_doc": quickstart,
        "safe_yaw_virtual_takeoff": safe_diag,
        "real_video_target_loss_diagnostics": target_diag,
        "real_video_target_loss_mixer": target_mixer,
    }
    no_hardware_ok = all(item["ok"] for item in no_hardware_items.values())

    external_items = {
        "physical_rc_device": {
            "ok": bool(joystick_devices),
            "reason": "ok" if joystick_devices else "waiting for /dev/input/js*",
            "devices": joystick_devices,
            "next_step": (
                "run tools/sitl_physical_rc_preflight.py with stick/switch movement "
                "before enabling a live external RC sender"
            ),
        },
        "other_machine_env_report": {
            "ok": False,
            "reason": "requires running tools/check_sitl_env.sh on another machine",
        },
    }

    return {
        "root": str(root),
        "no_hardware_ok": no_hardware_ok,
        "no_hardware_items": no_hardware_items,
        "external_items": external_items,
    }


def print_report(report: dict[str, Any]) -> None:
    print("SITL readiness report")
    print("root=%s" % report["root"])
    print("no_hardware=%s" % ("PASS" if report["no_hardware_ok"] else "INCOMPLETE"))
    print("\nNo-hardware evidence")
    for name, item in report["no_hardware_items"].items():
        print("  %s: %s (%s)" % (name, "PASS" if item["ok"] else "FAIL", item["reason"]))
        if item.get("path"):
            print("    path=%s" % item["path"])
    print("\nExternal gates")
    for name, item in report["external_items"].items():
        print("  %s: %s (%s)" % (name, "READY" if item["ok"] else "WAITING", item["reason"]))
        if item.get("devices"):
            print("    devices=%s" % ",".join(item["devices"]))
        if item.get("next_step"):
            print("    next=%s" % item["next_step"])


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=REPO_ROOT)
    parser.add_argument("--joystick-glob", default="/dev/input/js*")
    parser.add_argument("--json", action="store_true")
    parser.add_argument("--quality-check", nargs="+", type=Path, default=None,
                        metavar="DIAGNOSTICS",
                        help="Delegate validity/health/climb verdicts for these "
                             "diagnostics JSONL runs to sitl_run_quality_check")
    parser.add_argument("--quality-motor-udp", nargs="*", type=Path, default=[],
                        help="Motor UDP JSONL files matching --quality-check order")
    parser.add_argument("--expected-step", type=float, default=None)
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    report = build_report(args.root.resolve(), joystick_glob=args.joystick_glob)
    quality_ok = True
    if args.quality_check:
        report["quality_check"] = []
        for i, diag in enumerate(args.quality_check):
            motor = args.quality_motor_udp[i] if i < len(args.quality_motor_udp) else None
            verdict = quality_gate(diag, motor, args.expected_step)
            report["quality_check"].append(verdict)
            quality_ok = quality_ok and verdict["ok"]
    if args.json:
        print(json.dumps(report, indent=2, sort_keys=True))
    else:
        print_report(report)
        for verdict in report.get("quality_check", []):
            print("  quality: %s %s (valid=%s, armed_angle=%s, climb=%s m/s)" % (
                verdict.get("verdict", "?"), verdict["path"],
                verdict.get("validity"), verdict.get("armed_angle_samples"),
                "%.3f" % verdict["climb_rate"] if verdict.get("climb_rate") is not None else "n/a"))
    return 0 if (report["no_hardware_ok"] and quality_ok) else 1


if __name__ == "__main__":
    raise SystemExit(main())
