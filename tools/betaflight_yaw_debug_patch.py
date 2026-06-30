#!/usr/bin/env python3
"""Apply or remove a narrow Betaflight SITL yaw PID debug instrumentation patch."""

from __future__ import annotations

import argparse
import os
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
MARKER_BEGIN = "KENET_SITL_YAW_DEBUG_BEGIN"
MARKER_END = "KENET_SITL_YAW_DEBUG_END"

TARGET_SNIPPET = """\
        {
            pidData[axis].Sum = pidSum;
        }
    }

#ifdef USE_WING
"""

INSTRUMENTED_SNIPPET = """\
        {
            pidData[axis].Sum = pidSum;
        }

        // KENET_SITL_YAW_DEBUG_BEGIN
        // Temporary SITL instrumentation: DEBUG_AC_ERROR exposes yaw internals.
        // debug[0]=setpoint, [1]=gyroRate, [2]=errorRate, [3]=P,
        // debug[4]=I, [5]=F, [6]=S, [7]=Sum.
        if (axis == FD_YAW) {
            DEBUG_SET(DEBUG_AC_ERROR, 0, lrintf(constrainf(currentPidSetpoint, -32768.0f, 32767.0f)));
            DEBUG_SET(DEBUG_AC_ERROR, 1, lrintf(constrainf(gyroRate, -32768.0f, 32767.0f)));
            DEBUG_SET(DEBUG_AC_ERROR, 2, lrintf(constrainf(errorRate, -32768.0f, 32767.0f)));
            DEBUG_SET(DEBUG_AC_ERROR, 3, lrintf(constrainf(pidData[axis].P, -32768.0f, 32767.0f)));
            DEBUG_SET(DEBUG_AC_ERROR, 4, lrintf(constrainf(pidData[axis].I, -32768.0f, 32767.0f)));
            DEBUG_SET(DEBUG_AC_ERROR, 5, lrintf(constrainf(pidData[axis].F, -32768.0f, 32767.0f)));
            DEBUG_SET(DEBUG_AC_ERROR, 6, lrintf(constrainf(pidData[axis].S, -32768.0f, 32767.0f)));
            DEBUG_SET(DEBUG_AC_ERROR, 7, lrintf(constrainf(pidData[axis].Sum, -32768.0f, 32767.0f)));
        }
        // KENET_SITL_YAW_DEBUG_END
    }

#ifdef USE_WING
"""


def betaflight_root(value: str | None = None) -> Path:
    if value:
        return Path(value).expanduser()
    env_value = os.environ.get("BETAFLIGHT_ROOT")
    if env_value:
        return Path(env_value).expanduser()
    return (REPO_ROOT / "../betaflight").resolve()


def target_path(root: Path) -> Path:
    return root / "src/main/flight/pid.c"


def patch_state(text: str) -> str:
    if MARKER_BEGIN in text and MARKER_END in text:
        return "applied"
    if TARGET_SNIPPET in text:
        return "clean"
    return "unknown"


def apply_patch_text(text: str) -> tuple[str, bool]:
    state = patch_state(text)
    if state == "applied":
        return text, False
    if state != "clean":
        raise RuntimeError("target snippet not found and instrumentation marker absent")
    return text.replace(TARGET_SNIPPET, INSTRUMENTED_SNIPPET, 1), True


def revert_patch_text(text: str) -> tuple[str, bool]:
    state = patch_state(text)
    if state == "clean":
        return text, False
    if state != "applied":
        raise RuntimeError("instrumentation marker state is inconsistent")
    return text.replace(INSTRUMENTED_SNIPPET, TARGET_SNIPPET, 1), True


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--betaflight-root", default=None)
    group = parser.add_mutually_exclusive_group()
    group.add_argument("--apply", action="store_true", help="Apply the yaw debug instrumentation")
    group.add_argument("--revert", action="store_true", help="Remove the yaw debug instrumentation")
    group.add_argument("--check", action="store_true", help="Print whether the patch is applied")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    root = betaflight_root(args.betaflight_root)
    path = target_path(root)
    text = path.read_text(encoding="utf-8")

    if args.revert:
        new_text, changed = revert_patch_text(text)
        action = "reverted"
    elif args.apply:
        new_text, changed = apply_patch_text(text)
        action = "applied"
    else:
        print("state=%s path=%s" % (patch_state(text), path))
        return 0 if patch_state(text) in ("clean", "applied") else 1

    if changed:
        path.write_text(new_text, encoding="utf-8")
    print("%s=%s changed=%s path=%s" % (action, patch_state(new_text), changed, path))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
