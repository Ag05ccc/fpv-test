#!/usr/bin/env python3
"""MSP Override write-path loopback gate (roadmap-v2 A5, metric 5).

The reader audit flagged a possible channel-order defect: the pipeline builds
its MSP_SET_RAW_RC override frame in pilot/AETR order (roll, pitch, throttle,
yaw, aux...) and sends it directly, while MSP_RC readback is remapped on the
read side. This gate settles the question empirically against a real Betaflight
SITL instead of by reasoning.

It reproduces the exact production merge (pipeline._build_override_channels):
start from a pilot RC frame, overwrite ONLY pitch and yaw with controller
values, send via the same MSPConnection.send_rc the pipeline uses, then read
MSP_RC back and assert:

- the controller's yaw value lands in Betaflight's internal YAW channel,
- the controller's pitch value lands in Betaflight's internal PITCH channel,
- the pilot's throttle and roll are preserved in their internal channels
  (not overwritten and not swapped with yaw).

Distinct values per channel make any cross-wiring (e.g. yaw leaking into
throttle) impossible to miss. Exit 0 PASS, 1 FAIL.

Usage (SITL already running on tcp://127.0.0.1:5761):

    fpv_env/bin/python tools/sitl_msp_override_loopback.py --msp-tcp 127.0.0.1:5761
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from kenet.msp import MSPConnection  # noqa: E402
from kenet.rc_channels import (  # noqa: E402
    PILOT_TO_MSP_INDEX,
    PITCH_CH,
    ROLL_CH,
    THROTTLE_CH,
    YAW_CH,
    pilot_to_msp_rc_channels,
)


def build_override_like_pipeline(pilot_frame, controller_pitch, controller_yaw):
    """Mirror pipeline._build_override_channels: overwrite only pitch & yaw."""
    frame = list(pilot_frame)
    frame[PITCH_CH] = controller_pitch
    frame[YAW_CH] = controller_yaw
    return frame


def evaluate_loopback(sent_pilot_frame, msp_rc_readback):
    """Assert each pilot/controller value reached the correct internal channel.

    Betaflight returns MSP_RC in internal order; pilot_to_msp_rc_channels maps
    our pilot/AETR frame into that internal order, so the readback must match
    it channel-for-channel on the four primary axes.
    """
    failures = []
    if not msp_rc_readback or len(msp_rc_readback) < 4:
        return ["MSP_RC readback missing or too short: %s" % (msp_rc_readback,)]
    expected_internal = pilot_to_msp_rc_channels(sent_pilot_frame)
    labels = {ROLL_CH: "roll", PITCH_CH: "pitch", THROTTLE_CH: "throttle",
              YAW_CH: "yaw"}
    for pilot_index, label in labels.items():
        internal_index = PILOT_TO_MSP_INDEX[pilot_index]
        want = expected_internal[internal_index]
        got = msp_rc_readback[internal_index]
        if got != want:
            failures.append(
                "%s: pilot value %d expected at internal channel %d, got %d"
                % (label, sent_pilot_frame[pilot_index], internal_index, got))
    return failures


def run_loopback(conn, *, settle=0.3):
    # Distinct value per channel so any cross-wiring is unmistakable.
    # pilot/AETR order: roll, pitch, throttle, yaw, aux1, aux2, aux3, aux4
    pilot_frame = [1450, 1400, 1250, 1550, 1500, 1500, 1500, 1500]
    controller_pitch = 1620   # what the tracking PID would command
    controller_yaw = 1380
    override = build_override_like_pipeline(pilot_frame, controller_pitch,
                                            controller_yaw)
    conn.send_rc(override)
    time.sleep(settle)
    conn.send_rc(override)  # ensure Betaflight latches the frame
    time.sleep(settle)
    readback = conn.get_rc_channels()
    failures = evaluate_loopback(override, readback)
    return {
        "ok": not failures,
        "failures": failures,
        "sent_pilot_order": override,
        "expected_internal": pilot_to_msp_rc_channels(override),
        "readback_internal": list(readback) if readback else None,
        "controller_pitch": controller_pitch,
        "controller_yaw": controller_yaw,
    }


def resolve_port(args):
    if args.msp_tcp:
        return args.msp_tcp if args.msp_tcp.startswith("tcp://") else "tcp://%s" % args.msp_tcp
    return args.port


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--msp-tcp", default="127.0.0.1:5761",
                        help="Betaflight SITL MSP endpoint (default 127.0.0.1:5761)")
    parser.add_argument("--port", default=None, help="Serial port alternative")
    parser.add_argument("--timeout", type=float, default=1.0)
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    conn = MSPConnection(resolve_port(args), timeout=args.timeout)
    conn.connect()
    try:
        verdict = run_loopback(conn)
    finally:
        conn.disconnect()
    print("MSP override loopback: %s" % ("PASS" if verdict["ok"] else "FAIL"))
    print("  sent (pilot/AETR order):   %s" % verdict["sent_pilot_order"])
    print("  expected (internal order): %s" % verdict["expected_internal"])
    print("  readback (internal order): %s" % verdict["readback_internal"])
    print("  controller pitch=%d yaw=%d must not cross into throttle/roll" % (
        verdict["controller_pitch"], verdict["controller_yaw"]))
    for failure in verdict["failures"]:
        print("  FAIL: %s" % failure)
    return 0 if verdict["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
