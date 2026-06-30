"""Shared RC channel metadata for SITL tools.

Betaflight's UDP rc_packet is AETR ordered for the first four channels:
roll, pitch, throttle, yaw. Some MSP responses report RPYT, so tools that
compare pilot RC to MSP RC use PILOT_TO_MSP_INDEX for the first four channels.
"""

from __future__ import annotations

import sys
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from kenet.rc_channels import (  # noqa: E402
    ARM_CH,
    AUTOPILOT_MODE_CHANNEL,
    KENET_STATE_CH,
    PILOT_TO_MSP_INDEX,
    PITCH_CH,
    ROLL_CH,
    THROTTLE_CH,
    YAW_CH,
    msp_rc_to_pilot_channels,
    pilot_to_msp_rc_channels,
)

RC_CHANNEL_LABELS = [
    "Roll",
    "Pitch",
    "Throttle",
    "Yaw",
    "AUX1 ARM",
    "AUX2 Kenet",
    "AUX3 Mode",
    "AUX4",
    "CH9",
    "CH10",
    "CH11",
    "CH12",
    "CH13",
    "CH14",
    "CH15",
    "CH16",
]

def channel_label(index: int) -> str:
    return RC_CHANNEL_LABELS[index] if index < len(RC_CHANNEL_LABELS) else "CH%d" % (index + 1)


def channel_neutral(index: int) -> int:
    return 1000 if index == THROTTLE_CH else 1500


def first_n_channel_labels(count: int) -> list[str]:
    return [channel_label(index) for index in range(count)]
