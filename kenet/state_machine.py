"""Shared Kenet state-machine constants and AUX decoding helpers."""

IDLE = 0
AI_ARMED = 1
TRACKING = 2

STATE_NAMES = {
    IDLE: "IDLE",
    AI_ARMED: "AI-ARMED",
    TRACKING: "TRACKING",
}

DEFAULT_KENET_AUX_CH = 5
DEFAULT_AUX_ARM_THRESHOLD = 1300
DEFAULT_AUX_TRACK_THRESHOLD = 1700


def state_from_aux(
    value,
    arm_threshold=DEFAULT_AUX_ARM_THRESHOLD,
    track_threshold=DEFAULT_AUX_TRACK_THRESHOLD,
    tracking_inhibited=False,
):
    """Map a 3-position AUX RC value to a Kenet state.

    Thresholds intentionally use strict comparisons to match the production
    pipeline's original behavior: 1300 stays IDLE, 1700 stays AI-ARMED.
    tracking_inhibited forces HIGH to remain AI-ARMED after long target loss
    until the pilot cycles the switch below the tracking threshold.
    """
    if value > track_threshold and not tracking_inhibited:
        return TRACKING
    if value > arm_threshold:
        return AI_ARMED
    return IDLE
