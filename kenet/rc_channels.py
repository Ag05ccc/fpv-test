"""Shared RC channel indices and Betaflight MSP_RC mapping helpers."""

ROLL_CH = 0
PITCH_CH = 1
THROTTLE_CH = 2
YAW_CH = 3
ARM_CH = 4
KENET_STATE_CH = 5
AUTOPILOT_MODE_CHANNEL = 6

PILOT_TO_MSP_INDEX = {
    ROLL_CH: 0,
    PITCH_CH: 1,
    THROTTLE_CH: 3,
    YAW_CH: 2,
    ARM_CH: 4,
    KENET_STATE_CH: 5,
    AUTOPILOT_MODE_CHANNEL: 6,
}


def pilot_to_msp_rc_channels(pilot_channels):
    """Return expected MSP_RC order for pilot/AETR channel values."""
    msp_channels = list(pilot_channels)
    for pilot_index, msp_index in PILOT_TO_MSP_INDEX.items():
        if pilot_index < len(pilot_channels) and msp_index < len(msp_channels):
            msp_channels[msp_index] = pilot_channels[pilot_index]
    return msp_channels


def msp_rc_to_pilot_channels(msp_channels):
    """Return pilot/AETR order from a Betaflight MSP_RC response."""
    pilot_channels = list(msp_channels)
    for pilot_index, msp_index in PILOT_TO_MSP_INDEX.items():
        if pilot_index < len(pilot_channels) and msp_index < len(msp_channels):
            pilot_channels[pilot_index] = msp_channels[msp_index]
    return pilot_channels
