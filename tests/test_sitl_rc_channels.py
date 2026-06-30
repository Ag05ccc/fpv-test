import sys
from pathlib import Path

TOOLS_DIR = Path(__file__).resolve().parents[1] / "tools"
if str(TOOLS_DIR) not in sys.path:
    sys.path.insert(0, str(TOOLS_DIR))

from sitl_rc_channels import (  # noqa: E402
    AUTOPILOT_MODE_CHANNEL,
    PILOT_TO_MSP_INDEX,
    THROTTLE_CH,
    YAW_CH,
    channel_label,
    channel_neutral,
    first_n_channel_labels,
    msp_rc_to_pilot_channels,
    pilot_to_msp_rc_channels,
)


def test_shared_channel_labels_cover_sitl_first_eight_channels():
    assert first_n_channel_labels(8) == [
        "Roll",
        "Pitch",
        "Throttle",
        "Yaw",
        "AUX1 ARM",
        "AUX2 Kenet",
        "AUX3 Mode",
        "AUX4",
    ]
    assert channel_label(15) == "CH16"
    assert channel_label(16) == "CH17"


def test_shared_channel_neutrals_and_msp_index_mapping():
    assert channel_neutral(THROTTLE_CH) == 1000
    assert channel_neutral(YAW_CH) == 1500
    assert PILOT_TO_MSP_INDEX[THROTTLE_CH] == 3
    assert PILOT_TO_MSP_INDEX[YAW_CH] == 2
    assert AUTOPILOT_MODE_CHANNEL == 6


def test_shared_msp_rc_mapping_converts_between_readback_and_pilot_order():
    pilot = [1500, 1600, 1120, 1400, 2000, 2000, 1500, 1500]
    msp_rc = [1500, 1600, 1400, 1120, 2000, 2000, 1500, 1500]

    assert pilot_to_msp_rc_channels(pilot) == msp_rc
    assert msp_rc_to_pilot_channels(msp_rc) == pilot
