import pytest

from kenet.pipeline import PipelineConfig
from kenet.state_machine import AI_ARMED, IDLE, TRACKING, state_from_aux


def test_default_kenet_aux_channel_is_ch6_aux2():
    assert PipelineConfig().aux_ch == 5


@pytest.mark.parametrize(
    ("value", "expected"),
    [
        (1299, IDLE),
        (1300, IDLE),
        (1301, AI_ARMED),
        (1699, AI_ARMED),
        (1700, AI_ARMED),
        (1701, TRACKING),
    ],
)
def test_state_from_aux_thresholds_are_strict(value, expected):
    assert state_from_aux(value) == expected


def test_state_from_aux_can_inhibit_tracking_reentry():
    assert state_from_aux(2000, tracking_inhibited=True) == AI_ARMED
    assert state_from_aux(1500, tracking_inhibited=True) == AI_ARMED
    assert state_from_aux(1000, tracking_inhibited=True) == IDLE
