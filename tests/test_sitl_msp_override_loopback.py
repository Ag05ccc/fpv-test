import sys
from pathlib import Path

TOOLS_DIR = Path(__file__).resolve().parents[1] / "tools"
if str(TOOLS_DIR) not in sys.path:
    sys.path.insert(0, str(TOOLS_DIR))

from sitl_msp_override_loopback import (  # noqa: E402
    build_override_like_pipeline,
    evaluate_loopback,
)
from kenet.rc_channels import pilot_to_msp_rc_channels


def test_override_merge_only_touches_pitch_and_yaw():
    pilot = [1450, 1400, 1250, 1550, 1500, 1500, 1500, 1500]
    override = build_override_like_pipeline(pilot, controller_pitch=1620,
                                            controller_yaw=1380)
    assert override[0] == 1450   # roll preserved
    assert override[1] == 1620   # pitch overwritten
    assert override[2] == 1250   # throttle preserved
    assert override[3] == 1380   # yaw overwritten


def test_evaluate_loopback_pass_on_correct_mapping():
    sent = [1450, 1620, 1250, 1380, 1500, 1500, 1500, 1500]
    # Betaflight returns internal order = pilot_to_msp_rc_channels(sent)
    readback = pilot_to_msp_rc_channels(sent) + [1500] * 8
    assert evaluate_loopback(sent, readback) == []


def test_evaluate_loopback_catches_yaw_throttle_swap():
    sent = [1450, 1620, 1250, 1380, 1500, 1500, 1500, 1500]
    correct = pilot_to_msp_rc_channels(sent)
    # simulate a defect: yaw and throttle crossed in the internal readback
    broken = list(correct)
    yaw_internal, thr_internal = 2, 3
    broken[yaw_internal], broken[thr_internal] = broken[thr_internal], broken[yaw_internal]
    failures = evaluate_loopback(sent, broken)
    assert failures
    assert any("yaw" in f for f in failures)
    assert any("throttle" in f for f in failures)


def test_evaluate_loopback_missing_readback():
    sent = [1450, 1620, 1250, 1380]
    assert evaluate_loopback(sent, None)
    assert evaluate_loopback(sent, [1500, 1500])
