import sys
from pathlib import Path

TOOLS_DIR = Path(__file__).resolve().parents[1] / "tools"
if str(TOOLS_DIR) not in sys.path:
    sys.path.insert(0, str(TOOLS_DIR))

from sitl_physical_rc_preflight import (  # noqa: E402
    AUTOPILOT_MODE_CHANNEL,
    HIGH,
    KENET_STATE_CH,
    LOW,
    MID,
    SWITCH_EXPECTATIONS,
    evaluate_switch_observations,
    pwm_bucket,
    validate_channel_map_contract,
)


def test_pwm_bucket_boundaries():
    assert pwm_bucket(1000) == LOW
    assert pwm_bucket(1250) == LOW
    assert pwm_bucket(1500) == MID
    assert pwm_bucket(1750) == HIGH
    assert pwm_bucket(2000) == HIGH


def test_channel_map_contract_matches_bench_switch_roles():
    assert validate_channel_map_contract() == []
    assert [item["role"] for item in SWITCH_EXPECTATIONS] == [
        "ARM",
        "Kenet state",
        "Autopilot mode",
    ]


def test_evaluate_switch_observations_passes_full_physical_mapping():
    observed = {
        4: {LOW, HIGH},
        KENET_STATE_CH: {LOW, MID, HIGH},
        AUTOPILOT_MODE_CHANNEL: {LOW, MID, HIGH},
    }

    results = evaluate_switch_observations(observed)

    assert all(result["ok"] for result in results)


def test_evaluate_switch_observations_allows_forced_angle_mode():
    observed = {
        4: {LOW, HIGH},
        KENET_STATE_CH: {LOW, MID, HIGH},
        AUTOPILOT_MODE_CHANNEL: set(),
    }

    results = evaluate_switch_observations(observed, force_mode_pwm=1500)
    mode_result = [result for result in results if result["channel"] == AUTOPILOT_MODE_CHANNEL][0]

    assert all(result["ok"] for result in results)
    assert mode_result["forced"] is True
    assert mode_result["expected"] == [MID]
    assert mode_result["seen"] == [MID]


def test_evaluate_switch_observations_reports_missing_positions():
    observed = {
        4: {LOW},
        KENET_STATE_CH: {LOW, HIGH},
        AUTOPILOT_MODE_CHANNEL: {MID},
    }

    results = evaluate_switch_observations(observed)

    assert results[0]["ok"] is False
    assert results[0]["missing"] == [HIGH]
    assert results[1]["missing"] == [MID]
    assert results[2]["missing"] == [HIGH, LOW]
