import sys
from pathlib import Path

TOOLS_DIR = Path(__file__).resolve().parents[1] / "tools"
if str(TOOLS_DIR) not in sys.path:
    sys.path.insert(0, str(TOOLS_DIR))

from sitl_mode_status_check import default_cases, validate_status  # noqa: E402


def test_default_mode_cases_cover_arm_msp_override_angle_and_horizon():
    cases = default_cases()

    assert [case.name for case in cases] == [
        "all-low",
        "angle-mid",
        "horizon-high",
        "arm-angle",
        "kenet-tracking-angle",
    ]
    assert cases[1].channels()[:7] == [1500, 1500, 1000, 1500, 1000, 1000, 1500]
    assert cases[2].channels()[:7] == [1500, 1500, 1000, 1500, 1000, 1000, 2000]
    assert "MSP OVERRIDE" in cases[4].expected_active


def test_validate_status_passes_expected_active_and_inactive_modes():
    case = default_cases()[4]
    status = {
        "valid": True,
        "active_modes_valid": True,
        "active_modes": ["ARM", "MSP OVERRIDE", "ANGLE"],
    }

    assert validate_status(case, status) == []


def test_validate_status_reports_missing_and_unexpected_modes():
    case = default_cases()[1]
    status = {
        "valid": True,
        "active_modes_valid": True,
        "active_modes": ["ARM", "HORIZON"],
    }

    assert validate_status(case, status) == [
        "ANGLE expected active",
        "ARM expected inactive",
        "HORIZON expected inactive",
    ]


def test_validate_status_requires_box_metadata():
    case = default_cases()[0]
    status = {
        "valid": False,
        "active_modes_valid": False,
        "active_modes": [],
    }

    assert validate_status(case, status) == [
        "MSP_STATUS_EX invalid",
        "BOX metadata unavailable",
    ]
