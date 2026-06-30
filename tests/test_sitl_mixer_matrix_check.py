import sys
from pathlib import Path

TOOLS_DIR = Path(__file__).resolve().parents[1] / "tools"
if str(TOOLS_DIR) not in sys.path:
    sys.path.insert(0, str(TOOLS_DIR))

from sitl_mixer_matrix_check import matrix_results  # noqa: E402


def test_mixer_matrix_all_cases_pass():
    rows = matrix_results()

    assert [row["name"] for row in rows] == [
        "P6.1 AUX2 LOW",
        "P6.2 AUX2 MID",
        "P6.3 AUX2 HIGH target lost",
        "P6.4 AUX2 HIGH target found",
        "P6.5 ARM low/high channel contract",
        "P6.6 throttle remains pilot in TRACKING",
        "P6.7 roll remains pilot in TRACKING",
        "P6.8 target loss releases pitch/yaw",
    ]
    assert all(row["ok"] for row in rows)
