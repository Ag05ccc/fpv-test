import sys
from pathlib import Path

TOOLS_DIR = Path(__file__).resolve().parents[1] / "tools"
if str(TOOLS_DIR) not in sys.path:
    sys.path.insert(0, str(TOOLS_DIR))

from sitl_pid_direction_check import main, parse_args, run_checks  # noqa: E402


def result_by_name(results):
    return {result["name"]: result for result in results}


def test_pid_direction_default_cases_pass():
    results = result_by_name(run_checks(parse_args([])))

    assert all(result["ok"] for result in results.values())
    assert results["target_right"]["yaw"] > 1500
    assert results["target_left"]["yaw"] < 1500
    assert results["target_small"]["pitch"] > 1500
    assert results["target_large"]["pitch"] < 1500
    assert results["centered"]["yaw"] == 1500
    assert results["centered"]["pitch"] == 1500
    assert results["target_lost"]["yaw"] == 1500
    assert results["target_lost"]["pitch"] == 1500


def test_pid_direction_check_rejects_deadband_hidden_offset():
    try:
        parse_args(["--target-offset-px", "5", "--deadband", "10"])
    except SystemExit:
        pass
    else:
        raise AssertionError("offset inside deadband should fail")


def test_pid_direction_main_reports_pass(capsys):
    assert main([]) == 0

    output = capsys.readouterr().out
    assert "PID direction check: PASS" in output
    assert "target_right: PASS" in output
    assert "target_small: PASS" in output
