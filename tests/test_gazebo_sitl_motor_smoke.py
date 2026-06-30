import sys
from argparse import Namespace
from pathlib import Path


TOOLS_DIR = Path(__file__).resolve().parents[1] / "tools"
if str(TOOLS_DIR) not in sys.path:
    sys.path.insert(0, str(TOOLS_DIR))

from gazebo_sitl_motor_smoke import SmokeError, betaflight_work_dir, evaluate_smoke_result  # noqa: E402


def args(**overrides):
    values = {
        "betaflight_cwd": "temp",
    }
    values.update(overrides)
    return Namespace(**values)


def test_betaflight_work_dir_defaults_to_temp_directory(tmp_path):
    fpv_root = tmp_path / "repo"
    temp_dir = tmp_path / "run"

    work_dir = betaflight_work_dir(args(), fpv_root, temp_dir)

    assert work_dir == temp_dir / "betaflight-cwd"
    assert work_dir.is_dir()


def test_betaflight_work_dir_can_use_repo_directory(tmp_path):
    fpv_root = tmp_path / "repo"
    temp_dir = tmp_path / "run"

    work_dir = betaflight_work_dir(args(betaflight_cwd="repo"), fpv_root, temp_dir)

    assert work_dir == fpv_root
    assert not (temp_dir / "betaflight-cwd").exists()


def test_smoke_result_passes_on_motor_output_even_when_rotor_motion_missing():
    warning = evaluate_smoke_result((1200, 1210, 1190, 1220), ["rotor_0"], require_rotor_motion=False)

    assert warning == "not all rotor joints moved: rotor_0"


def test_smoke_result_can_require_rotor_motion():
    try:
        evaluate_smoke_result((1200, 1210, 1190, 1220), ["rotor_0"], require_rotor_motion=True)
    except SmokeError as exc:
        assert "not all rotor joints moved" in str(exc)
    else:
        raise AssertionError("missing rotor motion should fail in strict mode")


def test_smoke_result_fails_when_motor_output_stays_idle():
    try:
        evaluate_smoke_result((1000, 1000, 1000, 1000), ["rotor_0", "rotor_1", "rotor_2", "rotor_3"], require_rotor_motion=False)
    except SmokeError as exc:
        assert "motors did not rise above idle" in str(exc)
    else:
        raise AssertionError("idle motor output should fail")
