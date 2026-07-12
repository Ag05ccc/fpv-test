import json
import sys
from pathlib import Path

import pytest


TOOLS_DIR = Path(__file__).resolve().parents[1] / "tools"
if str(TOOLS_DIR) not in sys.path:
    sys.path.insert(0, str(TOOLS_DIR))

from sitl_video_tracking_check import (  # noqa: E402
    build_checker_command,
    build_mixer_command,
    mixer_duration,
    parse_args,
    summarize_mixer_log,
    validate_mixer_summary,
)


def write_jsonl(path, records):
    with path.open("w", encoding="utf-8") as handle:
        for record in records:
            handle.write(json.dumps(record) + "\n")


def test_checker_command_uses_external_safe_yaw_and_motor_capture(tmp_path):
    args = parse_args([
        "--yaw-pid", "23,0,0",
        "--pitch-pid", "23,0,0",
        "--pitch-rc-rate", "5",
        "--pitch-rate", "30",
        "--pitch-rate-limit", "120",
    ])
    diagnostics = tmp_path / "diag.jsonl"
    motor = tmp_path / "motor.jsonl"

    command = build_checker_command(args, diagnostics, motor)

    assert "sitl_virtual_takeoff_check.py" in command[1]
    assert command[command.index("--rc-driver") + 1] == "external"
    assert "--safe-yaw-authority" in command
    assert command[command.index("--yaw-pid") + 1] == "23,0,0"
    assert command[command.index("--pitch-pid") + 1] == "23,0,0"
    assert command[command.index("--pitch-rc-rate") + 1] == "5"
    assert command[command.index("--pitch-rate") + 1] == "30"
    assert command[command.index("--pitch-rate-limit") + 1] == "120"
    assert "--capture-motor-udp" in command
    assert command[command.index("--motor-udp-log-file") + 1] == str(motor)
    assert "--max-step-size" not in command


def test_checker_command_passes_yaw_rate_overrides_and_config_file(tmp_path):
    args = parse_args([
        "--yaw-rc-rate", "5",
        "--yaw-rate", "30",
        "--yaw-rate-limit", "60",
        "--betaflight-config-file", "configs/sim-tune.txt",
    ])
    diagnostics = tmp_path / "diag.jsonl"

    command = build_checker_command(args, diagnostics, None)

    assert command[command.index("--yaw-rc-rate") + 1] == "5"
    assert command[command.index("--yaw-rate") + 1] == "30"
    assert command[command.index("--yaw-rate-limit") + 1] == "60"
    assert command[command.index("--betaflight-config-file") + 1] == "configs/sim-tune.txt"


def test_checker_command_omits_yaw_rate_overrides_by_default(tmp_path):
    args = parse_args([])
    diagnostics = tmp_path / "diag.jsonl"

    command = build_checker_command(args, diagnostics, None)

    assert "--yaw-rc-rate" not in command
    assert "--yaw-rate" not in command
    assert "--yaw-rate-limit" not in command
    assert "--betaflight-config-file" not in command


def test_parse_args_rejects_out_of_range_yaw_rate_values():
    with pytest.raises(SystemExit):
        parse_args(["--yaw-rc-rate", "256"])
    with pytest.raises(SystemExit):
        parse_args(["--yaw-rate-limit", "65536"])


def test_checker_command_passes_max_step_size(tmp_path):
    args = parse_args(["--max-step-size", "0.0025"])
    diagnostics = tmp_path / "diag.jsonl"

    command = build_checker_command(args, diagnostics, None)

    assert command[command.index("--max-step-size") + 1] == "0.0025"
    assert "--sync-betaflight-looptime" not in command


def test_checker_command_passes_sync_betaflight_looptime(tmp_path):
    args = parse_args(["--sync-betaflight-looptime"])
    diagnostics = tmp_path / "diag.jsonl"

    command = build_checker_command(args, diagnostics, None)

    assert "--sync-betaflight-looptime" in command


def test_mixer_command_uses_real_video_not_synthetic(tmp_path):
    args = parse_args([
        "--camera", "test-2.mp4",
        "--yaw-limit", "12",
        "--forward-limit", "10",
        "--kenet-delay-seconds", "18",
        "--target-loss-after-seconds", "24",
    ])
    log = tmp_path / "mixer.jsonl"

    command = build_mixer_command(args, log)

    assert "kenet_sitl_mixer.py" in command[1]
    assert command[command.index("--pilot-source") + 1] == "virtual"
    assert command[command.index("--camera") + 1] == "test-2.mp4"
    assert "--synthetic-target" not in command
    assert command[command.index("--virtual-kenet-delay-seconds") + 1] == "18.0"
    assert command[command.index("--yaw-limit") + 1] == "12.0"
    assert command[command.index("--forward-limit") + 1] == "10.0"
    assert command[command.index("--target-loss-after-seconds") + 1] == "24.0"
    assert command[command.index("--flight-log") + 1] == str(log)


def test_mixer_duration_defaults_to_virtual_script_length():
    assert mixer_duration(parse_args(["--virtual-hold-seconds", "40"])) == 54.0
    assert mixer_duration(parse_args(["--mixer-duration", "12.5"])) == 12.5


def test_summarize_mixer_log_counts_real_target_found_samples(tmp_path):
    path = tmp_path / "mixer.jsonl"
    write_jsonl(path, [
        {"event": "session_start"},
        {
            "event": "kenet_mixer_sample",
            "state": "TRACKING",
            "source": "kenet",
            "target_found": True,
            "first8": {"delta": [0, 0, 0, 0, 0, 0, 0, 0]},
            "frame_shape": [1280, 720],
        },
        {
            "event": "kenet_mixer_sample",
            "state": "TRACKING",
            "source": "pilot-target-lost",
            "target_found": False,
            "first8": {"delta": [0, 0, 0, 0, 0, 0, 0, 0]},
            "frame_shape": [1280, 720],
        },
        {
            "event": "kenet_mixer_sample",
            "state": "AI-ARMED",
            "source": "pilot",
            "target_found": False,
            "first8": {"delta": [0, 0, 0, 0, 0, 0, 0, 0]},
        },
    ])

    summary = summarize_mixer_log(path)

    assert summary["samples"] == 3
    assert summary["tracking_samples"] == 2
    assert summary["target_found_samples"] == 1
    assert summary["target_lost_source_samples"] == 1
    assert summary["ai_armed_samples"] == 1
    assert summary["kenet_source_samples"] == 1
    assert summary["max_abs_delta"] == 0
    assert summary["frame_shapes"] == [[1280, 720]]


def test_validate_mixer_summary_rejects_missing_target_found():
    args = parse_args(["--min-target-found-samples", "1"])
    summary = {
        "samples": 1,
        "tracking_samples": 1,
        "target_found_samples": 0,
        "target_lost_source_samples": 0,
        "kenet_source_samples": 0,
        "ai_armed_samples": 0,
        "tracker_errors": [],
        "max_abs_delta": 0,
    }

    failures = validate_mixer_summary(summary, args)

    assert any("target_found" in failure for failure in failures)


def test_validate_mixer_summary_enforces_target_loss_gate():
    args = parse_args([
        "--min-tracking-samples", "2",
        "--min-target-found-samples", "1",
        "--min-kenet-source-samples", "1",
        "--min-target-lost-samples", "2",
        "--min-ai-armed-samples", "1",
    ])
    summary = {
        "samples": 3,
        "tracking_samples": 2,
        "target_found_samples": 1,
        "target_lost_source_samples": 1,
        "kenet_source_samples": 1,
        "ai_armed_samples": 0,
        "tracker_errors": [],
        "max_abs_delta": 0,
        "max_abs_pitch_delta": 0,
        "max_abs_yaw_delta": 0,
    }

    failures = validate_mixer_summary(summary, args)

    assert "source=pilot-target-lost sample sayisi yetersiz: 1" in failures
    assert "AI-ARMED sample sayisi yetersiz: 0" in failures


def test_validate_mixer_summary_enforces_axis_delta_gates(tmp_path):
    path = tmp_path / "mixer.jsonl"
    write_jsonl(path, [
        {
            "event": "kenet_mixer_sample",
            "state": "TRACKING",
            "source": "kenet",
            "target_found": True,
            "first8": {"delta": [0, 3, 0, 5, 0, 0, 0, 0]},
        }
    ])
    args = parse_args([
        "--min-tracking-samples", "1",
        "--min-target-found-samples", "1",
        "--min-kenet-source-samples", "1",
        "--max-abs-delta", "10",
        "--min-abs-pitch-delta", "4",
        "--min-abs-yaw-delta", "5",
        "--max-abs-pitch-delta", "2",
        "--max-abs-yaw-delta", "4",
    ])

    failures = validate_mixer_summary(summarize_mixer_log(path), args)

    assert "pitch delta yetersiz: 3 < 4" in failures
    assert "pitch delta limiti asildi: 3 > 2" in failures
    assert "yaw delta limiti asildi: 5 > 4" in failures
    assert not any("yaw delta yetersiz" in failure for failure in failures)
