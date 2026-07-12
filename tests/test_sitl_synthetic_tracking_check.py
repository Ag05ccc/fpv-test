import json
import sys
from pathlib import Path

TOOLS_DIR = Path(__file__).resolve().parents[1] / "tools"
if str(TOOLS_DIR) not in sys.path:
    sys.path.insert(0, str(TOOLS_DIR))

from sitl_synthetic_tracking_check import (  # noqa: E402
    apply_profile_defaults,
    build_checker_command,
    build_mixer_command,
    empty_mixer_summary,
    estimated_command,
    parse_args,
    summarize_mixer_log,
    validate_mixer_summary,
)


def test_profile_defaults_fill_target_values():
    args = parse_args(["--profile", "yaw-negative"])

    apply_profile_defaults(args)

    assert args.synthetic_target_x == 300.0
    assert args.synthetic_target_width == 120.0
    assert args.synthetic_target_height == 120.0
    assert args.yaw_limit == 300.0
    assert args.forward_limit == 0.0


def test_micro_profile_defaults_limit_single_axis():
    args = parse_args(["--profile", "pitch-micro-positive"])

    apply_profile_defaults(args)

    assert args.synthetic_target_x == 320.0
    assert args.synthetic_target_width == 95.0
    assert args.yaw_limit == 0.0
    assert args.forward_limit == 10.0
    assert args.yaw_ki == 0.0
    assert args.yaw_kd == 0.0
    assert args.forward_ki == 0.0
    assert args.forward_kd == 0.0


def test_estimated_command_reports_micro_pwm_delta():
    args = apply_profile_defaults(parse_args(["--profile", "combined-micro"]))

    estimate = estimated_command(args)

    assert estimate["yaw_error"] == 15.0
    assert estimate["forward_error"] == 25.0
    assert estimate["yaw_delta"] == 12.0
    assert estimate["pitch_delta"] == 10.0


def test_build_checker_command_uses_external_acceptance_gate(tmp_path):
    args = apply_profile_defaults(parse_args([
        "--profile", "centered",
        "--yaw-pid", "23,0,0",
        "--pitch-pid", "23,0,0",
        "--pitch-rc-rate", "60",
        "--pitch-rate", "70",
        "--pitch-rate-limit", "450",
    ]))
    log = tmp_path / "diag.jsonl"

    command = build_checker_command(args, log)

    assert "sitl_virtual_takeoff_check.py" in command[1]
    assert "--rc-driver" in command
    assert command[command.index("--rc-driver") + 1] == "external"
    assert command[command.index("--diagnostics-log-file") + 1] == str(log)
    assert "--safe-yaw-authority" in command
    assert command[command.index("--yaw-pid") + 1] == "23,0,0"
    assert command[command.index("--pitch-pid") + 1] == "23,0,0"
    assert command[command.index("--pitch-rc-rate") + 1] == "60"
    assert command[command.index("--pitch-rate") + 1] == "70"
    assert command[command.index("--pitch-rate-limit") + 1] == "450"
    assert "--max-step-size" not in command


def test_build_checker_command_passes_max_step_size(tmp_path):
    args = apply_profile_defaults(parse_args([
        "--profile", "centered",
        "--max-step-size", "0.0025",
    ]))
    log = tmp_path / "diag.jsonl"

    command = build_checker_command(args, log)

    assert command[command.index("--max-step-size") + 1] == "0.0025"


def test_build_mixer_command_uses_synthetic_target_profile(tmp_path):
    args = apply_profile_defaults(parse_args([
        "--profile", "pitch-positive",
        "--synthetic-target-delay-seconds", "18",
        "--synthetic-target-loss-after-seconds", "24",
        "--lost-seconds", "0.5",
    ]))
    log = tmp_path / "mixer.jsonl"

    command = build_mixer_command(args, log)

    assert "kenet_sitl_mixer.py" in command[1]
    assert "--pilot-source" in command
    assert command[command.index("--pilot-source") + 1] == "virtual"
    assert "--synthetic-target" in command
    assert command[command.index("--frame-width") + 1] == "640"
    assert command[command.index("--desired-target-width") + 1] == "120.0"
    assert command[command.index("--synthetic-target-x") + 1] == "320.0"
    assert command[command.index("--synthetic-target-width") + 1] == "80.0"
    assert command[command.index("--synthetic-target-delay-seconds") + 1] == "18.0"
    assert command[command.index("--synthetic-target-loss-after-seconds") + 1] == "24.0"
    assert command[command.index("--lost-seconds") + 1] == "0.5"
    assert command[command.index("--yaw-ki") + 1] == "0.05"
    assert command[command.index("--yaw-kd") + 1] == "0.15"
    assert command[command.index("--yaw-limit") + 1] == "0.0"
    assert command[command.index("--forward-ki") + 1] == "0.02"
    assert command[command.index("--forward-kd") + 1] == "0.1"
    assert command[command.index("--forward-limit") + 1] == "250.0"
    assert command[command.index("--flight-log") + 1] == str(log)


def write_jsonl(path, records):
    with path.open("w", encoding="utf-8") as handle:
        for record in records:
            handle.write(json.dumps(record) + "\n")


def test_synthetic_mixer_summary_validates_target_found_and_delta(tmp_path):
    log = tmp_path / "mixer.jsonl"
    write_jsonl(log, [
        {
            "event": "kenet_mixer_sample",
            "state": "TRACKING",
            "source": "kenet",
            "target_found": True,
            "first8": {"delta": [0, 0, 0, 0, 0, 0, 0, 0]},
        }
        for _ in range(6)
    ])
    args = apply_profile_defaults(parse_args([
        "--profile", "centered",
        "--max-abs-delta", "0",
    ]))

    summary = summarize_mixer_log(log)

    assert summary["samples"] == 6
    assert summary["tracking_samples"] == 6
    assert summary["target_found_samples"] == 6
    assert summary["kenet_source_samples"] == 6
    assert validate_mixer_summary(summary, args) == []


def test_synthetic_mixer_summary_reports_missing_target_found_and_delta_limit(tmp_path):
    log = tmp_path / "mixer.jsonl"
    write_jsonl(log, [
        {
            "event": "kenet_mixer_sample",
            "state": "TRACKING",
            "source": "pilot-target-lost",
            "target_found": False,
            "first8": {"delta": [0, 0, 0, 12, 0, 0, 0, 0]},
        }
    ])
    args = apply_profile_defaults(parse_args([
        "--profile", "centered",
        "--max-abs-delta", "0",
    ]))

    failures = validate_mixer_summary(summarize_mixer_log(log), args)

    assert "target_found sample sayisi yetersiz: 0" in failures
    assert "source=kenet sample sayisi yetersiz: 0" in failures
    assert "mixer delta limiti asildi: 12 > 0" in failures


def test_synthetic_mixer_summary_enforces_axis_delta_gates(tmp_path):
    log = tmp_path / "mixer.jsonl"
    write_jsonl(log, [
        {
            "event": "kenet_mixer_sample",
            "state": "TRACKING",
            "source": "kenet",
            "target_found": True,
            "first8": {"delta": [0, 2, 0, 6, 0, 0, 0, 0]},
        }
    ])
    args = apply_profile_defaults(parse_args([
        "--profile", "centered",
        "--min-tracking-samples", "1",
        "--min-target-found-samples", "1",
        "--min-kenet-source-samples", "1",
        "--max-abs-delta", "10",
        "--min-abs-pitch-delta", "3",
        "--min-abs-yaw-delta", "6",
        "--max-abs-pitch-delta", "1",
        "--max-abs-yaw-delta", "5",
    ]))

    failures = validate_mixer_summary(summarize_mixer_log(log), args)

    assert "pitch delta yetersiz: 2 < 3" in failures
    assert "pitch delta limiti asildi: 2 > 1" in failures
    assert "yaw delta limiti asildi: 6 > 5" in failures
    assert not any("yaw delta yetersiz" in failure for failure in failures)


def test_empty_mixer_summary_validates_as_failures_not_key_error():
    args = apply_profile_defaults(parse_args(["--profile", "centered"]))

    failures = validate_mixer_summary(empty_mixer_summary(), args)

    assert "mixer log sample yok" in failures
    assert "TRACKING sample sayisi yetersiz: 0" in failures


def test_synthetic_mixer_summary_validates_target_loss_gate(tmp_path):
    log = tmp_path / "mixer.jsonl"
    write_jsonl(log, [
        {
            "event": "kenet_mixer_sample",
            "state": "TRACKING",
            "source": "kenet",
            "target_found": True,
            "first8": {"delta": [0, 0, 0, 0, 0, 0, 0, 0]},
        },
        {
            "event": "kenet_mixer_sample",
            "state": "TRACKING",
            "source": "pilot-target-lost",
            "target_found": False,
            "first8": {"delta": [0, 0, 0, 0, 0, 0, 0, 0]},
        },
        {
            "event": "kenet_mixer_sample",
            "state": "AI-ARMED",
            "source": "pilot",
            "target_found": False,
            "first8": {"delta": [0, 0, 0, 0, 0, 0, 0, 0]},
        },
    ])
    args = apply_profile_defaults(parse_args([
        "--profile", "centered",
        "--min-tracking-samples", "2",
        "--min-target-found-samples", "1",
        "--min-kenet-source-samples", "1",
        "--min-target-lost-samples", "1",
        "--min-ai-armed-samples", "1",
        "--max-abs-delta", "0",
    ]))

    summary = summarize_mixer_log(log)

    assert summary["target_lost_source_samples"] == 1
    assert summary["ai_armed_samples"] == 1
    assert validate_mixer_summary(summary, args) == []
