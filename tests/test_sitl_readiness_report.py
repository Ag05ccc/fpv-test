import json
import sys
from pathlib import Path


TOOLS_DIR = Path(__file__).resolve().parents[1] / "tools"
if str(TOOLS_DIR) not in sys.path:
    sys.path.insert(0, str(TOOLS_DIR))

from sitl_readiness_report import build_report, diagnostic_pass, mixer_target_loss_pass  # noqa: E402


def write_jsonl(path, records):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as handle:
        for record in records:
            handle.write(json.dumps(record) + "\n")


def diagnostic_record(roll=0.0, pitch=0.0, motors=None):
    return {
        "event": "diagnostic_sample",
        "time_iso": "2026-06-30T00:00:00Z",
        "msp": {
            "connected": True,
            "status": {"armed": True, "active_modes": ["ARM", "ANGLE"]},
            "attitude": {"roll": roll, "pitch": pitch},
            "motor": motors or [1200, 1200, 1200, 1200],
        },
    }


def mixer_record(state, source, target_found, delta=None):
    return {
        "event": "kenet_mixer_sample",
        "state": state,
        "source": source,
        "target_found": target_found,
        "first8": {"delta": delta or [0, 0, 0, 0, 0, 0, 0, 0]},
    }


def test_diagnostic_pass_rejects_threshold_exceeded(tmp_path):
    path = tmp_path / "diag.jsonl"
    write_jsonl(path, [diagnostic_record(roll=70.0)])

    result = diagnostic_pass(path)

    assert result["ok"] is False
    assert result["reason"] == "threshold exceeded"


def test_mixer_target_loss_pass_requires_found_lost_and_ai_armed(tmp_path):
    path = tmp_path / "mixer.jsonl"
    write_jsonl(path, [
        mixer_record("TRACKING", "kenet", True),
        mixer_record("TRACKING", "pilot-target-lost", False),
        mixer_record("AI-ARMED", "pilot", False),
    ])

    result = mixer_target_loss_pass(path)

    assert result["ok"] is True
    assert result["target_found_samples"] == 1
    assert result["target_lost_source_samples"] == 1
    assert result["ai_armed_samples"] == 1


def test_build_report_marks_no_hardware_ready_and_external_waiting(tmp_path):
    write_jsonl(
        tmp_path / "logs/sitl/20260630-064350-takeoff-diagnostics.jsonl",
        [diagnostic_record()],
    )
    write_jsonl(
        tmp_path / "logs/sitl/20260630-video-target-loss-p23-delay18-final-diagnostics.jsonl",
        [diagnostic_record()],
    )
    write_jsonl(
        tmp_path / "logs/sitl/20260630-video-target-loss-p23-delay18-final-mixer.jsonl",
        [
            mixer_record("TRACKING", "kenet", True),
            mixer_record("TRACKING", "pilot-target-lost", False),
            mixer_record("AI-ARMED", "pilot", False),
        ],
    )
    (tmp_path / "docs").mkdir()
    (tmp_path / "docs/sitl-quickstart.md").write_text("# quickstart\n", encoding="utf-8")

    report = build_report(tmp_path, joystick_glob=str(tmp_path / "missing-js*"))

    assert report["no_hardware_ok"] is True
    assert report["external_items"]["physical_rc_device"]["ok"] is False
    assert "waiting" in report["external_items"]["physical_rc_device"]["reason"]


def test_build_report_marks_physical_device_ready_but_keeps_motion_next_step(tmp_path):
    write_jsonl(
        tmp_path / "logs/sitl/20260630-064350-takeoff-diagnostics.jsonl",
        [diagnostic_record()],
    )
    write_jsonl(
        tmp_path / "logs/sitl/20260630-video-target-loss-p23-delay18-final-diagnostics.jsonl",
        [diagnostic_record()],
    )
    write_jsonl(
        tmp_path / "logs/sitl/20260630-video-target-loss-p23-delay18-final-mixer.jsonl",
        [
            mixer_record("TRACKING", "kenet", True),
            mixer_record("TRACKING", "pilot-target-lost", False),
            mixer_record("AI-ARMED", "pilot", False),
        ],
    )
    (tmp_path / "docs").mkdir()
    (tmp_path / "docs/sitl-quickstart.md").write_text("# quickstart\n", encoding="utf-8")
    joystick = tmp_path / "js0"
    joystick.touch()

    report = build_report(tmp_path, joystick_glob=str(tmp_path / "js*"))

    assert report["external_items"]["physical_rc_device"]["ok"] is True
    assert report["external_items"]["physical_rc_device"]["devices"] == [str(joystick)]
    assert "preflight" in report["external_items"]["physical_rc_device"]["next_step"]
