import json
import sys
from pathlib import Path

TOOLS_DIR = Path(__file__).resolve().parents[1] / "tools"
if str(TOOLS_DIR) not in sys.path:
    sys.path.insert(0, str(TOOLS_DIR))

from analyze_sitl_log import print_summary, summarize  # noqa: E402


def write_jsonl(path, records):
    with path.open("w", encoding="utf-8") as handle:
        for record in records:
            handle.write(json.dumps(record) + "\n")


def test_analyzer_reports_missing_dashboard_data_instead_of_all_clear(tmp_path, capsys):
    path = tmp_path / "mixer-only.jsonl"
    write_jsonl(path, [
        {
            "event": "kenet_mixer_sample",
            "time_iso": "2026-06-23T00:00:00Z",
            "state": "TRACKING",
            "source": "kenet",
            "target_found": True,
            "first8": {
                "delta": [0, -120, 0, 90, 0, 0, 0, 0],
                "final": [1500, 1380, 1000, 1590, 1000, 2000, 1000, 1500],
            },
        }
    ])

    print_summary(summarize([path]))

    output = capsys.readouterr().out
    assert "attitude samples: 0" in output
    assert "motor samples: 0" in output
    assert "attitude data unavailable" in output
    assert "motor data unavailable" in output
    assert "attitude stayed below 35 deg" not in output
    assert "motor spread did not show a large correction" not in output
    assert "using legacy first8 channel fields: 1" in output


def test_analyzer_uses_available_attitude_and_motor_samples(tmp_path, capsys):
    path = tmp_path / "dashboard.jsonl"
    write_jsonl(path, [
        {
            "event": "dashboard_sample",
            "time_iso": "2026-06-23T00:00:01Z",
            "msp": {
                "connected": True,
                "armed": True,
                "attitude": {"roll": 70.0, "pitch": 5.0, "yaw": 0.0},
                "motor": [1000, 1500, 1000, 1500],
            },
            "kenet": {"state": "TRACKING"},
            "autopilot_mode": {"state": "ANGLE"},
        }
    ])

    print_summary(summarize([path]))

    output = capsys.readouterr().out
    assert "attitude samples: 1" in output
    assert "motor samples: 1" in output
    assert "WARNING: attitude exceeded 60 deg" in output
    assert "WARNING: motor spread exceeded 400 us" in output
    assert "attitude data unavailable" not in output
    assert "motor data unavailable" not in output


def test_analyzer_reads_diagnostic_samples_and_ignores_unused_motor_slots(tmp_path, capsys):
    path = tmp_path / "diagnostics.jsonl"
    write_jsonl(path, [
        {
            "event": "diagnostic_sample",
            "time_iso": "2026-06-29T21:23:16+0300",
            "msp": {
                "connected": True,
                "attitude": {"roll": 72.6, "pitch": 30.8, "yaw": 270},
                "motor": [2000, 1176, 1055, 1277, 0, 0, 0, 0],
                "status": {
                    "armed": True,
                    "active_modes_valid": True,
                    "active_modes": ["ARM", "ANGLE"],
                    "arming_disable_names": [],
                },
            },
            "channels": [
                {"label": "Throttle", "delta": 0},
            ],
        }
    ])

    print_summary(summarize([path]))

    output = capsys.readouterr().out
    assert "samples: 1" in output
    assert "attitude samples: 1" in output
    assert "motor samples: 1" in output
    assert "ARM: 1" in output
    assert "ANGLE: 1" in output
    assert "max motor spread: 945" in output
    assert "motors=2000,1176,1055,1277" in output
    assert "WARNING: attitude exceeded 60 deg" in output


def test_analyzer_reads_all_mixer_channels_when_available(tmp_path, capsys):
    path = tmp_path / "mixer-all-channels.jsonl"
    pilot = [1500] * 16
    pilot[2] = 1000
    final = list(pilot)
    final[8] = 1700
    final[15] = 1300
    write_jsonl(path, [
        {
            "event": "kenet_mixer_sample",
            "time_iso": "2026-06-30T00:00:00Z",
            "state": "TRACKING",
            "source": "kenet",
            "target_found": True,
            "pilot_channels": pilot,
            "final_channels": final,
        }
    ])

    print_summary(summarize([path]))

    output = capsys.readouterr().out
    assert "CH9: +200" in output
    assert "CH16: -200" in output
    assert "using legacy first8 channel fields" not in output
