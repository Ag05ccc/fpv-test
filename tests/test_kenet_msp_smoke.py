import sys
from pathlib import Path
from types import SimpleNamespace

TOOLS_DIR = Path(__file__).resolve().parents[1] / "tools"
if str(TOOLS_DIR) not in sys.path:
    sys.path.insert(0, str(TOOLS_DIR))

from kenet_msp_smoke import (  # noqa: E402
    expected_msp_rc_readback,
    main,
    parse_channels,
    rc_readback_mismatches,
    resolve_port,
)


def test_parse_channels_validates_count_and_range():
    assert parse_channels("1500,1500,1000,1500,2000,1000,1500,1500") == [
        1500,
        1500,
        1000,
        1500,
        2000,
        1000,
        1500,
        1500,
    ]
    for value in ("1500,1500", "1500,not-a-number,1500,1500,1500,1500,1500,1500", "999,1500,1500,1500,1500,1500,1500,1500"):
        try:
            parse_channels(value)
        except Exception:
            pass
        else:
            raise AssertionError("invalid channel list should fail")


def test_resolve_port_prefers_msp_tcp():
    assert resolve_port(SimpleNamespace(msp_tcp="127.0.0.1:5761", port="/dev/ttyAMA0")) == "tcp://127.0.0.1:5761"
    assert resolve_port(SimpleNamespace(msp_tcp="tcp://127.0.0.1:5761", port="/dev/ttyAMA0")) == "tcp://127.0.0.1:5761"
    assert resolve_port(SimpleNamespace(msp_tcp=None, port="/dev/ttyAMA0")) == "/dev/ttyAMA0"


def test_set_raw_rc_readback_uses_msp_rc_order():
    pilot_channels = [1500, 1600, 1000, 1400, 2000, 2000, 1500, 1500]
    expected = [1500, 1600, 1400, 1000, 2000, 2000, 1500, 1500]

    assert expected_msp_rc_readback(pilot_channels) == expected
    assert rc_readback_mismatches(pilot_channels, expected) == []
    assert rc_readback_mismatches(pilot_channels, [1500] * 8)


def test_main_reports_pass_with_fake_msp_connection(monkeypatch, capsys):
    calls = []

    class FakeConnection:
        def __init__(self, port, baudrate, timeout):
            calls.append(("init", port, baudrate, timeout))
            self.sent_rc = None

        def connect(self):
            calls.append("connect")

        def disconnect(self):
            calls.append("disconnect")

        def get_api_version(self):
            return {"protocol_version": 0, "api_major": 1, "api_minor": 45}

        def get_rc_channels(self):
            if self.sent_rc is not None:
                return expected_msp_rc_readback(self.sent_rc)
            return [1500] * 8

        def get_attitude(self):
            return {"roll": 0.0, "pitch": 0.0, "yaw": 90}

        def send_rc(self, channels):
            calls.append(("send_rc", channels))
            self.sent_rc = channels

    monkeypatch.setattr("kenet_msp_smoke.MSPConnection", FakeConnection)
    monkeypatch.setattr("sys.argv", [
        "kenet_msp_smoke.py",
        "--msp-tcp", "127.0.0.1:5761",
        "--set-raw-rc", "1500,1500,1000,1500,2000,1000,1500,1500",
    ])

    assert main() == 0

    output = capsys.readouterr().out
    assert "MSP smoke: PASS" in output
    assert "set_raw_rc_check: PASS" in output
    assert ("init", "tcp://127.0.0.1:5761", 115200, 1.0) in calls
    assert ("send_rc", [1500, 1500, 1000, 1500, 2000, 1000, 1500, 1500]) in calls
