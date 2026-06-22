#!/usr/bin/env python3
"""
Single-file web GUI for watching Kenet and autopilot switch states.

This intentionally uses only Python stdlib. It reads /dev/input/jsX directly and
serves a small local web UI with:
  - Kenet algorithm state from the 3-position mode switch
  - Autopilot arm command state from the 2-position switch
  - Autopilot mode command state from the second 3-position switch
  - RC channel and raw axis/button values
"""

import argparse
import errno
import glob
import json
import os
import select
import struct
import sys
import threading
import time
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse


JS_EVENT_FORMAT = "<IhBB"
JS_EVENT_SIZE = struct.calcsize(JS_EVENT_FORMAT)
JS_EVENT_BUTTON = 0x01
JS_EVENT_AXIS = 0x02
JS_EVENT_INIT = 0x80
AXIS_MAX = 32767.0

RC_LABELS = ["Roll", "Pitch", "Throttle", "Yaw", "AUX1", "AUX2", "AUX3", "AUX4"]


class LinuxJoystick:
    def __init__(self, device):
        self.device = device
        self.fd = None
        self.axes = {}
        self.buttons = {}

    def open(self):
        self.close()
        self.fd = os.open(self.device, os.O_RDONLY | os.O_NONBLOCK)

    def close(self):
        if self.fd is not None:
            os.close(self.fd)
            self.fd = None

    def poll(self, timeout=0):
        if self.fd is None:
            return []
        ready, _, _ = select.select([self.fd], [], [], timeout)
        if not ready:
            return []

        events = []
        while True:
            try:
                data = os.read(self.fd, JS_EVENT_SIZE)
            except BlockingIOError:
                break
            except OSError as exc:
                if exc.errno in (errno.EAGAIN, errno.EWOULDBLOCK):
                    break
                raise

            if not data or len(data) < JS_EVENT_SIZE:
                break

            t_ms, value, event_type, number = struct.unpack(JS_EVENT_FORMAT, data)
            is_init = bool(event_type & JS_EVENT_INIT)
            event_type &= ~JS_EVENT_INIT

            if event_type == JS_EVENT_AXIS:
                self.axes[number] = value
                kind = "axis"
            elif event_type == JS_EVENT_BUTTON:
                self.buttons[number] = value
                kind = "button"
            else:
                kind = "unknown"
            events.append({
                "time_ms": t_ms,
                "kind": kind,
                "number": number,
                "value": value,
                "init": is_init,
            })
        return events


def clamp_rc(value):
    return int(max(1000, min(2000, round(value))))


def axis_to_rc(value, invert=False):
    if invert:
        value = -value
    return clamp_rc(1500 + (value / AXIS_MAX) * 500)


def axis_to_two_pos_rc(value, invert=False):
    if invert:
        value = -value
    return 2000 if value > 0 else 1000


def axis_to_three_pos_rc(value, invert=False):
    if invert:
        value = -value
    if value < -AXIS_MAX / 3:
        return 1000
    if value > AXIS_MAX / 3:
        return 2000
    return 1500


def kenet_state(value, arm_threshold, track_threshold):
    if value >= track_threshold:
        return "TRACKING"
    if value >= arm_threshold:
        return "AI-ARMED"
    return "IDLE"


def autopilot_state(value, arm_threshold):
    return "ARMED" if value >= arm_threshold else "DISARMED"


def three_position_state(value):
    if value < 1300:
        return "LOW"
    if value > 1700:
        return "HIGH"
    return "MID"


class StateSampler:
    def __init__(self, args):
        self.args = args
        self.joystick = LinuxJoystick(args.device)
        self.lock = threading.Lock()
        self.running = False
        self.thread = None
        self.error = None
        self.start_time = time.monotonic()
        self.last_event_time = None
        self.axes = {}
        self.buttons = {}
        self.channels = [1500] * 8

    def start(self):
        self.running = True
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def stop(self):
        self.running = False
        if self.thread:
            self.thread.join(timeout=1.0)
        self.joystick.close()

    def _run(self):
        period = 1.0 / self.args.rate_hz
        next_open_attempt = 0.0
        while self.running:
            try:
                if self.joystick.fd is None:
                    now = time.monotonic()
                    if now < next_open_attempt:
                        time.sleep(min(period, next_open_attempt - now))
                        continue
                    try:
                        self.joystick.open()
                    except OSError as exc:
                        next_open_attempt = now + 1.0
                        with self.lock:
                            self.error = self._device_error(exc)
                        continue

                events = self.joystick.poll(timeout=0.02)
                with self.lock:
                    self.axes = dict(self.joystick.axes)
                    self.buttons = dict(self.joystick.buttons)
                    self.channels = self._make_channels(self.axes)
                    if events:
                        self.last_event_time = time.monotonic()
                    self.error = None
            except Exception as exc:
                self.joystick.close()
                with self.lock:
                    self.error = str(exc)
            time.sleep(period)

    def _device_error(self, exc):
        devices = sorted(glob.glob("/dev/input/js*"))
        if devices:
            return (
                "Failed to open %s: %s. Available joystick devices: %s" %
                (self.args.device, exc, ", ".join(devices))
            )
        return (
            "Waiting for joystick device %s: %s. No /dev/input/js* devices found." %
            (self.args.device, exc)
        )

    def _make_channels(self, axes):
        channels = [1500] * 8
        channels[0] = axis_to_rc(axes.get(0, 0))
        channels[1] = axis_to_rc(axes.get(1, 0), invert=True)
        channels[2] = axis_to_rc(axes.get(2, 0))
        channels[3] = axis_to_rc(axes.get(3, 0))

        if 0 <= self.args.autopilot_ch < len(channels):
            channels[self.args.autopilot_ch] = axis_to_two_pos_rc(
                axes.get(self.args.autopilot_axis, 0),
                invert=self.args.autopilot_invert,
            )
        if 0 <= self.args.algorithm_ch < len(channels):
            channels[self.args.algorithm_ch] = axis_to_three_pos_rc(
                axes.get(self.args.algorithm_axis, 0),
                invert=self.args.algorithm_invert,
            )
        if 0 <= self.args.autopilot_mode_ch < len(channels):
            channels[self.args.autopilot_mode_ch] = axis_to_three_pos_rc(
                axes.get(self.args.autopilot_mode_axis, 0),
                invert=self.args.autopilot_mode_invert,
            )
        return channels

    def snapshot(self):
        with self.lock:
            axes = dict(self.axes)
            buttons = dict(self.buttons)
            channels = list(self.channels)
            error = self.error
            last_event_time = self.last_event_time
            connected = self.joystick.fd is not None

        algo_value = channels[self.args.algorithm_ch]
        auto_value = channels[self.args.autopilot_ch]
        mode_value = channels[self.args.autopilot_mode_ch]
        if connected:
            algo_state = kenet_state(
                algo_value,
                self.args.algorithm_arm_threshold,
                self.args.algorithm_track_threshold,
            )
            auto_state = autopilot_state(auto_value, self.args.autopilot_arm_threshold)
            mode_state = three_position_state(mode_value)
        else:
            algo_state = "NO DEVICE"
            auto_state = "NO DEVICE"
            mode_state = "NO DEVICE"

        now = time.monotonic()
        return {
            "device": self.args.device,
            "connected": connected,
            "uptime_s": round(now - self.start_time, 2),
            "last_event_age_s": None if last_event_time is None else round(now - last_event_time, 2),
            "error": error,
            "algorithm": {
                "state": algo_state,
                "value": algo_value,
                "raw_axis_value": axes.get(self.args.algorithm_axis),
                "axis": self.args.algorithm_axis,
                "channel": self.args.algorithm_ch,
                "channel_label": "CH%d" % (self.args.algorithm_ch + 1),
            },
            "autopilot": {
                "state": auto_state,
                "value": auto_value,
                "raw_axis_value": axes.get(self.args.autopilot_axis),
                "axis": self.args.autopilot_axis,
                "channel": self.args.autopilot_ch,
                "channel_label": "CH%d" % (self.args.autopilot_ch + 1),
            },
            "autopilot_mode": {
                "state": mode_state,
                "value": mode_value,
                "raw_axis_value": axes.get(self.args.autopilot_mode_axis),
                "axis": self.args.autopilot_mode_axis,
                "channel": self.args.autopilot_mode_ch,
                "channel_label": "CH%d" % (self.args.autopilot_mode_ch + 1),
            },
            "channels": [
                {"index": idx, "label": RC_LABELS[idx], "value": value}
                for idx, value in enumerate(channels)
            ],
            "axes": [
                {"index": idx, "value": value}
                for idx, value in sorted(axes.items())
            ],
            "buttons": [
                {"index": idx, "value": value}
                for idx, value in sorted(buttons.items())
            ],
        }


HTML = """<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Kenet State Monitor</title>
  <style>
    :root {
      color-scheme: dark;
      --bg: #0d1117;
      --panel: #161b22;
      --panel2: #21262d;
      --line: #30363d;
      --text: #e6edf3;
      --muted: #8b949e;
      --blue: #2f81f7;
      --green: #3fb950;
      --yellow: #d29922;
      --red: #f85149;
    }
    * { box-sizing: border-box; }
    body {
      margin: 0;
      background: var(--bg);
      color: var(--text);
      font-family: system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
      letter-spacing: 0;
    }
    main {
      width: min(1180px, calc(100vw - 28px));
      margin: 0 auto;
      padding: 18px 0 28px;
    }
    header {
      display: flex;
      align-items: baseline;
      justify-content: space-between;
      gap: 16px;
      padding: 2px 0 14px;
      border-bottom: 1px solid var(--line);
    }
    h1 {
      margin: 0;
      font-size: 22px;
      font-weight: 700;
    }
    .status {
      color: var(--muted);
      font-size: 13px;
      text-align: right;
    }
    .states {
      display: grid;
      grid-template-columns: repeat(3, minmax(0, 1fr));
      gap: 12px;
      margin-top: 14px;
    }
    .state-panel, .panel {
      background: var(--panel);
      border: 1px solid var(--line);
      border-radius: 8px;
      padding: 14px;
    }
    .state-title {
      color: var(--muted);
      font-size: 13px;
      margin-bottom: 8px;
    }
    .state-value {
      font-size: 34px;
      line-height: 1;
      font-weight: 800;
    }
    .state-meta {
      margin-top: 10px;
      color: var(--muted);
      font-size: 13px;
    }
    .state-panel.idle .state-value,
    .state-panel.disarmed .state-value,
    .state-panel.low .state-value,
    .state-panel.no-device .state-value { color: var(--red); }
    .state-panel.ai-armed .state-value,
    .state-panel.mid .state-value { color: var(--yellow); }
    .state-panel.tracking .state-value,
    .state-panel.high .state-value,
    .state-panel.armed .state-value { color: var(--green); }
    .grid {
      display: grid;
      grid-template-columns: 1.15fr 0.85fr;
      gap: 12px;
      margin-top: 12px;
    }
    h2 {
      margin: 0 0 10px;
      font-size: 15px;
      font-weight: 700;
    }
    .row {
      display: grid;
      grid-template-columns: 112px 1fr 56px;
      align-items: center;
      gap: 10px;
      min-height: 28px;
      font-size: 13px;
    }
    .bar {
      height: 16px;
      background: var(--panel2);
      border: 1px solid var(--line);
      border-radius: 4px;
      overflow: hidden;
      position: relative;
    }
    .fill {
      height: 100%;
      background: var(--blue);
      width: 0%;
    }
    .center {
      position: absolute;
      top: 0;
      bottom: 0;
      left: 50%;
      width: 1px;
      background: rgba(230, 237, 243, 0.75);
    }
    .buttons {
      display: grid;
      grid-template-columns: repeat(8, minmax(34px, 1fr));
      gap: 6px;
      margin-top: 8px;
    }
    .button-cell {
      background: var(--panel2);
      border: 1px solid var(--line);
      border-radius: 4px;
      padding: 6px 4px;
      text-align: center;
      font-size: 12px;
      color: var(--muted);
    }
    .button-cell.on {
      background: #1f6f43;
      color: #fff;
    }
    .error {
      display: none;
      margin-top: 12px;
      padding: 10px 12px;
      background: rgba(248, 81, 73, 0.16);
      border: 1px solid rgba(248, 81, 73, 0.45);
      border-radius: 8px;
      color: #ffb3ad;
      font-size: 13px;
    }
    @media (max-width: 760px) {
      header, .states, .grid { display: block; }
      .state-panel, .panel { margin-top: 12px; }
      .row { grid-template-columns: 96px 1fr 48px; }
      .status { text-align: left; margin-top: 8px; }
    }
  </style>
</head>
<body>
  <main>
    <header>
      <h1>Kenet State Monitor</h1>
      <div class="status" id="status">connecting</div>
    </header>

    <section class="states">
      <div class="state-panel" id="algorithmPanel">
        <div class="state-title">Kenet algorithm state</div>
        <div class="state-value" id="algorithmState">-</div>
        <div class="state-meta" id="algorithmMeta">-</div>
      </div>
      <div class="state-panel" id="autopilotPanel">
        <div class="state-title">Autopilot arm state</div>
        <div class="state-value" id="autopilotState">-</div>
        <div class="state-meta" id="autopilotMeta">-</div>
      </div>
      <div class="state-panel" id="autopilotModePanel">
        <div class="state-title">Autopilot mode switch</div>
        <div class="state-value" id="autopilotModeState">-</div>
        <div class="state-meta" id="autopilotModeMeta">-</div>
      </div>
    </section>

    <div class="error" id="errorBox"></div>

    <section class="grid">
      <div class="panel">
        <h2>RC/PWM commands</h2>
        <div id="channels"></div>
      </div>
      <div class="panel">
        <h2>Raw inputs</h2>
        <div id="axes"></div>
        <div class="buttons" id="buttons"></div>
      </div>
    </section>
  </main>

  <script>
    const channelRoot = document.getElementById("channels");
    const axesRoot = document.getElementById("axes");
    const buttonsRoot = document.getElementById("buttons");

    function cls(state) {
      return String(state).toLowerCase().replace(/_/g, "-");
    }
    function pct(value, min, max) {
      return Math.max(0, Math.min(100, ((value - min) / (max - min)) * 100));
    }
    function row(label, value, min, max, center) {
      return `<div class="row">
        <div>${label}</div>
        <div class="bar"><div class="fill" style="width:${pct(value, min, max)}%"></div>${center ? '<div class="center"></div>' : ''}</div>
        <div>${value}</div>
      </div>`;
    }
    function setPanel(id, state) {
      const panel = document.getElementById(id);
      panel.className = `state-panel ${cls(state)}`;
    }
    async function refresh() {
      try {
        const res = await fetch("/api/state", {cache: "no-store"});
        const data = await res.json();
        document.getElementById("algorithmState").textContent = data.algorithm.state;
        document.getElementById("algorithmMeta").textContent =
          `${data.algorithm.channel_label} PWM ${data.algorithm.value} us | raw axis ${data.algorithm.axis}: ${data.algorithm.raw_axis_value ?? "none"}`;
        setPanel("algorithmPanel", data.algorithm.state);

        document.getElementById("autopilotState").textContent = data.autopilot.state;
        document.getElementById("autopilotMeta").textContent =
          `${data.autopilot.channel_label} PWM ${data.autopilot.value} us | raw axis ${data.autopilot.axis}: ${data.autopilot.raw_axis_value ?? "none"}`;
        setPanel("autopilotPanel", data.autopilot.state);

        document.getElementById("autopilotModeState").textContent = data.autopilot_mode.state;
        document.getElementById("autopilotModeMeta").textContent =
          `${data.autopilot_mode.channel_label} PWM ${data.autopilot_mode.value} us | raw axis ${data.autopilot_mode.axis}: ${data.autopilot_mode.raw_axis_value ?? "none"}`;
        setPanel("autopilotModePanel", data.autopilot_mode.state);

        document.getElementById("status").textContent =
          `${data.device} | uptime ${data.uptime_s}s | last event ${data.last_event_age_s ?? "none"}s`;

        channelRoot.innerHTML = data.channels
          .map(ch => row(`${ch.label} CH${ch.index + 1}`, ch.value, 1000, 2000, true))
          .join("");
        axesRoot.innerHTML = data.axes.length
          ? data.axes.map(axis => row(`Axis ${axis.index}`, axis.value, -32767, 32767, true)).join("")
          : `<div class="state-meta">waiting for axis events</div>`;
        buttonsRoot.innerHTML = data.buttons.length
          ? data.buttons.map(btn => `<div class="button-cell ${btn.value ? "on" : ""}">B${String(btn.index).padStart(2, "0")}</div>`).join("")
          : "";

        const errorBox = document.getElementById("errorBox");
        if (data.error) {
          errorBox.style.display = "block";
          errorBox.textContent = data.error;
        } else {
          errorBox.style.display = "none";
        }
      } catch (err) {
        document.getElementById("status").textContent = `connection lost: ${err}`;
      }
    }
    refresh();
    setInterval(refresh, 100);
  </script>
</body>
</html>
"""


class Handler(BaseHTTPRequestHandler):
    def do_GET(self):
        path = urlparse(self.path).path
        if path == "/":
            self._send(200, "text/html; charset=utf-8", HTML.encode("utf-8"))
        elif path == "/api/state":
            payload = json.dumps(self.server.sampler.snapshot()).encode("utf-8")
            self._send(200, "application/json; charset=utf-8", payload)
        else:
            self._send(404, "text/plain; charset=utf-8", b"not found")

    def _send(self, status, content_type, body):
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, _fmt, *_args):
        return


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--device", default="/dev/input/js0")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--rate-hz", type=float, default=50.0)
    parser.add_argument("--algorithm-axis", type=int, default=6)
    parser.add_argument("--algorithm-ch", type=int, default=5,
                        help="0-indexed RC channel for Kenet state switch")
    parser.add_argument("--algorithm-invert", action="store_true")
    parser.add_argument("--algorithm-arm-threshold", type=int, default=1300)
    parser.add_argument("--algorithm-track-threshold", type=int, default=1700)
    parser.add_argument("--autopilot-axis", type=int, default=4)
    parser.add_argument("--autopilot-ch", type=int, default=4,
                        help="0-indexed RC channel for autopilot arm switch")
    parser.add_argument("--autopilot-invert", action="store_true")
    parser.add_argument("--autopilot-arm-threshold", type=int, default=1700)
    parser.add_argument("--autopilot-mode-axis", type=int, default=5)
    parser.add_argument("--autopilot-mode-ch", type=int, default=6,
                        help="0-indexed RC channel for autopilot mode switch")
    parser.add_argument("--autopilot-mode-invert", action="store_true")
    parser.add_argument("--open", action="store_true", help="Open browser")
    args = parser.parse_args()
    if args.rate_hz <= 0:
        parser.error("--rate-hz must be positive")
    for name in ("algorithm_ch", "autopilot_ch", "autopilot_mode_ch"):
        if not 0 <= getattr(args, name) < 8:
            parser.error("--%s must be between 0 and 7" % name.replace("_", "-"))
    return args


def main():
    args = parse_args()
    sampler = StateSampler(args)
    try:
        sampler.start()
    except OSError as exc:
        print("Failed to open %s: %s" % (args.device, exc), file=sys.stderr)
        return 1

    server = ThreadingHTTPServer((args.host, args.port), Handler)
    server.sampler = sampler
    url = "http://%s:%d/" % (args.host, args.port)
    print("Kenet state monitor: %s" % url)
    print("algorithm: axis %d -> index %d / CH%d" %
          (args.algorithm_axis, args.algorithm_ch, args.algorithm_ch + 1))
    print("autopilot: axis %d -> index %d / CH%d" %
          (args.autopilot_axis, args.autopilot_ch, args.autopilot_ch + 1))
    print("autopilot mode: axis %d -> index %d / CH%d" %
          (args.autopilot_mode_axis, args.autopilot_mode_ch, args.autopilot_mode_ch + 1))
    print("Press Ctrl+C to stop.")
    if args.open:
        webbrowser.open(url)

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        server.server_close()
        sampler.stop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
