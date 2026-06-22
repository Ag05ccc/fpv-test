# Kenet

Visual target tracking + PID control + Betaflight MSP for FPV drones.

A camera feeds an OpenCV tracker, a PID controller turns the target's position
into yaw and pitch commands, and those commands go to a Betaflight flight
controller over MSP. Telemetry is sent to a ground station over UDP. It runs on a
companion board (Raspberry Pi / Jetson) next to the flight controller, and the
whole thing can also be tested on a PC with no hardware.

## Architecture

```
Camera -> CameraCapture (threaded) -> ObjectTracker (CSRT/KCF)
    -> FlightController (PID: yaw + pitch) -> MSP_SET_RAW_RC -> Betaflight FC
    -> GCSLink (UDP telemetry) -> Ground station
```

A single 3-position AUX switch drives the state machine: IDLE (pilot flies),
AI-ARMED (camera ready, pilot still flies), TRACKING (Kenet takes over pitch and
yaw through MSP Override).

## Project structure

```
kenet/
  __init__.py    - Package exports
  camera.py      - Threaded OpenCV video capture
  tracker.py     - CSRT / KCF object tracking (needs opencv-contrib)
  controller.py  - PID controllers + rate-limited flight control
  msp.py         - MSPv1 serial protocol for Betaflight
  gcs.py         - UDP telemetry to the ground station
  joystick.py    - Desk-test AUX input from a USB joystick (no FC)
  pipeline.py    - 3-position state machine + control loop
kenet.py         - CLI entry point
quick_gcs.py     - Minimal telemetry viewer
tools/           - Betaflight SITL RC bridge, probe, and switch monitors
tests/           - Unit tests (pytest)
```

## Install

```bash
python -m venv fpv_env
source fpv_env/bin/activate
pip install -r requirements.txt
```

`requirements.txt` pulls in `opencv-contrib-python`, `pyserial`, and `numpy`.
The contrib build of OpenCV matters: the CSRT and KCF trackers only exist there.
Plain `opencv-python` does not have them, and Kenet raises a clear
`TrackerUnavailableError` if it can't create a tracker.

For development and tests:

```bash
pip install -r requirements-dev.txt
python -m pytest
```

---

## How to run

There are three ways to run Kenet, depending on what hardware you have. All of
them use the same pipeline; only the source of the AUX switch differs.

### 1. With a flight controller (normal use and desk testing)

This is the real setup. Kenet reads the AUX switch from the flight controller
over MSP and, in TRACKING, sends pitch/yaw back to it. You need the FC connected
(over USB for desk testing, or a UART on a Pi/Jetson) with a real receiver bound
to your transmitter. Close Betaflight Configurator first — it locks the port.

```bash
# Find the FC serial port (USB FC is usually /dev/ttyACM0)
ls /dev/ttyACM*

# Terminal 1 — run the pipeline, send telemetry to this same PC
python kenet.py --port /dev/ttyACM0 --gcs-host 127.0.0.1

# Terminal 2 — watch telemetry
python quick_gcs.py
```

On a companion board the port is typically a UART, e.g.
`python kenet.py --port /dev/ttyAMA0 --gcs-host 192.168.1.100`.

Then flip the 3-position AUX switch and watch the state transitions in the logs
(IDLE → AI-ARMED → TRACKING). See "Betaflight setup" below for the required MSP
Override configuration, and always bench-test with the propellers removed.

### 2. Without a flight controller (joystick mode)

When you plug the transmitter into the PC in USB "game controller" mode it shows
up as a joystick (`/dev/input/js0`), not as an FC. In this mode Kenet reads the
AUX switch straight from the joystick, so you can exercise the state machine, the
camera, and the tracker with no flight controller at all. The MSP override has
nowhere to go (there is no FC), which is expected.

First identify which joystick axis is your 3-position switch — flip it through
all three positions and note the axis that jumps between three values:

```bash
python -m kenet.joystick
```

Then run Kenet in joystick mode. If there is no webcam, use a video file:

```bash
python kenet.py --joystick --camera test-2.mp4 --no-gcs
```

The TBS Joystick defaults are already wired in (mode switch on axis 6, arm switch
on axis 4, autopilot mode switch on axis 5). Use `--joy-axis` and the other
`--joy-*` flags below if your transmitter differs, or `--joy-invert` if a switch
runs backwards. A preview window opens unless you pass `--headless`; if OpenCV
can't open a window it logs a warning and keeps running headless.

### 3. With Betaflight SITL (a real Betaflight on the PC)

Betaflight has a SITL ("software in the loop") build that runs as a normal Linux
program and speaks real MSP. This is the way to test the MSP protocol, MSP
Override, and attitude telemetry against an actual Betaflight without hardware.

Build and run it in a separate workspace:

```bash
git clone https://github.com/betaflight/betaflight ~/sitl-work/betaflight
cd ~/sitl-work/betaflight
make TARGET=SITL
./obj/main/betaflight_SITL.elf      # RC input on UDP :9004, MSP on TCP :5761
```

First prove the RC chain with just the transmitter. Check the mapping, then send
the pilot sticks and switches to SITL:

```bash
python tools/sitl_rc_bridge.py --dry-run      # print the mapped channels
python tools/sitl_rc_bridge.py --send         # send pilot RC to SITL UDP :9004 at 50 Hz
```

Confirm Betaflight received it without opening Configurator:

```bash
python tools/sitl_rc_probe.py   # sends RC, then reads MSP_RC back over TCP :5761
```

Then bring Kenet's tracking into the loop with the mixer. It reads the same pilot
RC, runs the tracker on a camera or video, and merges the two: in TRACKING with a
target it replaces pitch and yaw with Kenet's output and leaves roll, throttle,
and the AUX channels with the pilot; in IDLE/AI-ARMED, with no target, or if the
tracker can't load, everything passes through from the pilot. It sends one merged
packet to SITL and does not use MSP override.

```bash
# Dry-run: print pilot vs. merged channels without sending
python tools/kenet_sitl_mixer.py --device /dev/input/js0 --camera test-2.mp4

# Send the merged RC to SITL
python tools/kenet_sitl_mixer.py --device /dev/input/js0 --camera test-2.mp4 --send
```

Flip the Kenet state switch (CH6/AUX2 by default) to High and watch pitch and yaw
move in Configurator's Receiver tab while roll and throttle still follow your
sticks. Add `--preview` to see the tracker window.

To configure SITL in Betaflight Configurator, bridge its TCP port to a websocket
(`pip install websockify`, then `websockify 127.0.0.1:6761 127.0.0.1:5761`) and
connect to `ws://127.0.0.1:6761`. The full step-by-step is in `plan-sitl.md`.

A later option is to use Kenet's real MSP path against SITL instead of the mixer:
Kenet speaks serial MSP while SITL offers TCP, so you bridge them with socat
(`socat pty,raw,echo=0,link=/tmp/ttyBF tcp:127.0.0.1:5761`) and run
`python kenet.py --port /tmp/ttyBF`, or add native TCP support to `msp.py`.

---

## The 3-position AUX state machine

The camera and pipeline start when the program starts. The AUX switch only
controls whether the tracker and MSP override are active.

```
AUX Low              AUX Mid                 AUX High
   IDLE      -->    AI-ARMED      -->       TRACKING
pilot flies        pilot flies          Kenet drives pitch/yaw,
normally           normally             pilot keeps roll/throttle
```

| AUX position    | State        | What happens                              | Who controls the drone        |
|-----------------|--------------|-------------------------------------------|-------------------------------|
| Low (~1000)     | **IDLE**     | Monitors AUX, sends no RC                  | Pilot (real RX)               |
| Mid (~1500)     | **AI-ARMED** | Tracker init zone ready, sends no RC       | Pilot (real RX)               |
| High (~2000)    | **TRACKING** | Tracker + PID active, Kenet sends pitch/yaw| Kenet pitch/yaw, pilot the rest |

When the switch drops below High, Kenet stops sending RC and the real receiver
takes over immediately. This handoff does not rely on triggering failsafe, but a
normal Betaflight receiver/failsafe setup is still required.

---

## Betaflight setup

You need a real RC receiver on the FC for normal flying, plus MSP Override so
Kenet can take over specific channels while tracking.

1. **Ports tab** — enable MSP on the UART connected to your companion board. For
   desk testing over USB, MSP is on by default.

2. **Enable MSP Override (CLI)** — Kenet only drives pitch and yaw, so override
   only those two channels. With the channel order roll=0, pitch=1, throttle=2,
   yaw=3, the bitmask for pitch+yaw is `10`:

   ```
   feature MSP_OVERRIDE
   set msp_override_channels = 10
   save
   ```

   Do not use `15` — that would also freeze the pilot's roll and throttle at
   center during tracking.

3. **Modes tab** — assign MSP OVERRIDE to the High range of the same 3-position
   AUX switch you use for TRACKING, and set up ANGLE mode as appropriate for your
   airframe.

### Bench safety checklist

Before any powered test with a flight controller:

1. Remove the propellers.
2. Verify `get msp_override_channels` reports `10`.
3. In TRACKING, confirm pitch/yaw move from Kenet while roll/throttle still
   follow the transmitter.
4. Drop AUX below High and confirm MSP Override stops immediately.
5. Save the Betaflight `diff all` output for the test setup.

---

## Control scheme

- **Yaw** — horizontal pixel error turns the drone to face the target.
- **Pitch** — target bounding-box width vs. the desired width flies
  forward/backward to hold distance.
- **Roll and throttle** — not driven by Kenet with the recommended override mask;
  the pilot keeps them.
- A rate limiter smooths channel changes so attitude can't jump suddenly.

PID tuning, the usual way: set `ki=0, kd=0` and raise `kp` until it follows but
oscillates, add `kd` to damp the oscillation, then a little `ki` to remove
steady-state offset.

---

## CLI reference

### kenet.py

```
--port            FC serial port (default: /dev/ttyAMA0; USB FC is often /dev/ttyACM0)
--camera          Camera index (0,1,...) or video file path (default: 0)
--tracker         Tracker type: CSRT or KCF (default: CSRT)
--aux-ch          3-position AUX channel, 0-indexed (default: 7 / AUX4)
--track-size      Tracker init bbox size in pixels (default: 100)
--loop-hz         Control loop frequency (default: 30)
--headless        Disable the GUI preview window
--gcs-host        GCS IP address (default: 192.168.1.100)
--gcs-port        GCS telemetry port (default: 14550)
--no-gcs          Disable GCS telemetry

Joystick mode (test without a flight controller):
--joystick            Read AUX from a USB joystick instead of MSP/FC
--joy-device          Joystick device (default: /dev/input/js0)
--joy-axis            Axis of the 3-position mode switch -> Kenet state (default: 6)
--joy-invert          Invert the mode axis
--joy-arm-axis        Axis of the 2-position arm switch (default: 4; -1 to disable)
--joy-arm-ch          RC channel for the arm switch (default: 4 / CH5)
--joy-arm-invert      Invert the arm axis
--joy-ap-mode-axis    Axis of the autopilot 3-position mode switch (default: 5; -1 to disable)
--joy-ap-mode-ch      RC channel for the autopilot mode switch (default: 6 / CH7)
--joy-ap-mode-invert  Invert the autopilot mode axis
```

### quick_gcs.py

```
--port            UDP listen port (default: 14550)
```

### GCS commands

Send a JSON object over UDP to the pipeline's command port (default 14551):

- `stop` — stop the pipeline.
- `set_target_width` with `{"width": 150}` — change the follow distance.
- `ping` — heartbeat.

Example:

```bash
python -c "import socket,json; s=socket.socket(socket.AF_INET,socket.SOCK_DGRAM); \
  s.sendto(json.dumps({'command':'stop'}).encode(),('127.0.0.1',14551))"
```

---

## tools/

Helpers for the SITL path and for mapping your transmitter. The bridge, probe,
and monitors are standard-library only; the mixer also imports Kenet (camera,
tracker, controller), so run it from the project venv.

- `kenet_sitl_mixer.py` — the main Kenet-into-SITL test tool. Reads pilot RC,
  runs the tracker on a camera/video, and sends one merged RC packet to SITL
  (pitch/yaw from Kenet in TRACKING, pilot everywhere else). No MSP override.
- `sitl_rc_bridge.py` — reads the joystick and sends Betaflight SITL RC packets
  over UDP `:9004` (`--send`), or just prints the mapped channels (`--dry-run`).
- `sitl_rc_probe.py` — sends RC to SITL, then reads `MSP_RC` back over TCP
  `:5761` to confirm Betaflight received it. Close Configurator first.
- `rc_monitor.py` — desktop GUI (Tkinter, with a text fallback) showing the
  mapped RC channels and raw axes/buttons.
- `state_monitor.py` — a local web dashboard at `http://127.0.0.1:8765` showing
  the Kenet state switch, the autopilot arm and mode switches, and the channels.

---

## Deployment on a drone (Pi / Jetson)

Wire a companion-board UART to a spare FC UART (TX→RX, RX→TX, GND→GND), then run
the pipeline on the board and the viewer on a laptop on the same network:

```bash
# On the Pi/Jetson
python kenet.py --port /dev/ttyAMA0 --gcs-host 192.168.1.100

# On the laptop
python quick_gcs.py --port 14550
```

The board and the laptop must share a network — Wi-Fi, a direct Ethernet cable
with static IPs, or a USB Ethernet adapter on the Pi.

---

## Key classes

- `CameraCapture` (`camera.py`) — threaded frame grabber.
- `ObjectTracker` (`tracker.py`) — CSRT / KCF tracking.
- `PIDController` and `FlightController` (`controller.py`) — generic PID, and the
  converter from tracking results to RC channels.
- `MSPConnection` (`msp.py`) — serial MSP protocol to Betaflight.
- `JoystickMSP` (`joystick.py`) — drop-in MSP replacement sourced from a USB
  joystick, for FC-less desk testing.
- `GCSLink` (`gcs.py`) — UDP telemetry and command receiver.
- `TrackingPipeline` (`pipeline.py`) — the state machine and control loop.
