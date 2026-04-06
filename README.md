# fpv-test

Visual target tracking + PID control + Betaflight MSP for FPV drones.

## Architecture

```
USB Camera -> CameraCapture (threaded) -> ObjectTracker (CSRT/KCF)
    -> FlightController (PID yaw + forward) -> MSP_SET_RAW_RC -> Betaflight FC (UART)
    -> GCSLink (UDP telemetry) -> Ground Control Station (Ethernet)
```

## Project Structure

```
modules/
  __init__.py       - Package exports
  camera.py         - Threaded OpenCV video capture
  tracker.py        - CSRT / KCF object tracking
  controller.py     - PID controllers + rate-limited flight control
  msp.py            - MSPv1 serial protocol for Betaflight
  gcs.py            - UDP telemetry to ground control station
  pipeline.py       - Two-stage state machine + control loop
example.py          - CLI entry point
quick_gcs.py        - Minimal telemetry viewer for testing
```

## Install

```bash
pip install -r requirements.txt
```

---

## Testing on a PC (Desk Testing)

You can test the full system on your PC using the FC's USB-C port.
Your PC acts as both the companion board (running the pipeline) and the GCS
(receiving telemetry). No soldering required.

### What you need
- FC connected to PC via USB-C
- USB webcam (or laptop camera)
- RC transmitter + receiver bound to the FC (for AUX switch testing)
- Close Betaflight Configurator first (it locks the USB port)

### Find the FC serial port
```bash
# Linux
ls /dev/ttyACM*       # usually /dev/ttyACM0

# Windows
# Check Device Manager -> Ports -> COMx
```

### Run pipeline + GCS on the same PC

```bash
# Terminal 1 — run the tracking pipeline
python example.py --port /dev/ttyACM0 --gcs-host 127.0.0.1

# Terminal 2 — view live telemetry
python quick_gcs.py
```

### Send commands to the pipeline from a third terminal

```bash
# Stop the pipeline remotely
python -c "import socket,json; s=socket.socket(socket.AF_INET,socket.SOCK_DGRAM); s.sendto(json.dumps({'command':'stop'}).encode(),('127.0.0.1',14551))"

# Change follow distance
python -c "import socket,json; s=socket.socket(socket.AF_INET,socket.SOCK_DGRAM); s.sendto(json.dumps({'command':'set_target_width','params':{'width':150}}).encode(),('127.0.0.1',14551))"
```

### What to verify during desk testing
1. MSP connection works (no serial errors in Terminal 1)
2. Camera feed is captured (no "Cannot open camera" error)
3. AUX switches are read correctly (flip switches, watch state transitions in logs)
4. Telemetry arrives in Terminal 2
5. Tracker initializes when AUX2 is flipped (you should see "ARMED -> TRACKING" in logs)

---

## Deployment on Drone (Pi / Jetson)

### Hardware wiring

| Companion Board | FC (Betaflight)   |
|-----------------|-------------------|
| UART TX (GPIO14)| Spare UART RX     |
| UART RX (GPIO15)| Spare UART TX     |
| GND             | GND               |

### Run on the companion board

```bash
# SSH into your Pi/Jetson, then:
python example.py --port /dev/ttyAMA0 --gcs-host 192.168.1.100
```

### Run GCS on your laptop (same network)

```bash
python quick_gcs.py --port 14550
```

### Network setup
The companion board and GCS laptop must be on the same network.
Options:
- Wi-Fi (both on the same router)
- Direct Ethernet cable (set static IPs, e.g. Pi=192.168.1.50, Laptop=192.168.1.100)
- USB Ethernet adapter on the Pi

---

## Two-Stage AUX Arming

Tracking uses two AUX switches on your RC transmitter:

```
AUX1 off             AUX1 on              AUX2 on
┌──────────┐     ┌──────────────┐     ┌─────────────────┐
│   IDLE   │ --> │    ARMED     │ --> │   TRACKING      │
│          │ <-- │  camera on   │ <-- │  sending RC     │
└──────────┘     │  ready       │     │  PID active     │
 pilot flies     └──────────────┘     └─────────────────┘
 normally         pilot flies          Pi overrides
                  normally             roll/pitch/yaw
```

| Switch | State | What happens | Who controls drone |
|--------|-------|---|---|
| Both off | **IDLE** | Pi only reads AUX channels | Pilot (real RX) |
| AUX1 on | **ARMED** | Camera starts, systems ready | Pilot (real RX) |
| AUX1+AUX2 on | **TRACKING** | Tracker + PID active, Pi sends RC | Pi (MSP Override) |
| AUX2 off | back to **ARMED** | Pi stops sending RC | Pilot (real RX) |
| AUX1 off | back to **IDLE** | Camera stops | Pilot (real RX) |

When any AUX switch goes low, Pi **stops sending RC** and the real receiver
takes over immediately. No failsafe.

---

## Betaflight Setup

You need a **real RC receiver** on the FC for normal flying, plus **MSP Override**
so the Pi can take over specific channels when tracking.

### 1. Ports tab
Enable MSP on the UART connected to your companion board (Pi / Jetson).
For desk testing via USB, MSP is enabled on USB by default.

### 2. Enable MSP Override (Betaflight CLI)
```
# Enable the MSP override feature
feature MSP_OVERRIDE

# Override channels 0-3 (roll, pitch, throttle, yaw) — bitmask 15
set msp_override_channels = 15

save
```

### 3. Assign AUX switches (Modes tab)
- Assign **ANGLE** mode to an AUX switch (for stable autonomous flight)
- Assign **MSP OVERRIDE** mode to the same AUX2 switch you use for tracking
  (so MSP Override is only active when tracking is active)

### 4. How it works
- Real receiver handles all normal flying (angle, acro, horizon, etc.)
- When AUX2 is flipped: Betaflight enables MSP Override on the configured channels
- Pi sends MSP_SET_RAW_RC which overrides roll/pitch/yaw/throttle
- When AUX2 goes low: MSP Override disabled, real receiver values used instantly
- Real receiver is always connected, so **no failsafe**

---

## Control Scheme

- **Yaw**: horizontal pixel error -> turn to face target
- **Pitch**: target bbox width vs desired width -> fly forward/backward
- **Roll**: clamped to +/-20 degrees (Betaflight angle mode)
- **Rate limiter**: smooth channel transitions, prevents sudden attitude changes

---

## CLI Reference

### example.py

```
--port          FC serial port (default: /dev/ttyAMA0)
--camera        Camera index (default: 0)
--tracker       Tracker type: CSRT or KCF (default: CSRT)
--arm-ch        AUX channel to arm pipeline, 0-indexed (default: 4)
--track-ch      AUX channel to start tracking, 0-indexed (default: 5)
--track-size    Fixed bbox size in pixels (default: 100)
--loop-hz       Control loop frequency (default: 30)
--show-preview  Show GUI preview window
--gcs-host      GCS IP address (default: 192.168.1.100)
--gcs-port      GCS telemetry port (default: 14550)
--no-gcs        Disable GCS telemetry
```

### quick_gcs.py

```
--port          UDP listen port (default: 14550)
```

### GCS commands (JSON over UDP to port 14551)

| Command | Params | Description |
|---------|--------|-------------|
| `stop` | — | Stop the pipeline |
| `set_target_width` | `{"width": 150}` | Change follow distance |
| `ping` | — | Heartbeat |

---

## PID Tuning

Start with low gains and increase gradually:
1. Set `ki=0, kd=0`, increase `kp` until the drone follows the target but oscillates.
2. Add `kd` to dampen oscillation.
3. Add small `ki` to eliminate steady-state offset.

## Key Classes

| Class               | Module          | Role                                      |
|---------------------|-----------------|-------------------------------------------|
| `CameraCapture`     | `camera.py`     | Threaded frame grabber                    |
| `ObjectTracker`     | `tracker.py`    | CSRT / KCF tracking                       |
| `PIDController`     | `controller.py` | Generic PID with anti-windup              |
| `FlightController`  | `controller.py` | Converts tracking to RC channels          |
| `MSPConnection`     | `msp.py`        | Serial MSP protocol to Betaflight         |
| `GCSLink`           | `gcs.py`        | UDP telemetry + command receiver          |
| `TrackingPipeline`  | `pipeline.py`   | Two-stage state machine + control loop    |
