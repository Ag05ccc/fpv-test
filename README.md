# fpv-test

Visual target tracking + PID control + Betaflight MSP for FPV drones.

## Architecture

```
USB Camera -> CameraCapture (threaded) -> ObjectTracker (CSRT/KCF)
    -> FlightController (PID yaw + forward) -> MSP_SET_RAW_RC -> Betaflight FC (UART)
    -> GCSLink (UDP telemetry) -> Ground Control Station (Ethernet)
```

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

## Betaflight Setup

You need a **real RC receiver** on the FC for normal flying, plus **MSP Override**
so the Pi can take over specific channels when tracking.

### 1. Ports tab
Enable MSP on the UART connected to your companion board (Pi / Jetson).

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
```

## Control Scheme

- **Yaw**: horizontal pixel error -> turn to face target
- **Pitch**: target bbox width vs desired width -> fly forward/backward
- **Roll**: clamped to +/-20 degrees (Betaflight angle mode)
- **Rate limiter**: smooth channel transitions, prevents sudden attitude changes

## Hardware Wiring

| Companion Board | FC (Betaflight)   |
|-----------------|-------------------|
| UART TX         | Spare UART RX     |
| UART RX         | Spare UART TX     |
| GND             | GND               |

## Install

```bash
pip install -r requirements.txt
```

## Quick Start

```bash
# Default: AUX1=ch4 (arm), AUX2=ch5 (track)
python example.py --port /dev/ttyAMA0

# Custom AUX channels and bbox size
python example.py --arm-ch 4 --track-ch 5 --track-size 120

# With GCS telemetry
python example.py --gcs-host 192.168.1.100

# Disable GCS
python example.py --no-gcs
```

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
