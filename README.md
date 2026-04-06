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
  pipeline.py       - Orchestrates all modules in a control loop
example.py          - CLI entry point
```

## Control Scheme

- **Yaw**: horizontal pixel error -> turn to face target
- **Pitch**: target bbox width vs desired width -> fly forward/backward
- **Roll**: clamped to +/-20 degrees (Betaflight angle mode)
- **Rate limiter**: smooth channel transitions, prevents sudden attitude changes

## Tracker Initialization

Tracking is started via an **RC AUX switch** read from the FC over MSP:

1. Pilot flips AUX switch (e.g. AUX1, channel index 4)
2. Pipeline reads the channel value via MSP_RC
3. When above threshold (default 1500), tracker initializes with a fixed-size
   bbox centered in the frame
4. When switch goes low, tracking stops and channels return to neutral

Alternatively, pass `--bbox X Y W H` to skip RC trigger and start immediately.

## Betaflight Setup

1. Enable MSP on the UART connected to your companion board (Configurator -> Ports tab).
2. Enable angle mode via an AUX switch or always-on:
   ```
   aux 0 1 0 900 2100 0 0
   save
   ```
3. If using MSP as the sole RC source (no physical receiver):
   ```
   set serialrx_provider = MSP
   save
   ```

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
# RC trigger mode (default) — flip AUX switch to start tracking
python example.py --port /dev/ttyAMA0

# Pre-set bbox (skip RC trigger)
python example.py --bbox 270 190 100 100

# Custom AUX channel and bbox size
python example.py --track-ch 5 --track-size 120

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
| `TrackingPipeline`  | `pipeline.py`   | Orchestrates everything in a control loop |
