# fpv-test

Visual target tracking + PID control + Betaflight MSP for FPV drones.

## Architecture

```
USB Camera -> CameraCapture (threaded) -> ObjectTracker (CSRT/KCF/Color)
    -> FlightController (PID yaw + forward) -> MSP_SET_RAW_RC -> Betaflight FC (UART)
    -> GCSLink (UDP telemetry) -> Ground Control Station (Ethernet)
```

## Project Structure

```
modules/
  __init__.py       - Package exports
  camera.py         - Threaded OpenCV video capture
  tracker.py        - CSRT / KCF / HSV color object tracking
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

## Betaflight Setup

1. Set receiver to MSP in Betaflight CLI:
   ```
   set serialrx_provider = MSP
   save
   ```
2. Enable angle mode (always on):
   ```
   aux 0 1 0 900 2100 0 0
   save
   ```
3. Enable MSP on the UART connected to your companion board (Configurator -> Ports tab).

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
# Interactive ROI selection (needs display)
python example.py --port /dev/ttyAMA0

# Headless with predefined bbox
python example.py --headless --bbox 200 150 80 80

# Color tracking (red object)
python example.py --tracker COLOR --port /dev/ttyS0

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
| `ObjectTracker`     | `tracker.py`    | CSRT / KCF / HSV color tracking           |
| `PIDController`     | `controller.py` | Generic PID with anti-windup              |
| `FlightController`  | `controller.py` | Converts tracking to RC channels          |
| `MSPConnection`     | `msp.py`        | Serial MSP protocol to Betaflight         |
| `GCSLink`           | `gcs.py`        | UDP telemetry + command receiver          |
| `TrackingPipeline`  | `pipeline.py`   | Orchestrates everything in a control loop |
