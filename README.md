# drone_tracker

Visual target tracking → PID control → Betaflight MSP for FPV drones.

## Architecture

```
USB Camera → CameraCapture (threaded) → ObjectTracker (CSRT/KCF/Color)
     → PID (yaw + pitch) → MSP_SET_RAW_RC → Betaflight FC (UART)
```

## Hardware Wiring

| Companion Board | FC (Betaflight)   |
|-----------------|-------------------|
| UART TX         | Spare UART RX     |
| UART RX         | Spare UART TX     |
| GND             | GND               |

Enable `MSP` on the corresponding UART in Betaflight Configurator → Ports tab.

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
```

## PID Tuning

Start with low gains and increase gradually:
1. Set `ki=0, kd=0`, increase `kp` until the drone follows the target but oscillates.
2. Add `kd` to dampen oscillation.
3. Add small `ki` to eliminate steady-state offset.

## Key Classes

| Class              | Role                                      |
|--------------------|-------------------------------------------|
| `CameraCapture`    | Threaded frame grabber                    |
| `ObjectTracker`    | CSRT / KCF / HSV color tracking           |
| `PIDController`    | Generic PID with anti-windup              |
| `MSPConnection`    | Serial MSP protocol to Betaflight         |
| `TrackingPipeline` | Orchestrates everything in a control loop |
