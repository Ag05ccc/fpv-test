#!/usr/bin/env python3
"""
Kenet — Visual target tracking + PID control for FPV drones.

Usage:
    # Default: AUX ch5 / CH6 / AUX2 (3-position)
    python kenet.py --port /dev/ttyAMA0

    # Custom AUX channel and bbox size
    python kenet.py --aux-ch 4 --track-size 120

    # With GCS telemetry
    python kenet.py --gcs-host 192.168.1.100

    # Disable GCS
    python kenet.py --no-gcs

    # Test with a video file instead of a camera
    python kenet.py --camera test_video.mp4 --no-gcs
"""

import argparse
from kenet import PipelineConfig, TrackingPipeline
from kenet.state_machine import DEFAULT_KENET_AUX_CH


def _apply_pid_override(gains, kp, ki, kd):
    if kp is not None:
        gains.kp = kp
    if ki is not None:
        gains.ki = ki
    if kd is not None:
        gains.kd = kd


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyAMA0", help="FC serial port")
    ap.add_argument("--msp-tcp", default=None,
                    help="Betaflight SITL MSP TCP endpoint, e.g. 127.0.0.1:5761")
    ap.add_argument("--camera", default="0",
                    help="Camera index (0,1,...) or video file path")
    ap.add_argument("--tracker", default="CSRT", choices=["CSRT", "KCF"])
    # PID tuning overrides. Defaults live in PipelineConfig (single source);
    # a flag left unset keeps that default so tune runs are config-tracked.
    ap.add_argument("--yaw-kp", type=float, default=None)
    ap.add_argument("--yaw-ki", type=float, default=None)
    ap.add_argument("--yaw-kd", type=float, default=None)
    ap.add_argument("--forward-kp", type=float, default=None)
    ap.add_argument("--forward-ki", type=float, default=None)
    ap.add_argument("--forward-kd", type=float, default=None)
    ap.add_argument("--aux-ch", default=DEFAULT_KENET_AUX_CH, type=int,
                    help="3-position AUX channel (0-indexed, default=5 / CH6 / AUX2)")
    ap.add_argument("--track-size", default=100, type=int,
                    help="Fixed bbox size in pixels for tracker init")
    ap.add_argument("--loop-hz", default=30, type=int)
    ap.add_argument("--headless", action="store_true", help="Disable GUI preview")
    ap.add_argument("--gcs-host", default="192.168.1.100", help="GCS IP address")
    ap.add_argument("--gcs-port", default=14550, type=int, help="GCS telemetry port")
    ap.add_argument("--no-gcs", action="store_true", help="Disable GCS link")
    ap.add_argument("--joystick", action="store_true",
                    help="Drive AUX from a USB joystick (TX in game mode), no FC/MSP")
    ap.add_argument("--joy-device", default="/dev/input/js0",
                    help="Joystick device (default: /dev/input/js0)")
    ap.add_argument("--joy-axis", default=6, type=int,
                    help="Axis index of the 3-position mode AUX switch "
                         "(default: 6 for TBS Joystick; find it with: "
                         "python -m kenet.joystick)")
    ap.add_argument("--joy-invert", action="store_true",
                    help="Invert the 3-position mode AUX axis direction")
    ap.add_argument("--joy-arm-axis", default=4, type=int,
                    help="Axis index of the 2-position arm AUX switch "
                         "(default: 4 for TBS Joystick; use -1 to disable)")
    ap.add_argument("--joy-arm-ch", default=4, type=int,
                    help="0-indexed RC channel for the 2-position arm switch "
                         "(default: 4 / CH5 / AUX1)")
    ap.add_argument("--joy-arm-invert", action="store_true",
                    help="Invert the 2-position arm AUX axis direction")
    ap.add_argument("--joy-ap-mode-axis", default=5, type=int,
                    help="Axis index of the autopilot 3-position mode switch "
                         "(default: 5 for TBS Joystick; use -1 to disable)")
    ap.add_argument("--joy-ap-mode-ch", default=6, type=int,
                    help="0-indexed RC channel for the autopilot mode switch "
                         "(default: 6 / CH7 / AUX3)")
    ap.add_argument("--joy-ap-mode-invert", action="store_true",
                    help="Invert the autopilot mode axis direction")
    args = ap.parse_args()
    if args.joy_axis < 0:
        ap.error("--joy-axis must be >= 0")
    if args.joy_arm_axis < -1:
        ap.error("--joy-arm-axis must be >= -1")
    if args.joy_arm_ch < 0:
        ap.error("--joy-arm-ch must be >= 0")
    if args.joy_ap_mode_axis < -1:
        ap.error("--joy-ap-mode-axis must be >= -1")
    if args.joy_ap_mode_ch < 0:
        ap.error("--joy-ap-mode-ch must be >= 0")

    # Use int for camera index, string for video file
    cam = int(args.camera) if args.camera.isdigit() else args.camera
    msp_port = args.port
    if args.msp_tcp:
        msp_port = args.msp_tcp if args.msp_tcp.startswith("tcp://") else "tcp://%s" % args.msp_tcp

    cfg = PipelineConfig(
        camera_source=cam,
        serial_port=msp_port,
        tracker_type=args.tracker,
        loop_hz=args.loop_hz,
        show_preview=not args.headless,
        aux_ch=args.aux_ch,
        track_bbox_size=args.track_size,
        gcs_host=args.gcs_host,
        gcs_port=args.gcs_port,
        gcs_enabled=not args.no_gcs,
    )
    # PipelineConfig holds the default gains; apply CLI overrides in place so
    # there is a single source of truth for the tune.
    _apply_pid_override(cfg.yaw_pid, args.yaw_kp, args.yaw_ki, args.yaw_kd)
    _apply_pid_override(cfg.forward_pid, args.forward_kp, args.forward_ki,
                        args.forward_kd)

    pipeline = TrackingPipeline(cfg)

    # No FC? Source the AUX switch from a USB joystick instead of MSP.
    if args.joystick:
        from kenet.joystick import JoystickMSP
        aux_maps = [
            {
                "channel": cfg.aux_ch,
                "axis": args.joy_axis,
                "mode": "three",
                "invert": args.joy_invert,
            },
        ]
        if args.joy_arm_axis >= 0:
            aux_maps.insert(0, {
                "channel": args.joy_arm_ch,
                "axis": args.joy_arm_axis,
                "mode": "two",
                "invert": args.joy_arm_invert,
            })
        if args.joy_ap_mode_axis >= 0:
            aux_maps.append({
                "channel": args.joy_ap_mode_ch,
                "axis": args.joy_ap_mode_axis,
                "mode": "three",
                "invert": args.joy_ap_mode_invert,
            })
        pipeline.msp = JoystickMSP(
            aux_ch=cfg.aux_ch, num_channels=cfg.num_channels,
            device=args.joy_device, aux_axis=args.joy_axis,
            invert=args.joy_invert, aux_maps=aux_maps)
        print("Joystick AUX mode: %s (no FC; MSP override disabled)" %
              args.joy_device)
        print("  mode: axis=%d -> index %d / CH%d (%s)" %
              (args.joy_axis, cfg.aux_ch, cfg.aux_ch + 1,
               "inverted" if args.joy_invert else "normal"))
        if args.joy_arm_axis >= 0:
            print("  arm : axis=%d -> index %d / CH%d (%s)" %
                  (args.joy_arm_axis, args.joy_arm_ch, args.joy_arm_ch + 1,
                   "inverted" if args.joy_arm_invert else "normal"))
        if args.joy_ap_mode_axis >= 0:
            print("  ap mode: axis=%d -> index %d / CH%d (%s)" %
                  (args.joy_ap_mode_axis, args.joy_ap_mode_ch,
                   args.joy_ap_mode_ch + 1,
                   "inverted" if args.joy_ap_mode_invert else "normal"))

    pipeline.start()

    print("Pipeline running. AUX ch%d (3-position):" % args.aux_ch)
    print("  Low  = IDLE      (pilot control)")
    print("  Mid  = AI-ARMED  (camera ready)")
    print("  High = TRACKING  (Pi override)")
    print("Press Ctrl+C to stop.")
    pipeline.run()


if __name__ == "__main__":
    main()
