#!/usr/bin/env python3
"""
Kenet — Visual target tracking + PID control for FPV drones.

Usage:
    # Default: AUX ch7 (3-position)
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
from kenet import PipelineConfig, TrackingPipeline, PIDGains


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyAMA0", help="FC serial port")
    ap.add_argument("--camera", default="0",
                    help="Camera index (0,1,...) or video file path")
    ap.add_argument("--tracker", default="CSRT", choices=["CSRT", "KCF"])
    ap.add_argument("--aux-ch", default=7, type=int,
                    help="3-position AUX channel (0-indexed, default=7 / AUX4)")
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

    cfg = PipelineConfig(
        camera_source=cam,
        serial_port=args.port,
        tracker_type=args.tracker,
        loop_hz=args.loop_hz,
        show_preview=not args.headless,
        aux_ch=args.aux_ch,
        track_bbox_size=args.track_size,
        gcs_host=args.gcs_host,
        gcs_port=args.gcs_port,
        gcs_enabled=not args.no_gcs,
        yaw_pid=PIDGains(kp=0.8, ki=0.05, kd=0.15, output_min=-300, output_max=300),
        forward_pid=PIDGains(kp=0.4, ki=0.02, kd=0.1, output_min=-250, output_max=250),
    )

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
