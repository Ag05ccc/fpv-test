#!/usr/bin/env python3
"""
Example: run the tracking pipeline with two-stage AUX arming.

  AUX1 (--arm-ch)   = arm pipeline (start camera, systems ready)
  AUX2 (--track-ch) = start tracking (send RC override)

Usage:
    # Default: AUX1=ch4, AUX2=ch5
    python example.py --port /dev/ttyAMA0

    # Custom AUX channel and bbox size
    python example.py --aux-ch 4 --track-size 120

    # With GCS telemetry
    python example.py --gcs-host 192.168.1.100

    # Disable GCS
    python example.py --no-gcs

    # Test with a video file instead of a camera
    python example.py --camera test_video.mp4 --no-gcs
"""

import argparse
from modules import PipelineConfig, TrackingPipeline, PIDGains


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
    args = ap.parse_args()

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
    pipeline.start()

    print("Pipeline running. AUX ch%d (3-position):" % args.aux_ch)
    print("  Low  = IDLE      (pilot control)")
    print("  Mid  = AI-ARMED  (camera ready)")
    print("  High = TRACKING  (Pi override)")
    print("Press Ctrl+C to stop.")
    pipeline.run()


if __name__ == "__main__":
    main()
