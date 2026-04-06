#!/usr/bin/env python3
"""
Example: run the tracking pipeline with default settings.

Usage:
    # With display (for ROI selection):
    python example.py

    # Headless with a known bounding box:
    python example.py --headless --bbox 200 150 80 80

    # Color tracking (red object):
    python example.py --tracker COLOR

    # With GCS telemetry:
    python example.py --gcs-host 192.168.1.100
"""

import argparse
from modules import PipelineConfig, TrackingPipeline, PIDGains


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyAMA0", help="FC serial port")
    ap.add_argument("--camera", default=0, type=int, help="Camera index")
    ap.add_argument("--tracker", default="CSRT", choices=["CSRT", "KCF", "COLOR"])
    ap.add_argument("--headless", action="store_true", help="No GUI preview")
    ap.add_argument("--bbox", nargs=4, type=int, metavar=("X", "Y", "W", "H"),
                    help="Initial bounding box (skip interactive selection)")
    ap.add_argument("--loop-hz", default=30, type=int)
    ap.add_argument("--gcs-host", default="192.168.1.100", help="GCS IP address")
    ap.add_argument("--gcs-port", default=14550, type=int, help="GCS telemetry port")
    ap.add_argument("--no-gcs", action="store_true", help="Disable GCS link")
    args = ap.parse_args()

    cfg = PipelineConfig(
        camera_source=args.camera,
        serial_port=args.port,
        tracker_type=args.tracker,
        loop_hz=args.loop_hz,
        show_preview=not args.headless,
        gcs_host=args.gcs_host,
        gcs_port=args.gcs_port,
        gcs_enabled=not args.no_gcs,

        # Tune these for your setup
        yaw_pid=PIDGains(kp=0.8, ki=0.05, kd=0.15, output_min=-300, output_max=300),
        forward_pid=PIDGains(kp=0.4, ki=0.02, kd=0.1, output_min=-250, output_max=250),
    )

    pipeline = TrackingPipeline(cfg)
    pipeline.start()

    bbox = tuple(args.bbox) if args.bbox else None
    pipeline.init_target(bbox=bbox)

    print("Tracking started. Press Ctrl+C to stop.")
    pipeline.run()


if __name__ == "__main__":
    main()
