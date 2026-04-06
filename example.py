#!/usr/bin/env python3
"""
Example: run the tracking pipeline.

Usage:
    # RC trigger mode (default) — flip AUX switch to start tracking:
    python example.py

    # Pre-set bbox (skip RC trigger, useful for testing):
    python example.py --bbox 270 190 100 100

    # With GCS telemetry:
    python example.py --gcs-host 192.168.1.100
"""

import argparse
from modules import PipelineConfig, TrackingPipeline, PIDGains


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyAMA0", help="FC serial port")
    ap.add_argument("--camera", default=0, type=int, help="Camera index")
    ap.add_argument("--tracker", default="CSRT", choices=["CSRT", "KCF"])
    ap.add_argument("--bbox", nargs=4, type=int, metavar=("X", "Y", "W", "H"),
                    help="Pre-set bounding box (disables RC trigger)")
    ap.add_argument("--track-ch", default=4, type=int,
                    help="AUX channel index for tracking trigger (0-indexed, default=4)")
    ap.add_argument("--track-size", default=100, type=int,
                    help="Fixed bbox size in pixels for RC trigger init")
    ap.add_argument("--loop-hz", default=30, type=int)
    ap.add_argument("--show-preview", action="store_true", help="Show GUI preview")
    ap.add_argument("--gcs-host", default="192.168.1.100", help="GCS IP address")
    ap.add_argument("--gcs-port", default=14550, type=int, help="GCS telemetry port")
    ap.add_argument("--no-gcs", action="store_true", help="Disable GCS link")
    args = ap.parse_args()

    cfg = PipelineConfig(
        camera_source=args.camera,
        serial_port=args.port,
        tracker_type=args.tracker,
        loop_hz=args.loop_hz,
        show_preview=args.show_preview,
        track_aux_ch=args.track_ch,
        track_bbox_size=args.track_size,
        gcs_host=args.gcs_host,
        gcs_port=args.gcs_port,
        gcs_enabled=not args.no_gcs,
        yaw_pid=PIDGains(kp=0.8, ki=0.05, kd=0.15, output_min=-300, output_max=300),
        forward_pid=PIDGains(kp=0.4, ki=0.02, kd=0.1, output_min=-250, output_max=250),
    )

    pipeline = TrackingPipeline(cfg)
    pipeline.start()

    if args.bbox:
        pipeline.init_target(bbox=tuple(args.bbox))
        print("Tracking started with preset bbox. Press Ctrl+C to stop.")
    else:
        print("Waiting for RC trigger on channel %d. Press Ctrl+C to stop." % args.track_ch)

    pipeline.run()


if __name__ == "__main__":
    main()
