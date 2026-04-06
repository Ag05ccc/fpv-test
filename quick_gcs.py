#!/usr/bin/env python3
"""
Minimal GCS telemetry listener — prints drone state in real time.

Usage:
    python quick_gcs.py
    python quick_gcs.py --port 14550
"""

import socket
import json
import argparse


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default=14550, type=int, help="UDP listen port")
    args = ap.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", args.port))
    print("GCS listening on :%d ..." % args.port)

    try:
        while True:
            data, addr = sock.recvfrom(4096)
            pkt = json.loads(data)
            print(
                "found=%(target_found)s  "
                "yaw_err=%(yaw_error).1f  "
                "fwd_err=%(forward_error).1f  "
                "ch=%(channels)s  "
                "att=[%(roll).1f, %(pitch).1f, %(yaw).1f]  "
                "fps=%(loop_fps).1f" % pkt
            )
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        sock.close()


if __name__ == "__main__":
    main()
