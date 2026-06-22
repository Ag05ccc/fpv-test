#!/usr/bin/env python3
"""
Probe Betaflight SITL RC input without using Configurator.

It sends the same UDP rc_packet as sitl_rc_bridge.py, then asks Betaflight over
MSP/TCP for MSP_RC and prints the channel values seen by the FC.

Close Betaflight Configurator before running this probe; the SITL TCP serial
port accepts one MSP client at a time.
"""

import argparse
import socket
import struct
import sys
import time
from pathlib import Path


SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from sitl_rc_bridge import CHANNEL_MAP, LinuxJoystick, make_channels, pack_rc_packet


MSP_API_VERSION = 1
MSP_RC = 105


def msp_checksum(code, payload=b""):
    checksum = len(payload) ^ int(code)
    for b in payload:
        checksum ^= b
    return checksum


def msp_encode(code, payload=b""):
    return b"$M<" + bytes([len(payload), code]) + payload + bytes([msp_checksum(code, payload)])


def read_msp_frame(sock):
    while True:
        b = sock.recv(1)
        if not b:
            raise RuntimeError("MSP connection closed")
        if b != b"$":
            continue
        if sock.recv(1) != b"M":
            continue
        direction = sock.recv(1)
        if direction not in (b">", b"!"):
            continue
        header = sock.recv(2)
        if len(header) != 2:
            raise RuntimeError("short MSP header")
        size, code = header[0], header[1]
        payload = b""
        while len(payload) < size + 1:
            chunk = sock.recv(size + 1 - len(payload))
            if not chunk:
                raise RuntimeError("short MSP payload")
            payload += chunk
        body = payload[:-1]
        checksum = payload[-1]
        expected = msp_checksum(code, body)
        if checksum != expected:
            raise RuntimeError("bad MSP checksum for code %d" % code)
        if direction == b"!":
            raise RuntimeError("MSP returned error for code %d" % code)
        return code, body


def msp_request(sock, code):
    sock.sendall(msp_encode(code))
    for _ in range(8):
        response_code, payload = read_msp_frame(sock)
        if response_code == code:
            return payload
    raise RuntimeError("MSP code %d response not received" % code)


def send_rc(args):
    joystick = LinuxJoystick(args.device)
    joystick.open()
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    deadline = time.monotonic() + args.send_seconds
    period = 1.0 / args.rate_hz
    next_send = time.monotonic()
    count = 0
    channels = [1500] * 16
    try:
        while time.monotonic() < deadline:
            joystick.poll(timeout=0.01)
            channels = make_channels(joystick, CHANNEL_MAP)
            now = time.monotonic()
            if now >= next_send:
                sock.sendto(pack_rc_packet(channels), (args.rc_host, args.rc_port))
                count += 1
                next_send = now + period
            time.sleep(0.002)
    finally:
        sock.close()
        joystick.close()
    return count, channels


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--device", default="/dev/input/js0")
    parser.add_argument("--rc-host", default="127.0.0.1")
    parser.add_argument("--rc-port", type=int, default=9004)
    parser.add_argument("--msp-host", default="127.0.0.1")
    parser.add_argument("--msp-port", type=int, default=5761)
    parser.add_argument("--rate-hz", type=float, default=50.0)
    parser.add_argument("--send-seconds", type=float, default=0.5)
    parser.add_argument("--timeout", type=float, default=2.0)
    args = parser.parse_args()
    if args.rate_hz <= 0:
        parser.error("--rate-hz must be positive")
    if args.send_seconds <= 0:
        parser.error("--send-seconds must be positive")
    return args


def main():
    args = parse_args()
    count, sent_channels = send_rc(args)
    print("sent %d UDP RC packets to %s:%d" % (count, args.rc_host, args.rc_port), flush=True)
    print("last sent rc_us:", ",".join(str(ch) for ch in sent_channels[:8]), flush=True)

    try:
        with socket.create_connection((args.msp_host, args.msp_port), timeout=args.timeout) as sock:
            sock.settimeout(args.timeout)
            api = msp_request(sock, MSP_API_VERSION)
            if len(api) >= 3:
                print("MSP API version: %d.%d.%d" % (api[0], api[1], api[2]))
            rc_payload = msp_request(sock, MSP_RC)
    except Exception as exc:
        print("MSP probe failed: %s" % exc, file=sys.stderr)
        print("Close Betaflight Configurator and retry this probe.", file=sys.stderr)
        return 1

    values = list(struct.unpack("<%dH" % (len(rc_payload) // 2), rc_payload))
    print("MSP_RC seen by Betaflight:", ",".join(str(ch) for ch in values[:8]))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
