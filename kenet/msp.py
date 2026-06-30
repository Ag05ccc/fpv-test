"""
msp - Multiwii Serial Protocol (MSPv1) communication with Betaflight FC.
"""
import struct
import time
import threading
import logging
import socket
from enum import IntEnum
from urllib.parse import urlparse

import serial

logger = logging.getLogger(__name__)


class MSPCode(IntEnum):
    MSP_API_VERSION = 1
    MSP_SET_RAW_RC = 200
    MSP_RC = 105
    MSP_STATUS = 101
    MSP_ATTITUDE = 108


class MSPError(RuntimeError):
    """Base class for MSP protocol errors."""


class MSPChecksumError(MSPError):
    """Raised when an MSP response checksum does not match."""


class MSPResponseError(MSPError):
    """Raised when the FC returns an MSP error frame."""


class TCPSocketSerial:
    """Small serial-like wrapper around Betaflight SITL's MSP TCP socket."""

    def __init__(self, host, port, timeout=0.1):
        self.host = host
        self.port = int(port)
        self.timeout = timeout
        self._sock = None
        self.is_open = False

    def open(self):
        self._sock = socket.create_connection((self.host, self.port), timeout=self.timeout)
        self._sock.settimeout(self.timeout)
        self.is_open = True

    def close(self):
        if self._sock is not None:
            self._sock.close()
        self._sock = None
        self.is_open = False

    def write(self, data):
        if not self._sock:
            return 0
        self._sock.sendall(data)
        return len(data)

    def read(self, size):
        if not self._sock:
            return b""
        chunks = bytearray()
        while len(chunks) < size:
            try:
                chunk = self._sock.recv(size - len(chunks))
            except socket.timeout:
                break
            if not chunk:
                break
            chunks.extend(chunk)
        return bytes(chunks)


def parse_tcp_endpoint(value):
    """Return (host, port) for tcp://host:port or host:port strings."""
    text = str(value)
    if text.startswith("tcp://"):
        parsed = urlparse(text)
        if not parsed.hostname or parsed.port is None:
            raise ValueError("invalid TCP MSP endpoint: %s" % value)
        return parsed.hostname, parsed.port
    if ":" in text and not text.startswith("/"):
        host, port_text = text.rsplit(":", 1)
        if host and port_text.isdigit():
            return host, int(port_text)
    raise ValueError("not a TCP MSP endpoint: %s" % value)


def is_tcp_endpoint(value):
    try:
        parse_tcp_endpoint(value)
    except ValueError:
        return False
    return True


def msp_checksum(code: int, payload: bytes = b"") -> int:
    checksum = len(payload) ^ int(code)
    for b in payload:
        checksum ^= b
    return checksum


def msp_encode(code: int, payload: bytes = b"") -> bytes:
    """Encode an MSPv1 message: $M< len code payload checksum."""
    size = len(payload)
    checksum = msp_checksum(code, payload)
    return b"$M<" + bytes([size, code]) + payload + bytes([checksum])


def msp_encode_rc(channels):
    """Encode MSP_SET_RAW_RC with 8-16 channel values (each 1000-2000)."""
    payload = struct.pack(f"<{len(channels)}H", *channels)
    return msp_encode(MSPCode.MSP_SET_RAW_RC, payload)


class MSPConnection:
    """Serial connection to a Betaflight flight controller using MSP."""

    def __init__(self, port="/dev/ttyAMA0", baudrate=115200, timeout=0.1):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self._serial = None
        self._lock = threading.Lock()

    def connect(self):
        if is_tcp_endpoint(self.port):
            host, port = parse_tcp_endpoint(self.port)
            self._serial = TCPSocketSerial(host, port, timeout=self.timeout)
            self._serial.open()
            logger.info("MSP connected on tcp://%s:%d", host, port)
        else:
            self._serial = serial.Serial(self.port, self.baudrate,
                                         timeout=self.timeout)
            logger.info("MSP connected on %s @ %d", self.port, self.baudrate)
        time.sleep(0.5)

    def disconnect(self):
        if self._serial and self._serial.is_open:
            self._serial.close()
            logger.info("MSP disconnected")

    def send(self, data):
        with self._lock:
            if self._serial and self._serial.is_open:
                self._serial.write(data)

    def send_rc(self, channels):
        """Send MSP_SET_RAW_RC. Channels clamped to [1000, 2000]."""
        clamped = [max(1000, min(2000, int(ch))) for ch in channels]
        self.send(msp_encode_rc(clamped))

    def request(self, code):
        """Send an MSP request and read the response payload."""
        with self._lock:
            if not self._serial or not self._serial.is_open:
                return None
            self._serial.write(msp_encode(code))
            for _ in range(4):
                try:
                    frame = self._read_frame()
                except MSPError as e:
                    logger.debug("MSP request %s failed: %s", code, e)
                    return None
                if frame is None:
                    return None
                response_code, payload = frame
                if response_code == int(code):
                    return payload
                logger.debug("Ignoring stale MSP response code %s", response_code)
            return None

    def _read_frame(self):
        """Read one MSPv1 response frame, scanning past unrelated bytes."""
        while True:
            b = self._serial.read(1)
            if not b:
                return None
            if b != b"$":
                continue

            if self._serial.read(1) != b"M":
                continue

            direction = self._serial.read(1)
            if direction not in (b">", b"!"):
                continue

            header = self._serial.read(2)
            if len(header) < 2:
                return None
            size, code = header[0], header[1]

            rest = self._serial.read(size + 1)
            if len(rest) < size + 1:
                return None
            payload = rest[:size]
            checksum = rest[size]
            expected = msp_checksum(code, payload)
            if checksum != expected:
                raise MSPChecksumError(
                    "bad checksum for code %d: got 0x%02x, expected 0x%02x" %
                    (code, checksum, expected)
                )
            if direction == b"!":
                raise MSPResponseError("FC returned MSP error for code %d" % code)
            return code, payload

    def get_rc_channels(self):
        """Read current RC channel values from FC via MSP_RC."""
        data = self.request(MSPCode.MSP_RC)
        if data is None or len(data) < 2:
            return None
        n = len(data) // 2
        return list(struct.unpack("<%dH" % n, data[:n * 2]))

    def get_api_version(self):
        """Request MSP_API_VERSION and return protocol/api version fields."""
        data = self.request(MSPCode.MSP_API_VERSION)
        if data is None or len(data) < 3:
            return None
        return {
            "protocol_version": data[0],
            "api_major": data[1],
            "api_minor": data[2],
        }

    def get_attitude(self):
        """Request MSP_ATTITUDE and return roll/pitch/yaw in degrees."""
        data = self.request(MSPCode.MSP_ATTITUDE)
        if data is None or len(data) < 6:
            return None
        roll, pitch, yaw = struct.unpack("<hhH", data[:6])
        return {"roll": roll / 10.0, "pitch": pitch / 10.0, "yaw": yaw}
