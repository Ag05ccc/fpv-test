#!/usr/bin/env python3
"""Read-only MSP probe over pr0p's Configurator-style websocket."""

from __future__ import annotations

import argparse
import json
import os
import socket
import struct
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

PROBE_DIR = Path(__file__).resolve().parent
if str(PROBE_DIR) not in sys.path:
    sys.path.insert(0, str(PROBE_DIR))

from msp_ws_probe import (  # noqa: E402
    build_handshake,
    expected_accept,
    parse_headers,
    websocket_key,
)


PASS = "PASS"
WAITING = "WAITING"
FAIL = "FAIL"
DEFAULT_LOG_DIR = Path("logs/simitl_pr0p")
MSP_API_VERSION = 1
MSP_RC = 105
MSP_ATTITUDE = 108


@dataclass
class MspWsSemanticResult:
    status: str
    summary: str
    metrics: dict[str, Any] = field(default_factory=dict)
    notes: list[str] = field(default_factory=list)

    def as_dict(self) -> dict[str, Any]:
        return {
            "status": self.status,
            "summary": self.summary,
            "metrics": self.metrics,
            "notes": self.notes,
        }


class WebSocketProtocolError(RuntimeError):
    pass


class MSPProtocolError(RuntimeError):
    pass


class MSPErrorFrame(MSPProtocolError):
    pass


def msp_checksum(code: int, payload: bytes = b"") -> int:
    checksum = len(payload) ^ int(code)
    for byte in payload:
        checksum ^= byte
    return checksum


def msp_encode(code: int, payload: bytes = b"") -> bytes:
    if len(payload) > 255:
        raise ValueError("MSPv1 payload is too large")
    return b"$M<" + bytes([len(payload), int(code)]) + payload + bytes([msp_checksum(code, payload)])


def parse_msp_frame(buffer: bytes, expected_code: int) -> tuple[int, bytes] | None:
    offset = 0
    while True:
        start = buffer.find(b"$M", offset)
        if start < 0:
            return None
        if len(buffer) < start + 6:
            return None
        direction = buffer[start + 2:start + 3]
        if direction not in (b">", b"!"):
            offset = start + 1
            continue
        size = buffer[start + 3]
        code = buffer[start + 4]
        end = start + 5 + size + 1
        if len(buffer) < end:
            return None
        payload = buffer[start + 5:start + 5 + size]
        checksum = buffer[start + 5 + size]
        expected = msp_checksum(code, payload)
        if checksum != expected:
            raise MSPProtocolError(
                "bad checksum for code %d: got 0x%02x, expected 0x%02x" %
                (code, checksum, expected)
            )
        if direction == b"!":
            raise MSPErrorFrame("FC returned MSP error for code %d" % code)
        if code == int(expected_code):
            return code, payload
        offset = end


def encode_ws_frame(payload: bytes, *, opcode: int = 2, mask_key: bytes | None = None) -> bytes:
    if not 0 <= opcode <= 15:
        raise ValueError("invalid websocket opcode")
    mask_key = os.urandom(4) if mask_key is None else mask_key
    if len(mask_key) != 4:
        raise ValueError("websocket mask key must be 4 bytes")
    header = bytearray([0x80 | opcode])
    length = len(payload)
    if length < 126:
        header.append(0x80 | length)
    elif length <= 0xFFFF:
        header.append(0x80 | 126)
        header.extend(struct.pack("!H", length))
    else:
        header.append(0x80 | 127)
        header.extend(struct.pack("!Q", length))
    header.extend(mask_key)
    masked = bytes(byte ^ mask_key[index % 4] for index, byte in enumerate(payload))
    return bytes(header) + masked


def read_exact(sock: socket.socket, size: int) -> bytes:
    chunks = bytearray()
    while len(chunks) < size:
        chunk = sock.recv(size - len(chunks))
        if not chunk:
            raise WebSocketProtocolError("socket closed while reading websocket frame")
        chunks.extend(chunk)
    return bytes(chunks)


def read_ws_frame(sock: socket.socket) -> tuple[int, bytes]:
    first, second = read_exact(sock, 2)
    opcode = first & 0x0F
    masked = bool(second & 0x80)
    length = second & 0x7F
    if length == 126:
        length = struct.unpack("!H", read_exact(sock, 2))[0]
    elif length == 127:
        length = struct.unpack("!Q", read_exact(sock, 8))[0]
    mask_key = read_exact(sock, 4) if masked else b""
    payload = read_exact(sock, length) if length else b""
    if masked:
        payload = bytes(byte ^ mask_key[index % 4] for index, byte in enumerate(payload))
    if opcode == 8:
        raise WebSocketProtocolError("server closed websocket")
    return opcode, payload


def websocket_connect(host: str, port: int, path: str, *, timeout: float) -> socket.socket:
    key = websocket_key()
    sock = socket.create_connection((host, port), timeout=timeout)
    try:
        sock.settimeout(timeout)
        sock.sendall(build_handshake(host, port, path, key))
        response = sock.recv(4096)
        status_code, headers = parse_headers(response)
        if status_code != 101 or headers.get("sec-websocket-accept") != expected_accept(key):
            raise WebSocketProtocolError("websocket handshake was not accepted")
        return sock
    except Exception:
        sock.close()
        raise


def request_msp_over_websocket(
    *,
    host: str,
    port: int,
    path: str,
    code: int,
    timeout: float,
    max_frames: int,
) -> tuple[bytes | None, dict[str, Any]]:
    started = time.monotonic()
    with websocket_connect(host, port, path, timeout=timeout) as sock:
        sock.sendall(encode_ws_frame(msp_encode(code)))
        received = bytearray()
        frame_count = 0
        while frame_count < max_frames:
            try:
                opcode, payload = read_ws_frame(sock)
            except socket.timeout:
                break
            frame_count += 1
            if opcode in (1, 2):
                received.extend(payload)
                parsed = parse_msp_frame(bytes(received), expected_code=code)
                if parsed is not None:
                    return parsed[1], {
                        "frames_read": frame_count,
                        "bytes_read": len(received),
                        "latency_ms": (time.monotonic() - started) * 1000.0,
                    }
            elif opcode == 9:
                sock.sendall(encode_ws_frame(payload, opcode=10))
        return None, {
            "frames_read": frame_count,
            "bytes_read": len(received),
            "latency_ms": (time.monotonic() - started) * 1000.0,
        }


def parse_api_version(payload: bytes | None) -> dict[str, int] | None:
    if payload is None or len(payload) < 3:
        return None
    return {
        "protocol_version": payload[0],
        "api_major": payload[1],
        "api_minor": payload[2],
    }


def run_msp_ws_semantic_probe(
    *,
    host: str,
    port: int,
    path: str,
    timeout: float,
    max_frames: int,
) -> MspWsSemanticResult:
    try:
        payload, metrics = request_msp_over_websocket(
            host=host,
            port=port,
            path=path,
            code=MSP_API_VERSION,
            timeout=timeout,
            max_frames=max_frames,
        )
    except (ConnectionRefusedError, TimeoutError, socket.timeout, OSError) as exc:
        return MspWsSemanticResult(
            status=WAITING,
            summary="MSP websocket endpoint is not reachable yet",
            metrics={"host": host, "port": port, "path": path, "error": str(exc)},
            notes=["start pr0p, enter a local race, then retry"],
        )
    except MSPErrorFrame as exc:
        return MspWsSemanticResult(
            status=FAIL,
            summary="FC returned an MSP error frame",
            metrics={"host": host, "port": port, "path": path, "error": str(exc)},
            notes=["MSP_ERROR_FRAME"],
        )
    except (WebSocketProtocolError, MSPProtocolError) as exc:
        return MspWsSemanticResult(
            status=FAIL,
            summary="websocket endpoint did not behave like MSP-over-websocket",
            metrics={"host": host, "port": port, "path": path, "error": str(exc)},
            notes=["MSP_WS_PROTOCOL_FAIL"],
        )

    api = parse_api_version(payload)
    if api is None:
        return MspWsSemanticResult(
            status=WAITING,
            summary="websocket handshake is available, but MSP_API_VERSION did not return yet",
            metrics={
                "host": host,
                "port": port,
                "path": path,
                "request_code": MSP_API_VERSION,
                "api_version": None,
                **metrics,
            },
            notes=[
                "enter a local pr0p race and close other Configurator clients before retrying",
                "this is WAITING, not PASS, because FC semantics are not proven",
            ],
        )
    return MspWsSemanticResult(
        status=PASS,
        summary="read-only MSP_API_VERSION responded over websocket",
        metrics={
            "host": host,
            "port": port,
            "path": path,
            "request_code": MSP_API_VERSION,
            "api_version": api,
            **metrics,
        },
        notes=[
            "this proves the endpoint is more than a bare websocket handshake",
            "no arm, throttle, or RC write command was sent",
        ],
    )


def build_markdown(result: MspWsSemanticResult, *, run_id: str) -> str:
    lines = [
        "# SimITL / pr0p MSP Websocket Semantic Probe",
        "",
        "Run: `%s`" % run_id,
        "Verdict: `%s`" % result.status,
        "",
        result.summary,
        "",
        "```json",
        json.dumps(result.metrics, indent=2, sort_keys=True),
        "```",
    ]
    for note in result.notes:
        lines.append("- %s" % note)
    lines.append("")
    return "\n".join(lines)


def write_reports(result: MspWsSemanticResult, log_dir: Path, *, run_id: str) -> tuple[Path, Path]:
    log_dir.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime("%Y%m%d-%H%M%S", time.localtime())
    json_path = log_dir / ("%s-%s-msp-ws-semantic.json" % (stamp, run_id))
    md_path = log_dir / ("%s-%s-msp-ws-semantic.md" % (stamp, run_id))
    json_path.write_text(json.dumps(result.as_dict(), indent=2, sort_keys=True) + "\n",
                         encoding="utf-8")
    md_path.write_text(build_markdown(result, run_id=run_id), encoding="utf-8")
    return json_path, md_path


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=5761)
    parser.add_argument("--path", default="/")
    parser.add_argument("--timeout", type=float, default=1.0)
    parser.add_argument("--max-frames", type=int, default=8)
    parser.add_argument("--log-dir", type=Path, default=DEFAULT_LOG_DIR)
    parser.add_argument("--run-id", default="pr0p-msp-ws-semantic")
    args = parser.parse_args(argv)
    if args.port <= 0:
        parser.error("--port must be positive")
    if args.timeout <= 0:
        parser.error("--timeout must be positive")
    if args.max_frames <= 0:
        parser.error("--max-frames must be positive")
    if not args.path.startswith("/"):
        parser.error("--path must start with /")
    return args


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    result = run_msp_ws_semantic_probe(
        host=args.host,
        port=args.port,
        path=args.path,
        timeout=args.timeout,
        max_frames=args.max_frames,
    )
    json_path, md_path = write_reports(result, args.log_dir, run_id=args.run_id)
    print("simitl-pr0p-msp-ws-semantic %s report=%s summary=%s" % (
        result.status,
        json_path,
        md_path,
    ))
    print(result.summary)
    return 1 if result.status == FAIL else 0


if __name__ == "__main__":
    raise SystemExit(main())
