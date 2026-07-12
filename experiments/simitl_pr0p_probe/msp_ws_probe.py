#!/usr/bin/env python3
"""Minimal websocket endpoint probe for pr0p / SimITL virtual FC."""

from __future__ import annotations

import argparse
import base64
import hashlib
import json
import os
import socket
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any


PASS = "PASS"
WAITING = "WAITING"
FAIL = "FAIL"
DEFAULT_LOG_DIR = Path("logs/simitl_pr0p")


@dataclass
class WsProbeResult:
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


def websocket_key() -> str:
    return base64.b64encode(os.urandom(16)).decode("ascii")


def expected_accept(key: str) -> str:
    digest = hashlib.sha1((key + "258EAFA5-E914-47DA-95CA-C5AB0DC85B11").encode("ascii"))
    return base64.b64encode(digest.digest()).decode("ascii")


def build_handshake(host: str, port: int, path: str, key: str) -> bytes:
    text = (
        "GET %s HTTP/1.1\r\n"
        "Host: %s:%d\r\n"
        "Upgrade: websocket\r\n"
        "Connection: Upgrade\r\n"
        "Sec-WebSocket-Key: %s\r\n"
        "Sec-WebSocket-Version: 13\r\n"
        "\r\n"
    ) % (path, host, port, key)
    return text.encode("ascii")


def parse_headers(response: bytes) -> tuple[int | None, dict[str, str]]:
    text = response.decode("iso-8859-1", errors="replace")
    lines = text.split("\r\n")
    status_code = None
    if lines and lines[0].startswith("HTTP/"):
        parts = lines[0].split()
        if len(parts) >= 2 and parts[1].isdigit():
            status_code = int(parts[1])
    headers: dict[str, str] = {}
    for line in lines[1:]:
        if not line or ":" not in line:
            continue
        name, value = line.split(":", 1)
        headers[name.strip().lower()] = value.strip()
    return status_code, headers


def probe_websocket(host: str, port: int, path: str, *, timeout: float) -> WsProbeResult:
    key = websocket_key()
    request = build_handshake(host, port, path, key)
    started = time.monotonic()
    try:
        with socket.create_connection((host, port), timeout=timeout) as sock:
            sock.settimeout(timeout)
            sock.sendall(request)
            response = sock.recv(4096)
    except OSError as exc:
        return WsProbeResult(
            status=WAITING,
            summary="websocket endpoint is not reachable yet",
            metrics={
                "host": host,
                "port": port,
                "path": path,
                "error": str(exc),
                "latency_ms": (time.monotonic() - started) * 1000.0,
            },
            notes=["start pr0p and a local race, then retry"],
        )
    status_code, headers = parse_headers(response)
    accept = headers.get("sec-websocket-accept")
    expected = expected_accept(key)
    ok = status_code == 101 and accept == expected
    return WsProbeResult(
        status=PASS if ok else FAIL,
        summary=(
            "websocket handshake accepted"
            if ok else
            "endpoint responded but did not complete a valid websocket handshake"
        ),
        metrics={
            "host": host,
            "port": port,
            "path": path,
            "status_code": status_code,
            "upgrade": headers.get("upgrade"),
            "connection": headers.get("connection"),
            "accept_matches": accept == expected,
            "latency_ms": (time.monotonic() - started) * 1000.0,
        },
        notes=[
            "this proves the Configurator-style websocket surface exists",
            "it does not yet prove MSP command semantics",
        ],
    )


def build_markdown(result: WsProbeResult, *, run_id: str) -> str:
    lines = [
        "# SimITL / pr0p Websocket Probe",
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


def write_reports(result: WsProbeResult, log_dir: Path, *, run_id: str) -> tuple[Path, Path]:
    log_dir.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime("%Y%m%d-%H%M%S", time.localtime())
    json_path = log_dir / ("%s-%s-ws-probe.json" % (stamp, run_id))
    md_path = log_dir / ("%s-%s-ws-probe.md" % (stamp, run_id))
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
    parser.add_argument("--log-dir", type=Path, default=DEFAULT_LOG_DIR)
    parser.add_argument("--run-id", default="pr0p-ws")
    args = parser.parse_args(argv)
    if args.port <= 0:
        parser.error("--port must be positive")
    if args.timeout <= 0:
        parser.error("--timeout must be positive")
    if not args.path.startswith("/"):
        parser.error("--path must start with /")
    return args


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    result = probe_websocket(args.host, args.port, args.path, timeout=args.timeout)
    json_path, md_path = write_reports(result, args.log_dir, run_id=args.run_id)
    print("simitl-pr0p-ws %s report=%s summary=%s" % (result.status, json_path, md_path))
    print(result.summary)
    return 1 if result.status == FAIL else 0


if __name__ == "__main__":
    raise SystemExit(main())
