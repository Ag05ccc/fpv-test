#!/usr/bin/env python3
"""MSP_SET_RAW_RC loopback probe over pr0p's websocket FC endpoint."""

from __future__ import annotations

import argparse
import json
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

REPO_ROOT = PROBE_DIR.parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from kenet.rc_channels import pilot_to_msp_rc_channels  # noqa: E402
from msp_ws_semantic_probe import (  # noqa: E402
    FAIL,
    PASS,
    WAITING,
    MSP_RC,
    WebSocketProtocolError,
    encode_ws_frame,
    msp_encode,
    parse_msp_frame,
    read_ws_frame,
    request_msp_over_websocket,
    websocket_connect,
)


DEFAULT_LOG_DIR = Path("logs/simitl_pr0p")
MSP_SET_RAW_RC = 200
DEFAULT_CHANNELS = [1450, 1620, 1120, 1380, 1000, 1000, 1000, 1000]


@dataclass
class MspWsRcResult:
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


def parse_channels(value: str) -> list[int]:
    try:
        channels = [int(part.strip()) for part in value.split(",") if part.strip()]
    except ValueError as exc:
        raise argparse.ArgumentTypeError("channels must be comma-separated integers") from exc
    if not 8 <= len(channels) <= 16:
        raise argparse.ArgumentTypeError("expected 8..16 RC channels")
    bad = [channel for channel in channels if not 1000 <= channel <= 2000]
    if bad:
        raise argparse.ArgumentTypeError("RC channels must be in 1000..2000")
    return channels


def encode_rc_payload(channels: list[int]) -> bytes:
    return struct.pack("<%dH" % len(channels), *channels)


def parse_rc_payload(payload: bytes | None) -> list[int] | None:
    if payload is None or len(payload) < 2:
        return None
    count = len(payload) // 2
    return list(struct.unpack("<%dH" % count, payload[:count * 2]))


def rc_readback_mismatches(sent_pilot_order: list[int], readback_internal: list[int] | None) -> list[str]:
    if not readback_internal:
        return ["MSP_RC readback missing after MSP_SET_RAW_RC"]
    expected_internal = pilot_to_msp_rc_channels(sent_pilot_order)
    if len(readback_internal) < len(expected_internal):
        return [
            "MSP_RC readback shorter than sent frame: got %d, expected at least %d" %
            (len(readback_internal), len(expected_internal))
        ]
    mismatches = []
    labels = ["roll", "pitch", "throttle", "yaw"]
    for index, expected in enumerate(expected_internal[:4]):
        actual = readback_internal[index]
        if actual != expected:
            mismatches.append("%s/internal[%d] expected %d, got %d" % (
                labels[index],
                index,
                expected,
                actual,
            ))
    return mismatches


def rc_prefix_matches(left: list[int] | None, right: list[int] | None, count: int) -> bool:
    if left is None or right is None:
        return False
    if len(left) < count or len(right) < count:
        return False
    return left[:count] == right[:count]


def rc_channel_deltas(samples: list[list[int]]) -> list[int]:
    if not samples:
        return []
    count = min(len(sample) for sample in samples)
    return [
        max(sample[index] for sample in samples) - min(sample[index] for sample in samples)
        for index in range(count)
    ]


def sample_rc_baseline(
    *,
    host: str,
    port: int,
    path: str,
    timeout: float,
    max_frames: int,
    samples: int,
    interval_s: float,
) -> tuple[list[list[int]], list[dict[str, Any]]]:
    rc_samples: list[list[int]] = []
    metrics: list[dict[str, Any]] = []
    for index in range(samples):
        payload, request_metrics = request_msp_over_websocket(
            host=host,
            port=port,
            path=path,
            code=MSP_RC,
            timeout=timeout,
            max_frames=max_frames,
        )
        parsed = parse_rc_payload(payload)
        metrics.append({"index": index, "rc": parsed, **request_metrics})
        if parsed is not None:
            rc_samples.append(parsed)
        if index + 1 < samples:
            time.sleep(interval_s)
    return rc_samples, metrics


def send_raw_rc_once(
    *,
    host: str,
    port: int,
    path: str,
    channels: list[int],
    timeout: float,
    max_frames: int,
) -> dict[str, Any]:
    started = time.monotonic()
    with websocket_connect(host, port, path, timeout=timeout) as sock:
        sock.sendall(encode_ws_frame(msp_encode(MSP_SET_RAW_RC, encode_rc_payload(channels))))
        received = bytearray()
        frame_count = 0
        while frame_count < max_frames:
            try:
                opcode, payload = read_ws_frame(sock)
            except (TimeoutError, socket.timeout):
                break
            frame_count += 1
            if opcode in (1, 2):
                received.extend(payload)
                parsed = parse_msp_frame(bytes(received), expected_code=MSP_SET_RAW_RC)
                if parsed is not None:
                    return {
                        "ack": True,
                        "frames_read": frame_count,
                        "bytes_read": len(received),
                        "latency_ms": (time.monotonic() - started) * 1000.0,
                    }
        return {
            "ack": False,
            "frames_read": frame_count,
            "bytes_read": len(received),
            "latency_ms": (time.monotonic() - started) * 1000.0,
        }


def run_msp_ws_rc_probe(
    *,
    host: str,
    port: int,
    path: str,
    timeout: float,
    channels: list[int],
    write: bool,
    settle_s: float,
    max_frames: int,
    baseline_samples: int,
    baseline_interval_s: float,
    max_baseline_delta: int,
    allow_unstable_baseline: bool,
) -> MspWsRcResult:
    metrics: dict[str, Any] = {
        "host": host,
        "port": port,
        "path": path,
        "sent_pilot_order": channels,
        "expected_internal": pilot_to_msp_rc_channels(channels),
        "settle_s": settle_s,
        "write_requested": write,
        "real_msp_write_sent": False,
    }
    try:
        baseline, baseline_metrics = sample_rc_baseline(
            host=host,
            port=port,
            path=path,
            timeout=timeout,
            max_frames=max_frames,
            samples=baseline_samples,
            interval_s=baseline_interval_s,
        )
        deltas = rc_channel_deltas(baseline)
        metrics.update({
            "baseline_samples": baseline,
            "baseline_request_metrics": baseline_metrics,
            "baseline_deltas": deltas,
            "max_baseline_delta": max(deltas) if deltas else None,
            "baseline_threshold": max_baseline_delta,
        })
        if not baseline:
            return MspWsRcResult(
                status=WAITING,
                summary="MSP_RC baseline could not be read yet",
                metrics=metrics,
                notes=["MSP_RC_BASELINE_MISSING"],
            )
        unstable = bool(deltas and max(deltas) > max_baseline_delta)
        if unstable and not allow_unstable_baseline:
            return MspWsRcResult(
                status=WAITING,
                summary="MSP_RC baseline is moving; physical/controller input may be active",
                metrics=metrics,
                notes=["RC_INPUT_CONTENTION", "leave sticks neutral or disconnect competing input before write tests"],
            )
        if not write:
            return MspWsRcResult(
                status=PASS,
                summary="MSP_RC baseline was readable and stable; no RC write was sent",
                metrics=metrics,
                notes=["read-only RC baseline", "rerun with --write --ack-live-msp-write for loopback"],
            )

        first = send_raw_rc_once(
            host=host,
            port=port,
            path=path,
            channels=channels,
            timeout=timeout,
            max_frames=max_frames,
        )
        time.sleep(settle_s)
        second = send_raw_rc_once(
            host=host,
            port=port,
            path=path,
            channels=channels,
            timeout=timeout,
            max_frames=max_frames,
        )
        time.sleep(settle_s)
        payload, rc_metrics = request_msp_over_websocket(
            host=host,
            port=port,
            path=path,
            code=MSP_RC,
            timeout=timeout,
            max_frames=max_frames,
        )
    except (ConnectionRefusedError, TimeoutError, WebSocketProtocolError, OSError) as exc:
        return MspWsRcResult(
            status=WAITING,
            summary="MSP websocket RC endpoint is not reachable yet",
            metrics={**metrics, "error": str(exc)},
            notes=["start pr0p local race and rerun P2/P5 RC loopback"],
        )

    readback = parse_rc_payload(payload)
    metrics.update({
        "send_attempts": [first, second],
        "readback_internal": readback,
        "readback_metrics": rc_metrics,
        "real_msp_write_sent": True,
    })
    mismatches = rc_readback_mismatches(channels, readback)
    if mismatches:
        expected_internal = pilot_to_msp_rc_channels(channels)
        unchanged_from_baseline = rc_prefix_matches(
            readback,
            baseline[-1] if baseline else None,
            len(expected_internal),
        )
        if unchanged_from_baseline:
            return MspWsRcResult(
                status=WAITING,
                summary="MSP_SET_RAW_RC was acknowledged, but MSP_RC stayed at baseline",
                metrics={**metrics, "mismatches": mismatches},
                notes=[
                    "MSP_RC_WRITE_NOT_LATCHED_OR_RX_OVERRIDDEN",
                    "physical RC, input priority, or MSP override mode may be gating the write",
                ],
            )
        return MspWsRcResult(
            status=FAIL,
            summary="MSP_SET_RAW_RC write/readback did not match expected channel mapping",
            metrics={**metrics, "mismatches": mismatches},
            notes=["MSP_RC_LOOPBACK_MISMATCH"],
        )
    return MspWsRcResult(
        status=PASS,
        summary="MSP_SET_RAW_RC loopback matched MSP_RC readback over websocket",
        metrics=metrics,
        notes=[
            "this proves a command path into the virtual FC",
            "default channels keep throttle low and arm low",
        ],
    )


def build_markdown(result: MspWsRcResult, *, run_id: str) -> str:
    lines = [
        "# SimITL / pr0p MSP Websocket RC Probe",
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


def write_reports(result: MspWsRcResult, log_dir: Path, *, run_id: str) -> tuple[Path, Path]:
    log_dir.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime("%Y%m%d-%H%M%S", time.localtime())
    json_path = log_dir / ("%s-%s-msp-ws-rc.json" % (stamp, run_id))
    md_path = log_dir / ("%s-%s-msp-ws-rc.md" % (stamp, run_id))
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
    parser.add_argument("--settle", type=float, default=0.2)
    parser.add_argument("--max-frames", type=int, default=8)
    parser.add_argument("--baseline-samples", type=int, default=3)
    parser.add_argument("--baseline-interval", type=float, default=0.1)
    parser.add_argument("--max-baseline-delta", type=int, default=4)
    parser.add_argument("--allow-unstable-baseline", action="store_true",
                        help="Allow MSP write even if RC readback is moving")
    parser.add_argument("--channels", type=parse_channels, default=DEFAULT_CHANNELS)
    parser.add_argument("--write", action="store_true",
                        help="Send MSP_SET_RAW_RC after the read-only baseline")
    parser.add_argument("--ack-live-msp-write", action="store_true",
                        help="Required before sending MSP_SET_RAW_RC")
    parser.add_argument("--log-dir", type=Path, default=DEFAULT_LOG_DIR)
    parser.add_argument("--run-id", default="pr0p-msp-ws-rc")
    args = parser.parse_args(argv)
    if args.write and not args.ack_live_msp_write:
        parser.error("--write requires --ack-live-msp-write")
    if args.port <= 0:
        parser.error("--port must be positive")
    if args.timeout <= 0:
        parser.error("--timeout must be positive")
    if args.settle < 0:
        parser.error("--settle must be non-negative")
    if args.baseline_samples <= 0:
        parser.error("--baseline-samples must be positive")
    if args.baseline_interval < 0:
        parser.error("--baseline-interval must be non-negative")
    if args.max_baseline_delta < 0:
        parser.error("--max-baseline-delta must be non-negative")
    if args.max_frames <= 0:
        parser.error("--max-frames must be positive")
    if not args.path.startswith("/"):
        parser.error("--path must start with /")
    return args


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    result = run_msp_ws_rc_probe(
        host=args.host,
        port=args.port,
        path=args.path,
        timeout=args.timeout,
        channels=args.channels,
        write=args.write,
        settle_s=args.settle,
        max_frames=args.max_frames,
        baseline_samples=args.baseline_samples,
        baseline_interval_s=args.baseline_interval,
        max_baseline_delta=args.max_baseline_delta,
        allow_unstable_baseline=args.allow_unstable_baseline,
    )
    json_path, md_path = write_reports(result, args.log_dir, run_id=args.run_id)
    print("simitl-pr0p-msp-ws-rc %s report=%s summary=%s" % (
        result.status,
        json_path,
        md_path,
    ))
    print(result.summary)
    return 1 if result.status == FAIL else 0


if __name__ == "__main__":
    raise SystemExit(main())
