#!/usr/bin/env python3
"""Read Betaflight mode ranges over pr0p's Configurator-style websocket."""

from __future__ import annotations

import argparse
import json
import socket
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

PROBE_DIR = Path(__file__).resolve().parent
if str(PROBE_DIR) not in sys.path:
    sys.path.insert(0, str(PROBE_DIR))
TOOLS_DIR = PROBE_DIR.parents[1] / "tools"
if str(TOOLS_DIR) not in sys.path:
    sys.path.insert(0, str(TOOLS_DIR))

from msp_ws_semantic_probe import (  # noqa: E402
    FAIL,
    PASS,
    WAITING,
    MSPErrorFrame,
    MSPProtocolError,
    WebSocketProtocolError,
    request_msp_over_websocket,
)
from pr0p_capture_probe import DEFAULT_LOG_DIR  # noqa: E402
from sitl_configure_modes import MSP_MODE_RANGES, ModeRange, step_to_us  # noqa: E402
from sitl_msp import MSP_BOXIDS, MSP_BOXNAMES, parse_box_ids, parse_box_names  # noqa: E402


PERMANENT_ID_FALLBACKS = {
    0: "ARM",
    1: "ANGLE",
    2: "HORIZON",
    50: "MSP OVERRIDE",
}


@dataclass
class MspWsModeRangesResult:
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


def parse_mode_ranges(payload: bytes) -> list[ModeRange]:
    ranges: list[ModeRange] = []
    for index, offset in enumerate(range(0, len(payload) - 3, 4)):
        permanent_id, aux_channel_index, start_step, end_step = payload[offset:offset + 4]
        ranges.append(ModeRange(
            index=index,
            permanent_id=permanent_id,
            aux_channel_index=aux_channel_index,
            start_us=step_to_us(start_step),
            end_us=step_to_us(end_step),
        ))
    return ranges


def mode_name(permanent_id: int, *, box_names: list[str], box_ids: list[int]) -> str:
    for name, box_id in zip(box_names, box_ids):
        if box_id == permanent_id:
            return name
    return PERMANENT_ID_FALLBACKS.get(permanent_id, "mode_%d" % permanent_id)


def mode_range_dict(item: ModeRange, *, box_names: list[str], box_ids: list[int]) -> dict[str, Any]:
    aux_channel = item.aux_channel_index + 5
    return {
        "index": item.index,
        "permanent_id": item.permanent_id,
        "mode_name": mode_name(item.permanent_id, box_names=box_names, box_ids=box_ids),
        "aux_channel_index": item.aux_channel_index,
        "rc_channel": aux_channel,
        "channel_label": "AUX%d / CH%d" % (item.aux_channel_index + 1, aux_channel),
        "start_us": item.start_us,
        "end_us": item.end_us,
        "usable": item.is_usable(),
    }


def request_optional_payload(
    *,
    host: str,
    port: int,
    path: str,
    code: int,
    timeout: float,
    max_frames: int,
) -> tuple[bytes | None, dict[str, Any], list[str]]:
    try:
        payload, metrics = request_msp_over_websocket(
            host=host,
            port=port,
            path=path,
            code=code,
            timeout=timeout,
            max_frames=max_frames,
        )
    except Exception as exc:
        return None, {"code": code, "error": str(exc)}, ["MSP_OPTIONAL_ERROR:%d" % code]
    return payload, {"code": code, **metrics}, []


def run_msp_ws_mode_ranges_probe(
    *,
    host: str,
    port: int,
    path: str,
    timeout: float,
    max_frames: int,
) -> MspWsModeRangesResult:
    metrics: dict[str, Any] = {
        "host": host,
        "port": port,
        "path": path,
        "real_msp_write_sent": False,
    }
    notes: list[str] = []
    box_names_payload, box_names_metrics, box_name_notes = request_optional_payload(
        host=host,
        port=port,
        path=path,
        code=MSP_BOXNAMES,
        timeout=timeout,
        max_frames=max_frames,
    )
    box_ids_payload, box_ids_metrics, box_id_notes = request_optional_payload(
        host=host,
        port=port,
        path=path,
        code=MSP_BOXIDS,
        timeout=timeout,
        max_frames=max_frames,
    )
    notes.extend(box_name_notes)
    notes.extend(box_id_notes)
    box_names = parse_box_names(box_names_payload or b"")
    box_ids = parse_box_ids(box_ids_payload or b"")
    metrics.update({
        "box_names_request": box_names_metrics,
        "box_ids_request": box_ids_metrics,
        "box_names": box_names,
        "box_ids": box_ids,
    })

    try:
        payload, mode_metrics = request_msp_over_websocket(
            host=host,
            port=port,
            path=path,
            code=MSP_MODE_RANGES,
            timeout=timeout,
            max_frames=max_frames,
        )
    except (ConnectionRefusedError, TimeoutError, socket.timeout, OSError) as exc:
        return MspWsModeRangesResult(
            status=WAITING,
            summary="MSP mode range endpoint is not reachable yet",
            metrics={**metrics, "mode_ranges_request": {"code": MSP_MODE_RANGES, "error": str(exc)}},
            notes=notes + ["start pr0p, enter a local race, then retry"],
        )
    except MSPErrorFrame as exc:
        return MspWsModeRangesResult(
            status=WAITING,
            summary="MSP_MODE_RANGES is not available from this FC endpoint",
            metrics={**metrics, "mode_ranges_request": {"code": MSP_MODE_RANGES, "error": str(exc)}},
            notes=notes + ["MSP_MODE_RANGES_UNAVAILABLE"],
        )
    except (WebSocketProtocolError, MSPProtocolError) as exc:
        return MspWsModeRangesResult(
            status=FAIL,
            summary="websocket endpoint did not return a valid MSP mode range frame",
            metrics={**metrics, "mode_ranges_request": {"code": MSP_MODE_RANGES, "error": str(exc)}},
            notes=notes + ["MSP_MODE_RANGES_PROTOCOL_FAIL"],
        )

    parsed = parse_mode_ranges(payload or b"")
    ranges = [
        mode_range_dict(item, box_names=box_names, box_ids=box_ids)
        for item in parsed
    ]
    usable = [row for row in ranges if row["usable"]]
    arm_ranges = [row for row in usable if row["mode_name"] == "ARM" or row["permanent_id"] == 0]
    metrics.update({
        "mode_ranges_request": {"code": MSP_MODE_RANGES, **mode_metrics},
        "raw_payload_hex": (payload or b"").hex(),
        "mode_ranges": ranges,
        "usable_mode_ranges": usable,
        "arm_ranges": arm_ranges,
    })
    if arm_ranges:
        notes.append("ARM_MODE_RANGE:%s" % ",".join(row["channel_label"] for row in arm_ranges))
    else:
        notes.append("ARM_MODE_RANGE_MISSING")
    return MspWsModeRangesResult(
        status=PASS,
        summary="read-only MSP_MODE_RANGES returned %d usable mode ranges" % len(usable),
        metrics=metrics,
        notes=notes,
    )


def build_markdown(result: MspWsModeRangesResult, *, run_id: str) -> str:
    lines = [
        "# SimITL / pr0p MSP Mode Ranges Probe",
        "",
        "Run: `%s`" % run_id,
        "Verdict: `%s`" % result.status,
        "",
        result.summary,
        "",
        "| Mode | Channel | Range | Usable |",
        "| --- | --- | --- | --- |",
    ]
    for row in result.metrics.get("mode_ranges", []):
        lines.append("| %s | %s | `%s-%s` | `%s` |" % (
            row.get("mode_name"),
            row.get("channel_label"),
            row.get("start_us"),
            row.get("end_us"),
            row.get("usable"),
        ))
    lines.extend([
        "",
        "## Metrics",
        "",
        "```json",
        json.dumps(result.metrics, indent=2, sort_keys=True),
        "```",
    ])
    for note in result.notes:
        lines.append("- %s" % note)
    lines.append("")
    return "\n".join(lines)


def write_reports(result: MspWsModeRangesResult, log_dir: Path, *, run_id: str) -> tuple[Path, Path]:
    log_dir.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime("%Y%m%d-%H%M%S", time.localtime())
    json_path = log_dir / ("%s-%s-msp-ws-mode-ranges.json" % (stamp, run_id))
    md_path = log_dir / ("%s-%s-msp-ws-mode-ranges.md" % (stamp, run_id))
    json_path.write_text(json.dumps(result.as_dict(), indent=2, sort_keys=True) + "\n",
                         encoding="utf-8")
    md_path.write_text(build_markdown(result, run_id=run_id), encoding="utf-8")
    return json_path, md_path


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-id", default="pr0p-msp-mode-ranges")
    parser.add_argument("--log-dir", type=Path, default=DEFAULT_LOG_DIR)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=5761)
    parser.add_argument("--path", default="/")
    parser.add_argument("--timeout", type=float, default=0.5)
    parser.add_argument("--max-frames", type=int, default=8)
    args = parser.parse_args(argv)
    if args.port <= 0:
        parser.error("--port must be positive")
    if args.timeout <= 0:
        parser.error("--timeout must be positive")
    if args.max_frames <= 0:
        parser.error("--max-frames must be positive")
    return args


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    result = run_msp_ws_mode_ranges_probe(
        host=args.host,
        port=args.port,
        path=args.path,
        timeout=args.timeout,
        max_frames=args.max_frames,
    )
    json_path, md_path = write_reports(result, args.log_dir, run_id=args.run_id)
    print("simitl-pr0p-msp-mode-ranges %s report=%s summary=%s" % (
        result.status,
        json_path,
        md_path,
    ))
    print(result.summary)
    return 1 if result.status == FAIL else 0


if __name__ == "__main__":
    raise SystemExit(main())
