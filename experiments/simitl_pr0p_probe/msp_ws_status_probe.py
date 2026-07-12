#!/usr/bin/env python3
"""Read-only FC status probe over pr0p's Configurator-style websocket."""

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
from sitl_msp import (  # noqa: E402
    MSP_BOXIDS,
    MSP_BOXNAMES,
    MSP_STATUS_EX,
    parse_box_ids,
    parse_box_names,
    parse_status_ex,
)


DEFAULT_LOG_DIR = Path("logs/simitl_pr0p")
MSP_STATUS = 101


@dataclass
class MspWsStatusResult:
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


def _mode_flag_set(flags: int, bit: int) -> bool:
    if bit < 0 or bit >= 32:
        return False
    return bool(flags & (1 << bit))


def parse_status_basic(
    payload: bytes | None,
    *,
    box_names: list[str] | None = None,
    box_ids: list[int] | None = None,
) -> dict[str, Any]:
    if payload is None or len(payload) < 10:
        return {
            "valid": False,
            "armed": None,
            "flight_mode_flags": 0,
            "active_modes_valid": False,
            "active_modes": [],
            "active_mode_ids": [],
            "box_names": box_names or [],
            "box_ids": box_ids or [],
            "arming_disable_flags": None,
            "arming_disable_names": [],
        }

    flight_mode_flags = struct.unpack_from("<I", payload, 6)[0]
    names = box_names or []
    ids = box_ids or []
    active_modes = [
        name
        for index, name in enumerate(names[:32])
        if _mode_flag_set(flight_mode_flags, index)
    ]
    active_mode_ids = [
        ids[index]
        for index in range(min(len(names), len(ids), 32))
        if _mode_flag_set(flight_mode_flags, index)
    ]
    active_modes_valid = bool(names)
    return {
        "valid": True,
        "armed": ("ARM" in active_modes) if active_modes_valid else None,
        "cycle_time_us": struct.unpack_from("<H", payload, 0)[0],
        "i2c_errors": struct.unpack_from("<H", payload, 2)[0],
        "sensor_flags": struct.unpack_from("<H", payload, 4)[0],
        "flight_mode_flags": flight_mode_flags,
        "active_modes_valid": active_modes_valid,
        "active_modes": active_modes,
        "active_mode_ids": active_mode_ids,
        "box_names": names,
        "box_ids": ids,
        "arming_disable_flags": None,
        "arming_disable_names": [],
    }


def status_notes(status: dict[str, Any], *, source: str) -> list[str]:
    notes = ["FC_STATUS_READONLY", "status source: %s" % source]
    armed = status.get("armed")
    if armed is True:
        notes.append("FC_ARMED")
    elif armed is False:
        notes.append("FC_NOT_ARMED")
    else:
        notes.append("FC_ARM_STATE_UNKNOWN")
    if not status.get("active_modes_valid"):
        notes.append("BOX_METADATA_MISSING")
    arming_names = status.get("arming_disable_names") or []
    if arming_names:
        notes.append("ARMING_DISABLED:%s" % ",".join(str(name) for name in arming_names))
    if "THROTTLE" in arming_names:
        notes.append("THROTTLE_NOT_LOW_FOR_ARM")
    return notes


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
    except MSPErrorFrame as exc:
        return None, {"code": code, "error": str(exc)}, ["MSP_OPTIONAL_ERROR:%d" % code]
    except (ConnectionRefusedError, TimeoutError, socket.timeout, OSError) as exc:
        return None, {"code": code, "error": str(exc)}, ["MSP_OPTIONAL_UNREACHABLE:%d" % code]
    except (WebSocketProtocolError, MSPProtocolError) as exc:
        return None, {"code": code, "error": str(exc)}, ["MSP_OPTIONAL_PROTOCOL_FAIL:%d" % code]
    return payload, {"code": code, **metrics}, []


def run_msp_ws_status_once(
    *,
    host: str,
    port: int,
    path: str,
    timeout: float,
    max_frames: int,
) -> MspWsStatusResult:
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
        status_payload, status_metrics = request_msp_over_websocket(
            host=host,
            port=port,
            path=path,
            code=MSP_STATUS_EX,
            timeout=timeout,
            max_frames=max_frames,
        )
        status = parse_status_ex(status_payload or b"", box_names=box_names, box_ids=box_ids)
        source = "MSP_STATUS_EX"
        metrics["status_request"] = {"code": MSP_STATUS_EX, **status_metrics}
    except MSPErrorFrame as exc:
        metrics["status_ex_error"] = str(exc)
        try:
            status_payload, status_metrics = request_msp_over_websocket(
                host=host,
                port=port,
                path=path,
                code=MSP_STATUS,
                timeout=timeout,
                max_frames=max_frames,
            )
        except (ConnectionRefusedError, TimeoutError, socket.timeout, OSError) as fallback_exc:
            return MspWsStatusResult(
                status=WAITING,
                summary="MSP status endpoint is not reachable yet",
                metrics={**metrics, "status_request": {"code": MSP_STATUS, "error": str(fallback_exc)}},
                notes=notes + ["MSP_STATUS_UNREACHABLE"],
            )
        except (MSPErrorFrame, WebSocketProtocolError, MSPProtocolError) as fallback_exc:
            return MspWsStatusResult(
                status=FAIL,
                summary="FC status requests returned MSP/protocol errors",
                metrics={**metrics, "status_request": {"code": MSP_STATUS, "error": str(fallback_exc)}},
                notes=notes + ["MSP_STATUS_PROTOCOL_FAIL"],
            )
        status = parse_status_basic(status_payload, box_names=box_names, box_ids=box_ids)
        source = "MSP_STATUS"
        metrics["status_request"] = {"code": MSP_STATUS, **status_metrics}
        notes.append("MSP_STATUS_EX_UNAVAILABLE")
    except (ConnectionRefusedError, TimeoutError, socket.timeout, OSError) as exc:
        return MspWsStatusResult(
            status=WAITING,
            summary="MSP status endpoint is not reachable yet",
            metrics={**metrics, "status_request": {"code": MSP_STATUS_EX, "error": str(exc)}},
            notes=notes + ["start pr0p, enter a local race, then retry"],
        )
    except (WebSocketProtocolError, MSPProtocolError) as exc:
        return MspWsStatusResult(
            status=FAIL,
            summary="websocket endpoint did not return a valid MSP status frame",
            metrics={**metrics, "status_request": {"code": MSP_STATUS_EX, "error": str(exc)}},
            notes=notes + ["MSP_STATUS_PROTOCOL_FAIL"],
        )

    metrics["status_source"] = source
    metrics["fc_status"] = status
    if not status.get("valid"):
        return MspWsStatusResult(
            status=WAITING,
            summary="MSP status response was present but too short to parse",
            metrics=metrics,
            notes=notes + ["MSP_STATUS_TOO_SHORT"],
        )
    return MspWsStatusResult(
        status=PASS,
        summary="read-only FC status responded over %s" % source,
        metrics=metrics,
        notes=notes + status_notes(status, source=source),
    )


def combine_sample_statuses(statuses: list[str]) -> str:
    if any(status == FAIL for status in statuses):
        return FAIL
    if any(status == PASS for status in statuses):
        return PASS
    return WAITING


def summarize_status_samples(samples: list[dict[str, Any]]) -> dict[str, Any]:
    pass_samples = [sample for sample in samples if sample.get("status") == PASS]
    latest_pass = pass_samples[-1] if pass_samples else None
    blockers_seen: set[str] = set()
    armed_values: list[Any] = []
    for sample in pass_samples:
        fc_status = sample.get("metrics", {}).get("fc_status", {})
        armed_values.append(fc_status.get("armed"))
        blockers_seen.update(str(name) for name in fc_status.get("arming_disable_names", []) or [])
    return {
        "sample_count": len(samples),
        "pass_count": len(pass_samples),
        "status_counts": {
            status: sum(1 for sample in samples if sample.get("status") == status)
            for status in sorted({str(sample.get("status")) for sample in samples})
        },
        "latest_fc_status": latest_pass.get("metrics", {}).get("fc_status") if latest_pass else None,
        "armed_values": armed_values,
        "arming_disable_names_seen": sorted(blockers_seen),
    }


def run_msp_ws_status_probe(
    *,
    host: str,
    port: int,
    path: str,
    timeout: float,
    max_frames: int,
    samples: int = 1,
    interval_s: float = 0.0,
) -> MspWsStatusResult:
    if samples <= 1:
        return run_msp_ws_status_once(
            host=host,
            port=port,
            path=path,
            timeout=timeout,
            max_frames=max_frames,
        )

    sample_results: list[MspWsStatusResult] = []
    for index in range(samples):
        sample_results.append(run_msp_ws_status_once(
            host=host,
            port=port,
            path=path,
            timeout=timeout,
            max_frames=max_frames,
        ))
        if index + 1 < samples and interval_s > 0:
            time.sleep(interval_s)

    sample_dicts = [sample.as_dict() for sample in sample_results]
    combined_status = combine_sample_statuses([sample.status for sample in sample_results])
    summary = summarize_status_samples(sample_dicts)
    latest_pass = next(
        (sample for sample in reversed(sample_results) if sample.status == PASS),
        None,
    )
    notes = sorted({
        note
        for sample in sample_results
        for note in sample.notes
    })
    metrics = {
        "host": host,
        "port": port,
        "path": path,
        "real_msp_write_sent": False,
        "samples": sample_dicts,
        "monitor_summary": summary,
    }
    if latest_pass is not None:
        metrics["status_source"] = latest_pass.metrics.get("status_source")
        metrics["fc_status"] = latest_pass.metrics.get("fc_status")
    return MspWsStatusResult(
        status=combined_status,
        summary=(
            "read-only FC status monitor collected %d samples" % samples
            if combined_status == PASS else
            "MSP status monitor did not collect a readable FC status sample"
        ),
        metrics=metrics,
        notes=notes,
    )


def build_markdown(result: MspWsStatusResult, *, run_id: str) -> str:
    status = result.metrics.get("fc_status") if isinstance(result.metrics, dict) else None
    lines = [
        "# SimITL / pr0p MSP FC Status Probe",
        "",
        "Run: `%s`" % run_id,
        "Verdict: `%s`" % result.status,
        "",
        result.summary,
        "",
        "| Metric | Value |",
        "| --- | --- |",
    ]
    for key in ("status_source", "real_msp_write_sent"):
        if key in result.metrics:
            lines.append("| %s | `%s` |" % (key, result.metrics[key]))
    if isinstance(status, dict):
        for key in ("armed", "active_modes", "arming_disable_names", "flight_mode_flags"):
            lines.append("| fc_status.%s | `%s` |" % (key, status.get(key)))
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


def write_reports(result: MspWsStatusResult, log_dir: Path, *, run_id: str) -> tuple[Path, Path]:
    log_dir.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime("%Y%m%d-%H%M%S", time.localtime())
    json_path = log_dir / ("%s-%s-msp-ws-status.json" % (stamp, run_id))
    md_path = log_dir / ("%s-%s-msp-ws-status.md" % (stamp, run_id))
    json_path.write_text(
        json.dumps(result.as_dict(), indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    md_path.write_text(build_markdown(result, run_id=run_id), encoding="utf-8")
    return json_path, md_path


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-id", default="pr0p-msp-status")
    parser.add_argument("--log-dir", type=Path, default=DEFAULT_LOG_DIR)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=5761)
    parser.add_argument("--path", default="/")
    parser.add_argument("--timeout", type=float, default=0.5)
    parser.add_argument("--max-frames", type=int, default=8)
    parser.add_argument("--samples", type=int, default=1)
    parser.add_argument("--interval", type=float, default=0.0)
    args = parser.parse_args(argv)
    if args.port <= 0:
        parser.error("--port must be positive")
    if args.timeout <= 0:
        parser.error("--timeout must be positive")
    if args.max_frames <= 0:
        parser.error("--max-frames must be positive")
    if args.samples <= 0:
        parser.error("--samples must be positive")
    if args.interval < 0:
        parser.error("--interval must be non-negative")
    return args


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    result = run_msp_ws_status_probe(
        host=args.host,
        port=args.port,
        path=args.path,
        timeout=args.timeout,
        max_frames=args.max_frames,
        samples=args.samples,
        interval_s=args.interval,
    )
    json_path, md_path = write_reports(result, args.log_dir, run_id=args.run_id)
    print("simitl-pr0p-msp-status %s report=%s summary=%s" % (
        result.status,
        json_path,
        md_path,
    ))
    print(result.summary)
    return 1 if result.status == FAIL else 0


if __name__ == "__main__":
    raise SystemExit(main())
