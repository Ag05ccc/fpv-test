#!/usr/bin/env python3
"""Small JSONL logging helpers for SITL tools."""

from __future__ import annotations

import json
import os
import time
from pathlib import Path
from typing import Any


def timestamp_slug(now: float | None = None) -> str:
    return time.strftime("%Y%m%d-%H%M%S", time.localtime(now or time.time()))


def resolve_log_dir(value: str | os.PathLike[str] | None = None,
                    repo_root: Path | None = None) -> Path:
    if value:
        return Path(value).expanduser()
    env_value = os.environ.get("KENET_SITL_LOG_DIR")
    if env_value:
        return Path(env_value).expanduser()
    root = repo_root or Path(__file__).resolve().parents[1]
    return root / "logs" / "sitl"


def make_log_path(log_dir: Path, prefix: str, suffix: str = ".jsonl") -> Path:
    log_dir.mkdir(parents=True, exist_ok=True)
    return log_dir / ("%s-%s%s" % (timestamp_slug(), prefix, suffix))


class JsonlLogger:
    def __init__(
        self,
        path: Path,
        metadata: dict[str, Any] | None = None,
        *,
        append: bool = False,
        flush_every: int = 1,
        flush_interval: float = 0.0,
        max_bytes: int | None = None,
    ):
        if flush_every <= 0:
            raise ValueError("flush_every must be positive")
        if flush_interval < 0:
            raise ValueError("flush_interval must be non-negative")
        if max_bytes is not None and max_bytes <= 0:
            raise ValueError("max_bytes must be positive")
        self.path = Path(path)
        self.active_path = self.path
        self.paths = [self.active_path]
        self._metadata = metadata
        self._append = append
        self._flush_every = flush_every
        self._flush_interval = flush_interval
        self._max_bytes = max_bytes
        self._pending_flush = 0
        self._last_flush = time.monotonic()
        self._rotation_index = 0
        self.path.parent.mkdir(parents=True, exist_ok=True)
        mode = "a" if append else "w"
        self._handle = self.active_path.open(mode, encoding="utf-8")
        self._bytes_written = self.active_path.stat().st_size if append and self.active_path.exists() else 0
        if metadata:
            self.write("session_start", metadata=metadata)

    def write(self, event: str, **fields: Any) -> None:
        record = {
            "event": event,
            "time": time.time(),
            "time_iso": time.strftime("%Y-%m-%dT%H:%M:%S%z", time.localtime()),
            "monotonic": time.monotonic(),
        }
        record.update(fields)
        self._write_record(record)

    def flush(self) -> None:
        if not self._handle.closed:
            self._handle.flush()
            self._pending_flush = 0
            self._last_flush = time.monotonic()

    def _encode_record(self, record: dict[str, Any]) -> str:
        return json.dumps(record, sort_keys=True, separators=(",", ":")) + "\n"

    def _write_record(self, record: dict[str, Any]) -> None:
        line = self._encode_record(record)
        self._rotate_if_needed(len(line.encode("utf-8")))
        self._handle.write(line)
        self._bytes_written += len(line.encode("utf-8"))
        self._pending_flush += 1
        now = time.monotonic()
        if (
            self._pending_flush >= self._flush_every
            or (self._flush_interval and now - self._last_flush >= self._flush_interval)
        ):
            self.flush()

    def _rotate_if_needed(self, next_bytes: int) -> None:
        if self._max_bytes is None:
            return
        if self._bytes_written == 0 or self._bytes_written + next_bytes <= self._max_bytes:
            return
        self.flush()
        self._handle.close()
        self._rotation_index += 1
        self.active_path = self.path.with_name(
            "%s.%03d%s" % (self.path.stem, self._rotation_index, self.path.suffix)
        )
        self.paths.append(self.active_path)
        self._handle = self.active_path.open("w", encoding="utf-8")
        self._bytes_written = 0
        self._pending_flush = 0
        self._last_flush = time.monotonic()
        if self._metadata:
            self._write_record({
                "event": "session_start",
                "time": time.time(),
                "time_iso": time.strftime("%Y-%m-%dT%H:%M:%S%z", time.localtime()),
                "monotonic": time.monotonic(),
                "metadata": self._metadata,
                "rotation_index": self._rotation_index,
            })

    def close(self) -> None:
        if not self._handle.closed:
            self.write("session_end")
            self.flush()
            self._handle.close()

    def __enter__(self) -> "JsonlLogger":
        return self

    def __exit__(self, _exc_type, _exc, _tb) -> None:
        self.close()
