#!/usr/bin/env python3
"""Dry-run or restore pr0p input.json from a Codex-created backup."""

from __future__ import annotations

import argparse
import json
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

PROBE_DIR = Path(__file__).resolve().parent
if str(PROBE_DIR) not in sys.path:
    sys.path.insert(0, str(PROBE_DIR))

from pr0p_input_config_probe import (  # noqa: E402
    DEFAULT_INPUT_CONFIG,
    DEFAULT_LOG_DIR,
    FAIL,
    PASS,
    WAITING,
    run_input_config_probe,
)


@dataclass
class InputConfigRestoreResult:
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


def backup_prefix(input_config: Path) -> str:
    return "%s.codex-backup-" % input_config.name


def current_snapshot_prefix(input_config: Path) -> str:
    return "%s.codex-restore-current-" % input_config.name


def backup_sort_key(path: Path) -> tuple[int, str]:
    try:
        mtime_ns = path.stat().st_mtime_ns
    except OSError:
        mtime_ns = -1
    return mtime_ns, path.name


def list_backups(input_config: Path) -> list[Path]:
    if not input_config.parent.exists():
        return []
    return sorted(
        input_config.parent.glob("%s*" % backup_prefix(input_config)),
        key=backup_sort_key,
        reverse=True,
    )


def is_backup_for_input_config(input_config: Path, backup: Path) -> bool:
    try:
        input_parent = input_config.expanduser().resolve().parent
        backup_path = backup.expanduser().resolve()
    except OSError:
        return False
    return backup_path.parent == input_parent and backup_path.name.startswith(backup_prefix(input_config))


def load_json_object(path: Path) -> tuple[dict[str, Any] | None, str | None]:
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        return None, str(exc)
    if not isinstance(data, dict):
        return None, "JSON root is not an object"
    return data, None


def run_input_config_restore(
    *,
    input_config: Path,
    log_dir: Path,
    backup: Path | None,
    restore: bool,
    expected_device: str,
) -> InputConfigRestoreResult:
    backups = list_backups(input_config)
    selected = backup.expanduser() if backup else (backups[0] if backups else None)
    metrics: dict[str, Any] = {
        "input_config": str(input_config),
        "backup_count": len(backups),
        "backups": [str(path) for path in backups],
        "selected_backup": str(selected) if selected else None,
        "restore_requested": restore,
        "real_config_write": False,
        "current_snapshot_path": None,
    }
    if selected is None:
        return InputConfigRestoreResult(
            status=WAITING,
            summary="no Codex pr0p input config backup is available yet",
            metrics=metrics,
            notes=["PR0P_INPUT_CONFIG_BACKUP_MISSING"],
        )
    if not is_backup_for_input_config(input_config, selected):
        return InputConfigRestoreResult(
            status=FAIL,
            summary="selected backup is not a Codex backup for this input config",
            metrics=metrics,
            notes=["PR0P_INPUT_CONFIG_BACKUP_INVALID"],
        )
    if not selected.exists():
        return InputConfigRestoreResult(
            status=WAITING,
            summary="selected pr0p input config backup does not exist",
            metrics=metrics,
            notes=["PR0P_INPUT_CONFIG_BACKUP_MISSING"],
        )
    backup_data, backup_error = load_json_object(selected)
    if backup_error:
        metrics["backup_error"] = backup_error
        return InputConfigRestoreResult(
            status=FAIL,
            summary="selected pr0p input config backup could not be parsed",
            metrics=metrics,
            notes=["PR0P_INPUT_CONFIG_BACKUP_PARSE_FAIL"],
        )
    assert backup_data is not None

    log_dir.mkdir(parents=True, exist_ok=True)
    temp_config = log_dir / ("_tmp-%s-input-config-restore.json" % int(time.time() * 1000))
    try:
        temp_config.write_text(json.dumps(backup_data, separators=(",", ":"), ensure_ascii=False),
                               encoding="utf-8")
        backup_probe = run_input_config_probe(
            input_config=temp_config,
            expected_device=expected_device,
            require_virtual_mapping=False,
            allow_generic_uinput_profile=True,
        )
    finally:
        try:
            temp_config.unlink()
        except OSError:
            pass
    metrics["backup_probe_status"] = backup_probe.status
    metrics["backup_probe_notes"] = backup_probe.notes
    metrics["backup_probe_bindings"] = backup_probe.metrics.get("bindings")
    if backup_probe.status == FAIL:
        return InputConfigRestoreResult(
            status=FAIL,
            summary="selected pr0p input config backup did not pass parse/probe validation",
            metrics=metrics,
            notes=["PR0P_INPUT_CONFIG_BACKUP_VERIFY_FAIL"],
        )

    if not restore:
        return InputConfigRestoreResult(
            status=PASS,
            summary="dry-run restore is ready",
            metrics=metrics,
            notes=["DRY_RUN_ONLY", "rerun with --restore --ack-config-restore to apply"],
        )

    if input_config.exists():
        stamp = time.strftime("%Y%m%d-%H%M%S", time.localtime())
        current_snapshot = input_config.with_name("%s%s" % (
            current_snapshot_prefix(input_config),
            stamp,
        ))
        current_snapshot.write_text(input_config.read_text(encoding="utf-8"), encoding="utf-8")
        metrics["current_snapshot_path"] = str(current_snapshot)
    input_config.parent.mkdir(parents=True, exist_ok=True)
    input_config.write_text(selected.read_text(encoding="utf-8"), encoding="utf-8")
    metrics["real_config_write"] = True
    return InputConfigRestoreResult(
        status=PASS,
        summary="pr0p input config was restored from backup",
        metrics=metrics,
        notes=["PR0P_INPUT_CONFIG_RESTORED", "CURRENT_SNAPSHOT_WRITTEN"],
    )


def build_markdown(result: InputConfigRestoreResult, *, run_id: str) -> str:
    lines = [
        "# SimITL / pr0p Input Config Restore",
        "",
        "Run: `%s`" % run_id,
        "Verdict: `%s`" % result.status,
        "",
        result.summary,
        "",
        "| Field | Value |",
        "| --- | --- |",
        "| input_config | `%s` |" % result.metrics.get("input_config"),
        "| selected_backup | `%s` |" % (result.metrics.get("selected_backup") or "-"),
        "| backup_count | `%s` |" % result.metrics.get("backup_count"),
        "| real_config_write | `%s` |" % result.metrics.get("real_config_write"),
        "| current_snapshot_path | `%s` |" % (result.metrics.get("current_snapshot_path") or "-"),
        "",
        "```json",
        json.dumps(result.metrics, indent=2, sort_keys=True),
        "```",
    ]
    for note in result.notes:
        lines.append("- %s" % note)
    lines.append("")
    return "\n".join(lines)


def write_reports(result: InputConfigRestoreResult, log_dir: Path, *, run_id: str) -> tuple[Path, Path]:
    log_dir.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime("%Y%m%d-%H%M%S", time.localtime())
    json_path = log_dir / ("%s-%s-input-config-restore.json" % (stamp, run_id))
    md_path = log_dir / ("%s-%s-input-config-restore.md" % (stamp, run_id))
    json_path.write_text(json.dumps(result.as_dict(), indent=2, sort_keys=True) + "\n",
                         encoding="utf-8")
    md_path.write_text(build_markdown(result, run_id=run_id), encoding="utf-8")
    return json_path, md_path


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input-config", type=Path, default=DEFAULT_INPUT_CONFIG)
    parser.add_argument("--log-dir", type=Path, default=DEFAULT_LOG_DIR)
    parser.add_argument("--run-id", default="pr0p-input-config-restore")
    parser.add_argument("--backup", type=Path,
                        help="Specific input.json.codex-backup-* path; default uses latest")
    parser.add_argument("--expected-device", default="Kenet Game Sandbox")
    parser.add_argument("--restore", action="store_true",
                        help="Restore selected backup into pr0p input.json")
    parser.add_argument("--ack-config-restore", action="store_true",
                        help="Required with --restore; current input.json is snapshotted first")
    args = parser.parse_args(argv)
    if args.restore and not args.ack_config_restore:
        parser.error("--restore requires --ack-config-restore")
    return args


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    result = run_input_config_restore(
        input_config=args.input_config,
        log_dir=args.log_dir,
        backup=args.backup,
        restore=args.restore,
        expected_device=args.expected_device,
    )
    json_path, md_path = write_reports(result, args.log_dir, run_id=args.run_id)
    print("simitl-pr0p-input-config-restore %s report=%s summary=%s real_config_write=%s" % (
        result.status,
        json_path,
        md_path,
        result.metrics.get("real_config_write"),
    ))
    print(result.summary)
    return 1 if result.status == FAIL else 0


if __name__ == "__main__":
    raise SystemExit(main())
