#!/usr/bin/env python3
"""Prepare or apply a reversible pr0p input.json virtual RC mapping patch."""

from __future__ import annotations

import argparse
import json
import sys
import time
from copy import deepcopy
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
    PRIMARY_AXIS_SLOTS,
    WAITING,
    expected_controls_by_role,
    load_binding_override,
    parse_input_bindings,
    run_input_config_probe,
)


TARGET_DEVICE = "Joystick"
AUX_ROLE_INDEXES = {
    "aux1": 4,
}
AUX_CONTROLS_BY_ROLE = {
    "aux1": "Z",
}


@dataclass
class InputConfigPatchResult:
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


def expand_roles(roles: list[str]) -> list[str]:
    expanded: list[str] = []
    for role in roles:
        values = list(PRIMARY_AXIS_SLOTS) if role == "all" else [role]
        for value in values:
            if value not in expanded:
                expanded.append(value)
    return expanded


def target_paths_by_role(*, target_device: str = TARGET_DEVICE) -> dict[str, str]:
    controls = {**expected_controls_by_role(), **AUX_CONTROLS_BY_ROLE}
    return {
        role: "<%s>/%s" % (target_device, control)
        for role, control in controls.items()
    }


def default_axis_flips() -> dict[str, float]:
    return {
        "roll": 1.0,
        "pitch": -1.0,
        "throttle": 1.0,
        "yaw": 1.0,
        "aux1": 1.0,
    }


def ensure_list_size(values: list[Any], size: int, fill: Any) -> list[Any]:
    result = list(values)
    while len(result) < size:
        result.append(fill)
    return result


def binding_override_with_path(raw: str | None, *, binding_id: str, path: str) -> str:
    parsed = load_binding_override(raw or "") or {}
    rows = parsed.get("bindings")
    if not isinstance(rows, list) or not rows:
        rows = [{}]
    first = rows[0] if isinstance(rows[0], dict) else {}
    first = dict(first)
    first.setdefault("action", "")
    first["id"] = first.get("id") or binding_id
    first["path"] = path
    first.setdefault("interactions", "null")
    first.setdefault("processors", "null")
    rows[0] = first
    parsed["bindings"] = rows
    return json.dumps(parsed, separators=(",", ":"))


def patch_input_config_data(
    data: dict[str, Any],
    *,
    roles: list[str],
    target_device: str = TARGET_DEVICE,
) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    patched = deepcopy(data)
    role_sequence = expand_roles(roles)
    role_indexes = {
        **{role: index for index, role in enumerate(PRIMARY_AXIS_SLOTS)},
        **AUX_ROLE_INDEXES,
    }
    targets = target_paths_by_role(target_device=target_device)
    flips = default_axis_flips()
    max_slot = max(role_indexes[role] for role in role_sequence) + 1

    overrides = patched.get("bindingOverrides")
    if not isinstance(overrides, list):
        overrides = []
    overrides = ensure_list_size(overrides, max_slot, "")

    axis_flips = patched.get("axisFlips")
    if not isinstance(axis_flips, list):
        axis_flips = []
    axis_flips = ensure_list_size(axis_flips, max_slot, None)

    guids = patched.get("guids")
    if not isinstance(guids, list):
        guids = []
    guids = ensure_list_size(guids, max_slot, "")

    original_by_slot = {binding.slot: binding for binding in parse_input_bindings(data)}
    changes: list[dict[str, Any]] = []
    for role in role_sequence:
        index = role_indexes[role]
        target_path = targets[role]
        current_path = original_by_slot.get(index).path if index in original_by_slot else None
        binding_id = ""
        parsed = load_binding_override(overrides[index] if isinstance(overrides[index], str) else "")
        if parsed:
            rows = parsed.get("bindings")
            if isinstance(rows, list) and rows and isinstance(rows[0], dict):
                binding_id = str(rows[0].get("id") or "")
        binding_id = binding_id or str(guids[index] or "kenet-virtual-%s" % role)
        overrides[index] = binding_override_with_path(
            overrides[index] if isinstance(overrides[index], str) else "",
            binding_id=binding_id,
            path=target_path,
        )
        if axis_flips[index] is None:
            axis_flips[index] = flips[role]
        changed = current_path != target_path
        changes.append({
            "role": role,
            "slot": index,
            "old_path": current_path,
            "new_path": target_path,
            "changed": changed,
            "binding_id": binding_id,
        })

    patched["bindingOverrides"] = overrides
    patched["axisFlips"] = axis_flips
    if "guids" in patched:
        patched["guids"] = guids
    return patched, changes


def compact_json(data: dict[str, Any]) -> str:
    return json.dumps(data, separators=(",", ":"), ensure_ascii=False)


def probe_data_with_temp_config(
    data: dict[str, Any],
    *,
    temp_config: Path,
    expected_device: str,
    require_virtual_mapping: bool,
) -> dict[str, Any]:
    temp_config.write_text(compact_json(data), encoding="utf-8")
    result = run_input_config_probe(
        input_config=temp_config,
        expected_device=expected_device,
        require_virtual_mapping=require_virtual_mapping,
        allow_generic_uinput_profile=True,
    )
    return result.as_dict()


def run_input_config_patch(
    *,
    input_config: Path,
    log_dir: Path,
    roles: list[str],
    expected_device: str,
    target_device: str,
    write: bool,
) -> InputConfigPatchResult:
    if not input_config.exists():
        return InputConfigPatchResult(
            status=WAITING,
            summary="pr0p input config does not exist yet",
            metrics={"input_config": str(input_config), "real_config_write": False},
            notes=["PR0P_INPUT_CONFIG_MISSING"],
        )
    try:
        original_text = input_config.read_text(encoding="utf-8")
        original = json.loads(original_text)
    except (OSError, json.JSONDecodeError) as exc:
        return InputConfigPatchResult(
            status=FAIL,
            summary="pr0p input config could not be parsed",
            metrics={"input_config": str(input_config), "error": str(exc), "real_config_write": False},
            notes=["PR0P_INPUT_CONFIG_PARSE_FAIL"],
        )
    if not isinstance(original, dict):
        return InputConfigPatchResult(
            status=FAIL,
            summary="pr0p input config is not a JSON object",
            metrics={"input_config": str(input_config), "real_config_write": False},
            notes=["PR0P_INPUT_CONFIG_INVALID"],
        )

    role_sequence = expand_roles(roles)
    patched, changes = patch_input_config_data(
        original,
        roles=role_sequence,
        target_device=target_device,
    )
    log_dir.mkdir(parents=True, exist_ok=True)
    temp_config = log_dir / ("_tmp-%s-input-config-patch.json" % int(time.time() * 1000))
    require_primary_virtual = any(role in PRIMARY_AXIS_SLOTS for role in role_sequence)
    try:
        patched_probe = probe_data_with_temp_config(
            patched,
            temp_config=temp_config,
            expected_device=expected_device,
            require_virtual_mapping=require_primary_virtual,
        )
    finally:
        try:
            temp_config.unlink()
        except OSError:
            pass

    changed_roles = [change["role"] for change in changes if change["changed"]]
    backup_path = None
    write_performed = False
    if write and changed_roles:
        stamp = time.strftime("%Y%m%d-%H%M%S", time.localtime())
        backup = input_config.with_name("%s.codex-backup-%s" % (input_config.name, stamp))
        backup.write_text(original_text, encoding="utf-8")
        input_config.write_text(compact_json(patched), encoding="utf-8")
        backup_path = str(backup)
        write_performed = True

    metrics = {
        "input_config": str(input_config),
        "expected_device": expected_device,
        "target_device": target_device,
        "roles": role_sequence,
        "target_paths_by_role": target_paths_by_role(target_device=target_device),
        "path_changes": changes,
        "changed_roles": changed_roles,
        "patched_probe_status": patched_probe.get("status"),
        "patched_probe_notes": patched_probe.get("notes"),
        "write_requested": write,
        "real_config_write": write_performed,
        "backup_path": backup_path,
    }
    if patched_probe.get("status") != PASS:
        return InputConfigPatchResult(
            status=FAIL,
            summary="proposed pr0p input config patch did not verify as virtual mapping",
            metrics=metrics,
            notes=["PR0P_INPUT_CONFIG_PATCH_VERIFY_FAIL"],
        )
    if not changed_roles:
        return InputConfigPatchResult(
            status=PASS,
            summary="pr0p input config already matches the virtual RC profile",
            metrics=metrics,
            notes=["PR0P_INPUT_CONFIG_ALREADY_VIRTUAL"],
        )
    if write_performed:
        return InputConfigPatchResult(
            status=PASS,
            summary="pr0p input config was patched and a backup was written",
            metrics=metrics,
            notes=["PR0P_INPUT_CONFIG_PATCHED", "BACKUP_WRITTEN"],
        )
    return InputConfigPatchResult(
        status=PASS,
        summary="dry-run pr0p input config patch is ready",
        metrics=metrics,
        notes=["DRY_RUN_ONLY", "rerun with --write --ack-config-write to apply"],
    )


def build_markdown(result: InputConfigPatchResult, *, run_id: str) -> str:
    lines = [
        "# SimITL / pr0p Input Config Patch",
        "",
        "Run: `%s`" % run_id,
        "Verdict: `%s`" % result.status,
        "",
        result.summary,
        "",
        "| Role | Current | Proposed | Changed |",
        "| --- | --- | --- | --- |",
    ]
    for change in result.metrics.get("path_changes", []):
        lines.append("| %s | `%s` | `%s` | `%s` |" % (
            change.get("role"),
            change.get("old_path") or "-",
            change.get("new_path") or "-",
            change.get("changed"),
        ))
    lines.extend([
        "",
        "```json",
        json.dumps(result.metrics, indent=2, sort_keys=True),
        "```",
    ])
    for note in result.notes:
        lines.append("- %s" % note)
    lines.append("")
    return "\n".join(lines)


def write_reports(result: InputConfigPatchResult, log_dir: Path, *, run_id: str) -> tuple[Path, Path]:
    log_dir.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime("%Y%m%d-%H%M%S", time.localtime())
    json_path = log_dir / ("%s-%s-input-config-patch.json" % (stamp, run_id))
    md_path = log_dir / ("%s-%s-input-config-patch.md" % (stamp, run_id))
    json_path.write_text(json.dumps(result.as_dict(), indent=2, sort_keys=True) + "\n",
                         encoding="utf-8")
    md_path.write_text(build_markdown(result, run_id=run_id), encoding="utf-8")
    return json_path, md_path


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input-config", type=Path, default=DEFAULT_INPUT_CONFIG)
    parser.add_argument("--log-dir", type=Path, default=DEFAULT_LOG_DIR)
    parser.add_argument("--run-id", default="pr0p-input-config-patch")
    parser.add_argument("--role", action="append", choices=("all",) + PRIMARY_AXIS_SLOTS + tuple(AUX_ROLE_INDEXES))
    parser.add_argument("--expected-device", default="Kenet Game Sandbox")
    parser.add_argument("--target-device", default=TARGET_DEVICE,
                        help="Unity device name to write in binding paths; default accepts generic Joystick")
    parser.add_argument("--write", action="store_true",
                        help="Apply the patch to pr0p input.json")
    parser.add_argument("--ack-config-write", action="store_true",
                        help="Required with --write; a backup is written first")
    args = parser.parse_args(argv)
    if args.role is None:
        args.role = ["all"]
    if args.write and not args.ack_config_write:
        parser.error("--write requires --ack-config-write")
    return args


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    result = run_input_config_patch(
        input_config=args.input_config,
        log_dir=args.log_dir,
        roles=args.role,
        expected_device=args.expected_device,
        target_device=args.target_device,
        write=args.write,
    )
    json_path, md_path = write_reports(result, args.log_dir, run_id=args.run_id)
    print("simitl-pr0p-input-config-patch %s report=%s summary=%s real_config_write=%s" % (
        result.status,
        json_path,
        md_path,
        result.metrics.get("real_config_write"),
    ))
    print(result.summary)
    return 1 if result.status == FAIL else 0


if __name__ == "__main__":
    raise SystemExit(main())
