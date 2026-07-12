#!/usr/bin/env python3
"""Inspect pr0p input bindings for physical/virtual controller readiness."""

from __future__ import annotations

import argparse
import json
import re
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any


PASS = "PASS"
WAITING = "WAITING"
FAIL = "FAIL"
DEFAULT_LOG_DIR = Path("logs/simitl_pr0p")
DEFAULT_INPUT_CONFIG = Path.home() / ".config" / "unity3d" / "sigsegowl" / "pr0p" / "input.json"
DEFAULT_EXPECTED_DEVICE = "Kenet Game Sandbox"
PRIMARY_AXIS_SLOTS = ("roll", "pitch", "throttle", "yaw")
KENET_UINPUT_PROFILE = {
    "roll": {
        "evdev_axis": "ABS_X",
        "unity_controls": ("Stick/x", "x", "X"),
    },
    "pitch": {
        "evdev_axis": "ABS_Y",
        "unity_controls": ("Stick/y", "y", "Y"),
    },
    "throttle": {
        "evdev_axis": "ABS_RY",
        "unity_controls": ("RotateY", "RY", "Ry", "ry"),
    },
    "yaw": {
        "evdev_axis": "ABS_RX",
        "unity_controls": ("RotateX", "RX", "Rx", "rx"),
    },
}


@dataclass
class InputBinding:
    slot: int
    role: str
    binding_id: str | None
    path: str | None
    device: str | None
    control: str | None
    axis_flip: float | None

    def as_dict(self) -> dict[str, Any]:
        return {
            "slot": self.slot,
            "role": self.role,
            "binding_id": self.binding_id,
            "path": self.path,
            "device": self.device,
            "control": self.control,
            "axis_flip": self.axis_flip,
        }


@dataclass
class InputConfigResult:
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


def expected_controls_by_role() -> dict[str, str]:
    return {
        role: str(profile["unity_controls"][0])
        for role, profile in KENET_UINPUT_PROFILE.items()
    }


def parse_binding_path(path: str | None) -> tuple[str | None, str | None]:
    if not path:
        return None, None
    match = re.match(r"<(?P<device>[^>]+)>/(?P<control>.+)", path)
    if not match:
        return None, path
    return match.group("device"), match.group("control")


def load_binding_override(raw: str) -> dict[str, Any] | None:
    if not raw:
        return None
    try:
        data = json.loads(raw)
    except json.JSONDecodeError:
        return None
    return data if isinstance(data, dict) else None


def parse_input_bindings(data: dict[str, Any]) -> list[InputBinding]:
    overrides = data.get("bindingOverrides") or []
    axis_flips = data.get("axisFlips") or []
    bindings: list[InputBinding] = []
    for index, raw in enumerate(overrides):
        role = PRIMARY_AXIS_SLOTS[index] if index < len(PRIMARY_AXIS_SLOTS) else "axis_%d" % index
        parsed = load_binding_override(raw if isinstance(raw, str) else "")
        binding_id = None
        path = None
        if parsed:
            rows = parsed.get("bindings")
            if isinstance(rows, list) and rows and isinstance(rows[0], dict):
                binding_id = rows[0].get("id")
                path = rows[0].get("path")
        device, control = parse_binding_path(path)
        axis_flip = axis_flips[index] if index < len(axis_flips) else None
        bindings.append(InputBinding(
            slot=index,
            role=role,
            binding_id=binding_id,
            path=path,
            device=device,
            control=control,
            axis_flip=axis_flip,
        ))
    return bindings


def binding_matches_expected_device(binding: InputBinding, expected_device: str) -> bool:
    return bool(binding.device and expected_device.lower() in binding.device.lower())


def binding_matches_generic_uinput_profile(binding: InputBinding) -> bool:
    profile = KENET_UINPUT_PROFILE.get(binding.role)
    if not profile or not binding.device or not binding.control:
        return False
    if binding.device != "Joystick":
        return False
    return binding.control in profile["unity_controls"]


def binding_matches_virtual_profile(
    binding: InputBinding,
    *,
    expected_device: str,
    allow_generic_uinput_profile: bool,
) -> bool:
    return (
        binding_matches_expected_device(binding, expected_device)
        or (allow_generic_uinput_profile and binding_matches_generic_uinput_profile(binding))
    )


def build_controls_rebind_plan(
    bindings: list[InputBinding],
    *,
    expected_device: str,
    allow_generic_uinput_profile: bool,
) -> list[dict[str, Any]]:
    expected_controls = expected_controls_by_role()
    plan = []
    primary_by_role = {
        binding.role: binding
        for binding in bindings[:len(PRIMARY_AXIS_SLOTS)]
    }
    for role in PRIMARY_AXIS_SLOTS:
        binding = primary_by_role.get(role)
        if binding and binding_matches_virtual_profile(
            binding,
            expected_device=expected_device,
            allow_generic_uinput_profile=allow_generic_uinput_profile,
        ):
            continue
        plan.append({
            "role": role,
            "current_path": binding.path if binding else None,
            "current_device": binding.device if binding else None,
            "current_control": binding.control if binding else None,
            "expected_device": expected_device,
            "expected_generic_path": "<Joystick>/%s" % expected_controls[role],
            "expected_control": expected_controls[role],
            "controls_page": "Controls -> RC Channels",
        })
    return plan


def run_input_config_probe(
    *,
    input_config: Path,
    expected_device: str,
    require_virtual_mapping: bool,
    allow_generic_uinput_profile: bool,
) -> InputConfigResult:
    if not input_config.exists():
        return InputConfigResult(
            status=WAITING,
            summary="pr0p input config does not exist yet",
            metrics={"input_config": str(input_config)},
            notes=["PR0P_INPUT_CONFIG_MISSING"],
        )
    try:
        data = json.loads(input_config.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        return InputConfigResult(
            status=FAIL,
            summary="pr0p input config could not be parsed",
            metrics={"input_config": str(input_config), "error": str(exc)},
            notes=["PR0P_INPUT_CONFIG_PARSE_FAIL"],
        )
    if not isinstance(data, dict):
        return InputConfigResult(
            status=FAIL,
            summary="pr0p input config is not a JSON object",
            metrics={"input_config": str(input_config)},
            notes=["PR0P_INPUT_CONFIG_INVALID"],
        )
    bindings = parse_input_bindings(data)
    primary = bindings[:len(PRIMARY_AXIS_SLOTS)]
    primary_by_role = {binding.role: binding for binding in primary}
    missing_primary = [
        role
        for role in PRIMARY_AXIS_SLOTS
        if not primary_by_role.get(role) or not primary_by_role[role].path
    ]
    devices = sorted({binding.device for binding in bindings if binding.device})
    expected_matches = [binding.as_dict() for binding in primary if binding_matches_expected_device(binding, expected_device)]
    generic_profile_matches = [
        binding.as_dict()
        for binding in primary
        if allow_generic_uinput_profile and binding_matches_generic_uinput_profile(binding)
    ]
    virtual_profile_by_role = {}
    for role in PRIMARY_AXIS_SLOTS:
        binding = primary_by_role.get(role)
        virtual_profile_by_role[role] = bool(binding and binding_matches_virtual_profile(
            binding,
            expected_device=expected_device,
            allow_generic_uinput_profile=allow_generic_uinput_profile,
        ))
    controls_rebind_plan = build_controls_rebind_plan(
        bindings,
        expected_device=expected_device,
        allow_generic_uinput_profile=allow_generic_uinput_profile,
    )
    metrics = {
        "input_config": str(input_config),
        "expected_device": expected_device,
        "require_virtual_mapping": require_virtual_mapping,
        "allow_generic_uinput_profile": allow_generic_uinput_profile,
        "kenet_uinput_profile": KENET_UINPUT_PROFILE,
        "expected_virtual_controls_by_role": expected_controls_by_role(),
        "bindings": [binding.as_dict() for binding in bindings],
        "primary_axis_roles": list(PRIMARY_AXIS_SLOTS),
        "devices": devices,
        "missing_primary_roles": missing_primary,
        "expected_device_primary_matches": expected_matches,
        "generic_uinput_profile_matches": generic_profile_matches,
        "virtual_profile_by_role": virtual_profile_by_role,
        "controls_rebind_plan": controls_rebind_plan,
    }
    if missing_primary:
        return InputConfigResult(
            status=WAITING,
            summary="one or more primary pr0p input axes are not mapped",
            metrics=metrics,
            notes=[
                "PR0P_PRIMARY_AXIS_MAPPING_MISSING",
                "REMAP_ROLES:%s" % ",".join(missing_primary),
                "configure pr0p Controls -> RC Channels for the missing primary axes",
            ],
        )
    virtual_profile_ok = all(virtual_profile_by_role.get(role) for role in PRIMARY_AXIS_SLOTS)
    if require_virtual_mapping and not virtual_profile_ok:
        missing_roles = [row["role"] for row in controls_rebind_plan]
        return InputConfigResult(
            status=WAITING,
            summary="pr0p primary axes are mapped, but not to the expected virtual input profile",
            metrics=metrics,
            notes=[
                "PR0P_VIRTUAL_INPUT_MAPPING_MISSING",
                "REMAP_ROLES:%s" % ",".join(missing_roles),
                "configure pr0p Controls -> RC Channels for the virtual device or use the mapped physical controller",
            ],
        )
    return InputConfigResult(
        status=PASS,
        summary="pr0p primary input axes are mapped",
        metrics=metrics,
        notes=["virtual input mapping was required and found" if require_virtual_mapping else "physical/generic mapping present"],
    )


def build_markdown(result: InputConfigResult, *, run_id: str) -> str:
    lines = [
        "# SimITL / pr0p Input Config Probe",
        "",
        "Run: `%s`" % run_id,
        "Verdict: `%s`" % result.status,
        "",
        result.summary,
        "",
        "## Controls -> RC Channels",
        "",
        "| Role | Current | Expected virtual control |",
        "| --- | --- | --- |",
    ]
    rebind_plan = result.metrics.get("controls_rebind_plan") or []
    expected_controls = result.metrics.get("expected_virtual_controls_by_role") or {}
    bindings_by_role = {
        row.get("role"): row
        for row in result.metrics.get("bindings", [])
        if isinstance(row, dict) and row.get("role") in PRIMARY_AXIS_SLOTS
    }
    for role in PRIMARY_AXIS_SLOTS:
        current = "-"
        binding = bindings_by_role.get(role)
        if binding:
            current = binding.get("path") or "-"
        expected = expected_controls.get(role) or "-"
        needs_rebind = any(row.get("role") == role for row in rebind_plan)
        prefix = "REMAP: " if needs_rebind else "OK: "
        lines.append("| %s | `%s` | `%s%s` |" % (role, current, prefix, expected))
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


def write_reports(result: InputConfigResult, log_dir: Path, *, run_id: str) -> tuple[Path, Path]:
    log_dir.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime("%Y%m%d-%H%M%S", time.localtime())
    json_path = log_dir / ("%s-%s-input-config.json" % (stamp, run_id))
    md_path = log_dir / ("%s-%s-input-config.md" % (stamp, run_id))
    json_path.write_text(json.dumps(result.as_dict(), indent=2, sort_keys=True) + "\n",
                         encoding="utf-8")
    md_path.write_text(build_markdown(result, run_id=run_id), encoding="utf-8")
    return json_path, md_path


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input-config", type=Path, default=DEFAULT_INPUT_CONFIG)
    parser.add_argument("--expected-device", default=DEFAULT_EXPECTED_DEVICE)
    parser.add_argument("--allow-physical-mapping", action="store_true",
                        help="PASS when primary axes are mapped to any controller")
    parser.add_argument("--strict-device-name", action="store_true",
                        help="Require the expected device name, not only a generic Joystick profile")
    parser.add_argument("--log-dir", type=Path, default=DEFAULT_LOG_DIR)
    parser.add_argument("--run-id", default="pr0p-input-config")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    result = run_input_config_probe(
        input_config=args.input_config,
        expected_device=args.expected_device,
        require_virtual_mapping=not args.allow_physical_mapping,
        allow_generic_uinput_profile=not args.strict_device_name,
    )
    json_path, md_path = write_reports(result, args.log_dir, run_id=args.run_id)
    print("simitl-pr0p-input-config %s report=%s summary=%s" % (
        result.status,
        json_path,
        md_path,
    ))
    print(result.summary)
    return 1 if result.status == FAIL else 0


if __name__ == "__main__":
    raise SystemExit(main())
