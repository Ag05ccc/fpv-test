#!/usr/bin/env python3
"""Check that the isolated pr0p/game-screen tools do not depend on Gazebo paths."""

from __future__ import annotations

import argparse
import ast
import json
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

from pr0p_capture_probe import DEFAULT_LOG_DIR


PASS = "PASS"
FAIL = "FAIL"
DEFAULT_ROOTS = (
    Path("experiments/simitl_pr0p_probe"),
    Path("experiments/game_screen_sandbox"),
)
FORBIDDEN_IMPORT_PREFIXES = (
    "gazebo",
    "gazebo_",
    "gz_",
    "sitl_gz_",
    "gazebo_motor",
    "kenet_sitl_mixer",
    "run_gazebo_betaflight",
    "sitl_virtual_takeoff_check",
    "sitl_gazebo",
)
FORBIDDEN_STRING_FRAGMENTS = (
    "tools/run_gazebo_betaflight.sh",
    "run_gazebo_betaflight",
    "launch-fpv-sim.sh",
    "launch-physical-rc-sim.sh",
    "gazebo_motor_moment_probe",
    "sitl_gz_camera_probe",
    "sitl_virtual_takeoff_check",
)
ALLOWED_IMPORTS = (
    "sitl_log",
)
SELF_FILE = Path(__file__).name
STRING_LITERAL_RULE_FILES = {
    SELF_FILE,
    "isolation_audit.py",
}


@dataclass
class IsolationCheckResult:
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


def iter_python_files(roots: list[Path]) -> list[Path]:
    files: list[Path] = []
    for root in roots:
        if root.is_file() and root.suffix == ".py":
            files.append(root)
        elif root.exists():
            files.extend(sorted(root.rglob("*.py")))
    return sorted(set(files))


def module_root(module: str) -> str:
    return module.split(".", 1)[0]


def import_is_allowed(module: str) -> bool:
    return module in ALLOWED_IMPORTS or module_root(module) in ALLOWED_IMPORTS


def import_is_forbidden(module: str) -> bool:
    if import_is_allowed(module):
        return False
    root = module_root(module)
    return any(module.startswith(prefix) or root.startswith(prefix) for prefix in FORBIDDEN_IMPORT_PREFIXES)


def string_is_forbidden(value: str) -> str | None:
    for fragment in FORBIDDEN_STRING_FRAGMENTS:
        if fragment in value:
            return fragment
    return None


def scan_python_file(path: Path) -> list[dict[str, Any]]:
    violations: list[dict[str, Any]] = []
    check_string_literals = path.name not in STRING_LITERAL_RULE_FILES
    try:
        source = path.read_text(encoding="utf-8")
        tree = ast.parse(source, filename=str(path))
    except (OSError, SyntaxError) as exc:
        return [{
            "file": str(path),
            "line": getattr(exc, "lineno", None),
            "kind": "parse_error",
            "value": str(exc),
        }]
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            for alias in node.names:
                if import_is_forbidden(alias.name):
                    violations.append({
                        "file": str(path),
                        "line": node.lineno,
                        "kind": "forbidden_import",
                        "value": alias.name,
                    })
        elif isinstance(node, ast.ImportFrom):
            module = node.module or ""
            if module and import_is_forbidden(module):
                violations.append({
                    "file": str(path),
                    "line": node.lineno,
                    "kind": "forbidden_import",
                    "value": module,
                })
        elif check_string_literals and isinstance(node, ast.Constant) and isinstance(node.value, str):
            fragment = string_is_forbidden(node.value)
            if fragment:
                violations.append({
                    "file": str(path),
                    "line": getattr(node, "lineno", None),
                    "kind": "forbidden_string",
                    "value": fragment,
                })
    return violations


def run_isolation_check(*, roots: list[Path]) -> IsolationCheckResult:
    files = iter_python_files(roots)
    violations: list[dict[str, Any]] = []
    for path in files:
        violations.extend(scan_python_file(path))
    metrics = {
        "roots": [str(root) for root in roots],
        "python_files": [str(path) for path in files],
        "file_count": len(files),
        "forbidden_import_prefixes": list(FORBIDDEN_IMPORT_PREFIXES),
        "forbidden_string_fragments": list(FORBIDDEN_STRING_FRAGMENTS),
        "allowed_imports": list(ALLOWED_IMPORTS),
        "violations": violations,
    }
    if violations:
        return IsolationCheckResult(
            status=FAIL,
            summary="isolated pr0p/game-screen code references forbidden Gazebo/SITL paths",
            metrics=metrics,
            notes=["GAZEBO_COUPLING_DETECTED"],
        )
    return IsolationCheckResult(
        status=PASS,
        summary="isolated pr0p/game-screen code has no forbidden Gazebo runtime coupling",
        metrics=metrics,
        notes=["GAZEBO_INDEPENDENCE_VERIFIED"],
    )


def build_markdown(result: IsolationCheckResult, *, run_id: str) -> str:
    lines = [
        "# SimITL / pr0p Isolation Check",
        "",
        "Run: `%s`" % run_id,
        "Verdict: `%s`" % result.status,
        "",
        result.summary,
        "",
        "| Metric | Value |",
        "| --- | --- |",
        "| file_count | `%s` |" % result.metrics.get("file_count"),
        "| violations | `%s` |" % len(result.metrics.get("violations", [])),
        "",
        "```json",
        json.dumps(result.metrics, indent=2, sort_keys=True),
        "```",
    ]
    for note in result.notes:
        lines.append("- %s" % note)
    lines.append("")
    return "\n".join(lines)


def write_reports(result: IsolationCheckResult, log_dir: Path, *, run_id: str) -> tuple[Path, Path]:
    log_dir.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime("%Y%m%d-%H%M%S", time.localtime())
    json_path = log_dir / ("%s-%s-isolation-check.json" % (stamp, run_id))
    md_path = log_dir / ("%s-%s-isolation-check.md" % (stamp, run_id))
    json_path.write_text(json.dumps(result.as_dict(), indent=2, sort_keys=True) + "\n",
                         encoding="utf-8")
    md_path.write_text(build_markdown(result, run_id=run_id), encoding="utf-8")
    return json_path, md_path


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", action="append", type=Path, dest="roots")
    parser.add_argument("--log-dir", type=Path, default=DEFAULT_LOG_DIR)
    parser.add_argument("--run-id", default="pr0p-isolation")
    args = parser.parse_args(argv)
    if args.roots is None:
        args.roots = list(DEFAULT_ROOTS)
    return args


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    result = run_isolation_check(roots=args.roots)
    json_path, md_path = write_reports(result, args.log_dir, run_id=args.run_id)
    print("simitl-pr0p-isolation %s report=%s summary=%s" % (
        result.status,
        json_path,
        md_path,
    ))
    print(result.summary)
    return 1 if result.status == FAIL else 0


if __name__ == "__main__":
    raise SystemExit(main())
