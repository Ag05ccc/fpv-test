#!/usr/bin/env python3
"""Isolation checks for the game/screen sandbox."""

from __future__ import annotations

import argparse
import ast
import json
import subprocess
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any


PASS = "PASS"
FAIL = "FAIL"
DEFAULT_FORBIDDEN_IMPORT_PREFIXES = ("gazebo", "gz", "betaflight")
DEFAULT_FORBIDDEN_PROCESS_PATTERN = (
    r"(^|/|\s)(gzserver|gzclient|gazebo|betaflight|run_gazebo_betaflight)(\s|$)"
)


@dataclass
class IsolationAuditResult:
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


def sandbox_python_files(root: Path) -> list[Path]:
    return sorted(path for path in root.glob("*.py") if path.is_file())


def imported_module_names(tree: ast.AST) -> list[str]:
    names: list[str] = []
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            names.extend(alias.name for alias in node.names)
        elif isinstance(node, ast.ImportFrom) and node.module:
            names.append(node.module)
    return names


def is_forbidden_import(module_name: str, prefixes: tuple[str, ...]) -> bool:
    return any(
        module_name == prefix or module_name.startswith(prefix + ".")
        for prefix in prefixes
    )


def audit_static_imports(
    root: Path,
    *,
    forbidden_prefixes: tuple[str, ...] = DEFAULT_FORBIDDEN_IMPORT_PREFIXES,
) -> IsolationAuditResult:
    files = sandbox_python_files(root)
    forbidden: list[dict[str, str]] = []
    parse_errors: list[dict[str, str]] = []
    for path in files:
        try:
            tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
        except SyntaxError as exc:
            parse_errors.append({"path": str(path), "error": str(exc)})
            continue
        for module_name in imported_module_names(tree):
            if is_forbidden_import(module_name, forbidden_prefixes):
                forbidden.append({"path": str(path), "module": module_name})

    notes: list[str] = []
    if parse_errors:
        notes.append("PYTHON_PARSE_ERRORS")
    if forbidden:
        notes.append("FORBIDDEN_IMPORTS")
    status = PASS if not notes else FAIL
    summary = (
        "sandbox Python imports do not depend on Gazebo/Betaflight modules"
        if status == PASS else
        "sandbox Python import boundary is not isolated"
    )
    return IsolationAuditResult(
        status=status,
        summary=summary,
        metrics={
            "root": str(root),
            "files_checked": [str(path) for path in files],
            "forbidden_import_prefixes": list(forbidden_prefixes),
            "forbidden_imports": forbidden,
            "parse_errors": parse_errors,
        },
        notes=notes,
    )


def process_snapshot(pattern: str = DEFAULT_FORBIDDEN_PROCESS_PATTERN) -> list[str]:
    result = subprocess.run(
        ["pgrep", "-af", pattern],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        check=False,
    )
    if result.returncode not in (0, 1):
        raise RuntimeError("pgrep failed: %s" % result.stderr.strip())
    return [line.strip() for line in result.stdout.splitlines() if line.strip()]


def process_ids(snapshot: list[str]) -> set[str]:
    ids: set[str] = set()
    for line in snapshot:
        parts = line.split(maxsplit=1)
        if parts:
            ids.add(parts[0])
    return ids


def new_processes(before: list[str], after: list[str]) -> list[str]:
    before_ids = process_ids(before)
    return [
        line for line in after
        if line.split(maxsplit=1)[0] not in before_ids
    ]


def audit_process_delta(
    before: list[str],
    after: list[str],
) -> IsolationAuditResult:
    new_items = new_processes(before, after)
    status = PASS if not new_items else FAIL
    summary = (
        "acceptance run did not launch Gazebo/Betaflight processes"
        if status == PASS else
        "acceptance run launched forbidden Gazebo/Betaflight processes"
    )
    return IsolationAuditResult(
        status=status,
        summary=summary,
        metrics={
            "forbidden_processes_before": before,
            "forbidden_processes_after": after,
            "new_forbidden_processes": new_items,
            "forbidden_process_pattern": DEFAULT_FORBIDDEN_PROCESS_PATTERN,
        },
        notes=[] if status == PASS else ["FORBIDDEN_PROCESS_STARTED"],
    )


def combine_audits(*audits: IsolationAuditResult) -> IsolationAuditResult:
    failed = [audit for audit in audits if audit.status != PASS]
    metrics = {
        "audits": [audit.as_dict() for audit in audits],
    }
    notes: list[str] = []
    for audit in failed:
        notes.extend(audit.notes)
    status = PASS if not failed else FAIL
    summary = (
        "sandbox isolation audit passed"
        if status == PASS else
        "sandbox isolation audit failed"
    )
    return IsolationAuditResult(
        status=status,
        summary=summary,
        metrics=metrics,
        notes=notes,
    )


def run_audit(root: Path) -> IsolationAuditResult:
    static = audit_static_imports(root)
    process = audit_process_delta([], process_snapshot())
    return combine_audits(static, process)


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=Path(__file__).resolve().parent)
    parser.add_argument("--json", action="store_true")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    result = run_audit(args.root)
    if args.json:
        print(json.dumps(result.as_dict(), indent=2, sort_keys=True))
    else:
        print("isolation-audit %s %s" % (result.status, result.summary))
    return 0 if result.status == PASS else 1


if __name__ == "__main__":
    raise SystemExit(main())
