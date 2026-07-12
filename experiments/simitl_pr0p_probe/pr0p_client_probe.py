#!/usr/bin/env python3
"""Check whether an isolated pr0p client executable is installed."""

from __future__ import annotations

import argparse
import json
import os
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

PROBE_DIR = Path(__file__).resolve().parent
if str(PROBE_DIR) not in sys.path:
    sys.path.insert(0, str(PROBE_DIR))

from preflight_probe import DEFAULT_INSTALL_ROOT, DEFAULT_LOG_DIR  # noqa: E402
from pr0p_install_probe import is_allowed_install_root, scan_install_root  # noqa: E402


PASS = "PASS"
WAITING = "WAITING"
FAIL = "FAIL"
DEFAULT_CLIENT_EXECUTABLE_NAMES = ("pr0p.x86_64", "pr0p")


@dataclass
class ClientProbeResult:
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


def executable_row(path: Path, *, install_root: Path) -> dict[str, Any] | None:
    try:
        stat = path.stat()
    except OSError:
        return None
    return {
        "name": path.name,
        "path": str(path),
        "relative_path": str(path.relative_to(install_root)),
        "size_bytes": stat.st_size,
        "mode": oct(stat.st_mode & 0o777),
        "executable": os.access(path, os.X_OK),
    }


def candidate_score(row: dict[str, Any]) -> tuple[int, int, str]:
    name = str(row.get("name", "")).lower()
    relative_path = str(row.get("relative_path", ""))
    if name in DEFAULT_CLIENT_EXECUTABLE_NAMES:
        rank = 0
    elif "pr0p" in name:
        rank = 1
    elif "prop" in name:
        rank = 2
    else:
        rank = 3
    return rank, relative_path.count(os.sep), relative_path


def find_client_candidates(install_root: Path, *, max_depth: int) -> list[dict[str, Any]]:
    if not install_root.exists():
        return []
    candidates: list[dict[str, Any]] = []
    for path in sorted(install_root.rglob("*")):
        try:
            relative_parts = path.relative_to(install_root).parts
        except ValueError:
            continue
        if len(relative_parts) > max_depth or not path.is_file() or not os.access(path, os.X_OK):
            continue
        name = path.name.lower()
        if name == "updater":
            continue
        if name not in DEFAULT_CLIENT_EXECUTABLE_NAMES and "pr0p" not in name and "prop" not in name:
            continue
        row = executable_row(path, install_root=install_root)
        if row:
            candidates.append(row)
    return sorted(candidates, key=candidate_score)


def run_client_probe(
    *,
    install_root: Path,
    max_depth: int,
    run_id: str,
) -> ClientProbeResult:
    if not is_allowed_install_root(install_root):
        return ClientProbeResult(
            status=FAIL,
            summary="install root is outside the allowed isolated roots",
            metrics={"install_root": str(install_root)},
            notes=["REFUSE_NON_ISOLATED_INSTALL_ROOT"],
        )
    if not install_root.exists():
        return ClientProbeResult(
            status=WAITING,
            summary="isolated install root does not exist yet",
            metrics={
                "run_id": run_id,
                "install_root": str(install_root),
                "max_depth": max_depth,
                "client_candidates": [],
                "install_scan": {"exists": False},
            },
            notes=["RUN_INSTALL_DISCOVERY_FIRST"],
        )
    candidates = find_client_candidates(install_root, max_depth=max_depth)
    contents = sorted(item.name for item in install_root.iterdir())[:30]
    metrics = {
        "run_id": run_id,
        "install_root": str(install_root),
        "max_depth": max_depth,
        "install_root_contents": contents,
        "install_scan": scan_install_root(install_root),
        "client_candidates": candidates,
        "selected_client": candidates[0] if candidates else None,
    }
    if candidates:
        return ClientProbeResult(
            status=PASS,
            summary="pr0p client executable is visible in the isolated root",
            metrics=metrics,
            notes=[],
        )
    return ClientProbeResult(
        status=WAITING,
        summary="pr0p client executable is not installed in the isolated root",
        metrics=metrics,
        notes=["run the downloaded updater manually, then rerun this probe"],
    )


def build_markdown(result: ClientProbeResult, *, run_id: str) -> str:
    lines = [
        "# SimITL / pr0p Client Probe",
        "",
        "Run: `%s`" % run_id,
        "Verdict: `%s`" % result.status,
        "",
        result.summary,
        "",
        "| Metric | Value |",
        "| --- | --- |",
        "| install_root | `%s` |" % result.metrics.get("install_root"),
        "| candidate_count | `%s` |" % len(result.metrics.get("client_candidates") or []),
    ]
    selected = result.metrics.get("selected_client")
    if selected:
        lines.append("| selected_client | `%s` |" % selected.get("path"))
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


def write_reports(result: ClientProbeResult, log_dir: Path, *, run_id: str) -> tuple[Path, Path]:
    log_dir.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime("%Y%m%d-%H%M%S", time.localtime())
    json_path = log_dir / ("%s-%s-client-probe.json" % (stamp, run_id))
    md_path = log_dir / ("%s-%s-client-probe.md" % (stamp, run_id))
    json_path.write_text(
        json.dumps(result.as_dict(), indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    md_path.write_text(build_markdown(result, run_id=run_id), encoding="utf-8")
    return json_path, md_path


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-id", default="pr0p-client")
    parser.add_argument("--install-root", type=Path, default=DEFAULT_INSTALL_ROOT)
    parser.add_argument("--log-dir", type=Path, default=DEFAULT_LOG_DIR)
    parser.add_argument("--max-depth", type=int, default=3)
    args = parser.parse_args(argv)
    if args.max_depth < 1:
        parser.error("--max-depth must be at least 1")
    return args


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    result = run_client_probe(
        install_root=args.install_root,
        max_depth=args.max_depth,
        run_id=args.run_id,
    )
    json_path, md_path = write_reports(result, args.log_dir, run_id=args.run_id)
    print("simitl-pr0p-client %s report=%s summary=%s" % (
        result.status,
        json_path,
        md_path,
    ))
    print(result.summary)
    return 1 if result.status == FAIL else 0


if __name__ == "__main__":
    raise SystemExit(main())
