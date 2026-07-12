#!/usr/bin/env python3
"""Discover/download the pr0p Linux updater into an isolated install root."""

from __future__ import annotations

import argparse
import hashlib
import html
import json
import os
import re
import shutil
import sys
import time
import urllib.error
import urllib.parse
import urllib.request
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

PROBE_DIR = Path(__file__).resolve().parent
if str(PROBE_DIR) not in sys.path:
    sys.path.insert(0, str(PROBE_DIR))

from preflight_probe import DEFAULT_INSTALL_ROOT, DEFAULT_LOG_DIR  # noqa: E402


PASS = "PASS"
WAITING = "WAITING"
FAIL = "FAIL"
DEFAULT_DOWNLOAD_PAGE = "https://pr0p.dev/download"
ALLOWED_INSTALL_ROOTS = (
    Path("/tmp/fpv-test-simitl-pr0p"),
    Path.home() / ".local" / "state" / "fpv-test" / "simitl-pr0p",
)


@dataclass
class InstallProbeResult:
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


def request_url(url: str, *, method: str = "GET", timeout: float = 20.0) -> tuple[bytes, dict[str, str], str]:
    request = urllib.request.Request(
        url,
        method=method,
        headers={"User-Agent": "fpv-test-simitl-pr0p-probe/1.0"},
    )
    with urllib.request.urlopen(request, timeout=timeout) as response:
        data = response.read() if method != "HEAD" else b""
        headers = {key.lower(): value for key, value in response.headers.items()}
        return data, headers, response.geturl()


def discover_linux_updater(page_html: str, *, base_url: str) -> dict[str, Any] | None:
    anchor_re = re.compile(r"<a\b(?P<attrs>[^>]*)>(?P<body>.*?)</a>", re.IGNORECASE | re.DOTALL)
    href_re = re.compile(r"href=[\"'](?P<href>.*?)[\"']", re.IGNORECASE | re.DOTALL)
    best: dict[str, Any] | None = None
    for match in anchor_re.finditer(page_html):
        attrs = match.group("attrs")
        body = re.sub(r"<.*?>", " ", match.group("body"))
        body_text = " ".join(html.unescape(body).split())
        href_match = href_re.search(attrs)
        if not href_match:
            continue
        href = html.unescape(href_match.group("href"))
        normalized = "%s %s" % (href.lower(), body_text.lower())
        if "linux" not in normalized or "updater" not in normalized:
            continue
        version_match = re.search(r"\b(\d+\.\d+(?:\.\d+)?)\b", body_text)
        candidate = {
            "url": urllib.parse.urljoin(base_url, href),
            "label": body_text,
            "version": version_match.group(1) if version_match else None,
        }
        if "build-linux-current" in href:
            return candidate
        best = candidate
    return best


def is_allowed_install_root(path: Path) -> bool:
    resolved = path.resolve()
    for allowed in ALLOWED_INSTALL_ROOTS:
        allowed_resolved = allowed.resolve()
        if resolved == allowed_resolved or allowed_resolved in resolved.parents:
            return True
    return False


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def download_file(url: str, destination: Path, *, timeout: float, force: bool) -> dict[str, Any]:
    if destination.exists() and not force:
        return {
            "path": str(destination),
            "downloaded": False,
            "size_bytes": destination.stat().st_size,
            "sha256": sha256_file(destination),
            "mode": oct(destination.stat().st_mode & 0o777),
            "reason": "already exists; use --force to replace",
        }
    tmp = destination.with_suffix(destination.suffix + ".tmp")
    request = urllib.request.Request(
        url,
        headers={"User-Agent": "fpv-test-simitl-pr0p-probe/1.0"},
    )
    with urllib.request.urlopen(request, timeout=timeout) as response, tmp.open("wb") as output:
        shutil.copyfileobj(response, output)
    tmp.chmod(0o755)
    tmp.replace(destination)
    destination.chmod(0o755)
    return {
        "path": str(destination),
        "downloaded": True,
        "size_bytes": destination.stat().st_size,
        "sha256": sha256_file(destination),
        "mode": oct(destination.stat().st_mode & 0o777),
    }


def file_type(path: Path) -> str | None:
    if not path.exists():
        return None
    try:
        import subprocess

        proc = subprocess.run(
            ["file", "-b", str(path)],
            check=False,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            timeout=3.0,
        )
    except (OSError, subprocess.SubprocessError):
        return None
    return proc.stdout.strip() if proc.returncode == 0 else None


def scan_install_root(path: Path) -> dict[str, Any]:
    if not path.exists():
        return {"exists": False, "executables": [], "client_candidates": []}
    executables: list[dict[str, Any]] = []
    client_candidates: list[dict[str, Any]] = []
    for item in sorted(path.iterdir()):
        if not item.is_file():
            continue
        executable = os.access(item, os.X_OK)
        row = {
            "name": item.name,
            "path": str(item),
            "size_bytes": item.stat().st_size,
            "executable": executable,
        }
        if executable:
            executables.append(row)
        lower = item.name.lower()
        if executable and lower != "updater" and ("pr0p" in lower or "prop" in lower):
            client_candidates.append(row)
    return {
        "exists": True,
        "executables": executables,
        "client_candidates": client_candidates,
    }


def run_install_probe(
    *,
    install_root: Path,
    download_page: str,
    download: bool,
    force: bool,
    timeout: float,
    run_id: str,
) -> InstallProbeResult:
    if not is_allowed_install_root(install_root):
        return InstallProbeResult(
            status=FAIL,
            summary="install root is outside the allowed isolated roots",
            metrics={
                "install_root": str(install_root),
                "allowed_roots": [str(path) for path in ALLOWED_INSTALL_ROOTS],
            },
            notes=["REFUSE_NON_ISOLATED_INSTALL_ROOT"],
        )

    try:
        install_root.mkdir(parents=True, exist_ok=True)
    except OSError as exc:
        return InstallProbeResult(
            status=FAIL,
            summary="isolated install root could not be created",
            metrics={"install_root": str(install_root), "error": str(exc)},
            notes=["INSTALL_ROOT_CREATE_FAIL"],
        )

    destination = install_root / "updater"
    if destination.exists() and not download:
        download_metrics = {
            "path": str(destination),
            "downloaded": False,
            "size_bytes": destination.stat().st_size,
            "sha256": sha256_file(destination),
            "mode": oct(destination.stat().st_mode & 0o777),
            "file_type": file_type(destination),
            "reason": "already present in install root",
        }
        return InstallProbeResult(
            status=PASS,
            summary="Linux updater is already present in the isolated install root",
            metrics={
                "run_id": run_id,
                "download_page": download_page,
                "install_root": str(install_root),
                "install_root_contents": sorted(item.name for item in install_root.iterdir())[:30],
                "install_scan": scan_install_root(install_root),
                "download_requested": download,
                "download": download_metrics,
            },
            notes=["existing updater was not executed"],
        )

    try:
        page_bytes, page_headers, final_page_url = request_url(download_page, timeout=timeout)
    except (urllib.error.URLError, TimeoutError, OSError) as exc:
        return InstallProbeResult(
            status=FAIL,
            summary="could not fetch pr0p download page",
            metrics={"download_page": download_page, "error": str(exc)},
            notes=["PR0P_DOWNLOAD_PAGE_FAIL"],
        )

    page_text = page_bytes.decode("utf-8", errors="replace")
    candidate = discover_linux_updater(page_text, base_url=final_page_url)
    if candidate is None:
        return InstallProbeResult(
            status=FAIL,
            summary="could not discover a Linux updater link",
            metrics={
                "download_page": download_page,
                "final_page_url": final_page_url,
                "page_headers": page_headers,
            },
            notes=["PR0P_LINUX_LINK_NOT_FOUND"],
        )

    head_headers: dict[str, str] = {}
    final_download_url = candidate["url"]
    try:
        _, head_headers, final_download_url = request_url(
            candidate["url"],
            method="HEAD",
            timeout=timeout,
        )
    except (urllib.error.URLError, TimeoutError, OSError) as exc:
        return InstallProbeResult(
            status=FAIL,
            summary="Linux updater link is not reachable",
            metrics={"candidate": candidate, "error": str(exc)},
            notes=["PR0P_LINUX_LINK_UNREACHABLE"],
        )

    download_metrics: dict[str, Any] | None = None
    status = WAITING
    summary = "Linux updater discovered; rerun with --download to place it in the install root"
    notes = ["discovery only; no binary was downloaded"]
    if download:
        try:
            download_metrics = download_file(
                final_download_url,
                destination,
                timeout=timeout,
                force=force,
            )
        except (urllib.error.URLError, TimeoutError, OSError) as exc:
            return InstallProbeResult(
                status=FAIL,
                summary="could not download Linux updater",
                metrics={
                    "candidate": candidate,
                    "final_download_url": final_download_url,
                    "install_root": str(install_root),
                    "error": str(exc),
                },
                notes=["PR0P_DOWNLOAD_FAIL"],
            )
        download_metrics["file_type"] = file_type(destination)
        status = PASS
        summary = "Linux updater is present in the isolated install root"
        notes = ["downloaded updater only; it was not executed"]

    contents = sorted(item.name for item in install_root.iterdir())[:30]
    return InstallProbeResult(
        status=status,
        summary=summary,
        metrics={
            "run_id": run_id,
            "download_page": download_page,
            "final_page_url": final_page_url,
            "page_headers": page_headers,
            "candidate": candidate,
            "final_download_url": final_download_url,
            "head_headers": head_headers,
            "install_root": str(install_root),
            "install_root_contents": contents,
            "install_scan": scan_install_root(install_root),
            "download_requested": download,
            "download": download_metrics,
        },
        notes=notes,
    )


def build_markdown(result: InstallProbeResult, *, run_id: str) -> str:
    lines = [
        "# SimITL / pr0p Install Probe",
        "",
        "Run: `%s`" % run_id,
        "Verdict: `%s`" % result.status,
        "",
        result.summary,
        "",
        "| Metric | Value |",
        "| --- | --- |",
    ]
    candidate = result.metrics.get("candidate") or {}
    for key, value in (
        ("version", candidate.get("version")),
        ("download_url", result.metrics.get("final_download_url")),
        ("install_root", result.metrics.get("install_root")),
        ("download_requested", result.metrics.get("download_requested")),
    ):
        lines.append("| %s | `%s` |" % (key, value))
    download = result.metrics.get("download") or {}
    if download:
        lines.append("| downloaded_path | `%s` |" % download.get("path"))
        lines.append("| sha256 | `%s` |" % download.get("sha256"))
        lines.append("| size_bytes | `%s` |" % download.get("size_bytes"))
        lines.append("| file_type | `%s` |" % download.get("file_type"))
    scan = result.metrics.get("install_scan") or {}
    if scan:
        lines.append("| executable_count | `%s` |" % len(scan.get("executables", [])))
        lines.append("| client_candidate_count | `%s` |" % len(scan.get("client_candidates", [])))
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


def write_reports(result: InstallProbeResult, log_dir: Path, *, run_id: str) -> tuple[Path, Path]:
    log_dir.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime("%Y%m%d-%H%M%S", time.localtime())
    json_path = log_dir / ("%s-%s-install-probe.json" % (stamp, run_id))
    md_path = log_dir / ("%s-%s-install-probe.md" % (stamp, run_id))
    json_path.write_text(
        json.dumps(result.as_dict(), indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    md_path.write_text(build_markdown(result, run_id=run_id), encoding="utf-8")
    return json_path, md_path


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-id", default="pr0p-install")
    parser.add_argument("--install-root", type=Path, default=DEFAULT_INSTALL_ROOT)
    parser.add_argument("--log-dir", type=Path, default=DEFAULT_LOG_DIR)
    parser.add_argument("--download-page", default=DEFAULT_DOWNLOAD_PAGE)
    parser.add_argument("--download", action="store_true")
    parser.add_argument("--force", action="store_true")
    parser.add_argument("--timeout", type=float, default=30.0)
    args = parser.parse_args(argv)
    if args.timeout <= 0:
        parser.error("--timeout must be positive")
    if args.force and not args.download:
        parser.error("--force requires --download")
    return args


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    result = run_install_probe(
        install_root=args.install_root,
        download_page=args.download_page,
        download=args.download,
        force=args.force,
        timeout=args.timeout,
        run_id=args.run_id,
    )
    json_path, md_path = write_reports(result, args.log_dir, run_id=args.run_id)
    print("simitl-pr0p-install %s report=%s summary=%s" % (
        result.status,
        json_path,
        md_path,
    ))
    print(result.summary)
    return 1 if result.status == FAIL else 0


if __name__ == "__main__":
    raise SystemExit(main())
