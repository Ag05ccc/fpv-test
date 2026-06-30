import os
import subprocess
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
SCRIPT = REPO_ROOT / "tools" / "run_gazebo_betaflight.sh"


def make_fake_aeroloop(tmp_path: Path) -> Path:
    root = tmp_path / "aeroloop_gazebo"
    (root / "plugins" / "build").mkdir(parents=True)
    (root / "plugins" / "build" / "libBetaflightPlugin.so").write_text("")
    (root / "worlds").mkdir()
    (root / "worlds" / "test_betaflight.sdf").write_text(
        "<sdf><world name='test'><physics><max_step_size>0.001</max_step_size></physics></world></sdf>\n"
    )
    (root / "models").mkdir()
    return root


def make_fake_gz(tmp_path: Path) -> Path:
    bin_dir = tmp_path / "bin"
    bin_dir.mkdir()
    gz = bin_dir / "gz"
    gz.write_text("#!/usr/bin/env sh\nexit 0\n")
    gz.chmod(0o755)
    return bin_dir


def run_launcher(tmp_path: Path, *args: str) -> subprocess.CompletedProcess[str]:
    env = dict(os.environ)
    env["AEROLOOP_GAZEBO"] = str(make_fake_aeroloop(tmp_path))
    env["PATH"] = "%s:%s" % (make_fake_gz(tmp_path), env.get("PATH", ""))
    return subprocess.run(
        [str(SCRIPT), *args],
        cwd=REPO_ROOT,
        env=env,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=False,
    )


def test_headless_dry_run_uses_headless_rendering_by_default(tmp_path):
    result = run_launcher(tmp_path, "--world", "test_betaflight.sdf", "--headless", "--dry-run")

    assert result.returncode == 0
    assert "command:" in result.stdout
    assert "--headless-rendering" in result.stdout


def test_headless_rendering_can_be_disabled_for_legacy_debug(tmp_path):
    result = run_launcher(
        tmp_path,
        "--world", "test_betaflight.sdf",
        "--headless",
        "--no-headless-rendering",
        "--dry-run",
    )

    assert result.returncode == 0
    assert "command:" in result.stdout
    assert "--headless-rendering" not in result.stdout
