"""Legacy Windows parameters forward to the current chain without network access."""

import os
from pathlib import Path
import shutil
import subprocess

import pytest

ROOT = Path(__file__).resolve().parents[1]
POWERSHELL = shutil.which("powershell.exe")
pytestmark = pytest.mark.skipif(os.name != "nt" or not POWERSHELL, reason="Windows compatibility entry points")


def run_legacy(tmp_path, script, *arguments):
    return subprocess.run(
        [
            POWERSHELL,
            "-NoProfile",
            "-NonInteractive",
            "-ExecutionPolicy",
            "Bypass",
            "-File",
            str(ROOT / "scripts/setup" / script),
            *arguments,
        ],
        cwd=tmp_path,
        stdin=subprocess.DEVNULL,
        capture_output=True,
        encoding="utf-8",
        errors="replace",
        timeout=30,
        env={**os.environ, "PYTHONUTF8": "1"},
    )


@pytest.mark.parametrize("source", ["manual", "pico", "mock"])
def test_old_simulation_parameters_preview_from_another_directory(tmp_path, source):
    result = run_legacy(tmp_path, "start_simulation.ps1", "-InputSource", source, "-Distro", "RM65-Servo", "-CheckOnly")
    assert result.returncode == 0, result.stdout + result.stderr
    assert f"input_source:={source}" in result.stdout
    assert "--distribution RM65-Servo" in result.stdout


def test_old_real_parameters_keep_custom_addresses_and_path_spaces(tmp_path):
    result = run_legacy(
        tmp_path,
        "start_real_robot.ps1",
        "-DryRun",
        "-CheckOnly",
        "-UdpIp",
        "192.0.2.100",
        "-ArmIp",
        "192.0.2.18",
        "-SshHost",
        "test-board",
        "-RemoteDir",
        "workspace with spaces",
    )
    assert result.returncode == 0, result.stdout + result.stderr
    for marker in ("mode:=hardware-dry-run", "udp_ip:=192.0.2.100", "arm_ip:=192.0.2.18", "workspace with spaces"):
        assert marker in result.stdout


@pytest.mark.parametrize("arguments", [(), ("-UdpIp", "192.0.2.100"), ("-DryRun",)])
def test_legacy_real_entry_still_rejects_missing_safety_parameters(tmp_path, arguments):
    result = run_legacy(tmp_path, "start_real_robot.ps1", *arguments)
    assert result.returncode == 2, result.stdout + result.stderr
    assert "docker compose" not in result.stdout


@pytest.mark.parametrize("name,source", [("启动无手柄仿真.cmd", "manual"), ("启动PICO仿真.cmd", "pico")])
def test_existing_batch_names_accept_existing_preview_switch(tmp_path, name, source):
    result = subprocess.run(
        [os.environ["COMSPEC"], "/d", "/c", str(ROOT / name), "-CheckOnly"],
        cwd=tmp_path,
        stdin=subprocess.DEVNULL,
        capture_output=True,
        encoding="utf-8",
        errors="replace",
        timeout=30,
        env={**os.environ, "PYTHONUTF8": "1"},
    )
    assert result.returncode == 0, result.stdout + result.stderr
    assert f"input_source:={source}" in result.stdout
