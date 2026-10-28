"""Exercise the unified Windows simulation entry without starting ROS or networking."""

from __future__ import annotations

import json
import os
from pathlib import Path
import shutil
import subprocess

import pytest

POWERSHELL = shutil.which("powershell.exe")
pytestmark = pytest.mark.skipif(os.name != "nt" or POWERSHELL is None, reason="Windows PowerShell helper checks")
ENTRY = Path(__file__).resolve().parents[1] / "scripts/setup/start_simulation.ps1"


def ps_literal(value: str) -> str:
    return "'" + value.replace("'", "''") + "'"


def run_powershell(tmp_path, source):
    script = tmp_path / "pico_entry_check.ps1"
    script.write_text(
        "$ErrorActionPreference = 'Stop'\n"
        "[Console]::OutputEncoding = [Text.UTF8Encoding]::new($false)\n. " + ps_literal(str(ENTRY)) + "\n" + source,
        encoding="utf-8-sig",
    )
    return subprocess.run(
        [POWERSHELL, "-NoProfile", "-NonInteractive", "-ExecutionPolicy", "Bypass", "-File", str(script)],
        capture_output=True,
        text=True,
        encoding="utf-8",
        timeout=20,
    )


@pytest.mark.parametrize(
    "candidates,requested,expected",
    [
        (
            [
                dict(Address="192.168.1.2", Interface="WiFi", Wireless=True),
                dict(Address="10.0.0.2", Interface="LAN", Wireless=False),
            ],
            "",
            "192.168.1.2",
        ),
        ([dict(Address="10.0.0.2", Interface="LAN", Wireless=False)], "", "10.0.0.2"),
        (
            [
                dict(Address="192.168.1.2", Interface="WiFi", Wireless=True),
                dict(Address="10.0.0.2", Interface="LAN", Wireless=False),
            ],
            "10.0.0.2",
            "10.0.0.2",
        ),
    ],
)
def test_selects_wifi_or_explicit_assigned_address(tmp_path, candidates, requested, expected):
    result = run_powershell(
        tmp_path,
        "$candidates = ConvertFrom-Json " + ps_literal(json.dumps(candidates)) + "\n"
        "$selected = Select-PicoListenAddress -Candidates $candidates -Requested " + ps_literal(requested) + "\n"
        "[Console]::WriteLine($selected.Address)\n",
    )
    assert result.returncode == 0, result.stderr
    assert result.stdout.strip() == expected


@pytest.mark.parametrize(
    "candidates,requested",
    [
        ([], ""),
        (
            [
                dict(Address="10.0.0.2", Interface="LAN1", Wireless=False),
                dict(Address="10.1.0.2", Interface="LAN2", Wireless=False),
            ],
            "",
        ),
        (
            [
                dict(Address="10.0.0.2", Interface="WiFi1", Wireless=True),
                dict(Address="10.1.0.2", Interface="WiFi2", Wireless=True),
            ],
            "",
        ),
        ([dict(Address="10.0.0.2", Interface="LAN", Wireless=False)], "10.0.0.3"),
        ([dict(Address="10.0.0.2", Interface="LAN", Wireless=False)], "10.0.0.002"),
        ([dict(Address="10.0.0.2", Interface="LAN", Wireless=False)], "::1"),
    ],
)
def test_ambiguous_or_invalid_address_does_not_guess(tmp_path, candidates, requested):
    result = run_powershell(
        tmp_path,
        "$candidates = ConvertFrom-Json " + ps_literal(json.dumps(candidates)) + "\n"
        "Select-PicoListenAddress -Candidates $candidates -Requested " + ps_literal(requested),
    )
    assert result.returncode != 0


def test_check_only_never_starts_processes_or_mutates_network(tmp_path):
    result = run_powershell(
        tmp_path,
        """
function Get-PicoNetworkCandidates {
    [pscustomobject]@{Address='192.168.1.2';Interface='WiFi';Wireless=$true}
}
function Get-PicoPython { 'test-python.exe' }
function Invoke-PicoNative { throw 'CheckOnly attempted native process' }
function Invoke-PicoNetworkSetup { throw 'CheckOnly attempted network setup' }
function Start-Process { throw 'CheckOnly attempted elevation' }
Start-LocalSimulation -Source pico -Distribution 'RM65-Servo' -ReadOnly
""",
    )
    assert result.returncode == 0, result.stderr
    assert "192.168.1.2:63901" in result.stdout
    assert "CheckOnly" in result.stdout


def test_script_parses_in_windows_powershell(tmp_path):
    result = run_powershell(
        tmp_path,
        "$tokens = $null; $errors = $null\n"
        "[void][Management.Automation.Language.Parser]::ParseFile("
        + ps_literal(str(ENTRY))
        + ", [ref]$tokens, [ref]$errors)\n"
        "if ($errors.Count) { throw ($errors | Out-String) }\n",
    )
    assert result.returncode == 0, result.stderr


@pytest.mark.parametrize("source", ["manual", "mock"])
def test_sources_without_headset_never_read_network_or_prompt(tmp_path, source):
    result = run_powershell(
        tmp_path,
        """
function Read-Host { throw 'MUST_NOT_PROMPT' }
function Get-PicoNetworkCandidates { throw 'MUST_NOT_READ_NETWORK' }
function Get-PicoPython { 'test-python.exe' }
function Invoke-PicoNative { throw 'MUST_NOT_START_PROCESS' }
function Invoke-PicoNetworkSetup { throw 'MUST_NOT_CONFIGURE_NETWORK' }
""" + "Start-LocalSimulation -Source " + ps_literal(source) + " -Distribution 'RM65-Servo' -ReadOnly\n",
    )
    assert result.returncode == 0, result.stderr
    assert f"--input {source}" in result.stdout and "--target wsl" in result.stdout
    if source == "manual":
        assert "http://127.0.0.1:8766" in result.stdout


def test_three_explicit_batch_entries_replace_the_menu():
    root = ENTRY.parents[2]
    assert sorted(path.name for path in root.glob("*.cmd")) == sorted(
        ["启动无手柄仿真.cmd", "启动PICO仿真.cmd", "启动真机.cmd"]
    )
    for name, source in [("启动无手柄仿真.cmd", "manual"), ("启动PICO仿真.cmd", "pico")]:
        body = (root / name).read_text(encoding="utf-8")
        assert "start_simulation.ps1" in body and f"-InputSource {source}" in body
    assert "start_real_robot.ps1" in (root / "启动真机.cmd").read_text(encoding="utf-8")
    assert "Read-Host" not in ENTRY.read_text(encoding="utf-8-sig")


def test_actual_manual_batch_check_only_has_no_menu_or_network():
    entry = ENTRY.parents[2] / "启动无手柄仿真.cmd"
    result = subprocess.run(
        ["cmd.exe", "/d", "/c", str(entry), "-CheckOnly"],
        capture_output=True,
        encoding="utf-8",
        timeout=20,
    )
    assert result.returncode == 0, result.stderr
    assert "--input manual" in result.stdout and "http://127.0.0.1:8766" in result.stdout
    assert "CheckOnly" in result.stdout
    assert "Starting MuJoCo" not in result.stdout


def test_default_powershell_entry_is_manual_without_prompt():
    result = subprocess.run(
        [POWERSHELL, "-NoProfile", "-NonInteractive", "-ExecutionPolicy", "Bypass", "-File", str(ENTRY), "-CheckOnly"],
        capture_output=True,
        encoding="utf-8",
        timeout=20,
    )
    assert result.returncode == 0, result.stderr
    assert "--input manual" in result.stdout and "CheckOnly" in result.stdout
    assert "Starting MuJoCo" not in result.stdout


@pytest.mark.parametrize("source", ["manual", "mock", "pico"])
def test_all_sources_route_to_same_local_teleop_entry(tmp_path, source):
    result = run_powershell(
        tmp_path,
        """
$global:calls = [Collections.Generic.List[string]]::new()
function Get-PicoNetworkCandidates {
    if ($global:chosen -ne 'pico') { throw 'MOCK_MUST_NOT_READ_NETWORK' }
    [pscustomobject]@{Address='192.168.1.2';Interface='WiFi';Wireless=$true}
}
function Get-PicoPython { 'test-python.exe' }
function Invoke-PicoNetworkSetup {
    if ($global:chosen -ne 'pico') { throw 'MOCK_MUST_NOT_CONFIGURE_NETWORK' }
    $global:calls.Add('network')
}
function Invoke-PicoNative {
    param($Executable, $Arguments)
    if ($Executable -eq 'test-python.exe') {
        if ($Arguments[2] -notlike '*teleop.py') { throw 'WRONG_ENTRY' }
        $expected = @('--mode','sim','--target','wsl','--wsl-distro','RM65-Servo','--input',$global:chosen)
        for ($i=0; $i -lt $expected.Count; $i++) {
            if ($Arguments[$i+3] -cne $expected[$i]) { throw 'WRONG_ARGUMENTS' }
        }
        $global:calls.Add('teleop')
    } else {
        $global:calls.Add('docker')
    }
}
"""
        + "$global:chosen = "
        + ps_literal(source)
        + "\nStart-LocalSimulation -Source $global:chosen -Distribution 'RM65-Servo'\n"
        + "[Console]::WriteLine('CALLS=' + ($global:calls -join ','))\n",
    )
    assert result.returncode == 0, result.stderr
    expected = "docker,network,teleop" if source == "pico" else "docker,teleop"
    assert "CALLS=" + expected in result.stdout


@pytest.mark.parametrize("source", ["manual", "mock"])
def test_non_pico_rejects_listen_address_before_starting(tmp_path, source):
    result = run_powershell(
        tmp_path,
        "function Invoke-PicoNative { throw 'PROCESS_MUST_NOT_START' }\n"
        f"Start-LocalSimulation -Source {source} -Address '192.168.1.2' -Distribution 'RM65-Servo'\n",
    )
    assert result.returncode != 0
    assert "ListenAddress is only used" in result.stderr
    assert "PROCESS_MUST_NOT_START" not in result.stderr


def test_docker_failure_prevents_network_setup_and_ros(tmp_path):
    result = run_powershell(
        tmp_path,
        """
function Get-PicoNetworkCandidates {
    [pscustomobject]@{Address='192.168.1.2';Interface='WiFi';Wireless=$true}
}
function Get-PicoPython { 'test-python.exe' }
function Invoke-PicoNative { throw 'DOCKER_UNAVAILABLE' }
function Invoke-PicoNetworkSetup { throw 'NETWORK_SETUP_MUST_NOT_RUN' }
Start-LocalSimulation -Source pico -Distribution 'RM65-Servo'
""",
    )
    assert result.returncode != 0
    assert "DOCKER_UNAVAILABLE" in result.stderr
    assert "NETWORK_SETUP_MUST_NOT_RUN" not in result.stderr


def test_failed_network_setup_prevents_ros_launch(tmp_path):
    result = run_powershell(
        tmp_path,
        """
function Get-PicoNetworkCandidates {
    [pscustomobject]@{Address='192.168.1.2';Interface='WiFi';Wireless=$true}
}
function Get-PicoPython { 'test-python.exe' }
function Invoke-PicoNative {
    param($Executable, $Arguments)
    if ($Executable -eq 'test-python.exe') { throw 'ROS_MUST_NOT_START' }
}
function Invoke-PicoNetworkSetup { throw 'NETWORK_CONFIGURATION_FAILED' }
Start-LocalSimulation -Source pico -Distribution 'RM65-Servo'
""",
    )
    assert result.returncode != 0
    assert "NETWORK_CONFIGURATION_FAILED" in result.stderr
    assert "ROS_MUST_NOT_START" not in result.stderr


@pytest.mark.parametrize("previous", [None, "192.168.1.2", "192.168.9.2"])
def test_network_migration_removes_only_a_different_recorded_listen_address(tmp_path, previous):
    helper = tmp_path / "network_helper_stub.ps1"
    helper.write_text(
        """[CmdletBinding(SupportsShouldProcess=$true)]
param([switch]$Remove, [string]$ListenAddress, [string]$Distro)
$action = if ($Remove) { 'remove' } else { 'configure' }
$phase = if ($WhatIfPreference) { 'preview' } else { 'apply' }
$global:picoCalls.Add("$($action):$phase")
""",
        encoding="utf-8-sig",
    )
    state = tmp_path / "pico-state.json"
    if previous is not None:
        state.write_text(json.dumps({"listen_address": previous}), encoding="utf-8")
    expected = ["configure:preview", "configure:apply"]
    if previous and previous != "192.168.1.2":
        expected = ["remove:preview", "remove:apply", *expected]
    result = run_powershell(
        tmp_path,
        "$global:picoCalls = [Collections.Generic.List[string]]::new()\n"
        + "Invoke-PicoNetworkMigration -Helper "
        + ps_literal(str(helper))
        + " -Address '192.168.1.2' -Distribution 'RM65-Servo' -StateFile "
        + ps_literal(str(state))
        + "\n$expected = ConvertFrom-Json "
        + ps_literal(json.dumps(expected))
        + "\nif ($global:picoCalls.Count -ne $expected.Count) { throw 'Wrong number of network operations' }\n"
        + "for ($i=0; $i -lt $expected.Count; $i++) { if ($global:picoCalls[$i] -cne $expected[$i]) { throw 'Wrong migration order' } }\n",
    )
    assert result.returncode == 0, result.stderr


def test_migration_does_not_configure_when_helper_refuses_old_ownership(tmp_path):
    helper = tmp_path / "network_helper_stub.ps1"
    helper.write_text(
        """[CmdletBinding(SupportsShouldProcess=$true)]
param([switch]$Remove, [string]$ListenAddress, [string]$Distro)
if ($Remove) { throw 'OLD_RULE_OWNERSHIP_REFUSED' }
throw 'NEW_CONFIGURATION_MUST_NOT_RUN'
""",
        encoding="utf-8-sig",
    )
    state = tmp_path / "pico-state.json"
    state.write_text(json.dumps({"listen_address": "192.168.9.2"}), encoding="utf-8")
    result = run_powershell(
        tmp_path,
        "Invoke-PicoNetworkMigration -Helper "
        + ps_literal(str(helper))
        + " -Address '192.168.1.2' -Distribution 'RM65-Servo' -StateFile "
        + ps_literal(str(state)),
    )
    assert result.returncode != 0
    assert "OLD_RULE_OWNERSHIP_REFUSED" in result.stderr
    assert "NEW_CONFIGURATION_MUST_NOT_RUN" not in result.stderr
