"""Check real-robot startup routing without SSH, deployment, or robot access."""

from __future__ import annotations

import json
import os
from pathlib import Path
import shutil
import subprocess

import pytest

POWERSHELL = shutil.which("powershell.exe")
pytestmark = pytest.mark.skipif(os.name != "nt" or POWERSHELL is None, reason="Windows PowerShell entry checks")
ROOT = Path(__file__).resolve().parents[1]
ENTRY = ROOT / "scripts/setup/start_real_robot.ps1"


def ps_literal(value):
    return "'" + str(value).replace("'", "''") + "'"


def run_check(tmp_path, source):
    script = tmp_path / "real_entry_check.ps1"
    script.write_text(
        "$ErrorActionPreference = 'Stop'\n"
        "[Console]::OutputEncoding = [Text.UTF8Encoding]::new($false)\n. " + ps_literal(ENTRY) + "\n" + source,
        encoding="utf-8-sig",
    )
    return subprocess.run(
        [POWERSHELL, "-NoProfile", "-NonInteractive", "-ExecutionPolicy", "Bypass", "-File", str(script)],
        capture_output=True,
        text=True,
        encoding="utf-8",
        timeout=20,
    )


def test_actual_real_batch_without_arguments_is_closed_by_default():
    result = subprocess.run(
        ["cmd.exe", "/d", "/c", str(ROOT / "启动真机.cmd")],
        input="\n",
        capture_output=True,
        text=True,
        encoding="utf-8",
        timeout=20,
    )
    assert result.returncode == 2, result.stderr
    for required in ["-UdpIp", "-HardwareValidation", "-LiveAck"]:
        assert required in result.stdout
    assert "尚未启动真机" in result.stdout
    assert "先同步" not in result.stdout


@pytest.mark.parametrize(
    "options",
    [
        "",
        "-ReadOnly",
        "-StateOnly",
        "-BoardUdpIp '192.0.2.20' -ValidationFile 'test-only.yaml'",
        "-BoardUdpIp '192.0.2.20' -OperatorAck 'RM65 LIVE VALIDATION'",
        "-BoardUdpIp '192.0.2.20' -ValidationFile 'test-only.yaml' -OperatorAck 'wrong'",
        "-StateOnly -BoardUdpIp '192.0.2.020'",
        "-StateOnly -BoardUdpIp '::1'",
    ],
)
def test_incomplete_or_invalid_robot_options_fail_before_process_lookup(tmp_path, options):
    result = run_check(
        tmp_path,
        "function Get-RealRobotPython { throw 'PROCESS_LOOKUP_MUST_NOT_RUN' }\n"
        "function Invoke-RealRobotNative { throw 'PROCESS_MUST_NOT_RUN' }\n"
        f"Start-RealRobot {options}\n",
    )
    assert result.returncode != 0
    assert "PROCESS_LOOKUP_MUST_NOT_RUN" not in result.stderr
    assert "PROCESS_MUST_NOT_RUN" not in result.stderr


@pytest.mark.parametrize("dry_run", [False, True])
def test_explicit_robot_entry_forwards_same_teleop_chain_and_offline_preview(tmp_path, dry_run):
    options = "-StateOnly" if dry_run else "-ValidationFile 'test-only.yaml' -OperatorAck 'RM65 LIVE VALIDATION'"
    result = run_check(
        tmp_path,
        "function Get-RealRobotPython { 'not-executed-python' }\n"
        "function Invoke-RealRobotNative {\n"
        "param($Executable, $Arguments)\n"
        "if ($Executable -cne 'not-executed-python') { throw 'WRONG_EXECUTABLE' }\n"
        "[Console]::WriteLine('ARGV=' + (ConvertTo-Json -InputObject @($Arguments) -Compress))\n"
        "}\n"
        f"Start-RealRobot -BoardUdpIp '192.0.2.20' -ReadOnly {options}\n",
    )
    assert result.returncode == 0, result.stderr
    line = next(line for line in result.stdout.splitlines() if line.startswith("ARGV="))
    arguments = json.loads(line[len("ARGV=") :])
    assert arguments[:2] == ["-X", "utf8"]
    assert Path(arguments[2]) == ROOT / "teleop.py"
    assert arguments[3:7] == ["--mode", "hardware-dry-run" if dry_run else "hardware", "--target", "remote"]
    assert arguments[arguments.index("--input") + 1] == "pico"
    assert arguments[arguments.index("--remote-dir") + 1] == "EthanTao"
    assert arguments[arguments.index("--udp-ip") + 1] == "192.0.2.20"
    assert arguments[-1] == "--print-command"
    if dry_run:
        assert "--live-ack" not in arguments and "--hardware-validation" not in arguments
    else:
        assert arguments[arguments.index("--live-ack") + 1] == "RM65 LIVE VALIDATION"
        assert arguments[arguments.index("--hardware-validation") + 1] == "test-only.yaml"


def test_actual_dry_run_check_only_is_offline_and_uses_board_chain():
    result = subprocess.run(
        [
            POWERSHELL,
            "-NoProfile",
            "-NonInteractive",
            "-ExecutionPolicy",
            "Bypass",
            "-File",
            str(ENTRY),
            "-DryRun",
            "-UdpIp",
            "192.0.2.20",
            "-CheckOnly",
        ],
        capture_output=True,
        text=True,
        encoding="utf-8",
        timeout=20,
    )
    assert result.returncode == 0, result.stderr
    assert "CheckOnly" in result.stdout and "mode:=hardware-dry-run" in result.stdout
    assert "input_source:=pico" in result.stdout and "udp_ip:=192.0.2.20" in result.stdout
    assert "实际运行" not in result.stdout


def test_actual_live_preview_keeps_existing_template_rejection():
    result = subprocess.run(
        [
            POWERSHELL,
            "-NoProfile",
            "-NonInteractive",
            "-ExecutionPolicy",
            "Bypass",
            "-File",
            str(ENTRY),
            "-UdpIp",
            "192.0.2.20",
            "-HardwareValidation",
            "ros2_ws/src/rm65_servo_teleop/config/hardware_validation.example.yaml",
            "-LiveAck",
            "RM65 LIVE VALIDATION",
            "-CheckOnly",
        ],
        capture_output=True,
        text=True,
        encoding="utf-8",
        timeout=20,
    )
    assert result.returncode == 2, result.stderr
    assert "现场验证记录不完整" in result.stdout
    assert "先同步" not in result.stdout and "docker compose" not in result.stdout
