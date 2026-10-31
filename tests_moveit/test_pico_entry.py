"""Offline regression checks for Python shortcuts; never start WSL, ROS or networking."""

from __future__ import annotations

import base64
import builtins
import json
import os
import re
from pathlib import Path
import runpy
import shutil
import subprocess
import sys
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

import servo_launcher
from scripts.setup import launcher

ROOT = Path(__file__).resolve().parents[1]
POWERSHELL = shutil.which("powershell.exe")
WINDOWS_PS = pytest.mark.skipif(os.name != "nt" or not POWERSHELL, reason="Windows PowerShell migration stub")


def candidate(address="192.168.1.2", interface="WiFi", wireless=True):
    return dict(address=address, interface=interface, wireless=wireless)


def forbidden(*args, **kwargs):
    pytest.fail("Offline/safety boundary attempted a process, network query, elevation or prompt")


@pytest.mark.parametrize(
    "candidates,requested,expected",
    [
        ([candidate(), candidate("10.0.0.2", "LAN", False)], None, "192.168.1.2"),
        ([candidate("10.0.0.2", "LAN", False)], None, "10.0.0.2"),
        ([candidate(), candidate("10.0.0.2", "LAN", False)], "10.0.0.2", "10.0.0.2"),
    ],
)
def test_selects_wifi_or_explicit_assigned_address(candidates, requested, expected):
    assert launcher.select_pico_address(candidates, requested)["address"] == expected


@pytest.mark.parametrize(
    "candidates,requested",
    [
        ([], None),
        ([candidate("10.0.0.2", "LAN1", False), candidate("10.1.0.2", "LAN2", False)], None),
        ([candidate(), candidate("10.1.0.2", "WiFi2")], None),
        ([candidate()], "192.168.1.3"),
        ([candidate()], "192.168.1.002"),
        ([candidate()], "::1"),
        ([candidate("0.0.0.0")], None),
        ([candidate("127.0.0.1")], None),
        ([candidate("224.0.0.1")], None),
        ([candidate("255.255.255.255")], None),
    ],
)
def test_ambiguous_or_invalid_address_does_not_guess(candidates, requested):
    with pytest.raises(ValueError):
        launcher.select_pico_address(candidates, requested)


@pytest.mark.parametrize("kind", ["manual", "pico"])
def test_check_only_is_in_process_and_never_reads_or_changes_network(monkeypatch, kind):
    preview = Mock(return_value=0)
    monkeypatch.setattr(servo_launcher, "main", preview)
    for name in ("get_pico_candidates", "setup_pico_network", "run_command"):
        monkeypatch.setattr(launcher, name, forbidden)
    monkeypatch.setattr(subprocess, "run", forbidden)
    monkeypatch.setattr(
        launcher, "ctypes", SimpleNamespace(windll=SimpleNamespace(shell32=SimpleNamespace(IsUserAnAdmin=forbidden)))
    )
    monkeypatch.setattr(builtins, "input", forbidden)
    assert launcher.main(kind, ["--check-only"]) == 0
    assert preview.call_args.args[0] == [
        "--mode",
        "sim",
        "--target",
        "wsl",
        "--wsl-distro",
        "RM65-Servo",
        "--input",
        kind,
        "--print-command",
    ]


def test_explicit_pico_preview_does_not_claim_to_have_selected_a_nic(monkeypatch, capsys):
    monkeypatch.setattr(servo_launcher, "main", Mock(return_value=0))
    monkeypatch.setattr(launcher, "get_pico_candidates", forbidden)
    assert launcher.main("pico", ["--check-only", "--listen-address", "192.168.1.2"]) == 0
    output = capsys.readouterr().out
    assert "192.168.1.2:63901" in output and "预览不枚举网卡" in output


@pytest.mark.parametrize("kind", ["manual", "pico"])
@pytest.mark.parametrize("viewer", [[], ["--rviz"], ["--no-viewer"]])
def test_sim_arguments_preserve_existing_teleop_cli(kind, viewer):
    args = launcher.make_parser(kind).parse_args(["--distro", "RM65-Custom", *viewer])
    assert launcher.sim_arguments(kind, args) == [
        "--mode",
        "sim",
        "--target",
        "wsl",
        "--wsl-distro",
        "RM65-Custom",
        "--input",
        kind,
        *viewer,
    ]


@pytest.mark.parametrize(
    "options",
    [
        ["--listen-address", "192.168.1.2"],
        ["--rviz", "--no-viewer"],
        ["--distro", "bad; touch injected"],
        ["--distro", "bad'quote"],
        ["--distro", "../bad"],
    ],
)
def test_invalid_sim_options_fail_before_any_side_effect(monkeypatch, options):
    monkeypatch.setattr(launcher, "run_command", forbidden)
    monkeypatch.setattr(launcher, "get_pico_candidates", forbidden)
    monkeypatch.setattr(launcher, "preview", forbidden)
    assert launcher.main("manual", options) == 2


@pytest.mark.parametrize("kind", ["manual", "pico"])
def test_all_shortcuts_route_to_same_teleop_with_docker_before_network(monkeypatch, kind, capsys):
    # Patch this module's os binding, never global os.name (which breaks pathlib on POSIX).
    monkeypatch.setattr(launcher, "os", SimpleNamespace(name="nt", environ=os.environ))
    calls = []
    monkeypatch.setattr(launcher, "get_pico_candidates", lambda: [candidate()])
    if kind == "manual":
        monkeypatch.setattr(launcher, "get_pico_candidates", forbidden)
    monkeypatch.setattr(launcher, "setup_pico_network", lambda *args: calls.append(("network", args)))
    monkeypatch.setattr(launcher, "run_command", lambda command, **kwargs: calls.append(("process", command, kwargs)))
    assert launcher.main(kind, []) == 0
    assert calls[0][1][1:] == [
        "--distribution",
        "RM65-Servo",
        "--user",
        "root",
        "--exec",
        "docker",
        "info",
        "--format",
        "{{.ServerVersion}}",
    ]
    assert calls[0][2] == {"timeout": 90}
    if kind == "pico":
        assert calls[1] == ("network", ("192.168.1.2", "RM65-Servo"))
        assert len(calls) == 3
    else:
        assert len(calls) == 2
        assert "http://127.0.0.1:8766" in capsys.readouterr().out
    assert calls[-1][1][:-2] == [
        sys.executable,
        "-X",
        "utf8",
        "-u",
        str(ROOT / "teleop.py"),
        "--mode",
        "sim",
        "--target",
        "wsl",
        "--wsl-distro",
        "RM65-Servo",
        "--input",
        kind,
    ]
    assert calls[-1][1][-2] == "--sim-reset-token"
    assert re.fullmatch(r"[0-9a-f]{32}", calls[-1][1][-1])


def test_docker_failure_prevents_network_setup_and_ros(monkeypatch):
    monkeypatch.setattr(launcher, "os", SimpleNamespace(name="nt", environ=os.environ))
    monkeypatch.setattr(launcher, "get_pico_candidates", lambda: [candidate()])
    monkeypatch.setattr(launcher, "setup_pico_network", forbidden)
    native = Mock(side_effect=subprocess.CalledProcessError(19, ["docker", "info"]))
    monkeypatch.setattr(launcher, "run_command", native)
    assert launcher.main("pico", []) == 2
    assert native.call_count == 1
    assert "docker" in native.call_args.args[0]


def test_failed_network_setup_prevents_ros_launch(monkeypatch):
    monkeypatch.setattr(launcher, "os", SimpleNamespace(name="nt", environ=os.environ))
    monkeypatch.setattr(launcher, "get_pico_candidates", lambda: [candidate()])
    native = Mock()
    monkeypatch.setattr(launcher, "run_command", native)
    monkeypatch.setattr(launcher, "setup_pico_network", Mock(side_effect=subprocess.CalledProcessError(1, ["network"])))
    assert launcher.main("pico", []) == 2
    assert native.call_count == 1 and "docker" in native.call_args.args[0]


def test_non_windows_sim_fails_closed_before_query_or_launch(monkeypatch):
    monkeypatch.setattr(launcher, "os", SimpleNamespace(name="posix"))
    monkeypatch.setattr(launcher, "get_pico_candidates", forbidden)
    monkeypatch.setattr(launcher, "run_command", forbidden)
    assert launcher.main("pico", []) == 2


def test_mock_demo_remains_on_existing_teleop_cli(monkeypatch):
    # No fourth shortcut / new motion chain: mock is still teleop.py's original input.
    import teleop

    preview = Mock(return_value=0)
    monkeypatch.setattr(servo_launcher, "run", preview)
    assert teleop.main(["--mode", "sim", "--target", "local", "--input", "mock", "--print-command"]) == 0
    args = preview.call_args.args[0]
    assert (args.mode, args.target, args.input, args.print_command) == ("sim", "local", "mock", True)


@pytest.mark.parametrize(
    "mode,kind", [("sim", "manual"), ("pico", "pico"), ("real", "real")]
)
def test_single_root_entry_forwards_mode_arguments_and_exit_code(monkeypatch, mode, kind):
    cli = Mock(return_value=23)
    monkeypatch.setattr(launcher, "cli", cli)
    monkeypatch.setattr(sys, "argv", ["start.py", mode, "--check-only"])
    with pytest.raises(SystemExit) as exc:
        runpy.run_path(str(ROOT / "start.py"), run_name="__main__")
    assert exc.value.code == 23
    cli.assert_called_once_with(kind, ["--check-only"])


def test_only_english_daily_entry_remains_and_network_helper_is_preserved():
    assert (ROOT / "start.py").is_file()
    for name in ("启动.py", "启动无手柄仿真.py", "启动PICO仿真.py", "启动真机.py"):
        assert not (ROOT / name).exists()
    for name in ("start_simulation.ps1", "start_real_robot.ps1"):
        assert not (ROOT / "scripts/setup" / name).exists()
    for name in ("启动无手柄仿真.cmd", "启动PICO仿真.cmd", "启动真机.cmd"):
        assert not (ROOT / name).exists()
    assert (ROOT / "scripts/setup/connect_pico_wsl.ps1").is_file()


@pytest.mark.skipif(os.name != "nt", reason="Actual WSL preview requires a Windows drive root")
@pytest.mark.parametrize("mode,source", [("sim", "manual"), ("pico", "pico")])
def test_actual_python_sim_cli_check_only_from_other_cwd(tmp_path, mode, source):
    result = subprocess.run(
        [sys.executable, "-B", str(ROOT / "start.py"), mode, "--check-only"],
        cwd=tmp_path,
        capture_output=True,
        text=True,
        encoding="utf-8",
        timeout=30,
    )
    assert result.returncode == 0, result.stdout + result.stderr
    assert "CheckOnly" in result.stdout and f"input_source:={source}" in result.stdout
    assert "mode:=sim" in result.stdout
    if source == "pico":
        assert "不枚举网卡" in result.stdout
    assert "[3/3]" not in result.stdout


def test_run_command_uses_argv_current_python_repo_cwd_and_utf8(monkeypatch, tmp_path):
    root = tmp_path / "repo space ' quote & 中文"
    root.mkdir()
    command = [sys.executable, "-u", str(root / "teleop.py"), "argument ' & $ quoted"]
    native = Mock()
    monkeypatch.setattr(launcher, "ROOT", root)
    monkeypatch.setattr(subprocess, "run", native)
    launcher.run_command(command, timeout=17)
    args, kwargs = native.call_args
    assert args == (command,)
    assert kwargs["cwd"] == root and kwargs["check"] is True and kwargs["timeout"] == 17
    assert kwargs["env"]["PYTHONUTF8"] == "1"
    assert kwargs["env"].get("PATH") == os.environ.get("PATH")
    assert "shell" not in kwargs  # No string interpolation or shell re-parsing of argv.


def test_structured_network_query_uses_encoded_powershell_not_ipconfig(monkeypatch):
    native = Mock(return_value=SimpleNamespace(stdout=json.dumps([candidate()])))
    monkeypatch.setattr(subprocess, "run", native)
    assert launcher.get_pico_candidates() == [candidate()]
    command = native.call_args.args[0]
    source = base64.b64decode(command[-1]).decode("utf-16-le")
    assert "-EncodedCommand" in command
    assert "Get-NetAdapter -Physical" in source and "Get-NetIPAddress" in source
    assert "ipconfig" not in source.lower()
    assert native.call_args.kwargs["check"] is True


@pytest.mark.parametrize("response", ["{}", '["not-a-dict"]', "not-json"])
def test_invalid_network_query_fails_closed(monkeypatch, response):
    monkeypatch.setattr(subprocess, "run", Mock(return_value=SimpleNamespace(stdout=response)))
    with pytest.raises(ValueError):
        launcher.get_pico_candidates()


@pytest.mark.parametrize("admin", [False, True])
def test_network_elevates_only_encoded_helper_and_propagates_failure(monkeypatch, tmp_path, admin):
    helper = tmp_path / "scripts/setup/connect_pico_wsl.ps1"
    helper.parent.mkdir(parents=True)
    helper.write_text("# never executed", encoding="utf-8")
    monkeypatch.setattr(launcher, "ROOT", tmp_path)
    monkeypatch.setattr(
        launcher,
        "ctypes",
        SimpleNamespace(windll=SimpleNamespace(shell32=SimpleNamespace(IsUserAnAdmin=lambda: admin))),
    )
    keepalive = Mock()
    keepalive.poll.return_value = None
    monkeypatch.setattr(subprocess, "Popen", Mock(return_value=keepalive))
    native = Mock(side_effect=subprocess.CalledProcessError(1, ["network"]))
    monkeypatch.setattr(launcher, "run_command", native)
    with pytest.raises(subprocess.CalledProcessError):
        launcher.setup_pico_network("192.168.1.2", "RM65-Servo")
    native.assert_called_once()
    outer = base64.b64decode(native.call_args.args[0][-1]).decode("utf-16-le")
    if admin:
        assert "$env:WSL_UTF8 = '1'" in outer
        assert "[Console]::OutputEncoding" in outer
        assert "Start-Transcript" in outer and "-ListenAddress $address" in outer
        assert "-Verb RunAs" not in outer
    else:
        assert "-Verb RunAs" in outer and "-Wait -PassThru" in outer and "exit $child.ExitCode" in outer
        assert "teleop.py" not in outer and sys.executable not in outer


def run_migration(tmp_path, helper, state, address="192.168.1.2", distro="RM65-Servo", suffix=""):
    script = tmp_path / "migration.ps1"
    script.write_text(
        "$ErrorActionPreference = 'Stop'\n$global:picoCalls = [Collections.Generic.List[string]]::new()\n"
        + launcher.network_migration_source(address, distro, state, helper)
        + suffix,
        encoding="utf-8-sig",
    )
    return subprocess.run(
        [POWERSHELL, "-NoProfile", "-NonInteractive", "-ExecutionPolicy", "Bypass", "-File", str(script)],
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
        timeout=20,
    )


@WINDOWS_PS
@pytest.mark.parametrize("previous", [None, "192.168.1.2", "192.168.9.2"])
def test_network_migration_removes_only_different_recorded_listen_address(tmp_path, previous):
    helper = tmp_path / "network ' helper.ps1"
    helper.write_text(
        """[CmdletBinding(SupportsShouldProcess=$true)]
param([switch]$Remove, [string]$ListenAddress, [string]$Distro)
$action = if ($Remove) { 'remove' } else { 'configure' }
$phase = if ($WhatIfPreference) { 'preview' } else { 'apply' }
$global:picoCalls.Add("$($action):$phase")
""",
        encoding="utf-8-sig",
    )
    state = tmp_path / "pico ' state.json"
    if previous is not None:
        state.write_text(json.dumps({"listen_address": previous}), encoding="utf-8")
    expected = ["configure:preview", "configure:apply"]
    if previous and previous != "192.168.1.2":
        expected = ["remove:preview", "remove:apply", *expected]
    result = run_migration(
        tmp_path, helper, state, suffix="\n[Console]::WriteLine('CALLS=' + ($global:picoCalls -join ','))\n"
    )
    assert result.returncode == 0, result.stderr
    assert "CALLS=" + ",".join(expected) in result.stdout


@WINDOWS_PS
def test_migration_does_not_configure_when_helper_refuses_old_ownership(tmp_path):
    helper = tmp_path / "network_helper.ps1"
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
    result = run_migration(tmp_path, helper, state)
    assert result.returncode != 0
    assert "OLD_RULE_OWNERSHIP_REFUSED" in result.stderr
    assert "NEW_CONFIGURATION_MUST_NOT_RUN" not in result.stderr


@WINDOWS_PS
def test_migration_quotes_paths_and_arguments_without_interpolation(tmp_path):
    helper = tmp_path / "stub ' $ helper.ps1"
    helper.write_text(
        """[CmdletBinding(SupportsShouldProcess=$true)]
param([switch]$Remove, [string]$ListenAddress, [string]$Distro)
if ($Distro -cne 'a'';$(throw ''INJECTED'')#') { throw 'WRONG_LITERAL' }
""",
        encoding="utf-8-sig",
    )
    result = run_migration(tmp_path, helper, tmp_path / "state ' $.json", distro="a';$(throw 'INJECTED')#")
    assert result.returncode == 0, result.stderr


@pytest.mark.parametrize(
    "outcome,expected",
    [
        (0, 0),
        (7, 7),
        (KeyboardInterrupt(), 130),
        (OSError("missing tool"), 2),
        (subprocess.CalledProcessError(4, ["child"]), 2),
    ],
)
def test_main_preserves_results_and_reports_interrupts(monkeypatch, capsys, outcome, expected):
    run = Mock(side_effect=outcome) if isinstance(outcome, BaseException) else Mock(return_value=outcome)
    monkeypatch.setattr(launcher, "run", run)
    assert launcher.main("manual", []) == expected
    if expected == 130:
        assert "实体静止" in capsys.readouterr().out


@pytest.mark.parametrize(
    "tty,pause,owned,expected",
    [(False, True, False, 0), (True, False, False, 0), (True, True, False, 1), (True, False, True, 1)],
)
def test_cli_pause_only_for_interactive_requested_or_owned_console(monkeypatch, tmp_path, tty, pause, owned, expected):
    monkeypatch.setattr(launcher, "ROOT", tmp_path)
    monkeypatch.setattr(sys, "argv", ["shortcut.py", *(["--pause"] if pause else [])])
    monkeypatch.setattr(sys, "stdin", SimpleNamespace(isatty=lambda: tty))
    monkeypatch.setattr(launcher, "owns_console", lambda: owned)
    monkeypatch.setattr(launcher, "main", Mock(return_value=7))
    prompt = Mock(return_value="")
    monkeypatch.setattr(builtins, "input", prompt)
    assert launcher.cli("manual") == 7
    assert prompt.call_count == expected
    logs = list((tmp_path / "build/logs/startup").glob("*.log"))
    assert len(logs) == 1 and "退出 7" in logs[0].read_text(encoding="utf-8")


@pytest.mark.parametrize("code", [0, 2])
def test_cli_converts_parser_system_exit_without_losing_exit_code(monkeypatch, tmp_path, code):
    monkeypatch.setattr(launcher, "ROOT", tmp_path)
    monkeypatch.setattr(launcher, "owns_console", lambda: False)
    monkeypatch.setattr(launcher, "main", Mock(side_effect=SystemExit(code)))
    monkeypatch.setattr(sys, "argv", ["shortcut.py"])
    assert launcher.cli("manual") == code
