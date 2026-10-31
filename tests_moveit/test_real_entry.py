"""Real shortcut regression checks: no SSH, deployment, controller or arm access."""

from __future__ import annotations

from pathlib import Path
import subprocess
import sys
from unittest.mock import Mock

import pytest

import servo_launcher
from scripts.setup import launcher

ROOT = Path(__file__).resolve().parents[1]
ENTRY = ROOT / "启动真机.py"
UDP = ["--udp-ip", "192.0.2.20"]
LIVE = [*UDP, "--hardware-validation", "test-only.yaml", "--live-ack", "RM65 LIVE VALIDATION"]
TEMPLATE = "ros2_ws/src/rm65_servo_teleop/config/hardware_validation.example.yaml"


def forbidden(*args, **kwargs):
    pytest.fail("Rejected/offline real startup attempted a process, network query or elevation")


def run_entry(tmp_path, options):
    # This subprocess launches only Python itself. --check-only is independently
    # tested in-process with subprocess.run and network calls replaced by failures.
    return subprocess.run(
        [sys.executable, "-B", str(ENTRY), *options],
        cwd=tmp_path,
        input="",
        capture_output=True,
        text=True,
        encoding="utf-8",
        timeout=30,
    )


def test_actual_real_python_without_arguments_is_closed_by_default(tmp_path):
    result = run_entry(tmp_path, [])
    assert result.returncode == 2, result.stdout + result.stderr
    for required in ("--udp-ip", "--hardware-validation", "--live-ack"):
        assert required in result.stdout
    assert "尚未启动真机" in result.stdout
    assert "先同步" not in result.stdout and "docker compose" not in result.stdout


@pytest.mark.parametrize(
    "options",
    [
        [],
        ["--check-only"],
        ["--dry-run"],
        [*UDP, "--hardware-validation", "test-only.yaml"],
        [*UDP, "--live-ack", "RM65 LIVE VALIDATION"],
        [*UDP, "--hardware-validation", "test-only.yaml", "--live-ack", "wrong"],
        ["--dry-run", "--udp-ip", "192.0.2.020"],
        ["--dry-run", "--udp-ip", "::1"],
        ["--dry-run", "--udp-ip", "0.0.0.0"],
        ["--dry-run", "--udp-ip", "127.0.0.1"],
        ["--dry-run", "--udp-ip", "224.0.0.1"],
        ["--dry-run", *UDP, "--arm-ip", "127.0.0.1"],
    ],
)
def test_incomplete_or_invalid_robot_options_fail_before_any_side_effect(monkeypatch, options):
    monkeypatch.setattr(launcher, "run_command", forbidden)
    monkeypatch.setattr(launcher, "get_pico_candidates", forbidden)
    monkeypatch.setattr(launcher, "setup_pico_network", forbidden)
    monkeypatch.setattr(launcher, "preview", forbidden)
    monkeypatch.setattr(subprocess, "run", forbidden)
    assert launcher.main("real", options) == 2


@pytest.mark.parametrize("dry_run", [False, True])
def test_explicit_robot_check_only_forwards_same_chain_in_process(monkeypatch, dry_run):
    preview = Mock(return_value=0)
    monkeypatch.setattr(servo_launcher, "main", preview)
    monkeypatch.setattr(subprocess, "run", forbidden)
    monkeypatch.setattr(launcher, "run_command", forbidden)
    monkeypatch.setattr(launcher, "get_pico_candidates", forbidden)
    monkeypatch.setattr(launcher, "setup_pico_network", forbidden)
    options = [*UDP, "--dry-run"] if dry_run else LIVE
    assert launcher.main("real", [*options, "--check-only"]) == 0
    forwarded = preview.call_args.args[0]
    assert forwarded[:4] == ["--mode", "hardware-dry-run" if dry_run else "hardware", "--target", "remote"]
    for key, value in [
        ("--input", "pico"),
        ("--remote-dir", "EthanTao"),
        ("--host", "rm"),
        ("--arm-ip", "192.168.10.18"),
        ("--udp-ip", "192.0.2.20"),
    ]:
        assert forwarded[forwarded.index(key) + 1] == value
    assert forwarded[-1] == "--print-command"
    if dry_run:
        assert "--live-ack" not in forwarded and "--hardware-validation" not in forwarded
    else:
        assert forwarded[forwarded.index("--live-ack") + 1] == "RM65 LIVE VALIDATION"
        assert forwarded[forwarded.index("--hardware-validation") + 1] == "test-only.yaml"


@pytest.mark.parametrize("dry_run", [False, True])
def test_live_start_must_pass_offline_validation_before_current_python_child(monkeypatch, dry_run):
    calls = []
    monkeypatch.setattr(launcher, "preview", lambda options: calls.append(("preview", options)) or 0)
    monkeypatch.setattr(launcher, "run_command", lambda command: calls.append(("child", command)))
    monkeypatch.setattr(launcher, "get_pico_candidates", forbidden)
    monkeypatch.setattr(launcher, "setup_pico_network", forbidden)
    options = [*UDP, "--dry-run"] if dry_run else LIVE
    assert launcher.main("real", options) == 0
    assert [call[0] for call in calls] == ["preview", "child"]
    assert calls[1][1] == [sys.executable, "-X", "utf8", "-u", str(ROOT / "teleop.py"), *calls[0][1]]
    assert "--print-command" not in calls[1][1]


@pytest.mark.parametrize("code", [2, 17, 130])
def test_offline_validation_failure_blocks_all_real_subprocesses(monkeypatch, code):
    monkeypatch.setattr(launcher, "preview", Mock(return_value=code))
    monkeypatch.setattr(launcher, "run_command", forbidden)
    assert launcher.main("real", LIVE) == code


@pytest.mark.parametrize("check_only", [False, True])
def test_template_rejected_in_process_without_ssh_or_network(monkeypatch, check_only, capsys):
    monkeypatch.setattr(subprocess, "run", forbidden)
    monkeypatch.setattr(launcher, "run_command", forbidden)
    monkeypatch.setattr(launcher, "setup_pico_network", forbidden)
    options = [*UDP, "--hardware-validation", TEMPLATE, "--live-ack", "RM65 LIVE VALIDATION"]
    assert launcher.main("real", [*options, *(["--check-only"] if check_only else [])]) == 2
    output = capsys.readouterr().out
    assert "现场验证记录不完整" in output
    assert "先同步" not in output and "docker compose" not in output


def test_actual_dry_run_check_only_is_offline_and_uses_board_chain(tmp_path):
    result = run_entry(tmp_path, ["--dry-run", *UDP, "--check-only"])
    assert result.returncode == 0, result.stdout + result.stderr
    for marker in ("CheckOnly", "mode:=hardware-dry-run", "input_source:=pico", "udp_ip:=192.0.2.20"):
        assert marker in result.stdout
    assert "[真机] Dry Run 将连接" not in result.stdout
    assert "--print-command" not in result.stdout  # Already processed in-process, not another Python child.


def test_actual_live_preview_keeps_existing_template_rejection(tmp_path):
    result = run_entry(
        tmp_path, [*UDP, "--hardware-validation", TEMPLATE, "--live-ack", "RM65 LIVE VALIDATION", "--check-only"]
    )
    assert result.returncode == 2, result.stdout + result.stderr
    assert "现场验证记录不完整" in result.stdout
    assert "先同步" not in result.stdout and "docker compose" not in result.stdout


def test_real_arguments_preserve_spaces_quotes_and_ack_as_single_argv_values(monkeypatch, tmp_path):
    # Values remain separate argv elements until the existing teleop parser validates them.
    root = tmp_path / "repo ' space & 中文"
    root.mkdir()
    (root / "teleop.py").write_text("# stub, never run", encoding="utf-8")
    monkeypatch.setattr(launcher, "ROOT", root)
    host = "operator@board"
    remote = "folder ' quoted & $ value"
    validation = "ros2_ws/config/field ' validated.yaml"
    options = [
        *UDP,
        "--host",
        host,
        "--remote-dir",
        remote,
        "--arm-ip",
        "192.0.2.18",
        "--hardware-validation",
        validation,
        "--live-ack",
        launcher.ACK,
    ]
    preview = Mock(return_value=0)
    native = Mock()
    monkeypatch.setattr(launcher, "preview", preview)
    monkeypatch.setattr(launcher, "run_command", native)
    assert launcher.main("real", options) == 0
    argv = native.call_args.args[0]
    assert argv[:5] == [sys.executable, "-X", "utf8", "-u", str(root / "teleop.py")]
    for key, value in [
        ("--host", host),
        ("--remote-dir", remote),
        ("--hardware-validation", validation),
        ("--live-ack", launcher.ACK),
        ("--arm-ip", "192.0.2.18"),
    ]:
        assert argv[argv.index(key) + 1] == value
    assert argv[5:] == preview.call_args.args[0]


def test_missing_shared_teleop_entry_fails_without_child(monkeypatch, tmp_path):
    monkeypatch.setattr(launcher, "ROOT", tmp_path)
    monkeypatch.setattr(launcher, "preview", Mock(return_value=0))
    monkeypatch.setattr(launcher, "run_command", forbidden)
    assert launcher.main("real", [*UDP, "--dry-run"]) == 2
