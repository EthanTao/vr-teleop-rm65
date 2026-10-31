"""Startup diagnostics run without WSL, UAC, SSH, or device access."""

from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace

import pytest

from scripts.setup import launcher


def test_run_command_uses_argv_root_and_current_environment(monkeypatch):
    calls = []
    monkeypatch.setattr(launcher.subprocess, "run", lambda command, **kwargs: calls.append((command, kwargs)))
    command = [sys.executable, "-c", "print('项目 O\\'Brien; $()')"]
    launcher.run_command(command, timeout=12)
    assert calls[0][0] == command
    assert calls[0][1]["cwd"] == launcher.ROOT
    assert calls[0][1]["env"]["PYTHONUTF8"] == "1"
    assert calls[0][1]["env"]["WSL_UTF8"] == "1"
    assert calls[0][1]["check"] is True
    assert calls[0][1]["timeout"] == 12
    assert "shell" not in calls[0][1]


@pytest.mark.parametrize(
    "failure,code,text",
    [
        (KeyboardInterrupt(), 130, "不等于实体静止"),
        (subprocess.TimeoutExpired(["wsl.exe"], 90), 2, "90 秒"),
        (subprocess.CalledProcessError(7, ["wsl.exe"]), 2, "退出码 7"),
        (ModuleNotFoundError("No module named yaml"), 2, "pip install PyYAML"),
    ],
)
def test_failures_are_visible_and_never_success(monkeypatch, capsys, failure, code, text):
    def fail(*args):
        raise failure

    monkeypatch.setattr(launcher, "run", fail)
    assert launcher.main("manual", ["--check-only"]) == code
    assert text in capsys.readouterr().out


def configure_cli(monkeypatch, tmp_path, argv):
    monkeypatch.setattr(launcher, "ROOT", tmp_path)
    monkeypatch.setattr(sys, "argv", ["entry.py", *argv])
    monkeypatch.setattr(launcher, "owns_console", lambda: False)


def test_cli_writes_utf8_journal_and_never_prompts_in_automation(monkeypatch, tmp_path):
    configure_cli(monkeypatch, tmp_path, ["--pause"])
    monkeypatch.setattr(sys, "stdin", SimpleNamespace(isatty=lambda: False))
    monkeypatch.setattr("builtins.input", lambda *a: pytest.fail("noninteractive prompt"))

    def fail(kind):
        launcher.status("启动测试异常：中文路径")
        return 2

    monkeypatch.setattr(launcher, "main", fail)
    assert launcher.cli("manual") == 2
    logs = list((tmp_path / "build/logs/startup").glob("*.log"))
    assert len(logs) == 1
    assert "启动测试异常：中文路径" in logs[0].read_text(encoding="utf-8")
    assert "退出 2" in logs[0].read_text(encoding="utf-8")


@pytest.mark.parametrize("explicit,owned", [(True, False), (False, True)])
def test_pause_only_for_requested_or_explorer_console(monkeypatch, tmp_path, explicit, owned):
    configure_cli(monkeypatch, tmp_path, ["--pause"] if explicit else [])
    monkeypatch.setattr(launcher, "owns_console", lambda: owned)
    monkeypatch.setattr(sys, "stdin", SimpleNamespace(isatty=lambda: True))
    monkeypatch.setattr(launcher, "main", lambda kind: 0)
    prompts = []
    monkeypatch.setattr("builtins.input", lambda prompt: prompts.append(prompt))
    assert launcher.cli("manual") == 0
    assert len(prompts) == 1


def test_argument_error_keeps_failure_exit_code_and_journal(monkeypatch, tmp_path):
    configure_cli(monkeypatch, tmp_path, ["--wrong-option"])
    assert launcher.cli("manual") == 2
    log = next((tmp_path / "build/logs/startup").glob("*.log"))
    assert "退出 2" in log.read_text(encoding="utf-8")


def test_missing_entry_refuses_before_external_calls(monkeypatch, tmp_path):
    monkeypatch.setattr(launcher, "ROOT", tmp_path)
    monkeypatch.setattr(launcher, "run_command", lambda *a, **k: pytest.fail("external call"))
    monkeypatch.setattr(launcher, "get_pico_candidates", lambda: pytest.fail("network query"))
    assert launcher.main("pico", []) == 2
