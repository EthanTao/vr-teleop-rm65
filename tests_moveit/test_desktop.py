"""Desktop routing and admission checks; never create a window or contact devices."""

import io
import queue
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

import start
from scripts.setup import desktop, menu


def forbidden(*args, **kwargs):
    pytest.fail("Desktop command construction attempted an external action")


def test_default_entry_opens_desktop_and_returns_its_exit_code(monkeypatch):
    app = Mock(return_value=23)
    monkeypatch.setattr(desktop, "main", app)
    monkeypatch.setattr(desktop.subprocess, "Popen", forbidden)
    assert start.main([]) == 23
    app.assert_called_once_with()


def test_offline_menu_preview_does_not_open_desktop(monkeypatch):
    monkeypatch.setattr(desktop, "main", forbidden)
    dispatch = Mock(return_value=0)
    monkeypatch.setattr(start.launcher, "cli", dispatch)
    assert start.main(["--check-only"]) == 0
    dispatch.assert_called_once_with("menu", ["--check-only"])


@pytest.mark.parametrize("kind", ["sim", "pico"])
def test_simulation_builder_preserves_source_and_offline_option(monkeypatch, kind):
    monkeypatch.setattr(desktop.subprocess, "Popen", forbidden)
    command = desktop.simulation_command(kind, distro="RM65-Servo", address="192.0.2.50", rviz=True, preview=True)
    assert command[5:] == [
        kind,
        "--distro",
        "RM65-Servo",
        *(["--listen-address", "192.0.2.50"] if kind == "pico" else []),
        "--rviz",
        "--check-only",
    ]


def test_board_query_retains_literal_settings_and_rejects_unlisted_actions():
    settings = menu.BoardSettings(host="rm", remote_dir="/home/rm/project ' with spaces")
    command = desktop.board_command("status", settings)
    assert command[-5:] == ["status", "rm", settings.remote_dir, settings.arm_ip, settings.udp_ip]
    with pytest.raises(ValueError):
        desktop.board_command("stop", settings)


def test_invalid_gui_field_is_validation_error_instead_of_parser_exit():
    with pytest.raises(ValueError):
        desktop.simulation_command("sim", distro="--invalid-distro", preview=True)


@pytest.mark.parametrize("record,ack", [("record.yaml", ""), ("", "RM65 LIVE VALIDATION")])
def test_real_builder_refuses_missing_record_or_operator_ack(record, ack):
    with pytest.raises(ValueError):
        desktop.execution_command(menu.BoardSettings(), record, ack, preview=False)


def test_real_builder_preserves_exact_manual_ack_and_offline_mode():
    command = desktop.execution_command(
        menu.BoardSettings(), "record ' space.yaml", "RM65 LIVE VALIDATION", preview=True
    )
    assert command[5] == "real"
    assert command[command.index("--hardware-validation") + 1] == "record ' space.yaml"
    assert command[command.index("--live-ack") + 1] == "RM65 LIVE VALIDATION"
    assert command[-1] == "--check-only"


def test_full_log_queue_cannot_discard_task_completion():
    task = desktop.SessionTask.__new__(desktop.SessionTask)
    task.events = queue.Queue(maxsize=1)
    task.events.put(("line", "old display line"))
    task._emit("done", 0)
    assert task.events.get_nowait() == ("done", 0)


def test_owned_windows_interrupt_maps_to_standard_interrupted_exit():
    task = desktop.SessionTask.__new__(desktop.SessionTask)
    task.events = queue.Queue()
    task.interrupt_requested = True
    task.process = SimpleNamespace(stdout=io.StringIO("sample output\n"), wait=lambda: 0xC000013A)
    task._read()
    assert task.events.get_nowait() == ("line", "sample output\n")
    assert task.events.get_nowait() == ("done", 130)
