"""Interactive Servo startup contract; all external operations are mocked closed."""

from __future__ import annotations

import builtins
from dataclasses import asdict, is_dataclass
from pathlib import Path
import runpy
import shlex
import subprocess
import sys
from unittest.mock import Mock

import pytest

from scripts.setup import launcher, menu

ROOT = Path(__file__).resolve().parents[1]
TEMPLATE = "ros2_ws/src/rm65_servo_teleop/config/hardware_validation.example.yaml"


def forbidden(*args, **kwargs):
    pytest.fail("menu test attempted an external process, network operation or unexpected prompt")


@pytest.fixture(autouse=True)
def closed_environment(monkeypatch):
    monkeypatch.setattr(subprocess, "run", forbidden)
    monkeypatch.setattr(subprocess, "Popen", forbidden)
    monkeypatch.setattr(launcher, "run_command", forbidden)
    monkeypatch.setattr(launcher, "get_pico_candidates", forbidden)
    monkeypatch.setattr(launcher, "setup_pico_network", forbidden)
    monkeypatch.setattr(builtins, "input", forbidden)
    set_tty(monkeypatch, False, False)


def set_tty(monkeypatch, stdin=True, stdout=True):
    monkeypatch.setattr(sys.stdin, "isatty", lambda: stdin)
    monkeypatch.setattr(sys.stdout, "isatty", lambda: stdout)


def answers(monkeypatch, *values):
    remaining = iter(values)
    prompts = []

    def read(prompt=""):
        prompts.append(prompt)
        try:
            value = next(remaining)
        except StopIteration:
            pytest.fail(f"unexpected extra menu prompt: {prompt}")
        if isinstance(value, BaseException):
            raise value
        return value

    monkeypatch.setattr(builtins, "input", read)
    return prompts


def test_board_settings_dataclass_has_connection_defaults():
    settings = menu.BoardSettings()
    assert is_dataclass(settings)
    assert asdict(settings) == {
        "host": menu.DEFAULT_HOST,
        "remote_dir": menu.DEFAULT_REMOTE_DIR,
        "udp_ip": "192.168.10.100",
        "arm_ip": "192.168.10.18",
    }


@pytest.mark.parametrize("real_only", [False, True])
@pytest.mark.parametrize("choice", ["", "0"])
def test_empty_or_zero_menu_exits_without_dispatch(monkeypatch, real_only, choice):
    set_tty(monkeypatch)
    answers(monkeypatch, choice)
    monkeypatch.setattr(launcher, "main", forbidden)
    monkeypatch.setattr(menu, "run_board_action", forbidden)
    assert menu.run_menu(real_only=real_only) == 0


@pytest.mark.parametrize("choice,kind", [("1", "manual"), ("2", "pico")])
def test_root_simulation_routes_existing_launcher_without_extra_flags(monkeypatch, choice, kind):
    set_tty(monkeypatch)
    answers(monkeypatch, choice)
    dispatch = Mock(return_value=17)
    monkeypatch.setattr(launcher, "main", dispatch)
    monkeypatch.setattr(menu, "run_board_action", forbidden)
    assert menu.run_menu() == 17
    dispatch.assert_called_once_with(kind, [])


@pytest.mark.parametrize(
    "real_only,choice,action",
    [(False, "3", "start-readonly"), (True, "1", "start-readonly"), (True, "2", "status"), (True, "3", "input")],
)
def test_board_choices_use_only_whitelisted_action_and_safe_defaults(monkeypatch, real_only, choice, action):
    set_tty(monkeypatch)
    prompts = answers(monkeypatch, choice)
    dispatch = Mock(return_value=19)
    monkeypatch.setattr(menu, "run_board_action", dispatch)
    monkeypatch.setattr(launcher, "main", forbidden)
    assert menu.run_menu(real_only=real_only) == 19
    dispatch.assert_called_once_with(
        action,
        menu.DEFAULT_HOST,
        menu.DEFAULT_REMOTE_DIR,
        arm_ip="192.168.10.18",
        udp_ip="192.168.10.100",
    )
    assert len(prompts) == 1


def test_formal_dispatch_preserves_exact_values_as_separate_argv(monkeypatch):
    host = "operator@board.example"
    remote = "/home/rm/项目 O'Brien; $(not-executed)"
    validation = " ros2_ws/config/field ' approved & $ file.yaml "
    prompts = answers(monkeypatch, validation, launcher.ACK)
    settings = menu.BoardSettings(host, remote, "192.0.2.20", "192.0.2.18")
    dispatch = Mock(return_value=23)
    monkeypatch.setattr(launcher, "main", dispatch)
    monkeypatch.setattr(menu, "run_board_action", forbidden)
    assert menu.run_execution(settings) == 23
    assert len(prompts) == 2
    dispatch.assert_called_once()
    kind, argv = dispatch.call_args.args
    assert kind == "real"
    assert len(argv) == 12
    assert dict(zip(argv[::2], argv[1::2])) == {
        "--host": host,
        "--remote-dir": remote,
        "--udp-ip": "192.0.2.20",
        "--arm-ip": "192.0.2.18",
        "--hardware-validation": validation,
        "--live-ack": launcher.ACK,
    }


@pytest.mark.parametrize("real_only", [False, True])
def test_formal_defaults_only_ask_validation_and_ack(monkeypatch, real_only):
    set_tty(monkeypatch)
    prompts = answers(monkeypatch, "4", "field-validation.yaml", launcher.ACK)
    dispatch = Mock(return_value=23)
    monkeypatch.setattr(launcher, "main", dispatch)
    assert menu.run_menu(real_only=real_only) == 23
    dispatch.assert_called_once()
    kind, argv = dispatch.call_args.args
    assert kind == "real"
    assert len(argv) == 12
    assert dict(zip(argv[::2], argv[1::2])) == {
        "--host": "rm@192.168.0.115",
        "--remote-dir": menu.DEFAULT_REMOTE_DIR,
        "--udp-ip": "192.168.10.100",
        "--arm-ip": "192.168.10.18",
        "--hardware-validation": "field-validation.yaml",
        "--live-ack": launcher.ACK,
    }
    assert len(prompts) == 3


@pytest.mark.parametrize("explicit_defaults", [False, True])
def test_direct_execution_defaults_normalize_bare_board_address(monkeypatch, explicit_defaults):
    prompts = answers(monkeypatch, "field-validation.yaml", launcher.ACK)
    dispatch = Mock(return_value=0)
    monkeypatch.setattr(launcher, "main", dispatch)
    assert menu.run_execution(menu.BoardSettings() if explicit_defaults else None) == 0
    assert dispatch.call_args.args[0] == "real"
    options = dict(zip(dispatch.call_args.args[1][::2], dispatch.call_args.args[1][1::2]))
    assert options["--host"] == "rm@192.168.0.115"
    assert len(prompts) == 2


@pytest.mark.parametrize("values", [("",), ("field-validation.yaml", "")])
def test_empty_validation_or_ack_cancels_execution_without_dispatch(monkeypatch, values):
    prompts = answers(monkeypatch, *values)
    monkeypatch.setattr(launcher, "main", forbidden)
    assert menu.run_execution() is None
    assert len(prompts) == len(values)


@pytest.mark.parametrize("ack", ["wrong", " " + launcher.ACK, launcher.ACK + " ", " "])
def test_ack_must_be_exact_before_formal_dispatch(monkeypatch, ack):
    answers(monkeypatch, "field-validation.yaml", ack)
    monkeypatch.setattr(launcher, "main", forbidden)
    with pytest.raises(ValueError):
        menu.run_execution()


def test_example_validation_template_is_rejected_before_network(monkeypatch):
    answers(monkeypatch, TEMPLATE, launcher.ACK)
    assert menu.run_execution() == 2


@pytest.mark.parametrize("real_only", [False, True])
def test_rejected_execution_returns_to_menu_without_retry_or_fallback(monkeypatch, capsys, real_only):
    set_tty(monkeypatch)
    answers(monkeypatch, "4", "0")
    dispatch = Mock(return_value=2)
    monkeypatch.setattr(menu, "run_execution", dispatch)
    monkeypatch.setattr(menu, "run_board_action", forbidden)
    monkeypatch.setattr(launcher, "main", forbidden)
    assert menu.run_menu(real_only=real_only) == 0
    dispatch.assert_called_once_with(menu.BoardSettings())
    assert capsys.readouterr().out.count("[5]") == 2


@pytest.mark.parametrize("real_only", [False, True])
def test_actual_template_rejection_returns_to_menu_without_network(monkeypatch, capsys, real_only):
    set_tty(monkeypatch)
    answers(monkeypatch, "4", TEMPLATE, launcher.ACK, "0")
    monkeypatch.setattr(menu, "run_board_action", forbidden)
    assert menu.run_menu(real_only=real_only) == 0
    output = capsys.readouterr().out
    assert "[未完成]" in output
    assert output.count("[5]") == 2


@pytest.mark.parametrize("real_only", [False, True])
@pytest.mark.parametrize("values", [("",), ("field-validation.yaml", "")])
def test_cancelled_execution_redisplays_menu_and_can_start_readonly(monkeypatch, capsys, real_only, values):
    set_tty(monkeypatch)
    choice = "1" if real_only else "3"
    answers(monkeypatch, "4", *values, choice)
    dispatch = Mock(return_value=19)
    monkeypatch.setattr(menu, "run_board_action", dispatch)
    monkeypatch.setattr(launcher, "main", forbidden)
    assert menu.run_menu(real_only=real_only) == 19
    dispatch.assert_called_once_with(
        "start-readonly",
        menu.DEFAULT_HOST,
        menu.DEFAULT_REMOTE_DIR,
        arm_ip="192.168.10.18",
        udp_ip="192.168.10.100",
    )
    assert capsys.readouterr().out.count("[5]") >= 2


@pytest.mark.parametrize("real_only", [False, True])
@pytest.mark.parametrize("ack", ["wrong", " " + launcher.ACK, launcher.ACK + " "])
def test_wrong_ack_prints_error_and_returns_to_menu(monkeypatch, capsys, real_only, ack):
    set_tty(monkeypatch)
    answers(monkeypatch, "4", "field-validation.yaml", ack, "0")
    monkeypatch.setattr(launcher, "main", forbidden)
    monkeypatch.setattr(menu, "run_board_action", forbidden)
    assert menu.run_menu(real_only=real_only) == 0
    output = capsys.readouterr().out
    assert "确认" in output
    assert output.count("[5]") >= 2


@pytest.mark.parametrize("real_only", [False, True])
@pytest.mark.parametrize("stdin,stdout", [(False, False), (True, False), (False, True)])
def test_interactive_menu_requires_both_tty_streams(monkeypatch, real_only, stdin, stdout):
    set_tty(monkeypatch, stdin, stdout)
    with pytest.raises(ValueError):
        menu.run_menu(real_only=real_only)


@pytest.mark.parametrize("real_only", [False, True])
@pytest.mark.parametrize("stdin,stdout", [(False, False), (True, False), (False, True), (True, True)])
def test_check_only_never_prompts_or_executes_and_describes_servo_topology(
    monkeypatch, capsys, real_only, stdin, stdout
):
    set_tty(monkeypatch, stdin, stdout)
    monkeypatch.setattr(launcher, "main", forbidden)
    monkeypatch.setattr(menu, "run_board_action", forbidden)
    assert menu.run_menu(check_only=True, real_only=real_only) == 0
    output = capsys.readouterr().out
    assert "PICO" in output
    assert "板端" in output
    assert "机械臂" in output
    assert "[5]" in output
    assert "高级" in output
    for value in asdict(menu.BoardSettings()).values():
        assert value in output
    if not real_only:
        assert "Servo" in output


@pytest.mark.parametrize(
    "prefix",
    [[], ["4"], ["4", "field-validation.yaml"], ["5"], ["5", "rm"], ["5", "rm", "/home/rm/project", "192.0.2.20"]],
)
def test_eof_at_menu_or_followup_prompt_cancels_without_dispatch(monkeypatch, prefix):
    set_tty(monkeypatch)
    answers(monkeypatch, *prefix, EOFError())
    monkeypatch.setattr(launcher, "main", forbidden)
    monkeypatch.setattr(menu, "run_board_action", forbidden)
    assert menu.run_menu() == 0


@pytest.mark.parametrize("prefix", [[], ["4"], ["4", "field-validation.yaml"], ["5"], ["5", "rm"]])
def test_ctrl_c_propagates_from_run_menu(monkeypatch, prefix):
    set_tty(monkeypatch)
    answers(monkeypatch, *prefix, KeyboardInterrupt())
    with pytest.raises(KeyboardInterrupt):
        menu.run_menu()


@pytest.mark.parametrize("choice", ["-1", "6", "start", "1; status", "$(touch NEVER)"])
def test_invalid_selection_is_closed_before_any_dispatch(monkeypatch, choice):
    set_tty(monkeypatch)
    answers(monkeypatch, choice, "0")
    monkeypatch.setattr(menu, "run_board_action", forbidden)
    assert launcher.main("menu", []) == 0


def test_edit_settings_updates_all_four_fields_without_mutating_original(monkeypatch):
    original = menu.BoardSettings()
    before = asdict(original)
    remote = "/home/rm/项目 O'Brien; $(not-executed)"
    values = ("operator@board.example", remote, "192.0.2.20", "192.0.2.18")
    prompts = answers(monkeypatch, *values)
    canonical = Mock(wraps=launcher.canonical_ipv4)
    monkeypatch.setattr(launcher, "canonical_ipv4", canonical)
    updated = menu.edit_settings(original)
    assert isinstance(updated, menu.BoardSettings)
    assert asdict(updated) == dict(zip(before, values))
    assert asdict(original) == before
    assert len(prompts) == 4
    assert {call.args[0] for call in canonical.call_args_list} >= {values[2], values[3]}


def test_empty_settings_prompts_keep_current_nondefault_values(monkeypatch):
    settings = menu.BoardSettings("operator@board.example", "/srv/custom project", "192.0.2.20", "192.0.2.18")
    before = asdict(settings)
    prompts = answers(monkeypatch, "", "", "", "")
    assert asdict(menu.edit_settings(settings)) == before
    assert asdict(settings) == before
    assert len(prompts) == 4


@pytest.mark.parametrize(
    "field,value",
    [("host", value) for value in ("-oProxyCommand=evil", "rm;touch NEVER", "rm board", "rm\nstatus", "999.1.1.1")]
    + [
        ("remote_dir", value)
        for value in (
            "relative",
            "~/project",
            "C:/project",
            "/home/../project",
            "/home/rm\\project",
            "/home/rm\nstatus",
            "/home/rm\x00",
        )
    ]
    + [
        (field, value)
        for field in ("udp_ip", "arm_ip")
        for value in ("127.0.0.1", "0.0.0.0", "192.168.10.010", "224.0.0.1", "255.255.255.255", "board.example")
    ],
)
def test_invalid_settings_are_rejected_atomically_before_network(monkeypatch, field, value):
    settings = menu.BoardSettings()
    before = asdict(settings)
    candidate = {
        "host": "operator@board.example",
        "remote_dir": "/srv/changed project",
        "udp_ip": "192.0.2.20",
        "arm_ip": "192.0.2.18",
    }
    candidate[field] = value
    answers(monkeypatch, *candidate.values())
    monkeypatch.setattr(launcher, "main", forbidden)
    monkeypatch.setattr(menu, "run_board_action", forbidden)
    with pytest.raises(ValueError):
        menu.edit_settings(settings)
    assert asdict(settings) == before
    monkeypatch.setattr(builtins, "input", forbidden)
    with pytest.raises(ValueError):
        menu.run_execution(menu.BoardSettings(**candidate))


@pytest.mark.parametrize(
    "real_only,choice,action",
    [(False, "3", "start-readonly"), (True, "1", "start-readonly"), (True, "2", "status"), (True, "3", "input")],
)
def test_settings_persist_through_redisplay_and_blank_edits_with_no_writes(
    monkeypatch, capsys, real_only, choice, action
):
    set_tty(monkeypatch)
    values = ("operator@board.example", "/srv/custom project", "192.0.2.20", "192.0.2.18")
    prompts = answers(monkeypatch, "5", *values, "5", "", "", "", "", choice)
    dispatch = Mock(return_value=19)
    monkeypatch.setattr(menu, "run_board_action", dispatch)
    monkeypatch.setattr(launcher, "main", forbidden)
    monkeypatch.setattr(Path, "write_text", forbidden)
    monkeypatch.setattr(Path, "write_bytes", forbidden)
    monkeypatch.setattr(builtins, "open", forbidden)
    assert menu.run_menu(real_only=real_only) == 19
    dispatch.assert_called_once_with(action, *values[:2], arm_ip=values[3], udp_ip=values[2])
    output = capsys.readouterr().out
    for value in values:
        assert value in output
    assert output.count("[5]") >= 3
    assert len(prompts) == 11


@pytest.mark.parametrize("field", ["host", "remote_dir", "udp_ip", "arm_ip"])
def test_invalid_edit_preserves_previous_session_settings(monkeypatch, capsys, field):
    set_tty(monkeypatch)
    original = ("operator@board.example", "/srv/original project", "192.0.2.20", "192.0.2.18")
    candidate = dict(
        zip(
            ("host", "remote_dir", "udp_ip", "arm_ip"),
            ("changed@board.example", "/srv/changed", "192.0.2.30", "192.0.2.28"),
        )
    )
    candidate[field] = {"host": "-oBad", "remote_dir": "relative", "udp_ip": "127.0.0.1", "arm_ip": "224.0.0.1"}[field]
    answers(monkeypatch, "5", *original, "5", *candidate.values(), "4", "field-validation.yaml", launcher.ACK)
    dispatch = Mock(return_value=23)
    monkeypatch.setattr(launcher, "main", dispatch)
    assert menu.run_menu() == 23
    kind, argv = dispatch.call_args.args
    assert kind == "real"
    options = dict(zip(argv[::2], argv[1::2]))
    assert tuple(options[key] for key in ("--host", "--remote-dir", "--udp-ip", "--arm-ip")) == original
    assert capsys.readouterr().out.count("[5]") >= 3


def test_advanced_settings_apply_to_formal_dispatch_without_extra_connection_prompts(monkeypatch):
    set_tty(monkeypatch)
    prompts = answers(
        monkeypatch, "5", "192.0.2.115", "/srv/project", "192.0.2.20", "192.0.2.18", "4", " field.yaml ", launcher.ACK
    )
    dispatch = Mock(return_value=23)
    monkeypatch.setattr(launcher, "main", dispatch)
    assert menu.run_menu() == 23
    dispatch.assert_called_once_with(
        "real",
        [
            "--host",
            "rm@192.0.2.115",
            "--remote-dir",
            "/srv/project",
            "--udp-ip",
            "192.0.2.20",
            "--arm-ip",
            "192.0.2.18",
            "--hardware-validation",
            " field.yaml ",
            "--live-ack",
            launcher.ACK,
        ],
    )
    assert len(prompts) == 8


def test_advanced_settings_are_not_retained_between_menu_sessions(monkeypatch):
    set_tty(monkeypatch)
    answers(monkeypatch, "5", "operator@board.example", "/srv/project", "192.0.2.20", "192.0.2.18", "0", "3")
    dispatch = Mock(return_value=19)
    monkeypatch.setattr(menu, "run_board_action", dispatch)
    assert menu.run_menu() == 0
    assert menu.run_menu() == 19
    dispatch.assert_called_once_with(
        "start-readonly",
        menu.DEFAULT_HOST,
        menu.DEFAULT_REMOTE_DIR,
        arm_ip="192.168.10.18",
        udp_ip="192.168.10.100",
    )


@pytest.mark.parametrize(
    "real_only,choice,target",
    [
        (False, "1", "launcher"),
        (False, "2", "launcher"),
        (False, "3", "run_board_action"),
        (True, "1", "run_board_action"),
        (True, "2", "run_board_action"),
        (True, "3", "run_board_action"),
        (False, "4", "run_execution"),
        (True, "4", "run_execution"),
        (False, "5", "edit_settings"),
        (True, "5", "edit_settings"),
    ],
)
def test_selected_action_value_error_is_printed_then_menu_retries(monkeypatch, capsys, real_only, choice, target):
    set_tty(monkeypatch)
    answers(monkeypatch, choice, "0")
    failure = Mock(side_effect=ValueError("selected action invalid"))
    if target == "launcher":
        monkeypatch.setattr(launcher, "main", failure)
    else:
        monkeypatch.setattr(menu, target, failure)
    assert menu.run_menu(real_only=real_only) == 0
    failure.assert_called_once()
    output = capsys.readouterr().out
    assert "selected action invalid" in output
    assert output.count("[5]") >= 2


@pytest.mark.parametrize("choice,target", [("3", "run_board_action"), ("4", "run_execution")])
@pytest.mark.parametrize(
    "failure",
    [
        subprocess.CalledProcessError(7, ["ssh"]),
        subprocess.TimeoutExpired(["ssh"], 90),
        OSError("SSH unavailable"),
        RuntimeError("unexpected failure"),
    ],
)
def test_menu_does_not_swallow_execution_or_network_failures(monkeypatch, choice, target, failure):
    set_tty(monkeypatch)
    answers(monkeypatch, choice)
    dispatch = Mock(side_effect=failure)
    monkeypatch.setattr(menu, target, dispatch)
    with pytest.raises(type(failure)):
        menu.run_menu()
    dispatch.assert_called_once()


@pytest.mark.parametrize("action", ["start-readonly", "status", "input"])
def test_board_ssh_argv_and_quoted_absolute_path_are_literal(monkeypatch, action):
    remote = "/home/rm/项目 O'Brien; $(touch NEVER)"
    calls = []
    monkeypatch.setattr(launcher, "run_command", lambda command, **kwargs: calls.append((command, kwargs)))
    assert menu.run_board_action(action, "operator@board.example", remote) == 0
    assert len(calls) == 1
    command, kwargs = calls[0]
    assert Path(command[0]).name.lower() in {"ssh", "ssh.exe"}
    assert command[1:-1] == ["-o", "BatchMode=yes", "-o", "ConnectTimeout=8", "--", "operator@board.example"]
    expected = [remote + "/environment.sh", action]
    if action == "start-readonly":
        expected += ["&&", remote + "/environment.sh", "status"]
    assert shlex.split(command[-1]) == expected
    assert kwargs == {"timeout": 90}


@pytest.mark.parametrize("action", ["start-readonly", "status", "input"])
def test_custom_addresses_reach_both_board_launch_and_status_commands(monkeypatch, action):
    calls = []
    monkeypatch.setattr(launcher, "run_command", lambda command, **kwargs: calls.append(command))
    assert (
        menu.run_board_action(
            action,
            "rm",
            "/home/rm/EthanTao",
            arm_ip="192.0.2.18",
            udp_ip="192.0.2.100",
        )
        == 0
    )
    options = ["--arm-ip", "192.0.2.18", "--udp-ip", "192.0.2.100"]
    expected = ["/home/rm/EthanTao/environment.sh", action, *options]
    if action == "start-readonly":
        expected += ["&&", "/home/rm/EthanTao/environment.sh", "status", *options]
    assert shlex.split(calls[0][-1]) == expected


@pytest.mark.parametrize(
    "action", ["", "stop", "restart", "sync", "build", "start", "status; stop", "start-readonly && input"]
)
def test_board_rejects_non_whitelisted_actions_before_process(action):
    with pytest.raises(ValueError):
        menu.run_board_action(action, "rm", "/home/rm/EthanTao")


@pytest.mark.parametrize(
    "host",
    ["", "-oProxyCommand=evil", "rm;touch NEVER", "rm\nstatus", "rm board", "rm$(touch NEVER)", "rm|cat", "rm\x00"],
)
def test_board_rejects_injected_or_empty_host_before_process(host):
    with pytest.raises(ValueError):
        menu.run_board_action("status", host, "/home/rm/EthanTao")


@pytest.mark.parametrize(
    "remote", ["", "EthanTao", "./EthanTao", "~/EthanTao", "C:/EthanTao", "/home/rm\nstatus", "/home/rm\x00"]
)
def test_board_rejects_relative_or_control_character_remote_dir_before_process(remote):
    with pytest.raises(ValueError):
        menu.run_board_action("status", "rm", remote)


@pytest.mark.parametrize(
    "host,expected",
    [
        ("rm", "rm"),
        ("rm-ts", "rm-ts"),
        ("100.127.90.0", "rm@100.127.90.0"),
        ("192.168.0.115", "rm@192.168.0.115"),
        ("rm@192.168.0.115", "rm@192.168.0.115"),
        ("operator@board.example", "operator@board.example"),
    ],
)
def test_bare_address_uses_board_user_instead_of_windows_account(host, expected):
    assert menu.normalize_host(host) == expected


@pytest.mark.parametrize("host", ["999.1.1.1", "-oProxyCommand=evil", "rm;stop"])
def test_invalid_bare_host_rejected_before_any_process(host):
    with pytest.raises(ValueError):
        menu.normalize_host(host)


def test_board_action_prefixes_user_for_bare_ip_before_ssh(monkeypatch):
    calls = []
    monkeypatch.setattr(launcher, "run_command", lambda command, **kwargs: calls.append(command))
    assert menu.run_board_action("status", "100.127.90.0", "/home/rm/EthanTao") == 0
    assert calls[0][-2] == "rm@100.127.90.0"


@pytest.mark.parametrize(
    "failure",
    [subprocess.CalledProcessError(7, ["ssh"]), subprocess.TimeoutExpired(["ssh"], 90), OSError("SSH unavailable")],
)
def test_board_command_failure_propagates_without_fallback(monkeypatch, failure):
    command = Mock(side_effect=failure)
    monkeypatch.setattr(launcher, "run_command", command)
    with pytest.raises(type(failure)):
        menu.run_board_action("start-readonly", "rm", "/home/rm/EthanTao")
    command.assert_called_once()


@pytest.mark.parametrize("failure,code", [(subprocess.CalledProcessError(7, ["ssh"]), 2), (KeyboardInterrupt(), 130)])
def test_launcher_menu_uses_existing_failure_path(monkeypatch, failure, code):
    monkeypatch.setattr(menu, "run_menu", Mock(side_effect=failure))
    assert launcher.main("menu", []) == code


@pytest.mark.parametrize("options,check_only", [([], False), (["--check-only"], True), (["--pause"], False)])
def test_launcher_menu_routes_without_adding_control_flags(monkeypatch, options, check_only):
    dispatch = Mock(return_value=29)
    monkeypatch.setattr(menu, "run_menu", dispatch)
    assert launcher.main("menu", options) == 29
    dispatch.assert_called_once()
    assert dispatch.call_args.kwargs.get("check_only", False) is check_only
    assert not dispatch.call_args.kwargs.get("real_only", False)


@pytest.mark.parametrize("options", [[], ["--pause"]])
def test_bare_real_shortcut_enters_real_menu_only_when_both_tty(monkeypatch, options):
    set_tty(monkeypatch)
    dispatch = Mock(return_value=31)
    monkeypatch.setattr(menu, "run_menu", dispatch)
    assert launcher.main("real", options) == 31
    dispatch.assert_called_once()
    assert dispatch.call_args.kwargs.get("real_only", False) is True
    assert not dispatch.call_args.kwargs.get("check_only", False)


@pytest.mark.parametrize("stdin,stdout", [(False, False), (True, False), (False, True)])
@pytest.mark.parametrize("options", [[], ["--pause"], ["--check-only"]])
def test_no_arg_real_automation_stays_closed_and_never_captures_input(monkeypatch, stdin, stdout, options):
    set_tty(monkeypatch, stdin, stdout)
    monkeypatch.setattr(menu, "run_menu", forbidden)
    assert launcher.main("real", options) == 2


def test_check_only_real_with_missing_arguments_does_not_open_menu_even_on_tty(monkeypatch):
    set_tty(monkeypatch)
    monkeypatch.setattr(menu, "run_menu", forbidden)
    assert launcher.main("real", ["--check-only"]) == 2


@pytest.mark.parametrize("options", [["--udp-ip", "192.0.2.20"], ["--host", "rm"], ["--dry-run"]])
def test_explicit_real_arguments_keep_existing_validation_path_on_tty(monkeypatch, options):
    set_tty(monkeypatch)
    monkeypatch.setattr(menu, "run_menu", forbidden)
    assert launcher.main("real", options) == 2


def test_root_entry_routes_cli_to_menu_without_starting_child(monkeypatch):
    dispatch = Mock(return_value=0)
    monkeypatch.setattr(launcher, "cli", dispatch)
    with pytest.raises(SystemExit) as raised:
        runpy.run_path(str(ROOT / "启动.py"), run_name="__main__")
    assert raised.value.code == 0
    dispatch.assert_called_once_with("menu")
