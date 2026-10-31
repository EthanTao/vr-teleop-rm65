"""Offline acceptance tests for the owned simulation-session restart protocol."""

import ast
import errno
import json
import os
from pathlib import Path
import re
import subprocess

import pytest

from rm65_servo_teleop.simulation_reset import write_request
from scripts.setup import sim_session


TOKEN = "a" * 32
NEXT_TOKEN = "b" * 32
STARTED = 100.0
NOW = 120.0


def request_value(**changes):
    value = {
        "version": 1,
        "mode": "sim",
        "action": "reset",
        "token": TOKEN,
        "requested_at": NOW,
    }
    value.update(changes)
    return value


def put_request(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value), encoding="utf-8")
    return path


@pytest.fixture
def request_path(tmp_path, monkeypatch):
    monkeypatch.setattr(sim_session.time, "time", lambda: NOW)
    return tmp_path / f"{TOKEN}.json"


def test_write_consume_roundtrip_is_single_use(tmp_path, monkeypatch):
    monkeypatch.setattr(sim_session.time, "time", lambda: NOW)
    path = write_request(tmp_path / "build/experiments", TOKEN, now=NOW)

    assert path == tmp_path / "build/experiments/.sim-reset" / f"{TOKEN}.json"
    assert json.loads(path.read_text(encoding="utf-8")) == request_value()
    assert sim_session.consume_request(path, TOKEN, STARTED) is True
    assert not path.exists()
    assert sim_session.consume_request(path, TOKEN, STARTED) is False


def test_missing_request_is_not_a_reset(request_path):
    assert sim_session.consume_request(request_path, TOKEN, STARTED) is False


@pytest.mark.parametrize(
    "value",
    [
        None,
        [],
        [request_value()],
        "reset",
        1,
        {},
        {key: value for key, value in request_value().items() if key != "action"},
        request_value(extra="untrusted"),
    ],
)
def test_consume_rejects_nonexact_shape_without_deleting(request_path, value):
    put_request(request_path, value)
    before = request_path.read_bytes()

    with pytest.raises(ValueError):
        sim_session.consume_request(request_path, TOKEN, STARTED)

    assert request_path.read_bytes() == before


@pytest.mark.parametrize(
    "changes",
    [
        {"version": True},
        {"version": False},
        {"version": 1.0},
        {"version": "1"},
        {"version": 0},
        {"version": 2},
        {"version": None},
        {"mode": "hardware"},
        {"mode": "real"},
        {"mode": "SIM"},
        {"mode": True},
        {"action": "home"},
        {"action": "reset_world"},
        {"action": True},
        {"token": NEXT_TOKEN},
        {"token": "A" * 32},
        {"token": None},
        {"token": 1},
        {"token": []},
        {"requested_at": True},
        {"requested_at": False},
        {"requested_at": None},
        {"requested_at": "120"},
        {"requested_at": []},
        {"requested_at": float("nan")},
        {"requested_at": float("inf")},
        {"requested_at": -float("inf")},
        {"requested_at": STARTED - 0.000001},
        {"requested_at": NOW + 0.100001},
        {"requested_at": -1},
        {"requested_at": 10**30},
    ],
)
def test_consume_rejects_unowned_or_invalid_fields(request_path, changes):
    put_request(request_path, request_value(**changes))

    with pytest.raises(ValueError):
        sim_session.consume_request(request_path, TOKEN, STARTED)

    assert request_path.exists()


@pytest.mark.parametrize("token", ["", "a" * 31, "a" * 33, "g" * 32, "A" * 32, "../outside", True, None])
def test_consume_rejects_malformed_expected_token_even_when_payload_matches(request_path, token):
    put_request(request_path, request_value(token=token))

    error_type = ValueError if isinstance(token, str) else (ValueError, TypeError)
    with pytest.raises(error_type):
        sim_session.consume_request(request_path, token, STARTED)

    assert request_path.exists()


@pytest.mark.parametrize("stamp", [STARTED, 110, NOW, NOW + 0.1])
def test_consume_accepts_inclusive_freshness_bounds(request_path, stamp):
    put_request(request_path, request_value(requested_at=stamp))

    assert sim_session.consume_request(request_path, TOKEN, STARTED) is True
    assert not request_path.exists()


@pytest.mark.parametrize("text", ["not-json", "{", "", "{\"version\": 1,}", "x" * 2049])
def test_consume_rejects_malformed_or_oversized_file(request_path, text):
    request_path.write_text(text, encoding="utf-8")

    with pytest.raises(ValueError):
        sim_session.consume_request(request_path, TOKEN, STARTED)

    assert request_path.read_text(encoding="utf-8") == text


def test_consume_rejects_directory(request_path):
    request_path.mkdir()

    with pytest.raises(ValueError):
        sim_session.consume_request(request_path, TOKEN, STARTED)

    assert request_path.is_dir()


def test_consume_rejects_fifo_without_opening_it(request_path):
    if not hasattr(os, "mkfifo"):
        pytest.skip("FIFO creation is not available on this platform")
    os.mkfifo(request_path)

    with pytest.raises(ValueError):
        sim_session.consume_request(request_path, TOKEN, STARTED)

    assert request_path.exists()


@pytest.mark.parametrize("dangling", [False, True])
def test_consume_rejects_symlink_without_touching_target(request_path, tmp_path, dangling):
    target = tmp_path / "unowned.json"
    if not dangling:
        put_request(target, request_value())
    try:
        request_path.symlink_to(target)
    except OSError as exc:
        if exc.errno in {errno.EPERM, errno.EACCES, errno.ENOTSUP} or getattr(exc, "winerror", None) == 1314:
            pytest.skip("Creating symlinks requires privileges unavailable to this test")
        raise

    with pytest.raises(ValueError):
        sim_session.consume_request(request_path, TOKEN, STARTED)

    assert request_path.is_symlink()
    assert target.exists() is (not dangling)
    if not dangling:
        assert json.loads(target.read_text(encoding="utf-8")) == request_value()


def test_failed_child_without_request_is_not_retried(tmp_path):
    command = ["owned-launcher", "--mode", "sim"]
    calls = []
    announcements = []
    error = subprocess.CalledProcessError(9, command)

    def execute(args):
        calls.append(args)
        raise error

    with pytest.raises(subprocess.CalledProcessError) as raised:
        sim_session.run_simulation(command, root=tmp_path, execute=execute, announce=announcements.append)

    assert raised.value is error
    assert len(calls) == 1
    assert calls[0][:-2] == command
    assert announcements == []
    assert list((tmp_path / "build/experiments/.sim-reset").iterdir()) == []


def test_successful_ordinary_exit_is_not_retried(tmp_path):
    calls = []
    announcements = []

    sim_session.run_simulation(
        ["owned-launcher", "--mode", "sim"],
        root=tmp_path,
        execute=lambda args: calls.append(args),
        announce=announcements.append,
    )

    assert len(calls) == 1
    assert announcements == []


@pytest.mark.parametrize("failed_exit", [False, True])
def test_explicit_request_restarts_once_only_after_exit_with_new_nonce_and_args(tmp_path, monkeypatch, failed_exit):
    command = ["owned-launcher", "--mode", "sim", "--input-source", "manual"]
    original = command.copy()
    tokens = iter([TOKEN, NEXT_TOKEN])
    monkeypatch.setattr(sim_session.secrets, "token_hex", lambda size: next(tokens) if size == 16 else pytest.fail(size))
    monkeypatch.setattr(sim_session.time, "time", lambda: NOW)
    root = tmp_path / "build/experiments"
    unowned = put_request(root / ".sim-reset" / f"{'c' * 32}.json", request_value(token="c" * 32))
    events = []
    calls = []
    announcements = []
    consume = sim_session.consume_request
    child_running = False

    def checked_consume(path, token, started):
        assert not child_running, "The reset file must never be consumed while the child graph is alive"
        events.append(("consume", token))
        return consume(path, token, started)

    def execute(args):
        nonlocal child_running
        assert not child_running
        child_running = True
        calls.append(args)
        token = args[-1]
        events.append(("start", token))
        assert args[:-2] == original
        assert args[-2] == "--sim-reset-token"
        try:
            if len(calls) == 1:
                write_request(root, token, now=NOW)
                assert (root / ".sim-reset" / f"{token}.json").exists()
            else:
                assert len(calls) == 2
                assert token == NEXT_TOKEN
                assert not (root / ".sim-reset" / f"{TOKEN}.json").exists()
        finally:
            child_running = False
            events.append(("exit", token))
        if len(calls) == 1 and failed_exit:
            raise subprocess.CalledProcessError(1, args)

    monkeypatch.setattr(sim_session, "consume_request", checked_consume)
    sim_session.run_simulation(command, root=tmp_path, execute=execute, announce=announcements.append)

    assert command == original
    assert len(calls) == 2
    assert calls[0] is not calls[1]
    assert calls[0][-1] == TOKEN
    assert calls[1][-1] == NEXT_TOKEN
    assert len(announcements) == 1
    assert events == [
        ("start", TOKEN),
        ("exit", TOKEN),
        ("consume", TOKEN),
        ("start", NEXT_TOKEN),
        ("exit", NEXT_TOKEN),
        ("consume", NEXT_TOKEN),
    ]
    assert unowned.exists(), "Another session's file must not authorize or be consumed by this session"


@pytest.mark.parametrize("exception_type", [KeyboardInterrupt, OSError])
@pytest.mark.parametrize("request_exists", [False, True])
def test_interrupt_or_oserror_never_consumes_or_retries(tmp_path, monkeypatch, exception_type, request_exists):
    monkeypatch.setattr(sim_session.secrets, "token_hex", lambda size: TOKEN)
    monkeypatch.setattr(sim_session.time, "time", lambda: NOW)
    calls = []
    announcements = []
    error = exception_type("child interrupted")
    path = tmp_path / "build/experiments/.sim-reset" / f"{TOKEN}.json"

    def execute(args):
        calls.append(args)
        if request_exists:
            write_request(tmp_path / "build/experiments", TOKEN, now=NOW)
        raise error

    monkeypatch.setattr(sim_session, "consume_request", lambda *args: pytest.fail("aborted child consumed a request"))
    with pytest.raises(exception_type) as raised:
        sim_session.run_simulation(["owned-launcher"], root=tmp_path, execute=execute, announce=announcements.append)

    assert raised.value is error
    assert len(calls) == 1
    assert announcements == []
    assert path.exists() is request_exists


@pytest.mark.parametrize("failed_exit", [False, True])
def test_invalid_request_after_child_exit_fails_closed_without_retry(tmp_path, monkeypatch, failed_exit):
    monkeypatch.setattr(sim_session.secrets, "token_hex", lambda size: TOKEN)
    monkeypatch.setattr(sim_session.time, "time", lambda: NOW)
    calls = []
    announcements = []
    path = tmp_path / "build/experiments/.sim-reset" / f"{TOKEN}.json"

    def execute(args):
        calls.append(args)
        put_request(path, request_value(token=NEXT_TOKEN))
        if failed_exit:
            raise subprocess.CalledProcessError(1, args)

    with pytest.raises(ValueError):
        sim_session.run_simulation(["owned-launcher"], root=tmp_path, execute=execute, announce=announcements.append)

    assert len(calls) == 1
    assert announcements == []
    assert path.exists()


def test_host_reset_loop_has_no_motion_hardware_world_or_process_cleanup_authority():
    source = Path(sim_session.__file__).read_text(encoding="utf-8")
    tree = ast.parse(source)
    calls = [node for node in ast.walk(tree) if isinstance(node, ast.Call)]
    attributes = {node.func.attr for node in calls if isinstance(node.func, ast.Attribute)}
    assert not attributes & {
        "create_publisher",
        "publish",
        "reset_world",
        "send_joint_command",
        "movej",
        "movel",
        "system",
        "Popen",
        "run",
        "check_call",
        "check_output",
        "stop",
        "restart",
        "shutdown",
    }
    strings = [node.value for node in ast.walk(tree) if isinstance(node, ast.Constant) and isinstance(node.value, str)]
    executable_strings = "\n".join(strings)
    assert not re.search(r"\bdocker(?:\.exe)?\s+(?:stop|restart|rm)\b", executable_strings, re.IGNORECASE)
    assert not re.search(r"\bwsl(?:\.exe)?\b[^\n]*(?:--shutdown|--terminate)", executable_strings, re.IGNORECASE)
    assert not re.search(r"(?:^|\s)--mode(?:\s+|=)(?:real|hardware|live)\b", executable_strings)
