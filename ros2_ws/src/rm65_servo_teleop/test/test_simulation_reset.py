"""Pure protocol and mocked-ROS checks; no ROS, Docker, network or hardware."""

import ast
import errno
import json
from pathlib import Path
import re
import sys
from types import ModuleType, SimpleNamespace

import pytest

from rm65_servo_teleop import simulation_reset

TOKEN = "a" * 32
NOW = 120.0


def test_write_request_has_exact_sim_only_shape_and_owned_path(tmp_path, monkeypatch):
    monkeypatch.setattr(simulation_reset.time, "time", lambda: NOW)

    path = simulation_reset.write_request(tmp_path, TOKEN)

    assert path == tmp_path / ".sim-reset" / f"{TOKEN}.json"
    assert path.is_file()
    assert not path.is_symlink()
    assert json.loads(path.read_text(encoding="utf-8")) == {
        "version": 1,
        "mode": "sim",
        "action": "reset",
        "token": TOKEN,
        "requested_at": NOW,
    }


def test_write_request_is_exclusive_and_never_overwrites_a_previous_request(tmp_path):
    path = simulation_reset.write_request(tmp_path, TOKEN, now=NOW)
    original = path.read_bytes()

    with pytest.raises(FileExistsError):
        simulation_reset.write_request(tmp_path, TOKEN, now=NOW + 1)

    assert path.read_bytes() == original
    assert list(path.parent.iterdir()) == [path]


@pytest.mark.parametrize(
    "token",
    ["", "a" * 31, "a" * 33, "A" * 32, "g" * 32, "a" * 31 + "\n", "../outside", "/outside", True, None, 123, []],
)
def test_write_request_rejects_invalid_token_before_creating_a_file(tmp_path, token):
    error_type = ValueError if isinstance(token, str) else (ValueError, TypeError)
    with pytest.raises(error_type):
        simulation_reset.write_request(tmp_path, token, now=NOW)

    assert list(tmp_path.rglob("*.json")) == []


@pytest.mark.parametrize("stamp", [True, False, float("nan"), float("inf"), -float("inf"), "120", [], {}])
def test_write_request_rejects_boolean_nonfinite_or_nonnumeric_timestamp(tmp_path, stamp):
    with pytest.raises(ValueError):
        simulation_reset.write_request(tmp_path, TOKEN, now=stamp)

    assert list(tmp_path.rglob("*.json")) == []


def test_write_request_fsyncs_completed_json_before_returning(tmp_path, monkeypatch):
    seen = []

    def fsync(file_descriptor):
        assert isinstance(file_descriptor, int)
        path = tmp_path / ".sim-reset" / f"{TOKEN}.json"
        seen.append(json.loads(path.read_text(encoding="utf-8")))

    monkeypatch.setattr(simulation_reset.os, "fsync", fsync)

    path = simulation_reset.write_request(tmp_path, TOKEN, now=NOW)

    assert len(seen) == 1
    assert seen[0]["requested_at"] == NOW
    assert seen[0]["token"] == TOKEN
    assert path.exists()


@pytest.mark.parametrize("existing_type", ["file", "directory"])
def test_write_request_does_not_replace_an_existing_file_or_directory(tmp_path, existing_type):
    folder = tmp_path / ".sim-reset"
    folder.mkdir()
    path = folder / f"{TOKEN}.json"
    if existing_type == "directory":
        path.mkdir()
    else:
        path.write_text("owned by someone else", encoding="utf-8")

    with pytest.raises((OSError, ValueError)):
        simulation_reset.write_request(tmp_path, TOKEN, now=NOW)

    if existing_type == "directory":
        assert path.is_dir()
    else:
        assert path.read_text(encoding="utf-8") == "owned by someone else"


@pytest.mark.parametrize("dangling", [False, True])
def test_write_request_rejects_symlink_without_following_or_overwriting(tmp_path, dangling):
    folder = tmp_path / ".sim-reset"
    folder.mkdir()
    path = folder / f"{TOKEN}.json"
    target = tmp_path / "other-session.json"
    if not dangling:
        target.write_text("do not overwrite", encoding="utf-8")
    try:
        path.symlink_to(target)
    except OSError as exc:
        if exc.errno in {errno.EPERM, errno.EACCES, errno.ENOTSUP} or getattr(exc, "winerror", None) == 1314:
            pytest.skip("Creating symlinks requires privileges unavailable to this test")
        raise

    with pytest.raises((OSError, ValueError)):
        simulation_reset.write_request(tmp_path, TOKEN, now=NOW)

    assert path.is_symlink()
    assert target.exists() is (not dangling)
    if not dangling:
        assert target.read_text(encoding="utf-8") == "do not overwrite"


@pytest.fixture
def fake_ros(tmp_path, monkeypatch):
    """Inject all ROS imports so main() remains a completely offline unit test."""
    state = SimpleNamespace(
        parameters={"simulation_only": True, "record_root": str(tmp_path)},
        declared=[],
        subscriptions=[],
        timers=[],
        messages=[],
        events=[],
        ok=True,
        node=None,
        spin=lambda node: None,
    )

    class Empty:
        pass

    class ExternalShutdownException(Exception):
        pass

    class Node:
        def __init__(self, name):
            state.node = self
            state.events.append(("node", name))

        def declare_parameter(self, name, default):
            state.declared.append((name, default))
            return SimpleNamespace(value=state.parameters.get(name, default))

        def create_subscription(self, message_type, topic, callback, depth):
            state.subscriptions.append((message_type, topic, callback, depth))
            return object()

        def create_timer(self, period, callback):
            state.timers.append((period, callback))
            return object()

        def get_logger(self):
            return SimpleNamespace(info=state.messages.append)

        def destroy_node(self):
            state.events.append(("destroy",))

    def install(name, **members):
        module = ModuleType(name)
        module.__dict__.update(members)
        monkeypatch.setitem(sys.modules, name, module)
        return module

    def shutdown():
        state.ok = False
        state.events.append(("shutdown",))

    def spin(node):
        state.events.append(("spin",))
        state.spin(node)

    rclpy = install(
        "rclpy",
        init=lambda **kwargs: state.events.append(("init", kwargs)),
        spin=spin,
        ok=lambda: state.ok,
        shutdown=shutdown,
    )
    rclpy.node = install("rclpy.node", Node=Node)
    rclpy.executors = install("rclpy.executors", ExternalShutdownException=ExternalShutdownException)
    rclpy.qos = install(
        "rclpy.qos",
        DurabilityPolicy=SimpleNamespace(TRANSIENT_LOCAL="transient_local"),
        QoSProfile=lambda **kwargs: SimpleNamespace(**kwargs),
    )
    std_msgs = install("std_msgs")
    std_msgs.msg = install("std_msgs.msg", Empty=Empty)
    state.Empty = Empty
    state.ExternalShutdownException = ExternalShutdownException
    monkeypatch.setenv("RM65_SIM_RESET_TOKEN", TOKEN)
    return state


def assert_ros_cleanup(state):
    assert state.events[-2:] == [("destroy",), ("shutdown",)]
    assert not state.ok


def test_reset_node_idle_ticks_never_write_a_request(tmp_path, fake_ros):
    def spin(node):
        assert node is fake_ros.node
        for _ in range(5):
            fake_ros.timers[0][1]()
        assert not (tmp_path / ".sim-reset").exists()

    fake_ros.spin = spin

    simulation_reset.main(args=["--ros-args"])

    assert fake_ros.events[0] == ("init", {"args": ["--ros-args"]})
    assert fake_ros.events[1] == ("node", "rm65_simulation_reset")
    assert len(fake_ros.subscriptions) == 1
    message_type, topic, _, depth = fake_ros.subscriptions[0]
    assert message_type is fake_ros.Empty
    assert topic == "/rm65/simulation/reset_request"
    assert depth.depth == 1 and depth.durability == "transient_local"
    assert len(fake_ros.timers) == 1
    assert fake_ros.timers[0][0] == pytest.approx(0.02)
    assert ("simulation_only", False) in fake_ros.declared
    assert ("record_root", "/data/experiments") in fake_ros.declared
    assert_ros_cleanup(fake_ros)


def test_explicit_empty_request_writes_once_then_propagates_critical_node_exit(tmp_path, fake_ros, monkeypatch):
    calls = []
    real_write = simulation_reset.write_request
    monkeypatch.setattr(simulation_reset.time, "time", lambda: NOW)

    def write(root, token):
        calls.append((root, token))
        return real_write(root, token)

    monkeypatch.setattr(simulation_reset, "write_request", write)

    def spin(node):
        tick = fake_ros.timers[0][1]
        request = fake_ros.subscriptions[0][2]
        tick()
        assert calls == []
        request(fake_ros.Empty())
        request(fake_ros.Empty())
        assert calls == [], "The subscription must only latch the request until the timer can exit safely"
        tick()
        pytest.fail("Critical reset callback must end the old graph rather than continue spinning")

    fake_ros.spin = spin

    with pytest.raises(RuntimeError, match="Explicit simulation reset"):
        simulation_reset.main()

    assert calls == [(tmp_path, TOKEN)]
    path = tmp_path / ".sim-reset" / f"{TOKEN}.json"
    assert json.loads(path.read_text(encoding="utf-8"))["requested_at"] == NOW
    assert_ros_cleanup(fake_ros)


@pytest.mark.parametrize("simulation_only", [False, 0, 1, "true", "True", None])
def test_reset_node_requires_explicit_boolean_simulation_gate(tmp_path, fake_ros, monkeypatch, simulation_only):
    fake_ros.parameters["simulation_only"] = simulation_only
    monkeypatch.setattr(simulation_reset, "write_request", lambda *args: pytest.fail("non-sim request was written"))

    with pytest.raises(ValueError, match="simulation"):
        simulation_reset.main()

    assert fake_ros.subscriptions == []
    assert fake_ros.timers == []
    assert ("spin",) not in fake_ros.events
    assert list(tmp_path.iterdir()) == []
    assert_ros_cleanup(fake_ros)


def test_reset_node_simulation_gate_defaults_closed(tmp_path, fake_ros):
    fake_ros.parameters.pop("simulation_only")

    with pytest.raises(ValueError, match="simulation"):
        simulation_reset.main()

    assert fake_ros.subscriptions == []
    assert list(tmp_path.iterdir()) == []
    assert_ros_cleanup(fake_ros)


@pytest.mark.parametrize("token", [None, "", "a" * 31, "a" * 33, "A" * 32, "g" * 32, "../unowned"])
def test_reset_node_refuses_missing_or_unowned_token_before_subscribing(tmp_path, fake_ros, monkeypatch, token):
    if token is None:
        monkeypatch.delenv("RM65_SIM_RESET_TOKEN", raising=False)
    else:
        monkeypatch.setenv("RM65_SIM_RESET_TOKEN", token)

    with pytest.raises(ValueError, match="simulation entry"):
        simulation_reset.main()

    assert fake_ros.subscriptions == []
    assert fake_ros.timers == []
    assert ("spin",) not in fake_ros.events
    assert list(tmp_path.iterdir()) == []
    assert_ros_cleanup(fake_ros)


def test_request_write_failure_propagates_without_retry(fake_ros, monkeypatch):
    error = OSError("read-only experiment volume")
    calls = []

    def write(root, token):
        calls.append((root, token))
        raise error

    def spin(node):
        fake_ros.subscriptions[0][2](fake_ros.Empty())
        fake_ros.timers[0][1]()

    monkeypatch.setattr(simulation_reset, "write_request", write)
    fake_ros.spin = spin

    with pytest.raises(OSError) as raised:
        simulation_reset.main()

    assert raised.value is error
    assert len(calls) == 1
    assert_ros_cleanup(fake_ros)


@pytest.mark.parametrize("exception_name", ["KeyboardInterrupt", "ExternalShutdownException"])
def test_ros_shutdown_never_creates_a_reset_request(tmp_path, fake_ros, exception_name):
    error_type = KeyboardInterrupt if exception_name == "KeyboardInterrupt" else fake_ros.ExternalShutdownException

    def spin(node):
        raise error_type()

    fake_ros.spin = spin

    simulation_reset.main()

    assert list(tmp_path.iterdir()) == []
    assert_ros_cleanup(fake_ros)


def test_reset_node_source_has_no_motion_world_reset_hardware_or_host_shutdown_authority():
    source = Path(simulation_reset.__file__).read_text(encoding="utf-8")
    tree = ast.parse(source)
    imports = {name.name for node in ast.walk(tree) if isinstance(node, ast.Import) for name in node.names}
    imports.update(node.module for node in ast.walk(tree) if isinstance(node, ast.ImportFrom))
    assert not imports & {"subprocess", "socket", "paramiko", "docker", "ctypes"}
    calls = [node for node in ast.walk(tree) if isinstance(node, ast.Call)]
    attributes = {node.func.attr for node in calls if isinstance(node.func, ast.Attribute)}
    assert not attributes & {
        "create_publisher",
        "publish",
        "create_client",
        "reset_world",
        "send_joint_command",
        "movej",
        "movel",
        "system",
        "Popen",
        "run",
        "stop",
        "restart",
    }
    shutdown_calls = [node for node in calls if isinstance(node.func, ast.Attribute) and node.func.attr == "shutdown"]
    assert all(isinstance(node.func.value, ast.Name) and node.func.value.id == "rclpy" for node in shutdown_calls)
    strings = [node.value for node in ast.walk(tree) if isinstance(node, ast.Constant) and isinstance(node.value, str)]
    executable_strings = "\n".join(strings)
    assert not re.search(r"\bdocker(?:\.exe)?\s+(?:stop|restart|rm)\b", executable_strings, re.IGNORECASE)
    assert not re.search(r"\bwsl(?:\.exe)?\b[^\n]*(?:--shutdown|--terminate)", executable_strings, re.IGNORECASE)
    assert not re.search(r"(?:^|\s)--mode(?:\s+|=)(?:real|hardware|live)\b", executable_strings)
