"""Protect existing board sessions from accidental replacement or live launch."""

import importlib.util
from pathlib import Path, PurePosixPath
from types import SimpleNamespace

import pytest

SPEC = importlib.util.spec_from_file_location(
    "board_environment", Path(__file__).resolve().parents[1] / "scripts/tools/board_environment.py"
)
board = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(board)


@pytest.fixture
def session():
    return {
        "status": "running",
        "image": board.IMAGE,
        "command": [
            "ros2",
            "launch",
            "rm65_servo_teleop",
            "teleop.launch.py",
            "mode:=hardware-dry-run",
            "input_source:=pico",
            "foxglove:=true",
            "rviz:=false",
            "arm_ip:=" + board.ARM_IP,
            "udp_ip:=" + board.UDP_IP,
        ],
        "mounts": [{"Source": "/home/rm/EthanTao/build/experiments", "Destination": "/data/experiments"}],
    }


def test_reuse_does_not_launch_or_restart(monkeypatch, session):
    monkeypatch.setattr(board, "inspect_container", lambda _: session)
    monkeypatch.setattr(board, "execute", lambda *a, **k: pytest.fail("Existing session was disturbed"))
    board.start_readonly(["docker"], PurePosixPath("/home/rm/EthanTao"))


@pytest.mark.parametrize("change", ["live", "override", "stopped", "workspace", "input"])
def test_unrecognized_session_is_preserved_and_refused(monkeypatch, session, change):
    if change == "live":
        session["command"][4] = "mode:=hardware"
    elif change == "override":
        session["command"].append("mode:=hardware")
    elif change == "stopped":
        session["status"] = "exited"
    elif change == "workspace":
        session["mounts"][0]["Source"] = "/home/rm/another-project/build/experiments"
    else:
        session["command"][5] = "input_source:=external"
    monkeypatch.setattr(board, "inspect_container", lambda _: session)
    monkeypatch.setattr(board, "execute", lambda *a, **k: pytest.fail("Conflicting session was disturbed"))
    with pytest.raises(RuntimeError, match="refusing to stop"):
        board.start_readonly(["docker"], PurePosixPath("/home/rm/EthanTao"))


def test_route_change_refuses_before_creating_container(monkeypatch, tmp_path):
    monkeypatch.setattr(board, "inspect_container", lambda _: None)
    calls = []

    def run(command, **kwargs):
        calls.append(command)
        return SimpleNamespace(stdout='[{"prefsrc":"192.168.99.100"}]')

    monkeypatch.setattr(board, "execute", run)
    with pytest.raises(RuntimeError, match="route has changed"):
        board.start_readonly(["docker"], tmp_path)
    assert len(calls) == 1 and calls[0][:2] == ["ip", "-j"]
    assert not (tmp_path / "build").exists()


@pytest.mark.parametrize("port", ["8089", "8765", "63901"])
def test_occupied_port_refuses_competing_receiver(monkeypatch, tmp_path, port):
    monkeypatch.setattr(board, "inspect_container", lambda _: None)
    calls = []

    def run(command, **kwargs):
        calls.append(command)
        if command[0] == "ip":
            return SimpleNamespace(stdout='[{"prefsrc":"192.168.10.100"}]')
        return SimpleNamespace(stdout="udp UNCONN 0 0 0.0.0.0:" + port + " 0.0.0.0:*")

    monkeypatch.setattr(board, "execute", run)
    with pytest.raises(RuntimeError, match="occupied"):
        board.start_readonly(["docker"], tmp_path)
    assert all(command[0] != "docker" for command in calls)


def test_fresh_start_uses_pico_service_and_readonly_driver_only(monkeypatch, tmp_path):
    monkeypatch.setattr(board, "inspect_container", lambda _: None)
    calls = []

    def run(command, **kwargs):
        calls.append(command)
        return SimpleNamespace(stdout='[{"prefsrc":"192.168.10.100"}]' if command[0] == "ip" else "")

    monkeypatch.setattr(board, "execute", run)
    board.start_readonly(["docker"], tmp_path)
    command = calls[-1]
    assert command[:2] == ["docker", "run"]
    assert "input_source:=pico" in command and "RM65_START_XR_SERVICE=1" in command
    assert "63901:63901/tcp" in command and "8089:8089/udp" in command
    assert "mode:=hardware-dry-run" in command and "mode:=hardware" not in command
    assert all("allow_servo_motion" not in part and "hardware_validation" not in part for part in command)
