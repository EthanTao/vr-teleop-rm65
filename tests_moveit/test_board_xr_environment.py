"""An SDK readiness service must never replace a robot or control session."""
import importlib.util
from pathlib import Path
import sys

import pytest

TOOLS = Path(__file__).resolve().parents[1] / "scripts/tools"
SPEC = importlib.util.spec_from_file_location("board_xr_environment", TOOLS / "board_xr_environment.py")
sys.path.insert(0, str(TOOLS))
try:
    xr = importlib.util.module_from_spec(SPEC)
    SPEC.loader.exec_module(xr)
finally:
    sys.path.remove(str(TOOLS))


def metadata():
    return {
        "Config": {"Cmd": xr.COMMAND.copy(), "Labels": {
            "org.rm65.role": xr.ROLE, "org.rm65.workspace": "/home/rm/EthanTao",
            "org.rm65.xr-sdk": "ON",
        }, "Env": ["ROS_DOMAIN_ID=86", "RM65_START_XR_SERVICE=1"]},
        "HostConfig": {"NetworkMode": "container:readonly-id"},
        "State": {"Running": True},
    }


@pytest.mark.parametrize("change", ["controller", "domain", "role", "workspace", "sdk", "network"])
def test_stop_refuses_every_unrecognized_service(monkeypatch, change):
    value = metadata()
    if change == "controller":
        value["Config"]["Cmd"] = ["ros2", "launch", "rm65_servo_teleop", "teleop.launch.py"]
    elif change == "domain":
        value["Config"]["Env"][0] = "ROS_DOMAIN_ID=0"
    elif change == "network":
        value["HostConfig"]["NetworkMode"] = "host"
    else:
        key = {"role": "role", "workspace": "workspace", "sdk": "xr-sdk"}[change]
        value["Config"]["Labels"]["org.rm65." + key] = "other"
    monkeypatch.setattr(xr, "inspect", lambda *args: value)
    monkeypatch.setattr(xr.board, "execute", lambda *a, **k: pytest.fail("Unknown session was changed"))
    with pytest.raises(RuntimeError, match="refusing to stop"):
        xr.stop(["docker"], Path("/home/rm/EthanTao"))


def test_start_refuses_without_preserved_readonly_parent(monkeypatch):
    monkeypatch.setattr(xr.board, "inspect_container", lambda _: {"status": "running"})
    monkeypatch.setattr(xr.board, "is_our_readonly_session", lambda *_: False)
    monkeypatch.setattr(xr, "inspect", lambda *a: pytest.fail("Launch preparation was reached"))
    with pytest.raises(RuntimeError, match="read-only session is required"):
        xr.start(["docker"], Path("/home/rm/EthanTao"))


def test_main_pico_session_prevents_starting_a_second_sdk_service(monkeypatch):
    monkeypatch.setattr(xr.board, "inspect_container", lambda _: {"status": "running", "command": ["input_source:=pico"]})
    monkeypatch.setattr(xr, "inspect", lambda *a: pytest.fail("Duplicate service preparation was reached"))
    with pytest.raises(RuntimeError, match="refusing a duplicate"):
        xr.start(["docker"], Path("/home/rm/EthanTao"))
