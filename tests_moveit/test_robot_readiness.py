"""A readiness report must never mistake missing/synthetic/live evidence for a pass."""

import copy
import json
import runpy
import platform
from types import SimpleNamespace

import pytest

from scripts.tools import check_robot_readiness as check


def result(value):
    return {"returncode": 0, "stdout": json.dumps(value)}


@pytest.fixture
def healthy():
    return {
        "read_only": True,
        "image": result({"xr_sdk": "ON", "id": "sha256:test", "source_sha": "a" * 64}),
        "container": {"image_id": "sha256:test"},
        "probes": {
            "dependencies": result({"passed": True}),
            "feedback": result({"fresh_feedback": True, "packet_id_advanced": True,
                                "latest": {"mode": "hardware-dry-run", "hardware_writes": False,
                                           "controller_errors": [], "joint_errors": [0] * 6}}),
            "graph": result({"passed": True}),
            "input": result({"valid_frames": 200, "advancing_timestamps": 199,
                             "invalid_frames": 0, "synthetic_frames": 0}),
        },
    }


def test_offline_and_bad_json_fail_closed():
    assert not check.assess({})["software_readonly_checks_passed"]
    assert check.decode_result({"returncode": 0, "stdout": "not json"}) == {}
    assert check.decode_result({"returncode": 2, "stdout": '{"passed":true}'}) == {}
    assert check.decode_result({"returncode": 0, "stdout": "[]"}) == {}


@pytest.mark.parametrize("field,value", [
    ("mode", "hardware"), ("hardware_writes", True), ("hardware_writes", None),
    ("joint_errors", [0, 0, 1, 0, 0, 0]), ("controller_errors", [1]),
])
def test_live_or_faulted_feedback_cannot_pass(healthy, field, value):
    feedback = json.loads(healthy["probes"]["feedback"]["stdout"])
    feedback["latest"][field] = value
    healthy["probes"]["feedback"] = result(feedback)
    assert not check.assess(healthy)["software_readonly_checks_passed"]


@pytest.mark.parametrize("field,value", [
    ("valid_frames", 0), ("advancing_timestamps", 0), ("invalid_frames", 1), ("synthetic_frames", 1),
])
def test_bad_pico_is_reported_and_required_on_demand(healthy, field, value):
    headset = json.loads(healthy["probes"]["input"]["stdout"])
    headset[field] = value
    healthy["probes"]["input"] = result(headset)
    assert check.assess(healthy)["software_readonly_checks_passed"]
    assert not check.assess(healthy)["checks"]["real_pico_input"]
    assert not check.assess(healthy, require_input=True)["software_readonly_checks_passed"]


def test_no_dependency_or_graph_evidence_cannot_pass(healthy):
    for name in ("dependencies", "graph", "feedback"):
        damaged = copy.deepcopy(healthy)
        damaged["probes"].pop(name)
        assert not check.assess(damaged)["software_readonly_checks_passed"]
    assert check.assess(healthy, require_input=True)["software_readonly_checks_passed"]
    assert check.assess(healthy)["motion_authorized"] is False


@pytest.mark.parametrize("change", ["retag", "source", "missing_label", "sdk_off"])
def test_prepared_tag_does_not_prove_running_image_matches(healthy, change):
    if change == "retag":
        healthy["container"]["image_id"] = "sha256:older"
    image = json.loads(healthy["image"]["stdout"])
    if change == "missing_label":
        image.pop("source_sha")
    if change == "sdk_off":
        image["xr_sdk"] = "OFF"
    healthy["image"] = result(image)
    expected = "b" * 64 if change == "source" else "a" * 64
    assert not check.assess(healthy, expected_source_sha=expected)["software_readonly_checks_passed"]


@pytest.mark.parametrize("live", [True, False])
def test_remote_probe_preserves_live_or_absent_session(monkeypatch, tmp_path, capsys, live):
    helper = tmp_path / "scripts/tools/board_environment.py"
    helper.parent.mkdir(parents=True)
    helper.touch()
    commands = []

    def fake_run(command, **kwargs):
        commands.append(command)
        return SimpleNamespace(returncode=0, stdout="{}", stderr="")

    monkeypatch.setattr(check.subprocess, "run", fake_run)
    monkeypatch.setattr(platform, "system", lambda: "Linux")
    board = {
        "docker_prefix": lambda: ["docker"],
        "ARM_IP": "192.168.10.18", "IMAGE": "rm65-servo:humble",
        "inspect_container": lambda _: {"status": "running"} if live else None,
        "is_our_readonly_session": lambda *_: False,
    }
    monkeypatch.setattr(runpy, "run_path", lambda _: board)
    exec(compile(check.REMOTE_PROBE, "remote-probe", "exec"), {
        "SETTINGS": {"workspace": str(tmp_path), "graph_probe": check.GRAPH_PROBE},
    })
    report = json.loads(capsys.readouterr().out)
    assert "error" in report
    assert report["motion_authorized"] is False
    assert not any("exec" in cmd or "stop" in cmd or "restart" in cmd for cmd in commands)
    if live:
        assert not any("check" in cmd for cmd in commands)
    else:
        assert any(cmd[-1] == "check" for cmd in commands)


def test_embedded_ros_probe_is_valid_python():
    compile(check.GRAPH_PROBE, "readonly-ros-probe", "exec")
