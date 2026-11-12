"""Removed simulation sources must fail before creating processes or loading SDKs."""

from types import SimpleNamespace

import pytest

import start
from scripts.setup import desktop, launcher
from rm65_servo_teleop import official_follow_runner


def forbidden(*args, **kwargs):
    pytest.fail("Removed entry attempted to launch a process or load the SDK")


def test_removed_daily_sim_shortcut_does_not_dispatch(monkeypatch):
    monkeypatch.setattr(start.launcher, "cli", forbidden)
    with pytest.raises(SystemExit) as error:
        start.main(["sim", "--check-only"])
    assert error.value.code == 2


@pytest.mark.parametrize("source", ["manual", "mock", "sim"])
def test_removed_gui_and_shortcut_sources_do_not_build_commands(source):
    with pytest.raises(ValueError):
        desktop.simulation_command(source, distro="RM65-Servo", preview=True)
    with pytest.raises(ValueError):
        launcher.sim_arguments(source, SimpleNamespace())


@pytest.mark.parametrize("removed", [["--demo"], ["--sim", "--web"], ["--sim", "--headless-seconds", "26"]])
def test_official_runner_rejects_removed_demo_before_loading_sdk(monkeypatch, tmp_path, removed):
    monkeypatch.setattr(official_follow_runner, "OfficialAlgo", forbidden)
    output = tmp_path / "must-not-exist.jsonl"
    with pytest.raises(SystemExit) as error:
        official_follow_runner.main(["--input", "recording.jsonl", "--output", str(output), *removed],
                                    default_model_config="model.yaml", default_follow_config="follow.yaml")
    assert error.value.code == 2
    assert not output.exists()
