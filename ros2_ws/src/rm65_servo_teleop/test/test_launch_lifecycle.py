"""Launch exit policy checks without starting ROS nodes, SDKs or subprocesses."""

from __future__ import annotations

import importlib.util
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace

import pytest


@pytest.fixture
def launch_module(monkeypatch):
    # Load the actual launch file on Windows too. These imports only describe
    # ROS actions; none may execute while testing the exit callback policy.
    modules = {
        "ament_index_python.packages": {"get_package_share_directory": None},
        "launch": {"LaunchDescription": None},
        "launch.actions": {
            "DeclareLaunchArgument": None,
            "LogInfo": None,
            "OpaqueFunction": None,
            "RegisterEventHandler": None,
        },
        "launch.event_handlers": {"OnProcessExit": None},
        "launch.substitutions": {"LaunchConfiguration": None},
        "launch_ros.actions": {"Node": None},
    }
    with monkeypatch.context() as imports:
        for name, attributes in modules.items():
            module = ModuleType(name)
            module.__dict__.update(attributes)
            imports.setitem(sys.modules, name, module)
        path = Path(__file__).resolve().parents[1] / "launch/teleop.launch.py"
        spec = importlib.util.spec_from_file_location("rm65_launch_exit_policy", path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
    return module


@pytest.mark.parametrize("returncode", [0, 1, -2, -15])
def test_critical_exit_without_requested_shutdown_fails(launch_module, returncode):
    with pytest.raises(RuntimeError, match=rf"journal.*return code {returncode}"):
        launch_module._required_process_exit(
            SimpleNamespace(returncode=returncode), SimpleNamespace(is_shutdown=False), component="journal"
        )


@pytest.mark.parametrize("returncode", [0, 1, -2, -15])
@pytest.mark.parametrize("next_actions", [None, ["next startup stage"]])
def test_shutdown_does_not_fail_or_start_more_nodes(launch_module, returncode, next_actions):
    assert (
        launch_module._required_process_exit(
            SimpleNamespace(returncode=returncode),
            SimpleNamespace(is_shutdown=True),
            component="controller",
            next_actions=next_actions,
        )
        == []
    )


def test_successful_spawner_advances(launch_module):
    next_actions = [object()]
    assert (
        launch_module._required_process_exit(
            SimpleNamespace(returncode=0),
            SimpleNamespace(is_shutdown=False),
            component="controller spawner",
            next_actions=next_actions,
        )
        is next_actions
    )


def test_failed_spawner_fails_launch(launch_module):
    with pytest.raises(RuntimeError, match="state broadcaster"):
        launch_module._required_process_exit(
            SimpleNamespace(returncode=1),
            SimpleNamespace(is_shutdown=False),
            component="state broadcaster",
            next_actions=[object()],
        )


@pytest.mark.parametrize("requested_shutdown", [False, True])
@pytest.mark.parametrize("returncode", [0, 1])
def test_actual_launch_runtime_return_code_without_nodes(launch_module, requested_shutdown, returncode):
    launch = pytest.importorskip("launch", reason="actual launch return-code check requires the Humble runtime")
    actions = pytest.importorskip("launch.actions", reason="ROS launch actions are not installed")

    service = launch.LaunchService(noninteractive=True)

    def exit_callback(context):
        if requested_shutdown:
            service.shutdown(force_sync=True)
        return launch_module._required_process_exit(
            SimpleNamespace(returncode=returncode), context, component="test control component"
        )

    # Only an in-process Python callback runs; no ROS nodes or child processes.
    service.include_launch_description(launch.LaunchDescription([actions.OpaqueFunction(function=exit_callback)]))
    assert service.run() == (0 if requested_shutdown else 1)
