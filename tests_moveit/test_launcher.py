"""Launcher checks exercise no SSH, Docker, SDK or robot operation."""

from __future__ import annotations

import io
import json
from pathlib import Path
from types import SimpleNamespace
import tarfile

import pytest

import servo_launcher


@pytest.fixture
def wsl_root(monkeypatch):
    # Apostrophes, spaces, Chinese characters and shell metacharacters must stay
    # inside one argv item; no shell is used for either wslpath or Docker.
    root = Path(r"D:\RM65 仿真\O'Brien; $(not-executed)")
    monkeypatch.setattr(servo_launcher, "ROOT", root)
    monkeypatch.setattr(servo_launcher, "source_digest", lambda: "fixture-source-hash")
    monkeypatch.setattr(servo_launcher, "source_archive", lambda: pytest.fail("WSL must not sync source over SSH"))
    return root


def test_print_command_is_offline_and_uses_one_stack(monkeypatch, capsys):
    def forbidden(*args, **kwargs):
        raise AssertionError("print-command must never execute an external process")

    monkeypatch.setattr(servo_launcher.subprocess, "run", forbidden)
    assert servo_launcher.main(["--mode", "sim", "--target", "remote", "--print-command"]) == 0
    output = capsys.readouterr().out
    assert "RM65_WITH_XR_SDK=OFF" in output
    assert "rm65_servo_teleop" in output
    assert "input_source:=mock" in output
    assert "simulator:=mujoco" in output
    assert "placo" not in output and "damped_ik" not in output


def test_live_refuses_before_network_or_process(monkeypatch):
    monkeypatch.setattr(servo_launcher.subprocess, "run", lambda *a, **k: pytest.fail("attempted external command"))
    assert servo_launcher.main(["--mode", "hardware", "--udp-ip", "192.168.0.115"]) == 2


@pytest.mark.parametrize("mode", ["hardware", "hardware-dry-run", "sync-only", "test"])
def test_mock_hand_is_rejected_outside_sim_before_any_process(mode, monkeypatch):
    monkeypatch.setattr(servo_launcher.subprocess, "run", lambda *a, **k: pytest.fail("unexpected process"))
    assert servo_launcher.main(["--mode", mode, "--hand", "mock", "--udp-ip", "192.168.0.115"]) == 2


def test_mock_hand_uses_same_sim_stack_and_offline_command_preview(capsys):
    assert servo_launcher.main(["--mode", "sim", "--target", "local", "--hand", "mock", "--print-command"]) == 0
    output = capsys.readouterr().out
    assert "hand_mode:=mock" in output and "teleop.launch.py" in output


def test_simulator_choice_cannot_modify_hardware_mode(monkeypatch):
    monkeypatch.setattr(servo_launcher.subprocess, "run", lambda *a, **k: pytest.fail("attempted external command"))
    assert (
        servo_launcher.main(["--mode", "hardware-dry-run", "--simulator", "mujoco", "--udp-ip", "192.168.0.115"]) == 2
    )


def test_explicit_mock_uses_same_servo_launch_without_mujoco_window(capsys, wsl_root):
    assert (
        servo_launcher.main(
            ["--mode", "sim", "--target", "wsl", "--simulator", "mock", "--no-viewer", "--print-command"]
        )
        == 0
    )
    output = capsys.readouterr().out
    assert "simulator:=mock" in output and "mujoco_viewer:=false" in output
    assert "rm65_servo_teleop" in output


@pytest.mark.parametrize("input_source", ["manual", "mock", "pico"])
def test_rviz_only_opens_when_explicitly_requested(input_source, capsys, wsl_root):
    assert (
        servo_launcher.main(["--mode", "sim", "--target", "wsl", "--input", input_source, "--rviz", "--print-command"])
        == 0
    )
    output = capsys.readouterr().out
    assert "rviz:=true" in output and "mujoco_viewer:=true" in output
    assert f"input_source:={input_source}" in output
    assert "simulator:=mujoco" in output


@pytest.mark.parametrize(
    "options",
    [
        ["--target", "wsl", "--mode", "sim", "--no-viewer"],
        ["--target", "wsl", "--mode", "test"],
        ["--target", "local", "--mode", "sim"],
    ],
)
def test_rviz_rejects_incompatible_modes_before_starting(monkeypatch, options):
    monkeypatch.setattr(servo_launcher.subprocess, "run", lambda *a, **k: pytest.fail("unexpected process"))
    assert servo_launcher.main([*options, "--rviz", "--print-command"]) == 2


@pytest.mark.parametrize("mode", ["hardware", "hardware-dry-run"])
def test_synthetic_input_cannot_reach_hardware(mode):
    assert servo_launcher.main(["--mode", mode, "--input", "mock", "--udp-ip", "192.168.0.115", "--print-command"]) == 2


@pytest.mark.parametrize("mode", ["hardware", "hardware-dry-run", "test", "sync-only"])
def test_manual_input_is_simulation_only_and_fails_before_any_process(monkeypatch, mode):
    monkeypatch.setattr(servo_launcher.subprocess, "run", lambda *a, **k: pytest.fail("attempted external command"))
    assert servo_launcher.main(["--mode", mode, "--input", "manual", "--udp-ip", "192.168.0.115"]) == 2


@pytest.mark.parametrize("input_source,sdk", [("manual", "OFF"), ("pico", "ON"), ("mock", "OFF")])
def test_simulation_input_changes_only_the_source_and_sdk(monkeypatch, capsys, input_source, sdk, wsl_root):
    monkeypatch.setattr(servo_launcher.subprocess, "run", lambda *a, **k: pytest.fail("preview ran process"))
    assert servo_launcher.main(["--mode", "sim", "--target", "wsl", "--input", input_source, "--print-command"]) == 0
    output = capsys.readouterr().out
    assert f"input_source:={input_source}" in output
    assert f"RM65_WITH_XR_SDK={sdk}" in output
    assert "teleop.launch.py" in output and "simulator:=mujoco" in output


def test_manual_remote_panel_tunnel_belongs_only_to_its_running_session(monkeypatch, capsys):
    calls = []
    monkeypatch.setattr(servo_launcher.subprocess, "run", lambda command, **kwargs: calls.append(command))
    monkeypatch.setattr(servo_launcher, "source_archive", lambda: b"source-only")
    assert servo_launcher.main(["--mode", "sim", "--target", "remote", "--input", "manual"]) == 0
    assert "8766" in capsys.readouterr().out
    assert len(calls) == 4
    assert all("-L" not in command for command in calls[:3])
    assert "127.0.0.1:8766:127.0.0.1:8766" in calls[3]
    assert "input_source:=manual" in calls[3][-1]


def test_template_is_not_a_live_approval():
    assert (
        servo_launcher.main(
            [
                "--mode",
                "hardware",
                "--udp-ip",
                "192.168.0.115",
                "--live-ack",
                "RM65 LIVE VALIDATION",
                "--hardware-validation",
                "ros2_ws/src/rm65_servo_teleop/config/hardware_validation.example.yaml",
                "--print-command",
            ]
        )
        == 2
    )


def test_test_runner_is_network_isolated_and_does_not_launch_a_driver(capsys):
    assert servo_launcher.main(["--mode", "test", "--print-command"]) == 0
    output = capsys.readouterr().out
    assert "--network none" in output
    assert "colcon test" in output
    assert "mode:=hardware" not in output


def test_sync_archive_excludes_old_control_and_generated_vendor_tree():
    payload = servo_launcher.source_archive()
    with tarfile.open(fileobj=io.BytesIO(payload), mode="r:gz") as archive:
        names = archive.getnames()
    assert "teleop.py" in names
    assert "docker/Dockerfile.moveit" in names
    assert any(name.endswith("pose_tracking_node.cpp") for name in names)
    assert all(not name.startswith("xrobotoolkit_teleop/") for name in names)
    assert "docker/Dockerfile" not in names
    assert not any("/vendor/" in name or "/__pycache__/" in name for name in names)


def test_source_order_uses_identical_case_sensitive_names_on_windows_and_linux(tmp_path):
    package = tmp_path / "ros2_ws"
    (package / "config").mkdir(parents=True)
    for name in ("CMakeLists.txt", "alpha.py", "config/teleop.yaml"):
        (package / name).write_text(name, encoding="utf-8")
    names = [path.relative_to(tmp_path).as_posix() for path in servo_launcher.source_files(tmp_path)]
    assert names == ["ros2_ws/CMakeLists.txt", "ros2_ws/alpha.py", "ros2_ws/config/teleop.yaml"]


def test_remote_launch_tunnel_only_belongs_to_running_session(monkeypatch):
    calls = []
    monkeypatch.setattr(servo_launcher.subprocess, "run", lambda command, **kwargs: calls.append((command, kwargs)))
    monkeypatch.setattr(servo_launcher, "source_archive", lambda: b"source-only")
    assert servo_launcher.main(["--mode", "sim", "--target", "remote"]) == 0
    assert len(calls) == 4
    assert all("-L" not in call[0] for call in calls[:3])
    assert "127.0.0.1:8765:127.0.0.1:8765" in calls[3][0]


@pytest.mark.parametrize(
    "source,expected",
    [
        (r"D:\HDU-EILab\vr-teleop-rm65", "/mnt/d/HDU-EILab/vr-teleop-rm65"),
        (r"C:\项目 Space\O'Brien; $(touch bad)", "/mnt/c/项目 Space/O'Brien; $(touch bad)"),
        ("e:/Mixed Paths/repo", "/mnt/e/Mixed Paths/repo"),
    ],
)
def test_wsl_offline_path_conversion_preserves_literal_components(source, expected):
    assert servo_launcher.windows_path_to_wsl(source) == expected


@pytest.mark.parametrize("source", [r"\\server\share\repo", r"C:relative\repo", "relative/repo", "/linux/repo"])
def test_wsl_path_conversion_rejects_roots_without_windows_drive(source):
    with pytest.raises(ValueError, match="Windows"):
        servo_launcher.windows_path_to_wsl(source)


def test_wsl_print_command_is_offline_and_contains_root_docker_without_sudo(monkeypatch, capsys, wsl_root):
    monkeypatch.setattr(servo_launcher.subprocess, "run", lambda *a, **k: pytest.fail("preview started a process"))
    monkeypatch.setattr(
        servo_launcher, "wsl_distro_registered", lambda name: pytest.fail("explicit WSL needs no probe")
    )
    assert servo_launcher.main(["--target", "wsl", "--mode", "sim", "--sudo-docker", "--print-command"]) == 0
    output = capsys.readouterr().out
    assert "wsl.exe --distribution RM65-Servo --user root --cd" in output
    assert "--exec docker compose" in output and "input_source:=mock" in output
    assert "sudo" not in output and "ssh" not in output
    assert "wslpath" in output and "不启动 WSL" in output


@pytest.mark.parametrize("mode", ["sim", "test"])
def test_wsl_executes_same_docker_commands_with_literal_argv(monkeypatch, wsl_root, mode):
    calls = []
    converted = "/custom-mount/d/RM65 仿真/O'Brien; $(not-executed)"
    distro = "RM65 Lab'; $(not-a-command)"

    def fake_run(command, **kwargs):
        calls.append((command, kwargs))
        return SimpleNamespace(stdout=converted + "\n")

    monkeypatch.setattr(servo_launcher.subprocess, "run", fake_run)
    assert servo_launcher.main(["--target", "wsl", "--wsl-distro", distro, "--mode", mode, "--sudo-docker"]) == 0
    prefix = ["wsl.exe", "--distribution", distro, "--user", "root"]
    assert calls[0][0] == [*prefix, "--exec", "wslpath", "-a", "-u", str(wsl_root)]
    assert calls[0][1]["encoding"] == "utf-8"
    assert len(calls) == 4
    assert calls[1][0] == [
        *prefix,
        "--cd",
        converted,
        "--exec",
        "docker",
        "image",
        "inspect",
        "rm65-servo:humble",
        "--format",
        "{{json .Config.Labels}}",
    ]
    assert calls[1][1]["encoding"] == "utf-8" and calls[1][1]["timeout"] == 10.0
    for command, kwargs in calls[2:]:
        assert command[: len(prefix) + 4] == [*prefix, "--cd", converted, "--exec", "docker"]
        assert kwargs == {"check": True}
        assert "sudo" not in command and "ssh" not in command
        assert "shell" not in kwargs
    assert calls[2][0][-5:] == [
        "build",
        "--build-arg",
        "RM65_WITH_XR_SDK=OFF",
        "--build-arg",
        "RM65_SOURCE_SHA=fixture-source-hash",
    ]
    if mode == "test":
        assert "--network" in calls[3][0] and "none" in calls[3][0]
        assert "colcon test" in calls[3][0][-1]
    else:
        assert "input_source:=mock" in calls[3][0] and "mode:=sim" in calls[3][0]


def mock_wsl_cache_probe(monkeypatch, *, stdout, error=None):
    calls = []

    def fake_run(command, **kwargs):
        calls.append((command, kwargs))
        if "wslpath" in command:
            return SimpleNamespace(stdout="/mnt/d/cached repo\n")
        if "inspect" in command:
            if error is not None:
                raise error
            return SimpleNamespace(stdout=stdout)
        return SimpleNamespace(stdout="")

    monkeypatch.setattr(servo_launcher.subprocess, "run", fake_run)
    return calls


@pytest.mark.parametrize("mode", ["sim", "test"])
@pytest.mark.parametrize("input_source,sdk", [("mock", "OFF"), ("external", "OFF"), ("pico", "ON")])
def test_matching_wsl_image_skips_build_and_retains_run_constraints(
    monkeypatch, capsys, wsl_root, mode, input_source, sdk
):
    digest_calls = []
    monkeypatch.setattr(servo_launcher, "source_digest", lambda: digest_calls.append(True) or "fixture-source-hash")
    calls = mock_wsl_cache_probe(
        monkeypatch,
        stdout=json.dumps({"org.rm65.source-sha": "fixture-source-hash", "org.rm65.xr-sdk": sdk}),
    )
    assert servo_launcher.main(["--target", "wsl", "--mode", mode, "--input", input_source]) == 0
    assert len(digest_calls) == 1  # Build plan and cache check use the same source snapshot hash.
    assert len(calls) == 3 and "wslpath" in calls[0][0] and "inspect" in calls[1][0]
    execution = calls[2][0]
    assert all("build" not in command and "ssh" not in command for command, _ in calls)
    assert "复用已构建镜像" in capsys.readouterr().out
    if mode == "test":
        assert "--network" in execution and "none" in execution
        assert "docker/compose.wsl.yml" not in execution
        assert "colcon test" in execution[-1]
    else:
        assert "docker/compose.wsl.yml" in execution and "rviz:=false" in execution
        assert "mujoco_viewer:=true" in execution
        assert f"input_source:={input_source}" in execution


@pytest.mark.parametrize(
    "input_source,labels",
    [
        ("mock", {"org.rm65.source-sha": "old-source", "org.rm65.xr-sdk": "OFF"}),
        ("mock", {"org.rm65.source-sha": "fixture-source-hash", "org.rm65.xr-sdk": "ON"}),
        ("pico", {"org.rm65.source-sha": "fixture-source-hash", "org.rm65.xr-sdk": "OFF"}),
        ("mock", {"org.rm65.source-sha": "fixture-source-hash", "org.rm65.xr-sdk": "off"}),
        ("mock", {"org.rm65.source-sha": "fixture-source-hash"}),
        ("mock", {"org.rm65.xr-sdk": "OFF"}),
        ("mock", {}),
        ("mock", None),
        ("mock", []),
    ],
)
def test_wsl_cache_mismatch_or_missing_labels_performs_normal_build(monkeypatch, wsl_root, input_source, labels):
    calls = mock_wsl_cache_probe(monkeypatch, stdout=json.dumps(labels))
    assert servo_launcher.main(["--target", "wsl", "--mode", "sim", "--input", input_source]) == 0
    assert len(calls) == 4 and "inspect" in calls[1][0] and "build" in calls[2][0]
    sdk = "ON" if input_source == "pico" else "OFF"
    assert f"RM65_WITH_XR_SDK={sdk}" in calls[2][0]
    assert "RM65_SOURCE_SHA=fixture-source-hash" in calls[2][0]
    assert "mode:=sim" in calls[3][0]
    assert all(command[0] == "wsl.exe" and "ssh" not in command for command, _ in calls)


@pytest.mark.parametrize(
    "error",
    [
        servo_launcher.subprocess.CalledProcessError(1, ["docker", "image", "inspect"]),
        servo_launcher.subprocess.TimeoutExpired(["docker", "image", "inspect"], 10),
        OSError("daemon inspect unavailable"),
        None,
    ],
)
def test_wsl_cache_inspect_failure_or_invalid_json_builds_without_ssh_fallback(monkeypatch, wsl_root, error):
    calls = mock_wsl_cache_probe(monkeypatch, stdout="not JSON", error=error)
    assert servo_launcher.main(["--target", "wsl", "--mode", "test"]) == 0
    assert len(calls) == 4 and "build" in calls[2][0]
    execution = calls[3][0]
    assert "--network" in execution and "none" in execution
    assert "docker/compose.wsl.yml" not in execution
    assert all("ssh" not in command for command, _ in calls)


@pytest.mark.parametrize("target", ["remote", "local"])
def test_non_wsl_never_inspects_or_reuses_wsl_image_cache(monkeypatch, target):
    calls = []
    monkeypatch.setattr(servo_launcher.subprocess, "run", lambda command, **kwargs: calls.append(command))
    monkeypatch.setattr(servo_launcher, "source_archive", lambda: b"source-only")
    monkeypatch.setattr(servo_launcher, "wsl_cached_image_matches", lambda *a, **k: pytest.fail("non-WSL cache probe"))
    assert servo_launcher.main(["--target", target, "--mode", "test"]) == 0
    assert all("inspect" not in command and "wsl.exe" not in command for command in calls)
    assert any("build" in command if target == "local" else " build " in command[-1] for command in calls)


@pytest.mark.parametrize("labels,expected", [
    ({"org.rm65.source-sha": "prepared", "org.rm65.xr-sdk": "ON"}, True),
    ({"org.rm65.source-sha": "different", "org.rm65.xr-sdk": "ON"}, False),
    ({"org.rm65.source-sha": "prepared", "org.rm65.xr-sdk": "OFF"}, False),
    ({"org.rm65.xr-sdk": "ON"}, False),
    (None, False),
    ([], False),
])
def test_remote_prepared_cache_requires_source_and_sdk(monkeypatch, labels, expected):
    calls = []
    monkeypatch.setattr(servo_launcher.subprocess, "run", lambda command, **kwargs:
                        calls.append((command, kwargs)) or SimpleNamespace(stdout=json.dumps(labels)))
    args = SimpleNamespace(host="rm", sudo_docker=True, input="pico")
    assert servo_launcher.remote_cached_image_matches(args, "prepared") is expected
    assert len(calls) == 1
    assert calls[0][0][-1].startswith("sudo -n docker image inspect")
    assert calls[0][1]["timeout"] == 15.0


def test_matching_prepared_board_image_skips_build_but_keeps_launch(monkeypatch):
    calls = []
    monkeypatch.setattr(servo_launcher, "source_archive", lambda: b"source-only")
    monkeypatch.setattr(servo_launcher, "remote_cached_image_matches", lambda *a: True)
    monkeypatch.setattr(servo_launcher.subprocess, "run", lambda command, **kwargs: calls.append(command))
    assert servo_launcher.main(["--target", "remote", "--mode", "hardware-dry-run", "--input", "pico",
                                "--udp-ip", "192.168.10.100"]) == 0
    assert len(calls) == 2
    assert not any(" build " in command[-1] for command in calls)
    assert "mode:=hardware-dry-run" in calls[-1][-1]


@pytest.mark.parametrize(
    "mode,viewer,expected_gui",
    [("sim", True, True), ("sim", False, False), ("test", True, False), ("test", False, False)],
)
def test_wsl_viewer_overlay_is_only_for_visible_simulation(monkeypatch, wsl_root, mode, viewer, expected_gui):
    calls = []
    monkeypatch.setattr(
        servo_launcher.subprocess,
        "run",
        lambda command, **kwargs: calls.append(command) or SimpleNamespace(stdout="/mnt/d/repo\n"),
    )
    assert servo_launcher.main(["--target", "wsl", "--mode", mode, "--viewer" if viewer else "--no-viewer"]) == 0
    build, execution = calls[2:]
    overlay = "docker/compose.wsl.yml"
    assert (overlay in build) is expected_gui
    assert (overlay in execution) is expected_gui
    if mode == "test":
        assert "--network" in execution and "none" in execution
        assert not any(arg.startswith("rviz:=") or arg.startswith("foxglove:=") for arg in execution)
        assert "--service-ports" not in execution
    else:
        assert "rviz:=false" in execution
        assert f"mujoco_viewer:={'true' if expected_gui else 'false'}" in execution
        assert f"foxglove:={'true' if viewer else 'false'}" in execution
    assert all("ssh" not in command for command in calls)


@pytest.mark.parametrize("target", ["local", "remote"])
@pytest.mark.parametrize("viewer", [True, False])
def test_non_wsl_viewer_keeps_existing_foxglove_behavior(monkeypatch, target, viewer):
    monkeypatch.setattr(servo_launcher, "source_digest", lambda: "fixture-source-hash")
    args = SimpleNamespace(
        target=target,
        mode="sim",
        simulator="mujoco",
        viewer=viewer,
        rviz=False,
        sudo_docker=False,
        input="mock",
        hand="off",
        arm_ip="192.168.10.18",
        udp_ip="",
        model_only=False,
    )
    build, execution = servo_launcher.commands(args)
    assert "docker/compose.wsl.yml" not in build and "docker/compose.wsl.yml" not in execution
    assert "rviz:=false" in execution
    assert f"foxglove:={'true' if viewer else 'false'}" in execution


def test_wsl_graphics_overlay_is_part_of_source_delivery(tmp_path):
    assert "docker/compose.wsl.yml" in servo_launcher.SOURCE_PATHS
    directory = tmp_path / "docker"
    directory.mkdir()
    overlay = directory / "compose.wsl.yml"
    overlay.write_text("services: {}\n", encoding="utf-8")
    assert overlay in list(servo_launcher.source_files(tmp_path))


@pytest.mark.parametrize("mode", ["hardware", "hardware-dry-run", "sync-only"])
def test_wsl_rejects_every_hardware_or_sync_mode_before_process(monkeypatch, mode):
    monkeypatch.setattr(
        servo_launcher.subprocess, "run", lambda *a, **k: pytest.fail("forbidden WSL mode ran a process")
    )
    monkeypatch.setattr(servo_launcher, "source_archive", lambda: pytest.fail("forbidden WSL mode synced source"))
    assert servo_launcher.main(["--target", "wsl", "--mode", mode]) == 2


@pytest.mark.parametrize("mode", ["sim", "test"])
def test_windows_default_prefers_registered_distro_without_process_probe(monkeypatch, capsys, wsl_root, mode):
    looked_up = []
    monkeypatch.setattr(servo_launcher, "is_windows_host", lambda: True)
    monkeypatch.setattr(servo_launcher, "wsl_distro_registered", lambda name: looked_up.append(name) or True)
    monkeypatch.setattr(
        servo_launcher.subprocess, "run", lambda *a, **k: pytest.fail("offline default probed a process")
    )
    assert servo_launcher.main(["--mode", mode, "--print-command"]) == 0
    assert looked_up == ["RM65-Servo"]
    output = capsys.readouterr().out
    assert "wsl.exe --distribution RM65-Servo" in output and "先同步" not in output


@pytest.mark.parametrize(
    "windows,registered,mode,expected",
    [
        (True, False, "sim", "remote"),
        (True, True, "hardware", "remote"),
        (True, True, "hardware-dry-run", "remote"),
        (True, True, "sync-only", "remote"),
        (False, True, "sim", "local"),
        (False, True, "test", "local"),
    ],
)
def test_target_defaults_preserve_existing_remote_and_local_behavior(monkeypatch, windows, registered, mode, expected):
    monkeypatch.setattr(servo_launcher, "is_windows_host", lambda: windows)
    monkeypatch.setattr(servo_launcher, "wsl_distro_registered", lambda name: registered)
    args = SimpleNamespace(target=None, mode=mode, wsl_distro="RM65-Servo")
    assert servo_launcher.resolve_target(args) == expected


def test_offline_registry_discovery_handles_partial_entries_and_case_insensitive_name(monkeypatch):
    import sys

    class Key:
        def __init__(self, name):
            self.name = name

        def __enter__(self):
            return self

        def __exit__(self, *args):
            pass

    def enumerate_key(registry, index):
        if index >= 2:
            raise OSError("no more entries")
        return ["partial", "working"][index]

    def query(key, value):
        assert value == "DistributionName"
        if key.name == "partial":
            raise FileNotFoundError("incomplete registration")
        return "rm65-servo", 1

    fake_registry = SimpleNamespace(
        HKEY_CURRENT_USER=object(),
        OpenKey=lambda parent, name: Key(name),
        EnumKey=enumerate_key,
        QueryValueEx=query,
    )
    monkeypatch.setitem(sys.modules, "winreg", fake_registry)
    monkeypatch.setattr(servo_launcher, "is_windows_host", lambda: True)
    monkeypatch.setattr(servo_launcher.subprocess, "run", lambda *a, **k: pytest.fail("registry probe ran a process"))
    assert servo_launcher.wsl_distro_registered("RM65-Servo")
    assert not servo_launcher.wsl_distro_registered("Ubuntu-other")


@pytest.mark.parametrize("target", ["remote", "local", "wsl"])
def test_explicit_target_never_probes_or_changes_destination(monkeypatch, target):
    monkeypatch.setattr(servo_launcher, "wsl_distro_registered", lambda name: pytest.fail("explicit target was probed"))
    args = SimpleNamespace(target=target, mode="sim", wsl_distro="RM65-Servo")
    assert servo_launcher.resolve_target(args) == target


def test_failed_wslpath_never_falls_back_to_ssh_or_launches_docker(monkeypatch, wsl_root):
    calls = []

    def fake_run(command, **kwargs):
        calls.append(command)
        raise servo_launcher.subprocess.CalledProcessError(1, command)

    monkeypatch.setattr(servo_launcher.subprocess, "run", fake_run)
    assert servo_launcher.main(["--target", "wsl", "--mode", "sim"]) == 2
    assert len(calls) == 1 and "wslpath" in calls[0]


@pytest.mark.parametrize("output", ["", "relative/path\n", "/mnt/d/repo\nextra output\n"])
def test_invalid_wslpath_output_stops_before_build(monkeypatch, wsl_root, output):
    calls = []
    monkeypatch.setattr(
        servo_launcher.subprocess,
        "run",
        lambda command, **kwargs: calls.append(command) or SimpleNamespace(stdout=output),
    )
    assert servo_launcher.main(["--target", "wsl", "--mode", "sim"]) == 2
    assert len(calls) == 1


def test_retired_package_is_absent_and_cannot_be_imported():
    import subprocess
    import sys

    assert not (servo_launcher.ROOT / "xrobotoolkit_teleop").exists()
    result = subprocess.run(
        [sys.executable, "-B", "-c", "import xrobotoolkit_teleop"],
        cwd=servo_launcher.ROOT,
        capture_output=True,
        text=True,
    )
    assert result.returncode != 0
    assert "No module named 'xrobotoolkit_teleop'" in result.stderr
