import importlib
import sys
import types

import numpy as np
import pytest

if "meshcat.transformations" not in sys.modules:
    meshcat = types.ModuleType("meshcat")
    transformations = types.ModuleType("meshcat.transformations")
    meshcat.transformations = transformations
    sys.modules.setdefault("meshcat", meshcat)
    sys.modules.setdefault("meshcat.transformations", transformations)

if "xrobotoolkit_sdk" not in sys.modules:
    sys.modules["xrobotoolkit_sdk"] = types.ModuleType("xrobotoolkit_sdk")

if "tyro" not in sys.modules:
    tyro = types.ModuleType("tyro")
    tyro.cli = lambda function: function
    sys.modules["tyro"] = tyro

import scripts.hardware.teleop_realman_rm65_safe_hardware as hardware_entrypoint
import scripts.hardware.teleop_realman_rm65_hardware as legacy_hardware_entrypoint
import xrobotoolkit_teleop.hardware.legacy.cartesian_teleop_controller as controller_module
from xrobotoolkit_teleop.hardware.legacy.cartesian_teleop_controller import (
    RealmanRM65CartesianTeleopController,
)


def _quat_multiply(left: np.ndarray, right: np.ndarray) -> np.ndarray:
    w1, x1, y1, z1 = left
    w2, x2, y2, z2 = right
    return np.array(
        [
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        ],
        dtype=float,
    )


def _quat_conjugate(quat: np.ndarray) -> np.ndarray:
    return np.array([quat[0], -quat[1], -quat[2], -quat[3]], dtype=float)


def _quat_from_matrix(matrix: np.ndarray) -> np.ndarray:
    rotation = np.asarray(matrix, dtype=float)[:3, :3]
    trace = float(np.trace(rotation))
    if trace > 0.0:
        scale = np.sqrt(trace + 1.0) * 2.0
        return np.array(
            [
                0.25 * scale,
                (rotation[2, 1] - rotation[1, 2]) / scale,
                (rotation[0, 2] - rotation[2, 0]) / scale,
                (rotation[1, 0] - rotation[0, 1]) / scale,
            ]
        )

    diagonal = np.diag(rotation)
    axis = int(np.argmax(diagonal))
    next_axis = (axis + 1) % 3
    last_axis = (axis + 2) % 3
    scale = np.sqrt(1.0 + rotation[axis, axis] - rotation[next_axis, next_axis] - rotation[last_axis, last_axis]) * 2.0
    quat = np.zeros(4, dtype=float)
    quat[axis + 1] = 0.25 * scale
    quat[0] = (rotation[last_axis, next_axis] - rotation[next_axis, last_axis]) / scale
    quat[next_axis + 1] = (rotation[next_axis, axis] + rotation[axis, next_axis]) / scale
    quat[last_axis + 1] = (rotation[last_axis, axis] + rotation[axis, last_axis]) / scale
    return quat


def _quat_diff_as_rotvec(source: np.ndarray, target: np.ndarray) -> np.ndarray:
    delta = _quat_multiply(target, _quat_conjugate(source))
    delta = delta / np.linalg.norm(delta)
    if delta[0] < 0.0:
        delta = -delta
    angle = 2.0 * np.arccos(np.clip(delta[0], -1.0, 1.0))
    if angle < 1e-9:
        return np.zeros(3, dtype=float)
    return delta[1:] / np.sin(angle / 2.0) * angle


def _make_controller(monkeypatch) -> RealmanRM65CartesianTeleopController:
    monkeypatch.setattr(controller_module, "RealmanRM65Interface", lambda **kwargs: object())
    monkeypatch.setattr(controller_module, "XrClient", lambda: object())
    monkeypatch.setattr(controller_module.tf, "quaternion_from_matrix", _quat_from_matrix, raising=False)
    monkeypatch.setattr(controller_module.tf, "quaternion_multiply", _quat_multiply, raising=False)
    monkeypatch.setattr(controller_module.tf, "quaternion_conjugate", _quat_conjugate, raising=False)
    monkeypatch.setattr(controller_module, "quat_diff_as_angle_axis", _quat_diff_as_rotvec)
    return RealmanRM65CartesianTeleopController(
        scale_factor=1.0,
        max_offset_m=2.0,
        max_rot_offset_rad=1.0,
        motion_deadband_m=0.0,
        rot_deadband_rad=0.0,
        suppress_rotation_during_translation=False,
    )


def test_default_hardware_mapping_sends_xr_translation_and_rotation_x_to_rm_positive_y(monkeypatch):
    controller = _make_controller(monkeypatch)
    controller._process_xr_pose(np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]))
    angle = 0.2
    moved_pose = np.array([1.0, 0.0, 0.0, np.sin(angle / 2.0), 0.0, 0.0, np.cos(angle / 2.0)])

    delta_xyz, delta_rot = controller._process_xr_pose(moved_pose)

    np.testing.assert_allclose(delta_xyz, [0.0, 1.0, 0.0], atol=1e-9)
    np.testing.assert_allclose(delta_rot, [0.0, angle, 0.0], atol=1e-9)


@pytest.mark.parametrize(
    "entry_module",
    [
        "scripts.hardware.teleop_realman_rm65_safe_hardware",
        "scripts.hardware.teleop_realman_rm65_placo_hardware",
    ],
)
def test_recommended_hardware_entrypoint_does_not_apply_an_extra_xy_inversion(monkeypatch, entry_module):
    captured = {}

    class _Controller:
        def __init__(self, **kwargs):
            captured.update(kwargs)

        def run(self):
            return None

    monkeypatch.setattr(hardware_entrypoint, "RealmanRM65SafeTeleopController", _Controller)

    importlib.import_module(entry_module).main()

    assert captured["invert_tcp_xy"] is False
    assert captured["suppress_rotation_during_translation"] is False
    assert captured["move_v"] == 5
    assert captured["motion_command"] == "damped_ik_movej_follow"
    assert captured["scale_factor"] == 0.25
    assert captured["rotation_scale_factor"] == 0.25
    assert captured["max_offset_m"] == 0.03
    assert captured["max_rot_offset_rad"] == 0.20
    assert captured["max_linear_velocity_m_s"] == 0.05
    assert captured["max_angular_velocity_rad_s"] == 0.25
    # Singularity avoidance defaults must reach the controller unchanged.
    assert captured["enable_singularity_avoidance"] is True
    assert captured["singularity_slowdown_ratio"] == 0.04
    assert captured["singularity_stop_ratio"] == 0.01
    assert captured["singularity_slowdown_joint_deg"] == 15.0
    assert captured["singularity_stop_joint_deg"] == 5.0
    assert captured["singularity_min_speed_scale"] == 0.05
    assert captured["singularity_escape_speed_scale"] == 0.10
    assert captured["singularity_max_damping"] == 0.08
    assert captured["singularity_report_interval_s"] == 2.0


def test_legacy_single_file_entrypoint_rejects_live_motion():
    with pytest.raises(RuntimeError, match="motion-disabled"):
        legacy_hardware_entrypoint.main(legacy_hardware_entrypoint.Args(dry_run=False))


def test_rm65_sim_entrypoint_uses_the_hardware_axis_mapping(monkeypatch):
    captured = {}

    class _JointsTask:
        def set_joints(self, joints):
            return None

        def configure(self, *args):
            return None

    class _Solver:
        def add_joints_task(self):
            return _JointsTask()

    class _Robot:
        def joint_names(self):
            return [f"joint_{index}" for index in range(6)]

    class _Controller:
        def __init__(self, **kwargs):
            captured.update(kwargs)
            self.solver = _Solver()
            self.placo_robot = _Robot()

        def run(self):
            return None

    fake_placo_module = types.ModuleType("xrobotoolkit_teleop.simulation.placo_teleop_controller")
    fake_placo_module.PlacoTeleopController = _Controller
    module_name = "scripts.simulation.teleop_rm65_sim"
    monkeypatch.setitem(sys.modules, "xrobotoolkit_teleop.simulation.placo_teleop_controller", fake_placo_module)
    monkeypatch.delitem(sys.modules, module_name, raising=False)
    sim_entrypoint = importlib.import_module(module_name)

    sim_entrypoint.main(home_joint_deg=(1.0, -2.0, 3.0, -4.0, 5.0, -6.0))

    np.testing.assert_array_equal(
        captured["R_headset_world"],
        [[0.0, 1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, -1.0]],
    )

    official_module = types.ModuleType("xrobotoolkit_teleop.simulation.realman_official_ik_teleop_controller")
    official_module.RealmanOfficialIKSimulationController = _Controller
    monkeypatch.setitem(
        sys.modules,
        "xrobotoolkit_teleop.simulation.realman_official_ik_teleop_controller",
        official_module,
    )
    captured.clear()

    sim_entrypoint.main(
        home_joint_deg=(1.0, -2.0, 6.0, -4.0, 5.0, -6.0),
        ik_backend="official",
    )

    assert captured["official_ik_tool_or_work"] == 1
    assert captured["official_ik_j3_exclusion_deg"] == 5.0
    assert captured["official_ik_max_joint_speed_ratio"] == 0.10


def test_legacy_controller_import_preserves_class_identity():
    from xrobotoolkit_teleop.hardware.realman_rm65_cartesian_teleop_controller import (
        RealmanRM65CartesianTeleopController as compatible_controller,
    )

    assert compatible_controller is RealmanRM65CartesianTeleopController


@pytest.mark.parametrize(
    "filename",
    [
        "teleop_realman_rm65_safe_hardware.py",
        "teleop_realman_rm65_placo_hardware.py",
    ],
)
def test_hardware_cli_script_routes_to_safe_controller_without_motion(monkeypatch, filename):
    import runpy
    from pathlib import Path

    script_dir = Path(__file__).resolve().parents[1] / "scripts" / "hardware"
    monkeypatch.syspath_prepend(str(script_dir))
    cli_targets = []
    monkeypatch.setattr(sys.modules["tyro"], "cli", cli_targets.append)
    runpy.run_path(str(script_dir / filename), run_name="__main__")

    assert len(cli_targets) == 1
    assert (
        cli_targets[0].__globals__["RealmanRM65SafeTeleopController"]
        is hardware_entrypoint.RealmanRM65SafeTeleopController
    )
