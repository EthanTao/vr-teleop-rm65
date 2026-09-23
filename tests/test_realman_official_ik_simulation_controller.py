import importlib
import sys
import types
from types import SimpleNamespace

import numpy as np
import pytest

from xrobotoolkit_teleop.hardware.rm65_model import RM65_B_SPEC
from xrobotoolkit_teleop.simulation.realman_official_ik_guard import (
    OfficialIKSimulationGuard,
)


def _controller_for_validation():
    return OfficialIKSimulationGuard(j3_exclusion_deg=5.0, max_joint_speed_ratio=0.10)


def test_official_simulation_accepts_small_safe_joint_step():
    controller = _controller_for_validation()
    seed = np.deg2rad(np.array([0.0, -30.0, 90.0, 0.0, 90.0, 0.0]))
    max_step = RM65_B_SPEC.max_joint_velocity_rad_s * 0.10 * 0.02
    target = seed + max_step * 0.5

    assert controller.validate(seed, target, 0.02) == (True, "")


def test_official_simulation_rejects_large_joint_step():
    controller = _controller_for_validation()
    seed = np.deg2rad(np.array([0.0, -30.0, 90.0, 0.0, 90.0, 0.0]))
    target = seed.copy()
    target[0] += np.deg2rad(1.0)

    valid, reason = controller.validate(seed, target, 0.02)

    assert not valid
    assert "J1 single-cycle step" in reason


def test_official_simulation_rejects_j3_zero_zone():
    controller = _controller_for_validation()
    seed = np.deg2rad(np.array([0.0, -30.0, 6.0, 0.0, 90.0, 0.0]))
    target = seed.copy()
    target[2] = np.deg2rad(4.9)

    valid, reason = controller.validate(seed, target, 0.02)

    assert not valid
    assert "J3 zero-angle exclusion zone" in reason


@pytest.fixture
def official_simulation(monkeypatch):
    transformations = types.ModuleType("meshcat.transformations")
    transformations.quaternion_from_matrix = lambda _matrix: np.array([1.0, 0.0, 0.0, 0.0])
    meshcat = types.ModuleType("meshcat")
    meshcat.transformations = transformations
    parent_module = types.ModuleType("xrobotoolkit_teleop.simulation.placo_teleop_controller")

    class _FakePlacoController:
        pass

    parent_module.PlacoTeleopController = _FakePlacoController
    monkeypatch.setitem(sys.modules, "meshcat", meshcat)
    monkeypatch.setitem(sys.modules, "meshcat.transformations", transformations)
    monkeypatch.setitem(
        sys.modules,
        "xrobotoolkit_teleop.simulation.placo_teleop_controller",
        parent_module,
    )
    module_name = "xrobotoolkit_teleop.simulation.realman_official_ik_teleop_controller"
    monkeypatch.delitem(sys.modules, module_name, raising=False)
    module = importlib.import_module(module_name)

    class _FakeIK:
        def __init__(self, target):
            self.target = target
            self.seed = None

        def solve(self, _xyz, _quat, seed):
            self.seed = np.asarray(seed).copy()
            return self.target.copy()

    try:
        controller = object.__new__(module.RealmanOfficialIKSimulationController)
        seed = np.deg2rad(np.array([0.0, -30.0, 90.0, 0.0, 90.0, 0.0]))
        target = seed + np.deg2rad(np.full(6, 0.1))
        controller.joint_guard = OfficialIKSimulationGuard(5.0, 0.10)
        controller.dt = 0.02
        controller.official_ik_solver = _FakeIK(target)
        controller.placo_robot = SimpleNamespace(state=SimpleNamespace(q=np.r_[np.zeros(7), seed]))
        controller.manipulator_config = {"right_hand": {"link_name": "r_link6"}}
        controller.active = {"right_hand": True}
        controller.effector_control_mode = {"right_hand": "pose"}
        target_frame = np.eye(4)
        target_frame[:3, 3] = [0.3, -0.1, 0.5]
        controller.effector_task = {"right_hand": SimpleNamespace(T_world_frame=target_frame)}
        controller.last_official_ik_error = None
        controller._last_official_ik_error_t = 0.0

        yield controller, seed, target
    finally:
        sys.modules.pop(module_name, None)


def test_official_simulation_controller_applies_solver_joint_target(official_simulation):
    controller, seed, target = official_simulation
    controller._solve_ik()
    np.testing.assert_allclose(controller.placo_robot.state.q[7:], target)
    np.testing.assert_allclose(controller.official_ik_solver.seed, seed)


def test_official_simulation_continues_from_last_accepted_state(official_simulation):
    controller, seed, first = official_simulation
    controller._solve_ik()
    second = first + np.deg2rad(0.1)
    controller.official_ik_solver.target = second
    controller._solve_ik()
    np.testing.assert_allclose(controller.official_ik_solver.seed, first)
    np.testing.assert_allclose(controller.placo_robot.state.q[7:], second)


@pytest.mark.parametrize("failure", ["step", "j3", "solver"])
def test_official_simulation_rejection_does_not_advance_reference(official_simulation, monkeypatch, failure):
    controller, seed, first = official_simulation
    controller._solve_ik()
    invalid = first.copy()
    if failure == "j3":
        invalid[2] = 0.0
    else:
        invalid[0] += np.deg2rad(10.0)
    controller.official_ik_solver.target = invalid
    with monkeypatch.context() as patch:
        if failure == "solver":

            def fail(*_args):
                raise RuntimeError("injected IK failure")

            patch.setattr(controller.official_ik_solver, "solve", fail)
        controller._solve_ik()
    assert controller.last_official_ik_error is not None
    np.testing.assert_allclose(controller.placo_robot.state.q[7:], first)
    accepted = first + np.deg2rad(0.1)
    controller.official_ik_solver.target = accepted
    controller._solve_ik()
    np.testing.assert_allclose(controller.official_ik_solver.seed, first)
    np.testing.assert_allclose(controller.placo_robot.state.q[7:], accepted)


def test_official_simulation_reactivation_uses_reset_state(official_simulation):
    controller, seed, first = official_simulation
    controller._solve_ik()
    controller.active["right_hand"] = False
    reset_state = seed + np.deg2rad(2.0)
    controller.placo_robot.state.q[7:] = reset_state
    controller._solve_ik()
    np.testing.assert_allclose(controller.placo_robot.state.q[7:], reset_state)
    controller.active["right_hand"] = True
    controller.official_ik_solver.target = reset_state + np.deg2rad(0.1)
    controller._solve_ik()
    np.testing.assert_allclose(controller.official_ik_solver.seed, reset_state)


def test_official_simulation_sdk_cannot_mutate_guard_reference(official_simulation, monkeypatch):
    controller, seed, _ = official_simulation

    def mutate(_xyz, _quat, solver_seed):
        solver_seed[0] += np.deg2rad(10.0)
        return solver_seed

    monkeypatch.setattr(controller.official_ik_solver, "solve", mutate)
    controller._solve_ik()
    np.testing.assert_allclose(controller.placo_robot.state.q[7:], seed)
    assert "single-cycle step" in controller.last_official_ik_error
