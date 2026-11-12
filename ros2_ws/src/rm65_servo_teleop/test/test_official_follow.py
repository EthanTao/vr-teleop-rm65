"""Continuous-follow policy and SDK boundary tests, with no robot connection."""

import ast
import io
import json
import math
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from rm65_servo_teleop.follow_targets import limit_pose_step
from rm65_servo_teleop.input_adapter import Pose, quaternion_rotvec, rotvec_quaternion
from rm65_servo_teleop.official_follow import FollowConfig, OfficialFollower
from rm65_servo_teleop.official_follow_runner import demo_rows, run_rows
from rm65_servo_teleop.official_sdk import IKRejected, OfficialAlgo


class Clock:
    value = 0.0

    def __call__(self):
        return self.value


class AnalyticTestSolver:
    """Test-only invertible map; explicitly not RM65 kinematics or dynamics."""

    def __init__(self, scale=1.0):
        self.scale, self.fail, self.calls = scale, False, 0

    def forward(self, q):
        return Pose(np.asarray(q[:3]) / self.scale, rotvec_quaternion(np.asarray(q[3:])))

    def solve(self, target, seed):
        self.calls += 1
        if self.fail:
            seed[:] = 123.0  # A vendor implementation must not mutate our reference.
            raise IKRejected("injected")
        return np.r_[target.xyz * self.scale, quaternion_rotvec(target.quaternion_xyzw)]


def config(**changes):
    return FollowConfig(joint_lower=(-3.0,) * 6, joint_upper=(3.0,) * 6, joint_velocity=(1.0,) * 6,
                        workspace_min=(-1.0,) * 3, workspace_max=(1.0,) * 3, **changes)


def goal(x=0.1):
    return Pose(np.array([x, 0.0, 0.0]), np.array([0.0, 0.0, 0.0, 1.0]))


def step(follower, solver, now, *, enabled=True, q=None, target=None, **changes):
    q = np.zeros(6) if q is None else q
    args = dict(enabled=enabled, desired=goal() if target is None else target, target_at=now,
                measured_pose=solver.forward(q), measured_joints=q, feedback_at=now, now=now)
    args.update(changes)
    return follower.step(**args)


def started(**changes):
    solver, clock = AnalyticTestSolver(), Clock()
    follower = OfficialFollower(config(**changes), solver, clock=clock)
    assert step(follower, solver, 0.0, enabled=False).state == "idle"
    assert step(follower, solver, 0.02).state == "anchoring"
    return follower, solver, clock


def test_held_grip_continuously_advances_without_reanchoring():
    follower, solver, _ = started()
    q = np.zeros(6)
    results = []
    for index in range(2, 102):
        result = step(follower, solver, index * 0.02, q=q)
        assert result.accepted and result.state == "tracking"
        next_q = np.asarray(result.command_rad)
        assert np.max(np.abs(next_q - q)) <= 0.02 + 1e-9
        q = next_q
        results.append(result)
    assert q[0] == pytest.approx(0.1)
    assert all(row.record()["execution_authorized"] is False for row in results)


def test_one_rejected_frame_keeps_reference_and_resumes_without_regrip():
    follower, solver, _ = started()
    accepted = step(follower, solver, 0.04)
    q = np.asarray(accepted.command_rad)
    solver.fail = True
    result = step(follower, solver, 0.06, q=q)
    assert result.state == "hold" and not result.accepted
    np.testing.assert_array_equal(result.command_rad, q)
    np.testing.assert_array_equal(follower.seed, q)
    assert result.attempts == 4
    solver.fail = False
    assert step(follower, solver, 0.08, q=q).accepted


def test_persistent_rejection_faults_until_release_then_anchors_measured():
    follower, solver, _ = started()
    solver.fail = True
    assert step(follower, solver, 0.04).state == "hold"
    assert step(follower, solver, 0.18).state == "fault"
    solver.fail = False
    assert step(follower, solver, 0.20).state == "fault"
    assert step(follower, solver, 0.22, enabled=False).state == "idle"
    q = np.array([0.3, 0, 0, 0, 0, 0])
    result = step(follower, solver, 0.24, q=q)
    assert result.state == "anchoring"
    np.testing.assert_allclose(result.command_rad, q)


def test_joint_jump_retries_smaller_cartesian_step_without_component_clipping():
    solver = AnalyticTestSolver(scale=10)
    follower = OfficialFollower(config(), solver, clock=Clock())
    step(follower, solver, 0.0, enabled=False)
    step(follower, solver, 0.02)
    result = step(follower, solver, 0.04)
    assert result.accepted and result.attempts == 2
    assert result.command_rad[0] == pytest.approx(0.015)
    assert result.target.xyz[0] == pytest.approx(0.0015)


def test_stationary_feedback_cannot_support_runaway_candidates():
    follower, solver, _ = started()
    last = np.zeros(6)
    for index in range(2, 100):
        result = step(follower, solver, index * 0.02)
        if result.command_rad is not None:
            last = np.asarray(result.command_rad)
        assert np.max(np.abs(last)) <= 0.06 + 1e-9
        if result.state == "fault":
            break
    assert result.state == "fault"


@pytest.mark.parametrize("change", [{"target_at": -1.0}, {"feedback_at": None}, {"feedback_at": 1.0}])
def test_stale_missing_or_future_sources_fault(change):
    follower, solver, _ = started()
    assert step(follower, solver, 0.04, **change).state == "fault"


def test_startup_held_grip_does_not_engage():
    solver = AnalyticTestSolver()
    follower = OfficialFollower(config(), solver)
    assert step(follower, solver, 0.0).reason == "release_grip_to_start"


def test_measured_fk_mismatch_is_not_hidden_by_anchor_reset():
    follower, solver, _ = started()
    result = step(follower, solver, 0.04, measured_pose=goal(0.1))
    assert result.reason == "official_fk_feedback_frame_mismatch" and result.command_rad is None


def test_late_sdk_result_does_not_advance_reference():
    follower, solver, clock = started()
    original = solver.solve

    def late(*args):
        value = original(*args)
        clock.value += 0.016
        return value

    solver.solve = late
    result = step(follower, solver, 0.04)
    assert result.reason == "compute_budget" and not result.accepted
    np.testing.assert_array_equal(follower.seed, np.zeros(6))


def test_solution_fk_residual_is_checked():
    follower, solver, _ = started()
    solver.solve = lambda target, seed: np.zeros(6)
    result = step(follower, solver, 0.04)
    # Backtracking may legitimately reach a target within the residual tolerance.
    if result.accepted:
        assert np.linalg.norm(result.target.xyz) <= follower.config.solution_position_tolerance_m
    else:
        assert result.reason == "ik_fk_residual"


def test_workspace_goal_is_clipped_and_outside_feedback_faults():
    follower, solver, _ = started()
    result = step(follower, solver, 0.04, target=goal(3.0))
    assert result.accepted and result.target.xyz[0] <= 1.0
    assert step(follower, solver, 0.06, q=np.array([1.2, 0, 0, 0, 0, 0])).state == "fault"


def test_upstream_rotational_limit_handles_equivalent_pi_representations():
    value = limit_pose_step((0, 0, 0, 0, 0, math.pi - .001),
                            (0, 0, 0, 0, 0, -math.pi + .001), .01, .01)
    assert abs(abs(value[5]) - math.pi) < .002


def test_replay_demo_marks_all_feedback_synthetic_and_never_authorizes_execution():
    solver, cfg, output = AnalyticTestSolver(), config(), io.StringIO()
    summary = run_rows(demo_rows(solver, [0] * 6, cfg), solver, cfg, output, synthetic=True)
    assert summary["accepted"] > 50 and summary["hardware_commands_sent"] == 0
    rows = [json.loads(line) for line in output.getvalue().splitlines()]
    assert all(row["synthetic_feedback"] and row["execution_authorized"] is False for row in rows)


@pytest.mark.parametrize("change", [{"period_s": -1}, {"solve_budget_s": .03},
                                    {"reject_hold_s": .3}, {"max_attempts": 99}])
def test_config_rejects_unbounded_or_incoherent_timing(change):
    with pytest.raises(ValueError):
        config(**change)


def test_sdk_adapter_uses_only_algo_and_converts_units_and_quaternion_order(monkeypatch):
    monkeypatch.setattr(OfficialAlgo, "_context_claimed", False)
    calls = []

    class FakeAlgo:
        def __init__(self, *args):
            calls.append(("init", args))
            self.dh = {name: [0.0] * 6 for name in ("d", "a", "alpha", "offset")}

        def rm_algo_get_dh(self):
            return {name: list(values) for name, values in self.dh.items()}

        def rm_algo_set_dh(self, value):
            self.dh = value
            calls.append(("dh", value))

        def rm_algo_set_workframe(self, value):
            calls.append(("work", value))

        def rm_algo_set_toolframe(self, value):
            calls.append(("tool", value))

        def rm_algo_inverse_kinematics(self, params):
            calls.append(("ik", params))
            return 0, [90, 0, 0, 0, 0, 0]

        def rm_algo_forward_kinematics(self, q, flag):
            assert flag == 0
            return [0.4, 0, 0.3, 1, 0, 0, 0]

    sdk = SimpleNamespace(Algo=FakeAlgo, rm_robot_arm_model_e=SimpleNamespace(RM_MODEL_RM_65_E=1),
                          rm_force_type_e=SimpleNamespace(RM_MODEL_RM_B_E=0),
                          rm_frame_t=lambda *args: args,
                          rm_dh_t=lambda **kwargs: kwargs,
                          rm_inverse_kinematics_params_t=lambda *args: args)
    adapter = OfficialAlgo([0, 0, .1], [0, 0, 0], d6_m=.1612, sdk=sdk)
    assert adapter.algo.dh["d"][5] == pytest.approx(.1612)
    q = adapter.solve(goal(), np.array([math.pi/2, 0, 0, 0, 0, 0]))
    assert q[0] == pytest.approx(math.pi/2)
    params = next(args for name, args in calls if name == "ik")
    assert params[0][0] == pytest.approx(90)
    assert params[1][3:] == [1.0, 0.0, 0.0, 0.0] and params[2] == 0
    np.testing.assert_array_equal(adapter.forward(q).quaternion_xyzw, [0, 0, 0, 1])
    adapter.algo.rm_algo_inverse_kinematics = lambda params: (1, [0] * 6)
    with pytest.raises(IKRejected):
        adapter.solve(goal(), np.zeros(6))
    with pytest.raises(RuntimeError, match="process-global"):
        OfficialAlgo([0, 0, 0], [0, 0, 0], sdk=sdk)
    monkeypatch.setattr(OfficialAlgo, "_context_claimed", False)


def test_shadow_executable_has_only_diagnostic_publisher_and_no_service_clients():
    source = Path(__file__).resolve().parents[1] / "rm65_servo_teleop/official_follow_node.py"
    tree = ast.parse(source.read_text(encoding="utf-8"))
    calls = [n for n in ast.walk(tree) if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)]
    assert not any(call.func.attr == "create_client" for call in calls)
    publishers = [call for call in calls if call.func.attr == "create_publisher"]
    assert len(publishers) == 1
    assert publishers[0].args[1].value == "/rm65/official_follow/shadow"
