"""Offline FK, fake-arm and admission tests; never connects to hardware."""

from dataclasses import replace
import math
import numpy as np
import pytest

from xrobotoolkit_teleop.hardware.damped_ik import DampedIK, ControllerFrameKinematics, DLSBudgetExceeded
from xrobotoolkit_teleop.hardware.cartesian_command_limiter import (
    _angle_axis_quaternion,
    _quaternion_multiply,
    shortest_world_rotation_vector,
    CartesianCommandLimiter,
)
from xrobotoolkit_teleop.hardware.rm65_model import RM65_B_SPEC
from tests.test_realman_safe_controller import _Arm, _Xr, _FeedbackReceiver, _feedback
from xrobotoolkit_teleop.hardware.realman_rm65_safe_teleop_controller import RealmanRM65SafeTeleopController

Q = np.array([0.1, -0.3, 1.0, 0.2, 0.6, -0.2])
SPEEDS = RM65_B_SPEC.max_joint_velocity_rad_s * 0.1


class WristFK:
    """Independent analytical fixture: Cartesian translation plus Z-Y-Z wrist.

    Has a known wrist singularity at q5=0 and no sockets or URDF dependency.
    """

    def forward(self, q):
        xyz = np.array([0.3, 0, 0.5]) + 0.1 * q[:3]
        quat = _quaternion_multiply(
            _quaternion_multiply(
                _angle_axis_quaternion(np.array([0, 0, q[3]])), _angle_axis_quaternion(np.array([0, q[4], 0]))
            ),
            _angle_axis_quaternion(np.array([0, 0, q[5]])),
        )
        return xyz, quat


class FakeArm(_Arm):
    def _pose_is_within_bounds(self, xyz):
        valid = np.shape(xyz) == (3,) and np.all(np.isfinite(xyz)) and np.all(np.abs(xyz) <= 1)
        return bool(valid), "workspace bounds"


def make_dls(q=Q, **kwargs):
    fk = kwargs.pop("dls_kinematics", WristFK())
    frame_fk = ControllerFrameKinematics(fk, kwargs.get("dls_work_from_base"), kwargs.get("dls_flange_to_tcp"))
    xyz, quat = frame_fk.forward(q)
    fb = replace(_feedback(), joint_rad=q.copy(), tcp_xyz_m=xyz, tcp_quat_wxyz=quat)
    arm, receiver = FakeArm(), _FeedbackReceiver(fb)
    c = RealmanRM65SafeTeleopController(
        arm=arm, xr=_Xr(), feedback_receiver=receiver, dls_kinematics=fk, monotonic_fn=lambda: 1.0, **kwargs
    )
    c.limiter.reset(xyz, quat)
    return c, arm, receiver


def send_step(c, receiver, index=0, delta=0.002):
    q = receiver.feedback.joint_rad.copy()
    q[index] += delta
    xyz, quat = c.dls.forward(q)
    return c._send_motion(xyz, quat, receiver.feedback.joint_rad)


def test_dls_is_default_and_actual_wire_target_is_joint_space():
    c, arm, rx = make_dls()
    assert c.motion_command == "damped_ik_movej_follow"
    assert send_step(c, rx)
    assert len(arm.movej_follow_commands) == 1
    assert arm.movep_follow_commands == []
    target = arm.movej_follow_commands[0]
    assert np.any(target != Q)
    assert np.all(np.abs(target - Q) <= SPEEDS * 0.02 + 1e-9)
    xyz, quat = c.dls.forward(target)
    np.testing.assert_allclose(c.limiter.position, xyz)
    np.testing.assert_allclose(c.limiter.quaternion, quat)


@pytest.mark.parametrize("angle", [0.01, 0.1, 0.4, 3.0])
def test_inward_near_zero_is_held_but_outward_can_exit(angle):
    q = Q.copy()
    q[4] = np.deg2rad(angle)
    solver = DampedIK(WristFK())
    inward = q.copy()
    inward[4] -= np.deg2rad(0.001)
    outward = q.copy()
    outward[4] += np.deg2rad(0.2)
    assert solver.step(*solver.forward(inward), q, 0.02, SPEEDS).hold
    step = solver.step(*solver.forward(outward), q, 0.02, SPEEDS)
    assert not step.hold
    assert step.joint_rad[4] > q[4]
    assert np.all(np.abs(step.joint_rad - q) <= SPEEDS * 0.02 + 1e-9)


def test_path_cannot_cross_wrist_zero_to_a_better_endpoint():
    solver = DampedIK(WristFK())
    q = Q.copy()
    q[4] = np.deg2rad(0.1)
    target = q.copy()
    target[4] = np.deg2rad(-0.9)
    assert not solver.path_allowed(q, target)


def test_realtime_step_has_hard_fk_and_candidate_budgets():
    solver = DampedIK(WristFK(), max_fk_calls=13, max_candidate_attempts=3)
    target_q = Q.copy()
    target_q[0] += 0.01
    xyz, quat = solver.forward(target_q)
    with pytest.raises(DLSBudgetExceeded, match="FK budget") as caught:
        solver.step(xyz, quat, Q, 0.02, SPEEDS)
    assert caught.value.fk_calls == 13
    assert solver.candidate_attempts == 1


def test_realtime_step_stops_on_wall_clock_budget():
    class Clock:
        now = 0.0

        def __call__(self):
            return self.now

    class SlowFK(WristFK):
        def __init__(self, clock):
            self.clock = clock

        def forward(self, q):
            self.clock.now += 0.002
            return super().forward(q)

    clock = Clock()
    solver = DampedIK(SlowFK(clock), compute_budget_s=0.018, monotonic_fn=clock)
    with pytest.raises(DLSBudgetExceeded, match="compute budget") as caught:
        solver.step(*WristFK().forward(Q + np.array([0.002, 0, 0, 0, 0, 0])), Q, 0.02, SPEEDS)
    assert caught.value.elapsed_s >= 0.018
    assert caught.value.fk_calls < solver.max_fk_calls


def test_controller_rejects_dls_budget_at_or_above_control_period():
    with pytest.raises(ValueError, match="below the control period"):
        make_dls(dls_compute_budget_s=0.02)


def test_hold_cancels_pending_trajectory_and_waits_for_stationary_feedback():
    q = Q.copy()
    q[4] = np.deg2rad(0.1)
    c, arm, rx = make_dls(q)
    c.limiter.rebase(rx.feedback.tcp_xyz_m, rx.feedback.tcp_quat_wxyz, np.array([0.05, 0, 0]), np.zeros(3))
    assert send_step(c, rx, 4, -0.00001)
    assert c._singularity_hold
    assert arm.slow_stop_calls == 1 and arm.movej_follow_commands == []
    np.testing.assert_array_equal(c.limiter.linear_velocity, np.zeros(3))
    rx.feedback = replace(rx.feedback, joint_speed_rad_s=np.ones(6) * 0.1)
    assert send_step(c, rx, 4, 0.01)
    assert arm.movej_follow_commands == [] and arm.slow_stop_calls == 1
    rx.feedback = replace(rx.feedback, joint_speed_rad_s=np.zeros(6))
    assert send_step(c, rx, 4, 0.01)
    assert len(arm.movej_follow_commands) == 1
    assert not c._singularity_hold


@pytest.mark.parametrize("kind", ["translation", "rotation"])
def test_fk_udp_mismatch_fails_before_any_target(kind):
    c, arm, rx = make_dls()
    if kind == "translation":
        rx.feedback = replace(rx.feedback, tcp_xyz_m=rx.feedback.tcp_xyz_m + np.array([0.02, 0, 0]))
    else:
        rx.feedback = replace(rx.feedback, tcp_quat_wxyz=np.array([1.0, 0, 0, 0]))
    assert not send_step(c, rx)
    assert "frame mismatch" in arm.fault_reason
    assert arm.movej_follow_commands == []


def test_work_rotation_and_tool_offset_are_applied_before_jacobian():
    c, arm, rx = make_dls(
        dls_work_from_base=[0.05, 0.02, 0, math.sqrt(0.5), 0, 0, math.sqrt(0.5)],
        dls_flange_to_tcp=[0.08, 0, 0, 1, 0, 0, 0],
    )
    assert send_step(c, rx, 4, 0.005)
    assert len(arm.movej_follow_commands) == 1
    np.testing.assert_allclose(c.dls.jacobian(Q)[:3, 0], [0, 1 / 3, 0], atol=1e-9)
    # The TCP offset gives the wrist a nonzero linear Jacobian.
    assert np.linalg.norm(c.dls.jacobian(Q)[:3, 4]) > 0.05


@pytest.mark.parametrize("dry_run", [False, True])
def test_feedback_budget_prevents_command_runaway(dry_run):
    c, arm, rx = make_dls(dry_run=dry_run)
    for _ in range(200):
        if not send_step(c, rx, 0, 0.1):
            break
    assert arm.fault_latched
    assert "step exceeded" in arm.fault_reason
    assert c._last_accepted_joint_rad is None
    if dry_run:
        assert arm.movej_follow_commands == [] and arm.slow_stop_calls == 0


def test_failed_send_does_not_commit_target():
    c, arm, rx = make_dls()
    arm.send_movej_follow_joint_rad = lambda *a, **kw: False
    assert not send_step(c, rx)
    assert c._last_accepted_joint_rad is None and arm.fault_latched


def test_timeout_holds_then_stops_and_stale_feedback_prevents_sending():
    c, arm, rx = make_dls()
    xyz, quat = c.dls.forward(Q + np.array([0.002, 0, 0, 0, 0, 0]))
    c.limiter.rebase(xyz, quat, np.array([0.05, 0, 0]), np.zeros(3))
    for expected in (1, 2):
        assert c._send_damped_motion(xyz, quat, 0.9)
        assert c._dls_consecutive_failures == expected
        assert arm.slow_stop_calls == 0
        np.testing.assert_array_equal(c.limiter.position, rx.feedback.tcp_xyz_m)
    assert c._send_damped_motion(xyz, quat, 0.9)
    assert c._dls_consecutive_failures == 3
    assert arm.slow_stop_calls == 1
    assert not arm.fault_latched
    assert arm.movej_follow_commands == []
    c, arm, rx = make_dls()
    rx.feedback = replace(rx.feedback, received_monotonic_s=0.7)
    assert not send_step(c, rx)
    assert arm.movej_follow_commands == []


def test_joint_margin_allows_retreat_but_not_further_approach():
    q = Q.copy()
    q[3] = np.deg2rad(175)
    solver = DampedIK(WristFK())
    plus = q.copy()
    plus[3] += 0.001
    minus = q.copy()
    minus[3] -= 0.001
    assert solver.step(*solver.forward(plus), q, 0.02, SPEEDS).hold
    assert not solver.step(*solver.forward(minus), q, 0.02, SPEEDS).hold


@pytest.mark.parametrize("mode", ["movep_follow", "movel", "official_ik_movej_follow"])
def test_legacy_modes_cannot_claim_dls_protection(mode):
    with pytest.raises(ValueError, match="singularity avoidance requires"):
        make_dls(motion_command=mode)


def test_dry_run_uses_same_checks_without_sending_or_stopping():
    q = Q.copy()
    q[4] = np.deg2rad(0.1)
    c, arm, rx = make_dls(q, dry_run=True)
    assert send_step(c, rx, 4, -0.00001)
    assert c._singularity_hold
    assert arm.slow_stop_calls == 0 and arm.movej_follow_commands == []


def test_fk_exception_never_falls_back_to_cartesian_motion():
    c, arm, rx = make_dls()

    def bad(q):
        raise RuntimeError("FK failed")

    c.dls.kinematics.official.forward = bad
    assert not c._send_motion(rx.feedback.tcp_xyz_m, rx.feedback.tcp_quat_wxyz, Q)
    assert arm.movep_follow_commands == [] and arm.movej_follow_commands == []


def test_reduced_speed_does_not_reduce_braking_acceleration():
    lim = CartesianCommandLimiter(0.05, 0.4, 0.25, 1.2)
    lim.reset(np.zeros(3), [1, 0, 0, 0])
    for _ in range(20):
        lim.step(np.array([1.0, 0, 0]), [1, 0, 0, 0], 0.02)
    start = lim.position.copy()
    lim.set_speed_scale(0.05)
    for _ in range(10):
        lim.step(lim.position, lim.quaternion, 0.02)
    assert (lim.position - start)[0] < 0.004
    assert np.linalg.norm(lim.linear_velocity) < 1e-10


@pytest.mark.parametrize("danger", [False, True])
def test_b_return_uses_dls_guard(monkeypatch, danger):
    from xrobotoolkit_teleop.utils import geometry
    from xrobotoolkit_teleop.hardware.cartesian_command_limiter import _quaternion_conjugate

    monkeypatch.setattr(geometry.tf, "quaternion_inverse", _quaternion_conjugate, raising=False)
    monkeypatch.setattr(geometry.tf, "quaternion_multiply", _quaternion_multiply, raising=False)
    q = Q.copy()
    if danger:
        q[4] = np.deg2rad(0.1)
    c, arm, rx = make_dls(q)
    c.home_xyz = rx.feedback.tcp_xyz_m + np.array([0.01, 0, 0])
    c.home_quat = rx.feedback.tcp_quat_wxyz.copy()
    c._start_reset(rx.feedback)
    c._reset_step = c._reset_steps_total // 2
    c._tick_reset(1.0, rx.feedback)
    assert not arm.fault_latched
    assert len(arm.movej_follow_commands) == (0 if danger else 1)
    assert c._singularity_hold == danger
    assert arm.movep_follow_commands == []


@pytest.mark.parametrize("danger", [False, True])
def test_self_test_uses_dls_guard(danger):
    q = Q.copy()
    if danger:
        q[4] = np.deg2rad(0.1)
    c, arm, rx = make_dls(q, self_test_ramp=True)
    c._tick_self_test(1.0)
    c._tick_self_test(1.1)
    assert not arm.fault_latched
    assert len(arm.movej_follow_commands) == (0 if danger else 1)
    assert c._singularity_hold == danger
    assert arm.movep_follow_commands == []


def test_rearm_discards_old_target_and_regrip_anchors_before_motion():
    c, arm, rx = make_dls()
    assert send_step(c, rx)
    c._latch_safety_fault("test fault")
    c._handle_rearm(grip=0.0, reset_edge=True, now=1.0)
    assert not c._rearm_required and not arm.fault_latched
    assert c._last_accepted_joint_rad is None
    c.xr.grip = 1.0
    c._run_teleop_cycle(1.0)
    assert len(arm.movej_follow_commands) == 1
    assert c._last_accepted_joint_rad is None
    np.testing.assert_array_equal(c.ref_arm_xyz, rx.feedback.tcp_xyz_m)
    # Even after software rearm, a moving new segment must stop before restarting.
    rx.feedback = replace(rx.feedback, joint_speed_rad_s=np.ones(6) * 0.1)
    assert send_step(c, rx)
    assert len(arm.movej_follow_commands) == 1
    assert c._dls_stopping


def test_failed_hold_stop_ack_latches_instead_of_resuming():
    q = Q.copy()
    q[4] = np.deg2rad(0.1)
    c, arm, rx = make_dls(q)
    arm.slow_stop = lambda: False
    assert not send_step(c, rx, 4, -0.00001)
    assert arm.fault_latched and "not acknowledged" in arm.fault_reason
    assert arm.movej_follow_commands == []


def test_failed_reset_stop_ack_prevents_return():
    c, arm, rx = make_dls()
    c.home_xyz, c.home_quat = rx.feedback.tcp_xyz_m, rx.feedback.tcp_quat_wxyz
    arm.slow_stop = lambda: False
    c._start_reset(rx.feedback)
    assert arm.fault_latched and not c._resetting
    assert arm.movej_follow_commands == []


def test_feedback_changes_during_computation_are_rechecked():
    c, arm, rx = make_dls()
    original = c.dls.step

    def step(*args, **kwargs):
        result = original(*args, **kwargs)
        rx.feedback = replace(rx.feedback, received_monotonic_s=0.5)
        return result

    c.dls.step = step
    assert not send_step(c, rx)
    assert arm.fault_latched and arm.movej_follow_commands == []


def test_numeric_jacobian_matches_independent_zyz_derivative():
    solver = DampedIK(WristFK())
    jac = solver.jacobian(Q)
    expected = np.zeros((6, 6))
    expected[:3, :3] = np.eye(3) / 3
    expected[3:, 3] = [0, 0, 1]
    expected[3:, 4] = [-np.sin(Q[3]), np.cos(Q[3]), 0]
    expected[3:, 5] = [np.cos(Q[3]) * np.sin(Q[4]), np.sin(Q[3]) * np.sin(Q[4]), np.cos(Q[4])]
    np.testing.assert_allclose(jac, expected, atol=1e-9)


def test_outward_motion_from_negative_wrist_side_does_not_cross_zero():
    q = Q.copy()
    q[4] = np.deg2rad(-0.1)
    solver = DampedIK(WristFK())
    inward, outward = q.copy(), q.copy()
    inward[4] += np.deg2rad(0.01)
    outward[4] -= np.deg2rad(0.2)
    assert solver.step(*solver.forward(inward), q, 0.02, SPEEDS).hold
    result = solver.step(*solver.forward(outward), q, 0.02, SPEEDS)
    assert not result.hold and result.joint_rad[4] < q[4]


@pytest.mark.parametrize("explicit_tool", [False, True])
def test_dls_configuration_only_queries_controller_and_preserves_explicit_frames(explicit_tool):
    from unittest.mock import Mock
    from tests.test_realman_official_ik import _controller_dh

    work = [0.1, 0.2, 0.3, 1, 0, 0, 0]
    tool = [0, 0, 0.07, 1, 0, 0, 0] if explicit_tool else None
    c, arm, rx = make_dls(dls_work_from_base=work, dls_flange_to_tcp=tool)
    c._load_controller_dh = True
    original_work = c.dls.kinematics.work_from_base
    official = Mock()
    official.controller_tool_pose.return_value = [0, 0, 0.06986, 1, 0, 0, 0]
    c.dls.kinematics.official = official
    commands = []

    def query(payload, wait_s):
        commands.append(payload["command"])
        if payload["command"] == "get_DH_data":
            return [_controller_dh()]
        return [dict(state="current_tool_frame", tool_name="finger", pose=[0] * 6)]

    arm.send_json = query
    c._configure_dls_from_controller()
    assert commands == (["get_DH_data"] if explicit_tool else ["get_DH_data", "get_current_tool_frame"])
    official.configure_controller_dh.assert_called_once_with(_controller_dh())
    assert c.dls.kinematics.work_from_base is original_work
    np.testing.assert_allclose(c.dls.kinematics.flange_to_tcp[0], [0, 0, 0.07 if explicit_tool else 0.06986])
    assert arm.movej_follow_commands == []


def test_dls_missing_controller_parameters_stops_initialization():
    c, arm, rx = make_dls()
    c._load_controller_dh = True
    arm.send_json = lambda *a, **kw: []
    with pytest.raises(RuntimeError, match="get_DH_data"):
        c._configure_dls_from_controller()
    assert arm.movej_follow_commands == []
