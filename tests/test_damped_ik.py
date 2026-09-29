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
import xrobotoolkit_teleop.hardware.realman_rm65_safe_teleop_controller as controller_module

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
    clock = kwargs.pop("monotonic_fn", lambda: 1.0)
    frame_fk = ControllerFrameKinematics(fk, kwargs.get("dls_work_from_base"), kwargs.get("dls_flange_to_tcp"))
    xyz, quat = frame_fk.forward(q)
    fb = replace(_feedback(), joint_rad=q.copy(), tcp_xyz_m=xyz, tcp_quat_wxyz=quat)
    arm, receiver = FakeArm(), _FeedbackReceiver(fb)
    c = RealmanRM65SafeTeleopController(
        arm=arm, xr=_Xr(), feedback_receiver=receiver, dls_kinematics=fk, monotonic_fn=clock, **kwargs
    )
    c.limiter.reset(xyz, quat)
    return c, arm, receiver


def send_step(c, receiver, index=0, delta=0.002):
    q = receiver.feedback.joint_rad.copy()
    q[index] += delta
    xyz, quat = c.dls.forward(q)
    return c._send_motion(xyz, quat, receiver.feedback.joint_rad)


def advance_cycle(c, receiver, seconds=None):
    """Advance a test clock by one control period and keep the UDP packet fresh.

    The fake arm's measured joints stay put, but its feedback packet must keep arriving or
    the 0.20 s freshness gate - not the lag governor - would be the thing under test.
    """
    seconds = c.dt if seconds is None else seconds
    c._monotonic.advance(seconds)
    receiver.feedback = replace(receiver.feedback, received_monotonic_s=c._monotonic.now)


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


def test_dls_receives_the_nominal_joint_speed_budget_with_no_extra_shrink(monkeypatch):
    """Speed scale 1: the solver gets the model limit x ratio, not a dt/dwell haircut.

    The 2026-09-26 morning version additionally clamped the budget to
    ``_joint_step_budget_rad / tracking_lag_hold_dwell_s``, a constant 1/15 factor that made
    J1/J2 crawl at 1.2 deg/s. How far the command may lead the measured joints is a separate
    budget (the lag governor) and must not be folded into the solver speed limit.
    """
    c, arm, rx = make_dls()
    nominal = RM65_B_SPEC.max_joint_velocity_rad_s * c.max_joint_speed_ratio
    np.testing.assert_allclose(c._dls_joint_velocity_limit_rad_s, nominal)
    assert math.degrees(float(nominal[0])) == pytest.approx(18.0)
    assert math.degrees(float(nominal[2])) == pytest.approx(22.5)

    captured = {}
    real_step = c.dls.step

    def spy(*args, **kwargs):
        captured["speed_limits"] = np.asarray(args[4], dtype=float)
        return real_step(*args, **kwargs)

    monkeypatch.setattr(c.dls, "step", spy)
    assert send_step(c, rx)
    np.testing.assert_allclose(captured["speed_limits"], nominal)

    # Guard against reintroducing the removed haircut: it was nominal / 15 here.
    haircut = c._joint_step_budget_rad / c.tracking_lag_hold_dwell_s
    assert np.all(nominal / haircut > 5.0)
    assert math.degrees(float(haircut[0])) == pytest.approx(1.2)


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
    with pytest.raises(DLSBudgetExceeded, match="FK 调用将超过上限") as caught:
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
    with pytest.raises(DLSBudgetExceeded, match="计算预算已耗尽") as caught:
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
    assert "坐标系不一致" in arm.fault_reason
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
    """A dead arm still cannot support an unbounded replay: the allowance holds, then the dwell latches.

    The allowance is the agreed tracking-lag budget (``max_tracking_lag_cycles`` control
    periods), not the one-period command-continuity budget. A command that keeps running
    while the measured joints never move must still stop the machine; the difference is that
    it is now reported as persistent lag instead of as a one-cycle step violation.
    """
    c, arm, rx = make_dls(dry_run=dry_run, monotonic_fn=_LagClock())
    furthest_lead = 0.0
    held_at_least_once = False
    for _ in range(400):
        advance_cycle(c, rx)  # one control period between commands
        accepted = send_step(c, rx, 0, 0.1)
        held_at_least_once = held_at_least_once or c._lag_hold_reason is not None
        if c._dls_seed_joint_rad is not None:
            furthest_lead = max(
                furthest_lead,
                float(np.max(np.abs(c._dls_seed_joint_rad - rx.feedback.joint_rad))),
            )
        # A hold legitimately reports "no command this cycle"; keep going until the dwell latches.
        if arm.fault_latched:
            break
    assert held_at_least_once, "a dead arm must first enter the bounded hold"
    assert arm.fault_latched
    assert "跟随误差持续恶化" in arm.fault_reason
    assert c._last_accepted_joint_rad is None
    # The command stream freezes at the hold line; it never runs away to the hard allowance.
    assert furthest_lead <= float(np.max(c._joint_lag_hold_trigger_rad)) + 1e-9
    if dry_run:
        assert arm.movej_follow_commands == [] and arm.slow_stop_calls == 0


def _run_lag_follower(lag_cycles, *, old_gate, cycles=60):
    """Drive the admission gate with an arm that follows the command ``lag_cycles`` late.

    Returns ``(faulted_on_cycle, max_gap_rad)``. ``old_gate`` restores the pre-change criterion,
    where the measured-feedback comparison reused the one-period command budget.
    """
    controller, arm, receiver = make_dls(dry_run=False, monotonic_fn=_LagClock())
    if old_gate:
        controller._joint_lag_limit_rad = controller._joint_step_budget_rad.copy()
        controller._joint_lag_hold_trigger_rad = controller._joint_step_budget_rad.copy()
        controller._joint_lag_release_rad = controller._joint_step_budget_rad.copy()
    max_gap = 0.0
    for index in range(cycles):
        advance_cycle(controller, receiver)
        send_step(controller, receiver, 0, 0.003)
        commands = arm.movej_follow_commands
        if not commands:
            continue
        delivered = commands[max(0, len(commands) - 1 - lag_cycles)]
        if controller._dls_seed_joint_rad is not None:
            max_gap = max(
                max_gap,
                float(np.max(np.abs(controller._dls_seed_joint_rad - receiver.feedback.joint_rad))),
            )
        xyz, quat = controller.dls.forward(delivered)
        receiver.feedback = replace(
            receiver.feedback,
            joint_rad=delivered.copy(),
            tcp_xyz_m=xyz,
            tcp_quat_wxyz=quat,
            joint_speed_rad_s=np.zeros(6),
        )
        if arm.fault_latched:
            return index, max_gap
    return None, max_gap


def test_the_field_lead_is_inside_the_new_band_and_outside_the_old_one():
    """The recorded field number, 0.408/0.377/0.365 deg on J1/J2, put on both criteria.

    The old criterion compared the lead against the SAME joint's one-period budget, so 0.365 deg
    > 0.360 deg was a hard stop. The new criterion allows three periods, so the whole recorded
    band is admitted. A discontinuous command step is still refused by the adjacent-command
    criterion, which keeps its one-period budget.
    """
    controller, arm, receiver = make_dls(dry_run=True, monotonic_fn=_LagClock())
    budget = controller._joint_step_budget_rad
    measured = receiver.feedback.joint_rad.copy()

    # Exactly the recorded J1/J2 leads.
    for recorded_deg in (0.365, 0.377, 0.408, 0.465):
        lead = np.full(6, np.deg2rad(recorded_deg))
        assert float(np.max(lead)) > float(budget[0]) + 1e-12, "the old criterion refused these"
        assert float(np.max(lead)) < float(controller._joint_lag_limit_rad[0]), "the new one allows them"

    # And the governor must actually leave normal motion available at the recorded lead.
    controller._dls_seed_joint_rad = measured + np.deg2rad(0.377)
    controller._last_accepted_joint_rad = controller._dls_seed_joint_rad.copy()
    scale, reason, holding = controller._govern_tracking_lag(measured, controller._monotonic())
    assert not holding and reason is None
    assert 0.0 < scale < 1.0, "the recorded lead must cost speed, not motion"
    assert not arm.fault_latched


def test_a_lagging_arm_keeps_receiving_commands_and_stays_inside_the_allowance():
    """End-to-end offline run with a lagging fake arm: following stays bounded and never latches."""
    fault_cycle, max_gap = _run_lag_follower(2, old_gate=False, cycles=150)

    assert fault_cycle is None, "a two-period-late servo must not be treated as a runaway"
    assert max_gap <= float(np.max(make_dls()[0]._joint_lag_limit_rad))


class _LagClock:
    """Monotonic clock that only moves when a test says so.

    The DLS wall-clock budget must never trip in these tests, while the lag dwell timer still
    needs real progress. Tests advance it explicitly by one control period per cycle; the
    per-call reads must not move it or the 0.20 s UDP freshness gate would fire instead.
    """

    def __init__(self):
        self.now = 1.0

    def advance(self, seconds: float) -> None:
        self.now += seconds

    def __call__(self) -> float:
        return self.now


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
    """B 回位与遥操作共用同一个发送门槛；门槛本身在 2026-09-26 变成方向级。

    ``danger`` 把 |J5| 放到 `0.1°`：那正是腕部奇异。旧门槛（配置级 ratio 硬停）会冻结
    **每一个**方向，所以回位命令被拒；新门槛只拒「按这个方向走不了」的命令，而回位这一段
    是纯 TCP 平移（WristFK 里平移只由 J1–J3 产生），方向本身完全可行，因此必须放行。
    真正被拒的情形由 `test_unwalkable_return_direction_is_refused_without_faulting` 覆盖。
    """
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
    assert len(arm.movej_follow_commands) == 1
    assert not c._singularity_hold
    assert arm.movep_follow_commands == []


def test_unwalkable_return_direction_is_refused_without_faulting(monkeypatch):
    """方向不可行时回位仍被拒（HOLD，不锁存），门槛对回位与遥操作一视同仁。"""
    from xrobotoolkit_teleop.utils import geometry
    from xrobotoolkit_teleop.hardware.cartesian_command_limiter import _quaternion_conjugate

    monkeypatch.setattr(geometry.tf, "quaternion_inverse", _quaternion_conjugate, raising=False)
    monkeypatch.setattr(geometry.tf, "quaternion_multiply", _quaternion_multiply, raising=False)
    # RankOneFK 的可达方向是 +x；把 Home 放到 +y 上，这一段回位方向就不可行。
    c, arm, rx = make_dls(Q, dls_kinematics=RankOneFK())
    c.home_xyz = rx.feedback.tcp_xyz_m + np.array([0.0, 0.01, 0.0])
    c.home_quat = rx.feedback.tcp_quat_wxyz.copy()
    c._start_reset(rx.feedback)
    c._reset_step = c._reset_steps_total // 2
    c._tick_reset(1.0, rx.feedback)
    assert not arm.fault_latched
    assert arm.movej_follow_commands == []
    assert c._singularity_hold


@pytest.mark.parametrize("axis,expected", [("+x", 1), ("-y", 0)])
def test_self_test_uses_dls_guard(axis, expected):
    """自检同样走方向级门槛：+x 可行；-y 在 RankOneFK 下不可行，保持且不锁存。"""
    c, arm, rx = make_dls(
        Q,
        dls_kinematics=RankOneFK(),
        self_test_ramp=True,
        self_test_axis=axis,
    )
    c._tick_self_test(1.0)
    c._tick_self_test(1.1)
    assert not arm.fault_latched
    assert len(arm.movej_follow_commands) == expected
    assert c._singularity_hold == (expected == 0)
    assert arm.movep_follow_commands == []


def test_rearm_discards_old_target_and_regrip_anchors_before_motion():
    c, arm, rx = make_dls()
    assert send_step(c, rx)
    c._latch_safety_fault("test fault")
    c._handle_rearm(grip=1.0, now=1.0)
    c._handle_rearm(grip=0.0, now=1.0)
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
    assert arm.fault_latched and "未被控制器确认" in arm.fault_reason
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


def _previous_path_verdict(solver, q, target, ratio):
    """``bounded_path_allowed`` exactly as it stood before 2026-09-26.

    Kept in the test file on purpose: the new guard is one-sided, and the differential check
    below proves it never refuses a step the previous revision allowed.
    """
    stop_joint_rad = solver.stop_joint_rad
    q3, q5 = float(q[2]), float(q[4])
    if q3 * float(target[2]) < 0.0 or q5 * float(target[4]) < 0.0:
        return False
    angles = np.abs(q[[2, 4]])
    next_angles = np.abs(np.asarray(target, dtype=float)[[2, 4]])
    hard = angles <= stop_joint_rad
    if ratio <= solver.stop_ratio and not np.any(hard):
        return False
    if np.any(hard):
        if np.any(next_angles[hard] < angles[hard] - 1e-10):
            return False
        if not np.any(next_angles[hard] > angles[hard] + 1e-10):
            return False
    elif np.any(next_angles < stop_joint_rad):
        return False
    return True


def test_path_guard_never_refuses_what_the_previous_revision_allowed():
    """差分检查：新守卫是旧守卫的单侧放宽，旧版放行的样本新版必须全部放行。

    「旧拒新放」的样本数必须大于零（否则这条改动没有生效），且必须恰好来自两处有意的
    放宽：硬区内的切向运动、以及配置级 ratio 硬停区里仍然可行的方向。
    """
    solver = DampedIK(WristFK())
    relaxed = 0
    for j3_deg in (60.0, 20.0, 5.5, 5.0, 4.9, 3.0, 0.5):
        for j5_deg in (60.0, 20.0, 5.5, 5.0, 4.9, 3.0, 0.5):
            q = Q.copy()
            q[2] = np.deg2rad(j3_deg)
            q[4] = np.deg2rad(j5_deg)
            ratio = solver.metrics(q)[0]
            angles = np.abs(q[[2, 4]])
            for index in range(6):
                for sign in (1.0, -1.0):
                    target = q.copy()
                    target[index] += sign * np.deg2rad(0.5)
                    old = _previous_path_verdict(solver, q, target, ratio)
                    new = solver.bounded_path_allowed(q, target, angles=angles)
                    assert not (old and not new), (j3_deg, j5_deg, index, sign)
                    relaxed += int(new and not old)

    # 配置级 ratio 已经进硬停线、但 J3/J5 都不在 5° 硬区：旧版冻结所有方向。
    rank_one = DampedIK(RankOneFK())
    q = Q.copy()
    ratio = rank_one.metrics(q)[0]
    assert ratio <= rank_one.stop_ratio and not np.any(np.abs(q[[2, 4]]) <= rank_one.stop_joint_rad)
    target = q.copy()
    target[0] += np.deg2rad(0.5)
    assert not _previous_path_verdict(rank_one, q, target, ratio)
    assert rank_one.bounded_path_allowed(q, target, angles=np.abs(q[[2, 4]]))
    relaxed += 1

    assert relaxed > 0, "the deliberate relaxations must admit steps the previous revision refused"


def test_hard_zone_guard_is_one_sided():
    """硬区内只禁止「更深」：切向与向外放行，向内（更接近奇异）拒绝。"""
    solver = DampedIK(WristFK())
    q = Q.copy()
    q[4] = np.deg2rad(3.0)
    angles = np.abs(q[[2, 4]])
    tangential = q.copy()
    tangential[0] += np.deg2rad(1.0)
    deeper = q.copy()
    deeper[4] -= np.deg2rad(0.5)
    escape = q.copy()
    escape[4] += np.deg2rad(0.5)
    assert solver.bounded_path_allowed(q, tangential, angles=angles)
    assert solver.bounded_path_allowed(q, escape, angles=angles)
    assert not solver.bounded_path_allowed(q, deeper, angles=angles)
    # 从硬区外进入硬区仍然拒绝：这才是不让位形继续深入。
    outside = Q.copy()
    outside[4] = np.deg2rad(6.0)
    entering = outside.copy()
    entering[4] -= np.deg2rad(2.0)
    assert not solver.bounded_path_allowed(outside, entering, angles=np.abs(outside[[2, 4]]))
    # 穿零位一律拒绝（唯一与方向无关的硬规则）。
    crossing = Q.copy()
    crossing[4] = np.deg2rad(-1.0)
    assert not solver.bounded_path_allowed(Q, crossing, angles=np.abs(Q[[2, 4]]))


def test_step_preserves_the_commanded_direction_instead_of_rotating_it():
    """步长只许被缩短，不许被改写方向。

    在同一雅可比上重放被删掉的阻尼权重 ``sigma/(sigma^2+lambda^2)``：它把可达的末端运动拧到
    别处；现行的最小范数步与「指令方向在可达子空间上的正交投影」严格同向。
    """
    solver = DampedIK(WristFK(), min_speed_scale=1.0, escape_speed_scale=1.0)
    q = Q.copy()
    q[2] = np.deg2rad(45.0)
    q[4] = np.deg2rad(5.5)  # 刚出 5° 硬区：严重度 ≈ 0.95，λ ≈ 0.076（现场手感区）
    origin, orientation = solver.forward(q)
    jac = solver.jacobian(q)
    u, s, vt = np.linalg.svd(jac, full_matrices=False)
    # 请求方向 = 最难走的方向与最好走的方向各一半（内部归一化度量下）。
    requested = 0.5 * u[:, -1] + 0.5 * u[:, 0]
    requested = requested / np.linalg.norm(requested)
    step_size = 0.002
    target_xyz = origin + requested[:3] * solver.length * step_size
    target_quat = _quaternion_multiply(_angle_axis_quaternion(requested[3:] * step_size), orientation)
    # 只测方向：关掉速度下限与加速度爬坡，让步长只受每关节预算约束（量化误差 ≤0.3%）。
    step = solver.step(
        target_xyz,
        target_quat,
        q,
        0.02,
        SPEEDS,
        max_joint_acceleration=100.0,
    )
    assert not step.hold

    # 用求解器自己的度量复算「实际请求方向」，不依赖上式的小角度近似。
    request = np.concatenate(
        (
            (target_xyz - origin) / solver.length,
            shortest_world_rotation_vector(orientation, target_quat),
        )
    )
    projection = u @ (u.T @ request)
    achieved = np.concatenate(
        (
            (step.xyz - origin) / solver.length,
            shortest_world_rotation_vector(orientation, step.quat),
        )
    )

    def angle_deg(left, right):
        cosine = left @ right / (np.linalg.norm(left) * np.linalg.norm(right))
        return float(np.rad2deg(np.arccos(np.clip(cosine, -1.0, 1.0))))

    assert angle_deg(achieved, projection) < 2.0
    damping = max(0.001, solver.max_damping * solver.severity(s[-1] / s[0], np.abs(q[[2, 4]])))
    old_achieved = jac @ (vt.T @ ((s / (s * s + damping * damping)) * (u.T @ request)))
    assert angle_deg(old_achieved, projection) > 5.0


def test_the_first_step_of_a_segment_keeps_the_commanded_direction():
    """段首（上一速度为零）的加速度限幅也必须是公共系数。

    按关节各自截断时，想要快走的关节先被削平，第一步的末端运动可以被整个转向：候选以
    「可达误差反而变大」被拒 → 命令流被清除 → 同一拒绝逐周期重复，机械臂在那个方向上
    永远不动（现场「前后不动」的可复现机制之一）。
    """
    solver = DampedIK(WristFK(), min_speed_scale=1.0, escape_speed_scale=1.0)
    q = Q.copy()
    q[2] = np.deg2rad(45.0)
    q[4] = np.deg2rad(5.5)
    origin, orientation = solver.forward(q)
    jac = solver.jacobian(q)
    u, s, vt = np.linalg.svd(jac, full_matrices=False)
    requested = 0.5 * u[:, -1] + 0.5 * u[:, 0]
    requested = requested / np.linalg.norm(requested)
    target_xyz = origin + requested[:3] * solver.length * 0.002
    target_quat = _quaternion_multiply(_angle_axis_quaternion(requested[3:] * 0.002), orientation)
    # previous_velocity 缺省即「上一速度为零」的段首。加速度上限放大到 5 rad/s²，让段首步长
    # 远大于 0.001° 下发分辨率，测出来的角度才不是量化噪声。
    acceleration = 5.0
    step = solver.step(target_xyz, target_quat, q, 0.02, SPEEDS, max_joint_acceleration=acceleration)
    assert not step.hold

    request = np.concatenate(
        (
            (target_xyz - origin) / solver.length,
            shortest_world_rotation_vector(orientation, target_quat),
        )
    )
    projection = u @ (u.T @ request)
    achieved = np.concatenate(
        (
            (step.xyz - origin) / solver.length,
            shortest_world_rotation_vector(orientation, step.quat),
        )
    )

    def angle_deg(left, right):
        cosine = left @ right / (np.linalg.norm(left) * np.linalg.norm(right))
        return float(np.rad2deg(np.arccos(np.clip(cosine, -1.0, 1.0))))

    assert angle_deg(achieved, projection) < 2.0
    assert float(achieved @ projection) > 0.0

    # The removed per-joint acceleration clip, replayed on the same request.
    error = request.copy()
    error *= min(
        1.0,
        0.05 * 0.02 / max(np.linalg.norm(error[:3]), 1e-15),
        0.25 * 0.02 / max(np.linalg.norm(error[3:]), 1e-15),
    )
    error[:3] /= solver.length
    raw = vt.T @ ((1.0 / np.maximum(s, s[0] * 1e-9)) * (u.T @ error))
    allowed_velocity = acceleration * 0.02
    old_velocity = np.clip(raw / 0.02, -allowed_velocity, allowed_velocity)
    old_achieved = jac @ (old_velocity * 0.02)
    assert angle_deg(old_achieved, projection) > 10.0


def test_rank_deficient_config_walks_the_reachable_direction_and_holds_the_others():
    """不能按原方向走就停；能走的方向必须照走（旧版连唯一能走的方向也一起冻结）。"""
    solver = DampedIK(RankOneFK())
    q = Q.copy()
    origin, quat = solver.forward(q)
    reachable = solver.step(origin + np.array([0.004, 0.0, 0.0]), quat, q, 0.02, SPEEDS)
    assert not reachable.hold and reachable.direction_ratio > 0.99
    assert np.any(np.abs(reachable.joint_rad - q) > 1e-9)
    assert np.all(np.abs(reachable.joint_rad - q) <= SPEEDS * 0.02 + 1e-9)

    blocked = solver.step(origin + np.array([0.0, 0.004, 0.0]), quat, q, 0.02, SPEEDS)
    assert blocked.hold and blocked.direction_blocked
    assert "手柄方向不可行" in blocked.detail
    np.testing.assert_allclose(blocked.joint_rad, q)


class BoundaryFK:
    """边界（全伸）奇异的玩具模型：x 只通过 ``|J3|`` 的二次项产生。

    在 ``J3 ≈ 0`` 处沿臂轴的第一阶运动不可达（这就是硬停线要拦的方向），而增大 ``|J3|``
    是收臂——退出方向。用来验证「退出优先」：方向增益已判为不可行时，向外步仍要放行。
    """

    def forward(self, q):
        return (
            np.array([0.5 - 0.1 * float(q[2]) ** 2, 0.1 * float(q[0]), 0.1 * float(q[1])]),
            np.array([1.0, 0.0, 0.0, 0.0]),
        )


def test_escape_is_admitted_even_when_the_direction_gain_is_inside_the_stop_line():
    """离开奇异优先：方向的等效增益已被判为不可行时，结构性向外步仍然放行。"""
    solver = DampedIK(BoundaryFK())
    q = Q.copy()
    q[2] = np.deg2rad(0.2)  # |J3| 在 5° 硬区内，臂几乎全伸
    origin, quat = solver.forward(q)
    solver.begin_cycle()
    assert solver.metrics(q)[0] <= solver.stop_ratio, "该位形必须已在硬停线以内"

    deeper = solver.step(origin + np.array([0.002, 0.0, 0.0]), quat, q, 0.02, SPEEDS)
    assert deeper.hold and deeper.direction_blocked
    np.testing.assert_allclose(deeper.joint_rad, q)

    escape = solver.step(origin + np.array([-0.002, 0.0, 0.0]), quat, q, 0.02, SPEEDS)
    assert escape.direction_blocked, "向外与向内的一阶增益相同，只有退出规则能区分它们"
    assert not escape.hold and escape.status == "escaping"
    assert abs(escape.joint_rad[2]) > abs(q[2])
    assert np.all(np.abs(escape.joint_rad - q) <= SPEEDS * 0.02 + 1e-9)


def test_a_direction_blocked_hold_is_reported_and_sends_nothing():
    """方向不可行 = 保持（软停、不锁存、不下发），并且诊断里必须说清是方向问题。"""
    c, arm, rx = make_dls(Q, dls_kinematics=RankOneFK(), dry_run=False, log_motion_debug=True)
    target = rx.feedback.tcp_xyz_m + np.array([0.0, 0.004, 0.0])
    # 保持不算故障：这一周期只是没有命令下发。
    assert c._send_motion(target, rx.feedback.tcp_quat_wxyz, rx.feedback.joint_rad)
    assert not arm.fault_latched
    assert arm.movej_follow_commands == []
    assert c._last_accepted_joint_rad is None
    assert c._singularity_hold
    assert c._dls_debug["direction_blocked"] is True
    assert c._dls_debug["direction_ratio"] < 1e-6
    assert "手柄方向不可行" in c._dls_debug["detail"]


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


def test_fixed_joint_home_preflight_rejects_j5_zero_crossing():
    home = Q.copy()
    home[4] = -Q[4]
    controller, arm, receiver = make_dls(Q, home_joint_deg=tuple(np.rad2deg(home)))
    assert controller.enable_home_path_preflight
    with pytest.raises(ValueError, match="穿越 J3/J5 零位"):
        controller._plan_fixed_joint_reset(receiver.feedback.joint_rad)
    assert arm.movej_follow_commands == []


def test_disabled_home_preflight_returns_through_j5_zero_by_joint_interpolation(monkeypatch):
    """Without the preflight the reset must cross J3/J5 zero instead of holding forever."""
    from xrobotoolkit_teleop.utils import geometry
    from xrobotoolkit_teleop.hardware.cartesian_command_limiter import _quaternion_conjugate

    monkeypatch.setattr(geometry.tf, "quaternion_inverse", _quaternion_conjugate, raising=False)
    monkeypatch.setattr(geometry.tf, "quaternion_multiply", _quaternion_multiply, raising=False)
    home = Q.copy()
    home[4] = -2.0 * Q[4]
    controller, arm, receiver = make_dls(
        Q,
        home_joint_deg=tuple(np.rad2deg(home)),
        enable_home_path_preflight=False,
    )
    controller.home_xyz, controller.home_quat = controller.dls.kinematics.forward(home)
    controller._start_reset(receiver.feedback)
    controller._tick_reset(1.0, receiver.feedback)  # stationary: plans then sends waypoint 0
    assert not controller._reset_waiting_for_stop
    assert not arm.fault_latched
    assert len(arm.movej_follow_commands) == 1

    # Past the zero crossing the commanded J5 must already be on the Home side.
    controller._reset_step = int(0.7 * controller._reset_steps_total)
    controller._tick_reset(1.0, receiver.feedback)
    commanded = arm.movej_follow_commands[-1]
    assert commanded[4] < 0.0
    assert controller._last_accepted_joint_rad[4] == commanded[4]
    assert not arm.fault_latched
    assert controller._resetting

    # Reaching Home still ends the reset through the measured-joint tolerance.
    controller._reset_step = controller._reset_steps_total
    joints_at_home = replace(
        receiver.feedback,
        joint_rad=home.copy(),
        tcp_xyz_m=controller.home_xyz,
        tcp_quat_wxyz=controller.home_quat,
    )
    controller._tick_reset(1.0, joints_at_home)
    assert not controller._resetting
    assert not arm.fault_latched


def test_disabled_home_preflight_full_reset_stays_within_joint_speed_and_finishes(monkeypatch):
    """Drive a whole reset at 50 Hz with a one-cycle-lag arm: bounded speed, then Home."""
    from xrobotoolkit_teleop.utils import geometry
    from xrobotoolkit_teleop.hardware.cartesian_command_limiter import _quaternion_conjugate

    monkeypatch.setattr(geometry.tf, "quaternion_inverse", _quaternion_conjugate, raising=False)
    monkeypatch.setattr(geometry.tf, "quaternion_multiply", _quaternion_multiply, raising=False)
    home = Q.copy()
    home[4] = -2.0 * Q[4]
    controller, arm, receiver = make_dls(
        Q,
        home_joint_deg=tuple(np.rad2deg(home)),
        enable_home_path_preflight=False,
    )
    controller.home_xyz, controller.home_quat = controller.dls.kinematics.forward(home)
    feedback = receiver.feedback
    controller._start_reset(feedback)
    dt = controller.dt
    joint_speed = controller.model_spec.max_joint_velocity_rad_s * controller.max_joint_speed_ratio
    delivered = 0
    previous_command = Q.copy()
    max_ratio = 0.0
    now = 0.0
    for _ in range(4000):
        if not controller._resetting:
            break
        now += dt
        controller._tick_reset(now, feedback)
        if arm.fault_latched:
            break
        if len(arm.movej_follow_commands) > delivered:
            command = arm.movej_follow_commands[-1]
            delivered = len(arm.movej_follow_commands)
            ratio = np.abs(command - previous_command) / (joint_speed * dt)
            max_ratio = max(max_ratio, float(np.max(ratio)))
            previous_command = command
        # The arm follows one cycle late, which is what the completion check tolerates.
        xyz, quat = controller.dls.kinematics.forward(previous_command)
        feedback = replace(
            feedback,
            joint_rad=previous_command.copy(),
            joint_speed_rad_s=np.zeros(6),
            tcp_xyz_m=xyz,
            tcp_quat_wxyz=quat,
        )
    assert not arm.fault_latched
    assert not controller._resetting
    assert delivered > 100
    assert max_ratio <= 1.0 + 1e-9
    np.testing.assert_allclose(previous_command, home, atol=controller.home_joint_tolerance_rad)


def test_disabled_home_preflight_still_enforces_joint_limits():
    home = Q.copy()
    home[4] = -2.0 * Q[4]
    controller, arm, receiver = make_dls(
        Q,
        home_joint_deg=tuple(np.rad2deg(home)),
        enable_home_path_preflight=False,
    )
    broken = receiver.feedback.joint_rad.copy()
    broken[2] = controller.model_spec.joint_limit_rad[2, 1] + 0.5
    assert controller._send_home_joint_waypoint(broken, *controller.dls.kinematics.forward(home)) is False
    assert arm.fault_latched
    assert arm.movej_follow_commands == []


def test_fixed_joint_reset_uses_joint_waypoint_and_waits_for_joint_feedback(monkeypatch):
    from xrobotoolkit_teleop.utils import geometry
    from xrobotoolkit_teleop.hardware.cartesian_command_limiter import _quaternion_conjugate

    monkeypatch.setattr(geometry.tf, "quaternion_inverse", _quaternion_conjugate, raising=False)
    monkeypatch.setattr(geometry.tf, "quaternion_multiply", _quaternion_multiply, raising=False)
    home = Q + np.array([0.05, 0, 0, 0, 0, 0])
    controller, arm, receiver = make_dls(Q, home_joint_deg=tuple(np.rad2deg(home)))
    controller.home_xyz, controller.home_quat = controller.dls.kinematics.forward(home)
    controller._start_reset(receiver.feedback)
    assert controller._resetting and controller._reset_waiting_for_stop
    assert not arm.fault_latched

    controller._diagnostic_row = {}
    controller._send_motion = lambda *args, **kwargs: True
    controller._tick_reset(1.0, receiver.feedback)
    assert not controller._reset_waiting_for_stop
    assert controller._reset_duration_actual_s >= controller.reset_duration_s
    controller._reset_step = controller._reset_steps_total // 2
    controller._tick_reset(1.0, receiver.feedback)
    expected_xyz, _ = controller.dls.kinematics.forward((Q + home) / 2)
    np.testing.assert_allclose(controller._diagnostic_row["target_xyz"], expected_xyz, atol=1e-6)

    controller._reset_step = controller._reset_steps_total
    tcp_at_home = replace(receiver.feedback, tcp_xyz_m=controller.home_xyz, tcp_quat_wxyz=controller.home_quat)
    controller._tick_reset(1.0, tcp_at_home)
    assert controller._resetting  # Matching TCP alone is insufficient.
    joints_at_home = replace(tcp_at_home, joint_rad=home)
    controller._tick_reset(1.0, joints_at_home)
    assert not controller._resetting
    assert controller._rearm_required


def test_auto_home_starts_without_operator_confirmation(monkeypatch):
    """Default startup auto-home must not wait for a typed phrase."""
    controller, arm, receiver = make_dls(
        Q,
        home_joint_deg=(0, 0, -60, 0, 60, 0),
        auto_home_on_start=True,
    )
    controller._startup = lambda: None
    controller._checked_feedback = lambda *args, **kwargs: receiver.feedback
    controller.home_start_delay_s = 0.0
    started = []
    prompts = []

    def start(feedback):
        started.append(feedback)
        controller._resetting = True

    controller._start_reset = start
    controller._run_cycle = lambda now: (_ for _ in ()).throw(KeyboardInterrupt())
    # Any read of stdin would return a phrase the old gate rejects, so a silent default is
    # proven by the reset still starting with this stub installed.
    monkeypatch.setattr("builtins.input", lambda prompt: prompts.append(prompt) or "no")
    controller.run()
    assert prompts == []
    assert started == [receiver.feedback]
    assert controller._startup_homing
    assert not arm.fault_latched


def test_auto_home_grace_window_waits_before_the_trip():
    controller, arm, receiver = make_dls(
        Q,
        home_joint_deg=(0, 0, -60, 0, 60, 0),
        auto_home_on_start=True,
    )
    controller._startup = lambda: None
    controller.home_start_delay_s = 2.5
    events = []
    feedback_reads = []

    def feedback_once(*args, **kwargs):
        # The trip must be planned from feedback read *after* the window, not from the packet
        # the deviation check used before it: the arm may not still be where it was.
        feedback_reads.append(len(events))
        return receiver.feedback

    controller._checked_feedback = feedback_once
    controller._sleep = lambda seconds: events.append(("sleep", seconds))

    def start(feedback):
        events.append(("reset", feedback))
        controller._resetting = True

    controller._start_reset = start
    controller._run_cycle = lambda now: (_ for _ in ()).throw(KeyboardInterrupt())
    controller.run()
    assert events == [
        ("sleep", 1.0),
        ("sleep", 1.0),
        ("sleep", 0.5),
        ("reset", receiver.feedback),
    ]
    assert feedback_reads == [0, 3]
    assert controller._startup_homing


def test_auto_home_countdown_labels_count_down(capsys):
    """Fractional windows must still read 3-2-1: no repeated label and no '0s' start."""
    controller, arm, receiver = make_dls(
        Q,
        home_joint_deg=(0, 0, -60, 0, 60, 0),
        auto_home_on_start=True,
    )
    controller._startup = lambda: None
    controller._checked_feedback = lambda *args, **kwargs: receiver.feedback
    controller.home_start_delay_s = 2.5
    controller._sleep = lambda seconds: None
    controller._start_reset = lambda feedback: setattr(controller, "_resetting", True)
    controller._run_cycle = lambda now: (_ for _ in ()).throw(KeyboardInterrupt())
    controller.run()
    labels = [
        line.removeprefix("[HOME] ").split("s 后")[0]
        for line in capsys.readouterr().out.splitlines()
        if "后开始启动回位" in line
    ]
    assert labels == ["3", "2", "1"]


@pytest.mark.parametrize("value", [float("inf"), float("nan"), -1.0])
def test_home_start_delay_must_be_finite_and_non_negative(value):
    with pytest.raises(ValueError):
        make_dls(Q, home_joint_deg=(0, 0, -60, 0, 60, 0), home_start_delay_s=value)


def test_auto_home_ctrl_c_during_the_grace_window_cancels_the_trip():
    controller, arm, receiver = make_dls(
        Q,
        home_joint_deg=(0, 0, -60, 0, 60, 0),
        auto_home_on_start=True,
    )
    controller._startup = lambda: None
    controller._checked_feedback = lambda *args, **kwargs: receiver.feedback
    controller.home_start_delay_s = 3.0
    started = []
    controller._start_reset = lambda feedback: started.append(feedback)
    calls = []

    def interrupt_on_second_window_step(seconds):
        calls.append(seconds)
        if len(calls) == 2:
            raise KeyboardInterrupt()

    controller._sleep = interrupt_on_second_window_step
    controller.run()  # KeyboardInterrupt is handled inside run(), not propagated.
    assert calls == [1.0, 1.0]
    assert started == []
    assert arm.movej_follow_commands == []


def test_auto_home_typed_confirmation_is_opt_in(monkeypatch):
    controller, arm, receiver = make_dls(
        Q,
        home_joint_deg=(0, 0, -60, 0, 60, 0),
        auto_home_on_start=True,
        home_start_confirmation=True,
    )
    controller._startup = lambda: None
    controller._checked_feedback = lambda *args, **kwargs: receiver.feedback
    started = []
    controller._start_reset = lambda feedback: started.append(feedback)
    monkeypatch.setattr("builtins.input", lambda prompt: "no")
    controller.run()
    assert started == []
    assert arm.movej_follow_commands == []

    def start(feedback):
        started.append(feedback)
        controller._resetting = True

    controller._start_reset = start
    controller._run_cycle = lambda now: (_ for _ in ()).throw(KeyboardInterrupt())
    monkeypatch.setattr("builtins.input", lambda prompt: "HOME 0 0 -60 0 60 0")
    controller.run()
    assert started == [receiver.feedback]
    assert controller._startup_homing


def test_fixed_joint_reset_waits_for_stationary_fresh_joint_feedback():
    home = Q + np.array([0.05, 0, 0, 0, 0, 0])
    controller, arm, receiver = make_dls(Q, home_joint_deg=tuple(np.rad2deg(home)))
    controller.home_xyz, controller.home_quat = controller.dls.kinematics.forward(home)
    moving = replace(receiver.feedback, joint_speed_rad_s=np.ones(6) * 0.1)
    controller._start_reset(moving)
    controller._tick_reset(1.0, moving)
    assert controller._reset_waiting_for_stop
    assert controller._reset_start_joint_rad is None
    assert arm.movej_follow_commands == []

    stopped_at = Q + np.array([0.005, 0, 0, 0, 0, 0])
    xyz, quat = controller.dls.kinematics.forward(stopped_at)
    stopped = replace(moving, joint_rad=stopped_at, joint_speed_rad_s=np.zeros(6), tcp_xyz_m=xyz, tcp_quat_wxyz=quat)
    controller._send_motion = lambda *args, **kwargs: True
    controller._tick_reset(1.1, stopped)
    assert not controller._reset_waiting_for_stop
    np.testing.assert_allclose(controller._reset_start_joint_rad, stopped_at)


class RankOneFK:
    """Rank-deficient Jacobian away from J3/J5 zero: the ratio-only hard-stop branch."""

    def forward(self, q):
        return np.array([0.3 + 0.1 * q[0], 0.0, 0.5]), np.array([1.0, 0.0, 0.0, 0.0])


def _preview(controller, receiver, monkeypatch, delta=np.array([0.004, 0.0, 0.0])):
    """Drive one cycle through the same entry the operator loop uses, without real XR data."""
    controller.ref_arm_xyz = receiver.feedback.tcp_xyz_m.copy()
    controller.ref_arm_quat = receiver.feedback.tcp_quat_wxyz.copy()
    controller._debug_xr_delta_m = delta.copy()
    controller._debug_xr_delta_rot_rad = np.zeros(3)
    controller._last_motion_preview_t = -math.inf
    monkeypatch.setattr(controller, "_process_xr_pose", lambda pose: (delta.copy(), np.zeros(3)))
    # The meshcat stub has no quaternion helpers, so compose the target the same way the
    # diagnostics tests do; the DLS step under test is untouched.
    monkeypatch.setattr(controller_module, "apply_delta_pose", lambda xyz, quat, dpos, drot: (xyz + dpos, quat.copy()))
    controller._send_xr_target(1.0, 0.9, receiver.feedback, np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]))


def test_motion_preview_reports_the_computed_joint_target(capsys, monkeypatch):
    controller, arm, receiver = make_dls(dry_run=True, log_motion_debug=True)
    _preview(controller, receiver, monkeypatch)
    out = capsys.readouterr().out
    assert "[指令 DRY-RUN]" in out
    assert "手柄位移mm=[4.0, 0.0, 0.0]" in out and "目标位移mm=" in out and "实测误差mm=" in out
    assert "  DLS=tracking" in out
    assert "关节°=" in out and "步长°=" in out and "超前°=" in out
    assert "-> 已接受(未下发)" in out
    assert arm.movej_follow_commands == []  # dry run never reaches the wire


def test_lag_hold_reports_the_axis_and_both_numbers():
    """The operator needs the axis name and the exact allowance, not a bare "step exceeded".

    The one-period budget for J3 is 0.450 deg, so the agreed three-period allowance is
    1.350 deg and the hold line is two thirds of it.
    """
    controller, arm, receiver = make_dls(dry_run=True, monotonic_fn=_LagClock())
    controller._dls_seed_joint_rad = receiver.feedback.joint_rad + np.deg2rad([0.0, 0.0, 1.0, 0.0, 0.0, 0.0])
    controller._last_accepted_joint_rad = controller._dls_seed_joint_rad.copy()

    accepted = controller._send_motion(
        receiver.feedback.tcp_xyz_m,
        receiver.feedback.tcp_quat_wxyz,
        receiver.feedback.joint_rad,
    )

    assert not accepted
    assert not arm.fault_latched
    assert "J3" in controller._lag_hold_reason
    assert "1.000°" in controller._lag_hold_reason
    assert "1.350°" in controller._lag_hold_reason


def test_a_command_loop_past_the_hold_line_freezes_the_command():
    """A seed far from the measured joints must stop the command stream, not extend it.

    The seed is set 2 deg from measured: well past J1's 0.720 deg hold line and past the
    1.080 deg allowance. The controller must freeze the stream, keep the arm's existing
    command alive, and report the axis with both numbers.
    """
    controller, arm, receiver = make_dls(dry_run=True, monotonic_fn=_LagClock())
    controller._dls_seed_joint_rad = receiver.feedback.joint_rad + np.deg2rad([2.0, 0, 0, 0, 0, 0])
    controller._last_accepted_joint_rad = controller._dls_seed_joint_rad.copy()

    accepted = controller._send_motion(
        receiver.feedback.tcp_xyz_m,
        receiver.feedback.tcp_quat_wxyz,
        receiver.feedback.joint_rad,
    )

    assert not accepted
    assert not arm.fault_latched
    assert "J1" in controller._lag_hold_reason
    assert "2.000°" in controller._lag_hold_reason
    assert "1.080°" in controller._lag_hold_reason
    assert controller._last_accepted_joint_rad is not None, "a lag hold must keep the command stream"


def test_the_two_criteria_use_two_different_budgets():
    """The bug in one line: command continuity is one period, measured lag is the allowance.

    Both are derived from the same joint-speed budget, but only the command-continuity side may
    use one control period. The measured side has to absorb the servo/transport delay.
    """
    controller, _, _ = make_dls()

    assert controller.max_tracking_lag_cycles == 3.0
    np.testing.assert_allclose(
        controller._joint_lag_limit_rad,
        controller._joint_step_budget_rad * 3.0,
    )
    # J1/J2: 180 deg/s x 10% x 20 ms = 0.360 deg per period, so the allowance is 1.080 deg.
    np.testing.assert_allclose(np.rad2deg(controller._joint_step_budget_rad[:2]), [0.36, 0.36], atol=1e-9)
    np.testing.assert_allclose(np.rad2deg(controller._joint_lag_limit_rad[:2]), [1.08, 1.08], atol=1e-9)
    assert np.all(controller._joint_lag_hold_trigger_rad < controller._joint_lag_limit_rad)


def test_command_continuity_keeps_adjacent_commands_inside_one_period():
    """Every accepted command stays within one period of the previous one, even under lag.

    This is the criterion the fix must not touch: the lag allowance only ever decides whether a
    step happens, never how big an accepted step may be.
    """
    controller, arm, receiver = make_dls(dry_run=False, monotonic_fn=_LagClock())
    for _ in range(40):
        advance_cycle(controller, receiver)
        send_step(controller, receiver, 0, 0.002)
    assert not arm.fault_latched
    commands = arm.movej_follow_commands
    assert len(commands) > 10
    previous = receiver.feedback.joint_rad
    worst = 0.0
    for command in commands:
        worst = max(worst, float(np.max(np.abs(command - previous) / controller._joint_step_budget_rad)))
        previous = command
    assert worst <= 1.0 + 1e-9


def test_a_small_lead_slows_the_command_instead_of_holding_or_latching():
    """Ordinary servo lag has to cost speed, not motion.

    J1's one-period budget is 0.360 deg and its hold line is 0.720 deg, so a 0.5 deg lead sits
    in the middle of the slow-down zone: the command must still be accepted (so the arm keeps
    moving), only slower than the full budget.
    """
    controller, arm, receiver = make_dls(dry_run=True, monotonic_fn=_LagClock())
    controller._dls_seed_joint_rad = receiver.feedback.joint_rad + np.deg2rad([0.5, 0, 0, 0, 0, 0])
    controller._last_accepted_joint_rad = controller._dls_seed_joint_rad.copy()
    xyz, quat = controller.dls.forward(controller._dls_seed_joint_rad + np.array([0.002, 0, 0, 0, 0, 0]))

    accepted = controller._send_motion(xyz, quat, receiver.feedback.joint_rad)

    assert accepted
    assert not arm.fault_latched
    assert controller._lag_hold_reason is None
    assert controller._dls_debug == {} or controller._dls_debug is None


def test_a_lead_past_the_hold_line_freezes_the_command_without_latching():
    """The bug that stopped the real arm: servo lag must not latch a fault.

    A lead past the hold line (but inside the allowance) freezes the command stream so the arm
    can close the gap, and reports it; it must not latch the machine on the first cycle.
    """
    controller, arm, receiver = make_dls(dry_run=True, monotonic_fn=_LagClock())
    controller._dls_seed_joint_rad = receiver.feedback.joint_rad + np.deg2rad([0.0, 0.0, 1.0, 0.0, 0.0, 0.0])
    controller._last_accepted_joint_rad = controller._dls_seed_joint_rad.copy()

    accepted = controller._send_motion(
        receiver.feedback.tcp_xyz_m,
        receiver.feedback.tcp_quat_wxyz,
        receiver.feedback.joint_rad,
    )

    assert not accepted
    assert not arm.fault_latched
    assert controller._lag_hold_reason is not None


def test_motion_preview_explains_a_direction_blocked_hold(capsys, monkeypatch):
    """RankOneFK 只能沿 +x 走；请求 +y 时方向本身不可行，必须印出是「方向」的问题。

    同一夹具下的 +x 请求（可行方向）必须被接受：这正是 2026-09-26 修掉的
    「rank-deficient 位形下连唯一能走的方向也一起冻结」。
    """
    controller, arm, receiver = make_dls(
        Q,
        dls_kinematics=RankOneFK(),
        dry_run=True,
        log_motion_debug=True,
    )
    _preview(controller, receiver, monkeypatch, delta=np.array([0.0, 0.004, 0.0]))
    out = capsys.readouterr().out
    assert "  DLS=hold_singularity 奇异比=0.00000" in out
    assert " 方向比=0.00000" in out
    assert "-> 保持(该手柄方向不可行)" in out
    assert arm.movej_follow_commands == []

    controller._last_preview_text = None
    _preview(controller, receiver, monkeypatch, delta=np.array([0.004, 0.0, 0.0]))
    out = capsys.readouterr().out
    assert "  DLS=tracking" in out
    assert "-> 已接受(未下发)" in out


def test_motion_preview_prints_only_what_changed(capsys, monkeypatch):
    """The operator asked for a quiet console: an unchanged cycle must not reprint its block."""
    controller, arm, receiver = make_dls(dry_run=True, log_motion_debug=True)
    controller._dls_debug = {
        "status": "tracking",
        "detail": "",
        "ratio": 0.2,
        "severity": 0.0,
        "scale": 1.0,
        "damping": 0.001,
        "hold": False,
        "seed_rad": np.zeros(6),
        "measured_rad": np.zeros(6),
        "candidate_rad": np.full(6, 0.01),
        "fk_calls": 4,
        "target_rad": np.full(6, 0.01),
        "achieved_xyz": np.zeros(3),
    }
    values = (
        0.9,
        receiver.feedback,
        np.array([0.004, 0.0, 0.0]),
        np.zeros(3),
        receiver.feedback.tcp_xyz_m,
        True,
    )
    controller._print_motion_preview(*values)
    assert capsys.readouterr().out.count("[指令 DRY-RUN]") == 1

    controller._print_motion_preview(*values)
    assert capsys.readouterr().out == ""

    controller._print_motion_preview(values[0], values[1], np.array([0.005, 0.0, 0.0]), *values[3:])
    assert "[指令 DRY-RUN]" in capsys.readouterr().out


def test_motion_preview_stays_silent_without_the_flag(capsys, monkeypatch):
    controller, arm, receiver = make_dls(dry_run=True)
    _preview(controller, receiver, monkeypatch)
    assert "[指令" not in capsys.readouterr().out
