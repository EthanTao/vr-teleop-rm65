"""Offline regression tests for the simulation IK pipeline.

The Placo controller itself needs placo and meshcat, so these tests cover the pure pipeline it
runs: delta filter -> rate-limited task target -> IK -> joint limit/velocity constraint. That is
where the discontinuity and the missing velocity limit lived. Nothing here opens a socket,
connects to the robot or starts a viewer.
"""

from unittest.mock import MagicMock

import numpy as np
import pytest

from xrobotoolkit_teleop.simulation.ik_pipeline import SimIkPipeline, TargetRateLimiter
from xrobotoolkit_teleop.simulation.joint_constraints import JointConstraints
from xrobotoolkit_teleop.simulation.motion_filter import MotionFilter

DT = 0.02
IDENTITY_QUAT = np.array([1.0, 0.0, 0.0, 0.0])
EE = np.array([0.30, 0.0, 0.50])


def make_joint_constraints(
    *,
    enable_velocity_limit=True,
    max_joint_velocity=1.0,
    max_joint_acceleration=None,
):
    robot = MagicMock()
    names = ["j1", "j2", "j3", "j4", "j5", "j6"]
    robot.joint_names.return_value = names
    limits = {name: np.array([-3.0, 3.0]) for name in names}
    robot.get_joint_limits = lambda name: limits[name]
    return JointConstraints(
        placo_robot=robot,
        dt=DT,
        enable_velocity_limit=enable_velocity_limit,
        max_joint_velocity=max_joint_velocity,
        max_joint_acceleration=max_joint_acceleration,
    )


def make_pipeline(*, position_control=False, joint_constraints=None, target_limiter=None):
    return SimIkPipeline(
        dt=DT,
        motion_filter=MotionFilter(
            deadband_m=0.002,
            deadband_rad=0.03,
            smooth_alpha_pos=0.35,
            smooth_alpha_rot=0.3,
        ),
        joint_constraints=joint_constraints or make_joint_constraints(),
        target_limiter=target_limiter or TargetRateLimiter(),
        position_control=position_control,
    )


class DampedSolver:
    """Stand-in for the IK solver: converges the joints onto the task target's x displacement.

    The real solver is not available offline, but any solver that chases the task target turns a
    discontinuous target into a discontinuous joint state. This one makes that visible: joint 0
    tracks the target's x offset at ``scale`` rad per metre, the others stay put.
    """

    def __init__(self, q, gain=0.8, scale=10.0):
        self.q = np.asarray(q, dtype=float).copy()
        self.gain = gain
        self.scale = scale
        self.target_x = 0.0

    def solve(self, target_xyz):
        self.target_x = float(np.asarray(target_xyz, dtype=float)[0] - EE[0]) * self.scale
        self.q[0] += self.gain * (self.target_x - self.q[0])
        return self.q.copy()


# --------------------------------------------------------------------------------------- target


def test_small_continuous_hand_motion_moves_the_target_in_small_steps():
    """The reported symptom: a slow hand should produce a slow, predictable target, not a jump.

    The filter's per-component dead band makes a slow sweep a stair-step, so the target does move
    in variable-size increments. The property that matters is that every increment is inside the
    rate budget and the sequence is non-decreasing - it must not jump to the full requested pose.
    """
    limiter = TargetRateLimiter()
    pipeline = make_pipeline(target_limiter=limiter)
    pipeline.sync(np.zeros(6), EE, IDENTITY_QUAT)
    steps = []
    for index in range(60):
        # 0.2 mm per cycle: about 10 mm/s, a deliberately gentle hand motion.
        delta = np.array([0.0002 * (index + 1), 0.0, 0.0])
        target = pipeline.step(delta, np.zeros(3), EE, IDENTITY_QUAT)
        steps.append(float(np.linalg.norm(target[0] - EE)))
    diffs = np.diff(steps)
    assert np.all(diffs >= -1e-12), "the rate-limited target must not step backwards"
    # The hand moves up to 12 mm over the run; the target must follow it, not stay put.
    assert steps[-1] > 0.005
    # No cycle may move further than the configured per-cycle budget.
    assert np.max(diffs) <= limiter.max_linear_velocity_m_s * DT + 1e-12
    # And no cycle may be far larger than the previous one: the acceleration bound permits at
    # most a 2x step-up, so a bigger ratio would mean a branch jump, not a ramp.
    moving = diffs[diffs > 1e-9]
    assert np.max(moving[1:] / np.maximum(moving[:-1], 1e-9)) <= 3.0


def test_a_sudden_hand_jump_does_not_become_a_target_jump():
    """A big input must be absorbed by the ramp, exactly like the hardware Cartesian limiter."""
    limiter = TargetRateLimiter(max_linear_velocity_m_s=0.25, max_linear_acceleration_m_s2=1.0)
    pipeline = make_pipeline(target_limiter=limiter)
    pipeline.sync(np.zeros(6), EE, IDENTITY_QUAT)
    target, _ = pipeline.step(np.array([0.50, 0.0, 0.0]), np.zeros(3), EE, IDENTITY_QUAT)
    step = float(np.linalg.norm(target - EE))
    assert step <= limiter.max_linear_velocity_m_s * DT + 1e-12
    assert not np.allclose(target, EE + np.array([0.5, 0.0, 0.0]))


def test_target_cannot_exceed_its_velocity_budget_while_chasing():
    limiter = TargetRateLimiter(max_linear_velocity_m_s=0.25, max_linear_acceleration_m_s2=1.0)
    pipeline = make_pipeline(target_limiter=limiter)
    pipeline.sync(np.zeros(6), EE, IDENTITY_QUAT)
    previous = EE.copy()
    for _ in range(200):
        target, _ = pipeline.step(np.array([5.0, 0.0, 0.0]), np.zeros(3), EE, IDENTITY_QUAT)
        assert float(np.linalg.norm(target - previous)) <= limiter.max_linear_velocity_m_s * DT + 1e-12
        previous = target


def test_sync_reanchors_the_ramp_after_a_reset():
    pipeline = make_pipeline()
    pipeline.sync(np.zeros(6), EE, IDENTITY_QUAT)
    pipeline.step(np.array([0.10, 0.0, 0.0]), np.zeros(3), EE, IDENTITY_QUAT)
    moved = EE + np.array([0.05, 0.02, 0.0])
    pipeline.sync(np.zeros(6), moved, IDENTITY_QUAT)
    target, _ = pipeline.step(np.zeros(3), np.zeros(3), moved, IDENTITY_QUAT)
    np.testing.assert_allclose(target, moved, atol=1e-12)


def test_release_stops_producing_targets():
    pipeline = make_pipeline()
    pipeline.sync(np.zeros(6), EE, IDENTITY_QUAT)
    assert pipeline.step(np.zeros(3), np.zeros(3), EE, IDENTITY_QUAT) is not None
    pipeline.release()
    assert pipeline.step(np.zeros(3), np.zeros(3), EE, IDENTITY_QUAT) is None


# --------------------------------------------------------------------------------------- joints


def test_joint_velocity_limit_is_enforced_by_default():
    """The reported gap: the simulation had no frame-to-frame joint constraint at all."""
    constraints = make_joint_constraints()
    assert constraints.enable_velocity_limit
    constraints.sync(np.zeros(6))
    out = constraints.apply(np.full(6, 2.0))
    assert float(np.max(np.abs(out))) <= constraints.max_joint_velocity * DT + 1e-12


def test_joint_velocity_limit_ramps_instead_of_reversing_at_full_speed():
    """A hard reversal must respect the acceleration bound, not swing at full speed in one cycle."""
    constraints = make_joint_constraints(max_joint_velocity=1.0, max_joint_acceleration=10.0)
    q = np.zeros(6)
    constraints.sync(q)
    for _ in range(50):
        q = constraints.apply(q + np.array([10.0, 0, 0, 0, 0, 0]))
    forward = q.copy()
    assert forward[0] > 0.1
    reversed_q = constraints.apply(np.zeros(6))
    step = float(forward[0] - reversed_q[0])
    assert step <= constraints.max_joint_acceleration * DT * DT + 1e-12


def test_joint_limits_still_clip():
    constraints = make_joint_constraints(enable_velocity_limit=False)
    out = constraints.apply(np.full(6, 9.0))
    np.testing.assert_allclose(out, np.full(6, 3.0))


def test_sync_lets_a_new_segment_start_where_the_robot_is():
    """A re-anchor or a finished reset must not be limited against a stale reference."""
    constraints = make_joint_constraints()
    constraints.sync(np.zeros(6))
    constraints.apply(np.zeros(6))
    elsewhere = np.array([0.5, -0.4, 0.3, 0.2, -0.1, 0.4])
    constraints.sync(elsewhere)
    out = constraints.apply(elsewhere)
    np.testing.assert_allclose(out, elsewhere, atol=1e-12)


# ------------------------------------------------------------------------------------ pipeline


def test_pipeline_turns_a_slow_hand_into_a_slow_joint_ramp():
    """End-to-end offline analogue of the reported run: no single-cycle joint jump."""
    constraints = make_joint_constraints(max_joint_velocity=1.0)
    pipeline = make_pipeline(joint_constraints=constraints)
    q = np.zeros(6)
    pipeline.sync(q, EE, IDENTITY_QUAT)
    solver = DampedSolver(q, scale=10.0)
    max_step = 0.0
    previous = q.copy()
    for index in range(120):
        delta = np.array([0.0004 * (index + 1), 0.0, 0.0])
        target, target_quat = pipeline.step(delta, np.zeros(3), EE, IDENTITY_QUAT)
        solved = solver.solve(target)
        q = pipeline.constrain(solved)
        max_step = max(max_step, float(np.max(np.abs(q - previous))))
        previous = q.copy()
    assert max_step <= constraints.max_joint_velocity * DT + 1e-12
    assert q[0] > 0.01, "a continuous hand motion must actually move the joints"


def test_pipeline_joint_step_is_logged_for_continuity_checks():
    pipeline = make_pipeline()
    q = np.zeros(6)
    pipeline.sync(q, EE, IDENTITY_QUAT)
    pipeline.step(np.array([0.001, 0, 0]), np.zeros(3), EE, IDENTITY_QUAT)
    # A solve that is inside the budget passes through untouched.
    assert pipeline.constrain(np.array([0.001, 0, 0, 0, 0, 0]))[0] == pytest.approx(0.001)
    # A solve that is outside it must report the amount it was cut back.
    constrained = pipeline.constrain(np.full(6, 1.0))
    telemetry = pipeline.telemetry
    assert telemetry["joint_clipped"]
    assert telemetry["joint_step_deg"] == pytest.approx(
        float(np.rad2deg(np.max(np.abs(1.0 - constrained)))), rel=1e-6
    )
