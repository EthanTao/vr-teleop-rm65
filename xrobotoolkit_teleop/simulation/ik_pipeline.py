"""Rate-limited target generation shared by the simulation IK backends.

The simulation used to feed the IK solver a target that could jump discontinuously: the
motion filter zeroes each component below its dead band (so a slow hand motion is a
stair-step, not a ramp), and the target was then handed to the solver as an absolute pose.
Whatever the solver returned became the next joint state with no velocity limit at all, so a
momentary discontinuity in the target showed up as a large joint jump.

This module holds the pure, testable part of the fix:

* :class:`TargetRateLimiter` moves the solver target continuously toward the requested pose
  under a Cartesian velocity/acceleration budget, so a hand that moves a little produces a
  target that moves a little.
* :class:`SimIkPipeline` wires that limiter, the dead-band/EMA motion filter and the joint
  limit/velocity constraint together in the same order the controller runs them.

No placo or meshcat import here: the pipeline is the part that must be covered by offline tests.
"""

from __future__ import annotations

import numpy as np

from xrobotoolkit_teleop.hardware.cartesian_command_limiter import (
    _angle_axis_quaternion,
    _normalized_quaternion,
    _quaternion_multiply,
    shortest_world_rotation_vector,
)
from xrobotoolkit_teleop.simulation.joint_constraints import JointConstraints
from xrobotoolkit_teleop.simulation.motion_filter import MotionFilter


class TargetRateLimiter:
    """Bounded Cartesian velocity/acceleration ramp for one IK task target.

    ``step`` returns the next point on the ramp, never the requested pose itself. It is reset
    whenever the operator re-anchors or a reset finishes, so the ramp always starts from the
    pose the robot is actually in.
    """

    def __init__(
        self,
        *,
        max_linear_velocity_m_s: float = 0.25,
        max_linear_acceleration_m_s2: float = 1.0,
        max_angular_velocity_rad_s: float = 1.0,
        max_angular_acceleration_rad_s2: float = 4.0,
        fallback_linear_velocity_m_s: float = 0.05,
        fallback_angular_velocity_rad_s: float = 0.25,
    ):
        values = (
            max_linear_velocity_m_s,
            max_linear_acceleration_m_s2,
            max_angular_velocity_rad_s,
            max_angular_acceleration_rad_s2,
            fallback_linear_velocity_m_s,
            fallback_angular_velocity_rad_s,
        )
        if not all(np.isfinite(float(value)) and float(value) > 0.0 for value in values):
            raise ValueError("target rate limits must be finite and positive")
        self.max_linear_velocity_m_s = float(max_linear_velocity_m_s)
        self.max_linear_acceleration_m_s2 = float(max_linear_acceleration_m_s2)
        self.max_angular_velocity_rad_s = float(max_angular_velocity_rad_s)
        self.max_angular_acceleration_rad_s2 = float(max_angular_acceleration_rad_s2)
        self.fallback_linear_velocity_m_s = float(fallback_linear_velocity_m_s)
        self.fallback_angular_velocity_rad_s = float(fallback_angular_velocity_rad_s)
        self.reset(np.zeros(3), np.array([1.0, 0.0, 0.0, 0.0]))

    def reset(self, xyz: np.ndarray, quat_wxyz: np.ndarray) -> None:
        self._position = np.asarray(xyz, dtype=float).copy()
        self._quaternion = np.asarray(_normalized_quaternion(quat_wxyz, "target quaternion"), dtype=float)
        self._linear_velocity = np.zeros(3)
        self._angular_velocity = np.zeros(3)

    @property
    def position(self) -> np.ndarray:
        return self._position.copy()

    @property
    def quaternion(self) -> np.ndarray:
        return np.asarray(self._quaternion, dtype=float)

    def step(self, target_xyz, target_quat_wxyz, dt: float):
        """Advance the ramp toward the requested pose by at most one bounded step."""
        target_xyz = np.asarray(target_xyz, dtype=float)
        if target_xyz.shape != (3,) or not np.all(np.isfinite(target_xyz)):
            raise ValueError("target xyz must contain three finite values")
        target_quat = np.asarray(_normalized_quaternion(target_quat_wxyz, "target quaternion"), dtype=float)
        timestep = float(dt)
        if not np.isfinite(timestep) or timestep <= 0.0:
            raise ValueError("dt must be finite and positive")

        self._linear_velocity = self._bounded_velocity(
            target_xyz - self._position,
            self._linear_velocity,
            self.max_linear_velocity_m_s,
            self.max_linear_acceleration_m_s2,
            timestep,
        )
        self._position = self._position + self._linear_velocity * timestep
        rotation_error = shortest_world_rotation_vector(self._quaternion, target_quat)
        self._angular_velocity = self._bounded_velocity(
            rotation_error,
            self._angular_velocity,
            self.max_angular_velocity_rad_s,
            self.max_angular_acceleration_rad_s2,
            timestep,
        )
        increment = _angle_axis_quaternion(self._angular_velocity * timestep)
        self._quaternion = np.asarray(
            _normalized_quaternion(_quaternion_multiply(increment, self._quaternion), "integrated quaternion"),
            dtype=float,
        )
        return self._position.copy(), self._quaternion.copy()

    def _bounded_velocity(self, error, current_velocity, max_velocity, max_acceleration, timestep):
        """Aim at the requested pose, then bound speed and acceleration.

        The speed cap uses the braking-distance form ``sqrt(2 a e)`` so the ramp decelerates into
        the target instead of overshooting it, and the velocity term keeps a small continuous hand
        motion from being rounded to a standstill. Falls back to a slow constant speed only when
        neither bound is usable (both limits are effectively zero).
        """
        error = np.asarray(error, dtype=float)
        error_norm = float(np.linalg.norm(error))
        if error_norm == 0.0:
            desired = np.zeros(3)
        else:
            speed = float(min(max_velocity, np.sqrt(2.0 * max_acceleration * error_norm)))
            if speed <= 0.0:
                speed = min(self.fallback_linear_velocity_m_s, error_norm / timestep)
            desired = error * (speed / error_norm)
        delta = desired - current_velocity
        delta_norm = float(np.linalg.norm(delta))
        max_delta = max_acceleration * timestep
        if delta_norm > max_delta:
            delta *= max_delta / delta_norm
        return current_velocity + delta


class SimIkPipeline:
    """Delta filter -> rate-limited target -> IK -> joint limit/velocity constraint.

    Exactly the order the simulation loop uses, with no simulator dependency, so the offline
    tests cover the same code the Placo controller runs.
    """

    def __init__(
        self,
        *,
        dt: float,
        motion_filter: MotionFilter,
        joint_constraints: JointConstraints,
        target_limiter: TargetRateLimiter,
        position_control: bool,
    ):
        self.dt = float(dt)
        self.motion_filter = motion_filter
        self.joint_constraints = joint_constraints
        self.target_limiter = target_limiter
        self.position_control = bool(position_control)
        self._active = False
        self._reference_ee_xyz: np.ndarray | None = None
        self._reference_ee_quat: np.ndarray | None = None
        self._latest_joint: np.ndarray | None = None
        self._telemetry: dict = {}

    @property
    def telemetry(self) -> dict:
        """Per-cycle record of the values the jump question is asked about."""
        return dict(self._telemetry)

    @property
    def reference_ee_xyz(self):
        return None if self._reference_ee_xyz is None else self._reference_ee_xyz.copy()

    @property
    def reference_ee_quat(self):
        return None if self._reference_ee_quat is None else self._reference_ee_quat.copy()

    def sync(self, joint, ee_xyz, ee_quat) -> None:
        """Anchor to the pose the robot is in now: new segment, or a finished reset."""
        self.motion_filter.reset()
        self.joint_constraints.sync(joint)
        self._reference_ee_xyz = np.asarray(ee_xyz, dtype=float).copy()
        self._reference_ee_quat = np.asarray(ee_quat, dtype=float).copy()
        self.target_limiter.reset(ee_xyz, ee_quat)

    def release(self) -> None:
        """Grip released: the next activation re-anchors from the current pose."""
        self._active = False
        self._reference_ee_xyz = None
        self._reference_ee_quat = None
        self._telemetry = {}

    def step(self, delta_xyz, delta_rot, ee_xyz, ee_quat):
        """Return the rate-limited task target for this cycle, or ``None`` when inactive."""
        if self._reference_ee_xyz is None:
            return None
        filtered_xyz, filtered_rot = self.motion_filter.apply(delta_xyz, delta_rot)
        if self.position_control:
            requested_xyz = self._reference_ee_xyz + filtered_xyz
            requested_quat = np.asarray(ee_quat, dtype=float)
        else:
            requested_xyz, requested_quat = _apply_delta_pose(
                self._reference_ee_xyz, self._reference_ee_quat, filtered_xyz, filtered_rot
            )
        previous = self.target_limiter.position
        limited_xyz, limited_quat = self.target_limiter.step(requested_xyz, requested_quat, self.dt)
        self._telemetry = {
            "filtered_delta_xyz": np.asarray(filtered_xyz, dtype=float).copy(),
            "filtered_delta_rot": np.asarray(filtered_rot, dtype=float).copy(),
            "requested_xyz": np.asarray(requested_xyz, dtype=float).copy(),
            "target_xyz": np.asarray(limited_xyz, dtype=float).copy(),
            "target_step_m": float(np.linalg.norm(limited_xyz - previous)),
            "joint_step_deg": None,
            "joint_clipped": False,
        }
        return limited_xyz, limited_quat

    def constrain(self, solved_joint):
        """Apply the joint limit and velocity constraint to one solver result."""
        solved = np.asarray(solved_joint, dtype=float)
        constrained = self.joint_constraints.apply(solved)
        self._telemetry["joint_step_deg"] = float(np.rad2deg(np.max(np.abs(constrained - solved))))
        self._telemetry["joint_clipped"] = bool(np.max(np.abs(constrained - solved)) > 1e-12)
        self._latest_joint = constrained.copy()
        return constrained


def _apply_delta_pose(xyz, quat_wxyz, delta_xyz, delta_rot):
    """Compose an xyz/wxyz pose with a translation and an angle-axis rotation."""
    xyz = np.asarray(xyz, dtype=float)
    quat = np.asarray(_normalized_quaternion(quat_wxyz, "reference quaternion"), dtype=float)
    increment = _angle_axis_quaternion(np.asarray(delta_rot, dtype=float))
    return xyz + np.asarray(delta_xyz, dtype=float), np.asarray(
        _normalized_quaternion(_quaternion_multiply(increment, quat), "composed quaternion"), dtype=float
    )
