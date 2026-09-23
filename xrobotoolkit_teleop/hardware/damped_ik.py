"""Local differential IK; only the returned, guarded joint step may be sent.

Uses official FK in explicitly configured work/TCP frames, never the display URDF.
Linear Jacobian rows are normalized by 0.3 m before SVD and damping.
"""

from dataclasses import dataclass
import time

import numpy as np

from .cartesian_command_limiter import (
    _finite_tuple,
    _finite_vector,
    _normalized_quaternion,
    _quaternion_multiply,
    _quaternion_conjugate,
    shortest_world_rotation_vector,
)
from .rm65_model import RM65_B_SPEC


def compose_pose(left, right):
    """Compose xyz/wxyz poses without depending on visualization libraries."""
    x, q = left
    y, r = right
    rotated = _quaternion_multiply(_quaternion_multiply(q, np.r_[0.0, y]), _quaternion_conjugate(q))[1:]
    return x + rotated, _normalized_quaternion(_quaternion_multiply(q, r), "composed quaternion")


class DLSBudgetExceeded(RuntimeError):
    """The realtime DLS budget was exhausted before a safe target was ready."""

    def __init__(self, reason, *, fk_calls, elapsed_s):
        super().__init__(reason)
        self.reason = str(reason)
        self.fk_calls = int(fk_calls)
        self.elapsed_s = float(elapsed_s)


class ControllerFrameKinematics:
    """T_work_tcp = T_work_base @ T_base_flange(q) @ T_flange_tcp.

    Transforms must come from the actual configured frames; never infer them by
    aligning one measured pose. Runtime FK/UDP agreement is an additional gate.
    """

    def __init__(self, official, work_from_base=None, flange_to_tcp=None):
        self.official = official
        self.work_from_base = self._pose(work_from_base)
        self.flange_to_tcp = self._pose(flange_to_tcp)
        self._work_identity = work_from_base is None
        self._tool_identity = flange_to_tcp is None

    @staticmethod
    def _pose(value):
        a = _finite_vector([0, 0, 0, 1, 0, 0, 0] if value is None else value, 7, "frame xyz/wxyz")
        return a[:3], _normalized_quaternion(a[3:], "frame quaternion")

    def forward(self, joint):
        xyz, quat = self.official.forward(_finite_vector(joint, 6, "joints").copy())
        pose = (
            _finite_vector(xyz, 3, "FK xyz"),
            np.asarray(_normalized_quaternion(quat, "FK quaternion"), dtype=float),
        )
        if not self._work_identity:
            pose = compose_pose(self.work_from_base, pose)
        if not self._tool_identity:
            pose = compose_pose(pose, self.flange_to_tcp)
        return pose


@dataclass
class DampedStep:
    joint_rad: np.ndarray
    xyz: np.ndarray
    quat: np.ndarray
    ratio: float
    predicted_ratio: float
    scale: float
    damping: float
    status: str

    @property
    def hold(self):
        return self.status.startswith("hold")


class DampedIK:
    def __init__(
        self,
        kinematics,
        *,
        slowdown_ratio=0.04,
        stop_ratio=0.01,
        slowdown_joint_rad=np.deg2rad(15),
        stop_joint_rad=np.deg2rad(5),
        min_speed_scale=0.05,
        escape_speed_scale=0.1,
        max_damping=0.08,
        compute_budget_s=0.018,
        max_fk_calls=16,
        max_candidate_attempts=3,
        monotonic_fn=time.perf_counter,
    ):
        self.kinematics = kinematics
        self.slowdown_ratio = float(slowdown_ratio)
        self.stop_ratio = float(stop_ratio)
        self.slowdown_joint_rad = float(slowdown_joint_rad)
        self.stop_joint_rad = float(stop_joint_rad)
        self.min_speed_scale = float(min_speed_scale)
        self.escape_speed_scale = float(escape_speed_scale)
        self.max_damping = float(max_damping)
        self.compute_budget_s = float(compute_budget_s)
        if isinstance(max_fk_calls, bool) or not isinstance(max_fk_calls, int) or max_fk_calls < 13:
            raise ValueError("max_fk_calls must be an integer of at least 13")
        if (
            isinstance(max_candidate_attempts, bool)
            or not isinstance(max_candidate_attempts, int)
            or not 1 <= max_candidate_attempts <= 4
        ):
            raise ValueError("max_candidate_attempts must be an integer from 1 through 4")
        self.max_fk_calls = max_fk_calls
        self.max_candidate_attempts = max_candidate_attempts
        self._monotonic = monotonic_fn
        values = (
            self.slowdown_ratio,
            self.stop_ratio,
            self.slowdown_joint_rad,
            self.stop_joint_rad,
            self.min_speed_scale,
            self.escape_speed_scale,
            self.max_damping,
            self.compute_budget_s,
        )
        if not all(np.isfinite(v) and v > 0 for v in values):
            raise ValueError("DLS parameters must be finite and positive")
        if not 0 < self.stop_ratio < self.slowdown_ratio < 1:
            raise ValueError("DLS ratios require 0 < stop < slowdown < 1")
        if not 0 < self.stop_joint_rad < self.slowdown_joint_rad < np.pi:
            raise ValueError("DLS joint zones require 0 < stop < slowdown < pi")
        if not 0 < self.min_speed_scale <= self.escape_speed_scale <= 1:
            raise ValueError("DLS speed scales require 0 < minimum <= escape <= 1")
        self.length = 0.3
        self.difference_step = 0.001  # SDK FK uses float32: avoid cancellation at 1e-6 rad.
        self.begin_cycle()

    def begin_cycle(self, *, deadline=None, max_fk_calls=None, enforce_budget=False):
        self._pose_cache = {}
        self._metric_cache = {}
        self._path_cache = {}
        self._cycle_started_at = self._monotonic()
        self._cycle_deadline = self._cycle_started_at + self.compute_budget_s if deadline is None else float(deadline)
        self._cycle_max_fk_calls = self.max_fk_calls if max_fk_calls is None else int(max_fk_calls)
        self._fk_calls = 0
        self._candidate_attempts = 0
        self._budget_active = bool(enforce_budget)

    def end_cycle(self):
        self._budget_active = False

    @property
    def fk_calls(self):
        return self._fk_calls

    @property
    def candidate_attempts(self):
        return self._candidate_attempts

    @property
    def elapsed_s(self):
        return max(0.0, self._monotonic() - self._cycle_started_at)

    def check_budget(self, where="DLS"):
        self._require_budget(0, where)

    def _require_budget(self, required_fk, where):
        if not self._budget_active:
            return
        now = self._monotonic()
        if now >= self._cycle_deadline:
            self._budget_active = False
            raise DLSBudgetExceeded(
                f"{where}: {self.compute_budget_s * 1000:.1f} ms compute budget exhausted",
                fk_calls=self._fk_calls,
                elapsed_s=now - self._cycle_started_at,
            )
        if self._fk_calls + required_fk > self._cycle_max_fk_calls:
            self._budget_active = False
            raise DLSBudgetExceeded(
                f"{where}: FK budget would exceed {self._cycle_max_fk_calls}",
                fk_calls=self._fk_calls,
                elapsed_s=now - self._cycle_started_at,
            )

    def forward(self, joint):
        """Return ``(xyz, quat)`` as numpy arrays; the cache stores validated tuples.

        Validation is the expensive part on the realtime container, so each distinct
        pose is validated once and converted to arrays only when a caller needs them.
        """
        q = _finite_tuple(joint, 6, "joints")
        cached = self._pose_cache.get(q)
        if cached is None:
            self._require_budget(1, "FK")
            self._fk_calls += 1
            xyz, quat = self.kinematics.forward(np.asarray(q, dtype=float))
            cached = (
                _finite_tuple(xyz, 3, "FK xyz"),
                _normalized_quaternion(quat, "FK quaternion"),
            )
            self._pose_cache[q] = cached
            self._require_budget(0, "FK")
        xyz, quat = cached
        return np.asarray(xyz, dtype=float), np.asarray(quat, dtype=float)

    def jacobian(self, q):
        q = np.asarray(_finite_tuple(q, 6, "joints"), dtype=float)
        # Build the whole central-difference stencil as one (12, 6) block so each
        # stepped configuration is converted and validated once instead of twice.
        # Every numpy call costs tens of microseconds on the realtime container.
        # Index rows and columns as matching pairs, never as a meshgrid.
        stencil = np.repeat(q.reshape(1, 6), 2 * 6, axis=0)
        rows = np.arange(6)
        stencil[2 * rows, rows] += self.difference_step
        stencil[2 * rows + 1, rows] -= self.difference_step
        missing = sum(tuple(row) not in self._pose_cache for row in stencil)
        self._require_budget(missing, "Jacobian")
        jac = np.empty((6, 6))
        divisor = 2 * self.difference_step
        for i in range(6):
            xp, rp = self.forward(stencil[2 * i])
            xm, rm = self.forward(stencil[2 * i + 1])
            jac[:3, i] = (xp - xm) / (divisor * self.length)
            jac[3:, i] = shortest_world_rotation_vector(rm, rp) / divisor
        return jac

    def metrics(self, q):
        q = _finite_tuple(q, 6, "joints")
        if q not in self._metric_cache:
            s = np.linalg.svd(self.jacobian(q), compute_uv=False)
            self._metric_cache[q] = float(s[-1] / max(s[0], 1e-12))
        return self._metric_cache[q], np.abs(np.asarray(q, dtype=float)[[2, 4]])

    def severity(self, ratio, angles):
        ratio_ramp = (self.slowdown_ratio - ratio) / (self.slowdown_ratio - self.stop_ratio)
        angle_ramp = (self.slowdown_joint_rad - angles) / (self.slowdown_joint_rad - self.stop_joint_rad)
        return float(np.clip(max(ratio_ramp, *angle_ramp), 0, 1))

    def path_allowed(self, q, target):
        """Check ordered local samples, with explicit J3/J5 zero crossing rejection.

        In a hard zone only strictly improving affected metrics may move; at an
        exact singularity unavailable Cartesian directions are held, not invented.
        Verdicts are cached per cycle: the caller asks the same question again after
        rounding the step, and repeating it would rebuild four Jacobians for nothing.
        """
        key = (tuple(np.asarray(q, dtype=float)), tuple(np.asarray(target, dtype=float)))
        cached = self._path_cache.get(key)
        if cached is not None:
            return cached
        verdict = self._path_allowed_uncached(q, target)
        self._path_cache[key] = verdict
        return verdict

    def bounded_path_allowed(self, q, target, *, ratio, angles):
        """Constant-time realtime guard which never evaluates another Jacobian.

        The DLS step is already velocity/acceleration limited. This guard adds
        exact J3/J5 zero-crossing protection and only permits motion from a hard
        structural singularity when every affected joint moves outward. If the
        SVD ratio alone is already in the hard zone, there is no cheap, reliable
        escape proof, so the realtime path holds instead of searching.
        """
        q = _finite_vector(q, 6, "path start")
        target = _finite_vector(target, 6, "path target")
        ratio = float(ratio)
        angles = _finite_vector(angles, 2, "singularity joint angles")
        q3, q5 = float(q[2]), float(q[4])
        target3, target5 = float(target[2]), float(target[4])
        if q3 * target3 < 0.0 or q5 * target5 < 0.0:
            return False
        next_angles = np.abs(target[[2, 4]])
        hard = angles <= self.stop_joint_rad
        if ratio <= self.stop_ratio and not np.any(hard):
            return False
        if np.any(hard):
            if np.any(next_angles[hard] < angles[hard] - 1e-10):
                return False
            if not np.any(next_angles[hard] > angles[hard] + 1e-10):
                return False
        elif np.any(next_angles < self.stop_joint_rad):
            return False
        return True

    def _path_allowed_uncached(self, q, target):
        ratio, angles = self.metrics(q)
        # Scalar thresholds and scalar J3/J5 reads: numpy indexing on three elements
        # costs more on the realtime container than the whole comparison.
        stop_ratio = self.stop_ratio
        stop_joint_rad = self.stop_joint_rad
        q3, q5 = float(q[2]), float(q[4])
        for fraction in (0.25, 0.5, 0.75, 1.0):
            sample = q + (target - q) * fraction
            sample3, sample5 = float(sample[2]), float(sample[4])
            if q3 * sample3 < 0.0 or q5 * sample5 < 0.0:
                return False
            new_ratio, new_angles = self.metrics(sample)
            if ratio <= stop_ratio:
                if new_ratio <= ratio + 1e-10:
                    return False
            elif new_ratio < stop_ratio:
                return False
            for old, new in zip(angles, new_angles):
                if old <= stop_joint_rad:
                    if new < old - 1e-10:
                        return False
                elif new < stop_joint_rad:
                    return False
            ratio, angles = new_ratio, new_angles
        return True

    def step(
        self,
        xyz,
        quat,
        seed,
        dt,
        speed_limits,
        *,
        previous_velocity=None,
        max_linear_speed=0.05,
        max_angular_speed=0.25,
        max_joint_acceleration=0.5,
        reset_cycle=True,
    ):
        if reset_cycle:
            self.begin_cycle(enforce_budget=True)
        q = _finite_vector(seed, 6, "seed")
        xyz = _finite_vector(xyz, 3, "target xyz")
        quat = _normalized_quaternion(quat, "target quaternion")
        speeds = _finite_vector(speed_limits, 6, "joint speeds")
        if np.any(speeds <= 0) or not all(
            np.isfinite(v) and v > 0 for v in (dt, max_linear_speed, max_angular_speed, max_joint_acceleration)
        ):
            raise ValueError("DLS time and velocity/acceleration limits must be positive")
        valid, reason = RM65_B_SPEC.validate_joint_feedback(q, 0)
        if not valid:
            raise ValueError(reason)
        origin, orientation = self.forward(q)
        u, s, vt = np.linalg.svd(self.jacobian(q), full_matrices=False)
        ratio = float(s[-1] / max(s[0], 1e-12))
        angles = np.abs(q[[2, 4]])
        self._metric_cache[_finite_tuple(q, 6, "seed")] = ratio
        severity = self.severity(ratio, angles)
        damping = max(0.001, self.max_damping * severity)
        scale = max(self.min_speed_scale, 1 - severity)

        def result(target, status, factor=0.0):
            xp, rp = self.forward(target)
            value = DampedStep(
                np.asarray(target, dtype=float).copy(),
                np.asarray(xp, dtype=float),
                np.asarray(rp, dtype=float),
                ratio,
                ratio,
                factor,
                damping,
                status,
            )
            if reset_cycle:
                self.end_cycle()
            return value

        error = np.concatenate((xyz - origin, shortest_world_rotation_vector(orientation, quat)))
        if np.linalg.norm(error[:3]) < 1e-7 and np.linalg.norm(error[3:]) < 1e-6:
            return result(q, "hold_at_target")
        # Common scale preserves the operator's six-dimensional direction.
        error *= min(
            1.0,
            max_linear_speed * dt / max(np.linalg.norm(error[:3]), 1e-15),
            max_angular_speed * dt / max(np.linalg.norm(error[3:]), 1e-15),
        )
        error[:3] /= self.length
        delta = vt.T @ ((s / (s * s + damping * damping)) * (u.T @ error))
        delta *= min(1.0, float(np.min(speeds * dt / np.maximum(np.abs(delta), 1e-15))))
        # A hard-zone speed floor is only granted for an unambiguous local
        # J3/J5 outward direction. No second Jacobian is built to search for it.
        preview_angles = np.abs((q + delta * 0.01)[[2, 4]])
        hard = angles <= self.stop_joint_rad
        improving = bool(
            np.any(hard)
            and np.all(preview_angles[hard] >= angles[hard] - 1e-10)
            and np.any(preview_angles[hard] > angles[hard] + 1e-10)
        )
        if improving:
            scale = max(scale, self.escape_speed_scale)
        velocity = delta * scale / dt
        old = np.zeros(6) if previous_velocity is None else _finite_vector(previous_velocity, 6, "previous velocity")
        velocity = old + np.clip(velocity - old, -max_joint_acceleration * dt, max_joint_acceleration * dt)
        velocity = np.clip(velocity, -speeds * scale, speeds * scale)
        delta = velocity * dt
        limits = RM65_B_SPEC.joint_limit_rad
        lo, hi = limits[:, 0] + np.deg2rad(5), limits[:, 1] - np.deg2rad(5)
        if np.any((q < lo) & (delta < -1e-12)) or np.any((q > hi) & (delta > 1e-12)):
            return result(q, "hold_joint_margin")
        factors = (1.0, 0.5, 0.25, 0.125)[: self.max_candidate_attempts]
        for factor in factors:
            self._candidate_attempts += 1
            self.check_budget("candidate search")
            candidate = np.deg2rad(np.round(np.rad2deg(q + delta * factor) * 1000) / 1000)
            if np.max(np.abs(candidate - q)) < 1e-10:
                continue
            if np.any(candidate < limits[:, 0]) or np.any(candidate > limits[:, 1]):
                continue
            if np.any((q >= lo) & (candidate < lo)) or np.any((q <= hi) & (candidate > hi)):
                continue
            if not self.bounded_path_allowed(q, candidate, ratio=ratio, angles=angles):
                continue
            xp, rp = self.forward(candidate)
            achieved = np.concatenate(((xp - origin) / self.length, shortest_world_rotation_vector(orientation, rp)))
            if np.linalg.norm(error - achieved) > np.linalg.norm(error) + 1e-10:
                continue
            return result(candidate, "escaping" if improving and severity else "tracking", scale * factor)
        return result(q, "hold_singularity")
