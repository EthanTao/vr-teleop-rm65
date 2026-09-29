"""Local differential IK; only the returned, guarded joint step may be sent.

Uses official FK in explicitly configured work/TCP frames, never the display URDF.
Linear Jacobian rows are normalized by 0.3 m before SVD.

Direction policy (2026-09-26 revision)
-------------------------------------
The step is the *minimum-norm* least-squares solution, i.e. the least joint motion that
realizes the operator's Cartesian direction, and the singularity only limits how fast that
direction may be walked:

* the requested direction is never rotated. The previous damped least-squares weights
  ``sigma / (sigma^2 + lambda^2)`` attenuated the weakly conditioned components far more
  than the strong ones, so the achieved motion drifted towards "a direction that still
  works" while the handle asked for another one;
* a direction whose *effective* Jacobian gain ``sigma_eff = |e| / |J+ e|`` falls below the
  hard stop line cannot be walked at all, so the solver holds instead of burning the joint
  budget on a direction with no Cartesian progress. That is the direction-level counterpart
  of ``sigma_min / sigma_max``;
* leaving a singular configuration always has priority: a structurally outward step is
  admitted even when its gain is inside the hard stop line, and the joint-space guard only
  rejects steps that make an affected J3/J5 axis *worse*.
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
    # Human-readable reason for this step or refusal; diagnostics only, never a control input.
    detail: str = ""
    # Effective Jacobian gain of the commanded direction, ``sigma_eff / sigma_max`` in (0, 1].
    # It is the direction-level counterpart of ``ratio``: a large value means the operator's
    # direction is easy for the arm, a value at or below ``stop_ratio`` means it cannot be
    # walked at all. Diagnostics and the hold decision only.
    direction_ratio: float = 1.0
    # True when the hold comes from that direction-level test rather than from the structural
    # J3/J5 guard, so the console can say *why* the arm is not following the handle.
    direction_blocked: bool = False

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
                f"{where}：{self.compute_budget_s * 1000:.1f} ms 计算预算已耗尽",
                fk_calls=self._fk_calls,
                elapsed_s=now - self._cycle_started_at,
            )
        if self._fk_calls + required_fk > self._cycle_max_fk_calls:
            self._budget_active = False
            raise DLSBudgetExceeded(
                f"{where}：FK 调用将超过上限 {self._cycle_max_fk_calls}",
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
        self._require_budget(missing, "雅可比")
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

        Legacy, *not* on the hardware path (the realtime path uses the constant-time
        :meth:`bounded_path_allowed`): it samples four Jacobians per call. It still encodes
        the pre-2026-09-26 blanket policy - "a hard zone may only move when an affected metric
        strictly improves, and a configuration ratio at or below the stop line holds every
        step" - so it is deliberately left unchanged for the offline cache/benchmark tooling
        and must not be used to decide motion. Verdicts are cached per cycle: the caller asks
        the same question again after rounding the step, and repeating it would rebuild four
        Jacobians for nothing.
        """
        key = (tuple(np.asarray(q, dtype=float)), tuple(np.asarray(target, dtype=float)))
        cached = self._path_cache.get(key)
        if cached is not None:
            return cached
        verdict = self._path_allowed_uncached(q, target)
        self._path_cache[key] = verdict
        return verdict

    def bounded_path_allowed(self, q, target, *, angles, lag_limit_rad=None):
        """Constant-time realtime guard which never evaluates another Jacobian.

        The DLS step is already velocity/acceleration limited. This guard adds the structural
        joint-space rules and is deliberately one-sided: it rejects a J3/J5 zero crossing,
        rejects *entering* the J3/J5 hard zone from outside, and inside the hard zone rejects
        only a step that makes an affected axis worse. Tangential motion inside the hard zone
        and every outward step pass, so the operator can keep following the handle (slowly)
        and can always drive back out.

        The previous revision also required a hard-zone step to *strictly* improve an affected
        axis and held every step whenever the configuration ratio was already at or below the
        stop line. Both clauses froze the arm in every direction except the escape one, which
        is what "the arm stops following the handle near the singularity" looked like on site.
        The direction-level decision ("can this commanded direction be walked at all") now
        lives in :meth:`step`, where the Jacobian is already available.

        ``lag_limit_rad`` keeps every structural guard and additionally allows one case the
        seed-only criterion cannot express: the seed is a command the arm has not finished
        yet, and the target may legitimately sit between the seed and the measured joints.
        When the target is within ``lag_limit_rad`` of the measured joints, the hard J3/J5
        hard-zone *entry* rejection is dropped, but the exact J3/J5 zero-crossing rejection
        is still evaluated from the measured joints - the position the arm physically
        occupies. Passing ``None`` keeps the seed-only criterion.
        """
        q = _finite_vector(q, 6, "path start")
        target = _finite_vector(target, 6, "path target")
        angles = _finite_vector(angles, 2, "singularity joint angles")
        within_measured_step = False
        if lag_limit_rad is not None:
            lag_limit = _finite_vector(lag_limit_rad, 6, "tracking allowance")
            if np.any(lag_limit <= 0.0):
                raise ValueError("tracking allowance must be positive")
            within_measured_step = bool(np.all(np.abs(target - np.asarray(q, dtype=float)) <= lag_limit + 1e-9))
        q3, q5 = float(q[2]), float(q[4])
        target3, target5 = float(target[2]), float(target[4])
        if q3 * target3 < 0.0 or q5 * target5 < 0.0:
            return False
        next_angles = np.abs(target[[2, 4]])
        if within_measured_step:
            # One bounded catch-up step measured from where the arm actually is. It may not
            # enter the J3/J5 hard zone and may not make a hard-zone axis worse than measured.
            return not bool(np.any(next_angles < np.minimum(angles, self.stop_joint_rad) - 1e-10))
        hard = angles <= self.stop_joint_rad
        if np.any(hard):
            # Already inside the hard zone: only *going deeper* on an affected axis is
            # refused. A tangential step keeps that axis where it is and must pass; so must
            # the escape step. Axes outside the hard zone keep the seed-only criterion.
            return not bool(np.any(next_angles[hard] < angles[hard] - 1e-10))
        return not bool(np.any(next_angles < self.stop_joint_rad))

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
        lag_speed_scale=1.0,
        reset_cycle=True,
    ):
        """Compute one bounded joint step.

        ``speed_limits`` is the per-joint velocity budget (rad/s). ``lag_speed_scale``
        additionally shrinks this step so the command cannot keep running away from the
        measured joints: the caller derives it from the remaining tracking allowance, so
        1.0 means "full speed" and 0.0 means "hold here". It is deliberately applied to
        the already acceleration-limited velocity, which keeps the reported speed
        monotonic instead of producing a one-cycle spike.

        The returned step walks the operator's direction (see the module docstring): it is
        shortened, never rotated. ``hold_singularity`` is returned when that direction cannot
        be walked at all (``direction_blocked``) or when the structural J3/J5 guard rejects
        every candidate; ``escaping`` is reported when the step moves an affected J3/J5 axis
        outward. The singularity is a speed limit on the requested direction, not a reason to
        choose a different direction.
        """
        if reset_cycle:
            self.begin_cycle(enforce_budget=True)
        q = _finite_vector(seed, 6, "seed")
        xyz = _finite_vector(xyz, 3, "target xyz")
        quat = _normalized_quaternion(quat, "target quaternion")
        speeds = _finite_vector(speed_limits, 6, "joint speeds")
        try:
            lag_scale = float(lag_speed_scale)
        except (TypeError, ValueError) as exc:
            raise ValueError("lag_speed_scale must be a finite number") from exc
        if not np.isfinite(lag_scale) or not 0.0 <= lag_scale <= 1.0:
            raise ValueError("lag_speed_scale must be finite and within [0, 1]")
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
        # Diagnostic only: the step below no longer uses direction damping. A damped
        # least-squares weight attenuates the weakly conditioned components more than the
        # strong ones, which rotates the achieved motion away from the commanded direction.
        damping = max(0.001, self.max_damping * severity)
        scale = max(self.min_speed_scale, 1 - severity)

        def result(target, status, factor=0.0, detail=""):
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
                detail,
                direction_ratio,
                blocked_direction,
            )
            if reset_cycle:
                self.end_cycle()
            return value

        direction_ratio = 1.0
        blocked_direction = False
        error = np.concatenate((xyz - origin, shortest_world_rotation_vector(orientation, quat)))
        if np.linalg.norm(error[:3]) < 1e-7 and np.linalg.norm(error[3:]) < 1e-6:
            return result(q, "hold_at_target", detail="目标位姿已到达")
        # Common scale preserves the operator's six-dimensional direction.
        error *= min(
            1.0,
            max_linear_speed * dt / max(np.linalg.norm(error[:3]), 1e-15),
            max_angular_speed * dt / max(np.linalg.norm(error[3:]), 1e-15),
        )
        error[:3] /= self.length
        # Minimum-norm least-squares step: the least joint motion that realizes the commanded
        # direction. Only the components the arm cannot produce are projected away; no
        # weighting may rotate the rest, or the arm walks where the operator did not ask.
        # The floor is purely numerical: the very directions it caps are the ones the
        # direction test below refuses anyway.
        sigma_max = max(float(s[0]), 1e-12)
        weights = 1.0 / np.maximum(s, sigma_max * 1e-9)
        delta = vt.T @ (weights * (u.T @ error))
        delta_norm = float(np.linalg.norm(delta))
        # Effective Jacobian gain of this command direction, ``|e| / |J+ e| / sigma_max``:
        # the direction-level counterpart of ``sigma_min / sigma_max``. A direction near the
        # hard stop line needs ~100x the joint motion of the best direction, so it cannot be
        # walked within one control period; that is a "hold", not a reason to pick another way.
        direction_ratio = 0.0 if delta_norm <= 0.0 else float(np.linalg.norm(error)) / delta_norm / sigma_max
        blocked_direction = bool(direction_ratio <= self.stop_ratio)
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
        if blocked_direction and not improving:
            # The handle asks for a direction this configuration cannot walk. Stop here
            # instead of redistributing the request into a direction that still works.
            return result(
                q,
                "hold_singularity",
                detail=(
                    f"手柄方向不可行：该方向的等效奇异比 {direction_ratio:.5f} ≤ 硬停线 {self.stop_ratio:g}"
                    "（σ_eff/σ_max；该方向的等效关节代价超过本周期预算）；"
                    "保持不动，切向与退出方向不受影响"
                ),
            )
        velocity = delta * scale / dt
        old = np.zeros(6) if previous_velocity is None else _finite_vector(previous_velocity, 6, "previous velocity")
        # The acceleration limit is ONE common factor on this cycle's velocity change, not a
        # per-joint clip. A per-joint clip saturates the fast joints more than the slow ones, so
        # it rotates the commanded direction: at the start of a segment (previous velocity zero)
        # the first step could end up pointing away from the target, the candidate was refused
        # with "reachable error grew", the stream was held and cleared, and the identical refusal
        # repeated - the arm never started moving in that direction at all. Every joint still
        # changes by at most ``max_joint_acceleration * dt``; the slow joints now change by less.
        change = velocity - old
        allowed_change = max_joint_acceleration * dt
        peak_change = float(np.max(np.abs(change)))
        if peak_change > allowed_change:
            velocity = old + change * (allowed_change / peak_change)
        velocity = np.clip(velocity, -speeds * scale, speeds * scale)
        delta = velocity * dt
        # Tracking-allowance governor: one common factor preserves the commanded
        # direction while bounding how far the command may lead the measured joints.
        if lag_scale < 1.0:
            delta = delta * lag_scale
        effective_scale = scale * lag_scale
        limits = RM65_B_SPEC.joint_limit_rad
        lo, hi = limits[:, 0] + np.deg2rad(5), limits[:, 1] - np.deg2rad(5)
        if np.any((q < lo) & (delta < -1e-12)) or np.any((q > hi) & (delta > 1e-12)):
            return result(q, "hold_joint_margin", detail="有轴已进入 5° 软限位余量")
        factors = (1.0, 0.5, 0.25, 0.125)[: self.max_candidate_attempts]
        rejected = []
        for factor in factors:
            self._candidate_attempts += 1
            self.check_budget("候选搜索")
            candidate = np.deg2rad(np.round(np.rad2deg(q + delta * factor) * 1000) / 1000)
            if np.max(np.abs(candidate - q)) < 1e-10:
                rejected.append("步长低于 0.001° 下发分辨率")
                continue
            if np.any(candidate < limits[:, 0]) or np.any(candidate > limits[:, 1]):
                rejected.append("关节硬限位")
                continue
            if np.any((q >= lo) & (candidate < lo)) or np.any((q <= hi) & (candidate > hi)):
                rejected.append("关节软限位余量")
                continue
            if not self.bounded_path_allowed(q, candidate, angles=angles):
                rejected.append("奇异路径守卫")
                continue
            xp, rp = self.forward(candidate)
            achieved = np.concatenate(((xp - origin) / self.length, shortest_world_rotation_vector(orientation, rp)))
            if np.linalg.norm(error - achieved) > np.linalg.norm(error) + 1e-10:
                rejected.append("可达误差反而变大")
                continue
            return result(candidate, "escaping" if improving and severity else "tracking", effective_scale * factor)
        if blocked_direction:
            context = f"手柄方向不可行：该方向的等效奇异比 {direction_ratio:.5f} ≤ 硬停线 {self.stop_ratio:g}"
        elif np.any(angles <= self.stop_joint_rad):
            context = "J3/J5 处于 5° 硬区，且没有不加深该轴的步"
        else:
            context = f"奇异比 {ratio:.5f} 高于硬停线 {self.stop_ratio:g}"
        return result(
            q,
            "hold_singularity",
            detail=context + ("" if not rejected else "；候选被拒：" + "；".join(dict.fromkeys(rejected))),
        )
