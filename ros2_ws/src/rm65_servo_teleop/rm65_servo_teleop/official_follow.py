"""UR-style bounded pose following, initially for replay and shadow calculation.

No device I/O. Existing InputAdapter supplies measured-anchored targets. The
upstream UR5 target limiter is reused; RealMan Algo supplies FK/IK. An isolated
bad solve holds the last accepted command; persistent failure disarms. Results
are candidates, not authorization to bypass TransportGuard or collision checks.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
import time
from typing import Protocol

import numpy as np

from .follow_targets import clip_pose, limit_pose_step
from .input_adapter import Pose
from .official_sdk import joints6
from . import ur_follow_frames as frames


def pose6(pose: Pose) -> tuple:
    return tuple(pose.xyz) + frames.quat_to_rotvec(pose.quaternion_xyzw)


def pose_from6(value) -> Pose:
    return Pose(np.asarray(value[:3]), np.asarray(frames.rotvec_to_quat(value[3:])))


def pose_errors(a: Pose, b: Pose) -> tuple[float, float]:
    relative = frames.mat_mul(frames.quat_to_matrix(a.quaternion_xyzw),
                              frames.mat_transpose(frames.quat_to_matrix(b.quaternion_xyzw)))
    return float(np.linalg.norm(a.xyz - b.xyz)), frames.rotation_angle(relative)


class Solver(Protocol):
    def solve(self, target: Pose, seed: np.ndarray) -> np.ndarray: ...
    def forward(self, joints: np.ndarray) -> Pose: ...


@dataclass(frozen=True)
class FollowConfig:
    joint_lower: tuple[float, ...]
    joint_upper: tuple[float, ...]
    joint_velocity: tuple[float, ...]
    workspace_min: tuple[float, float, float]
    workspace_max: tuple[float, float, float]
    period_s: float = 0.02
    linear_velocity_m_s: float = 0.15
    angular_velocity_rad_s: float = 0.25
    source_timeout_s: float = 0.20
    solve_budget_s: float = 0.015
    reject_hold_s: float = 0.12
    progress_timeout_s: float = 0.30
    frame_position_tolerance_m: float = 0.005
    frame_rotation_tolerance_rad: float = math.radians(2)
    solution_position_tolerance_m: float = 0.001
    solution_rotation_tolerance_rad: float = 0.01
    max_lead_cycles: float = 3.0
    max_attempts: int = 4

    def __post_init__(self):
        lower, upper, speed = map(joints6, (self.joint_lower, self.joint_upper, self.joint_velocity))
        if np.any(lower >= upper) or np.any(speed <= 0):
            raise ValueError("invalid six-axis limits or speeds")
        lo, hi = np.asarray(self.workspace_min), np.asarray(self.workspace_max)
        if lo.shape != (3,) or hi.shape != (3,) or not np.all(np.isfinite(np.r_[lo, hi])) or np.any(lo >= hi):
            raise ValueError("workspace requires three ordered finite bounds")
        for name in (
            "period_s", "linear_velocity_m_s", "angular_velocity_rad_s", "source_timeout_s",
            "solve_budget_s", "reject_hold_s", "progress_timeout_s", "frame_position_tolerance_m", "frame_rotation_tolerance_rad",
            "solution_position_tolerance_m", "solution_rotation_tolerance_rad", "max_lead_cycles",
        ):
            value = getattr(self, name)
            if isinstance(value, bool) or not math.isfinite(value) or value <= 0:
                raise ValueError(f"{name} must be finite and positive")
        if not self.solve_budget_s < self.period_s or not self.reject_hold_s < self.source_timeout_s:
            raise ValueError("solve/hold windows must be shorter than control/source periods")
        if type(self.max_attempts) is not int or not 1 <= self.max_attempts <= 8:
            raise ValueError("max_attempts must be in 1..8")
        if self.max_lead_cycles < 1:
            raise ValueError("max_lead_cycles must be at least one")


@dataclass(frozen=True)
class FollowResult:
    state: str
    reason: str
    command_rad: tuple[float, ...] | None = None
    target: Pose | None = None
    accepted: bool = False
    compute_s: float = 0.0
    attempts: int = 0

    def record(self) -> dict:
        return {
            "state": self.state, "reason": self.reason, "candidate_joint_rad": self.command_rad,
            "shaped_target_xyzw": None if self.target is None else
                [*self.target.xyz.tolist(), *self.target.quaternion_xyzw.tolist()],
            "accepted": self.accepted, "compute_s": self.compute_s, "attempts": self.attempts,
            "execution_authorized": False,
        }


class OfficialFollower:
    def __init__(self, config: FollowConfig, solver: Solver, *, clock=time.monotonic):
        self.config, self.solver, self.clock = config, solver, clock
        self.lower, self.upper, self.velocity = map(joints6, (
            config.joint_lower, config.joint_upper, config.joint_velocity))
        self.seed = None
        self.last_pose = None
        self.fault = None
        self.require_release = True
        self.reject_since = None
        self.last_now = None
        self.progress_since = self.progress_reference = None

    def _fault(self, reason, elapsed=0.0, attempts=0):
        self.fault, self.require_release = reason, True
        self.seed = self.last_pose = None
        return FollowResult("fault", reason, compute_s=elapsed, attempts=attempts)

    def _hold(self, reason, now, elapsed, attempts):
        self.reject_since = now if self.reject_since is None else self.reject_since
        if now - self.reject_since >= self.config.reject_hold_s:
            return self._fault("persistent_" + reason, elapsed, attempts)
        return FollowResult("hold", reason, None if self.seed is None else tuple(self.seed),
                            self.last_pose, compute_s=elapsed, attempts=attempts)

    def step(self, *, enabled: bool, desired: Pose | None, target_at: float | None,
             measured_pose: Pose | None, measured_joints, feedback_at: float | None,
             now: float) -> FollowResult:
        started = self.clock()
        cfg = self.config
        if not math.isfinite(now) or (self.last_now is not None and now < self.last_now):
            return self._fault("clock_regressed_or_invalid")
        self.last_now = now
        if not enabled:
            self.seed = self.last_pose = self.fault = self.reject_since = None
            self.progress_since = self.progress_reference = None
            self.require_release = False
            return FollowResult("idle", "released")
        if self.fault:
            return FollowResult("fault", self.fault)
        if self.require_release:
            return FollowResult("idle", "release_grip_to_start")
        if any(stamp is None or not math.isfinite(stamp) or not 0 <= now - stamp <= cfg.source_timeout_s
               for stamp in (target_at, feedback_at)):
            return self._fault("target_or_feedback_stale")
        try:
            measured = joints6(measured_joints)
        except (TypeError, ValueError):
            return self._fault("invalid_measured_joints")
        if np.any(measured < self.lower) or np.any(measured > self.upper):
            return self._fault("measured_joint_limit")
        if not isinstance(desired, Pose) or not isinstance(measured_pose, Pose):
            return self._fault("missing_target_or_feedback_pose")
        if np.any(measured_pose.xyz < cfg.workspace_min) or np.any(measured_pose.xyz > cfg.workspace_max):
            return self._fault("measured_tcp_outside_workspace")
        try:
            official_pose = self.solver.forward(measured.copy())
        except Exception as exc:
            return self._fault("feedback_fk_failed_" + type(exc).__name__)
        position_error, rotation_error = pose_errors(official_pose, measured_pose)
        if position_error > cfg.frame_position_tolerance_m or rotation_error > cfg.frame_rotation_tolerance_rad:
            return self._fault("official_fk_feedback_frame_mismatch")
        if self.clock() - started >= cfg.solve_budget_s:
            return self._hold("compute_budget", now, self.clock() - started, 0)
        if self.seed is None:
            self.seed, self.last_pose = measured.copy(), measured_pose.copied()
            self.progress_since, self.progress_reference = now, measured.copy()
            return FollowResult("anchoring", "measured_origin", tuple(measured), self.last_pose)
        step_budget = self.velocity * cfg.period_s
        goal = clip_pose(pose6(desired), cfg.workspace_min, cfg.workspace_max)
        distance, angle = pose_errors(pose_from6(goal), measured_pose)
        needs_motion = distance > cfg.solution_position_tolerance_m or angle > cfg.solution_rotation_tolerance_rad
        if not needs_motion or np.any(np.abs(measured - self.progress_reference) >= step_budget * 0.1):
            self.progress_since, self.progress_reference = now, measured.copy()
        elif now - self.progress_since >= cfg.progress_timeout_s:
            return self._fault("feedback_no_progress", self.clock() - started)
        # Bound target progression, then its distance from measured feedback,
        # matching the two-stage upstream idea. Do not renew anchors on lag.
        goal = limit_pose_step(pose6(self.last_pose), goal,
                               cfg.linear_velocity_m_s * cfg.period_s,
                               cfg.angular_velocity_rad_s * cfg.period_s)
        goal = limit_pose_step(pose6(measured_pose), goal,
                               cfg.linear_velocity_m_s * cfg.period_s,
                               cfg.angular_velocity_rad_s * cfg.period_s)
        reason = "ik_rejected"
        attempts = 0
        for attempt in range(cfg.max_attempts):
            if self.clock() - started >= cfg.solve_budget_s:
                reason = "compute_budget"
                break
            attempts += 1
            factor = 0.5 ** attempt
            candidate_pose = pose_from6(limit_pose_step(
                pose6(self.last_pose), goal, cfg.linear_velocity_m_s * cfg.period_s * factor,
                cfg.angular_velocity_rad_s * cfg.period_s * factor))
            # Never accept an SDK result solely because it returned success.
            try:
                candidate = joints6(self.solver.solve(candidate_pose, self.seed.copy()))
                if np.any(candidate < self.lower) or np.any(candidate > self.upper):
                    reason = "ik_joint_limit"
                    continue
                if np.any(np.abs(candidate - self.seed) > step_budget + 1e-9):
                    reason = "ik_joint_jump"
                    continue
                if np.any(np.abs(candidate - measured) > step_budget * cfg.max_lead_cycles + 1e-9):
                    reason = "feedback_lead"
                    continue
                achieved = self.solver.forward(candidate.copy())
            except Exception as exc:
                reason = "ik_failed_" + type(exc).__name__
                continue
            elapsed = self.clock() - started
            if elapsed >= cfg.solve_budget_s:
                reason = "compute_budget"
                break
            position_error, rotation_error = pose_errors(achieved, candidate_pose)
            if position_error > cfg.solution_position_tolerance_m or rotation_error > cfg.solution_rotation_tolerance_rad:
                reason = "ik_fk_residual"
                continue
            if np.any(achieved.xyz < cfg.workspace_min) or np.any(achieved.xyz > cfg.workspace_max):
                reason = "achieved_tcp_outside_workspace"
                continue
            self.seed, self.last_pose, self.reject_since = candidate.copy(), achieved.copied(), None
            return FollowResult("tracking", "accepted", tuple(candidate), achieved, True, elapsed, attempts)
        return self._hold(reason, now, self.clock() - started, attempts)
