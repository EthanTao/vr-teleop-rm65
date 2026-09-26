from __future__ import annotations

import numpy as np


class JointConstraints:
    """关节限位裁剪 + 速度限制。

    从 URDF 自动读取关节限位，对 IK 求解后的关节配置
    依次做限位裁剪和速度限制，防止仿真中出现关节角度超限或帧间跳变过大。
    """

    def __init__(
        self,
        placo_robot,
        dt: float,
        enable_velocity_limit: bool = False,
        max_joint_velocity: float = 1.0,
        max_joint_acceleration: float | None = None,
    ):
        self.dt = float(dt)
        if not np.isfinite(self.dt) or self.dt <= 0.0:
            raise ValueError("dt must be a finite positive number")
        self.enable_velocity_limit = enable_velocity_limit
        self.max_joint_velocity = float(max_joint_velocity)
        if not np.isfinite(self.max_joint_velocity) or self.max_joint_velocity < 0.0:
            raise ValueError("max_joint_velocity must be finite and non-negative")
        # Without an acceleration bound the limiter can reverse direction at full speed in one
        # cycle, which is itself a jump. Default to reaching the speed limit in 0.1 s.
        acceleration = 10.0 * self.max_joint_velocity if max_joint_acceleration is None else max_joint_acceleration
        self.max_joint_acceleration = float(acceleration)
        if not np.isfinite(self.max_joint_acceleration) or self.max_joint_acceleration <= 0.0:
            raise ValueError("max_joint_acceleration must be finite and positive")
        self._prev_q: np.ndarray | None = None
        self._prev_qd = np.zeros(0)

        # 从 URDF 读取关节限位
        # 使用 placo.RobotWrapper.get_joint_limits(name) -> np.ndarray[2] [lower, upper]
        joint_names = placo_robot.joint_names()
        limits = np.array(
            [placo_robot.get_joint_limits(name) for name in joint_names], dtype=np.float64
        )  # shape (n_joints, 2)
        if limits.shape != (len(joint_names), 2):
            raise ValueError("placo joint limits must have shape (n_joints, 2)")
        if not np.all(np.isfinite(limits)) or np.any(limits[:, 0] > limits[:, 1]):
            raise ValueError("placo joint limits must be finite and ordered [lower, upper]")
        self.lower_limits = limits[:, 0]
        self.upper_limits = limits[:, 1]
        self._prev_qd = np.zeros(self.lower_limits.shape)

    def sync(self, q: np.ndarray) -> None:
        """Re-anchor the velocity limiter to ``q`` with zero speed.

        The limit is relative to the previous *output*. A new control segment (Grip press) or a
        finished reset legitimately starts somewhere else, so carrying the old reference across
        it would either forbid the first step or turn the difference into a fake step.
        """
        q = self._validated(q)
        self._prev_q = q.copy()
        self._prev_qd = np.zeros(q.shape)

    def apply(self, q: np.ndarray) -> np.ndarray:
        """Return a finite joint vector constrained by configured limits."""
        q = self._validated(q)
        """关节限位 clip → 速度限制 clip → 返回安全 q"""
        # 1. 关节限位
        q_safe = np.clip(q, self.lower_limits, self.upper_limits)

        # 2. 速度限制（加速度有界，避免一周期内正反向满速切换）
        if self.enable_velocity_limit and self._prev_q is not None:
            max_step = self.max_joint_velocity * self.dt
            desired = np.clip(q_safe - self._prev_q, -max_step, max_step)
            desired_velocity = desired / self.dt
            max_delta = self.max_joint_acceleration * self.dt
            limited_velocity = self._prev_qd + np.clip(
                desired_velocity - self._prev_qd, -max_delta, max_delta
            )
            limited_velocity = np.clip(limited_velocity, -self.max_joint_velocity, self.max_joint_velocity)
            q_safe = self._prev_q + limited_velocity * self.dt
            self._prev_qd = limited_velocity

        if self.enable_velocity_limit:
            self._prev_q = q_safe
        return q_safe

    def _validated(self, q: np.ndarray) -> np.ndarray:
        q = np.asarray(q, dtype=np.float64)
        if q.ndim != 1 or q.shape != self.lower_limits.shape:
            raise ValueError(f"q must have shape ({self.lower_limits.size},)")
        if not np.all(np.isfinite(q)):
            raise ValueError("q must contain only finite values")
        return q
