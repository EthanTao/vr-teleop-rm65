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
        max_joint_velocity: float = 3.0,
    ):
        self.dt = float(dt)
        if not np.isfinite(self.dt) or self.dt <= 0.0:
            raise ValueError("dt must be a finite positive number")
        self.enable_velocity_limit = enable_velocity_limit
        self.max_joint_velocity = float(max_joint_velocity)
        if not np.isfinite(self.max_joint_velocity) or self.max_joint_velocity < 0.0:
            raise ValueError("max_joint_velocity must be finite and non-negative")
        self._prev_q: np.ndarray | None = None

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

    def apply(self, q: np.ndarray) -> np.ndarray:
        """Return a finite joint vector constrained by configured limits."""
        q = np.asarray(q, dtype=np.float64)
        if q.ndim != 1 or q.shape != self.lower_limits.shape:
            raise ValueError(f"q must have shape ({self.lower_limits.size},)")
        if not np.all(np.isfinite(q)):
            raise ValueError("q must contain only finite values")
        """关节限位 clip → 速度限制 clip → 返回安全 q"""
        # 1. 关节限位
        q_safe = np.clip(q, self.lower_limits, self.upper_limits)

        # 2. 速度限制
        if self.enable_velocity_limit and self._prev_q is not None:
            max_step = self.max_joint_velocity * self.dt
            delta = q_safe - self._prev_q
            delta = np.clip(delta, -max_step, max_step)
            q_safe = self._prev_q + delta

        if self.enable_velocity_limit:
            self._prev_q = q_safe
        return q_safe
