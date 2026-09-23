from __future__ import annotations

import numpy as np


class MotionFilter:
    """死区过滤 + EMA 平滑，作用于 XR delta 信号。

    - 死区：每个分量独立判断，|v| < deadband 则归零
    - EMA：y_t = alpha * x_t + (1-alpha) * y_{t-1}
    """

    def __init__(
        self,
        deadband_m: float = 0.002,
        deadband_rad: float = 0.03,
        smooth_alpha_pos: float = 0.35,
        smooth_alpha_rot: float = 0.3,
    ):
        self.deadband_m = float(deadband_m)
        self.deadband_rad = float(deadband_rad)
        if not np.isfinite(self.deadband_m) or self.deadband_m < 0.0:
            raise ValueError("deadband_m must be finite and non-negative")
        if not np.isfinite(self.deadband_rad) or self.deadband_rad < 0.0:
            raise ValueError("deadband_rad must be finite and non-negative")
        if not np.isfinite(smooth_alpha_pos) or not np.isfinite(smooth_alpha_rot):
            raise ValueError("smoothing coefficients must be finite")
        self.smooth_alpha_pos = float(np.clip(smooth_alpha_pos, 0.0, 1.0))
        self.smooth_alpha_rot = float(np.clip(smooth_alpha_rot, 0.0, 1.0))
        self._prev_xyz: np.ndarray = np.zeros(3)
        self._prev_rot: np.ndarray = np.zeros(3)

    def reset(self) -> None:
        """重置 EMA 历史，grip 按下时调用"""
        self._prev_xyz.fill(0.0)
        self._prev_rot.fill(0.0)

    def apply(
        self,
        delta_xyz: np.ndarray,
        delta_rot: np.ndarray,
    ) -> tuple[np.ndarray, np.ndarray]:
        """Return filtered translation and angle-axis rotation deltas."""
        delta_xyz = np.asarray(delta_xyz, dtype=np.float64)
        delta_rot = np.asarray(delta_rot, dtype=np.float64)
        if delta_xyz.shape != (3,) or delta_rot.shape != (3,):
            raise ValueError("delta_xyz and delta_rot must both have shape (3,)")
        if not np.all(np.isfinite(delta_xyz)) or not np.all(np.isfinite(delta_rot)):
            raise ValueError("delta_xyz and delta_rot must contain only finite values")
        """死区过滤 → EMA 平滑 → 返回 (filtered_xyz, filtered_rot)"""
        # 死区
        out_xyz = np.where(np.abs(delta_xyz) >= self.deadband_m, delta_xyz, 0.0)
        out_rot = np.where(np.abs(delta_rot) >= self.deadband_rad, delta_rot, 0.0)

        # EMA 平滑：y_t = alpha * x_t + (1-alpha) * y_{t-1}
        out_xyz = self._prev_xyz + self.smooth_alpha_pos * (out_xyz - self._prev_xyz)
        out_rot = self._prev_rot + self.smooth_alpha_rot * (out_rot - self._prev_rot)

        self._prev_xyz = out_xyz
        self._prev_rot = out_rot

        return out_xyz, out_rot
