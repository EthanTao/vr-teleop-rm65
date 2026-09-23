"""Pure NumPy conditioning helpers for XR teleoperation input."""

import numpy as np


def continuous_radial_deadzone(vector: np.ndarray, deadband: float) -> np.ndarray:
    """Apply a continuous radial deadzone without changing vector direction."""
    values = np.asarray(vector, dtype=float)
    if values.ndim != 1 or values.size == 0 or not np.all(np.isfinite(values)):
        raise ValueError("vector must be a non-empty finite one-dimensional array")
    if not np.isfinite(deadband) or deadband < 0:
        raise ValueError("deadband must be finite and non-negative")

    norm = float(np.linalg.norm(values))
    if norm <= deadband or norm == 0.0:
        return np.zeros_like(values)
    return values * ((norm - deadband) / norm)


def rotation_gain_for_linear_speed(
    speed_m_s: float, full_gain_m_s: float, zero_gain_m_s: float
) -> float:
    """Return rotation gain decreasing linearly as translation speed increases."""
    values = (speed_m_s, full_gain_m_s, zero_gain_m_s)
    if not all(np.isfinite(value) for value in values):
        raise ValueError("speed and thresholds must be finite")
    if speed_m_s < 0 or full_gain_m_s < 0 or full_gain_m_s >= zero_gain_m_s:
        raise ValueError("invalid speed or gain thresholds")
    if speed_m_s <= full_gain_m_s:
        return 1.0
    if speed_m_s >= zero_gain_m_s:
        return 0.0
    return float((zero_gain_m_s - speed_m_s) / (zero_gain_m_s - full_gain_m_s))


def xr_pose_step(
    previous_pose_xyzw: np.ndarray, current_pose_xyzw: np.ndarray
) -> tuple[float, float]:
    """Return translation distance and shortest quaternion rotation step."""
    previous = np.asarray(previous_pose_xyzw, dtype=float)
    current = np.asarray(current_pose_xyzw, dtype=float)
    for pose in (previous, current):
        if pose.shape != (7,) or not np.all(np.isfinite(pose)):
            raise ValueError("XR pose must contain exactly seven finite values")

    q0 = previous[3:]
    q1 = current[3:]
    q0_norm = float(np.linalg.norm(q0))
    q1_norm = float(np.linalg.norm(q1))
    if q0_norm == 0.0 or q1_norm == 0.0:
        raise ValueError("XR pose quaternion must be non-zero")
    q0 = q0 / q0_norm
    q1 = q1 / q1_norm

    translation_distance = float(np.linalg.norm(current[:3] - previous[:3]))
    dot = float(np.clip(abs(np.dot(q0, q1)), 0.0, 1.0))
    rotation_angle = float(2.0 * np.arccos(dot))
    return translation_distance, rotation_angle

