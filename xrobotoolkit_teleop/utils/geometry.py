import numpy as np
import meshcat.transformations as tf

R_HEADSET_TO_WORLD = np.array(
    [
        [0, 0, -1],
        [-1, 0, 0],
        [0, 1, 0],
    ]
)

# RM65 side-mounted installation: base on a vertical surface, not a horizontal floor.
# Swaps human forward/back with robot vertical axis and inverts both for natural teleop.
# 仿真验证矩阵（2026-07-13，Placo 仿真中的自然手感）；真机在 invert_tcp_xy=True 时的
# 有效映射与本矩阵等价。注意：仿真入口 teleop_rm65_sim.py 当前引用的是下方 _HARDWARE
# （代码为准，2026-09-09 定案：回到改动前行为），方向是否直觉以逐轴实测为准。
R_HEADSET_TO_WORLD_RM65_SIDE_MOUNT = np.array(
    [
        [0, -1, 0],
        [-1, 0, 0],
        [0, 0, -1],
    ]
)

# Controller TCP frame differs from URDF visualization: invert horizontal axes only.
# 真机控制器矩阵；仿真入口 teleop_rm65_sim.py 当前亦引用本矩阵（以代码为准）。
# 真机配 invert_tcp_xy=True 时 delta 的 X/Y 再取反 → 有效映射回到上方 SIDE_MOUNT 语义。
R_HEADSET_TO_WORLD_RM65_SIDE_MOUNT_HARDWARE = np.array(
    [
        [0, 1, 0],
        [1, 0, 0],
        [0, 0, -1],
    ]
)


def is_valid_quaternion(quat, tol=1e-6):
    if not isinstance(quat, (list, tuple, np.ndarray)) or len(quat) != 4:
        return False

    if not np.all(np.isfinite(quat)):
        return False

    magnitude = np.sqrt(np.sum(np.square(quat)))
    return abs(magnitude - 1.0) <= tol


def quaternion_to_angle_axis(quat: np.ndarray, eps: float = 1e-6) -> np.ndarray:
    """Converts a quaternion to an angle-axis representation.

    Args:
        quat: Quaternion in (w, x, y, z) format.
        eps: Tolerance for checking if the angle is close to zero.

    Returns:
        Angle-axis vector [ax*angle, ay*angle, az*angle].
    """
    q = np.asarray(quat, dtype=np.float64)
    if q.shape != (4,) or not np.all(np.isfinite(q)):
        raise ValueError("quat must be a finite array with shape (4,)")
    norm = float(np.linalg.norm(q))
    if norm < eps:
        raise ValueError("quat norm must be non-zero")
    q = q / norm
    if q[0] < 0.0:
        q = -q

    w = q[0]
    vec_part = q[1:]

    angle = 2.0 * np.arccos(np.clip(w, -1.0, 1.0))
    if angle < eps:
        return np.zeros(3, dtype=np.float64)

    sin_half_angle = np.sin(angle / 2.0)
    if sin_half_angle < eps:
        return np.zeros(3, dtype=np.float64)

    axis = vec_part / sin_half_angle
    return axis * angle


def quat_diff_as_angle_axis(q1: np.ndarray, q2: np.ndarray, eps: float = 1e-6) -> np.ndarray:
    """Calculates the rotation from q1 to q2 as an angle-axis vector.

    This computes DeltaQ such that q2 = DeltaQ * q1.
    The result is the angle-axis representation of DeltaQ.

    Args:
        q1: Source quaternion (w, x, y, z).
        q2: Target quaternion (w, x, y, z).
        eps: Tolerance for small angle calculations.

    Returns:
        Angle-axis vector [ax*angle, ay*angle, az*angle] representing DeltaQ.
    """
    q1 = np.asarray(q1, dtype=np.float64)
    q2 = np.asarray(q2, dtype=np.float64)
    if q1.shape != (4,) or q2.shape != (4,) or not np.all(np.isfinite(q1)) or not np.all(np.isfinite(q2)):
        raise ValueError("q1 and q2 must be finite arrays with shape (4,)")
    q1_norm = float(np.linalg.norm(q1))
    q2_norm = float(np.linalg.norm(q2))
    if q1_norm < eps or q2_norm < eps:
        raise ValueError("q1 and q2 must have non-zero norms")
    q1 = q1 / q1_norm
    q2 = q2 / q2_norm

    q1_inv = tf.quaternion_inverse(q1)
    delta_q = tf.quaternion_multiply(q2, q1_inv)

    return quaternion_to_angle_axis(delta_q, eps)


def apply_delta_pose(
    source_pos: np.ndarray,
    source_rot: np.ndarray,
    delta_pos: np.ndarray,
    delta_rot: np.ndarray,
    eps: float = 1.0e-6,
) -> tuple[np.ndarray, np.ndarray]:
    """Applies delta pose transformation on source pose using NumPy.

    Args:
        source_pos: Position of source frame. Shape is (3,).
        source_rot: Quaternion orientation of source frame in (w, x, y, z). Shape is (4,).
        delta_pos: Cartesian position displacement. Shape is (3,).
        delta_rot: Orientation displacement in angle-axis format. Shape is (3,).
        eps: The tolerance to consider orientation displacement as zero. Defaults to 1.0e-6.

    Returns:
        A tuple containing the displaced position and orientation frames.
        target_pos: Shape is (3,).
        target_rot: Shape is (4,).
    """
    if not (
        isinstance(source_pos, np.ndarray)
        and source_pos.shape == (3,)
        and isinstance(source_rot, np.ndarray)
        and source_rot.shape == (4,)
        and isinstance(delta_pos, np.ndarray)
        and delta_pos.shape == (3,)
        and isinstance(delta_rot, np.ndarray)
        and delta_rot.shape == (3,)
    ):
        raise ValueError(
            "Inputs must be 1D NumPy arrays with shapes: "
            "source_pos (3,), source_rot (4,), delta_pos (3,), delta_rot (3,)."
        )

    # Calculate target position
    target_pos = source_pos + delta_pos

    # Calculate target rotation
    angle = np.linalg.norm(delta_rot)
    rot_delta_quat: np.ndarray
    if angle > eps:
        axis = delta_rot / angle
        rot_delta_quat = tf.quaternion_about_axis(angle, axis)
    else:
        rot_delta_quat = np.array([1.0, 0.0, 0.0, 0.0])

    target_rot = tf.quaternion_multiply(rot_delta_quat, source_rot)

    return target_pos, target_rot
