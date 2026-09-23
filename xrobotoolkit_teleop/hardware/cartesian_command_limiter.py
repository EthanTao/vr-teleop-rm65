"""Stateful Cartesian velocity and acceleration limiting for RM65 commands."""

import math

import numpy as np

MAX_SCALAR_VECTOR_LENGTH = 8
"""Vectors up to this length are validated with scalar arithmetic.

Every numpy call costs tens of microseconds on the realtime container (measured:
about 40 us per operation in docs/决策与结论汇总.md §2.10), so the tiny pose/quaternion
vectors must not pay numpy's call overhead. Behaviour is unchanged: the same
shape, finiteness and rejection rules are applied to the same values.
"""


def _finite_vector(value: np.ndarray, size: int, name: str) -> np.ndarray:
    """Return a fresh finite float vector as a numpy array, or raise ValueError."""
    return np.asarray(_finite_tuple(value, size, name), dtype=float)


def _finite_tuple(value, size: int, name: str) -> tuple[float, ...]:
    """Return a fresh finite float tuple of ``size`` items, or raise ValueError."""
    message = f"{name} must be a finite numeric vector with shape ({size},)"
    if size <= MAX_SCALAR_VECTOR_LENGTH:
        candidates: tuple = ()
        try:
            if isinstance(value, np.ndarray) and value.ndim != 1:
                raise ValueError(message)
            candidates = tuple(value)
        except (TypeError, ValueError) as exc:
            raise ValueError(message) from exc
        if len(candidates) != size:
            raise ValueError(message)
        converted = []
        for item in candidates:
            number = float(item)
            if not math.isfinite(number):
                raise ValueError(message)
            converted.append(number)
        return tuple(converted)
    try:
        vector = np.array(value, dtype=float, copy=True)
    except (TypeError, ValueError) as exc:
        raise ValueError(message) from exc
    if vector.shape != (size,) or not np.all(np.isfinite(vector)):
        raise ValueError(message)
    return tuple(float(item) for item in vector)


def _normalized_quaternion(value, name: str) -> tuple[float, float, float, float]:
    quaternion = _finite_tuple(value, 4, name)
    norm = math.sqrt(sum(component * component for component in quaternion))
    if norm == 0.0:
        raise ValueError(f"{name} must be non-zero")
    quaternion = tuple(component / norm for component in quaternion)

    # w >= 0 selects the shortest-angle representation. At exactly 180 degrees,
    # use the first non-zero vector component to make q and -q deterministic.
    for component in quaternion:
        if component > 0.0:
            break
        if component < 0.0:
            quaternion = tuple(-item for item in quaternion)
            break
    return quaternion


def _quaternion_conjugate(quaternion) -> tuple[float, float, float, float]:
    w, x, y, z = (float(item) for item in quaternion)
    return (w, -x, -y, -z)


def _quaternion_multiply(left, right) -> tuple[float, float, float, float]:
    """Hamilton product with scalar arithmetic; numpy costs too much per call here."""
    left_w, left_x, left_y, left_z = (float(item) for item in left)
    right_w, right_x, right_y, right_z = (float(item) for item in right)
    return (
        left_w * right_w - (left_x * right_x + left_y * right_y + left_z * right_z),
        left_w * right_x + right_w * left_x + (left_y * right_z - left_z * right_y),
        left_w * right_y + right_w * left_y + (left_z * right_x - left_x * right_z),
        left_w * right_z + right_w * left_z + (left_x * right_y - left_y * right_x),
    )


def shortest_world_rotation_vector(source, target) -> np.ndarray:
    """Return the shortest world-frame rotation vector from ``source`` to ``target`` (wxyz)."""
    delta = _normalized_quaternion(
        _quaternion_multiply(target, _quaternion_conjugate(source)),
        "relative quaternion",
    )
    vector_norm = math.sqrt(delta[1] * delta[1] + delta[2] * delta[2] + delta[3] * delta[3])
    if vector_norm == 0.0:
        return np.zeros(3, dtype=float)
    angle = 2.0 * math.atan2(vector_norm, delta[0])
    scale = angle / vector_norm
    return np.array([delta[1] * scale, delta[2] * scale, delta[3] * scale], dtype=float)


def _angle_axis_quaternion(rotation_vector: np.ndarray) -> np.ndarray:
    angle = float(np.linalg.norm(rotation_vector))
    if angle == 0.0:
        return np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
    half_angle = 0.5 * angle
    return np.concatenate(([np.cos(half_angle)], rotation_vector * (np.sin(half_angle) / angle)))


def _limited_velocity(
    error: np.ndarray,
    current_velocity: np.ndarray,
    max_velocity: float,
    max_acceleration: float,
    dt: float,
) -> np.ndarray:
    error_norm = float(np.linalg.norm(error))
    if error_norm == 0.0:
        desired_velocity = np.zeros(3, dtype=float)
    else:
        desired_speed = min(max_velocity, np.sqrt(2.0 * max_acceleration * error_norm))
        desired_velocity = error * (desired_speed / error_norm)

    velocity_delta = desired_velocity - current_velocity
    delta_norm = float(np.linalg.norm(velocity_delta))
    max_delta = max_acceleration * dt
    if delta_norm > max_delta:
        velocity_delta *= max_delta / delta_norm
    return current_velocity + velocity_delta


class CartesianCommandLimiter:
    """Limit Cartesian command velocity and acceleration in world coordinates.

    ``set_speed_scale`` scales velocity limits while preserving braking acceleration. It is used by the
    singularity guard and can only ever reduce a limit, never raise it above the configured value.
    """

    def __init__(
        self,
        max_linear_velocity_m_s: float,
        max_linear_acceleration_m_s2: float,
        max_angular_velocity_rad_s: float,
        max_angular_acceleration_rad_s2: float,
    ) -> None:
        self._max_linear_velocity = self._positive_limit(max_linear_velocity_m_s, "max_linear_velocity_m_s")
        self._max_linear_acceleration = self._positive_limit(
            max_linear_acceleration_m_s2, "max_linear_acceleration_m_s2"
        )
        self._max_angular_velocity = self._positive_limit(max_angular_velocity_rad_s, "max_angular_velocity_rad_s")
        self._max_angular_acceleration = self._positive_limit(
            max_angular_acceleration_rad_s2, "max_angular_acceleration_rad_s2"
        )
        self._speed_scale = 1.0
        self._position: np.ndarray | None = None
        self._quaternion: np.ndarray | None = None
        self._linear_velocity = np.zeros(3, dtype=float)
        self._angular_velocity = np.zeros(3, dtype=float)

    @property
    def speed_scale(self) -> float:
        return self._speed_scale

    def set_speed_scale(self, scale: float) -> None:
        """Scale velocity limits by ``scale``; values are clamped to ``[0, 1]``."""
        try:
            value = float(scale)
        except (TypeError, ValueError) as exc:
            raise ValueError("speed scale must be a finite number") from exc
        if not np.isfinite(value):
            raise ValueError("speed scale must be a finite number")
        self._speed_scale = float(min(1.0, max(0.0, value)))

    @staticmethod
    def _positive_limit(value: float, name: str) -> float:
        try:
            result = float(value)
        except (TypeError, ValueError) as exc:
            raise ValueError(f"{name} must be finite and positive") from exc
        if not np.isfinite(result) or result <= 0.0:
            raise ValueError(f"{name} must be finite and positive")
        return result

    @property
    def position(self) -> np.ndarray:
        if self._position is None:
            raise RuntimeError("Cartesian command limiter is not initialized")
        return self._position.copy()

    @property
    def quaternion(self) -> np.ndarray:
        if self._quaternion is None:
            raise RuntimeError("Cartesian command limiter is not initialized")
        return np.asarray(self._quaternion, dtype=float)

    @property
    def linear_velocity(self) -> np.ndarray:
        return self._linear_velocity.copy()

    @property
    def angular_velocity(self) -> np.ndarray:
        return self._angular_velocity.copy()

    def reset(self, xyz: np.ndarray, quat_wxyz: np.ndarray) -> None:
        self._position = _finite_vector(xyz, 3, "xyz")
        self._quaternion = _normalized_quaternion(quat_wxyz, "quat_wxyz")
        self._linear_velocity = np.zeros(3, dtype=float)
        self._angular_velocity = np.zeros(3, dtype=float)

    def rebase(self, xyz, quat_wxyz, linear_velocity, angular_velocity) -> None:
        """Use achieved FK commands instead of accumulating unreachable task error."""
        self.reset(xyz, quat_wxyz)
        self._linear_velocity = _finite_vector(linear_velocity, 3, "linear velocity")
        self._angular_velocity = _finite_vector(angular_velocity, 3, "angular velocity")

    def clear(self) -> None:
        self._position = None
        self._quaternion = None
        self._linear_velocity = np.zeros(3, dtype=float)
        self._angular_velocity = np.zeros(3, dtype=float)

    def step(
        self,
        target_xyz: np.ndarray,
        target_quat_wxyz: np.ndarray,
        dt: float,
    ) -> tuple[np.ndarray, np.ndarray]:
        if self._position is None or self._quaternion is None:
            raise RuntimeError("Cartesian command limiter must be reset before step")
        target_position = _finite_vector(target_xyz, 3, "target_xyz")
        target_quaternion = _normalized_quaternion(target_quat_wxyz, "target_quat_wxyz")
        try:
            timestep = float(dt)
        except (TypeError, ValueError) as exc:
            raise ValueError("dt must be finite and positive") from exc
        if not np.isfinite(timestep) or timestep <= 0.0:
            raise ValueError("dt must be finite and positive")

        position_error = target_position - self._position
        self._linear_velocity = _limited_velocity(
            position_error,
            self._linear_velocity,
            self._max_linear_velocity * self._speed_scale,
            self._max_linear_acceleration,
            timestep,
        )
        self._position = self._position + self._linear_velocity * timestep

        rotation_error = shortest_world_rotation_vector(self._quaternion, target_quaternion)
        self._angular_velocity = _limited_velocity(
            rotation_error,
            self._angular_velocity,
            self._max_angular_velocity * self._speed_scale,
            self._max_angular_acceleration,
            timestep,
        )
        increment = _angle_axis_quaternion(self._angular_velocity * timestep)
        self._quaternion = _normalized_quaternion(
            _quaternion_multiply(increment, self._quaternion),
            "integrated quaternion",
        )
        return self._position.copy(), np.asarray(self._quaternion, dtype=float)
