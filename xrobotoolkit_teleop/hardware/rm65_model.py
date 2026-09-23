"""Pure model metadata and feedback safety checks for the RM65-B arm."""

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class RM65ModelSpec:
    product_prefix: str
    joint_limit_rad: np.ndarray
    max_joint_velocity_rad_s: np.ndarray
    max_tcp_linear_velocity_m_s: float
    reach_m: float

    def __post_init__(self) -> None:
        if not isinstance(self.product_prefix, str) or not self.product_prefix.strip():
            raise ValueError("product_prefix must be a non-empty string")
        object.__setattr__(self, "product_prefix", self.product_prefix.strip())

        joint_limit_rad = self._copy_array(self.joint_limit_rad, "joint_limit_rad")
        if joint_limit_rad.shape != (6, 2):
            raise ValueError("joint_limit_rad must have shape (6, 2)")
        if not np.all(np.isfinite(joint_limit_rad)):
            raise ValueError("joint_limit_rad must contain only finite values")
        if np.any(joint_limit_rad[:, 0] >= joint_limit_rad[:, 1]):
            raise ValueError("joint_limit_rad lower values must be strictly below upper values")

        max_joint_velocity_rad_s = self._copy_array(
            self.max_joint_velocity_rad_s, "max_joint_velocity_rad_s"
        )
        if max_joint_velocity_rad_s.shape != (6,):
            raise ValueError("max_joint_velocity_rad_s must have shape (6,)")
        if not np.all(np.isfinite(max_joint_velocity_rad_s)) or np.any(
            max_joint_velocity_rad_s <= 0.0
        ):
            raise ValueError("max_joint_velocity_rad_s must contain finite positive values")

        try:
            max_tcp_linear_velocity_m_s = float(self.max_tcp_linear_velocity_m_s)
            reach_m = float(self.reach_m)
        except (TypeError, ValueError) as exc:
            raise ValueError("TCP speed and reach must be finite positive values") from exc
        if not np.isfinite(max_tcp_linear_velocity_m_s) or max_tcp_linear_velocity_m_s <= 0.0:
            raise ValueError("max_tcp_linear_velocity_m_s must be finite and positive")
        if not np.isfinite(reach_m) or reach_m <= 0.0:
            raise ValueError("reach_m must be finite and positive")

        object.__setattr__(self, "joint_limit_rad", joint_limit_rad)
        object.__setattr__(self, "max_joint_velocity_rad_s", max_joint_velocity_rad_s)
        object.__setattr__(self, "max_tcp_linear_velocity_m_s", max_tcp_linear_velocity_m_s)
        object.__setattr__(self, "reach_m", reach_m)

    @staticmethod
    def _copy_array(value: np.ndarray, name: str) -> np.ndarray:
        try:
            copied = np.array(value, dtype=float, copy=True)
        except (TypeError, ValueError) as exc:
            raise ValueError(f"{name} must be a numeric array") from exc
        copied.setflags(write=False)
        return copied

    @staticmethod
    def _feedback_array(joint_rad: np.ndarray) -> np.ndarray:
        try:
            feedback = np.asarray(joint_rad, dtype=float)
        except (TypeError, ValueError) as exc:
            raise ValueError("joint_rad must be a numeric array with shape (6,)") from exc
        if feedback.shape != (6,):
            raise ValueError("joint_rad must have shape (6,)")
        if not np.all(np.isfinite(feedback)):
            raise ValueError("joint_rad must contain only finite values")
        return feedback

    @staticmethod
    def _non_negative_finite(value: float, name: str) -> float:
        try:
            result = float(value)
        except (TypeError, ValueError) as exc:
            raise ValueError(f"{name} must be finite and non-negative") from exc
        if not np.isfinite(result) or result < 0.0:
            raise ValueError(f"{name} must be finite and non-negative")
        return result

    def matches_product(self, product_version: str) -> bool:
        if not isinstance(product_version, str):
            return False
        return product_version.strip().casefold().startswith(self.product_prefix.casefold())

    def validate_joint_feedback(
        self, joint_rad: np.ndarray, tolerance_rad: float
    ) -> tuple[bool, str]:
        tolerance = self._non_negative_finite(tolerance_rad, "tolerance_rad")
        feedback = self._feedback_array(joint_rad)
        lower = self.joint_limit_rad[:, 0] - tolerance
        upper = self.joint_limit_rad[:, 1] + tolerance
        violations = np.flatnonzero((feedback < lower) | (feedback > upper))
        if violations.size:
            joint_index = int(violations[0])
            return False, f"J{joint_index + 1} joint feedback outside RM65-B limits"
        return True, ""

    def singularity_warnings(
        self, joint_rad: np.ndarray, warning_rad: float
    ) -> tuple[str, ...]:
        warning = self._non_negative_finite(warning_rad, "warning_rad")
        try:
            feedback = self._feedback_array(joint_rad)
        except ValueError:
            return ()
        warnings = []
        if abs(float(feedback[2])) <= warning:
            warnings.append("J3 near 0 deg")
        if abs(float(feedback[4])) <= warning:
            warnings.append("J5 near 0 deg")
        return tuple(warnings)


RM65_B_SPEC = RM65ModelSpec(
    product_prefix="RM65-B",
    joint_limit_rad=np.deg2rad(
        np.array(
            [
                [-178, 178],
                [-130, 130],
                [-135, 135],
                [-178, 178],
                [-128, 128],
                [-360, 360],
            ],
            dtype=float,
        )
    ),
    max_joint_velocity_rad_s=np.deg2rad(
        np.array([180, 180, 225, 225, 225, 225], dtype=float)
    ),
    max_tcp_linear_velocity_m_s=1.8,
    reach_m=0.610,
)
