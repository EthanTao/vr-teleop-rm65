"""Pure safety checks shared by the official-IK simulation path."""

from __future__ import annotations

import math

import numpy as np

from xrobotoolkit_teleop.hardware.rm65_model import RM65_B_SPEC


class OfficialIKSimulationGuard:
    def __init__(
        self,
        j3_exclusion_deg: float = 5.0,
        max_joint_speed_ratio: float = 0.10,
    ) -> None:
        self.j3_exclusion_rad = math.radians(
            self._positive_finite(j3_exclusion_deg, "j3_exclusion_deg")
        )
        self.max_joint_speed_ratio = self._positive_finite(
            max_joint_speed_ratio,
            "max_joint_speed_ratio",
        )
        if self.max_joint_speed_ratio > 1.0:
            raise ValueError("max_joint_speed_ratio must not exceed 1.0")

    @staticmethod
    def _positive_finite(value: float, name: str) -> float:
        try:
            result = float(value)
        except (TypeError, ValueError) as exc:
            raise ValueError(f"{name} must be finite and positive") from exc
        if not math.isfinite(result) or result <= 0.0:
            raise ValueError(f"{name} must be finite and positive")
        return result

    def validate(
        self,
        seed_joint_rad: np.ndarray,
        target_joint_rad: np.ndarray,
        dt: float,
    ) -> tuple[bool, str]:
        try:
            seed = np.asarray(seed_joint_rad, dtype=float)
            target = np.asarray(target_joint_rad, dtype=float)
        except (TypeError, ValueError):
            return False, "official IK joint target is not numeric"
        if (
            seed.shape != (6,)
            or target.shape != (6,)
            or not np.all(np.isfinite(seed))
            or not np.all(np.isfinite(target))
        ):
            return False, "official IK seed and target must be finite six-joint arrays"
        valid, reason = RM65_B_SPEC.validate_joint_feedback(target, 0.0)
        if not valid:
            return False, reason
        if abs(float(target[2])) < self.j3_exclusion_rad:
            return False, "official IK target entered the J3 zero-angle exclusion zone"
        max_step = (
            RM65_B_SPEC.max_joint_velocity_rad_s
            * self.max_joint_speed_ratio
            * self._positive_finite(dt, "dt")
        )
        violations = np.flatnonzero(np.abs(target - seed) > max_step + 1e-9)
        if violations.size:
            index = int(violations[0])
            return False, f"official IK J{index + 1} single-cycle step exceeded the simulation limit"
        return True, ""
