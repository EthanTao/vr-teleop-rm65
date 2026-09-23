"""RM65 Meshcat simulation driven by RealMan's official continuous IK."""

from __future__ import annotations

import math
import time
from typing import Any

import meshcat.transformations as tf
import numpy as np

from xrobotoolkit_teleop.hardware.realman_official_ik import (
    RealmanOfficialRemoteIK,
)
from xrobotoolkit_teleop.simulation.placo_teleop_controller import (
    PlacoTeleopController,
)
from xrobotoolkit_teleop.simulation.realman_official_ik_guard import (
    OfficialIKSimulationGuard,
)


class RealmanOfficialIKSimulationController(PlacoTeleopController):
    """Use official RM65 FK/IK while retaining the URDF only for visualization.

    Placo still owns the robot wrapper, Meshcat scene and target-frame objects,
    but its kinematics solver is not called in this mode.
    """

    def __init__(
        self,
        robot_urdf_path: str,
        manipulator_config: dict[str, dict[str, Any]],
        *,
        official_ik_tool_or_work: int = 1,
        official_ik_j3_exclusion_deg: float = 5.0,
        official_ik_max_joint_speed_ratio: float = 0.10,
        ik_solver: Any | None = None,
        **kwargs,
    ) -> None:
        if len(manipulator_config) != 1:
            raise ValueError("official RM65 simulation requires exactly one manipulator")
        if kwargs.get("enable_self_collision_avoidance", False):
            raise ValueError("Placo self-collision tasks are unavailable with the official IK backend")
        if kwargs.get("enable_velocity_limit", False):
            raise ValueError("use official_ik_max_joint_speed_ratio instead of the Placo velocity limiter")
        self.joint_guard = OfficialIKSimulationGuard(
            j3_exclusion_deg=official_ik_j3_exclusion_deg,
            max_joint_speed_ratio=official_ik_max_joint_speed_ratio,
        )
        self._official_link_names = {config["link_name"] for config in manipulator_config.values()}
        dt = float(kwargs.get("dt", 0.01))
        self.official_ik_solver = ik_solver or RealmanOfficialRemoteIK(
            control_period_s=dt,
            tool_or_work=official_ik_tool_or_work,
        )
        self.last_official_ik_error: str | None = None
        self._last_official_ik_error_t = -math.inf
        super().__init__(
            robot_urdf_path=robot_urdf_path,
            manipulator_config=manipulator_config,
            **kwargs,
        )
        initial_q = np.asarray(self.placo_robot.state.q[7:], dtype=float)
        if abs(float(initial_q[2])) < self.joint_guard.j3_exclusion_rad:
            raise ValueError("home_joint_deg J3 must stay outside the official IK zero-angle exclusion zone")

    def _get_link_pose(self, link_name):
        if link_name in self._official_link_names:
            joint_rad = np.asarray(self.placo_robot.state.q[7:], dtype=float)
            return self.official_ik_solver.forward(joint_rad)
        return super()._get_link_pose(link_name)

    def _target_pose(self, name: str, seed_joint_rad: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        if self.effector_control_mode[name] == "position":
            target_xyz = np.asarray(self.effector_task[name].target_world, dtype=float)
            _, target_quat = self.official_ik_solver.forward(seed_joint_rad)
            return target_xyz, target_quat
        target = np.asarray(self.effector_task[name].T_world_frame, dtype=float)
        if target.shape != (4, 4) or not np.all(np.isfinite(target)):
            raise ValueError("official IK target frame must be a finite 4x4 matrix")
        return target[:3, 3].copy(), tf.quaternion_from_matrix(target)

    def _report_official_ik_error(self, reason: str) -> None:
        now = time.monotonic()
        if reason != self.last_official_ik_error or now - self._last_official_ik_error_t >= 1.0:
            print(f"[OFFICIAL IK HOLD] {reason}")
            self._last_official_ik_error_t = now
        self.last_official_ik_error = reason

    def _apply_joint_constraints(self):
        """Update the URDF display without modifying the guarded official IK output."""
        self.placo_robot.update_kinematics()

    def _solve_ik(self):
        for name in self.manipulator_config:
            if not self.active.get(name, False):
                continue
            # This state is the last accepted simulated command (or the current
            # reset state). A rejected candidate must never become the next seed.
            seed = np.asarray(self.placo_robot.state.q[7:], dtype=float).copy()
            try:
                target_xyz, target_quat = self._target_pose(name, seed)
                target = self.official_ik_solver.solve(target_xyz, target_quat, seed.copy())
            except Exception as exc:
                self._report_official_ik_error(f"official RM65 IK failed: {exc}")
                continue
            valid, reason = self.joint_guard.validate(seed, target, self.dt)
            if not valid:
                self._report_official_ik_error(reason)
                continue
            self.placo_robot.state.q[7:] = np.asarray(target, dtype=float)
            self.last_official_ik_error = None
