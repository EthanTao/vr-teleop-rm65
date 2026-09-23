"""Optional adapter for RealMan's model-specific continuous IK solver."""

from __future__ import annotations

from ctypes import c_float
import importlib
import sys
from typing import Any, Protocol

import numpy as np


class RM65IKSolver(Protocol):
    """Minimal solver contract used by the fail-closed hardware controller."""

    def solve(
        self,
        target_xyz_m: np.ndarray,
        target_quat_wxyz: np.ndarray,
        seed_joint_rad: np.ndarray,
    ) -> np.ndarray:
        """Return six target joints in radians or raise on failure."""


class RealmanOfficialIKError(RuntimeError):
    """Raised when the official SDK is unavailable or its IK call fails."""


class RealmanOfficialRemoteIK:
    """Use RealMan API2 ``rm_algo_ik_remote`` without opening a control socket.

    The target pose and seed are supplied by the existing TCP/UDP control path.
    The SDK is imported lazily so the established Cartesian modes do not gain a
    mandatory binary dependency.
    """

    _ERRORS = {
        -4: "unknown robot type",
        -2: "solution exceeds IK limits",
        -1: "inverse kinematics failed",
    }

    def __init__(
        self,
        control_period_s: float,
        tool_or_work: int = 1,
        *,
        sdk_module: Any | None = None,
        fk_only: bool = False,
    ) -> None:
        period = float(control_period_s)
        if not np.isfinite(period) or period <= 0.0:
            raise ValueError("control_period_s must be finite and positive")
        if isinstance(tool_or_work, bool) or tool_or_work not in {0, 1}:
            raise ValueError("tool_or_work must be 0 (tool) or 1 (work)")

        if sdk_module is None:
            try:
                sdk_module = importlib.import_module("Robotic_Arm.rm_robot_interface")
            except ImportError as exc:
                raise RealmanOfficialIKError(
                    "official RealMan Python SDK is required for official FK / IK motion modes "
                    "(including default DLS dry-run). Install in the runtime container with: "
                    f"{sys.executable} -m pip install Robotic_Arm"
                ) from exc

        self._sdk_module = sdk_module
        self._fk_only = fk_only
        try:
            arm_model = sdk_module.rm_robot_arm_model_e.RM_MODEL_RM_65_E
            force_type = sdk_module.rm_force_type_e.RM_MODEL_RM_B_E
            self._matrix_type = sdk_module.rm_Mat_t
            self._algo = sdk_module.Algo(arm_model, force_type)
        except (AttributeError, TypeError) as exc:
            raise RealmanOfficialIKError("installed RealMan SDK does not expose the required API2 IK symbols") from exc

        if fk_only:
            if not callable(getattr(self._algo, "rm_algo_forward_kinematics", None)):
                raise RealmanOfficialIKError("installed RealMan SDK does not provide official FK")
            return
        init = getattr(self._algo, "rm_algo_ik_remote_init", None)
        solve = getattr(self._algo, "rm_algo_ik_remote", None)
        if not callable(init) or not callable(solve):
            raise RealmanOfficialIKError("installed RealMan SDK does not provide rm_algo_ik_remote")
        init(period, int(tool_or_work))

    def configure_controller_dh(self, response: dict) -> None:
        """Configure only the local FK library from a read-only get_DH_data reply."""
        if not self._fk_only:
            raise RealmanOfficialIKError("controller DH configuration requires an FK-only adapter")
        if not isinstance(response, dict) or response.get("command") != "get_DH_data":
            raise RealmanOfficialIKError("missing controller DH response")
        rows = self._finite_array([response.get(f"joint_{i}") for i in range(1, 7)], (6, 4), "controller DH")
        dh = self._sdk_module.rm_dh_t(
            alpha=(rows[:, 0] / 1000).tolist(),
            a=(rows[:, 1] / 1e6).tolist(),
            d=(rows[:, 2] / 1e6).tolist(),
            offset=(rows[:, 3] / 1000).tolist(),
        )
        self._algo.rm_algo_set_dh(dh)
        actual = self._algo.rm_algo_get_dh()
        for name, column, scale in (("alpha", 0, 1000), ("a", 1, 1e6), ("d", 2, 1e6), ("offset", 3, 1000)):
            readback = self._finite_array(actual.get(name), (6,), f"SDK DH {name}")
            if not np.allclose(readback, rows[:, column] / scale, rtol=1e-6, atol=1e-7):
                raise RealmanOfficialIKError(f"local SDK DH readback mismatch: {name}")

    def controller_tool_pose(self, response: dict) -> list[float]:
        """Convert controller micrometres/milliradian Euler pose to metres/wxyz."""
        if not isinstance(response, dict) or response.get("state") != "current_tool_frame":
            raise RealmanOfficialIKError("missing controller tool frame response")
        pose = self._finite_array(response.get("pose"), (6,), "controller tool pose")
        quat = self._finite_array(
            self._algo.rm_algo_euler2quaternion((pose[3:] / 1000).tolist()), (4,), "tool quaternion"
        )
        norm = float(np.linalg.norm(quat))
        if norm < 1e-12:
            raise RealmanOfficialIKError("controller tool conversion returned a zero quaternion")
        return [*(pose[:3] / 1e6), *(quat / norm)]

    @staticmethod
    def _finite_array(value: np.ndarray, shape: tuple[int, ...], name: str) -> np.ndarray:
        try:
            array = np.asarray(value, dtype=float)
        except (TypeError, ValueError) as exc:
            raise ValueError(f"{name} must be numeric with shape {shape}") from exc
        if array.shape != shape or not np.all(np.isfinite(array)):
            raise ValueError(f"{name} must be finite with shape {shape}")
        return array

    @classmethod
    def _target_matrix(cls, xyz: np.ndarray, quat: np.ndarray) -> np.ndarray:
        xyz = cls._finite_array(xyz, (3,), "target_xyz_m")
        quat = cls._finite_array(quat, (4,), "target_quat_wxyz")
        norm = float(np.linalg.norm(quat))
        if norm < 1e-12:
            raise ValueError("target_quat_wxyz must be non-zero")
        w, x, y, z = quat / norm
        rotation = np.array(
            [
                [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
                [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
                [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
            ],
            dtype=float,
        )
        transform = np.eye(4, dtype=float)
        transform[:3, :3] = rotation
        transform[:3, 3] = xyz
        return transform

    def solve(
        self,
        target_xyz_m: np.ndarray,
        target_quat_wxyz: np.ndarray,
        seed_joint_rad: np.ndarray,
    ) -> np.ndarray:
        seed = self._finite_array(seed_joint_rad, (6,), "seed_joint_rad")
        transform = self._target_matrix(target_xyz_m, target_quat_wxyz)
        matrix = self._matrix_type()
        matrix.row = 4
        matrix.col = 4
        for row in range(4):
            for col in range(4):
                matrix.data[row][col] = float(transform[row, col])

        output = (c_float * 6)()
        result = int(
            self._algo.rm_algo_ik_remote(
                matrix,
                np.rad2deg(seed).tolist(),
                output,
            )
        )
        if result != 0:
            detail = self._ERRORS.get(result, f"error code {result}")
            raise RealmanOfficialIKError(f"RealMan continuous IK failed: {detail}")
        joint_rad = np.deg2rad(np.array(list(output), dtype=float))
        if joint_rad.shape != (6,) or not np.all(np.isfinite(joint_rad)):
            raise RealmanOfficialIKError("RealMan continuous IK returned an invalid joint target")
        return joint_rad

    def forward(self, joint_rad: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """Return official RM65 end-effector pose as xyz metres and wxyz quaternion."""
        joint = self._finite_array(joint_rad, (6,), "joint_rad")
        forward = getattr(self._algo, "rm_algo_forward_kinematics", None)
        if not callable(forward):
            raise RealmanOfficialIKError("installed RealMan SDK does not provide rm_algo_forward_kinematics")
        try:
            pose = np.asarray(forward(np.rad2deg(joint).tolist(), 0), dtype=float)
        except Exception as exc:
            raise RealmanOfficialIKError(f"RealMan forward kinematics failed: {exc}") from exc
        if pose.shape != (7,) or not np.all(np.isfinite(pose)):
            raise RealmanOfficialIKError("RealMan forward kinematics returned an invalid quaternion pose")
        xyz = pose[:3].copy()
        quat = pose[3:].copy()
        norm = float(np.linalg.norm(quat))
        if norm < 1e-12:
            raise RealmanOfficialIKError("RealMan forward kinematics returned a zero quaternion")
        return xyz, quat / norm

    def forward_batch(self, joints: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """Evaluate an ``(N, 6)`` joint block, returning ``(3, N)`` and ``(4, N)``.

        The SDK call is unchanged and still one call per pose; this only lets the
        caller prepare and post-process the whole block at once, which is what the
        realtime container is actually slow at.
        """
        try:
            block = np.asarray(joints, dtype=float)
        except (TypeError, ValueError) as exc:
            raise ValueError("joints must be a finite (N, 6) array") from exc
        if block.ndim != 2 or block.shape[1] != 6 or not np.all(np.isfinite(block)):
            raise ValueError("joints must be a finite (N, 6) array")
        xyz = np.empty((3, len(block)))
        quat = np.empty((4, len(block)))
        for index, row in enumerate(block):
            pose_xyz, pose_quat = self.forward(row)
            xyz[:, index] = pose_xyz
            quat[:, index] = pose_quat
        return xyz, quat
