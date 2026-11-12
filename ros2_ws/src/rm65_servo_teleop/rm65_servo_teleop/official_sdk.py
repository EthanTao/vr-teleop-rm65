"""Compute-only RealMan Algo adapter; never instantiate a robot connection.

One SDK algorithm context per process: the vendor's Algo frame setters operate
on global library state. Run this adapter in its own process, not in rm_driver.
Public interface: metres, radians, xyzw; vendor joints: degrees, quaternion: wxyz.
"""

from __future__ import annotations

import importlib
import threading

import numpy as np

from .input_adapter import Pose


class IKRejected(RuntimeError):
    """The official library did not produce a usable six-joint solution."""


def joints6(value) -> np.ndarray:
    result = np.array(value, dtype=float, copy=True)
    if result.shape != (6,) or not np.all(np.isfinite(result)):
        raise ValueError("expected six finite joint angles in radians")
    return result


class OfficialAlgo:
    """Offline SDK FK/IK with explicitly configured identity work and TCP tool.

    No IP address, robot handle creation, power, stop or motion API is exposed.
    A missing SDK fails closed; numerical substitutes are never selected.
    """

    _context_lock = threading.Lock()
    _context_claimed = False

    def __init__(self, tool_xyz, tool_rpy, *, d6_m=None, sdk=None):
        xyz, rpy = np.asarray(tool_xyz, dtype=float), np.asarray(tool_rpy, dtype=float)
        if xyz.shape != (3,) or rpy.shape != (3,) or not np.all(np.isfinite(np.r_[xyz, rpy])):
            raise ValueError("tool translation and RPY must be three finite metres/radians")
        if d6_m is not None and (not np.isfinite(d6_m) or not 0.05 < d6_m < 0.3):
            raise ValueError("d6_m must match the shared model, in metres")
        self.d6_m = d6_m
        self.sdk = sdk if sdk is not None else importlib.import_module("Robotic_Arm.rm_robot_interface")
        with self._context_lock:
            if OfficialAlgo._context_claimed:
                raise RuntimeError("Algo has process-global state: use a dedicated process per context")
            self.algo = self.sdk.Algo(
                self.sdk.rm_robot_arm_model_e.RM_MODEL_RM_65_E,
                self.sdk.rm_force_type_e.RM_MODEL_RM_B_E,
            )
            # Even partial initialization changes vendor global state. Never
            # silently create a second context after an initialization failure.
            OfficialAlgo._context_claimed = True
            if d6_m is not None:
                dh = self.algo.rm_algo_get_dh()
                if set(dh) != {"d", "a", "alpha", "offset"} or any(len(v) != 6 for v in dh.values()):
                    raise RuntimeError("SDK did not return the expected six-axis DH schema")
                dh["d"][5] = float(d6_m)
                self.algo.rm_algo_set_dh(self.sdk.rm_dh_t(**dh))
                if abs(self.algo.rm_algo_get_dh()["d"][5] - d6_m) > 1e-6:
                    raise RuntimeError("SDK did not apply the shared d6 geometry")
            self.algo.rm_algo_set_workframe(self.sdk.rm_frame_t("base", (0.0,) * 6))
            self.algo.rm_algo_set_toolframe(self.sdk.rm_frame_t("tcp", tuple(np.r_[xyz, rpy])))

    def solve(self, target: Pose, seed) -> np.ndarray:
        seed_deg = np.rad2deg(joints6(seed)).tolist()
        x, y, z, w = target.quaternion_xyzw
        params = self.sdk.rm_inverse_kinematics_params_t(
            seed_deg, [*target.xyz.tolist(), float(w), float(x), float(y), float(z)], 0
        )
        with self._context_lock:
            code, result = self.algo.rm_algo_inverse_kinematics(params)
        if type(code) is not int or code != 0:
            raise IKRejected(f"official_inverse_kinematics_code_{code}")
        return joints6(np.deg2rad(result))

    def forward(self, joints) -> Pose:
        with self._context_lock:
            value = self.algo.rm_algo_forward_kinematics(np.rad2deg(joints6(joints)).tolist(), 0)
        value = np.asarray(value, dtype=float)
        if value.shape != (7,) or not np.all(np.isfinite(value)):
            raise IKRejected("official_forward_kinematics_invalid")
        # Official flag=0 returns x,y,z,w,x,y,z, unlike OpenXR xyzw.
        return Pose(value[:3], value[[4, 5, 6, 3]])

    @property
    def identity(self) -> dict:
        return {
            "adapter": "RealMan Algo (no connection)",
            "module_file": str(getattr(self.sdk, "__file__", "injected-test-double")),
            "model": "RM65-B",
            "d6_m": self.d6_m,
            "joint_unit": "rad_public_deg_sdk",
            "pose_frame": "base_to_configured_tcp",
            "finite_deadline_preemption": False,
        }
