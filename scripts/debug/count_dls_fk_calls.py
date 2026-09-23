"""Count official-FK calls per control cycle inside the DLS path (read-only analysis)."""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from xrobotoolkit_teleop.hardware.damped_ik import ControllerFrameKinematics, DampedIK  # noqa: E402
from xrobotoolkit_teleop.hardware.singularity_avoidance import (  # noqa: E402
    RM65_CHAIN,
    rm65_forward_kinematics,
)


class CountingKinematics:
    """Same contract as RealmanOfficialRemoteIK.forward, with a call counter."""

    def __init__(self):
        self.calls = 0

    def forward(self, joint_rad):
        self.calls += 1
        matrix = rm65_forward_kinematics(np.asarray(joint_rad, dtype=float), RM65_CHAIN)
        quat = np.empty(4)
        # matrix -> wxyz
        trace = matrix[0, 0] + matrix[1, 1] + matrix[2, 2]
        quat[0] = np.sqrt(max(0.0, 1.0 + trace)) / 2.0
        quat[1] = (matrix[2, 1] - matrix[1, 2]) / (4.0 * quat[0]) if quat[0] > 1e-9 else 0.0
        quat[2] = (matrix[0, 2] - matrix[2, 0]) / (4.0 * quat[0]) if quat[0] > 1e-9 else 0.0
        quat[3] = (matrix[1, 0] - matrix[0, 1]) / (4.0 * quat[0]) if quat[0] > 1e-9 else 0.0
        return matrix[:3, 3].copy(), quat / np.linalg.norm(quat)


def count_calls(seed: np.ndarray, target_xyz: np.ndarray, tool_offset_m: float) -> tuple[int, object]:
    kinematics = CountingKinematics()
    frames = ControllerFrameKinematics(kinematics, None, [0.0, 0.0, tool_offset_m, 1.0, 0.0, 0.0, 0.0])
    dls = DampedIK(frames)
    speed_limits = np.array([1.047, 1.047, 1.047, 1.047, 1.047, 1.047]) * 0.10
    dls.begin_cycle()
    kinematics.calls = 0
    result = dls.step(
        target_xyz,
        frames.forward(seed)[1],
        seed,
        0.02,
        speed_limits,
        max_linear_speed=0.05,
        max_angular_speed=0.25,
    )
    return kinematics.calls, result


def main() -> int:
    # Configurations near the reported home TCP height, from the offline search.
    base = np.deg2rad(np.array([-3.65, 30.32, -39.54, 108.02, -0.02, 139.81]))
    for tool_mm in (0.0, 161.2):
        kinematics = CountingKinematics()
        frames = ControllerFrameKinematics(kinematics, None, [0.0, 0.0, tool_mm / 1000.0, 1.0, 0.0, 0.0, 0.0])
        origin, quat = frames.forward(base)
        print(f"tool {tool_mm:.1f} mm: home TCP = {np.round(origin, 4).tolist()}")

        for label, command in (
            ("no operator input (target == anchor)", origin + np.array([1e-6, -1e-6, 1e-6])),
            ("1 mm target step", origin + np.array([0.001, 0.0, 0.0])),
            ("5 mm target step", origin + np.array([0.005, 0.0, 0.0])),
            ("20 mm target step", origin + np.array([0.02, 0.0, 0.0])),
        ):
            calls, result = count_calls(base, command, tool_mm / 1000.0)
            print(
                f"  {label:38s} official-FK calls = {calls:3d}  status={result.status:16s}"
                f" ratio={result.ratio:.5f} scale={result.scale:.3f}"
            )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
