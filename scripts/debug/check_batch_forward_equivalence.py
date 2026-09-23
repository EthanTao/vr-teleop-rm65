"""Check that the vectorised pose batch equals the scalar pose loop, exactly.

``ControllerFrameKinematics.forward_batch`` exists to cut the per-pose Python
overhead of the DLS Jacobian stencil on the realtime container. It must produce the
same poses as the scalar path (bit-for-bit for the FK and frame composition), and
it must refuse the same invalid inputs.

Usage:
    python scripts/debug/check_batch_forward_equivalence.py
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from xrobotoolkit_teleop.hardware.damped_ik import ControllerFrameKinematics  # noqa: E402
from xrobotoolkit_teleop.hardware.singularity_avoidance import (  # noqa: E402
    RM65_CHAIN,
    rm65_forward_kinematics,
)


def _pose(joint_rad):
    matrix = rm65_forward_kinematics(np.asarray(joint_rad, dtype=float), RM65_CHAIN)
    trace = matrix[0, 0] + matrix[1, 1] + matrix[2, 2]
    w = np.sqrt(max(0.0, 1.0 + trace)) / 2.0
    quat = np.array(
        [
            w,
            (matrix[2, 1] - matrix[1, 2]) / (4.0 * w),
            (matrix[0, 2] - matrix[2, 0]) / (4.0 * w),
            (matrix[1, 0] - matrix[0, 1]) / (4.0 * w),
        ]
    )
    return matrix[:3, 3].copy(), quat / np.linalg.norm(quat)


class BatchKinematics:
    """Official-adapter twin: scalar FK plus the batched entry point."""

    def forward(self, joint_rad):
        return _pose(joint_rad)

    def forward_batch(self, joints):
        block = np.asarray(joints, dtype=float)
        xyz = np.empty((3, len(block)))
        quat = np.empty((4, len(block)))
        for index, row in enumerate(block):
            pose_xyz, pose_quat = self.forward(row)
            xyz[:, index] = pose_xyz
            quat[:, index] = pose_quat
        return xyz, quat


def main() -> int:
    rng = np.random.default_rng(7)
    frames = [
        (None, None),
        (None, [0.01, 0.02, 0.1612, 1.0, 0.0, 0.0, 0.0]),
        ([0.3, -0.2, 0.5, 0.9238795, 0.0, 0.3826834, 0.0], [0.01, 0.02, 0.1612, 1.0, 0.0, 0.0, 0.0]),
    ]
    failures = 0
    comparisons = 0
    for work, tool in frames:
        kinematics = ControllerFrameKinematics(BatchKinematics(), work, tool)
        for _ in range(60):
            block = rng.uniform(-3.0, 3.0, size=(13, 6))
            batch_xyz, batch_quat = kinematics.forward_batch(block)
            for column in range(len(block)):
                scalar_xyz, scalar_quat = kinematics.forward(block[column])
                comparisons += 1
                if not np.array_equal(batch_xyz[:, column], scalar_xyz):
                    failures += 1
                    print(f"  xyz mismatch work={work} column={column}")
                if not np.array_equal(batch_quat[:, column], scalar_quat):
                    failures += 1
                    print(f"  quat mismatch work={work} column={column}")

    # invalid inputs must be rejected by both paths
    kinematics = ControllerFrameKinematics(BatchKinematics(), None, None)
    for bad in (np.zeros((13, 5)), np.zeros(6), np.full((13, 6), np.nan), np.zeros((13, 6, 1))):
        try:
            kinematics.forward_batch(bad)
            rejected = False
        except ValueError:
            rejected = True
        comparisons += 1
        if not rejected:
            failures += 1
            print(f"  forward_batch accepted invalid input of shape {np.shape(bad)}")

    print(f"pose comparisons={comparisons} failures={failures}")
    if failures:
        print("NOT equivalent: the batch path must not be used")
        return 1
    print("batch poses are bit-identical to the scalar loop")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
