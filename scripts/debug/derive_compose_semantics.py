"""Decide, offline, whether compose_pose implements frame composition correctly.

The deployed chain is ``T_work_tcp = T_work_base @ T_base_flange @ T_flange_tcp``.
Written out with positions ``p`` and rotations ``R``:

    p_work = p_workbase + R_workbase @ ( p_flange + R_flange @ p_tool )

``ControllerFrameKinematics.forward`` composes ``compose_pose(work_from_base, pose)``
and then ``compose_pose(pose, flange_to_tcp)``. This script checks the deployed
composition against the chain above by comparing quaternion algebra with an explicit
rotation-matrix reference, and reports the exact condition under which they differ.

Usage:
    python scripts/debug/derive_compose_semantics.py
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from xrobotoolkit_teleop.hardware.damped_ik import compose_pose  # noqa: E402


def quaternion_matrix(quat) -> np.ndarray:
    """Rotation matrix for a wxyz quaternion (no library dependency)."""
    w, x, y, z = (float(v) for v in quat)
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def mat_multiply(left, right) -> np.ndarray:
    lw, lx, ly, lz = (float(v) for v in left)
    rw, rx, ry, rz = (float(v) for v in right)
    return np.array(
        [
            lw * rw - (lx * rx + ly * ry + lz * rz),
            lw * rx + rw * lx + (ly * rz - lz * ry),
            lw * ry + rw * ly + (lz * rx - lx * rz),
            lw * rz + rw * lz + (lx * ry - ly * rx),
        ]
    )


def chain_reference(work, flange, tool):
    """Correct chain: p = p_wb + R_wb @ (p_flange + R_flange @ p_tool)."""
    rotation_work = quaternion_matrix(work[1])
    rotation_flange = quaternion_matrix(flange[1])
    position = work[0] + rotation_work @ (flange[0] + rotation_flange @ tool[0])
    quaternion = mat_multiply(mat_multiply(work[1], flange[1]), tool[1])
    return position, quaternion


def deployed_two_step(work, flange, tool):
    """Exactly what ControllerFrameKinematics.forward does."""
    first = compose_pose(work, flange)
    second = compose_pose(first, tool)
    return np.asarray(second[0], dtype=float), np.asarray(second[1], dtype=float)


def main() -> int:
    rng = np.random.default_rng(23)
    worst = 0.0
    worst_only_work_identity = 0.0
    differing = 0
    cases = 0

    for trial in range(3000):

        def random_frame(identity):
            if identity:
                return np.zeros(3), np.array([1.0, 0.0, 0.0, 0.0])
            raw = rng.normal(size=4)
            return rng.normal(size=3), raw / np.linalg.norm(raw)

        work = random_frame(trial % 3 == 0)  # every third case: identity work frame
        flange = random_frame(False)
        tool = random_frame(trial % 5 == 0)  # every fifth case: identity tool frame

        expected_position, expected_quaternion = chain_reference(work, flange, tool)
        actual_position, actual_quaternion = deployed_two_step(work, flange, tool)

        position_error = float(np.max(np.abs(actual_position - expected_position)))
        cases += 1
        if position_error > 1e-9:
            differing += 1
            worst = max(worst, position_error)
        if trial % 3 == 0:
            worst_only_work_identity = max(worst_only_work_identity, position_error)
        quaternion_error = min(
            float(np.max(np.abs(actual_quaternion - expected_quaternion))),
            float(np.max(np.abs(actual_quaternion + expected_quaternion))),
        )
        if quaternion_error > 1e-9:
            print(f"  quaternion mismatch at trial {trial}: {quaternion_error:.3e}")

    print(f"cases={cases} differing={differing} worst position error={worst:.6f} m")
    print(f"worst error when the work frame is IDENTITY: {worst_only_work_identity:.3e} m")
    if worst_only_work_identity < 1e-9 and worst > 1e-3:
        print(
            "\nCONFIRMED: the deployed two-step composition matches the chain only when the\n"
            "work frame is identity. With a rotated work frame it offsets the tool pose by\n"
            "an extra R(work) factor, i.e. up to |R(work) @ p_tool - p_tool|."
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
