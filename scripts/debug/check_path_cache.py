"""Verify the per-cycle path-verdict cache removes the duplicate safety check.

``DampedIK.step`` already runs ``path_allowed(seed, candidate)`` internally, and
``RealmanRM65SafeTeleopController._send_damped_motion`` repeats the same question
with the same ``(seed, target)`` pair before sending. The cache must make the
second ask free while returning the identical verdict, and must not leak across
cycles or across different pairs.

Usage:
    python scripts/debug/check_path_cache.py
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from xrobotoolkit_teleop.hardware.damped_ik import ControllerFrameKinematics, DampedIK  # noqa: E402
from xrobotoolkit_teleop.hardware.rm65_model import RM65_B_SPEC  # noqa: E402
from xrobotoolkit_teleop.hardware.singularity_avoidance import (  # noqa: E402
    RM65_CHAIN,
    rm65_forward_kinematics,
)


class CountingKinematics:
    def __init__(self):
        self.calls = 0

    def forward(self, joint_rad):
        self.calls += 1
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


def main() -> int:
    kinematics = CountingKinematics()
    solver = DampedIK(ControllerFrameKinematics(kinematics, None, [0.0, 0.0, 0.1612, 1.0, 0.0, 0.0, 0.0]))
    seed = np.deg2rad(np.array([19.625, -31.532, 55.599, -12.3, -59.301, -98.92]))
    target = seed + np.deg2rad(np.array([0.02, 0.0, 0.0, 0.0, 0.0, 0.0]))

    solver.begin_cycle()
    before = kinematics.calls
    first = solver.path_allowed(seed, target)
    after_first = kinematics.calls
    second = solver.path_allowed(seed, target)
    after_second = kinematics.calls
    print(f"first call : verdict={first}  FK calls={after_first - before}")
    print(f"second call: verdict={second} FK calls={after_second - after_first}")
    assert first == second, "the duplicate ask changed the verdict"
    assert after_second == after_first, "the duplicate ask was not served from the cache"

    # A different pair must still be evaluated for real.
    other = seed + np.deg2rad(np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.4]))
    third = solver.path_allowed(seed, other)
    print(f"different pair: verdict={third} FK calls={kinematics.calls - after_second}")
    assert kinematics.calls > after_second, "a different pair reused the cached verdict"

    # A new cycle must not reuse the previous cycle's verdict.
    solver.begin_cycle()
    marker = kinematics.calls
    fourth = solver.path_allowed(seed, target)
    print(f"after begin_cycle: verdict={fourth} FK calls={kinematics.calls - marker}")
    assert fourth == first and kinematics.calls > marker, "the cache leaked across cycles"

    print("path-verdict cache: duplicate ask is free, verdict identical, no cross-cycle reuse")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
