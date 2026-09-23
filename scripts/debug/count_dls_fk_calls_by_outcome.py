"""Count FK calls per bounded DLS cycle, including rejected candidates.

The historical implementation used 74 FK calls for a nominal accepted step and
could reach 222 through four repeated path searches. The realtime path now uses
one numerical Jacobian, a constant-time local singularity guard and at most three
endpoint candidates.

Usage:
    python scripts/debug/count_dls_fk_calls_by_outcome.py
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


HOMES = {
    "home #1 (ratio~0.014, DLS slowdown band)": np.deg2rad(
        np.array([20.151, -45.538, 95.208, -8.048, -62.972, -98.893])
    ),
    "home #2 (ratio~0.028, near band edge)": np.deg2rad(np.array([19.628, -31.532, 55.601, -12.300, -59.301, -98.919])),
}

# Target offsets, in metres along the controller x axis.
TARGETS = {
    "no operator input (target == anchor)": 0.0,
    "1 mm": 0.001,
    "5 mm": 0.005,
    "30 mm (max_offset_m envelope)": 0.030,
}


def run_case(seed, offset_m, path_verdict=None):
    """Return (fk_calls, status, path_calls, factor_attempts)."""
    kinematics = CountingKinematics()
    solver = DampedIK(ControllerFrameKinematics(kinematics, None, [0.0, 0.0, 0.1612, 1.0, 0.0, 0.0, 0.0]))
    origin, quat = solver.forward(seed)
    target_xyz = np.asarray(origin, dtype=float) + np.array([offset_m, 0.0, 0.0])

    attempts = [0]
    original_path = solver.bounded_path_allowed

    def counting_path(q, candidate, **kwargs):
        attempts[0] += 1
        if path_verdict is not None:
            return path_verdict
        return original_path(q, candidate, **kwargs)

    solver.bounded_path_allowed = counting_path
    solver.begin_cycle()
    kinematics.calls = 0
    result = solver.step(
        target_xyz,
        np.asarray(quat, dtype=float),
        seed,
        0.02,
        RM65_B_SPEC.max_joint_velocity_rad_s * 0.10,
        max_linear_speed=0.05,
        max_angular_speed=0.25,
    )
    return kinematics.calls, result.status, attempts[0]


def main() -> int:
    print("FK calls per DLS step (each call is one official rm_algo_forward_kinematics)\n")
    print(f"  {'pose':42s} {'target':34s} {'FK':>5s} {'guard checks':>12s}  status")
    worst = 0
    for home_label, seed in HOMES.items():
        for target_label, offset in TARGETS.items():
            calls, status, paths = run_case(seed, offset)
            worst = max(worst, calls)
            print(f"  {home_label:42s} {target_label:34s} {calls:5d} {paths:12d}  {status}")

    # Force every candidate factor to be rejected by the constant-time guard.
    print("\nforced bounded-guard rejection (every candidate factor rejected):")
    for home_label, seed in HOMES.items():
        calls, status, paths = run_case(seed, 0.005, path_verdict=False)
        worst = max(worst, calls)
        print(f"  {home_label:42s} {'5 mm, all factors rejected':34s} {calls:5d} {paths:12d}  {status}")

    print(f"\nworst observed FK calls per step: {worst}")
    print(
        "  bounded accounting: 13 (seed FK + one numerical Jacobian)\n"
        "    + N x 1 endpoint FK, N <= 3 candidate attempts\n"
        "  nominal accepted candidate -> 14 calls; hard maximum -> 16 calls"
    )
    print(
        "  at the container-measured 1.2 ms per composed call: "
        f"{14 * 1.2:.1f} ms (nominal) / {16 * 1.2:.1f} ms (FK-only hard maximum)"
    )
    print("  independent wall-clock guard: 18 ms; over-budget cycles hold instead of sending")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
