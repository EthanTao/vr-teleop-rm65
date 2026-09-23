"""Offline probe: geometric hint for the dry-run home configuration. Do not trust its ratios.

Read-only. Uses the local structural RM65 chain (vendor URDF constants) to search
for joint configurations whose flange (or flange + tool offset along local z)
reproduces the UDP-reported TCP position, then prints the singular-value ratio
used by ``damped_ik`` and the joint direction that leaves the hard band.

Two caveats, both of which make the printed ratios and joint angles unreliable:

* the controller's own DH is only reproduced at runtime (``get_DH_data``), so the
  structural chain here is a different model;
* the search fits three position coordinates with six unknowns, so it returns an
  underdetermined family of postures, and a stretched-arm family can always reach
  a small ratio artificially.

Use it only to check reach/geometry (is the home point near the limit of the
arm?), or better, run ``scripts/hardware/benchmark_dls_cycle.py`` in the runtime
container for the real numbers.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np
from scipy.optimize import minimize

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from xrobotoolkit_teleop.hardware.singularity_avoidance import (  # noqa: E402
    RM65_CHAIN,
    rm65_forward_kinematics,
    rm65_geometric_jacobian,
)

LENGTH_NORM = 0.3  # damped_ik.DampedIK.length


def frame(joint_rad: np.ndarray) -> np.ndarray:
    return rm65_forward_kinematics(joint_rad, RM65_CHAIN)


def tcp_position(joint_rad: np.ndarray, tool_m: float) -> np.ndarray:
    tip = frame(joint_rad)
    return tip[:3, 3] + tool_m * tip[:3, 2]


def ratio_at(joint_rad: np.ndarray) -> tuple[float, np.ndarray]:
    jacobian = rm65_geometric_jacobian(joint_rad, RM65_CHAIN)
    jacobian[:3] /= LENGTH_NORM
    singular = np.linalg.svd(jacobian, compute_uv=False)
    return float(singular[-1] / singular[0]), singular


def solve(target: np.ndarray, seed: np.ndarray, tool_m: float) -> np.ndarray | None:
    def cost(q: np.ndarray) -> float:
        delta = tcp_position(q, tool_m) - target
        return float(delta @ delta + 1e-8 * (q @ q))

    result = minimize(cost, seed, method="L-BFGS-B", options={"maxiter": 6000, "ftol": 1e-18})
    if float(np.linalg.norm(tcp_position(result.x, tool_m) - target)) > 2e-4:
        return None
    return result.x


def report(label: str, solved: np.ndarray, tool_m: float) -> None:
    ratio, singular = ratio_at(solved)
    degrees = np.rad2deg(solved)
    print(
        f"{label}\n  q_deg = {np.round(degrees, 2).tolist()}"
        f"\n  J3 = {degrees[2]:+.3f} deg, J5 = {degrees[4]:+.3f} deg"
        f"\n  sigma_min={singular[-1]:.6f} sigma_max={singular[0]:.6f} ratio={ratio:.5f}"
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--tcp-xyz", type=float, nargs=3, default=[0.0777, -0.0052, 0.7794])
    parser.add_argument("--tool-mm", type=float, nargs="+", default=[0.0, 161.2])
    parser.add_argument("--stop-ratio", type=float, default=0.01)
    parser.add_argument("--slowdown-ratio", type=float, default=0.04)
    args = parser.parse_args()

    target = np.array(args.tcp_xyz, dtype=float)
    rng = np.random.default_rng(1)
    print(f"target TCP xyz_m = {np.round(target, 4).tolist()}\n")

    for tool_mm in args.tool_mm:
        tool_m = tool_mm / 1000.0
        seeds = [np.zeros(6)] + [np.deg2rad(rng.uniform(-150.0, 150.0, size=6)) for _ in range(600)]
        solutions: list[np.ndarray] = []
        for seed in seeds:
            solved = solve(target, seed, tool_m)
            if solved is None:
                continue
            if any(np.max(np.abs(solved - other)) < 1e-3 for other in solutions):
                continue
            solutions.append(solved)
        if not solutions:
            print(f"tool={tool_mm:.1f} mm: no configuration reproduces this TCP position\n")
            continue
        solutions.sort(key=lambda q: ratio_at(q)[0])
        print(f"=== tool offset {tool_mm:.1f} mm along flange z: {len(solutions)} solutions ===")
        print("  elbow is fully extended at J3 = 0; wrist alignment is at J5 = 0\n")
        header = "   J1      J2      J3      J4      J5      J6  |   ratio   sigma_min  J3_deg  J5_deg"
        print(header)
        for solved in solutions[:6]:
            ratio, singular = ratio_at(solved)
            degrees = np.rad2deg(solved)
            print(
                " ".join(f"{value:7.2f}" for value in degrees)
                + f" | {ratio:8.5f} {singular[-1]:10.6f} {abs(degrees[2]):7.2f} {abs(degrees[4]):7.2f}"
            )
        best = solutions[0]
        ratio, _ = ratio_at(best)
        print(f"\n  lowest-ratio solution: ratio={ratio:.5f}")
        if ratio <= args.stop_ratio:
            zone = (
                "stop band: speed scale floor + max damping, and the path filter only allows strictly improving moves"
            )
        elif ratio <= args.slowdown_ratio:
            zone = "hard-slowdown band: speed scale at floor (>=0.05) and max damping 0.08"
        else:
            zone = "clear"
        print(f"  DLS zone: {zone}")

        jacobian = rm65_geometric_jacobian(best, RM65_CHAIN)
        jacobian[:3] /= LENGTH_NORM
        _, _, vt = np.linalg.svd(jacobian, full_matrices=False)
        print("\n  last right-singular vector (motion that barely moves the TCP):")
        print("   " + "  ".join(f"J{i + 1}={value:+.3f}" for i, value in enumerate(vt[-1])))

        print(f"\n  single-joint displacement to reach ratio > {args.slowdown_ratio}:")
        for index in range(6):
            found = None
            for delta_deg in np.arange(0.25, 90.0, 0.25):
                for sign in (1.0, -1.0):
                    probe = best.copy()
                    probe[index] += sign * np.deg2rad(delta_deg)
                    if ratio_at(probe)[0] > args.slowdown_ratio:
                        found = sign * delta_deg
                        break
                if found is not None:
                    break
            print(f"    J{index + 1}: {'none within 90 deg' if found is None else f'{found:+.2f} deg'}")
        print()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
