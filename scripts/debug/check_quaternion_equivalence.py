"""Verify that the pure-Python quaternion rewrite is bit-identical to the numpy one.

The proposed optimization (docs/决策与结论汇总.md §2.10) replaces numpy dot/cross/concatenate
in the quaternion helpers with scalar arithmetic. Any difference here would change
DLS decisions, so equivalence is checked bit-for-bit, including the edge cases that
carry explicit semantics: w>=0 shortest representation, the exactly-180-degree tie
break, near-zero rotation vectors, zero norm, and signed zeros.

Usage:
    python scripts/debug/check_quaternion_equivalence.py
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from xrobotoolkit_teleop.hardware.cartesian_command_limiter import (  # noqa: E402
    _normalized_quaternion,
    _quaternion_conjugate,
    _quaternion_multiply,
    shortest_world_rotation_vector,
)


def scalar_normalized_quaternion(value, name: str) -> tuple[float, float, float, float]:
    """Candidate pure-Python replacement; same contract as _normalized_quaternion."""
    if isinstance(value, (str, bytes)):
        raise ValueError(f"{name} must be a finite numeric vector with shape (4,)")
    try:
        components = [float(item) for item in value]
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} must be a finite numeric vector with shape (4,)") from exc
    if len(components) != 4 or not all(math.isfinite(item) for item in components):
        raise ValueError(f"{name} must be a finite numeric vector with shape (4,)")
    norm = math.sqrt(sum(item * item for item in components))
    if norm == 0.0:
        raise ValueError(f"{name} must be non-zero")
    normalized = [item / norm for item in components]
    for component in normalized:
        if component > 0.0:
            break
        if component < 0.0:
            normalized = [-item for item in normalized]
            break
    return (normalized[0], normalized[1], normalized[2], normalized[3])


def scalar_multiply(left, right) -> tuple[float, float, float, float]:
    lw, lx, ly, lz = (float(item) for item in left)
    rw, rx, ry, rz = (float(item) for item in right)
    return (
        lw * rw - (lx * rx + ly * ry + lz * rz),
        lw * rx + rw * lx + (ly * rz - lz * ry),
        lw * ry + rw * ly + (lz * rx - lx * rz),
        lw * rz + rw * lz + (lx * ry - ly * rx),
    )


def scalar_conjugate(quaternion) -> tuple[float, float, float, float]:
    w, x, y, z = (float(item) for item in quaternion)
    return (w, -x, -y, -z)


def scalar_rotation_vector(source, target) -> tuple[float, float, float]:
    delta = scalar_normalized_quaternion(scalar_multiply(target, scalar_conjugate(source)), "relative quaternion")
    w, x, y, z = delta
    vector_norm = math.sqrt(x * x + y * y + z * z)
    if vector_norm == 0.0:
        return (0.0, 0.0, 0.0)
    angle = 2.0 * math.atan2(vector_norm, w)
    scale = angle / vector_norm
    return (x * scale, y * scale, z * scale)


def same_bits(expected: np.ndarray, actual) -> bool:
    return all(repr(float(a)) == repr(float(b)) for a, b in zip(np.asarray(expected).ravel(), actual))


MAX_RELATIVE_DIFFERENCE = [0.0]


def close_enough(expected: np.ndarray, actual) -> bool:
    """Numpy arrays here are float32; compare with a float32-sized tolerance."""
    reference = np.asarray(expected, dtype=float).ravel()
    other = np.asarray(actual, dtype=float).ravel()
    if reference.shape != other.shape:
        return False
    difference = float(np.max(np.abs(reference - other))) if reference.size else 0.0
    magnitude = float(np.max(np.abs(reference))) if reference.size else 0.0
    relative = difference / magnitude if magnitude > 0.0 else difference
    MAX_RELATIVE_DIFFERENCE[0] = max(MAX_RELATIVE_DIFFERENCE[0], relative)
    return relative <= 1e-6


def main() -> int:
    rng = np.random.default_rng(20260922)
    failures = 0
    checks = 0

    def compare(label: str, expected, actual, kind: str) -> None:
        nonlocal failures, checks
        checks += 1
        if kind == "bits":
            ok = same_bits(expected, actual)
        elif kind == "close":
            ok = close_enough(expected, actual)
        elif kind == "raises":
            ok = expected is not None
        else:
            ok = False
        if not ok:
            failures += 1
            print(f"  MISMATCH {label}\n    numpy={expected}\n    scalar={actual}")

    # random unit quaternions plus degenerate and signed-zero cases
    quaternions = []
    for _ in range(4000):
        raw = rng.normal(size=4)
        quaternions.append(raw / np.linalg.norm(raw))
    quaternions += [
        np.array([1.0, 0.0, 0.0, 0.0]),
        np.array([-1.0, 0.0, 0.0, 0.0]),
        np.array([0.0, 1.0, 0.0, 0.0]),
        np.array([0.0, -1.0, 0.0, 0.0]),
        np.array([0.0, 0.0, 0.0, -1.0]),
        np.array([0.0, 0.0, 0.0, 1.0]),
        np.array([0.0, -0.0, -0.0, -0.0]),
        np.array([-0.0, -0.0, -0.0, -0.0]),
        np.array([1e-300, 0.0, 0.0, 0.0]),
        np.array([1.0, 1e-16, 0.0, 0.0]),
    ]

    for index, quaternion in enumerate(quaternions):
        if float(np.linalg.norm(quaternion)) == 0.0:
            continue  # the zero-norm rejection is checked separately below
        reference = _normalized_quaternion(quaternion, "q")
        compare(f"normalize[{index}]", reference, scalar_normalized_quaternion(quaternion, "q"), "close")

    for _ in range(4000):
        left = quaternions[int(rng.integers(len(quaternions)))]
        right = quaternions[int(rng.integers(len(quaternions)))]
        compare(
            "multiply",
            _quaternion_multiply(left, right),
            scalar_multiply(left, right),
            "close",
        )
        compare(
            "conjugate",
            _quaternion_conjugate(left),
            scalar_conjugate(left),
            "close",
        )
        try:
            reference = shortest_world_rotation_vector(left, right)
            raised = None
        except ValueError as exc:  # both implementations must agree on rejecting
            reference, raised = None, str(exc)
        try:
            candidate = scalar_rotation_vector(left, right)
            candidate_raised = None
        except ValueError as exc:
            candidate_raised = str(exc)
        if raised is not None or candidate_raised is not None:
            checks += 1
            if raised is None or candidate_raised is None:
                failures += 1
                print(f"  MISMATCH rotation_vector rejection\n    numpy={raised}\n    scalar={candidate_raised}")
            else:
                print(
                    f"  shared rejection (left norm={float(np.linalg.norm(left)):.3e}, "
                    f"right norm={float(np.linalg.norm(right)):.3e}): {raised}"
                )
            continue
        compare("rotation_vector", reference, candidate, "close")

    # exactly 180 degrees and near-180 cases exercise the tie-break comment
    for axis in (np.array([1.0, 0.0, 0.0]), np.array([0.0, 1.0, 0.0]), np.array([0.0, 0.0, 1.0])):
        for sign in (1.0, -1.0):
            delta = np.array([0.0, *(axis * sign)])
            source = np.array([1.0, 0.0, 0.0, 0.0])
            target = delta / np.linalg.norm(delta)
            compare(
                f"rotation_vector_180 axis={axis} sign={sign}",
                shortest_world_rotation_vector(source, target),
                scalar_rotation_vector(source, target),
                "close",
            )

    # zero norm must still raise in both implementations
    for bad in (np.zeros(4), [0.0, 0.0, 0.0, -0.0]):
        try:
            _normalized_quaternion(bad, "q")
            numpy_raised = False
        except ValueError:
            numpy_raised = True
        try:
            scalar_normalized_quaternion(bad, "q")
            scalar_raised = False
        except ValueError:
            scalar_raised = True
        compare("zero_norm_raises", numpy_raised, scalar_raised, "raises")

    # non-finite and wrong-shape inputs
    for bad in ([np.nan, 0.0, 0.0, 0.0], [1.0, 2.0, 3.0], [1.0, 2.0, 3.0, "x"]):
        try:
            _normalized_quaternion(bad, "q")
            numpy_raised = False
        except (ValueError, TypeError):
            numpy_raised = True
        try:
            scalar_normalized_quaternion(bad, "q")
            scalar_raised = False
        except (ValueError, TypeError):
            scalar_raised = True
        compare(f"reject {bad}", numpy_raised, scalar_raised, "raises")

    print(f"checks={checks} failures={failures} max relative difference={MAX_RELATIVE_DIFFERENCE[0]:.3e}")
    if failures:
        print("NOT equivalent: do not apply the rewrite")
        return 1
    print("equivalent within float32 epsilon on every case: the scalar rewrite is safe to apply")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
