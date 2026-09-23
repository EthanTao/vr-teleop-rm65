"""Count Python-level numpy operations per DLS cycle, with and without the adapter.

The container measurement in docs/决策与结论汇总.md §2.10 showed cost scaling with the number
of Python-level numpy operations (~40-150us each in the runtime container), not with
the official SDK call itself (0.147ms). This script counts those operations so an
optimization can be judged on the metric that actually moves the container time,
instead of on a local wall clock that cannot reproduce it.

Usage:
    python scripts/debug/count_dls_numpy_ops.py
"""

from __future__ import annotations

import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from xrobotoolkit_teleop.hardware.damped_ik import ControllerFrameKinematics, DampedIK  # noqa: E402
from xrobotoolkit_teleop.hardware.rm65_model import RM65_B_SPEC  # noqa: E402
from xrobotoolkit_teleop.hardware.singularity_avoidance import (  # noqa: E402
    RM65_CHAIN,
    rm65_forward_kinematics,
)

_ORIGINAL_NUMPY = np


class CountingKinematics:
    """In-process stand-in for the official FK, with a call counter."""

    def __init__(self):
        self.calls = 0

    def forward(self, joint_rad):
        self.calls += 1
        matrix = rm65_forward_kinematics(_ORIGINAL_NUMPY.asarray(joint_rad, dtype=float), RM65_CHAIN)
        trace = matrix[0, 0] + matrix[1, 1] + matrix[2, 2]
        w = _ORIGINAL_NUMPY.sqrt(max(0.0, 1.0 + trace)) / 2.0
        quat = _ORIGINAL_NUMPY.array(
            [
                w,
                (matrix[2, 1] - matrix[1, 2]) / (4.0 * w),
                (matrix[0, 2] - matrix[2, 0]) / (4.0 * w),
                (matrix[1, 0] - matrix[0, 1]) / (4.0 * w),
            ]
        )
        return matrix[:3, 3].copy(), quat / _ORIGINAL_NUMPY.linalg.norm(quat)


class CountingNumpy:
    """Transparent numpy proxy that records which callables are invoked."""

    def __init__(self, inner):
        self._inner = inner
        self.counts: dict[str, int] = {}

    def __getattr__(self, name):
        value = getattr(self._inner, name)
        if not callable(value) or isinstance(value, type):
            return value
        counts = self.counts

        def counted(*args, **kwargs):
            counts[name] = counts.get(name, 0) + 1
            return value(*args, **kwargs)

        return counted


def measure_case(name: str, kinematics, tool_offset_m: float) -> None:
    frames = ControllerFrameKinematics(
        kinematics, None, [0.0, 0.0, tool_offset_m, 1.0, 0.0, 0.0, 0.0]
    )
    dls = DampedIK(frames)
    seed = _ORIGINAL_NUMPY.deg2rad(_ORIGINAL_NUMPY.array([19.625, -31.532, 55.599, -12.3, -59.301, -98.92]))

    # Warm the caches, then reset the counters for one clean cycle.
    frames.forward(seed)
    dls.begin_cycle()
    target_position = frames.forward(seed)[0] + _ORIGINAL_NUMPY.array([0.005, 0.0, 0.0])
    target_quat = frames.forward(seed)[1]

    proxy = CountingNumpy(_ORIGINAL_NUMPY)
    previous_numpy = sys.modules["numpy"]
    dls_module = sys.modules["xrobotoolkit_teleop.hardware.damped_ik"]
    limiter_module = sys.modules["xrobotoolkit_teleop.hardware.cartesian_command_limiter"]
    sys.modules["numpy"] = proxy
    dls_module.np = proxy
    limiter_module.np = proxy
    kinematics.calls = 0
    dls.begin_cycle()
    try:
        result = dls.step(
            target_position,
            target_quat,
            seed,
            0.02,
            RM65_B_SPEC.max_joint_velocity_rad_s * 0.10,
            max_linear_speed=0.05,
            max_angular_speed=0.25,
        )
    finally:
        dls_module.np = _ORIGINAL_NUMPY
        limiter_module.np = _ORIGINAL_NUMPY
        sys.modules["numpy"] = previous_numpy

    total = sum(proxy.counts.values())
    distinct = len(dls._pose_cache)
    print(f"\n=== {name} ===")
    print(f"  status={result.status} FK calls={kinematics.calls} distinct FK poses={distinct}")
    print(f"  numpy operations this cycle: {total}")
    for key, value in sorted(proxy.counts.items(), key=lambda item: -item[1]):
        print(f"    {key:28s} {value:5d}")


def _benchmark(label: str, call, repeat: int = 200000) -> float:
    call()
    start = time.perf_counter()
    for _ in range(repeat):
        call()
    elapsed = (time.perf_counter() - start) / repeat * 1e6
    print(f"  {label:52s} {elapsed:7.3f}us/op")
    return elapsed


def micro_benchmarks() -> None:
    """Compare the current numpy helpers with pure-Python scalar rewrites.

    Only the relative cost ratio matters here: the container's absolute numpy cost
    is 10x-30x this PC's, and the rewrite is judged on how much of that it removes.
    """
    import time

    from xrobotoolkit_teleop.hardware.cartesian_command_limiter import (
        _quaternion_conjugate,
        _quaternion_multiply,
    )

    left = _ORIGINAL_NUMPY.array([0.9, 0.1, 0.2, 0.3])
    right = _ORIGINAL_NUMPY.array([0.8, -0.2, 0.1, 0.4])
    small = _ORIGINAL_NUMPY.array([0.1, 0.2, 0.3])

    def numpy_multiply():
        return _quaternion_multiply(left, right)

    def scalar_multiply():
        lw, lx, ly, lz = left
        rw, rx, ry, rz = right
        return (
            lw * rw - (lx * rx + ly * ry + lz * rz),
            lw * rx + rw * lx + (ly * rz - lz * ry),
            lw * ry + rw * ly + (lz * rx - lx * rz),
            lw * rz + rw * lz + (lx * ry - ly * rx),
        )

    def numpy_cross():
        return _ORIGINAL_NUMPY.cross(small, small)

    def scalar_cross():
        x, y, z = small
        return (y * z - z * y, z * x - x * z, x * y - y * x)

    def numpy_validate():
        vector = _ORIGINAL_NUMPY.array(small, dtype=float, copy=True)
        return bool(_ORIGINAL_NUMPY.all(_ORIGINAL_NUMPY.isfinite(vector))) and vector.shape == (3,)

    def scalar_validate():
        x, y, z = small
        return all(value == value and -float("inf") < value < float("inf") for value in (x, y, z))

    print("\n[relative cost of the two helper rewrites, this PC]")
    numpy_quat = _benchmark("numpy _quaternion_multiply (dot+cross+concatenate)", numpy_multiply)
    scalar_quat = _benchmark("pure-Python quaternion multiply", scalar_multiply)
    numpy_cross_cost = _benchmark("numpy cross(3,3)", numpy_cross)
    scalar_cross_cost = _benchmark("pure-Python cross", scalar_cross)
    numpy_valid = _benchmark("numpy validate (array+isfinite+all)", numpy_validate)
    scalar_valid = _benchmark("pure-Python validate", scalar_validate)
    print(
        f"  -> quaternion {numpy_quat / scalar_quat:.1f}x, cross {numpy_cross_cost / scalar_cross_cost:.1f}x, "
        f"validate {numpy_valid / scalar_valid:.1f}x cheaper as scalars"
    )


def main() -> int:
    measure_case("synthetic chain, tool offset 161.2mm", CountingKinematics(), 0.1612)
    measure_case("synthetic chain, identity tool", CountingKinematics(), 0.0)
    micro_benchmarks()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
