"""Offline check: is the adapter layer itself expensive, or only under contention?

Times raw kinematics, ``ControllerFrameKinematics.forward``, and a full
``DampedIK.step`` on this PC with in-process synthetic kinematics (no SDK, no
UDP thread). Compared with the board numbers in docs/决策与结论汇总.md §2.10 this separates
"the adapter's Python work costs ~1 ms" from "the board is contended".
"""

from __future__ import annotations

import gc
import statistics
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


class SyntheticKinematics:
    def forward(self, joint_rad):
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


def summary(label, samples):
    ordered = sorted(samples)
    p95 = ordered[min(len(ordered) - 1, int(round(0.95 * (len(ordered) - 1))))]
    print(
        f"  {label:44s} n={len(samples):4d} median={statistics.median(samples) * 1000:8.4f}ms "
        f"p95={p95 * 1000:8.4f}ms max={max(samples) * 1000:8.4f}ms"
    )


def main() -> int:
    seed = np.deg2rad(np.array([20.151, -45.538, 95.208, -8.048, -62.972, -98.893]))
    official = SyntheticKinematics()
    frames = ControllerFrameKinematics(official, None, [0.0, 0.0, 0.1612, 1.0, 0.0, 0.0, 0.0])
    dls = DampedIK(frames)

    official.forward(seed)
    raw = []
    for _ in range(2000):
        start = time.perf_counter()
        official.forward(seed)
        raw.append(time.perf_counter() - start)
    composed = []
    for _ in range(2000):
        dls.begin_cycle()
        start = time.perf_counter()
        frames.forward(seed)
        composed.append(time.perf_counter() - start)

    summary("synthetic raw kinematics", raw)
    summary("ControllerFrameKinematics.forward", composed)
    print(f"  adapter overhead factor: {statistics.median(composed) / statistics.median(raw):.2f}x")
    print(f"  adapter added per call:  {(statistics.median(composed) - statistics.median(raw)) * 1000:.4f}ms")

    stats = gc.get_stats()
    print(
        f"  gc collections during the loops: gen0={stats[0]['collections']} "
        f"gen1={stats[1]['collections']} gen2={stats[2]['collections']}"
    )

    # Cyclic-GC sensitivity: if the adapter's many small arrays dominate the cost
    # through collector work, disabling the collector must collapse the ratio.
    gc.disable()
    try:
        composed_nogc = []
        for _ in range(2000):
            dls.begin_cycle()
            start = time.perf_counter()
            frames.forward(seed)
            composed_nogc.append(time.perf_counter() - start)
    finally:
        gc.enable()
    summary("ControllerFrameKinematics.forward (gc off)", composed_nogc)
    print(
        f"  gc-off vs gc-on adapter overhead: "
        f"{(statistics.median(composed) - statistics.median(raw)) * 1000:.4f}ms -> "
        f"{(statistics.median(composed_nogc) - statistics.median(raw)) * 1000:.4f}ms"
    )

    origin, quat = frames.forward(seed)
    command = origin + np.array([0.005, 0.0, 0.0])
    speed_limits = RM65_B_SPEC.max_joint_velocity_rad_s * 0.10
    steps, calls = [], []
    for _ in range(50):
        dls.begin_cycle()
        start = time.perf_counter()
        result = dls.step(
            command,
            quat,
            seed,
            0.02,
            speed_limits,
            max_linear_speed=0.05,
            max_angular_speed=0.25,
        )
        steps.append(time.perf_counter() - start)
        calls.append(len(dls._pose_cache))
    summary("DampedIK.step (synthetic)", steps)
    print(f"  distinct FK poses per step: {min(calls)}..{max(calls)}  status={result.status}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
