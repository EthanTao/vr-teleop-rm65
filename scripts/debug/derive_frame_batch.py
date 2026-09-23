"""Derive and verify a vectorised pose-frame composition, independent of the repo.

The scalar reference is ``compose_pose`` from ``damped_ik``: for a frame ``(x, q)``
and a pose ``(y, r)`` it returns ``(x + q*y*q_conj, q*r)``. This script builds the
batch form and checks it against the scalar one exhaustively (including identity
frames, 180-degree frames, and random stacks) before anything is wired into the DLS.

Usage:
    python scripts/debug/derive_frame_batch.py
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from xrobotoolkit_teleop.hardware.damped_ik import compose_pose  # noqa: E402


def quat_multiply_batch(left: np.ndarray, right: np.ndarray) -> np.ndarray:
    """Hamilton product for (4, N) stacks, in the same order as the scalar helper.

    Components must be stacked, never passed to ``np.array`` as a list of row
    vectors: that yields shape (4, 1) and later ``[1:]`` slices the wrong axis.
    """
    lw, lx, ly, lz = left[0], left[1], left[2], left[3]
    rw, rx, ry, rz = right[0], right[1], right[2], right[3]
    return np.stack(
        (
            lw * rw - (lx * rx + ly * ry + lz * rz),
            lw * rx + rw * lx + (ly * rz - lz * ry),
            lw * ry + rw * ly + (lz * rx - lx * rz),
            lw * rz + rw * lz + (lx * ry - ly * rx),
        )
    )


def quat_conjugate_batch(quat: np.ndarray) -> np.ndarray:
    out = np.array(quat, dtype=float, copy=True)
    out[1:] *= -1.0
    return out


def normalize_batch(quat: np.ndarray) -> np.ndarray:
    """Normalise (4, N) quaternions with the same w>=0 convention as the scalar path.

    The sign is not cosmetic: ``shortest_world_rotation_vector`` derives a signed
    angle from ``w``, so a flipped quaternion changes the Jacobian's rotation rows.
    """
    normalized = quat / np.linalg.norm(quat, axis=0, keepdims=True)
    negative = normalized[0] < 0.0
    if np.any(negative):
        normalized = np.where(negative[None, :], -normalized, normalized)
    return normalized


def apply_frame_batch(xyz: np.ndarray, quat: np.ndarray, frame) -> tuple[np.ndarray, np.ndarray]:
    """Vectorised twin of compose_pose for (3, N) / (4, N) stacks.

    .. warning::
       This is NOT correct yet and must not be wired in. The deployed
       ``compose_pose`` rotates the *incoming pose offset* by the frame quaternion
       **after** translating: ``x + off + R(q_frame) @ y``. The version below adds the
       offset unrotated, which is why the wired path disagreed with the scalar one
       (see docs/决策与结论汇总.md §4.4 and docs/archive/2026.9.22.md, ninth round). The correct vectorised form is
       ``offset + rotated`` where ``rotated`` comes from the *same* composition the
       scalar path performs, i.e. rotate ``offset + xyz`` rather than ``xyz``.
    """
    offset = np.asarray(frame[0], dtype=float).reshape(3, 1)
    frame_quat = np.asarray(frame[1], dtype=float).reshape(4, 1)
    pose_quat = np.asarray(quat, dtype=float).reshape(4, -1)
    xyz = np.asarray(xyz, dtype=float).reshape(3, -1)
    # The scalar path translates BEFORE rotating: rotated = R(q_frame) @ (offset + xyz).
    shifted = offset + xyz
    pure = np.zeros((4, xyz.shape[1]))
    pure[1:] = shifted
    rotated = quat_multiply_batch(quat_multiply_batch(frame_quat, pure), quat_conjugate_batch(frame_quat))[1:]
    composed = normalize_batch(quat_multiply_batch(frame_quat, pose_quat))
    return offset + rotated, composed


def main() -> int:
    rng = np.random.default_rng(11)
    failures = 0
    checks = 0

    for trial in range(400):
        count = int(rng.integers(1, 14))
        poses_xyz = rng.normal(size=(3, count))
        raw = rng.normal(size=(4, count))
        poses_quat = raw / np.linalg.norm(raw, axis=0, keepdims=True)

        if trial % 4 == 0:
            frame = (np.zeros(3), np.array([1.0, 0.0, 0.0, 0.0]))
        elif trial % 4 == 1:
            frame = (np.zeros(3), np.array([0.0, 1.0, 0.0, 0.0]))  # 180 degrees about x
        else:
            frame_quat = rng.normal(size=4)
            frame = (rng.normal(size=3), frame_quat / np.linalg.norm(frame_quat))

        batch_xyz, batch_quat = apply_frame_batch(poses_xyz, poses_quat, frame)
        for column in range(count):
            scalar_xyz, scalar_quat = compose_pose(frame, (poses_xyz[:, column], poses_quat[:, column]))
            checks += 1
            if not np.allclose(batch_xyz[:, column], scalar_xyz, rtol=0.0, atol=1e-12):
                failures += 1
                if failures <= 3:
                    print(f"  xyz mismatch trial={trial} column={column}: {batch_xyz[:, column]} vs {scalar_xyz}")
            if not np.allclose(batch_quat[:, column], scalar_quat, rtol=0.0, atol=1e-12):
                failures += 1
                if failures <= 3:
                    print(f"  quat mismatch trial={trial} column={column}: {batch_quat[:, column]} vs {scalar_quat}")

    print(f"pose comparisons={checks} failures={failures}")
    if failures:
        print("NOT equivalent: do not wire this in")
        return 1
    print("vectorised frame composition matches compose_pose on every case")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
