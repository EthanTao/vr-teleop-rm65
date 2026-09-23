"""Offline scan: TCP poses of the RM65-B that sit far away from kinematic singularities.

Read-only, no SDK, no robot connection. It is a *forward* computation only: every
number below comes from an explicitly given joint vector, so - unlike
``analyze_home_singularity.py`` - the Jacobian here is not fitted to an
underdetermined target and its singular-value ratio is meaningful for this
structural chain.

Model / criterion (must match the runtime DLS path, see docs/真机安全与故障处理.md):

* chain      : ``singularity_avoidance.RM65_CHAIN`` (vendor RM65 URDF constants)
* TCP point  : flange frame translated by ``--tool-mm`` along the flange local z
* Jacobian   : base-frame geometric Jacobian, linear rows divided by 0.3 m
* risk ratio : sigma_min / sigma_max of that normalized Jacobian
* zones      : ratio <= stop_ratio (0.01) -> danger, <= slowdown_ratio (0.04) -> slowdown
* joint band : |J3| or |J5| <= stop_joint_deg (5) -> danger, <= slowdown_joint_deg (15) -> slowdown
* joint limit: RM65-B limits shrunk by 5 deg (the DLS candidate gate) and an optional
               absolute cap per joint

Caveats, stated up front:

* This is the vendor *structural* chain. The controller reports its own DH
  (``get_DH_data``, e.g. d6 = 161.2 mm) and its own tool frame; the real TCP for a
  given joint vector can differ by millimetres/degrees. Treat the printed poses as
  geometry-level plan candidates, not as calibrated on-robot coordinates.
* Poses are expressed in the FK base frame. The controller's work frame is applied
  on top at runtime (``--dls-work-from-base``); with the current field configuration
  it is the identity, but re-check it before using these numbers on hardware.
* Clearance is marched along straight Cartesian lines, so it is a local measure: it
  says nothing about a global path, collision or self-collision.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from xrobotoolkit_teleop.hardware.rm65_model import RM65_B_SPEC  # noqa: E402
from xrobotoolkit_teleop.hardware.singularity_avoidance import (  # noqa: E402
    RM65_CHAIN,
    _origin_transform,
    rm65_forward_kinematics,
    rm65_geometric_jacobian,
)

LENGTH_NORM = 0.3  # damped_ik.DampedIK.length
LIMIT_MARGIN_RAD = np.deg2rad(5.0)  # damped_ik candidate gate
REACH_TCP_M = 0.779  # RM65-6F maximum reach

_ORIGIN_TRANSFORMS = np.array([_origin_transform(RM65_CHAIN, index) for index in range(6)])
_ORIGIN_ROTATIONS = _ORIGIN_TRANSFORMS[:, :3, :3]
_ORIGIN_POSITIONS = _ORIGIN_TRANSFORMS[:, :3, 3]


class Risk:
    """Projection of one joint vector onto the DLS criterion."""

    __slots__ = ("joint_rad", "position", "rotation", "ratio", "sigma_min", "sigma_max", "j3", "j5")

    def __init__(
        self,
        joint_rad: np.ndarray,
        tool_m: float,
        ratio: float,
        sigma_min: float,
        sigma_max: float,
        origin: np.ndarray,
        rotation: np.ndarray,
    ):
        self.joint_rad = np.asarray(joint_rad, dtype=float)
        self.rotation = np.asarray(rotation, dtype=float)
        # tool z axis of this frame: index the last axis, not ``rotation[:, 2]`` (that is row 2)
        self.position = np.asarray(origin, dtype=float) + tool_m * self.rotation[:, 2]
        self.ratio = float(ratio)
        self.sigma_min = float(sigma_min)
        self.sigma_max = float(sigma_max)
        self.j3 = float(self.joint_rad[2])
        self.j5 = float(self.joint_rad[4])


def batch_frames(joints: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Vectorized vendor FK: joint origins (N,6,3), joint axes (N,6,3), tip rotation (N,3,3), tip position (N,3)."""
    count = joints.shape[0]
    transform_rotation = np.broadcast_to(np.eye(3), (count, 3, 3)).copy()
    transform_position = np.zeros((count, 3), dtype=float)
    origins = np.zeros((count, 6, 3), dtype=float)
    axes = np.zeros((count, 6, 3), dtype=float)
    rotation = transform_rotation
    for index in range(6):
        rotation = transform_rotation @ _ORIGIN_ROTATIONS[index]
        position = transform_position + np.einsum("nij,j->ni", transform_rotation, _ORIGIN_POSITIONS[index])
        origins[:, index] = position
        axes[:, index] = rotation[:, :, 2]
        angle = joints[:, index]
        cos_a, sin_a = np.cos(angle), np.sin(angle)
        local = np.zeros((count, 3, 3), dtype=float)
        local[:, 0, 0] = cos_a
        local[:, 0, 1] = -sin_a
        local[:, 1, 0] = sin_a
        local[:, 1, 1] = cos_a
        local[:, 2, 2] = 1.0
        # The joint rotation has to be applied as well: without it the returned frame is the
        # pre-J6 flange frame (same z axis, wrong x/y).
        transform_rotation = rotation @ local
        transform_position = position.copy()
    return origins, axes, transform_rotation, transform_position


def batch_ratios(joints: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Return ratio, sigma_min, sigma_max, tip rotation and tip position for ``(N, 6)`` joint vectors."""
    origins, axes, rotation, position = batch_frames(joints)
    jacobian = np.empty((joints.shape[0], 6, 6), dtype=float)
    for index in range(6):
        jacobian[:, :3, index] = np.cross(axes[:, index], position - origins[:, index]) / LENGTH_NORM
        jacobian[:, 3:, index] = axes[:, index]
    singular = np.linalg.svd(jacobian, compute_uv=False)
    sigma_max = singular[:, 0]
    sigma_min = singular[:, -1]
    return sigma_min / sigma_max, sigma_min, sigma_max, rotation, position


def tool_z_axis(rotation: np.ndarray) -> np.ndarray:
    """Third column of each ``(N, 3, 3)`` frame; ``rotation[:, 2]`` would select row 2 instead."""
    return rotation[:, :, 2]


def joint_bounds(limit_deg: float) -> tuple[np.ndarray, np.ndarray]:
    lower = RM65_B_SPEC.joint_limit_rad[:, 0] + LIMIT_MARGIN_RAD
    upper = RM65_B_SPEC.joint_limit_rad[:, 1] - LIMIT_MARGIN_RAD
    if 0.0 < limit_deg < 178.0:
        cap = np.deg2rad(limit_deg)
        lower = np.maximum(lower, -cap)
        upper = np.minimum(upper, cap)
    return lower, upper


def margins(risk: Risk, args: argparse.Namespace) -> tuple[float, float, float]:
    """Ratio/J3/J5 headroom over the slowdown thresholds, as multiplicative factors."""
    return (
        risk.ratio / args.slowdown_ratio,
        abs(risk.j3) / np.deg2rad(args.slowdown_joint_deg),
        abs(risk.j5) / np.deg2rad(args.slowdown_joint_deg),
    )


def worst_margin(risk: Risk, args: argparse.Namespace) -> float:
    return min(margins(risk, args))


def in_workspace(risk: Risk, args: argparse.Namespace) -> bool:
    radius = float(np.linalg.norm(risk.position))
    if radius > args.max_radius_m or radius < args.min_radius_m:
        return False
    if args.max_height_m is not None and risk.position[2] > args.max_height_m:
        return False
    if args.min_height_m is not None and risk.position[2] < args.min_height_m:
        return False
    return True


def refine(risk: Risk, args: argparse.Namespace, rounds: int = 8) -> Risk:
    """Greedy ascent on the worst margin, keeping the posture clear and reachable."""
    lower, upper = joint_bounds(args.joint_limit_deg)
    tool_m = args.tool_mm / 1000.0
    best, best_score = risk, worst_margin(risk, args)
    for _ in range(rounds):
        improved = False
        for joint in range(6):
            for delta_deg in (4.0, 2.0, 1.0, -1.0, -2.0, -4.0):
                probe = best.joint_rad.copy()
                probe[joint] += np.deg2rad(delta_deg)
                if np.any(probe < lower) or np.any(probe > upper):
                    continue
                candidate = evaluate(probe, tool_m)
                if not in_workspace(candidate, args):
                    continue
                if args.min_self_clearance_m > 0.0 and float(
                    batch_self_clearance(probe.reshape(1, 6), tool_m)[0]
                ) < args.min_self_clearance_m:
                    continue
                score = worst_margin(candidate, args)
                if score > best_score + 1e-6:
                    best, best_score, improved = candidate, score, True
        if not improved:
            break
    return best


def evaluate(joint_rad: np.ndarray, tool_m: float) -> Risk:
    joints = np.asarray(joint_rad, dtype=float).reshape(1, 6)
    ratios, sigma_min, sigma_max, rotation, position = batch_ratios(joints)
    return Risk(joints[0], tool_m, ratios[0], sigma_min[0], sigma_max[0], position[0], rotation[0])


def batch_self_clearance(joints: np.ndarray, tool_m: float) -> np.ndarray:
    """Minimum distance between non-adjacent arm segments, as a crude self-collision margin.

    Segments are the polyline base -> J2 -> J3 -> wrist -> TCP, so the number is a
    segment-to-segment distance, not a mesh clearance. It only filters out postures that
    fold the arm onto itself; it is not collision detection.
    """
    origins, axes, rotation, position = batch_frames(joints)
    wrist = origins[:, 5]
    tcp = position + tool_m * tool_z_axis(rotation)
    nodes = [origins[:, 1], origins[:, 2], wrist, tcp]
    # segments: 0 = upper arm (J2->J3), 1 = forearm (J3->wrist), 2 = wrist+tool (wrist->TCP)
    pairs = [(0, 2)]
    count = joints.shape[0]
    best = np.full(count, np.inf)
    for first, second in pairs:
        best = np.minimum(best, _segment_distance(nodes[first], nodes[first + 1], nodes[second], nodes[second + 1]))
    return best


def _segment_distance(p1: np.ndarray, p2: np.ndarray, p3: np.ndarray, p4: np.ndarray) -> np.ndarray:
    """Vectorized closest distance between segments ``p1p2`` and ``p3p4``, sampled along the first."""
    samples = np.linspace(0.0, 1.0, 41)
    points = p1[:, None, :] + samples[None, :, None] * (p2 - p1)[:, None, :]
    direction = p4 - p3
    length_squared = np.sum(direction * direction, axis=1)
    length_squared = np.where(length_squared < 1e-12, 1.0, length_squared)
    offset = points - p3[:, None, :]
    t = np.clip(np.sum(offset * direction[:, None, :], axis=2) / length_squared[:, None], 0.0, 1.0)
    closest = p3[:, None, :] + t[:, :, None] * direction[:, None, :]
    return np.min(np.linalg.norm(points - closest, axis=2), axis=1)


def clear_positions(args: argparse.Namespace, rng: np.random.Generator) -> list[Risk]:
    lower, upper = joint_bounds(args.joint_limit_deg)
    tool_m = args.tool_mm / 1000.0
    joints = rng.uniform(lower, upper, size=(args.samples, 6))
    ratios, sigma_min, sigma_max, rotation, position = batch_ratios(joints)
    # The workspace filter is applied to the TCP point, the same point that gets reported.
    tcp = position + tool_m * tool_z_axis(rotation)
    j3 = np.abs(joints[:, 2])
    j5 = np.abs(joints[:, 4])
    radius = np.linalg.norm(tcp, axis=1)
    # Absolute floors chosen by the operator, not fractions of the runtime thresholds.
    ratio_floor = args.min_ratio
    joint_floor = np.deg2rad(args.min_joint_deg)
    keep = (
        (ratios > ratio_floor)
        & (j3 > joint_floor)
        & (j5 > joint_floor)
        & (radius <= args.max_radius_m)
        & (radius >= args.min_radius_m)
    )
    if args.max_height_m is not None:
        keep &= tcp[:, 2] <= args.max_height_m
    if args.min_height_m is not None:
        keep &= tcp[:, 2] >= args.min_height_m
    if args.min_self_clearance_m > 0.0:
        keep &= batch_self_clearance(joints, tool_m) >= args.min_self_clearance_m
    accepted = [
        Risk(joints[index], tool_m, ratios[index], sigma_min[index], sigma_max[index], position[index], rotation[index])
        for index in np.flatnonzero(keep)
    ]
    accepted.sort(key=lambda item: worst_margin(item, args), reverse=True)
    return accepted


def radial_sweep(
    args: argparse.Namespace, rng: np.random.Generator, candidates: list[Risk] | None = None
) -> list[tuple[float, float, Risk]]:
    """Best achievable worst-margin per TCP radius band, so the operator can pick a working radius."""
    pool = clear_positions(args, rng) if candidates is None else candidates
    rows: list[tuple[float, float, Risk]] = []
    low = args.sweep_start_m
    while low < args.max_radius_m - 1e-9:
        high = min(low + args.sweep_step_m, args.max_radius_m)
        band = [item for item in pool if low <= float(np.linalg.norm(item.position)) <= high]
        if band:
            band.sort(key=lambda item: worst_margin(item, args), reverse=True)
            rows.append((low, high, refine(band[0], args)))
        low = high
    return rows


def deduplicate(candidates: list[Risk], args: argparse.Namespace, keep: int) -> list[Risk]:
    chosen: list[Risk] = []
    for candidate in candidates:
        if any(
            np.linalg.norm(candidate.position - other.position) < args.dedup_m
            and abs(candidate.j3 - other.j3) < np.deg2rad(10.0)
            for other in chosen
        ):
            continue
        chosen.append(candidate)
        if len(chosen) >= keep:
            break
    return chosen


def rotation_vector(rotation: np.ndarray) -> np.ndarray:
    cos_angle = float(np.clip((np.trace(rotation) - 1.0) / 2.0, -1.0, 1.0))
    angle = float(np.arccos(cos_angle))
    if angle < 1e-9:
        return np.zeros(3)
    if abs(angle - np.pi) < 1e-6:
        axis = np.sqrt(np.maximum((np.diag(rotation) + 1.0) / 2.0, 0.0))
        return axis / max(np.linalg.norm(axis), 1e-12) * angle
    skew = (rotation - rotation.T) / (2.0 * np.sin(angle))
    return np.array([skew[2, 1], skew[0, 2], skew[1, 0]]) * angle


def tcp_of(joint_rad: np.ndarray, tool_m: float) -> tuple[np.ndarray, np.ndarray]:
    tip = rm65_forward_kinematics(joint_rad, RM65_CHAIN)
    return tip[:3, 3] + tool_m * tip[:3, 2], tip[:3, :3]


def local_move(joint_rad: np.ndarray, target_position: np.ndarray, target_rotation: np.ndarray, args, iterations=30):
    """Damped least-squares step toward a Cartesian target; ``None`` when it does not converge."""
    lower, upper = joint_bounds(args.joint_limit_deg)
    tool_m = args.tool_mm / 1000.0
    joints = np.array(joint_rad, dtype=float)
    for _ in range(iterations):
        position, rotation = tcp_of(joints, tool_m)
        error = np.r_[target_position - position, rotation_vector(target_rotation @ rotation.T)]
        if np.linalg.norm(error[:3]) < 2e-4 and np.linalg.norm(error[3:]) < 2e-3:
            return joints
        jacobian = rm65_geometric_jacobian(joints, RM65_CHAIN)
        step = np.linalg.solve(jacobian.T @ jacobian + (1e-3**2) * np.eye(6), jacobian.T @ error)
        joints = np.clip(joints + 0.5 * np.clip(step, -0.1, 0.1), lower, upper)
    return None


def clearance_m(risk: Risk, args: argparse.Namespace) -> float:
    """Straight-line Cartesian travel, averaged over the six axis directions, until the band."""
    total, used = 0.0, 0
    for axis in np.eye(3):
        for sign in (1.0, -1.0):
            reach = 0.0
            for step in np.arange(args.clearance_step_m, args.clearance_max_m + 1e-9, args.clearance_step_m):
                moved = local_move(risk.joint_rad, risk.position + sign * step * axis, risk.rotation, args)
                if moved is None:
                    break
                if worst_margin(evaluate(moved, args.tool_mm / 1000.0), args) <= 1.0:
                    break
                reach = float(step)
            total += reach
            used += 1
    return total / max(used, 1)


def joint_robustness(risk: Risk, args: argparse.Namespace, span_deg: float | None = None) -> list[float]:
    """Per-joint travel (deg) in the worse direction until the joint leaves the clear zone."""
    span = args.joint_robust_span_deg if span_deg is None else span_deg
    lower, upper = joint_bounds(args.joint_limit_deg)
    tool_m = args.tool_mm / 1000.0
    result = []
    for joint in range(6):
        reach = 0.0
        for sign in (1.0, -1.0):
            for step in np.arange(1.0, span + 1e-9, 1.0):
                probe = risk.joint_rad.copy()
                probe[joint] += sign * np.deg2rad(step)
                if np.any(probe < lower) or np.any(probe > upper):
                    break
                if worst_margin(evaluate(probe, tool_m), args) <= 1.0:
                    break
                reach = max(reach, float(step))
        result.append(reach)
    return result


def danger_margins(risk: Risk, args: argparse.Namespace) -> dict:
    """Headroom down to the danger band (not the slowdown band), the tighter pair of limits."""
    return {
        "ratio_over_danger": risk.ratio / args.stop_ratio,
        "j3_over_danger": abs(risk.j3) / np.deg2rad(args.stop_joint_deg),
        "j5_over_danger": abs(risk.j5) / np.deg2rad(args.stop_joint_deg),
    }


def quaternion_wxyz(rotation: np.ndarray) -> np.ndarray:
    trace = float(np.trace(rotation))
    if trace > 0.0:
        scale = np.sqrt(trace + 1.0) * 2.0
        quat = np.array(
            [
                0.25 * scale,
                (rotation[2, 1] - rotation[1, 2]) / scale,
                (rotation[0, 2] - rotation[2, 0]) / scale,
                (rotation[1, 0] - rotation[0, 1]) / scale,
            ]
        )
    else:
        index = int(np.argmax(np.diag(rotation)))
        if index == 0:
            scale = np.sqrt(1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2]) * 2.0
            quat = np.array(
                [
                    (rotation[2, 1] - rotation[1, 2]) / scale,
                    0.25 * scale,
                    (rotation[0, 1] + rotation[1, 0]) / scale,
                    (rotation[0, 2] + rotation[2, 0]) / scale,
                ]
            )
        elif index == 1:
            scale = np.sqrt(1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2]) * 2.0
            quat = np.array(
                [
                    (rotation[0, 2] - rotation[2, 0]) / scale,
                    (rotation[0, 1] + rotation[1, 0]) / scale,
                    0.25 * scale,
                    (rotation[1, 2] + rotation[2, 1]) / scale,
                ]
            )
        else:
            scale = np.sqrt(1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1]) * 2.0
            quat = np.array(
                [
                    (rotation[1, 0] - rotation[0, 1]) / scale,
                    (rotation[0, 2] + rotation[2, 0]) / scale,
                    (rotation[1, 2] + rotation[2, 1]) / scale,
                    0.25 * scale,
                ]
            )
    quat = quat / np.linalg.norm(quat)
    return quat if quat[0] >= 0.0 else -quat


def rpy_deg(rotation: np.ndarray) -> np.ndarray:
    """URDF-convention rpy (R = Rz(yaw) @ Ry(pitch) @ Rx(roll)), in degrees."""
    sy = float(np.hypot(rotation[0, 0], rotation[1, 0]))
    if sy > 1e-9:
        roll = np.arctan2(rotation[2, 1], rotation[2, 2])
        pitch = np.arctan2(-rotation[2, 0], sy)
        yaw = np.arctan2(rotation[1, 0], rotation[0, 0])
    else:
        roll = np.arctan2(-rotation[1, 2], rotation[1, 1])
        pitch = np.arctan2(-rotation[2, 0], sy)
        yaw = 0.0
    return np.rad2deg([roll, pitch, yaw])


def self_clearance_m(risk: Risk, args: argparse.Namespace) -> float:
    return float(batch_self_clearance(risk.joint_rad.reshape(1, 6), args.tool_mm / 1000.0)[0])


def describe(risk: Risk, args: argparse.Namespace, index: int, probe: bool) -> dict:
    ratio_margin, j3_margin, j5_margin = margins(risk, args)
    record = {
        "id": index,
        "joint_deg": np.round(np.rad2deg(risk.joint_rad), 3).tolist(),
        "tcp_xyz_m": np.round(risk.position, 5).tolist(),
        "tcp_quat_wxyz": np.round(quaternion_wxyz(risk.rotation), 6).tolist(),
        "tcp_rpy_deg": np.round(rpy_deg(risk.rotation), 3).tolist(),
        "sigma_min": risk.sigma_min,
        "sigma_max": risk.sigma_max,
        "ratio": risk.ratio,
        "ratio_over_slowdown": ratio_margin,
        "j3_deg": float(np.rad2deg(risk.j3)),
        "j5_deg": float(np.rad2deg(risk.j5)),
        "j3_over_slowdown": j3_margin,
        "j5_over_slowdown": j5_margin,
        "worst_margin": min(ratio_margin, j3_margin, j5_margin),
        "tcp_radius_m": float(np.linalg.norm(risk.position)),
        "self_clearance_m": round(self_clearance_m(risk, args), 4),
        **danger_margins(risk, args),
    }
    if probe:
        record["joint_robustness_deg"] = joint_robustness(risk, args)
        record["cartesian_clearance_m"] = round(clearance_m(risk, args), 4)
    return record


def print_detail(record: dict) -> None:
    print(f"  --- pose {record['id']} ---")
    print(f"    q_deg         = {record['joint_deg']}")
    print(
        f"    tcp xyz [m]   = [{record['tcp_xyz_m'][0]:.4f}, {record['tcp_xyz_m'][1]:.4f},"
        f" {record['tcp_xyz_m'][2]:.4f}]   radius {record['tcp_radius_m']:.4f} m"
    )
    print(f"    tcp quat wxyz = {record['tcp_quat_wxyz']}")
    print(f"    tcp rpy [deg] = {record['tcp_rpy_deg']}")
    print(
        f"    sigma_min={record['sigma_min']:.5f} sigma_max={record['sigma_max']:.5f}"
        f" ratio={record['ratio']:.5f} = {record['ratio_over_slowdown']:.2f}x slowdown"
        f" / {record['ratio'] / 0.01:.2f}x danger"
    )
    print(
        f"    J3={record['j3_deg']:+.2f} deg ({record['j3_over_slowdown']:.2f}x slowdown),"
        f" J5={record['j5_deg']:+.2f} deg ({record['j5_over_slowdown']:.2f}x slowdown)"
    )
    print(f"    worst margin {record['worst_margin']:.2f}x | segment self-clearance {record['self_clearance_m']:.3f} m")
    print(
        f"    headroom to the danger band: ratio {record['ratio_over_danger']:.1f}x,"
        f" J3 {record['j3_over_danger']:.1f}x, J5 {record['j5_over_danger']:.1f}x"
    )
    if "joint_robustness_deg" in record:
        print(f"    single-joint travel to the band [deg]: {record['joint_robustness_deg']}")
        print(f"    mean straight-line Cartesian clearance to the band [m]: {record['cartesian_clearance_m']}")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--tool-mm", type=float, default=161.2, help="tool offset along flange z (controller DH d6)")
    parser.add_argument("--samples", type=int, default=60000)
    parser.add_argument("--refine-top", type=int, default=40)
    parser.add_argument("--keep", type=int, default=6)
    parser.add_argument("--sweep-start-m", type=float, default=0.35, help="first TCP radius band of the sweep")
    parser.add_argument("--sweep-step-m", type=float, default=0.05)
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--slowdown-ratio", type=float, default=0.04)
    parser.add_argument("--stop-ratio", type=float, default=0.01)
    parser.add_argument("--slowdown-joint-deg", type=float, default=15.0)
    parser.add_argument("--stop-joint-deg", type=float, default=5.0)
    parser.add_argument("--joint-limit-deg", type=float, default=150.0, help="absolute cap applied to every joint")
    parser.add_argument("--min-radius-m", type=float, default=0.30)
    parser.add_argument("--max-radius-m", type=float, default=REACH_TCP_M)
    parser.add_argument(
        "--min-self-clearance-m",
        type=float,
        default=0.06,
        help="minimum segment-to-segment arm-torso/wrist distance; 0 disables the filter",
    )
    parser.add_argument("--max-height-m", type=float, default=None)
    parser.add_argument("--min-height-m", type=float, default=None, help="reject TCP points below this base-frame height")
    parser.add_argument(
        "--min-ratio",
        type=float,
        default=0.15,
        help="required sigma_min/sigma_max for a candidate pose (runtime slowdown threshold is 0.04)",
    )
    parser.add_argument(
        "--min-joint-deg",
        type=float,
        default=60.0,
        help="required |J3| and |J5| for a candidate pose (runtime slowdown threshold is 15 deg)",
    )
    parser.add_argument("--joint-robust-span-deg", type=float, default=90.0)
    parser.add_argument("--dedup-m", type=float, default=0.15)
    parser.add_argument("--clearance-step-m", type=float, default=0.02)
    parser.add_argument("--clearance-max-m", type=float, default=0.15)
    parser.add_argument("--json-out", type=Path, default=None)
    args = parser.parse_args()

    rng = np.random.default_rng(args.seed)
    accepted = clear_positions(args, rng)
    print(
        f"tool offset {args.tool_mm:.1f} mm along flange z | sampled {args.samples} | clear candidates {len(accepted)}",
        flush=True,
    )
    if not accepted:
        print("no clear configuration found; relax --joint-limit-deg/--min-radius-m or raise --samples")
        return 1

    pool = accepted[: args.refine_top]
    refined = sorted((refine(risk, args) for risk in pool), key=lambda item: worst_margin(item, args), reverse=True)

    extremes = {
        "max ratio": max(refined, key=lambda item: item.ratio),
        "max |J3|": max(refined, key=lambda item: abs(item.j3)),
        "max |J5|": max(refined, key=lambda item: abs(item.j5)),
        "max worst-margin": refined[0],
    }

    print(
        f"thresholds: ratio slowdown {args.slowdown_ratio} / danger {args.stop_ratio};"
        f" |J3|,|J5| slowdown {args.slowdown_joint_deg} deg / danger {args.stop_joint_deg} deg"
    )
    print(
        f"candidate floors: ratio >= {args.min_ratio}, |J3|,|J5| >= {args.min_joint_deg} deg"
    )
    print(
        f"joint caps +-{args.joint_limit_deg} deg and RM65-B limits shrunk by 5 deg |"
        f" workspace radius {args.min_radius_m:.2f}-{args.max_radius_m:.2f} m,"
        f" TCP z {args.min_height_m} .. {args.max_height_m} m,"
        f" self-clearance >= {args.min_self_clearance_m:.3f} m"
    )

    for name, risk in extremes.items():
        print(f"\n########## {name} ##########")
        print_detail(describe(risk, args, 0, probe=False))

    chosen = deduplicate(refined, args, args.keep)
    table = [describe(risk, args, index + 1, probe=True) for index, risk in enumerate(chosen)]
    print(f"\n########## recommended set: {len(table)} deduplicated singularity-free TCP poses ##########")
    header = (
        "  id |     x       y       z   |  ratio r/sd |    J3      J5   | j3/sd j5/sd | radius | clr[m] | self[m]"
    )
    print(header)
    for record in table:
        print(
            f"  {record['id']:>2} |"
            f" {record['tcp_xyz_m'][0]:7.4f} {record['tcp_xyz_m'][1]:7.4f} {record['tcp_xyz_m'][2]:7.4f} |"
            f" {record['ratio']:.4f} {record['ratio_over_slowdown']:5.2f} |"
            f" {record['j3_deg']:7.2f} {record['j5_deg']:7.2f} |"
            f" {record['j3_over_slowdown']:5.2f} {record['j5_over_slowdown']:5.2f} |"
            f" {record['tcp_radius_m']:6.4f} | {record['cartesian_clearance_m']:6.4f} |"
            f" {record['self_clearance_m']:6.3f}"
        )
    for record in table:
        print()
        print_detail(record)

    sweep = radial_sweep(args, rng, candidates=accepted)
    print("\n########## best achievable worst-margin per TCP radius band ##########")
    print(
        "  radius band [m] |  ratio  r/sd |    J3      J5   | worst margin |  tcp xyz [m]         "
        "| tcp quat wxyz"
    )
    sweep_records = []
    for low, high, risk in sweep:
        record = describe(risk, args, 0, probe=False)
        sweep_records.append({"band_m": [low, high], **record})
        print(
            f"   {low:.2f} - {high:.2f}   | {record['ratio']:.4f} {record['ratio_over_slowdown']:5.2f} |"
            f" {record['j3_deg']:7.2f} {record['j5_deg']:7.2f} | {record['worst_margin']:11.2f}x |"
            f" [{record['tcp_xyz_m'][0]:+.4f}, {record['tcp_xyz_m'][1]:+.4f}, {record['tcp_xyz_m'][2]:+.4f}] |"
            f" {[round(value, 4) for value in record['tcp_quat_wxyz']]}"
        )
    if sweep:
        peak = max(sweep, key=lambda item: worst_margin(item[2], args))
        print(
            f"\n  the cleanest TCP radius band is {peak[0]:.2f}-{peak[1]:.2f} m"
            f" (worst margin {worst_margin(peak[2], args):.2f}x);"
            " the ratio stays healthy across the whole band and falls off towards full extension"
        )
        print("  full joint vectors and per-joint travel for the band peaks:")
        for low, high, risk in sweep:
            record = describe(risk, args, 0, probe=True)
            print(
                f"    {low:.2f}-{high:.2f} m: q_deg={record['joint_deg']} "
                f"robust={record['joint_robustness_deg']} clr={record['cartesian_clearance_m']} "
                f"self={record['self_clearance_m']}"
            )

    best = refined[0]
    azimuth = np.rad2deg(np.arctan2(best.position[1], best.position[0])) % 360.0
    print(
        f"\nreference: best worst-margin pose sits at base azimuth {azimuth:.1f} deg,"
        f" radius {np.linalg.norm(best.position):.4f} m, height {best.position[2]:.4f} m"
    )
    print(
        "reminder: vendor structural chain, identity work frame assumed,"
        " tool offset = controller d6; verify against UDP feedback on the real controller before use."
    )

    if args.json_out is not None:
        args.json_out.write_text(
            json.dumps(
                {
                    "tool_mm": args.tool_mm,
                    "thresholds": {
                        "slowdown_ratio": args.slowdown_ratio,
                        "stop_ratio": args.stop_ratio,
                        "slowdown_joint_deg": args.slowdown_joint_deg,
                        "stop_joint_deg": args.stop_joint_deg,
                    },
                    "extremes": {name: describe(risk, args, 0, probe=False) for name, risk in extremes.items()},
                    "recommended": table,
                    "radial_sweep": sweep_records,
                },
                indent=2,
            ),
            encoding="utf-8",
        )
        print(f"wrote {args.json_out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
