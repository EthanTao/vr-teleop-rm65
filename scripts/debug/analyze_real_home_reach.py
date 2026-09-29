"""Offline analysis of what the REAL RM65 can do from its fixed Home posture.

Why this exists
---------------
On 2026-09-26 the operator reported that on the real arm (``teleop_realman_rm65_safe_hardware.py``,
live, fixed Home ``[0, 0, -60, 0, 60, 0]``) the hand's *left/right* motion drove the arm normally
while *up/down* and *forward/back* appeared not to move at all.  The simulation URDF cannot answer
this: its FK disagrees with the controller by hundreds of mm (CLAUDE.md 注意事项 2), so its Jacobian
describes a different posture.

This script rebuilds the kinematics the DLS actually uses, from the read-only snapshot
``config/rm65_dls_calibration.json`` (captured from the real controller on 2026-09-24):

* DH rows are ``[alpha(1e-3 deg), a(1e-6 m), d(1e-6 m), offset(1e-3 deg)]`` and are loaded into the
  official SDK as ``alpha/1000, a/1e6, d/1e6, offset/1000`` (``realman_official_ik.configure_controller_dh``).
* The flange -> TCP transform is the tool frame's ``pose`` field, **not** its ``position`` field
  (``realman_official_ik.controller_tool_pose`` uses ``pose``).
* The convention is **modified DH (Craig)**: ``A_i = Rx(alpha_i) Tx(a_i) Rz(theta_i) Tz(d_i)``.
  Validated against two independent field facts: the DLS's startup singularity ratio at the fixed
  Home was 0.09722 (docs/2026.9.26.md) and this model gives 0.09696; all-zeros is the documented
  boundary singularity and this model gives 0.0.

Output: the Home TCP pose, the singularity ratio, the per-direction joint cost per cm, the achievable
TCP speed under the DLS's per-joint budget, the TCP workspace-box headroom, and how far the TCP can
travel along each base-frame axis before a DLS guard or the reach limit stops it.

Run from the repository root:  python scripts/debug/analyze_real_home_reach.py [--home J1 ... J6]
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

D2R = np.pi / 180.0
CAL_PATH = Path(__file__).resolve().parents[2] / "config" / "rm65_dls_calibration.json"
LENGTH = 0.3  # DLS normalizes the linear Jacobian rows by this before the SVD
WORKSPACE_BOX_M = 1.0  # realman_rm65.py default per-axis bound
JOINT_LIMIT_DEG = np.array(
    [[-178, 178], [-130, 130], [-135, 135], [-178, 178], [-128, 128], [-360, 360]], dtype=float
)
MAX_JOINT_SPEED_DEG = np.array([180.0, 180.0, 225.0, 225.0, 225.0, 225.0])
MAX_JOINT_SPEED_RATIO = 0.10  # --max-joint-speed-ratio
MAX_LINEAR_SPEED_M_S = 0.05  # --max-linear-velocity-m-s
STOP_RATIO = 0.01  # --singularity-stop-ratio
SLOWDOWN_RATIO = 0.04  # --singularity-slowdown-ratio
STOP_JOINT_RAD = 5.0 * D2R  # --singularity-stop-joint-deg
SLOWDOWN_JOINT_RAD = 15.0 * D2R  # --singularity-slowdown-joint-deg
DEFAULT_HOME_DEG = (0.0, 0.0, -60.0, 0.0, 60.0, 0.0)


def load_model(cal_path: Path = CAL_PATH) -> dict:
    cal = json.loads(cal_path.read_text(encoding="utf-8"))
    rows = np.array([cal["dh"][f"joint_{i}"] for i in range(1, 7)], dtype=float)
    pose = np.array(cal["tool_frame"]["pose"], dtype=float)
    return {
        "alpha": rows[:, 0] / 1000.0 * D2R,
        "a": rows[:, 1] / 1e6,
        "d": rows[:, 2] / 1e6,
        "offset": rows[:, 3] / 1000.0 * D2R,
        "tool_xyz": pose[:3] / 1e6,
        "tool_rpy": pose[3:] / 1000.0 * D2R,
        "cal_path": cal_path,
    }


def _Rz(t):
    c, s = np.cos(t), np.sin(t)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def _Rx(t):
    c, s = np.cos(t), np.sin(t)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def _Ry(t):
    c, s = np.cos(t), np.sin(t)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def _T(R=np.eye(3), p=(0.0, 0.0, 0.0)):
    m = np.eye(4)
    m[:3, :3] = R
    m[:3, 3] = p
    return m


def _rpy_to_R(roll, pitch, yaw):
    return _Rz(yaw) @ _Ry(pitch) @ _Rx(roll)


def flange_and_axes(model, q):
    """Return (T_base_flange, [(joint axis, joint origin)] in base coordinates)."""
    acc = np.eye(4)
    axes = []
    for i in range(6):
        acc = acc @ _T(_Rx(model["alpha"][i]), (model["a"][i], 0.0, 0.0))
        acc = acc @ _T(_Rz(model["offset"][i] + q[i]), (0.0, 0.0, model["d"][i]))
        axes.append((acc[:3, :3] @ np.array([0.0, 0.0, 1.0]), acc[:3, 3].copy()))
    return acc, axes


def tcp_pose(model, q):
    flange, _ = flange_and_axes(model, q)
    return flange @ _T(_rpy_to_R(*model["tool_rpy"]), model["tool_xyz"])


def jacobian(model, q, normalise=True):
    """Geometric Jacobian of the TCP: rows 0-2 linear, rows 3-5 angular."""
    _, axes = flange_and_axes(model, q)
    tcp = tcp_pose(model, q)
    p = tcp[:3, 3]
    J = np.zeros((6, 6))
    for i, (axis, point) in enumerate(axes):
        J[:3, i] = np.cross(axis, p - point)
        J[3:, i] = axis
    if normalise:
        J[:3] /= LENGTH
    return J


def ratio_and_severity(model, q):
    s = np.linalg.svd(jacobian(model, q), compute_uv=False)
    ratio = float(s[-1] / max(s[0], 1e-12))
    angles = np.abs(q[[2, 4]])
    ratio_ramp = (SLOWDOWN_RATIO - ratio) / (SLOWDOWN_RATIO - STOP_RATIO)
    angle_ramp = (SLOWDOWN_JOINT_RAD - angles) / (SLOWDOWN_JOINT_RAD - STOP_JOINT_RAD)
    return ratio, float(np.clip(max(ratio_ramp, *angle_ramp), 0.0, 1.0)), angles


def speed_scale(severity):
    """damped_ik.DampedIK.step: scale = max(min_speed_scale, 1 - severity)."""
    return max(0.05, 1.0 - severity)


def per_direction_table(model, q_home):
    """For each base-frame axis: joint cost per cm and the achievable TCP speed."""
    J = jacobian(model, q_home, normalise=False)
    speeds = MAX_JOINT_SPEED_DEG * D2R * MAX_JOINT_SPEED_RATIO
    rows = []
    for k, name in enumerate("XYZ"):
        tw = np.zeros(6)
        tw[k] = 0.01  # one centimetre
        dq, *_ = np.linalg.lstsq(J, tw, rcond=None)
        achieved = J[:3] @ dq
        unit = 1.0 if np.linalg.norm(achieved) == 0 else tw[k] / np.linalg.norm(achieved)
        worst = float(np.max(np.abs(dq) / speeds))
        rows.append(
            {
                "axis": name,
                "per_joint_deg_per_cm": np.rad2deg(np.abs(dq)),
                "total_deg_per_cm": float(np.rad2deg(np.linalg.norm(dq))),
                "max_speed_cm_s": 100.0 * unit / worst,
            }
        )
    return rows


def _ik_hold_orientation(model, T_target, q_seed, iters=600, damping=0.01):
    q = q_seed.copy()
    lower = JOINT_LIMIT_DEG[:, 0] * D2R
    upper = JOINT_LIMIT_DEG[:, 1] * D2R
    target_p = T_target[:3, 3]
    for _ in range(iters):
        J = jacobian(model, q)
        T = tcp_pose(model, q)
        position_error = target_p - T[:3, 3]
        R_err = T_target[:3, :3] @ T[:3, :3].T
        angle = np.arccos(np.clip((np.trace(R_err) - 1.0) / 2.0, -1.0, 1.0))
        axis = np.array([R_err[2, 1] - R_err[1, 2], R_err[0, 2] - R_err[2, 0], R_err[1, 0] - R_err[0, 1]])
        norm = np.linalg.norm(axis)
        rot_vec = np.zeros(3) if norm < 1e-12 else axis / norm * angle
        if np.linalg.norm(position_error) < 1e-5 and np.linalg.norm(rot_vec) < 1e-5:
            break
        error = np.concatenate((position_error / LENGTH, rot_vec))
        dq = np.linalg.solve(J.T @ J + damping * damping * np.eye(6), J.T @ error)
        q = np.clip(q + np.clip(dq, -0.02, 0.02), lower, upper)
    return q, float(np.linalg.norm(tcp_pose(model, q)[:3, 3] - target_p))


def travel_limit(model, q_home, direction, step=0.0025, max_travel=0.40):
    """Travel along one unit direction until a DLS guard or the reach limit stops it."""
    T_home = tcp_pose(model, q_home)
    p_home = T_home[:3, 3]
    q = q_home.copy()
    usable = 0.0
    state = {"ratio": np.nan, "angles": np.zeros(2), "margin_deg": np.nan}
    distance = 0.0
    while distance <= max_travel + 1e-9:
        distance += step
        target_p = p_home + np.asarray(direction, dtype=float) * distance
        target = T_home.copy()
        target[:3, 3] = target_p
        q, _ = _ik_hold_orientation(model, target, q)
        if np.linalg.norm(tcp_pose(model, q)[:3, 3] - target_p) > 1e-4:
            return usable, "reach limit", state
        ratio, severity, angles = ratio_and_severity(model, q)
        margin = np.rad2deg(min(np.min(q - JOINT_LIMIT_DEG[:, 0] * D2R), np.min(JOINT_LIMIT_DEG[:, 1] * D2R - q)))
        state = {"ratio": ratio, "angles": np.rad2deg(angles), "margin_deg": margin}
        usable = distance
        if ratio <= STOP_RATIO or severity >= 1.0 or np.any(angles <= STOP_JOINT_RAD) or margin <= 1.0:
            return usable - step, "DLS guard", state
    return usable, "no limit within scan", state


def report(model, home_deg):
    q_home = np.array(home_deg, dtype=float) * D2R
    tcp = tcp_pose(model, q_home)
    p = tcp[:3, 3]
    ratio, severity, angles = ratio_and_severity(model, q_home)
    print(f"DH snapshot        : {model['cal_path']}")
    print(f"Home joint (deg)   : {list(home_deg)}")
    print(f"Home flange xyz (m): {np.round(flange_and_axes(model, q_home)[0][:3, 3], 4).tolist()}")
    print(f"Home TCP xyz (m)   : {np.round(p, 4).tolist()}   |TCP| = {np.linalg.norm(p):.3f} m")
    print(f"tool offset (m)    : {np.round(model['tool_xyz'], 4).tolist()}")
    print(f"singularity ratio  : {ratio:.5f}   severity = {severity:.2f}   speed scale = {speed_scale(severity):.2f}")
    print(f"|J3| = {np.rad2deg(angles[0]):.2f} deg   |J5| = {np.rad2deg(angles[1]):.2f} deg")
    print(
        f"per-axis box headroom (m, bound {WORKSPACE_BOX_M:g}): "
        f"X+ {WORKSPACE_BOX_M - p[0]:.4f} / X- {p[0] + WORKSPACE_BOX_M:.4f} | "
        f"Y+ {WORKSPACE_BOX_M - p[1]:.4f} / Y- {p[1] + WORKSPACE_BOX_M:.4f} | "
        f"Z+ {WORKSPACE_BOX_M - p[2]:.4f} / Z- {p[2] + WORKSPACE_BOX_M:.4f}"
    )

    print("\nper base-frame axis (orientation held):")
    print(f"  {'axis':>4} {'joint cost':>26} {'total':>9} {'achievable TCP speed':>21}")
    for row in per_direction_table(model, q_home):
        cost = " ".join(f"J{i + 1}={v:5.2f}" for i, v in enumerate(row["per_joint_deg_per_cm"]) if v > 0.01)
        print(
            f"  {row['axis']:>4} {cost:>26} {row['total_deg_per_cm']:6.2f}°/cm "
            f"{min(row['max_speed_cm_s'], 100 * MAX_LINEAR_SPEED_M_S):8.2f} cm/s"
        )

    print("\ntravel before a DLS guard / the reach limit stops the command:")
    for name, direction in (
        ("+X", (1, 0, 0)),
        ("-X", (-1, 0, 0)),
        ("+Y", (0, 1, 0)),
        ("-Y", (0, -1, 0)),
        ("+Z", (0, 0, 1)),
        ("-Z", (0, 0, -1)),
    ):
        usable, why, state = travel_limit(model, q_home, direction)
        print(
            f"  {name:>3} {usable * 100:6.1f} cm  ({why:>20})  ratio={state['ratio']:.4f} "
            f"|J3|={state['angles'][0]:7.2f}° |J5|={state['angles'][1]:7.2f}° "
            f"worst joint margin={state['margin_deg']:6.2f}°"
        )
    print(
        "\nNote: 'reach limit' means the offline damped-least-squares search could no longer hold the\n"
        "orientation exactly; it is an upper bound on useful travel, not a measured machine limit."
    )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--home",
        type=float,
        nargs=6,
        default=list(DEFAULT_HOME_DEG),
        metavar=("J1", "J2", "J3", "J4", "J5", "J6"),
        help="joint Home in degrees (default: 0 0 -60 0 60 0)",
    )
    parser.add_argument("--calibration", type=Path, default=CAL_PATH)
    args = parser.parse_args()
    report(load_model(args.calibration), args.home)


if __name__ == "__main__":
    main()
