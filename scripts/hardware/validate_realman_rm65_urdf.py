#!/usr/bin/env python3
"""Validate RM65 URDF FK against Realman controller joint/pose feedback."""

from __future__ import annotations

import json
import os
import socket
import sys

import meshcat.transformations as tf
import numpy as np
import placo

from xrobotoolkit_teleop.utils.path_utils import ASSET_PATH

ARM_HOST = os.environ.get("ARM_HOST", "192.168.10.18")
ARM_PORT = int(os.environ.get("ARM_PORT", "8080"))
URDF_PATH = os.environ.get(
    "RM65_URDF",
    os.path.join(ASSET_PATH, "realman/RM65-official/urdf/RM65-official-arm.urdf"),
)
EE_LINK = os.environ.get("RM65_EE_LINK", "r_link6")
JOINT_SCALE = 1000.0  # 0.001 deg per int
POSE_POS_SCALE = 1_000_000.0  # 0.001 mm per int
POSE_ROT_SCALE = 1000.0  # 0.001 rad per int


def fetch_arm_state(host: str, port: int) -> dict:
    sock = socket.create_connection((host, port), timeout=3.0)
    sock.settimeout(0.5)
    payload = (json.dumps({"command": "get_current_arm_state"}) + "\r\n").encode("utf-8")
    sock.sendall(payload)
    buf = b""
    end = __import__("time").time() + 0.5
    while __import__("time").time() < end:
        try:
            chunk = sock.recv(4096)
            if not chunk:
                break
            buf += chunk
        except socket.timeout:
            break
    sock.close()
    for line in buf.decode("utf-8", "ignore").splitlines():
        try:
            obj = json.loads(line)
        except Exception:
            continue
        if isinstance(obj, dict) and obj.get("state") == "current_arm_state":
            return obj.get("arm_state", {})
    raise RuntimeError("No current_arm_state in response")


def joints_to_rad_raw(joint_mdeg: list[int]) -> np.ndarray:
    return np.deg2rad(np.array(joint_mdeg[:6], dtype=float) / JOINT_SCALE)


def pose6_to_matrix(pose6: list[int]) -> np.ndarray:
    xyz_m = np.array(pose6[:3], dtype=float) / POSE_POS_SCALE
    rotvec = np.array(pose6[3:6], dtype=float) / POSE_ROT_SCALE
    angle = float(np.linalg.norm(rotvec))
    if angle < 1e-9:
        quat = np.array([1.0, 0.0, 0.0, 0.0])
    else:
        axis = rotvec / angle
        quat = tf.quaternion_about_axis(angle, axis)
    T = tf.quaternion_matrix(quat)
    T[:3, 3] = xyz_m
    return T


def fk_placo(q_rad: np.ndarray, link_name: str = EE_LINK) -> np.ndarray:
    robot = placo.RobotWrapper(URDF_PATH)
    solver = placo.KinematicsSolver(robot)
    solver.mask_fbase(True)
    names = [f"joint{i}" for i in range(1, 7)]
    for i, name in enumerate(names):
        off = robot.get_joint_offset(name)
        robot.state.q[off] = float(q_rad[i])
    robot.update_kinematics()
    return robot.get_T_world_frame(link_name)


def rot_err_deg(T_a: np.ndarray, T_b: np.ndarray) -> float:
    Ra = T_a[:3, :3]
    Rb = T_b[:3, :3]
    R = Ra.T @ Rb
    tr = float(np.trace(R))
    c = max(-1.0, min(1.0, (tr - 1.0) * 0.5))
    return float(np.rad2deg(np.arccos(c)))


def score_mapping(q_raw: np.ndarray, signs: np.ndarray, offsets: np.ndarray, T_ctrl: np.ndarray) -> tuple[float, float]:
    q = q_raw * signs + offsets
    T_urdf = fk_placo(q)
    pos_err_mm = float(np.linalg.norm(T_urdf[:3, 3] - T_ctrl[:3, 3]) * 1000.0)
    ori_err = rot_err_deg(T_urdf, T_ctrl)
    return pos_err_mm, ori_err


def main() -> int:
    print(f"[INFO] URDF: {URDF_PATH}")
    print(f"[INFO] exists: {os.path.exists(URDF_PATH)}")
    print(f"[INFO] arm: {ARM_HOST}:{ARM_PORT}")

    arm_state = fetch_arm_state(ARM_HOST, ARM_PORT)
    joint_raw = arm_state.get("joint")
    pose_raw = arm_state.get("pose")
    if not isinstance(joint_raw, list) or not isinstance(pose_raw, list):
        print("[ERR] invalid arm_state", arm_state)
        return 1

    q_raw = joints_to_rad_raw(joint_raw)
    T_ctrl = pose6_to_matrix(pose_raw)
    print(f"[RAW] joint_mdeg={joint_raw}")
    print(f"[RAW] joint_deg={[round(v / JOINT_SCALE, 3) for v in joint_raw[:6]]}")
    print(f"[RAW] pose6={pose_raw}")
    print(f"[CTRL] xyz_m={np.round(T_ctrl[:3, 3], 4).tolist()}")
    print(f"[CTRL] rpy_deg={np.round(np.rad2deg(tf.euler_from_matrix(T_ctrl)), 3).tolist()}")

    T_urdf_identity = fk_placo(q_raw)
    print(f"[URDF] xyz_m={np.round(T_urdf_identity[:3, 3], 4).tolist()}")
    print(f"[URDF] rpy_deg={np.round(np.rad2deg(tf.euler_from_matrix(T_urdf_identity)), 3).tolist()}")

    pos_err_mm = float(np.linalg.norm(T_urdf_identity[:3, 3] - T_ctrl[:3, 3]) * 1000.0)
    ori_err_deg = rot_err_deg(T_urdf_identity, T_ctrl)
    print(f"[ERR ] identity signs pos_mm={pos_err_mm:.2f} rot_deg={ori_err_deg:.2f}")

    best = (pos_err_mm, ori_err_deg, np.ones(6), np.zeros(6), "identity")
    sign_options = [-1.0, 1.0]
    for s1 in sign_options:
        for s2 in sign_options:
            for s3 in sign_options:
                for s4 in sign_options:
                    for s5 in sign_options:
                        for s6 in sign_options:
                            signs = np.array([s1, s2, s3, s4, s5, s6], dtype=float)
                            p_mm, o_deg = score_mapping(q_raw, signs, np.zeros(6), T_ctrl)
                            if p_mm < best[0]:
                                best = (p_mm, o_deg, signs, np.zeros(6), "signs")

    print(
        f"[BEST] pos_mm={best[0]:.2f} rot_deg={best[1]:.2f} "
        f"mode={best[4]} signs={best[2].astype(int).tolist()}"
    )

    if best[0] > 50.0:
        print("[WARN] Large FK mismatch (>50mm). URDF frame/units likely inconsistent with controller pose.")
    elif best[0] > 10.0:
        print("[WARN] Moderate FK mismatch (10-50mm). May need tool-frame offset calibration.")
    else:
        print("[OK  ] FK position mismatch is small; URDF joint mapping is plausible.")

  # quick placo joint name offsets
    robot = placo.RobotWrapper(URDF_PATH)
    print("[URDF] joint offsets:", {f"joint{i}": robot.get_joint_offset(f"joint{i}") for i in range(1, 7)})
    return 0


if __name__ == "__main__":
    sys.exit(main())
