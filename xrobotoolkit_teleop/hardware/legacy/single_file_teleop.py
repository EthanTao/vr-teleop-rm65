"""Historical single-file mapping diagnostic; live motion remains disabled."""
import json
import socket
import time
from dataclasses import dataclass

import meshcat.transformations as tf
import numpy as np
import tyro
import xrobotoolkit_sdk as xrt

from xrobotoolkit_teleop.utils.geometry import (
    R_HEADSET_TO_WORLD_RM65_SIDE_MOUNT_HARDWARE,
    apply_delta_pose,
    quat_diff_as_angle_axis,
)


@dataclass
class Args:
    arm_host: str = "192.168.10.18"
    arm_port: int = 8080
    pose_source: str = "right"  # right or left controller
    control_mode: str = "movel"  # movel | pos_step
    scale_m_per_m: float = 0.8
    control_rate_hz: float = 20.0
    move_v: int = 8
    move_r: int = 80  # non-zero blend radius for smoother consecutive movel
    pose_unit_scale: float = 1_000_000.0  # Realman pose XYZ unit is 0.001 mm
    rot_unit_scale: float = 1000.0  # Realman pose RxRyRz unit is 0.001 rad
    auto_detect_pose_units: bool = True
    auto_detect_rot_units: bool = True
    max_offset_m: float = 0.20
    max_step_m: float = 0.002
    motion_deadband_m: float = 0.0
    pos_smooth_alpha: float = 1.0
    rot_scale: float = 1.0
    max_rot_offset_rad: float = 1.2
    max_rot_step_rad: float = 0.02
    rot_deadband_rad: float = 0.0
    rot_smooth_alpha: float = 1.0
    state_refresh_hz: float = 4.0  # low-rate real feedback correction
    use_headset_world_transform: bool = True
    enable_rot_step: bool = False  # requires controller support for set_ort_step
    grip_on_threshold: float = 0.75
    grip_off_threshold: float = 0.55
    log_grip_transitions: bool = True
    log_motion_debug: bool = True
    send_stop_on_release: bool = True
    dry_run: bool = False


class RealmanJsonClient:
    def __init__(self, host: str, port: int):
        self.host = host
        self.port = port
        self.sock: socket.socket | None = None

    def connect(self) -> None:
        self.sock = socket.create_connection((self.host, self.port), timeout=3.0)
        self.sock.settimeout(0.2)

    def close(self) -> None:
        if self.sock is not None:
            try:
                self.sock.close()
            except Exception:
                pass
            self.sock = None

    def send(self, payload: dict, wait_s: float = 0.12) -> list[dict]:
        if self.sock is None:
            raise RuntimeError("Realman socket not connected")
        data = (json.dumps(payload, ensure_ascii=False) + "\r\n").encode("utf-8")
        self.sock.sendall(data)
        end_t = time.time() + wait_s
        buf = b""
        while time.time() < end_t:
            try:
                chunk = self.sock.recv(4096)
                if not chunk:
                    break
                buf += chunk
            except socket.timeout:
                break
        msgs = []
        for line in buf.decode("utf-8", "ignore").splitlines():
            line = line.strip()
            if not line:
                continue
            try:
                obj = json.loads(line)
                msgs.append(obj if isinstance(obj, dict) else {"raw": line})
            except Exception:
                msgs.append({"raw": line})
        return msgs

    def get_current_pose(self) -> list[int] | None:
        resp = self.send({"command": "get_current_arm_state"}, wait_s=0.2)
        for item in reversed(resp):
            if not isinstance(item, dict) or item.get("state") != "current_arm_state":
                continue
            arm_state = item.get("arm_state")
            if not isinstance(arm_state, dict):
                continue
            pose = arm_state.get("pose")
            if isinstance(pose, list) and len(pose) >= 6:
                return [int(round(float(v))) for v in pose[:6]]
        return None


def _quat_wxyz_normalize(q: np.ndarray) -> np.ndarray:
    n = float(np.linalg.norm(q))
    if n < 1e-12:
        return np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
    return q / n


def _quat_wxyz_mul(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array(
        [
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        ],
        dtype=float,
    )


def _quat_wxyz_conj(q: np.ndarray) -> np.ndarray:
    return np.array([q[0], -q[1], -q[2], -q[3]], dtype=float)


def _rotmat_to_quat_wxyz(R: np.ndarray) -> np.ndarray:
    m = R
    tr = float(m[0, 0] + m[1, 1] + m[2, 2])
    if tr > 0.0:
        s = np.sqrt(tr + 1.0) * 2.0
        w = 0.25 * s
        x = (m[2, 1] - m[1, 2]) / s
        y = (m[0, 2] - m[2, 0]) / s
        z = (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
        w = (m[2, 1] - m[1, 2]) / s
        x = 0.25 * s
        y = (m[0, 1] + m[1, 0]) / s
        z = (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
        w = (m[0, 2] - m[2, 0]) / s
        x = (m[0, 1] + m[1, 0]) / s
        y = 0.25 * s
        z = (m[1, 2] + m[2, 1]) / s
    else:
        s = np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
        w = (m[1, 0] - m[0, 1]) / s
        x = (m[0, 2] + m[2, 0]) / s
        y = (m[1, 2] + m[2, 1]) / s
        z = 0.25 * s
    return _quat_wxyz_normalize(np.array([w, x, y, z], dtype=float))


def _quat_wxyz_to_rotvec(q: np.ndarray) -> np.ndarray:
    qn = _quat_wxyz_normalize(q)
    if qn[0] < 0.0:
        qn = -qn
    w = float(np.clip(qn[0], -1.0, 1.0))
    angle = 2.0 * np.arccos(w)
    s = np.sqrt(max(0.0, 1.0 - w * w))
    if s < 1e-8 or angle < 1e-8:
        return np.zeros(3, dtype=float)
    axis = qn[1:] / s
    return axis * angle


def _rotvec_to_quat_wxyz(rotvec: np.ndarray) -> np.ndarray:
    angle = float(np.linalg.norm(rotvec))
    if angle < 1e-9:
        return np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
    axis = rotvec / angle
    return _quat_wxyz_normalize(tf.quaternion_about_axis(angle, axis))


def get_controller_pose_world(args: Args) -> tuple[np.ndarray, np.ndarray]:
    if args.pose_source == "left":
        p = xrt.get_left_controller_pose()
    else:
        p = xrt.get_right_controller_pose()
    pos = np.array([float(p[0]), float(p[1]), float(p[2])], dtype=float)
    # xrt quat is [x, y, z, w]
    q_controller = _quat_wxyz_normalize(np.array([float(p[6]), float(p[3]), float(p[4]), float(p[5])], dtype=float))
    if args.use_headset_world_transform:
        pos_world = R_HEADSET_TO_WORLD_RM65_SIDE_MOUNT_HARDWARE @ pos
        q_world = _rotmat_to_quat_wxyz(R_HEADSET_TO_WORLD_RM65_SIDE_MOUNT_HARDWARE)
        q_controller_world = _quat_wxyz_mul(_quat_wxyz_mul(q_world, q_controller), _quat_wxyz_conj(q_world))
    else:
        pos_world = pos
        q_controller_world = q_controller
    return pos_world, q_controller_world


def get_grip(args: Args) -> float:
    if args.pose_source == "left":
        return float(xrt.get_left_grip())
    return float(xrt.get_right_grip())


def clamp_step(target_xyz_m: np.ndarray, current_xyz_m: np.ndarray, max_step_m: float) -> np.ndarray:
    d = target_xyz_m - current_xyz_m
    n = float(np.linalg.norm(d))
    if n <= max_step_m or n < 1e-9:
        return target_xyz_m
    return current_xyz_m + d / n * max_step_m


def _infer_pose_unit_scale_from_state(pose_xyz_raw: list[int], default_scale: float) -> float:
    # Heuristic from observed arm-state ranges:
    # - 0.001 mm unit -> |x| commonly hundreds of thousands
    # - mm unit       -> |x| commonly hundreds
    max_abs = max(abs(int(v)) for v in pose_xyz_raw)
    if max_abs >= 20_000:
        return 1_000_000.0
    if max_abs >= 20:
        return 1_000.0
    return float(default_scale)


def _infer_rot_unit_scale_from_state(pose_r_raw: list[int], default_scale: float) -> float:
    # - 0.001 rad unit -> |r| usually around hundreds to thousands
    # - rad unit       -> |r| usually < 7
    max_abs = max(abs(int(v)) for v in pose_r_raw)
    if max_abs >= 100:
        return 1000.0
    if max_abs <= 10:
        return 1.0
    return float(default_scale)


def main(args: Args) -> None:
    if not args.dry_run:
        raise RuntimeError(
            "This legacy single-file entry is motion-disabled because it can bypass the "
            "current UDP feedback and overspeed protections. Use "
            "scripts/hardware/teleop_realman_rm65_safe_hardware.py instead. "
            "Only --dry-run is retained here for mapping diagnostics."
        )
    print(
        f"[INFO] connect arm {args.arm_host}:{args.arm_port}, source={args.pose_source}, "
        f"mode={args.control_mode}, dry_run={args.dry_run}"
    )
    arm = RealmanJsonClient(args.arm_host, args.arm_port)
    arm.connect()
    xrt.init()
    pose_unit_scale = float(args.pose_unit_scale)
    rot_unit_scale = float(args.rot_unit_scale)

    ctrl_anchor: np.ndarray | None = None
    ctrl_quat_anchor: np.ndarray | None = None
    arm_anchor_pos_m: np.ndarray | None = None
    arm_anchor_quat: np.ndarray | None = None
    arm_anchor_pose: list[int] | None = None
    cmd_pose: list[int] | None = None
    last_sent_t = 0.0
    dt = 1.0 / max(1.0, args.control_rate_hz)
    refresh_dt = 1.0 / max(0.5, args.state_refresh_hz)
    last_refresh_t = 0.0
    grip_active = False
    filtered_target_xyz: np.ndarray | None = None
    filtered_target_rot: np.ndarray | None = None

    try:
        while True:
            t0 = time.time()
            grip = get_grip(args)
            # Hysteresis only (no debounce).
            if grip_active:
                if grip >= args.grip_off_threshold:
                    pass
                else:
                    grip_active = False
                    if args.log_grip_transitions:
                        print(f"[INFO] grip released -> hold (grip={grip:.3f})")
            else:
                if grip >= args.grip_on_threshold:
                    grip_active = True
                    if args.log_grip_transitions:
                        print(f"[INFO] grip engaged (grip={grip:.3f})")
            active = grip_active

            if not active:
                ctrl_anchor = None
                ctrl_quat_anchor = None
                arm_anchor_pos_m = None
                arm_anchor_quat = None
                arm_anchor_pose = None
                cmd_pose = None
                filtered_target_xyz = None
                filtered_target_rot = None
                if args.send_stop_on_release and not args.dry_run:
                    # Deadman behavior: stop trajectory immediately when grip is released.
                    arm.send({"command": "set_arm_stop"}, wait_s=0.02)
            else:
                ctrl_now, ctrl_quat_now = get_controller_pose_world(args)
                if ctrl_anchor is None:
                    pose = arm.get_current_pose()
                    if pose is None:
                        time.sleep(0.05)
                        continue
                    if args.auto_detect_pose_units:
                        pose_unit_scale = _infer_pose_unit_scale_from_state(pose[:3], pose_unit_scale)
                    if args.auto_detect_rot_units:
                        rot_unit_scale = _infer_rot_unit_scale_from_state(pose[3:6], rot_unit_scale)
                    if args.log_grip_transitions:
                        print(
                            f"[INFO] unit_scale pose={pose_unit_scale:g} (raw/m), "
                            f"rot={rot_unit_scale:g} (raw/rad)"
                        )
                    ctrl_anchor = ctrl_now.copy()
                    ctrl_quat_anchor = ctrl_quat_now.copy()
                    arm_anchor_pose = pose
                    arm_anchor_pos_m = np.array(
                        [
                            pose[0] / pose_unit_scale,
                            pose[1] / pose_unit_scale,
                            pose[2] / pose_unit_scale,
                        ],
                        dtype=float,
                    )
                    arm_anchor_rotvec = np.array(
                        [
                            pose[3] / rot_unit_scale,
                            pose[4] / rot_unit_scale,
                            pose[5] / rot_unit_scale,
                        ],
                        dtype=float,
                    )
                    arm_anchor_quat = _rotvec_to_quat_wxyz(arm_anchor_rotvec)
                    cmd_pose = pose[:]
                    print(f"[INFO] grip engaged -> anchor pose={arm_anchor_pose[:3]}")
                else:
                    assert arm_anchor_pose is not None
                    assert ctrl_quat_anchor is not None
                    assert arm_anchor_pos_m is not None
                    assert arm_anchor_quat is not None
                    assert cmd_pose is not None
                    # MuJoCo-style anchor mapping:
                    # target_pose = arm_anchor_pose (+) delta(controller_now - controller_anchor)
                    delta_pos = (ctrl_now - ctrl_anchor) * float(args.scale_m_per_m)
                    delta_norm = float(np.linalg.norm(delta_pos))
                    if delta_norm > float(args.max_offset_m):
                        delta_pos = delta_pos / max(delta_norm, 1e-9) * float(args.max_offset_m)
                    if delta_norm < float(args.motion_deadband_m):
                        delta_pos[:] = 0.0

                    delta_rot = quat_diff_as_angle_axis(ctrl_quat_anchor, ctrl_quat_now) * float(args.rot_scale)
                    delta_rot_norm = float(np.linalg.norm(delta_rot))
                    if delta_rot_norm > float(args.max_rot_offset_rad):
                        delta_rot = delta_rot / max(delta_rot_norm, 1e-9) * float(args.max_rot_offset_rad)
                    if delta_rot_norm < float(args.rot_deadband_rad):
                        delta_rot[:] = 0.0

                    # Optional low-rate feedback correction (avoid every-cycle blocking query).
                    now = time.time()
                    if now - last_refresh_t >= refresh_dt:
                        refreshed = arm.get_current_pose()
                        if refreshed is not None:
                            cmd_pose = refreshed[:]
                        last_refresh_t = now

                    cur_xyz_m = np.array(
                        [
                            cmd_pose[0] / pose_unit_scale,
                            cmd_pose[1] / pose_unit_scale,
                            cmd_pose[2] / pose_unit_scale,
                        ],
                        dtype=float,
                    )
                    cur_rot = np.array(
                        [
                            cmd_pose[3] / rot_unit_scale,
                            cmd_pose[4] / rot_unit_scale,
                            cmd_pose[5] / rot_unit_scale,
                        ],
                        dtype=float,
                    )

                    target_xyz_abs, target_quat_abs = apply_delta_pose(
                        arm_anchor_pos_m,
                        arm_anchor_quat,
                        delta_pos,
                        delta_rot,
                    )
                    target_rot_abs = _quat_wxyz_to_rotvec(target_quat_abs)

                    # Low-pass filter absolute target to improve smoothness.
                    if filtered_target_xyz is None:
                        filtered_target_xyz = target_xyz_abs.copy()
                    else:
                        a = float(np.clip(args.pos_smooth_alpha, 0.0, 1.0))
                        filtered_target_xyz = filtered_target_xyz + a * (target_xyz_abs - filtered_target_xyz)

                    if filtered_target_rot is None:
                        filtered_target_rot = target_rot_abs.copy()
                    else:
                        a_rot = float(np.clip(args.rot_smooth_alpha, 0.0, 1.0))
                        filtered_target_rot = filtered_target_rot + a_rot * (target_rot_abs - filtered_target_rot)

                    next_xyz = clamp_step(filtered_target_xyz, cur_xyz_m, float(args.max_step_m))
                    rot_step = filtered_target_rot - cur_rot
                    rot_step_norm = float(np.linalg.norm(rot_step))
                    if rot_step_norm > float(args.max_rot_step_rad):
                        rot_step = rot_step / rot_step_norm * float(args.max_rot_step_rad)
                    next_rot = cur_rot + rot_step

                    next_pose = cmd_pose[:]
                    next_pose[0] = int(round(next_xyz[0] * pose_unit_scale))
                    next_pose[1] = int(round(next_xyz[1] * pose_unit_scale))
                    next_pose[2] = int(round(next_xyz[2] * pose_unit_scale))
                    next_pose[3] = int(round(next_rot[0] * rot_unit_scale))
                    next_pose[4] = int(round(next_rot[1] * rot_unit_scale))
                    next_pose[5] = int(round(next_rot[2] * rot_unit_scale))
                    if args.log_motion_debug:
                        dx_raw = next_pose[0] - cmd_pose[0]
                        dy_raw = next_pose[1] - cmd_pose[1]
                        dz_raw = next_pose[2] - cmd_pose[2]
                        drx_raw = next_pose[3] - cmd_pose[3]
                        dry_raw = next_pose[4] - cmd_pose[4]
                        drz_raw = next_pose[5] - cmd_pose[5]
                        raw_to_mm = 1000.0 / pose_unit_scale
                        raw_to_mrad = 1000.0 / rot_unit_scale
                        print(
                            f"[DBG] grip={grip:.3f} dxyz_raw=({dx_raw},{dy_raw},{dz_raw}) "
                            f"dxyz_mm=({dx_raw * raw_to_mm:.3f},{dy_raw * raw_to_mm:.3f},{dz_raw * raw_to_mm:.3f}) "
                            f"drot_raw=({drx_raw},{dry_raw},{drz_raw}) "
                            f"drot_mrad=({drx_raw * raw_to_mrad:.3f},{dry_raw * raw_to_mrad:.3f},{drz_raw * raw_to_mrad:.3f}) "
                            f"cur=({cmd_pose[0]},{cmd_pose[1]},{cmd_pose[2]},{cmd_pose[3]},{cmd_pose[4]},{cmd_pose[5]}) "
                            f"tgt=({next_pose[0]},{next_pose[1]},{next_pose[2]},{next_pose[3]},{next_pose[4]},{next_pose[5]})"
                        )

                    if now - last_sent_t >= dt:
                        if args.control_mode == "movel":
                            payload = {"command": "movel", "pose": next_pose, "v": int(args.move_v), "r": int(args.move_r)}
                            if args.dry_run:
                                print("[DRY]", payload)
                            else:
                                arm.send(payload, wait_s=0.01)
                            cmd_pose = next_pose
                        else:
                            # Trajectory-flow style: stream incremental step commands.
                            inc_ctrl_m = (ctrl_now - ctrl_anchor) * float(args.scale_m_per_m)
                            inc_norm = float(np.linalg.norm(inc_ctrl_m))
                            if inc_norm > float(args.max_step_m):
                                inc_ctrl_m = inc_ctrl_m / inc_norm * float(args.max_step_m)
                            inc_step = np.round(inc_ctrl_m * pose_unit_scale).astype(int)
                            axis_map = [("x", int(inc_step[0])), ("y", int(inc_step[1])), ("z", int(inc_step[2]))]
                            for axis, step in axis_map:
                                if abs(step) <= 0:
                                    continue
                                payload = {"command": "set_pos_step", "step_type": axis, "step": step, "v": int(args.move_v)}
                                if args.dry_run:
                                    print("[DRY]", payload)
                                else:
                                    arm.send(payload, wait_s=0.005)

                            if args.enable_rot_step:
                                inc_rot = quat_diff_as_angle_axis(ctrl_quat_anchor, ctrl_quat_now) * float(args.rot_scale)
                                inc_rot_norm = float(np.linalg.norm(inc_rot))
                                if inc_rot_norm > float(args.max_rot_step_rad):
                                    inc_rot = inc_rot / inc_rot_norm * float(args.max_rot_step_rad)
                                inc_rot_step = np.round(inc_rot * rot_unit_scale).astype(int)
                                rot_map = [("rx", int(inc_rot_step[0])), ("ry", int(inc_rot_step[1])), ("rz", int(inc_rot_step[2]))]
                                for axis, step in rot_map:
                                    if abs(step) <= 0:
                                        continue
                                    payload = {"command": "set_ort_step", "step_type": axis, "step": step, "v": int(args.move_v)}
                                    if args.dry_run:
                                        print("[DRY]", payload)
                                    else:
                                        arm.send(payload, wait_s=0.005)

                        # Keep anchor-based mapping active during grip hold.

                        last_sent_t = now

            elapsed = time.time() - t0
            if elapsed < dt:
                time.sleep(dt - elapsed)
    except KeyboardInterrupt:
        print("\n[INFO] stopped by user")
    finally:
        try:
            xrt.close()
        except Exception:
            pass
        arm.close()


if __name__ == "__main__":
    main(tyro.cli(Args))
