"""Historical movel controller for regression/audit; not the recommended live controller."""
import time

import meshcat.transformations as tf
import numpy as np

from xrobotoolkit_teleop.common.xr_client import XrClient
from xrobotoolkit_teleop.hardware.interface.realman_rm65 import RealmanRM65Interface
from xrobotoolkit_teleop.utils.geometry import (
    R_HEADSET_TO_WORLD_RM65_SIDE_MOUNT_HARDWARE,
    apply_delta_pose,
    quat_diff_as_angle_axis,
)


class RealmanRM65CartesianTeleopController:
    """XR delta mapping in controller TCP frame, executed via movel."""

    def __init__(
        self,
        arm_host: str = "192.168.10.18",
        arm_port: int = 8080,
        pose_source: str = "right_controller",
        control_trigger: str = "right_grip",
        scale_factor: float = 1.2,
        control_rate_hz: float = 50.0,
        move_v: int = 30,
        move_r: int = 80,
        max_offset_m: float = 0.25,
        max_rot_offset_rad: float = 1.2,
        motion_deadband_m: float = 0.003,
        rot_deadband_rad: float = 0.04,
        pos_smooth_alpha: float = 0.5,
        rot_smooth_alpha: float = 0.45,
        grip_on_threshold: float = 0.2,
        grip_off_threshold: float = 0.15,
        use_headset_world_transform: bool = True,
        invert_tcp_xy: bool = False,
        suppress_rotation_during_translation: bool = False,
        freeze_rotation: bool = False,
        send_stop_on_release: bool = True,
        log_motion_debug: bool = False,
        reset_button: str = "B",
        reset_duration_s: float = 3.0,
        xr_watchdog_timeout_s: float = 0.20,
        feedback_interval_s: float = 0.20,
        max_tracking_error_m: float = 0.12,
        max_tracking_error_rad: float = 0.80,
        reset_position_tolerance_m: float = 0.01,
        reset_rotation_tolerance_rad: float = 0.08,
        reset_settle_timeout_s: float = 3.0,
        workspace_min_xyz_m: list[float] | None = None,
        workspace_max_xyz_m: list[float] | None = None,
    ):
        self.pose_source = pose_source
        self.control_trigger = control_trigger
        self.scale_factor = scale_factor
        self.dt = 1.0 / max(1.0, control_rate_hz)
        self.max_offset_m = max_offset_m
        self.max_rot_offset_rad = max_rot_offset_rad
        self.motion_deadband_m = motion_deadband_m
        self.rot_deadband_rad = rot_deadband_rad
        self.pos_smooth_alpha = float(np.clip(pos_smooth_alpha, 0.0, 1.0))
        self.rot_smooth_alpha = float(np.clip(rot_smooth_alpha, 0.0, 1.0))
        self.grip_on_threshold = grip_on_threshold
        self.grip_off_threshold = grip_off_threshold
        self.send_stop_on_release = send_stop_on_release
        self.log_motion_debug = log_motion_debug
        self.reset_button = reset_button
        self.reset_duration_s = max(0.2, float(reset_duration_s))
        self.xr_watchdog_timeout_s = float(xr_watchdog_timeout_s)
        self.feedback_interval_s = float(feedback_interval_s)
        self.max_tracking_error_m = float(max_tracking_error_m)
        self.max_tracking_error_rad = float(max_tracking_error_rad)
        self.reset_position_tolerance_m = float(reset_position_tolerance_m)
        self.reset_rotation_tolerance_rad = float(reset_rotation_tolerance_rad)
        self.reset_settle_timeout_s = float(reset_settle_timeout_s)
        if self.xr_watchdog_timeout_s <= 0.0 or self.feedback_interval_s <= 0.0:
            raise ValueError("watchdog and feedback intervals must be positive")
        if self.max_tracking_error_m <= 0.0 or self.max_tracking_error_rad <= 0.0:
            raise ValueError("tracking error limits must be positive")
        if (
            self.reset_position_tolerance_m <= 0.0
            or self.reset_rotation_tolerance_rad <= 0.0
            or self.reset_settle_timeout_s <= 0.0
        ):
            raise ValueError("reset tolerances and settle timeout must be positive")
        self.invert_tcp_xy = invert_tcp_xy
        self.suppress_rotation_during_translation = suppress_rotation_during_translation
        self.freeze_rotation = freeze_rotation
        self.R_headset_world = (
            R_HEADSET_TO_WORLD_RM65_SIDE_MOUNT_HARDWARE if use_headset_world_transform else np.eye(3)
        )
        self.arm = RealmanRM65Interface(
            host=arm_host,
            port=arm_port,
            move_v=move_v,
            move_r=move_r,
            command_wait_s=0.005,
            state_wait_s=0.08,
            workspace_min_xyz_m=workspace_min_xyz_m,
            workspace_max_xyz_m=workspace_max_xyz_m,
        )
        self.xr = XrClient()
        self.grip_active = False
        self.ref_arm_xyz: np.ndarray | None = None
        self.ref_arm_quat: np.ndarray | None = None
        self.ref_ctrl_xyz: np.ndarray | None = None
        self.ref_ctrl_quat: np.ndarray | None = None
        self.filtered_target_xyz: np.ndarray | None = None
        self.filtered_target_quat: np.ndarray | None = None
        self.home_xyz: np.ndarray | None = None
        self.home_quat: np.ndarray | None = None
        self._resetting = False
        self._reset_start_xyz: np.ndarray | None = None
        self._reset_start_quat: np.ndarray | None = None
        self._reset_step = 0
        self._reset_started_t: float | None = None
        self._reset_steps_total = max(1, int(round(self.reset_duration_s / self.dt)))
        self._prev_reset_button = False
        self._last_xr_timestamp_ns: int | None = None
        self._last_xr_update_t: float | None = None
        self._last_feedback_t = 0.0
        self._rearm_required = False

    def _process_xr_pose(self, xr_pose: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        xr_pose = np.asarray(xr_pose, dtype=float)
        if xr_pose.shape != (7,) or not np.all(np.isfinite(xr_pose)):
            raise ValueError("XR pose must be a finite array with shape (7,)")
        controller_xyz = np.array([xr_pose[0], xr_pose[1], xr_pose[2]], dtype=float)
        controller_quat = np.array([xr_pose[6], xr_pose[3], xr_pose[4], xr_pose[5]], dtype=float)
        if float(np.linalg.norm(controller_quat)) < 1e-9:
            raise ValueError("XR pose contains a zero quaternion")
        controller_xyz = self.R_headset_world @ controller_xyz
        R_transform = np.eye(4)
        R_transform[:3, :3] = self.R_headset_world
        R_quat = tf.quaternion_from_matrix(R_transform)
        controller_quat = tf.quaternion_multiply(
            tf.quaternion_multiply(R_quat, controller_quat),
            tf.quaternion_conjugate(R_quat),
        )
        if self.ref_ctrl_xyz is None:
            self.ref_ctrl_xyz = controller_xyz.copy()
            self.ref_ctrl_quat = controller_quat.copy()
            return np.zeros(3, dtype=float), np.zeros(3, dtype=float)

        delta_xyz = (controller_xyz - self.ref_ctrl_xyz) * self.scale_factor
        delta_rot = quat_diff_as_angle_axis(self.ref_ctrl_quat, controller_quat)

        delta_norm = float(np.linalg.norm(delta_xyz))
        if delta_norm > self.max_offset_m:
            delta_xyz = delta_xyz / delta_norm * self.max_offset_m
        if delta_norm < self.motion_deadband_m:
            delta_xyz[:] = 0.0

        rot_norm = float(np.linalg.norm(delta_rot))
        if rot_norm > self.max_rot_offset_rad:
            delta_rot = delta_rot / rot_norm * self.max_rot_offset_rad
        if rot_norm < self.rot_deadband_rad:
            delta_rot[:] = 0.0

        if self.invert_tcp_xy:
            delta_xyz[0] *= -1.0
            delta_xyz[1] *= -1.0

        # Lateral moves naturally tilt the controller; don't couple that into wrist rotation.
        if self.suppress_rotation_during_translation and delta_norm >= self.motion_deadband_m:
            delta_rot[:] = 0.0

        # 临时调试：冻结姿态，只测平移方向（--freeze-rotation true 开启）
        if self.freeze_rotation:
            delta_rot[:] = 0.0

        return delta_xyz, delta_rot

    def _xr_sample_is_fresh(self, now: float) -> bool:
        timestamp_ns = int(self.xr.get_timestamp_ns())
        if timestamp_ns <= 0:
            return False
        if timestamp_ns != self._last_xr_timestamp_ns:
            self._last_xr_timestamp_ns = timestamp_ns
            self._last_xr_update_t = now
        return self._last_xr_update_t is not None and now - self._last_xr_update_t <= self.xr_watchdog_timeout_s

    def _latch_safety_fault(self, reason: str) -> None:
        self.grip_active = False
        self._rearm_required = True
        self._resetting = False
        self._clear_anchor()
        self.arm.latch_fault(reason)

    def _poll_arm_feedback(self, now: float) -> bool:
        if now - self._last_feedback_t < self.feedback_interval_s:
            return True
        self._last_feedback_t = now
        measured_pose6 = self.arm.get_pose6(force=True)
        if measured_pose6 is None:
            self._latch_safety_fault("RM65 feedback timed out")
            return False
        commanded_pose6 = self.arm.get_last_commanded_pose6()
        if commanded_pose6 is None:
            return True
        measured_xyz, measured_quat = self.arm.pose6_to_xyz_quat(measured_pose6)
        commanded_xyz, commanded_quat = self.arm.pose6_to_xyz_quat(commanded_pose6)
        position_error = float(np.linalg.norm(commanded_xyz - measured_xyz))
        rotation_error = float(np.linalg.norm(quat_diff_as_angle_axis(measured_quat, commanded_quat)))
        if position_error > self.max_tracking_error_m or rotation_error > self.max_tracking_error_rad:
            self._latch_safety_fault(
                f"RM65 tracking error exceeded limit: position={position_error:.3f}m, rotation={rotation_error:.3f}rad"
            )
            return False
        return True

    def _smooth_target(self, target_xyz: np.ndarray, target_quat: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        if self.filtered_target_xyz is None:
            self.filtered_target_xyz = target_xyz.copy()
            self.filtered_target_quat = target_quat.copy()
            return target_xyz, target_quat

        self.filtered_target_xyz = (
            self.filtered_target_xyz + self.pos_smooth_alpha * (target_xyz - self.filtered_target_xyz)
        )
        # SLERP-style blend via incremental rotation in angle-axis space.
        delta_q = quat_diff_as_angle_axis(self.filtered_target_quat, target_quat)
        delta_q *= self.rot_smooth_alpha
        angle = float(np.linalg.norm(delta_q))
        if angle > 1e-9:
            rot_delta_quat = tf.quaternion_about_axis(angle, delta_q / angle)
            self.filtered_target_quat = tf.quaternion_multiply(rot_delta_quat, self.filtered_target_quat)
        return self.filtered_target_xyz.copy(), self.filtered_target_quat.copy()

    def _reset_anchor(self) -> None:
        pose6 = self.arm.get_pose6(force=True)
        if pose6 is None:
            raise RuntimeError("Failed to read controller TCP pose for anchor reset")
        xyz, quat = self.arm.pose6_to_xyz_quat(pose6)
        self.arm.sync_command_reference(pose6)
        self.ref_arm_xyz = xyz.copy()
        self.ref_arm_quat = quat.copy()
        self.ref_ctrl_xyz = None
        self.ref_ctrl_quat = None
        self.filtered_target_xyz = xyz.copy()
        self.filtered_target_quat = quat.copy()
        print(f"[INFO] anchor reset at controller xyz_m={np.round(xyz, 4).tolist()}")

    def _clear_anchor(self) -> None:
        self.ref_arm_xyz = None
        self.ref_arm_quat = None
        self.ref_ctrl_xyz = None
        self.ref_ctrl_quat = None
        self.filtered_target_xyz = None
        self.filtered_target_quat = None

    def _capture_home_pose(self) -> None:
        pose6 = self.arm.get_pose6(force=True)
        if pose6 is None:
            raise RuntimeError("Failed to read controller TCP pose for home pose")
        self.home_xyz, self.home_quat = self.arm.pose6_to_xyz_quat(pose6)
        print(f"[INFO] home pose captured at controller xyz_m={np.round(self.home_xyz, 4).tolist()}")

    def _start_reset(self) -> None:
        if self.home_xyz is None or self.home_quat is None:
            print("[WARN] reset ignored: home pose is not available")
            return
        pose6 = self.arm.get_pose6(force=True)
        if pose6 is None:
            print("[WARN] reset ignored: failed to read current TCP pose")
            return
        self._reset_start_xyz, self._reset_start_quat = self.arm.pose6_to_xyz_quat(pose6)
        self.arm.sync_command_reference(pose6)
        self._reset_step = 0
        self._reset_started_t = time.monotonic()
        self._resetting = True
        self.grip_active = False
        self._clear_anchor()
        self.arm.stop()
        print(f"[RESET] returning to startup pose over {self.reset_duration_s:.2f}s")

    def _tick_reset(self) -> None:
        if (
            not self._resetting
            or self._reset_start_xyz is None
            or self._reset_start_quat is None
            or self.home_xyz is None
            or self.home_quat is None
        ):
            return
        alpha = min(1.0, self._reset_step / self._reset_steps_total)
        # Smoothstep gives zero velocity at both ends and avoids a hard jerk.
        s = alpha * alpha * (3.0 - 2.0 * alpha)
        xyz = self._reset_start_xyz + s * (self.home_xyz - self._reset_start_xyz)
        delta_rot = quat_diff_as_angle_axis(self._reset_start_quat, self.home_quat) * s
        _, quat = apply_delta_pose(self._reset_start_xyz, self._reset_start_quat, np.zeros(3), delta_rot)
        self.arm.movel_xyz_quat(xyz, quat, active=True)
        self._reset_step += 1
        if alpha >= 1.0:
            measured_pose6 = self.arm.get_pose6(force=False)
            if measured_pose6 is not None:
                measured_xyz, measured_quat = self.arm.pose6_to_xyz_quat(measured_pose6)
                position_error = float(np.linalg.norm(self.home_xyz - measured_xyz))
                rotation_error = float(np.linalg.norm(quat_diff_as_angle_axis(measured_quat, self.home_quat)))
                if (
                    position_error <= self.reset_position_tolerance_m
                    and rotation_error <= self.reset_rotation_tolerance_rad
                ):
                    self._resetting = False
                    self._reset_start_xyz = None
                    self._reset_start_quat = None
                    self._reset_started_t = None
                    self._rearm_required = True
                    print("[RESET] complete; release and re-grip to resume teleoperation")
                    return
            if self._reset_started_t is not None:
                reset_deadline = self.reset_duration_s + self.reset_settle_timeout_s
                if time.monotonic() - self._reset_started_t > reset_deadline:
                    self._latch_safety_fault("RM65 reset did not reach home before timeout")

    def run(self) -> None:
        print(
            f"[INFO] Cartesian teleop (side-mount, movel), source={self.pose_source}, "
            f"scale={self.scale_factor}, invert_tcp_xy={self.invert_tcp_xy}, "
            f"suppress_rot_during_move={self.suppress_rotation_during_translation}"
        )
        self.arm.connect()
        try:
            self._capture_home_pose()
            while True:
                t0 = time.monotonic()
                grip = float(self.xr.get_key_value_by_name(self.control_trigger))
                xr_fresh = self._xr_sample_is_fresh(t0)
                reset_button = bool(self.xr.get_button_state_by_name(self.reset_button))
                if reset_button and not self._prev_reset_button and not self._resetting and xr_fresh:
                    self._start_reset()
                self._prev_reset_button = reset_button

                if self.arm.fault_latched:
                    self._rearm_required = True

                if self._rearm_required:
                    if grip <= self.grip_off_threshold and xr_fresh:
                        self.arm.clear_fault()
                        self._rearm_required = False
                        print("[SAFETY] fault cleared; press Grip again to re-arm")
                    elapsed = time.monotonic() - t0
                    if elapsed < self.dt:
                        time.sleep(self.dt - elapsed)
                    continue

                if self._resetting:
                    if not xr_fresh:
                        self._latch_safety_fault("XR data watchdog expired during reset")
                        continue
                    if not self._poll_arm_feedback(t0):
                        continue
                    self._tick_reset()
                    elapsed = time.monotonic() - t0
                    if elapsed < self.dt:
                        time.sleep(self.dt - elapsed)
                    continue

                was_active = self.grip_active
                if self.grip_active:
                    if grip < self.grip_off_threshold:
                        self.grip_active = False
                elif grip >= self.grip_on_threshold:
                    if xr_fresh:
                        self.grip_active = True
                    else:
                        self._latch_safety_fault("XR data is stale while Grip is pressed")
                        continue

                if self.grip_active and not xr_fresh:
                    self._latch_safety_fault("XR data watchdog expired")
                    continue

                if was_active and not self.grip_active:
                    print("[INFO] grip released")
                    self._clear_anchor()
                    self.arm.clear_fault()
                    if self.send_stop_on_release:
                        self.arm.stop()
                elif self.grip_active and not was_active:
                    self._reset_anchor()

                if self.grip_active and self.ref_arm_xyz is not None and self.ref_arm_quat is not None:
                    if not self._poll_arm_feedback(t0):
                        continue
                    xr_pose = self.xr.get_pose_by_name(self.pose_source)
                    delta_xyz, delta_rot = self._process_xr_pose(xr_pose)
                    if float(np.linalg.norm(delta_xyz)) > 0.0 or float(np.linalg.norm(delta_rot)) > 0.0:
                        target_xyz, target_quat = apply_delta_pose(
                            self.ref_arm_xyz,
                            self.ref_arm_quat,
                            delta_xyz,
                            delta_rot,
                        )
                        cmd_xyz, cmd_quat = self._smooth_target(target_xyz, target_quat)
                        if self.log_motion_debug:
                            print(
                                f"[DBG] dxyz={np.round(delta_xyz, 4).tolist()} "
                                f"drot={np.round(delta_rot, 4).tolist()} "
                                f"cmd_xyz={np.round(cmd_xyz, 4).tolist()}"
                            )
                        if not self.arm.movel_xyz_quat(cmd_xyz, cmd_quat, active=True) and self.arm.fault_latched:
                            self._rearm_required = True

                elapsed = time.monotonic() - t0
                if elapsed < self.dt:
                    time.sleep(self.dt - elapsed)
        except KeyboardInterrupt:
            print("\n[INFO] stopped by user")
        except Exception as exc:
            # A broken XR/network loop must not leave the last movel running.
            print(f"[ERROR] teleop loop failed: {exc}")
            raise
        finally:
            try:
                self.arm.stop()
            except Exception as stop_exc:
                print(f"[WARN] failed to send arm stop during shutdown: {stop_exc}")
            self.arm.close()
            try:
                self.xr.close()
            except Exception as xr_close_exc:
                print(f"[WARN] failed to close XR client: {xr_close_exc}")
