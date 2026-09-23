"""Fail-closed RM65 Cartesian teleoperation with bounded commands and UDP safety feedback."""

from collections.abc import Callable
import math
import time
from typing import TYPE_CHECKING

import meshcat.transformations as tf
import numpy as np

if TYPE_CHECKING:
    from xrobotoolkit_teleop.common.xr_client import XrClient

from xrobotoolkit_teleop.hardware.motion_diagnostics import MotionLog
from xrobotoolkit_teleop.hardware.cartesian_command_limiter import (
    CartesianCommandLimiter,
    shortest_world_rotation_vector,
)
from xrobotoolkit_teleop.hardware.control_timing import FixedRateScheduler, LoopTimingStats
from xrobotoolkit_teleop.hardware.interface.realman_rm65 import RealmanRM65Interface
from xrobotoolkit_teleop.hardware.realman_official_ik import (
    RM65IKSolver,
    RealmanOfficialRemoteIK,
)
from xrobotoolkit_teleop.hardware.realman_udp_feedback import ArmFeedback, RealmanUDPFeedbackReceiver
from xrobotoolkit_teleop.hardware.rm65_model import RM65_B_SPEC
from xrobotoolkit_teleop.hardware.damped_ik import ControllerFrameKinematics, DampedIK, DLSBudgetExceeded
from xrobotoolkit_teleop.hardware.teleop_input_conditioning import (
    continuous_radial_deadzone,
    rotation_gain_for_linear_speed,
    xr_pose_step,
)
from xrobotoolkit_teleop.utils.geometry import (
    R_HEADSET_TO_WORLD_RM65_SIDE_MOUNT_HARDWARE,
    apply_delta_pose,
    quat_diff_as_angle_axis,
)


class RealmanRM65SafeTeleopController:
    """RM65 teleoperation with one TCP writer and independent UDP safety feedback."""

    def __init__(
        self,
        arm_host: str = "192.168.10.18",
        arm_port: int = 8080,
        pose_source: str = "right_controller",
        control_trigger: str = "right_grip",
        scale_factor: float = 0.25,
        rotation_scale_factor: float = 0.25,
        control_rate_hz: float = 50.0,
        move_v: int = 5,
        move_r: int = 80,
        max_offset_m: float = 0.03,
        max_rot_offset_rad: float = 0.20,
        motion_deadband_m: float = 0.003,
        rot_deadband_rad: float = 0.04,
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
        max_tracking_error_m: float = 0.12,
        max_tracking_error_rad: float = 0.80,
        reset_position_tolerance_m: float = 0.01,
        reset_rotation_tolerance_rad: float = 0.08,
        reset_settle_timeout_s: float = 3.0,
        workspace_min_xyz_m: list[float] | None = None,
        workspace_max_xyz_m: list[float] | None = None,
        motion_command: str = "damped_ik_movej_follow",
        official_ik_tool_or_work: int = 1,
        official_ik_frames_verified: bool = False,
        official_ik_j3_soft_limit_verified: bool = False,
        expected_product_prefix: str = "RM65-B",
        udp_bind_host: str = "0.0.0.0",
        udp_port: int = 8089,
        udp_feedback_timeout_s: float = 0.20,
        max_linear_velocity_m_s: float = 0.05,
        max_linear_acceleration_m_s2: float = 0.40,
        max_angular_velocity_rad_s: float = 0.25,
        max_angular_acceleration_rad_s2: float = 1.20,
        max_xr_step_m: float = 0.05,
        max_xr_rot_step_rad: float = 0.26,
        rotation_full_gain_linear_speed_m_s: float = 0.05,
        rotation_zero_gain_linear_speed_m_s: float = 0.25,
        max_joint_speed_ratio: float = 0.10,
        rearm_stationary_speed_rad_s: float = math.radians(1.0),
        enable_singularity_avoidance: bool = True,
        singularity_slowdown_ratio: float = 0.04,
        singularity_stop_ratio: float = 0.01,
        singularity_slowdown_joint_deg: float = 15.0,
        singularity_stop_joint_deg: float = 5.0,
        singularity_min_speed_scale: float = 0.05,
        singularity_escape_speed_scale: float = 0.10,
        singularity_max_damping: float = 0.08,
        singularity_report_interval_s: float = 2.0,
        dls_compute_budget_s: float = 0.018,
        dls_max_fk_calls: int = 16,
        dls_max_candidate_attempts: int = 3,
        dls_max_consecutive_failures: int = 3,
        dls_work_from_base: list[float] | None = None,
        dls_flange_to_tcp: list[float] | None = None,
        dry_run: bool = False,
        motion_log_path: str = "",
        self_test_ramp: bool = False,
        self_test_axis: str = "+x",
        self_test_distance_m: float = 0.02,
        *,
        arm: RealmanRM65Interface | None = None,
        xr: "XrClient | None" = None,
        feedback_receiver: RealmanUDPFeedbackReceiver | None = None,
        ik_solver: RM65IKSolver | None = None,
        dls_kinematics=None,
        monotonic_fn: Callable[[], float] = time.monotonic,
        sleep_fn: Callable[[float], None] = time.sleep,
    ) -> None:
        self.pose_source = pose_source
        self.control_trigger = control_trigger
        self.scale_factor = self._finite(scale_factor, "scale_factor")
        self.rotation_scale_factor = self._finite(rotation_scale_factor, "rotation_scale_factor")
        self.control_rate_hz = self._positive(control_rate_hz, "control_rate_hz")
        self.dt = 1.0 / self.control_rate_hz
        self._monotonic = monotonic_fn
        self.max_offset_m = self._positive(max_offset_m, "max_offset_m")
        self.max_rot_offset_rad = self._positive(max_rot_offset_rad, "max_rot_offset_rad")
        self.motion_deadband_m = self._non_negative(motion_deadband_m, "motion_deadband_m")
        self.rot_deadband_rad = self._non_negative(rot_deadband_rad, "rot_deadband_rad")
        self.grip_on_threshold = self._finite(grip_on_threshold, "grip_on_threshold")
        self.grip_off_threshold = self._finite(grip_off_threshold, "grip_off_threshold")
        if self.grip_off_threshold >= self.grip_on_threshold:
            raise ValueError("grip_off_threshold must be below grip_on_threshold")
        if not send_stop_on_release:
            raise ValueError("send_stop_on_release cannot be disabled on the safe hardware path")
        self.send_stop_on_release = True
        self.log_motion_debug = bool(log_motion_debug)
        self.reset_button = str(reset_button)
        self.reset_duration_s = self._positive(reset_duration_s, "reset_duration_s")
        self.xr_watchdog_timeout_s = self._positive(xr_watchdog_timeout_s, "xr_watchdog_timeout_s")
        self.max_tracking_error_m = self._positive(max_tracking_error_m, "max_tracking_error_m")
        self.max_tracking_error_rad = self._positive(max_tracking_error_rad, "max_tracking_error_rad")
        self.reset_position_tolerance_m = self._positive(reset_position_tolerance_m, "reset_position_tolerance_m")
        self.reset_rotation_tolerance_rad = self._positive(reset_rotation_tolerance_rad, "reset_rotation_tolerance_rad")
        self.reset_settle_timeout_s = self._positive(reset_settle_timeout_s, "reset_settle_timeout_s")
        if motion_command not in {"movep_follow", "movel", "official_ik_movej_follow", "damped_ik_movej_follow"}:
            raise ValueError("unknown motion_command")
        if enable_singularity_avoidance != (motion_command == "damped_ik_movej_follow"):
            raise ValueError(
                "singularity avoidance requires damped_ik_movej_follow; legacy modes require explicit "
                "--no-enable-singularity-avoidance and do not provide DLS protection"
            )
        if isinstance(official_ik_tool_or_work, bool) or official_ik_tool_or_work not in {0, 1}:
            raise ValueError("official_ik_tool_or_work must be 0 (tool) or 1 (work)")
        if motion_command == "official_ik_movej_follow":
            if not official_ik_frames_verified:
                raise ValueError("official_ik_frames_verified must be true after checking controller work/tool frames")
            if not official_ik_j3_soft_limit_verified:
                raise ValueError(
                    "official_ik_j3_soft_limit_verified must be true after configuring a non-zero J3 soft limit"
                )
        self.motion_command = motion_command
        self.expected_product_prefix = str(expected_product_prefix).strip()
        if not self.expected_product_prefix:
            raise ValueError("expected_product_prefix must be non-empty")
        if isinstance(udp_port, bool) or not isinstance(udp_port, int) or not 1 <= udp_port <= 65535:
            raise ValueError("udp_port must be an integer from 1 through 65535")
        if isinstance(move_v, bool) or not isinstance(move_v, int) or not 1 <= move_v <= 100:
            raise ValueError("move_v must be an integer from 1 through 100")
        if isinstance(move_r, bool) or not isinstance(move_r, int) or not 0 <= move_r <= 100:
            raise ValueError("move_r must be an integer from 0 through 100")
        self.udp_feedback_timeout_s = self._positive(udp_feedback_timeout_s, "udp_feedback_timeout_s")
        self.max_xr_step_m = self._positive(max_xr_step_m, "max_xr_step_m")
        self.max_xr_rot_step_rad = self._positive(max_xr_rot_step_rad, "max_xr_rot_step_rad")
        self.rotation_full_gain_linear_speed_m_s = self._non_negative(
            rotation_full_gain_linear_speed_m_s,
            "rotation_full_gain_linear_speed_m_s",
        )
        self.rotation_zero_gain_linear_speed_m_s = self._positive(
            rotation_zero_gain_linear_speed_m_s,
            "rotation_zero_gain_linear_speed_m_s",
        )
        if self.rotation_full_gain_linear_speed_m_s >= self.rotation_zero_gain_linear_speed_m_s:
            raise ValueError("rotation full-gain speed must be below zero-gain speed")
        self.max_joint_speed_ratio = self._positive(max_joint_speed_ratio, "max_joint_speed_ratio")
        if self.max_joint_speed_ratio > 1.0:
            raise ValueError("max_joint_speed_ratio must not exceed 1.0")
        self.rearm_stationary_speed_rad_s = self._non_negative(
            rearm_stationary_speed_rad_s,
            "rearm_stationary_speed_rad_s",
        )
        self.dls = self._build_dls_solver(
            motion_command=motion_command,
            dls_kinematics=dls_kinematics,
            dls_work_from_base=dls_work_from_base,
            dls_flange_to_tcp=dls_flange_to_tcp,
            dls_compute_budget_s=dls_compute_budget_s,
            dls_max_fk_calls=dls_max_fk_calls,
            dls_max_candidate_attempts=dls_max_candidate_attempts,
            singularity_slowdown_ratio=singularity_slowdown_ratio,
            singularity_stop_ratio=singularity_stop_ratio,
            singularity_slowdown_joint_deg=singularity_slowdown_joint_deg,
            singularity_stop_joint_deg=singularity_stop_joint_deg,
            singularity_min_speed_scale=singularity_min_speed_scale,
            singularity_escape_speed_scale=singularity_escape_speed_scale,
            singularity_max_damping=singularity_max_damping,
        )
        self.dls_max_consecutive_failures = self._positive_int(
            dls_max_consecutive_failures,
            "dls_max_consecutive_failures",
        )
        self.singularity_report_interval_s = self._positive(
            singularity_report_interval_s,
            "singularity_report_interval_s",
        )
        self.dry_run = bool(dry_run)
        self.invert_tcp_xy = bool(invert_tcp_xy)
        self.suppress_rotation_during_translation = bool(suppress_rotation_during_translation)
        self.freeze_rotation = bool(freeze_rotation)
        self.R_headset_world = R_HEADSET_TO_WORLD_RM65_SIDE_MOUNT_HARDWARE if use_headset_world_transform else np.eye(3)

        self.arm = arm or RealmanRM65Interface(
            host=arm_host,
            port=arm_port,
            move_v=move_v,
            move_r=move_r,
            command_wait_s=0.005,
            state_wait_s=0.08,
            workspace_min_xyz_m=workspace_min_xyz_m,
            workspace_max_xyz_m=workspace_max_xyz_m,
        )
        self._configure_self_test(self_test_ramp, self_test_axis, self_test_distance_m)
        self.xr = xr
        if not self.self_test_ramp and self.xr is None:
            from xrobotoolkit_teleop.common.xr_client import XrClient

            self.xr = XrClient()
        self.feedback_receiver = feedback_receiver or RealmanUDPFeedbackReceiver(
            bind_host=udp_bind_host,
            port=udp_port,
        )
        self.model_spec = RM65_B_SPEC
        self.ik_solver = ik_solver
        if self.motion_command == "official_ik_movej_follow" and self.ik_solver is None:
            self.ik_solver = RealmanOfficialRemoteIK(
                control_period_s=self.dt,
                tool_or_work=official_ik_tool_or_work,
            )
        self.limiter = CartesianCommandLimiter(
            max_linear_velocity_m_s=max_linear_velocity_m_s,
            max_linear_acceleration_m_s2=max_linear_acceleration_m_s2,
            max_angular_velocity_rad_s=max_angular_velocity_rad_s,
            max_angular_acceleration_rad_s2=max_angular_acceleration_rad_s2,
        )
        # Same per-joint per-cycle budget the official IK path already enforces.
        self._joint_step_budget_rad = self.model_spec.max_joint_velocity_rad_s * self.max_joint_speed_ratio * self.dt
        self.max_linear_velocity_m_s = self._positive(max_linear_velocity_m_s, "max_linear_velocity_m_s")
        self.max_angular_velocity_rad_s = self._positive(max_angular_velocity_rad_s, "max_angular_velocity_rad_s")
        self._initialize_runtime_state(
            sleep_fn=sleep_fn,
            motion_log_path=motion_log_path,
            max_linear_velocity_m_s=max_linear_velocity_m_s,
            max_linear_acceleration_m_s2=max_linear_acceleration_m_s2,
            max_angular_velocity_rad_s=max_angular_velocity_rad_s,
            max_angular_acceleration_rad_s2=max_angular_acceleration_rad_s2,
        )

    def _build_dls_solver(
        self,
        *,
        motion_command: str,
        dls_kinematics,
        dls_work_from_base: list[float] | None,
        dls_flange_to_tcp: list[float] | None,
        dls_compute_budget_s: float,
        dls_max_fk_calls: int,
        dls_max_candidate_attempts: int,
        singularity_slowdown_ratio: float,
        singularity_stop_ratio: float,
        singularity_slowdown_joint_deg: float,
        singularity_stop_joint_deg: float,
        singularity_min_speed_scale: float,
        singularity_escape_speed_scale: float,
        singularity_max_damping: float,
    ) -> DampedIK | None:
        """Build the DLS backend while preserving the flat public constructor API."""
        self._load_controller_dh = dls_kinematics is None
        self._load_controller_tool = dls_flange_to_tcp is None
        if motion_command != "damped_ik_movej_follow":
            return None

        dls_budget = self._positive(dls_compute_budget_s, "dls_compute_budget_s")
        if dls_budget >= self.dt:
            raise ValueError("dls_compute_budget_s must be below the control period")
        kinematics = dls_kinematics or RealmanOfficialRemoteIK(self.dt, fk_only=True)
        return DampedIK(
            ControllerFrameKinematics(kinematics, dls_work_from_base, dls_flange_to_tcp),
            slowdown_ratio=self._positive(singularity_slowdown_ratio, "singularity_slowdown_ratio"),
            stop_ratio=self._positive(singularity_stop_ratio, "singularity_stop_ratio"),
            slowdown_joint_rad=math.radians(
                self._positive(singularity_slowdown_joint_deg, "singularity_slowdown_joint_deg")
            ),
            stop_joint_rad=math.radians(self._non_negative(singularity_stop_joint_deg, "singularity_stop_joint_deg")),
            min_speed_scale=self._positive(singularity_min_speed_scale, "singularity_min_speed_scale"),
            escape_speed_scale=self._positive(
                singularity_escape_speed_scale,
                "singularity_escape_speed_scale",
            ),
            max_damping=self._positive(singularity_max_damping, "singularity_max_damping"),
            compute_budget_s=dls_budget,
            max_fk_calls=self._positive_int(dls_max_fk_calls, "dls_max_fk_calls", minimum=13),
            max_candidate_attempts=self._positive_int(
                dls_max_candidate_attempts,
                "dls_max_candidate_attempts",
                maximum=4,
            ),
            monotonic_fn=self._monotonic,
        )

    def _configure_self_test(self, enabled: bool, axis: str, distance_m: float) -> None:
        """Validate the optional self-test independently from normal teleoperation state."""
        self.self_test_ramp = bool(enabled)
        self.self_test_axis = str(axis)
        self.self_test_distance_m = self._positive(distance_m, "self_test_distance_m")
        if not self.self_test_ramp:
            return
        if self.self_test_axis not in {"+x", "-x", "+y", "-y", "+z", "-z"}:
            raise ValueError("self_test_axis must be +x, -x, +y, -y, +z or -z (controller TCP frame)")
        if not 0.02 <= self.self_test_distance_m <= 0.03:
            raise ValueError("self_test_distance_m must be between 0.02 and 0.03")
        if self.self_test_distance_m > self.max_offset_m:
            raise ValueError("self-test distance exceeds max_offset_m")
        if self.motion_command not in {"movep_follow", "damped_ik_movej_follow"}:
            raise ValueError("self-test requires movep_follow or damped_ik_movej_follow")

    def _initialize_runtime_state(
        self,
        *,
        sleep_fn: Callable[[float], None],
        motion_log_path: str,
        max_linear_velocity_m_s: float,
        max_linear_acceleration_m_s2: float,
        max_angular_velocity_rad_s: float,
        max_angular_acceleration_rad_s2: float,
    ) -> None:
        """Initialize mutable session state after all configuration has been validated."""
        self._singularity_zone = "clear"
        self._singularity_hold = False
        self._singularity_hold_reported = False
        self._last_singularity_report_t = -math.inf
        self._sleep = sleep_fn
        self.scheduler: FixedRateScheduler | None = None
        self.timing_stats = LoopTimingStats()

        # Last locally accepted joint command, never a replacement for UDP feedback.
        self._last_accepted_joint_rad: np.ndarray | None = None
        self._dls_velocity = np.zeros(6)
        self._dls_stopping = False
        self._dls_consecutive_failures = 0
        self.grip_active = False
        self.ref_arm_xyz: np.ndarray | None = None
        self.ref_arm_quat: np.ndarray | None = None
        self.ref_ctrl_xyz: np.ndarray | None = None
        self.ref_ctrl_quat: np.ndarray | None = None
        self.home_xyz: np.ndarray | None = None
        self.home_quat: np.ndarray | None = None
        self._resetting = False
        self._reset_start_xyz: np.ndarray | None = None
        self._reset_start_quat: np.ndarray | None = None
        self._reset_step = 0
        self._reset_started_t: float | None = None
        self._reset_steps_total = max(1, int(round(self.reset_duration_s / self.dt)))
        self._last_reset_pressed = False
        self._last_xr_timestamp_ns: int | None = None
        self._last_xr_update_t: float | None = None
        self._last_valid_xr_pose: np.ndarray | None = None
        self._last_valid_xr_t: float | None = None
        self._current_rotation_gain = 1.0
        self._rearm_required = False
        self._rearm_release_seen = False
        self._rearm_button_required = False
        self._last_singularity_warning_t = -math.inf
        self._last_debug_summary_t = 0.0
        self._last_motion_preview_t = -math.inf
        self._last_cycle_start_t: float | None = None
        self._last_send_duration_s = 0.0
        self._started = False
        self._motion_log = MotionLog(motion_log_path) if motion_log_path else None
        self.arm.motion_diagnostics_enabled = self._motion_log is not None
        self._cycle_index = 0
        self._diagnostic_row = None
        self._self_test_started_t = None
        self._self_test_done = False
        self._self_test_origin = None
        self._self_test_quat = None
        self._self_test_speed = min(0.02, max_linear_velocity_m_s)
        self._self_test_limiter = (
            CartesianCommandLimiter(
                self._self_test_speed,
                max_linear_acceleration_m_s2,
                max_angular_velocity_rad_s,
                max_angular_acceleration_rad_s2,
            )
            if self.self_test_ramp
            else None
        )

    @staticmethod
    def _finite(value: float, name: str) -> float:
        try:
            result = float(value)
        except (TypeError, ValueError) as exc:
            raise ValueError(f"{name} must be finite") from exc
        if not math.isfinite(result):
            raise ValueError(f"{name} must be finite")
        return result

    @classmethod
    def _positive(cls, value: float, name: str) -> float:
        result = cls._finite(value, name)
        if result <= 0.0:
            raise ValueError(f"{name} must be positive")
        return result

    @classmethod
    def _non_negative(cls, value: float, name: str) -> float:
        result = cls._finite(value, name)
        if result < 0.0:
            raise ValueError(f"{name} must be non-negative")
        return result

    @staticmethod
    def _positive_int(value: int, name: str, *, minimum: int = 1, maximum: int | None = None) -> int:
        if isinstance(value, bool) or not isinstance(value, int) or value < minimum:
            raise ValueError(f"{name} must be an integer of at least {minimum}")
        if maximum is not None and value > maximum:
            raise ValueError(f"{name} must not exceed {maximum}")
        return value

    def _process_xr_pose(self, xr_pose: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        xr_pose = np.asarray(xr_pose, dtype=float)
        if xr_pose.shape != (7,) or not np.all(np.isfinite(xr_pose)):
            raise ValueError("XR pose must be a finite array with shape (7,)")
        controller_xyz = xr_pose[:3].copy()
        controller_quat = np.array([xr_pose[6], xr_pose[3], xr_pose[4], xr_pose[5]], dtype=float)
        if float(np.linalg.norm(controller_quat)) < 1e-9:
            raise ValueError("XR pose contains a zero quaternion")
        controller_xyz = self.R_headset_world @ controller_xyz
        rotation_transform = np.eye(4)
        rotation_transform[:3, :3] = self.R_headset_world
        rotation_quat = tf.quaternion_from_matrix(rotation_transform)
        controller_quat = tf.quaternion_multiply(
            tf.quaternion_multiply(rotation_quat, controller_quat),
            tf.quaternion_conjugate(rotation_quat),
        )
        if self.ref_ctrl_xyz is None:
            self.ref_ctrl_xyz = controller_xyz.copy()
            self.ref_ctrl_quat = controller_quat.copy()
            if self._diagnostic_row is not None:
                self._diagnostic_row.update(delta_pos_norm=0.0, delta_rot_norm=0.0)
            return np.zeros(3, dtype=float), np.zeros(3, dtype=float)

        delta_xyz = (controller_xyz - self.ref_ctrl_xyz) * self.scale_factor
        delta_rot = quat_diff_as_angle_axis(self.ref_ctrl_quat, controller_quat) * self.rotation_scale_factor
        translation_norm = float(np.linalg.norm(delta_xyz))
        if self._diagnostic_row is not None:
            self._diagnostic_row["delta_pos_norm"] = translation_norm
            self._diagnostic_row["delta_rot_norm"] = float(np.linalg.norm(delta_rot))
        if translation_norm > self.max_offset_m:
            delta_xyz *= self.max_offset_m / translation_norm
        rotation_norm = float(np.linalg.norm(delta_rot))
        if rotation_norm > self.max_rot_offset_rad:
            delta_rot *= self.max_rot_offset_rad / rotation_norm
        delta_xyz = continuous_radial_deadzone(delta_xyz, self.motion_deadband_m)
        delta_rot = continuous_radial_deadzone(delta_rot, self.rot_deadband_rad)
        if self.invert_tcp_xy:
            delta_xyz[:2] *= -1.0
        if self.suppress_rotation_during_translation:
            delta_rot *= self._current_rotation_gain
        if self.freeze_rotation:
            delta_rot[:] = 0.0
        return delta_xyz, delta_rot

    def _xr_sample_is_fresh(self, now: float) -> bool:
        timestamp_ns = int(self.xr.get_timestamp_ns())
        if self._diagnostic_row is not None:
            self._diagnostic_row["xr_timestamp_ns"] = timestamp_ns
        if timestamp_ns <= 0:
            return False
        if timestamp_ns != self._last_xr_timestamp_ns:
            self._last_xr_timestamp_ns = timestamp_ns
            self._last_xr_update_t = now
        return self._last_xr_update_t is not None and now - self._last_xr_update_t <= self.xr_watchdog_timeout_s

    def _enter_rearm(self, require_button: bool) -> None:
        self._last_accepted_joint_rad = None
        self._rearm_required = True
        self._rearm_release_seen = False
        self._rearm_button_required = require_button

    def _latch_safety_fault(self, reason: str) -> None:
        self.grip_active = False
        self._resetting = False
        self._clear_anchor()
        self._enter_rearm(require_button=True)
        self.arm.latch_fault(reason)

    def _feedback_age(self, now: float) -> float:
        feedback = self.feedback_receiver.latest()
        if feedback is None:
            return math.inf
        return max(0.0, now - feedback.received_monotonic_s)

    def _validate_feedback(
        self,
        feedback: ArmFeedback,
        *,
        check_overspeed: bool,
        check_tracking: bool,
        now: float,
    ) -> str | None:
        if feedback.arm_error_codes:
            return f"RM65 reported error codes: {feedback.arm_error_codes}"
        valid, reason = self.model_spec.validate_joint_feedback(feedback.joint_rad, np.deg2rad(2.0))
        if not valid:
            return reason
        if feedback.joint_speed_rad_s is None:
            return "RM65 UDP feedback does not contain joint_speed"
        if check_overspeed:
            limits = self.model_spec.max_joint_velocity_rad_s * self.max_joint_speed_ratio
            violations = np.flatnonzero(np.abs(feedback.joint_speed_rad_s) > limits)
            if violations.size:
                index = int(violations[0])
                actual = math.degrees(float(abs(feedback.joint_speed_rad_s[index])))
                limit = math.degrees(float(limits[index]))
                return f"J{index + 1} overspeed: {actual:.2f}deg/s > {limit:.2f}deg/s"
        if check_tracking:
            commanded_pose6 = self.arm.get_last_commanded_pose6()
            if commanded_pose6 is not None:
                commanded_xyz, commanded_quat = self.arm.pose6_to_xyz_quat(commanded_pose6)
                position_error = float(np.linalg.norm(commanded_xyz - feedback.tcp_xyz_m))
                rotation_error = float(np.linalg.norm(quat_diff_as_angle_axis(feedback.tcp_quat_wxyz, commanded_quat)))
                if position_error > self.max_tracking_error_m or rotation_error > self.max_tracking_error_rad:
                    return (
                        "RM65 tracking error exceeded limit: "
                        f"position={position_error:.3f}m, rotation={rotation_error:.3f}rad"
                    )
        warnings = self.model_spec.singularity_warnings(feedback.joint_rad, np.deg2rad(10.0))
        if warnings and now - self._last_singularity_warning_t >= 5.0:
            self._last_singularity_warning_t = now
            print(f"[SAFETY WARN] near singularity: {', '.join(warnings)}")
        return None

    def _checked_feedback(
        self,
        now: float,
        *,
        require_fresh: bool,
        check_overspeed: bool = True,
        check_tracking: bool = True,
    ) -> ArmFeedback | None:
        thread_error = self.feedback_receiver.thread_error
        if thread_error is not None:
            self._latch_safety_fault(thread_error)
            return None
        feedback = self.feedback_receiver.latest()
        if feedback is None:
            if require_fresh:
                self._latch_safety_fault("RM65 UDP feedback unavailable")
            return None
        age = now - feedback.received_monotonic_s
        if age < -0.01:
            self._latch_safety_fault("RM65 UDP feedback timestamp is in the future")
            return None
        if require_fresh and age > self.udp_feedback_timeout_s:
            self._latch_safety_fault(f"RM65 UDP feedback stale: age={age:.3f}s")
            return None
        reason = self._validate_feedback(
            feedback,
            check_overspeed=check_overspeed,
            check_tracking=check_tracking,
            now=now,
        )
        if reason is not None:
            self._latch_safety_fault(reason)
            return None
        return feedback

    def _configure_dls_from_controller(self) -> None:
        """Read actual model/tool parameters; never write controller configuration."""
        if self.dls is None or not self._load_controller_dh:
            return

        def query(command, field, value):
            messages = self.arm.send_json({"command": command}, wait_s=1.0)
            for message in reversed(messages):
                if message.get(field) == value:
                    return message
            raise RuntimeError(f"failed to read {command}; refusing default FK parameters")

        dh = query("get_DH_data", "command", "get_DH_data")
        frames = self.dls.kinematics
        tool = None
        if self._load_controller_tool:
            tool = query("get_current_tool_frame", "state", "current_tool_frame")
            tool_pose = frames.official.controller_tool_pose(tool)
        frames.official.configure_controller_dh(dh)
        if tool is not None:
            frames.flange_to_tcp = frames._pose(tool_pose)
            frames._tool_identity = False
        self.dls.begin_cycle()
        print(
            "[INFO] DLS loaded controller DH; "
            f"d6={dh['joint_6'][2] / 1e6:.6f}m; "
            f"tool={tool.get('tool_name') if tool is not None else 'explicit transform'}; "
            "work/base transform unchanged; FK/UDP gate remains enabled"
        )

    def _startup(self) -> None:
        self.feedback_receiver.start()
        self.arm.connect()
        info = self.arm.get_arm_software_info()
        if info is None:
            raise RuntimeError("failed to read RM65 software information")
        product = info.get("Product_version", info.get("product_version"))
        if not isinstance(product, str) or not product.casefold().startswith(self.expected_product_prefix.casefold()):
            raise RuntimeError(f"expected {self.expected_product_prefix}, got {product!r}")
        controller_generation = str(info.get("robot_controller_version", "")).strip()
        if controller_generation.startswith("4"):
            raise RuntimeError("this controller expects third-generation joint_speed units, got generation 4")
        print(f"[INFO] motion_command={self.motion_command} dry_run={self.dry_run}")
        print("[INFO] RM65 preflight " f"product={product}, ctrl={info.get('ctrl_info')}, plan={info.get('plan_info')}")
        self._configure_dls_from_controller()
        deadline = self._monotonic() + max(1.0, 5.0 * self.udp_feedback_timeout_s)
        feedback = self.feedback_receiver.latest()
        while feedback is None and self._monotonic() < deadline:
            self._sleep(0.01)
            feedback = self.feedback_receiver.latest()
        if feedback is None:
            raise RuntimeError("no valid RM65 UDP feedback received during startup")
        now = self._monotonic()
        if now - feedback.received_monotonic_s > self.udp_feedback_timeout_s:
            raise RuntimeError("initial RM65 UDP feedback is stale")
        reason = self._validate_feedback(
            feedback,
            check_overspeed=True,
            check_tracking=False,
            now=now,
        )
        if reason is not None:
            raise RuntimeError(reason)
        if self.dls is not None:
            self.dls.begin_cycle()
            self._dls_frame_matches(feedback)
        self.home_xyz = feedback.tcp_xyz_m.copy()
        self.home_quat = feedback.tcp_quat_wxyz.copy()
        self.arm.sync_command_reference(self.arm.xyz_quat_to_pose6(feedback.tcp_xyz_m, feedback.tcp_quat_wxyz))
        self.scheduler = FixedRateScheduler(self.control_rate_hz, now)
        self._started = True
        print(f"[INFO] home pose captured from UDP xyz_m={np.round(self.home_xyz, 4).tolist()}")

    def _clear_anchor(self) -> None:
        self._dls_velocity = np.zeros(6)
        self._dls_stopping = False
        self._dls_consecutive_failures = 0
        self._last_accepted_joint_rad = None
        self.ref_arm_xyz = None
        self.ref_arm_quat = None
        self.ref_ctrl_xyz = None
        self.ref_ctrl_quat = None
        self._last_valid_xr_pose = None
        self._last_valid_xr_t = None
        self._current_rotation_gain = 1.0
        self.limiter.clear()
        self.limiter.set_speed_scale(1.0)
        self._singularity_zone = "clear"
        self._singularity_hold = False
        self._singularity_hold_reported = False

    def _reset_anchor(self, feedback: ArmFeedback) -> None:
        self._dls_velocity = np.zeros(6)
        self._dls_consecutive_failures = 0
        # The next accepted command starts a new segment from fresh measured joints.
        self._last_accepted_joint_rad = None
        self.ref_arm_xyz = feedback.tcp_xyz_m.copy()
        self.ref_arm_quat = feedback.tcp_quat_wxyz.copy()
        self.ref_ctrl_xyz = None
        self.ref_ctrl_quat = None
        self._last_valid_xr_pose = None
        self._last_valid_xr_t = None
        self.limiter.reset(feedback.tcp_xyz_m, feedback.tcp_quat_wxyz)
        self.limiter.set_speed_scale(1.0)
        self.arm.sync_command_reference(self.arm.xyz_quat_to_pose6(feedback.tcp_xyz_m, feedback.tcp_quat_wxyz))
        print(f"[INFO] anchor reset from UDP xyz_m={np.round(self.ref_arm_xyz, 4).tolist()}")

    def _accept_xr_sample(self, xr_pose: np.ndarray, now: float) -> bool:
        xr_pose = np.asarray(xr_pose, dtype=float)
        if self._last_valid_xr_pose is None:
            self._last_valid_xr_pose = xr_pose.copy()
            self._last_valid_xr_t = now
            self._current_rotation_gain = 1.0
            return True
        try:
            translation_step, rotation_step = xr_pose_step(self._last_valid_xr_pose, xr_pose)
        except ValueError as exc:
            self._latch_safety_fault(f"invalid XR pose: {exc}")
            return False
        if translation_step > self.max_xr_step_m or rotation_step > self.max_xr_rot_step_rad:
            self._latch_safety_fault(
                "XR pose jump exceeded limit: " f"translation={translation_step:.3f}m, rotation={rotation_step:.3f}rad"
            )
            return False
        previous_time = self._last_valid_xr_t if self._last_valid_xr_t is not None else now
        sample_dt = max(self.dt, now - previous_time)
        linear_speed = translation_step / sample_dt
        self._current_rotation_gain = rotation_gain_for_linear_speed(
            linear_speed,
            self.rotation_full_gain_linear_speed_m_s,
            self.rotation_zero_gain_linear_speed_m_s,
        )
        self._last_valid_xr_pose = xr_pose.copy()
        self._last_valid_xr_t = now
        return True

    def _solve_joint_target(
        self,
        xyz: np.ndarray,
        quat: np.ndarray,
        measured_joint_rad: np.ndarray,
    ) -> np.ndarray | None:
        """Continue from the accepted command while independently bounding feedback error."""
        if self.ik_solver is None:
            self._latch_safety_fault("official RealMan IK solver is not initialized")
            return None
        try:
            measured = np.array(measured_joint_rad, dtype=float, copy=True)
            if measured.shape != (6,) or not np.all(np.isfinite(measured)):
                raise ValueError("measured joints must be a finite six-joint array")
            previous = self._last_accepted_joint_rad
            seed = measured.copy() if previous is None else previous.copy()
            if self._diagnostic_row is not None:
                self._diagnostic_row.update(
                    joint_measured_rad=measured.tolist(),
                    joint_prev_accepted_rad=None if previous is None else previous.tolist(),
                    joint_seed_rad=seed.tolist(),
                )
            # Do not let an SDK implementation mutate the reference used by our checks.
            target_joint_rad = np.array(
                self.ik_solver.solve(xyz, quat, seed.copy()),
                dtype=float,
                copy=True,
            )
        except Exception as exc:
            self._latch_safety_fault(f"official RM65 IK failed: {exc}")
            return None
        if self._diagnostic_row is not None:
            self._diagnostic_row["joint_candidate_rad"] = target_joint_rad.tolist()
        if target_joint_rad.shape != (6,) or not np.all(np.isfinite(target_joint_rad)):
            self._latch_safety_fault("official RM65 IK returned an invalid joint target")
            return None
        valid, reason = self.model_spec.validate_joint_feedback(target_joint_rad, 0.0)
        if not valid:
            self._latch_safety_fault(f"official RM65 IK target rejected: {reason}")
            return None
        max_step_rad = self.model_spec.max_joint_velocity_rad_s * self.max_joint_speed_ratio * self.dt
        # Keep the original conservative measured-state budget. This is a position
        # discrepancy bound, not a measured velocity or a model of feedback latency.
        # Also bound adjacent commands: two safe targets around a noisy measurement
        # can otherwise be too far apart. Never wrap angles across mechanical limits.
        for reference, label in ((seed, "command continuity"), (measured, "measured feedback")):
            step_rad = np.abs(target_joint_rad - reference)
            violations = np.flatnonzero(step_rad > max_step_rad + 1e-9)
            if violations.size:
                index = int(violations[0])
                self._latch_safety_fault(
                    f"official RM65 IK J{index + 1} step exceeded limit ({label}): "
                    f"{math.degrees(float(step_rad[index])):.3f}deg > "
                    f"{math.degrees(float(max_step_rad[index])):.3f}deg"
                )
                return None
        return target_joint_rad

    def _dls_frame_matches(self, feedback):
        xyz, quat = self.dls.forward(feedback.joint_rad)
        error_m = float(np.linalg.norm(xyz - feedback.tcp_xyz_m))
        error_rad = float(np.linalg.norm(shortest_world_rotation_vector(quat, feedback.tcp_quat_wxyz)))
        if error_m > 0.005 or error_rad > math.radians(2.0):
            raise ValueError(
                f"official FK / UDP frame mismatch: {error_m:.4f}m, "
                f"{math.degrees(error_rad):.2f}deg; verify work/base and flange/TCP transforms"
            )

    def _hold_damped_motion(self, feedback):
        # Stop the controller's already pending trajectory, not just the next Python target.
        if not self._dls_stopping and not self.dry_run:
            if not self.arm.slow_stop():
                raise RuntimeError("DLS hold slow stop was not acknowledged")
        self._dls_stopping = True
        self._singularity_hold = True
        self._dls_velocity = np.zeros(6)
        self._last_accepted_joint_rad = None
        limiter = self._self_test_limiter if self.self_test_ramp else self.limiter
        limiter.reset(feedback.tcp_xyz_m, feedback.tcp_quat_wxyz)
        self.arm.sync_command_reference(self.arm.xyz_quat_to_pose6(feedback.tcp_xyz_m, feedback.tcp_quat_wxyz))
        return True

    def _degrade_damped_motion(self, feedback, exc):
        """Hold the last accepted target; stop only after repeated budget failures."""
        self._dls_consecutive_failures += 1
        self._dls_velocity = np.zeros(6)
        self._singularity_hold = True
        # The Cartesian limiter already advanced before IK was called. Rebase it
        # to measured feedback so unsent targets cannot accumulate across holds.
        limiter = self._self_test_limiter if self.self_test_ramp else self.limiter
        limiter.reset(feedback.tcp_xyz_m, feedback.tcp_quat_wxyz)
        self.arm.sync_command_reference(self.arm.xyz_quat_to_pose6(feedback.tcp_xyz_m, feedback.tcp_quat_wxyz))
        if self._diagnostic_row is not None:
            self._diagnostic_row.update(
                dls_status="hold_budget",
                dls_budget_reason=exc.reason,
                dls_fk_calls=exc.fk_calls,
                dls_elapsed_ms=exc.elapsed_s * 1000.0,
                dls_consecutive_failures=self._dls_consecutive_failures,
                joint_target_accepted=False,
            )
        now = self._monotonic()
        if now - self._last_singularity_report_t >= self.singularity_report_interval_s:
            print(
                "[DLS] hold_budget "
                f"failure={self._dls_consecutive_failures}/{self.dls_max_consecutive_failures} "
                f"fk={exc.fk_calls} elapsed={exc.elapsed_s * 1000:.2f}ms reason={exc.reason}"
            )
            self._last_singularity_report_t = now
        if self._dls_consecutive_failures >= self.dls_max_consecutive_failures:
            return self._hold_damped_motion(feedback)
        return True

    def _send_damped_motion(self, xyz, quat, started):
        """Common admission gate for teleop, B-return and self-test; no Cartesian fallback."""
        if self._rearm_required or self.arm.fault_latched:
            return False
        feedback = None
        try:
            feedback = self._checked_feedback(self._monotonic(), require_fresh=True, check_tracking=True)
            if feedback is None:
                return False
            self.dls.begin_cycle(
                deadline=started + self.dls.compute_budget_s,
                max_fk_calls=self.dls.max_fk_calls,
                enforce_budget=True,
            )
            self._dls_frame_matches(feedback)
            # A new Grip/reset segment must not seed from a still-stopping arm.
            if self._dls_stopping or self._last_accepted_joint_rad is None:
                if np.any(np.abs(feedback.joint_speed_rad_s) > self.rearm_stationary_speed_rad_s):
                    return self._hold_damped_motion(feedback)
            seed = (
                feedback.joint_rad if self._last_accepted_joint_rad is None else self._last_accepted_joint_rad
            ).copy()
            origin, orientation = self.dls.forward(seed)
            valid, reason = self.arm._pose_is_within_bounds(xyz)
            if not valid:
                raise ValueError(reason)
            result = self.dls.step(
                xyz,
                quat,
                seed,
                self.dt,
                self.model_spec.max_joint_velocity_rad_s * self.max_joint_speed_ratio,
                previous_velocity=self._dls_velocity,
                max_linear_speed=self.max_linear_velocity_m_s,
                max_angular_speed=self.max_angular_velocity_rad_s,
                reset_cycle=False,
            )
            severity = self.dls.severity(result.ratio, np.abs(seed[[2, 4]]))
            self._singularity_zone = "danger" if severity >= 1 else ("slowdown" if severity > 0 else "clear")
            if self._diagnostic_row is not None:
                self._diagnostic_row.update(
                    singularity_zone=self._singularity_zone,
                    singularity_severity=severity,
                    joint_prev_accepted_rad=(
                        None if self._last_accepted_joint_rad is None else self._last_accepted_joint_rad.tolist()
                    ),
                    joint_measured_rad=feedback.joint_rad.tolist(),
                    joint_seed_rad=seed.tolist(),
                    joint_candidate_rad=result.joint_rad.tolist(),
                    joint_target_accepted=False,
                    singularity_sigma_ratio=result.ratio,
                    singularity_predicted_ratio=result.predicted_ratio,
                    singularity_speed_scale=result.scale,
                    singularity_damping=result.damping,
                    singularity_hold=result.hold,
                    dls_status=result.status,
                    dls_fk_calls=self.dls.fk_calls,
                    dls_candidate_attempts=self.dls.candidate_attempts,
                )
            now = self._monotonic()
            if now - self._last_singularity_report_t >= self.singularity_report_interval_s:
                if result.hold or result.scale < 1 or self.log_motion_debug:
                    print(f"[DLS] {result.status} ratio={result.ratio:.5f} scale={result.scale:.3f}")
                    self._last_singularity_report_t = now
            if result.hold:
                return self._hold_damped_motion(feedback)
            # Match the actual wire resolution before performing final safety checks.
            target = np.deg2rad(np.round(np.rad2deg(result.joint_rad) * 1000) / 1000)
            if np.max(np.abs(target - seed)) < 1e-10:
                return self._hold_damped_motion(feedback)
            if not self.dls.bounded_path_allowed(
                seed,
                target,
                ratio=result.ratio,
                angles=np.abs(seed[[2, 4]]),
            ):
                return self._hold_damped_motion(feedback)
            # The actual FK pose, not the unreachable requested pose, is the next reference.
            achieved_xyz, achieved_quat = self.dls.forward(target)
            valid, reason = self.arm._pose_is_within_bounds(achieved_xyz)
            if not valid:
                raise ValueError(reason)
            linear = (achieved_xyz - origin) / self.dt
            angular = shortest_world_rotation_vector(orientation, achieved_quat) / self.dt
            if (
                np.linalg.norm(linear) > self.max_linear_velocity_m_s + 1e-6
                or np.linalg.norm(angular) > self.max_angular_velocity_rad_s + 1e-6
            ):
                return self._hold_damped_motion(feedback)
            # Refresh after potentially expensive FK calls; do not send a late or stale target.
            fresh = self._checked_feedback(self._monotonic(), require_fresh=True, check_tracking=True)
            if fresh is None:
                return False
            self._dls_frame_matches(fresh)
            for reference in (seed, fresh.joint_rad):
                if np.any(np.abs(target - reference) > self._joint_step_budget_rad + 1e-9):
                    raise ValueError("DLS joint command continuity / measured feedback step exceeded limit")
            if not self.dls.bounded_path_allowed(
                fresh.joint_rad,
                target,
                ratio=result.ratio,
                angles=np.abs(fresh.joint_rad[[2, 4]]),
            ):
                return self._hold_damped_motion(fresh)
            self.dls.check_budget("final admission")
            if not self.dry_run:
                if not self.arm.send_movej_follow_joint_rad(target.copy(), active=True) or self.arm.fault_latched:
                    raise RuntimeError("DLS joint target submission failed")
            self.arm.sync_command_reference(self.arm.xyz_quat_to_pose6(achieved_xyz, achieved_quat))
            limiter = self._self_test_limiter if self.self_test_ramp else self.limiter
            limiter.rebase(achieved_xyz, achieved_quat, linear, angular)
            self._dls_velocity = (target - seed) / self.dt
            self._last_accepted_joint_rad = target.copy()
            self._dls_consecutive_failures = 0
            self._singularity_hold = False
            self._dls_stopping = False
            if self._diagnostic_row is not None:
                self._diagnostic_row.update(
                    joint_target_accepted=True,
                    joint_candidate_rad=target.tolist(),
                    dls_achieved_xyz=achieved_xyz.tolist(),
                    dls_achieved_quat_wxyz=achieved_quat.tolist(),
                    dls_fk_calls=self.dls.fk_calls,
                    dls_candidate_attempts=self.dls.candidate_attempts,
                    dls_elapsed_ms=self.dls.elapsed_s * 1000.0,
                    dls_consecutive_failures=0,
                )
            return True
        except DLSBudgetExceeded as exc:
            if feedback is None:
                return False
            return self._degrade_damped_motion(feedback, exc)
        except Exception as exc:
            self._latch_safety_fault(f"DLS rejected motion: {exc}")
            return False
        finally:
            self.dls.end_cycle()

    def _send_motion(
        self,
        xyz: np.ndarray,
        quat: np.ndarray,
        measured_joint_rad: np.ndarray | None = None,
    ) -> bool:
        started = self._monotonic()
        sent = None
        if self._motion_log is not None:
            self.arm.motion_diagnostic = None
        try:
            if self.motion_command == "damped_ik_movej_follow":
                accepted = self._send_damped_motion(xyz, quat, started)
                sent = accepted and self._last_accepted_joint_rad is not None and not self.dry_run
                return accepted
            if self.motion_command == "official_ik_movej_follow":
                if self._diagnostic_row is not None:
                    self._diagnostic_row["joint_target_accepted"] = False
                if self.arm.fault_latched:
                    self._enter_rearm(require_button=True)
                    return False
                if self._rearm_required:
                    return False
                if measured_joint_rad is None:
                    self._latch_safety_fault("official RM65 IK requires current UDP joint feedback")
                    return False
                target_joint_rad = self._solve_joint_target(xyz, quat, measured_joint_rad)
                if target_joint_rad is None:
                    return False
                if not self.dry_run:
                    try:
                        sent = self.arm.send_movej_follow_joint_rad(target_joint_rad.copy(), active=True)
                        if not sent or self.arm.fault_latched:
                            self._latch_safety_fault(
                                self.arm.fault_reason or "official RM65 joint target submission failed"
                            )
                            return False
                        self.arm.sync_command_reference(self.arm.xyz_quat_to_pose6(xyz, quat))
                    except Exception as exc:
                        self._latch_safety_fault(f"official RM65 joint target submission failed: {exc}")
                        return False
                # In dry-run this is a virtual acceptance only; feedback checks still
                # apply, so a stationary real arm cannot support an unbounded replay.
                self._last_accepted_joint_rad = target_joint_rad.copy()
                if self._diagnostic_row is not None:
                    self._diagnostic_row["joint_target_accepted"] = True
                return True
            if self.dry_run:
                return True
            if self.motion_command == "movep_follow":
                sent = self.arm.send_movep_follow_xyz_quat(xyz, quat, active=True)
                return sent
            sent = self.arm.movel_xyz_quat(xyz, quat, active=True)
            return sent or not self.arm.fault_latched
        finally:
            self._last_send_duration_s = max(0.0, self._monotonic() - started)
            if self._diagnostic_row is not None:
                limiter = self._self_test_limiter if self.self_test_ramp else self.limiter
                self._diagnostic_row.update(
                    state="reset" if self._resetting else "active",
                    command_xyz=np.asarray(xyz).tolist(),
                    command_quat_wxyz=np.asarray(quat).tolist(),
                    limiter_linear_velocity=limiter.linear_velocity.tolist(),
                    limiter_angular_velocity=limiter.angular_velocity.tolist(),
                    send_started_monotonic=started,
                    send_duration_s=self._last_send_duration_s,
                    send_result=sent,
                    transport=getattr(self.arm, "motion_diagnostic", None),
                )

    def _start_reset(self, feedback: ArmFeedback) -> None:
        if self.home_xyz is None or self.home_quat is None:
            self._latch_safety_fault("reset home pose is unavailable")
            return
        self.grip_active = False
        self._clear_anchor()
        try:
            if not self.dry_run and not self.arm.slow_stop():
                raise RuntimeError("stop was not acknowledged")
            self._dls_stopping = self.dls is not None
        except Exception as exc:
            self._latch_safety_fault(f"RM65 slow stop failed before reset: {exc}")
            return
        self._reset_start_xyz = feedback.tcp_xyz_m.copy()
        self._reset_start_quat = feedback.tcp_quat_wxyz.copy()
        self.limiter.reset(feedback.tcp_xyz_m, feedback.tcp_quat_wxyz)
        # Every return target goes through the same guarded _send_motion path.
        self.limiter.set_speed_scale(1.0)
        self._singularity_zone = "clear"
        self._singularity_hold = False
        self._singularity_hold_reported = False
        self._reset_step = 0
        self._reset_started_t = self._monotonic()
        self._resetting = True
        print(f"[RESET] returning to startup pose over at least {self.reset_duration_s:.2f}s")

    def _tick_reset(self, now: float, feedback: ArmFeedback) -> None:
        if (
            not self._resetting
            or self._reset_start_xyz is None
            or self._reset_start_quat is None
            or self.home_xyz is None
            or self.home_quat is None
        ):
            return
        alpha = min(1.0, self._reset_step / self._reset_steps_total)
        smooth = alpha * alpha * (3.0 - 2.0 * alpha)
        target_xyz = self._reset_start_xyz + smooth * (self.home_xyz - self._reset_start_xyz)
        delta_rot = quat_diff_as_angle_axis(self._reset_start_quat, self.home_quat) * smooth
        _, target_quat = apply_delta_pose(
            self._reset_start_xyz,
            self._reset_start_quat,
            np.zeros(3),
            delta_rot,
        )
        command_xyz, command_quat = self.limiter.step(target_xyz, target_quat, self.dt)
        self._record_target(target_xyz, target_quat)
        if not self._send_motion(command_xyz, command_quat, feedback.joint_rad):
            self._resetting = False
            self._enter_rearm(require_button=True)
            return
        self._reset_step += 1
        if alpha < 1.0:
            return
        position_error = float(np.linalg.norm(self.home_xyz - feedback.tcp_xyz_m))
        rotation_error = float(np.linalg.norm(quat_diff_as_angle_axis(feedback.tcp_quat_wxyz, self.home_quat)))
        if position_error <= self.reset_position_tolerance_m and rotation_error <= self.reset_rotation_tolerance_rad:
            self._resetting = False
            self._reset_start_xyz = None
            self._reset_start_quat = None
            self._reset_started_t = None
            self._clear_anchor()
            self._enter_rearm(require_button=False)
            print("[RESET] complete; release and re-grip to resume teleoperation")
            return
        if (
            self._reset_started_t is not None
            and now - self._reset_started_t > self.reset_duration_s + self.reset_settle_timeout_s
        ):
            self._latch_safety_fault("RM65 reset did not reach home before timeout")

    def _feedback_allows_rearm(self, now: float) -> bool:
        if self.feedback_receiver.thread_error is not None:
            return False
        feedback = self.feedback_receiver.latest()
        if feedback is None:
            return False
        age = now - feedback.received_monotonic_s
        if age < -0.01 or age > self.udp_feedback_timeout_s:
            return False
        reason = self._validate_feedback(
            feedback,
            check_overspeed=False,
            check_tracking=False,
            now=now,
        )
        if reason is not None or feedback.joint_speed_rad_s is None:
            return False
        return bool(np.all(np.abs(feedback.joint_speed_rad_s) <= self.rearm_stationary_speed_rad_s))

    def _handle_rearm(self, grip: float, reset_edge: bool, now: float) -> None:
        if grip <= self.grip_off_threshold:
            self._rearm_release_seen = True
        if not self._rearm_release_seen:
            return
        if self._rearm_button_required:
            if not reset_edge:
                return
            if not self._feedback_allows_rearm(now):
                print("[SAFETY] B ignored: wait for fresh, error-free, stationary joint feedback")
                return
        self.arm.clear_fault()
        self._rearm_required = False
        self._rearm_release_seen = False
        self._rearm_button_required = False
        self._clear_anchor()
        print("[SAFETY] re-arm acknowledged; press Grip again to move")

    def _record_target(self, xyz, quat):
        if self._diagnostic_row is not None:
            self._diagnostic_row.update(target_xyz=xyz.tolist(), target_quat_wxyz=quat.tolist())

    def _diagnostic_state(self):
        if self._rearm_required:
            return "rearm"
        if self._resetting:
            return "reset"
        return "active" if self.grip_active or self.self_test_ramp and not self._self_test_done else "idle"

    def _run_cycle(self, now: float) -> None:
        self._singularity_hold = False
        if self._motion_log is None:
            if self.self_test_ramp:
                self._tick_self_test(now)
            else:
                self._run_teleop_cycle(now)
            return
        self._cycle_index += 1
        self._diagnostic_row = {
            "type": "cycle",
            "t_monotonic": now,
            "cycle_index": self._cycle_index,
            "state": self._diagnostic_state(),
            "dry_run": self.dry_run,
            "self_test": self.self_test_ramp,
            "motion_command": self.motion_command,
            "grip": None,
            "xr_timestamp_ns": None,
            "xr_pose": None,
            "xr_pose_used": False,
            "delta_pos_norm": None,
            "delta_rot_norm": None,
            "target_xyz": None,
            "target_quat_wxyz": None,
            "command_xyz": None,
            "command_quat_wxyz": None,
            "limiter_linear_velocity": None,
            "limiter_angular_velocity": None,
            "send_duration_s": None,
            "send_started_monotonic": None,
            "send_result": None,
            "transport": None,
            "joint_measured_rad": None,
            "joint_prev_accepted_rad": None,
            "joint_seed_rad": None,
            "joint_candidate_rad": None,
            "joint_target_accepted": None,
            "singularity_zone": None,
            "singularity_sigma_min": None,
            "singularity_sigma_ratio": None,
            "singularity_predicted_ratio": None,
            "singularity_severity": None,
            "singularity_speed_scale": None,
            "singularity_damping": None,
            "singularity_step_scale": None,
            "singularity_inward": None,
            "singularity_hold": None,
            "singularity_reasons": None,
            "dls_status": None,
            "dls_fk_calls": None,
            "dls_candidate_attempts": None,
            "dls_elapsed_ms": None,
            "dls_budget_reason": None,
            "dls_consecutive_failures": 0,
        }
        try:
            if self.self_test_ramp:
                self._tick_self_test(now)
            else:
                self._run_teleop_cycle(now)
        finally:
            row = self._diagnostic_row
            if not self.self_test_ramp and row["xr_pose"] is None:
                # Extra diagnostic sample on no-command cycles; never used for motion.
                try:
                    observed_pose = self.xr.get_pose_by_name(self.pose_source)
                    if observed_pose is not None:
                        row["xr_pose"] = np.asarray(observed_pose).tolist()
                except Exception as exc:
                    row["observation_error"] = str(exc)
            age = self._feedback_age(now)
            row.update(
                state_after=self._diagnostic_state(),
                feedback_age_s=age if math.isfinite(age) else None,
                fault_reason=getattr(self.arm, "fault_reason", None),
            )
            self._motion_log.record(row)
            self._diagnostic_row = None

    def _tick_self_test(self, now):
        if self._self_test_done or self._rearm_required or self.arm.fault_latched:
            self._self_test_done = True
            return
        feedback = self._checked_feedback(now, require_fresh=True, check_tracking=True)
        if feedback is None:
            self._self_test_done = True
            return
        direction = np.zeros(3)
        direction["xyz".index(self.self_test_axis[1])] = 1.0 if self.self_test_axis[0] == "+" else -1.0
        if self._self_test_started_t is None:
            self._self_test_started_t = now
            self._self_test_origin = (self.home_xyz if self.home_xyz is not None else feedback.tcp_xyz_m).copy()
            self._self_test_quat = (self.home_quat if self.home_quat is not None else feedback.tcp_quat_wxyz).copy()
            endpoint = self._self_test_origin + direction * self.self_test_distance_m
            valid, reason = self.arm._pose_is_within_bounds(endpoint)
            if not valid:
                self._latch_safety_fault(reason)
                self._self_test_done = True
                return
            self._self_test_limiter.reset(self._self_test_origin, self._self_test_quat)
        elapsed = max(0.0, now - self._self_test_started_t)
        duration = self.self_test_distance_m / self._self_test_speed
        endpoint = self._self_test_origin + direction * self.self_test_distance_m
        if elapsed >= duration:
            measured_error = np.linalg.norm(endpoint - feedback.tcp_xyz_m)
            command_error = np.linalg.norm(endpoint - self._self_test_limiter.position)
            if (self.dry_run and command_error <= 0.001) or (not self.dry_run and measured_error <= 0.001):
                self._self_test_done = True
                if not self.dry_run:
                    self.arm.slow_stop()
                return
            if elapsed > duration + self.reset_settle_timeout_s:
                self._latch_safety_fault("self-test did not reach endpoint before timeout")
                self._self_test_done = True
                return
        target = self._self_test_origin + direction * min(self.self_test_distance_m, self._self_test_speed * elapsed)
        command, quat = self._self_test_limiter.step(target, self._self_test_quat, self.dt)
        valid, reason = self.arm._pose_is_within_bounds(command)
        if not valid or np.linalg.norm(command - self._self_test_origin) > self.max_offset_m + 1e-12:
            self._latch_safety_fault(reason or "self-test command exceeds max_offset_m")
            self._self_test_done = True
            return
        self._record_target(target, self._self_test_quat)
        if not self._send_motion(command, quat, feedback.joint_rad):
            self._enter_rearm(require_button=True)
            self._self_test_done = True

    def _read_control_state(self) -> tuple[float, bool]:
        """Read Grip/reset once so every branch in this cycle uses one input snapshot."""
        grip = float(self.xr.get_key_value_by_name(self.control_trigger))
        if self._diagnostic_row is not None:
            self._diagnostic_row["grip"] = grip
            self._diagnostic_row["xr_timestamp_ns"] = int(self.xr.get_timestamp_ns())
        reset_pressed = bool(self.xr.get_button_state_by_name(self.reset_button))
        reset_edge = reset_pressed and not self._last_reset_pressed
        self._last_reset_pressed = reset_pressed
        return grip, reset_edge

    def _update_grip_state(self, grip: float) -> bool:
        """Apply the existing hysteresis and return whether Grip was active before it."""
        was_active = self.grip_active
        if was_active:
            self.grip_active = grip > self.grip_off_threshold
        else:
            self.grip_active = grip >= self.grip_on_threshold
        return was_active

    def _stop_motion_on_grip_release(self) -> None:
        """Clear the segment anchor and issue the normal acknowledged soft stop."""
        self._clear_anchor()
        try:
            if not self.dry_run and not self.arm.slow_stop():
                raise RuntimeError("stop was not acknowledged")
            self._dls_stopping = self.dls is not None
        except Exception as exc:
            self._latch_safety_fault(f"RM65 slow stop failed on Grip release: {exc}")

    def _send_xr_target(self, now: float, grip: float, feedback: ArmFeedback, xr_pose: np.ndarray) -> None:
        """Transform one accepted XR sample, limit it, then submit one motion command."""
        delta_pos, delta_rot = self._process_xr_pose(xr_pose)
        target_xyz, target_quat = apply_delta_pose(
            self.ref_arm_xyz,
            self.ref_arm_quat,
            delta_pos,
            delta_rot,
        )
        command_xyz, command_quat = self.limiter.step(target_xyz, target_quat, self.dt)
        self._record_target(target_xyz, target_quat)
        if self.log_motion_debug and now - self._last_motion_preview_t >= 0.2:
            self._last_motion_preview_t = now
            # Same transformed controller frame as the mapping, before scale/deadzone.
            xr_delta = self.R_headset_world @ np.asarray(xr_pose[:3]) - self.ref_ctrl_xyz
            print(
                f"[CMD {'DRY-RUN' if self.dry_run else 'LIVE'}] grip={grip:.2f} "
                f"xr_delta_m={np.round(xr_delta, 4).tolist()} "
                f"target_delta_m={np.round(delta_pos, 4).tolist()}\n"
                f"  target_xyz_m={np.round(target_xyz, 4).tolist()} "
                f"command_xyz_m={np.round(command_xyz, 4).tolist()} "
                f"udp_xyz_m={np.round(feedback.tcp_xyz_m, 4).tolist()}\n"
                f"  target_rotvec_deg={np.round(np.rad2deg(delta_rot), 2).tolist()} "
                f"target_quat_wxyz={np.round(target_quat, 4).tolist()} "
                f"command_quat_wxyz={np.round(command_quat, 4).tolist()} "
                f"rot_gain={self._current_rotation_gain:.2f} "
                f"fb_age={self._feedback_age(now):.3f}s"
            )
        if not self._send_motion(command_xyz, command_quat, feedback.joint_rad):
            self._enter_rearm(require_button=True)

    def _run_teleop_cycle(self, now: float) -> None:
        grip, reset_edge = self._read_control_state()

        if self.arm.fault_latched and not self._rearm_required:
            self._enter_rearm(require_button=True)

        if self._rearm_required:
            self._handle_rearm(grip, reset_edge, now)
            return

        if self._resetting:
            if not self._xr_sample_is_fresh(now):
                self._latch_safety_fault("XR tracking sample became stale during reset")
                return
            feedback = self._checked_feedback(now, require_fresh=True, check_tracking=True)
            if feedback is not None:
                self._tick_reset(now, feedback)
            return

        if reset_edge:
            feedback = self._checked_feedback(now, require_fresh=True, check_tracking=False)
            if feedback is not None:
                self._start_reset(feedback)
            return

        was_active = self._update_grip_state(grip)

        if was_active and not self.grip_active:
            self._stop_motion_on_grip_release()
            return

        if not self.grip_active:
            self._checked_feedback(now, require_fresh=False, check_tracking=False)
            return

        if not self._xr_sample_is_fresh(now):
            self._latch_safety_fault("XR tracking sample is stale or missing")
            return

        feedback = self._checked_feedback(now, require_fresh=True, check_tracking=True)
        if feedback is None:
            return

        xr_pose = self.xr.get_pose_by_name(self.pose_source)
        if self._diagnostic_row is not None and xr_pose is not None:
            self._diagnostic_row["xr_pose"] = np.asarray(xr_pose).tolist()
            self._diagnostic_row["xr_pose_used"] = True
        if xr_pose is None:
            self._latch_safety_fault("XR controller pose is unavailable")
            return

        if not was_active:
            self._reset_anchor(feedback)
            return
        if not self._accept_xr_sample(xr_pose, now):
            if not self._rearm_required:
                self._latch_safety_fault("XR controller pose jump exceeded the per-frame safety limit")
            return

        self._send_xr_target(now, grip, feedback, xr_pose)

    def _print_timing_summary(self) -> None:
        if self._motion_log is not None:
            self._motion_log.summary(
                {
                    "missed_periods": self.scheduler.missed_periods if self.scheduler else 0,
                    "valid_packet_count": getattr(self.feedback_receiver, "valid_packet_count", None),
                    "invalid_packet_count": getattr(self.feedback_receiver, "invalid_packet_count", None),
                }
            )
            return
        if self.scheduler is not None:
            self.timing_stats.deadline_misses = self.scheduler.missed_periods
        summary = self.timing_stats.summary()
        print(
            "[TIMING] "
            f"period_p50={summary['period_p50_ms']:.2f}ms "
            f"period_p95={summary['period_p95_ms']:.2f}ms "
            f"period_max={summary['period_max_ms']:.2f}ms "
            f"send_max={summary['send_max_ms']:.2f}ms "
            f"udp_age_max={summary['udp_age_max_ms']:.2f}ms "
            f"missed={summary['deadline_misses']}"
        )

    def run(self) -> None:
        last_cycle_t: float | None = None
        last_summary_t = self._monotonic()
        try:
            if self.self_test_ramp and not self.dry_run:
                confirmation = f"RUN {self.self_test_axis} {self.self_test_distance_m:.3f}m"
                answer = input(
                    "[SELF TEST] Verify controller-frame direction, clear workspace and guard E-stop. "
                    f"No XR/Grip control and no automatic return. Type '{confirmation}': "
                )
                if answer.strip() != confirmation:
                    return
            self._startup()
            while True:
                now = self._monotonic()
                self._last_send_duration_s = 0.0
                self._run_cycle(now)
                if self.self_test_ramp and (self._self_test_done or self._rearm_required):
                    break
                if last_cycle_t is not None:
                    feedback_age = self._feedback_age(now)
                    if np.isfinite(feedback_age):
                        self.timing_stats.record(
                            period_s=now - last_cycle_t,
                            send_s=self._last_send_duration_s,
                            udp_age_s=feedback_age,
                        )
                last_cycle_t = now
                if (self.log_motion_debug or self._motion_log is not None) and now - last_summary_t >= 5.0:
                    self._print_timing_summary()
                    last_summary_t = now
                if self.scheduler is None:
                    raise RuntimeError("fixed-rate scheduler was not initialized")
                next_deadline = self.scheduler.advance(self._monotonic())
                delay = next_deadline - self._monotonic()
                if delay > 0.0:
                    self._sleep(delay)
        except KeyboardInterrupt:
            print("\n[INFO] teleoperation interrupted by user")
        except Exception as exc:
            print(f"\n[ERROR] teleoperation stopped: {exc}")
            try:
                if not self.self_test_ramp or self._started:
                    self.arm.latch_fault(str(exc))
            except Exception:
                pass
        finally:
            self._last_accepted_joint_rad = None
            if self._started:
                self._print_timing_summary()
            try:
                if not self.self_test_ramp or self._started and not self.dry_run:
                    self.arm.stop()
            except Exception:
                pass
            try:
                self.feedback_receiver.stop()
            except Exception:
                pass
            try:
                self.arm.close()
            except Exception:
                pass
            try:
                if self.xr is not None:
                    self.xr.close()
            except Exception:
                pass
            if self._motion_log is not None:
                self._motion_log.close()
