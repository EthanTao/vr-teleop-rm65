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


class RM65ControlCore:
    """Shared RM65 control state machine using injected arm, XR and feedback ports.

    Hardware and simulation run the same mapping, limiter, DLS and admission path.
    """

    def __init__(
        self,
        arm_host: str = "192.168.10.18",
        arm_port: int = 8080,
        pose_source: str = "right_controller",
        control_trigger: str = "right_grip",
        scale_factor: float = 1.0,
        rotation_scale_factor: float = 0.25,
        control_rate_hz: float = 50.0,
        move_v: int = 5,
        move_r: int = 80,
        max_offset_m: float | None = 0.15,
        max_rot_offset_rad: float = 0.20,
        motion_deadband_m: float = 0.0,
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
        home_joint_deg: tuple[float, float, float, float, float, float] | None = None,
        auto_home_on_start: bool = False,
        home_joint_tolerance_deg: float = 1.0,
        enable_home_path_preflight: bool = True,
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
        max_tracking_lag_cycles: float = 3.0,
        tracking_lag_hold_dwell_s: float = 0.3,
        max_command_stall_cycles: int = 6,
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
        # None disables the per-Grip translation envelope: the hand offset is then bounded only
        # by the TCP workspace box, the XR single-frame jump check and the joint admission gates.
        # Every explicit value keeps the original positive-number semantics.
        self.max_offset_m = None if max_offset_m is None else self._positive(max_offset_m, "max_offset_m")
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
        self.home_joint_rad = None
        self.home_joint_tolerance_rad = math.radians(
            self._positive(home_joint_tolerance_deg, "home_joint_tolerance_deg")
        )
        self.auto_home_on_start = bool(auto_home_on_start)
        self.enable_home_path_preflight = bool(enable_home_path_preflight)
        if home_joint_deg is not None:
            values = np.asarray(home_joint_deg, dtype=float)
            if values.shape != (6,) or not np.all(np.isfinite(values)):
                raise ValueError("home_joint_deg must contain six finite joint angles")
            self.home_joint_rad = np.deg2rad(values)
            valid, reason = self.model_spec.validate_joint_feedback(self.home_joint_rad, 0.0)
            if not valid:
                raise ValueError(reason)
            if self.dls is None:
                raise ValueError("fixed joint home requires damped_ik_movej_follow")
        if self.auto_home_on_start and self.home_joint_rad is None:
            raise ValueError("auto_home_on_start requires home_joint_deg")
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
        # Two separate budgets, deliberately not the same number:
        #   * command continuity  - how far one sent command may differ from the previous one;
        #   * tracking allowance  - how far the command may lead the *measured* joints.
        # The second one has to absorb the servo/transport lag (measured up to 48 ms on site),
        # so an allowance of one control period only ever measured "the arm has not caught up
        # yet", not joint speed. It is a lag budget, and it is what the lag governor below uses.
        self._joint_step_budget_rad = self.model_spec.max_joint_velocity_rad_s * self.max_joint_speed_ratio * self.dt
        self.max_tracking_lag_cycles = self._positive(max_tracking_lag_cycles, "max_tracking_lag_cycles")
        if self.max_tracking_lag_cycles <= 1.0:
            raise ValueError("max_tracking_lag_cycles must exceed one control period of command lead")
        self.tracking_lag_hold_dwell_s = self._positive(tracking_lag_hold_dwell_s, "tracking_lag_hold_dwell_s")
        self.max_command_stall_cycles = self._positive_int(
            max_command_stall_cycles, "max_command_stall_cycles", minimum=2
        )
        self._joint_lag_limit_rad = self._joint_step_budget_rad * self.max_tracking_lag_cycles
        # A proportional deceleration can only approach the allowance asymptotically, so the
        # hold must trigger below it or persistent degradation could never be reached. With the
        # gain below, the trigger sits exactly at the equilibrium lead of an arm that never
        # moves, which is the smallest value that still lets a dead servo be detected.
        self._joint_lag_hold_trigger_rad = self._joint_lag_limit_rad * (
            (self.max_tracking_lag_cycles - 1.0) / self.max_tracking_lag_cycles
        )
        # Release once the arm has recovered half of the soft zone above the per-period budget.
        self._joint_lag_release_rad = 0.5 * (self._joint_lag_limit_rad + self._joint_step_budget_rad)
        # Per-joint velocity budget handed to the DLS solver: the model limit scaled by
        # --max-joint-speed-ratio, with no extra shrink (speed scale 1). How far the command may
        # lead the *measured* joints is a separate budget owned by the lag governor above, so it
        # must not be folded into this speed limit. The earlier version of this line additionally
        # clamped it to ``_joint_step_budget_rad / tracking_lag_hold_dwell_s``, i.e. a constant
        # dt/dwell = 1/15 haircut, which made J1/J2 crawl at 1.2 deg/s instead of 18 deg/s.
        self._dls_joint_velocity_limit_rad_s = self.model_spec.max_joint_velocity_rad_s * self.max_joint_speed_ratio
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
        if self.max_offset_m is not None and self.self_test_distance_m > self.max_offset_m:
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
        # Continuous seed for velocity/acceleration shaping. Kept even across a lag hold,
        # so the next accepted step is one small step from where the command stream already is.
        self._dls_seed_joint_rad: np.ndarray | None = None
        self._dls_velocity = np.zeros(6)
        self._dls_stopping = False
        self._dls_consecutive_failures = 0
        # Tracking-lag governor state: when the command first exceeded its allowance.
        self._lag_exceeded_since_t: float | None = None
        self._lag_holding = False
        self._lag_hold_reason: str | None = None
        self._command_stalled_cycles = 0
        self.grip_active = False
        self.ref_arm_xyz: np.ndarray | None = None
        self.ref_arm_quat: np.ndarray | None = None
        self.ref_ctrl_xyz: np.ndarray | None = None
        self.ref_ctrl_quat: np.ndarray | None = None
        self.home_xyz: np.ndarray | None = None
        self.home_quat: np.ndarray | None = None
        self._resetting = False
        self._startup_homing = False
        self._reset_start_xyz: np.ndarray | None = None
        self._reset_start_quat: np.ndarray | None = None
        self._reset_start_joint_rad: np.ndarray | None = None
        self._reset_waiting_for_stop = False
        self._reset_duration_actual_s = self.reset_duration_s
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
        self._rearm_seen_pressed = False
        self._last_singularity_warning_t = -math.inf
        self._last_debug_summary_t = 0.0
        self._last_motion_preview_t = -math.inf
        # Preview-only snapshots for --log-motion-debug. Nothing in the control path reads them.
        self._debug_xr_delta_m: np.ndarray | None = None
        self._debug_xr_delta_rot_rad: np.ndarray | None = None
        self._dls_debug: dict | None = None
        self._last_preview_text: str | None = None
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
            if self.log_motion_debug:
                self._debug_xr_delta_m = np.zeros(3)
                self._debug_xr_delta_rot_rad = np.zeros(3)
            return np.zeros(3, dtype=float), np.zeros(3, dtype=float)

        raw_xyz = controller_xyz - self.ref_ctrl_xyz
        raw_rot = quat_diff_as_angle_axis(self.ref_ctrl_quat, controller_quat)
        if self.log_motion_debug:
            self._debug_xr_delta_m = raw_xyz.copy()
            self._debug_xr_delta_rot_rad = raw_rot.copy()
        delta_xyz = raw_xyz * self.scale_factor
        delta_rot = raw_rot * self.rotation_scale_factor
        translation_norm = float(np.linalg.norm(delta_xyz))
        if self._diagnostic_row is not None:
            self._diagnostic_row["delta_pos_norm"] = translation_norm
            self._diagnostic_row["delta_rot_norm"] = float(np.linalg.norm(delta_rot))
        if self.max_offset_m is not None and translation_norm > self.max_offset_m:
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

    def _enter_rearm(self, *, require_fresh_release: bool = True) -> None:
        self._last_accepted_joint_rad = None
        seen_pressed = self.grip_active or not require_fresh_release
        if self._rearm_required:
            seen_pressed = self._rearm_seen_pressed or seen_pressed
        self._rearm_required = True
        self._rearm_seen_pressed = seen_pressed

    def _latch_safety_fault(self, reason: str) -> None:
        self._enter_rearm()
        self.grip_active = False
        self._resetting = False
        self._startup_homing = False
        self._reset_waiting_for_stop = False
        self._clear_anchor()
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
            return f"RM65 上报错误码：{feedback.arm_error_codes}"
        valid, reason = self.model_spec.validate_joint_feedback(feedback.joint_rad, np.deg2rad(2.0))
        if not valid:
            return reason
        if feedback.joint_speed_rad_s is None:
            return "RM65 UDP 反馈缺少 joint_speed 字段"
        if check_overspeed:
            limits = self.model_spec.max_joint_velocity_rad_s * self.max_joint_speed_ratio
            violations = np.flatnonzero(np.abs(feedback.joint_speed_rad_s) > limits)
            if violations.size:
                index = int(violations[0])
                actual = math.degrees(float(abs(feedback.joint_speed_rad_s[index])))
                limit = math.degrees(float(limits[index]))
                return f"J{index + 1} 关节超速：{actual:.2f}°/s > {limit:.2f}°/s"
        if check_tracking:
            commanded_pose6 = self.arm.get_last_commanded_pose6()
            if commanded_pose6 is not None:
                commanded_xyz, commanded_quat = self.arm.pose6_to_xyz_quat(commanded_pose6)
                position_error = float(np.linalg.norm(commanded_xyz - feedback.tcp_xyz_m))
                rotation_error = float(np.linalg.norm(quat_diff_as_angle_axis(feedback.tcp_quat_wxyz, commanded_quat)))
                if position_error > self.max_tracking_error_m or rotation_error > self.max_tracking_error_rad:
                    return "RM65 跟踪误差超限：" f"位置={position_error:.3f}m，姿态={rotation_error:.3f}rad"
        warnings = self.model_spec.singularity_warnings(feedback.joint_rad, np.deg2rad(10.0))
        if warnings and now - self._last_singularity_warning_t >= 5.0:
            self._last_singularity_warning_t = now
            print(f"[SAFETY WARN] 逼近奇异：{', '.join(warnings)}")
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
                self._latch_safety_fault("读不到 RM65 UDP 反馈")
            return None
        age = now - feedback.received_monotonic_s
        if age < -0.01:
            self._latch_safety_fault("RM65 UDP 反馈时间戳来自未来")
            return None
        if require_fresh and age > self.udp_feedback_timeout_s:
            self._latch_safety_fault(f"RM65 UDP 反馈已过期：{age:.3f}s")
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
            raise RuntimeError(f"读取 {command} 失败；拒绝使用默认 FK 参数")

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
            "[INFO] DLS 已载入控制器 DH；"
            f"d6={dh['joint_6'][2] / 1e6:.6f}m；"
            f"工具帧={tool.get('tool_name') if tool is not None else '显式变换'}；"
            "work/base 变换保持原样；FK/UDP 一致性校验收仍然开启"
        )

    def _startup(self) -> None:
        self.feedback_receiver.start()
        self.arm.connect()
        info = self.arm.get_arm_software_info()
        if info is None:
            raise RuntimeError("读取 RM65 软件版本信息失败")
        product = info.get("Product_version", info.get("product_version"))
        if not isinstance(product, str) or not product.casefold().startswith(self.expected_product_prefix.casefold()):
            raise RuntimeError(f"机型应为 {self.expected_product_prefix}，实际为 {product!r}")
        controller_generation = str(info.get("robot_controller_version", "")).strip()
        if controller_generation.startswith("4"):
            raise RuntimeError("本控制器按第三代 joint_speed 单位解析，但检测到第四代控制器")
        print(f"[INFO] 运动指令={self.motion_command} dry_run={self.dry_run}")
        print("[INFO] RM65 预检 " f"product={product}, ctrl={info.get('ctrl_info')}, plan={info.get('plan_info')}")
        self._configure_dls_from_controller()
        deadline = self._monotonic() + max(1.0, 5.0 * self.udp_feedback_timeout_s)
        feedback = self.feedback_receiver.latest()
        while feedback is None and self._monotonic() < deadline:
            self._sleep(0.01)
            feedback = self.feedback_receiver.latest()
        if feedback is None:
            raise RuntimeError("启动期间没有收到有效的 RM65 UDP 反馈")
        now = self._monotonic()
        if now - feedback.received_monotonic_s > self.udp_feedback_timeout_s:
            raise RuntimeError("启动时的 RM65 UDP 反馈已过期")
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
            # Report the pose the arm is actually parked in: the fixed joint home above is only
            # a target, and a singular measured pose holds every teleop cycle with no other clue.
            ratio, angles = self.dls.metrics(feedback.joint_rad)
            severity = self.dls.severity(ratio, angles)
            zone = "硬停区" if severity >= 1 else ("减速区" if severity > 0 else "正常")
            print(
                f"[INFO] 实测 UDP 位形奇异比={ratio:.5f} 严重度={severity:.2f} 区域={zone} "
                f"|J3|={math.degrees(float(angles[0])):.2f}° |J5|={math.degrees(float(angles[1])):.2f}°"
            )
            if severity >= 1:
                print(
                    "[WARN] 实测位形处于 DLS 硬停区：每个遥操作周期都会保持不动。"
                    "请先用示教器或拖动把机械臂挪出该位形，再重新握 Grip。"
                )
            elif severity > 0:
                print("[WARN] 实测位形处于 DLS 减速区：运动速度会被按比例降低。")
        if self.home_joint_rad is None:
            self.home_xyz = feedback.tcp_xyz_m.copy()
            self.home_quat = feedback.tcp_quat_wxyz.copy()
            print(f"[INFO] 已从 UDP 记录启动 TCP 位姿 xyz_m={np.round(self.home_xyz, 4).tolist()}")
        else:
            self.home_xyz, self.home_quat = self.dls.kinematics.forward(self.home_joint_rad)
            valid, reason = self.arm._pose_is_within_bounds(self.home_xyz)
            if not valid:
                raise RuntimeError(f"固定六轴 Home 的 TCP 超出工作空间：{reason}")
            self.dls.begin_cycle()
            ratio, angles = self.dls.metrics(self.home_joint_rad)
            if self.dls.severity(ratio, angles) > 0.0:
                raise RuntimeError(f"固定六轴 Home 落在奇异减速区：奇异比={ratio:.5f}")
            print(
                f"[INFO] 固定六轴 Home deg={np.round(np.rad2deg(self.home_joint_rad), 3).tolist()} "
                f"奇异比={ratio:.5f} TCP_xyz_m={np.round(self.home_xyz, 4).tolist()}"
            )
        self.arm.sync_command_reference(self.arm.xyz_quat_to_pose6(feedback.tcp_xyz_m, feedback.tcp_quat_wxyz))
        self.scheduler = FixedRateScheduler(self.control_rate_hz, now)
        self._started = True

    def _clear_anchor(self) -> None:
        self._dls_velocity = np.zeros(6)
        self._dls_stopping = False
        self._dls_consecutive_failures = 0
        self._last_accepted_joint_rad = None
        self._dls_seed_joint_rad = None
        self._lag_exceeded_since_t = None
        self._lag_holding = False
        self._lag_hold_reason = None
        self._command_stalled_cycles = 0
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
        # A new segment must show its first preview even if the text matches the old segment.
        self._last_preview_text = None

    def _reset_anchor(self, feedback: ArmFeedback) -> None:
        self._dls_velocity = np.zeros(6)
        self._dls_consecutive_failures = 0
        # The next accepted command starts a new segment from fresh measured joints.
        self._last_accepted_joint_rad = None
        self._dls_seed_joint_rad = None
        self._lag_exceeded_since_t = None
        self._lag_holding = False
        self._command_stalled_cycles = 0
        self.ref_arm_xyz = feedback.tcp_xyz_m.copy()
        self.ref_arm_quat = feedback.tcp_quat_wxyz.copy()
        self.ref_ctrl_xyz = None
        self.ref_ctrl_quat = None
        self._last_valid_xr_pose = None
        self._last_valid_xr_t = None
        self.limiter.reset(feedback.tcp_xyz_m, feedback.tcp_quat_wxyz)
        self.limiter.set_speed_scale(1.0)
        self.arm.sync_command_reference(self.arm.xyz_quat_to_pose6(feedback.tcp_xyz_m, feedback.tcp_quat_wxyz))
        print(f"[INFO] 已按 UDP 实测重建锚点 xyz_m={np.round(self.ref_arm_xyz, 4).tolist()}")

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
            self._latch_safety_fault(f"XR 位姿非法：{exc}")
            return False
        if translation_step > self.max_xr_step_m or rotation_step > self.max_xr_rot_step_rad:
            self._latch_safety_fault(
                "XR 位姿单帧跳变超限：" f"平移={translation_step:.3f}m，旋转={rotation_step:.3f}rad"
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
            self._latch_safety_fault("睿尔曼官方 IK 求解器未初始化")
            return None
        try:
            measured = np.array(measured_joint_rad, dtype=float, copy=True)
            if measured.shape != (6,) or not np.all(np.isfinite(measured)):
                raise ValueError("实测关节必须是 6 个有限值")
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
            self._latch_safety_fault(f"官方 RM65 IK 求解失败：{exc}")
            return None
        if self._diagnostic_row is not None:
            self._diagnostic_row["joint_candidate_rad"] = target_joint_rad.tolist()
        if target_joint_rad.shape != (6,) or not np.all(np.isfinite(target_joint_rad)):
            self._latch_safety_fault("官方 RM65 IK 返回了非法的关节目标")
            return None
        valid, reason = self.model_spec.validate_joint_feedback(target_joint_rad, 0.0)
        if not valid:
            self._latch_safety_fault(f"官方 RM65 IK 目标被拒：{reason}")
            return None
        max_step_rad = self.model_spec.max_joint_velocity_rad_s * self.max_joint_speed_ratio * self.dt
        # Keep the original conservative measured-state budget. This is a position
        # discrepancy bound, not a measured velocity or a model of feedback latency.
        # Also bound adjacent commands: two safe targets around a noisy measurement
        # can otherwise be too far apart. Never wrap angles across mechanical limits.
        for reference, label in ((seed, "上一命令连续性"), (measured, "实测反馈")):
            step_rad = np.abs(target_joint_rad - reference)
            violations = np.flatnonzero(step_rad > max_step_rad + 1e-9)
            if violations.size:
                index = int(violations[0])
                self._latch_safety_fault(
                    f"官方 RM65 IK J{index + 1} 单步超限（对比{label}）："
                    f"{math.degrees(float(step_rad[index])):.3f}° > "
                    f"{math.degrees(float(max_step_rad[index])):.3f}°"
                )
                return None
        return target_joint_rad

    def _dls_frame_matches(self, feedback):
        xyz, quat = self.dls.forward(feedback.joint_rad)
        error_m = float(np.linalg.norm(xyz - feedback.tcp_xyz_m))
        error_rad = float(np.linalg.norm(shortest_world_rotation_vector(quat, feedback.tcp_quat_wxyz)))
        if error_m > 0.005 or error_rad > math.radians(2.0):
            raise ValueError(
                f"官方 FK 与 UDP 反馈的坐标系不一致：位置差 {error_m:.4f}m，"
                f"姿态差 {math.degrees(error_rad):.2f}°；请核对 work/base 与法兰/TCP 变换"
            )

    def _govern_tracking_lag(self, measured_joint_rad, now: float):
        """Bound how far the command may lead the measured joints; decelerate, then hold.

        Returns ``(speed_scale, reason, holding)``. ``speed_scale`` in ``[0, 1]`` is handed to
        the DLS step so the remaining velocity budget shrinks as the lead approaches its
        allowance; ``reason`` names the axis and both numbers while the allowance is spent.
        One reference is used by the governor, the bound handed to the solver and the
        bounded-path guard, so an accepted target can never exceed the allowance.

        The gate has hysteresis: the hold engages at the full allowance and releases only
        after the arm has recovered half of the soft zone, so a command sitting exactly on
        the boundary cannot chatter between "hold" and "tiny step".

        The hand-off is the *command stream* (``_dls_seed_joint_rad``), not the last accepted
        target: a lag hold deliberately keeps the command stream alive, so the lead has to be
        measured from it for the gap to shrink while the arm catches up. Both are the same
        value whenever the controller is not holding.
        """
        reference = self._dls_seed_joint_rad
        if reference is None:
            reference = np.asarray(measured_joint_rad, dtype=float)
        gap = np.abs(np.asarray(reference, dtype=float) - np.asarray(measured_joint_rad, dtype=float))
        index = int(np.argmax(gap))
        allowed = float(self._joint_lag_limit_rad[index])
        trigger = float(self._joint_lag_hold_trigger_rad[index])
        release = float(self._joint_lag_release_rad[index])
        actual = float(gap[index])
        if self._lag_holding:
            self._lag_holding = actual > release
        else:
            self._lag_holding = actual >= trigger
        if self._lag_holding:
            if self._lag_exceeded_since_t is None:
                self._lag_exceeded_since_t = now
        else:
            self._lag_exceeded_since_t = None
        if self._lag_holding:
            reason = (
                f"J{index + 1} 命令超前实测 {math.degrees(actual):.3f}° ≥ 跟随保持线 "
                f"{math.degrees(trigger):.3f}°（跟随余量 {math.degrees(allowed):.3f}°，"
                f"即 {self.max_tracking_lag_cycles:g} 个周期额度）"
            )
            return 0.0, reason, True
        # The command stream is free to advance again: forget the previous hold.
        self._lag_hold_reason = None
        if actual <= self._joint_step_budget_rad[index]:
            return 1.0, None, False
        # Slow-down zone: earn the remaining allowance in proportion to how much is left.
        remaining = 1.0 - (actual - self._joint_step_budget_rad[index]) / (
            self._joint_lag_limit_rad[index] - self._joint_step_budget_rad[index]
        )
        scale = min(1.0, max(0.0, float(remaining)))
        return scale, None, False

    def _hold_damped_motion(self, feedback, *, lag_hold=False):
        """Soft stop; keep the pending trajectory from advancing.

        ``lag_hold=True`` is the tracking-allowance case. The command stream is frozen
        rather than discarded: the arm keeps executing the command it already has, the gap
        therefore only shrinks, and the next accepted step stays one small step away from
        where the command stream already is. Only an explicit segment end (Grip release,
        fault, singularity hold) drops the old command reference.
        """
        # Stop the controller's already pending trajectory, not just the next Python target.
        if not self._dls_stopping and not self.dry_run:
            if not self.arm.slow_stop():
                raise RuntimeError("DLS 保持时的软停止未被控制器确认")
        self._dls_stopping = True
        self._singularity_hold = True
        self._dls_velocity = np.zeros(6)
        if not lag_hold:
            self._last_accepted_joint_rad = None
            self._dls_seed_joint_rad = None
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
        if self.log_motion_debug:
            self._dls_debug = {
                "status": "hold_budget",
                "detail": exc.reason,
                "ratio": None,
                "severity": None,
                "scale": 0.0,
                "damping": None,
                "hold": True,
                "seed_rad": None,
                "measured_rad": None,
                "candidate_rad": None,
                "fk_calls": exc.fk_calls,
                "target_rad": None,
                "achieved_xyz": None,
            }
        now = self._monotonic()
        if now - self._last_singularity_report_t >= self.singularity_report_interval_s:
            print(
                "[DLS] 预算不足保持 "
                f"连续失败={self._dls_consecutive_failures}/{self.dls_max_consecutive_failures} "
                f"fk={exc.fk_calls} 耗时={exc.elapsed_s * 1000:.2f}ms 原因={exc.reason}"
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
            self._dls_frame_matches(feedback)
            # A new Grip/reset segment must not seed from a still-stopping arm.
            if self._dls_stopping or self._last_accepted_joint_rad is None:
                if np.any(np.abs(feedback.joint_speed_rad_s) > self.rearm_stationary_speed_rad_s):
                    return self._hold_damped_motion(feedback)
            # Decide the tracking-allowance governor before opening the DLS budget window:
            # it is a scalar comparison over seven numbers, and running it inside the window
            # would charge already-spent wall-clock time against this cycle's FK budget.
            started_now = self._monotonic()
            lag_scale, lag_reason, lag_holding = self._govern_tracking_lag(feedback.joint_rad, started_now)
            self.dls.begin_cycle(
                deadline=started + self.dls.compute_budget_s,
                max_fk_calls=self.dls.max_fk_calls,
                enforce_budget=True,
            )
            seed = (
                feedback.joint_rad if self._dls_seed_joint_rad is None else self._dls_seed_joint_rad
            ).copy()
            origin, orientation = self.dls.forward(seed)
            valid, reason = self.arm._pose_is_within_bounds(xyz)
            if not valid:
                raise ValueError(reason)
            if lag_holding:
                # The command has used up the whole measured allowance. Stop the pending
                # trajectory and wait for the arm instead of latching a fault: this is the
                # ordinary servo-lag case, and it recovers by itself once the arm catches up.
                if (
                    self._lag_exceeded_since_t is not None
                    and started_now - self._lag_exceeded_since_t > self.tracking_lag_hold_dwell_s
                ):
                    raise ValueError(f"跟随误差持续恶化（超过 {self.tracking_lag_hold_dwell_s:.2f}s）：{lag_reason}")
                self._lag_hold_reason = lag_reason
                now = self._monotonic()
                if now - self._last_singularity_report_t >= self.singularity_report_interval_s:
                    print(f"[DLS] hold_tracking_lag 原因={lag_reason}")
                    self._last_singularity_report_t = now
                if self._diagnostic_row is not None:
                    self._diagnostic_row.update(
                        dls_status="hold_tracking_lag",
                        singularity_hold=True,
                        lag_speed_scale=0.0,
                        lag_hold_reason=lag_reason,
                        joint_target_accepted=False,
                    )
                self._hold_damped_motion(feedback, lag_hold=True)
                return False
            result = self.dls.step(
                xyz,
                quat,
                seed,
                self.dt,
                self._dls_joint_velocity_limit_rad_s,
                previous_velocity=self._dls_velocity,
                max_linear_speed=self.max_linear_velocity_m_s,
                max_angular_speed=self.max_angular_velocity_rad_s,
                lag_speed_scale=lag_scale,
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
                    lag_speed_scale=lag_scale,
                )
            if self.log_motion_debug:
                self._dls_debug = {
                    "status": result.status,
                    "detail": result.detail,
                    "ratio": result.ratio,
                    "severity": severity,
                    "scale": result.scale,
                    "lag_scale": lag_scale,
                    "damping": result.damping,
                    "hold": result.hold,
                    "seed_rad": np.asarray(seed, dtype=float).copy(),
                    "measured_rad": np.asarray(feedback.joint_rad, dtype=float).copy(),
                    "candidate_rad": np.asarray(result.joint_rad, dtype=float).copy(),
                    "fk_calls": self.dls.fk_calls,
                    "target_rad": None,
                    "achieved_xyz": None,
                }
            now = self._monotonic()
            if now - self._last_singularity_report_t >= self.singularity_report_interval_s:
                # Only report a slow-down or a hold; normal tracking stays one line per change.
                if result.hold or result.scale < 1:
                    print(
                        f"[DLS] {result.status} 奇异比={result.ratio:.5f} 速度缩放={result.scale:.3f} "
                        f"严重度={severity:.2f}" + (f" 原因={result.detail}" if result.detail else "")
                    )
                    self._last_singularity_report_t = now
            if result.hold:
                self._lag_holding = False
                return self._hold_damped_motion(feedback)
            # Match the actual wire resolution before performing final safety checks.
            target = np.deg2rad(np.round(np.rad2deg(result.joint_rad) * 1000) / 1000)
            blocked = "步长低于 0.001° 下发分辨率" if np.max(np.abs(target - seed)) < 1e-10 else None
            if blocked is None and not self.dls.bounded_path_allowed(
                seed,
                target,
                ratio=result.ratio,
                angles=np.abs(seed[[2, 4]]),
                lag_limit_rad=self._joint_lag_limit_rad,
            ):
                blocked = "奇异路径守卫"
            if blocked is None:
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
                    blocked = "命令速度超出笛卡尔限值"
            if blocked is not None:
                if blocked == "奇异路径守卫":
                    return self._hold_damped_motion(feedback)
                # The command stream is frozen even though the operator is asking for motion.
                # While the lag governor is active it means the arm is not closing the lead at
                # all; the slow-down zone can hide that behind an ever-smaller step.
                self._command_stalled_cycles = self._command_stalled_cycles + 1 if lag_holding else 0
                if self._command_stalled_cycles >= self.max_command_stall_cycles:
                    raise ValueError(
                        f"命令流停滞后手臂仍未跟上（连续 {self._command_stalled_cycles} 个周期）："
                        f"{self._lag_hold_reason or '命令步长低于下发分辨率'}"
                    )
                return self._hold_damped_motion(feedback)
            self._command_stalled_cycles = 0
            # Refresh after potentially expensive FK calls; do not send a late or stale target.
            fresh = self._checked_feedback(self._monotonic(), require_fresh=True, check_tracking=True)
            if fresh is None:
                return False
            self._dls_frame_matches(fresh)
            # Criterion 1 - command continuity: one sent command may differ from the previous
            # one by at most one control period of joint velocity. This is the real speed gate.
            for reference, label in ((seed, "上一命令连续性"), (fresh.joint_rad, "实测反馈")):
                step_rad = np.abs(target - reference)
                limit = self._joint_step_budget_rad if label == "上一命令连续性" else self._joint_lag_limit_rad
                violations = np.flatnonzero(step_rad > limit + 1e-9)
                if violations.size:
                    index = int(violations[0])
                    # The target may never lead the measured joints by more than the lag
                    # allowance, so a stationary arm cannot support an unbounded dry-run replay.
                    hint = (
                        "（dry run 下真机不动，松开再握 Grip 可重新对齐）"
                        if self.dry_run and label == "实测反馈"
                        else ""
                    )
                    raise ValueError(
                        f"DLS J{index + 1} 单步超限（对比{label}）："
                        f"{math.degrees(float(step_rad[index])):.3f}° > "
                        f"{math.degrees(float(limit[index])):.3f}°{hint}"
                    )
            if not self.dls.bounded_path_allowed(
                fresh.joint_rad,
                target,
                ratio=result.ratio,
                angles=np.abs(fresh.joint_rad[[2, 4]]),
                lag_limit_rad=self._joint_lag_limit_rad,
            ):
                return self._hold_damped_motion(fresh)
            self.dls.check_budget("最终准入")
            if not self.dry_run:
                if not self.arm.send_movej_follow_joint_rad(target.copy(), active=True) or self.arm.fault_latched:
                    raise RuntimeError("DLS 关节目标下发失败")
            self.arm.sync_command_reference(self.arm.xyz_quat_to_pose6(achieved_xyz, achieved_quat))
            limiter = self._self_test_limiter if self.self_test_ramp else self.limiter
            limiter.rebase(achieved_xyz, achieved_quat, linear, angular)
            self._dls_velocity = (target - seed) / self.dt
            self._dls_seed_joint_rad = target.copy()
            self._last_accepted_joint_rad = target.copy()
            self._dls_consecutive_failures = 0
            self._singularity_hold = False
            self._lag_hold_reason = None
            self._dls_stopping = False
            if self._dls_debug is not None:
                self._dls_debug.update(
                    target_rad=np.asarray(target, dtype=float).copy(), achieved_xyz=achieved_xyz.copy()
                )
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
            self._latch_safety_fault(f"DLS 拒绝本次运动：{exc}")
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
                    self._enter_rearm()
                    return False
                if self._rearm_required:
                    return False
                if measured_joint_rad is None:
                    self._latch_safety_fault("官方 RM65 IK 需要当前的 UDP 关节反馈")
                    return False
                target_joint_rad = self._solve_joint_target(xyz, quat, measured_joint_rad)
                if target_joint_rad is None:
                    return False
                if not self.dry_run:
                    try:
                        sent = self.arm.send_movej_follow_joint_rad(target_joint_rad.copy(), active=True)
                        if not sent or self.arm.fault_latched:
                            self._latch_safety_fault(self.arm.fault_reason or "官方 RM65 关节目标下发失败")
                            return False
                        self.arm.sync_command_reference(self.arm.xyz_quat_to_pose6(xyz, quat))
                    except Exception as exc:
                        self._latch_safety_fault(f"官方 RM65 关节目标下发失败：{exc}")
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

    def _plan_fixed_joint_reset(self, start_joint_rad: np.ndarray) -> float:
        """Estimate the trip time to a fixed joint home; optionally reject unsafe paths.

        With ``enable_home_path_preflight`` the sampled singular/workspace verdicts are
        hard rejections. With the preflight disabled they degrade to console warnings
        (the caller then sends the joint-space interpolation directly), so the arm can
        still be homed from the far side of a J3/J5 zero without a pendant.
        """
        if self.dls is None or self.home_joint_rad is None:
            raise RuntimeError("固定六轴回位需要 DLS 模式")
        delta = self.home_joint_rad - start_joint_rad
        # Any continuous joint-space path to this home sweeps J3 and J5 through the
        # +/-5deg hard zone from the far side. Report the offending axis with both
        # endpoints: the operator has to jog the arm to the home side by hand (pendant
        # or drag) when the preflight is enabled, because no guarded path exists here.
        crossings = []
        for name, index in (("J3", 2), ("J5", 4)):
            start_rad = float(start_joint_rad[index])
            home_rad = float(self.home_joint_rad[index])
            if start_rad * home_rad < 0.0:
                crossings.append(f"{name} 起点={math.degrees(start_rad):+.3f}° Home={math.degrees(home_rad):+.3f}°")
        if crossings and self.enable_home_path_preflight:
            raise ValueError(
                "关节回位路径穿越 J3/J5 零位（"
                + "；".join(crossings)
                + "）；请先用示教器或拖动把该轴挪到 Home 一侧，再重试回位"
            )
        sample_count = max(1, int(math.ceil(np.max(np.abs(np.rad2deg(delta))) / 5.0)))
        linear_length = 0.0
        angular_length = 0.0
        previous_pose = None
        if not self.enable_home_path_preflight:
            # One diagnostic sweep only: never blocks, never latches, and any failure
            # here still leaves the reset itself untouched.
            self.dls.begin_cycle()
        hard_sample = None
        outside_sample = None
        for index in range(sample_count + 1):
            joint = start_joint_rad + delta * (index / sample_count)
            if self.enable_home_path_preflight:
                self.dls.begin_cycle()
                ratio, angles = self.dls.metrics(joint)
                if ratio <= self.dls.stop_ratio or np.any(angles <= self.dls.stop_joint_rad):
                    raise ValueError(f"关节回位路径在第 {index}/{sample_count} 个采样点进入奇异硬区")
            elif hard_sample is None:
                try:
                    ratio, angles = self.dls.metrics(joint)
                except Exception:
                    ratio = None
                if ratio is not None and (ratio <= self.dls.stop_ratio or np.any(angles <= self.dls.stop_joint_rad)):
                    hard_sample = (index, sample_count, ratio)
            pose = self.dls.kinematics.forward(joint)
            valid, reason = self.arm._pose_is_within_bounds(pose[0])
            if not valid:
                if self.enable_home_path_preflight:
                    raise ValueError(f"关节回位路径在第 {index}/{sample_count} 个采样点越出 TCP 工作空间：{reason}")
                if outside_sample is None:
                    outside_sample = (index, sample_count, reason)
            if previous_pose is not None:
                linear_length += float(np.linalg.norm(pose[0] - previous_pose[0]))
                angular_length += float(np.linalg.norm(shortest_world_rotation_vector(previous_pose[1], pose[1])))
            previous_pose = pose
        if not self.enable_home_path_preflight:
            print(
                "[HOME] 回位路径预检已关闭（--home-path-preflight 可重新开启）："
                "不拒绝 J3/J5 零位穿越、奇异硬区与 TCP 工作空间越界；"
                "回位改为关节空间插值直发，逐周期的 UDP 新鲜度/跟踪误差/关节限位/超速检查仍然生效"
            )
            if crossings:
                print("[HOME] 注意：本次回位将穿越 J3/J5 零位（" + "；".join(crossings) + "）")
            if hard_sample is not None:
                index, total, ratio = hard_sample
                print(f"[HOME] 注意：回位路径第 {index}/{total} 个采样点进入奇异硬区（奇异比={ratio:.5f}）")
            if outside_sample is not None:
                index, total, reason = outside_sample
                print(f"[HOME] 注意：回位路径第 {index}/{total} 个采样点越出 TCP 工作空间：{reason}")
        joint_speed = self.model_spec.max_joint_velocity_rad_s * self.max_joint_speed_ratio
        minimum_motion_s = max(
            float(np.max(np.abs(delta) / joint_speed)),
            linear_length / self.max_linear_velocity_m_s,
            angular_length / self.max_angular_velocity_rad_s,
        )
        return max(self.reset_duration_s, 2.0 * minimum_motion_s)

    def _send_home_joint_waypoint(
        self,
        waypoint_rad: np.ndarray,
        waypoint_xyz: np.ndarray,
        waypoint_quat: np.ndarray,
    ) -> bool:
        """Send one fixed-Home joint waypoint straight to ``movej_follow``.

        Only used while the reset preflight is disabled: the DLS singularity guard
        rejects any step across a J3/J5 zero, so a reset that starts on the far side of
        one would hold forever and then latch a timeout. The per-cycle protections that
        do not depend on the DLS solver still run in the teleop loop: UDP freshness,
        tracking error, joint limits/overspeed and controller error codes. Joint speed
        is bounded because the reset duration is derived from the joint-speed limit.
        """
        if self.arm.fault_latched:
            return False
        valid, reason = self.model_spec.validate_joint_feedback(np.asarray(waypoint_rad, dtype=float), 0.0)
        if not valid:
            self._latch_safety_fault(f"固定六轴 Home 回位插值越界：{reason}")
            return False
        started = self._monotonic()
        sent = True
        if not self.dry_run:
            sent = bool(self.arm.send_movej_follow_joint_rad(waypoint_rad.copy(), active=True))
            if not sent or self.arm.fault_latched:
                self._latch_safety_fault(self.arm.fault_reason or "固定六轴 Home 关节目标下发失败")
                return False
        # The tracking check compares measured TCP against this reference; a joint
        # command does not update it by itself.
        self.arm.sync_command_reference(self.arm.xyz_quat_to_pose6(waypoint_xyz, waypoint_quat))
        self._last_accepted_joint_rad = np.asarray(waypoint_rad, dtype=float).copy()
        self._dls_velocity = np.zeros(6)
        self._dls_stopping = False
        if self._diagnostic_row is not None:
            seed = self._reset_start_joint_rad if self._reset_start_joint_rad is not None else waypoint_rad
            self._diagnostic_row.update(
                joint_target_accepted=True,
                joint_seed_rad=np.asarray(seed, dtype=float).tolist(),
                joint_candidate_rad=np.asarray(waypoint_rad, dtype=float).tolist(),
                dls_status="home_joint_direct",
                dls_achieved_xyz=np.asarray(waypoint_xyz, dtype=float).tolist(),
                dls_achieved_quat_wxyz=np.asarray(waypoint_quat, dtype=float).tolist(),
                send_started_monotonic=started,
                send_duration_s=max(0.0, self._monotonic() - started),
                send_result=sent,
            )
        return True

    def _start_reset(self, feedback: ArmFeedback) -> None:
        if self.home_xyz is None or self.home_quat is None:
            self._latch_safety_fault("回位目标位姿不可用")
            return
        self.grip_active = False
        self._clear_anchor()
        try:
            if not self.dry_run and not self.arm.slow_stop():
                raise RuntimeError("软停止未被控制器确认")
            self._dls_stopping = self.dls is not None
        except Exception as exc:
            self._latch_safety_fault(f"回位前的 RM65 软停止失败：{exc}")
            return
        self._reset_start_xyz = feedback.tcp_xyz_m.copy()
        self._reset_start_quat = feedback.tcp_quat_wxyz.copy()
        self._reset_start_joint_rad = None
        # A B press can arrive while the arm is still moving. Plan only after a
        # fresh UDP frame confirms the slow stop has completed.
        self._reset_waiting_for_stop = self.home_joint_rad is not None
        self._reset_duration_actual_s = self.reset_duration_s
        self.limiter.reset(feedback.tcp_xyz_m, feedback.tcp_quat_wxyz)
        # The same rule as a new teleop segment: a reset commands the arm from where it
        # actually is, so the previous segment's command reference is replaced here too.
        self.arm.sync_command_reference(self.arm.xyz_quat_to_pose6(feedback.tcp_xyz_m, feedback.tcp_quat_wxyz))
        # Return targets go through the guarded _send_motion path unless the fixed-Home
        # preflight is disabled, in which case they are sent as joint-space waypoints.
        self.limiter.set_speed_scale(1.0)
        self._singularity_zone = "clear"
        self._singularity_hold = False
        self._singularity_hold_reported = False
        self._reset_step = 0
        self._reset_started_t = self._monotonic()
        self._resetting = True
        if self._reset_waiting_for_stop:
            print("[RESET] 等待 UDP 反馈确认机械臂已静止，再做固定六轴 Home 回位规划")
        else:
            print(f"[RESET] 正在返回启动 TCP 位姿，最短耗时 {self.reset_duration_s:.2f}s")

    def _tick_reset(self, now: float, feedback: ArmFeedback) -> None:
        if (
            not self._resetting
            or self._reset_start_xyz is None
            or self._reset_start_quat is None
            or self.home_xyz is None
            or self.home_quat is None
        ):
            return
        if self._reset_waiting_for_stop:
            if np.any(np.abs(feedback.joint_speed_rad_s) > self.rearm_stationary_speed_rad_s):
                if now - self._reset_started_t > self.reset_settle_timeout_s:
                    self._latch_safety_fault("固定六轴 Home 回位规划前 RM65 未停下")
                return
            try:
                duration = self._plan_fixed_joint_reset(feedback.joint_rad)
            except Exception as exc:
                self._latch_safety_fault(f"固定六轴 Home 回位规划失败：{exc}")
                return
            self._reset_start_xyz = feedback.tcp_xyz_m.copy()
            self._reset_start_quat = feedback.tcp_quat_wxyz.copy()
            self._reset_start_joint_rad = feedback.joint_rad.copy()
            self._reset_duration_actual_s = duration
            self._reset_steps_total = max(1, int(math.ceil(duration / self.dt)))
            self._reset_step = 0
            self._reset_started_t = now
            self._reset_waiting_for_stop = False
            self.limiter.reset(feedback.tcp_xyz_m, feedback.tcp_quat_wxyz)
            print(f"[RESET] 正在返回固定六轴 Home，最短耗时 {duration:.2f}s")
        alpha = min(1.0, self._reset_step / self._reset_steps_total)
        smooth = alpha * alpha * (3.0 - 2.0 * alpha)
        if self.home_joint_rad is not None:
            if self._reset_start_joint_rad is None:
                self._latch_safety_fault("缺少回位的起始关节角")
                return
            waypoint = self._reset_start_joint_rad + smooth * (self.home_joint_rad - self._reset_start_joint_rad)
            target_xyz, target_quat = self.dls.kinematics.forward(waypoint)
            self._record_target(target_xyz, target_quat)
            if self.enable_home_path_preflight:
                command_xyz, command_quat = self.limiter.step(target_xyz, target_quat, self.dt)
                accepted = self._send_motion(command_xyz, command_quat, feedback.joint_rad)
            else:
                # Joint-space interpolation sent straight to movej_follow: the DLS guard
                # rejects every step across a J3/J5 zero, so the guarded path can never
                # finish a reset that starts on the far side of one.
                accepted = self._send_home_joint_waypoint(waypoint, target_xyz, target_quat)
        else:
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
            accepted = self._send_motion(command_xyz, command_quat, feedback.joint_rad)
        if not accepted:
            self._resetting = False
            self._startup_homing = False
            self._reset_waiting_for_stop = False
            self._enter_rearm()
            return
        self._reset_step += 1
        if alpha < 1.0:
            return
        position_error = float(np.linalg.norm(self.home_xyz - feedback.tcp_xyz_m))
        rotation_error = float(np.linalg.norm(quat_diff_as_angle_axis(feedback.tcp_quat_wxyz, self.home_quat)))
        joint_error = (
            0.0 if self.home_joint_rad is None else float(np.max(np.abs(self.home_joint_rad - feedback.joint_rad)))
        )
        if (
            position_error <= self.reset_position_tolerance_m
            and rotation_error <= self.reset_rotation_tolerance_rad
            and joint_error <= self.home_joint_tolerance_rad
        ):
            self._resetting = False
            self._startup_homing = False
            self._reset_waiting_for_stop = False
            self._reset_start_xyz = None
            self._reset_start_quat = None
            self._reset_start_joint_rad = None
            self._reset_started_t = None
            self._clear_anchor()
            self._enter_rearm(require_fresh_release=False)
            print("[RESET] 回位完成；松开 Grip 再重新握持即可恢复遥操作")
            return
        if (
            self._reset_started_t is not None
            and now - self._reset_started_t > self._reset_duration_actual_s + self.reset_settle_timeout_s
        ):
            self._latch_safety_fault("RM65 回位未在超时前到达 Home")

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

    def _handle_rearm(self, grip: float, now: float) -> None:
        # Releasing Grip is the only operator gesture for re-arm. B is reserved
        # for reset and is ignored while a fault is latched.
        if grip >= self.grip_on_threshold:
            self._rearm_seen_pressed = True
            return
        if grip > self.grip_off_threshold or not self._rearm_seen_pressed:
            return
        if not self._feedback_allows_rearm(now):
            return
        self.arm.clear_fault()
        self._rearm_required = False
        self._rearm_seen_pressed = False
        self._clear_anchor()
        print("[SAFETY] 已解除锁存；请再次握 Grip 开始运动")

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
        if self.log_motion_debug:
            # Drop the previous cycle's snapshot so a stale DLS result is never reported as current.
            self._debug_xr_delta_m = None
            self._debug_xr_delta_rot_rad = None
            self._dls_debug = None
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
            "lag_speed_scale": None,
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
                self._latch_safety_fault("自检未在超时前到达终点")
                self._self_test_done = True
                return
        target = self._self_test_origin + direction * min(self.self_test_distance_m, self._self_test_speed * elapsed)
        command, quat = self._self_test_limiter.step(target, self._self_test_quat, self.dt)
        valid, reason = self.arm._pose_is_within_bounds(command)
        beyond_envelope = self.max_offset_m is not None and (
            np.linalg.norm(command - self._self_test_origin) > self.max_offset_m + 1e-12
        )
        if not valid or beyond_envelope:
            self._latch_safety_fault(reason or "自检命令超出 max_offset_m")
            self._self_test_done = True
            return
        self._record_target(target, self._self_test_quat)
        if not self._send_motion(command, quat, feedback.joint_rad):
            self._enter_rearm()
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
                raise RuntimeError("软停止未被控制器确认")
            self._dls_stopping = self.dls is not None
        except Exception as exc:
            self._latch_safety_fault(f"松开 Grip 时的 RM65 软停止失败：{exc}")

    @staticmethod
    def _preview_list(values, digits: int = 4, scale: float = 1.0) -> list[float]:
        return [float(value) for value in np.round(np.asarray(values, dtype=float) * scale, digits)]

    def _preview_verdict(self, info: dict, accepted: bool) -> str:
        """One short token for what this cycle did; the long explanation stays in the [DLS] line."""
        if self._lag_hold_reason is not None:
            return "保持(跟随余量用尽)"
        if info is not None and info["hold"]:
            status = info["status"]
            if status == "hold_singularity":
                stop_ratio = self.dls.stop_ratio
                reason = "奇异硬停区" if info["ratio"] is not None and info["ratio"] <= stop_ratio else "无可用步"
            else:
                reason = {
                    "hold_at_target": "已到达目标",
                    "hold_joint_margin": "接近限位",
                    "hold_budget": "计算预算不足",
                }.get(status, status.removeprefix("hold_"))
            return f"保持({reason})"
        if accepted:
            return "已接受(未下发)" if self.dry_run else "已接受"
        return "被拒绝"

    def _print_motion_preview(
        self,
        grip: float,
        feedback: ArmFeedback,
        delta_xyz: np.ndarray,
        delta_rot: np.ndarray,
        target_xyz: np.ndarray,
        accepted: bool,
    ) -> None:
        """Two compact lines for the XR -> target -> DLS joint chain, and only when they change."""
        tag = getattr(self, "preview_mode", "DRY-RUN" if self.dry_run else "LIVE")
        head = f"[指令 {tag}] 握持={grip:.2f}"
        if self._current_rotation_gain < 0.999:
            head += f" 旋转增益={self._current_rotation_gain:.2f}"
        if self._debug_xr_delta_m is not None:
            head += f" 手柄位移mm={self._preview_list(self._debug_xr_delta_m, 1, 1000.0)}"
        head += f" 目标位移mm={self._preview_list(delta_xyz, 1, 1000.0)}"
        head += f" 目标旋转°={self._preview_list(np.rad2deg(delta_rot), 1)}"
        head += f" 实测误差mm={float(np.linalg.norm(target_xyz - feedback.tcp_xyz_m)) * 1000.0:.1f}"
        info = self._dls_debug
        if info is None:
            tail = f"  DLS=不适用({self.motion_command})"
        else:
            tail = f"  DLS={info['status']}"
            if info["ratio"] is not None:
                tail += f" 奇异比={info['ratio']:.5f}"
            if info["scale"] < 1.0:
                tail += f" 速度缩放={info['scale']:.2f}"
            if info.get("lag_scale", 1.0) < 1.0:
                tail += f" 跟随缩放={info['lag_scale']:.2f}"
            candidate = info["target_rad"] if info["target_rad"] is not None else info["candidate_rad"]
            if candidate is None:
                tail += " 关节=无"
            else:
                tail += f" 关节°={self._preview_list(np.rad2deg(candidate), 2)}"
                if info["target_rad"] is not None and info["seed_rad"] is not None:
                    step_deg = float(np.max(np.abs(np.rad2deg(info["target_rad"] - info["seed_rad"]))))
                    tail += f" 步长°={step_deg:.3f}"
                measured = info.get("measured_rad")
                if measured is not None:
                    # Direct predictor of the continuity guard: it latches above one step budget.
                    gap_deg = float(np.max(np.abs(np.rad2deg(candidate - measured))))
                    tail += f" 超前°={gap_deg:.3f}"
        tail += f" -> {self._preview_verdict(info, accepted)}"
        if self.arm.fault_latched:
            tail += f" 故障={getattr(self.arm, 'fault_reason', None)}"
        text = head + "\n" + tail
        if text == self._last_preview_text:
            return
        self._last_preview_text = text
        print(text)

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
        accepted = self._send_motion(command_xyz, command_quat, feedback.joint_rad)
        if self.log_motion_debug and now - self._last_motion_preview_t >= 0.2:
            self._last_motion_preview_t = now
            # Printed after the send so the console write cannot delay this cycle's command.
            self._print_motion_preview(grip, feedback, delta_pos, delta_rot, target_xyz, accepted)
        if not accepted:
            self._enter_rearm()

    def _run_teleop_cycle(self, now: float) -> None:
        grip, reset_edge = (0.0, False) if self._startup_homing else self._read_control_state()

        if self.arm.fault_latched and not self._rearm_required:
            self._enter_rearm()

        if self._rearm_required:
            self._handle_rearm(grip, now)
            return

        if self._resetting:
            if not self._startup_homing and not self._xr_sample_is_fresh(now):
                self._latch_safety_fault("回位过程中 XR 追踪样本过期")
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
            self._latch_safety_fault("XR 追踪样本过期或缺失")
            return

        # A new segment has not commanded anything yet, so its first check must not be held
        # against the command reference left over from the previous segment: after a fault,
        # re-arm, pendant jog or drag, the arm is legitimately somewhere else. The anchor
        # below re-syncs that reference to the measured pose before any motion is computed.
        feedback = self._checked_feedback(now, require_fresh=True, check_tracking=was_active)
        if feedback is None:
            return

        xr_pose = self.xr.get_pose_by_name(self.pose_source)
        if self._diagnostic_row is not None and xr_pose is not None:
            self._diagnostic_row["xr_pose"] = np.asarray(xr_pose).tolist()
            self._diagnostic_row["xr_pose_used"] = True
        if xr_pose is None:
            self._latch_safety_fault("读不到 XR 手柄位姿")
            return

        if not was_active:
            self._reset_anchor(feedback)
            return
        if not self._accept_xr_sample(xr_pose, now):
            if not self._rearm_required:
                self._latch_safety_fault("XR 手柄位姿单帧跳变超过安全上限")
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
            f"周期p50={summary['period_p50_ms']:.2f}ms "
            f"周期p95={summary['period_p95_ms']:.2f}ms "
            f"周期max={summary['period_max_ms']:.2f}ms "
            f"发送max={summary['send_max_ms']:.2f}ms "
            f"UDP反馈最大年龄={summary['udp_age_max_ms']:.2f}ms "
            f"错过周期={summary['deadline_misses']}"
        )

    def run(self) -> None:
        last_cycle_t: float | None = None
        last_summary_t = self._monotonic()
        try:
            if self.self_test_ramp and not self.dry_run:
                confirmation = f"RUN {self.self_test_axis} {self.self_test_distance_m:.3f}m"
                answer = input(
                    "[自检] 请先确认控制器坐标系下的运动方向、工作空间已净空、急停就绪。"
                    f"自检期间不接受 XR/Grip 控制，也不会自动回位。请输入 '{confirmation}'："
                )
                if answer.strip() != confirmation:
                    return
            self._startup()
            if self.auto_home_on_start and self.home_joint_rad is not None:
                feedback = self._checked_feedback(self._monotonic(), require_fresh=True, check_tracking=False)
                if feedback is None:
                    return
                joint_error = float(np.max(np.abs(self.home_joint_rad - feedback.joint_rad)))
                if joint_error > self.home_joint_tolerance_rad:
                    if self.dry_run:
                        print("[DRY RUN] 已配置固定六轴 Home；dry run 跳过启动回位")
                    else:
                        phrase = "HOME " + " ".join(format(value, ".10g") for value in np.rad2deg(self.home_joint_rad))
                        answer = input(
                            "[HOME] 请确认整条回程路径净空、急停就绪，且允许机械臂低速移动到 "
                            f"{phrase[5:]}。请输入 '{phrase}'："
                        )
                        if answer.strip() != phrase:
                            print("[HOME] 未收到确认口令；已取消启动回位")
                            return
                        feedback = self._checked_feedback(self._monotonic(), require_fresh=True, check_tracking=False)
                        if feedback is None:
                            return
                        self._start_reset(feedback)
                        if self.arm.fault_latched or not self._resetting:
                            return
                        self._startup_homing = True
                else:
                    print("[HOME] 实测关节已与固定六轴 Home 一致")
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
                    raise RuntimeError("定周期调度器未初始化")
                next_deadline = self.scheduler.advance(self._monotonic())
                delay = next_deadline - self._monotonic()
                if delay > 0.0:
                    self._sleep(delay)
        except KeyboardInterrupt:
            print("\n[INFO] 用户中断遥操作")
        except Exception as exc:
            print(f"\n[ERROR] 遥操作已停止：{exc}")
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


class RealmanRM65SafeTeleopController(RM65ControlCore):
    """Hardware entrypoint for the shared RM65 control core."""
