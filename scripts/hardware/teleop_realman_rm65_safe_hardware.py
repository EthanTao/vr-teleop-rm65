import math

import tyro

from xrobotoolkit_teleop.hardware.realman_rm65_safe_teleop_controller import (
    RealmanRM65SafeTeleopController,
)


def main(
    arm_host: str = "192.168.10.18",
    arm_port: int = 8080,
    pose_source: str = "right",
    scale_factor: float = 1.0,
    rotation_scale_factor: float = 0.25,
    control_rate_hz: float = 50.0,
    move_v: int = 5,
    move_r: int = 80,
    max_offset_m: float | None = None,
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
    home_joint_deg: tuple[float, float, float, float, float, float] = (0, 0, -60, 0, 60, 0),
    auto_home_on_start: bool = True,
    home_joint_tolerance_deg: float = 1.0,
    enable_home_path_preflight: bool = False,
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
):
    """
    Realman RM65 teleoperation using side-mount XR delta mapping.

    This is the recommended hardware entry. It uses independent UDP feedback,
    bounded velocity/acceleration commands, overspeed latching, and explicit re-arm.

    Use --dry-run --log-motion-debug to print, at up to 5 Hz and without sending motion,
    the whole chain: mapped hand delta, scaled/bounded target, limited command, UDP
    measurement, the DLS singularity verdict, and the computed joint target. The
    one-click launcher enables this preview for its dry-run mode.

    Fixed-Home return (B button and --auto-home-on-start) does not require the arm to
    start on the Home side of the J3/J5 zero: with --no-enable-home-path-preflight
    (the default here) the trip is sent as a joint-space interpolation and the sampled
    J3/J5 crossing, singularity and TCP-workspace verdicts become console warnings
    instead of refusals. Pass --enable-home-path-preflight to restore the hard
    preflight, which refuses any reset whose straight joint path crosses a zero.
    Either way the operator still confirms that the whole path is clear before a
    reset starts; the preflight is not collision detection.

    The per-Grip translation envelope is off by default here (--max-offset-m defaults to
    None instead of the controller's 0.15 m): the mapped hand offset is added to the
    anchor pose as-is. What still bounds one segment is the TCP workspace box (1 m per
    axis unless --workspace-min/max-xyz-m narrows it), the XR single-frame jump gate
    (--max-xr-step-m / --max-xr-rot-step-rad), the joint limit/overspeed gates and the
    tracking-lag allowance. The rotation envelope (--max-rot-offset-rad) is unchanged.
    Pass --max-offset-m 0.15 (or 0.03 for a first live run) to restore the clip.

    Two separate budgets decide admission. The adjacent-command step keeps the original
    one-control-period budget (180 deg/s x 10% x 20 ms = 0.360 deg on J1/J2). The lead of
    the command over the UDP-measured joints gets --max-tracking-lag-cycles periods
    (default 3 -> 1.080 deg), because it has to absorb the servo/transport delay measured on
    site (up to 48 ms) and is not a joint speed. Inside that allowance the command slows
    down in proportion to the lead; at the hold line the stream freezes and recovers by
    itself once the arm catches up. Only a lead that keeps degrading for
    --tracking-lag-hold-dwell-s, or a command stream that cannot advance at all while the
    governor is active, latches a fault.

    The DLS per-joint speed limit is the model limit times --max-joint-speed-ratio
    (J1/J2 = 18 deg/s, J3-J6 = 22.5 deg/s) with no further shrink: speed scale 1. The lead
    allowance is the separate budget described above and is not folded into that limit, so
    the Cartesian velocity cap and the governor are what actually bound a fast command.
    """
    fixed_home_enabled = not self_test_ramp and motion_command == "damped_ik_movej_follow"
    if not self_test_ramp and not fixed_home_enabled:
        print("[HOME] fixed joint home is available only in DLS mode; legacy mode keeps startup TCP reset")
    if fixed_home_enabled and not enable_home_path_preflight:
        print(
            "[HOME] 固定六轴 Home 回位路径预检已关闭：起点在 J3/J5 零位另一侧时也能回位，"
            "回位按关节空间插值直发；重新开启请加 --enable-home-path-preflight"
        )
    if max_offset_m is None:
        print(
            "[SAFETY] 单次 Grip 平移包络已取消：手柄位移 1:1 直接进目标，只受 TCP 工作空间盒、"
            "XR 单帧跳变门槛、关节限位/超速与跟随余量限制；重新开启请加 --max-offset-m 0.15"
        )
    controller = RealmanRM65SafeTeleopController(
        arm_host=arm_host,
        arm_port=arm_port,
        pose_source=f"{pose_source}_controller",
        control_trigger=f"{pose_source}_grip",
        scale_factor=scale_factor,
        rotation_scale_factor=rotation_scale_factor,
        control_rate_hz=control_rate_hz,
        move_v=move_v,
        move_r=move_r,
        max_offset_m=max_offset_m,
        max_rot_offset_rad=max_rot_offset_rad,
        motion_deadband_m=motion_deadband_m,
        rot_deadband_rad=rot_deadband_rad,
        grip_on_threshold=grip_on_threshold,
        grip_off_threshold=grip_off_threshold,
        use_headset_world_transform=use_headset_world_transform,
        invert_tcp_xy=invert_tcp_xy,
        suppress_rotation_during_translation=suppress_rotation_during_translation,
        freeze_rotation=freeze_rotation,
        send_stop_on_release=send_stop_on_release,
        log_motion_debug=log_motion_debug,
        reset_button=reset_button,
        reset_duration_s=reset_duration_s,
        home_joint_deg=home_joint_deg if fixed_home_enabled else None,
        auto_home_on_start=auto_home_on_start and fixed_home_enabled,
        home_joint_tolerance_deg=home_joint_tolerance_deg,
        enable_home_path_preflight=enable_home_path_preflight,
        xr_watchdog_timeout_s=xr_watchdog_timeout_s,
        max_tracking_error_m=max_tracking_error_m,
        max_tracking_error_rad=max_tracking_error_rad,
        reset_position_tolerance_m=reset_position_tolerance_m,
        reset_rotation_tolerance_rad=reset_rotation_tolerance_rad,
        reset_settle_timeout_s=reset_settle_timeout_s,
        workspace_min_xyz_m=workspace_min_xyz_m,
        workspace_max_xyz_m=workspace_max_xyz_m,
        motion_command=motion_command,
        official_ik_tool_or_work=official_ik_tool_or_work,
        official_ik_frames_verified=official_ik_frames_verified,
        official_ik_j3_soft_limit_verified=official_ik_j3_soft_limit_verified,
        expected_product_prefix=expected_product_prefix,
        udp_bind_host=udp_bind_host,
        udp_port=udp_port,
        udp_feedback_timeout_s=udp_feedback_timeout_s,
        max_linear_velocity_m_s=max_linear_velocity_m_s,
        max_linear_acceleration_m_s2=max_linear_acceleration_m_s2,
        max_angular_velocity_rad_s=max_angular_velocity_rad_s,
        max_angular_acceleration_rad_s2=max_angular_acceleration_rad_s2,
        max_xr_step_m=max_xr_step_m,
        max_xr_rot_step_rad=max_xr_rot_step_rad,
        rotation_full_gain_linear_speed_m_s=rotation_full_gain_linear_speed_m_s,
        rotation_zero_gain_linear_speed_m_s=rotation_zero_gain_linear_speed_m_s,
        max_joint_speed_ratio=max_joint_speed_ratio,
        max_tracking_lag_cycles=max_tracking_lag_cycles,
        tracking_lag_hold_dwell_s=tracking_lag_hold_dwell_s,
        rearm_stationary_speed_rad_s=rearm_stationary_speed_rad_s,
        enable_singularity_avoidance=enable_singularity_avoidance,
        singularity_slowdown_ratio=singularity_slowdown_ratio,
        singularity_stop_ratio=singularity_stop_ratio,
        singularity_slowdown_joint_deg=singularity_slowdown_joint_deg,
        singularity_stop_joint_deg=singularity_stop_joint_deg,
        singularity_min_speed_scale=singularity_min_speed_scale,
        singularity_escape_speed_scale=singularity_escape_speed_scale,
        singularity_max_damping=singularity_max_damping,
        singularity_report_interval_s=singularity_report_interval_s,
        dls_compute_budget_s=dls_compute_budget_s,
        dls_max_fk_calls=dls_max_fk_calls,
        dls_max_candidate_attempts=dls_max_candidate_attempts,
        dls_max_consecutive_failures=dls_max_consecutive_failures,
        dls_work_from_base=dls_work_from_base,
        dls_flange_to_tcp=dls_flange_to_tcp,
        dry_run=dry_run,
        motion_log_path=motion_log_path,
        self_test_ramp=self_test_ramp,
        self_test_axis=self_test_axis,
        self_test_distance_m=self_test_distance_m,
    )
    controller.run()


if __name__ == "__main__":
    tyro.cli(main)
