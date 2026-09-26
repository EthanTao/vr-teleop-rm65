"""RM65 simulation teleoperation with selectable IK + Meshcat visualization.

Supports configurable motion filter (deadband + EMA), joint constraints
(limit clipping + velocity limiting), smooth reset (B button),
self-collision avoidance, and runtime keyboard tuning.

Mapping convention (code-wins, 2026-09-09 定案): 机器人按 URDF 竖立显示；
手柄映射矩阵保持会话前状态 R_HEADSET_TO_WORLD_RM65_SIDE_MOUNT_HARDWARE，
方向是否直觉以逐轴实测为准（判定表见 docs/RM65参数与坐标系.md §14）。
"""

import os

import numpy as np
import tyro

from xrobotoolkit_teleop.simulation.placo_teleop_controller import PlacoTeleopController
from xrobotoolkit_teleop.utils.geometry import R_HEADSET_TO_WORLD_RM65_SIDE_MOUNT_HARDWARE
from xrobotoolkit_teleop.utils.path_utils import ASSET_PATH


def main(
    # --- 原有关参数 ---
    robot_urdf_path: str = os.path.join(ASSET_PATH, "realman/RM65-official/urdf/RM65-official-arm.urdf"),
    scale_factor: float = 1.5,
    follow_orientation: bool = True,
    enable_log_data: bool = False,
    log_dir: str = "logs",
    home_joint_deg: tuple[float, float, float, float, float, float] | None = (0, 0, -60, 0, 60, 0),
    ik_backend: str = "placo",
    shared_calibration_path: str = "config/rm65_dls_calibration.json",
    shared_scale_factor: float = 1.0,
    shared_rotation_scale_factor: float = 0.25,
    official_ik_tool_or_work: int = 1,
    official_ik_j3_exclusion_deg: float = 5.0,
    official_ik_max_joint_speed_ratio: float = 0.10,
    # --- MotionFilter ---
    deadband_m: float = 0.002,
    deadband_rad: float = 0.03,
    smooth_alpha_pos: float = 0.35,
    smooth_alpha_rot: float = 0.3,
    # --- JointConstraints ---
    # 关节速度/加速度约束默认开启：IK 求解结果必须经过明确的速度预算才能成为仿真关节状态，
    # 旧默认（enable_velocity_limit=False）等于没有任何帧间约束。
    enable_velocity_limit: bool = True,
    max_joint_velocity: float = 1.0,
    max_joint_acceleration: float | None = None,
    # --- 任务目标限速 ---
    enable_target_rate_limit: bool = True,
    max_target_linear_velocity_m_s: float = 0.25,
    max_target_linear_acceleration_m_s2: float = 1.0,
    max_target_angular_velocity_rad_s: float = 1.0,
    max_target_angular_acceleration_rad_s2: float = 4.0,
    # --- SmoothReset ---
    reset_duration_s: float = 3.0,
    # --- 自碰撞避免 ---
    enable_self_collision_avoidance: bool = False,
    self_collision_margin: float = 0.03,
    # --- 运行时调参 ---
    enable_tuning: bool = False,
):
    if ik_backend not in {"placo", "official", "shared-dls"}:
        raise ValueError("--ik-backend must be 'placo', 'official' or 'shared-dls'")
    if home_joint_deg is None:
        raise ValueError(
            "--home-joint-deg is required; configure a collision-free side-mount posture instead of using zero joints"
        )
    # The Placo joint-velocity/target-rate limits are on by default now; shared-dls carries its
    # own DLS velocity budget, so its own defaults are switched off before the checks below.
    if ik_backend == "shared-dls" and (enable_velocity_limit or enable_target_rate_limit):
        print("[INFO] shared-dls 使用自身 DLS 速度预算；忽略 Placo 关节限速与目标限速选项")
        enable_velocity_limit = False
        enable_target_rate_limit = False
    if ik_backend == "shared-dls":
        if not follow_orientation:
            raise ValueError("shared-dls follows the hardware full-pose control path")
        if enable_log_data or enable_self_collision_avoidance or enable_tuning:
            raise ValueError("shared-dls does not use Placo-only logging, collision, velocity or tuning options")
        from xrobotoolkit_teleop.simulation.rm65_shared_control_simulation import (
            RM65SharedControlSimulation,
        )

        controller = RM65SharedControlSimulation(
            robot_urdf_path=robot_urdf_path,
            home_joint_deg=home_joint_deg,
            calibration_path=shared_calibration_path,
            scale_factor=shared_scale_factor,
            rotation_scale_factor=shared_rotation_scale_factor,
            reset_duration_s=reset_duration_s,
            log_motion_debug=True,
        )
        print("[INFO] ik_backend=shared-dls; 与真机共用映射、DLS 与安全准入逻辑")
        print("[INFO] 虚拟关节立即跟随已接受命令；不模拟真机执行延迟或碰撞")
        print(f"[INFO] calibration={shared_calibration_path}")
        controller.run()
        return

    q_home = np.deg2rad(np.asarray(home_joint_deg, dtype=float))
    control_mode = "pose" if follow_orientation else "position"
    config = {
        "right_hand": {
            "link_name": "r_link6",
            "pose_source": "right_controller",
            "control_trigger": "right_grip",
            "control_mode": control_mode,
        },
    }

    common_kwargs = dict(
        robot_urdf_path=robot_urdf_path,
        manipulator_config=config,
        R_headset_world=R_HEADSET_TO_WORLD_RM65_SIDE_MOUNT_HARDWARE,
        scale_factor=scale_factor,
        q_init=q_home,
        enable_log_data=enable_log_data,
        log_dir=log_dir,
        deadband_m=deadband_m,
        deadband_rad=deadband_rad,
        smooth_alpha_pos=smooth_alpha_pos,
        smooth_alpha_rot=smooth_alpha_rot,
        enable_velocity_limit=enable_velocity_limit,
        max_joint_velocity=max_joint_velocity,
        max_joint_acceleration=max_joint_acceleration,
        enable_target_rate_limit=enable_target_rate_limit,
        max_target_linear_velocity_m_s=max_target_linear_velocity_m_s,
        max_target_linear_acceleration_m_s2=max_target_linear_acceleration_m_s2,
        max_target_angular_velocity_rad_s=max_target_angular_velocity_rad_s,
        max_target_angular_acceleration_rad_s2=max_target_angular_acceleration_rad_s2,
        reset_duration_s=reset_duration_s,
        enable_self_collision_avoidance=enable_self_collision_avoidance,
        self_collision_margin=self_collision_margin,
        enable_tuning=enable_tuning,
    )
    if ik_backend == "official":
        from xrobotoolkit_teleop.simulation.realman_official_ik_teleop_controller import (
            RealmanOfficialIKSimulationController,
        )

        # The official IK path carries its own per-cycle step guard; the Placo limiters must be
        # explicitly off or they would both reject the same thing with different numbers.
        official_kwargs = dict(common_kwargs)
        official_kwargs["enable_velocity_limit"] = False
        official_kwargs["enable_target_rate_limit"] = False
        controller = RealmanOfficialIKSimulationController(
            **official_kwargs,
            official_ik_tool_or_work=official_ik_tool_or_work,
            official_ik_j3_exclusion_deg=official_ik_j3_exclusion_deg,
            official_ik_max_joint_speed_ratio=official_ik_max_joint_speed_ratio,
        )
    else:
        controller = PlacoTeleopController(**common_kwargs)
        joints_task = controller.solver.add_joints_task()
        joints_task.set_joints(dict(zip(controller.placo_robot.joint_names(), q_home)))
        joints_task.configure("joints_regularization", "soft", 1e-4)
    print(f"[INFO] calibrated home_joint_deg={np.round(home_joint_deg, 1).tolist()}")

    print(f"[INFO] ik_backend={ik_backend}")
    print(f"[INFO] control_mode={control_mode}")
    print(f"[INFO] deadband_m={deadband_m}, deadband_rad={deadband_rad}")
    print(f"[INFO] smooth_alpha_pos={smooth_alpha_pos}, smooth_alpha_rot={smooth_alpha_rot}")
    print(f"[INFO] enable_velocity_limit={enable_velocity_limit}")
    print(f"[INFO] reset_duration_s={reset_duration_s}")
    print(
        f"[INFO] enable_self_collision_avoidance={enable_self_collision_avoidance}, "
        f"margin={self_collision_margin:.3f}"
    )
    print(f"[INFO] enable_tuning={enable_tuning}")
    print(f"Open Placo/Meshcat: http://localhost:{os.environ.get('MESHCAT_HOST_PORT', '18081')}/static/")
    controller.run()


if __name__ == "__main__":
    tyro.cli(main)
