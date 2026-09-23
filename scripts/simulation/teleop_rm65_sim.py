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
    home_joint_deg: tuple[float, float, float, float, float, float] | None = None,
    ik_backend: str = "placo",
    official_ik_tool_or_work: int = 1,
    official_ik_j3_exclusion_deg: float = 5.0,
    official_ik_max_joint_speed_ratio: float = 0.10,
    # --- MotionFilter ---
    deadband_m: float = 0.002,
    deadband_rad: float = 0.03,
    smooth_alpha_pos: float = 0.35,
    smooth_alpha_rot: float = 0.3,
    # --- JointConstraints ---
    enable_velocity_limit: bool = False,
    max_joint_velocity: float = 3.0,
    # --- SmoothReset ---
    reset_duration_s: float = 3.0,
    # --- 自碰撞避免 ---
    enable_self_collision_avoidance: bool = False,
    self_collision_margin: float = 0.03,
    # --- 运行时调参 ---
    enable_tuning: bool = False,
):
    if ik_backend not in {"placo", "official"}:
        raise ValueError("--ik-backend must be 'placo' or 'official'")
    if home_joint_deg is None:
        raise ValueError(
            "--home-joint-deg is required; configure a collision-free side-mount posture instead of using zero joints"
        )
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
        reset_duration_s=reset_duration_s,
        enable_self_collision_avoidance=enable_self_collision_avoidance,
        self_collision_margin=self_collision_margin,
        enable_tuning=enable_tuning,
    )
    if ik_backend == "official":
        from xrobotoolkit_teleop.simulation.realman_official_ik_teleop_controller import (
            RealmanOfficialIKSimulationController,
        )

        controller = RealmanOfficialIKSimulationController(
            **common_kwargs,
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
