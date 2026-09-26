import sys
import time
from typing import Any

import meshcat.geometry as g
import meshcat.transformations as tf
import numpy as np

from xrobotoolkit_teleop.common.base_teleop_controller import BaseTeleopController
from xrobotoolkit_teleop.simulation.ik_pipeline import SimIkPipeline, TargetRateLimiter
from xrobotoolkit_teleop.simulation.joint_constraints import JointConstraints
from xrobotoolkit_teleop.simulation.motion_filter import MotionFilter
from xrobotoolkit_teleop.utils.geometry import (
    R_HEADSET_TO_WORLD,
    quat_diff_as_angle_axis,
)


class PlacoTeleopController(BaseTeleopController):
    """
    Placo teleoperation controller for a robot using inverse kinematics.

    Extends BaseTeleopController with:
    - MotionFilter (deadband + EMA) applied on delta before IK
    - SimIkPipeline: rate-limited task target, then JointConstraints after the solve
    - SmoothReset: B button smooth interpolation back to home joints
    """

    # 调参描述符: {key: ((obj_attr, sub_attr), step, fmt, clamp_min, clamp_max, condition_attr)}
    _TUNE_PARAMS: dict[str, tuple] = {
        "d": (("motion_filter", "deadband_m"), 0.001, ".4f", 0.0, None, None),
        "r": (("motion_filter", "deadband_rad"), 0.005, ".4f", 0.0, None, None),
        "a": (("motion_filter", "smooth_alpha_pos"), 0.05, ".3f", 0.0, 1.0, None),
        "o": (("motion_filter", "smooth_alpha_rot"), 0.05, ".3f", 0.0, 1.0, None),
        "v": (("joint_constraints", "max_joint_velocity"), 0.5, ".2f", 0.0, None, "enable_velocity_limit"),
    }

    def __init__(
        self,
        robot_urdf_path: str,
        manipulator_config: dict[str, dict[str, Any]],
        floating_base: bool = False,
        R_headset_world=R_HEADSET_TO_WORLD,
        scale_factor: float = 1.0,
        q_init: np.ndarray | None = None,
        dt: float = 0.01,
        enable_log_data: bool = False,
        log_dir: str = "logs",
        # --- 运动滤波 ---
        deadband_m: float = 0.002,
        deadband_rad: float = 0.03,
        smooth_alpha_pos: float = 0.35,
        smooth_alpha_rot: float = 0.3,
        # --- 关节约束 ---
        enable_velocity_limit: bool = True,
        max_joint_velocity: float = 1.0,
        max_joint_acceleration: float | None = None,
        # --- 任务目标限速 ---
        enable_target_rate_limit: bool = True,
        max_target_linear_velocity_m_s: float = 0.25,
        max_target_linear_acceleration_m_s2: float = 1.0,
        max_target_angular_velocity_rad_s: float = 1.0,
        max_target_angular_acceleration_rad_s2: float = 4.0,
        # --- 平滑复位 ---
        reset_duration_s: float = 3.0,
        # --- 自碰撞避免 ---
        enable_self_collision_avoidance: bool = False,
        self_collision_margin: float = 0.03,
        # --- 运行时调参 ---
        enable_tuning: bool = False,
    ):
        super().__init__(
            robot_urdf_path,
            manipulator_config,
            floating_base,
            R_headset_world,
            scale_factor,
            q_init,
            dt,
            enable_log_data=enable_log_data,
            log_dir=log_dir,
        )
        self._init_placo_viz()
        self._init_reset_viz()

        # MotionFilter
        self.motion_filter = MotionFilter(
            deadband_m=deadband_m,
            deadband_rad=deadband_rad,
            smooth_alpha_pos=smooth_alpha_pos,
            smooth_alpha_rot=smooth_alpha_rot,
        )

        # JointConstraints
        self.joint_constraints = JointConstraints(
            placo_robot=self.placo_robot,
            dt=self.dt,
            enable_velocity_limit=enable_velocity_limit,
            max_joint_velocity=max_joint_velocity,
            max_joint_acceleration=max_joint_acceleration,
        )

        # 任务目标限速：手柄小幅连续运动 → 目标小幅连续移动。
        # 关闭时退化为旧行为（IK 任务直接收到滤波器输出）。
        self._target_rate_limit_enabled = bool(enable_target_rate_limit)
        self.target_limiter = TargetRateLimiter(
            max_linear_velocity_m_s=max_target_linear_velocity_m_s,
            max_linear_acceleration_m_s2=max_target_linear_acceleration_m_s2,
            max_angular_velocity_rad_s=max_target_angular_velocity_rad_s,
            max_angular_acceleration_rad_s2=max_target_angular_acceleration_rad_s2,
        )
        self._pipeline: dict[str, SimIkPipeline] = {}

        # --- 平滑复位状态 ---
        self._resetting = False
        self._reset_step = 0
        self._reset_steps_total = max(1, int(reset_duration_s / max(self.dt, 1e-6)))
        self._q_start: np.ndarray | None = None
        self._q_home = None
        if self.q_init is not None:
            q_home = np.asarray(self.q_init, dtype=float)
            if q_home.shape != (len(self.placo_robot.joint_names()),):
                raise ValueError(
                    f"q_init must contain {len(self.placo_robot.joint_names())} joint values, got shape {q_home.shape}"
                )
            if not np.all(np.isfinite(q_home)):
                raise ValueError("q_init must contain only finite joint values")
            if np.any(q_home < self.joint_constraints.lower_limits) or np.any(
                q_home > self.joint_constraints.upper_limits
            ):
                raise ValueError("q_init is outside the URDF joint limits")
            self._q_home = q_home.copy()
        self._prev_b_button = False

        # --- 自碰撞避免 ---
        self._self_collision_enabled = enable_self_collision_avoidance
        if enable_self_collision_avoidance:
            try:
                sc_task = self.solver.add_self_collision_avoidance_task(
                    "self_collision_avoidance",
                    margin=self_collision_margin,
                )
                sc_task.configure("self_collision_avoidance", "soft", 1.0)
                print(f"[INFO] 自碰撞避免已启用，安全间距={self_collision_margin:.3f}m")
            except Exception as e:
                print(f"[WARN] 自碰撞避免初始化失败: {e}")

        # --- 运行时调参 ---
        self._enable_tuning = enable_tuning
        if enable_tuning:
            print("[TUNE] 运行时调参已启用，按 h 查看帮助")

    # ---- 钩子重写 ----

    def _filter_delta(self, name: str, delta_xyz: np.ndarray, delta_rot: np.ndarray):
        """重写基类钩子：对 active 的机械臂做运动滤波"""
        if self.active.get(name, False):
            return self.motion_filter.apply(delta_xyz, delta_rot)
        return delta_xyz, delta_rot

    def _on_grip_state_change(self, name: str, active: bool) -> None:
        """重写基类钩子：grip 状态变化时重置滤波与限速历史"""
        if active:
            self.motion_filter.reset()
            self._pipeline.pop(name, None)
        else:
            pipeline = self._pipeline.pop(name, None)
            if pipeline is not None:
                pipeline.release()

    # ---- 仿真后端 ----

    def _robot_setup(self):
        pass

    def _send_command(self):
        self._update_placo_viz()
        self._update_reset_viz()

    def _update_robot_state(self):
        pass

    def _get_link_pose(self, link_name):
        link_xyz = self.placo_robot.get_T_world_frame(link_name)[:3, 3]
        link_quat = tf.quaternion_from_matrix(self.placo_robot.get_T_world_frame(link_name))
        return link_xyz, link_quat

    # ---- IK 管线 ----

    def _solve_ik(self):
        """执行 IK 求解（从基类 _update_ik 中移出）"""
        try:
            self.solver.solve(True)
        except RuntimeError as e:
            print(f"IK solver failed: {e}")

    def _pipeline_for(self, name, config):
        """每个手柄一个限速管线；首次激活时锚定到当前末端位姿。"""
        pipeline = self._pipeline.get(name)
        link_name = config["link_name"]
        ee_xyz, ee_quat = self._get_link_pose(link_name)
        if pipeline is None:
            pipeline = SimIkPipeline(
                dt=self.dt,
                motion_filter=self.motion_filter,
                joint_constraints=self.joint_constraints,
                target_limiter=self.target_limiter,
                position_control=self.effector_control_mode.get(name, "pose") == "position",
            )
            self._pipeline[name] = pipeline
            pipeline.sync(self.placo_robot.state.q[7:], ee_xyz, ee_quat)
        return pipeline, ee_xyz, ee_quat

    def _apply_joint_constraints(self):
        """对 IK 求解后的关节值做限位 + 速度限制"""
        q = self.placo_robot.state.q[7:]  # 去掉浮动基座 7 个虚拟关节
        q_safe = self.joint_constraints.apply(q)
        self.placo_robot.state.q[7:] = q_safe
        self.placo_robot.update_kinematics()

    def _update_ik(self):
        """基类管线，但把任务目标先过一层笛卡尔限速。

        原来的实现对每个手柄直接把 ``ref_ee + filtered_delta`` 写进任务目标，滤波器在
        死区附近的逐分量归零会让目标出现台阶；IK 求解结果又直接成为下一帧关节状态，
        没有任何速度约束，于是目标台阶变成了关节跳变。这里改为：
        手柄增量 → 滤波 → **限速后的任务目标** → IK → 关节限位/速度约束。
        """
        self._update_robot_state()
        self.placo_robot.update_kinematics()

        if self._debug_xr and time.time() - self._last_debug_t > 2.0:
            self._last_debug_t = time.time()
            for src_name, config in self.manipulator_config.items():
                grip = self.xr_client.get_key_value_by_name(config["control_trigger"])
                pose = self.xr_client.get_pose_by_name(config["pose_source"])
                print(f"[XR] {src_name}: grip={grip:.2f} " f"pose_xyz={[round(p, 3) for p in pose[:3]]}")
            lg = self.xr_client.get_key_value_by_name("left_grip")
            rg = self.xr_client.get_key_value_by_name("right_grip")
            if max(lg, rg) < self._grip_threshold and not self._grip_hint_shown:
                print("[XR] 按住手柄侧边 Grip 键（握把）才能驱动机械臂，" f"当前阈值 {self._grip_threshold:.1f}")
                self._grip_hint_shown = True

        for src_name, config in self.manipulator_config.items():
            xr_grip_val = self.xr_client.get_key_value_by_name(config["control_trigger"])
            self.active[src_name] = xr_grip_val > self._grip_threshold

            # Grip 边界触发数据记录器
            if self.data_collector is not None:
                self._handle_data_collector_grip(src_name)
            # Grip 状态变更钩子（通知子类）: 在更新 _prev_grip_active 之前检查
            if self.active[src_name] != self._prev_grip_active.get(src_name, False):
                self._on_grip_state_change(src_name, self.active[src_name])
            self._prev_grip_active[src_name] = self.active[src_name]

            if self.active[src_name]:
                pipeline, ee_xyz, ee_quat = self._pipeline_for(src_name, config)
                if self.ref_ee_xyz[src_name] is None:
                    print(f"{src_name} is activated.")
                    self.ref_ee_xyz[src_name] = pipeline.reference_ee_xyz
                    self.ref_ee_quat[src_name] = pipeline.reference_ee_quat

                xr_pose = self.xr_client.get_pose_by_name(config["pose_source"])
                delta_xyz, delta_rot = self._process_xr_pose(xr_pose, src_name)
                if not self._target_rate_limit_enabled:
                    delta_xyz, delta_rot = self.motion_filter.apply(delta_xyz, delta_rot)
                    self._write_task_target(src_name, config, *self._unlimited_target(
                        src_name, delta_xyz, delta_rot, ee_xyz, ee_quat
                    ))
                else:
                    target = pipeline.step(delta_xyz, delta_rot, ee_xyz, ee_quat)
                    self._write_task_target(src_name, config, *target)

                if self.data_collector is not None:
                    filtered_xyz = pipeline.telemetry.get("filtered_delta_xyz", delta_xyz)
                    filtered_rot = pipeline.telemetry.get("filtered_delta_rot", delta_rot)
                    self._record_data_step(src_name, config, filtered_xyz, filtered_rot, xr_grip_val)
            else:
                if self.ref_ee_xyz[src_name] is not None:
                    print(f"{src_name} is deactivated.")
                    self.ref_ee_xyz[src_name] = None
                    self.ref_controller_xyz[src_name] = None

        # Process motion tracker data
        self._update_motion_tracker_tasks()

    def _unlimited_target(self, name, delta_xyz, delta_rot, ee_xyz, ee_quat):
        """旧行为（--no-enable-target-rate-limit）：滤波后的增量直接进任务目标。"""
        from xrobotoolkit_teleop.utils.geometry import apply_delta_pose

        if self.effector_control_mode.get(name, "pose") == "position":
            return self.ref_ee_xyz[name] + delta_xyz, np.asarray(ee_quat, dtype=float)
        return apply_delta_pose(self.ref_ee_xyz[name], self.ref_ee_quat[name], delta_xyz, delta_rot)

    def _write_task_target(self, name, config, target_xyz, target_quat_wxyz):
        if target_xyz is None or target_quat_wxyz is None:
            return
        if self.effector_control_mode[name] == "position":
            self.effector_task[name].target_world = np.asarray(target_xyz, dtype=float)
            return
        target_pose = tf.quaternion_matrix(np.asarray(target_quat_wxyz, dtype=float))
        target_pose[:3, 3] = np.asarray(target_xyz, dtype=float)
        self.effector_task[name].T_world_frame = target_pose

    def _finalize_ik(self):
        """IK 求完之后再过关节限位/速度约束，并写回显示状态。"""
        self._apply_joint_constraints()

    # ---- 平滑复位 ----

    def _start_smooth_reset(self):
        """开始平滑复位：记录当前 q，切换到复位模式"""
        if self._q_home is None:
            print("[WARN] reset ignored: configure an explicit safe q_init/home_joint_deg first")
            return
        self._resetting = True
        self._reset_step = 0
        self._q_start = self.placo_robot.state.q[7:].copy()
        print("[RESET] 平滑复位开始...")

    def _tick_smooth_reset(self):
        """执行一帧平滑复位插值。插值完成后恢复 IK。"""
        if self._q_home is None:
            self._resetting = False
            return
        alpha = self._reset_step / self._reset_steps_total
        if alpha >= 1.0:
            self.placo_robot.state.q[7:] = self._q_home.copy()
            self._resetting = False
            self._reset_step = 0
            print("[RESET] 复位完成")
        else:
            q_cur = (1.0 - alpha) * self._q_start + alpha * self._q_home
            self.placo_robot.state.q[7:] = q_cur
            self._reset_step += 1

        self._apply_joint_constraints()  # 内部已调用 update_kinematics
        if not self._resetting:
            # The reset ends at a pose the arm reached by interpolation; the velocity limiter
            # must re-anchor there or the first IK cycle would be measured against the previous
            # interpolation step.
            self._reanchor_after_reset()

    def _reanchor_after_reset(self):
        self.joint_constraints.sync(self.placo_robot.state.q[7:])
        for name, config in self.manipulator_config.items():
            pipeline = self._pipeline.get(name)
            if pipeline is None:
                continue
            ee_xyz, ee_quat = self._get_link_pose(config["link_name"])
            pipeline.sync(self.placo_robot.state.q[7:], ee_xyz, ee_quat)
            self.ref_ee_xyz[name] = pipeline.reference_ee_xyz
            self.ref_ee_quat[name] = pipeline.reference_ee_quat

    # ---- 复位状态可视化 ----

    def _init_reset_viz(self):
        """在 Meshcat 场景中添加复位状态指示灯。"""
        viewer = self.placo_vis.viewer

        indicator_pos = tf.translation_matrix([0.0, 0.0, 0.62])
        self._reset_indicator = viewer["status/reset_indicator"]
        self._reset_indicator.set_object(g.Sphere(0.025))
        self._reset_indicator.set_transform(indicator_pos)
        self._reset_indicator.set_property("color", 0x44CC44)
        self._reset_indicator.set_property("material", {"transparent": True, "opacity": 0.8})

        self._prev_resetting_viz = False

    def _update_reset_viz(self):
        """每帧更新复位状态显示（指示灯颜色）。"""
        if self._resetting:
            if not self._prev_resetting_viz:
                self._reset_indicator.set_property("color", 0xFF8800)  # 橙色
                self._prev_resetting_viz = True
        else:
            if self._prev_resetting_viz:
                self._reset_indicator.set_property("color", 0x44CC44)  # 绿色
                self._prev_resetting_viz = False

    # ---- 运行时调参 ----

    def _print_tuning_help(self):
        """打印当前可调参数值和快捷键"""
        print("=" * 48)
        print(f"  [d/D] deadband_m     = {self.motion_filter.deadband_m:.4f}  m")
        print(f"  [r/R] deadband_rad   = {self.motion_filter.deadband_rad:.4f}  rad")
        print(f"  [a/A] smooth_alpha_pos = {self.motion_filter.smooth_alpha_pos:.3f}")
        print(f"  [o/O] smooth_alpha_rot = {self.motion_filter.smooth_alpha_rot:.3f}")
        if self.joint_constraints.enable_velocity_limit:
            print(f"  [v/V] max_joint_vel   = {self.joint_constraints.max_joint_velocity:.2f}  rad/s")
        print("  [h]   显示帮助")
        print("  [q]   关闭调参模式")
        print("=" * 48)

    def _handle_tuning(self):
        """非阻塞键盘输入，运行时调参（仅 Linux TTY 环境）。"""
        if not self._enable_tuning:
            return

        try:
            import select

            if sys.platform == "win32":
                return
            if sys.stdin not in select.select([sys.stdin], [], [], 0)[0]:
                return
            key = sys.stdin.read(1)
        except (ValueError, AttributeError, OSError):
            return

        c = key.lower()

        if key == "h":
            self._print_tuning_help()
        elif key == "q":
            self._enable_tuning = False
            print("[TUNE] 调参模式已关闭")
        elif c in self._TUNE_PARAMS:
            (obj_name, attr_name), step, fmt, clamp_min, clamp_max, cond_attr = self._TUNE_PARAMS[c]
            obj = getattr(self, obj_name)
            if cond_attr is not None and not getattr(obj, cond_attr):
                return
            # Uppercase increases the parameter, lowercase decreases it.
            delta = step if not key.isupper() else -step
            new_val = getattr(obj, attr_name) - delta
            if clamp_max is not None:
                new_val = float(np.clip(new_val, clamp_min, clamp_max))
            else:
                new_val = max(clamp_min, new_val)
            setattr(obj, attr_name, new_val)
            print(f"  {attr_name} = {new_val:{fmt}}")

    # ---- 主循环 ----

    def run(self):
        """主遥操作循环：检测 B 按钮 → 复位/正常 IK 管线 → 显示"""
        while not self._stop_event.is_set():
            try:
                start_time = time.time()

                # 检测 B 按钮上升沿触发复位
                b_button = self.xr_client.get_button_state_by_name("B")
                if b_button and not self._prev_b_button and not self._resetting:
                    self._start_smooth_reset()
                self._prev_b_button = b_button

                if self._resetting:
                    self._tick_smooth_reset()
                else:
                    # 正常 IK 管线
                    self._update_ik()  # 读 XR → 算 delta → 限速任务目标
                    self._solve_ik()  # IK 求解
                    self._finalize_ik()  # 关节限位 + 速度/加速度约束

                self._send_command()  # Meshcat 显示
                end_time = time.time()
                time.sleep(max(0, self.dt - (end_time - start_time)))
                self._handle_tuning()  # 运行时调参（if enabled）
            except KeyboardInterrupt:
                print("\nTeleoperation stopped.")
                self._stop_event.set()
