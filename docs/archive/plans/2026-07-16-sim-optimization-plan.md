# 仿真环境优化实施计划

> **状态**：✅ 已完成（MotionFilter / JointConstraints / SmoothReset 均已落地）
> **原始日期**：2026-07-16 | **本文档整理于**：2026-08-27

**目标：** 为仿真遥操作环境新增 MotionFilter（死区+EMA平滑）、JointConstraints（关节限位+速度限制）、SmoothReset（B按钮一键平滑复位），同时修复阻塞性 bug（DataLogger 导入崩溃）。

**架构：** 在 `xrobotoolkit_teleop/simulation/` 下新建 `motion_filter.py` 和 `joint_constraints.py` 两个纯函数模块，修改 `PlacoTeleopController` 在 `run()` 循环中嵌入 filter → IK → constraints 管线，并加入 B 按钮检测的平滑复位状态机。`BaseTeleopController` 只做最小改动（删无效导入 + 加钩子 + 移 solver.solve）。硬件路径完全不受影响。

**技术栈：** Python 3.10+, numpy, Placo, Meshcat, pytest

**交叉依赖检查：**
- 硬件控制器 `RealmanRM65CartesianTeleopController` 是独立类（不继承 `BaseTeleopController`），不调用 `_update_ik()`——所有基类改动对硬件零影响。
- `parallel_gripper_utils` 导入错误保留不动（本计划不涉及夹爪）。
- 旧入口 `teleop_dual_ur5e_placo.py` 最终删除，在此之前新旧入口可共存，互不影响。

---

## 文件结构

| 文件 | 操作 | 职责 |
|------|------|------|
| `xrobotoolkit_teleop/simulation/motion_filter.py` | **新建** | 死区过滤 + EMA 平滑 |
| `xrobotoolkit_teleop/simulation/joint_constraints.py` | **新建** | 关节限位裁剪 + 速度限制 |
| `xrobotoolkit_teleop/simulation/placo_teleop_controller.py` | **修改** | 集成 filter/constraints/reset 管线 |
| `xrobotoolkit_teleop/common/base_teleop_controller.py` | **修改** | 删 DataLogger、加钩子、移 solver.solve |
| `scripts/simulation/teleop_rm65_sim.py` | **新建** | 新入口脚本（CLI 参数扩展） |
| `scripts/simulation/teleop_dual_ur5e_placo.py` | **删除** | 旧入口 |
| `tests/test_motion_filter.py` | **新建** | MotionFilter 单元测试 |
| `tests/test_joint_constraints.py` | **新建** | JointConstraints 单元测试 |
| `docker/Dockerfile` | **修改** | CMD 文件名 |
| `CLAUDE.md` | **修改** | 更新运行命令 |

---

### Task 1: 修复 BaseTeleopController — 删 DataLogger + 加钩子 + 移 solver.solve

**文件：**
- Modify: `xrobotoolkit_teleop/common/base_teleop_controller.py:19,61,291-294`
- 注意：不改其他行，不改硬件路径

- [ ] **Step 1: 删除 DataLogger 无效导入和第 61 行构造**

  替换第 19 行 `from xrobotoolkit_teleop.common.data_logger import DataLogger` 为空（删掉这行）。

  替换第 60-61 行（删掉 DataLogger 构造）：

  ```python
          self.enable_log_data = enable_log_data
          self.log_dir = log_dir
          self.log_freq = log_freq
          if enable_log_data:
              self.data_logger = DataLogger(log_dir=log_dir)
  ```

  改为：

  ```python
          self.enable_log_data = enable_log_data
          self.log_dir = log_dir
          self.log_freq = log_freq
  ```

  注意保留 `if enable_log_data:` 下面的 DataCollector 那段（第 88-94 行不动）。

- [ ] **Step 2: 新增 `_filter_delta` 钩子和 `_on_grip_state_change` 钩子**

  在 `_process_xr_pose` 方法之后、`_placo_setup` 方法之前（第 132-133 行之间），插入：

  ```python
      def _filter_delta(self, name: str, delta_xyz: np.ndarray, delta_rot: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
          """钩子：子类可在此对 delta 做滤波处理。默认透传，不影响硬件子类。"""
          return delta_xyz, delta_rot

      def _on_grip_state_change(self, name: str, active: bool) -> None:
          """钩子：grip 状态变更时调用。基类默认空实现。"""
          pass
  ```

- [ ] **Step 3: 在 `_update_ik()` 中插入 `_filter_delta` 调用**

  在第 249 行之后、第 251 行之前（`delta_xyz, delta_rot = self._process_xr_pose(...)` 之后，`if self.effector_control_mode...` 之前），插入：

  ```python
                  delta_xyz, delta_rot = self._filter_delta(src_name, delta_xyz, delta_rot)
  ```

- [ ] **Step 4: 在 grip 激活/释放处插入 `_on_grip_state_change` 调用**

  在第 240 行（`self._prev_grip_active[src_name] = self.active[src_name]`）之后，插入：

  ```python
              # Grip 状态变更钩子
              is_active = self.active[src_name]
              was_active = self._prev_grip_active.get(src_name, False)
              if is_active != was_active:
                  pass  # 钩子逻辑移到了 _prev_grip_active 更新之前
  ```

  等等，这里的逻辑需要重新组织。当前代码的顺序是：

  ```
  line 228: self.active[src_name] = xr_grip_val > self._grip_threshold
  line 231-239: 数据采集器边界检测（用 self.active 和 self._prev_grip_active）
  line 240: self._prev_grip_active[src_name] = self.active[src_name]
  line 243: if self.active[src_name]:
  ```

  正确的做法：在 `self._prev_grip_active` 更新之前（第 240 行前），插入 `_on_grip_state_change` 调用：

  将第 240 行 `self._prev_grip_active[src_name] = self.active[src_name]` **之后**，插入：

  ```python
              # Grip 状态变更钩子（通知子类）
              if self.active[src_name] != self._prev_grip_active[src_name]:
                  self._on_grip_state_change(src_name, self.active[src_name])
  ```

  注意：这需要在 `self._prev_grip_active` 更新之前判断旧值。当前代码 `self._prev_grip_active[src_name] = self.active[src_name]` 在第 240 行，已经覆盖了旧值。因此需要**先将第 240 行移到后面**，或者用 `was_active`。

  更简洁的方式：用已有的 `was_active` 变量（第 233 行有 `was_active = self._prev_grip_active.get(src_name, False)`），在数据采集块之后插入：

  ```python
              # --- Grip 边界触发结束 ---

              # Grip 状态变更钩子（通知子类）
              if self.active[src_name] != self._prev_grip_active.get(src_name, False):
                  self._on_grip_state_change(src_name, self.active[src_name])
  ```

  插入在 `# --- Grip 边界触发结束 ---` 那一行（第 241 行）之后、`if self.active[src_name]:`（第 243 行）之前。

- [ ] **Step 5: 从 `_update_ik()` 中移除 `solver.solve()` 调用**

  删除第 291-294 行：

  ```python
          try:
              self.solver.solve(True)
          except RuntimeError as e:
              print(f"IK solver failed: {e}")
  ```

  这段移到 `PlacoTeleopController` 的 `_solve_ik()` 方法中。

- [ ] **Step 6: 验证修改后的文件不报语法错误**

  Run: `python -c "import ast; ast.parse(open('xrobotoolkit_teleop/common/base_teleop_controller.py').read()); print('Syntax OK')"`
  Expected: `Syntax OK`

- [ ] **Step 7: Commit**

  ```bash
  git add -A
  git commit -m "fix: remove broken DataLogger import, add filter/grip hooks, move solver.solve to sim"
  ```

---

### Task 2: 创建 MotionFilter（死区 + EMA 平滑）+ 单元测试

**文件：**
- Create: `xrobotoolkit_teleop/simulation/motion_filter.py`
- Create: `tests/test_motion_filter.py`

- [ ] **Step 1: 编写 MotionFilter 单元测试**

  `tests/test_motion_filter.py`：

  ```python
  import numpy as np
  import pytest
  from xrobotoolkit_teleop.simulation.motion_filter import MotionFilter

  class TestMotionFilter:
      def test_deadband_zeroes_small_deltas(self):
          """平移分量小于 deadband 时应归零"""
          f = MotionFilter(deadband_m=0.01, deadband_rad=1.0)
          delta_xyz = np.array([0.005, 0.02, 0.003])  # 0.005<0.01, 0.02>0.01, 0.003<0.01
          delta_rot = np.array([0.0, 0.0, 0.0])
          out_xyz, out_rot = f.apply(delta_xyz, delta_rot)
          assert out_xyz[0] == 0.0  # 0.005 < 0.01 → 归零
          assert out_xyz[1] == pytest.approx(0.02)  # 0.02 > 0.01 → 保留
          assert out_xyz[2] == 0.0  # 0.003 < 0.01 → 归零

      def test_rot_deadband_zeroes_small_rotations(self):
          """旋转分量小于 deadband 时应归零"""
          f = MotionFilter(deadband_m=1.0, deadband_rad=0.05)
          delta_xyz = np.zeros(3)
          delta_rot = np.array([0.03, 0.06, 0.01])  # 0.03<0.05, 0.06>0.05, 0.01<0.05
          out_xyz, out_rot = f.apply(delta_xyz, delta_rot)
          assert out_rot[0] == 0.0
          assert out_rot[1] == pytest.approx(0.06)
          assert out_rot[2] == 0.0

      def test_ema_smooths_over_time(self):
          """EMA 应以 alpha 系数趋近稳定值"""
          f = MotionFilter(deadband_m=0, deadband_rad=0, smooth_alpha_pos=0.5, smooth_alpha_rot=1.0)
          delta = np.array([1.0, 0.0, 0.0])
          # 第一次: y = 0.5*1 + 0.5*0 = 0.5
          out, _ = f.apply(delta, np.zeros(3))
          assert out[0] == pytest.approx(0.5)
          # 第二次: y = 0.5*1 + 0.5*0.5 = 0.75
          out, _ = f.apply(delta, np.zeros(3))
          assert out[0] == pytest.approx(0.75)
          # 第三次: y = 0.5*1 + 0.5*0.75 = 0.875
          out, _ = f.apply(delta, np.zeros(3))
          assert out[0] == pytest.approx(0.875)

      def test_reset_clears_history(self):
          """reset 后 EMA 应从零重新开始"""
          f = MotionFilter(deadband_m=0, deadband_rad=0, smooth_alpha_pos=0.5)
          delta = np.array([1.0, 0.0, 0.0])
          f.apply(delta, np.zeros(3))  # → 0.5
          f.apply(delta, np.zeros(3))  # → 0.75
          f.reset()
          out, _ = f.apply(delta, np.zeros(3))  # → 0.5
          assert out[0] == pytest.approx(0.5)

      def test_zero_alpha_means_no_smoothing(self):
          """alpha=1.0 时应透传（无平滑效果）"""
          f = MotionFilter(deadband_m=0, deadband_rad=0, smooth_alpha_pos=1.0)
          delta = np.array([0.42, 0.0, 0.0])
          out, _ = f.apply(delta, np.zeros(3))
          assert out[0] == pytest.approx(0.42)

      def test_components_independent(self):
          """每个分量独立处理"""
          f = MotionFilter(deadband_m=0, deadband_rad=0, smooth_alpha_pos=0.3)
          delta_xyz = np.array([1.0, 2.0, 3.0])
          out, _ = f.apply(delta_xyz, np.zeros(3))
          assert out[0] < 1.0
          assert out[1] < 2.0
          assert out[2] < 3.0
  ```

- [ ] **Step 2: 运行测试验证失败**

  Run: `python -m pytest tests/test_motion_filter.py -v --no-header 2>&1 | head -20`
  Expected: `ModuleNotFoundError: No module named 'xrobotoolkit_teleop.simulation.motion_filter'`

- [ ] **Step 3: 编写 MotionFilter 实现**

  `xrobotoolkit_teleop/simulation/motion_filter.py`：

  ```python
  from __future__ import annotations

  import numpy as np


  class MotionFilter:
      """死区过滤 + EMA 平滑，作用于 XR delta 信号。

      - 死区：每个分量独立判断，|v| < deadband 则归零
      - EMA：y_t = alpha * x_t + (1-alpha) * y_{t-1}
      """

      def __init__(
          self,
          deadband_m: float = 0.002,
          deadband_rad: float = 0.03,
          smooth_alpha_pos: float = 0.35,
          smooth_alpha_rot: float = 0.3,
      ):
          self.deadband_m = deadband_m
          self.deadband_rad = deadband_rad
          self.smooth_alpha_pos = float(np.clip(smooth_alpha_pos, 0.0, 1.0))
          self.smooth_alpha_rot = float(np.clip(smooth_alpha_rot, 0.0, 1.0))
          self._prev_xyz: np.ndarray | None = None
          self._prev_rot: np.ndarray | None = None

      def reset(self) -> None:
          """重置 EMA 历史，grip 按下时调用"""
          self._prev_xyz = None
          self._prev_rot = None

      def apply(
          self,
          delta_xyz: np.ndarray,
          delta_rot: np.ndarray,
      ) -> tuple[np.ndarray, np.ndarray]:
          """死区过滤 → EMA 平滑 → 返回 (filtered_xyz, filtered_rot)"""
          # 死区
          out_xyz = np.where(np.abs(delta_xyz) >= self.deadband_m, delta_xyz, 0.0)
          out_rot = np.where(np.abs(delta_rot) >= self.deadband_rad, delta_rot, 0.0)

          # EMA 平滑
          if self._prev_xyz is not None:
              out_xyz = self._prev_xyz + self.smooth_alpha_pos * (out_xyz - self._prev_xyz)
              out_rot = self._prev_rot + self.smooth_alpha_rot * (out_rot - self._prev_rot)

          self._prev_xyz = out_xyz.copy()
          self._prev_rot = out_rot.copy()

          return out_xyz, out_rot
  ```

- [ ] **Step 4: 运行测试验证通过**

  Run: `python -m pytest tests/test_motion_filter.py -v --no-header`
  Expected: 6 passed

- [ ] **Step 5: Commit**

  ```bash
  git add -A
  git commit -m "feat: add MotionFilter (deadband + EMA smoothing)"
  ```

---

### Task 3: 创建 JointConstraints（关节限位 + 速度限制）+ 单元测试

**文件：**
- Create: `xrobotoolkit_teleop/simulation/joint_constraints.py`
- Create: `tests/test_joint_constraints.py`

- [ ] **Step 1: 编写 JointConstraints 单元测试**

  `tests/test_joint_constraints.py`：

  ```python
  import numpy as np
  import pytest
  from unittest.mock import MagicMock
  from xrobotoolkit_teleop.simulation.joint_constraints import JointConstraints


  def _make_mock_robot(joint_names, lower, upper):
      """创建模拟 placo_robot，提供 joint_names 和 limit 属性。"""
      robot = MagicMock()
      robot.joint_names.return_value = joint_names
      # 模拟 joint 的 limit 属性
      limits = {}
      for i, name in enumerate(joint_names):
          limit_mock = MagicMock()
          limit_mock.lower = lower[i]
          limit_mock.upper = upper[i]
          limits[name] = limit_mock
      robot.joint = lambda name: limits[name]
      return robot


  class TestJointConstraints:
      def test_clips_to_limits(self):
          """关节值超出限位时应被裁剪到限位范围内"""
          robot = _make_mock_robot(
              ["j1", "j2", "j3"],
              np.array([-1.0, -2.0, -3.0]),
              np.array([1.0, 2.0, 3.0]),
          )
          c = JointConstraints(placo_robot=robot, dt=0.01)
          q = np.array([5.0, 0.0, -5.0])  # j1 超上限, j3 超下限
          q_out = c.apply(q)
          assert q_out[0] == pytest.approx(1.0)  # clip 到上限
          assert q_out[1] == pytest.approx(0.0)
          assert q_out[2] == pytest.approx(-3.0)  # clip 到下限

      def test_within_limits_not_modified(self):
          """关节值在限位内时不做裁剪"""
          robot = _make_mock_robot(
              ["j1", "j2"],
              np.array([-1.0, -1.0]),
              np.array([1.0, 1.0]),
          )
          c = JointConstraints(placo_robot=robot, dt=0.01)
          q = np.array([0.5, -0.5])
          q_out = c.apply(q)
          assert q_out[0] == pytest.approx(0.5)
          assert q_out[1] == pytest.approx(-0.5)

      def test_velocity_limit_enforced(self):
          """启用了速度限制时，帧间变化不应超过 max_vel * dt"""
          robot = _make_mock_robot(
              ["j1", "j2"],
              np.array([-10.0, -10.0]),
              np.array([10.0, 10.0]),
          )
          c = JointConstraints(
              placo_robot=robot,
              dt=0.01,
              enable_velocity_limit=True,
              max_joint_velocity=1.0,  # rad/s
          )
          # 第一帧：无历史，透传
          q1 = np.array([0.0, 0.0])
          c.apply(q1)
          # 第二帧：从 0 到 5，超出 1.0*0.01=0.01
          q2 = np.array([5.0, 0.0])
          q_out = c.apply(q2)
          assert q_out[0] == pytest.approx(0.01)  # 限速到 0.01 rad
          assert q_out[1] == pytest.approx(0.0)

      def test_velocity_limit_off_by_default(self):
          """默认不开启速度限制，帧间跳变不被限制"""
          robot = _make_mock_robot(
              ["j1"],
              np.array([-10.0]),
              np.array([10.0]),
          )
          c = JointConstraints(placo_robot=robot, dt=0.01)
          c.apply(np.array([0.0]))
          q_out = c.apply(np.array([5.0]))  # 默认未开启速度限制
          assert q_out[0] == pytest.approx(5.0)  # 直接通过

      def test_boundary_values(self):
          """关节值等于限位边界时不被裁剪"""
          robot = _make_mock_robot(
              ["j1"],
              np.array([-1.0]),
              np.array([1.0]),
          )
          c = JointConstraints(placo_robot=robot, dt=0.01)
          q_out = c.apply(np.array([1.0]))
          assert q_out[0] == pytest.approx(1.0)
          q_out = c.apply(np.array([-1.0]))
          assert q_out[0] == pytest.approx(-1.0)
  ```

- [ ] **Step 2: 运行测试验证失败**

  Run: `python -m pytest tests/test_joint_constraints.py -v --no-header 2>&1 | head -20`
  Expected: `ModuleNotFoundError: No module named 'xrobotoolkit_teleop.simulation.joint_constraints'`

- [ ] **Step 3: 编写 JointConstraints 实现**

  `xrobotoolkit_teleop/simulation/joint_constraints.py`：

  ```python
  from __future__ import annotations

  import numpy as np
  import placo


  class JointConstraints:
      """关节限位裁剪 + 速度限制。

      从 URDF 自动读取 <limit lower="..." upper="..."/>，对 IK 求解后的关节配置
      依次做限位裁剪和速度限制，防止仿真中出现关节角度超限或帧间跳变过大。
      """

      def __init__(
          self,
          placo_robot: placo.RobotWrapper,
          dt: float,
          enable_velocity_limit: bool = False,
          max_joint_velocity: float = 3.0,
      ):
          self.dt = dt
          self.enable_velocity_limit = enable_velocity_limit
          self.max_joint_velocity = max_joint_velocity
          self._prev_q: np.ndarray | None = None

          # 从 URDF 读取关节限位（跳过浮动基座的 7 个虚拟关节）
          joint_names = placo_robot.joint_names()
          lower = []
          upper = []
          for name in joint_names:
              limit = placo_robot.joint(name)
              lower.append(float(limit.lower))
              upper.append(float(limit.upper))
          self.lower_limits = np.asarray(lower, dtype=np.float64)
          self.upper_limits = np.asarray(upper, dtype=np.float64)

      def apply(self, q: np.ndarray) -> np.ndarray:
          """关节限位 clip → 速度限制 clip → 返回安全 q"""
          # 1. 关节限位
          q_safe = np.clip(q, self.lower_limits, self.upper_limits)

          # 2. 速度限制
          if self.enable_velocity_limit and self._prev_q is not None:
              max_step = self.max_joint_velocity * self.dt
              delta = q_safe - self._prev_q
              delta = np.clip(delta, -max_step, max_step)
              q_safe = self._prev_q + delta

          self._prev_q = q_safe.copy()
          return q_safe
  ```

- [ ] **Step 4: 运行测试验证通过**

  Run: `python -m pytest tests/test_joint_constraints.py -v --no-header`
  Expected: 5 passed

- [ ] **Step 5: Commit**

  ```bash
  git add -A
  git commit -m "feat: add JointConstraints (joint limit clipping + velocity limiting)"
  ```

---

### Task 4: 重写 PlacoTeleopController — 集成 filter/constraints/reset

**文件：**
- Modify: `xrobotoolkit_teleop/simulation/placo_teleop_controller.py`

注意：这个文件原本约 74 行，重写后约 180 行。修改前通读原文件确定要保留的部分：

**保留：** `_init_placo_viz()`、`_update_placo_viz()`、`_get_link_pose()`、`__init__` 的 super() 调用
**修改：** `__init__` 加参数、重写 `run()`、新增 `_apply_motion_filter()`、`_solve_ik()`、`_apply_joint_constraints()`、`_start_smooth_reset()`、`_tick_smooth_reset()`、`_filter_delta()`、`_on_grip_state_change()`

- [ ] **Step 1: 读取原文件确保了解当前状态**

  Run: `cat -n xrobotoolkit_teleop/simulation/placo_teleop_controller.py`

- [ ] **Step 2: 重写 PlacoTeleopController**

  `xrobotoolkit_teleop/simulation/placo_teleop_controller.py`（完整替换）：

  ```python
  import time
  from typing import Any, Dict

  import meshcat.transformations as tf
  import numpy as np

  from xrobotoolkit_teleop.common.base_teleop_controller import BaseTeleopController
  from xrobotoolkit_teleop.simulation.joint_constraints import JointConstraints
  from xrobotoolkit_teleop.simulation.motion_filter import MotionFilter
  from xrobotoolkit_teleop.utils.geometry import (
      R_HEADSET_TO_WORLD,
  )


  class PlacoTeleopController(BaseTeleopController):
      """
      Placo teleoperation controller for a robot using inverse kinematics.

      Extends BaseTeleopController with:
      - MotionFilter (deadband + EMA) applied on delta before IK
      - JointConstraints (limit clipping + velocity limiting) after IK solve
      - SmoothReset: B button smooth interpolation back to home joints
      """

      def __init__(
          self,
          robot_urdf_path: str,
          manipulator_config: Dict[str, Dict[str, Any]],
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
          enable_velocity_limit: bool = False,
          max_joint_velocity: float = 3.0,
          # --- 平滑复位 ---
          reset_duration_s: float = 1.0,
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
          )

          # --- 平滑复位状态 ---
          self._resetting = False
          self._reset_step = 0
          self._reset_steps_total = int(reset_duration_s / max(self.dt, 1e-6))
          self._q_start: np.ndarray | None = None
          if self.q_init is not None:
              self._q_home = self.q_init.copy()
          else:
              self._q_home = np.zeros(len(self.placo_robot.joint_names()))
          self._prev_b_button = False

      # ---- 钩子重写 ----

      def _filter_delta(self, name: str, delta_xyz: np.ndarray, delta_rot: np.ndarray):
          """重写基类钩子：对 active 的机械臂做运动滤波"""
          if self.active.get(name, False):
              return self.motion_filter.apply(delta_xyz, delta_rot)
          return delta_xyz, delta_rot

      def _on_grip_state_change(self, name: str, active: bool) -> None:
          """重写基类钩子：grip 按下时重置滤波历史"""
          if active:
              self.motion_filter.reset()

      # ---- 仿真后端 ----

      def _robot_setup(self):
          pass

      def _send_command(self):
          self._update_placo_viz()

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

      def _apply_joint_constraints(self):
          """对 IK 求解后的关节值做限位 + 速度限制"""
          q = self.placo_robot.state.q[7:]  # 去掉浮动基座 7 个虚拟关节
          q_safe = self.joint_constraints.apply(q)
          self.placo_robot.state.q[7:] = q_safe
          self.placo_robot.update_kinematics()

      # ---- 平滑复位 ----

      def _start_smooth_reset(self):
          """开始平滑复位：记录当前 q，切换到复位模式"""
          self._resetting = True
          self._reset_step = 0
          self._q_start = self.placo_robot.state.q[7:].copy()
          print("[RESET] 平滑复位开始...")

      def _tick_smooth_reset(self):
          """执行一帧平滑复位插值。插值完成后恢复 IK。"""
          alpha = self._reset_step / self._reset_steps_total
          if alpha >= 1.0:
              self.placo_robot.state.q[7:] = self._q_home.copy()
              self._apply_joint_constraints()
              self.placo_robot.update_kinematics()
              self._resetting = False
              self._reset_step = 0
              print("[RESET] 复位完成")
              return

          q_cur = (1.0 - alpha) * self._q_start + alpha * self._q_home
          self.placo_robot.state.q[7:] = q_cur
          self._apply_joint_constraints()
          self.placo_robot.update_kinematics()
          self._reset_step += 1

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
                      self._update_ik()                # 基类：读 XR → 算 delta → 更新 task
                      self._solve_ik()                 # IK 求解
                      self._apply_joint_constraints()  # 限位 + 速度限制

                  self._send_command()                 # Meshcat 显示
                  end_time = time.time()
                  time.sleep(max(0, self.dt - (end_time - start_time)))
              except KeyboardInterrupt:
                  print("\nTeleoperation stopped.")
                  self._stop_event.set()
  ```

- [ ] **Step 3: 验证语法**

  Run: `python -c "import ast; ast.parse(open('xrobotoolkit_teleop/simulation/placo_teleop_controller.py').read()); print('Syntax OK')"`
  Expected: `Syntax OK`

- [ ] **Step 4: 验证导入**

  Run: `python -c "from xrobotoolkit_teleop.simulation.placo_teleop_controller import PlacoTeleopController; print('Import OK')" 2>&1`
  Expected: `Import OK`（可能报 Placo 未安装的错误，只要不是语法错误即可）

- [ ] **Step 5: Commit**

  ```bash
  git add -A
  git commit -m "feat: integrate MotionFilter, JointConstraints, SmoothReset into PlacoTeleopController"
  ```

---

### Task 5: 创建新入口脚本 teleop_rm65_sim.py

**文件：**
- Create: `scripts/simulation/teleop_rm65_sim.py`

- [ ] **Step 1: 创建新入口脚本**

  `scripts/simulation/teleop_rm65_sim.py`：

  ```python
  """RM65 simulation teleoperation with Placo IK + Meshcat visualization.

  Supports configurable motion filter (deadband + EMA), joint constraints
  (limit clipping + velocity limiting), and smooth reset (B button).
  """
  import os

  import tyro

  from xrobotoolkit_teleop.simulation.placo_teleop_controller import PlacoTeleopController
  from xrobotoolkit_teleop.utils.geometry import R_HEADSET_TO_WORLD_RM65_SIDE_MOUNT
  from xrobotoolkit_teleop.utils.path_utils import ASSET_PATH


  def main(
      # --- 原有关参数 ---
      robot_urdf_path: str = os.path.join(ASSET_PATH, "realman/RM65-official/urdf/RM65-official-arm.urdf"),
      scale_factor: float = 1.5,
      follow_orientation: bool = True,
      enable_log_data: bool = False,
      log_dir: str = "logs",
      # --- MotionFilter ---
      deadband_m: float = 0.002,
      deadband_rad: float = 0.03,
      smooth_alpha_pos: float = 0.35,
      smooth_alpha_rot: float = 0.3,
      # --- JointConstraints ---
      enable_velocity_limit: bool = False,
      max_joint_velocity: float = 3.0,
      # --- SmoothReset ---
      reset_duration_s: float = 1.0,
  ):
      control_mode = "pose" if follow_orientation else "position"
      config = {
          "right_hand": {
              "link_name": "r_link6",
              "pose_source": "right_controller",
              "control_trigger": "right_grip",
              "control_mode": control_mode,
          },
      }

      controller = PlacoTeleopController(
          robot_urdf_path=robot_urdf_path,
          manipulator_config=config,
          R_headset_world=R_HEADSET_TO_WORLD_RM65_SIDE_MOUNT,
          scale_factor=scale_factor,
          enable_log_data=enable_log_data,
          log_dir=log_dir,
          deadband_m=deadband_m,
          deadband_rad=deadband_rad,
          smooth_alpha_pos=smooth_alpha_pos,
          smooth_alpha_rot=smooth_alpha_rot,
          enable_velocity_limit=enable_velocity_limit,
          max_joint_velocity=max_joint_velocity,
          reset_duration_s=reset_duration_s,
      )

      joints_task = controller.solver.add_joints_task()
      joints_task.set_joints({joint: 0.0 for joint in controller.placo_robot.joint_names()})
      joints_task.configure("joints_regularization", "soft", 1e-4)

      print(f"[INFO] control_mode={control_mode}")
      print(f"[INFO] deadband_m={deadband_m}, deadband_rad={deadband_rad}")
      print(f"[INFO] smooth_alpha_pos={smooth_alpha_pos}, smooth_alpha_rot={smooth_alpha_rot}")
      print(f"[INFO] enable_velocity_limit={enable_velocity_limit}")
      print(f"[INFO] reset_duration_s={reset_duration_s}")
      print(f"Open Placo/Meshcat: http://localhost:{os.environ.get('MESHCAT_HOST_PORT', '18081')}/static/")
      controller.run()


  if __name__ == "__main__":
      tyro.cli(main)
  ```

- [ ] **Step 2: 验证语法**

  Run: `python -c "import ast; ast.parse(open('scripts/simulation/teleop_rm65_sim.py').read()); print('Syntax OK')"`
  Expected: `Syntax OK`

- [ ] **Step 3: 验证 --help 输出**

  Run: `python scripts/simulation/teleop_rm65_sim.py --help 2>&1 | head -30`
  Expected: 显示所有 12 个参数名和默认值

- [ ] **Step 4: Commit**

  ```bash
  git add -A
  git commit -m "feat: add teleop_rm65_sim.py entry script with full CLI params"
  ```

---

### Task 6: 清理旧入口 + 更新 Docker + 更新文档

**文件：**
- Delete: `scripts/simulation/teleop_dual_ur5e_placo.py`
- Modify: `docker/Dockerfile`
- Modify: `CLAUDE.md`

- [ ] **Step 1: 删除旧入口脚本**

  ```bash
  rm scripts/simulation/teleop_dual_ur5e_placo.py
  ```

  同时删除同目录下的 macOS 隐藏文件（如果存在）：
  ```bash
  rm -f scripts/simulation/._teleop_dual_ur5e_placo.py
  ```

- [ ] **Step 2: 更新 Dockerfile 中的 CMD**

  在 `docker/Dockerfile` 中，替换 CMD 中的旧脚本名：

  查找：`python scripts/simulation/teleop_dual_ur5e_placo.py`
  替换为：`python scripts/simulation/teleop_rm65_sim.py`

- [ ] **Step 3: 更新 CLAUDE.md**

  找到 CLAUDE.md 中 `scripts/simulation/teleop_dual_ur5e_placo.py` 的引用，替换为 `scripts/simulation/teleop_rm65_sim.py`。

  至少需要更新两处：
  - `## Commands` 区块中 `# 仿真运行（远程 Docker 内）` 的示例命令
  - `### 入口脚本` 表格中 `simulation/teleop_dual_ur5e_placo.py` 的用途说明

- [ ] **Step 4: 最终验证**

  Run: `python -c "
  from xrobotoolkit_teleop.simulation.motion_filter import MotionFilter
  from xrobotoolkit_teleop.simulation.joint_constraints import JointConstraints
  from xrobotoolkit_teleop.simulation.placo_teleop_controller import PlacoTeleopController
  print('All new modules import OK')
  " 2>&1`
  Expected: `All new modules import OK`（如 Placo 未安装，至少应显示 `ImportError` 而不是 `SyntaxError` 或 `ModuleNotFoundError`）

- [ ] **Step 5: 运行全部新增测试**

  Run: `python -m pytest tests/test_motion_filter.py tests/test_joint_constraints.py -v --no-header`
  Expected: 11 passed

- [ ] **Step 6: Commit**

  ```bash
  git add -A
  git commit -m "chore: clean up old entry, update Dockerfile and docs"
  ```

---

## 规范覆盖检查

| Spec 需求 | 对应 Task | 状态 |
|-----------|-----------|------|
| MotionFilter（死区+EMA） | Task 2 | ✅ |
| JointConstraints（限位裁剪） | Task 3 | ✅ |
| JointConstraints（速度限制） | Task 3 | ✅ |
| 位移死区参数 `deadband_m` | Task 2, 5 | ✅ |
| 旋转死区参数 `deadband_rad` | Task 2, 5 | ✅ |
| EMA 平滑系数 `smooth_alpha_pos/rot` | Task 2, 5 | ✅ |
| 速度限制开关 `enable_velocity_limit` | Task 3, 5 | ✅ |
| 删除无效 DataLogger 导入 | Task 1 | ✅ |
| 添加 `_filter_delta` 钩子 | Task 1 | ✅ |
| 添加 `_on_grip_state_change` 钩子 | Task 1, 4 | ✅ |
| 移 solver.solve() 到子类 | Task 1, 4 | ✅ |
| SmoothReset（B 按钮） | Task 4 | ✅ |
| 复位时长参数 `reset_duration_s` | Task 4, 5 | ✅ |
| 入口重命名 `teleop_rm65_sim.py` | Task 5, 6 | ✅ |
| Docker CMD 更新 | Task 6 | ✅ |
| 单元测试覆盖 | Task 2, 3 | ✅ |
