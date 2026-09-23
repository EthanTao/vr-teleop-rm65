# 仿真环境优化：操作手感 + 关节物理约束

**日期**: 2026-07-16
**状态**: ✅ 已实现（MotionFilter / JointConstraints / SmoothReset 已落地）
**整理于**: 2026-08-27
**范围**: 仅仿真路径（`xrobotoolkit_teleop/simulation/`），不动硬件路径和数据采集

---

## 1. 背景与动机

当前仿真环境（`PlacoTeleopController` + `BaseTeleopController`）存在以下问题：

| # | 问题 | 影响 |
|---|------|------|
| 1 | **无信号滤波**：XR 手柄抖动（人手微颤 + 传感器噪声）直接传递到末端位姿 | 虚拟机械臂不停"抽搐"，操作手感差 |
| 2 | **无物理约束**：Placo IK 求解结果不做限位裁剪，可能超出关节物理范围 | 仿真结果与真机不匹配，无法预判真机可行性 |
| 3 | **`data_logger` 导入崩溃**：`BaseTeleopController` 导入了不存在的模块 | 无法在仿真中开启数据记录 |
| 4 | **入口文件名误导**：`teleop_dual_ur5e_placo.py` 实际是 RM65 | 新用户困惑 |

**目标**：在不触及硬件路径和数据采集的前提下，提升仿真操作手感和物理合理性。

---

## 2. 架构改动

### 改动前

```
XR 数据 ─→ BaseTeleopController._update_ik() ─→ Placo IK 求解 ─→ Meshcat
              (无死区 无平滑 无关节限位)
```

### 改动后

```
XR 数据 ─→ MotionFilter ─→ Placo IK ─→ JointConstraints ─→ Meshcat
           死区+EMA平滑              关节限位+速度限制
```

两个新模块放在 `xrobotoolkit_teleop/simulation/` 下，只被 `PlacoTeleopController` 调用，不影响 `BaseTeleopController` 及其硬件子类。

---

## 3. 新增模块

### 3.1 MotionFilter（死区 + EMA 平滑）

**文件**：`xrobotoolkit_teleop/simulation/motion_filter.py`

**接口**：

```python
class MotionFilter:
    def __init__(
        self,
        deadband_m: float = 0.002,        # 平移死区 (m)
        deadband_rad: float = 0.03,       # 旋转死区 (rad)
        smooth_alpha_pos: float = 0.35,   # 平移 EMA 系数
        smooth_alpha_rot: float = 0.3,    # 旋转 EMA 系数
    )

    def reset(self) -> None:
        """重置内部 EMA 历史，grip 按下时调用"""

    def apply(
        self,
        delta_xyz: np.ndarray,   # (3,)
        delta_rot: np.ndarray,   # (3,)
    ) -> tuple[np.ndarray, np.ndarray]:
        """死区过滤 → EMA 平滑 → 返回 (filtered_xyz, filtered_rot)"""
```

**死区过滤**：每个分量独立判断，`abs(v) < deadband` 则归零。

**EMA 平滑**：`y_t = α · x_t + (1-α) · y_{t-1}`，`α` 越小越平滑但响应越慢。平移和旋转使用不同的 `α`（旋转惯性更大，防止姿态抖动）。

**生命周期**：
```
grip 按下 → reset()（清空 EMA 历史、prev 设为当前 delta）
grip 保持 → apply() 每帧
grip 松开 → 不做任何事（冻结上次状态）
```

### 3.2 JointConstraints（关节限位 + 速度限制）

**文件**：`xrobotoolkit_teleop/simulation/joint_constraints.py`

**接口**：

```python
class JointConstraints:
    def __init__(
        self,
        placo_robot: placo.RobotWrapper,
        dt: float,
        enable_velocity_limit: bool = False,
        max_joint_velocity: float = 3.0,   # rad/s
    ):
        # 从 URDF <limit lower="..." upper="..."/> 自动读取关节限位
        self.lower_limits: np.ndarray
        self.upper_limits: np.ndarray
        self.prev_q: np.ndarray | None = None

    def apply(self, q: np.ndarray) -> np.ndarray:
        """关节限位 clip → 速度限制 clip → 返回安全 q"""
```

**关节限位**：`np.clip(q, self.lower_limits, self.upper_limits)`

**速度限制**：`delta_q = q - prev_q`，按分量限制 `|delta_q| ≤ max_vel * dt`。开关默认关闭（`enable_velocity_limit=False`），开启后防止 IK 两帧之间跳变过大。

**注意**：速度限制依赖帧间状态（`prev_q`），因此 `JointConstraints` 实例与仿真循环生命周期绑定。

---

### 3.3 SmoothReset（平滑复位）

B 按钮按下时，将机械臂从当前关节位置平滑插值到初始位置。复位过程中 IK 暂停，复位完成后恢复正常操作。

**设计要点：**

| 问题 | 方案 |
| ---- | ---- |
| 触发方式 | 手柄 B 按钮（`xr_client.get_button_state_by_name("B")`） |
| 复位目标 | `q_home` — 显式传入的 `q_init`；未配置时禁用 B 键复位 |
| 插值方式 | 线性插值（`np.linspace`），每帧推进一步 |
| 复位时长 | 可配置参数 `reset_duration_s`，默认 3.0 秒 |
| 复位中 IK 状态 | 暂停 IK 求解，joint targets 由插值器直接写入 `placo_robot.state.q` |
| 复位中约束 | JointConstraints 仍然生效（限位 + 速度限制），防止复位路径不安全 |
| 复位中用户控制 | Grip/手柄输入被忽略，复位完成后自动恢复 |
| 连续触发 | 复位中 B 按钮再次按下无效（正在复位），完成后才响应下一次 |

**数据流（复位模式）：**

```text
B 按钮按下
  ↓
记录当前 q_start = placo_robot.state.q[7:]
计算总步数 N = reset_duration_s / dt
初始化步进计数器 step = 0
设置 _resetting = True
  ↓
每帧: alpha = step / N
      q_current = (1-alpha) * q_start + alpha * q_home
      placo_robot.state.q[7:] = q_current
      JointConstraints.apply(q_current)
      placo_robot.update_kinematics()
      _send_command()  →  Meshcat 显示
      step += 1
      if step >= N: _resetting = False
```

---

## 4. PlacoTeleopController 改动

**文件**：`xrobotoolkit_teleop/simulation/placo_teleop_controller.py`

`_update_ik()` 从基类继承不变，`run()` 循环拆分为清晰的管线：

```python
class PlacoTeleopController(BaseTeleopController):
    def __init__(
        self,
        robot_urdf_path: str,
        manipulator_config: dict,
        floating_base: bool = False,
        R_headset_world: np.ndarray = R_HEADSET_TO_WORLD,
        scale_factor: float = 1.0,
        q_init: np.ndarray | None = None,
        dt: float = 0.01,
        enable_log_data: bool = False,
        log_dir: str = "logs",
        # --- 仿真新增 ---
        deadband_m: float = 0.002,
        deadband_rad: float = 0.03,
        smooth_alpha_pos: float = 0.35,
        smooth_alpha_rot: float = 0.3,
        enable_velocity_limit: bool = False,
        max_joint_velocity: float = 3.0,
        reset_duration_s: float = 3.0,
    ):
        super().__init__(...)
        self._init_placo_viz()
        self.motion_filter = MotionFilter(
            deadband_m=deadband_m,
            deadband_rad=deadband_rad,
            smooth_alpha_pos=smooth_alpha_pos,
            smooth_alpha_rot=smooth_alpha_rot,
        )
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

    def run(self):
        while not self._stop_event.is_set():
            start_time = time.time()

            # --- 检测 B 按钮上升沿触发复位 ---
            b_button = self.xr_client.get_button_state_by_name("B")
            if b_button and not self._prev_b_button and not self._resetting:
                self._start_smooth_reset()
            self._prev_b_button = b_button

            if self._resetting:
                self._tick_smooth_reset()
            else:
                self._update_ik()                # 基类：读 XR → 算 delta → 更新 task 目标
                self._apply_motion_filter()      # 新增：死区 + EMA 平滑
                self._solve_ik()                 # 从 _update_ik 中移出
                self._apply_joint_constraints()  # 新增：限位 + 速度

            self._send_command()                 # 基类：Meshcat 显示
            elapsed = time.time() - start_time
            time.sleep(max(0, self.dt - elapsed))

    def _start_smooth_reset(self):
        self._resetting = True
        self._reset_step = 0
        self._q_start = self.placo_robot.state.q[7:].copy()
        print("[RESET] 平滑复位开始...")

    def _tick_smooth_reset(self):
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

    def _apply_motion_filter(self):
        for name in self.manipulator_config:
            if self.active.get(name):
                filtered_xyz, filtered_rot = self.motion_filter.apply(
                    self._current_delta_xyz[name],
                    self._current_delta_rot[name],
                )
                # 用过滤后的 delta 更新 task 目标
                ...

    def _solve_ik(self):
        try:
            self.solver.solve(True)
        except RuntimeError as e:
            print(f"IK solver failed: {e}")

    def _apply_joint_constraints(self):
        q = self.placo_robot.state.q[7:]  # 去掉浮动基座
        q_safe = self.joint_constraints.apply(q)
        self.placo_robot.state.q[7:] = q_safe
        self.placo_robot.update_kinematics()
```

**MotionFilter 生命周期绑定 Grip**：`_update_ik()` 中 grip 按下时调用 `self.motion_filter.reset()`，松开时不处理。需要在 `BaseTeleopController._update_ik()` 的 grip 激活/释放逻辑处暴露一个钩子。

---

## 5. 数据流细节：delta 如何穿过 filter

当前 `_update_ik()` 在基类里是一个闭环：读 XR → 算 delta → 设置 task 目标 → 调 `solver.solve()`。要插入 MotionFilter，需要把"算 delta"和"设置 task 目标"拆开。

**方案**：基类新增一个可重写的 `_filter_delta` 钩子，默认透传（不影响硬件子类），仿真子类在此调用 `motion_filter.apply()`。

```python
# base_teleop_controller.py — _update_ik() 中 for 循环内：

delta_xyz, delta_rot = self._process_xr_pose(xr_pose, src_name)

# 钩子：子类可在此做滤波（基类默认透传）
delta_xyz, delta_rot = self._filter_delta(src_name, delta_xyz, delta_rot)

# 然后用（可能已过滤的）delta 更新 task 目标
target_xyz, target_quat = apply_delta_pose(...)
```

基类默认实现：
```python
def _filter_delta(self, name: str, delta_xyz: np.ndarray, delta_rot: np.ndarray):
    """钩子：子类可在此对 delta 做滤波处理。默认透传。"""
    return delta_xyz, delta_rot
```

`PlacoTeleopController` 重写：
```python
def _filter_delta(self, name, delta_xyz, delta_rot):
    return self.motion_filter.apply(delta_xyz, delta_rot)
```

**同时**，`_update_ik()` 中的 `solver.solve(True)` 移到 `PlacoTeleopController.run()` 中作为独立的 `_solve_ik()` 步骤，让 JointConstraints 在 solve 之后插入。

---

## 6. BaseTeleopController 改动

**文件**：`xrobotoolkit_teleop/common/base_teleop_controller.py`

**三处最小改动：**

1. **删除无效导入**（第 19 行）：`from xrobotoolkit_teleop.common.data_logger import DataLogger`
2. **删除 `DataLogger` 构造**（第 61 行）：`self.data_logger = DataLogger(log_dir=log_dir)`
3. **新增 `_filter_delta` 钩子**（默认透传，供子类重写）
4. **新增 `_on_grip_state_change` 钩子**（grip 激活/释放时调用，供子类调用 `motion_filter.reset()`）
5. **`_update_ik()` 移除 `solver.solve()`**——solve 调用移至子类 `run()` 中（原代码在 `_update_ik()` 末尾调用 `self.solver.solve(True)`）

---

## 7. 入口脚本

**文件**：`scripts/simulation/teleop_dual_ur5e_placo.py` → **重命名**为 `scripts/simulation/teleop_rm65_sim.py`

**CLI 参数**（tyro，与函数签名一一对应）：

```python
def main(
    # --- 原有 ---
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
    reset_duration_s: float = 3.0,
):
    ...
```

---

## 8. Docker 更新

**文件**：`docker/Dockerfile`

将 CMD 中的旧脚本名替换为 `teleop_rm65_sim.py`。

---

## 9. 改动文件清单

| 文件 | 操作 | 风险 |
|------|------|------|
| `xrobotoolkit_teleop/simulation/motion_filter.py` | **新建** | 低 |
| `xrobotoolkit_teleop/simulation/joint_constraints.py` | **新建** | 低 |
| `xrobotoolkit_teleop/simulation/placo_teleop_controller.py` | **修改** | 中（核心仿真逻辑） |
| `xrobotoolkit_teleop/common/base_teleop_controller.py` | **修改**（3 处小改动） | 低 |
| `scripts/simulation/teleop_rm65_sim.py` | **新建** | 低 |
| `scripts/simulation/teleop_dual_ur5e_placo.py` | **删除** | 低 |
| `docker/Dockerfile` | **修改**（CMD 文件名） | 低 |

---

## 10. 不变的范围

- `xrobotoolkit_teleop/common/data_collector.py` — 不动
- `xrobotoolkit_teleop/hardware/` — 不动
- `xrobotoolkit_teleop/utils/` — 不动（geometry、meshcat_utils、path_utils 均不变）
- `config/` — 不动
- `tests/` — 不动

---

## 11. 测试计划

| 被测模块 | 测试方式 | 验证点 |
| -------- | -------- | ------ |
| `MotionFilter` | 单元测试（纯函数，无外部依赖） | 死区归零、EMA 收敛、reset 清空、多分量独立 |
| `JointConstraints` | 单元测试（需 mock placo_robot） | 限位裁剪、速度限制裁剪、边界值 |
| `PlacoTeleopController` | 集成测试（需 XR mock + Placo） | 整条管线跑通、grip 触发 reset、约束生效、平滑复位 |
| 入口脚本 | 手动测试 | `--help` 参数正确、远程 Docker 仿真正常运行 |

**单元测试文件**：

- `tests/test_motion_filter.py`
- `tests/test_joint_constraints.py`

---

## 12. 兼容性

- 默认参数值保持原有行为一致（死区 0.002m、平滑 α=0.35 与硬件默认接近）
- 仿真仍可通过 CLI 覆盖所有新参数
- 硬件路径（`RealmanRM65CartesianTeleopController`）不受影响——它的滤波逻辑在 `xrobotoolkit_teleop/hardware/` 中独立实现
