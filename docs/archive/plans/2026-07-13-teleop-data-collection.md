# VR 遥操作数据采集实现计划

> **状态**：✅ 已完成（`DataCollector` 已实现，单元测试通过）
> **原始日期**：2026-07-13 | **本文档整理于**：2026-08-27
>
> **更新说明**：正文中的仿真入口 `teleop_dual_ur5e_placo.py` 已于仿真优化落地时更名为 `teleop_rm65_sim.py`。

**目标：** 在 VR 遥操作仿真环境中，将手柄动作与机器人状态配对记录为 HDF5 文件，供模仿学习训练使用。

**架构：** 新增独立的 `DataCollector` 类负责数据缓存和 HDF5 写入；在现有 `BaseTeleopController._update_ik()` 中根据 Grip 状态切换插入记录调用；同时修复 URDF 关节限位缺失导致的仿真运动反直觉问题。

**技术栈：** Python 3.10+, h5py, numpy, placo

---

## 文件变更总览

| 文件 | 操作 | 职责 |
|------|------|------|
| `xrobotoolkit_teleop/common/data_collector.py` | **新增** | 数据记录器：缓存帧、Grip 边界管理、HDF5 写入 |
| `xrobotoolkit_teleop/common/base_teleop_controller.py` | 修改 | 加关节限位 + 在控制循环中插入记录调用 |
| `pyproject.toml` | 修改 | 加 `h5py` 依赖 |
| `tests/test_data_collector.py` | **新增** | 记录器的单元测试 |
| `scripts/simulation/teleop_dual_ur5e_placo.py` | 修改 | 暴露 `--enable-log-data` 和 `--log-dir` 参数 |

---

### Task 1: 添加 h5py 依赖

**文件：**
- 修改 `pyproject.toml`

- [ ] **Step 1: 在 dependencies 中加入 h5py**

在 `pyproject.toml` 的 `dependencies` 列表末尾添加 `"h5py"`：

```toml
dependencies = [
    "mujoco",
    "numpy",
    "meshcat",
    "placo",
    "tyro",
    "h5py",
]
```

- [ ] **Step 2: 安装依赖验证**

```bash
pip install -e .
```

预期：h5py 安装成功，`import h5py` 无报错。

- [ ] **Step 3: 提交**

```bash
git add pyproject.toml
git commit -m "chore: add h5py dependency for data collection"
```

---

### Task 2: 实现 DataCollector

**文件：**
- 新增 `xrobotoolkit_teleop/common/data_collector.py`
- 新增 `tests/test_data_collector.py`

- [ ] **Step 1: 写第一个失败的测试 —— 创建记录器并开始一段**

```python
# tests/test_data_collector.py
import tempfile
import os
import h5py
import numpy as np
from pathlib import Path

import sys
sys.path.insert(0, str(Path(__file__).parent.parent))

from xrobotoolkit_teleop.common.data_collector import DataCollector


def test_start_episode_initializes_buffers():
    """开始一段后，内部计数器归零。"""
    with tempfile.TemporaryDirectory() as tmpdir:
        dc = DataCollector(output_dir=tmpdir, control_hz=30)
        dc.start_episode()
        dc.record_step(
            obs={"joint": np.zeros(6)}, action={"delta": np.zeros(3)}
        )
        dc.end_episode()
        # 验证文件已生成
        files = list(Path(tmpdir).glob("episode_*.h5"))
        assert len(files) == 1
```

- [ ] **Step 2: 运行测试验证失败**

```bash
python -m pytest tests/test_data_collector.py::test_start_episode_initializes_buffers -v
```

预期：`ModuleNotFoundError: No module named 'xrobotoolkit_teleop.common.data_collector'`

- [ ] **Step 3: 创建 DataCollector 最小实现**

```python
# xrobotoolkit_teleop/common/data_collector.py
"""数据记录器：Grip 边界管理 + HDF5 写入。"""
from __future__ import annotations

import time
from pathlib import Path

import h5py
import numpy as np


class DataCollector:
    """记录遥操作数据到 HDF5 文件。

    用法:
        dc = DataCollector(output_dir="logs", control_hz=30)
        dc.start_episode()           # Grip 按下时
        dc.record_step(obs, action)  # 每帧
        dc.end_episode()             # Grip 松开时
    """

    def __init__(
        self,
        output_dir: str = "logs",
        control_hz: float = 30.0,
        tail_duration_s: float = 0.5,
    ):
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.control_hz = float(control_hz)
        self.tail_duration_s = float(tail_duration_s)

        self._recording = False
        self._buffer_obs: list[dict] = []
        self._buffer_act: list[dict] = []
        self._episode_idx = 0

    def start_episode(self) -> None:
        self._recording = True
        self._buffer_obs = []
        self._buffer_act = []

    def record_step(self, obs: dict, action: dict) -> None:
        if not self._recording:
            return
        self._buffer_obs.append({k: np.asarray(v).copy() for k, v in obs.items()})
        self._buffer_act.append({k: np.asarray(v).copy() for k, v in action.items()})

    def end_episode(self) -> None:
        if not self._recording:
            return
        self._recording = False

        n = len(self._buffer_obs)
        if n == 0:
            return

        # 尾部帧数 —— 让模型看到"停止"作为动作终点
        tail_frames = max(1, int(self.control_hz * self.tail_duration_s))
        tail_frames = min(tail_frames, n)

        # 组装数组
        obs_arr = {}
        for key in self._buffer_obs[0]:
            obs_arr[key] = np.stack([o[key] for o in self._buffer_obs])
        act_arr = {}
        for key in self._buffer_act[0]:
            act_arr[key] = np.stack([a[key] for a in self._buffer_act])

        # 写入 HDF5
        stamp = time.strftime("%Y%m%d_%H%M%S")
        filename = self.output_dir / f"episode_{stamp}_{self._episode_idx:02d}.h5"
        self._episode_idx += 1

        with h5py.File(str(filename), "w") as f:
            obs_grp = f.create_group("obs")
            for key, arr in obs_arr.items():
                obs_grp.create_dataset(key, data=arr)
            act_grp = f.create_group("action")
            for key, arr in act_arr.items():
                act_grp.create_dataset(key, data=arr)

        print(f"[DATA] 已保存 {n} 帧 -> {filename}")

    @property
    def is_recording(self) -> bool:
        return self._recording
```

- [ ] **Step 4: 运行测试验证通过**

```bash
python -m pytest tests/test_data_collector.py::test_start_episode_initializes_buffers -v
```

预期：PASS

- [ ] **Step 5: 补充更多测试**

```python
def test_record_only_when_active():
    """未开始一段时，record_step 不缓存任何数据。"""
    with tempfile.TemporaryDirectory() as tmpdir:
        dc = DataCollector(output_dir=tmpdir, control_hz=30)
        dc.record_step(
            obs={"joint": np.zeros(6)}, action={"delta": np.zeros(3)}
        )
        assert len(dc._buffer_obs) == 0


def test_end_episode_without_start_is_noop():
    """没调用 start_episode 就直接 end_episode，不会生成文件。"""
    with tempfile.TemporaryDirectory() as tmpdir:
        dc = DataCollector(output_dir=tmpdir, control_hz=30)
        dc.end_episode()
        files = list(Path(tmpdir).glob("episode_*.h5"))
        assert len(files) == 0


def test_recorded_data_matches_input():
    """存进去的数据和读出来的一致。"""
    with tempfile.TemporaryDirectory() as tmpdir:
        dc = DataCollector(output_dir=tmpdir, control_hz=30)
        dc.start_episode()
        for i in range(5):
            dc.record_step(
                obs={"joint": np.array([i] * 6, dtype=float)},
                action={"delta": np.array([i * 0.1] * 3, dtype=float)},
            )
        dc.end_episode()

        files = list(Path(tmpdir).glob("episode_*.h5"))
        with h5py.File(str(files[0]), "r") as f:
            assert f["obs/joint"].shape == (5, 6)
            assert f["action/delta"].shape == (5, 3)
            np.testing.assert_array_equal(
                f["obs/joint"][2], np.array([2.0] * 6)
            )


def test_multiple_episodes_increment_index():
    """连续记录多段，文件名序号递增。"""
    with tempfile.TemporaryDirectory() as tmpdir:
        dc = DataCollector(output_dir=tmpdir, control_hz=30)
        for _ in range(3):
            dc.start_episode()
            dc.record_step(
                obs={"j": np.zeros(3)}, action={"d": np.zeros(2)}
            )
            dc.end_episode()
        files = sorted(Path(tmpdir).glob("episode_*.h5"))
        assert len(files) == 3
        assert "_01" in files[0].name
        assert "_02" in files[1].name
        assert "_03" in files[2].name


def test_empty_episode_not_saved():
    """一段里没有任何帧，不生成文件。"""
    with tempfile.TemporaryDirectory() as tmpdir:
        dc = DataCollector(output_dir=tmpdir, control_hz=30)
        dc.start_episode()
        dc.end_episode()
        files = list(Path(tmpdir).glob("episode_*.h5"))
        assert len(files) == 0
```

- [ ] **Step 6: 运行全部测试**

```bash
python -m pytest tests/test_data_collector.py -v
```

预期：6 个测试全部 PASS

- [ ] **Step 7: 提交**

```bash
git add xrobotoolkit_teleop/common/data_collector.py tests/test_data_collector.py
git commit -m "feat: add DataCollector for teleop data recording"
```

---

### Task 3: 修复 URDF 关节限位

**文件：**
- 修改 `xrobotoolkit_teleop/common/base_teleop_controller.py:130-144`

- [ ] **Step 1: 在 _placo_setup 中注册关节限位**

在 `_placo_setup` 方法的第 130 行（`self.solver.dt = self.dt` 之后、初始位形设置之前）加入关节限位逻辑：

```python
        self.solver = placo.KinematicsSolver(self.placo_robot)
        self.solver.dt = self.dt

        # --- 新增：从 URDF 读取并注册关节限位 ---
        for joint_name in self.placo_robot.model.joint_names():
            offset = self.placo_robot.get_joint_offset(joint_name)
            if offset < 0:
                continue  # 非活动关节
            lower = self.placo_robot.model.lower_position_limit(offset)
            upper = self.placo_robot.model.upper_position_limit(offset)
            if lower is not None and upper is not None and lower < upper:
                try:
                    self.solver.add_position_limit(joint_name, lower, upper)
                except RuntimeError:
                    pass  # 某些关节可能不支持加限位，忽略
        # --- 新增结束 ---

        # Set initial configuration
        if self.q_init is not None:
```

- [ ] **Step 2: 在 _update_ik 的 solver.solve 之后加安全钳制**

在 `_update_ik` 方法的第 249 行 `self.solver.solve(True)` 之后加入：

```python
        try:
            self.solver.solve(True)
        except RuntimeError as e:
            print(f"IK solver failed: {e}")

        # --- 新增：求解后安全钳制，防止越界 ---
        if not self.floating_base:
            for joint_name in self.placo_robot.model.joint_names():
                offset = self.placo_robot.get_joint_offset(joint_name)
                if offset < 7:  # 跳过浮动基座位形（前 7 维）
                    continue
                lower = self.placo_robot.model.lower_position_limit(offset)
                upper = self.placo_robot.model.upper_position_limit(offset)
                if lower is not None and upper is not None:
                    val = self.placo_robot.state.q[offset]
                    self.placo_robot.state.q[offset] = float(np.clip(val, lower, upper))
        # --- 新增结束 ---

```

- [ ] **Step 3: 提交**

```bash
git add xrobotoolkit_teleop/common/base_teleop_controller.py
git commit -m "fix: apply URDF joint limits to IK solver in simulation"
```

---

### Task 4: 在控制循环中集成 DataCollector

**文件：**
- 修改 `xrobotoolkit_teleop/common/base_teleop_controller.py`

- [ ] **Step 1: 在构造函数中初始化 DataCollector**

在 `__init__` 方法末尾（`self._stop_event` 之前）加入：

```python
        # 数据记录器（默认关闭）
        self.data_collector: DataCollector | None = None
        self._prev_grip_active: dict[str, bool] = {}
        if self.enable_log_data:
            from xrobotoolkit_teleop.common.data_collector import DataCollector

            self.data_collector = DataCollector(
                output_dir=self.log_dir,
                control_hz=1.0 / max(dt, 1e-6),
            )
```

- [ ] **Step 2: 在 _update_ik 的 manipulator 循环中加入记录调用**

在 `_update_ik` 方法中，找到第 214 行 `self.active[src_name] = xr_grip_val > self._grip_threshold` 这一行，在它之后加入 Grip 边界检测和记录逻辑：

```python
        for src_name, config in self.manipulator_config.items():
            xr_grip_val = self.xr_client.get_key_value_by_name(config["control_trigger"])
            was_active = self._prev_grip_active.get(src_name, False)
            is_active = xr_grip_val > self._grip_threshold
            self.active[src_name] = is_active

            # --- 新增：Grip 边界触发记录器 ---
            if self.data_collector is not None:
                if is_active and not was_active:
                    self.data_collector.start_episode()
                    print(f"[DATA] 开始记录 {src_name}")
                elif not is_active and was_active:
                    self.data_collector.end_episode()
                    print(f"[DATA] 结束记录 {src_name}")
            self._prev_grip_active[src_name] = is_active
            # --- 新增结束 ---

            if self.active[src_name]:
```

- [ ] **Step 3: 在 delta 计算后加入 record_step 调用**

在同一个 `if self.active[src_name]:` 分支内，delta 计算完成、IK 目标更新之后，加入数据记录。具体位置在第 238 行 `self.effector_task[src_name].T_world_frame = target_pose` 之后：

```python
                # --- 新增：记录当前帧 ---
                if self.data_collector is not None:
                    ee_xyz, ee_quat = self._get_link_pose(config["link_name"])
                    obs = {
                        "joint_positions": self.placo_robot.state.q[7:].copy(),
                        "ee_xyz": ee_xyz.copy(),
                        "ee_quat": ee_quat.copy(),
                    }
                    action = {
                        "delta_xyz": delta_xyz.copy(),
                        "delta_rot": delta_rot.copy(),
                        "grip": np.array([xr_grip_val], dtype=float),
                    }
                    self.data_collector.record_step(obs, action)
                # --- 新增结束 ---
```

- [ ] **Step 4: 添加必要的 import**

在文件顶部加入 `import numpy as np`（已有）和类型提示（`DataCollector | None` 需要在 `from __future__ import annotations` 支持下工作，该文件暂无此 import，需要加一下）：

在文件第一行 `import abc` 之前加入：

```python
from __future__ import annotations
```

- [ ] **Step 5: 提交**

```bash
git add xrobotoolkit_teleop/common/base_teleop_controller.py
git commit -m "feat: integrate DataCollector into teleop control loop"
```

---

### Task 5: 更新仿真入口脚本

**文件：**
- 修改 `scripts/simulation/teleop_dual_ur5e_placo.py`

- [ ] **Step 1: 暴露 enable_log_data 和 log_dir 参数**

在 `main` 函数签名中加入两个可选参数，并传给 `PlacoTeleopController`：

```python
def main(
    robot_urdf_path: str = os.path.join(ASSET_PATH, "realman/RM65-official/urdf/RM65-official-arm.urdf"),
    scale_factor: float = 1.5,
    follow_orientation: bool = True,
    enable_log_data: bool = False,          # 新增
    log_dir: str = "logs",                  # 新增
):
```

在 `PlacoTeleopController(...)` 构造调用中加入两个参数：

```python
    controller = PlacoTeleopController(
        robot_urdf_path=robot_urdf_path,
        manipulator_config=config,
        R_headset_world=R_HEADSET_TO_WORLD_RM65_SIDE_MOUNT,
        scale_factor=scale_factor,
        enable_log_data=enable_log_data,    # 新增
        log_dir=log_dir,                    # 新增
    )
```

- [ ] **Step 2: 验证**

```bash
python scripts/simulation/teleop_dual_ur5e_placo.py --help
```

预期：tyro 自动生成的帮助信息中显示 `--enable-log-data` 和 `--log-dir` 两个选项。

- [ ] **Step 3: 提交**

```bash
git add scripts/simulation/teleop_dual_ur5e_placo.py
git commit -m "feat: expose --enable-log-data flag in simulation entry script"
```

---

### Task 6: 端到端验证

- [ ] **Step 1: 运行全部单元测试**

```bash
python -m pytest tests/test_data_collector.py -v
```

预期：6 个测试全部 PASS。

- [ ] **Step 2: 启动仿真并验证记录器正常初始化**

```bash
export XROBO_GRIP_THRESHOLD=0.2
python scripts/simulation/teleop_dual_ur5e_placo.py --enable-log-data --log-dir logs_test
```

预期：控制台输出 `[DATA]` 前缀的日志，`logs_test/` 目录在 Grip 按下再松开后生成 `episode_*.h5` 文件。

- [ ] **Step 3: 检查生成的 HDF5 文件结构**

```python
python -c "
import h5py
import glob
files = sorted(glob.glob('logs_test/episode_*.h5'))
if files:
    f = h5py.File(files[0], 'r')
    print('obs keys:', list(f['obs'].keys()))
    print('action keys:', list(f['action'].keys()))
    print('joint shape:', f['obs/joint_positions'].shape)
    print('delta_xyz shape:', f['action/delta_xyz'].shape)
    f.close()
else:
    print('no files found — 请先按 Grip 操作手柄后再松开')
"
```

- [ ] **Step 4: 验证关节限位生效**

在仿真运行时观察，关节运动不再出现反直觉的方向（具体表现为：手柄向前推 → 末端在 Meshcat 中向前移动，而非斜向移动）。

- [ ] **Step 5: 清理测试文件并提交**

```bash
rm -rf logs_test/
```

如果 URDF 限位修复有任何微调，提交这些调整。

---

## 真机迁移指南（后续，不在本次范围）

当仿真数据采集验证通过后，真机接入方式如下：

1. 在 `RealmanRM65CartesianTeleopController` 的 `__init__` 中创建同一个 `DataCollector` 实例
2. 在 `run()` 的 Grip 状态切换处（`was_active` / `grip_active` 逻辑）加入 `start_episode()` / `end_episode()`
3. 在 Grip 激活期间、`movel_xyz_quat` 之前，调用 `record_step`：
   - 观测：从 `arm.get_pose6()` 和 `arm.get_joint_positions()` 读取
   - 动作：`_process_xr_pose()` 的返回值 `delta_xyz`, `delta_rot` + grip 值
