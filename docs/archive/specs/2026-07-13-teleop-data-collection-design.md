# VR 遥操作数据采集设计规范

> **状态**：✅ 已实现（`DataCollector` + HDF5 落地，单元测试通过）
> **原始日期**：2026-07-13 | **本文档整理于**：2026-08-27
>
> **更新说明**：正文中的仿真入口 `teleop_dual_ur5e_placo.py` 已于仿真优化落地时更名为 `teleop_rm65_sim.py`（见 [2026-07-16-sim-optimization-design.md](2026-07-16-sim-optimization-design.md)）。

## 1. 目标

在 VR 遥操作仿真环境中，采集「VR 手柄动作 ↔ 机器人状态」的配对数据，用于模仿学习训练。后续迁移到真机环境。

## 2. 前置条件：仿真关节限位修复

**问题**：当前 Placo IK 未从 URDF 读取关节限位，导致仿真中运动反直觉。

**修复**：在 `BaseTeleopController._placo_setup()` 中：

1. 遍历每个关节，从 URDF 模型读取 `lower` / `upper` 限位值
2. 将限位注册为 IK 求解器的硬约束
3. 在求解后对关节角度做一次安全钳制（防止边界处越界）

影响文件：`xrobotoolkit_teleop/common/base_teleop_controller.py`，`_placo_setup` 方法。

## 3. 整体方案：独立记录器 + 控制器内埋点

新增 `xrobotoolkit_teleop/common/data_collector.py`，提供统一的数据记录接口。在仿真控制器的控制循环中插入记录调用。真机迁移时复用同一套接口。

### 3.1 核心接口

```
记录器 = DataCollector(输出目录, 采样率)

记录器.开始一段()          # Grip 刚按下
记录器.存一帧(观测, 动作)  # Grip 按住期间，每帧调用
记录器.结束一段(尾部帧数)  # Grip 松开，补尾部帧后保存
```

### 3.2 一段数据的边界（与 Grip 联动）

| 阶段 | Grip 状态 | 记录器行为 |
|------|-----------|------------|
| 开始 | 上一帧未激活 → 当前帧激活 | 调用 `开始一段()`，创建新的 HDF5 文件 |
| 记录中 | 持续激活 | 每帧调用 `存一帧()`，追加数据 |
| 结束 | 激活 → 未激活 | 调用 `结束一段()`，将缓存中最后约 0.5 秒的帧作为尾巴一并写入文件 |

**尾部帧的作用**：让模型看到"停止"作为动作终点，避免数据在 Grip 松开瞬间被截断。尾部窗口长度默认为记录器采样率 × 0.5 秒（如 30Hz 时为 15 帧），可通过构造参数调整。

### 3.3 开关控制

通过 `BaseTeleopController` 已有的 `enable_log_data` 和 `log_dir` 参数控制。默认关闭。

```python
controller = PlacoTeleopController(
    ...,
    enable_log_data=True,
    log_dir="logs",
)
```

## 4. 数据格式

### 4.1 每帧记录内容

**观测（机器人状态）**：

| 字段 | 说明 | 维度 |
|------|------|------|
| 关节角度 | 六个关节的弧度值 | 6 |
| 末端位置 | 世界坐标系 XYZ，米 | 3 |
| 末端姿态 | 四元数 w,x,y,z | 4 |

**动作（手柄输入）**：

| 字段 | 说明 | 维度 |
|------|------|------|
| 位置增量 | 手柄相对于按下点的位移，米 | 3 |
| 旋转增量 | 手柄相对于按下点的旋转，角轴表示 | 3 |
| Grip 按下程度 | 0.0 ~ 1.0 | 1 |

一帧共约 17 个浮点数。

### 4.2 文件结构

每个文件存一段操作，HDF5 格式：

```
logs/episode_20260713_143022_01.h5
├── /obs/joint_positions    (N × 6)
├── /obs/ee_xyz             (N × 3)
├── /obs/ee_quat            (N × 4)
├── /action/delta_xyz       (N × 3)
├── /action/delta_rot       (N × 3)
├── /action/grip            (N × 1)
└── /meta/scale_factor      (1)
```

## 5. 集成位置

在 `BaseTeleopController._update_ik()` 方法中：

```
_update_ik() 执行流程：
  1. 更新机器人状态
  2. 读取手柄数据
  3. 判断 Grip 激活状态
  4. [插入] 根据 Grip 状态调用记录器 ← 新增
  5. 计算 IK 并求解
  6. 更新可视化
```

## 6. 文件变更清单

| 文件 | 变更 |
|------|------|
| `xrobotoolkit_teleop/common/data_collector.py` | **新增**，记录器实现 |
| `xrobotoolkit_teleop/common/base_teleop_controller.py` | 修改，加关节限位 + 插入记录调用 |
| `scripts/simulation/teleop_rm65_sim.py` | 修改（可选），加 `enable_log_data` 参数（原 `teleop_dual_ur5e_placo.py`，已更名） |
| `pyproject.toml` | 修改，加 `h5py` 依赖 |

## 7. 后续迁移到真机

真机控制器 `RealmanRM65CartesianTeleopController` 不继承 `BaseTeleopController`，但数据记录逻辑相同：

- 在 `run()` 的 Grip 状态切换处加入 `开始一段()` / `结束一段()`
- 在 Grip 激活期间每帧调用 `存一帧()`
- 观测从 `arm.get_pose6()` 读取，动作为 `_process_xr_pose()` 的返回值

此时只需 import 同一个 `DataCollector`，接口不变。

## 8. 不在本次范围内

- 夹爪数据采集（当前仿真未使用夹爪）
- 相机图像记录
- 实时数据可视化
- `config/*.json` 自动加载
- 真机数据采集实现（后续单独做）
