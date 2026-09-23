

## 项目概述

VR 遥操作（teleoperation）系统，将 PICO 头显 + XRobo·Toolkit 的 VR 手柄输入映射到 **Realman RM65 六轴机械臂**（侧装安装，非水平地面）。支持仿真（默认 Placo IK、可选睿尔曼官方连续 IK + Meshcat）和真机（TCP/JSON 轨迹下发 + UDP 独立安全反馈）两种模式。

**设计原则**：仿真先行、真机后置；URDF FK 与真机位姿偏差大（660–1300mm），真机不走 Placo IK，改用控制器 TCP 帧。

## 命令

```bash
# 安装（开发模式）
pip install -e .
uv pip install -e .            # 容器内用 uv

# 格式化
black .                        # line-length=120 (pyproject.toml)

# 仿真运行（远程 Docker 内）
export XROBO_GRIP_THRESHOLD=0.2
python scripts/simulation/teleop_rm65_sim.py --home-joint-deg J1 J2 J3 J4 J5 J6

# 可选：以官方 FK 建锚点、官方连续 IK 求关节、Meshcat 显示
python scripts/simulation/teleop_rm65_sim.py --home-joint-deg J1 J2 J3 J4 J5 J6 --ik-backend official

# 真机运行
python scripts/hardware/teleop_realman_rm65_safe_hardware.py --arm-host 192.168.10.18

# 真机参数示例：B 键返回启动时记录的 TCP 姿态
python scripts/hardware/teleop_realman_rm65_safe_hardware.py --arm-host 192.168.10.18 --reset-button B --reset-duration-s 3.0

# 可选：官方连续 IK dry-run（需官方 API2 SDK、坐标系实测确认、J3 非零软限位）
python scripts/hardware/teleop_realman_rm65_safe_hardware.py --arm-host 192.168.10.18 --motion-command official_ik_movej_follow --no-enable-singularity-avoidance --official-ik-frames-verified --official-ik-j3-soft-limit-verified --dry-run

# 真机保留 XYZ 每轴边界（默认 -1~1m）

# 真机启动前只读预检（不会发送运动指令）
python scripts/hardware/preflight_realman_rm65.py --arm-host 192.168.10.18 --arm-port 8080

# 旧版单文件入口仅允许 dry-run，真机运动已禁用
python scripts/hardware/teleop_realman_rm65_hardware.py --dry-run

# URDF FK 验证（对比控制器 joint/pose 反馈）
python scripts/hardware/validate_realman_rm65_urdf.py

# 启动 Meshcat Viewer（本地 SSH 隧道 + 浏览器）
bash scripts/tools/start_meshcat_viewer.sh

# 远程容器验证（安装依赖 + 跑测试 + 检查数据）
python scripts/tools/run_validation.py

# 调试脚本（极限验证、Meshcat 连通性等）
python scripts/debug/check_limits.py

# Docker 构建启动
docker compose -f docker/docker-compose.yml up --build
```

## 架构

```
PICO 头显 → WiFi → PC Service (63901)
  → SSH 隧道 → 远程板/容器 → RM65 真机 (192.168.10.18:8080)
                                  或 → Placo/官方连续 IK → Meshcat 可视化
```

### 仿真路径
XR delta → 侧装旋转矩阵 → 默认 Placo IK → 关节角 → Meshcat（3D 可视化）
                              或 → 官方 FK 锚点 + 官方连续 IK → 安全检查 → Meshcat（可选）

### 真机路径
XR delta → 侧装旋转矩阵 → 锚点目标 → 速度/加速度限幅 → 官方 FK / SVD / DLS → 受限 movej_follow（默认）
                                                     或 → 官方连续 IK（上一已接受目标作 seed）→ 命令连续性/实测偏差检查 → movej_follow（可选）
RM65 实时 UDP → 关节速度/限位/错误/跟踪误差 → 软停止或硬停止 + 故障锁存

### 包结构 (`xrobotoolkit_teleop/` + `scripts/`)

| 模块 | 职责 |
|------|------|
| `common/BaseTeleopController` | 抽象基类：XR 数据读取、锚点/增量映射、Placo IK 循环、夹爪控制 |
| `common/XrClient` | 封装 `xrobotoolkit_sdk`，提供手柄/头显/体感追踪器数据 |
| `simulation/PlacoTeleopController` | 仿真子类：Placo IK + Meshcat 可视化，单线程事件循环 |
| `simulation/RealmanOfficialIKSimulationController` | 可选仿真子类：官方 FK/连续 IK 求关节，URDF/Placo wrapper 仅用于 Meshcat 显示 |
| `simulation/OfficialIKSimulationGuard` | 官方 IK 仿真的 RM65-B 关节范围、J3 零位排除区和单周期步长检查 |
| `hardware/RealmanRM65SafeTeleopController` | **推荐真机控制器**：delta → 限速 → DLS → 受限 `movej_follow`，UDP 超速检测与故障锁存 |
| `hardware/realman_official_ik` | 可选睿尔曼 API2 `rm_algo_ik_remote` 适配器；SDK 延迟导入，错误时 fail-closed |
| `hardware/RealmanRM65CartesianTeleopController` | 历史模块化控制器，实现位于 `hardware/legacy/`，原路径仅兼容导入；保留回归/审计，不作为真机回退入口 |
| `hardware/interface/RealmanRM65Interface` | TCP/JSON 协议层：`movep_follow`、`movel`、`movej_follow`、软/硬停止、启动信息查询 |
| `hardware/realman_udp_feedback` | latest-only 实时 UDP 反馈：关节角/速度、TCP 位姿、错误码 |
| `hardware/cartesian_command_limiter` | TCP 线/角速度与加速度限幅 |
| `hardware/control_timing` | 绝对截止时间调度与循环时序统计 |
| `utils/geometry` | 三套 `R_headset_world` 矩阵、delta pose 运算、四元数工具 |
| `utils/meshcat_utils` | Docker 环境下打印 Meshcat Viewer URL |
| **`scripts/tools/`** | **辅助工具：安装依赖、远程验证、启动 Meshcat Viewer** |
| **`scripts/debug/`** | **调试脚本：极限验证、Meshcat 连通性测试、机器人可视化测试** |

### 入口脚本 (`scripts/`)

| 文件 | 用途 |
|------|------|
| `simulation/teleop_rm65_sim.py` | 仿真主入口（`--ik-backend placo|official`，含滤波/约束/复位参数） |
| `hardware/teleop_realman_rm65_safe_hardware.py` | **真机推荐入口**，tyro CLI |
| `hardware/teleop_realman_rm65_hardware.py` | 旧版单文件入口；真机运动已禁用，仅保留 dry-run 诊断 |
| `hardware/validate_realman_rm65_urdf.py` | URDF FK 与控制器姿态对比验证 |

## 关键配置

**侧装映射矩阵**（`xrobotoolkit_teleop/utils/geometry.py`，三套）：

| 矩阵 | 用途 |
| --- | --- |
| `R_HEADSET_TO_WORLD` | 标准桌面安装（UR5 用） |
| `R_HEADSET_TO_WORLD_RM65_SIDE_MOUNT` | 仿真验证矩阵（2026-07-13 Placo 仿真自然手感；真机 `invert_tcp_xy=True` 时有效映射等价；当前仿真入口未引用） |
| `R_HEADSET_TO_WORLD_RM65_SIDE_MOUNT_HARDWARE` | **仿真入口与真机控制器当前均引用**（代码为准，2026-09-09 定案：回到改动前行为）；真机配 `invert_tcp_xy=True` 时有效手感等价 `SIDE_MOUNT` |

> ⚠️ **仿真手感**：机器人按 URDF 竖立显示、映射保持代码原状（手柄映射矩阵不因显示而改）。按第 12 条原则，方向是否符合人体直觉以逐轴实测为准（判定表见 `docs/RM65参数与坐标系.md` §14），实测结论前不要盲改矩阵。（2026-09-09：用户已确认当前 `_HARDWARE` 映射关系无问题，维持现状、冻结。）

**URDF 模型**（两个变体，`assets/realman/`）：

| 模型 | 关节命名 | 特点 |
| --- | --- | --- |
| `RM65-official/` | `joint1`~`joint6`, `r_link1`~`r_link6` | 官方 URDF，无惯量/碰撞，mesh 依赖 |
| `RM65-B/` | `joint_1`~`joint_6`, `link_1`~`link_6` | SW 导出，含惯量/碰撞几何 |

**参数快照**（参考用，代码未自动加载）：`config/realman_rm65_side_mount_teleop.json`

**环境变量**：
- `XROBO_GRIP_THRESHOLD` — Grip 激活阈值（默认 0.5，仿真推荐 0.2）
- `XROBO_DEBUG=1` — XR 数据调试日志
- `MESHCAT_HOST_PORT` — 容器内 Meshcat 端口映射

## 远程开发（Docker）

代码运行在远程 Linux 板（`192.168.0.115`）的 Docker 容器内。所有仿真可视化通过 SSH 端口转发到本地浏览器：

```bash
ssh -N -o ExitOnForwardFailure=yes -L 18081:127.0.0.1:7000 -L 63901:127.0.0.1:63901 rm
```

`rm` 是已配置的 SSH 别名：实验室内网自动走 `192.168.0.115`，宿舍/校外自动回退 Tailscale（`rm-ts` 强制走 Tailscale）。
浏览器打开 `http://127.0.0.1:18081/static/` 查看 Meshcat。

代码同步流程：本地 → `rsync`/`scp` → 板子宿主机 → `docker cp` → 容器内 `/workspace/`。推荐直接用 `python teleop.py --mode sync-only`。

## 文档地图（2026-09-23 整理后）

`docs/` 下只有 5 篇核心文档，历史全文在 `docs/archive/`：

| 文档 | 用途 |
| --- | --- |
| `docs/项目交接.md` | **系统全貌**：设计原则、运行环境、架构与仿真/真机一致性边界、可复现流程、代码入口职责、下一步优先级 |
| `docs/仿真操作手册.md` | 仿真上手、操作、调参、FAQ、仿真参数速查 |
| `docs/真机安全与故障处理.md` | DLS 控制链、风险判据、解锁四步、真机参数速查、验证顺序、上机自检清单 |
| `docs/RM65参数与坐标系.md` | RM65 本体参数、MDH 警示、奇异点分类、XR SDK 坐标系语义、侧装矩阵、Meshcat 轴色、逐轴实测记录 |
| `docs/决策与结论汇总.md` | 全部历史日志/specs/plans 的结论集 + **⚠️ 已废弃/被推翻结论清单**；改动前先读 §4 |
| `docs/plans/2026.9.22-降低每周期FK调用.md` | DLS 性能历史提案；bounded-search 已落地并通过目标容器 1000 周期 Dry Run 时序验证，真实运动待验收 |

> 引用旧路径（`docs/2026.9.*.md`、`docs/VR_TELEOP_HANDOFF.md`、`docs/奇异区域保护.md` 等）时，请改用上表的新路径；旧文件已移入 `docs/archive/` 且不再维护。

## 重要注意事项

1. **仿真和真机 controller 仍分离**: 仿真默认 `PlacoTeleopController`，`--ik-backend official` 时使用 `RealmanOfficialIKSimulationController`；推荐真机入口使用 `RealmanRM65SafeTeleopController`。两种官方 IK 模式共享算法适配器，但仿真状态来自内部关节，真机新段首帧 seed 来自 UDP，后续来自上一已接受目标，并独立检查相对 UDP 实测状态的偏差，不能混用状态源。
2. **URDF FK 与真机偏差大**：官方 IK 仿真以官方 FK 建立锚点，URDF/Placo wrapper 只显示关节构型；真机仍依赖控制器报告的 TCP 位姿。不要用 Meshcat 中的 URDF 末端坐标替代控制器 TCP 坐标。
3. **侧装**：旋转矩阵与桌面安装的 UR5 不同，调整前先确认安装方式。
4. **位置姿态同步跟随**：默认 `suppress_rotation_during_translation=False`、`freeze_rotation=False`。平移时抑制旋转仅作为显式选项，不因连续 IK 优化而默认开启。
5. `config/*.json` 暂未被代码加载，需手动传入参数。
6. **自碰撞避免默认关闭**（`enable_self_collision_avoidance=False`），需显式开启。开启后 IK 求解器增加不等式约束，可能轻微影响跟踪精度。
7. **运行时调参**仅 Linux TTY 环境可用（Docker 内 `docker exec -it`），Windows 下 `select` 不适用。在容器内用 `--enable-tuning` 开启后按键盘 `d/r/a/o` 调整参数。
8. **`meshcat.geometry.Label` 不存在于已发布版本**（0.3.2 及之前均无）。`placo_teleop_controller.py` 中已移除相关调用，改用球体指示灯（绿=就绪/橙=复位中）替代文字标签。若升级 meshcat 版本后 `Label` 类可用，可重新添加——但需同时兼容旧版本，或用 try/except 兜底。

9. **真机安全停止**：推荐控制器默认 `move_v=5`、TCP 线速度 `0.05 m/s`、角速度 `0.25 rad/s`，每周期先过速度/加速度限制器。实时 UDP 反馈必须包含 `joint_speed`；反馈超时、关节超速/越界、控制器错误、跟踪误差、XR 超时/跳变或 TCP 边界违规都会硬停止并锁存。故障后必须松开 Grip，待反馈新鲜且各关节低于 `1 deg/s`，再按 B 解锁；正常松 Grip 使用软停止。旧单文件入口禁止真机运动。该保护不等价于碰撞检测或现场安全认证，首次上机仍须从当前默认的收紧幅度（平移/旋转比例 `0.25`、单次 Grip 包络 `--max-offset-m 0.03` / `--max-rot-offset-rad 0.20`）起验证，确认平稳后再逐项放宽。

10. **代码改动须记入工作日志**：每次修改代码（新增功能、修 bug、临时调试开关等），都要在 `docs/` 下按日期命名的工作日志（如 `docs/2026.9.23.md`）追加记录，包含日期、修改文件、背景/原因、改动内容、验证状态。先记日志再视为完成。（旧的 `docs/modification-log.md` 已于 2026-09-09 停用，现归档在 `docs/archive/`。）
11. **仿真与真机代码并行维护**：两端共用 XR 映射和 `hardware/realman_official_ik.py` 适配器。修改时同步检查映射、锚点、周期、seed、边界和 J3 保护。2026-09-19 起真机使用 `hardware/damped_ik.py`，仿真仍为 Placo / 官方连续 IK，尚未接入新 DLS 后端；不能声称两端运动算法相同。旧 `hardware/singularity_avoidance.py` 仅供离线结构分析，不能用其 URDF 雅可比批准真机 TCP 运动。详见 `docs/真机安全与故障处理.md`。
12. **仿真映射必须符合人体直觉**：仿真是手感标定场，仿真入口的映射关系是「直觉参考系」（手柄上抬→工具端升高、前推→伸远、右推→向右，按真实侧装语境验收），不能用真机适配矩阵反向定义仿真手感。真机矩阵修正（`_HARDWARE`、`invert_tcp_xy`）只是把这份直觉搬运到物理机器上。仿真矩阵的选择以逐轴实测为准（判定表见 `docs/RM65参数与坐标系.md` §14）；任何让仿真映射偏离直觉的改动视为问题。

## 2026-09-16 代码整理

- 推荐真机入口：`scripts/hardware/teleop_realman_rm65_safe_hardware.py`；旧 `teleop_realman_rm65_placo_hardware.py` 仅转发到同一 main，CLI 参数保持一致。
- 历史实现集中于 `xrobotoolkit_teleop/hardware/legacy/`，现有测试继续覆盖其映射/保护逻辑；旧单文件命令仍禁止真机运动。
- 当前真机默认 `invert_tcp_xy=False`；上文提及 True 的矩阵等价关系仅适用于显式开启该选项的平移分量，不代表当前默认值。参考 JSON 已按代码修正，仍不自动加载。
- 历史代码及生成物保留策略见 `docs/项目交接.md` §6.1，本次日志见 `docs/archive/2026.9.16.md`。

## 2026-09-23 文档整理

- 原先 24 篇文档（约 400 KB，含大量重复）合并为 **5 篇核心文档**：`项目交接.md`、`仿真操作手册.md`、`真机安全与故障处理.md`、`RM65参数与坐标系.md`、`决策与结论汇总.md`。
- 全部原始文档（8 篇日期工作日志、8 篇已被取代的专题文档、3 组已完成 specs/plans、已停用的 `modification-log.md`）移入 `docs/archive/`，**保留可查但不维护**。
- DLS 历史提案 `docs/plans/2026.9.22-降低每周期FK调用.md` 保留在原位；2026-09-23 的实际 bounded-search 实现以代码和当日工作日志为准。
- `docs/决策与结论汇总.md` §4 是新维护者改动前的必读清单（已废弃/被推翻结论）。
- 本次仅改文档与文档引用，**未改任何代码或运动参数**。
