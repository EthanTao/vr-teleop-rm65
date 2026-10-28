

## 项目概述

VR 遥操作（teleoperation）系统，将 PICO 头显 + XRobo·Toolkit 的 VR 手柄输入映射到 **Realman RM65 六轴机械臂**（侧装安装，非水平地面）。支持仿真（默认 Placo IK、可选睿尔曼官方连续 IK + Meshcat）和真机（TCP/JSON 轨迹下发 + UDP 独立安全反馈）两种模式。

**设计原则**：仿真先行、真机后置；URDF FK 与真机位姿偏差大（660–1300mm），真机不走 Placo IK，改用控制器 TCP 帧。

**末端灵巧手**：傲意科技（OYMotion）**ROH-LiteS001**，**没有独立力传感器**（用户于 2026-10-25 确认）。电流反馈不能当作实测接触力；后续方案不得默认具备手指力传感器反馈。型号确认不代表灵巧手控制已接入，接入状态以源码与验证记录为准。详见 `docs/项目交接.md` §2.1。

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

# 真机参数示例：B 键返回固定六轴 Home [0, 0, -60, 0, 60, 0]
# 回位路径预检默认关闭（起点在 J3/J5 零位另一侧也能回位）；加 --enable-home-path-preflight 恢复拒绝式预检
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

# 真机验收判定（配合 --motion-log-path 录到的 JSONL；退出码 0 = 判据全过，2 = 未通过）
python scripts/debug/report_teleop_acceptance.py logs/motion.jsonl
# 逐轴方向定案（从日志拟合「手柄→末端」映射矩阵，不需要额外试动作）
python scripts/debug/estimate_teleop_mapping.py logs/motion.jsonl
# 每段「手动了多少 / 臂实际动了多少」
python scripts/debug/analyze_motion_log.py logs/motion.jsonl
# 查看实时 UDP 数据报里到底推了哪些通道（含是否有 joint_current）
python scripts/debug/dump_udp_frame.py

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
| `simulation/teleop_rm65_sim.py` | 仿真主入口（`--ik-backend` 选择 `placo` / `official` / `shared-dls`，不同后端的参数边界见仿真操作手册） |
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

## 文档地图（2026-10-03 更新）

完整导航见 `docs/README.md`，全仓库目录说明见 `docs/仓库结构.md`，脚本导航见 `scripts/README.md`。`docs/` 根目录维护 5 篇核心文档、真机实验记录说明与索引；日期工作日志和改动路线在 `docs/logs/`，离线数据快照在 `docs/data/`，历史工程文档在 `docs/archive/`，研究报告与导师版 Word 在 `docs/research/`，原始分题草稿在 `docs/research/drafts/`：

| 文档 | 用途 |
| --- | --- |
| `docs/项目交接.md` | **系统全貌**：设计原则、运行环境、架构与仿真/真机一致性边界、可复现流程、代码入口职责、下一步优先级 |
| `docs/仿真操作手册.md` | 仿真上手、操作、调参、FAQ、仿真参数速查 |
| `docs/真机安全与故障处理.md` | DLS 控制链、风险判据、Grip 重新使能、真机参数速查、验证顺序、上机自检清单 |
| `docs/RM65参数与坐标系.md` | RM65 本体参数、MDH 警示、奇异点分类、XR SDK 坐标系语义、侧装矩阵、Meshcat 轴色、逐轴实测记录 |
| `docs/决策与结论汇总.md` | 全部历史日志/specs/plans 的结论集 + **⚠️ 已废弃/被推翻结论清单**；改动前先读 §4 |
| `docs/plans/2026.9.22-降低每周期FK调用.md` | DLS 性能历史提案；bounded-search 已落地并通过目标容器 1000 周期 Dry Run 时序验证，真实运动待验收 |

> 引用旧路径（`docs/2026.9.*.md`、`docs/VR_TELEOP_HANDOFF.md`、`docs/奇异区域保护.md` 等）时，请改用上表的新路径；旧文件已移入 `docs/archive/` 且不再维护。

## 重要注意事项

1. **仿真和真机 controller 仍分离**: 仿真默认 `PlacoTeleopController`，`--ik-backend official` 时使用 `RealmanOfficialIKSimulationController`；推荐真机入口使用 `RealmanRM65SafeTeleopController`。两种官方 IK 模式共享算法适配器，但仿真状态来自内部关节，真机新段首帧 seed 来自 UDP，后续来自上一已接受目标，并独立检查相对 UDP 实测状态的偏差，不能混用状态源。
2. **URDF FK 与真机偏差大**：官方 IK 仿真以官方 FK 建立锚点，URDF/Placo wrapper 只显示关节构型；真机仍依赖控制器报告的 TCP 位姿。不要用 Meshcat 中的 URDF 末端坐标替代控制器 TCP 坐标。
3. **侧装**：旋转矩阵与桌面安装的 UR5 不同，调整前先确认安装方式。
4. **位置姿态同步跟随**：默认 `suppress_rotation_during_translation=False`、`freeze_rotation=False`。平移时抑制旋转仅作为显式选项，不因连续 IK 优化而默认开启。
5. `config/realman_rm65_side_mount_teleop.json` 是未自动加载的参考参数，需手动传入 CLI；`config/rm65_dls_calibration.json` 是 shared-dls 会读取的控制器 DH/工具帧快照，已被 Git 忽略，两者用途不同。
6. **自碰撞避免默认关闭**（`enable_self_collision_avoidance=False`），需显式开启。开启后 IK 求解器增加不等式约束，可能轻微影响跟踪精度。
7. **运行时调参**仅 Linux TTY 环境可用（Docker 内 `docker exec -it`），Windows 下 `select` 不适用。在容器内用 `--enable-tuning` 开启后按键盘 `d/r/a/o` 调整参数。
8. **`meshcat.geometry.Label` 不存在于已发布版本**（0.3.2 及之前均无）。`placo_teleop_controller.py` 中已移除相关调用，改用球体指示灯（绿=就绪/橙=复位中）替代文字标签。若升级 meshcat 版本后 `Label` 类可用，可重新添加——但需同时兼容旧版本，或用 try/except 兜底。

9. **真机安全停止**：推荐控制器默认 `move_v=5`、TCP 线速度 `0.05 m/s`、角速度 `0.25 rad/s`，每周期先过速度/加速度限制器。实时 UDP 反馈必须包含 `joint_speed`；反馈超时、关节超速/越界、控制器错误、跟踪误差、XR 超时/跳变或 TCP 边界违规都会硬停止并锁存。故障后松开 Grip，待反馈新鲜且各关节低于 `1 deg/s` 即重新使能；若故障时 Grip 已松开，需先扣住再松开一次。B 键只负责固定 Home 复位；正常松 Grip 使用软停止。旧单文件入口禁止真机运动。该保护不等价于碰撞检测或现场安全认证，当前按用户要求将平移映射改为 1:1，平移死区为 0，单次 Grip 平移包络入口默认**不设限**（真机入口 `--max-offset-m` 默认 `None`，控制器库默认仍为 `0.15`；2026-09-26 用户要求取消）；旋转比例仍为 `0.25`、旋转包络仍为 `--max-rot-offset-rad 0.20`。取消包络后，一段 Grip 仍受 TCP 工作空间盒（每轴 `±1.0 m`，应按现场收窄）、XR 单帧跳变门槛、关节限位/超速与跟随余量限制，失去的是单段累计漂移上限。首次真机验收仍须低速、小位移、净空与急停就绪，可显式收紧 `--max-offset-m 0.03`。

10. **代码改动须记入工作日志**：每次修改代码（新增功能、修 bug、临时调试开关等），都要在 `docs/logs/` 下按日期命名的工作日志（如 `docs/logs/2026.10.3.md`）追加记录，包含日期、修改文件、背景/原因、改动内容、验证状态。先记日志再视为完成。（旧的 `docs/modification-log.md` 已于 2026-09-09 停用，现归档在 `docs/archive/`。）
11. **仿真与真机代码并行维护**：两端共用 XR 映射和 `hardware/realman_official_ik.py` 适配器。修改时同步检查映射、锚点、周期、seed、边界和 J3 保护。2026-09-19 起真机使用 `hardware/damped_ik.py`，仿真默认仍为 Placo / 官方连续 IK；显式选择 shared-dls 时调用同一 RM65ControlCore 与 DLS 准入链，并从控制器快照加载 DH/工具帧。虚拟反馈立即跟随已接受命令，不能据此声称真机执行效果相同。旧 `hardware/singularity_avoidance.py` 仅供离线结构分析，不能用其 URDF 雅可比批准真机 TCP 运动。**2026-09-26 晚起 DLS 奇异保护是「方向优先 + 单侧」**：步长用方向保持的最小范数解（阻尼不再改写方向），`stop_ratio 0.01` 按**手柄方向的等效增益** `σ_eff/σ_max` 判定，只保持「按原方向走不了」的命令；能走就降速走，结构性退出与硬区切向放行，进入硬区/穿越 J3/J5 零位仍拒绝；加速度限幅改用公共系数。阈值、锁存、停止与跟随余量语义未变。详见 `docs/真机安全与故障处理.md`、`docs/logs/2026.9.26.md`。
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

## 2026-10-03 仓库目录整理

- 日期工作日志与路线总览集中在 `docs/logs/`；代码改动继续按第 10 条记日志。
- 离线扫描快照集中在 `docs/data/`；文档生成脚本位于 `scripts/tools/make_advisor_docx.py`，输出仍在 `docs/research/`。
- 仓库结构见 `docs/仓库结构.md`，全部手动脚本的用途见 `scripts/README.md`，两类配置的加载边界见 `config/README.md`。
- pytest 缓存固定为 `build/cache/pytest/`；实验会话、验证报告、演示与历史备份分别保留在 `build/` 对应用途目录。
- 本次迁移和路径核对不修改控制链、安全门槛或仿真/真机行为；详细验证见 `docs/logs/2026.10.3.md`。
