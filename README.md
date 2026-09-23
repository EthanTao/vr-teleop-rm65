# RM65 VR 遥操作包

Realman RM65 侧装 VR 遥操作最小代码包（仿真 + 真机笛卡尔）。

> **文档入口**：完整索引见 [docs/README.md](docs/README.md)。核心 5 篇：系统全貌与可复现流程见 [docs/项目交接.md](docs/项目交接.md)；新手操作手册见 [docs/仿真操作手册.md](docs/仿真操作手册.md)；真机安全与故障解锁见 [docs/真机安全与故障处理.md](docs/真机安全与故障处理.md)；RM65 参数与坐标系语义见 [docs/RM65参数与坐标系.md](docs/RM65参数与坐标系.md)；历史决策与已废弃结论见 [docs/决策与结论汇总.md](docs/决策与结论汇总.md)。

## 快速开始

### 仿真（远程 Docker 内）
```bash
export XROBO_GRIP_THRESHOLD=0.2
python scripts/simulation/teleop_rm65_sim.py --home-joint-deg J1 J2 J3 J4 J5 J6
```

`J1..J6` 必须替换为已在侧装环境验证无碰撞、远离奇异点的 RM65 Home Joint 角度（单位：度）。官方将 `[0,0,0,0,0,0]` 列为边界奇异示意点，不能把它作为默认 Home Pose；详见 [RM65 参数与坐标系语义](docs/RM65参数与坐标系.md)。

默认 `--ik-backend placo` 保留原仿真行为。若容器已安装睿尔曼 API2 Python SDK，可使用历史官方连续 IK 仿真（与新默认 DLS 真机路径不同）：

```bash
python scripts/simulation/teleop_rm65_sim.py \
  --home-joint-deg J1 J2 J3 J4 J5 J6 \
  --ik-backend official
```

官方模式以官方 FK 计算 Grip 锚点，以当前仿真关节作为 `rm_algo_ik_remote` seed，再把输出关节角写入 URDF/Meshcat 显示；Placo 不参与关节求解。该模式会拒绝 J3 进入默认 ±5° 排除区以及超过 RM65-B 10% 型号速度的单周期关节跳变。它用于算法链验证，不证明 URDF 末端显示位置与控制器 TCP 坐标一致。

### 真机

推荐入口默认使用 `damped_ik_movej_follow`：独立 UDP 反馈、笛卡尔限速、奇异区域降速、阻尼 IK 和受限关节目标跟随。运行容器需要睿尔曼 API2 SDK，并核对工作/工具帧配置；详见[真机安全与故障处理](docs/真机安全与故障处理.md)。先确认机械臂实时推送已指向运行本程序的主机 UDP `8089`，并包含 `joint_status`、`waypoint` 和错误信息。

```bash
# 1. 只读 TCP 预检（不发送运动指令）
python scripts/hardware/preflight_realman_rm65.py --arm-host 192.168.10.18 --arm-port 8080

# 2. 安全链路 dry-run：检查 XR / UDP / 型号 / 时序，不发送轨迹目标
python scripts/hardware/teleop_realman_rm65_safe_hardware.py --arm-host 192.168.10.18 --dry-run

# 3. 首次低速、小范围运动（默认幅度已收紧，可先用默认值起测）
python scripts/hardware/teleop_realman_rm65_safe_hardware.py --arm-host 192.168.10.18
```

默认 `move_v=5`、TCP 线速度上限 `0.05 m/s`、角速度上限 `0.25 rad/s`；默认幅度为平移比例 `--scale-factor 0.25`、旋转比例 `--rotation-scale-factor 0.25`、单次 Grip 包络 `--max-offset-m 0.03` / `--max-rot-offset-rad 0.20`（约 11.5°），需要恢复更大行程时在命令行显式覆盖；关节速度超过 RM65-B 型号上限的 10% 时立即硬停止并锁存。故障后先松开 Grip，确认机械臂静止且反馈正常，再按 B 解锁；正常松开 Grip 使用软停止。首次上机仍必须保证安全初始姿态、净空、急停可触达并有现场监护，自动化测试不能替代真机验收。

奇异点保护默认开启，并实际发送 DLS 算出的受限关节目标。危险区阻止继续深入，满足关节限位、局部路径及反馈约束的退出方向可低速通过。`[DLS] hold_*` 请求软停止并等待实测静止；单纯 HOLD 不需要 B 解锁，停止失败或其他安全错误仍会锁存。正常 B 回位与自检也经过同一检查。它不保证任意姿态都能退出，不提供自动换肘或绕路。实现与验证记录见 [真机安全与故障处理](docs/真机安全与故障处理.md)。

历史可选的官方连续 IK 模式使用 `rm_algo_ik_remote` + `movej_follow`，不提供上述 DLS 保护，必须显式关闭奇异保护开关。它要求容器已按睿尔曼官方方式安装 API2 Python SDK，并且现场已核对当前工作/工具坐标系、设置 RM65 J3 非零软限位。先运行 dry-run；该模式下 dry-run 会执行 IK 和关节步长检查，但不会发送运动目标：

```bash
python scripts/hardware/teleop_realman_rm65_safe_hardware.py \
  --arm-host 192.168.10.18 \
  --motion-command official_ik_movej_follow \
  --no-enable-singularity-avoidance \
  --official-ik-tool-or-work 1 \
  --official-ik-frames-verified \
  --official-ik-j3-soft-limit-verified \
  --dry-run
```

上述两个 `verified` 开关是 fail-closed 门槛，不代表程序自动完成了坐标系标定或软限位配置。未安装 SDK、IK 失败、解超关节限位或单周期关节步长超限时，程序会拒绝启动或锁存停止。

`teleop_realman_rm65_placo_hardware.py` 保留为兼容入口，参数与上述新入口一致。

旧入口 `teleop_realman_rm65_hardware.py` 的真机运动已禁用，只保留 `--dry-run` 映射诊断，避免绕开上述保护。

### Meshcat 可视化（本地）
```bash
bash scripts/tools/start_meshcat_viewer.sh
```

### 一键启动（推荐 🚀）
一条命令完成代码同步、模式选择、SSH 隧道和脚本启动：

```bash
# 交互菜单
python teleop.py

# 直接启动仿真
python teleop.py --mode sim --home-joint-deg J1 J2 J3 J4 J5 J6

# 一键启动官方 IK 仿真
python teleop.py --mode sim --sim-ik-backend official --home-joint-deg J1 J2 J3 J4 J5 J6

# 真机检查：连接和计算目标，但不运动
python teleop.py --mode hardware-dry-run

# 直接启动真机
python teleop.py --mode hardware

# 仅同步代码（手动调试用）
python teleop.py --mode sync-only
```

## 依赖

### 宿主机（Windows）
一键脚本 `teleop.py` 使用 Python 标准库，无需额外安装。需要确保：
- `ssh` 可用，且安装了 `rsync` 或 `scp`（Windows OpenSSH 即提供 `scp`）
- 已配置 SSH 免密登录到远程板（`ssh rm@192.168.0.115`）

### 容器内（Docker）
`pyproject.toml` 包含 `placo`, `meshcat`, `numpy`, `tyro`, `h5py` 和官方 `Robotic_Arm`；`xrobotoolkit_sdk` 仍由 Docker 安装脚本提供。默认 DLS 模式（包括 Dry Run）需要官方 FK，可选官方 IK 模式也需要此 SDK。[官方安装说明](https://develop.realman-robotics.com/robot/apipython/getStarted/)。

旧容器只同步源码不会自动安装新增依赖。出现 `No module named 'Robotic_Arm'` 时，在 Windows PowerShell 执行：

```powershell
ssh rm@192.168.0.115 "sudo docker exec xrobo-vr-teleop python -m pip install Robotic_Arm"
```

使用 `python -m pip`，确保装入 `teleop.py` 实际调用的容器 Python，而不是 Windows 的 `(base)` 环境。`docker exec` 不会继承入口进程运行时的 `conda activate`；若手动切换解释器，安装和启动必须使用同一个解释器。新版 Dockerfile 已将 `xrobo/bin` 放到 PATH 首位（重建镜像后生效）。本次在现有 `/opt/conda/bin/python`（3.13）中验证了 `Robotic_Arm 1.1.6` 的离线 FK，真机仍须核对控制器版本和工作/工具帧。

如需在容器内手动安装依赖，建议使用清华镜像源加速：
```bash
pip install <包名> -i https://pypi.tuna.tsinghua.edu.cn/simple
```

### DLS 启动参数与帧一致性

默认真机 DLS（含 Dry Run）启动时只读获取实际 DH 和当前工具参数，用于本地官方 FK。显式 `--dls-flange-to-tcp` 优先；`--dls-work-from-base` 保持显式配置，不按当前姿态自动拟合。读取失败、SDK 读回不一致或 FK/UDP 偏差超过 5 mm / 2° 时拒绝启动。

现场 RM65-BI 的 d6=161.2 mm，区别于标准 SDK 默认的 144 mm；finger 工具也不是单位变换。详见 [真机安全与故障处理](docs/真机安全与故障处理.md) §1.1 与 [决策与结论汇总](docs/决策与结论汇总.md) §2.10。停止其他 UDP 8089 接收程序后，可运行不启动 XR、不发送运动命令的复验：

```powershell
ssh rm@192.168.0.115 "sudo docker exec -w /workspace xrobo-vr-teleop python scripts/hardware/check_realman_dls_frames.py"
```

2026-09-23 的目标容器 1000 周期 Dry Run 中，完整 `_send_motion` 安全准入链 median `10.169 ms`、P95 `11.782 ms`、max `14.397 ms`，满足 `18 ms` DLS 准入预算与 `20 ms` 控制周期；测试姿态位于奇异减速区（`ratio=0.03189`、`scale=0.730`）。这只证明当前 RM65-BI、实际 DH、`finger` 工具帧和 14-FK 路径下的**计算与准入时序**，不代表真实 `movej_follow` 发送、XR 主循环、机械臂跟踪、方向手感或物理安全已经验收。

## 项目结构

```text
vr-teleop-rm65/
├── xrobotoolkit_teleop/      # 核心库（控制器、XR 客户端、协议层）
│   ├── common/               #   抽象基类、XR 客户端
│   ├── simulation/           #   Placo/睿尔曼官方 IK + Meshcat 仿真
│   ├── hardware/             #   真机控制器 + TCP/JSON 接口
│   └── utils/                #   几何运算、Meshcat 工具
├── scripts/
│   ├── simulation/           # 仿真入口
│   ├── hardware/             # 真机入口
│   ├── tools/                # 辅助工具（安装/验证/启动 Viewer）
│   └── debug/                # 调试脚本
├── assets/realman/           # RM65 URDF 模型（官方 + SW 导出）
├── config/                   # 参数参考快照（不会自动加载）
├── docker/                   # Docker 构建文件
├── docs/                     # 文档、开发计划、设计文档
└── tests/                    # 单元测试
```

## 不含
- 多机器人（UR5 等）相关代码
- Placo IK 真机路径（真机默认官方 FK + DLS + 受限 `movej_follow`，详见奇异区域保护说明）
- RM65 mesh 二进制（需补齐 `assets/realman/*/meshes/`）

## 历史代码与生成物

历史控制器和单文件诊断实现集中在 `xrobotoolkit_teleop/hardware/legacy/`，旧导入路径与旧命令保留兼容包装。详见 [项目交接](docs/项目交接.md) §6.1。

`config/realman_rm65_side_mount_teleop.json` 是参考快照；修改它不会改变运行参数。当前代码默认 `invert_tcp_xy=False`，应通过 CLI 显式传参。
