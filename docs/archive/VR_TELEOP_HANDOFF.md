# Realman RM65 VR 遥操作 — 进展与可复现流程

> 2026-09-19 更新：默认真机已切换为 DLS 受限关节跟随。最新实现与验证边界见 [2026.9.19.md](2026.9.19.md) 和[奇异区域保护](奇异区域保护.md)。

> 本文档供**无上下文 AI / 接手开发者**继续工作。代码包见同目录上级 `vr-teleop-rm65/`。本文档最近整理于 2026-08-31。

---

## 0. 阅读须知：开发与运行环境

| 角色 | 操作系统 | 说明 |
|------|----------|------|
| **当前开发机** | **Windows** | 代码位于 `D:\HDU-EILab\vr-teleop-rm65`；通过 Git Bash / PowerShell 运行 `ssh`、`rsync` 等命令 |
| **目标运行环境** | Linux 板卡 + Docker | 仿真与 PC Service 跑在远程 `realman` 板子的容器内，**不在**本机直接运行 Placo/Meshcat |

**核心约定**：仿真可视化面板（Meshcat）**始终通过 SSH 本地端口转发（反代）到本机浏览器**访问，不在板子上直接开公网端口。容器内 Meshcat 监听 `7000`，本机浏览器打开 `http://127.0.0.1:18081/static/`。

---

## 1. 开发意图

本项目要把 **PICO 头显 + XRoboToolkit** 的 VR 手柄输入，映射到 **Realman RM65 六轴机械臂**的遥操作控制，且机械臂为 **侧装**（底座固定在垂直面，非水平地面安装）。

**设计原则（按优先级）**：

1. **仿真先行、真机后置** — 所有映射矩阵、scale、姿态跟随先在 Docker 仿真里验证，确认运动方向与手感合理后，再动真机。
2. **仿真 IK 可选** — 默认沿用 RM65 URDF + Placo IK；`--ik-backend official` 使用睿尔曼官方 FK 建锚点、官方连续 IK 求关节，Placo wrapper 只用于 Meshcat 显示。
3. **真机不走 Placo IK** — 显示 URDF 的 FK 不可替代控制器 TCP。默认 `damped_ik_movej_follow` 使用官方 FK、显式工作/工具帧变换、DLS 局部关节步长和 UDP 检查，详见[奇异区域保护](奇异区域保护.md)。
4. **侧装专用映射** — 不复用 UR5 桌面安装的 `R_headset_world`；仿真与真机入口当前均引用 `R_HEADSET_TO_WORLD_RM65_SIDE_MOUNT_HARDWARE`（代码为准，2026-09-09 定案回到改动前行为）；`R_HEADSET_TO_WORLD_RM65_SIDE_MOUNT` 为 2026-07-13 仿真验证矩阵，真机 `invert_tcp_xy=True` 时有效映射与之等价。方向直觉性以逐轴实测为准。
5. **可交接、可复现** — 远程板子 + 容器路径、端口、已知坑位写进本文档，使无历史上下文的 AI 能按步骤拉起环境。

**当前阶段判断**：仿真链路已打通；真机防超速控制第一版已完成源码与主机测试，但**尚未完成 UDP dry-run 和真机最终验收**（见 §8 下一步）。**不要跳过仿真、预检和 dry-run 直接上真机**。

---

## 2. 目标机器（`realman`）已知情况

以下为远程板子与容器内**已确认的环境事实**，接手时先核对是否仍成立。

### 2.1 硬件与网络

| 项 | 值 | 备注 |
|----|-----|------|
| SSH 主机名 | `realman` | 需在 `~/.ssh/config`（Windows: `C:\Users\<user>\.ssh\config`）中配置 |
| 板子 IP | `192.168.0.115` | 与开发机同网段；**板子重启或断网后 SSH 会超时**，需重建隧道 |
| 板子用户 | `rm` | 部分 Docker 命令需 `sudo` |
| 真机 RM65 控制器 | `192.168.10.18:8080` | **另一网段**；从板子/容器内访问，非本机直连 |
| 头显连接地址 | `192.168.0.115:63901` | 头显**直连远程板**；容器 63901 已由 docker-compose 映射到板子外网口 |

### 2.1.1 宿舍／校外访问（2026-09-22 变更）

**入口统一为 SSH 别名 `rm`，代码里不再硬编码板子 IP。** 别名在实验室走内网、在宿舍自动回退 Tailscale：

```bash
ssh rm      # 自动判断：LAN 可达走 192.168.0.115，否则走 Tailscale
ssh rm-ts   # 强制走 Tailscale
```

| 项 | 值 |
|----|-----|
| 本机 SSH 别名 | `rm`（自动切换）、`rm-ts`（强制 Tailscale） |
| 板子 tailnet 名 | `realman.tail1047d7.ts.net`（MagicDNS） |
| 板子 tailnet IP | `100.84.120.15` |
| Tailscale 账号 | `EthanTao@github`（tailnet `ethantao.github`） |
| LAN 探测脚本 | `scripts/tools/check-lan-reachable.ps1`（供 `Match exec` 使用） |

⚠️ **板子已于 2026-09-22 从实验室共用 tailnet `tailc5c182.ts.net`（`EverNightCN@github`）过户到 `ethantao.github`。**
原 tailnet 里的 `realman`(100.127.90.0) 节点已失效 —— **该 tailnet 的成员不再能通过 Tailscale 连到这块板子**，如需共享请在 `ethantao.github` tailnet 内单独 Share 该设备。**内网 `192.168.0.115` 通路不受影响。**

配置细节、踩坑与验收证据见 [2026.9.22 工作日志](2026.9.22.md) 末节「宿舍远程访问」。

### 2.2 Docker 与路径

| 项 | 值 | 备注 |
|----|-----|------|
| 容器名 | `xrobo-vr-teleop` | 仿真 + PC Service 均在此容器 |
| 容器内工作区 | `/workspace` | Python 入口、`xrobotoolkit_teleop/`、`scripts/`、`assets/` |
| 板子宿主机代码目录 | `/home/rm/xrobo` | `rsync` 目标；**仅 rsync 不会更新容器内文件**，必须再 `docker cp` |
| Conda 环境 | `xrobo` | **坑：容器内有 base(3.13) 和 xrobo(3.10) 两个 Python**，依赖装在 xrobo；`conda activate xrobo` 后 `which python` 可能仍指向 base，必要时用 `/opt/conda/envs/xrobo/bin/python` 显式调用 |
| 仿真日志 | `/workspace/logs/` | 脚本输出到 stdout，启动时用 `> ...log 2>&1` 重定向 |
| PC Service 进程 | `RoboticsService`，端口 **63901** | 头显直连「远程板 IP:63901」即可（容器端口已映射到板子） |

### 2.3 已踩过的坑（目标机上）

| 现象 | 原因 | 处理 |
|------|------|------|
| Meshcat 只有网格、无机械臂 | 多个 Meshcat 实例 / 端口不一致 | 杀光旧 `teleop_rm65_sim.py` 与 `meshcat` 进程，**只保留一个 7000** |
| 改代码不生效 | 只 rsync 到宿主机，未 `docker cp` 进容器 | 见 §6.6 |
| `pkill` 在 `docker exec bash -lc` 里误杀 shell | 退出码 143 | 用 `docker exec -d` 后台启动仿真 |
| 板子重启后隧道全断 | SSH 转发进程退出 | 本机重建 §6.2 隧道并重启仿真 §6.3 |
| 容器内 import 失败 | 用了 base Python（3.13），依赖装在 xrobo（3.10） | 用 `/opt/conda/envs/xrobo/bin/python` 或确保 `conda activate xrobo` 生效 |
| URDF mesh 未入库 | `assets/realman/.../meshes/*.STL` 缺失 | Meshcat 可能只显示关节轴；**不影响 IK 与映射测试**，但不利于目视核对外形 |
| 仿真加关节限速后映射错乱 | IK 后钳制 `q` 破坏了侧装映射 | **已回退**；若需防瞬移，只限末端目标步长，不限关节 `q` |

### 2.4 真机侧已知（仿真通过后才会用到）

- 曾出现 **左右/前后反向**：已加 `invert_tcp_xy=True`（**仅翻转平移 X/Y，不翻转旋转分量**）。
- 曾出现 **平移时手腕翻转**：保留 `suppress_rotation_during_translation` 开关；当前按要求恢复同步 6-DoF，默认值为 `False`。
- 曾出现 **关节锁死**：需在示教器清报警、重新使能；急停后松开 Grip 会 `set_arm_stop`。
- Placo IK 真机路径 **已弃用**（URDF 与控制器位姿严重不一致）。

---

## 3. 当前进展（截至 2026-08-31）

### 已完成

| 项 | 状态 | 说明 |
|----|------|------|
| 远程 Docker 仿真环境 | ✅ | 容器 `xrobo-vr-teleop`，板子 `192.168.0.115` |
| RM65 URDF 仿真模型 | ✅ | `RM65-official-arm.urdf`，末端 `r_link6` |
| RM65 官方本体参数归档 | ✅ | `docs/rm65-ontology-parameters.md`，包含关节范围、MDH、工作空间和奇异点 |
| 侧装映射矩阵 | ✅ | 代码现状：仿真入口与真机控制器均引用 `R_HEADSET_TO_WORLD_RM65_SIDE_MOUNT_HARDWARE`；`SIDE_MOUNT` 为 2026-07-13 仿真验证矩阵（真机 `invert_tcp_xy=True` 有效等价） |
| 6DOF 姿态跟随（仿真） | ✅ | `follow_orientation=True`，`control_mode=pose` |
| 官方连续 IK 仿真 | 🟡 源码/主机测试通过，待目标容器 | 官方 FK 锚点 + `rm_algo_ik_remote` + 关节保护 + Meshcat；当前开发机无官方 SDK |
| 仿真手感优化 | ✅ | MotionFilter（死区+EMA）、JointConstraints（限位+速度）、SmoothReset（B 键复位） |
| 数据采集 | ✅ | `DataCollector` + HDF5 写入，`--enable-log-data` 开启，单元测试 6/6 通过 |
| 一键启动脚本 | ✅ | `teleop.py`：同步代码 + 选模式 + 隧道 + 启动 + 清理 |
| 真机阻尼 IK 控制链路 | 🟡 源码/主机测试通过，待上机 | 默认 `damped_ik_movej_follow` + UDP；尚未验收官方 SDK 目标板时序与真机运动 |
| 真机水平轴校正 | ⚠️ 待真机复核 | 当前默认 `invert_tcp_xy=False`；True 为显式选项，只翻转平移 X/Y |
| 真机同步平移与姿态跟随 | 🟡 源码/主机测试通过，待上机 | `suppress_rotation_during_translation=False`；保护开关仍可显式开启 |
| 配置归档 | ✅ | `config/realman_rm65_side_mount_teleop.json` |

### 已验证结论

- **仿真映射**：`scale_factor=1.5`，Grip 阈值 `0.2`，侧装矩阵见 §5.2。
- **官方参数边界**：RM65-B 工作半径 610 mm、RM65-6F 为 627 mm；这是本体最大工作半径，不是安全遥操作空间。完整参数及奇异点见 [RM65 本体参数与遥操作安全含义](rm65-ontology-parameters.md)。
- **零位不是安全 Home**：官方将 `[0,0,0,0,0,0]` 列为边界奇异示意点；侧装 Home 必须现场验证，不能使用零位或文档示例代替。
- **关节步长限幅（仿真）曾导致映射异常**，已回退；速度限制开关默认关闭（`enable_velocity_limit=False`）。
- **Placo IK + URDF 不适合直接上真机**；默认 DLS 依赖匹配版本的睿尔曼官方 FK，工作/工具帧不一致时拒绝发送，不自动回退。
- **SSH 反代 Meshcat** 是标准查看方式：`18081 → 容器 7000`。
- **`meshcat.geometry.Label` 不存在于已发布版本**，已改用球体指示灯（绿=就绪/橙=复位中）。

### 未完成 / 待验收

- **真机最终验收**（当前首要任务）：真机逐轴核对运动方向、`invert_tcp_xy` 是否足够、平移时手腕是否翻转（见 §8）。
- `config/*.json` **未被代码自动加载**，参数靠 CLI 传入。
- 真机/仿真 scale 未对齐（仿真 1.5 vs 硬件 1.2）。
- URDF mesh 二进制未补齐（`assets/realman/*/meshes/`）。

---

## 4. 架构

```text
PICO 头显
  │  WiFi → 直连远程板「192.168.0.115:63901」(PC Service)
  ▼
远程板 realman (192.168.0.115)
  容器 xrobo-vr-teleop
    ├─ RoboticsService :63901  ← 头显直连
    ├─ 仿真: Placo/RealMan Official IK → Meshcat :7000
    └─ 真机（后续）: CartesianTeleop → TCP 8080 → RM65 (192.168.10.18)
      ▲
      │ SSH -L 18081→7000（Meshcat 反代到本机浏览器）
开发机（Windows）
```

**本机浏览器**：`http://127.0.0.1:18081/static/`（SSH 反代，**非**直接访问 192.168.0.115:7000）

**仿真路径**：XR delta → 侧装旋转矩阵 → 滤波 → 默认 Placo IK → 关节约束 → Meshcat；可选官方模式以官方 FK 建锚点，再用官方连续 IK + RM65-B 关节/J3/步长检查求关节，URDF 仅负责显示。  
**真机路径**：XR delta → 侧装映射 → Grip 锚点 → 笛卡尔限速 → 官方 FK / SVD / DLS → 局部路径、关节步长和最新 UDP 检查 → `movej_follow`。

---

## 5. 关键参数

### 5.1 官方本体参数

官方参数来源：[睿尔曼《RM65系列参数及D-H模型》](https://develop.realman-robotics.com/robot4th/robotParameter/RM65OntologyParameters/)。本仓库整理版见 [rm65-ontology-parameters.md](rm65-ontology-parameters.md)。

需要严格区分三类边界：

- 官方关节运动范围和本体工作半径；
- 本项目配置的 XYZ、遥操作总偏移和反馈误差限制；
- 现场根据侧装方式、障碍物、工具负载和线缆确定的安全范围。

前一类参数不能替代后两类安全边界。尤其是 610/627 mm 工作半径不能直接作为真机可达性或无碰撞判据。

### 5.2 侧装映射矩阵（代码为准）

**代码现状（2026-09-09 定案：映射保持不变）**——仿真入口 `teleop_rm65_sim.py` 与推荐真机 `RealmanRM65SafeTeleopController` 当前均引用：

```python
R_HEADSET_TO_WORLD_RM65_SIDE_MOUNT_HARDWARE = np.array([
    [0, 1, 0],
    [1, 0, 0],
    [0, 0, -1],
])
```

**仿真验证矩阵**（2026-07-13 在 Placo 仿真中验证过自然手感；真机在 `invert_tcp_xy=True` 时 delta 的 X/Y 再取反，有效映射与本矩阵等价）：

```python
R_HEADSET_TO_WORLD_RM65_SIDE_MOUNT = np.array([
    [0, -1, 0],
    [-1, 0, 0],
    [0, 0, -1],
])
```

> 两套矩阵仅水平 X/Y 两轴符号相反（水平面差 180°）。仿真用 `_HARDWARE`（无二次取反）与 `SIDE_MOUNT` 的差异是否被感知为"反直觉"，以逐轴实测判定（`docs/2026.9.9.md` §9.3），实测前不要盲改矩阵。两者均定义于：`xrobotoolkit_teleop/utils/geometry.py`

### 5.3 仿真默认（`teleop_rm65_sim.py`）

| 参数 | 值 |
|------|-----|
| 入口脚本 | `scripts/simulation/teleop_rm65_sim.py` |
| URDF | `assets/realman/RM65-official/urdf/RM65-official-arm.urdf` |
| 侧装矩阵 | `R_HEADSET_TO_WORLD_RM65_SIDE_MOUNT_HARDWARE`（以代码为准） |
| 末端连杆 | `r_link6` |
| 手柄 | `right_controller` / `right_grip` |
| scale_factor | 1.5 |
| control_mode | pose |
| IK 后端 | `--ik-backend placo`（默认）或 `official`（需 API2 SDK） |
| 官方 IK 安全检查 | J3 默认 ±5° 排除区；单周期关节步长不超过型号速度的 10% |
| 死区 | `--deadband-m 0.002` / `--deadband-rad 0.03` |
| EMA 平滑 | `--smooth-alpha-pos 0.35` / `--smooth-alpha-rot 0.3` |
| 速度限制 | `--enable-velocity-limit false`（默认关） |
| 平滑复位 | `--reset-duration-s 3.0`（B 键触发） |
| 自碰撞避免 | `--enable-self-collision-avoidance false`（默认关） |
| 运行时调参 | `--enable-tuning false`（容器内 `docker exec -it` 可用） |
| XROBO_GRIP_THRESHOLD | 0.2（环境变量） |

`--home-joint-deg` 没有安全默认值：必须提供六个现场验证的角度（J1～J6，单位为度）。B 键在 3 秒内回到这一关节姿态。

### 5.4 真机默认（`teleop_realman_rm65_safe_hardware.py`）

| 参数 | 值 |
|------|-----|
| 入口脚本 | `scripts/hardware/teleop_realman_rm65_safe_hardware.py` |
| 臂 IP:端口 | `192.168.10.18:8080` |
| scale_factor | 0.25（2026-09-18 再收紧，见 `docs/2026.9.18.md`） |
| rotation_scale_factor | 0.25（同上） |
| 控制频率 | `--control-rate-hz 50` |
| 运动命令 | 默认 `damped_ik_movej_follow`；历史模式须同时显式关闭奇异保护，无 DLS 保障 |
| 官方 IK 数据流 | 新段首帧以 UDP 实测关节为 seed，后续以上一已接受目标为 seed → 相邻目标/实测偏差检查 → `movej_follow` |
| 官方 IK 启动门槛 | 安装匹配控制器的 API2 SDK；显式确认 work/tool frame 与 J3 非零软限位；先跑 dry-run |
| move_v / move_r | 5 / 80（用于 `movel` 回退） |
| TCP 线速度 / 线加速度上限 | `0.05 m/s` / `0.40 m/s²` |
| TCP 角速度 / 角加速度上限 | `0.25 rad/s` / `1.20 rad/s²` |
| UDP 反馈 | `0.0.0.0:8089`，超时 `0.20 s`；必须含 `joint_speed` |
| 关节超速阈值 | RM65-B 各关节型号最大速度的 10% |
| 遥操作总偏移上限 | `--max-offset-m 0.03` / `--max-rot-offset-rad 0.20`（2026-09-18 再收紧） |
| 工作空间 XYZ 边界 | 默认每轴 `[-1.0, 1.0] m`，上机前应按现场收紧 |
| 平移死区 | `--motion-deadband-m 0.003` |
| 旋转死区 | `--rot-deadband-rad 0.04` |
| 复位按钮 / 时长 | `--reset-button B` / `--reset-duration-s 3.0` |
| invert_tcp_xy | false（当前代码默认） |
| suppress_rotation_during_translation | false |
| 奇异点保护 | 默认官方 FK + DLS；归一化雅可比比例阈值 `0.04 / 0.01`，J3/J5 阈值 `15° / 5°`；HOLD 请求软停止并等待实测静止，退出方向仍须通过约束；见 [2026.9.19.md](2026.9.19.md) |

真机启动时从 UDP 记录实测 TCP 作为 Home；正常 B 键尝试受保护的回位，3 秒是最短插值时长，受阻会超时。故障后须松 Grip、确认反馈正常且关节静止，再按 B 解锁。解锁不会解除奇异姿态。`[DLS] hold_*` 单独不锁存；停止失败、帧不匹配等仍会锁存。退出不是无条件放行。

### 5.5 网络与端口

| 角色 | 地址 | 端口 |
|------|------|------|
| 远程 SSH | `realman` → `192.168.0.115` | 22 |
| Meshcat（容器内） | `127.0.0.1`（板子本地） | **7000** |
| Meshcat（本机浏览器，SSH 反代） | `127.0.0.1` | **18081** |
| PC Service（头显直连） | 远程板 `192.168.0.115` | **63901** |
| PC Service（本机调试，经隧道） | 本机 `127.0.0.1` | **63901** |
| RM65 控制器 | `192.168.10.18` | 8080 |
| RM65 实时状态推送 | 运行真机程序的主机（控制器端预先配置） | UDP 8089 |

---

## 6. 可复现流程

### 6.1 前置条件

- 本机可 `ssh realman`（已配置 SSH 免密）
- 远程容器 `xrobo-vr-teleop` 存在且可 `docker exec`
- PICO 与**远程板**同局域网（能 ping 通 `192.168.0.115`）
- 头显 XRoboToolkit：**Controller + Send** 开启，状态 **WORKING**

### 6.2 本机：建立 SSH 隧道（仿真面板反代）

仿真 Meshcat **必须**经隧道访问；不要在浏览器里直接打开 `192.168.0.115:7000`（容器端口未映射到板子外网口，且多实例时易混乱）。

**PowerShell / Git Bash（Windows，OpenSSH）**：

```bash
# 别名 rm 会自动在「实验室内网 / 宿舍 Tailscale」之间切换，无需改命令
ssh -N -o ExitOnForwardFailure=yes -o ServerAliveInterval=20 -o ServerAliveCountMax=3 \
  -L 18081:127.0.0.1:7000 \
  -L 63901:127.0.0.1:63901 \
  rm
```

也可以直接用仓库里的 `scripts/tools/start_meshcat_viewer.sh`（Git Bash）或 `start_meshcat_viewer.bat`（Windows 双击）自动完成建隧道 + 开浏览器。

- 隧道里的 `63901→63901` 仅供**本机调试**访问 PC Service；头显**直连远程板** `192.168.0.115:63901`，不依赖这条转发。
- 隧道窗口需**保持打开**；关闭即断连。

验证：

```bash
curl -I http://127.0.0.1:18081/static/
# Windows 无 curl 时，浏览器直接打开 http://127.0.0.1:18081/static/
```

### 6.3 远程：启动仿真（单实例）

```bash
ssh realman "sudo docker exec xrobo-vr-teleop pkill -9 -f teleop_rm65_sim.py; \
  sudo docker exec xrobo-vr-teleop pkill -9 -f meshcat.servers.zmqserver; sleep 1; \
  sudo docker exec -d xrobo-vr-teleop bash -lc '
    source /opt/conda/etc/profile.d/conda.sh && conda activate xrobo &&
    export LD_LIBRARY_PATH=\$CONDA_PREFIX/lib:\${LD_LIBRARY_PATH:-} &&
    export XROBO_GRIP_THRESHOLD=0.2 XROBO_DEBUG=1 &&
    python -u /workspace/scripts/simulation/teleop_rm65_sim.py \
      --home-joint-deg J1 J2 J3 J4 J5 J6 \
      > /workspace/logs/remote_sim_rm65.log 2>&1
  '"
```

> **提示**：`J1..J6` 必须替换为现场验证无碰撞的侧装 home joint。若 `conda activate xrobo` 后仍报 `ModuleNotFoundError`（容器内有两个 Python），改用 `/opt/conda/envs/xrobo/bin/python` 显式调用，或用一键脚本 `python teleop.py --mode sim --home-joint-deg J1 J2 J3 J4 J5 J6` 代劳。

检查：

```bash
ssh realman "sudo docker exec -i xrobo-vr-teleop bash -lc \
  'grep -E \"control_mode|Viewer URL|Open Placo\" /workspace/logs/remote_sim_rm65.log | tail -3; ss -tlnp | grep 700'"
```

期望：`control_mode=pose`，`Open Placo/Meshcat: http://localhost:18081/static/`，**仅一个** 7000 监听。

### 6.4 头显连接

| 项 | 值 |
|----|-----|
| IP | **远程板** IP：`192.168.0.115` |
| 端口 | 63901 |

数据流：头显 → 远程板 `192.168.0.115:63901` → 容器内 RoboticsService（容器 63901 端口已映射到板子）。

### 6.5 浏览器看仿真（SSH 反代）

**[http://127.0.0.1:18081/static/](http://127.0.0.1:18081/static/)**（SSH 反代）

操作：按住右手 **Grip** 再移动/旋转手柄；对照 Meshcat 中 RM65 关节运动。按 **B** 键平滑复位。

### 6.6 同步代码到远程容器

**推荐：一键同步**（Windows 本机，Git Bash/PowerShell）：

```bash
python teleop.py --mode sync-only
```

手动方式（Windows 路径）：

```bash
rsync -avz --delete --exclude=.git --exclude=__pycache__ --exclude=*.pyc \
  D:/HDU-EILab/vr-teleop-rm65/ realman:/tmp/vr-teleop-rm65/

ssh realman "sudo docker cp /tmp/vr-teleop-rm65/. xrobo-vr-teleop:/workspace/"
```

Windows 若无 `rsync`，可用 `scp -r` 替代，或 WinSCP 上传到 `/home/rm/xrobo/` 后执行 `docker cp`。

### 6.7 真机试跑（仅仿真验收通过后）

```bash
ssh -t realman "sudo docker exec -it xrobo-vr-teleop bash -lc '
  cd /workspace &&
  source /opt/conda/etc/profile.d/conda.sh && conda activate xrobo &&
  python -u scripts/hardware/teleop_realman_rm65_safe_hardware.py \
    --arm-host 192.168.10.18 \
    --scale-factor 1.2 \
    --max-offset-m 0.03 \
    --move-v 5
'"
```

**安全提醒**：真机启动前先跑只读预检（不会发送运动指令）：

```bash
python scripts/hardware/preflight_realman_rm65.py --arm-host 192.168.10.18 --arm-port 8080
```

随后运行 `--dry-run` 检查 XR、UDP、RM65-B 型号与时序；该模式不发送轨迹目标，但退出时仍会发送一次最佳努力的硬停止：

```bash
python scripts/hardware/teleop_realman_rm65_safe_hardware.py --arm-host 192.168.10.18 --dry-run
```

---

## 7. 代码入口与职责

| 文件 | 用途 |
|------|------|
| `teleop.py` | **一键启动**：同步代码 + 选模式 + 建隧道 + 启动 + 清理 |
| `scripts/simulation/teleop_rm65_sim.py` | **仿真主入口**（tyro CLI，含滤波/约束/复位参数） |
| `scripts/hardware/teleop_realman_rm65_safe_hardware.py` | **真机推荐入口**（tyro CLI） |
| `scripts/hardware/teleop_realman_rm65_hardware.py` | 旧版单文件入口；真机运动已禁用，仅保留 dry-run |
| `xrobotoolkit_teleop/hardware/realman_rm65_safe_teleop_controller.py` | 推荐真机控制核心：限幅、UDP 超速保护、停止与复位状态机 |
| `xrobotoolkit_teleop/hardware/realman_rm65_cartesian_teleop_controller.py` | 旧模块化控制器，保留审计/回退参考 |
| `xrobotoolkit_teleop/hardware/interface/realman_rm65.py` | RM65 TCP/JSON 运动与软/硬停止协议层 |
| `xrobotoolkit_teleop/hardware/realman_udp_feedback.py` | RM65 UDP 实时状态接收与单位换算 |
| `xrobotoolkit_teleop/simulation/placo_teleop_controller.py` | Placo 仿真循环（滤波/约束/复位管线） |
| `xrobotoolkit_teleop/simulation/motion_filter.py` | 死区 + EMA 平滑 |
| `xrobotoolkit_teleop/simulation/joint_constraints.py` | 关节限位 + 速度限制 |
| `xrobotoolkit_teleop/common/data_collector.py` | 遥操作数据采集（HDF5） |
| `xrobotoolkit_teleop/utils/geometry.py` | 映射矩阵与 delta pose |
| `config/realman_rm65_side_mount_teleop.json` | 参数快照（参考，未自动加载） |

> Placo IK 真机路径已移除。默认官方 FK + DLS + `movej_follow`；历史 `official_ik_movej_follow` 是另一模式。两者均不能将主机测试当作真机验收。

---

## 8. 下一步工作（接手者优先级）

### 8.1 首要：真机验收（当前阶段）

仿真已优化完成，接下来在真机上做最终验收，逐项核对并记录结果：

| 检查项 | 期望 | 不符合时 |
|--------|------|----------|
| 手柄前/后/左/右/上/下 | 末端沿**侧装坐标系**合理方向运动 | 查 `invert_tcp_xy` 是否需单轴调整，或改侧装矩阵 |
| 旋转手腕 | 末端姿态跟随，无明显乱翻 | 查 Grip 锚点与姿态坐标映射 |
| 平移并旋转手腕 | 末端同步跟随位置与姿态 | 默认 `suppress_rotation_during_translation=False`；若出现危险乱翻，停止测试并显式改为 `True` |
| Grip 松开 | 机械臂停止跟随（`send_stop_on_release=True`） | 查 Grip 阈值参数 |
| 复位按钮 | B 键回到程序启动时记录的 TCP 位姿 | 启动前先确认实体姿态安全，再查 `reset_button` / `reset_duration_s` |

**验收标准**：团队确认真机运动方向与「现场侧装 RM65 的物理直觉」一致后，冻结参数。

### 8.2 后续项

1. 将 `config/realman_rm65_side_mount_teleop.json` 接入 tyro 自动加载，消除手动传参。
2. 补齐 URDF mesh，便于 Meshcat 目视核对外形。
3. 对齐仿真/真机 `scale_factor`（仿真 1.5 vs 硬件 0.25，2026-09-18 真机再收紧后差距扩大）。
4. 先复核官方 FK 与 UDP 工作/工具帧，再做 DLS dry-run 与目标板时序测量。当前阈值使用归一化官方 FK 雅可比，不能沿用旧 URDF 雅可比的阈值；不要为测试主动把真机推入奇异点。
5. 仿真侧接入同一 `hardware/singularity_avoidance.py`（纯 NumPy，无 placo/meshcat 依赖）：`PlacoTeleopController._filter_delta` 按 `speed_scale` 缩放 delta、`_apply_joint_constraints` 用 `joint_gate` 拦往里跳变；需在远程容器内验证后单独记日志。

---

## 9. 常见问题

| 现象 | 原因 | 处理 |
|------|------|------|
| Meshcat 只有网格无机械臂 | 多实例 / 端口错位 | 杀旧进程；隧道 `18081→7000`；浏览器只开 18081 |
| `18081` 无法访问 | 隧道未建或板子离线 | 确认 `ssh rm` 通（宿舍下用 `ssh rm-ts` 强制走 Tailscale），重建 §6.2 |
| grip=0.00 不动 | 头显未开 Send/Controller | 头显里打开并确认 WORKING |
| 仿真 import 报错 | 用了 base Python（3.13） | 用 `/opt/conda/envs/xrobo/bin/python` 或一键脚本 |
| 映射突然错误 | 容器内旧代码 | `docker cp` 后重启仿真 |
| SSH 隧道断开 | 远程重启 / 网络 | 重建隧道 + 重启仿真 |

---

## 10. 源码位置

| 位置 | 路径 |
|------|------|
| 本仓库（Windows 开发机） | `D:\HDU-EILab\vr-teleop-rm65` |
| 远程板子宿主机 | `realman:/home/rm/xrobo` |
| 远程容器内 | `/workspace` |

---

*文档作者环境：Windows。目标运行环境：Linux 板卡 `192.168.0.115` + Docker 容器 `xrobo-vr-teleop`。*
