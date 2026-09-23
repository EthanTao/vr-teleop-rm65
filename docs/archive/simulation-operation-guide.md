# RM65 VR 遥操作仿真 — 操作手册

> **写给谁看**：第一次接触这套系统的操作者。你不需要懂代码、不需要懂机器人学，按步骤来就能上手。
>
> **这是什么**：用 VR 手柄（PICO 头盔）远程控制一个六轴机械臂的仿真环境。你在虚拟空间里移动手柄，仿真里的机械臂就会跟着动。

---

## 目录

1. [你需要什么](#1-你需要什么)
2. [整体结构](#2-整体结构)
3. [第一步：打通网络（SSH 隧道）](#3-第一步打通网络ssh-隧道)
4. [第二步：启动仿真](#4-第二步启动仿真)
5. [第三步：连接头显](#5-第三步连接头显)
6. [第四步：查看仿真画面](#6-第四步查看仿真画面)
7. [操作说明](#7-操作说明)
8. [调参指南](#8-调参指南)
9. [常见问题](#9-常见问题)
10. [参数速查表](#10-参数速查表)

---

## 1. 你需要什么

| 物品 | 说明 |
|------|------|
| **PICO 4 Ultra 头显** + 手柄 | 戴头上，手拿两个手柄 |
| **一台电脑**（Windows/Mac 均可） | 用来连远程服务器、开浏览器看仿真画面 |
| **远程服务器**（已部署好） | 实际上代码跑在远程 Linux 板子上，你的电脑只是"遥控器" |
| **WiFi 网络** | 头显和电脑在同一个局域网 |
| **SSH 客户端** | Windows 自带 OpenSSH（PowerShell/cmd 直接 `ssh`）；Mac 自带终端 |

> **💡 为什么不是直接在自己电脑上跑？** 仿真环境需要 Linux + 显卡 + 专用库，这些已经在一台远程板子上搭好了。你的电脑只需要连过去「看画面 + 发指令」就行。

---

## 2. 整体结构

```
你戴着 PICO 头显
      │  WiFi
      ▼
你的电脑
      │  SSH 隧道（远程通道）
      ▼
远程服务器 → Docker 容器（仿真在这里跑）
                  │
                  ▼
          Meshcat 3D 画面 ← 你打开浏览器看这里
```

三个窗口你要同时开着：

```
┌─────────────────┐  ┌─────────────────┐  ┌─────────────────┐
│  ① SSH 隧道窗口  │  │  ② 仿真控制台    │  │  ③ 浏览器画面    │
│  （后台保持就好） │  │  （跑仿真日志）   │  │  （看机械臂）    │
│                  │  │                  │  │                  │
│  ssh -N -L ...   │  │  export XXX...   │  │  http://127.0.0.1│
│                  │  │  python teleop... │  │  :18081/static/  │
└─────────────────┘  └─────────────────┘  └─────────────────┘
```

---

## 3. 第一步：打通网络（SSH 隧道）

> SSH 隧道就像在你家和远程服务器之间修了一条"专用传送带"——远程的仿真画面通过它传到你浏览器，手柄发出的指令通过它传到仿真程序。

### 3.1 打开终端

- **Windows**：按 `Win + R`，输入 `powershell`，回车
- **Mac**：打开「终端」App

### 3.2 执行 SSH 隧道命令

```bash
ssh -N -L 18081:127.0.0.1:7000 -L 63901:127.0.0.1:63901 rm@192.168.0.115
```

> 如果提示 `ssh: command not found`，说明 Windows 没装 OpenSSH。可以改用 PuTTY（见 [9. 常见问题](#9-常见问题)）。

这条命令执行后**看起来什么都没有发生**——没有输出就是最好的状态。**不要关掉这个窗口**，让它保持运行。

如果提示输入密码，输入远程服务器的密码（输入时看不到字符是正常的）。

> **💡 类比**：这就像你给电视机插上了 HDMI 线——线插好了，但电视还没开。这条隧道就是那根 HDMI 线。

---

## 4. 第二步：启动仿真

### 4.1 新开一个终端窗口

保持隧道窗口开着别动，再打开一个新的终端/PowerShell。

### 4.2 SSH 登录远程服务器

```bash
ssh rm@192.168.0.115
```

### 4.3 进入 Docker 容器

```bash
sudo docker exec -it xrobo-vr-teleop /bin/bash
```

看到提示符变成类似 `root@xxx:/workspace#` 就说明进到容器里了。

> **💡 类比**：远程服务器是一栋楼，Docker 容器是楼里的一个房间。我们要到房间里操作。

### 4.4 设置环境变量并启动

```bash
export XROBO_GRIP_THRESHOLD=0.2
python scripts/simulation/teleop_rm65_sim.py --home-joint-deg J1 J2 J3 J4 J5 J6
```

默认使用 Placo IK。如果容器已经安装与 RM65-B 匹配的睿尔曼 API2 Python SDK，并希望在仿真中先检查真机候选 IK 链路，使用：

```bash
python scripts/simulation/teleop_rm65_sim.py \
  --home-joint-deg J1 J2 J3 J4 J5 J6 \
  --ik-backend official
```

此时 Grip 锚点来自睿尔曼官方 FK，关节目标来自 `rm_algo_ik_remote`，Placo 只负责加载 URDF 并在 Meshcat 显示关节构型。出现 `[OFFICIAL IK HOLD]` 表示本周期 IK 失败、关节越限、J3 进入默认 ±5° 排除区或关节步长过大，仿真会保持上一帧关节角。官方模式暂不支持 Placo 的 `--enable-self-collision-avoidance true` 和 `--enable-velocity-limit true`；关节步长应使用 `--official-ik-max-joint-speed-ratio` 调整。

看到类似这样的输出，说明启动成功：

```
[INFO] control_mode=pose
[INFO] deadband_m=0.002, deadband_rad=0.03
[INFO] 平滑复位已启用
Open Placo/Meshcat: http://localhost:18081/static/
```

> **⚠️ 注意**：如果报 `ModuleNotFoundError`，说明代码还没同步到容器。需要先更新代码（见 [9. 常见问题](#9-常见问题)）。

---

## 5. 第三步：连接头显

### 5.1 确认远程板的 IP

远程板就是跑仿真的那台服务器，默认 IP 是 `192.168.0.115`。

如果不确定，SSH 登录后查看：

```bash
ssh rm@192.168.0.115
ip addr | grep "inet 192.168"
# 找到类似 192.168.x.x 的局域网地址，就是远程板 IP
```

### 5.2 在 PICO 头显上设置

1. 戴好头显，打开 **XRoboToolkit** App
2. 找到设置页面，填入：
   - **IP**：远程板的 IP（就是 5.1 确认的那一个，默认 `192.168.0.115`）
   - **端口**：63901
3. 点击 **Connect**
4. 确保 **Controller + Send** 状态显示 **WORKING**

> 数据流：头显 → 远程板 `192.168.0.115:63901` → 容器内 PC Service。头显**直连远程板**，不需要填你电脑的 IP。

---

## 6. 第四步：查看仿真画面

打开你电脑上的浏览器，访问：

```
http://127.0.0.1:18081/static/
```

> 这是通过 SSH 隧道转发到本地的仿真画面。你应该能看到一个 3D 机械臂模型。

---

## 7. 操作说明

### 7.1 基本操作

| 手柄操作 | 效果 |
|---------|------|
| **按住右手 Grip 键**（握把，食指和中指扣住） | 激活控制，此时移动手柄，机械臂会跟着动 |
| **松开 Grip** | 暂停控制，手柄再动不影响机械臂 |
| **移动右手柄**（前后左右上下） | 机械臂末端平移 |
| **旋转右手柄**（转动手腕） | 机械臂末端旋转 |
| **按 B 按钮**（右手柄上方） | **平滑复位** — 仅在配置安全 Home 后生效，约 3 秒 |

> 不要把六轴 `0°` 当作侧装 RM65 的安全姿态。官方将 `[0,0,0,0,0,0]` 列为边界奇异示意点。先在现场确认候选姿态无碰撞且远离奇异点，再将六个关节角（J1～J6，单位：度）显式传入；未传入时仿真入口会拒绝启动。关节范围和奇异点说明见 [RM65 本体参数与遥操作安全含义](rm65-ontology-parameters.md)。

```bash
python scripts/simulation/teleop_rm65_sim.py --home-joint-deg J1 J2 J3 J4 J5 J6
```

> **💡 Grip 是什么？** 手柄侧面有一块可以"捏"的感应区域，就像扣扳机或握拳的动作。轻轻握住它就开始操控。

### 7.2 操作技巧

| 你想做什么 | 怎么做 |
|-----------|--------|
| **刚开始接触，怕搞坏** | 先不按 Grip，随便移动手柄，看画面上的机械臂不动（正常！按了 Grip 才动） |
| **精细调整位置** | Grip 不要捏太紧，小幅移动手柄，机械臂会跟着小幅度移动 |
| **快速移动到远处** | 捏紧 Grip，大幅度移动手柄，机械臂会跟着大范围移动 |
| **机械臂姿势乱了** | 按一下 B 按钮，它会自动回到初始位置 |
| **复位过程中想取消** | 松口气等着，复位完成后就能继续操作 |

---

## 8. 调参指南

### 8.1 什么叫「手感」？

这就像调节鼠标灵敏度：

| 参数 | 像什么 | 调大 | 调小 |
|------|--------|------|------|
| **死区（deadband）** | 鼠标的「忽略微小移动」 | 手轻微抖动被忽略（更稳） | 微小移动也响应（更灵敏） |
| **平滑系数（alpha）** | 鼠标的「加速度」 | 反应快但可能抖 | 动作平滑但感觉迟钝 |

### 8.2 运行时调参（推荐）

启动时加上 `--enable-tuning true`，就可以在仿真运行中实时调参：

```bash
export XROBO_GRIP_THRESHOLD=0.2
python scripts/simulation/teleop_rm65_sim.py --home-joint-deg J1 J2 J3 J4 J5 J6 --enable-tuning true
```

在运行仿真的终端窗口里按键盘：

| 按键 | 调整的参数 | 每次变化 |
|------|-----------|---------|
| **小写 d** | 平移死区 **减**小 | -0.001m（更灵敏） |
| **大写 D** | 平移死区 **增**大 | +0.001m（更抗抖） |
| **小写 r** | 旋转死区 **减**小 | -0.005 rad |
| **大写 R** | 旋转死区 **增**大 | +0.005 rad |
| **小写 a** | 平移平滑系数 **减**小 | -0.05（更平滑但变慢） |
| **大写 A** | 平移平滑系数 **增**大 | +0.05（反应更快但可能抖） |
| **小写 o** | 旋转平滑系数 **减**小 | -0.05 |
| **大写 O** | 旋转平滑系数 **增**大 | +0.05 |
| **小写 v** | 关节速度上限 **减**小* | -0.5 rad/s |
| **大写 V** | 关节速度上限 **增**大* | +0.5 rad/s |
| **h** | 显示帮助和当前参数 | — |
| **q** | 退出调参模式 | — |

> *需要开启 `--enable-velocity-limit true` 才能用 v/V。

### 8.3 启动时直接设置

如果你已经知道要用的值，也可以启动时直接指定：

```bash
# 更抗抖（适合新手）
python scripts/simulation/teleop_rm65_sim.py \
  --home-joint-deg J1 J2 J3 J4 J5 J6 \
  --deadband-m 0.005 \
  --deadband-rad 0.05 \
  --smooth-alpha-pos 0.25 \
  --smooth-alpha-rot 0.2

# 更灵敏（适合老手）
python scripts/simulation/teleop_rm65_sim.py \
  --home-joint-deg J1 J2 J3 J4 J5 J6 \
  --deadband-m 0.001 \
  --deadband-rad 0.01 \
  --smooth-alpha-pos 0.5 \
  --smooth-alpha-rot 0.4
```

### 8.4 自碰撞避免

想让机械臂永远不会「自己打自己」，加这个参数：

```bash
python scripts/simulation/teleop_rm65_sim.py --home-joint-deg J1 J2 J3 J4 J5 J6 --enable-self-collision-avoidance true
```

默认安全间距 3cm，可调节：

```bash
python scripts/simulation/teleop_rm65_sim.py \
  --home-joint-deg J1 J2 J3 J4 J5 J6 \
  --enable-self-collision-avoidance true \
  --self-collision-margin 0.05   # 5cm，越大越保守
```

> 这个功能会让仿真更真实，但也可能让 IK 求解器偶尔报错。如果感觉机械臂动作不流畅，可以关掉。

### 8.5 推荐上手参数

```
阶段 1（先体验默认值）：
  --enable-tuning true
  （跑起来，按 h 看看当前值，感受一下）

阶段 2（觉得抖了）：
  在终端按几下大写 D → 死区变大，抖动减少
  再按几下大写 A → 平滑加强，更柔顺

阶段 3（觉得反应迟钝）：
  按小写 d → 死区减小
  按小写 a → 平滑减弱

阶段 4（找到舒服的平衡点）：
  记下按 h 显示的值，下次启动时直接用 --deadband-m xxx 指定
```

---

## 9. 常见问题

### Q: SSH 连不上远程服务器

```
ssh: connect to host 192.168.0.115 port 22: Connection timed out
```

原因：远程服务器关机了、网络断了、或者 IP 变了。

解决：联系管理员确认远程服务器状态。IP 变更时替换命令中的 `192.168.0.115` 为新 IP。

### Q: Windows 没有 ssh 命令

```
'ssh' is not recognized as an internal or external command
```

解决方式一（推荐）：在 PowerShell 中安装 OpenSSH：

```powershell
Add-WindowsCapability -Online -Name OpenSSH.Client~~~~0.0.1.0
```

解决方式二：下载 [PuTTY](https://www.putty.org/)：

1. 打开 PuTTY
2. Host Name 填 `192.168.0.115`
3. Connection → SSH → Tunnels：
   - `Source port: 18081` → `Destination: 127.0.0.1:7000` → Add
   - `Source port: 63901` → `Destination: 127.0.0.1:63901` → Add
4. 点 Open 登录

### Q: 浏览器打开 http://127.0.0.1:18081/static/ 显示空白

可能原因和解决办法：

| 原因 | 解决 |
|------|------|
| SSH 隧道没开 | 确认窗口还在跑，关掉重开一次 |
| 仿真没启动 | 检查控制台输出是否报错 |
| 端口冲突 | 换一个端口，如 `-L 18082:127.0.0.1:7000`，浏览器打开 `http://127.0.0.1:18082/static/` |
| 容器内 Meshcat 端口不对 | 在容器里执行 `ss -tlnp \| grep 700` 确认监听端口 |

### Q: 手柄按了 Grip 但机械臂不动

| 原因 | 解决 |
|------|------|
| 头显没连上 | 检查 XRoboToolkit App 状态是否为 WORKING |
| Grip 阈值太高 | 设 `export XROBO_GRIP_THRESHOLD=0.2`（已设就不用管） |
| 正在复位中 | 等复位完成再操作 |
| 隧道断了 | 检查第一个终端窗口，重建隧道 |

### Q: 怎么知道当前参数值？

开启 `--enable-tuning true`，在仿真终端窗口中按 **h** 键，会显示所有参数当前值。

### Q: 机械臂疯狂抖动

```
deadband 太小 + 手抖 + 手柄噪声 → 抖动放大
```

在调参终端中多按几次 **大写 D**（增大平移死区）+ **大写 A**（增大平滑系数）。

### Q: 代码改了但仿真里没变化

远程容器里跑的是旧代码。需要同步：

```bash
# 在你的电脑上（本地）把文件传到远程板子
scp xrobotoolkit_teleop/simulation/*.py rm@192.168.0.115:/tmp/

# 登录到板子，拷进容器
ssh rm@192.168.0.115
sudo docker cp /tmp/motion_filter.py xrobo-vr-teleop:/workspace/xrobotoolkit_teleop/simulation/
sudo docker cp /tmp/joint_constraints.py xrobo-vr-teleop:/workspace/xrobotoolkit_teleop/simulation/
sudo docker cp /tmp/placo_teleop_controller.py xrobo-vr-teleop:/workspace/xrobotoolkit_teleop/simulation/
sudo docker cp /tmp/teleop_rm65_sim.py xrobo-vr-teleop:/workspace/scripts/simulation/
```

然后重新启动仿真。

### Q: 调参模式在 Windows 下按键盘没反应

运行时调参（`--enable-tuning`）依赖 Linux 的键盘监听机制，在 Windows 的 PowerShell/CMD 里**按了也没用**。

解决办法：
1. 在远程服务器的容器里运行仿真（`docker exec -it` — 带 `-it` 才行）
2. 或者启动时直接用 CLI 参数指定数值，不用运行时调参

---

## 10. 参数速查表

下面是仿真软件参数，不是 RM65 本体额定参数。官方关节范围、最大角速度、工作半径和 MDH 参数见 [RM65 本体参数与遥操作安全含义](rm65-ontology-parameters.md)。其中 610/627 mm 是本体工作半径，不能直接替代仿真的关节约束、碰撞检查或真机安全边界。

| 参数 | 默认值 | 作用 | 推荐范围 |
|------|--------|------|----------|
| `--deadband-m` | 0.002 | 平移死区（米） | 0.001~0.01 |
| `--deadband-rad` | 0.03 | 旋转死区（弧度） | 0.01~0.1 |
| `--smooth-alpha-pos` | 0.35 | 平移 EMA 平滑系数 | 0.1~0.6 |
| `--smooth-alpha-rot` | 0.3 | 旋转 EMA 平滑系数 | 0.1~0.6 |
| `--scale-factor` | 1.5 | 手柄移动→机械臂移动的放大比 | 1.0~3.0 |
| `--follow-orientation` | true | 是否跟随手柄旋转 | true/false |
| `--enable-velocity-limit` | false | 是否限制关节角速度 | true/false |
| `--max-joint-velocity` | 3.0 | 关节速度上限（rad/s） | 1.0~5.0 |
| `--reset-duration-s` | 3.0 | B 按钮复位时长（秒） | 0.5~3.0 |
| `--enable-self-collision-avoidance` | false | 是否开启自碰撞避免 | true/false |
| `--self-collision-margin` | 0.03 | 自碰撞安全间距（米） | 0.02~0.08 |
| `--enable-tuning` | false | 是否开启运行时调参 | true/false |

### 环境变量

| 变量 | 默认值 | 作用 |
|------|--------|------|
| `XROBO_GRIP_THRESHOLD` | 0.5 | Grip 激活阈值，仿真推荐设为 **0.2** |
| `XROBO_DEBUG` | 空 | 设为 `1` 可打印手柄数据调试信息 |

---

> **最后提醒**：第一次上手时，建议先不按 Grip 随便动动手柄，熟悉一下操作感。然后轻握 Grip 小幅移动，再慢慢加大动作幅度。按 B 按钮只会返回启动时明确提供且已验证的 Home Pose，不会自动计算安全姿态。
