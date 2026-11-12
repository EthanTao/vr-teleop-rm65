# PICO 手柄 → 双版本 MuJoCo 4 浏览器仿真

两版使用相同 PICO SDK 接收器、InputAdapter、侧装映射、Grip 及共享模型；每次选择一版启动。页面只有查看功能，没有键盘/按钮运动、自动轨迹或自动回位。此入口不启动真实驱动，不连接机械臂，也不会切换板端正式控制链。

## 启动

Windows 项目根目录，使用现有 WSL2 `RM65-Servo` / Docker 环境：

```powershell
# 可先加 --check-only，仅离线打印命令；不启动 WSL、不读取网卡或配置网络
python start.py pico --browser --controller official --check-only
python start.py pico --browser --controller servo --check-only

# 实际运行：二选一，结束前一会话后再换另一版
python start.py pico --browser --controller official
python start.py pico --browser --controller servo
```

也可运行 `python start.py`，在“仿真实验”选择“官方 IK · PICO”或“MoveIt Servo · PICO”。两张卡片都提供实际启动及离线预览。服务就绪后打开 **http://127.0.0.1:8094/**；橙色球是手柄映射的目标 TCP，青色球和机械臂来自 MuJoCo 物理反馈。鼠标只调整视角。

PICO 与 Windows 电脑接入可互通局域网；在 XRoboToolkit 中按启动日志显示的 Windows LAN 地址连接 TCP63901，选择 Controller / Send。先松开右 Grip，等待“Grip 已松开”，再握住连续跟随；松开结束本段。每段以当前 TCP 和手柄位置重建起点。多网卡时加 `--listen-address <本机LAN地址>`，发行版更名时加 `--distro`。

SDK 启动、输入断流、重复/回退时间戳、跟踪质量或 TF/反馈异常仍沿现有输入规则撤销跟随；恢复后先释放 Grip。网页自身不生成目标，也不提供发送、暂停、复位或上传 API。结束使用 Ctrl+C，或桌面“结束当前会话”。再次启动重建模拟初始状态，不能在运行中热切控制器。

CLI 的 `--rviz`、`--no-viewer` 不适用于此浏览器入口。旧 `python start.py pico` 仍是原生 MuJoCo 3.12 Servo 仿真；本页的两版必须加 `--browser`。无手柄 `start.py sim`、manual/mock 输入和官方 `--demo` 均保持移除。

## 两版实际数据流

| 版本 | 目标到物理反馈 |
| --- | --- |
| 官方 IK | 现有映射目标 → `OfficialFollower` 限幅/官方 Algo IK/候选检查 → 私有 `/rm65/websim/joint_commands` → MuJoCo 4 执行器 → 实际 q/dq、TCP → 下一周期 |
| MoveIt Servo | 现有映射目标 → 原 `rm65_pose_tracking` / MoveIt Servo → 原 JTC → TopicBasedSystem 私有模拟关节话题 → 同一 MuJoCo 4 执行器 → 实际 q/dq → JTC/Servo |

Servo 没有被替换为另一个求解器。官方 IK 没有因仿真另写 IK，也未接入 Servo 的碰撞/奇异保护；两者保护能力和真实执行准入仍不同，详见[双版本说明](双版本跟随说明.md)。模型、d6、工具帧、关节范围/速度和有限力矩执行器参数沿用共享配置，参数仍未经实体辨识或标定。

Servo浏览器启动时，先等待`joint_state_broadcaster`收到与共享初始关节值一致的新鲜物理反馈，再激活JTC。这样可避免TopicBasedSystem尚未收到反馈时以默认全零状态启动控制器；没有通过关闭奇异保护来绕过该问题。

MuJoCo 4 的 Python 包要求 Python 3.11+，ROS2 Humble 的接口使用 Python 3.10。独立 `rm65-websim:4.0` 镜像从固定digest的官方Python镜像提取独立3.11.11运行时，安装MuJoCo Core 4.0.0；物理worker通过有界私有管道连接3.10 ROS桥。原ROS Python和原3.12 C++物理插件保持原状；浏览器版使用TopicBasedSystem，不加载3.12的MuJoCo插件。页面是项目的只读Three.js查看器，物理和控制在本机Linux执行，不宣称使用官方Studio WebViewer或浏览器内WASM物理。

物理以 1 ms 子步执行，每次接口请求推进 4 ms。关节反馈来自 `mj_step`，仅初始化使用 keyframe，不把关节候选直接写入 qpos。已有指令超过 200 ms、管道断开或响应超时会结束模拟链，不继续执行旧目标；任一关键节点退出也会结束整张图。这些检查不构成硬实时保证。

## 环境与验证边界

首次启动会构建带 PICO SDK 的原 Servo 镜像和独立浏览器镜像；Python、MuJoCo、官方 SDK 及网页依赖下载需要网络，之后 Docker 复用缓存。RealMan 算法包固定提交和 Git blob 哈希、Three.js 0.180.0 压缩包 SHA-256由安装脚本核对；不把厂商二进制提交仓库。第三方许可在镜像资源目录保留。

Docker 仅向局域网开放 PICO TCP63901，网页映射到宿主回环地址8094；没有机械臂 UDP8089、宿主网络、硬件设备或特权映射。两版使用同一个容器名、端口和独立 ROS domain，同一时间只允许一个会话。

离线物理、ROS 闭环、SDK 服务启动、真实 PICO 手柄及实体机械臂必须分别验收。本轮用户暂不方便操作 PICO，先完成代码和隔离验证；未将合成测试输入当成真实手柄操作。详细验证记录见[当日日志](logs/2026.11.12.md)。
