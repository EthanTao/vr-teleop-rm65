# 单一 MoveIt Servo 控制链

实现基线：Ubuntu 22.04 / ROS2 Humble；RealMan 上游固定提交 `61db3c367e8f5743ee68cee52c5574ec1ac645ca`，见 [dependencies.repos](dependencies.repos)。当前控制链只有 **InputAdapter → PoseTracking → MoveIt Servo → JTC**。无手柄、PICO 和自动验收只切换输入源，仿真与真机只切换设备接口。MuJoCo 与 GenericSystem 的历史证据见[2026-10-26日志](../docs/logs/2026.10.26.md)，本轮三个入口与 manual 输入的验证见[2026-10-27日志](../docs/logs/2026.10.27.md)；软件仿真不替代实体 PICO、板端 ARM64 和真机验收。

## 数据流与职责

```text
浏览器 manual / XR 原生 SDK / CLI mock 自动演示
  → /rm65/xr/raw                   String：设备 JSON + 时间；manual/mock 明确标记为合成输入
  → InputAdapter                  有效性、Grip、侧装映射、实际反馈锚点
  → /rm65/target_pose              PoseStamped：位置(3,)m + 四元数xyzw(4,)
  → Humble PoseTracking            标准位姿误差到速度
  → MoveIt Servo                  IK、平滑、关节界、模型碰撞/奇异处理
  → /rm65_arm_controller/joint_trajectory   六轴 JointTrajectory，rad
  → 标准 JointTrajectoryController
  → ros2_control
      sim: mujoco_ros2_control → MuJoCo 物理关节 → /joint_states
           GenericSystem 仅供 --simulator mock 控制图诊断
      hardware: TopicBasedSystem → 唯一设备准入 → 打补丁的官方 rm_driver
                 ↑ 同包真实 UDP q(6,), qdot(6,), error → 受检反馈
```

Servo 目标周期 20 ms；JTC 内部按 250 Hz 插值，设备桥接仍按名义 50 Hz 取最新目标，没有待发送轨迹队列。这不等于硬实时频率上限。窗口、关节界和速度配置在 [teleop.yaml](src/rm65_servo_teleop/config/teleop.yaml)，JTC 配置在 [controllers.yaml](src/rm65_servo_teleop/config/controllers.yaml)。不再运行 `RM65ControlCore`、自研 DLS/Placo 或每帧完整 action。

输入适配时间、内存复杂度均为 O(1)，固定六轴运输检查亦为 O(1)。Servo 的模型碰撞和求解开销依赖模型及场景，板端 50 Hz、延迟分位数和最坏耗时尚未测量，不能承诺实时期限已达标。

## 主要文件

| 文件 | 职责 |
| --- | --- |
| [launch](src/rm65_servo_teleop/launch/teleop.launch.py) | 同一模型/配置/节点；依次激活反馈广播和 JTC |
| [input_adapter.py](src/rm65_servo_teleop/rm65_servo_teleop/input_adapter.py)、[xr_node.py](src/rm65_servo_teleop/rm65_servo_teleop/xr_node.py) | 纯输入规则、ROS 时间/TF/状态检查 |
| [manual_input.py](src/rm65_servo_teleop/rm65_servo_teleop/manual_input.py)、[manual_node.py](src/rm65_servo_teleop/rm65_servo_teleop/manual_node.py) | 仿真专用人工输入、短期输入租约与 HTTP 状态；只发布标记 XR，不直接生成关节指令 |
| [pose_tracking_node.cpp](src/rm65_servo_teleop/src/pose_tracking_node.cpp) | 复用上游 PoseTracking，补齐目标/反馈过期、Grip与线程生命周期 |
| [model.py](src/rm65_servo_teleop/rm65_servo_teleop/model.py) | 官方 URDF/SRDF、历史 TCP 候选、设备接口 |
| [transport_node.py](src/rm65_servo_teleop/rm65_servo_teleop/transport_node.py)、[transport_guard.py](src/rm65_servo_teleop/rm65_servo_teleop/transport_guard.py) | 最新指令、真实反馈、准入、停止确认与恢复 |
| [驱动补丁](patches/README.md) | 原生唯一执行入口、代次隔离、同包反馈、SDK 串行和 watchdog |
| [validation.py](src/rm65_servo_teleop/rm65_servo_teleop/validation.py) | 现场检查项及模型/配置/补丁/驱动/IP 哈希绑定 |
| [journal_node.py](src/rm65_servo_teleop/rm65_servo_teleop/journal_node.py) | 有界异步原始记录，不阻塞控制循环 |
| [hand_session.py](src/rm65_servo_teleop/rm65_servo_teleop/hand_session.py)、[hand_protocol.py](src/rm65_servo_teleop/rm65_servo_teleop/hand_protocol.py) | 默认关闭的六通道任务流程、协议及独立只读预检 |
| [peripherals.launch.py](src/rm65_servo_teleop/launch/peripherals.launch.py) | 不启动机械臂的独立手驱动与D435i采集；受现场配置门控的真实写入、全文件记录校验，见[外设说明](../docs/外设驱动与采集.md) |
| [test_servo_launch.py](src/rm65_servo_teleop/test/test_servo_launch.py) | 真实 ROS 图集成测试；不生成假关节反馈 |

## 构建与测试

Windows 从根目录用 `python teleop.py --mode test`，或在有 Docker 的 Linux 根目录执行：

```bash
docker compose -f docker/compose.moveit.yml build
docker run --rm --network none rm65-servo:humble bash -c 'source /opt/rm65_ws/install/setup.bash && cd /opt/rm65_ws && colcon test --merge-install --packages-select rm65_servo_teleop --event-handlers console_direct+ && colcon test-result --verbose'
```

Dockerfile 获取固定上游，校验提交、应用补丁、按 x64/ARM64 放置官方 SDK 库，编译依赖及本包。本次 x64 实际编译通过；SDK mock 测试不替代硬件/ABI 运行验收。ROS 基础镜像 digest 已固定，实际安装包清单保存在镜像 `/opt/rm65-environment-packages.tsv`；APT 依赖仍随官方仓库更新，清除缓存重建时需重新验收。

默认未安装 XR 原生 SDK；`--input pico` 的启动器将构建参数切到 ON。测试容器无网络/设备映射，不启动 `rm_driver`。默认仿真为 MuJoCo 动力学反馈；不能验证实体停止、真实接触力、网络延迟或采集同步。物理参数见 [mujoco.yaml](src/rm65_servo_teleop/config/mujoco.yaml)，当前未实机辨识，假设理想重力补偿。

Windows 根目录可从 `python 启动.py` 进入统一菜单，也保留三个直接入口：

| 入口 | 路由 |
| --- | --- |
| `python 启动无手柄仿真.py` | `teleop.py --mode sim --target wsl --input manual` |
| `python 启动PICO仿真.py` | 同一 sim/WSL 入口，输入改为 pico |
| `python 启动真机.py` | 交互终端无参进入真机子菜单；显式参数走既有 remote hardware 链，正式执行仍须现场参数与人工准入 |

三入口共用 [Python 启动编排](../scripts/setup/launcher.py)，无手柄入口明确 manual；仅实际执行 PICO 模式准备本项目 TCP63901 转发，专用 PowerShell 网络助手保留。`--check-only` 只离线预览，不启动 WSL、不枚举网卡或提权。CLI mock 保留一次约15秒的小幅自动演示后释放 Grip，用于离线演示/验收，不是第四个双击入口；CLI 省略输入仍默认 mock。

manual 服务位于 [http://127.0.0.1:8766](http://127.0.0.1:8766)，启动器仅打印地址，待就绪后自行点击。Space + W/S、A/D、R/F 对应 ±X/Y/Z，Space + J/L、I/K、U/O 对应 ±RX/RY/RZ，或按住页面方向按钮。方向在 `base_link` 中定义；输入目标速率20 mm/s和0.12 rad/s，单段偏移10 cm/0.16 rad到限后须松开重按。松开、失焦、页面隐藏和断线撤销操作，恢复后不自动续动。控制台显示 MuJoCo 实际 q/dq、反馈 TCP、目标、误差、状态和数据年龄；实际仿真反馈不等于实体测量。详见[操作手册](../docs/仿真操作手册.md#2-无手柄控制台)。

PICO 的 PC Service 由容器直接管理前台进程，等待 TCP63901 和本地 gRPC60061 都就绪后启动控制图；服务意外退出会结束当前图。SDK 异步 Init 成功不等于头显已连接，只有新鲜合法输入与 Grip 条件均满足才允许模拟跟随。

本机 WSL 默认只打开 MuJoCo，RViz 关闭；需要 TF/模型/目标调试时，用 `python teleop.py --mode sim --input manual --target wsl --rviz` 显式开启。该选项仅支持 WSL sim，不能与 `--no-viewer` 同用。`--no-viewer` 关闭 MuJoCo 窗口和 Foxglove，manual 控制台仍由输入节点提供。GenericSystem 诊断如需窗口，也须显式加 `--rviz`。ROS 直接 launch 的 `rviz` 默认 false。

## 模式与硬件边界

- `sim`：同一 InputAdapter/PoseTracking/Servo/JTC，默认执行端为 MuJoCo；`--simulator mock` 使用 GenericSystem。允许明确标记的 manual/mock XR。
- MuJoCo 接入：同一修正URDF生成MJCF，有限力矩位置执行器；ROS控制保持wall time、物理名义1倍速，独立 `/clock` 进度保护在暂停/倒退时结束会话。记录保存引擎与模型哈希，场景暂仅机械臂/地面。
- `hardware-dry-run`：实际连接官方 SDK 读反馈，原生运动权限关闭，不创建执行服务和命令订阅；桥接层亦不创建执行客户端。
- `hardware`：只有现场检查文件、部署哈希、明确操作口令均通过才给原生执行权限；反馈、停止和 Grip 仍独立门控。

manual 和 mock 均禁止用于 hardware / hardware-dry-run。`python 启动真机.py` 在交互终端无参进入子菜单，非交互无参列缺项后退出；正式执行必须显式 `--udp-ip`、有效 `--hardware-validation` 与人工 `--live-ack "RM65 LIVE VALIDATION"`，不会生成或批准现场证据。`--check-only` 仅验证参数并预览；`--dry-run` 仍须显式 `--udp-ip`，实际启动会连接真实控制器读反馈。原有 `.cmd` / PowerShell 入口保留参数兼容转发，旧真机入口无参仍退出 2。

握持新段始于实测 TCP。故障后先释放 Grip，反馈新鲜无错、六轴静止且停止确认满足后才重使能。原生代次屏蔽停止前在途指令；桥断连与 UDP 超时由原生 watchdog 覆盖。

当前模型含机械臂碰撞网格，不含完整的手、相机支架、工件和现场障碍；默认禁止以此批准真机。TCP 名义速度界来自目标误差截断及标准比例增益，实际 TCP 速度、加速度/jerk 无物理验证或硬保证。当前不实现自动 Home。

可选 `hand_mode:=mock` 仅允许 sim，复用同一 launch，默认 off；手节点只发布 `/rm65/hand/status` 和 `events`。右手柄 Trigger 抓枪、A 确认握稳，松 Grip 后长按 B 释放；不做逐指遥控，首次长按 B 明确准备张手。也可通过带会话/序号/阶段版本的 `command` 单步操作。journal 按 hand_mode 检查手记录覆盖；mock 不含接触物理，也不改变臂的准入。用法和真实接入缺项见[灵巧手接入](../docs/灵巧手接入.md)。

上游依据：[Humble Servo 文档](https://moveit.picknik.ai/humble/doc/examples/realtime_servo/realtime_servo_tutorial.html)、[Humble PoseTracking 源码](https://github.com/moveit/moveit2/blob/humble/moveit_ros/moveit_servo/src/pose_tracking.cpp)、[RealMan ROS2](https://github.com/RealManRobot/ros2_rm_robot/tree/humble)。
