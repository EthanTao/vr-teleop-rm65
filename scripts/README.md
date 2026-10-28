# 脚本索引

Windows 根目录只有三个日常入口：`启动无手柄仿真.cmd`、`启动PICO仿真.cmd`、`启动真机.cmd`，没有选择菜单。它们都进入 `teleop.py`，共用 **InputAdapter → PoseTracking → MoveIt Servo → JTC**，只切换输入与设备接口。仿真默认只打开 MuJoCo，RViz 关闭。

无手柄入口明确选择 manual，终端打印 [http://127.0.0.1:8766](http://127.0.0.1:8766)，待服务就绪后自行点击，不自动打开浏览器。键盘 Space + W/S、A/D、R/F 控制 ±X/Y/Z，Space + J/L、I/K、U/O 控制 ±RX/RY/RZ；也可按住页面方向按钮。目标速率20 mm/s、0.12 rad/s，单段10 cm/0.16 rad上限须松开重按；失焦或断线撤销。实际 q/dq、TCP、目标和状态见控制台，完整操作见[手册](../docs/仿真操作手册.md#2-无手柄控制台)。

需要查看 TF、模型或目标时，用 `python teleop.py --mode sim --input manual --target wsl --rviz` 按需打开 RViz；它不计算物理或执行运动保护。`--rviz` 只支持 WSL sim，不能与 `--no-viewer` 同用；后者关闭 MuJoCo 窗口和 Foxglove，不移除 manual 控制台。

| 脚本 | 用途 |
| --- | --- |
| [setup/start_simulation.ps1](setup/start_simulation.ps1) | 仿真启动；默认 manual，也接受 pico / mock，支持 `-Distro`、PICO 的 `-ListenAddress` |
| [setup/start_real_robot.ps1](setup/start_real_robot.ps1) | 远端真机入口，默认工作区EthanTao；无参拒绝，`-CheckOnly` 只预览 |
| [setup/connect_pico_wsl.ps1](setup/connect_pico_wsl.ps1) | 准备本项目的局域网转发；只迁移具有所有权记录的规则 |
| [setup/install_docker_wsl.sh](setup/install_docker_wsl.sh) | 在专用 Ubuntu 22.04 WSL 中安装 / 检查 Docker |
| [tools/inspect_moveit_environment.py](tools/inspect_moveit_environment.py) | 本机或 SSH 只读环境核查，不连接机械臂、不启动节点 |
| [tools/board_environment.py](tools/board_environment.py) · [setup/board_environment.sh](setup/board_environment.sh) | 已部署到板端EthanTao；隔离自检、真实状态及头显输入采样、完整PICO真机只读启动，详见[远程板环境](../docs/远程板环境.md) |
| [tools/board_xr_environment.py](tools/board_xr_environment.py) · [setup/board_xr_environment.sh](setup/board_xr_environment.sh) | 保留的SDK独立输入诊断工具；当前SDK已在主会话运行，此工具拒绝重复启动，日常使用上行environment工具 |
| [tools/measure_manual_tracking.py](tools/measure_manual_tracking.py) | 在无网络Linux仿真容器中，通过manual HTTP测实际跟踪与停止；仅开发测量用途 |
| [tools/analyze_manual_tracking.py](tools/analyze_manual_tracking.py) | 源时间因果对齐、误差/收敛/重复性/停止指标及曲线；`--strict`未达标返回失败 |
| [tools/check-lan-reachable.ps1](tools/check-lan-reachable.ps1) | SSH 配置引用的网络探测；路径保持稳定 |
| [tools/make_advisor_docx.py](tools/make_advisor_docx.py) | 保留的历史研究文档生成器，输出旧 `docs/research/` 路径，非运行依赖 |

只读预览，不启动 WSL / Docker / ROS，也不配置网络：

```powershell
.\启动无手柄仿真.cmd -CheckOnly
.\启动PICO仿真.cmd -CheckOnly
```

mock 自动演示保留为 CLI `python teleop.py --mode sim --input mock`，不提供第四个双击入口；CLI 省略输入仍默认为 mock，PowerShell 仿真脚本则默认 manual。只有 PICO 读取局域网接口和准备转发。

真机入口要求显式 `-UdpIp`、有效 `-HardwareValidation` 和人工 `-LiveAck "RM65 LIVE VALIDATION"`，无参关闭且不连接。`-CheckOnly` 只预览；`-DryRun` 须显式提供 UdpIp，实际启动仍连接真实控制器读状态。现场验证模板不能当执行批准，详见[真机文档](../docs/真机安全与故障处理.md)。

开发检查在仓库根执行 `python -B -m pytest -q`。实际 ROS 检查使用 `python teleop.py --mode test`，验证范围见[当日日志](../docs/logs/2026.10.27.md)。

旧 `simulation/`、`hardware/`、`debug/` 以及 Placo / Meshcat 安装和启动工具已移入[源码归档](../docs/archive/2026-10-26-legacy-control/README.md)。当前操作从[首页](../README.md)进入。
