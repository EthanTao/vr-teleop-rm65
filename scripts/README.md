# 脚本索引

日常从根目录运行 `python 启动.py`，进入[统一菜单](../启动.py)：1 无手柄仿真、2 PICO 仿真、3 连接真机（只读）、4 正式执行、5 高级设置、0 退出。[启动无手柄仿真.py](../启动无手柄仿真.py)、[启动PICO仿真.py](../启动PICO仿真.py)、[启动真机.py](../启动真机.py)仍可直接运行，现有显式 Python CLI 选项不变。菜单只编排现有入口，不恢复旧控制器；运动链仍是 **InputAdapter → PoseTracking → MoveIt Servo → JTC**。仿真默认只打开 MuJoCo，RViz 关闭。

无手柄入口明确选择 manual，终端打印 [http://127.0.0.1:8766](http://127.0.0.1:8766)，待服务就绪后自行点击，不自动打开浏览器。键盘 Space + W/S、A/D、R/F 控制 ±X/Y/Z，Space + J/L、I/K、U/O 控制 ±RX/RY/RZ；也可按住页面方向按钮。目标速率20 mm/s、0.12 rad/s，单段10 cm/0.16 rad上限须松开重按；失焦或断线撤销。实际 q/dq、TCP、目标和状态见控制台，完整操作见[手册](../docs/仿真操作手册.md#2-无手柄控制台)。

需要查看 TF、模型或目标时，用 `python teleop.py --mode sim --input manual --target wsl --rviz` 按需打开 RViz；它不计算物理或执行运动保护。`--rviz` 只支持 WSL sim，不能与 `--no-viewer` 同用；后者关闭 MuJoCo 窗口和 Foxglove，不移除 manual 控制台。

| 脚本 | 用途 |
| --- | --- |
| [setup/menu.py](setup/menu.py) · [根目录启动.py](../启动.py) | 标准库交互菜单；编排仿真、板端只读和受验证约束的正式执行，不直接接入旧控制器 |
| [setup/launcher.py](setup/launcher.py) | Python 参数与启动编排；支持 `--distro`、PICO 的 `--listen-address`，交互终端真机无参进入子菜单，非交互无参退出 2，`--check-only` 仅离线预览 |
| [setup/start_simulation.ps1](setup/start_simulation.ps1) · [setup/start_real_robot.ps1](setup/start_real_robot.ps1) | 旧 PowerShell 参数兼容层；根目录三个同名 `.cmd` 继续转发到它们，再调用 Python，不包含控制或准入实现 |
| [setup/connect_pico_wsl.ps1](setup/connect_pico_wsl.ps1) | 准备本项目的局域网转发；只迁移具有所有权记录的规则 |
| [setup/install_docker_wsl.sh](setup/install_docker_wsl.sh) | 在专用 Ubuntu 22.04 WSL 中安装 / 检查 Docker |
| [tools/inspect_moveit_environment.py](tools/inspect_moveit_environment.py) | 本机或 SSH 只读环境核查，不连接机械臂、不启动节点 |
| [tools/board_environment.py](tools/board_environment.py) · [setup/board_environment.sh](setup/board_environment.sh) | 已部署到板端EthanTao；隔离自检、真实状态及头显输入采样、完整PICO真机只读启动，详见[远程板环境](../docs/远程板环境.md) |
| [tools/board_xr_environment.py](tools/board_xr_environment.py) · [setup/board_xr_environment.sh](setup/board_xr_environment.sh) | 保留的SDK独立输入诊断工具；当前SDK已在主会话运行，此工具拒绝重复启动，日常使用上行environment工具 |
| [tools/measure_manual_tracking.py](tools/measure_manual_tracking.py) | 在无网络Linux仿真容器中，通过manual HTTP测实际跟踪与停止；仅开发测量用途 |
| [tools/analyze_manual_tracking.py](tools/analyze_manual_tracking.py) | 源时间因果对齐、误差/收敛/重复性/停止指标及曲线；`--strict`未达标返回失败 |
| [tools/check-lan-reachable.ps1](tools/check-lan-reachable.ps1) | SSH 配置引用的网络探测；路径保持稳定 |
| [tools/check_repository.py](tools/check_repository.py) | 统一开发检查：源码/配置、安装清单、导航链接、Ruff 与本机 pytest；不启动控制环境 |
| [tools/check_robot_readiness.py](tools/check_robot_readiness.py) | 板端只读预检并保存报告；不部署、不使能运动，现场使用见[操作清单](../docs/真机现场操作清单.md) |
| [tools/make_advisor_docx.py](tools/make_advisor_docx.py) | 保留的历史研究文档生成器，输出旧 `docs/research/` 路径，非运行依赖 |

只读预览，不启动 WSL / Docker / ROS，也不配置网络：

```powershell
python 启动.py --check-only # 仅打印菜单与连接说明，不提示输入、不查询环境或启动进程
python 启动无手柄仿真.py --check-only
python 启动PICO仿真.py --check-only
```

已有自动化继续使用以下参数：`start_simulation.ps1 -InputSource manual\|pico\|mock -Distro RM65-Servo -CheckOnly`（PICO 可指定 `-ListenAddress`）；`start_real_robot.ps1 -DryRun -UdpIp <板端地址> -CheckOnly`。旧真机入口无参仍退出 2；`-SshHost`、`-RemoteDir`、`-ArmIp`、`-HardwareValidation`、`-LiveAck` 继续按原名传递。兼容层不缓存准入结果，也不补全操作确认。

mock 自动演示保留为 CLI `python teleop.py --mode sim --input mock`，没有独立快捷入口；CLI 省略输入仍默认为 mock，无手柄 Python 入口明确选择 manual。只有实际执行 PICO 仿真入口才读取局域网接口和准备转发；专用 PowerShell 网络助手保留，控制程序不提权。诊断日志见 `build/logs/startup/`，`--pause` 仅在交互终端等待按键。

真机无参在交互终端显示子菜单：1 只读、2 反馈检查、3 PICO 输入检查、4 正式执行、5 高级设置、0 退出；非交互无参仍退出 2。正式执行仍要求 `--udp-ip`、有效 `--hardware-validation` 和人工 `--live-ack "RM65 LIVE VALIDATION"`，菜单同样要求现场验证与手动输入确认，不自动 ACK 或回 Home。`--check-only` 只预览；`--dry-run` 须显式提供 `--udp-ip`，实际启动仍连接真实控制器读状态。现场验证模板不能当执行批准，详见[真机文档](../docs/真机安全与故障处理.md)。

选只读连接或查询即使用默认 `rm@192.168.0.115`、目录 `/home/rm/EthanTao`，不再询问地址；反馈网口默认 `192.168.10.100`，控制器默认 `192.168.10.18`。选 5 才修改这四项（仅本次菜单有效；主机可填 SSH 别名 `rm`），所有字段验证通过才整体采用，不修改板端。选 4 只输入现场验收文件及人工确认，留空或参数有误会返回菜单，模板仍不能批准运动。板端只读只调用 [setup/board_environment.sh](setup/board_environment.sh) 的 `start-readonly` 与 `status`；不要求手柄、头显或灵巧手，复用匹配会话，拒绝冲突会话，不同步、重建、重启或增加第二发送者。PICO 真机链路是**头显 → 板端 Wi-Fi TCP 63901 → 机械臂**，Windows 电脑仅负责启动与观察，不使用本机仿真的 WSL 转发链路。

开发检查在仓库根执行 `python -B -m pytest -q`。实际 ROS 检查使用 `python teleop.py --mode test`，验证范围见[当日日志](../docs/logs/2026.10.27.md)。

仿真快捷入口支持在 MuJoCo 窗口内按 R 结束旧控制图并重建会话；重复按键只请求一次，重连后先松 Grip。真机菜单高级设置的控制器/反馈地址会传给只读助手并核对已有会话；旧版板端助手不支持自定义地址时会拒绝参数，需要更新助手后使用。本次 Git 更新不自动部署板端。

旧 `simulation/`、`hardware/`、`debug/` 以及 Placo / Meshcat 安装和启动工具已移入[源码归档](../docs/archive/2026-10-26-legacy-control/README.md)。当前操作从[首页](../README.md)进入。
