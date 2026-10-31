# RM65 VR 遥操作

无手柄控制台 / PICO → **InputAdapter → PoseTracking → MoveIt Servo → JointTrajectoryController（JTC）→ MuJoCo / RM65 官方驱动**。输入方式和设备接口可切换，运动控制逻辑只有一条；原 Placo、官方 IK 和 shared-dls 已退出入口和安装依赖。

**机器人断电时，Windows 本机通过 WSL2 运行同一套 ROS 仿真。** 新环境为 D 盘的 `RM65-Servo`（Ubuntu 22.04 + Docker Engine），不需要 Docker Desktop。真机仍由板端 Linux 接入；两者共用控制代码。部署与验证状态见[本机仿真](docs/本机仿真部署.md)。

## 开始使用

Windows 需要 Python 3.10+。**日常运行 `python start.py`，打开 RM65 Studio 桌面控制台**：左侧切换仿真实验、真机连接和连接设置，下方集中显示运行日志。界面采用开源 CustomTkinter，仍调用现有 Servo 链；窗口打开时不连接设备。

| 命令 | 用途 |
| --- | --- |
| `python start.py` | [统一桌面入口](start.py)：仿真启动、真机只读/查询、正式执行、连接设置和日志 |
| `python start.py --cli` | 原终端交互菜单；`--check-only` 仅打印菜单和说明 |
| `python start.py sim` | 无手柄仿真：MuJoCo + 浏览器键盘/按钮 |
| `python start.py pico` | PICO 仿真：真实手柄输入，本机 MuJoCo 执行 |
| `python start.py real` | 真机子菜单；非交互无参仍拒绝启动，显式参数与原准入条件相同 |

首次安装桌面组件：`python -m pip install -e ".[desktop]"`。之后从项目根目录运行 `python start.py`，也可在正确的 Python 文件关联下双击 `start.py`。窗口中的“结束当前会话”向本次任务发送中断并等待退出，进程结束不代表实体停止已确认。桌面使用说明与开源选型见[桌面控制台](docs/桌面控制台.md)，启动日志保存在 `build/logs/startup/*.log`。

按用户最新要求，已移除中文命名入口、旧 `.cmd` 和两个 PowerShell 快捷启动脚本，统一使用 `start.py`。网络助手 `connect_pico_wsl.ps1` 保留；`teleop.py` 继续承担底层控制栈命令行入口。

通过两个仿真模式启动后，聚焦 **MuJoCo 仿真窗口**并按 **R** 可重建仿真初始状态。旧控制图先完整退出，再启动新会话；重连后先松开 Grip。终端和浏览器内的 R 不触发该复位，Backspace 也不是此功能；真机不提供该复位。

无手柄仿真启动后，点击终端显示的 [http://127.0.0.1:8766](http://127.0.0.1:8766) 打开控制台；入口不自动打开浏览器。按住 **Space + W/S、A/D、R/F** 平移，**Space + J/L、I/K、U/O** 旋转，或持续按住页面方向按钮。松开、失焦和断线撤销操作；完整按键、段限位和反馈说明见[操作手册](docs/仿真操作手册.md#2-无手柄控制台)。

已注册 `RM65-Servo` 时，sim/test 自动使用本机 WSL，读取当前仓库并复用构建缓存；首次构建需要网络。下列命令共用同一 `teleop.py` 入口：

```powershell
python -m pip install PyYAML
python teleop.py --mode sim --input manual --print-command # 仅预览
python teleop.py --mode sim --input manual                # 无手柄控制台
python teleop.py --mode sim --input pico                  # PICO 输入
python teleop.py --mode test                              # Linux 隔离验收
python teleop.py --mode sim --input manual --target wsl --rviz # 可选调试窗口
```

仿真默认只打开 MuJoCo 场景，RViz 关闭，不启动机器人驱动。RViz 用于查看模型、TF 和目标，按需加 `--rviz`；该选项只支持 WSL sim，不能与 `--no-viewer` 同用。`--no-viewer` 关闭 MuJoCo 窗口和 Foxglove，manual 输入的控制台仍由输入节点提供。

MuJoCo 复用同一机械臂模型、Servo 和 JTC，增加重力、有限力矩的位置执行器及接触计算；手、相机支架和充电枪场景仍待建模，动力学参数未实机辨识。Foxglove `ws://127.0.0.1:8765` 保留作备选。真实 PICO 的本机网络要求见[部署页](docs/本机仿真部署.md)；新版本不使用 Meshcat，不自动回 Home，也未提供 B 键回位。

**2026-10-27跟踪性能筛查未通过，暂不进入真机运动。** 三次重复均有明显响应滞后；松开后能停稳，但多数固定目标在6秒内未到位。量化结果、曲线及后续要求见[仿真跟踪验收](docs/仿真跟踪验收.md)。

自动演示保留为 CLI `python teleop.py --mode sim --input mock`，运行一次约15秒的小幅运动后释放 Grip；它用于离线演示和自动验收，没有独立快捷入口。CLI 省略 `--input` 时仍默认为 mock；无手柄 Python 入口明确选择 manual。PICO 仿真按终端显示地址连接，入口仅为该仿真模式准备专用局域网转发，Windows 网络规则仍由专用 [网络助手](scripts/setup/connect_pico_wsl.ps1) 管理。

`--check-only` 仅离线检查和命令预览，不启动 WSL、不枚举网卡、不请求提权；PICO 地址未指定时提示执行时选择。现有显式 Python CLI 选项不变：真机正式执行仍要求 `--udp-ip`、有效的 `--hardware-validation` 和人工 `--live-ack "RM65 LIVE VALIDATION"`；菜单也必须取得同样的现场验证与手动输入确认，不自动 ACK，不自动回 Home。板端位置及控制器地址可用 `--host`、`--remote-dir`、`--arm-ip` 指定。验证模板不能批准运动；`--dry-run` 仍会连接真实控制器读取反馈。Ctrl+C 退出码为 130，仅表示中断当前会话，不证明实体已经停止。现场流程见[真机安全与故障处理](docs/真机安全与故障处理.md)。

**日常选 3 即连接真机读反馈，不再逐项询问参数。** 菜单默认使用 `rm@192.168.0.115`、远程目录 `/home/rm/EthanTao`、板端反馈网口 `192.168.10.100`、控制器 `192.168.10.18`；仅换网络或目录时选 5 修改，本次菜单内生效，不写入磁盘或板端。SSH 主机也可填 `rm` 使用现有别名。选 4 只询问现场验收文件和人工确认，留空返回菜单；缺少记录不能启用运动，不会自动转成其他模式。板端只读项不以手柄、头显或灵巧手为启动前提，仅调用现有 [板端环境脚本](scripts/setup/board_environment.sh) 的 `start-readonly` 与 `status`。匹配会话直接复用，冲突会话拒绝操作；不自动同步、重建、重启或增加第二发送者。**PICO 真机连接为：头显 → 板端 Wi-Fi TCP 63901 → 机械臂；Windows 电脑仅负责启动与观察**，不是头显输入的真机中继。

远程板的现有工作区是 `/home/rm/EthanTao`。2026-10-30 到场后已恢复 LAN SSH（`rm@192.168.0.115`），确认唯一 ARM64 真机只读会话和真实 UDP 反馈正常；最近检查仍无真实 PICO 输入，运动写入关闭，尚未通过实体运动验收。到实验室按[现场操作清单](docs/真机现场操作清单.md)逐项执行；`python -B scripts/tools/check_robot_readiness.py --host rm` 可保存只读预检报告。远程查看、自检和只读启动见[远程板环境](docs/远程板环境.md)，复用匹配会话无需重新同步或构建。

| 任务 | 文档 |
| --- | --- |
| 仿真、查看、按键与故障排查 | [仿真操作手册](docs/仿真操作手册.md) |
| 真机只读接入与现场验证 | [真机安全与故障处理](docs/真机安全与故障处理.md) |
| 架构、接口与源码 | [ROS 工作空间](ros2_ws/README.md) |
| 目录用途、历史源码与数据位置 | [仓库结构](docs/仓库结构.md) |
| Windows / Linux 分工、硬件和当前证据 | [项目交接](docs/项目交接.md) |
| 原始记录与数据采集缺项 | [实验记录](docs/真机实验记录.md) |
| 灵巧手预设抓取、Trigger/A/B 操作及只读预检 | [灵巧手接入](docs/灵巧手接入.md) |
| 灵巧手驱动、D435i采集、外设配置与文件校验 | [外设驱动与采集](docs/外设驱动与采集.md) |
| 开发规范、修改证据 | [CLAUDE.md](CLAUDE.md) · [工作日志索引](docs/logs/README.md) |
| 开发环境、提交检查与持续集成 | [贡献指南](CONTRIBUTING.md) |

真机默认拒绝执行，需匹配部署版本的现场验证文件。腕部 D435i 和 OYMotion ROH-LiteS001 已登记安装；手已有六通道离线流程和默认只预览的只读预检工具，真实控制、采集、同步和标定仍待接入。目前保存的是控制链原始记录，还不是完整训练数据集。
