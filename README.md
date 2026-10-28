# RM65 VR 遥操作

无手柄控制台 / PICO → **InputAdapter → PoseTracking → MoveIt Servo → JointTrajectoryController（JTC）→ MuJoCo / RM65 官方驱动**。输入方式和设备接口可切换，运动控制逻辑只有一条；原 Placo、官方 IK 和 shared-dls 已退出入口和安装依赖。

**机器人断电时，Windows 本机通过 WSL2 运行同一套 ROS 仿真。** 新环境为 D 盘的 `RM65-Servo`（Ubuntu 22.04 + Docker Engine），不需要 Docker Desktop。真机仍由板端 Linux 接入；两者共用控制代码。部署与验证状态见[本机仿真](docs/本机仿真部署.md)。

## 开始使用

Windows 需要 Python 3.10+、PyYAML。项目根目录提供三个明确入口：

| 入口 | 用途 |
| --- | --- |
| [启动无手柄仿真.cmd](启动无手柄仿真.cmd) | MuJoCo + 浏览器键盘/按钮控制，不需要头显或机器人 |
| [启动PICO仿真.cmd](启动PICO仿真.cmd) | 真实 PICO 输入，执行端仍为本机 MuJoCo |
| [启动真机.cmd](启动真机.cmd) | 板端原有控制链；无参拒绝，要求现场参数和人工准入 |

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

自动演示保留为 CLI `python teleop.py --mode sim --input mock`，运行一次约15秒的小幅运动后释放 Grip；它用于离线演示和自动验收，不提供第四个双击入口。CLI 省略 `--input` 时仍默认为 mock；双击无手柄入口明确选择 manual。PICO 模式按终端显示地址连接，入口仅为该模式准备专用局域网转发。

真机入口必须显式提供 `-UdpIp`、`-HardwareValidation` 和人工 `-LiveAck "RM65 LIVE VALIDATION"`；`-CheckOnly` 只预览。无参双击会列出缺项并退出，不连接、同步或部署。验证模板不能批准运动；Dry Run 仍会连接真实控制器读取反馈。现场流程见[真机安全与故障处理](docs/真机安全与故障处理.md)。

远程板的现有工作区是 `/home/rm/EthanTao`，已运行 ARM64 真机只读会话。远程查看、自检和只读启动使用[远程板环境](docs/远程板环境.md)中的入口；复用当前会话无需重新同步或构建。

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

真机默认拒绝执行，需匹配部署版本的现场验证文件。腕部 D435i 和 OYMotion ROH-LiteS001 已登记安装；手已有六通道离线流程和默认只预览的只读预检工具，真实控制、采集、同步和标定仍待接入。目前保存的是控制链原始记录，还不是完整训练数据集。
