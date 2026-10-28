# EthanTao 远程板环境

2026-10-27 实查：SSH 别名 `rm` 可达，工作区为 `/home/rm/EthanTao`。
宿主是 Jetson / Ubuntu 20.04 / ARM64，ROS2 Humble 与 Python 3.10 运行在 Ubuntu 22.04 容器中。
当前镜像为 `rm65-servo:humble`，记录写入工作区 `build/experiments/`。

当前运行 `hardware-dry-run + external`，读取机械臂 `192.168.10.18` 的真实状态，
反馈目的地址为板端 `192.168.10.100:8089`。它会配置 UDP 推送，但不创建运动执行接口。
此镜像的 XR SDK 选项为 OFF；远程查看不要求头显。PICO 接入和真实运动另行验收。

## 检查与启动

在 Windows PowerShell 执行，脚本已安装在远端工作区：

```powershell
# 环境自检：隔离容器、无网络/设备，不连接机器人
ssh rm '/home/rm/EthanTao/environment.sh check'
# 检查当前容器，并采样四秒真实反馈
ssh rm '/home/rm/EthanTao/environment.sh status'
# 持续查看六轴角度/角速度；Ctrl+C结束查看
ssh -t rm '/home/rm/EthanTao/environment.sh state'
# 需要查看日志时
ssh -t rm '/home/rm/EthanTao/environment.sh logs'
# 当前没有运行会话时，明确启动只读模式；会连接真实控制器
ssh rm '/home/rm/EthanTao/environment.sh start-readonly'
```

`start-readonly` 复用匹配的运行中只读会话；遇到其他模式、其他工作区、停止的同名容器、
地址变化或端口占用时拒绝，不停止、删除或重启原会话。当前运行会话无需再次启动。
服务未配置开机自动连接。板子重启后，检查网络再明确执行 `start-readonly`。

`check` 验证 ROS 包、官方驱动动态库以及 MuJoCo C++/Python 版本一致性；
`status` 验证真实反馈包号前进和源年龄，不用节点存活或重复时间戳代替反馈新鲜性。
这些检查不验证帧/TCP 标定、运动性能、停止接口或实体运动。

## 图形查看

```powershell
ssh -N -o ExitOnForwardFailure=yes -L 127.0.0.1:8765:127.0.0.1:8765 rm
```

保留该窗口，在 Foxglove 中创建 Foxglove WebSocket 连接，地址为 `ws://127.0.0.1:8765`。
Raw Messages 查看 `/rm65/transport/joint_states` 和 `/rm65/transport/status`。
此地址不是 HTTP 网页。Foxglove 账户权限以其当前产品要求为准；终端查看无此依赖。
相机图像没有接入，不能以关节/模型显示当作现场视频。

本轮已实测 Windows → SSH → 板端8765的 WebSocket 握手通过，服务版本3.6.0，
协商协议为 `foxglove.sdk.v1`。自写检测客户端只提供旧 `foxglove.websocket.v1` 时返回400；
使用当前协议后返回101，无需修改或重启服务。这是连接检查，尚未验证 Foxglove 界面的模型显示。

10月28日已打开独立的本机三维窗口，订阅现有真实反馈，只做正向运动学绘图，
不执行仿真步进或发送运动。当前控制器只读查询的安装角为 `[0°, -90°, 0°]`，
已用于该窗口的侧装显示；生产控制配置没有改动。安装高度、世界原点和TCP仍未标定。
窗口标记LIVE/STALE，断流后保留最后实测姿态并标记过期；它没有实验室相机视频。
这是当前会话的临时查看工具，记录见[10月28日日志](../logs/2026.10.28.md)，不代表PICO或实体运动已验收。

## 配置与留存

工具源文件为 `scripts/tools/board_environment.py` 和 `scripts/setup/board_environment.sh`；
远端根目录的 `environment.sh` 指向后者。部署清单、自检及反馈采样保存在
`build/environment/`，本机核验记录保存在 `build/validation/remote-board-20261027/`。
原板端专用 `docker/Dockerfile.moveit.jetson` 原样留存；本轮没有重装宿主、重建镜像或重启当前会话。

未来通过 `teleop.py` 同步/构建时，明确传 `--remote-dir EthanTao`；
它会重新构建并尝试创建会话，不是仅连接现有查看服务的命令。板端镜像当前已可用，
上述检查和只读启动入口不依赖 Windows 再次同步。


## 2026-10-28 板端头显 SDK 准备

原 `rm65-servo` 仍使用 XR SDK=OFF 的旧只读镜像，容器没有重启。另备 `rm65-servo:xr-ready`（ARM64、SDK=ON），并运行轻量 `rm65-xr-input`：只启动官方 PC Service 和原生 `xr_sdk_node`。它复用原容器的网络端口，但使用独立 ROS domain 86，没有输入适配器、Servo/JTC 或机器人驱动，因此不会驱动真机或现有只读模型跟随。没有新增 UDP 接收者，也没有宿主安装或自启动服务。

```powershell
ssh rm 'bash /home/rm/EthanTao/scripts/setup/board_xr_environment.sh status'
ssh rm 'bash /home/rm/EthanTao/scripts/setup/board_xr_environment.sh sample'
ssh rm 'bash /home/rm/EthanTao/scripts/setup/board_xr_environment.sh logs'
```

`status` 检查 63901/60061 监听、SDK进程、镜像源码标记和官方包摘要；`sample` 仅订阅5秒原始输入，不发布任何命令。没有头显时 `valid_frames=0` 正常。`start` 只复用匹配会话，名称冲突或原只读会话不匹配时拒绝；`stop` 仅关闭并移除经角色、工作区、SDK及命令检查确认的这个输入服务，不操作机器人容器。

板端本次查询的 Wi-Fi 地址为 **192.168.0.115**，头显连接 PC Service 的 TCP 端口 **63901**；头显须在可访问该地址的实验室网络。`192.168.10.100` 是机械臂侧有线反馈地址；远程维护另用 Tailscale `100.127.90.0`。Wi-Fi DHCP地址到现场重新核对：`ssh rm 'ip -4 -brief addr show'`。

验证边界：官方v1.0.0 ARM64服务、动态库及ROS原生节点已经编译；带显式synthetic标记的884帧经真实服务/SDK进入输入适配器，最终准备镜像复测Grip保持断流后约221ms关闭跟随（无控制器/驱动，反馈为测试夹具；先前候选218ms）。另经Windows→Tailscale→板端63901→原生SDK收到611帧，源时间戳前进。后者是通路检查，不是50Hz端到端无丢帧或真实头显验收。完整模拟控制图与原机器人图同时运行时曾因TF超过200ms而停止；不放宽该保护，也不在6核板上长期并跑两套控制图。

本轮两端源码摘要为 `888539f173d18f1836ec1c0472f76ff6faadeec09bae80a675a7b2c49be5a7bd`。板端准备镜像 `xr-prepared-20261028` / `xr-ready` 的ID为 `sha256:8e11b490c534826f2847f93997d2f2273e59c223f20c3e2f109757b27316d64c`；原只读容器继续使用原镜像。SDK输入服务不设开机自启，重启板子后先核对只读会话，再显式执行上述脚本的 `start`。

### 到实验室后的切换顺序

1. 保持现有只读状态，确认头显应用、网络、真实原始输入及Grip/追踪状态。`sample` 能读到真实帧不等于坐标、时延、跟随和停止验收完成。
2. 现场人员确认后，关闭SDK输入服务，再关闭当前只读机器人容器，切换到新版 **PICO + hardware-dry-run**。严禁并行启动第二个UDP接收者。此处只记录顺序，远程准备期间没有执行这一步。
3. 现场将准备好的 `xr-ready` 镜像设为该次会话的 `rm65-servo:humble` 后，现行启动器会核对源码摘要和SDK=ON；匹配则复用已构建镜像。源码变动或SDK标记不匹配仍会重建。
4. 使用现行入口预览参数：`powershell -File scripts/setup/start_real_robot.ps1 -DryRun -RemoteDir EthanTao -UdpIp 192.168.10.100 -CheckOnly`。去掉 `-CheckOnly` 会连接控制器读取状态，但仍不创建运动接口。使用真实执行模式前，必须完成[现场准入](../真机安全与故障处理.md)，不能把示例检查项改为true。
5. 核对侧装base、d6、TCP、附件/场景、UDP单位及固件；随后做有监护和急停的低速跟随、松Grip、断流、停止和恢复测试。显示窗口采用控制器安装角，不代替现场标定。

仿真跟踪及停止筛查的最新结论见[跟踪验收](../仿真跟踪验收.md)和[当日日志](../logs/2026.10.28.md)。SDK已准备不表示真机可直接执行。
