# 项目工作规范

PICO + XRoboToolkit → 侧装 RealMan RM65。**2026-11-12 用户要求保留 Servo/官方 IK 两版、删除全部无手柄入口，随后要求两版都接 PICO 并在 MuJoCo 4 浏览器查看。** 现有真机仍为Servo，官方IK不接真实发送。两版浏览器入口为`python start.py pico --browser --controller servo|official`，共用真实PICO输入及MuJoCo 4物理反馈；网页只查看，无自动轨迹。见[双版本浏览器仿真](docs/PICO浏览器双版本仿真.md)及[双版本说明](docs/双版本跟随说明.md)。原`teleop.py` → `servo_launcher.py` → `teleop.launch.py`继续保留。

Windows 本机仿真采用 WSL2 `RM65-Servo`（Ubuntu22.04 + Docker Engine），用户已授权必要时清旧环境后重建；旧 Ubuntu20.04 已备份并清理。ROS2 Humble 仍在同一 Docker 镜像内运行，真机使用板端执行。当前验证以[本机部署页](docs/本机仿真部署.md)和工作日志为准。

## 按任务读取

| 任务 | 主文档 |
| --- | --- |
| 架构、接口、配置、构建 | [ROS 工作空间](ros2_ws/README.md) |
| 仿真与 PICO | [本机部署](docs/本机仿真部署.md) · [仿真操作手册](docs/仿真操作手册.md) |
| 真机接入、停止和恢复 | [真机安全与故障处理](docs/真机安全与故障处理.md) |
| 当前环境与验证状态 | [项目交接](docs/项目交接.md) |
| 数据记录 | [实验记录](docs/真机实验记录.md) |
| 历史坐标、参数及已推翻结论 | [参数与坐标系](docs/RM65参数与坐标系.md) · [历史决策](docs/决策与结论汇总.md#pitfalls) |

## 必须遵守

1. **唯一运动发送者，保留两版源码。** 现行Servo保持PoseTracking/Servo/JTC。官方IK独立核心用于回放、shadow和用户授权的PICO物理仿真；新的`official_sim_node.py`仅能发布私有`/rm65/websim/joint_commands`，不得连接真实驱动话题。原shadow仍只发诊断。每次只启动一版，不能同时接入真实发送；真实接管仍需独立验收。不恢复归档旧DLS。
2. **同一配置与明确物理版本。** 两版浏览器共用模型/输入/安全/执行器配置和MuJoCo 4.0.0，Servo保留同一Servo/JTC，官方版复用同一官方IK核心，不添加仿真专用IK。MuJoCo4使用独立Python3.11 worker，经私有管道连接ROS3.10；原3.12 C++物理插件不加载到浏览器版。原生Servo仿真保留3.12 / mujoco_ros2_control；GenericSystem仅诊断，真机为受控TopicBasedSystem。动力学与接触参数未经实机辨识，模拟反馈不批准真实执行。
3. **帧未验证不得上机。** 当前 d6、finger TCP 来源于历史快照，安装位姿仍需现场核对；与官方 FK / UDP TCP 对比。默认竖立显示不等于真实侧装方向已经标定。
4. **新鲜反馈来自真实 UDP 包。** 禁止用缓存重打时间戳；要求同帧 q/qdot/错误、包标识和源会话。官方旧 `/joint_states` 仅作诊断，不能代替该依据。
5. **唯一发送者。** 厂商补丁关闭旧运动入口；原生执行权限默认 false。Dry Run 不创建执行接口。真实使能、停止确认、反馈超时和故障锁存均须有记录。
6. **保护有边界。** Servo 模型碰撞检测已配置，手/相机/工件几何仍未完整建模；无硬加速度/jerk 保证。配置目标误差与比例增益给出名义 TCP 速度界，不能替代物理测速。
7. **不自动回位。** 启动、松 Grip、故障及 B 键都不发 Home 指令。未来需要 Home 时走统一命令仲裁并单独验收路径，禁止恢复旧直发分支。
8. **硬件事实。** ROH-LiteS001 无独立力传感器，电流不等于实测接触力；手采用张开/抓枪预设，不做逐指遥控。已有 Trigger/A/B mock、独立 RS485 读写驱动和只读预检；真实写入由未完成的现场配置锁住。D435i已有独立RGB-D采集和全文件校验，时间配对仅标为到达时间候选。2026-10-29两者源码与独立ARM64外设镜像已部署并通过隔离验证，真实设备保持关闭；实际USB/协议/动作、同步、标定和手/臂并发均待现场验证，见[外设说明](docs/外设驱动与采集.md)。
9. **保留已有未提交工作。** 旧 `xrobotoolkit_teleop`、测试、模型与启动脚本已移出工作目录，当前字节及SHA-256清单保存在 `docs/archive/2026-10-26-legacy-control/`；整理前工作树本机快照在 `build/archive/pre-repository-cleanup-20261026/`。历史快照仅供查阅，不恢复为第二运行入口。不要顺手删除其他研究和数据文件。
10. **代码修改必须记日志。** 在 `docs/logs/<年.月.日>.md` 同日追加：文件、原因、行为、证据、未完成项。区分本机测试、ROS 编译、仿真、硬件只读、真实发送与实体运动。
11. **授权范围逐项核对。** 用户已追加授权将灵巧手/相机驱动及采集环境部署板端；2026-10-29已核对身份、源码差异及镜像，完成ARM64隔离验证并恢复唯一PICO真机只读会话，最新摘要与回退见[远程板环境](docs/远程板环境.md)；真实手动作、实体采集与标定仍留待现场，不从仿真推导现场验收。
12. **失败时保持关闭。** SDK、反馈、TF、输入、驱动或记录异常，不自动放宽阈值或恢复旧链；停止请求成功与实体静止分别确认。

## 本机检查

```powershell
python -B -m pytest -q
python teleop.py --mode sim --print-command
```

新测试由 `pyproject.toml` 指定，旧测试不再是现行控制栈的验收。ROS 集成测试在缺少 ROS 的 Windows 会明确跳过；Linux 端执行 `python teleop.py --mode test`，不是用本机通过数替代。
