# 厂商驱动接入补丁

仅针对 `dependencies.repos` 固定的官方提交 `61db3c367e8f5743ee68cee52c5574ec1ac645ca`。补丁把该可执行程序收敛为 Servo 专用设备接口，不提供第二套 IK 或轨迹插值。

原生参数 `allow_servo_motion` 默认 `false`。此时只保留真实反馈发布；不创建命令订阅或停止/使能服务，退出也不发送停止指令。启动仍需建立 SDK 连接并配置 UDP 上报，这属于反馈接入配置。只有 `hardware` 模式通过现场验证文件后，launch 才能把该参数设为 `true`；`hardware-dry-run` 必须保持 `false`。

厂商旧 MoveJ/MoveL、CANFD、示教、力控、手、升降及配置命令的订阅均不创建。旧回调的定义留在 vendor 源码中，不构成 ROS 入口；四个旧 CANFD 回调还会直接返回。断连不会自动删除/重建正在使用的 SDK handle：故障锁存、关流并尝试硬停，须重启会话。具有运动权限的进程析构时，会先关流并同步请求硬停，再删除 handle，避免只依赖已经退出的 ROS 服务。

所有新增名称均相对于全局 namespace，沿用 `rm_driver/` 前缀；launch 将其 remap 到 `/rm65/driver/`。

| 相对接口 | 类型 | 语义 |
| --- | --- | --- |
| `rm_driver/feedback_snapshot` | `std_msgs/String` | 只从真正的 SDK UDP 回调发布同帧位置、速度、错误、使能状态、session、包号和接收时间 |
| `rm_driver/servo_send_result` | `std_msgs/String` | SDK 返回码、generation、序号；不表示实测到达目标 |
| `rm_driver/servo_joint_command` | `sensor_msgs/JointState` | 仅具有运动权限时创建；六轴名称与位置为 rad，`header.frame_id` 为使能返回的十进制 generation，stamp 为真实系统时间 |
| `rm_driver/servo_slow_stop` | `std_srvs/Trigger` | 先关流、增加 generation，再在 SDK 互斥锁内调用官方软停 |
| `rm_driver/servo_hard_stop` | `std_srvs/Trigger` | 同上，调用官方硬停 |
| `rm_driver/servo_enable_stream` | `std_srvs/Trigger` | 只在停止已确认且真实反馈新鲜、无错误、已使能、六轴≤1°/s时打开新一段流 |

三个服务只在 `allow_servo_motion=true` 时创建，响应 message 为严格 JSON：`{"api_code":0,"generation":1,"stream_enabled":false}`；成功使能的 `stream_enabled` 为 `true`。SDK 失败或过期 generation 不会恢复流。原生 20 ms watchdog 在命令/真实反馈超过 0.2 s、反馈故障或发布失败时关流并请求硬停；它与命令、停止共享 SDK 互斥锁。

bridge 启动时保持关闭，先等首个真实新鲜快照及三个服务都就绪，再开始有界停止握手。它不会把厂商至少 2 s 的初始化时间误算成停止超时。正常松 Grip 软停；软停失败/超时升级硬停；未确认停止不解锁。SDK 速度使用固定版本 `rm_define.h` 的 °/s 语义转换，原始值同时记录；仍须在现场核实单位及固件行为。

prepare_vendor.py 在应用前核对提交，并执行 git apply --check。补丁成功应用只证明文本与源版本匹配；不能代替 SDK 编译、真实反馈单位、停止效果或现场安全验证。

`test/test_native_contract.py` 从实际补丁抽取新 header，以 mock ROS/SDK 编译并测试默认只读、generation、防重放、并发停止、watchdog、断连锁存和析构停机。它验证门控逻辑，不能证明 ROS ABI、SDK 二进制链接或真实停止效果。
