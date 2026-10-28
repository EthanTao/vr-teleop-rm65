# 本机历史标定快照

现行控制与物理配置在 [ROS 包 config](../ros2_ws/src/rm65_servo_teleop/config/teleop.yaml)，本目录不参与新入口自动加载。

`rm65_dls_calibration.json` 是既有控制器 DH / 工具帧查询快照，随现场状态变化，Git 忽略并在本机保留。它仅提供历史模型线索，不能代替新链的帧核对或现场验收。

旧参考参数 `realman_rm65_side_mount_teleop.json` 与采集脚本保存在[旧源码归档](../docs/archive/2026-10-26-legacy-control/README.md)，不再作为现行操作入口。真机帧验证见[真机安全与故障处理](../docs/真机安全与故障处理.md)。
