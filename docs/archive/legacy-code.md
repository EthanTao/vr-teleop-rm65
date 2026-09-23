# 历史代码与兼容入口

整理日期：2026-09-16。当前运行入口和历史实现分开存放，便于定位维护范围。

## 当前入口

- 真机：`scripts/hardware/teleop_realman_rm65_safe_hardware.py`，调用 `RealmanRM65SafeTeleopController`。
- 仿真：`scripts/simulation/teleop_rm65_sim.py`，继续支持 Placo 和官方 IK。
- 一键启动：`teleop.py` 已指向新的真机入口。

## 兼容关系

| 原路径 | 当前职责 | 实现位置 |
| --- | --- | --- |
| `scripts/hardware/teleop_realman_rm65_placo_hardware.py` | 兼容旧 CLI，转发同一个 `main`，保留参数 | `scripts/hardware/teleop_realman_rm65_safe_hardware.py` |
| `scripts/hardware/teleop_realman_rm65_hardware.py` | 兼容旧映射诊断 CLI；仍只允许 dry-run | `xrobotoolkit_teleop/hardware/legacy/single_file_teleop.py` |
| `xrobotoolkit_teleop/hardware/realman_rm65_cartesian_teleop_controller.py` | 兼容旧类导入 | `xrobotoolkit_teleop/hardware/legacy/cartesian_teleop_controller.py` |

历史模块化控制器保留原始算法用于审计和回归，未接入当前独立 UDP 安全链，不作为真机推荐入口。其控制方法在本次整理中没有改变。测试直接导入 `legacy` 实现，另有兼容入口回归检查。

兼容范围是上述 CLI、单文件入口的 `Args/main` 和控制器类。旧模块中未公开的辅助函数、通过旧模块替换依赖进行 monkeypatch 等内部使用方式不作为兼容接口；维护测试应导入实现模块。

## 参数与依赖

参考 JSON 不会自动加载。2026-09-16 按当前源码校正映射矩阵和 `invert_tcp_xy=False`，不是新的现场标定结果。运行参数仍来自 CLI 默认值或显式参数。

`pyproject.toml` 的 `hardware`、`hand`、`mac` 是历史可选依赖组，当前 RM65 主路径不引用其中的 UR/灵巧手依赖。为兼容外部安装命令暂时保留，并在文件中标注；它们不属于默认安装依赖。

## 生成物与备份

- `build/lib/`、`.tmp-wheel/` 是旧打包产物；`*.egg-info/` 是安装元数据，均不作为源码维护。
- `build/*baseline/` 和验证报告保留作为历史证据。
- rsync 和 Docker 构建上下文已排除 build、dist、临时 wheel 及安装元数据，避免把旧副本当作运行代码同步。
- 本次修改前文件保存在 `build/cleanup-2026-09-16/before/`，原始 SHA-256 在同级 `original-sha256.json`；无需依赖 Git 即可核对。
- 根目录的旧 `MANIFEST.txt` 及异常文件归档到 `build/cleanup-2026-09-16/archived-root-files/`，原文件名见该目录的 `manifest.json`。

本次只整理本地工作区，没有同步远程容器。原有控制参数、几何运算、连续 IK、停止和故障恢复逻辑保持原样。
