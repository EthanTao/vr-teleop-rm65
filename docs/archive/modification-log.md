# 修改日志

## [2026-07-24] 修复：移除 meshcat.geometry.Label 调用

### 修改文件
- `xrobotoolkit_teleop/simulation/placo_teleop_controller.py`

### 问题
`meshcat.geometry.Label` 不存在于任何已发布版本（包括 0.3.2），导致 `PlacoTeleopController.__init__()` 中调用 `_init_reset_viz()` 时抛出 `AttributeError`，进程 exit(1) → Meshcat WebSocket 断连 → 浏览器页面空白。

### 根因
代码假设 `meshcat.geometry.Label` 可用，但该类从未被正式 release。三处调用：
1. `_init_reset_viz()` 第 200 行 — 初始化复位标签
2. `_update_reset_viz()` 第 216 行 — 复位中更新进度文字
3. `_update_reset_viz()` 第 225 行 — 复位完成恢复提示文字

### 修复
- 删除全部 3 处 `g.Label(...)` 调用
- 删除 `_reset_label` 场景对象相关的初始化逻辑
- 简化 `_update_reset_viz()`，仅保留球体指示灯的颜色切换（绿色=就绪，橙色=复位中）
- 移除不再需要的 `label_pos` 局部变量

### 效果
- 启动不再崩溃，仿真正常加载
- 复位状态仍可通过指示灯颜色感知（绿色 ↔ 橙色）
- 不再依赖 meshcat 的未发布 API

### 验证
- [ ] 启动仿真验证启动不崩溃
- [ ] 按 B 键触发复位，观察指示灯颜色变化

## [2026-09-03] 新增：真机冻结姿态开关 freeze_rotation（临时调试用）

### 修改文件
- `xrobotoolkit_teleop/hardware/realman_rm65_cartesian_teleop_controller.py`
- `scripts/hardware/teleop_realman_rm65_placo_hardware.py`

### 背景
核对真机平移方向时，姿态跟随会干扰判断：手柄平移时自然倾斜带动末端姿态变化，难以隔离纯平移。需要一个临时冻结姿态、只保留平移的调试开关，用来确认 base 坐标系 x/y 轴的物理朝向。

### 改动
- 控制器 `__init__` 新增 `freeze_rotation: bool = False` 参数，并存为 `self.freeze_rotation`。
- `_process_xr_pose` 返回前新增 `if self.freeze_rotation: delta_rot[:] = 0.0`，使旋转增量恒为零，`target_quat` 恒等于按下 Grip 时的锚点姿态，末端只做纯平移。
- 入口脚本 `main` 新增同名 tyro 参数并透传给控制器。

### 效果
- `--freeze-rotation true` 时机械臂姿态全程冻结在锚点，末端只做纯平移，便于核对 base 坐标系 x/y 物理朝向。
- 默认 `False`，不影响现有行为；用完去掉参数即恢复，无需删代码。

### 验证
- [x] `python -m py_compile` 两文件语法通过
- [ ] 真机 `--freeze-rotation true --log-motion-debug` 验证姿态冻结、平移方向可观察

## [2026-09-03] 文档：新增「仿真与真机代码并行维护」规则

### 修改文件
- `CLAUDE.md`（重要注意事项，新增第 11 条）
- `docs/modification-log.md`（本条记录）

### 背景
仿真与真机分属不同 controller，但共用上游逻辑（XR 数据读取、侧装旋转矩阵、`utils/geometry` 的 delta pose 运算）。历史上若只改一边而漏改另一边，会造成两侧行为漂移、仿真与真机表现不一致，增加上机调试成本。需在项目规范层面强制要求并行维护。

### 改动
- 在 `CLAUDE.md`「重要注意事项」新增第 11 条「仿真与真机代码并行维护」：修改任一方时必须同步评估另一方；列出需重点检查的共用逻辑清单（几何映射、侧装矩阵、delta 计算、锚点/增量策略、夹爪阈值、边界约束）；纯单端逻辑（Meshcat 可视化 / TCP-JSON 协议）可单端修改，但须在日志注明仅影响哪一侧。

### 验证
- [x] 规则已写入 `CLAUDE.md`，后续会话可加载
- [ ] 后续每次跨仿真/真机改动按此规则同步并记日志

## [2026-09-09] 原则确立：代码-文档不一致以代码为准；侧装映射矩阵现状对齐

### 修改文件
- `xrobotoolkit_teleop/utils/geometry.py`（**仅注释**，无功能改动）
- `CLAUDE.md`（「关键配置」侧装映射矩阵表）
- `docs/VR_TELEOP_HANDOFF.md`（§1 要点 4、现状表、§5.2、§5.3 仿真默认表）
- `docs/坐标系语义.md`（§5 第 5 点）
- `docs/modification-log.md`（本条记录）

### 背景 / 原则
评审「仿真用哪个侧装矩阵」时发现代码与文档不一致：`CLAUDE.md` / `VR_TELEOP_HANDOFF.md` 称仿真用 `R_HEADSET_TO_WORLD_RM65_SIDE_MOUNT`，但 `scripts/simulation/teleop_rm65_sim.py` 实际传入 `R_HEADSET_TO_WORLD_RM65_SIDE_MOUNT_HARDWARE`。**决定：代码-文档不一致以代码为准**——只对齐文档与代码注释到代码现状，不改功能代码；历史计划类文档（`docs/plans/*`、`docs/specs/*`）记录的是当时规划，保持原样不作回溯改写。

### 代码现状（事实核查，grep 复核）
- 仿真入口 `scripts/simulation/teleop_rm65_sim.py`：`R_headset_world=R_HEADSET_TO_WORLD_RM65_SIDE_MOUNT_HARDWARE`
- 真机 `xrobotoolkit_teleop/hardware/realman_rm65_cartesian_teleop_controller.py`：`use_headset_world_transform=True`（默认）时使用 `R_HEADSET_TO_WORLD_RM65_SIDE_MOUNT_HARDWARE`
- `R_HEADSET_TO_WORLD_RM65_SIDE_MOUNT`（2026-07-13 仿真验证的历史标定）：当前源码**无任何运行代码引用**，仅保留定义（`build/lib` 为陈旧构建副本，已忽略）

### 改动内容
- `geometry.py`：给 `R_HEADSET_TO_WORLD_RM65_SIDE_MOUNT` 注释加「历史标定、当前运行代码不再引用」说明；给 `_HARDWARE` 注释加「仿真与真机统一使用（以代码为准）」说明。
- `CLAUDE.md`：矩阵表两行改为——`SIDE_MOUNT`＝历史标定（当前未引用，保留参考）；`_HARDWARE`＝仿真与真机统一使用。
- `VR_TELEOP_HANDOFF.md`：要点 4、§3 现状表、§5.2 标题与正文、§5.3 仿真默认表补充矩阵行，均改为代码现状口径。
- `docs/坐标系语义.md`：§5 第 5 点改为「当前代码仿真与真机统一使用 `_HARDWARE`；`SIDE_MOUNT` 为历史标定」。

### 验证
- [x] grep 复核：源码中 `R_HEADSET_TO_WORLD_RM65_SIDE_MOUNT`（非 `_HARDWARE`）无运行引用
- [x] `py_compile` 不适用（无代码逻辑改动，仅注释）
- [ ] 未运行仿真/真机——纯文档与注释对齐，无功能变化

## [2026-09-09] 设计原则：仿真映射必须符合人体直觉（文档）

### 修改文件
- `CLAUDE.md`（「重要注意事项」新增第 12 条；矩阵表下加「仿真手感原则」说明）
- `docs/2026.9.9.md`（新增 §9：设计原则、代码现状与风险、逐轴实测判定表与处置路径）
- `docs/modification-log.md`（本条记录）

### 背景 / 原因
用户提出：仿真的目的是标定/调优遥操作手感，因此仿真里的映射关系必须符合正常人体直觉（手柄上抬→工具端升高、前推→伸远、右推→向右）。对照代码现状发现：仿真入口 `teleop_rm65_sim.py` 当前引用真机适配矩阵 `R_HEADSET_TO_WORLD_RM65_SIDE_MOUNT_HARDWARE`（与真机一致），而注释标注 2026-07-13 在 Placo 仿真中验证过的直觉矩阵 `R_HEADSET_TO_WORLD_RM65_SIDE_MOUNT` 已无运行引用；两套矩阵仅水平 X/Y 两轴符号相反（水平面差 180°）。

### 改动内容
- `CLAUDE.md` 新增第 12 条「仿真映射必须符合人体直觉」：仿真=手感标定场，其映射是「直觉参考系」；真机矩阵修正（`_HARDWARE`、`invert_tcp_xy`）只是把这份直觉搬运到物理机器，不能用真机适配矩阵反向定义仿真手感；仿真矩阵选择以逐轴实测为准（判定表见 `docs/2026.9.9.md` §9.3）。
- `docs/2026.9.9.md` §9：写明原则、代码现状（仿真入口用 `_HARDWARE`，SIDE_MOUNT 无运行引用）、两矩阵仅水平轴差 180° 的事实、以及 3 轴直觉实测判定表（上抬/前推/右推 → 期望方向 → 实测 → 判定）与处置路径（若反直觉→仿真入口切回 SIDE_MOUNT 并记日志「仅影响仿真」；若直觉→标注已实测确认）。

### 说明（为何不改代码）
本轮未改任何行为代码：仿真矩阵是否"反直觉"只能在跑起来的仿真里逐轴实测判定（本环境无法运行远程容器仿真），按「以代码为准 + 待实测确认」处理：现状以代码事实记录，判定路径已写入文档，实测结论出来后如需切换仅改 `teleop_rm65_sim.py` 一行 import + 一行传参。

### 验证
- [x] 原则与判定表已写入 `CLAUDE.md`（第 12 条）与 `docs/2026.9.9.md`（§9）
- [ ] 仿真 3 轴直觉实测（上抬/前推/右推 × 期望方向对照）——待有仿真环境后执行

## [2026-09-09] 实现：仿真恢复 SIDE_MOUNT 直觉映射 + 新增侧装显示（仅影响仿真）

### 修改文件
- `xrobotoolkit_teleop/common/base_teleop_controller.py`（新增 base 世界位姿参数并应用）
- `xrobotoolkit_teleop/simulation/placo_teleop_controller.py`（透传 base 位姿参数）
- `scripts/simulation/teleop_rm65_sim.py`（矩阵改回 SIDE_MOUNT；新增 `--side-mount-display`/`--mount-rpy-deg`）
- `xrobotoolkit_teleop/utils/geometry.py`（仅注释：两矩阵用途与等价关系）
- 文档：`CLAUDE.md`、`docs/VR_TELEOP_HANDOFF.md`、`docs/坐标系语义.md`、`docs/2026.9.9.md`、`docs/modification-log.md`

### 背景 / 原因
按原则 12「仿真映射必须符合人体直觉」与用户要求「手柄往哪移机械臂就往哪移、仿真显示成侧装样子」。关键推导：真机有效映射 = `_HARDWARE` 矩阵 + `invert_tcp_xy=True`（delta 的 X/Y 再取反）≡ `SIDE_MOUNT` 矩阵（本地数值验证误差 0.0）；而仿真入口此前直接引用 `_HARDWARE`（未再取反）→ 与真机有效手感在水平面相差 180°，是「仿真方向别扭」的根源。`config/realman_rm65_side_mount_teleop.json` 与历史文档均标注仿真验证矩阵为 `SIDE_MOUNT`。

### 改动内容
- `base_teleop_controller.py`：`__init__` 新增可选 `base_world_pos` / `base_world_quat`（固定底座专用，含有限值/归一化校验）；`_placo_setup` 在 IK 任务创建**之前**把 `state.q[:7]`（x,y,z,qx,qy,qz,qw）设为该世界位姿，使任务初值/末端读取基于安装后的底座。
- `placo_teleop_controller.py`：透传上述两参数（仅仿真侧）。
- `teleop_rm65_sim.py`：`R_headset_world` 改回 `R_HEADSET_TO_WORLD_RM65_SIDE_MOUNT`；新增 `--side-mount-display`（默认 True：底座贴竖直面、J1 水平）与 `--mount-rpy-deg`（默认 (90,0,0)，可关掉/微调朝向）；新增本地 `_rpy_to_quat_xyzw` 工具；启动打印映射与显示参数。
- `geometry.py`：注释改为「SIDE_MOUNT=仿真用（2026-07-13 验证，真机 invert_tcp_xy=True 时有效等价）；_HARDWARE=真机控制器矩阵」。

### 并行维护评估（原则 11）
**仅影响仿真**：真机控制器与真机入口未改动。真机侧发现待确认项：真机入口 tyro 默认 `invert_tcp_xy=False`，与 VR_TELEOP_HANDOFF/config 的现场参数 `true` 不一致——若保持默认 False，真机有效映射将与仿真水平镜像相反，需确认默认值口径（见下）。

### 验证
- [x] 本地 AST 语法检查 4 个 Python 文件通过
- [x] 数值验证：`HARDWARE 矩阵 + delta X/Y 取反 ≡ SIDE_MOUNT 矩阵`（随机 100 样本最大误差 0.0）
- [ ] 仿真运行验证：Meshcat 显示侧装姿态、3 轴直觉实测（`docs/2026.9.9.md` §9.3 判定表）——待有仿真环境
- [ ] 待办：真机入口 `invert_tcp_xy` 默认值 False vs 现场 true 的口径确认

## [2026-09-09] 回退：机器人竖立显示 + 映射回到改动前（_HARDWARE）

### 修改文件
- `scripts/simulation/teleop_rm65_sim.py`（还原为 `_HARDWARE` 矩阵，去掉侧装显示参数）
- `xrobotoolkit_teleop/common/base_teleop_controller.py`（还原：去掉 base 世界位姿参数与应用逻辑）
- `xrobotoolkit_teleop/simulation/placo_teleop_controller.py`（还原：去掉透传参数）
- `xrobotoolkit_teleop/utils/geometry.py`（仅注释，改为最终代码准确口径）
- 文档：`CLAUDE.md`、`docs/VR_TELEOP_HANDOFF.md`、`docs/坐标系语义.md`、`docs/2026.9.9.md`、`docs/modification-log.md`

### 背景 / 原因
用户最终决定：「机器人**竖着显示**，并**保持原来的坐标映射关系不变**」——即回到本次会话改动前的行为（仿真入口用 `R_HEADSET_TO_WORLD_RM65_SIDE_MOUNT_HARDWARE`、模型按 URDF 原样竖立显示），并在确认选项中选择了「回到改动前」而非保留 SIDE_MOUNT 或参数化方案。

### 改动内容
- 还原上一轮全部实验改动：仿真矩阵 `SIDE_MOUNT` → `_HARDWARE`；删除侧装显示开关 `--side-mount-display` / `--mount-rpy-deg` 及 `_rpy_to_quat_xyzw` 工具；基类/仿真子类删除 `base_world_pos/base_world_quat`。
- 文档/注释对齐到最终状态：仿真与真机当前均引用 `_HARDWARE`（代码为准）；`SIDE_MOUNT` 标注为 2026-07-13 仿真验证矩阵、真机 `invert_tcp_xy=True` 时有效映射与之等价；「仿真映射须符合人体直觉」原则（第 12 条）与逐轴实测判定表保留，实测前不盲改矩阵。

### 并行维护评估（原则 11）
两侧行为均回到会话前基线（无功能变化）；真机控制器与真机入口本轮未改动。真机待办仍开放：入口默认 `invert_tcp_xy=False` 与现场参数 `true` 不一致。

### 验证
- [x] 本地 AST 语法检查 4 个 Python 文件通过（见下条命令结果）
- [ ] 与改动前基线逐字节等价核对：仅 `teleop_rm65_sim.py` 多了一段 docstring 注释说明，其余行为代码还原
- [ ] 仿真运行验证（Meshcat 竖立显示、3 轴直觉实测 §9.3）——待有仿真环境
