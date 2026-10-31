# 开发与验证

从仓库根目录开发。日常入口、参数、ROS 话题和执行权限属于对外接口，整理目录时保持兼容；入口、配置和测试有改动时一起更新对应操作文档。

## 安装开发依赖

使用 Python 3.10+，建议使用独立环境；ROS2 Humble 的运行环境仍由现有 Docker / WSL / 板端流程管理。

```powershell
python -m venv .venv
.venv\Scripts\python.exe -m pip install -e ".[dev]"
.venv\Scripts\python.exe -X utf8 scripts/tools/check_repository.py
```

Linux 对应使用 `.venv/bin/python`。这里的 pip 安装用于本机 Python 开发，不安装 ROS、原生 XR SDK、RealMan 驱动或硬件服务；ROS 的构建和安装见 [ROS 工作空间](ros2_ws/README.md)。

## 提交前检查

| 检查 | 命令与范围 |
| --- | --- |
| 统一检查 | `python -X utf8 scripts/tools/check_repository.py`：Python 3.10 语法、YAML/XML、Linux 换行、ROS 脚本安装清单、导航链接、Ruff 和完整本机 pytest |
| 快速静态检查 | 同一命令加 `--static-only`，跳过 pytest |
| 既有回归入口 | `python -X utf8 -B -m pytest -q`，继续保持可用 |
| 仿真启动预览 | `python teleop.py --mode sim --print-command`，仅打印命令 |
| ROS 集成验证 | `python teleop.py --mode test`，在既有隔离环境内编译/运行；详见 [ROS 文档](ros2_ws/README.md) |

统一检查仅运行仓库静态检查和离线测试，不启动 WSL、Docker、ROS、SSH 或硬件。缺少 ROS 的本机明确跳过集成测试，跳过不代表通过了 ROS/真机验收。

[Ruff](https://docs.astral.sh/ruff/configuration/) 检查语法、未定义名称、错误的格式占位符与控制流；规则与版本放在 `pyproject.toml`。现有 ROS 导入顺序、订阅对象持有和排版无需为静态检查改写。Black 继续使用现有 120 列设置，按修改文件执行 `python -m black <文件>`。GitHub Actions 对 Windows / Ubuntu 的 Python 3.10、3.12 执行同一统一检查，另在 Ubuntu 检查 shell 语法；不包含部署。

## 目录边界

- 根目录保留现有 Python 入口和构建元数据；启动编排在 `scripts/setup/`，开发与诊断工具在 `scripts/tools/`。
- 当前运行源码、配置、ROS 测试在 `ros2_ws/src/rm65_servo_teleop/`，跨平台入口测试在 `tests_moveit/`。保持现有目录名称，避免破坏导入和外部调用。
- 缓存与临时脚本放 `build/cache/`、`build/tmp/`，验证结果放 `build/validation/`，原始会话放 `build/experiments/`。它们不上传 Git，原始记录需另行备份。
- 当前操作文档放 `docs/`，证据追加到 `docs/logs/年.月.日.md`；历史原文留在 `docs/archive/`，不作为当前接口说明。
- 本机 `.env`、设备配置和密钥不提交。示例用 `.env.example` 或现有 `*.template.yaml`，真实配置仍须经过原有现场准入。

具体目录职责见 [仓库结构](docs/仓库结构.md)。依赖上游提交及补丁继续由 [dependencies.repos](ros2_ws/dependencies.repos) 和 [补丁说明](ros2_ws/patches/README.md) 管理，不在整理任务中更新。

## 修改与交付

1. 保留他人已有未提交改动，先检查差异，再做范围明确的修改。
2. 对接口或安全逻辑的修改补充行为验证；纯排版和文档整理不增加重复测试。
3. 在 [当日日志](docs/logs/README.md) 记录文件、原因、行为、验证结果和未完成项。分别记录本机测试、ROS 编译、仿真、硬件只读、真实发送和实体运动。
4. 运行统一检查并检查 `git diff --check` 后提交；推送不自动部署到板端。

厂商 `.patch` 文件中的空白上下文用于精确匹配上游源码，保留原文；Git 属性仅对这些补丁关闭行尾空白和文件末尾空行告警，避免整理时破坏补丁应用。

真实执行继续遵守 [项目规范](CLAUDE.md) 和 [真机安全与故障处理](docs/真机安全与故障处理.md)。
