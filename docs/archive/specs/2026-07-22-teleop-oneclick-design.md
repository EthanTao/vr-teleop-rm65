---
title: RM65 VR 遥操作 — 一键启动脚本设计
date: 2026-07-22
status: done
---

# 一键启动脚本设计

> **状态**：✅ 已实现（`teleop.py` 落地，支持 `--mode {menu, sim, hardware, sync-only}`）。本文档整理于 2026-08-27。

## 一句话总结

一个 Python 脚本 (`teleop.py`)，放在仓库根目录，把"同步代码 → 选模式 → 建隧道 → 跑脚本 → 清理"串成一条命令。

## 解决的问题

当前工作流需要 3–4 步手动操作：

```
① 手动 rsync + docker cp 同步代码
② 手动 ssh docker exec 启动脚本
③ 另开窗口运行 start_meshcat_viewer.sh 建隧道
④ 手动打开浏览器
```

一键脚本将这四步合并为 `python teleop.py`。

## 技术选型

| 项 | 选择 | 理由 |
|----|------|------|
| 语言 | Python 3.10+ | 与项目一致，信号处理比 bash 可靠 |
| CLI 框架 | `tyro` | 已在依赖中，与项目其他入口风格一致 |
| 菜单 UI | ANSI 转义码 | 零依赖，所有终端支持 |
| 进程管理 | `subprocess.Popen` | 精确控制子进程生命周期 |
| 清理机制 | `signal` + `atexit` + `finally` | 三重保障，确保不留隧道孤儿进程 |

## 文件位置

```
vr-teleop-rm65/
├── teleop.py              # ← 新增：一键启动脚本（根目录）
├── README.md
├── pyproject.toml
├── ...
```

## 核心流程

```
python teleop.py
    ↓
显示菜单：仿真 / 真机 / 仅同步 / 退出
    ↓
[同步阶段]
  rsync --delete  →  失败？→ 报错退出
  docker cp       →  失败？→ 报错退出
    ↓
[启动阶段]
  仿真模式:
    └─ 后台 SSH 隧道 (18081→7000, 63901→63901)
    └─ 打开浏览器 http://127.0.0.1:18081/static/
    └─ 前台 docker exec 运行 teleop_rm65_sim.py
  真机模式:
    └─ 后台 SSH 隧道 (仅 63901→63901)
    └─ 前台 docker exec 运行 teleop_realman_rm65_placo_hardware.py
  仅同步:
    └─ 退出
    ↓
[等待 Ctrl+C]
    ↓
[清理阶段 — signal + atexit + finally]
  └─ 杀掉 SSH 隧道子进程
  └─ 真机模式 → 额外 set_arm_stop
```

## 模块结构（单文件内部拆分）

```python
# ===== 1. Config =====
# 主机、容器、路径、机械臂 IP 等常量

# ===== 2. Menu =====
# 显示选项菜单 + 读取用户输入

# ===== 3. Sync =====
# run_step() 通用包装: 执行命令 → 检查返回码 → 打印日志

# ===== 4. Tunnel =====
# start_tunnel() / stop_tunnel() 管理 SSH 子进程

# ===== 5. Runner =====
# run_in_container() → 前台 docker exec，输出实时显示

# ===== 6. Cli =====
# tyro 入口: --mode {sim, hardware, sync-only} 跳过菜单

# ===== 7. Cleanup =====
# signal handler + atexit + main 内 finally
```

## 菜单交互

```
╔══════════════════════════════════╗
║   VR Teleop — RM65 一键启动      ║
║   远程板: rm@192.168.0.115       ║
║   容器:   xrobo-vr-teleop        ║
╚══════════════════════════════════╝

  选择运行模式:

  [1] 🟢 仿真模式
      启动 Placo IK + Meshcat 3D 可视化
      自动打开浏览器查看虚拟机械臂

  [2] 🔴 真机模式
      启动 movel 笛卡尔控制，操控真实 RM65
      机械臂 IP: 192.168.10.18:8080

  [3]  ⚙ 仅同步代码（不启动）
      只同步到容器，让你手动 ssh 进去调试

  [4] ❌ 退出

  输入编号 [1-4]:
```

## SSH 隧道管理

```python
tunnel_process: subprocess.Popen | None = None

def start_tunnel(local_port, remote_port, host) -> subprocess.Popen:
    return subprocess.Popen([
        "ssh", "-N",
        "-L", f"{local_port}:127.0.0.1:{remote_port}",
        host
    ])

def stop_tunnel():
    global tunnel_process
    if tunnel_process and tunnel_process.poll() is None:
        tunnel_process.terminate()
        tunnel_process.wait(timeout=5)
```

- 隧道在后台运行 (`Popen`，不用 `wait()`)
- 清理时用 `terminate()`（SIGTERM）而非 `kill()`（SIGKILL），给 SSH 优雅关闭的机会
- 5 秒超时后强制 `kill()`

## 同步逻辑

同步采用 `run_step()` 通用模式，统一管理每一步的执行和日志输出。

```python
def run_step(step_name: str, cmd: list[str]) -> bool:
    """执行一步操作，返回成功/失败。失败时自动打印 stdout/stderr。"""
```

## 错误处理矩阵

| 阶段 | 失败处理 | 是否继续 |
|------|---------|---------|
| rsync | 打印完整 stderr，输出 exit code | ❌ 退出 |
| docker cp | 同上 | ❌ 退出 |
| SSH 隧道 | 打印警告 | ✅ 继续（真机模式不需要隧道） |
| docker exec | 前台退出，自动清理 | — |
| 清理 | 吞异常，确保全部执行 | — |

## CLI 接口（tyro）

```bash
# 交互菜单（默认）
python teleop.py

# 非交互：直接启动仿真
python teleop.py --mode sim

# 非交互：直接启动真机
python teleop.py --mode hardware

# 仅同步代码
python teleop.py --mode sync-only
```

## 不包含的功能（明确不做的）

- ❌ 不管理 Docker 容器生命周期（不负责 `docker compose up/down`）
- ❌ 不支持多机器人
- ❌ 不做代码差异检测（每次都全量 rsync）
- ❌ 不保存 SSH 密码/密钥状态（依赖已有的 SSH 免密配置）
