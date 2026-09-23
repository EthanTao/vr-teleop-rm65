# RM65 一键启动脚本实现计划

> **状态**：✅ 已完成（`teleop.py` 已落地）
> **原始日期**：2026-07-22 | **本文档整理于**：2026-08-27

**目标：** 实现 `teleop.py`，一条命令完成代码同步、模式选择、SSH 隧道、容器内脚本启动和清理。

**架构：** 单文件 Python 脚本，内部按 Config / Menu / Sync / Tunnel / Runner / Cleanup 六个模块拆分，用 `tyro` 做 CLI。

**技术栈：** Python 3.10+, tyro, subprocess

---

### Task 1: Config 常量 + tyro CLI 入口

**Files:**
- Create: `D:/HDU-EILab/vr-teleop-rm65/teleop.py`

- [ ] **Step 1: 写入 Config 常量和 import**

```python
"""VR Teleop — RM65 一键启动脚本

一键同步代码、选择模式、建立 SSH 隧道、启动容器内脚本。
"""
import atexit
import os
import signal
import subprocess
import sys
import time
from dataclasses import dataclass
from typing import Literal

import tyro


# ===== Config =====
HOST = "rm@192.168.0.115"
CONTAINER = "xrobo-vr-teleop"
WORKSPACE = "/workspace"
LOCAL_PATH = "D:/HDU-EILab/vr-teleop-rm65"

# RM65 真机
ARM_HOST = "192.168.10.18"
ARM_PORT = 8080

# 脚本映射
MODE_SCRIPT = {
    "sim": "python scripts/simulation/teleop_rm65_sim.py",
    "hardware": "python scripts/hardware/teleop_realman_rm65_placo_hardware.py --arm-host {arm_host}",
}

# SSH 隧道
TUNNEL_SPEC = {
    "sim": [(18081, 7000), (63901, 63901)],
    "hardware": [(63901, 63901)],
}

# rsync exclude
RSYNC_EXCLUDE = [
    "--exclude=.git",
    "--exclude=__pycache__",
    "--exclude=*.pyc",
    "--exclude=.pytest_cache",
    "--exclude=node_modules",
    "--exclude=.venv",
    "--exclude=__MACOSX",
    "--exclude=.DS_Store",
]
```

- [ ] **Step 2: 写入 tyro CLI 入口**

```python
@dataclass
class Args:
    """VR Teleop — RM65 一键启动"""

    mode: Literal["menu", "sim", "hardware", "sync-only"] = "menu"
    """运行模式: menu=交互菜单, sim=仿真, hardware=真机, sync-only=仅同步"""


def main():
    args = tyro.cli(Args, description="RM65 VR 遥操作一键启动")
    run(args)


if __name__ == "__main__":
    main()
```

- [ ] **Step 3: 写入 `run()` 骨架（占位，供后续填充）**

```python
def run(args: Args):
    """主流程编排"""
    if args.mode == "menu":
        mode = show_menu()
        if mode is None:
            return  # 用户选退出
    else:
        mode = args.mode

    if mode == "sync-only":
        sync_code()
        print("\n✅ 同步完成，可手动 ssh 进容器调试。")
        return

    # 后续步骤由 Task 4-6 填充
    print(f"[DEBUG] mode={mode}")
```

- [ ] **Step 4: 空运行验证**

Run: `cd /d/HDU-EILab/vr-teleop-rm65 && python teleop.py --mode sim`
Expected: 不报错，输出 "[DEBUG] mode=sim"

---

### Task 2: 交互菜单

- [ ] **Step 1: 写入 `show_menu()` 函数**

```python
def show_menu() -> str | None:
    """显示交互菜单，返回用户选择的模式 (sim/hardware/sync-only) 或 None（退出）。"""
    banner = f"""
╔══════════════════════════════════╗
║   VR Teleop — RM65 一键启动      ║
║   远程板: {HOST:<22}║
║   容器:   {CONTAINER:<22}║
╚══════════════════════════════════╝

  选择运行模式:

  [1] 🟢 仿真模式
      启动 Placo IK + Meshcat 3D 可视化
      自动打开浏览器查看虚拟机械臂

  [2] 🔴 真机模式
      启动 movel 笛卡尔控制，操控真实 RM65
      机械臂 IP: {ARM_HOST}:{ARM_PORT}

  [3]  ⚙ 仅同步代码（不启动）
      只同步到容器，供手动 ssh 调试

  [4] ❌ 退出

  输入编号 [1-4]: """

    while True:
        try:
            choice = input(banner).strip()
        except (EOFError, KeyboardInterrupt):
            print()
            return None

        if choice == "1":
            return "sim"
        elif choice == "2":
            return "hardware"
        elif choice == "3":
            return "sync-only"
        elif choice == "4":
            return None
        else:
            print(f"  无效输入 '{choice}'，请输入 1-4。")
```

- [ ] **Step 2: 验证菜单**

Run: `python teleop.py --mode menu`
Expected: 显示菜单，输入 1/2/3/4 各自正确返回/退出，输入非法值提示重试

---

### Task 3: 同步模块

- [ ] **Step 1: 写入 `run_step()` 工具函数**

```python
def run_step(step_name: str, cmd: list[str], timeout: int = 300) -> bool:
    """执行一步操作，返回成功/失败。实时输出日志。"""
    print(f"\n── {step_name} ──")
    print(f"$ {' '.join(cmd)}")
    try:
        proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        for line in proc.stdout:
            print(f"  {line}", end="")
        proc.wait(timeout=timeout)
        if proc.returncode != 0:
            print(f"  ❌ 返回码: {proc.returncode}")
            return False
        print(f"  ✅ 完成")
        return True
    except subprocess.TimeoutExpired:
        proc.kill()
        print(f"  ❌ 超时 ({timeout}s)")
        return False
    except FileNotFoundError:
        print(f"  ❌ 命令未找到: {cmd[0]}，请确认已安装")
        return False
```

- [ ] **Step 2: 写入 `sync_code()` 函数**

```python
def sync_code() -> bool:
    """rsync + docker cp 同步代码到容器。返回成功/失败。"""
    remote_tmp = f"/tmp/vr-teleop-rm65"

    # rsync 本地 → 远程板
    rsync_cmd = [
        "rsync", "-avz", "--delete",
        *RSYNC_EXCLUDE,
        f"{LOCAL_PATH}/",
        f"{HOST}:{remote_tmp}/",
    ]
    if not run_step("rsync 同步到远程板", rsync_cmd):
        return False

    # docker cp 远程板 → 容器
    docker_cp_cmd = [
        "ssh", HOST,
        f"sudo docker cp {remote_tmp}/. {CONTAINER}:{WORKSPACE}/",
    ]
    if not run_step("docker cp 到容器", docker_cp_cmd):
        return False

    return True
```

- [ ] **Step 3: 验证同步**

Run: `python teleop.py --mode sync-only`
Expected: rsync 输出文件列表 → docker cp → "✅ 同步完成"

---

### Task 4: SSH 隧道管理

- [ ] **Step 1: 写入全局变量和隧道函数**

```python
# 全局状态（供 cleanup 访问）
tunnel_process: subprocess.Popen | None = None
current_mode: str | None = None


def start_tunnel(mode: str) -> subprocess.Popen | None:
    """启动 SSH 隧道（后台）。仿真模式转发 18081→7000 + 63901→63901，
    真机模式只转发 63901→63901。返回 Popen 对象或 None（失败）。"""
    specs = TUNNEL_SPEC.get(mode, [])
    if not specs:
        return None

    cmd = ["ssh", "-N"]
    for local_port, remote_port in specs:
        cmd.extend(["-L", f"{local_port}:127.0.0.1:{remote_port}"])
    cmd.append(HOST)

    print(f"\n── SSH 隧道 ──")
    print(f"  {' '.join(cmd)}")
    try:
        proc = subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        time.sleep(1)  # 给 SSH 一点时间建立连接
        if proc.poll() is not None:
            print(f"  ⚠ SSH 隧道启动失败（进程已退出），继续执行...")
            return None
        print(f"  ✅ 隧道已建立 (PID {proc.pid})")
        return proc
    except FileNotFoundError:
        print(f"  ⚠ ssh 命令未找到，跳过隧道")
        return None


def stop_tunnel():
    """停止 SSH 隧道"""
    global tunnel_process
    if tunnel_process is None:
        return
    if tunnel_process.poll() is None:
        print(f"\n  关闭 SSH 隧道 (PID {tunnel_process.pid})...")
        tunnel_process.terminate()
        try:
            tunnel_process.wait(timeout=5)
            print(f"  ✅ 隧道已关闭")
        except subprocess.TimeoutExpired:
            tunnel_process.kill()
            tunnel_process.wait()
            print(f"  ✅ 隧道已强制关闭")
    tunnel_process = None
```

- [ ] **Step 2: 清理注册**

在 `show_menu()` 之后、启动流程之前，注册清理函数：

```python
def cleanup():
    """atexit + signal 调用的统一清理"""
    stop_tunnel()
```

在 `run()` 中 `show_menu()` 之后、启动隧道之前注册：

```python
atexit.register(cleanup)
signal.signal(signal.SIGINT, lambda sig, frame: sys.exit(0))
signal.signal(signal.SIGTERM, lambda sig, frame: sys.exit(0))
```

---

### Task 5: 容器内脚本启动 + 开浏览器

- [ ] **Step 1: 写入 `run_in_container()`**

```python
def run_in_container(mode: str) -> int:
    """前台运行容器内脚本。返回 exit code。"""
    script = MODE_SCRIPT[mode]
    if mode == "hardware":
        script = script.format(arm_host=ARM_HOST)

    cmd = [
        "ssh", HOST,
        f"sudo docker exec -it {CONTAINER} bash -c 'cd {WORKSPACE} && {script}'",
    ]

    print(f"\n── 启动容器内脚本 ──")
    print(f"  模式: {mode}")
    print(f"  $ {cmd[2]}")
    print(f"  (按 Ctrl+C 停止)\n")

    try:
        proc = subprocess.Popen(cmd)
        proc.wait()
        return proc.returncode
    except KeyboardInterrupt:
        return 0
```

- [ ] **Step 2: 写入 `open_browser()`**

```python
def open_browser():
    """打开 Meshcat viewer（仅 Windows Git Bash）。"""
    url = "http://127.0.0.1:18081/static/"
    print(f"\n  打开浏览器: {url}")
    try:
        subprocess.Popen(["start", url], shell=True)
    except FileNotFoundError:
        try:
            subprocess.Popen(["powershell.exe", "start", url])
        except FileNotFoundError:
            print(f"  ⚠ 无法自动打开浏览器，请手动访问: {url}")
```

---

### Task 6: 主流程编排

- [ ] **Step 1: 将 `run()` 补充完整**

```python
def run(args: Args):
    global tunnel_process, current_mode

    if args.mode == "menu":
        mode = show_menu()
        if mode is None:
            return
    else:
        mode = args.mode

    current_mode = mode

    if mode == "sync-only":
        sync_code()
        print("\n✅ 同步完成，可手动 ssh 进容器调试。")
        return

    # 注册清理
    atexit.register(cleanup)
    signal.signal(signal.SIGINT, lambda sig, frame: cleanup_and_exit(0))
    signal.signal(signal.SIGTERM, lambda sig, frame: cleanup_and_exit(1))

    # 1. 同步代码
    if not sync_code():
        cleanup_and_exit(1)

    # 2. 启动隧道
    tunnel_process = start_tunnel(mode)

    # 3. 仿真模式自动开浏览器
    if mode == "sim":
        open_browser()

    # 4. 前台运行容器脚本
    exit_code = run_in_container(mode)

    # 5. 脚本退出后清理
    cleanup()
    sys.exit(exit_code)


def cleanup_and_exit(code: int):
    """信号处理用：清理后退出"""
    cleanup()
    sys.exit(code)
```

---

### Task 7: 写入 `cleanup()` 最终版

- [ ] **Step 1: 完善 `cleanup()` 函数**

```python
def cleanup():
    """统一清理：关闭 SSH 隧道。atexit 和 signal handler 共用。"""
    stop_tunnel()
    # 注意：不在这里杀 docker exec 进程——容器内脚本的退出由
    # Ctrl+C 信号传播处理，强制 kill 会留下脏状态。
```

---

### Task 8: 集成验证

- [ ] **Step 1: 测试 --help**

Run: `python teleop.py --help`
Expected: 显示 tyro 生成的帮助信息

- [ ] **Step 2: 测试 --mode sync-only**

Run: `python teleop.py --mode sync-only`
Expected: rsync 打印文件列表 → docker cp → 同步完成

- [ ] **Step 3: 测试 --mode sim（快速验证隧道 + 启动）**

Run: `python teleop.py --mode sim` （等待几秒后按 Ctrl+C）
Expected: 同步 → 隧道建立 → 浏览器打开 → 容器内脚本跑起来 → Ctrl+C → 隧道关闭

---

### Task 9: 更新 README.md

**Files:**
- Modify: `D:/HDU-EILab/vr-teleop-rm65/README.md`

- [ ] **Step 1: 在 README.md 的"快速开始"中添加一键启动用法**

```markdown
### 一键启动

```bash
# 交互菜单（推荐）
python teleop.py

# 直接启动仿真
python teleop.py --mode sim

# 直接启动真机
python teleop.py --mode hardware

# 仅同步代码到容器
python teleop.py --mode sync-only
```
```

插入在 Meshcat Viewer 章节之后。

---

### Task 10: 最终检查

- [ ] **Step 1: 确认 `teleop.py` 中所有外部命令在 Git Bash 下可用**

Run: `which rsync ssh`
Expected: 输出路径（应为 `/usr/bin/rsync`, `/usr/bin/ssh` 等）

- [ ] **Step 2: 确认 `teleop.py` 导入无报错**

Run: `python -c "import tyro; print('tyro OK')"`
Expected: `tyro OK`

- [ ] **Step 3: 确认 `teleop.py` 文件头 shebang 可选**

添加可选的 shebang 行（在 Windows Git Bash 中 `python teleop.py` 优先级更高，但保持习惯一致性）：

```python
#!/usr/bin/env python3
```
