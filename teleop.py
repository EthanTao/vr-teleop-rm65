#!/usr/bin/env python3
"""VR Teleop — RM65 一键启动脚本

一键增量同步代码、选择模式、建立 SSH 隧道、启动容器内脚本。

每次运行都会直接同步代码到远程容器：先比较本地与远端内容哈希，
只上传发生变化的文件；内容无变化时跳过整个上传过程。
"""

import argparse
import atexit
import hashlib
import json
import os
from pathlib import Path
import shlex
import shutil
import signal
import subprocess
import sys
import tempfile
import time
from typing import Literal

# ===== 类型别名 =====
Mode = Literal["sim", "hardware-dry-run", "hardware", "sync-only"]


# ===== Config =====
# 用 ~/.ssh/config 里的主机别名而非硬编码 IP，这样同一套代码在实验室和宿舍都能用：
# `Host rm` 块先探测 LAN(192.168.0.115) 可达性，不可达时自动回退到 Tailscale 地址。
# 未配置该别名时，可把下面改回 "rm@192.168.0.115"（仅实验室内网可用）。
HOST = "rm"
CONTAINER = "xrobo-vr-teleop"
WORKSPACE = "/workspace"
LOCAL_PATH = str(Path(__file__).resolve().parent)

# RM65 真机
ARM_HOST = "192.168.10.18"
ARM_PORT = 8080

# 模式 → 容器内启动脚本
MODE_SCRIPT: dict[str, str] = {
    "sim": "python scripts/simulation/teleop_rm65_sim.py",
    "hardware-dry-run": (
        "python scripts/hardware/teleop_realman_rm65_safe_hardware.py " "--arm-host {arm_host} --dry-run"
    ),
    "hardware": "python scripts/hardware/teleop_realman_rm65_safe_hardware.py --arm-host {arm_host}",
}

# 模式 → SSH 隧道端口转发规则 [(local_port, remote_port), ...]
TUNNEL_SPEC: dict[str, list[tuple[int, int]]] = {
    "sim": [(18081, 7000), (63901, 63901)],
    "hardware-dry-run": [(63901, 63901)],
    "hardware": [(63901, 63901)],
}

# 同步范围：仓库根目录下的相对路径
SOURCE_PATHS = ["xrobotoolkit_teleop", "scripts", "tests", "config", "assets", "pyproject.toml"]

# 同步排除规则（片段匹配；以 "/" 结尾表示目录名）
SYNC_EXCLUDE = [
    ".git",
    "__pycache__",
    "*.pyc",
    ".pytest_cache",
    "node_modules",
    ".venv",
    "__MACOSX",
    ".DS_Store",
    "build/",
    "dist/",
    ".tmp-wheel/",
    "*.egg-info/",
]

# 相对路径常量同时用于远端校验，避免出现 "../" 逃逸
PYPROJECT_REL = "pyproject.toml"
PYCACHE_REL = "xrobotoolkit_teleop/__pycache__"

# 远端清单：比对两侧内容哈希、缓存上次同步结果
MANIFEST_NAME = "vr-teleop-rm65.manifest.json"
REMOTE_MANIFEST = f"/tmp/{MANIFEST_NAME}"
# 本地缓存：记录上次成功同步的文件哈希，未变化的文件不再重新读取
LOCAL_CACHE = Path(os.environ.get("VR_TELEOP_SYNC_CACHE") or Path(tempfile.gettempdir()) / MANIFEST_NAME)

# 全局状态（供 cleanup 访问）
tunnel_process: subprocess.Popen | None = None

# rsync 路径自动探测（保留探测结果用于日志提示）
RSYNC_PATH = shutil.which("rsync")
# 增量同步需要 ssh 把 tar 流送进容器
TAR_PATH = shutil.which("tar")
SSH_PATH = shutil.which("ssh")


# ===== Menu =====
def show_menu() -> Mode | None:
    """显示交互菜单，返回用户选择的模式或 None（退出）。"""
    top = f"""
╔══════════════════════════════════╗
║   VR Teleop — RM65 一键启动      ║
║   远程板: {HOST:<22} ║
║   容器:   {CONTAINER:<22} ║
╚══════════════════════════════════╝

  选择运行模式:

  [1] [SIM] 仿真模式
      启动可选 IK 后端 + Meshcat 3D 可视化
      自动打开浏览器查看虚拟机械臂

  [2] [CHECK] 真机 Dry Run（不运动）
      连接 RM65、检查 UDP 反馈并计算目标，但不下发运动目标

  [3] [HARD] 真机模式
      启动带 UDP 安全反馈的笛卡尔跟随控制，操控真实 RM65
      机械臂 IP: {ARM_HOST}:{ARM_PORT}

  [4]  [SYNC] 仅同步代码（不启动）
      只同步到容器，供手动 ssh 调试

  [5]  [EXIT] 退出

  输入编号 [1-5]: """

    while True:
        try:
            choice = input(top).strip()
        except (EOFError, KeyboardInterrupt):
            print()
            return None

        if choice == "1":
            return "sim"
        elif choice == "2":
            return "hardware-dry-run"
        elif choice == "3":
            return "hardware"
        elif choice == "4":
            return "sync-only"
        elif choice == "5":
            return None
        else:
            print(f"  无效输入 '{choice}'，请输入 1-5。")


# ===== Sync =====
def run_step(
    step_name: str,
    cmd: list[str],
    timeout: int = 300,
    stdin_data: str | None = None,
) -> bool:
    """执行一步操作，返回成功/失败。实时输出日志。"""
    print(f"\n── {step_name} ──")
    print(f"$ {' '.join(cmd)}")
    proc = None
    try:
        proc = subprocess.Popen(
            cmd,
            stdin=subprocess.PIPE if stdin_data is not None else None,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            encoding="utf-8",
            errors="replace",
        )
        if stdin_data is not None:
            # 边写边读，避免远端输出填满管道缓冲区造成死锁
            try:
                proc.stdin.write(stdin_data)
                proc.stdin.close()
            except (BrokenPipeError, OSError):
                pass
        for line in proc.stdout:
            print(f"  {line}", end="")
        proc.wait(timeout=timeout)
        if proc.returncode != 0:
            print(f"  [FAIL] 返回码: {proc.returncode}")
            return False
        print(f"  [OK] 完成")
        return True
    except subprocess.TimeoutExpired:
        if proc is not None:
            proc.kill()
        print(f"  [FAIL] 超时 ({timeout}s)")
        return False
    except FileNotFoundError:
        print(f"  [FAIL] 命令未找到: {cmd[0]}，请确认已安装")
        return False


def _normalize_relative(relative_path: str) -> str:
    """统一为 POSIX 相对路径，供远端 Linux 使用。"""
    return relative_path.replace("\\", "/")


def _matches_rule(name: str, rule: str) -> bool:
    if rule.endswith("/"):
        return name == rule[:-1]
    if rule.startswith("*."):
        return name.endswith(rule[1:])
    return name == rule


def _is_excluded(relative_path: str) -> bool:
    """判断仓库相对路径是否命中排除规则（按路径片段匹配）。"""
    return any(
        _matches_rule(part, rule) for part in _normalize_relative(relative_path).split("/") for rule in SYNC_EXCLUDE
    )


def _iter_source_files() -> list[Path]:
    """列出所有需要同步的文件，返回绝对路径并排序保证输出稳定。"""
    root = Path(LOCAL_PATH)
    collected: list[Path] = []
    for entry in SOURCE_PATHS:
        target = root / entry
        if target.is_file():
            collected.append(target)
        elif target.is_dir():
            collected.extend(sorted(path for path in target.rglob("*") if path.is_file()))
    return sorted(path for path in collected if not _is_excluded(path.relative_to(root).as_posix()))


def _file_sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _file_signature(path: Path) -> str | None:
    """用大小 + 修改时间做快速指纹；文件消失时返回 None。"""
    try:
        stat = path.stat()
    except OSError:
        return None
    return f"{stat.st_size}:{stat.st_mtime_ns}"


def _load_manifest() -> dict:
    try:
        data = json.loads(LOCAL_CACHE.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return {"files": {}, "signatures": {}}
    if not isinstance(data, dict):
        return {"files": {}, "signatures": {}}
    if not isinstance(data.get("files"), dict):
        data["files"] = {}
    if not isinstance(data.get("signatures"), dict):
        data["signatures"] = {}
    return data


def _save_manifest(manifest: dict) -> None:
    try:
        LOCAL_CACHE.parent.mkdir(parents=True, exist_ok=True)
        LOCAL_CACHE.write_text(json.dumps(manifest, indent=2, sort_keys=True), encoding="utf-8")
    except OSError as exc:
        print(f"  [WARN] 无法写入本地同步缓存 {LOCAL_CACHE}: {exc}")


def _read_remote_manifest() -> dict | None:
    try:
        result = subprocess.run(
            [SSH_PATH or "ssh", HOST, f"sudo docker exec {CONTAINER} cat {shlex.quote(REMOTE_MANIFEST)}"],
            capture_output=True,
            text=True,
            encoding="utf-8",
            errors="replace",
            timeout=60,
        )
    except (OSError, subprocess.SubprocessError):
        return None
    if result.returncode != 0 or not result.stdout.strip():
        return None
    try:
        data = json.loads(result.stdout)
    except ValueError:
        return None
    return data if isinstance(data, dict) else None


def _write_remote_manifest(files: dict[str, str]) -> bool:
    """Record successful transfers inside the same container as the code."""
    script = (
        "import os, sys; "
        f"path = {REMOTE_MANIFEST!r}; "
        "temporary = path + '.tmp'; "
        "open(temporary, 'w', encoding='utf-8').write(sys.stdin.read()); "
        "os.replace(temporary, path)"
    )
    try:
        result = subprocess.run(
            [SSH_PATH or "ssh", HOST, f"sudo docker exec -i {CONTAINER} python -c {shlex.quote(script)}"],
            input=json.dumps({"version": 1, "files": files}),
            capture_output=True,
            text=True,
            encoding="utf-8",
            errors="replace",
            timeout=60,
        )
    except (OSError, subprocess.SubprocessError) as exc:
        print(f"  [FAIL] remote manifest write failed: {exc}")
        return False
    if result.returncode != 0:
        print(f"  [FAIL] remote manifest write failed: {result.stderr.strip()}")
        return False
    return True


def _upload_changed_files(paths: list[Path]) -> bool:
    """只把发生变化的文件打进 tar 流，经 ssh 直接解包进容器。"""
    root = Path(LOCAL_PATH)
    relative = [_normalize_relative(str(path.relative_to(root))) for path in paths]
    remote_command = f"sudo docker exec -i {CONTAINER} tar -xf - -C {WORKSPACE}"
    if not relative:
        print("  待上传文件数: 0")
        return True

    print(f"  待上传文件数: {len(relative)} / tar → ssh → 容器 {WORKSPACE}")
    for name in relative[:12]:
        print(f"    {name}")
    if len(relative) > 12:
        print(f"    ... 其余 {len(relative) - 12} 个")

    tx_cmd = [TAR_PATH or "tar", "-cf", "-", "-C", str(root), *relative]
    print(f"$ {' '.join(tx_cmd[:6])} ... ({len(relative)} files)")
    print(f"  | ssh {HOST} '{remote_command}'")
    tar_proc = None
    try:
        # 先启动 ssh，再喂 tar：tar 的输出直接成为容器 tar 的输入，无需本地暂存
        ssh_proc = subprocess.Popen([SSH_PATH or "ssh", HOST, remote_command], stdin=subprocess.PIPE)
        tar_proc = subprocess.Popen(tx_cmd, stdout=ssh_proc.stdin, stderr=subprocess.PIPE)
        # 本地进程关闭管道副本，容器端 tar 退出后本地 tar 会收到 SIGPIPE
        if ssh_proc.stdin is not None:
            ssh_proc.stdin.close()
        ssh_code = ssh_proc.wait(timeout=600)
        _, tar_err = tar_proc.communicate(timeout=120)
        if tar_proc.returncode != 0:
            message = (tar_err or b"").decode("utf-8", "replace").strip()
            # 远端提前退出时 tar 会被 SIGPIPE 打断，这种情况以 ssh 返回码为准
            if ssh_code == 0:
                print(f"  [FAIL] tar 返回码: {tar_proc.returncode} {message}")
                return False
        if ssh_code != 0:
            print(f"  [FAIL] 容器内解包返回码: {ssh_code}")
            return False
        print(f"  [OK] 完成")
        return True
    except subprocess.TimeoutExpired:
        if tar_proc is not None:
            tar_proc.kill()
        print("  [FAIL] 上传超时 (600s)")
        return False
    except FileNotFoundError as exc:
        print(f"  [FAIL] 命令未找到: {exc}")
        return False


_REMOTE_DELETE_SCRIPT = """\
import hashlib, json, os, sys
root = os.path.realpath({workspace!r})
removed = failed = 0
for line in sys.stdin:
    line = line.strip()
    if not line:
        continue
    try:
        record = json.loads(line)
        relative = record["path"]
        expected = record["sha256"]
    except (ValueError, KeyError, TypeError):
        failed += 1
        print("BAD_RECORD", flush=True)
        continue
    if not isinstance(relative, str) or relative.startswith("/") or ".." in relative.split("/"):
        failed += 1
        print("REJECTED %s" % (relative,), flush=True)
        continue
    target = os.path.realpath(os.path.join(root, relative))
    if target != root and not target.startswith(root + os.sep):
        failed += 1
        print("REJECTED %s" % (relative,), flush=True)
        continue
    if not os.path.isfile(target):
        removed += 1
        continue
    try:
        with open(target, "rb") as handle:
            actual = hashlib.sha256(handle.read()).hexdigest()
    except OSError as exc:
        failed += 1
        print("FAILED %s: %s" % (relative, exc), flush=True)
        continue
    if actual != expected:
        # 远端内容与上次同步的清单不一致（被手动改过），保留而不删除
        failed += 1
        print("CHANGED %s" % (relative,), flush=True)
        continue
    try:
        os.remove(target)
        removed += 1
        parent = os.path.dirname(target)
        while parent.startswith(root + os.sep):
            if os.path.isdir(parent) and not os.listdir(parent):
                os.rmdir(parent)
                parent = os.path.dirname(parent)
            else:
                break
    except OSError as exc:
        failed += 1
        print("FAILED %s: %s" % (relative, exc), flush=True)
print("RESULT removed=%d failed=%d" % (removed, failed), flush=True)
sys.exit(1 if failed else 0)
"""


def _remove_remote_files(deletions: list[tuple[str, str]]) -> bool:
    """删除本地已移除的文件；只在远端内容与清单一致时删除，避免误删他人改动。"""
    if not deletions:
        print("  无需要删除的远端文件。")
        return True

    print(f"  待删除文件数: {len(deletions)}")
    for relative, _ in deletions[:10]:
        print(f"    {relative}")
    if len(deletions) > 10:
        print(f"    ... 其余 {len(deletions) - 10} 个")

    batch_size = 80
    chunks = [deletions[index : index + batch_size] for index in range(0, len(deletions), batch_size)]
    script = _REMOTE_DELETE_SCRIPT.format(workspace=WORKSPACE)
    cmd = [
        SSH_PATH or "ssh",
        HOST,
        f"sudo docker exec -i {CONTAINER} python3 -c {shlex.quote(script)}",
    ]

    for index, chunk in enumerate(chunks, 1):
        payload = "\n".join(json.dumps({"path": relative, "sha256": expected}) for relative, expected in chunk) + "\n"
        print(f"$ ssh {HOST} docker exec -i {CONTAINER} python3 -c <脚本>  (批次 {index}/{len(chunks)})")
        proc = None
        try:
            proc = subprocess.Popen(
                cmd,
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                encoding="utf-8",
                errors="replace",
            )
            out, _ = proc.communicate(payload, timeout=120)
        except subprocess.TimeoutExpired:
            if proc is not None:
                proc.kill()
            print(f"  [FAIL] 删除批次 {index} 超时 (120s)")
            return False
        except FileNotFoundError as exc:
            print(f"  [FAIL] 命令未找到: {exc}")
            return False

        for line in (out or "").splitlines():
            print(f"  {line}")
        if proc.returncode != 0:
            print(f"  [FAIL] 删除批次 {index} 返回码: {proc.returncode}")
            return False
    print("  [OK] 完成")
    return True


def sync_code() -> bool:
    """同步代码到容器：每次运行都比较内容哈希，只上传发生变化的文件。

    直接同步，不做整树覆盖：
      1. 计算本地文件哈希（大小 + mtime 未变时复用上次缓存）；
      2. 与上次同步的清单比较，得到「新增/改动」和「已删除」两类文件；
      3. 无改动 → 跳过上传；有改动 → tar 流经 ssh 直接解包进容器。
    """
    print(f"\n── 同步代码 (增量/直接同步) ──")
    print(f"  本地: {LOCAL_PATH}")
    print(f"  远端: {HOST} → {CONTAINER}:{WORKSPACE}")

    cached = _load_manifest()
    cached_files: dict[str, str] = cached["files"]
    cached_signatures: dict[str, str] = cached["signatures"]

    root = Path(LOCAL_PATH)
    files: dict[str, str] = {}
    signatures: dict[str, str] = {}
    hashed = 0
    for path in _iter_source_files():
        relative = _normalize_relative(str(path.relative_to(root)))
        signature = _file_signature(path)
        if signature is None:
            continue
        signatures[relative] = signature
        if cached_signatures.get(relative) == signature and relative in cached_files:
            files[relative] = cached_files[relative]
            continue
        files[relative] = _file_sha256(path)
        hashed += 1

    remote_manifest = _read_remote_manifest()
    remote_files = remote_manifest.get("files") if remote_manifest is not None else None
    if not isinstance(remote_files, dict):
        print("  远端无清单记录 → 执行全量同步")
        remote_files = {}
    elif remote_files == files:
        _save_manifest({"files": files, "signatures": signatures})
        print("  [OK] 远端已是最新，跳过上传。")
        return True

    # Local cache saves hashing work; only the remote manifest proves a transfer.
    changed_paths = [root / relative for relative, digest in files.items() if remote_files.get(relative) != digest]
    deletions = [(relative, digest) for relative, digest in remote_files.items() if relative not in files]
    print(
        f"  扫描完成: {len(files)} 个文件"
        f"（本地读取 {hashed}，待上传 {len(changed_paths)}，待删除 {len(deletions)}）"
    )

    if changed_paths:
        if not _upload_changed_files(changed_paths):
            print("  [FAIL] 增量上传失败")
            return False
    else:
        print("  本地无新增/改动文件，仅处理删除。")

    if deletions:
        if not _remove_remote_files(deletions):
            print("  [FAIL] 远端删除失败")
            return False

    if not _write_remote_manifest(files):
        return False
    _save_manifest({"files": files, "signatures": signatures})
    print(f"\n  [OK] 同步完成（缓存: {LOCAL_CACHE}）")
    return True


# ===== Tunnel =====
def start_tunnel(mode: Mode) -> subprocess.Popen | None:
    """启动 SSH 隧道（后台）。返回 Popen 对象或 None（失败）。"""
    specs = TUNNEL_SPEC.get(mode, [])
    if not specs:
        return None

    # Expose only the XR service tunnel to the LAN so a separate PICO headset
    # can reach the Windows host. Meshcat remains loopback-only.
    cmd = ["ssh", "-g", "-N"]
    for local_port, remote_port in specs:
        bind_host = "0.0.0.0" if local_port == 63901 else "127.0.0.1"
        cmd.extend(["-L", f"{bind_host}:{local_port}:127.0.0.1:{remote_port}"])
    cmd.append(HOST)

    print(f"\n── SSH 隧道 ──")
    print(f"  {' '.join(cmd)}")
    try:
        proc = subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        time.sleep(1)  # 给 SSH 一点时间建立连接
        if proc.poll() is not None:
            print(f"  [WARN] SSH 隧道启动失败（进程已退出），继续执行...")
            return None
        print(f"  [OK] 隧道已建立 (PID {proc.pid})")
        return proc
    except FileNotFoundError:
        print(f"  [WARN] ssh 命令未找到，跳过隧道")
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
            print(f"  [OK] 隧道已关闭")
        except subprocess.TimeoutExpired:
            tunnel_process.kill()
            tunnel_process.wait()
            print(f"  [OK] 隧道已强制关闭")
    tunnel_process = None


def cleanup_container():
    """杀掉容器内残留的 Meshcat 进程，释放端口 7000。"""
    kill_cmd = [
        "ssh",
        HOST,
        f"sudo docker exec {CONTAINER} bash -c 'pkill -f meshcat 2>/dev/null; pkill -f test_robot_viz 2>/dev/null'",
    ]
    subprocess.run(kill_cmd, timeout=10, capture_output=True)


# ===== Runner =====
def run_in_container(
    mode: Mode,
    home_joint_deg: list[float] | None = None,
    sim_ik_backend: str = "placo",
) -> int:
    """前台运行容器内脚本。返回 exit code。"""
    script = MODE_SCRIPT[mode]
    if mode in {"hardware", "hardware-dry-run"}:
        script = script.format(arm_host=ARM_HOST)
    elif mode == "sim":
        if home_joint_deg is None or len(home_joint_deg) != 6:
            raise ValueError("simulation requires six calibrated home joint angles")
        if sim_ik_backend not in {"placo", "official"}:
            raise ValueError("sim_ik_backend must be 'placo' or 'official'")
        joint_args = " ".join(format(value, ".10g") for value in home_joint_deg)
        script = f"{script} --home-joint-deg {joint_args} " f"--ik-backend {sim_ik_backend}"

    # TTY 检测：本地有 TTY 时 SSH 加 -t，docker 用 -it（支持 Ctrl+C 传播）
    # 本地无 TTY 时全用 -i（CI/自动化/管道）
    has_tty = sys.stdin.isatty()
    tty_flag = "it" if has_tty else "i"
    docker_cmd = f"sudo docker exec -{tty_flag} {CONTAINER} bash -c 'cd {WORKSPACE} && {script}'"

    cmd = ["ssh"]
    if has_tty:
        cmd.append("-t")  # SSH 分配 TTY 供 docker -it 使用
    cmd.extend([HOST, docker_cmd])

    print(f"\n── 启动容器内脚本 ──")
    print(f"  模式: {mode}")
    print(f"  $ {docker_cmd}")
    print(f"  (按 Ctrl+C 停止)\n")

    try:
        proc = subprocess.Popen(cmd)
        proc.wait()
        return proc.returncode
    except KeyboardInterrupt:
        return 0


def open_browser():
    """打开 Meshcat viewer（Windows Git Bash）。"""
    url = "http://127.0.0.1:18081/static/"
    print(f"\n  打开浏览器: {url}")
    try:
        subprocess.Popen(["start", url], shell=True)
    except FileNotFoundError:
        try:
            subprocess.Popen(["powershell.exe", "start", url])
        except FileNotFoundError:
            print(f"  [WARN] 无法自动打开浏览器，请手动访问: {url}")


# ===== Cleanup =====
def cleanup():
    """统一清理：关闭 SSH 隧道。atexit 和 signal handler 共用。"""
    stop_tunnel()
    # 不杀 docker exec——Ctrl+C 的信号传播已处理容器内进程退出


def cleanup_and_exit(code: int):
    """信号处理用：清理后退出"""
    cleanup()
    sys.exit(code)


# ===== CLI =====
def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="RM65 VR 遥操作一键启动",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=(
            "示例:\n"
            "  python teleop.py              # 交互菜单\n"
            "  python teleop.py --mode sim   # 直接启动仿真\n"
            "  python teleop.py --mode sim --home-joint-deg J1 J2 J3 J4 J5 J6\n"
            "  python teleop.py --mode sim --sim-ik-backend official "
            "--home-joint-deg J1 J2 J3 J4 J5 J6\n"
            "  python teleop.py --mode hardware-dry-run  # 真机检查，不运动\n"
            "  python teleop.py --mode hardware  # 直接启动真机\n"
            "  python teleop.py --mode sync-only # 仅同步代码\n"
        ),
    )
    parser.add_argument(
        "--mode",
        "-m",
        choices=["menu", "sim", "hardware-dry-run", "hardware", "sync-only"],
        default="menu",
        help="运行模式 (默认: menu 交互菜单)",
    )
    parser.add_argument(
        "--home-joint-deg",
        nargs=6,
        type=float,
        metavar=("J1", "J2", "J3", "J4", "J5", "J6"),
        help="仿真侧装安全 home joint，六个关节角度，单位为度",
    )
    parser.add_argument(
        "--sim-ik-backend",
        choices=["placo", "official"],
        default="placo",
        help="仿真 IK 后端；official 需要睿尔曼 API2 Python SDK",
    )
    return parser.parse_args()


# ===== Main =====
def check_prerequisites():
    missing = []
    if not SSH_PATH:
        missing.append("ssh")
    if not TAR_PATH:
        missing.append("tar")
    if missing:
        print("[FAIL] missing required commands: " + ", ".join(missing))
        sys.exit(1)
    print(f"[OK] local sync tools ready (仅检查本地 tar + ssh{'; 另检测到 rsync' if RSYNC_PATH else ''})")


def _resolve_sim_home_joint(home_joint_deg: list[float] | None) -> list[float]:
    if home_joint_deg is not None:
        return home_joint_deg
    if not sys.stdin.isatty():
        raise ValueError("--home-joint-deg is required for non-interactive simulation")
    while True:
        raw = input("输入已验证的 RM65 侧装 home joint（六个角度，单位 deg）: ").strip()
        try:
            values = [float(value) for value in raw.replace(",", " ").split()]
        except ValueError:
            values = []
        if len(values) == 6:
            return values
        print("  [FAIL] 需要恰好六个数，例如: 0 -30 60 0 60 0")


def run(
    mode_str: str,
    home_joint_deg: list[float] | None = None,
    sim_ik_backend: str = "placo",
):
    """主流程编排"""
    global tunnel_process

    # 每次启动都先同步，再显示菜单或运行远端脚本；同步失败时禁止启动旧代码。
    if not sync_code():
        print("\n[FAIL] source synchronization failed")
        sys.exit(1)

    if mode_str == "menu":
        mode = show_menu()
        if mode is None:
            return
    else:
        # argparse 已经做了 choices 校验
        mode = mode_str  # type: ignore[assignment]

    if mode == "sync-only":
        print("\n[OK] 同步完成，可手动 ssh 进容器调试。")
        return

    if mode == "sim":
        try:
            home_joint_deg = _resolve_sim_home_joint(home_joint_deg)
        except ValueError as exc:
            print(f"[FAIL] {exc}")
            sys.exit(2)

    # 注册清理
    atexit.register(cleanup)
    signal.signal(signal.SIGINT, lambda sig, frame: cleanup_and_exit(0))
    signal.signal(signal.SIGTERM, lambda sig, frame: cleanup_and_exit(1))

    # 1. 启动隧道
    tunnel_process = start_tunnel(mode)

    # 2. 清理残留 Meshcat 进程（释放端口 7000）
    if mode == "sim":
        cleanup_container()

    # 3. 仿真模式自动开浏览器
    if mode == "sim":
        open_browser()

    # 4. 前台运行容器脚本
    exit_code = run_in_container(
        mode,
        home_joint_deg=home_joint_deg,
        sim_ik_backend=sim_ik_backend,
    )

    # 5. 脚本退出后清理
    cleanup()
    sys.exit(exit_code)


def main():
    check_prerequisites()
    args = parse_args()
    run(
        args.mode,
        home_joint_deg=args.home_joint_deg,
        sim_ik_backend=args.sim_ik_backend,
    )


if __name__ == "__main__":
    main()
