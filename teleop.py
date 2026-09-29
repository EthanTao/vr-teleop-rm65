#!/usr/bin/env python3
"""VR Teleop — RM65 一键启动脚本

一次运行做完四件事：增量同步代码到远程容器 → 选模式 → 建 SSH 隧道 → 在容器内起脚本。

同步按内容哈希做增量：本地缓存记录「文件签名（大小 + mtime）→ 哈希」，签名没变就不重读文件；
远端清单证明上一次传输真的成功，因此只有远端与本地哈希完全一致才跳过上传。
"""

from __future__ import annotations

import argparse
import atexit
from dataclasses import dataclass
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

# ===== 配置 =====
# 远端主机用 ~/.ssh/config 里的别名而不是硬编码 IP：`Host rm` 先探 LAN(192.168.0.115)，
# 不可达时自动回退 Tailscale，实验室和宿舍共用同一套代码。没配这个别名就改回 "rm@192.168.0.115"。
HOST = "rm"
CONTAINER = "xrobo-vr-teleop"
WORKSPACE = "/workspace"
LOCAL_PATH = str(Path(__file__).resolve().parent)

# RM65 真机
ARM_HOST = "192.168.10.18"
ARM_PORT = 8080

# XR 服务端口要暴露给局域网里的 PICO 头显；Meshcat 只留本机回环
XR_PORT = 63901

Mode = Literal["sim", "hardware-dry-run", "hardware", "sync-only"]


@dataclass(frozen=True)
class ModeSpec:
    """一个运行模式的全部信息：菜单文案 + 容器内命令 + 要建的隧道端口。"""

    tag: str
    title: str
    blurb: tuple[str, ...]
    script: str = ""
    tunnels: tuple[tuple[int, int], ...] = ()


MODE_SPECS: dict[Mode, ModeSpec] = {
    "sim": ModeSpec(
        tag="SIM",
        title="仿真模式",
        blurb=("启动可选 IK 后端 + Meshcat 3D 可视化", "自动打开浏览器查看虚拟机械臂"),
        script="python scripts/simulation/teleop_rm65_sim.py",
        tunnels=((18081, 7000), (XR_PORT, XR_PORT)),
    ),
    "hardware-dry-run": ModeSpec(
        tag="CHECK",
        title="真机 Dry Run（不运动）",
        blurb=(
            "连接 RM65、检查 UDP 反馈并计算目标，但不下发运动目标",
            # --log-motion-debug 打印「手柄位移 → 目标位姿 → DLS 关节目标」整条计算链；
            # dry run 不下发运动，加日志没有安全代价
            "逐周期打印手柄位移 → 目标位姿 → DLS 关节目标的完整计算链",
        ),
        script=(
            "python scripts/hardware/teleop_realman_rm65_safe_hardware.py "
            f"--arm-host {ARM_HOST} --dry-run --log-motion-debug"
        ),
        tunnels=((XR_PORT, XR_PORT),),
    ),
    "hardware": ModeSpec(
        tag="HARD",
        title="真机模式",
        blurb=("启动带 UDP 安全反馈的笛卡尔跟随控制，操控真实 RM65", f"机械臂 IP: {ARM_HOST}:{ARM_PORT}"),
        script=f"python scripts/hardware/teleop_realman_rm65_safe_hardware.py --arm-host {ARM_HOST}",
        tunnels=((XR_PORT, XR_PORT),),
    ),
    "sync-only": ModeSpec(tag="SYNC", title="仅同步代码（不启动）", blurb=("只同步到容器，供手动 ssh 调试",)),
}

# 同步范围：仓库根目录下的相对路径
SOURCE_PATHS = ["xrobotoolkit_teleop", "scripts", "tests", "config", "assets", "pyproject.toml"]

# 排除规则（按路径片段匹配）：以 "/" 结尾表示目录名，以 "*." 开头表示后缀
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

MANIFEST_NAME = "vr-teleop-rm65.manifest.json"
REMOTE_MANIFEST = f"/tmp/{MANIFEST_NAME}"

SSH_PATH = shutil.which("ssh")
TAR_PATH = shutil.which("tar")

# 当前 SSH 隧道进程；由 start_tunnel 写入、stop_tunnel 收尾
tunnel_process: subprocess.Popen | None = None


# ===== 菜单 =====
def show_menu() -> Mode | None:
    """显示交互菜单，返回所选模式；选退出或 Ctrl+C/EOF 返回 None。"""
    modes = list(MODE_SPECS)
    lines = [
        "",
        "╔══════════════════════════════════╗",
        "║   VR Teleop — RM65 一键启动      ║",
        f"║   远程板: {HOST:<22} ║",
        f"║   容器:   {CONTAINER:<22} ║",
        "╚══════════════════════════════════╝",
        "",
        "  选择运行模式:",
        "",
    ]
    for index, spec in enumerate(MODE_SPECS.values(), 1):
        lines.append(f"  [{index}] [{spec.tag}] {spec.title}")
        lines.extend(f"      {line}" for line in spec.blurb)
        lines.append("")
    exit_index = len(modes) + 1
    lines.append(f"  [{exit_index}] [EXIT] 退出")
    menu = "\n".join(lines) + f"\n\n  输入编号 [1-{exit_index}]: "

    while True:
        try:
            choice = input(menu).strip()
        except (EOFError, KeyboardInterrupt):
            print()
            return None
        if choice.isdigit() and 1 <= int(choice) <= len(modes):
            return modes[int(choice) - 1]
        if choice == str(exit_index):
            return None
        print(f"  无效输入 '{choice}'，请输入 1-{exit_index}。")


# ===== 同步 =====
def _resolve_local_cache() -> Path:
    """本地同步清单缓存路径，可用 `VR_TELEOP_SYNC_CACHE` 覆盖。

    默认放系统临时目录；受限沙箱里标准临时目录全不可写时，`tempfile` 会顺位退回到仓库根，
    把缓存文件丢进源码树，因此一旦发现临时目录落在仓库内就改用 `build/tmp/`
    （根因见 docs/2026.9.26.md「仓库布局整理」）。
    """
    override = os.environ.get("VR_TELEOP_SYNC_CACHE")
    if override:
        return Path(override)
    default = Path(tempfile.gettempdir()) / MANIFEST_NAME
    repo_root = Path(LOCAL_PATH).resolve()
    try:
        parent = default.parent.resolve()
    except OSError:
        return default
    return (
        repo_root / "build" / "tmp" / MANIFEST_NAME if parent == repo_root or repo_root in parent.parents else default
    )


LOCAL_CACHE = _resolve_local_cache()


def _is_excluded(relative_path: str) -> bool:
    """仓库相对路径是否命中排除规则（按路径片段匹配）。"""
    parts = relative_path.replace("\\", "/").split("/")
    for rule in SYNC_EXCLUDE:
        if rule.endswith("/"):
            matched = rule[:-1] in parts
        elif rule.startswith("*."):
            matched = any(part.endswith(rule[1:]) for part in parts)
        else:
            matched = rule in parts
        if matched:
            return True
    return False


def _iter_source_files() -> list[Path]:
    """列出需要同步的文件（绝对路径，排序保证输出稳定）。"""
    root = Path(LOCAL_PATH)
    collected: list[Path] = []
    for entry in SOURCE_PATHS:
        target = root / entry
        if target.is_dir():
            collected.extend(path for path in target.rglob("*") if path.is_file())
        elif target.is_file():
            collected.append(target)
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
    """读取本地缓存；缺失或损坏时返回空清单。"""
    try:
        data = json.loads(LOCAL_CACHE.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return {"files": {}, "signatures": {}}
    if not isinstance(data, dict):
        return {"files": {}, "signatures": {}}
    return {
        "files": data["files"] if isinstance(data.get("files"), dict) else {},
        "signatures": data["signatures"] if isinstance(data.get("signatures"), dict) else {},
    }


def _save_manifest(manifest: dict) -> None:
    try:
        LOCAL_CACHE.parent.mkdir(parents=True, exist_ok=True)
        LOCAL_CACHE.write_text(json.dumps(manifest, indent=2, sort_keys=True), encoding="utf-8")
    except OSError as exc:
        print(f"  [WARN] 无法写入本地同步缓存 {LOCAL_CACHE}: {exc}")


def _read_remote_manifest() -> dict | None:
    """读容器里的清单；没有或读不动时返回 None（视作需要全量同步）。"""
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
    """把「已成功传输」的清单写进容器，作为下次比对的基准。"""
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
        print(f"  [FAIL] 远端清单写入失败: {exc}")
        return False
    if result.returncode != 0:
        print(f"  [FAIL] 远端清单写入失败: {result.stderr.strip()}")
        return False
    return True


def _upload_changed_files(paths: list[Path]) -> bool:
    """把变化的文件打成 tar 流，经 ssh 直接解包进容器（不落本地临时文件）。"""
    root = Path(LOCAL_PATH)
    relative = [path.relative_to(root).as_posix() for path in paths]
    if not relative:
        print("  待上传文件数: 0")
        return True

    print(f"  待上传文件数: {len(relative)}")
    for name in relative[:12]:
        print(f"    {name}")
    if len(relative) > 12:
        print(f"    ... 其余 {len(relative) - 12} 个")

    remote = f"sudo docker exec -i {CONTAINER} tar -xf - -C {WORKSPACE}"
    print(f"  $ tar -cf - -C {root} ... ({len(relative)} files) | ssh {HOST} '{remote}'")
    ssh_proc = tar_proc = None
    try:
        # 先起 ssh 再喂 tar：tar 的标准输出直接接进容器的 tar，本地不需要暂存
        ssh_proc = subprocess.Popen([SSH_PATH or "ssh", HOST, remote], stdin=subprocess.PIPE)
        tar_proc = subprocess.Popen(
            [TAR_PATH or "tar", "-cf", "-", "-C", str(root), *relative],
            stdout=ssh_proc.stdin,
            stderr=subprocess.PIPE,
        )
        # 本地关掉管道副本：容器端 tar 退出后，本地 tar 才会收到 SIGPIPE 收场
        if ssh_proc.stdin is not None:
            ssh_proc.stdin.close()
        ssh_code = ssh_proc.wait(timeout=600)
        _, tar_err = tar_proc.communicate(timeout=120)
    except subprocess.TimeoutExpired:
        for proc in (tar_proc, ssh_proc):
            if proc is not None:
                proc.kill()
        print("  [FAIL] 上传超时 (600s)")
        return False
    except FileNotFoundError as exc:
        print(f"  [FAIL] 命令未找到: {exc}")
        return False

    # 远端提前退出时本地 tar 会被 SIGPIPE 打断，那种情况以 ssh 返回码为准
    if tar_proc.returncode != 0 and ssh_code == 0:
        detail = (tar_err or b"").decode("utf-8", "replace").strip()
        print(f"  [FAIL] tar 返回码: {tar_proc.returncode} {detail}")
        return False
    if ssh_code != 0:
        print(f"  [FAIL] 容器内解包返回码: {ssh_code}")
        return False
    print("  [OK] 完成")
    return True


_REMOTE_DELETE_SCRIPT = """\
import hashlib, json, os, sys

root = os.path.realpath({workspace!r})
removed = failed = 0


def fail(message):
    global failed
    failed += 1
    print(message, flush=True)


for line in sys.stdin:
    line = line.strip()
    if not line:
        continue
    try:
        record = json.loads(line)
        relative, expected = record["path"], record["sha256"]
    except (ValueError, KeyError, TypeError):
        fail("BAD_RECORD")
        continue
    if not isinstance(relative, str) or relative.startswith("/") or ".." in relative.split("/"):
        fail("REJECTED %s" % (relative,))
        continue
    target = os.path.realpath(os.path.join(root, relative))
    if target != root and not target.startswith(root + os.sep):
        fail("REJECTED %s" % (relative,))
        continue
    if not os.path.isfile(target):
        removed += 1
        continue
    try:
        with open(target, "rb") as handle:
            actual = hashlib.sha256(handle.read()).hexdigest()
    except OSError as exc:
        fail("FAILED %s: %s" % (relative, exc))
        continue
    if actual != expected:
        # 远端内容与上次同步的清单不一致（被手动改过），保留而不删除
        fail("CHANGED %s" % (relative,))
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
        fail("FAILED %s: %s" % (relative, exc))

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

    script = _REMOTE_DELETE_SCRIPT.format(workspace=WORKSPACE)
    command = f"sudo docker exec -i {CONTAINER} python3 -c {shlex.quote(script)}"
    batches = [deletions[index : index + 80] for index in range(0, len(deletions), 80)]
    for index, batch in enumerate(batches, 1):
        payload = "".join(json.dumps({"path": path, "sha256": digest}) + "\n" for path, digest in batch)
        print(f"  $ ssh {HOST} docker exec -i {CONTAINER} python3 -c <删除脚本>  (批次 {index}/{len(batches)})")
        try:
            result = subprocess.run(
                [SSH_PATH or "ssh", HOST, command],
                input=payload,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                encoding="utf-8",
                errors="replace",
                timeout=120,
            )
        except (OSError, subprocess.SubprocessError) as exc:
            print(f"  [FAIL] 删除批次 {index} 执行失败: {exc}")
            return False
        for line in (result.stdout or "").splitlines():
            print(f"  {line}")
        if result.returncode != 0:
            print(f"  [FAIL] 删除批次 {index} 返回码: {result.returncode}")
            return False
    print("  [OK] 完成")
    return True


def sync_code() -> bool:
    """同步代码到容器：只上传内容变化的文件，只在远端未被人改动时删除文件。

    本地缓存省掉重复读盘，但不能证明传输成功，所以「是否跳过上传」一律以远端清单为准。
    """
    print("\n── 同步代码（增量） ──")
    print(f"  本地: {LOCAL_PATH}")
    print(f"  远端: {HOST} → {CONTAINER}:{WORKSPACE}")

    root = Path(LOCAL_PATH)
    cached = _load_manifest()
    files: dict[str, str] = {}
    signatures: dict[str, str] = {}
    hashed = 0
    for path in _iter_source_files():
        relative = path.relative_to(root).as_posix()
        signature = _file_signature(path)
        if signature is None:
            continue
        signatures[relative] = signature
        digest = cached["files"].get(relative)
        if digest is None or cached["signatures"].get(relative) != signature:
            digest = _file_sha256(path)
            hashed += 1
        files[relative] = digest

    remote_manifest = _read_remote_manifest()
    remote_files = remote_manifest.get("files") if remote_manifest else None
    if not isinstance(remote_files, dict):
        print("  远端无清单记录 → 执行全量同步")
        remote_files = {}
    elif remote_files == files:
        _save_manifest({"files": files, "signatures": signatures})
        print("  [OK] 远端已是最新，跳过上传。")
        return True

    changed = [root / relative for relative, digest in files.items() if remote_files.get(relative) != digest]
    deletions = [(relative, digest) for relative, digest in remote_files.items() if relative not in files]
    print(f"  扫描完成: {len(files)} 个文件（本地读取 {hashed}，待上传 {len(changed)}，待删除 {len(deletions)}）")

    if changed:
        if not _upload_changed_files(changed):
            print("  [FAIL] 增量上传失败")
            return False
    else:
        print("  本地无新增/改动文件，仅处理删除。")

    if deletions and not _remove_remote_files(deletions):
        print("  [FAIL] 远端删除失败")
        return False

    if not _write_remote_manifest(files):
        return False
    _save_manifest({"files": files, "signatures": signatures})
    print(f"\n  [OK] 同步完成（缓存: {LOCAL_CACHE}）")
    return True


# ===== SSH 隧道 =====
def start_tunnel(mode: Mode) -> subprocess.Popen | None:
    """后台建立该模式需要的隧道；建不起来就返回 None 并继续执行。"""
    global tunnel_process
    tunnels = MODE_SPECS[mode].tunnels
    if not tunnels:
        return None

    cmd = [SSH_PATH or "ssh", "-g", "-N"]
    for local_port, remote_port in tunnels:
        # 只有 XR 服务要暴露给局域网里的 PICO 头显，Meshcat 留本机回环
        bind_host = "0.0.0.0" if local_port == XR_PORT else "127.0.0.1"
        cmd.extend(["-L", f"{bind_host}:{local_port}:127.0.0.1:{remote_port}"])
    cmd.append(HOST)

    print("\n── SSH 隧道 ──")
    print(f"  {' '.join(cmd)}")
    try:
        tunnel_process = subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    except FileNotFoundError:
        print("  [WARN] ssh 命令未找到，跳过隧道")
        return None
    time.sleep(1)  # 给 SSH 一点时间建连
    if tunnel_process.poll() is not None:
        print("  [WARN] SSH 隧道启动失败（进程已退出），继续执行...")
        tunnel_process = None
        return None
    print(f"  [OK] 隧道已建立 (PID {tunnel_process.pid})")
    return tunnel_process


def stop_tunnel() -> None:
    """关闭 SSH 隧道；没有隧道或进程已退出时什么都不做。"""
    global tunnel_process
    proc, tunnel_process = tunnel_process, None
    if proc is None or proc.poll() is not None:
        return
    print(f"\n  关闭 SSH 隧道 (PID {proc.pid})...")
    proc.terminate()
    try:
        proc.wait(timeout=5)
        print("  [OK] 隧道已关闭")
    except subprocess.TimeoutExpired:
        proc.kill()
        proc.wait()
        print("  [OK] 隧道已强制关闭")


def cleanup_container() -> None:
    """杀掉容器内残留的 Meshcat 进程，释放端口 7000。"""
    cmd = [
        SSH_PATH or "ssh",
        HOST,
        f"sudo docker exec {CONTAINER} bash -c 'pkill -f meshcat; pkill -f test_robot_viz'",
    ]
    try:
        subprocess.run(cmd, timeout=10, capture_output=True, check=False)
    except (OSError, subprocess.SubprocessError):
        pass  # 清理失败不该拦住启动


# ===== 容器内执行 =====
def run_in_container(
    mode: Mode,
    home_joint_deg: list[float] | None = None,
    sim_ik_backend: str = "placo",
    enable_home_path_preflight: bool = False,
) -> int:
    """前台运行容器内脚本，返回它的退出码。"""
    script = MODE_SPECS[mode].script
    if mode == "sim":
        if home_joint_deg is None or len(home_joint_deg) != 6:
            raise ValueError("simulation requires six calibrated home joint angles")
        if sim_ik_backend not in {"placo", "official", "shared-dls"}:
            raise ValueError("sim_ik_backend must be 'placo', 'official' or 'shared-dls'")
        joints = " ".join(format(value, ".10g") for value in home_joint_deg)
        script = f"{script} --home-joint-deg {joints} --ik-backend {sim_ik_backend}"
    elif mode in {"hardware", "hardware-dry-run"} and enable_home_path_preflight:
        # 真机入口默认关闭固定 Home 回位路径预检；这里只在显式要求时恢复拒绝式预检
        script = f"{script} --enable-home-path-preflight"

    # 本地有 TTY 时 ssh 加 -t、docker 用 -it（Ctrl+C 能传进容器）；否则全用 -i
    has_tty = sys.stdin.isatty()
    docker_cmd = f"sudo docker exec -{'it' if has_tty else 'i'} {CONTAINER} bash -c 'cd {WORKSPACE} && {script}'"
    cmd = [SSH_PATH or "ssh", *(["-t"] if has_tty else []), HOST, docker_cmd]

    print("\n── 启动容器内脚本 ──")
    print(f"  模式: {mode}")
    print(f"  $ {docker_cmd}")
    print("  (按 Ctrl+C 停止)\n")

    try:
        proc = subprocess.Popen(cmd)
        proc.wait()
        return proc.returncode
    except KeyboardInterrupt:
        return 0


def open_browser() -> None:
    """自动打开 Meshcat viewer（Windows）。"""
    url = "http://127.0.0.1:18081/static/"
    print(f"\n  打开浏览器: {url}")
    try:
        subprocess.Popen(["start", url], shell=True)
        return
    except FileNotFoundError:
        pass
    try:
        subprocess.Popen(["powershell.exe", "start", url])
    except FileNotFoundError:
        print(f"  [WARN] 无法自动打开浏览器，请手动访问: {url}")


# ===== 清理 =====
def cleanup() -> None:
    """统一清理入口（atexit 与信号处理共用）：只关隧道。

    容器内进程交给 Ctrl+C 的信号传播退出，这里不再补刀。
    """
    stop_tunnel()


def _on_signal(signum, _frame) -> None:
    """收到 SIGINT/SIGTERM：先收隧道再退出（沿用原有退出码 0/1）。"""
    cleanup()
    sys.exit(0 if signum == signal.SIGINT else 1)


# ===== CLI =====
def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="RM65 VR 遥操作一键启动",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=(
            "示例:\n"
            "  python teleop.py                                      # 交互菜单\n"
            "  python teleop.py -m sim --home-joint-deg 0 0 -60 0 60 0\n"
            "  python teleop.py -m sim --sim-ik-backend official --home-joint-deg 0 0 -60 0 60 0\n"
            "  python teleop.py -m hardware-dry-run                  # 真机检查，不运动\n"
            "  python teleop.py -m hardware                          # 直接启动真机\n"
            "  python teleop.py -m sync-only                         # 仅同步代码\n"
        ),
    )
    parser.add_argument(
        "--mode",
        "-m",
        choices=["menu", *MODE_SPECS],
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
        choices=["placo", "official", "shared-dls"],
        default="placo",
        help="仿真后端；official 需要 API2 SDK，shared-dls 还需要当前控制器 DH/工具帧快照",
    )
    parser.add_argument(
        "--enable-home-path-preflight",
        action="store_true",
        help="真机模式恢复固定 Home 回位路径预检（默认关闭：起点在 J3/J5 零位另一侧也能回位）",
    )
    return parser.parse_args()


# ===== Main =====
def check_prerequisites() -> None:
    """同步依赖本地的 ssh 与 tar，缺一个都送不进容器。"""
    missing = [name for name, path in (("ssh", SSH_PATH), ("tar", TAR_PATH)) if not path]
    if missing:
        print("[FAIL] 缺少必需命令: " + ", ".join(missing))
        sys.exit(1)
    print("[OK] 本地同步工具就绪 (ssh + tar)")


def _resolve_sim_home_joint(home_joint_deg: list[float] | None) -> list[float]:
    """仿真未显式给 home 时用固定六轴 Home [0, 0, -60, 0, 60, 0]。"""
    if home_joint_deg is None:
        return [0.0, 0.0, -60.0, 0.0, 60.0, 0.0]
    return home_joint_deg


def run(
    mode_str: str,
    home_joint_deg: list[float] | None = None,
    sim_ik_backend: str = "placo",
    enable_home_path_preflight: bool = False,
) -> None:
    """主流程：先同步代码，再选模式 → 建隧道 → 跑容器脚本。"""
    # 同步失败就不启动：宁可停，也不要跑容器里的旧代码
    if not sync_code():
        print("\n[FAIL] 代码同步失败，已中止启动")
        sys.exit(1)

    mode = show_menu() if mode_str == "menu" else mode_str
    if mode is None:
        return
    if mode == "sync-only":
        print("\n[OK] 同步完成，可手动 ssh 进容器调试。")
        return
    if mode == "sim":
        home_joint_deg = _resolve_sim_home_joint(home_joint_deg)

    atexit.register(cleanup)
    signal.signal(signal.SIGINT, _on_signal)
    signal.signal(signal.SIGTERM, _on_signal)

    start_tunnel(mode)
    if mode == "sim":
        cleanup_container()  # 清掉上次残留的 Meshcat，释放端口 7000
        open_browser()

    exit_code = run_in_container(
        mode,
        home_joint_deg=home_joint_deg,
        sim_ik_backend=sim_ik_backend,
        enable_home_path_preflight=enable_home_path_preflight,
    )
    cleanup()
    sys.exit(exit_code)


def main() -> None:
    check_prerequisites()
    args = parse_args()
    run(
        args.mode,
        home_joint_deg=args.home_joint_deg,
        sim_ik_backend=args.sim_ik_backend,
        enable_home_path_preflight=args.enable_home_path_preflight,
    )


if __name__ == "__main__":
    main()
