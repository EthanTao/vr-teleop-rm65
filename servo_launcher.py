"""Windows-friendly source/build/launch client for the single Linux Servo stack."""

from __future__ import annotations

import argparse
import hashlib
import io
import json
import os
from pathlib import Path, PureWindowsPath
import re
import shlex
import subprocess
import tarfile

import yaml

ROOT = Path(__file__).resolve().parent
SOURCE_PATHS = (
    "ros2_ws",
    "docker/Dockerfile.moveit",
    "docker/compose.moveit.yml",
    "docker/compose.wsl.yml",
    "docker/servo_entrypoint.sh",
    "docker/install_pc_service.sh",
    "pyproject.toml",
    ".dockerignore",
    "teleop.py",
    "servo_launcher.py",
)


def is_windows_host() -> bool:
    return os.name == "nt"


def wsl_distro_registered(name: str) -> bool:
    """Read WSL registration locally without launching WSL or its distributions."""
    if not is_windows_host():
        return False
    try:
        import winreg

        with winreg.OpenKey(winreg.HKEY_CURRENT_USER, r"Software\Microsoft\Windows\CurrentVersion\Lxss") as registry:
            index = 0
            while True:
                try:
                    key_name = winreg.EnumKey(registry, index)
                except OSError:
                    break
                index += 1
                try:
                    with winreg.OpenKey(registry, key_name) as distribution:
                        value, _ = winreg.QueryValueEx(distribution, "DistributionName")
                        if isinstance(value, str) and value.casefold() == name.casefold():
                            return True
                except OSError:
                    continue  # An incomplete registration does not hide other distributions.
    except (ImportError, OSError):
        pass
    return False


def resolve_target(args) -> str:
    if args.target is not None:
        return args.target
    if not is_windows_host():
        return "local"
    if args.mode in {"sim", "test"} and wsl_distro_registered(args.wsl_distro):
        return "wsl"
    return "remote"


def windows_path_to_wsl(path: str | Path) -> str:
    """Offline preview using a clean WSL distribution's default drive mounts.

    Execution uses wslpath instead, so an actual custom automount configuration
    cannot silently redirect a build to this preview path.
    """
    windows = PureWindowsPath(str(path))
    if not windows.is_absolute() or not re.fullmatch(r"[A-Za-z]:", windows.drive):
        raise ValueError("WSL 仿真需要 Windows 盘符下的仓库绝对路径；不支持 UNC 网络目录")
    return "/mnt/" + windows.drive[0].lower() + "/" + "/".join(windows.parts[1:])


def wsl_prefix(args) -> list[str]:
    return ["wsl.exe", "--distribution", args.wsl_distro, "--user", "root"]


def wsl_repository_path(args) -> str:
    windows_path_to_wsl(ROOT)  # Reject unsupported Windows roots before starting a process.
    result = subprocess.run(
        [*wsl_prefix(args), "--exec", "wslpath", "-a", "-u", str(ROOT)],
        capture_output=True,
        text=True,
        encoding="utf-8",
        check=True,
    )
    converted = result.stdout.rstrip("\r\n")
    if not converted.startswith("/") or any(character in converted for character in "\r\n\0"):
        raise ValueError("wslpath 未返回有效的仓库绝对路径")
    return converted


def wsl_cached_image_matches(args, repository: str, source_sha: str) -> bool:
    """Inspect only the local daemon; an absent/unknown image always rebuilds."""
    try:
        result = subprocess.run(
            [
                *wsl_prefix(args),
                "--cd",
                repository,
                "--exec",
                "docker",
                "image",
                "inspect",
                "rm65-servo:humble",
                "--format",
                "{{json .Config.Labels}}",
            ],
            capture_output=True,
            text=True,
            encoding="utf-8",
            check=True,
            timeout=10.0,
        )
        labels = json.loads(result.stdout)
    except (OSError, ValueError, TypeError, subprocess.CalledProcessError, subprocess.TimeoutExpired):
        return False
    return (
        isinstance(labels, dict)
        and labels.get("org.rm65.source-sha") == source_sha
        and labels.get("org.rm65.xr-sdk") == ("ON" if args.input == "pico" else "OFF")
    )


def remote_cached_image_matches(args, source_sha: str) -> bool:
    """Reuse a prepared board image only when source and SDK labels both match."""
    docker = (["sudo", "-n"] if args.sudo_docker else []) + ["docker"]
    command = docker + ["image", "inspect", "rm65-servo:humble", "--format", "{{json .Config.Labels}}"]
    try:
        result = subprocess.run(
            ["ssh", "-o", "BatchMode=yes", args.host, shlex.join(command)],
            capture_output=True, text=True, encoding="utf-8", check=True, timeout=15.0,
        )
        labels = json.loads(result.stdout)
    except (OSError, ValueError, TypeError, AttributeError, subprocess.CalledProcessError, subprocess.TimeoutExpired):
        return False
    return (
        isinstance(labels, dict)
        and labels.get("org.rm65.source-sha") == source_sha
        and labels.get("org.rm65.xr-sdk") == ("ON" if args.input == "pico" else "OFF")
    )


def source_files(root: Path = ROOT):
    for relative in SOURCE_PATHS:
        base = root / relative
        if not base.exists():
            continue
        # WindowsPath comparison folds case; PosixPath comparison does not.
        # Hash/archive the same explicit relative-name order on both hosts.
        paths = [base] if base.is_file() else sorted(base.rglob("*"), key=lambda p: p.relative_to(root).as_posix())
        for path in paths:
            if not path.is_file() or path.is_symlink():
                continue
            name = path.relative_to(root)
            if any(part in {".git", "__pycache__", "build", "install", "log", "vendor"} for part in name.parts):
                continue
            if path.suffix not in {".pyc", ".pyo"}:
                yield path


def source_digest() -> str:
    digest = hashlib.sha256()
    for path in source_files():
        digest.update(path.relative_to(ROOT).as_posix().encode("utf-8") + b"\0")
        digest.update(path.read_bytes())
    return digest.hexdigest()


def source_archive() -> bytes:
    buffer = io.BytesIO()
    with tarfile.open(fileobj=buffer, mode="w:gz") as archive:
        for path in source_files():
            archive.add(path, arcname=path.relative_to(ROOT).as_posix(), recursive=False)
    return buffer.getvalue()


def validate_options(args) -> None:
    args.target = resolve_target(args)
    if args.hand != "off" and args.mode != "sim":
        raise ValueError("--hand mock 仅供 sim 离线演练；真实手控制尚未现场验收")
    if args.rviz and (args.target != "wsl" or args.mode != "sim" or not args.viewer):
        raise ValueError("--rviz 仅用于 WSL sim 调试，且不能与 --no-viewer 同用")
    if args.simulator is not None and args.mode != "sim":
        raise ValueError("--simulator 仅用于 sim；不会改变真机接口或测试集合")
    args.simulator = args.simulator or "mujoco"
    if args.target == "wsl" and args.mode not in {"sim", "test"}:
        raise ValueError("WSL 本次仅用于本机 sim/test；hardware、hardware-dry-run 和 sync-only 不允许使用 WSL")
    if not args.wsl_distro or args.wsl_distro.startswith("-") or any(c in args.wsl_distro for c in "\0\r\n"):
        raise ValueError("invalid WSL distribution name")
    if not re.fullmatch(r"[A-Za-z0-9_][A-Za-z0-9_.@:-]*", args.host):
        raise ValueError("invalid SSH host alias")
    if args.input is None:
        args.input = "mock" if args.mode in {"sim", "test", "sync-only"} else "pico"
    if args.input == "manual" and args.mode != "sim":
        raise ValueError("无手柄面板输入仅用于 sim；不会连接真机或真机 Dry Run")
    if args.mode in {"hardware", "hardware-dry-run"} and args.input == "mock":
        raise ValueError("模拟输入不能连接真机或真机 Dry Run")
    if args.mode in {"hardware", "hardware-dry-run"} and not args.udp_ip:
        raise ValueError("须显式提供 --udp-ip：机器人能访问的 Linux 板地址")
    if args.mode == "hardware":
        if args.live_ack != "RM65 LIVE VALIDATION" or not args.hardware_validation:
            raise ValueError("真机执行尚未验收：需要现场证据文件和显式 --live-ack，当前请用 sim 或 hardware-dry-run")
        path = (ROOT / args.hardware_validation).resolve()
        config_root = ROOT / "ros2_ws/src/rm65_servo_teleop/config"
        if not path.is_file() or path.parent != config_root:
            raise ValueError("验证文件必须位于 ros2_ws/src/rm65_servo_teleop/config，随镜像固定版本")
        value = yaml.safe_load(path.read_text(encoding="utf-8"))
        checks = (
            "allow_hardware_motion",
            "frames_verified",
            "stop_verified",
            "speed_units_verified",
            "firmware_compatible",
            "collision_model_verified",
        )
        if not isinstance(value, dict) or any(value.get(key) is not True for key in checks):
            raise ValueError("现场验证记录不完整；不会启动硬件执行")
        args.validation_in_image = "/opt/rm65_ws/install/share/rm65_servo_teleop/config/" + path.name


def commands(args, *, source_sha: str | None = None) -> list[list[str]]:
    docker = (["sudo", "-n"] if args.sudo_docker and args.target != "wsl" else []) + ["docker"]
    compose = docker + ["compose", "-f", "docker/compose.moveit.yml"]
    wsl_viewer = args.target == "wsl" and args.mode == "sim" and args.viewer
    if wsl_viewer:
        compose.extend(["-f", "docker/compose.wsl.yml"])
    result = [
        compose
        + [
            "build",
            "--build-arg",
            f"RM65_WITH_XR_SDK={'ON' if args.input == 'pico' else 'OFF'}",
            "--build-arg",
            f"RM65_SOURCE_SHA={source_digest() if source_sha is None else source_sha}",
        ]
    ]
    if args.mode == "test":
        result.append(
            docker
            + [
                "run",
                "--rm",
                "--network",
                "none",
                "rm65-servo:humble",
                "bash",
                "-c",
                "source /opt/rm65_ws/install/setup.bash && cd /opt/rm65_ws && "
                "colcon test --merge-install --packages-select rm65_servo_teleop --event-handlers console_direct+ && "
                "colcon test-result --verbose",
            ]
        )
        return result
    launch = [
        "ros2",
        "launch",
        "rm65_servo_teleop",
        "teleop.launch.py",
        f"mode:={args.mode}",
        f"input_source:={args.input}",
        f"foxglove:={'true' if args.viewer else 'false'}",
        f"rviz:={'true' if wsl_viewer and args.rviz else 'false'}",
        f"arm_ip:={args.arm_ip}",
    ]
    if args.udp_ip:
        launch.append(f"udp_ip:={args.udp_ip}")
    if args.mode == "sim":
        launch.extend([f"simulator:={args.simulator}", f"mujoco_viewer:={'true' if wsl_viewer else 'false'}"])
        launch.append(f"hand_mode:={args.hand}")
    if args.mode == "hardware":
        launch.extend([f"hardware_validation_file:={args.validation_in_image}", f"operator_ack:={args.live_ack}"])
    if args.model_only:
        launch.append("print_model_only:=true")
    result.append(
        compose
        + [
            "run",
            "--rm",
            "--service-ports",
            "--name",
            "rm65-servo",
            "-e",
            f"RM65_START_XR_SERVICE={'1' if args.input == 'pico' and not args.model_only else '0'}",
            "servo",
            *launch,
        ]
    )
    return result


def run(args) -> int:
    validate_options(args)
    source_sha = None if args.mode == "sync-only" else source_digest()
    planned = [] if args.mode == "sync-only" else commands(args, source_sha=source_sha)
    if args.print_command:
        if args.target == "remote":
            print(f"先同步明确的源码集合到 {args.host}:{args.remote_dir}；不包含旧 Python 控制包。")
            print("板端镜像缓存规则：源码摘要与 SDK 配置均匹配才复用，否则构建。")
        elif args.target == "wsl":
            wsl_root = windows_path_to_wsl(ROOT)
            print("WSL 路径按默认 /mnt/ 挂载预览；实际运行使用 wslpath 解析。此预览不启动 WSL。")
            print("实际运行会检查本机镜像的源码和 XR SDK 标签，匹配时跳过 build。")
            planned = [[*wsl_prefix(args), "--cd", wsl_root, "--exec", *command] for command in planned]
        for command in planned:
            print(shlex.join(command))
        return 0
    if args.mode == "sim" and args.input == "manual" and args.viewer and not args.model_only:
        print("无手柄控制面板：http://127.0.0.1:8766；仿真服务就绪后在浏览器打开。", flush=True)
    if args.target == "remote":
        remote = shlex.quote(args.remote_dir)
        subprocess.run(
            ["ssh", "-o", "BatchMode=yes", args.host, f"mkdir -p -- {remote} && tar -xzf - -C {remote}"],
            input=source_archive(),
            check=True,
        )
        if planned and remote_cached_image_matches(args, source_sha):
            print("板端镜像与源码、XR SDK 配置匹配，复用已构建镜像。")
            planned = planned[1:]
        for index, command in enumerate(planned):
            options = ["ssh", "-t"] if os.isatty(0) else ["ssh"]
            if args.viewer and index == len(planned) - 1 and args.mode not in {"test", "sync-only"}:
                options.extend(["-o", "ExitOnForwardFailure=yes", "-L", "127.0.0.1:8765:127.0.0.1:8765"])
                if args.mode == "sim" and args.input == "manual":
                    options.extend(["-L", "127.0.0.1:8766:127.0.0.1:8766"])
            subprocess.run([*options, args.host, f"cd {remote} && {shlex.join(command)}"], check=True)
    elif args.target == "wsl":
        wsl_root = wsl_repository_path(args)
        if wsl_cached_image_matches(args, wsl_root, source_sha):
            print("本机镜像与源码、XR SDK 配置匹配，复用已构建镜像。")
            planned = planned[1:]
        for command in planned:
            subprocess.run([*wsl_prefix(args), "--cd", wsl_root, "--exec", *command], check=True)
    else:
        if args.mode == "sync-only":
            return 0
        for command in planned:
            subprocess.run(command, cwd=ROOT, check=True)
    return 0


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--mode", "-m", choices=["sim", "test", "hardware-dry-run", "hardware", "sync-only"], default="sim"
    )
    parser.add_argument("--input", choices=["manual", "mock", "pico", "external"])
    parser.add_argument("--hand", choices=["off", "mock"], default="off", help="可选灵巧手流程演练，默认关闭；mock 无接触物理或真机发送")
    parser.add_argument(
        "--simulator", choices=["mujoco", "mock"], help="sim 默认使用 MuJoCo 物理仿真；mock 用于快速控制图诊断"
    )
    parser.add_argument(
        "--target",
        choices=["remote", "local", "wsl"],
        help="Windows sim/test 默认优先使用已注册的 RM65-Servo WSL；其余默认行为保持不变",
    )
    parser.add_argument("--wsl-distro", default="RM65-Servo", help="本机独立仿真的 WSL 发行版名称")
    parser.add_argument("--host", default="rm")
    parser.add_argument("--remote-dir", default="vr-teleop-rm65-servo")
    parser.add_argument("--sudo-docker", action=argparse.BooleanOptionalAction, default=os.name == "nt")
    parser.add_argument("--viewer", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--rviz", action="store_true", help="WSL 仿真额外打开 RViz 调试窗口；默认只显示 MuJoCo")
    parser.add_argument("--arm-ip", default="192.168.10.18")
    parser.add_argument("--udp-ip", default="")
    parser.add_argument("--hardware-validation", default="")
    parser.add_argument("--live-ack", default="")
    parser.add_argument("--model-only", action="store_true", help="只输出模型与部署哈希，不启动任何ROS节点")
    parser.add_argument("--print-command", action="store_true", help="仅打印将使用的命令，不同步或启动")
    args = parser.parse_args(argv)
    try:
        return run(args)
    except (ValueError, OSError, subprocess.CalledProcessError) as exc:
        print(f"[FAIL] {exc}")
        return 2
    except KeyboardInterrupt:
        print("\n控制会话已中断；设备级停机仍须由反馈和停止确认验证。")
        return 130
