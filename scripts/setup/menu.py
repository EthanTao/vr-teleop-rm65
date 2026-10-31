"""Interactive startup UX only: the current Servo controller remains unchanged."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import PurePosixPath
import ipaddress
import re
import shlex
import sys

from scripts.setup import launcher

DEFAULT_HOST = "192.168.0.115"
DEFAULT_USER = "rm"
DEFAULT_REMOTE_DIR = "/home/rm/EthanTao"
HOST_PATTERN = r"(?:[A-Za-z0-9_][A-Za-z0-9_.-]*@)?[A-Za-z0-9][A-Za-z0-9_.-]*"
BARE_IPV4 = re.compile(r"\d{1,3}(?:\.\d{1,3}){3}")


def interactive_available() -> bool:
    return sys.stdin.isatty() and sys.stdout.isatty()


def prompt_value(label: str, default: str = "") -> str:
    suffix = f" [{default}]" if default else ""
    return input(f"{label}{suffix}：") or default


@dataclass(frozen=True)
class BoardSettings:
    host: str = DEFAULT_HOST
    remote_dir: str = DEFAULT_REMOTE_DIR
    udp_ip: str = "192.168.10.100"
    arm_ip: str = "192.168.10.18"


def edit_settings(settings: BoardSettings) -> BoardSettings:
    """Apply session-local changes only after all four values pass validation."""
    launcher.status("[高级设置] 直接回车保留当前值；只对本次菜单有效，不修改板端。")
    candidate = BoardSettings(
        host=prompt_value("板端 SSH 地址/别名（裸 IPv4 自动补 rm@）", settings.host),
        remote_dir=prompt_value("板端项目绝对目录", settings.remote_dir),
        udp_ip=prompt_value("板端反馈网口 IPv4", settings.udp_ip),
        arm_ip=prompt_value("机械臂控制器 IPv4", settings.arm_ip),
    )
    validate_location(candidate.host, candidate.remote_dir)
    launcher.canonical_ipv4(candidate.udp_ip)
    launcher.canonical_ipv4(candidate.arm_ip)
    return candidate


def normalize_host(value: str) -> str:
    """A bare address would otherwise connect as the Windows account name."""
    if not re.fullmatch(HOST_PATTERN, value):
        raise ValueError("板端地址必须是 SSH 别名、主机名或 user@IPv4，不能包含选项或命令。")
    if "@" in value:
        return value
    if BARE_IPV4.fullmatch(value):
        try:
            ipaddress.IPv4Address(value)
        except ipaddress.AddressValueError as exc:
            raise ValueError("板端 IPv4 地址无效。") from exc
        return f"{DEFAULT_USER}@{value}"
    return value


def validate_location(host: str, remote_dir: str) -> tuple[str, PurePosixPath]:
    host = normalize_host(host)
    path = PurePosixPath(remote_dir)
    if (
        not path.is_absolute()
        or ".." in path.parts
        or "\\" in remote_dir
        or any(ord(character) < 32 or ord(character) == 127 for character in remote_dir)
    ):
        raise ValueError("板端项目目录必须是无控制字符、无 .. 的 Linux 绝对路径。")
    return host, path


def run_board_action(
    action: str,
    host: str,
    remote_dir: str,
    *,
    arm_ip: str = "192.168.10.18",
    udp_ip: str = "192.168.10.100",
) -> int:
    """Call the existing board tool; never sync, replace, stop, or grant motion."""
    if action not in {"start-readonly", "status", "input"}:
        raise ValueError("菜单仅允许 start-readonly、status 或 input 板端操作。")
    host, path = validate_location(host, remote_dir)
    launcher.canonical_ipv4(arm_ip)
    launcher.canonical_ipv4(udp_ip)
    script = shlex.quote(str(path / "environment.sh"))
    # Old deployed helpers reject these flags instead of silently using another
    # controller. Defaults retain compatibility with the current board image.
    address_options = ""
    if (arm_ip, udp_ip) != ("192.168.10.18", "192.168.10.100"):
        address_options = shlex.join(["--arm-ip", arm_ip, "--udp-ip", udp_ip])
    command = f"{script} {action}" + (f" {address_options}" if address_options else "")
    if action == "start-readonly":
        command += f" && {script} status" + (f" {address_options}" if address_options else "")
        launcher.status("[只读] 启动或复用唯一匹配会话，无需手柄在线；冲突即拒绝，不覆盖、不重启。")
    else:
        launcher.status("[查询] 仅观察现有会话，不启动、停止或切换其运动模式。")
    launcher.run_command(["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=8", "--", host, command], timeout=90)
    if action == "start-readonly":
        launcher.status("[只读] 反馈检查通过；未使能运动。PICO 之后可直连板端 WiFi 地址:63901，Controller + Send。")
    elif action == "input":
        launcher.status("[输入] 0 帧表示本次未收到手柄输入，不等于机械臂连接失败。查询不会使能运动。")
    return 0


def run_execution(settings: BoardSettings | None = None) -> int | None:
    settings = settings or BoardSettings()
    host, _ = validate_location(settings.host, settings.remote_dir)
    launcher.canonical_ipv4(settings.udp_ip)
    launcher.canonical_ipv4(settings.arm_ip)
    launcher.status("[执行] 需已有现场验收记录；没有记录可直接回车返回，选择只读连接。")
    launcher.status("[执行] 不自动停止现有会话，不自动回位。")
    validation = prompt_value("现场验收文件路径（回车返回菜单）")
    if not validation:
        launcher.status("[返回] 未启动执行；连接真机读反馈不需要此文件。")
        return None
    acknowledgement = prompt_value(f'人工确认：完整输入 "{launcher.ACK}"（留空返回）')
    if not acknowledgement:
        launcher.status("[返回] 未确认运动，未启动执行。")
        return None
    if acknowledgement != launcher.ACK:
        raise ValueError("人工确认未匹配，未启动执行。")
    return launcher.main(
        "real",
        [
            "--host",
            host,
            "--remote-dir",
            settings.remote_dir,
            "--udp-ip",
            settings.udp_ip,
            "--arm-ip",
            settings.arm_ip,
            "--hardware-validation",
            validation,
            "--live-ack",
            acknowledgement,
        ],
    )


def run_menu(*, check_only: bool = False, real_only: bool = False) -> int:
    launcher.status("\nRM65 真机菜单（MoveIt Servo）" if real_only else "\nRM65 统一启动菜单（MoveIt Servo）")
    launcher.status("[连接] 真机：PICO 头显 → 板端 WiFi → 机械臂；电脑负责启动/查询，无需先连接手柄。")
    launcher.status("[控制] teleop.py → servo_launcher.py → ROS / MoveIt Servo；只读操作复用已部署环境，不恢复旧后端。")
    if real_only:
        choices = {
            "1": "连接真机（读反馈，不运动）",
            "2": "查询真实机械臂反馈",
            "3": "查询 PICO 输入（5 秒，不运动）",
            "4": "正式执行（需要真实现场验证文件及人工确认）",
        }
    else:
        choices = {
            "1": "无手柄仿真（本机 Windows / WSL）",
            "2": "PICO 仿真（本机 Windows / WSL）",
            "3": "连接真机（读反馈，不运动）",
            "4": "正式真机执行（需要真实现场验证文件及人工确认）",
        }
    choices["5"] = "高级设置（仅更换地址或目录时使用）"
    settings = BoardSettings()
    try:
        while True:
            launcher.status(f"[板端] {normalize_host(settings.host)} | {settings.remote_dir}")
            launcher.status(f"[地址] 控制器 {settings.arm_ip} → 反馈网口 {settings.udp_ip}（更换时选 5）")
            for number, text in choices.items():
                launcher.status(f"  [{number}] {text}")
            launcher.status("  [0] 退出（直接回车也退出）")
            if check_only:
                launcher.status("[CheckOnly] 只显示菜单与连接说明，不等待输入，不查询、不启动任何进程。")
                return 0
            if not interactive_available():
                raise ValueError(
                    "菜单需要交互终端；自动化请使用 start.py sim/pico/real 并提供显式参数。--check-only 可离线查看菜单。"
                )
            choice = input("选择编号：").strip()
            if choice in {"", "0"}:
                launcher.status("[取消] 菜单已退出；没有继续操作，现有板端会话保持原状。")
                return 0
            if choice not in choices:
                launcher.status("无效编号，请输入 0–5；不会默认启动任何模式。")
                continue
            try:
                if choice == "5":
                    settings = edit_settings(settings)
                    launcher.status("[设置] 本次菜单参数已更新，尚未连接或修改板端。")
                    continue
                if choice == "4":
                    result = run_execution(settings)
                    if result is None:
                        continue
                    if result == 2:
                        launcher.status("[未完成] 执行入口返回 2；不自动重试或切换模式，可选择只读连接。")
                        continue
                    return result
                if not real_only and choice in {"1", "2"}:
                    return launcher.main("manual" if choice == "1" else "pico", [])
                action = {"1": "start-readonly", "2": "status", "3": "input"}[choice] if real_only else "start-readonly"
                return run_board_action(
                    action,
                    settings.host,
                    settings.remote_dir,
                    arm_ip=settings.arm_ip,
                    udp_ip=settings.udp_ip,
                )
            except ValueError as exc:
                launcher.status(f"[未继续] {exc}；已返回菜单。")
    except EOFError:
        launcher.status("[取消] 输入已结束，未继续启动；现有板端会话保持原状。")
        return 0
