"""Python shortcuts for the existing teleop.py chain; never a second controller."""

from __future__ import annotations

import argparse
import base64
import ctypes
from contextlib import contextmanager
from datetime import datetime
import ipaddress
import json
import logging
import os
from pathlib import Path
import re
import subprocess
import sys
import traceback
from typing import Sequence

ROOT = Path(__file__).resolve().parents[2]
LOGGER = logging.getLogger("rm65.startup")
ACK = "RM65 LIVE VALIDATION"


def status(message: str) -> None:
    print(message, flush=True)
    LOGGER.info(message)


def system_tool(name: str) -> str:
    return str(Path(os.environ.get("SystemRoot", r"C:\Windows")) / "System32" / name)


def run_command(command: Sequence[str], *, timeout: float | None = None) -> None:
    """Keep live child output in its console; the journal records stages and failures."""
    LOGGER.info("Command: %s", subprocess.list2cmdline(list(command)))
    subprocess.run(
        list(command), cwd=ROOT, env={**os.environ, "PYTHONUTF8": "1", "WSL_UTF8": "1"}, check=True, timeout=timeout
    )


def ps_literal(value: str | Path) -> str:
    return "'" + str(value).replace("'", "''") + "'"


def ps_command(source: str) -> list[str]:
    encoded = base64.b64encode(source.encode("utf-16-le")).decode("ascii")
    return [
        system_tool("WindowsPowerShell/v1.0/powershell.exe"),
        "-NoProfile",
        "-NonInteractive",
        "-ExecutionPolicy",
        "Bypass",
        "-EncodedCommand",
        encoded,
    ]


def canonical_ipv4(value: str) -> str:
    try:
        address = ipaddress.IPv4Address(value)
    except ipaddress.AddressValueError as exc:
        raise ValueError("必须提供规范的 IPv4 地址。") from exc
    if (
        str(address) != value
        or address.is_unspecified
        or address.is_loopback
        or address.is_multicast
        or int(address) >= 0xE0000000
    ):
        raise ValueError("必须提供单播 LAN IPv4 地址，不能使用通配、回环或组播地址。")
    return value


def get_pico_candidates() -> list[dict]:
    """Use Windows' structured network API output, never localized ipconfig text."""
    source = r"""
$ErrorActionPreference = 'Stop'
[Console]::OutputEncoding = [Text.UTF8Encoding]::new($false)
$items = @(foreach ($adapter in @(Get-NetAdapter -Physical | Where-Object Status -eq 'Up')) {
    foreach ($ip in @(Get-NetIPAddress -InterfaceIndex $adapter.ifIndex -AddressFamily IPv4)) {
        if ($ip.AddressState -eq 'Preferred' -and $ip.IPAddress -notmatch '^(127\.|169\.254\.|0\.)') {
            [pscustomobject]@{address=[string]$ip.IPAddress; interface=[string]$adapter.Name;
                wireless=([int]$adapter.NdisPhysicalMedium -in @(1, 9))}
        }
    }
})
ConvertTo-Json -InputObject $items -Compress
"""
    result = subprocess.run(
        ps_command(source), capture_output=True, text=True, encoding="utf-8", check=True, timeout=30, cwd=ROOT
    )
    candidates = json.loads(result.stdout.lstrip("\ufeff"))
    if not isinstance(candidates, list) or any(not isinstance(item, dict) for item in candidates):
        raise ValueError("无法读取 Windows 网卡列表，未配置网络。")
    return candidates


def select_pico_address(candidates: list[dict], requested: str | None = None) -> dict:
    if requested:
        canonical_ipv4(requested)
        selected = [item for item in candidates if item.get("address") == requested]
    else:
        selected = [item for item in candidates if item.get("wireless") is True] or candidates
    if len(selected) != 1:
        available = "; ".join(f"{item.get('interface')}: {item.get('address')}" for item in candidates)
        raise ValueError("无法唯一选择 PICO 网络地址，请用 --listen-address 指定本机实际 LAN IP。可选：" + available)
    canonical_ipv4(selected[0]["address"])
    return selected[0]


def network_migration_source(address: str, distro: str, state_path: Path, helper: Path) -> str:
    """The helper independently checks ownership before every removal or change."""
    return f"""
$ErrorActionPreference = 'Stop'
$helper = {ps_literal(helper)}
$stateFile = {ps_literal(state_path)}
$address = {ps_literal(address)}
$distro = {ps_literal(distro)}
if (Test-Path -LiteralPath $stateFile -PathType Leaf) {{
    $owned = Get-Content -LiteralPath $stateFile -Raw -Encoding UTF8 | ConvertFrom-Json
    if ($null -eq $owned -or $null -eq $owned.PSObject.Properties['listen_address']) {{
        throw "Cannot inspect PICO ownership state: $stateFile"
    }}
    if ([string]$owned.listen_address -ne $address) {{
        & $helper -Remove -WhatIf
        & $helper -Remove -Confirm:$false
    }}
}}
& $helper -ListenAddress $address -Distro $distro -WhatIf
& $helper -ListenAddress $address -Distro $distro -Confirm:$false
"""


@contextmanager
def wsl_session(distro: str):
    """Keep WSL awake through UAC; closing stdin ends only this idle cat process."""
    child = subprocess.Popen(
        [system_tool("wsl.exe"), "--distribution", distro, "--exec", "cat"],
        stdin=subprocess.PIPE,
        stdout=subprocess.DEVNULL,
        cwd=ROOT,
        env={**os.environ, "WSL_UTF8": "1"},
    )
    try:
        if child.poll() is not None:
            raise OSError("WSL 保活会话未能启动；未继续网络配置。")
        yield
    finally:
        try:
            child.communicate(timeout=10)
        except subprocess.TimeoutExpired:
            child.terminate()
            try:
                child.wait(timeout=5)
            except subprocess.TimeoutExpired:
                child.kill()
                child.wait(timeout=5)


def setup_pico_network(address: str, distro: str) -> None:
    helper = ROOT / "scripts/setup/connect_pico_wsl.ps1"
    if not helper.is_file():
        raise ValueError(f"缺少 PICO 网络助手：{helper}")
    log = ROOT / "build/setup/local-servo/pico-network-last.log"
    log.parent.mkdir(parents=True, exist_ok=True)
    migration = network_migration_source(address, distro, log.with_name("pico-portproxy.json"), helper)
    source = f"""
$ErrorActionPreference = 'Stop'
$env:WSL_UTF8 = '1'
[Console]::OutputEncoding = [Text.UTF8Encoding]::new($false)
try {{
    Start-Transcript -LiteralPath {ps_literal(log)} -Force | Out-Null
    {migration}
    Stop-Transcript | Out-Null
    exit 0
}} catch {{
    [Console]::Error.WriteLine($_.Exception.Message)
    try {{ Stop-Transcript | Out-Null }} catch {{}}
    exit 1
}}
"""
    command = ps_command(source)
    status(f"[网络] 本次配置日志：{log}")
    with wsl_session(distro):
        if ctypes.windll.shell32.IsUserAnAdmin():
            run_command(command)
        else:
            status("[网络] Windows 将请求管理员权限，仅用于本项目 PICO TCP63901 转发；取消即停止启动。")
            # Only the short-lived networking process is elevated, not Python/ROS.
            encoded = command[-1]
            elevate = f"""
$ErrorActionPreference = 'Stop'
try {{
    $child = Start-Process -FilePath {ps_literal(command[0])} -ArgumentList @('-NoProfile', '-NonInteractive', '-ExecutionPolicy', 'Bypass', '-EncodedCommand', '{encoded}') -Verb RunAs -WindowStyle Hidden -Wait -PassThru
    exit $child.ExitCode
}} catch {{ [Console]::Error.WriteLine($_.Exception.Message); exit 1 }}
"""
            run_command(ps_command(elevate))
    status(f"[网络] 转发配置完成；详情：{log}")


def make_parser(kind: str) -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="RM65 Python 启动入口（复用 teleop.py，不改变运动控制与准入）。")
    parser.add_argument("--check-only", action="store_true", help="仅离线验证参数并打印命令，不启动 WSL/SSH/网络配置")
    parser.add_argument("--pause", action="store_true", help="退出前按 Enter；终端调用默认不等待")
    if kind in {"manual", "pico"}:
        parser.add_argument("--distro", default="RM65-Servo")
        parser.add_argument("--listen-address", help="仅 PICO 使用：本机实际 LAN IPv4 地址")
        parser.add_argument("--rviz", action="store_true")
        parser.add_argument("--no-viewer", action="store_true")
    elif kind == "menu":
        pass  # Interactive selection only; explicit deployment parameters use the existing shortcuts.
    elif kind == "real":
        parser.add_argument("--udp-ip", default="")
        parser.add_argument("--hardware-validation", default="")
        parser.add_argument("--live-ack", default="")
        parser.add_argument("--host", default="rm")
        parser.add_argument("--remote-dir", default="EthanTao")
        parser.add_argument("--arm-ip", default="192.168.10.18")
        parser.add_argument("--dry-run", action="store_true", help="连接真实控制器读反馈，不创建运动接口；不是离线预览")
    else:
        raise ValueError(f"Unknown shortcut: {kind}")
    return parser


def sim_arguments(kind: str, args: argparse.Namespace) -> list[str]:
    if not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9._-]{0,63}", args.distro):
        raise ValueError("--distro 必须是有效的 WSL 发行版名称。")
    if kind != "pico" and args.listen_address:
        raise ValueError("--listen-address 仅供 PICO 仿真使用。")
    if args.listen_address:
        canonical_ipv4(args.listen_address)
    if args.rviz and args.no_viewer:
        raise ValueError("--rviz 不能与 --no-viewer 同用。")
    options = ["--mode", "sim", "--target", "wsl", "--wsl-distro", args.distro, "--input", kind]
    if args.rviz:
        options.append("--rviz")
    if args.no_viewer:
        options.append("--no-viewer")
    return options


def real_arguments(args: argparse.Namespace) -> list[str]:
    missing = []
    if not args.udp_ip:
        missing.append("--udp-ip：机器人能访问的 Linux 板 IPv4 地址")
    if not args.dry_run:
        if not args.hardware_validation:
            missing.append("--hardware-validation：有效现场验证文件（config 内）")
        if args.live_ack != ACK:
            missing.append(f'--live-ack：人工提供的 "{ACK}"')
    if missing:
        raise ValueError(
            "尚未启动真机。必须显式提供：\n  "
            + "\n  ".join(missing)
            + "\n--check-only 仅预览；--dry-run 仍会连接真实控制器读取反馈。"
        )
    canonical_ipv4(args.udp_ip)
    canonical_ipv4(args.arm_ip)
    options = [
        "--mode",
        "hardware-dry-run" if args.dry_run else "hardware",
        "--target",
        "remote",
        "--host",
        args.host,
        "--remote-dir",
        args.remote_dir,
        "--input",
        "pico",
        "--arm-ip",
        args.arm_ip,
        "--udp-ip",
        args.udp_ip,
    ]
    if not args.dry_run:
        options += ["--hardware-validation", args.hardware_validation, "--live-ack", args.live_ack]
    return options


def preview(options: list[str]) -> int:
    # Import only after the startup banner, so missing PyYAML is diagnosable.
    import servo_launcher

    status("[CheckOnly] 仅离线校验与预览；不启动 WSL，不连接、同步、部署，不提权或修改网络。")
    return servo_launcher.main([*options, "--print-command"])


def run(kind: str, args: argparse.Namespace) -> int:
    options = real_arguments(args) if kind == "real" else sim_arguments(kind, args)
    entry = ROOT / "teleop.py"
    if not entry.is_file():
        raise ValueError(f"缺少统一入口：{entry}")
    if kind == "manual":
        status("[无手柄] 服务就绪后打开 http://127.0.0.1:8766；不自动打开浏览器。")
    if args.check_only:
        if kind == "pico":
            status("[PICO] 预览不枚举网卡；执行时校验/选择本机 LAN 地址并准备 TCP63901 转发。")
            if args.listen_address:
                status(f"[PICO] 待执行时确认的地址：{args.listen_address}:63901")
        return preview(options)
    if kind != "real":
        if os.name != "nt":
            raise ValueError("快捷仿真入口面向 Windows/WSL；Linux 请用 python teleop.py --mode sim --target local。")
        # Validate import/dependencies before any WSL/network side effects.
        import servo_launcher  # noqa: F401

        network = None
        if kind == "pico":
            status("[1/3] 正在检查 PICO 局域网地址……")
            network = select_pico_address(get_pico_candidates(), args.listen_address)
            status(f"[PICO] 手动连接 {network['address']}:63901（{network['interface']}）；头显需处于可达局域网。")
            status("[PICO] XRoboToolkit 选择 Controller / Send；先松开右 Grip，再握住。")
        status(f"[1/3] 正在检查 WSL {args.distro} / Docker（首次启动可能需要等待，最多 90 秒）……")
        run_command(
            [
                system_tool("wsl.exe"),
                "--distribution",
                args.distro,
                "--user",
                "root",
                "--exec",
                "docker",
                "info",
                "--format",
                "{{.ServerVersion}}",
            ],
            timeout=90,
        )
        if network:
            status("[2/3] 正在准备本项目 PICO 转发规则……")
            setup_pico_network(network["address"], args.distro)
        else:
            status("[2/3] 无手柄模式，不读取网卡、不配置网络。")
        status("[3/3] 启动 MuJoCo / Servo；首次构建较慢，下方显示实时进度。Ctrl+C 结束会话。")
    else:
        # Existing validation must pass offline before SSH/sync/deployment.
        result = preview(options)
        if result:
            return result
        status(
            "[真机] Dry Run 将连接真实控制器读取状态。"
            if args.dry_run
            else "[真机] 执行模式，保留现场准入与唯一发送链，不自动回位。"
        )
    command = [sys.executable, "-X", "utf8", "-u", str(entry), *options]
    if kind == "real":
        run_command(command)
    else:
        from scripts.setup.sim_session import run_simulation

        status("[复位] 仿真窗口内按 R 重建初始状态（不是 Backspace）；重连后须先松 Grip。")
        run_simulation(command, root=ROOT, execute=run_command, announce=status)
    return 0


def main(kind: str, argv: Sequence[str] | None = None) -> int:
    try:
        arguments = list(sys.argv[1:] if argv is None else argv)
        args = make_parser(kind).parse_args(arguments)
        status(f"[启动] RM65 {kind}；Python：{sys.executable}")
        if kind == "menu":
            from scripts.setup.menu import run_menu

            return run_menu(check_only=args.check_only)
        if kind == "real" and all(argument == "--pause" for argument in arguments):
            from scripts.setup.menu import interactive_available, run_menu

            if interactive_available():
                return run_menu(real_only=True)
        return run(kind, args)
    except KeyboardInterrupt:
        status("[中断] 控制会话被中断；请求结束不等于实体静止，须另行确认反馈与停止状态。")
        return 130
    except (OSError, ValueError, ImportError, subprocess.SubprocessError) as exc:
        LOGGER.error("Startup failed\n%s", traceback.format_exc())
        if isinstance(exc, subprocess.TimeoutExpired):
            status(f"[失败] 当前阶段等待超过 {exc.timeout} 秒，未继续启动；请检查上方 WSL/Docker/网络步骤。")
        elif isinstance(exc, subprocess.CalledProcessError):
            status(f"[失败] 外部命令退出码 {exc.returncode}；未继续后续步骤，请查看上方输出和诊断日志。")
            if exc.stderr:
                status(str(exc.stderr))
        else:
            status(f"[失败] {exc}")
        if isinstance(exc, ImportError):
            status(f'请在当前解释器安装依赖："{sys.executable}" -m pip install PyYAML')
        return 2


def owns_console() -> bool:
    """Explorer creates a new console with Python as its sole process."""
    if os.name != "nt" or not sys.stdin.isatty() or not sys.stdout.isatty():
        return False
    try:
        processes = (ctypes.c_uint32 * 8)()
        return ctypes.windll.kernel32.GetConsoleProcessList(processes, 8) == 1
    except (AttributeError, OSError):
        return False


def cli(kind: str) -> int:
    for stream in (sys.stdout, sys.stderr):
        if hasattr(stream, "reconfigure"):
            stream.reconfigure(encoding="utf-8", errors="replace")
    handler = None
    log_path = None
    try:
        folder = ROOT / "build/logs/startup"
        folder.mkdir(parents=True, exist_ok=True)
        log_path = folder / f"{datetime.now():%Y%m%d-%H%M%S-%f}-{kind}-{os.getpid()}.log"
        handler = logging.FileHandler(log_path, encoding="utf-8")
        handler.setFormatter(logging.Formatter("%(asctime)s %(levelname)s %(message)s"))
        LOGGER.addHandler(handler)
        LOGGER.setLevel(logging.INFO)
    except OSError as exc:
        print(f"[警告] 无法创建诊断日志：{exc}", flush=True)
    pause = "--pause" in sys.argv[1:] or owns_console()
    try:
        if log_path and handler:
            status(f"[日志] 启动阶段与异常（子进程实时输出仍在终端）：{log_path}")
        try:
            result = main(kind)
        except SystemExit as exc:
            result = int(exc.code or 0)
        if result == 0:
            status("[完成] 启动入口已退出；此信息不代表真实设备已停止或完成现场验收。")
        elif log_path and handler:
            status(f"[退出 {result}] 请查看上方错误和启动诊断日志：{log_path}")
        if pause and sys.stdin.isatty():
            try:
                input("按 Enter 关闭窗口……")
            except (EOFError, KeyboardInterrupt):
                pass
        return result
    finally:
        if handler:
            LOGGER.removeHandler(handler)
            handler.close()
