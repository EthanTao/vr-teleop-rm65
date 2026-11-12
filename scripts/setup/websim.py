"""Local-only PICO browser simulation launcher for official IK or MoveIt Servo."""
from pathlib import Path
import os
import subprocess

from scripts.setup import launcher
import servo_launcher


def commands(controller, source_sha):
    if controller not in {"servo", "official"}:
        raise ValueError("invalid browser simulation controller")
    return [
        ["docker", "compose", "-f", "docker/compose.moveit.yml", "build", "--build-arg", "RM65_WITH_XR_SDK=ON",
         "--build-arg", "RM65_SOURCE_SHA=" + source_sha],
        ["docker", "compose", "-f", "docker/compose.websim.yml", "build", "--build-arg", "RM65_SOURCE_SHA=" + source_sha],
        ["docker", "compose", "-f", "docker/compose.websim.yml", "run", "--rm", "--service-ports",
         "--name", "rm65-pico-websim", "-e", "RM65_START_XR_SERVICE=1", "websim", "ros2", "launch",
         "rm65_servo_teleop", "pico_web.launch.py", "controller:=" + controller,
         "input_source:=pico", "address:=0.0.0.0"],
    ]


def run(args):
    if args.rviz or args.no_viewer:
        raise ValueError("浏览器仿真不使用 --rviz/--no-viewer；网页仅查看手柄控制效果")
    launcher.sim_arguments("pico", args)
    plan = commands(args.controller, servo_launcher.source_digest())
    local_windows = os.name == "nt"
    prefix = [launcher.system_tool("wsl.exe"), "--distribution", args.distro, "--user", "root"]
    if args.check_only:
        launcher.status("[CheckOnly] PICO 浏览器仿真；不启动环境、不读取网卡、不配置网络。")
        root = servo_launcher.windows_path_to_wsl(launcher.ROOT) if local_windows else str(launcher.ROOT)
        for command in plan:
            command = [*prefix, "--cd", root, "--exec", *command] if local_windows else command
            launcher.status(subprocess.list2cmdline(command))
        return 0
    if local_windows:
        network = launcher.select_pico_address(launcher.get_pico_candidates(), args.listen_address)
        launcher.status(f"[PICO] 头显连接 {network['address']}:63901，选择 Controller / Send。")
        launcher.run_command([*prefix, "--exec", "docker", "info", "--format", "{{.ServerVersion}}"], timeout=90)
        launcher.setup_pico_network(network["address"], args.distro)
        result = subprocess.run([*prefix, "--exec", "wslpath", "-a", "-u", str(launcher.ROOT)],
                                capture_output=True, text=True, encoding="utf-8", check=True)
        root = result.stdout.strip()
        if not root.startswith("/") or any(c in root for c in "\r\n\0"):
            raise ValueError("invalid WSL repository path")
        plan = [[*prefix, "--cd", root, "--exec", *command] for command in plan]
    elif args.listen_address:
        launcher.status("[PICO] Linux 本机服务；请确认头显可访问本机 TCP63901。")
    launcher.status("[仿真] 构建独立 MuJoCo 4 环境；每次只启动一版，不连接机械臂。")
    for command in plan:
        launcher.run_command(command)
    return 0
