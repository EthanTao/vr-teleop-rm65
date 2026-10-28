#!/usr/bin/env python3
"""Inspect host/container prerequisites without starting ROS or contacting the arm.

The remote copy runs from SSH stdin. It does not install packages, write remote
files, launch containers/nodes, import hardware SDKs, or query the robot.
"""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import re
import shlex
import shutil
import subprocess
import sys

SCOPE_PROBE = r"""
import importlib.metadata
import importlib.util
import json
import os
import platform
from pathlib import Path
import shutil
import subprocess
import sys

def run(command):
    try:
        result = subprocess.run(command, capture_output=True, text=True, encoding="utf-8", errors="replace", timeout=10)
        return {"returncode": result.returncode, "stdout": result.stdout[-12000:], "stderr": result.stderr[-2000:]}
    except (OSError, subprocess.TimeoutExpired) as exc:
        return {"returncode": None, "error": str(exc)}

def python_info():
    modules = {}
    for name in ("rclpy", "xrobotoolkit_sdk", "Robotic_Arm", "pyrealsense2", "numpy"):
        try:
            spec = importlib.util.find_spec(name)
            modules[name] = None if spec is None else spec.origin
        except (ImportError, ValueError) as exc:
            modules[name] = {"error": str(exc)}
    versions = {}
    for name in ("numpy", "placo", "Robotic_Arm", "pyrealsense2", "lerobot"):
        try:
            versions[name] = importlib.metadata.version(name)
        except importlib.metadata.PackageNotFoundError:
            versions[name] = None
    return {"executable": sys.executable, "version": sys.version, "modules": modules, "packages": versions}

ros_root = Path("/opt/ros")
sdk_root = Path("/opt/apps/roboticsservice/SDK")
python_probes = {sys.executable: python_info()}
python_query = "import json,sys; print(json.dumps({'executable':sys.executable,'version':sys.version}))"
for candidate in ("/usr/bin/python3", "/opt/conda/bin/python", "/opt/conda/envs/xrobo/bin/python"):
    if Path(candidate).is_file() and candidate != sys.executable:
        python_probes[candidate] = run([candidate, "-B", "-c", python_query])
report = {
    "platform": platform.platform(), "machine": platform.machine(), "cpu_count": os.cpu_count(),
    "os_release": Path("/etc/os-release").read_text() if Path("/etc/os-release").is_file() else None,
    "ros_prefixes": sorted(path.name for path in ros_root.iterdir()) if ros_root.is_dir() else [],
    "tools": {name: shutil.which(name) for name in ("ros2", "colcon", "docker", "git", "cmake")},
    "python": python_probes,
    "xr_sdk_directories": sorted(path.name for path in sdk_root.iterdir()) if sdk_root.is_dir() else [],
    "disk_root": dict(zip(("total", "used", "free"), shutil.disk_usage(Path.cwd().anchor))),
}
if Path("/proc/meminfo").is_file():
    report["memory"] = [line for line in Path("/proc/meminfo").read_text().splitlines()
                        if line.startswith(("MemTotal:", "MemAvailable:"))]
if shutil.which("dpkg-query"):
    packages = run(["dpkg-query", "-W", "-f=${binary:Package}\t${Version}\n", "ros-humble-*"])
    packages["stdout"] = "\n".join(line for line in packages.get("stdout", "").splitlines()
                                    if any(key in line for key in ("moveit", "ros-base", "rclcpp", "rclpy", "ros2-control", "controller")))
    report["ros_packages"] = packages
if shutil.which("ss"):
    ports = run(["ss", "-lntu"])
    ports["stdout"] = "\n".join(line for line in ports.get("stdout", "").splitlines()
                                if any(":" + port in line for port in ("63901", "8089", "7000")))
    report["relevant_listening_ports"] = ports
print(json.dumps(report, ensure_ascii=False))
"""


def run(command: list[str], *, input_text: str | None = None, timeout: float = 15) -> dict:
    try:
        result = subprocess.run(
            command,
            input=input_text,
            capture_output=True,
            text=True,
            encoding="utf-8",
            errors="replace",
            timeout=timeout,
        )
        return {"returncode": result.returncode, "stdout": result.stdout, "stderr": result.stderr[-4000:]}
    except (OSError, subprocess.TimeoutExpired) as exc:
        return {"returncode": None, "error": str(exc)}


def unpack(result: dict) -> dict:
    if result.get("returncode") == 0:
        try:
            return json.loads(result["stdout"])
        except json.JSONDecodeError:
            pass
    return result


def inventory(container: str, allow_sudo: bool) -> dict:
    report = {"host": unpack(run([sys.executable, "-B", "-c", SCOPE_PROBE]))}
    if shutil.which("docker") is None:
        report["docker"] = {"available": False}
        return report
    prefix = ["docker"]
    access = run(prefix + ["version", "--format", "{{.Server.Version}}"])
    if access.get("returncode") != 0 and allow_sudo and shutil.which("sudo"):
        prefix = ["sudo", "-n", "docker"]
        access = run(prefix + ["version", "--format", "{{.Server.Version}}"])
    report["docker"] = {"server": access, "used_sudo": len(prefix) > 1}
    if access.get("returncode") != 0:
        return report
    # Select individual fields: never dump Config.Env, credentials or full inspect output.
    template = (
        '{"name":{{json .Name}},"image":{{json .Config.Image}},"status":{{json .State.Status}},'
        '"network":{{json .HostConfig.NetworkMode}},"mounts":{{json .Mounts}}}'
    )
    metadata = unpack(run(prefix + ["inspect", "--format", template, container]))
    report["container"] = metadata
    if metadata.get("status") == "running":
        report["container_inventory"] = unpack(
            run(prefix + ["exec", container, "/usr/bin/python3", "-B", "-c", SCOPE_PROBE], timeout=40)
        )
    return report


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ssh-host", default="rm", help="SSH alias of the Linux board")
    parser.add_argument("--container", default="xrobo-vr-teleop")
    parser.add_argument(
        "--sudo-docker", action="store_true", help="Try read-only sudo -n docker after plain Docker fails"
    )
    parser.add_argument("--local", action="store_true", help="Inspect this machine instead of using SSH")
    parser.add_argument(
        "--output", type=Path, help="Create a new local JSON report; existing files are never overwritten"
    )
    parser.add_argument("--target-inventory", action="store_true", help=argparse.SUPPRESS)
    args = parser.parse_args()
    if not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_.-]*", args.container):
        parser.error("invalid container name")
    if args.target_inventory:
        print(json.dumps(inventory(args.container, args.sudo_docker), ensure_ascii=False))
        return 0
    report = {
        "checked_at_utc": datetime.now(timezone.utc).isoformat(),
        "read_only": True,
        "robot_connection_attempted": False,
    }
    exit_code = 0
    if args.local:
        report["scope"] = "local"
        report["inventory"] = inventory(args.container, args.sudo_docker)
    else:
        if not re.fullmatch(r"[A-Za-z0-9_][A-Za-z0-9_.@:-]*", args.ssh_host):
            parser.error("invalid SSH host; use an SSH alias")
        remote = ["python3", "-B", "-", "--target-inventory", "--container", args.container]
        if args.sudo_docker:
            remote.append("--sudo-docker")
        command = [
            "ssh",
            "-o",
            "BatchMode=yes",
            "-o",
            "StrictHostKeyChecking=yes",
            "-o",
            "ConnectTimeout=8",
            args.ssh_host,
            shlex.join(remote),
        ]
        result = run(command, input_text=Path(__file__).read_text(encoding="utf-8"), timeout=75)
        report["scope"] = args.ssh_host
        report["inventory"] = unpack(result)
        if result.get("returncode") != 0 or "host" not in report["inventory"]:
            exit_code = 2
    rendered = json.dumps(report, ensure_ascii=False, indent=2)
    if args.output is not None:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        with args.output.open("x", encoding="utf-8") as stream:
            stream.write(rendered + "\n")
    print(rendered)
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
