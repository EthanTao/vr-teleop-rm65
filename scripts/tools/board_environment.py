#!/usr/bin/env python3
"""Manage the board's existing Servo environment without granting motion access.

Runs on the Linux board (Python 3.8+). The only launch action is an explicit
hardware-dry-run with PICO input; it never stops or replaces a container.
"""

from __future__ import annotations

import argparse
import ipaddress
import json
from pathlib import Path
import platform
import shutil
import subprocess
import sys

WORKSPACE = Path(__file__).resolve().parents[2]
IMAGE = "rm65-servo:humble"
CONTAINER = "rm65-servo"
ARM_IP = "192.168.10.18"
UDP_IP = "192.168.10.100"
INSPECT = (
    '{"status":{{json .State.Status}},"image":{{json .Config.Image}},'
    '"image_id":{{json .Image}},"command":{{json .Config.Cmd}},"mounts":{{json .Mounts}}}'
)
SETUP_PYTHON = "source /opt/rm65_ws/install/setup.bash && python3 -B -"

CHECK_PROBE = r"""
import ctypes, importlib.metadata, json, os, pathlib, platform, subprocess, sys
import rclpy, mujoco
from ament_index_python.packages import get_package_prefix
from sensor_msgs.msg import JointState
from std_msgs.msg import String

packages = ('rm65_servo_teleop', 'rm_driver', 'moveit_servo', 'controller_manager',
            'joint_trajectory_controller', 'topic_based_ros2_control', 'foxglove_bridge')
prefixes = {name: get_package_prefix(name) for name in packages}
driver = pathlib.Path(prefixes['rm_driver']) / 'lib/rm_driver/rm_driver'
assert driver.is_file(), str(driver)
linked = subprocess.run(['ldd', str(driver)], text=True, capture_output=True, check=True)
assert 'not found' not in linked.stdout, linked.stdout
sdk_node = pathlib.Path(prefixes['rm65_servo_teleop']) / 'lib/rm65_servo_teleop/xr_sdk_node'
sdk_service = pathlib.Path('/opt/apps/roboticsservice/RoboticsServiceProcess')
assert sdk_node.is_file() and sdk_service.is_file(), 'PICO native SDK/service is missing'
for executable in (sdk_node, sdk_service):
    # Match the entrypoint's service-local paths without exporting vendor Qt6
    # libraries into the ROS/SDK node's environment.
    environment = os.environ.copy()
    if executable == sdk_service:
        service_root = sdk_service.parent
        sdk_arch = 'arm64' if platform.machine() in ('aarch64', 'arm64') else 'x64'
        environment['LD_LIBRARY_PATH'] = os.pathsep.join([environment.get('LD_LIBRARY_PATH', ''),
            str(service_root), str(service_root / 'lib'), str(service_root / 'SDK' / sdk_arch)])
    libraries_check = subprocess.run(['ldd', str(executable)], env=environment, text=True, capture_output=True, check=True)
    assert 'not found' not in libraries_check.stdout, libraries_check.stdout
sdk_manifest = json.loads(pathlib.Path('/opt/rm65-xr-sdk.json').read_text())
libraries = list(pathlib.Path('/opt/ros/humble').rglob('libmujoco.so*'))
assert libraries, 'C++ MuJoCo library missing'
library = ctypes.CDLL(str(libraries[0]))
library.mj_versionString.restype = ctypes.c_char_p
cpp = library.mj_versionString().decode()
assert cpp == mujoco.__version__ == '3.12.0', (cpp, mujoco.__version__)
print(json.dumps({'passed': True, 'architecture': platform.machine(),
    'python': sys.version.split()[0], 'packages': prefixes,
    'numpy': importlib.metadata.version('numpy'), 'mujoco_python': mujoco.__version__,
    'mujoco_cpp': cpp, 'driver_libraries_resolved': True,
    'xr_sdk': sdk_manifest, 'xr_native_libraries_resolved': True,
    'isolation': 'no network, no devices, no ROS launch, no robot connection'}, indent=2))
"""

STATUS_PROBE = r"""
import json, math, time
import rclpy
from std_msgs.msg import String
from rclpy.qos import qos_profile_sensor_data
rclpy.init()
node = rclpy.create_node('rm65_environment_observer')
observations, latest = [], {}
def receive(message):
    data = json.loads(message.data)
    if data.get('event') == 'feedback':
        observations.append((data['session'], data['packet_id']))
        latest.update(data)
subscription = node.create_subscription(String, '/rm65/transport/status', receive,
                                        qos_profile_sensor_data)
deadline = time.monotonic() + 4
while time.monotonic() < deadline:
    rclpy.spin_once(node, timeout_sec=0.1)
age = time.monotonic() - latest['source_s'] if latest else None
advanced = (len(observations) > 1 and observations[0][0] == observations[-1][0]
            and observations[-1][1] > observations[0][1])
fresh = advanced and age is not None and 0 <= age < 0.2
print(json.dumps({'fresh_feedback': fresh, 'messages_received': len(observations),
    'packet_id_advanced': advanced, 'source_age_ms': round(age * 1000, 1) if age is not None else None,
    'joint_degrees': [round(math.degrees(x), 3) for x in latest.get('position_rad', [])],
    'latest': latest}, indent=2))
node.destroy_node()
rclpy.shutdown()
raise SystemExit(0 if fresh else 2)
"""

INPUT_PROBE = r"""
import json, time
import rclpy
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from rm65_servo_teleop.input_adapter import decode_raw_packet, InputRejected
rclpy.init()
node = rclpy.create_node('rm65_readonly_headset_observer')
counts = {'valid_frames': 0, 'invalid_frames': 0, 'synthetic_frames': 0, 'advancing_timestamps': 0}
latest, previous_stamp, input_status = None, None, None
def receive(message):
    global latest, previous_stamp
    try:
        if 'sdk_event' in json.loads(message.data): return
        sample = decode_raw_packet(message.data)
    except (ValueError, TypeError, InputRejected):
        counts['invalid_frames'] += 1
        return
    counts['valid_frames'] += 1
    counts['synthetic_frames'] += int(sample.synthetic)
    counts['advancing_timestamps'] += int(previous_stamp is not None and sample.timestamp_ns > previous_stamp)
    previous_stamp = sample.timestamp_ns
    latest = {'device_timestamp_ns': sample.timestamp_ns, 'grip': sample.grip, 'synthetic': sample.synthetic}
def status(message):
    global input_status
    input_status = json.loads(message.data)
subscriptions = [node.create_subscription(String, '/rm65/xr/raw', receive, qos_profile_sensor_data),
                 node.create_subscription(String, '/rm65/input_status', status, qos_profile_sensor_data)]
deadline = time.monotonic() + 5
while time.monotonic() < deadline: rclpy.spin_once(node, timeout_sec=.1)
print(json.dumps(dict(counts, observation_seconds=5, latest=latest, input_status=input_status,
                     observer_published_robot_commands=False), indent=2))
node.destroy_node()
rclpy.shutdown()
"""


def execute(command, **kwargs):
    return subprocess.run(command, text=True, check=True, **kwargs)


def docker_prefix():
    if not shutil.which("docker"):
        raise RuntimeError("Docker is missing; the existing environment has not been changed")
    for prefix in (["docker"], ["sudo", "-n", "docker"]):
        result = subprocess.run(prefix + ["version", "--format", "{{.Server.Version}}"], capture_output=True, text=True)
        if result.returncode == 0:
            return prefix
    raise RuntimeError("Docker is unavailable; check the daemon or sudo -n access")


def inspect_container(docker):
    result = subprocess.run(docker + ["inspect", "--format", INSPECT, CONTAINER], capture_output=True, text=True)
    if result.returncode:
        if "No such" in result.stderr:
            return None
        raise RuntimeError(result.stderr.strip())
    return json.loads(result.stdout)


def is_our_readonly_session(metadata, workspace, *, arm_ip=ARM_IP, udp_ip=UDP_IP):
    expected = {
        "mode:=hardware-dry-run",
        "input_source:=pico",
        "foxglove:=true",
        "rviz:=false",
        "arm_ip:=" + arm_ip,
        "udp_ip:=" + udp_ip,
    }
    command = metadata.get("command") or []
    # Require the whole command: extra launch overrides must not enable motion.
    expected_prefix = ["ros2", "launch", "rm65_servo_teleop", "teleop.launch.py"]
    mounted = any(
        item.get("Source") == str(workspace / "build/experiments") and item.get("Destination") == "/data/experiments"
        for item in metadata.get("mounts", [])
    )
    return (
        metadata.get("image") == IMAGE
        and command[:4] == expected_prefix
        and len(command) == 4 + len(expected)
        and set(command[4:]) == expected
        and mounted
    )


def start_readonly(docker, workspace, *, arm_ip=ARM_IP, udp_ip=UDP_IP):
    current = inspect_container(docker)
    if current is not None:
        if current["status"] == "running" and is_our_readonly_session(current, workspace, arm_ip=arm_ip, udp_ip=udp_ip):
            print("Existing read-only session is running; reused without restarting it.")
            return
        raise RuntimeError("The container name is already in use; refusing to stop, replace or restart it")
    route = json.loads(execute(["ip", "-j", "route", "get", arm_ip], capture_output=True).stdout)
    if len(route) != 1 or route[0].get("prefsrc") != udp_ip:
        raise RuntimeError("Board-to-controller route has changed; verify the configured UDP address first")
    ports = execute(["ss", "-H", "-lntu"], capture_output=True).stdout.splitlines()
    if any(field.rsplit(":", 1)[-1] in {"8089", "8765", "63901"} for line in ports for field in line.split()[4:5]):
        raise RuntimeError("UDP 8089, viewer 8765 or PICO 63901 is occupied; refusing a competing session")
    records = workspace / "build/experiments"
    records.mkdir(parents=True, exist_ok=True)
    execute(
        docker
        + [
            "run",
            "--detach",
            "--rm",
            "--name",
            CONTAINER,
            "--label",
            "org.rm65.workspace=" + str(workspace),
            "--publish",
            "63901:63901/tcp",
            "--publish",
            "8089:8089/udp",
            "--publish",
            "127.0.0.1:8765:8765/tcp",
            "--volume",
            str(records) + ":/data/experiments:rw",
            "--env",
            "RM65_START_XR_SERVICE=1",
            IMAGE,
            "ros2",
            "launch",
            "rm65_servo_teleop",
            "teleop.launch.py",
            "mode:=hardware-dry-run",
            "input_source:=pico",
            "foxglove:=true",
            "rviz:=false",
            "arm_ip:=" + arm_ip,
            "udp_ip:=" + udp_ip,
        ]
    )
    print(
        "Started PICO + hardware-dry-run. Native XR input and real UDP feedback are active; motion access remains disabled."
    )


def canonical_address(value):
    address = ipaddress.IPv4Address(value)
    if str(address) != value or address.is_unspecified or address.is_loopback or int(address) >= 0xE0000000:
        raise argparse.ArgumentTypeError("Expected a canonical unicast LAN IPv4 address")
    return value


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "action", choices=["check", "status", "start-readonly", "input", "state", "logs"], nargs="?", default="status"
    )
    parser.add_argument("--arm-ip", type=canonical_address)
    parser.add_argument("--udp-ip", type=canonical_address)
    args = parser.parse_args(argv)
    if platform.system() != "Linux":
        parser.error("Run this tool on the Linux board, inside /home/rm/EthanTao")
    docker = docker_prefix()
    if args.action == "check":
        return execute(
            docker
            + [
                "run",
                "--rm",
                "--interactive",
                "--network",
                "none",
                "--cpus",
                "1",
                "--memory",
                "1g",
                "--cap-drop",
                "ALL",
                "--entrypoint",
                "/bin/bash",
                IMAGE,
                "-c",
                SETUP_PYTHON,
            ],
            input=CHECK_PROBE,
        ).returncode
    if args.action == "start-readonly":
        start_readonly(docker, WORKSPACE, arm_ip=args.arm_ip or ARM_IP, udp_ip=args.udp_ip or UDP_IP)
        return 0
    current = inspect_container(docker)
    print(json.dumps({"workspace": str(WORKSPACE), "container": current}, indent=2), flush=True)
    if current is None or current["status"] != "running":
        return 2
    if (args.arm_ip or args.udp_ip) and not is_our_readonly_session(
        current,
        WORKSPACE,
        arm_ip=args.arm_ip or ARM_IP,
        udp_ip=args.udp_ip or UDP_IP,
    ):
        raise RuntimeError("Existing session does not match the requested controller/UDP addresses; nothing changed")
    if args.action == "status":
        return execute(docker + ["exec", "-i", CONTAINER, "bash", "-c", SETUP_PYTHON], input=STATUS_PROBE).returncode
    if args.action == "input":
        return execute(docker + ["exec", "-i", CONTAINER, "bash", "-c", SETUP_PYTHON], input=INPUT_PROBE).returncode
    if args.action == "logs":
        return execute(docker + ["logs", "--tail", "40", "--follow", CONTAINER]).returncode
    return execute(
        docker
        + [
            "exec",
            "-i",
            CONTAINER,
            "bash",
            "-c",
            "source /opt/rm65_ws/install/setup.bash && ros2 topic echo /rm65/transport/joint_states",
        ]
    ).returncode


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        raise SystemExit(130)
    except (RuntimeError, OSError, ValueError, subprocess.CalledProcessError) as error:
        print("ERROR: " + str(error), file=sys.stderr)
        raise SystemExit(2)
