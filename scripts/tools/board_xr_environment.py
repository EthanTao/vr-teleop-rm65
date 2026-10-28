#!/usr/bin/env python3
"""SDK-only board service beside the preserved read-only robot session.

The sidecar has a separate ROS domain and starts no input adapter, controller or
robot driver. The existing Docker port mapping carries the headset TCP stream.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import subprocess
import sys

import board_environment as board

IMAGE = "rm65-servo:xr-ready"
CONTAINER = "rm65-xr-input"
ROLE = "xr-input-only"
COMMAND = ["/opt/rm65_ws/install/lib/rm65_servo_teleop/xr_sdk_node"]
SAMPLE = r'''
import json,time
import rclpy
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from rm65_servo_teleop.input_adapter import decode_raw_packet, InputRejected
rclpy.init()
node=rclpy.create_node('xr_readonly_input_observer')
count=invalid=synthetic=0
latest=None
def receive(message):
    global count,invalid,synthetic,latest
    if 'sdk_event' in json.loads(message.data): return
    try: sample=decode_raw_packet(message.data)
    except InputRejected:
        invalid+=1
        return
    count+=1
    synthetic+=int(sample.synthetic)
    latest={'device_timestamp_ns':sample.timestamp_ns,'grip':sample.grip,
            'synthetic':sample.synthetic}
subscription=node.create_subscription(String,'/rm65/xr/raw',receive,qos_profile_sensor_data)
deadline=time.monotonic()+5
while time.monotonic()<deadline: rclpy.spin_once(node,timeout_sec=.1)
print(json.dumps({'observation_seconds':5,'valid_frames':count,'invalid_frames':invalid,
    'synthetic_frames':synthetic,'latest':latest,'robot_commands_published':False},indent=2))
node.destroy_node()
rclpy.shutdown()
'''


def inspect(docker, name):
    result = subprocess.run(docker + ["inspect", name], capture_output=True, text=True)
    if result.returncode:
        if "No such" in result.stderr:
            return None
        raise RuntimeError(result.stderr.strip())
    return json.loads(result.stdout)[0]


def ours(metadata, workspace):
    config = metadata.get("Config", {})
    labels = config.get("Labels") or {}
    environment = config.get("Env") or []
    return (
        labels.get("org.rm65.role") == ROLE
        and labels.get("org.rm65.workspace") == str(workspace)
        and labels.get("org.rm65.xr-sdk") == "ON"
        and config.get("Cmd") == COMMAND
        and "ROS_DOMAIN_ID=86" in environment
        and "RM65_START_XR_SERVICE=1" in environment
        and metadata.get("HostConfig", {}).get("NetworkMode", "").startswith("container:")
    )


def start(docker, workspace):
    real = board.inspect_container(docker)
    if real and "input_source:=pico" in (real.get("command") or []):
        raise RuntimeError("The main PICO session owns its SDK service; refusing a duplicate input service")
    if not real or real["status"] != "running" or not board.is_our_readonly_session(real, workspace):
        raise RuntimeError("A matching external-input read-only session is required; no existing session was changed")
    parent = inspect(docker, board.CONTAINER)
    current = inspect(docker, CONTAINER)
    if current:
        same_network = current.get("HostConfig", {}).get("NetworkMode") == "container:" + parent["Id"]
        if ours(current, workspace) and same_network and current["State"]["Running"]:
            print("SDK-only input service already running; robot session preserved.")
            return
        raise RuntimeError("SDK container name is occupied; refusing to replace or restart it")
    image = inspect(docker, IMAGE)
    if not image or (image.get("Config", {}).get("Labels") or {}).get("org.rm65.xr-sdk") != "ON":
        raise RuntimeError("The prepared SDK=ON image is missing")
    board.execute(docker + [
        "run", "--detach", "--name", CONTAINER, "--init",
        "--network", "container:" + parent["Id"],
        "--cpus", "1", "--memory", "768m", "--pids-limit", "192",
        "--cap-drop", "ALL", "--security-opt", "no-new-privileges",
        "--label", "org.rm65.role=" + ROLE,
        "--label", "org.rm65.workspace=" + str(workspace),
        "--env", "ROS_DOMAIN_ID=86", "--env", "RM65_START_XR_SERVICE=1",
        "--env", "OPENBLAS_NUM_THREADS=1", "--env", "OMP_NUM_THREADS=1",
        image["Id"], *COMMAND,
    ])


def status(docker, workspace):
    current = inspect(docker, CONTAINER)
    if not current:
        print(json.dumps({"running": False, "headset_validated": False}))
        return
    if not ours(current, workspace):
        raise RuntimeError("Unrecognized SDK container; left unchanged")
    result = dict(
        running=current["State"]["Running"], container_id=current["Id"],
        image_id=current["Image"], role=ROLE, ros_domain=86,
        robot_driver_or_controller_started=False, headset_validated=False,
        source_sha=current["Config"]["Labels"].get("org.rm65.source-sha"),
    )
    if result["running"]:
        sockets = board.execute(docker + ["exec", CONTAINER, "ss", "-H", "-lnt"], capture_output=True).stdout
        ports = {line.split()[3].rsplit(":", 1)[-1] for line in sockets.splitlines() if len(line.split()) >= 4}
        processes = board.execute(docker + ["top", CONTAINER, "-eo", "pid,comm"], capture_output=True).stdout
        result.update(
            headset_tcp_ready="63901" in ports, sdk_rpc_ready="60061" in ports,
            sdk_process_present="xr_sdk_node" in processes,
            sdk=json.loads(board.execute(docker + ["exec", CONTAINER, "cat", "/opt/rm65-xr-sdk.json"], capture_output=True).stdout),
        )
    print(json.dumps(result, indent=2))


def stop(docker, workspace):
    current = inspect(docker, CONTAINER)
    if current is None:
        return
    if not ours(current, workspace):
        raise RuntimeError("Unrecognized SDK container; refusing to stop it")
    if current["State"]["Running"]:
        board.execute(docker + ["stop", "--time", "15", CONTAINER])
    # Remove only this recognized stopped SDK sidecar; no images or data mounts.
    board.execute(docker + ["rm", CONTAINER])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("action", choices=("start", "status", "sample", "stop", "logs"), nargs="?", default="status")
    args = parser.parse_args()
    docker = board.docker_prefix()
    workspace = Path(__file__).resolve().parents[2]
    if args.action in {"logs", "sample"}:
        current = inspect(docker, CONTAINER)
        if not current or not ours(current, workspace):
            raise RuntimeError("Recognized SDK-only service is not present")
        if args.action == "logs":
            board.execute(docker + ["logs", "--tail", "60", CONTAINER])
        else:
            board.execute(docker + ["exec", "-i", CONTAINER, "bash", "-c", board.SETUP_PYTHON], input=SAMPLE)
    else:
        {"start": start, "status": status, "stop": stop}[args.action](docker, workspace)


if __name__ == "__main__":
    try:
        main()
    except (RuntimeError, subprocess.CalledProcessError) as error:
        print(str(error), file=sys.stderr)
        raise SystemExit(2)
