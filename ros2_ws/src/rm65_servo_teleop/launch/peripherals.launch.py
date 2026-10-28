"""Independent hand/camera sidecars. This launch NEVER starts the arm stack."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml

from rm65_servo_teleop.camera import CameraConfig
from rm65_servo_teleop.hand_driver import HandConnection
from rm65_servo_teleop.hand_session import HandProfile


def setup(context):
    value = lambda name: LaunchConfiguration(name).perform(context)
    hand, camera = value("hand"), value("camera")
    if hand not in {"off", "mock", "readonly", "live"} or camera not in {"off", "mock", "realsense"}:
        raise ValueError("invalid peripheral backend")
    if hand == camera == "off":
        raise ValueError("select at least one explicit peripheral backend")
    own = Path(get_package_share_directory("rm65_servo_teleop"))
    nodes = []
    for kind, mode in (("hand", hand), ("camera", camera)):
        if mode == "off":
            continue
        default = "hand_mock.yaml" if kind == "hand" and mode == "mock" else "camera.yaml"
        path = value(kind + "_profile") or str(own / "config" / default)
        if kind == "hand" and mode != "mock" and not value("hand_profile"):
            raise ValueError("hardware hand requires an explicit completed field profile")
        profile = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
        # Validate every configuration before launching ANY process.
        if kind == "camera":
            config = CameraConfig(**profile)
            if mode == "realsense" and not config.serial:
                raise ValueError("D435i serial must be filled explicitly")
        elif mode == "mock":
            HandProfile.from_dict(profile)
        else:
            connection = HandConnection(**profile["connection"])
            if mode == "live":
                connection.authorize(value("hand_ack"))
                task = HandProfile.from_dict(profile["task"])
                for target in (task.open, task.grasp):
                    if any(not lo <= q <= hi for q, lo, hi in zip(target, connection.lower, connection.upper)):
                        raise ValueError("field preset outside verified bounds")
        params = {
            "backend": mode,
            "profile": path,
            "record_root": value("record_root"),
            "run_label": value("run_label"),
        }
        if kind == "hand":
            params["operator_ack"] = value("hand_ack")
        nodes.append(
            Node(
                package="rm65_servo_teleop",
                executable="rm65_camera" if kind == "camera" else "rm65_hand_driver",
                parameters=[params],
                output="screen",
            )
        )
    return [
        *nodes,
        *[
            RegisterEventHandler(
                OnProcessExit(
                    target_action=node,
                    on_exit=[
                        EmitEvent(event=Shutdown(reason="peripheral exited; inspect capture and physical hand state"))
                    ],
                )
            )
            for node in nodes
        ],
    ]


def generate_launch_description():
    defaults = {
        "hand": "off",
        "camera": "off",
        "hand_profile": "",
        "camera_profile": "",
        "hand_ack": "",
        "record_root": "/data/experiments",
        "run_label": "",
    }
    return LaunchDescription(
        [*(DeclareLaunchArgument(k, default_value=v) for k, v in defaults.items()), OpaqueFunction(function=setup)]
    )
