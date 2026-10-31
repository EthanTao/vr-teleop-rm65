"""One Servo/JTC stack; only ros2_control's device interface changes by mode."""

from __future__ import annotations

from pathlib import Path
import hashlib
import os
import re

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml

from rm65_servo_teleop.model import JOINT_NAMES, build_description, build_semantic
from rm65_servo_teleop.hand_node import validate_mode as validate_hand_mode
from rm65_servo_teleop.validation import deployment_digest, model_digest, require_hardware_validation


def _boolean(context, name: str) -> bool:
    value = LaunchConfiguration(name).perform(context).lower()
    if value not in {"true", "false"}:
        raise ValueError(f"{name} must be true or false")
    return value == "true"


def _required_process_exit(event, context, *, component: str, next_actions=None):
    """Keep requested shutdown normal; make an unexpected control exit fail launch.

    Humble LaunchService catches callback exceptions, shuts down other processes
    and returns nonzero. A plain Shutdown event instead reports success.
    Only successful one-shot spawners may advance to their next startup stage.
    """
    if context.is_shutdown:
        return []
    if next_actions is not None and event.returncode == 0:
        return next_actions
    raise RuntimeError(f"{component} exited unexpectedly (return code {event.returncode})")


def _setup(context):
    own = Path(get_package_share_directory("rm65_servo_teleop"))
    vendor = Path(get_package_share_directory("rm_description"))
    mode = LaunchConfiguration("mode").perform(context)
    hand_mode = LaunchConfiguration("hand_mode").perform(context)
    validate_hand_mode(mode, hand_mode)
    source = LaunchConfiguration("input_source").perform(context)
    simulator = LaunchConfiguration("simulator").perform(context)
    if simulator not in {"mock", "mujoco"}:
        raise ValueError("simulator must be mock or mujoco")
    use_mujoco = mode == "sim" and simulator == "mujoco"
    if mode not in {"sim", "hardware-dry-run", "hardware"} or source not in {"mock", "manual", "pico", "external"}:
        raise ValueError("invalid execution mode or input source")
    if mode != "sim" and source in {"mock", "manual"}:
        raise ValueError("synthetic XR is forbidden for hardware and hardware-dry-run")
    if mode != "sim" and not LaunchConfiguration("udp_ip").perform(context):
        raise ValueError("hardware state input needs an explicit board-side udp_ip, not an inferred Windows address")
    config_path = LaunchConfiguration("config").perform(context) or str(own / "config/teleop.yaml")
    config = yaml.safe_load(Path(config_path).read_text(encoding="utf-8"))
    description = build_description(vendor / "urdf/rm_65.urdf", config, mode)
    # Semantic source ships with the pinned manufacturer MoveIt config package.
    semantic_source = Path(get_package_share_directory("rm_65_config")) / "config/rm_65_description.srdf"
    semantic = build_semantic(semantic_source, config)
    digest = model_digest(description)
    patch = own / "patches/realman_driver_feedback.patch"
    deployment = deployment_digest(
        description,
        semantic,
        config,
        hashlib.sha256(patch.read_bytes()).hexdigest(),
        LaunchConfiguration("arm_ip").perform(context),
    )
    if _boolean(context, "print_model_only"):
        return [LogInfo(msg=f"RM65 model_sha256={digest}; deployment_sha256={deployment}; no nodes or driver started")]

    simulation_cleanup = None
    simulation_metadata = ""
    physics = {}
    if use_mujoco:
        import tempfile
        from launch.event_handlers import OnShutdown
        from rm65_servo_teleop.mujoco_scene import write_scene

        physics = yaml.safe_load((own / "config/mujoco.yaml").read_text(encoding="utf-8"))
        temporary = tempfile.TemporaryDirectory(prefix="rm65-mujoco-")
        try:
            scene = write_scene(description, semantic, config, physics, Path(temporary.name))
            description = build_description(
                vendor / "urdf/rm_65.urdf",
                config,
                mode,
                simulator="mujoco",
                mujoco_model=str(scene),
                headless=not _boolean(context, "mujoco_viewer"),
            )
        except Exception:
            temporary.cleanup()
            raise
        simulation_metadata = str(scene.with_name("model.json"))

        # Retain ownership until launch shutdown, including startup failure.
        def cleanup_scene(event, shutdown_context):
            temporary.cleanup()
            return []

        simulation_cleanup = RegisterEventHandler(OnShutdown(on_shutdown=cleanup_scene))

    verified = {}
    if mode == "hardware":
        verified = require_hardware_validation(
            LaunchConfiguration("hardware_validation_file").perform(context),
            digest,
            operator_ack=LaunchConfiguration("operator_ack").perform(context),
            deployment_sha256=deployment,
        )
    common_model = {
        "robot_description": description,
        "robot_description_semantic": semantic,
        "robot_description_kinematics": {
            "rm65_arm": {
                "kinematics_solver": "kdl_kinematics_plugin/KDLKinematicsPlugin",
                "kinematics_solver_search_resolution": 0.005,
                "kinematics_solver_timeout": 0.005,
            }
        },
        "robot_description_planning": {
            "joint_limits": {
                name: {"has_velocity_limits": True, "max_velocity": limit}
                for name, limit in zip(JOINT_NAMES, config["model"]["joint_velocity_rad_s"])
            }
        },
        "use_sim_time": False,
    }
    feedback_topic = "/joint_states" if mode == "sim" else "/rm65/transport/joint_states"
    servo = dict(config["servo"])
    servo.update(config["pose_tracking"])
    tracking = Node(
        package="rm65_servo_teleop",
        executable="rm65_pose_tracking",
        name="rm65_pose_tracking",
        output="screen",
        parameters=[
            common_model,
            {"moveit_servo": servo},
            {
                "feedback_topic": feedback_topic,
                "input_timeout_s": 0.20,
                "feedback_timeout_s": 0.20,
                "max_linear_velocity_m_s": config["safety"]["max_linear_velocity_m_s"],
                "max_angular_velocity_rad_s": config["safety"]["max_angular_velocity_rad_s"],
            },
        ],
        remappings=[("/target_pose", "/rm65/internal/target_pose")],
    )
    input_node = Node(
        package="rm65_servo_teleop",
        executable="xr_input_node",
        output="screen",
        parameters=[config["input"], {"feedback_topic": feedback_topic, "allow_synthetic": mode == "sim"}],
    )
    robot_state = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": description, "publish_frequency": 50.0, "use_sim_time": False}],
    )
    manager = Node(
        package="mujoco_ros2_control" if use_mujoco else "controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[{"robot_description": description, "use_sim_time": False}, str(own / "config/controllers.yaml")],
    )
    state_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "60",
        ],
        output="screen",
    )
    arm_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "rm65_arm_controller",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "60",
        ],
        output="screen",
    )
    after_controller = [tracking, input_node]
    input_source_node = None
    if source == "mock":
        input_source_node = Node(
            package="rm65_servo_teleop",
            executable="mock_xr_input",
            output="screen",
            parameters=[{"simulation_only": True}],
        )
    elif source == "manual":
        input_source_node = Node(
            package="rm65_servo_teleop",
            executable="manual_simulation_input",
            output="screen",
            parameters=[
                {
                    "simulation_only": True,
                    "config_file": str(config_path),
                    "port": int(LaunchConfiguration("manual_port").perform(context)),
                    "position_scale": config["input"]["position_scale"],
                    "rotation_scale": config["input"]["rotation_scale"],
                }
            ],
        )
    elif source == "pico":
        input_source_node = Node(package="rm65_servo_teleop", executable="xr_sdk_node", output="screen")
    if input_source_node is not None:
        after_controller.append(input_source_node)
    hand = None
    if hand_mode == "mock":
        hand = Node(
            package="rm65_servo_teleop",
            executable="rm65_hand",
            output="screen",
            parameters=[
                {
                    "mode": mode,
                    "profile": str(own / "config/hand_mock.yaml"),
                    "input_timeout_s": config["input"]["xr_timeout_s"],
                    "grip_on": config["input"]["grip_on"],
                    "grip_off": config["input"]["grip_off"],
                }
            ],
        )
        after_controller.append(hand)
    journal = None
    if _boolean(context, "record"):
        journal = Node(
            package="rm65_servo_teleop",
            executable="rm65_journal",
            output="screen",
            parameters=[
                {
                    "record_root": LaunchConfiguration("record_root").perform(context),
                    "mode": mode,
                    "journal_capacity": int(config["recording"]["queue_size"]),
                    "simulation_metadata": simulation_metadata,
                    "hand_mode": hand_mode,
                }
            ],
        )
        after_controller.append(journal)
    actions = [
        LogInfo(
            msg=f"RM65 {mode}, source={source}, simulator={simulator if mode == 'sim' else 'none'}, "
            f"model_sha256={digest}; one Servo/JTC stack"
        )
    ]
    extra_critical = [node for node in (input_source_node, journal, hand) if node is not None]
    if use_mujoco:
        guard = Node(
            package="rm65_servo_teleop",
            executable="rm65_simulation_guard",
            output="screen",
            parameters=[
                {
                    "timeout_s": physics["progress_timeout_s"],
                    "startup_timeout_s": physics["startup_timeout_s"],
                    "use_sim_time": False,
                }
            ],
        )
        after_controller.insert(0, guard)
        extra_critical.append(guard)
        actions.append(simulation_cleanup)
    token = os.environ.get("RM65_SIM_RESET_TOKEN", "")
    if token:
        if not use_mujoco or not re.fullmatch(r"[0-9a-f]{32}", token):
            raise ValueError("Owned simulation reset is only valid for MuJoCo simulation")
        reset = Node(
            package="rm65_servo_teleop",
            executable="rm65_simulation_reset",
            output="screen",
            parameters=[{
                "simulation_only": True,
                "record_root": LaunchConfiguration("record_root").perform(context),
                "use_sim_time": False,
            }],
        )
        after_controller.append(reset)
        extra_critical.append(reset)
    device_nodes = []
    if mode != "sim":
        driver_topics = [
            "feedback_snapshot",
            "servo_send_result",
            "servo_slow_stop",
            "servo_hard_stop",
            "servo_enable_stream",
            "servo_joint_command",
            "movej_canfd_cmd",
            "movej_canfd_custom_cmd",
            "move_stop_cmd",
        ]
        driver = Node(
            package="rm_driver",
            executable="rm_driver",
            output="screen",
            parameters=[
                {
                    "arm_ip": LaunchConfiguration("arm_ip").perform(context),
                    "tcp_port": 8080,
                    "arm_type": "RM_65",
                    "arm_dof": 6,
                    "udp_ip": LaunchConfiguration("udp_ip").perform(context),
                    "udp_port": 8089,
                    "udp_cycle": 5,
                    "udp_joint_speed_state": True,
                    "arm_joints": list(JOINT_NAMES),
                    "allow_servo_motion": mode == "hardware" and bool(verified),
                }
            ],
            remappings=[
                *((f"rm_driver/{topic}", f"/rm65/driver/{topic}") for topic in driver_topics),
                ("joint_states", "/rm65/driver/joint_states"),
            ],
        )
        bridge = Node(
            package="rm65_servo_teleop",
            executable="rm65_driver_bridge",
            output="screen",
            parameters=[
                {
                    "mode": mode,
                    "driver_prefix": "/rm65/driver",
                    "control_period_s": 0.02,
                    "feedback_timeout_s": 0.20,
                    "joint_velocity_limits": config["model"]["joint_velocity_rad_s"],
                    "joint_lower_limits": config["model"]["joint_lower_rad"],
                    "joint_upper_limits": config["model"]["joint_upper_rad"],
                    **{
                        key: verified.get(key) is True
                        for key in (
                            "allow_hardware_motion",
                            "frames_verified",
                            "stop_verified",
                            "speed_units_verified",
                        )
                    },
                }
            ],
        )
        device_nodes.extend([driver, bridge])
        extra_critical.extend([driver, bridge])

    def controller_ready(event, context):
        return _required_process_exit(event, context, component="arm controller spawner", next_actions=after_controller)

    def critical_exited(event, context):
        return _required_process_exit(event, context, component="control component")

    actions.append(RegisterEventHandler(OnProcessExit(target_action=arm_spawner, on_exit=controller_ready)))
    for critical in (manager, tracking, input_node, *extra_critical):
        actions.append(
            RegisterEventHandler(
                OnProcessExit(
                    target_action=critical,
                    on_exit=critical_exited,
                )
            )
        )

    def state_ready(event, context):
        return _required_process_exit(event, context, component="state broadcaster spawner", next_actions=[arm_spawner])

    actions.append(RegisterEventHandler(OnProcessExit(target_action=state_spawner, on_exit=state_ready)))
    actions.extend([*device_nodes, robot_state, manager, state_spawner])
    if _boolean(context, "rviz"):
        actions.append(
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", str(own / "config/viewer.rviz")],
                parameters=[common_model],
                output="screen",
            )
        )
    if _boolean(context, "foxglove"):
        actions.append(
            Node(
                package="foxglove_bridge",
                executable="foxglove_bridge",
                output="screen",
                parameters=[{"port": 8765, "address": "0.0.0.0"}],
            )
        )
    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("mode", default_value="sim"),
            DeclareLaunchArgument("hand_mode", default_value="off"),
            DeclareLaunchArgument("input_source", default_value="mock"),
            DeclareLaunchArgument("manual_port", default_value="8766"),
            DeclareLaunchArgument("simulator", default_value="mujoco"),
            DeclareLaunchArgument("mujoco_viewer", default_value="false"),
            DeclareLaunchArgument("config", default_value=""),
            DeclareLaunchArgument("record", default_value="true"),
            DeclareLaunchArgument("record_root", default_value="/data/experiments"),
            DeclareLaunchArgument("rviz", default_value="false"),
            DeclareLaunchArgument("foxglove", default_value="false"),
            DeclareLaunchArgument("arm_ip", default_value="192.168.10.18"),
            DeclareLaunchArgument("udp_ip", default_value=""),
            DeclareLaunchArgument("hardware_validation_file", default_value=""),
            DeclareLaunchArgument("operator_ack", default_value=""),
            DeclareLaunchArgument("print_model_only", default_value="false"),
            OpaqueFunction(function=_setup),
        ]
    )
