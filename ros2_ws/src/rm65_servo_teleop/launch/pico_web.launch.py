"""PICO -> shared input adapter -> selected controller -> MuJoCo 4 browser physics.

This launch has no hardware mode and never constructs rm_driver or transport.
"""
from pathlib import Path
import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml

from rm65_servo_teleop.model import JOINT_NAMES, build_description, build_semantic


def _setup(context):
    value = lambda name: LaunchConfiguration(name).perform(context)
    controller, source = value("controller"), value("input_source")
    if controller not in {"official", "servo"} or source not in {"pico", "external"}:
        raise ValueError("invalid PICO browser configuration")
    test_only = value("test_only") == "true"
    if test_only and (source != "external" or os.environ.get("RM65_WEBSIM_TEST_ONLY") != "1"):
        raise ValueError("synthetic fixtures require the explicit isolated-test environment")
    own = Path(get_package_share_directory("rm65_servo_teleop"))
    vendor = Path(get_package_share_directory("rm_description"))
    semantic_path = Path(get_package_share_directory("rm_65_config")) / "config/rm_65_description.srdf"
    config_path = own / "config/teleop.yaml"
    shared = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    description = build_description(vendor / "urdf/rm_65.urdf", shared, "sim", simulator="websim")
    semantic = build_semantic(semantic_path, shared)
    common = {"robot_description": description, "robot_description_semantic": semantic,
              "use_sim_time": False,
              "robot_description_kinematics": {"rm65_arm": {
                  "kinematics_solver": "kdl_kinematics_plugin/KDLKinematicsPlugin",
                  "kinematics_solver_search_resolution": .005, "kinematics_solver_timeout": .005}},
              "robot_description_planning": {"joint_limits": {
                  name: {"has_velocity_limits": True, "max_velocity": speed}
                  for name, speed in zip(JOINT_NAMES, shared["model"]["joint_velocity_rad_s"])}}}
    temporary = tempfile.TemporaryDirectory(prefix="rm65-pico-web-")
    physics = Node(package="rm65_servo_teleop", executable="rm65_websim", output="screen", parameters=[{
        "simulation_only": True, "controller": controller, "model_config": str(config_path),
        "physics_config": str(own / "config/mujoco.yaml"), "vendor": str(vendor),
        "semantic": str(semantic_path), "directory": temporary.name, "page": str(own / "config/websim.html"),
        "address": value("address"), "port": int(value("port"))}])
    robot_state = Node(package="robot_state_publisher", executable="robot_state_publisher", output="screen",
                       parameters=[{"robot_description": description, "publish_frequency": 50.0, "use_sim_time": False}])
    input_node = Node(package="rm65_servo_teleop", executable="xr_input_node", output="screen",
                      parameters=[shared["input"], {"feedback_topic": "/joint_states", "allow_synthetic": test_only}])
    started = [input_node]
    if source == "pico":
        started.append(Node(package="rm65_servo_teleop", executable="xr_sdk_node", output="screen"))
    critical = [physics, robot_state, *started]

    def exited(event, ctx):
        if not ctx.is_shutdown:
            raise RuntimeError(f"PICO web simulation component exited: {event.returncode}")
        return []

    actions = [RegisterEventHandler(OnShutdown(on_shutdown=lambda event, ctx: temporary.cleanup()))]
    if controller == "official":
        follower = Node(package="rm65_servo_teleop", executable="rm65_official_sim_follow", output="screen",
                        parameters=[{"simulation_only": True, "model_config": str(config_path),
                                     "follow_config": str(own / "config/official_follow.yaml")}])
        critical.append(follower)
        actions.extend([physics, robot_state, *started, follower])
    else:
        servo = {**shared["servo"], **shared["pose_tracking"]}
        tracking = Node(package="rm65_servo_teleop", executable="rm65_pose_tracking", output="screen",
                        parameters=[common, {"moveit_servo": servo}, {
                            "feedback_topic": "/joint_states", "input_timeout_s": .20, "feedback_timeout_s": .20,
                            "max_linear_velocity_m_s": shared["safety"]["max_linear_velocity_m_s"],
                            "max_angular_velocity_rad_s": shared["safety"]["max_angular_velocity_rad_s"]}],
                        remappings=[("/target_pose", "/rm65/internal/target_pose")])
        manager = Node(package="controller_manager", executable="ros2_control_node", output="screen",
                       parameters=[{"robot_description": description, "use_sim_time": False},
                                   str(own / "config/controllers.yaml")])
        states = Node(package="controller_manager", executable="spawner", output="screen",
                      arguments=["joint_state_broadcaster", "--controller-manager-timeout", "60"])
        arm = Node(package="controller_manager", executable="spawner", output="screen",
                   arguments=["rm65_arm_controller", "--controller-manager-timeout", "60"])
        feedback_ready = Node(package="rm65_servo_teleop", executable="rm65_websim_ready", output="screen",
                              parameters=[{"model_config": str(config_path)}])

        def state_ready(event, ctx):
            return [feedback_ready] if not ctx.is_shutdown and event.returncode == 0 else exited(event, ctx)

        def physical_ready(event, ctx):
            return [arm] if not ctx.is_shutdown and event.returncode == 0 else exited(event, ctx)

        def arm_ready(event, ctx):
            return [tracking, *started] if not ctx.is_shutdown and event.returncode == 0 else exited(event, ctx)

        critical.extend([manager, tracking])
        actions.extend([RegisterEventHandler(OnProcessExit(target_action=states, on_exit=state_ready)),
                        RegisterEventHandler(OnProcessExit(target_action=feedback_ready, on_exit=physical_ready)),
                        RegisterEventHandler(OnProcessExit(target_action=arm, on_exit=arm_ready)),
                        physics, robot_state, manager, states])
    for process in critical:
        actions.insert(0, RegisterEventHandler(OnProcessExit(target_action=process, on_exit=exited)))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("controller", default_value="official"),
        DeclareLaunchArgument("input_source", default_value="pico"),
        DeclareLaunchArgument("test_only", default_value="false"),
        DeclareLaunchArgument("address", default_value="127.0.0.1"),
        DeclareLaunchArgument("port", default_value="8094"), OpaqueFunction(function=_setup)])
