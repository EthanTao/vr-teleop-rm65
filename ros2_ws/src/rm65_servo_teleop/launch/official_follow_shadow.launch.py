"""Observe a separate existing read-only control graph; never start a driver."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    own = Path(get_package_share_directory("rm65_servo_teleop"))
    return LaunchDescription([
        DeclareLaunchArgument("sdk_root", default_value=""),
        DeclareLaunchArgument("model_config", default_value=str(own / "config/teleop.yaml")),
        DeclareLaunchArgument("follow_config", default_value=str(own / "config/official_follow.yaml")),
        Node(package="rm65_servo_teleop", executable="rm65_official_follow_shadow", output="screen",
             parameters=[{"sdk_root": LaunchConfiguration("sdk_root"),
                          "model_config": LaunchConfiguration("model_config"),
                          "follow_config": LaunchConfiguration("follow_config"), "compute_only": True,
                          "use_sim_time": False}]),
    ])
