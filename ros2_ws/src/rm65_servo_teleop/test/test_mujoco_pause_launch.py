"""A paused physics engine must close the live teleoperation graph, not resume it."""

import os
import secrets
import time
import unittest

import pytest

rclpy = pytest.importorskip("rclpy", reason="physics stall acceptance requires Linux ROS")
launch = pytest.importorskip("launch")
launch_testing = pytest.importorskip("launch_testing")
import launch_testing.actions
import launch_testing.asserts
from launch_testing.util import KeepAliveProc
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from mujoco_ros2_control_msgs.srv import SetPause
from std_msgs.msg import Bool

DOMAIN = str(101 + secrets.randbelow(60))
PREVIOUS_DOMAIN = os.environ.get("ROS_DOMAIN_ID")


@pytest.mark.launch_test
def generate_test_description():
    os.environ["ROS_DOMAIN_ID"] = DOMAIN
    stack = ExecuteProcess(
        cmd=[
            "ros2",
            "launch",
            "rm65_servo_teleop",
            "teleop.launch.py",
            "mode:=sim",
            "simulator:=mujoco",
            "input_source:=mock",
            "record:=false",
            "rviz:=false",
        ],
        output="screen",
    )
    return launch.LaunchDescription(
        [
            SetEnvironmentVariable("ROS_DOMAIN_ID", DOMAIN),
            stack,
            KeepAliveProc(),
            launch_testing.actions.ReadyToTest(),
        ]
    ), {"stack": stack}


class TestPhysicsStall(unittest.TestCase):
    def test_pause_during_tracking_exits_the_whole_graph(self, proc_info, stack):
        rclpy.init()
        node = rclpy.create_node("mujoco_pause_acceptance")
        enabled = []
        node.create_subscription(Bool, "/rm65/enable", lambda msg: enabled.append(msg.data), 10)
        client = node.create_client(SetPause, "/mujoco_ros2_control_node/set_pause")
        try:
            deadline = time.monotonic() + 60
            while time.monotonic() < deadline and not (any(enabled) and client.service_is_ready()):
                rclpy.spin_once(node, timeout_sec=0.05)
            self.assertTrue(any(enabled), "the simulation never accepted Grip")
            self.assertTrue(client.service_is_ready(), "MuJoCo pause service is missing")
            request = SetPause.Request()
            request.paused = True
            future = client.call_async(request)
            rclpy.spin_until_future_complete(node, future, timeout_sec=3.0)
            self.assertTrue(future.done() and future.result().success)
            proc_info.assertWaitForShutdown(process=stack, timeout=15)
        finally:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
            if PREVIOUS_DOMAIN is None:
                os.environ.pop("ROS_DOMAIN_ID", None)
            else:
                os.environ["ROS_DOMAIN_ID"] = PREVIOUS_DOMAIN


@launch_testing.post_shutdown_test()
class TestExpectedFailureExit(unittest.TestCase):
    def test_stalled_simulation_is_not_reported_as_success(self, proc_info, stack):
        launch_testing.asserts.assertExitCodes(proc_info, process=stack, allowable_exit_codes=[1])
