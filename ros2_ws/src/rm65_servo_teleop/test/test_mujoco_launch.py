"""Actual Servo/JTC/MuJoCo motion and Grip-release acceptance, without hardware."""

import importlib.util
from pathlib import Path

import pytest

pytest.importorskip("rclpy", reason="MuJoCo graph acceptance requires Linux ROS")
spec = importlib.util.spec_from_file_location(
    "rm65_common_graph_test", Path(__file__).with_name("test_servo_launch.py")
)
common = importlib.util.module_from_spec(spec)
spec.loader.exec_module(common)


@pytest.mark.launch_test
def generate_test_description():
    return common.simulation_description("mujoco")


class TestActualMujocoSimulation(common.TestActualServoSimulation):
    pass


@common.launch_testing.post_shutdown_test()
class TestMujocoExitCodes(common.TestSimulationExitCodes):
    pass
