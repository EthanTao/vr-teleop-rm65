"""Linux ROS integration test of the actual Servo -> JTC -> mock hardware graph.

Run in the built/sourced Humble container, for example:
    colcon test --merge-install --packages-select rm65_servo_teleop \
        --event-handlers console_direct+ --return-code-on-test-failure

This test never publishes joint feedback or connects to a physical robot. A
Windows pytest collection skips it when ROS/launch_testing are unavailable.
Code/static checks alone do not constitute a passed ROS integration test.
The observer and launched graph share a randomly allocated test-only ROS domain.
Run the test in the supplied network-isolated, hardware-free test container.
"""

from __future__ import annotations

from collections import deque
import json
import math
import os
from pathlib import Path
import secrets
import time
import unittest

import pytest

rclpy = pytest.importorskip("rclpy", reason="ROS Humble integration test requires Linux ROS runtime")
launch = pytest.importorskip("launch", reason="ROS launch runtime is not installed")
launch_testing = pytest.importorskip("launch_testing", reason="ROS launch_testing is not installed")
pytest.importorskip("launch_ros", reason="ROS launch_ros is not installed")

from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing.actions
import launch_testing.asserts
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
import yaml

_TEST_ROS_DOMAIN_ID = str(101 + secrets.randbelow(60))
_PREVIOUS_ROS_DOMAIN_ID = os.environ.get("ROS_DOMAIN_ID")


@pytest.mark.launch_test
def generate_test_description():
    return simulation_description("mock")


def simulation_description(simulator):
    # Affect this test process only; a subprocess cannot change its caller's
    # shell environment. The launch action also pins delayed child processes to
    # the same domain rather than depending on observer teardown timing.
    os.environ["ROS_DOMAIN_ID"] = _TEST_ROS_DOMAIN_ID
    share = Path(get_package_share_directory("rm65_servo_teleop"))
    simulated_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(share / "launch" / "teleop.launch.py")),
        launch_arguments={
            "mode": "sim",
            "simulator": simulator,
            "input_source": "mock",
            "record": "false",
            "foxglove": "false",
            "rviz": "false",
        }.items(),
    )
    return launch.LaunchDescription(
        [
            SetEnvironmentVariable("ROS_DOMAIN_ID", _TEST_ROS_DOMAIN_ID),
            simulated_stack,
            launch_testing.actions.ReadyToTest(),
        ]
    )


class TestActualServoSimulation(unittest.TestCase):
    """Require observed mock-hardware motion and a settled post-release state."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        if rclpy.ok():
            rclpy.shutdown()
        if _PREVIOUS_ROS_DOMAIN_ID is None:
            os.environ.pop("ROS_DOMAIN_ID", None)
        else:
            os.environ["ROS_DOMAIN_ID"] = _PREVIOUS_ROS_DOMAIN_ID

    def setUp(self):
        self.node = rclpy.create_node("rm65_servo_integration_observer")
        share = Path(get_package_share_directory("rm65_servo_teleop"))
        configuration = yaml.safe_load((share / "config" / "teleop.yaml").read_text(encoding="utf-8"))
        self.names = tuple(f"joint{index}" for index in range(1, 7))
        self.lower = tuple(configuration["model"]["joint_lower_rad"])
        self.upper = tuple(configuration["model"]["joint_upper_rad"])
        self.velocity = tuple(configuration["model"]["joint_velocity_rad_s"])
        self.period = float(configuration["servo"]["publish_period"])
        self.states = deque(maxlen=20_000)
        self.enable_samples = deque(maxlen=10_000)
        self.pose_status_samples = deque(maxlen=10_000)
        self.errors = deque(maxlen=20)
        self.last_input_status = None
        self.raw_active_at = None
        self.raw_released_at = None
        sensor_qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.subscriptions = [
            self.node.create_subscription(JointState, "/joint_states", self._joint_callback, sensor_qos),
            self.node.create_subscription(Bool, "/rm65/enable", self._enable_callback, 100),
            self.node.create_subscription(String, "/rm65/pose_tracking/status", self._status_callback, 100),
            self.node.create_subscription(String, "/rm65/input_status", self._input_callback, 100),
            self.node.create_subscription(String, "/rm65/xr/raw", self._raw_callback, sensor_qos),
        ]

    def tearDown(self):
        self.node.destroy_node()

    def _joint_callback(self, message):
        try:
            if len(message.name) != len(message.position) or len(set(message.name)) != len(message.name):
                raise ValueError("invalid JointState dimensions or duplicate joint names")
            indices = [list(message.name).index(name) for name in self.names]
            positions = tuple(float(message.position[index]) for index in indices)
            if not all(math.isfinite(value) for value in positions):
                raise ValueError("nonfinite simulated joint state")
            stamp_ns = int(message.header.stamp.sec) * 1_000_000_000 + int(message.header.stamp.nanosec)
            if stamp_ns <= 0:
                raise ValueError("simulated feedback has no positive timestamp")
            for index, position in enumerate(positions):
                if not self.lower[index] - 1e-5 <= position <= self.upper[index] + 1e-5:
                    raise ValueError(f"{self.names[index]} exceeded configured position limits: {position}")
            if self.states:
                previous_stamp = self.states[-1][1]
                if stamp_ns < previous_stamp:
                    raise ValueError("JointState time regressed")
                if stamp_ns == previous_stamp:
                    return
                dt = (stamp_ns - previous_stamp) * 1e-9
                previous_positions = self.states[-1][2]
                # Allow one control period for asynchronous state/publication
                # scheduling. This checks discontinuities, not physical speed.
                for index, position in enumerate(positions):
                    allowance = self.velocity[index] * (dt + self.period) + 1e-5
                    if abs(position - previous_positions[index]) > allowance:
                        raise ValueError(f"{self.names[index]} discontinuity exceeds sample-time allowance")
            self.states.append((time.monotonic(), stamp_ns, positions))
        except (TypeError, ValueError, IndexError) as exception:
            self.errors.append(str(exception))

    def _enable_callback(self, message):
        self.enable_samples.append((time.monotonic(), bool(message.data)))

    def _status_callback(self, message):
        try:
            value = json.loads(message.data)
            if not isinstance(value, dict) or type(value.get("active")) is not bool:
                raise ValueError("malformed PoseTracking status")
            self.pose_status_samples.append((time.monotonic(), value))
        except (TypeError, ValueError) as exception:
            self.errors.append(str(exception))

    def _input_callback(self, message):
        try:
            self.last_input_status = json.loads(message.data)
        except (TypeError, ValueError) as exception:
            self.errors.append(str(exception))

    def _raw_callback(self, message):
        try:
            packet = json.loads(message.data)
            value = packet["value"]
            if packet.get("_rm65_mock") is not True or value.get("_rm65_mock") is not True:
                self.errors.append("simulation fixture was replaced by an unlabelled XR source")
                return
            grip = float(value["Controller"]["right"]["grip"])
            now = time.monotonic()
            if grip >= 0.2 and self.raw_active_at is None:
                self.raw_active_at = now
            elif grip <= 0.15 and self.raw_active_at is not None and self.raw_released_at is None:
                self.raw_released_at = now
        except (KeyError, TypeError, ValueError) as exception:
            self.errors.append(f"invalid mock XR packet: {exception}")

    def _diagnostics(self):
        latest_pose = self.pose_status_samples[-1][1] if self.pose_status_samples else None
        return (
            f"joint_samples={len(self.states)}, enable_samples={len(self.enable_samples)}, "
            f"raw_active={self.raw_active_at}, raw_released={self.raw_released_at}, "
            f"input_status={self.last_input_status}, pose_status={latest_pose}, errors={list(self.errors)}"
        )

    def _wait_until(self, predicate, deadline, description):
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            self.assertFalse(self.errors, self._diagnostics())
            if predicate():
                return
        self.fail(f"Timed out waiting for {description}; {self._diagnostics()}")

    def _observe_for(self, duration_s):
        deadline = time.monotonic() + duration_s
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=min(0.05, max(0.0, deadline - time.monotonic())))
            self.assertFalse(self.errors, self._diagnostics())

    def test_fixture_moves_and_release_stops_the_simulated_robot(self):
        overall_deadline = time.monotonic() + 90.0
        self._wait_until(
            lambda: len(self.states) >= 5 and any(enabled for _, enabled in self.enable_samples),
            min(overall_deadline, time.monotonic() + 60.0),
            "fresh robot feedback and accepted mock Grip input",
        )
        self._wait_until(
            lambda: self.raw_released_at is not None,
            min(overall_deadline, time.monotonic() + 19.0),
            "the fixture's six-direction/mixed-motion sequence to release Grip",
        )
        self.assertIsNotNone(self.raw_active_at)
        moving_samples = [sample for sample in self.states if self.raw_active_at <= sample[0] <= self.raw_released_at]
        self.assertGreaterEqual(len(moving_samples), 40, self._diagnostics())
        reference = moving_samples[0][2]
        excursion = max(abs(sample[2][index] - reference[index]) for sample in moving_samples for index in range(6))
        self.assertGreater(excursion, 1e-4, f"The actual /joint_states feedback did not move; {self._diagnostics()}")
        self.assertTrue(
            any(status["active"] for _, status in self.pose_status_samples),
            f"PoseTracking never entered its active state; {self._diagnostics()}",
        )

        def released_and_inactive():
            return (
                self.enable_samples
                and self.enable_samples[-1][0] >= self.raw_released_at
                and self.enable_samples[-1][1] is False
                and self.pose_status_samples
                and self.pose_status_samples[-1][0] >= self.raw_released_at
                and self.pose_status_samples[-1][1]["active"] is False
            )

        self._wait_until(released_and_inactive, min(overall_deadline, time.monotonic() + 2.0), "release/inactive state")
        self._observe_for(1.0)  # Let the last bounded JTC trajectory settle.
        settled_start = time.monotonic()
        self._observe_for(1.5)
        settled = [sample[2] for sample in self.states if sample[0] >= settled_start]
        self.assertGreaterEqual(len(settled), 25, self._diagnostics())
        for index in range(6):
            spread = max(sample[index] for sample in settled) - min(sample[index] for sample in settled)
            self.assertLessEqual(spread, 1e-5, f"{self.names[index]} kept moving after Grip release: {spread}")
        self.assertFalse(self.enable_samples[-1][1], self._diagnostics())
        self.assertFalse(self.pose_status_samples[-1][1]["active"], self._diagnostics())
        # Normal release is not a fault. A release_required=true assertion belongs
        # to a separate source/feedback-timeout test, not this normal-release case.


@launch_testing.post_shutdown_test()
class TestSimulationExitCodes(unittest.TestCase):
    def test_processes_exit_without_control_component_errors(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[0, -2, 130])
