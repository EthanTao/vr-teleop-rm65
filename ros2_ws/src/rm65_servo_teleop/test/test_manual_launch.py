"""Real Linux Servo/JTC/MuJoCo acceptance of the local manual HTTP input.

Only the input endpoint receives commands. Joint feedback is independently
observed from ros2_control; this test never publishes /joint_states or contacts
a physical robot. Run in the network-isolated, hardware-free test container.
"""

from __future__ import annotations

from collections import deque
import json
import math
import os
from pathlib import Path
import re
import secrets
import time
import unittest
from urllib.error import HTTPError, URLError
from urllib.request import ProxyHandler, Request, build_opener
import uuid

import pytest

rclpy = pytest.importorskip("rclpy", reason="manual simulation acceptance requires Linux ROS")
launch = pytest.importorskip("launch")
launch_testing = pytest.importorskip("launch_testing")
pytest.importorskip("launch_ros")

from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing.actions
import launch_testing.asserts
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState

DOMAIN = str(101 + secrets.randbelow(60))
PREVIOUS_DOMAIN = os.environ.get("ROS_DOMAIN_ID")
PORT = 21000 + secrets.randbelow(10001)
ZERO = [0, 0, 0, 0, 0, 0]
POSITIVE_Y = [0, 1, 0, 0, 0, 0]
POSITIVE_RX = [0, 0, 0, 1, 0, 0]


@pytest.mark.launch_test
def generate_test_description():
    os.environ["ROS_DOMAIN_ID"] = DOMAIN
    share = Path(get_package_share_directory("rm65_servo_teleop"))
    stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(share / "launch" / "teleop.launch.py")),
        launch_arguments={
            "mode": "sim",
            "input_source": "manual",
            "simulator": "mujoco",
            "manual_port": str(PORT),
            "rviz": "false",
            "foxglove": "false",
            "mujoco_viewer": "false",
            "record": "false",
        }.items(),
    )
    return launch.LaunchDescription(
        [SetEnvironmentVariable("ROS_DOMAIN_ID", DOMAIN), stack, launch_testing.actions.ReadyToTest()]
    )


def _position_delta(before, after):
    return tuple(float(b) - float(a) for a, b in zip(before["position"], after["position"]))


def _relative_quaternion(before, after):
    """q_after * inverse(q_before): rotation expressed in the base frame."""
    ax, ay, az, aw = after["quaternion_xyzw"]
    bx, by, bz, bw = before["quaternion_xyzw"]
    bx, by, bz = -bx, -by, -bz
    result = (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )
    return tuple(-value for value in result) if result[3] < 0 else result


class TestManualSimulation(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        if rclpy.ok():
            rclpy.shutdown()
        if PREVIOUS_DOMAIN is None:
            os.environ.pop("ROS_DOMAIN_ID", None)
        else:
            os.environ["ROS_DOMAIN_ID"] = PREVIOUS_DOMAIN

    def setUp(self):
        self.node = rclpy.create_node("rm65_manual_acceptance_observer")
        self.names = tuple(f"joint{index}" for index in range(1, 7))
        self.joints = deque(maxlen=20_000)
        self.feedback_errors = deque(maxlen=10)
        self.subscription = self.node.create_subscription(
            JointState,
            "/joint_states",
            self._joint_callback,
            QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT),
        )
        self.opener = build_opener(ProxyHandler({}))
        self.base_url = f"http://127.0.0.1:{PORT}"
        self.client_id = str(uuid.uuid4())
        self.seq = 0
        self.token = None
        self.latest = None

    def tearDown(self):
        self.node.destroy_node()

    def _joint_callback(self, message):
        try:
            self.assertEqual(len(message.name), len(message.position))
            self.assertEqual(len(message.name), len(set(message.name)))
            indices = [list(message.name).index(name) for name in self.names]
            positions = tuple(float(message.position[index]) for index in indices)
            self.assertTrue(all(math.isfinite(value) for value in positions))
            stamp_ns = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
            self.assertGreater(stamp_ns, 0)
            if self.joints:
                self.assertGreaterEqual(stamp_ns, self.joints[-1][1])
                if stamp_ns == self.joints[-1][1]:
                    return
            self.joints.append((time.monotonic(), stamp_ns, positions))
        except (AssertionError, TypeError, ValueError, IndexError) as exception:
            self.feedback_errors.append(str(exception))

    def _spin(self, timeout=0.005):
        rclpy.spin_once(self.node, timeout_sec=timeout)
        self.assertFalse(self.feedback_errors, list(self.feedback_errors))

    def _request(self, path, payload=None):
        data = None if payload is None else json.dumps(payload).encode("utf-8")
        request = Request(self.base_url + path, data=data, headers={"Content-Type": "application/json"})
        try:
            with self.opener.open(request, timeout=0.5) as response:
                raw = response.read().decode("utf-8")
        except HTTPError as exception:
            self.fail(f"manual endpoint {path} returned HTTP {exception.code}: {exception.read().decode('utf-8')}")
        return raw if path == "/" else json.loads(raw)

    def _state(self):
        state = self._request("/api/state")
        self.assertEqual(state.get("mode"), "simulation_manual")
        self.latest = state
        return state

    def _post(self, pressed, axes):
        self.seq += 1
        return self._request(
            "/api/input",
            {"token": self.token, "client_id": self.client_id, "seq": self.seq, "pressed": pressed, "axes": axes},
        )

    def _wait_for_server_and_feedback(self):
        deadline = time.monotonic() + 60.0
        last_error = None
        while time.monotonic() < deadline:
            self._spin(0.02)
            try:
                if self.token is None:
                    html = self._request("/")
                    match = re.search(r"[0-9a-f]{64}", html)
                    self.assertIsNotNone(match, "manual page does not contain its input token")
                    self.token = match.group(0)
                state = self._state()
                if len(self.joints) >= 5 and state.get("actual_pose") and state.get("joint"):
                    return state
            except (URLError, TimeoutError, ConnectionError) as exception:
                last_error = str(exception)
        self.fail(f"manual server/actual joint feedback did not become available: {last_error}; {self.latest}")

    def _drive(self, pressed, axes, duration_s, *, require_inactive=False):
        deadline = time.monotonic() + duration_s
        next_heartbeat = time.monotonic()
        states = []
        while time.monotonic() < deadline:
            now = time.monotonic()
            if now >= next_heartbeat:
                self._post(pressed, axes)
                state = self._state()
                states.append(state)
                if require_inactive:
                    self.assertFalse(state["control"]["active"], state)
                    self.assertFalse(state["input"]["enabled"], state)
                next_heartbeat = now + 0.01
            self._spin(0.002)
        self.assertTrue(states)
        return states

    def _observe_without_heartbeat(self, duration_s):
        deadline = time.monotonic() + duration_s
        next_state = time.monotonic()
        states = []
        while time.monotonic() < deadline:
            now = time.monotonic()
            if now >= next_state:
                states.append(self._state())
                next_state = now + 0.06
            self._spin()
        return states

    def _spread_since(self, started):
        samples = [sample[2] for sample in self.joints if sample[0] >= started]
        self.assertGreaterEqual(len(samples), 8, "insufficient independent JointState samples")
        return max(max(q[index] for q in samples) - min(q[index] for q in samples) for index in range(6))

    def test_manual_axes_release_and_lost_heartbeat(self):
        initial = self._wait_for_server_and_feedback()
        self.assertFalse(initial["control"]["active"], initial)
        initial_start = time.monotonic()
        idle_states = self._observe_without_heartbeat(0.6)
        idle_spread = self._spread_since(initial_start)
        self.assertLessEqual(idle_spread, 1e-5, "manual input moved before any press")
        self.assertTrue(all(not state["control"]["active"] for state in idle_states))

        deadline = time.monotonic() + 15.0
        while time.monotonic() < deadline:
            state = self._drive(False, ZERO, 0.12)[-1]
            if state["ready"]:
                break
        self.assertTrue(state["ready"], state)
        self.assertLess(state["feedback_age_s"], 0.2, state)
        self.assertLess(state["tf_age_s"], 0.2, state)
        start_q = self.joints[-1][2]
        start_pose = state["actual_pose"]
        # Establish the new measured anchor before testing axis motion. HTTP
        # snapshots update at 50Hz; three 10ms polls can still show the old
        # released segment's target, unlike the former three 60ms polls.
        self._drive(True, ZERO, 0.2)
        # Re-anchoring begins on the first press. Compare the accepted target
        # at the beginning of that segment with its final target.
        moving = self._drive(True, POSITIVE_Y, 1.8)
        self.assertTrue(any(sample["control"]["active"] for sample in moving), moving[-1])
        self.assertTrue(any(sample["tracking"]["active"] for sample in moving), moving[-1])
        # The HTTP snapshot can briefly retain the preceding segment's target
        # while the normal ROS adapter establishes its new anchor.
        target_samples = [sample["target_pose"] for sample in moving[3:] if sample.get("target_pose")]
        self.assertGreaterEqual(len(target_samples), 10)
        target_delta = _position_delta(target_samples[0], target_samples[-1])
        actual_delta = _position_delta(start_pose, moving[-1]["actual_pose"])
        joint_excursion = max(abs(a - b) for a, b in zip(start_q, self.joints[-1][2]))
        self.assertGreater(target_delta[1], 0.005, f"positive Y target did not advance: {target_delta}")
        self.assertLess(max(abs(target_delta[0]), abs(target_delta[2])), 1e-4, target_delta)
        self.assertGreater(actual_delta[1], 0.002, f"actual TCP did not move in positive Y: {actual_delta}")
        self.assertLess(max(abs(actual_delta[0]), abs(actual_delta[2])), max(0.003, actual_delta[1] * 0.4))
        self.assertGreater(joint_excursion, 1e-4, "actual /joint_states did not move")
        translation_rotation = _relative_quaternion(target_samples[0], target_samples[-1])
        self.assertLess(math.sqrt(sum(value * value for value in translation_rotation[:3])), 1e-4)

        self._drive(False, ZERO, 1.0)
        release_settled_at = time.monotonic()
        released = self._drive(False, ZERO, 1.0)
        release_spread = self._spread_since(release_settled_at)
        self.assertLessEqual(release_spread, 1e-5, "joints kept moving after release and 1 s settling")
        self.assertFalse(released[-1]["control"]["active"], released[-1])
        self.assertFalse(released[-1]["input"]["enabled"], released[-1])

        second_q = self.joints[-1][2]
        second_motion = self._drive(True, POSITIVE_Y, 1.0)
        self.assertTrue(any(sample["control"]["active"] for sample in second_motion), second_motion[-1])
        self.assertGreater(max(abs(a - b) for a, b in zip(second_q, self.joints[-1][2])), 1e-4)
        lost = self._observe_without_heartbeat(1.2)
        self.assertFalse(lost[-1]["control"]["active"], lost[-1])
        self.assertFalse(lost[-1]["input"]["enabled"], lost[-1])
        held_after_timeout_at = time.monotonic()
        self._drive(True, POSITIVE_Y, 0.8, require_inactive=True)
        timeout_spread = self._spread_since(held_after_timeout_at)
        self.assertLessEqual(timeout_spread, 1e-5, "held input re-enabled after heartbeat loss without release")

        rearmed = self._drive(False, ZERO, 0.5)[-1]
        self.assertTrue(rearmed["ready"], rearmed)
        rotation_start = rearmed["actual_pose"]
        self._drive(True, ZERO, 0.2)
        rotating = self._drive(True, POSITIVE_RX, 1.0)
        self.assertTrue(any(sample["control"]["active"] for sample in rotating), rotating[-1])
        rotation_targets = [sample["target_pose"] for sample in rotating[3:] if sample.get("target_pose")]
        self.assertGreaterEqual(len(rotation_targets), 8)
        rotation_delta = _relative_quaternion(rotation_targets[0], rotation_targets[-1])
        self.assertGreater(rotation_delta[0], 0.002, f"RX target is not positive in base frame: {rotation_delta}")
        self.assertLess(max(abs(rotation_delta[1]), abs(rotation_delta[2])), 1e-3, rotation_delta)
        self.assertLess(max(abs(value) for value in _position_delta(rotation_targets[0], rotation_targets[-1])), 1e-4)
        actual_rotation = _relative_quaternion(rotation_start, rotating[-1]["actual_pose"])
        actual_rotation_deg = math.degrees(
            2 * math.atan2(math.sqrt(sum(v * v for v in actual_rotation[:3])), actual_rotation[3])
        )
        self.assertGreater(actual_rotation_deg, 0.05, "actual TCP did not rotate after release/re-arm")
        self._drive(False, ZERO, 1.0)
        print(
            "MANUAL_ACCEPTANCE "
            f"idle_spread_rad={idle_spread:.9g} "
            f"target_delta_m={target_delta} actual_delta_m={actual_delta} "
            f"joint_excursion_rad={joint_excursion:.9g} "
            f"release_spread_rad={release_spread:.9g} timeout_spread_rad={timeout_spread:.9g} "
            f"rx_actual_deg={actual_rotation_deg:.6g}",
            flush=True,
        )


@launch_testing.post_shutdown_test()
class TestManualExitCodes(unittest.TestCase):
    def test_control_components_exit_cleanly(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[0, -2, 130])
