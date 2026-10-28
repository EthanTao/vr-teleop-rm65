"""Actual shared ROS launch with a synthetic hand; no controller network."""

from collections import deque
import json
import os
from pathlib import Path
import secrets
import subprocess
import sys
import time
import unittest

import pytest

rclpy = pytest.importorskip("rclpy", reason="hand ROS integration requires Linux ROS")
launch = pytest.importorskip("launch")
launch_testing = pytest.importorskip("launch_testing")
pytest.importorskip("launch_ros")

from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing.actions
import launch_testing.asserts
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy

DOMAIN = str(101 + secrets.randbelow(60))


@pytest.mark.launch_test
def generate_test_description():
    os.environ["ROS_DOMAIN_ID"] = DOMAIN
    share = Path(get_package_share_directory("rm65_servo_teleop"))
    stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(share / "launch/teleop.launch.py")),
        launch_arguments={
            "mode": "sim",
            "input_source": "external",
            "simulator": "mock",
            "hand_mode": "mock",
            "record": "false",
            "rviz": "false",
            "foxglove": "false",
        }.items(),
    )
    return launch.LaunchDescription(
        [
            SetEnvironmentVariable("ROS_DOMAIN_ID", DOMAIN),
            stack,
            launch_testing.actions.ReadyToTest(),
        ]
    )


class TestHandGraph(unittest.TestCase):
    def test_grasp_confirm_insert_and_explicit_release(self):
        rclpy.init()
        node = rclpy.create_node("hand_acceptance_observer")
        statuses, events = deque(maxlen=128), deque(maxlen=512)
        subscriptions = [
            node.create_subscription(
                String, "/rm65/hand/status", lambda msg: statuses.append(json.loads(msg.data)), 10
            ),
            node.create_subscription(String, "/rm65/hand/events", lambda msg: events.append(json.loads(msg.data)), 128),
        ]
        publisher = node.create_publisher(String, "/rm65/hand/command", 1)
        raw_publisher = node.create_publisher(
            String, "/rm65/xr/raw", QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        )

        def wait(condition, timeout=5.0):
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.02)
                if condition():
                    return
            self.fail(f"condition timed out; latest status={statuses[-1] if statuses else None}")

        def buttons(duration, *, grip=0.0, trigger=0.0, a=False, b=False):
            deadline = time.monotonic() + duration
            while time.monotonic() < deadline:
                raw = {
                    "_rm65_mock": True,
                    "Input": 1,
                    "timeStampNs": time.monotonic_ns(),
                    "Controller": {
                        "right": {
                            "pose": [0, 0, 0, 0, 0, 0, 1],
                            "grip": grip,
                            "trigger": trigger,
                            "primaryButton": a,
                            "secondaryButton": b,
                        }
                    },
                }
                raw_publisher.publish(String(data=json.dumps({"functionName": "Tracking", "value": json.dumps(raw)})))
                rclpy.spin_once(node, timeout_sec=0.02)
                time.sleep(0.02)

        try:
            wait(lambda: statuses and publisher.get_subscription_count() > 0, 60.0)
            self.assertEqual(statuses[-1]["stage"], "unknown")
            self.assertIsNone(statuses[-1]["target"])
            # Exercise the installed single-action tool through actual DDS,
            # including discovery, session/revision selection and ACK delivery.
            result = subprocess.run(
                [sys.executable, "-m", "rm65_servo_teleop.hand_command", "prepare"],
                capture_output=True,
                text=True,
                timeout=8.0,
            )
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            self.assertEqual(json.loads(result.stdout)["kind"], "accepted")
            wait(lambda: statuses[-1]["stage"] == "ready")
            wait(lambda: raw_publisher.get_subscription_count() >= 2)
            buttons(0.2)  # Neutral once before accepting any PICO button.
            buttons(1.0, trigger=1.0)  # A held Trigger must generate one target only.
            buttons(0.15)
            wait(lambda: statuses[-1]["stage"] == "awaiting_grasp_confirmation")
            self.assertFalse(statuses[-1]["physical_grasp_verified"])
            buttons(0.12, a=True)
            buttons(0.12)
            wait(lambda: statuses[-1]["stage"] == "holding")
            buttons(0.12, grip=1.0)
            buttons(0.15)  # Releasing Grip does not release the object.
            wait(lambda: statuses[-1]["stage"] == "inserting")
            before = len([row for row in events if row["kind"] == "sent"])
            status = statuses[-1]
            request = {
                "action": "release",
                "session_id": status["session_id"],
                "sequence": status["last_command_sequence"] + 1,
                "expected_revision": status["revision"],
            }
            publisher.publish(String(data=json.dumps(request)))
            wait(
                lambda: any(row["kind"] == "rejected" and row.get("sequence") == request["sequence"] for row in events)
            )
            wait(lambda: statuses[-1]["last_command_sequence"] == request["sequence"])
            self.assertEqual(statuses[-1]["stage"], "inserting")
            self.assertEqual(before, len([row for row in events if row["kind"] == "sent"]))
            buttons(0.2)  # Re-arm after any wait without advancing XR frames.
            buttons(0.4, b=True)
            buttons(0.15)
            self.assertEqual(statuses[-1]["stage"], "inserting")
            self.assertEqual(before, len([row for row in events if row["kind"] == "sent"]))
            buttons(1.3, b=True)
            buttons(1.0)
            wait(lambda: statuses[-1]["stage"] == "released")
            self.assertTrue(statuses[-1]["synthetic"])
            self.assertFalse(statuses[-1]["hardware_writes"])
            self.assertFalse(statuses[-1]["arm_motion_authorized"])
            self.assertEqual(len([row for row in events if row["kind"] == "sent"]), 3)
            candidates = [row for row in events if row["kind"] == "candidate" and row.get("source")]
            self.assertTrue(candidates)
            self.assertTrue(all(row["source"]["kind"] == "xr_buttons" for row in candidates))
            self.assertEqual(len([row for row in candidates if row["action"] == "grasp"]), 1)
            publishers = {name for name, _ in node.get_publisher_names_and_types_by_node("rm65_hand", "/")}
            self.assertFalse(
                {"/rm65/enable", "/rm65/target_pose", "/joint_states", "/rm65_arm_controller/joint_trajectory"}
                & publishers
            )
        finally:
            node.destroy_node()
            rclpy.shutdown()


@launch_testing.post_shutdown_test()
class TestHandExitCodes(unittest.TestCase):
    def test_components_exit_cleanly(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[0, -2, 130])
