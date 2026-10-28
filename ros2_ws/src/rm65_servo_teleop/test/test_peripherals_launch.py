"""Real ROS/DDS sidecar integration; synthetic sensors, no arm launch/network."""

from collections import deque
import json
import os
from pathlib import Path
import secrets
import tempfile
import time
import unittest

import pytest

rclpy = pytest.importorskip("rclpy", reason="peripheral graph test requires Linux ROS")
launch = pytest.importorskip("launch")
launch_testing = pytest.importorskip("launch_testing")
pytest.importorskip("launch_ros")
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing.actions
from std_msgs.msg import String

from rm65_servo_teleop.peripheral_recording import verify_capture

DOMAIN = str(101 + secrets.randbelow(60))
ROOT = Path(tempfile.mkdtemp(prefix="rm65-peripherals-test-", dir=os.environ.get("RM65_PERIPHERAL_TEST_ROOT")))


@pytest.mark.launch_test
def generate_test_description():
    os.environ["ROS_DOMAIN_ID"] = DOMAIN
    share = Path(get_package_share_directory("rm65_servo_teleop"))
    stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(share / "launch/peripherals.launch.py")),
        launch_arguments={
            "hand": "mock",
            "camera": "mock",
            "record_root": str(ROOT),
            "run_label": "synthetic-dds-test",
        }.items(),
    )
    return launch.LaunchDescription(
        [SetEnvironmentVariable("ROS_DOMAIN_ID", DOMAIN), stack, launch_testing.actions.ReadyToTest()]
    )


class TestPeripheralGraph(unittest.TestCase):
    def test_separate_camera_and_hand_data_no_arm_publishers(self):
        rclpy.init()
        node = rclpy.create_node("peripheral_acceptance")
        hands, cameras, events = deque(maxlen=64), deque(maxlen=64), deque(maxlen=512)
        subscriptions = [
            node.create_subscription(String, "/rm65/hand/status", lambda m: hands.append(json.loads(m.data)), 10),
            node.create_subscription(String, "/rm65/camera/status", lambda m: cameras.append(json.loads(m.data)), 10),
            node.create_subscription(String, "/rm65/hand/events", lambda m: events.append(json.loads(m.data)), 128),
        ]
        command = node.create_publisher(String, "/rm65/hand/command", 8)

        def until(predicate):
            deadline = time.monotonic() + 12
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.05)
                if predicate():
                    return
            self.fail("peripheral DDS state deadline exceeded")

        def action(name, stage):
            state = hands[-1]
            command.publish(
                String(
                    data=json.dumps(
                        {
                            "session_id": state["session_id"],
                            "sequence": state["last_command_sequence"] + 1,
                            "expected_revision": state["revision"],
                            "action": name,
                        }
                    )
                )
            )
            until(lambda: hands[-1].get("stage") == stage)

        try:
            until(lambda: hands and hands[-1].get("feedback_fresh") and cameras and cameras[-1]["frames"] >= 3)
            assert hands[-1]["synthetic"] is True and hands[-1]["hardware_writes"] is False
            assert cameras[-1]["synthetic"] is True and cameras[-1]["hand_eye"] is None
            action("prepare", "ready")
            action("grasp", "awaiting_grasp_confirmation")
            action("confirm_grasp", "holding")
            action("begin_insert", "inserting")
            action("finish_insert", "inserted")
            action("release", "released")
            sent = [e for e in events if e.get("kind") == "sent"]
            assert len(sent) == 3 and all(e["synthetic"] is True for e in sent)
            for driver in ("rm65_camera_driver", "rm65_hand_driver"):
                publishers = node.get_publisher_names_and_types_by_node(driver, "/")
                assert all(
                    name.startswith(("/rm65/camera/", "/rm65/hand/")) or name in ("/rosout", "/parameter_events")
                    for name, _ in publishers
                )
            assert node.count_publishers("/rm65/enable") == 0
            assert node.count_publishers("/rm65_arm_controller/joint_trajectory") == 0
        finally:
            node.destroy_node()
            rclpy.shutdown()


@launch_testing.post_shutdown_test()
class TestSealedCaptures(unittest.TestCase):
    def test_all_sidecar_files_sealed(self):
        captures = list(ROOT.glob("sensors_*"))
        assert len(captures) == 2
        manifests = [verify_capture(p) for p in captures]
        assert sum(m["frames"] > 0 for m in manifests) == 1
        assert all(m["rows"] > 0 and m["complete"] is True for m in manifests)
