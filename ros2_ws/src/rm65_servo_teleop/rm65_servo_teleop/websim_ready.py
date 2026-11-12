"""Do not activate JTC against TopicBasedSystem's not-yet-populated zero state."""
from pathlib import Path
import time

import numpy as np
import yaml

from .model import JOINT_NAMES
from .transport_guard import ordered_joints


def matches_initial_feedback(names, positions, expected):
    try:
        q = np.asarray(ordered_joints(names, positions))
        home = np.asarray(expected, dtype=float)
        return home.shape == (6,) and np.all(np.isfinite(home)) and bool(np.max(np.abs(q - home)) <= .001)
    except (TypeError, ValueError):
        return False


def main(args=None):
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import ExternalShutdownException
    from sensor_msgs.msg import JointState

    rclpy.init(args=args)
    node = Node("rm65_websim_initial_feedback")
    config = yaml.safe_load(Path(node.declare_parameter("model_config", "").value).read_text(encoding="utf-8"))
    expected = config["model"]["home_joint_rad"]
    accepted, previous = 0, -1

    def receive(message):
        nonlocal accepted, previous
        stamp = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
        age = (node.get_clock().now().nanoseconds - stamp) * 1e-9
        if stamp <= previous:
            return
        previous = stamp
        if not 0 <= age <= .2 or not matches_initial_feedback(message.name, message.position, expected):
            accepted = 0
            return
        accepted += 1

    node.create_subscription(JointState, "/joint_states", receive, 1)
    deadline = time.monotonic() + 15
    try:
        while rclpy.ok() and accepted < 2 and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.02)
        if accepted < 2:
            raise RuntimeError("JTC not activated: physical initial feedback did not reach joint_state_broadcaster")
        node.get_logger().info("Fresh MuJoCo feedback matches configured initial joints; JTC may activate")
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
