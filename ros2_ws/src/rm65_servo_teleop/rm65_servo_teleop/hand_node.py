"""ROS2 mock hand task component. No sockets, SDK handles or arm publishers."""

from __future__ import annotations

import json
from pathlib import Path
import time

import yaml

from .hand_session import HandProfile, HandSession, MockHand
from .hand_input import HandButtonAdapter


def validate_mode(mode: str, hand_mode: str):
    if hand_mode not in {"off", "mock"}:
        raise ValueError("hand_mode must be off or mock; real-hand writes are not implemented")
    if hand_mode == "mock" and mode != "sim":
        raise ValueError("synthetic hand feedback is forbidden in hardware sessions")


def main(args=None):
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from std_msgs.msg import String

    class HandNode(Node):
        def __init__(self):
            super().__init__("rm65_hand")
            mode = self.declare_parameter("mode", "disabled").value
            validate_mode(mode, "mock")
            profile_path = str(self.declare_parameter("profile", "").value)
            profile = HandProfile.from_dict(yaml.safe_load(Path(profile_path).read_text(encoding="utf-8")))
            self.session = HandSession(profile)
            self.transport = MockHand(profile)
            self.buttons = HandButtonAdapter(
                timeout_s=float(self.declare_parameter("input_timeout_s", 0.2).value),
                grip_on=float(self.declare_parameter("grip_on", 0.2).value),
                grip_off=float(self.declare_parameter("grip_off", 0.15).value),
            )
            self.status_pub = self.create_publisher(String, "/rm65/hand/status", 10)
            self.events_pub = self.create_publisher(String, "/rm65/hand/events", 64)
            self.subscription = self.create_subscription(String, "/rm65/hand/command", self.command, 1)
            from rclpy.qos import QoSProfile, ReliabilityPolicy

            self.xr_subscription = self.create_subscription(
                String,
                "/rm65/xr/raw",
                self.xr_input,
                QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
            )
            self.timer = self.create_timer(0.05, self.tick)
            self.get_logger().info("Synthetic ROH task rehearsal; no physical hand or arm commands")

        def command(self, message):
            try:
                if len(message.data) > 2048:
                    raise ValueError("hand command exceeds 2048 characters")
                envelope = json.loads(message.data)
            except ValueError:
                envelope = None
            self.session.command(envelope, time.monotonic())
            self.publish()

        def tick(self):
            now = time.monotonic()
            self.buttons.check(now)
            self.session.observe(self.transport.poll(now), now)
            self.session.flush_mock(self.transport, now)
            self.publish()

        def xr_input(self, message):
            now = time.monotonic()
            actions = self.buttons.receive_raw(message.data, now, self.session.stage)
            for action in actions:
                accepted = self.session.command(
                    {
                        "action": action,
                        "session_id": self.session.session_id,
                        "sequence": self.session.last_command_sequence + 1,
                        "expected_revision": self.session.revision,
                        "source": {"kind": "xr_buttons", **self.buttons.diagnostics(now)},
                    },
                    now,
                )
                if not accepted:
                    break  # Never replay a rejected action when its phase changes.
            self.publish()

        def publish(self):
            while self.session.events:
                self.events_pub.publish(String(data=json.dumps(self.session.events.popleft(), allow_nan=False)))
            now = time.monotonic()
            status = {**self.session.status(now), "controller": self.buttons.diagnostics(now)}
            self.status_pub.publish(String(data=json.dumps(status, allow_nan=False)))

    rclpy.init(args=args)
    node = None
    try:
        node = HandNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # No automatic open, reset or purported physical stop on shutdown.
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
