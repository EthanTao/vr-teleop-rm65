"""Deterministic, labelled XR fixture for the simulated execution interface only."""

from __future__ import annotations

import json
import math
import time

from .input_adapter import rotvec_quaternion


class MockStartupGate:
    """Require recent release acknowledgements from both input and tracking."""

    def __init__(self, timeout_s: float = 0.5):
        self.timeout_s = timeout_s
        self.input_released_at: float | None = None
        self.tracking_released_at: float | None = None

    @staticmethod
    def _status(raw: str) -> dict:
        try:
            value = json.loads(raw)
            return value if isinstance(value, dict) else {}
        except (TypeError, ValueError):
            return {}

    def input_status(self, raw: str, now: float) -> None:
        status = self._status(raw)
        released = status.get("reason") == "released" and status.get("enabled") is False
        self.input_released_at = now if released else None

    def tracking_status(self, raw: str, now: float) -> None:
        status = self._status(raw)
        released = status.get("state") == "grip_released" and all(
            status.get(name) is False for name in ("active", "requested", "release_required", "restart_required")
        )
        self.tracking_released_at = now if released else None

    def ready(self, now: float) -> bool:
        return math.isfinite(now) and all(
            then is not None and 0.0 <= now - then <= self.timeout_s
            for then in (self.input_released_at, self.tracking_released_at)
        )


def make_packet(elapsed_s: float, timestamp_ns: int) -> str:
    """Six gentle translation directions followed by mixed pose motion and release."""
    import numpy as np

    start_s, segment_s = 1.0, 2.0
    segment = int(max(0.0, elapsed_s - start_s) / segment_s)
    local = max(0.0, elapsed_s - start_s) % segment_s
    grip = 0.8 if start_s <= elapsed_s < start_s + 7 * segment_s else 0.0
    xyz = np.array([0.0, 0.0, 0.0])
    rotation = np.zeros(3)
    pulse = math.sin(math.pi * local / segment_s) ** 2
    if segment < 6:
        xyz[segment // 2] = (1.0 if segment % 2 == 0 else -1.0) * 0.004 * pulse
    elif segment == 6:
        xyz[:] = [0.003 * pulse, -0.002 * pulse, 0.002 * pulse]
        rotation[:] = [0.04 * pulse, 0.02 * pulse, -0.03 * pulse]
    pose = [*xyz, *rotvec_quaternion(rotation)]
    value = {
        "Input": 1,
        "timeStampNs": timestamp_ns,
        "_rm65_mock": True,
        "Controller": {"right": {"pose": pose, "grip": grip, "status": 3}},
    }
    return json.dumps({"functionName": "Tracking", "value": value, "_rm65_mock": True})


def main(args=None) -> None:
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from std_msgs.msg import String

    rclpy.init(args=args)

    class MockXrNode(Node):
        def __init__(self):
            super().__init__("mock_xr_input")
            self.declare_parameter("simulation_only", False)
            self.declare_parameter("raw_topic", "/rm65/xr/raw")
            self.declare_parameter("rate_hz", 50.0)
            self.declare_parameter("wait_for_input_ready", True)
            self.declare_parameter("input_status_topic", "/rm65/input_status")
            self.declare_parameter("tracking_status_topic", "/rm65/pose_tracking/status")
            if self.get_parameter("simulation_only").value is not True:
                raise ValueError("mock_xr_input requires simulation_only:=true; do not attach it to hardware")
            rate = float(self.get_parameter("rate_hz").value)
            if not math.isfinite(rate) or rate <= 0.0:
                raise ValueError("rate_hz must be finite and positive")
            self.publisher = self.create_publisher(
                String,
                self.get_parameter("raw_topic").value,
                QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
            )
            self.startup_gate = MockStartupGate()
            self.started = None if self.get_parameter("wait_for_input_ready").value else time.monotonic()
            self.create_subscription(String, self.get_parameter("input_status_topic").value, self.input_status, 1)
            self.create_subscription(String, self.get_parameter("tracking_status_topic").value, self.tracking_status, 1)
            self.last_timestamp = 0
            self.timer = self.create_timer(1.0 / rate, self.publish_sample)

        def input_status(self, message):
            if self.started is not None:
                return
            self.startup_gate.input_status(message.data, time.monotonic())

        def tracking_status(self, message):
            if self.started is not None:
                return
            self.startup_gate.tracking_status(message.data, time.monotonic())

        def publish_sample(self):
            now = time.monotonic()
            if self.started is None and self.startup_gate.ready(now):
                # No fixed initialization delay: the tracker must first confirm
                # it consumed an actual false Grip lease after finishing startup.
                self.started = now
            self.last_timestamp = max(self.last_timestamp + 1, int(now * 1e9))
            elapsed = 0.0 if self.started is None else now - self.started
            self.publisher.publish(String(data=make_packet(elapsed, self.last_timestamp)))

    node = MockXrNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except _rclpy.RCLError:
        if rclpy.ok():
            raise
    finally:
        try:
            node.destroy_node()
        finally:
            if rclpy.ok():
                rclpy.shutdown()
