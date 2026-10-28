"""Standalone peripheral ROS processes, with zero arm command publishers."""

import json
from pathlib import Path
import time

import yaml

from .camera import CameraConfig
from .camera_runtime import CameraRuntime
from .hand_runtime import HandRuntime
from .peripheral_recording import PeripheralRecorder


def main(kind, args=None):
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from std_msgs.msg import String

    class PeripheralNode(Node):
        def __init__(self):
            super().__init__("rm65_" + kind + "_driver")
            self.runtime = self.recorder = None
            profile_path = self.declare_parameter("profile", "").value
            mode = self.declare_parameter("backend", "disabled").value
            root = self.declare_parameter("record_root", "/data/experiments").value
            label = self.declare_parameter("run_label", "").value
            profile = yaml.safe_load(Path(profile_path).read_text(encoding="utf-8"))
            self.status_pub = self.create_publisher(String, f"/rm65/{kind}/status", 10)
            if kind == "camera":
                self.runtime = CameraRuntime(mode, CameraConfig(**profile), root, run_label=label)
            else:
                ack = self.declare_parameter("operator_ack", "").value
                from ament_index_python.packages import get_package_share_directory

                arm_config = yaml.safe_load(
                    (Path(get_package_share_directory("rm65_servo_teleop")) / "config/teleop.yaml").read_text()
                )
                inputs = arm_config["input"]
                self.runtime = HandRuntime(
                    mode,
                    profile,
                    acknowledgement=ack,
                    input_config={
                        "timeout_s": inputs["xr_timeout_s"],
                        "grip_on": inputs["grip_on"],
                        "grip_off": inputs["grip_off"],
                    },
                )
                self.recorder = PeripheralRecorder(
                    root,
                    {"hand": {"mode": mode, "profile": profile, "synthetic": mode == "mock"}},
                    capacity=128,
                    run_label=label,
                )
                self.events_pub = self.create_publisher(String, "/rm65/hand/events", 128)
                self.command_sub = self.create_subscription(String, "/rm65/hand/command", self.command, 8)
                self.xr_sub = self.create_subscription(
                    String,
                    "/rm65/xr/raw",
                    lambda msg: self.runtime.xr_input(msg.data),
                    QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
                )
            self.runtime.start()
            self.timer = self.create_timer(0.05 if kind == "hand" else 0.2, self.tick)

        def command(self, msg):
            try:
                envelope = json.loads(msg.data) if len(msg.data) <= 2048 else None
                if not self.runtime.command(envelope):
                    self.events_pub.publish(
                        String(data=json.dumps({"kind": "rejected", "reason": "readonly_or_queue_full"}))
                    )
            except ValueError:
                self.events_pub.publish(String(data=json.dumps({"kind": "rejected", "reason": "invalid_json"})))

        def tick(self):
            if kind == "camera":
                status = self.runtime.snapshot()
            else:
                status, events = self.runtime.snapshot()
                for event in events:
                    self.events_pub.publish(String(data=json.dumps(event, allow_nan=False)))
                    self.recorder.append({"kind": "hand_event", "event": event})
                self.recorder.append(
                    {"kind": "hand_status", "host_received_monotonic_s": time.monotonic(), "value": status}
                )
                status = {**status, "capture_path": str(self.recorder.path)}
            self.status_pub.publish(String(data=json.dumps(status, allow_nan=False)))
            error = self.runtime.error or (self.recorder.error if self.recorder else None)
            if error:
                raise RuntimeError(error)

        def close(self):
            if self.runtime:
                self.runtime.close()
            if self.recorder:
                status, events = self.runtime.snapshot()
                for event in events:
                    self.recorder.append({"kind": "hand_event", "event": event})
                self.recorder.append(
                    {"kind": "hand_status", "host_received_monotonic_s": time.monotonic(), "value": status}
                )
                if self.runtime.error:
                    self.recorder.fail(self.runtime.error)
                self.recorder.close()

    rclpy.init(args=args)
    node = None
    try:
        node = PeripheralNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
