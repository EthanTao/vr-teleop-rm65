"""Simulation-only ROS endpoint. MuJoCo 4 runs in an owned Python 3.11 worker."""
from __future__ import annotations

import json
import math
import time

from .model import JOINT_NAMES
from .transport_guard import ordered_joints
from .websim_process import PhysicsProcess


def main(args=None):
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import ExternalShutdownException
    from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy
    from geometry_msgs.msg import PoseStamped
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Bool, String

    class Bridge(Node):
        def __init__(self):
            super().__init__("rm65_websim_physics")
            param = lambda key, value: self.declare_parameter(key, value).value
            if param("simulation_only", False) is not True:
                raise ValueError("websim execution requires simulation_only:=true")
            self.controller = param("controller", "")
            if self.controller not in {"servo", "official"}:
                raise ValueError("invalid simulation controller")
            self.feedback = self.create_publisher(JointState, "/rm65/websim/joint_states", 1)
            self.global_feedback = (self.create_publisher(JointState, "/joint_states", 1)
                                    if self.controller == "official" else None)
            self.clock_output = self.create_publisher(Clock, "/clock", 1)
            self.create_subscription(JointState, "/rm65/websim/joint_commands", self.command, 1)
            self.create_subscription(PoseStamped, "/rm65/target_pose", self.target, 1)
            self.create_subscription(Bool, "/rm65/enable", self.enable, 1)
            self.create_subscription(String, "/rm65/input_status", self.input_status, 1)
            status = "/rm65/pose_tracking/status" if self.controller == "servo" else "/rm65/websim/controller_status"
            self.create_subscription(String, status, self.controller_status, 1)
            self.enabled, self.enable_at = False, None
            self.input_reason, self.controller_reason = "waiting_for_pico", ""
            self.desired = self.target_at = self.latest_command = self.command_stamp = None
            self.physics_time = 0.0
            options = ["--controller", self.controller]
            for key, default in {"model_config": "", "physics_config": "", "vendor": "", "semantic": "",
                                 "directory": "", "page": "", "assets": "/opt/rm65-web-assets",
                                 "address": "127.0.0.1", "port": 8094}.items():
                options += ["--" + key.replace("_", "-"), str(param(key, default))]
            self.worker = PhysicsProcess(options)
            self.create_timer(.004, self.tick)
            self.get_logger().info("PICO browser simulation ready: http://127.0.0.1:8094/ ; no robot connected")

        def command(self, msg):
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            age = self.get_clock().now().nanoseconds * 1e-9 - stamp
            if not 0 <= age <= .2 or (self.command_stamp is not None and stamp <= self.command_stamp):
                raise ValueError("stale or regressed simulation command")
            q = ordered_joints(msg.name, msg.position)
            self.latest_command = {"names": list(JOINT_NAMES), "position": list(q), "stamp_s": stamp}
            self.command_stamp = stamp

        def target(self, msg):
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            age = self.get_clock().now().nanoseconds * 1e-9 - stamp
            p = msg.pose.position
            xyz = [p.x, p.y, p.z]
            if msg.header.frame_id != "base_link" or not all(math.isfinite(x) for x in xyz) or not 0 <= age <= .2:
                self.desired = None
                return
            self.desired, self.target_at = xyz, time.monotonic() - age

        def enable(self, msg):
            self.enabled, self.enable_at = msg.data, time.monotonic()

        def input_status(self, msg):
            self.input_reason = str(json.loads(msg.data).get("reason", ""))[:256]

        def controller_status(self, msg):
            value = json.loads(msg.data)
            self.controller_reason = str(value.get("reason", value.get("state", "")))[:256]

        def tick(self):
            now = time.monotonic()
            target = self.desired if self.target_at is not None and 0 <= now - self.target_at <= .2 else None
            state = self.worker.exchange({"command": self.latest_command,
                                          "ros_now_s": self.get_clock().now().nanoseconds * 1e-9,
                                          "view": {"target": target, "input_reason": self.input_reason,
                                                   "controller_reason": self.controller_reason,
                                                   "enabled": self.enabled and self.enable_at is not None and
                                                   0 <= now - self.enable_at <= .2}})
            if not math.isfinite(state["time"]) or state["time"] <= self.physics_time:
                raise RuntimeError("MuJoCo physics clock did not advance")
            self.physics_time = state["time"]
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = list(JOINT_NAMES)
            msg.position = list(ordered_joints(JOINT_NAMES, state["joints"]))
            msg.velocity = list(ordered_joints(JOINT_NAMES, state["velocity"]))
            self.feedback.publish(msg)
            if self.global_feedback is not None:
                self.global_feedback.publish(msg)
            clock = Clock()
            seconds = state["time"]
            clock.clock.sec = int(seconds)
            clock.clock.nanosec = int((seconds - int(seconds)) * 1e9)
            self.clock_output.publish(clock)

    rclpy.init(args=args)
    node = None
    try:
        node = Bridge()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except (_rclpy.RCLError, RuntimeError):
        if rclpy.ok():
            raise
    finally:
        if node is not None:
            node.worker.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
