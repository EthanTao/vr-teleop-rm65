"""Official IK execution into private MuJoCo topics only; no robot I/O."""

from __future__ import annotations

import json
from pathlib import Path
import sys
import time

import numpy as np

from .input_adapter import Pose
from .official_follow import OfficialFollower
from .official_follow_runner import load_follow_config
from .official_sdk import OfficialAlgo
from .transport_guard import JOINT_NAMES, ordered_joints


def main(args=None):
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import PoseStamped
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Bool, String
    from tf2_ros import Buffer, TransformException, TransformListener
    from rclpy.time import Time

    class SimulationFollower(Node):
        def __init__(self):
            super().__init__("rm65_official_sim_follow")
            param = lambda name, value: self.declare_parameter(name, value).value
            if param("simulation_only", False) is not True:
                raise ValueError("This executable requires simulation_only:=true")
            sdk_root = param("sdk_root", "")
            if sdk_root:
                sys.path.insert(0, str(Path(sdk_root).resolve()))
            shared, self.config = load_follow_config(param("model_config", ""), param("follow_config", ""))
            solver = OfficialAlgo(shared["model"]["tool_xyz_m"], shared["model"]["tool_rpy_rad"],
                                  d6_m=shared["model"]["d6_m"])
            self.follower = OfficialFollower(self.config, solver)
            self.frame = shared["input"]["planning_frame"]
            self.tcp_frame = shared["input"]["tcp_frame"]
            self.buffer = Buffer()
            self.listener = TransformListener(self.buffer, self)
            self.target = self.joints = None
            self.target_at = self.feedback_at = self.enable_at = None
            self.target_stamp = self.feedback_stamp = -1
            self.enabled = False
            self.output = self.create_publisher(String, "/rm65/websim/controller_status", 10)
            self.command_output = self.create_publisher(JointState, "/rm65/websim/joint_commands", 1)
            self.create_subscription(PoseStamped, "/rm65/target_pose", self.on_target, 1)
            self.create_subscription(JointState, "/joint_states", self.on_feedback, 1)
            self.create_subscription(Bool, "/rm65/enable", self.on_enable, 1)
            self.create_timer(self.config.period_s, self.tick)
            self.get_logger().info("SIMULATION ONLY: official IK writes private MuJoCo commands, no robot driver")
            self.output.publish(String(data=json.dumps({"sdk": solver.identity, "joint_names": JOINT_NAMES})))

        def age(self, stamp):
            return (self.get_clock().now().nanoseconds - stamp.sec * 1_000_000_000 - stamp.nanosec) * 1e-9

        def on_enable(self, message):
            self.enabled, self.enable_at = message.data, time.monotonic()

        def on_target(self, message):
            stamp = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
            age = self.age(message.header.stamp)
            if message.header.frame_id != self.frame or not 0 <= age <= self.config.source_timeout_s:
                self.target_at = None
                return
            if stamp <= self.target_stamp:
                if stamp < self.target_stamp:
                    self.target_at = None
                return
            p, q = message.pose.position, message.pose.orientation
            try:
                self.target = Pose.from_vector([p.x, p.y, p.z, q.x, q.y, q.z, q.w])
            except ValueError:
                self.target_at = None
                return
            self.target_stamp, self.target_at = stamp, time.monotonic() - age

        def on_feedback(self, message):
            stamp = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
            age = self.age(message.header.stamp)
            if not 0 <= age <= self.config.source_timeout_s:
                self.feedback_at = None
                return
            if stamp <= self.feedback_stamp:
                if stamp < self.feedback_stamp:
                    self.feedback_at = None
                return
            try:
                joints = ordered_joints(message.name, message.position)
                velocity = ordered_joints(message.name, message.velocity)
                if np.any(np.abs(velocity) > np.asarray(self.config.joint_velocity) + 1e-9):
                    raise ValueError("measured_joint_overspeed")
            except (TypeError, ValueError):
                self.feedback_at = None
                return
            self.joints, self.feedback_stamp, self.feedback_at = joints, stamp, time.monotonic() - age

        def tick(self):
            now = time.monotonic()
            measured_pose = None
            feedback_at = self.feedback_at
            try:
                transform = self.buffer.lookup_transform(self.frame, self.tcp_frame, Time())
                age = self.age(transform.header.stamp)
                if not 0 <= age <= self.config.source_timeout_s:
                    raise ValueError("stale_tf")
                t, q = transform.transform.translation, transform.transform.rotation
                measured_pose = Pose.from_vector([t.x, t.y, t.z, q.x, q.y, q.z, q.w])
                feedback_at = None if feedback_at is None else min(feedback_at, now - age)
            except (TransformException, ValueError):
                feedback_at = None
            # Do not interpret a lost enable heartbeat as an operator release.
            # Retain requested=True and force the core stale-input fault.
            target_at = self.target_at
            if self.enabled and (self.enable_at is None or now - self.enable_at > self.config.source_timeout_s):
                target_at = None
            result = self.follower.step(enabled=self.enabled, desired=self.target, target_at=target_at,
                                        measured_pose=measured_pose, measured_joints=self.joints,
                                        feedback_at=feedback_at, now=now)
            # Only a fresh physical sample can support a hold or candidate command.
            command = result.command_rad if result.command_rad is not None else self.joints
            sent = False
            if command is not None and feedback_at is not None and 0 <= now - feedback_at <= self.config.source_timeout_s:
                message = JointState()
                message.header.stamp = self.get_clock().now().to_msg()
                message.name, message.position = list(JOINT_NAMES), list(map(float, command))
                self.command_output.publish(message)
                sent = True
            self.output.publish(String(data=json.dumps({**result.record(), "simulation_command_sent": sent},
                                                      allow_nan=False)))

    rclpy.init(args=args)
    node = None
    try:
        node = SimulationFollower()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except (_rclpy.RCLError, RuntimeError):
        if rclpy.ok():
            raise
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
