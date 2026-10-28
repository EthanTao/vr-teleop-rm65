"""ROS2 adapter from raw XR protocol to measured-anchored Servo pose targets."""

from __future__ import annotations

import json
import math
import time

import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformException, TransformListener

from .input_adapter import InputAdapter, InputConfig, InputRejected, Pose


class XrInputNode(Node):
    def __init__(self):
        super().__init__("xr_input")
        defaults = {
            "raw_topic": "/rm65/xr/raw",
            "target_topic": "/rm65/target_pose",
            "enable_topic": "/rm65/enable",
            "input_status_topic": "/rm65/input_status",
            "planning_frame": "base_link",
            "tcp_frame": "tcp",
            "feedback_topic": "/joint_states",
            "joint_names": [f"joint{i}" for i in range(1, 7)],
            "control_rate_hz": 50.0,
            "position_scale": 1.0,
            "rotation_scale": 0.25,
            "grip_on": 0.2,
            "grip_off": 0.15,
            "xr_timeout_s": 0.20,
            "feedback_timeout_s": 0.20,
            "tf_timeout_s": 0.20,
            "max_xr_step_m": 0.05,
            "max_xr_rot_step_rad": 0.26,
            "max_rotation_offset_rad": 0.20,
            "allow_synthetic": False,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        parameter = lambda name: self.get_parameter(name).value
        self.adapter = InputAdapter(InputConfig(**{name: parameter(name) for name in InputConfig.__dataclass_fields__}))
        self.planning_frame = str(parameter("planning_frame"))
        self.tcp_frame = str(parameter("tcp_frame"))
        self.joint_names = tuple(parameter("joint_names"))
        self.tf_timeout_s = float(parameter("tf_timeout_s"))
        rate_hz = float(parameter("control_rate_hz"))
        if len(self.joint_names) != 6 or len(set(self.joint_names)) != 6:
            raise ValueError("joint_names must contain six unique joints")
        if (
            not math.isfinite(rate_hz)
            or rate_hz <= 0.0
            or not math.isfinite(self.tf_timeout_s)
            or self.tf_timeout_s <= 0.0
        ):
            raise ValueError("control rate and TF timeout must be positive and finite")
        reliable = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        sensor = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self._target_publisher = self.create_publisher(PoseStamped, parameter("target_topic"), reliable)
        self._enable_publisher = self.create_publisher(Bool, parameter("enable_topic"), reliable)
        self._status_publisher = self.create_publisher(String, parameter("input_status_topic"), reliable)
        self.create_subscription(String, parameter("raw_topic"), self._raw_callback, sensor)
        self.create_subscription(JointState, parameter("feedback_topic"), self._feedback_callback, sensor)
        self._feedback_at: float | None = None
        self._feedback_stamp_ns: int | None = None
        self._last_reason: str | None = None
        self._tf_age_s: float | None = None
        self._last_tf_failure: dict | None = None
        self._tf_buffer = Buffer(cache_time=Duration(seconds=2.0))
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._timer = self.create_timer(1.0 / rate_hz, self._tick)

    def _raw_callback(self, message: String) -> None:
        self.adapter.receive_raw(message.data, time.monotonic())

    def _feedback_callback(self, message: JointState) -> None:
        try:
            names = list(message.name)
            if len(names) != len(set(names)) or len(message.position) != len(names):
                raise ValueError("invalid joint state shape")
            indices = [names.index(name) for name in self.joint_names]
            if not all(math.isfinite(message.position[index]) for index in indices):
                raise ValueError("nonfinite joint state")
            stamp_ns = int(message.header.stamp.sec) * 1_000_000_000 + int(message.header.stamp.nanosec)
            age_s = (self.get_clock().now().nanoseconds - stamp_ns) * 1e-9
            if stamp_ns <= 0 or age_s < -0.02 or age_s > self.adapter.config.feedback_timeout_s:
                raise ValueError("joint state timestamp invalid or stale")
            if self._feedback_stamp_ns is not None:
                if stamp_ns < self._feedback_stamp_ns:
                    raise ValueError("joint state time regressed")
                if stamp_ns == self._feedback_stamp_ns:
                    return
            self._feedback_stamp_ns = stamp_ns
            self._feedback_at = time.monotonic() - max(0.0, age_s)
        except (ValueError, TypeError, OverflowError):
            self._feedback_at = None
            self.adapter.invalidate("feedback_invalid")

    def _tcp_pose(self) -> tuple[Pose | None, bool]:
        try:
            transform = self._tf_buffer.lookup_transform(self.planning_frame, self.tcp_frame, Time())
            stamp_ns = int(transform.header.stamp.sec) * 1_000_000_000 + int(transform.header.stamp.nanosec)
            now_ns = self.get_clock().now().nanoseconds
            age_s = (now_ns - stamp_ns) * 1e-9
            self._tf_age_s = age_s
            if stamp_ns <= 0 or not -0.02 <= age_s <= self.tf_timeout_s:
                self._last_tf_failure = {
                    "reason": "timestamp_out_of_range",
                    "tf_stamp_ns": stamp_ns,
                    "ros_now_ns": now_ns,
                    "age_s": age_s,
                }
                return None, False
            translation, rotation = transform.transform.translation, transform.transform.rotation
            return (
                Pose.from_vector(
                    [translation.x, translation.y, translation.z, rotation.x, rotation.y, rotation.z, rotation.w]
                ),
                True,
            )
        except (TransformException, InputRejected, ValueError) as exc:
            self._tf_age_s = None
            self._last_tf_failure = {"reason": type(exc).__name__, "detail": str(exc)[:256]}
            return None, False

    def _tick(self) -> None:
        tcp, tf_fresh = self._tcp_pose()
        decision = self.adapter.step(time.monotonic(), tcp, self._feedback_at, tf_fresh=tf_fresh)
        if decision.target is not None:
            target = PoseStamped()
            target.header.frame_id = self.planning_frame
            target.header.stamp = self.get_clock().now().to_msg()
            target.pose.position.x, target.pose.position.y, target.pose.position.z = map(float, decision.target.xyz)
            q = decision.target.quaternion_xyzw
            (
                target.pose.orientation.x,
                target.pose.orientation.y,
                target.pose.orientation.z,
                target.pose.orientation.w,
            ) = map(float, q)
            self._target_publisher.publish(target)
        self._enable_publisher.publish(Bool(data=decision.enabled))
        diagnostics = decision.diagnostics()
        diagnostics.update(
            tf_age_s=self._tf_age_s,
            feedback_age_s=None if self._feedback_at is None else time.monotonic() - self._feedback_at,
            last_tf_failure=self._last_tf_failure,
        )
        self._status_publisher.publish(String(data=json.dumps(diagnostics, allow_nan=False)))
        if decision.reason != self._last_reason:
            self.get_logger().info(f"XR {decision.reason}; anchor={decision.anchor_id}; enabled={decision.enabled}")
            self._last_reason = decision.reason

    def disable(self) -> None:
        self.adapter.invalidate("input_node_stopping")
        if rclpy.ok():
            try:
                self._enable_publisher.publish(Bool(data=False))
            except _rclpy.RCLError:
                if rclpy.ok():
                    raise


def main(args=None) -> None:
    rclpy.init(args=args)
    node = XrInputNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except _rclpy.RCLError:
        # A signal can invalidate the context between a callback's checks and
        # publish(). Keep actual middleware failures fatal while ROS is alive.
        if rclpy.ok():
            raise
    finally:
        try:
            node.disable()
        finally:
            try:
                node.destroy_node()
            finally:
                if rclpy.ok():
                    rclpy.shutdown()
