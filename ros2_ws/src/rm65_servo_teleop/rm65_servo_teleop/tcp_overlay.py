"""Read-only simulation TCP text for the native MuJoCo viewer.

The goal is the external operator goal, not PoseTracking's rolling/clamped target.
Current TCP is measured from the simulated joint-state TF chain, not from a command.
This module deliberately imports ROS only in configure_overlay/main so formatting
and timestamp/freshness behavior can be tested without ROS or an OpenGL context.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
import time


OVERLAY_TOPIC = "/rm65/sim/tcp_overlay"
MAX_TEXT_BYTES = 1536


@dataclass(frozen=True)
class PoseObservation:
    xyz: tuple[float, float, float]
    quaternion_xyzw: tuple[float, float, float, float]
    stamp_ns: int
    received_at: float

    @classmethod
    def create(cls, xyz, quaternion_xyzw, stamp_ns: int, received_at: float):
        xyz = tuple(float(value) for value in xyz)
        quaternion = tuple(float(value) for value in quaternion_xyzw)
        if len(xyz) != 3 or len(quaternion) != 4:
            raise ValueError("pose_shape")
        if not all(math.isfinite(value) for value in (*xyz, *quaternion, received_at)):
            raise ValueError("pose_nonfinite")
        if max(abs(value) for value in xyz) > 100.0:
            raise ValueError("position_out_of_range")
        norm = math.hypot(*quaternion)
        if norm < 1e-12 or abs(norm - 1.0) > 1e-3:
            raise ValueError("quaternion_not_unit")
        if not isinstance(stamp_ns, int) or isinstance(stamp_ns, bool) or stamp_ns <= 0:
            raise ValueError("timestamp_invalid")
        return cls(xyz, tuple(value / norm for value in quaternion), stamp_ns, float(received_at))

    def rpy_degrees(self) -> tuple[float, float, float]:
        """Fixed-axis XYZ roll/pitch/yaw (R = Rz(yaw) Ry(pitch) Rx(roll))."""
        x, y, z, w = self.quaternion_xyzw
        roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
        pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
        yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
        return tuple(math.degrees(value) for value in (roll, pitch, yaw))


class TcpOverlay:
    """Pure bounded state; duplicate source stamps do not refresh receipt age."""

    def __init__(self, frame: str = "base_link", tcp_frame: str = "tcp", stale_s: float = 0.2):
        if not all(isinstance(value, str) and value and len(value) <= 48 for value in (frame, tcp_frame)):
            raise ValueError("frame_invalid")
        if not all(all(character.isascii() and (character.isalnum() or character in "_/") for character in value) for value in (frame, tcp_frame)):
            raise ValueError("frame_invalid")
        if not math.isfinite(stale_s) or not 0.01 <= stale_s <= 2.0:
            raise ValueError("stale_s_invalid")
        self.frame, self.tcp_frame, self.stale_s = frame, tcp_frame, stale_s
        self.samples: dict[str, PoseObservation | None] = {"goal": None, "current": None}
        self.errors = {"goal": "NO TARGET (release Grip, then grip)", "current": "NO TF"}

    def invalidate(self, source: str, reason: str) -> None:
        # Do not retain a previously valid pose as if this invalid observation were valid.
        self.samples[source] = None
        self.errors[source] = "INVALID " + "".join(c for c in str(reason) if c.isascii() and (c.isalnum() or c in "_ -"))[:80]

    def receive(self, source: str, xyz, quaternion_xyzw, stamp_ns: int, received_at: float, frame: str) -> None:
        if source not in self.samples:
            raise ValueError("source_invalid")
        if frame != self.frame:
            self.invalidate(source, "frame_mismatch")
            return
        try:
            sample = PoseObservation.create(xyz, quaternion_xyzw, stamp_ns, received_at)
        except (ValueError, TypeError, OverflowError) as exc:
            self.invalidate(source, str(exc))
            return
        previous = self.samples[source]
        if previous is not None and previous.stamp_ns == sample.stamp_ns:
            if previous.xyz != sample.xyz or previous.quaternion_xyzw != sample.quaternion_xyzw:
                self.invalidate(source, "timestamp_reused_with_changed_pose")
            # The unchanged source stamp is NOT fresh evidence just because TF was queried again.
            return
        self.samples[source] = sample
        self.errors[source] = ""

    def _rows(self, source: str, now_ns: int, monotonic_now: float) -> list[str]:
        label = "GOAL external" if source == "goal" else "CURRENT TF"
        sample = self.samples[source]
        if sample is None:
            return [f"{label}: {self.errors[source]}", "  XYZ(m): --  --  --", "  RPY(deg): --  --  --"]
        age = (now_ns - sample.stamp_ns) * 1e-9
        receipt_age = monotonic_now - sample.received_at
        if not all(math.isfinite(value) for value in (age, receipt_age)) or age < -0.02 or receipt_age < 0:
            status = "INVALID CLOCK"
        elif age > self.stale_s or receipt_age > self.stale_s:
            status = "STALE"
        else:
            status = "OK"
        # Even stale values retain their ORIGINAL source age; no restamping/cached freshness.
        return [
            f"{label}: {status} age={age:.3f}s rx={receipt_age:.3f}s",
            "  XYZ(m): " + "  ".join(f"{value:+.4f}" for value in sample.xyz),
            "  RPY(deg): " + "  ".join(f"{value:+.1f}" for value in sample.rpy_degrees()),
        ]

    def render(self, now_ns: int, monotonic_now: float) -> str:
        rows = [f"SIM TCP | {self.frame} -> {self.tcp_frame} | XYZ m / RPY deg"]
        rows.extend(self._rows("goal", now_ns, monotonic_now))
        rows.extend(self._rows("current", now_ns, monotonic_now))
        text = "\n".join(rows)
        if len(text.encode("ascii")) > MAX_TEXT_BYTES:
            raise ValueError("overlay_text_too_long")
        return text


def configure_overlay(node):
    """Install an observer only; launch exclusively for MuJoCo simulation."""
    from geometry_msgs.msg import PoseStamped
    from rcl_interfaces.msg import SetParametersResult
    from rclpy.clock import Clock, ClockType
    from rclpy.duration import Duration
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from rclpy.time import Time
    from std_msgs.msg import String
    from tf2_ros import Buffer, TransformException, TransformListener

    for name, default in (("mode", ""), ("planning_frame", "base_link"), ("tcp_frame", "tcp"), ("stale_s", 0.2)):
        node.declare_parameter(name, default)
    if node.get_parameter("mode").value != "sim" or not node.get_parameter("use_sim_time").value:
        raise ValueError("TCP overlay requires mode=sim and use_sim_time=true; not hardware telemetry")
    model = TcpOverlay(node.get_parameter("planning_frame").value, node.get_parameter("tcp_frame").value, float(node.get_parameter("stale_s").value))
    node.add_on_set_parameters_callback(lambda parameters: SetParametersResult(successful=not any(parameter.name == "use_sim_time" and parameter.value is not True for parameter in parameters), reason="TCP overlay requires use_sim_time=true"))
    publisher = node.create_publisher(String, OVERLAY_TOPIC, QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
    buffer = Buffer(cache_time=Duration(seconds=2.0))
    listener = TransformListener(buffer, node)

    def target(message):
        pose = message.pose
        model.receive("goal", (pose.position.x, pose.position.y, pose.position.z), (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w), message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec, time.monotonic(), message.header.frame_id)

    subscription = node.create_subscription(PoseStamped, "/rm65/target_pose", target, QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE))

    def tick():
        now = time.monotonic()
        try:
            transform = buffer.lookup_transform(model.frame, model.tcp_frame, Time())
            translation, rotation = transform.transform.translation, transform.transform.rotation
            model.receive("current", (translation.x, translation.y, translation.z), (rotation.x, rotation.y, rotation.z, rotation.w), transform.header.stamp.sec * 1_000_000_000 + transform.header.stamp.nanosec, now, transform.header.frame_id)
        except TransformException:
            model.invalidate("current", "TF_unavailable")
        publisher.publish(String(data=model.render(node.get_clock().now().nanoseconds, now)))

    # A steady timer still marks frozen TF stale when /clock is paused/stalled.
    timer = node.create_timer(0.1, tick, clock=Clock(clock_type=ClockType.STEADY_TIME))
    node._tcp_overlay_resources = model, buffer, listener, subscription, timer, publisher
    return model


def main(args=None) -> None:
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node

    rclpy.init(args=args)
    node = Node("rm65_tcp_overlay")
    try:
        configure_overlay(node)
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
