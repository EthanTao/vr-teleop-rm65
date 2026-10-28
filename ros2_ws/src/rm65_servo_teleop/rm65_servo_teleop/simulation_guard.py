"""Fail the entire local simulation if physics pauses, resets or stops advancing.

The control stack intentionally keeps wall ROS time, matching the hardware
contract and preserving steady watchdogs. Joint broadcaster timestamps alone do
not prove that MuJoCo physics is still running; /clock progress is checked too.
"""

from __future__ import annotations

import math
import time


class PhysicsProgress:
    def __init__(self, *, timeout_s=0.20, startup_timeout_s=15.0, now=None):
        if not all(math.isfinite(v) and v > 0 for v in (timeout_s, startup_timeout_s)):
            raise ValueError("physics watchdog timeouts must be finite and positive")
        self.timeout_s, self.startup_timeout_s = timeout_s, startup_timeout_s
        self.started = time.monotonic() if now is None else now
        self.last_ns = None
        self.progress_at = None

    def receive(self, stamp_ns: int, now: float):
        if stamp_ns < 0 or (self.last_ns is not None and stamp_ns < self.last_ns):
            raise RuntimeError("MuJoCo physics clock reset/regressed; restart the simulation")
        if self.last_ns is None or stamp_ns > self.last_ns:
            self.progress_at = now
            self.last_ns = stamp_ns

    def check(self, now: float):
        if self.progress_at is None:
            if now - self.started > self.startup_timeout_s:
                raise RuntimeError("MuJoCo physics did not start")
        elif now - self.progress_at > self.timeout_s:
            raise RuntimeError("MuJoCo physics paused/stalled; closing the control graph; restart required")


def configure_guard(node):
    """Attach a steady timer and reject changes to the wall-clock contract."""
    from rcl_interfaces.msg import SetParametersResult
    from rclpy.clock import Clock as ROSClock, ClockType
    from rclpy.parameter import Parameter
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from rosgraph_msgs.msg import Clock

    if node.get_parameter("use_sim_time").value:
        raise ValueError("simulation watchdog requires wall time")

    def require_wall_time(parameters):
        for parameter in parameters:
            if parameter.name == "use_sim_time" and (parameter.type_ != Parameter.Type.BOOL or parameter.value):
                return SetParametersResult(successful=False, reason="simulation watchdog requires use_sim_time=false")
        return SetParametersResult(successful=True)

    node.add_on_set_parameters_callback(require_wall_time)
    guard = PhysicsProgress(
        timeout_s=float(node.declare_parameter("timeout_s", 0.20).value),
        startup_timeout_s=float(node.declare_parameter("startup_timeout_s", 15.0).value),
    )
    node.create_subscription(
        Clock,
        "/clock",
        lambda msg: guard.receive(msg.clock.sec * 1_000_000_000 + msg.clock.nanosec, time.monotonic()),
        QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
    )
    # The timer itself must advance even if the node's ROS clock is ever paused.
    node.create_timer(0.02, lambda: guard.check(time.monotonic()), clock=ROSClock(clock_type=ClockType.STEADY_TIME))


def main(args=None):
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy
    from rclpy.node import Node

    rclpy.init(args=args)
    node = Node("rm65_simulation_guard")
    try:
        configure_guard(node)
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except _rclpy.RCLError:
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
