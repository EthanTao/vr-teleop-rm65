"""Explicit simulation-session reset request; never a Home/motion sender.

The host launcher waits for the entire old graph to exit, then creates a new
simulation. Native world-reset/clock-regression protection stays unchanged.
"""

from __future__ import annotations

import json
import math
import os
from pathlib import Path
import re
import time


def write_request(root: Path, token: str, *, now: float | None = None) -> Path:
    if not re.fullmatch(r"[0-9a-f]{32}", token):
        raise ValueError("Simulation reset requires an owned session token")
    folder = root / ".sim-reset"
    folder.mkdir(parents=True, exist_ok=True)
    path = folder / f"{token}.json"
    stamp = time.time() if now is None else now
    if isinstance(stamp, bool) or not isinstance(stamp, (int, float)) or not math.isfinite(stamp):
        raise ValueError("Invalid reset request time")
    value = {"version": 1, "mode": "sim", "action": "reset", "token": token, "requested_at": stamp}
    # Exclusive create: one session cannot overwrite/replay a previous request.
    with path.open("x", encoding="utf-8") as stream:
        json.dump(value, stream, allow_nan=False)
        stream.flush()
        os.fsync(stream.fileno())
    return path


def main(args=None):
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile
    from std_msgs.msg import Empty

    rclpy.init(args=args)
    node = Node("rm65_simulation_reset")
    try:
        if node.declare_parameter("simulation_only", False).value is not True:
            raise ValueError("Simulation reset is forbidden outside explicit simulation")
        token = os.environ.get("RM65_SIM_RESET_TOKEN", "")
        if not re.fullmatch(r"[0-9a-f]{32}", token):
            raise ValueError("Simulation reset requires the Python simulation entry")
        root = Path(node.declare_parameter("record_root", "/data/experiments").value)
        pending = False

        def request(_message=None):
            nonlocal pending
            pending = True

        def tick():
            if pending:
                write_request(root, token)
                # This is a critical node: launch shuts down EVERY control node.
                # There is no joint/enable/target publisher and no reset_world call.
                raise RuntimeError("Explicit simulation reset: closing the old graph before host restart")

        node.create_subscription(
            Empty,
            "/rm65/simulation/reset_request",
            request,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        node.create_timer(0.02, tick)
        node.get_logger().info(
            "Simulation reset ready; explicit request only; fresh Grip release required after restart"
        )
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
