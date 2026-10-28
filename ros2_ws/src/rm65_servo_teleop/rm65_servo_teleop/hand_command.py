"""One explicit action for the currently observed mock hand session."""

from __future__ import annotations

import argparse
from collections import deque
import json
import time

from .hand_session import ACTIONS
from .hand_driver import LIVE_ACK


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("action", choices=ACTIONS)
    parser.add_argument("--live-ack", default="", help="explicit field-only ROH motion acknowledgement")
    args = parser.parse_args(argv)
    import rclpy
    from std_msgs.msg import String

    rclpy.init()
    node = rclpy.create_node("rm65_hand_explicit_action")
    statuses, results = deque(maxlen=1), deque(maxlen=64)
    status_subscription = node.create_subscription(
        String, "/rm65/hand/status", lambda msg: statuses.append(json.loads(msg.data)), 1
    )
    event_subscription = node.create_subscription(
        String, "/rm65/hand/events", lambda msg: results.append(json.loads(msg.data)), 64
    )
    publisher = node.create_publisher(String, "/rm65/hand/command", 1)
    try:
        deadline = time.monotonic() + 3.0
        while (not statuses or publisher.get_subscription_count() == 0) and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if not statuses or publisher.get_subscription_count() == 0:
            raise RuntimeError("mock hand component unavailable; no action sent")
        status = statuses[-1]
        mock = status.get("synthetic") is True and status.get("hardware_writes") is False
        live = (
            args.live_ack == LIVE_ACK
            and status.get("mode") == "live"
            and status.get("synthetic") is False
            and status.get("hardware_writes") is True
        )
        if not mock and not live:
            raise RuntimeError("hand is readonly or explicit field acknowledgement is missing")
        request = {
            "session_id": status["session_id"],
            "sequence": status["last_command_sequence"] + 1,
            "expected_revision": status["revision"],
            "action": args.action,
        }
        publisher.publish(String(data=json.dumps(request)))
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            for result in results:
                if (
                    result.get("session_id") == request["session_id"]
                    and result.get("action") == args.action
                    and result.get("sequence") == request["sequence"]
                    and result.get("kind") in {"accepted", "rejected"}
                ):
                    print(json.dumps(result, ensure_ascii=False))
                    return 0 if result["kind"] == "accepted" else 2
        raise RuntimeError("action result unknown; inspect status, do not automatically retry")
    except (ValueError, KeyError, RuntimeError) as exc:
        print(f"[FAIL] {exc}")
        return 2
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
