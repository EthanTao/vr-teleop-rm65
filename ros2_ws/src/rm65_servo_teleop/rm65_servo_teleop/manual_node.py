"""Local simulation control panel, with read-only measured state and leased input."""

from __future__ import annotations

import copy
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
from pathlib import Path
import secrets
import threading
import time
from urllib.parse import urlsplit

from .manual_input import ManualInput


def make_server(address, manual, state_reader, html):
    token = secrets.token_hex(32)
    page = html.replace("__RM65_TOKEN__", token).encode("utf-8")

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def _send(self, code, body, kind="application/json; charset=utf-8"):
            payload = json.dumps(body, allow_nan=False).encode() if isinstance(body, dict) else body
            self.send_response(code)
            self.send_header("Content-Type", kind)
            self.send_header("Content-Length", str(len(payload)))
            self.send_header("Cache-Control", "no-store")
            self.send_header("X-Content-Type-Options", "nosniff")
            self.send_header(
                "Content-Security-Policy",
                "default-src 'self'; script-src 'unsafe-inline'; style-src 'unsafe-inline'; connect-src 'self'; frame-ancestors 'none'; base-uri 'none'",
            )
            self.end_headers()
            try:
                self.wfile.write(payload)
            except (BrokenPipeError, ConnectionResetError):
                pass

        def _local(self):
            host = self.headers.get("Host", "")
            try:
                parsed = urlsplit("http://" + host)
                if parsed.hostname not in {"127.0.0.1", "localhost"} or parsed.username is not None:
                    return False
                origin = self.headers.get("Origin")
                return origin is None or origin == "http://" + host
            except ValueError:
                return False

        def do_GET(self):
            if not self._local():
                self._send(403, {"error": "local origin required"})
            elif self.path == "/":
                self._send(200, page, "text/html; charset=utf-8")
            elif self.path == "/api/state":
                self._send(200, state_reader())
            else:
                self._send(404, {"error": "not found"})

        def do_POST(self):
            if not self._local():
                return self._send(403, {"error": "local origin required"})
            if self.path != "/api/input":
                return self._send(404, {"error": "not found"})
            try:
                length = int(self.headers.get("Content-Length", "0"))
                if not 0 < length <= 4096 or self.headers.get_content_type() != "application/json":
                    raise ValueError("bounded JSON body required")
                self.connection.settimeout(0.5)
                value = json.loads(self.rfile.read(length))
                if (
                    not isinstance(value, dict)
                    or not isinstance(value.get("token"), str)
                    or not secrets.compare_digest(value["token"], token)
                ):
                    return self._send(403, {"error": "invalid session token"})
                manual.submit(value, time.monotonic())
                self._send(200, {"ok": True})
            except PermissionError as error:
                self._send(409, {"error": str(error)})
            except (ValueError, TypeError, TimeoutError) as error:
                self._send(400, {"error": str(error)})

    server = ThreadingHTTPServer(address, Handler)
    server.daemon_threads = True
    return server


def main(args=None):
    import numpy as np
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from rclpy.time import Time
    from ament_index_python.packages import get_package_share_directory
    from geometry_msgs.msg import PoseStamped
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Int8, String
    from tf2_ros import Buffer, TransformException, TransformListener
    import yaml

    rclpy.init(args=args)

    class ManualNode(Node):
        def __init__(self):
            super().__init__("manual_simulation_input")
            if self.declare_parameter("simulation_only", False).value is not True:
                raise ValueError("manual input is simulation-only")
            port = int(self.declare_parameter("port", 8766).value)
            own = Path(get_package_share_directory("rm65_servo_teleop"))
            config_path = self.declare_parameter("config_file", str(own / "config/teleop.yaml")).value
            config = yaml.safe_load(Path(config_path).read_text(encoding="utf-8"))
            self.manual = ManualInput(
                position_scale=float(self.declare_parameter("position_scale", config["input"]["position_scale"]).value),
                rotation_scale=float(self.declare_parameter("rotation_scale", config["input"]["rotation_scale"]).value),
            )
            self.config = config
            self.joint, self.actual, self.target = None, None, None
            self.feedback_at = self.tf_at = self.input_at = self.tracking_at = None
            self.input, self.tracking, self.servo_code = {}, {}, None
            self.snapshot, self.snapshot_at = {"mode": "simulation_manual", "ready": False}, time.monotonic()
            self.snapshot_lock = threading.Lock()
            sensor = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
            self.publisher = self.create_publisher(String, "/rm65/xr/raw", sensor)
            self.create_subscription(JointState, "/joint_states", self.joint_callback, sensor)
            self.create_subscription(PoseStamped, "/rm65/target_pose", self.target_callback, 10)
            self.create_subscription(String, "/rm65/input_status", lambda m: self.status_callback("input", m), 10)
            self.create_subscription(
                String, "/rm65/pose_tracking/status", lambda m: self.status_callback("tracking", m), 10
            )
            self.create_subscription(Int8, "/rm65/servo/status", self.servo_callback, 10)
            self.buffer = Buffer()
            self.listener = TransformListener(self.buffer, self)
            self.server = make_server(
                ("0.0.0.0", port),
                self.manual,
                self.read_state,
                (own / "config/manual_panel.html").read_text(encoding="utf-8"),
            )
            self.thread = threading.Thread(target=self.server.serve_forever, kwargs={"poll_interval": 0.1}, daemon=True)
            self.thread.start()
            self.create_timer(0.02, self.tick)
            self.get_logger().info(
                f"Simulation-only control panel: http://127.0.0.1:{port}; no physical robot interface"
            )

        def joint_callback(self, message):
            try:
                indices = [list(message.name).index(f"joint{i}") for i in range(1, 7)]
                q = [float(message.position[i]) for i in indices]
                if len(set(message.name)) != len(message.name) or not all(map(math.isfinite, q)):
                    raise ValueError("invalid measured joints")
                stamp = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
                age = (self.get_clock().now().nanoseconds - stamp) * 1e-9
                if stamp <= 0 or not -0.02 <= age <= 0.2:
                    raise ValueError("stale measured joints")
                dq = (
                    [float(message.velocity[i]) for i in indices]
                    if len(message.velocity) == len(message.name)
                    else None
                )
                if dq is not None and not all(map(math.isfinite, dq)):
                    dq = None
                self.joint = {
                    "names": [f"joint{i}" for i in range(1, 7)],
                    "position_rad": q,
                    "velocity_rad_s": dq,
                    "lower_rad": self.config["model"]["joint_lower_rad"],
                    "upper_rad": self.config["model"]["joint_upper_rad"],
                }
                self.feedback_at = time.monotonic() - max(0.0, age)
            except (ValueError, IndexError, TypeError):
                self.feedback_at = None

        def target_callback(self, message):
            if message.header.frame_id == "base_link":
                p, q = message.pose.position, message.pose.orientation
                self.target = {"position": [p.x, p.y, p.z], "quaternion_xyzw": [q.x, q.y, q.z, q.w]}

        def status_callback(self, name, message):
            try:
                value = json.loads(message.data)
                if not isinstance(value, dict):
                    raise ValueError("status must be an object")
                setattr(self, name, value)
                setattr(self, name + "_at", time.monotonic())
            except (ValueError, TypeError):
                setattr(self, name + "_at", None)

        def servo_callback(self, message):
            self.servo_code = int(message.data)

        def read_state(self):
            with self.snapshot_lock:
                value = copy.deepcopy(self.snapshot)
                age = max(0.0, time.monotonic() - self.snapshot_at)
            value["telemetry_age_s"] = age
            if age > 0.25:
                value["ready"] = False
            return value

        def tick(self):
            now = time.monotonic()
            self.tf_at = None
            try:
                transform = self.buffer.lookup_transform("base_link", "tcp", Time())
                stamp = transform.header.stamp.sec * 1_000_000_000 + transform.header.stamp.nanosec
                age = (self.get_clock().now().nanoseconds - stamp) * 1e-9
                if stamp > 0 and -0.02 <= age <= 0.2:
                    p, q = transform.transform.translation, transform.transform.rotation
                    self.actual = {"position": [p.x, p.y, p.z], "quaternion_xyzw": [q.x, q.y, q.z, q.w]}
                    self.tf_at = now - max(0.0, age)
            except TransformException:
                pass
            feedback_age = None if self.feedback_at is None else now - self.feedback_at
            tf_age = None if self.tf_at is None else now - self.tf_at
            ready = (
                feedback_age is not None
                and 0 <= feedback_age <= 0.2
                and tf_age is not None
                and tf_age <= 0.2
                and self.input_at is not None
                and 0 <= now - self.input_at <= 0.2
                and self.tracking_at is not None
                and now - self.tracking_at <= 0.7
                and not self.tracking.get("release_required", True)
                and not self.tracking.get("restart_required", True)
            )
            self.publisher.publish(String(data=self.manual.packet(now, ready=ready)))
            error, rotation_error = None, None
            if self.actual is not None and self.target is not None:
                error = float(np.linalg.norm(np.array(self.actual["position"]) - self.target["position"])) * 1000
                dot = abs(float(np.dot(self.actual["quaternion_xyzw"], self.target["quaternion_xyzw"])))
                rotation_error = math.degrees(2 * math.acos(min(1.0, dot)))
            snapshot = {
                "mode": "simulation_manual",
                "ready": bool(ready),
                "feedback_age_s": feedback_age,
                "tf_age_s": tf_age,
                "input": self.input,
                "tracking": self.tracking,
                "servo_code": self.servo_code,
                "joint": self.joint,
                "actual_pose": self.actual,
                "target_pose": self.target,
                "pose_error_mm": error,
                "rotation_error_deg": rotation_error,
                "control": self.manual.status(),
            }
            with self.snapshot_lock:
                self.snapshot, self.snapshot_at = snapshot, now

        def close_panel(self):
            self.server.shutdown()
            self.server.server_close()
            self.thread.join(timeout=1)

    node = None
    try:
        node = ManualNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except _rclpy.RCLError:
        if rclpy.ok():
            raise
    finally:
        if node is not None:
            node.close_panel()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
