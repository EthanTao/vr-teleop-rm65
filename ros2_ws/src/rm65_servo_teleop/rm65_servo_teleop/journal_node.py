"""Bounded asynchronous raw ROS journal; diagnostic evidence, not a trained dataset."""

from __future__ import annotations

from datetime import datetime, timezone
import json
import math
from pathlib import Path
import queue
import threading
import time
import uuid

from .peripheral_clock import clock_identity

# A BEST_EFFORT reader accepts both sensor-style and RELIABLE writers. Requiring
# RELIABLE here would silently reject the native/mock XR publisher entirely.
# Commands and evidence/status topics keep RELIABLE readers for their known
# RELIABLE publishers. DDS/network loss is not inferred from these row counts.
TOPIC_QOS = {
    "/clock": ("best_effort", 128),
    "/joint_states": ("best_effort", 128),
    "/rm65/transport/joint_states": ("best_effort", 128),
    "/rm65/driver/joint_states": ("best_effort", 128),
    "/rm65/transport/joint_commands": ("reliable", 128),
    "/rm65_arm_controller/joint_trajectory": ("reliable", 128),
    "/rm65_arm_controller/controller_state": ("best_effort", 128),
    "/rm65/xr/raw": ("best_effort", 128),
    "/rm65/input_status": ("reliable", 64),
    "/rm65/target_pose": ("reliable", 128),
    "/rm65/pose_tracking/status": ("reliable", 64),
    "/rm65/transport/status": ("reliable", 128),
    "/rm65/driver/feedback_snapshot": ("best_effort", 128),
    "/rm65/driver/servo_send_result": ("reliable", 128),
    "/rm65/enable": ("reliable", 64),
    "/rm65/servo/status": ("reliable", 64),
    "/rm65/hand/command": ("reliable", 64),
    "/rm65/hand/events": ("reliable", 128),
    "/rm65/hand/status": ("reliable", 64),
    "/rm65/camera/status": ("reliable", 32),
}

COMMON_REQUIRED_TOPICS = frozenset(
    {
        "/joint_states",
        "/rm65/xr/raw",
        "/rm65/input_status",
        "/rm65/target_pose",
        "/rm65/pose_tracking/status",
        "/rm65/enable",
        "/rm65/servo/status",
        "/rm65_arm_controller/joint_trajectory",
        "/rm65_arm_controller/controller_state",
    }
)
HARDWARE_REQUIRED_TOPICS = frozenset(
    {
        "/rm65/transport/joint_states",
        "/rm65/transport/joint_commands",
        "/rm65/transport/status",
        "/rm65/driver/feedback_snapshot",
    }
)


def required_topics_for_mode(mode: str) -> frozenset[str]:
    if mode == "sim":
        return COMMON_REQUIRED_TOPICS
    if mode == "hardware-dry-run":
        # Dry Run never invokes the movement API, so a send result is neither
        # expected nor accepted as a substitute for a genuine feedback snapshot.
        return COMMON_REQUIRED_TOPICS | HARDWARE_REQUIRED_TOPICS
    if mode == "hardware":
        return COMMON_REQUIRED_TOPICS | HARDWARE_REQUIRED_TOPICS | {"/rm65/driver/servo_send_result"}
    raise ValueError("journal mode must be sim, hardware-dry-run, or hardware")


def normalize_diagnostic_values(topic: str, value):
    """Encode unknown optional ROS fields without replacing them with zero.

    Paths are JSON pointers relative to the complete record. Only explicitly
    optional state diagnostics are exempt from the control-data quality gate.
    Command trajectories, measured q/qdot and target poses remain strict.
    """
    issues = []
    joint_state_topics = {
        "/joint_states",
        "/rm65/transport/joint_states",
        "/rm65/driver/joint_states",
        "/rm65/transport/joint_commands",
    }

    def optional_diagnostic(path):
        if topic in joint_state_topics and path and path[0] == "effort":
            return True
        return (
            topic == "/rm65_arm_controller/controller_state"
            and len(path) >= 2
            and path[0] in {"desired", "actual", "error", "reference", "feedback", "output"}
            and path[1] in {"effort", "accelerations"}
        )

    def visit(item, path):
        if isinstance(item, float) and not math.isfinite(item):
            kind = "NaN" if math.isnan(item) else ("+Inf" if item > 0 else "-Inf")
            pointer = "/value" + "".join("/" + str(part).replace("~", "~0").replace("/", "~1") for part in path)
            issues.append(
                {
                    "path": pointer,
                    "kind": kind,
                    "classification": "optional_diagnostic" if optional_diagnostic(path) else "invalid_control",
                }
            )
            return None
        if isinstance(item, dict):
            return {key: visit(child, (*path, key)) for key, child in item.items()}
        if isinstance(item, (list, tuple)):
            return [visit(child, (*path, index)) for index, child in enumerate(item)]
        return item

    normalized = visit(value, ())
    return normalized, issues


def replace_metadata_file(temporary: Path, destination: Path) -> None:
    """Keep atomic replacement; tolerate short Windows sharing/access conflicts.

    Called only at session open/close, outside the control loop. Other errors
    propagate immediately; persistent failures retain both files for diagnosis.
    """
    for attempt in range(6):
        try:
            temporary.replace(destination)
            return
        except PermissionError as exc:
            if getattr(exc, "winerror", None) not in {5, 32, 33} or attempt == 5:
                raise
            time.sleep(0.01 * 2**attempt)


class JournalWriter:
    def __init__(
        self,
        root: str | Path,
        *,
        mode: str,
        capacity: int = 512,
        simulation_metadata: dict | None = None,
        hand_mode: str = "off",
    ):
        if capacity < 1:
            raise ValueError("journal capacity must be positive")
        self.required_topics = required_topics_for_mode(mode)
        if hand_mode not in {"off", "mock"} or (hand_mode == "mock" and mode != "sim"):
            raise ValueError("mock hand records require sim mode")
        self.hand_mode = hand_mode
        if hand_mode == "mock":
            self.required_topics = self.required_topics | {"/rm65/hand/status", "/rm65/hand/events"}
        if simulation_metadata is not None:
            if mode != "sim" or simulation_metadata.get("simulator") != "mujoco":
                raise ValueError("MuJoCo provenance is only valid for simulation")
            self.required_topics = self.required_topics | {"/clock"}
        session = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ") + "_" + uuid.uuid4().hex[:8]
        self.path = Path(root) / session
        self.path.mkdir(parents=True, exist_ok=False)
        self.mode = mode
        self.simulation_metadata = simulation_metadata
        if simulation_metadata is not None:
            (self.path / "simulation_model.json").write_text(
                json.dumps(simulation_metadata, indent=2), encoding="utf-8"
            )
        self.dropped = 0
        self.error = None
        self.written = 0
        self.invalid_rows = 0
        self.optional_nonfinite_fields = 0
        self._lock = threading.Lock()
        self._topic_counts = {topic: {"received": 0, "queued": 0, "written": 0, "dropped": 0} for topic in TOPIC_QOS}
        self._queue = queue.Queue(maxsize=capacity)
        self._stop = threading.Event()
        self._closed = False
        self._close_lock = threading.Lock()
        self._metadata_saved = False
        self._metadata(False)
        self._thread = threading.Thread(target=self._work, name="rm65-journal", daemon=True)
        self._thread.start()

    def _metadata(self, complete: bool):
        with self._lock:
            topic_counts = {topic: dict(counts) for topic, counts in sorted(self._topic_counts.items())}
            missing = sorted(topic for topic in self.required_topics if topic_counts[topic]["written"] == 0)
            value = {
                "format": "rm65_servo_raw_v1",
                "clock_identity": clock_identity(),
                "mode": self.mode,
                "hand_mode": self.hand_mode,
                "hand_contact_physics_simulated": False,
                "simulator": (
                    self.simulation_metadata.get("simulator")
                    if self.simulation_metadata
                    else ("mock" if self.mode == "sim" else None)
                ),
                "simulation_model": "simulation_model.json" if self.simulation_metadata else None,
                "complete": complete and not missing and self.invalid_rows == 0,
                "coverage_complete": not missing,
                "quality_valid": self.invalid_rows == 0,
                "invalid_rows": self.invalid_rows,
                "optional_nonfinite_fields": self.optional_nonfinite_fields,
                "nonfinite_encoding": "null_with_original_json_pointer_and_NaN_or_signed_Inf_kind",
                "integrity_scope": "writer_delivery_required_topic_presence_and_finite_control_fields",
                "required_topics": sorted(self.required_topics),
                "missing_required_topics": missing,
                "topic_counts": topic_counts,
                "subscription_qos": {
                    topic: {"reliability": policy, "depth": depth, "durability": "volatile"}
                    for topic, (policy, depth) in sorted(TOPIC_QOS.items())
                },
                "network_loss_checked": False,
                "rows": self.written,
                "dropped": self.dropped,
                "error": self.error,
                "observation_source": "recorded_topic",
                "robot_execution_requires_feedback": True,
            }
        temporary = self.path / "session.json.tmp"
        temporary.write_text(
            json.dumps(value, indent=2),
            encoding="utf-8",
        )
        replace_metadata_file(temporary, self.path / "session.json")

    def append(self, topic: str, value, *, monotonic_s: float | None = None) -> bool:
        row = {
            "type": "sample",
            "topic": topic,
            "received_monotonic_s": time.monotonic() if monotonic_s is None else monotonic_s,
            "value": value,
        }
        with self._lock:
            if self._closed:
                return False  # The closed session and its final metadata are immutable.
            counts = self._topic_counts.setdefault(topic, {"received": 0, "queued": 0, "written": 0, "dropped": 0})
            counts["received"] += 1
            if self.error is not None:
                self.dropped += 1
                counts["dropped"] += 1
                return False
            try:
                self._queue.put_nowait(row)
                counts["queued"] += 1
                return True
            except queue.Full:
                self.dropped += 1
                counts["dropped"] += 1
                return False

    def fail(self, reason: str) -> None:
        with self._lock:
            self.error = self.error or reason

    def _work(self):
        current_topic = None
        try:
            with (self.path / "records.jsonl").open("x", encoding="utf-8") as stream:
                last_flush = time.monotonic()
                while not self._stop.is_set() or not self._queue.empty():
                    try:
                        row = self._queue.get(timeout=0.1)
                    except queue.Empty:
                        continue
                    current_topic = row["topic"]
                    row["value"], issues = normalize_diagnostic_values(current_topic, row["value"])
                    invalid = [issue for issue in issues if issue["classification"] == "invalid_control"]
                    row["quality"] = {"valid": not invalid, "nonfinite_fields": issues}
                    stream.write(json.dumps(row, allow_nan=False, separators=(",", ":")) + "\n")
                    with self._lock:
                        self.written += 1
                        self._topic_counts[row["topic"]]["written"] += 1
                        self.optional_nonfinite_fields += len(issues) - len(invalid)
                        if invalid:
                            self.invalid_rows += 1
                            first = invalid[0]
                            self.error = self.error or (
                                f"invalid control data on {current_topic} at {first['path']} ({first['kind']})"
                            )
                    if time.monotonic() - last_flush >= 0.5:
                        stream.flush()
                        last_flush = time.monotonic()
                stream.write(json.dumps({"type": "log_end", "dropped": self.dropped, "error": self.error}) + "\n")
                stream.flush()
        except Exception as exc:
            self.fail(f"journal write failed on {current_topic}: {exc}")

    def close(self):
        # Serialize finalization and permit retry after a metadata save failure.
        # Appends stay disabled as soon as the first close begins.
        with self._close_lock:
            if self._metadata_saved:
                return
            with self._lock:
                self._closed = True
            self._stop.set()
            self._thread.join(timeout=3.0)
            if self._thread.is_alive():
                self.fail("journal did not drain within three seconds")
            self._metadata(self.error is None and self.dropped == 0 and not self._thread.is_alive())
            self._metadata_saved = True


def main(args=None) -> None:
    import rclpy
    from control_msgs.msg import JointTrajectoryControllerState
    from geometry_msgs.msg import PoseStamped
    from rclpy.node import Node
    from rclpy.executors import ExternalShutdownException
    from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from rosidl_runtime_py.convert import message_to_ordereddict
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Bool, Int8, String
    from trajectory_msgs.msg import JointTrajectory

    class JournalNode(Node):
        def __init__(self):
            super().__init__("rm65_journal")
            root = self.declare_parameter("record_root", "/data/experiments").value
            mode = self.declare_parameter("mode", "sim").value
            capacity = int(self.declare_parameter("journal_capacity", 512).value)
            metadata_path = str(self.declare_parameter("simulation_metadata", "").value)
            metadata = json.loads(Path(metadata_path).read_text(encoding="utf-8")) if metadata_path else None
            hand_mode = str(self.declare_parameter("hand_mode", "off").value)
            self.writer = JournalWriter(
                root, mode=mode, capacity=capacity, simulation_metadata=metadata, hand_mode=hand_mode
            )
            self.subscriptions_kept = []
            types = {
                "/clock": Clock,
                "/joint_states": JointState,
                "/rm65/transport/joint_states": JointState,
                "/rm65/driver/joint_states": JointState,
                "/rm65/transport/joint_commands": JointState,
                "/rm65_arm_controller/joint_trajectory": JointTrajectory,
                "/rm65_arm_controller/controller_state": JointTrajectoryControllerState,
                "/rm65/xr/raw": String,
                "/rm65/input_status": String,
                "/rm65/target_pose": PoseStamped,
                "/rm65/pose_tracking/status": String,
                "/rm65/transport/status": String,
                "/rm65/driver/feedback_snapshot": String,
                "/rm65/driver/servo_send_result": String,
                "/rm65/enable": Bool,
                "/rm65/servo/status": Int8,
                "/rm65/hand/command": String,
                "/rm65/hand/events": String,
                "/rm65/hand/status": String,
                "/rm65/camera/status": String,
            }
            for topic, message_type in types.items():
                policy, depth = TOPIC_QOS[topic]
                qos = QoSProfile(
                    depth=depth,
                    reliability=(
                        ReliabilityPolicy.BEST_EFFORT if policy == "best_effort" else ReliabilityPolicy.RELIABLE
                    ),
                    history=HistoryPolicy.KEEP_LAST,
                    durability=DurabilityPolicy.VOLATILE,
                )
                self.subscriptions_kept.append(
                    self.create_subscription(
                        message_type,
                        topic,
                        lambda msg, name=topic: self._record(name, msg),
                        qos,
                    )
                )
            self.get_logger().info(f"Raw Servo journal: {self.writer.path}")
            self.health_timer = self.create_timer(0.1, self._check_writer)

        def _record(self, topic, message):
            try:
                value = message_to_ordereddict(message)
            except Exception as exc:
                self.writer.fail(f"message conversion failed on {topic}: {exc}")
                raise RuntimeError(self.writer.error) from exc
            self.writer.append(topic, value)

        def _check_writer(self):
            # Stop the collection graph if its evidence can no longer be saved.
            # The launch file treats this process as critical; device watchdogs
            # retain responsibility for an actual confirmed physical stop.
            if self.writer.error is not None or self.writer.dropped:
                raise RuntimeError(f"journal incomplete: dropped={self.writer.dropped}, error={self.writer.error}")

    rclpy.init(args=args)
    node = JournalNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except _rclpy.RCLError:
        if rclpy.ok():
            raise
    finally:
        try:
            node.writer.close()
        finally:
            try:
                node.destroy_node()
            finally:
                if rclpy.ok():
                    rclpy.shutdown()
