import json
import socket
import time

import meshcat.transformations as tf
import numpy as np

JOINT_UNIT_SCALE = 1000.0
POSE_POS_SCALE = 1_000_000.0
POSE_ROT_SCALE = 1000.0
POSE_QUAT_SCALE = 1_000_000.0


class RealmanRM65Interface:
    """TCP/JSON interface for Realman RM65 teleoperation."""

    def __init__(
        self,
        host: str = "192.168.10.18",
        port: int = 8080,
        move_v: int = 20,
        move_r: int = 80,
        joint_signs: list[float] | None = None,
        joint_offsets_rad: list[float] | None = None,
        max_joint_step_rad: float = 0.08,
        min_joint_command_rad: float = 0.003,
        min_pose_command_m: float = 0.0005,
        min_rot_command_rad: float = 0.001,
        command_wait_s: float = 0.02,
        state_wait_s: float = 0.15,
        workspace_min_xyz_m: list[float] | None = None,
        workspace_max_xyz_m: list[float] | None = None,
    ):
        self.host = host
        self.port = port
        self.move_v = int(move_v)
        self.move_r = int(move_r)
        self.joint_signs = np.ones(6, dtype=float) if joint_signs is None else np.array(joint_signs, dtype=float)
        self.joint_offsets_rad = (
            np.zeros(6, dtype=float) if joint_offsets_rad is None else np.array(joint_offsets_rad, dtype=float)
        )
        self.max_joint_step_rad = float(max_joint_step_rad)
        self.min_joint_command_rad = float(min_joint_command_rad)
        self.min_pose_command_m = float(min_pose_command_m)
        self.min_rot_command_rad = float(min_rot_command_rad)
        self.command_wait_s = float(command_wait_s)
        self.state_wait_s = float(state_wait_s)
        self.workspace_min_xyz_m = np.array(
            [-1.0, -1.0, -1.0] if workspace_min_xyz_m is None else workspace_min_xyz_m, dtype=float
        )
        self.workspace_max_xyz_m = np.array(
            [1.0, 1.0, 1.0] if workspace_max_xyz_m is None else workspace_max_xyz_m, dtype=float
        )
        if self.joint_signs.shape != (6,) or self.joint_offsets_rad.shape != (6,):
            raise ValueError("joint_signs and joint_offsets_rad must have shape (6,)")
        if not np.all(np.isfinite(self.joint_signs)) or np.any(np.abs(self.joint_signs) < 1e-12):
            raise ValueError("joint_signs must be finite and non-zero")
        if not np.all(np.isfinite(self.joint_offsets_rad)):
            raise ValueError("joint_offsets_rad must be finite")
        if self.command_wait_s < 0 or self.state_wait_s < 0:
            raise ValueError("wait durations must be non-negative")
        if (
            self.workspace_min_xyz_m.shape != (3,)
            or self.workspace_max_xyz_m.shape != (3,)
            or not np.all(np.isfinite(self.workspace_min_xyz_m))
            or not np.all(np.isfinite(self.workspace_max_xyz_m))
            or np.any(self.workspace_min_xyz_m >= self.workspace_max_xyz_m)
        ):
            raise ValueError("workspace xyz limits must be finite arrays with min < max")
        self.sock: socket.socket | None = None
        self._recv_buffer = b""
        self._last_joint_read_t = 0.0
        self._last_pose_read_t = 0.0
        self._measured_joint_rad = np.zeros(6, dtype=float)
        self._measured_pose6: list[int] | None = None
        self._last_commanded_pose6: list[int] | None = None
        self._has_joint_measurement = False
        self._has_pose_measurement = False
        self.fault_latched = False
        self.fault_reason: str | None = None
        self._unacknowledged_movel_count = 0
        self.motion_diagnostics_enabled = False
        self.motion_diagnostic = None

    def connect(self) -> None:
        self.sock = socket.create_connection((self.host, self.port), timeout=3.0)
        self.sock.settimeout(0.2)
        pose6 = self.get_pose6(force=True)
        if pose6 is None:
            self.close()
            raise RuntimeError("RM65 returned no initial TCP pose")
        self._last_commanded_pose6 = pose6[:]

    def close(self) -> None:
        if self.sock is not None:
            try:
                self.sock.close()
            except Exception:
                pass
            self.sock = None
        self._recv_buffer = b""

    def _require_sock(self) -> socket.socket:
        if self.sock is None:
            raise RuntimeError("Realman socket not connected")
        return self.sock

    def send_json(self, payload: dict, wait_s: float) -> list[dict]:
        sock = self._require_sock()
        if not isinstance(payload, dict):
            raise TypeError("payload must be a dictionary")
        if wait_s < 0:
            raise ValueError("wait_s must be non-negative")
        data = (json.dumps(payload, ensure_ascii=False) + "\r\n").encode("utf-8")
        self._sendall_observed(sock, data, payload)
        end_t = time.monotonic() + wait_s
        buf = self._recv_buffer
        previous_timeout = sock.gettimeout()
        try:
            while True:
                remaining = end_t - time.monotonic()
                if remaining <= 0.0:
                    break
                sock.settimeout(remaining)
                chunk = sock.recv(4096)
                if not chunk:
                    raise ConnectionError("RM65 closed the TCP connection")
                buf += chunk
                if self._buffer_contains_response(buf, str(payload.get("command", ""))):
                    break
        except socket.timeout:
            pass
        finally:
            sock.settimeout(previous_timeout)
        lines = buf.splitlines(keepends=True)
        complete_lines = []
        self._recv_buffer = b""
        for line in lines:
            if line.endswith((b"\n", b"\r")):
                complete_lines.append(line)
            else:
                self._recv_buffer += line
        msgs: list[dict] = []
        for line in b"".join(complete_lines).decode("utf-8", "ignore").splitlines():
            line = line.strip()
            if not line:
                continue
            try:
                obj = json.loads(line)
                msgs.append(obj if isinstance(obj, dict) else {"raw": line})
            except json.JSONDecodeError:
                msgs.append({"raw": line})
        return msgs

    def send_only(self, payload: dict) -> None:
        """Send one CRLF-framed JSON command without waiting for a reply."""
        sock = self._require_sock()
        if not isinstance(payload, dict):
            raise TypeError("payload must be a dictionary")
        data = (json.dumps(payload, ensure_ascii=False) + "\r\n").encode("utf-8")
        self._sendall_observed(sock, data, payload)

    def _sendall_observed(self, sock, data: bytes, payload: dict) -> None:
        if not self.motion_diagnostics_enabled or payload.get("command") not in {
            "movep_follow",
            "movel",
            "movej_follow",
        }:
            sock.sendall(data)
            return
        record = {"started_monotonic": time.monotonic(), "wire_json": data.decode("utf-8"), "sendall_completed": False}
        self.motion_diagnostic = record
        try:
            sock.sendall(data)
            record["sendall_completed"] = True
        finally:
            record["duration_s"] = time.monotonic() - record["started_monotonic"]

    def get_arm_software_info(self) -> dict | None:
        messages = self.send_json(
            {"command": "get_arm_software_info"},
            wait_s=max(0.2, self.state_wait_s),
        )
        if self._response_error(messages) is not None:
            return None
        for message in reversed(messages):
            if not isinstance(message, dict):
                continue
            command = message.get("command")
            product = message.get("Product_version", message.get("product_version"))
            if command in {"get_arm_software_info", "arm_software_info"} and isinstance(product, str):
                return message
        return None

    @staticmethod
    def _buffer_contains_response(buffer: bytes, command: str) -> bool:
        """Check complete JSON lines for the response associated with command."""
        for raw_line in buffer.splitlines(keepends=True):
            if not raw_line.endswith((b"\n", b"\r")):
                continue
            try:
                message = json.loads(raw_line.decode("utf-8"))
            except (UnicodeDecodeError, json.JSONDecodeError):
                continue
            if not isinstance(message, dict):
                continue
            if command == "get_current_arm_state" and message.get("state") == "current_arm_state":
                return True
            if message.get("command") == command:
                return True
        return False

    @staticmethod
    def _controller_joint_to_rad(joint_mdeg: list[int], signs: np.ndarray, offsets_rad: np.ndarray) -> np.ndarray:
        if len(joint_mdeg) < 6 or signs.shape != (6,) or offsets_rad.shape != (6,):
            raise ValueError("joint state and calibration arrays must contain six values")
        deg = np.array([float(v) for v in joint_mdeg[:6]], dtype=float) / JOINT_UNIT_SCALE
        return np.deg2rad(deg) * signs + offsets_rad

    @staticmethod
    def _rad_to_controller_joint(q_rad: np.ndarray, signs: np.ndarray, offsets_rad: np.ndarray) -> list[int]:
        deg = np.rad2deg((q_rad - offsets_rad) / signs)
        return [int(round(v * JOINT_UNIT_SCALE)) for v in deg]

    @staticmethod
    def pose6_to_xyz_quat(pose6: list[int]) -> tuple[np.ndarray, np.ndarray]:
        if len(pose6) < 6:
            raise ValueError("pose6 must contain six values")
        xyz = np.array([float(pose6[0]), float(pose6[1]), float(pose6[2])], dtype=float) / POSE_POS_SCALE
        rotvec = np.array([float(pose6[3]), float(pose6[4]), float(pose6[5])], dtype=float) / POSE_ROT_SCALE
        angle = float(np.linalg.norm(rotvec))
        if angle < 1e-9:
            quat = np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
        else:
            quat = tf.quaternion_about_axis(angle, rotvec / angle)
        return xyz, quat

    @staticmethod
    def xyz_quat_to_pose6(xyz: np.ndarray, quat: np.ndarray) -> list[int]:
        xyz = np.asarray(xyz, dtype=float)
        quat = np.asarray(quat, dtype=float)
        if xyz.shape != (3,) or quat.shape != (4,) or not np.all(np.isfinite(xyz)) or not np.all(np.isfinite(quat)):
            raise ValueError("xyz must have shape (3,) and quat shape (4,), both finite")
        if np.linalg.norm(quat) < 1e-12:
            raise ValueError("quat norm must be non-zero")
        quat_n = quat / max(float(np.linalg.norm(quat)), 1e-12)
        if quat_n[0] < 0.0:
            quat_n = -quat_n
        angle = 2.0 * np.arccos(float(np.clip(quat_n[0], -1.0, 1.0)))
        if angle < 1e-9:
            rotvec = np.zeros(3, dtype=float)
        else:
            s = np.sqrt(max(0.0, 1.0 - quat_n[0] * quat_n[0]))
            axis = quat_n[1:] / max(s, 1e-12)
            rotvec = axis * angle
        return [
            int(round(float(xyz[0]) * POSE_POS_SCALE)),
            int(round(float(xyz[1]) * POSE_POS_SCALE)),
            int(round(float(xyz[2]) * POSE_POS_SCALE)),
            int(round(float(rotvec[0]) * POSE_ROT_SCALE)),
            int(round(float(rotvec[1]) * POSE_ROT_SCALE)),
            int(round(float(rotvec[2]) * POSE_ROT_SCALE)),
        ]

    def _refresh_state(self, force: bool, required: str) -> bool:
        now = time.monotonic()
        has_required = self._has_joint_measurement if required == "joint" else self._has_pose_measurement
        last_required_read_t = self._last_joint_read_t if required == "joint" else self._last_pose_read_t
        if not force and has_required and (now - last_required_read_t) < self.state_wait_s:
            return True
        resp = self.send_json({"command": "get_current_arm_state"}, wait_s=self.state_wait_s)
        response_error = self._response_error(resp)
        if response_error is not None:
            self._trip_fault(f"controller reported an asynchronous error: {response_error}")
            return False
        for item in reversed(resp):
            if not isinstance(item, dict) or item.get("state") != "current_arm_state":
                continue
            arm_state = item.get("arm_state")
            if not isinstance(arm_state, dict):
                continue
            joint = arm_state.get("joint")
            pose = arm_state.get("pose")
            updated_joint = False
            updated_pose = False
            if isinstance(joint, list) and len(joint) >= 6:
                self._measured_joint_rad = self._controller_joint_to_rad(
                    joint, self.joint_signs, self.joint_offsets_rad
                )
                self._has_joint_measurement = True
                self._last_joint_read_t = now
                updated_joint = True
            if isinstance(pose, list) and len(pose) >= 6:
                self._measured_pose6 = [int(round(float(v))) for v in pose[:6]]
                self._has_pose_measurement = True
                self._last_pose_read_t = now
                updated_pose = True
            if (required == "joint" and updated_joint) or (required == "pose" and updated_pose):
                return True
        return False

    def get_joint_positions(self, force: bool = False) -> np.ndarray | None:
        if not self._refresh_state(force or not self._has_joint_measurement, required="joint"):
            return None
        if not self._has_joint_measurement:
            return None
        return self._measured_joint_rad.copy()

    def get_pose6(self, force: bool = False) -> list[int] | None:
        if not self._refresh_state(force or not self._has_pose_measurement, required="pose"):
            return None
        return self._measured_pose6[:] if self._has_pose_measurement and self._measured_pose6 is not None else None

    def get_last_commanded_pose6(self) -> list[int] | None:
        return self._last_commanded_pose6[:] if self._last_commanded_pose6 is not None else None

    def sync_command_reference(self, pose6: list[int]) -> None:
        if len(pose6) < 6:
            raise ValueError("pose6 must contain six values")
        self._last_commanded_pose6 = [int(round(float(value))) for value in pose6[:6]]

    def _pose_is_within_bounds(self, xyz: np.ndarray) -> tuple[bool, str]:
        xyz = np.asarray(xyz, dtype=float)
        if xyz.shape != (3,) or not np.all(np.isfinite(xyz)):
            return False, "target TCP position is non-finite"
        if np.any(xyz < self.workspace_min_xyz_m) or np.any(xyz > self.workspace_max_xyz_m):
            return False, f"target TCP xyz {np.round(xyz, 3).tolist()} outside configured bounds"
        return True, ""

    @staticmethod
    def _response_error(messages: list[dict]) -> str | None:
        for message in messages:
            if not isinstance(message, dict):
                continue
            for flag in ("receive_state", "success", "result"):
                if message.get(flag) is False:
                    return str(message.get("message") or message.get("msg") or message)
            state = str(message.get("state", "")).lower()
            if state in {"error", "err", "failed", "fail"}:
                return str(message.get("message") or message.get("msg") or message)
            code = message.get("code", message.get("err_code", message.get("error_code")))
            if isinstance(code, (int, float)) and int(code) != 0:
                return str(message.get("message") or message.get("msg") or message)
        return None

    @classmethod
    def _command_acknowledged(cls, messages: list[dict], command: str) -> bool:
        """Return True only when the controller explicitly acknowledges command."""
        if cls._response_error(messages) is not None:
            return False
        for message in messages:
            if not isinstance(message, dict) or message.get("command") != command:
                continue
            ack_fields = ("receive_state", "success", "result")
            if command == "set_arm_stop":
                ack_fields += ("arm_stop",)
            elif command == "set_arm_slow_stop":
                ack_fields += ("arm_slow_stop",)
            if any(message.get(flag) is True for flag in ack_fields):
                return True
        return False

    def _trip_fault(self, reason: str) -> None:
        if not self.fault_latched:
            self.fault_latched = True
            self.fault_reason = reason
            print(f"[SAFETY STOP] {reason}")
        try:
            self.stop()
        except Exception as exc:
            print(f"[WARN] failed to send safety stop: {exc}")

    def clear_fault(self) -> None:
        self.fault_latched = False
        self.fault_reason = None

    def latch_fault(self, reason: str) -> None:
        """Latch an externally detected safety fault and request an arm stop."""
        self._trip_fault(reason)

    def movel_xyz_quat(self, xyz: np.ndarray, quat: np.ndarray, active: bool = True) -> bool:
        if not active or self.fault_latched:
            return False
        within_bounds, reason = self._pose_is_within_bounds(xyz)
        if not within_bounds:
            self._trip_fault(reason)
            return False
        # Keep the last command separate from measured feedback so deadband and
        # tracking-error checks use the appropriate reference.
        cur_pose6 = self._last_commanded_pose6[:] if self._last_commanded_pose6 else None
        if cur_pose6 is None:
            cur_pose6 = self.get_pose6(force=True)
        if cur_pose6 is None:
            return False
        target_xyz, target_quat = xyz, quat
        cur_xyz, cur_quat = self.pose6_to_xyz_quat(cur_pose6)
        dpos = float(np.linalg.norm(target_xyz - cur_xyz))
        q_inc = tf.quaternion_multiply(target_quat, tf.quaternion_conjugate(cur_quat))
        if q_inc[0] < 0.0:
            q_inc = -q_inc
        drot = 2.0 * np.arccos(float(np.clip(q_inc[0], -1.0, 1.0)))
        if dpos < self.min_pose_command_m and drot < self.min_rot_command_rad:
            return False
        pose6 = self.xyz_quat_to_pose6(target_xyz, target_quat)
        response = self.send_json(
            {"command": "movel", "pose": pose6, "v": self.move_v, "r": self.move_r}, wait_s=self.command_wait_s
        )
        error = self._response_error(response)
        if error is not None:
            self._trip_fault(f"controller rejected movel: {error}")
            return False
        if self._command_acknowledged(response, "movel"):
            self._unacknowledged_movel_count = 0
        else:
            self._unacknowledged_movel_count += 1
            if self._unacknowledged_movel_count == 1:
                print("[INFO] movel uses asynchronous confirmation; relying on RM65 feedback watchdog")
        self._last_commanded_pose6 = pose6
        return True

    def send_movep_follow_xyz_quat(self, xyz: np.ndarray, quat: np.ndarray, active: bool = True) -> bool:
        """Validate and send one absolute movep_follow target without a TCP receive."""
        if not active or self.fault_latched:
            return False
        xyz = np.asarray(xyz, dtype=float)
        quat = np.asarray(quat, dtype=float)
        within_bounds, reason = self._pose_is_within_bounds(xyz)
        if not within_bounds:
            self._trip_fault(reason)
            return False
        if quat.shape != (4,) or not np.all(np.isfinite(quat)):
            self._trip_fault("target TCP quaternion is invalid")
            return False
        norm = float(np.linalg.norm(quat))
        if norm < 1e-12:
            self._trip_fault("target TCP quaternion is zero")
            return False
        quat = quat / norm
        if quat[0] < 0.0:
            quat = -quat
        payload = {
            "command": "movep_follow",
            "pose_quat": [
                *(int(round(float(value) * POSE_POS_SCALE)) for value in xyz),
                *(int(round(float(value) * POSE_QUAT_SCALE)) for value in quat),
            ],
        }
        try:
            self.send_only(payload)
        except (OSError, ConnectionError, RuntimeError) as exc:
            self._trip_fault(f"RM65 movep_follow transport failed: {exc}")
            return False
        self._last_commanded_pose6 = self.xyz_quat_to_pose6(xyz, quat)
        return True

    def send_movej_follow_joint_rad(self, joint_rad: np.ndarray, active: bool = True) -> bool:
        """Send one official ``movej_follow`` target using controller joint units."""
        if not active or self.fault_latched:
            return False
        try:
            joint_rad = np.asarray(joint_rad, dtype=float)
        except (TypeError, ValueError) as exc:
            self._trip_fault(f"movej_follow target is not numeric: {exc}")
            return False
        if joint_rad.shape != (6,) or not np.all(np.isfinite(joint_rad)):
            self._trip_fault("movej_follow target must be a finite six-joint array")
            return False
        payload = {
            "command": "movej_follow",
            "joint": self._rad_to_controller_joint(
                joint_rad,
                self.joint_signs,
                self.joint_offsets_rad,
            ),
        }
        try:
            self.send_only(payload)
        except (OSError, ConnectionError, RuntimeError) as exc:
            self._trip_fault(f"RM65 movej_follow transport failed: {exc}")
            return False
        return True

    def stop(self) -> bool:
        response = self.send_json({"command": "set_arm_stop"}, wait_s=max(0.02, self.command_wait_s))
        error = self._response_error(response)
        if error is not None:
            raise RuntimeError(f"controller rejected set_arm_stop: {error}")
        acknowledged = self._command_acknowledged(response, "set_arm_stop")
        if not acknowledged:
            print("[WARN] set_arm_stop was sent but the controller did not confirm it")
        return acknowledged

    def slow_stop(self) -> bool:
        response = self.send_json(
            {"command": "set_arm_slow_stop"},
            wait_s=max(0.02, self.command_wait_s),
        )
        error = self._response_error(response)
        if error is not None:
            raise RuntimeError(f"controller rejected set_arm_slow_stop: {error}")
        acknowledged = self._command_acknowledged(response, "set_arm_slow_stop")
        if not acknowledged:
            print("[WARN] set_arm_slow_stop was sent but the controller did not confirm it")
        return acknowledged
