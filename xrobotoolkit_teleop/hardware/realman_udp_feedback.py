"""Read-only parser and latest-only receiver for RM65 realtime UDP feedback."""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
import numbers
import socket
import threading
import time

import numpy as np


def _finite_real(value: object, name: str) -> float:
    if isinstance(value, (bool, np.bool_)) or not isinstance(value, numbers.Real):
        raise ValueError(f"{name} must be a finite number")
    try:
        result = float(value)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError(f"{name} must be a finite number") from exc
    if not math.isfinite(result):
        raise ValueError(f"{name} must be a finite number")
    return result


def _readonly_vector(value: object, length: int, name: str) -> np.ndarray:
    try:
        array = np.asarray(value, dtype=float)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError(f"{name} must be a numeric vector") from exc
    if array.shape != (length,) or not np.all(np.isfinite(array)):
        raise ValueError(f"{name} must be a finite vector with shape ({length},)")
    result = array.copy()
    result.setflags(write=False)
    return result


@dataclass(frozen=True)
class ArmFeedback:
    """One validated realtime RM65 state frame in SI units."""

    received_monotonic_s: float
    joint_rad: np.ndarray
    tcp_xyz_m: np.ndarray
    tcp_quat_wxyz: np.ndarray
    arm_error_codes: tuple[int, ...]
    joint_speed_rad_s: np.ndarray | None = None

    def __post_init__(self) -> None:
        object.__setattr__(
            self,
            "received_monotonic_s",
            _finite_real(self.received_monotonic_s, "received_monotonic_s"),
        )
        object.__setattr__(self, "joint_rad", _readonly_vector(self.joint_rad, 6, "joint_rad"))
        object.__setattr__(self, "tcp_xyz_m", _readonly_vector(self.tcp_xyz_m, 3, "tcp_xyz_m"))
        quat = _readonly_vector(self.tcp_quat_wxyz, 4, "tcp_quat_wxyz")
        if float(np.linalg.norm(quat)) == 0.0:
            raise ValueError("tcp_quat_wxyz must not be zero")
        object.__setattr__(self, "tcp_quat_wxyz", quat)

        try:
            codes = tuple(self.arm_error_codes)
        except TypeError as exc:
            raise ValueError("arm_error_codes must be an iterable of integers") from exc
        normalized_codes = tuple(_error_code(code) for code in codes)
        object.__setattr__(self, "arm_error_codes", normalized_codes)
        if self.joint_speed_rad_s is not None:
            object.__setattr__(
                self,
                "joint_speed_rad_s",
                _readonly_vector(self.joint_speed_rad_s, 6, "joint_speed_rad_s"),
            )


def _mapping_value(mapping: object, key: str, parent: str) -> object:
    if not isinstance(mapping, dict) or key not in mapping:
        raise ValueError(f"missing {parent}.{key}")
    return mapping[key]


def _numeric_prefix(value: object, length: int, name: str) -> np.ndarray:
    if not isinstance(value, (list, tuple, np.ndarray)) or len(value) < length:
        raise ValueError(f"{name} must contain at least {length} values")
    prefix = value[:length]
    for item in prefix:
        _finite_real(item, name)
    try:
        return np.asarray(prefix, dtype=float)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError(f"{name} must contain finite numeric values") from exc


def _error_code(value: object) -> int:
    if isinstance(value, (bool, np.bool_)):
        raise ValueError("error code must be an integer")
    if isinstance(value, numbers.Integral):
        return int(value)
    if isinstance(value, numbers.Real):
        number = float(value)
        if math.isfinite(number) and number.is_integer():
            return int(number)
    raise ValueError("error code must be an integer")


def _parse_error_codes(root: dict) -> tuple[int, ...]:
    err = root.get("err")
    if err is None:
        return ()
    if isinstance(err, dict):
        # Legacy format assumed by the original parser: {"err_code": [...]}.
        raw_codes = err.get("err_code")
        if raw_codes is None:
            return ()
        values = raw_codes if isinstance(raw_codes, list) else [raw_codes]
    elif isinstance(err, list):
        # Third-generation RM controllers (e.g. RM65-BI V1.7.0) push `err` as a
        # plain list of codes, e.g. `[0]` when idle and `[7]` on fault.
        values = err
    else:
        raise ValueError("err must be an object or a list")
    codes = tuple(_error_code(value) for value in values)
    return tuple(code for code in codes if code != 0)


def parse_realtime_arm_state(payload: bytes, received_monotonic_s: float) -> ArmFeedback:
    """Decode one RM65 realtime UDP JSON datagram into immutable SI feedback."""
    timestamp = _finite_real(received_monotonic_s, "received_monotonic_s")
    if not isinstance(payload, bytes):
        raise ValueError("payload must be bytes")
    try:
        root = json.loads(payload.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise ValueError("payload must be one UTF-8 JSON datagram") from exc
    if not isinstance(root, dict):
        raise ValueError("payload root must be an object")

    joint_status = _mapping_value(root, "joint_status", "payload")
    waypoint = _mapping_value(root, "waypoint", "payload")
    joint_raw = _numeric_prefix(
        _mapping_value(joint_status, "joint_position", "joint_status"), 6, "joint_position"
    )
    joint_speed_value = joint_status.get("joint_speed") if isinstance(joint_status, dict) else None
    joint_speed_raw = (
        _numeric_prefix(joint_speed_value, 6, "joint_speed") if joint_speed_value is not None else None
    )
    position_raw = _numeric_prefix(
        _mapping_value(waypoint, "position", "waypoint"), 3, "waypoint.position"
    )
    quaternion_raw = _numeric_prefix(
        _mapping_value(waypoint, "quat", "waypoint"), 4, "waypoint.quat"
    )

    quaternion = quaternion_raw / 1_000_000.0
    norm = float(np.linalg.norm(quaternion))
    if not math.isfinite(norm) or norm == 0.0:
        raise ValueError("waypoint.quat must be a non-zero finite quaternion")
    quaternion /= norm
    if quaternion[0] < 0.0:
        quaternion *= -1.0

    return ArmFeedback(
        received_monotonic_s=timestamp,
        joint_rad=np.deg2rad(joint_raw / 1000.0),
        tcp_xyz_m=position_raw / 1_000_000.0,
        tcp_quat_wxyz=quaternion,
        arm_error_codes=_parse_error_codes(root),
        # Third-generation RM controllers report joint_speed at 0.01 degree/s.
        # Generation is checked by the hardware controller before motion is enabled.
        joint_speed_rad_s=None if joint_speed_raw is None else np.deg2rad(joint_speed_raw * 0.01),
    )


class RealmanUDPFeedbackReceiver:
    """Own one UDP socket and retain only its most recent valid state frame."""

    def __init__(self, bind_host: str = "0.0.0.0", port: int = 8089):
        if not isinstance(bind_host, str):
            raise ValueError("bind_host must be a string")
        if isinstance(port, bool) or not isinstance(port, numbers.Integral) or not 0 <= port <= 65535:
            raise ValueError("port must be an integer from 0 through 65535")
        self.bind_host = bind_host
        self.port = int(port)
        self.invalid_packet_count = 0
        self.valid_packet_count = 0
        self.thread_error: str | None = None
        self._latest: ArmFeedback | None = None
        self._socket: socket.socket | None = None
        self._thread: threading.Thread | None = None
        self._stop_event = threading.Event()
        self._lock = threading.Lock()
        self._lifecycle_lock = threading.Lock()
        self._lifecycle_state = "stopped"

    def start(self) -> None:
        with self._lifecycle_lock:
            if self._lifecycle_state != "stopped":
                raise RuntimeError("UDP feedback receiver is already started")
            self._lifecycle_state = "starting"
            self._stop_event.clear()
            with self._lock:
                self.thread_error = None
            udp_socket: socket.socket | None = None
            try:
                udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                udp_socket.settimeout(0.1)
                udp_socket.bind((self.bind_host, self.port))
                thread = threading.Thread(target=self._receive_loop, args=(udp_socket,), daemon=True)
            except BaseException:
                if udp_socket is not None:
                    try:
                        udp_socket.close()
                    except BaseException:
                        pass
                self._lifecycle_state = "stopped"
                raise

            self._socket = udp_socket
            self._thread = thread
            self._lifecycle_state = "running"
            try:
                thread.start()
            except BaseException:
                self._stop_event.set()
                try:
                    udp_socket.close()
                except BaseException:
                    pass
                finally:
                    self._socket = None
                    self._thread = None
                    self._lifecycle_state = "stopped"
                raise

    def stop(self) -> None:
        cleanup_error: BaseException | None = None
        with self._lifecycle_lock:
            if self._lifecycle_state == "stopped":
                return
            self._lifecycle_state = "stopping"
            self._stop_event.set()
            udp_socket = self._socket
            thread = self._thread
            try:
                if udp_socket is not None:
                    try:
                        udp_socket.close()
                    except BaseException as exc:
                        cleanup_error = exc
                if thread is not None:
                    try:
                        thread.join(timeout=1.0)
                    except BaseException as exc:
                        if cleanup_error is None:
                            cleanup_error = exc
            finally:
                self._socket = None
                self._thread = None
                self._lifecycle_state = "stopped"
        if cleanup_error is not None:
            raise cleanup_error

    def latest(self) -> ArmFeedback | None:
        with self._lock:
            return self._latest

    def age_s(self, now: float) -> float:
        timestamp = _finite_real(now, "now")
        with self._lock:
            latest = self._latest
        if latest is None:
            return math.inf
        return timestamp - latest.received_monotonic_s

    def _store_latest(self, feedback: ArmFeedback) -> None:
        if not isinstance(feedback, ArmFeedback):
            raise ValueError("feedback must be an ArmFeedback")
        with self._lock:
            self._latest = feedback

    def _receive_loop(self, udp_socket: socket.socket) -> None:
        while not self._stop_event.is_set():
            try:
                payload, _address = udp_socket.recvfrom(65535)
            except socket.timeout:
                continue
            except OSError as exc:
                if not self._stop_event.is_set():
                    with self._lock:
                        self.thread_error = f"UDP receive failed: {exc}"
                return

            received_monotonic_s = time.monotonic()
            try:
                feedback = parse_realtime_arm_state(payload, received_monotonic_s)
            except ValueError:
                with self._lock:
                    self.invalid_packet_count += 1
                continue
            self._store_latest(feedback)
            with self._lock:
                self.valid_packet_count += 1
