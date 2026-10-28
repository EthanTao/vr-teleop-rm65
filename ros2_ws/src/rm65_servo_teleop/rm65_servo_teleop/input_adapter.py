"""Validated XR intent and Grip anchors; no IK, PID, or trajectory generation.

All device timestamps remain in their device clock. Host monotonic time measures
how long that clock has failed to advance. Poses use metres and xyzw quaternions.
"""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
from typing import Any

import numpy as np

SIDE_MOUNT_MAPPING = np.array([[0.0, 1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, -1.0]])


class InputRejected(ValueError):
    """A raw sample must not become a robot intent."""


def _finite_vector(value: Any, length: int, name: str) -> np.ndarray:
    if isinstance(value, str):
        value = value.split(",")
    try:
        result = np.asarray(value, dtype=float)
    except (TypeError, ValueError) as exc:
        raise InputRejected(f"invalid_{name}") from exc
    if result.shape != (length,) or not np.all(np.isfinite(result)):
        raise InputRejected(f"invalid_{name}")
    return result.copy()


def normalized_quaternion(value: Any) -> np.ndarray:
    result = _finite_vector(value, 4, "quaternion")
    norm = float(np.linalg.norm(result))
    if norm < 1e-9 or abs(norm - 1.0) > 0.05:
        raise InputRejected("invalid_quaternion_norm")
    return result / norm


def quaternion_multiply(left: np.ndarray, right: np.ndarray) -> np.ndarray:
    lx, ly, lz, lw = left
    rx, ry, rz, rw = right
    return np.array(
        [
            lw * rx + lx * rw + ly * rz - lz * ry,
            lw * ry - lx * rz + ly * rw + lz * rx,
            lw * rz + lx * ry - ly * rx + lz * rw,
            lw * rw - lx * rx - ly * ry - lz * rz,
        ]
    )


def quaternion_rotvec(quaternion: np.ndarray) -> np.ndarray:
    q = normalized_quaternion(quaternion)
    if q[3] < 0.0:
        q = -q
    magnitude = float(np.linalg.norm(q[:3]))
    if magnitude < 1e-12:
        return 2.0 * q[:3]
    return q[:3] * (2.0 * math.atan2(magnitude, float(q[3])) / magnitude)


def rotvec_quaternion(rotation: np.ndarray) -> np.ndarray:
    angle = float(np.linalg.norm(rotation))
    if angle < 1e-12:
        return normalized_quaternion(np.r_[rotation * 0.5, 1.0])
    return np.r_[rotation * (math.sin(angle / 2.0) / angle), math.cos(angle / 2.0)]


def quaternion_difference(reference: np.ndarray, current: np.ndarray) -> np.ndarray:
    inverse = reference * np.array([-1.0, -1.0, -1.0, 1.0])
    return quaternion_multiply(current, inverse)


def continuous_rotvec(quaternion: np.ndarray, previous: np.ndarray) -> np.ndarray:
    """Choose the rotation-log branch closest to the preceding XR intent.

    The shortest quaternion logarithm jumps from +pi to -pi. Clipping that
    logarithm to the Grip rotation envelope would otherwise flip a saturated
    target between its two ends during a smooth hand rotation.
    """
    principal = quaternion_rotvec(quaternion)
    angle = float(np.linalg.norm(principal))
    previous_norm = float(np.linalg.norm(previous))
    if angle < 1e-10:
        if previous_norm < math.pi:
            return principal
        turns = round(previous_norm / (2.0 * math.pi))
        return previous * (turns * 2.0 * math.pi / previous_norm)
    axis = principal / angle
    nearest = round(float(np.dot(previous - principal, axis)) / (2.0 * math.pi))
    candidates = [principal + (2.0 * math.pi * turn) * axis for turn in (nearest - 1, nearest, nearest + 1)]
    return min(candidates, key=lambda candidate: float(np.linalg.norm(candidate - previous)))


@dataclass(frozen=True)
class Pose:
    xyz: np.ndarray
    quaternion_xyzw: np.ndarray

    def __post_init__(self) -> None:
        object.__setattr__(self, "xyz", _finite_vector(self.xyz, 3, "position"))
        object.__setattr__(self, "quaternion_xyzw", normalized_quaternion(self.quaternion_xyzw))

    @classmethod
    def from_vector(cls, value: Any) -> "Pose":
        vector = _finite_vector(value, 7, "pose")
        return cls(vector[:3], normalized_quaternion(vector[3:]))

    def copied(self) -> "Pose":
        return Pose(self.xyz.copy(), self.quaternion_xyzw.copy())


@dataclass(frozen=True)
class ControllerSample:
    timestamp_ns: int
    pose: Pose
    grip: float
    status_present: bool = False
    synthetic: bool = False

    def __post_init__(self) -> None:
        if (
            isinstance(self.timestamp_ns, bool)
            or not isinstance(self.timestamp_ns, int)
            or not 0 < self.timestamp_ns < 2**63
        ):
            raise InputRejected("invalid_device_timestamp")
        if isinstance(self.grip, bool) or not math.isfinite(self.grip) or not 0.0 <= self.grip <= 1.0:
            raise InputRejected("invalid_grip")
        if not isinstance(self.pose, Pose):
            raise InputRejected("invalid_pose")


def decode_raw_packet(raw: str, *, side: str = "right", max_bytes: int = 131072) -> ControllerSample:
    """Decode the native SDK Tracking/value JSON, including its nested JSON string.

    The published XR protocol does not require a controller status field. Its
    absence is recorded, never replaced by a fabricated "tracked" bitmask.
    Explicit invalid status/isActive/valid fields are rejected when present.
    """
    if not isinstance(raw, str) or len(raw.encode("utf-8")) > max_bytes:
        raise InputRejected("invalid_json_size")
    try:
        envelope = json.loads(raw)
        if not isinstance(envelope, dict):
            raise InputRejected("invalid_json_object")
        if "sdk_event" in envelope:
            raise InputRejected("sdk_" + str(envelope["sdk_event"]))
        if envelope.get("functionName", "Tracking") != "Tracking":
            raise InputRejected("not_tracking")
        value = envelope.get("value", envelope)
        if isinstance(value, str):
            value = json.loads(value)
        if not isinstance(value, dict):
            raise InputRejected("invalid_tracking_object")
        if type(value.get("Input")) is not int or value.get("Input") != 1:
            raise InputRejected("controller_input_not_selected")
        controllers = value.get("Controller")
        if not isinstance(controllers, dict) or not isinstance(controllers.get(side), dict):
            raise InputRejected("controller_missing")
        controller = controllers[side]
        stamp = controller.get("timeStampNs", controllers.get("timeStampNs", value.get("timeStampNs")))
        if isinstance(stamp, bool) or not isinstance(stamp, int) or not 0 < stamp < 2**63:
            raise InputRejected("invalid_device_timestamp")
        status_present = "status" in controller
        if status_present:
            status = controller["status"]
            # Public examples use 1 (trusted) or 3 (valid pose bits).
            # Unknown flags do not establish valid position and orientation.
            if isinstance(status, bool) or not isinstance(status, int) or status not in (1, 3):
                raise InputRejected("controller_tracking_invalid")
        for key in ("isActive", "valid"):
            if key in controller and controller[key] not in (True, 1):
                raise InputRejected("controller_tracking_invalid")
        grip_value = controller.get("grip")
        if isinstance(grip_value, bool):
            raise InputRejected("invalid_grip")
        grip = float(grip_value)
        if not math.isfinite(grip) or not 0.0 <= grip <= 1.0:
            raise InputRejected("invalid_grip")
        return ControllerSample(
            stamp,
            Pose.from_vector(controller.get("pose")),
            grip,
            status_present=status_present,
            synthetic=envelope.get("_rm65_mock") is True or value.get("_rm65_mock") is True,
        )
    except InputRejected:
        raise
    except (KeyError, TypeError, ValueError, OverflowError) as exc:
        raise InputRejected("invalid_tracking_payload") from exc


@dataclass(frozen=True)
class InputConfig:
    position_scale: float = 1.0
    rotation_scale: float = 0.25
    grip_on: float = 0.2
    grip_off: float = 0.15
    xr_timeout_s: float = 0.20
    feedback_timeout_s: float = 0.20
    max_xr_step_m: float = 0.05
    max_xr_rot_step_rad: float = 0.26
    max_rotation_offset_rad: float = 0.20
    allow_synthetic: bool = False

    def __post_init__(self) -> None:
        for name in (
            "position_scale",
            "rotation_scale",
            "xr_timeout_s",
            "feedback_timeout_s",
            "max_xr_step_m",
            "max_xr_rot_step_rad",
            "max_rotation_offset_rad",
        ):
            value = getattr(self, name)
            if isinstance(value, bool) or not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be positive and finite")
        if not 0.0 <= self.grip_off < self.grip_on <= 1.0:
            raise ValueError("Grip thresholds must satisfy 0 <= off < on <= 1")


@dataclass(frozen=True)
class InputDecision:
    enabled: bool
    target: Pose | None
    reason: str
    anchor_id: int
    source_timestamp_ns: int | None
    source_age_s: float | None
    status_present: bool
    synthetic: bool

    def diagnostics(self) -> dict:
        return {
            "enabled": self.enabled,
            "reason": self.reason,
            "anchor_id": self.anchor_id,
            "source_timestamp_ns": self.source_timestamp_ns,
            "source_age_s": self.source_age_s,
            "controller_status_present": self.status_present,
            "synthetic": self.synthetic,
        }


class InputAdapter:
    """One source clock, one Grip segment, and one measured TCP anchor.

    A fault disarms immediately. A fresh released sample is required before the
    next Grip edge. Receiving duplicate packets never refreshes source age.
    """

    def __init__(self, config: InputConfig | None = None):
        self.config = config or InputConfig()
        self.sample: ControllerSample | None = None
        self._last_timestamp: int | None = None
        self._advanced_at: float | None = None
        self._last_active_sample: ControllerSample | None = None
        self._hand_anchor: Pose | None = None
        self._tcp_anchor: Pose | None = None
        self._needs_release = True
        self._pending_engage = False
        self._active = False
        self._reason = "release_grip_to_start"
        self._unwrapped_rotation = np.zeros(3)
        self.anchor_id = 0

    def invalidate(self, reason: str) -> None:
        self._active = False
        self._pending_engage = False
        self._needs_release = True
        self._hand_anchor = self._tcp_anchor = None
        self._last_active_sample = None
        self._unwrapped_rotation = np.zeros(3)
        self._reason = reason

    def receive_raw(self, raw: str, received_monotonic_s: float) -> None:
        try:
            self.receive(decode_raw_packet(raw), received_monotonic_s)
        except InputRejected as exc:
            self.invalidate(str(exc))

    def receive(self, sample: ControllerSample, received_monotonic_s: float) -> None:
        if not math.isfinite(received_monotonic_s):
            raise ValueError("host monotonic time must be finite")
        if sample.synthetic and not self.config.allow_synthetic:
            self.invalidate("synthetic_input_disabled")
            return
        if self._last_timestamp is not None:
            if sample.timestamp_ns < self._last_timestamp:
                # A new source-clock epoch may recover only after a fresh release.
                self._last_timestamp = sample.timestamp_ns
                self.sample = None
                self._advanced_at = None
                self.invalidate("device_timestamp_regressed")
                return
            if sample.timestamp_ns == self._last_timestamp:
                if self.sample is not None and (
                    sample.grip != self.sample.grip
                    or not np.array_equal(sample.pose.xyz, self.sample.pose.xyz)
                    or not np.array_equal(sample.pose.quaternion_xyzw, self.sample.pose.quaternion_xyzw)
                ):
                    self.invalidate("device_timestamp_reused")
                return
        if self._advanced_at is not None and received_monotonic_s < self._advanced_at:
            self.invalidate("host_monotonic_time_regressed")
            return
        if self._advanced_at is not None and received_monotonic_s - self._advanced_at > self.config.xr_timeout_s:
            # A delayed executor may run the new raw callback before its watchdog
            # timer. Do not let that arriving sample erase the preceding outage.
            self.invalidate("xr_timeout")
        self._last_timestamp = sample.timestamp_ns
        self._advanced_at = received_monotonic_s
        self.sample = sample
        if sample.grip <= self.config.grip_off:
            self._active = self._pending_engage = False
            self._needs_release = False
            self._hand_anchor = self._tcp_anchor = None
            self._last_active_sample = None
            self._unwrapped_rotation = np.zeros(3)
            self._reason = "released"
            return
        if self._active and self._last_active_sample is not None:
            previous = self._last_active_sample.pose
            if float(np.linalg.norm(sample.pose.xyz - previous.xyz)) > self.config.max_xr_step_m:
                self.invalidate("xr_translation_jump")
                return
            relative = quaternion_difference(previous.quaternion_xyzw, sample.pose.quaternion_xyzw)
            if float(np.linalg.norm(quaternion_rotvec(relative))) > self.config.max_xr_rot_step_rad:
                self.invalidate("xr_rotation_jump")
                return
        if self._active:
            self._last_active_sample = sample
        elif not self._needs_release and sample.grip >= self.config.grip_on:
            self._pending_engage = True

    def step(
        self,
        now_monotonic_s: float,
        tcp_pose: Pose | None,
        feedback_monotonic_s: float | None,
        *,
        tf_fresh: bool = True,
    ) -> InputDecision:
        if not math.isfinite(now_monotonic_s):
            raise ValueError("host monotonic time must be finite")
        age = None if self._advanced_at is None else now_monotonic_s - self._advanced_at
        if age is None:
            return self._decision(False, None, self._reason, age)
        if age < 0.0 or age > self.config.xr_timeout_s:
            self.invalidate("xr_timeout" if age >= 0.0 else "host_monotonic_time_regressed")
        elif feedback_monotonic_s is None or not math.isfinite(feedback_monotonic_s):
            self.invalidate("feedback_unavailable")
        elif not 0.0 <= now_monotonic_s - feedback_monotonic_s <= self.config.feedback_timeout_s:
            self.invalidate("feedback_stale")
        elif tcp_pose is None or not tf_fresh:
            self.invalidate("tcp_transform_unavailable")
        elif self._pending_engage:
            self._hand_anchor = self.sample.pose.copied()
            self._tcp_anchor = tcp_pose.copied()
            self._last_active_sample = self.sample
            self._unwrapped_rotation = np.zeros(3)
            self._pending_engage = False
            self._active = True
            self.anchor_id += 1
            self._reason = "anchoring"
            # One disabled heartbeat establishes the target before enabling Servo.
            return self._decision(False, self._tcp_anchor.copied(), self._reason, age)
        elif self._active:
            mapped_translation = SIDE_MOUNT_MAPPING @ (self.sample.pose.xyz - self._hand_anchor.xyz)
            relative = quaternion_difference(self._hand_anchor.quaternion_xyzw, self.sample.pose.quaternion_xyzw)
            self._unwrapped_rotation = continuous_rotvec(relative, self._unwrapped_rotation)
            mapped_rotation = SIDE_MOUNT_MAPPING @ self._unwrapped_rotation * self.config.rotation_scale
            magnitude = float(np.linalg.norm(mapped_rotation))
            if magnitude > self.config.max_rotation_offset_rad:
                mapped_rotation *= self.config.max_rotation_offset_rad / magnitude
            target = Pose(
                self._tcp_anchor.xyz + mapped_translation * self.config.position_scale,
                normalized_quaternion(
                    quaternion_multiply(rotvec_quaternion(mapped_rotation), self._tcp_anchor.quaternion_xyzw)
                ),
            )
            self._reason = "tracking"
            return self._decision(True, target, self._reason, age)
        return self._decision(False, None, self._reason, age)

    def _decision(self, enabled: bool, target: Pose | None, reason: str, age: float | None) -> InputDecision:
        return InputDecision(
            enabled,
            target,
            reason,
            self.anchor_id,
            None if self.sample is None else self.sample.timestamp_ns,
            age,
            False if self.sample is None else self.sample.status_present,
            False if self.sample is None else self.sample.synthetic,
        )
