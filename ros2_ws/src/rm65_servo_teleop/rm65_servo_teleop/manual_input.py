"""Simulation-only human input; emits labelled XR, never joints or trajectories."""

from __future__ import annotations

import json
import math
import re
import threading

import numpy as np

from .input_adapter import SIDE_MOUNT_MAPPING, rotvec_quaternion


class ManualInput:
    """Latest intent with a short lease, release fence and bounded hand offsets."""

    def __init__(self, *, position_scale=1.0, rotation_scale=0.25, timeout_s=0.03):
        if not all(math.isfinite(x) and x > 0 for x in (position_scale, rotation_scale, timeout_s)):
            raise ValueError("manual input scales and lease must be positive")
        self.position_scale, self.rotation_scale, self.timeout_s = position_scale, rotation_scale, timeout_s
        self.linear_speed, self.angular_speed = 0.02, 0.12
        self.lock = threading.Lock()
        self.owner = None
        self.sequence = -1
        self.received_at = None
        self.ticked_at = None
        self.needs_release = True
        self.pressed = False
        self.axes = np.zeros(6)
        self.xyz, self.rotation = np.zeros(3), np.zeros(3)
        self.reason = "release_required"
        self.stamp_ns = 0

    def _stop(self, reason):
        self.pressed = False
        self.needs_release = True
        self.axes[:] = 0
        self.xyz[:] = 0
        self.rotation[:] = 0
        self.reason = reason

    def submit(self, value: dict, now: float):
        if not isinstance(value, dict):
            raise ValueError("input must be an object")
        client, sequence, pressed, axes = (value.get(k) for k in ("client_id", "seq", "pressed", "axes"))
        if not isinstance(client, str) or not re.fullmatch(r"[a-zA-Z0-9_-]{8,64}", client):
            raise ValueError("invalid client identity")
        if type(sequence) is not int or not 0 <= sequence < 2**53 or type(pressed) is not bool:
            raise ValueError("invalid input sequence or hold state")
        if (
            not isinstance(axes, list)
            or len(axes) != 6
            or any(type(v) not in (int, float) or v not in (-1, 0, 1) for v in axes)
        ):
            raise ValueError("axes must be six directions in {-1, 0, 1}")
        with self.lock:
            expired = self.received_at is None or now - self.received_at > self.timeout_s
            if client != self.owner:
                if not expired:
                    raise PermissionError("another control page holds the input lease")
                self.owner, self.sequence = client, -1
                self._stop("release_required")
            if sequence <= self.sequence:
                raise ValueError("old input sequence; not accepted")
            if expired:
                self._stop("release_required")
            self.sequence, self.received_at = sequence, now
            if not pressed:
                self._stop("released")
                self.needs_release = False
            elif self.needs_release:
                self.reason = "release_required"
            else:
                self.pressed = True
                self.axes[:] = axes
                self.reason = "held"

    def packet(self, now: float, *, ready: bool) -> str:
        with self.lock:
            dt = 0.0 if self.ticked_at is None else now - self.ticked_at
            self.ticked_at = now
            if self.pressed:
                if self.received_at is None or not 0 <= now - self.received_at <= self.timeout_s:
                    self._stop("input_timeout")
                elif not ready:
                    self._stop("control_not_ready")
                elif not 0 <= dt <= 0.08:
                    self._stop("input_loop_stalled")
            if self.pressed:
                linear, angular = self.axes[:3].copy(), self.axes[3:].copy()
                linear /= max(1.0, float(np.linalg.norm(linear)))
                angular /= max(1.0, float(np.linalg.norm(angular)))
                self.xyz += linear * self.linear_speed * dt
                self.rotation += angular * self.angular_speed * dt
                if np.linalg.norm(self.xyz) > 0.10 or np.linalg.norm(self.rotation) > 0.16:
                    self._stop("segment_limit_release_to_continue")
            # Inverse input mapping only; the existing InputAdapter still owns
            # anchoring, Grip, pose validity and the published target semantics.
            hand_xyz = SIDE_MOUNT_MAPPING.T @ self.xyz / self.position_scale
            hand_rotation = SIDE_MOUNT_MAPPING.T @ self.rotation / self.rotation_scale
            self.stamp_ns = max(self.stamp_ns + 1, int(now * 1e9))
            value = {
                "Input": 1,
                "timeStampNs": self.stamp_ns,
                "_rm65_mock": True,
                "_rm65_input_kind": "manual",
                "Controller": {
                    "right": {
                        "pose": [*hand_xyz, *rotvec_quaternion(hand_rotation)],
                        "grip": 0.8 if self.pressed else 0.0,
                        "status": 3,
                    }
                },
            }
            return json.dumps({"functionName": "Tracking", "value": value, "_rm65_mock": True}, allow_nan=False)

    def status(self):
        with self.lock:
            return {
                "active": self.pressed,
                "reason": self.reason,
                "release_required": self.needs_release,
                "linear_speed_m_s": self.linear_speed,
                "angular_speed_rad_s": self.angular_speed,
            }
