"""Bounded transport admission for the standard Servo/JTC output, without IK.

Times are receiver-local monotonic seconds. A raw driver snapshot may only be
compared to that clock after its Linux boot ID matches the receiver's boot ID.
"""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
from typing import Any, Iterable

JOINT_NAMES = tuple(f"joint{i}" for i in range(1, 7))


def _vector(values: Iterable[float], name: str) -> tuple[float, ...]:
    result = tuple(float(value) for value in values)
    if len(result) != 6 or not all(math.isfinite(value) for value in result):
        raise ValueError(f"{name} must contain exactly six finite values")
    return result


def ordered_joints(names: Iterable[str], values: Iterable[float]) -> tuple[float, ...]:
    names = tuple(names)
    values = _vector(values, "joint values")
    if len(names) != 6 or len(set(names)) != 6 or set(names) != set(JOINT_NAMES):
        raise ValueError("joint names must contain joint1 through joint6 exactly once")
    lookup = dict(zip(names, values))
    return tuple(lookup[name] for name in JOINT_NAMES)


@dataclass(frozen=True)
class Feedback:
    session: str
    packet_id: int
    source_s: float
    ros_stamp_ns: int
    position: tuple[float, ...]
    velocity: tuple[float, ...]
    controller_errors: tuple[int, ...]
    joint_errors: tuple[int, ...]
    enabled: tuple[bool, ...]


@dataclass(frozen=True)
class StreamReply:
    api_code: int
    generation: int
    stream_enabled: bool


def parse_stream_reply(payload: str, *, success: bool, enabled: bool) -> StreamReply:
    """Validate the patched driver's correlated Trigger response, not a Bool ACK."""
    data = json.loads(payload)
    if not isinstance(data, dict) or set(data) != {"api_code", "generation", "stream_enabled"}:
        raise ValueError("invalid stream service response schema")
    code, generation, state = data["api_code"], data["generation"], data["stream_enabled"]
    if type(code) is not int or type(generation) is not int or not 0 < generation < 2**64:
        raise ValueError("invalid stream service code or generation")
    if type(state) is not bool or not success or code != 0 or state is not enabled:
        raise ValueError("driver did not confirm the requested stream state")
    return StreamReply(code, generation, state)


def parse_snapshot(payload: str | dict[str, Any], *, now_s: float, clock_id: str, max_age_s: float = 0.2) -> Feedback:
    data = json.loads(payload) if isinstance(payload, str) else payload
    if not isinstance(data, dict) or data.get("contract") != "rm65_udp_callback_v1":
        raise ValueError("missing coherent UDP callback contract")
    if not clock_id or data.get("clock_id") != clock_id:
        raise ValueError("feedback comes from a different or unverified monotonic clock")
    session = data.get("session")
    packet_id = data.get("packet_id")
    if not isinstance(session, str) or not session or len(session) > 128:
        raise ValueError("invalid driver session")
    if isinstance(packet_id, bool) or not isinstance(packet_id, int) or packet_id <= 0:
        raise ValueError("invalid source packet counter")
    if data.get("parse_error") != 0 or data.get("joint_speed_enabled") is not True:
        raise ValueError("driver reports an incomplete packet or disabled speed feedback")
    if data.get("speed_unit") != "rad/s_from_sdk_degree_s":
        raise ValueError("unverified SDK speed unit")
    source_ns = data.get("source_monotonic_ns")
    ros_stamp_ns = data.get("source_ros_ns")
    if isinstance(source_ns, bool) or not isinstance(source_ns, int) or source_ns <= 0:
        raise ValueError("missing true UDP receive time")
    if isinstance(ros_stamp_ns, bool) or not isinstance(ros_stamp_ns, int) or ros_stamp_ns <= 0:
        raise ValueError("missing source ROS/system timestamp")
    source_s = source_ns / 1e9
    if not math.isfinite(now_s) or not 0.0 <= now_s - source_s <= max_age_s:
        raise ValueError("source feedback is stale or in the future")
    position = ordered_joints(data.get("joint_names", ()), data.get("position_rad", ()))
    velocity = ordered_joints(data.get("joint_names", ()), data.get("velocity_rad_s", ()))
    errors = data.get("controller_errors")
    joint_errors = data.get("joint_errors")
    enabled = data.get("joint_enabled")
    if not isinstance(errors, list) or len(errors) > 64 or any(type(item) is not int for item in errors):
        raise ValueError("missing controller error list")
    if (
        not isinstance(joint_errors, list)
        or len(joint_errors) != 6
        or any(type(item) is not int for item in joint_errors)
    ):
        raise ValueError("missing same-packet joint errors")
    if not isinstance(enabled, list) or len(enabled) != 6 or any(type(item) is not bool for item in enabled):
        raise ValueError("missing same-packet joint enable flags")
    return Feedback(
        session,
        packet_id,
        source_s,
        ros_stamp_ns,
        position,
        velocity,
        tuple(item for item in errors if item),
        tuple(dict(zip(data["joint_names"], joint_errors))[name] for name in JOINT_NAMES),
        tuple(dict(zip(data["joint_names"], enabled))[name] for name in JOINT_NAMES),
    )


@dataclass(frozen=True)
class GuardConfig:
    period_s: float = 0.02
    timeout_s: float = 0.20
    stationary_rad_s: float = math.radians(1.0)
    speed_rad_s: tuple[float, ...] = tuple(math.radians(value) for value in (18, 18, 22.5, 22.5, 22.5, 22.5))
    lower_rad: tuple[float, ...] = (-3.11, -2.27, -2.36, -3.11, -2.23, -6.28)
    upper_rad: tuple[float, ...] = (3.11, 2.27, 2.36, 3.11, 2.23, 6.28)
    margin_rad: float = 0.0  # Limits already come from the single, margin-adjusted model.
    tracking_lag_cycles: float = 3.0
    lag_hold_dwell_s: float = 0.30
    stop_timeout_s: float = 0.50

    def __post_init__(self) -> None:
        for value in (
            self.period_s,
            self.timeout_s,
            self.stationary_rad_s,
            self.tracking_lag_cycles,
            self.lag_hold_dwell_s,
            self.stop_timeout_s,
        ):
            if not math.isfinite(value) or value <= 0:
                raise ValueError("transport timing and budgets must be finite and positive")
        if self.tracking_lag_cycles <= 1:
            raise ValueError("tracking_lag_cycles must exceed one")
        if not math.isfinite(self.margin_rad) or self.margin_rad < 0:
            raise ValueError("invalid joint margin")
        speed = _vector(self.speed_rad_s, "joint speeds")
        lower = _vector(self.lower_rad, "lower limits")
        upper = _vector(self.upper_rad, "upper limits")
        if any(value <= 0 for value in speed) or any(lo + 2 * self.margin_rad >= hi for lo, hi in zip(lower, upper)):
            raise ValueError("invalid model limits")


@dataclass(frozen=True)
class StopRequest:
    request_id: int
    kind: str
    reason: str
    requested_s: float


@dataclass(frozen=True)
class Admission:
    state: str
    reason: str
    target: tuple[float, ...] | None = None


class TransportGuard:
    """One latest command, explicit stop confirmation and release-based rearming."""

    def __init__(self, config: GuardConfig | None = None):
        self.config = config or GuardConfig()
        self.feedback: Feedback | None = None
        self.enabled = False
        self.enable_received_s = -math.inf
        self.command: tuple[float, ...] | None = None
        self.command_received_s = -math.inf
        self.servo_status: int | None = None
        self.servo_received_s = -math.inf
        self.last_submitted: tuple[float, ...] | None = None
        self.last_submit_s = -math.inf
        self.stream_armed = False
        self.armed_s = -math.inf
        self.startup_waiting = False
        self.epoch = 0
        self.latched_reason: str | None = None
        self.release_after_fault = False
        self.stop_confirmed = True
        self.pending_stop: StopRequest | None = None
        self._stop_dispatched = False
        self._request_counter = 0
        self._hold_started_s: float | None = None
        self._was_active = False

    def _stop(self, kind: str, reason: str, now_s: float) -> None:
        self.stream_armed = False
        self.epoch += 1
        self.command = None
        self.last_submitted = None
        self._was_active = False
        if self.startup_waiting:
            return
        if self.pending_stop is not None and (self.pending_stop.kind == "hard" or kind == "soft"):
            return
        self._request_counter += 1
        self.pending_stop = StopRequest(self._request_counter, kind, reason, now_s)
        self._stop_dispatched = False
        self.stop_confirmed = False

    def fault(self, reason: str, now_s: float) -> None:
        if self.latched_reason is None:
            self.latched_reason = reason
            self.release_after_fault = False
            self._stop("hard", reason, now_s)

    def request_stop(self, kind: str, reason: str, now_s: float) -> None:
        if kind not in ("soft", "hard"):
            raise ValueError("stop kind must be soft or hard")
        self._stop(kind, reason, now_s)

    def wait_for_driver(self) -> None:
        """No stop ACK deadline runs while the vendor process is still starting."""
        self.startup_waiting = True
        self.stream_armed = False
        self.stop_confirmed = False

    def begin_driver_handshake(self, now_s: float, *, services_ready: bool) -> bool:
        if not self.startup_waiting or not services_ready or self.feedback is None:
            return False
        if not 0 <= now_s - self.feedback.source_s <= self.config.timeout_s:
            return False
        self.startup_waiting = False
        self._stop("hard" if self.latched_reason else "soft", "startup_stop_confirmation", now_s)
        return True

    def revoke_obsolete_enable(self, now_s: float) -> None:
        """A late native enable must be closed even while a fault was already latched."""
        self.latched_reason = self.latched_reason or "obsolete_driver_stream_enable"
        self.release_after_fault = False
        self._stop("hard", self.latched_reason, now_s)

    def update_enable(self, enabled: bool, now_s: float) -> None:
        previous = self.enabled
        self.enabled = bool(enabled)
        self.enable_received_s = now_s
        if previous and not enabled:
            if self.latched_reason is not None:
                self.release_after_fault = True
            else:
                self._stop("soft", "grip_release", now_s)
            self.epoch += 1
            self.stream_armed = False
            self.command = None
            self.last_submitted = None
        if enabled and not previous:
            self.epoch += 1
            self.last_submitted = None
            self.command = None

    def update_servo_status(self, status: int, now_s: float) -> None:
        self.servo_status = int(status)
        self.servo_received_s = now_s
        # Humble status_codes.h: 1/6 singular deceleration, 3 collision
        # deceleration, 2/4 halt and 5 joint bound. These are not Rolling codes.
        if status in (-1, 2, 4, 5) or status not in (0, 1, 3, 6):
            if self.enabled:
                self.fault(f"servo_status_{status}", now_s)

    def update_feedback(self, feedback: Feedback, now_s: float) -> bool:
        previous = self.feedback
        if previous is not None:
            if feedback.session == previous.session and feedback.packet_id <= previous.packet_id:
                self.fault("feedback_packet_not_advancing", now_s)
                return False
            if feedback.session == previous.session and feedback.source_s <= previous.source_s:
                self.fault("feedback_receive_time_not_advancing", now_s)
                return False
            if feedback.session != previous.session:
                self.fault("driver_session_changed", now_s)
        self.feedback = feedback
        if feedback.controller_errors or any(feedback.joint_errors):
            self.fault("controller_or_joint_error", now_s)
        elif any(abs(value) > limit + 1e-9 for value, limit in zip(feedback.velocity, self.config.speed_rad_s)):
            self.fault("measured_joint_overspeed", now_s)
        elif not self._inside_limits(feedback.position):
            self.fault("measured_joint_limit", now_s)
        return True

    def update_command(self, names: Iterable[str], position: Iterable[float], now_s: float) -> None:
        self.command = ordered_joints(names, position)
        self.command_received_s = now_s

    def _inside_limits(self, position: tuple[float, ...]) -> bool:
        return all(
            lo + self.config.margin_rad <= q <= hi - self.config.margin_rad
            for q, lo, hi in zip(position, self.config.lower_rad, self.config.upper_rad)
        )

    def safe_feedback(self, now_s: float, *, stationary: bool = False) -> bool:
        frame = self.feedback
        return bool(
            frame is not None
            and 0 <= now_s - frame.source_s <= self.config.timeout_s
            and not frame.controller_errors
            and not any(frame.joint_errors)
            and all(frame.enabled)
            and self._inside_limits(frame.position)
            and all(abs(value) <= limit + 1e-9 for value, limit in zip(frame.velocity, self.config.speed_rad_s))
            and (not stationary or all(abs(value) <= self.config.stationary_rad_s for value in frame.velocity))
        )

    def can_arm(self, now_s: float) -> bool:
        return bool(
            not self.startup_waiting
            and self.enabled
            and 0 <= now_s - self.enable_received_s <= self.config.timeout_s
            and self.latched_reason is None
            and self.pending_stop is None
            and self.stop_confirmed
            and self.safe_feedback(now_s, stationary=True)
        )

    def mark_armed(self, epoch: int, success: bool, now_s: float) -> bool:
        if epoch != self.epoch or not self.can_arm(now_s) or not success:
            return False
        self.stream_armed = True
        self.armed_s = now_s
        self.last_submitted = self.feedback.position
        self.last_submit_s = now_s
        return True

    def take_stop_request(self) -> StopRequest | None:
        if self.pending_stop is None or self._stop_dispatched:
            return None
        self._stop_dispatched = True
        return self.pending_stop

    def confirm_stop(self, request_id: int, success: bool, now_s: float) -> bool:
        request = self.pending_stop
        if request is None or request.request_id != request_id:
            return False
        if now_s - request.requested_s > self.config.stop_timeout_s or not success:
            self.latched_reason = self.latched_reason or "stop_not_confirmed"
            self.stop_confirmed = False
            if request.kind == "soft":
                self._stop("hard", "soft_stop_not_confirmed", now_s)
            return False
        self.pending_stop = None
        self.stop_confirmed = True
        return True

    def tick(self, now_s: float) -> Admission:
        if self.startup_waiting:
            return Admission("hold", "awaiting_true_feedback_and_driver_services")
        if self.pending_stop is not None and now_s - self.pending_stop.requested_s > self.config.stop_timeout_s:
            self.latched_reason = self.latched_reason or "stop_ack_timeout"
            if self.pending_stop.kind == "soft":
                self._stop("hard", "soft_stop_ack_timeout", now_s)
            return Admission("fault", self.latched_reason)
        if self.latched_reason is not None:
            if (
                not self.enabled
                and self.release_after_fault
                and self.stop_confirmed
                and self.pending_stop is None
                and self.safe_feedback(now_s, stationary=True)
            ):
                self.latched_reason = None
                self.release_after_fault = False
                return Admission("disabled", "fault_rearmed_release_grip_again")
            return Admission("fault", self.latched_reason)
        if not self.enabled:
            return Admission("disabled", "grip_released")
        if now_s - self.enable_received_s > self.config.timeout_s:
            self.fault("enable_heartbeat_timeout", now_s)
            return Admission("fault", self.latched_reason)
        if not self.safe_feedback(now_s):
            self.fault("feedback_missing_stale_or_disabled", now_s)
            return Admission("fault", self.latched_reason)
        if self.servo_status not in (0, 1, 3, 6) or now_s - self.servo_received_s > self.config.timeout_s:
            self.fault("servo_status_missing_or_stale", now_s)
            return Admission("fault", self.latched_reason)
        if not self.stream_armed:
            return Admission("hold", "awaiting_driver_stream_enable")
        if self.command is None or now_s - self.command_received_s > self.config.timeout_s:
            if self._was_active or now_s - self.armed_s > self.config.timeout_s:
                self.fault("joint_command_timeout", now_s)
                return Admission("fault", self.latched_reason)
            return Admission("hold", "awaiting_new_joint_command")
        if not self._inside_limits(self.command):
            self.fault("command_joint_limit", now_s)
            return Admission("fault", self.latched_reason)
        reference = self.last_submitted or self.feedback.position
        step = tuple(limit * self.config.period_s for limit in self.config.speed_rad_s)
        if any(
            abs(target - previous) > budget + 1e-9 for target, previous, budget in zip(self.command, reference, step)
        ):
            self.fault("adjacent_command_step", now_s)
            return Admission("fault", self.latched_reason)
        lag = [
            abs(target - measured) / budget
            for target, measured, budget in zip(self.command, self.feedback.position, step)
        ]
        if max(lag) > self.config.tracking_lag_cycles + 1e-9:
            self.fault("tracking_lead_exceeded", now_s)
            return Admission("fault", self.latched_reason)
        if max(lag) >= self.config.tracking_lag_cycles - 1:
            self._hold_started_s = now_s if self._hold_started_s is None else self._hold_started_s
            if now_s - self._hold_started_s > self.config.lag_hold_dwell_s:
                self.fault("tracking_lead_no_progress", now_s)
                return Admission("fault", self.latched_reason)
            return Admission("hold", "tracking_lead")
        self._hold_started_s = None
        self.last_submitted = self.command
        self.last_submit_s = now_s
        self._was_active = True
        return Admission("submit", "admitted", self.command)
