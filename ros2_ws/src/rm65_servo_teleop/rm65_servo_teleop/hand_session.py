"""Grasp/hold/insert task stages; transport injected, never commands the arm."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math
import time
import uuid

from .hand_protocol import CHANNELS, HandError, integer, position_request, six

ACTIONS = ("prepare", "grasp", "confirm_grasp", "begin_insert", "finish_insert", "release", "abort")


@dataclass(frozen=True)
class HandProfile:
    open: tuple[int, ...]
    grasp: tuple[int, ...]
    mock_speed_units_s: int = 30000
    feedback_timeout_s: float = 0.5
    movement_timeout_s: float = 4.0
    position_tolerance: int = 200

    def __post_init__(self):
        object.__setattr__(self, "open", six(self.open, "open preset"))
        object.__setattr__(self, "grasp", six(self.grasp, "grasp preset"))
        if self.open == self.grasp:
            raise HandError("open and grasp fixtures must differ")
        integer(self.mock_speed_units_s, 1, 65535, "mock speed")
        integer(self.position_tolerance, 0, 1000, "position tolerance")
        for value in (self.feedback_timeout_s, self.movement_timeout_s):
            if isinstance(value, bool) or not math.isfinite(value) or value <= 0:
                raise HandError("timeouts must be positive and finite")

    @classmethod
    def from_dict(cls, value):
        return cls(**value["presets"], **{key: item for key, item in value.items() if key != "presets"})


@dataclass(frozen=True)
class HandFeedback:
    sequence: int
    received_monotonic_s: float
    position: tuple[int, ...]
    status: tuple[int, ...]
    current_ma: tuple[int, ...]
    synthetic: bool = True

    def __post_init__(self):
        integer(self.sequence, 1, 2**63 - 1, "feedback sequence")
        if not math.isfinite(self.received_monotonic_s):
            raise HandError("invalid feedback time")
        for key in ("position", "status", "current_ma"):
            object.__setattr__(self, key, six(getattr(self, key), key))
        if type(self.synthetic) is not bool:
            raise HandError("synthetic must be explicit")


class MockHand:
    """Bounded kinematic fixture, NOT contact physics, a driver or grasp proof."""

    def __init__(self, profile: HandProfile):
        self.profile = profile
        # Startup reads an arbitrary fixture; it does not send the open preset.
        self.position = [4000.0] * 6
        self.target = None
        self.sequence = 0
        self.last_at = None
        self.write_count = 0

    def send_position(self, target, now):
        self.target = six(target)
        self.write_count += 1
        return {"command": "write_registers", "write_state": True, "synthetic": True}

    def poll(self, now) -> HandFeedback:
        elapsed = 0.0 if self.last_at is None else max(0.0, min(now - self.last_at, 0.1))
        self.last_at = now
        statuses = []
        for index, actual in enumerate(self.position):
            desired = actual if self.target is None else self.target[index]
            step = self.profile.mock_speed_units_s * elapsed
            self.position[index] += max(-step, min(step, desired - actual))
            statuses.append(2 if self.position[index] == desired else (1 if desired > actual else 0))
        self.sequence += 1
        return HandFeedback(self.sequence, now, tuple(round(q) for q in self.position), tuple(statuses), (0,) * 6)


class HandSession:
    """Task annotation only: insertion stages never publish arm commands.

    Mock remains the default. A live worker must verify its independent field
    protocol/preset/stop gates and provide actual feedback with synthetic=False.
    """

    def __init__(self, profile: HandProfile, *, synthetic=True):
        if type(synthetic) is not bool:
            raise HandError("explicit feedback source required")
        self.synthetic = synthetic
        self.profile = profile
        self.session_id = uuid.uuid4().hex
        self.stage = "unknown"
        self.revision = 0
        self.last_command_sequence = 0
        self.feedback = None
        self.target = None
        self.sent_at = None
        self.pending_target = None
        self.events = deque(maxlen=128)
        self.fault_reason = None
        self.grasp_confirmed = False

    def _event(self, kind, now, **fields):
        self.events.append(
            {
                "kind": kind,
                "monotonic_s": now,
                "session_id": self.session_id,
                "revision": self.revision,
                "synthetic": self.synthetic,
                **fields,
            }
        )

    def _stage(self, stage, now):
        self.stage = stage
        self.revision += 1
        self._event("stage", now, stage=stage)

    def fault(self, reason, now):
        self.pending_target = None
        self.grasp_confirmed = False
        if self.stage != "fault":
            self.fault_reason = reason
            self._stage("fault", now)
            self._event("fault", now, reason=reason, physical_hand_stopped=None)

    def _fresh(self, now):
        return (
            self.feedback is not None
            and 0 <= now - self.feedback.received_monotonic_s <= self.profile.feedback_timeout_s
        )

    def check(self, now):
        if self.stage == "fault":
            return
        if self.feedback is not None and not self._fresh(now):
            self.fault("hand_feedback_stale", now)
        if self.sent_at is not None and self.stage in {"preparing", "grasping", "releasing"}:
            if now - self.sent_at > self.profile.movement_timeout_s:
                self.fault("hand_motion_timeout", now)

    def observe(self, feedback: HandFeedback, now):
        self.check(now)  # Detect a gap before a new sample can refresh the cache.
        if feedback.synthetic is not self.synthetic:
            self.fault(
                "mock_session_rejects_real_feedback" if self.synthetic else "real_session_rejects_mock_feedback", now
            )
            return
        if not 0 <= now - feedback.received_monotonic_s <= self.profile.feedback_timeout_s:
            self.fault("invalid_feedback_age", now)
            return
        if self.feedback is not None:
            if feedback.sequence <= self.feedback.sequence:
                self.check(now)  # A duplicate never refreshes the age.
                return
            if feedback.received_monotonic_s <= self.feedback.received_monotonic_s:
                self.fault("feedback_time_regressed", now)
                return
        self.feedback = feedback
        self._event("feedback", now, **self._feedback_dict())
        if any(status not in {0, 1, 2} for status in feedback.status):
            self.fault("hand_protection_or_unknown_status", now)
            return
        self.check(now)
        # A new post-send sample, six stopped channels and measured positions
        # are required. An ACK alone never advances a movement stage.
        if self.sent_at is None or feedback.received_monotonic_s <= self.sent_at or self.target is None:
            return
        reached = all(s == 2 for s in feedback.status) and all(
            abs(actual - desired) <= self.profile.position_tolerance
            for actual, desired in zip(feedback.position, self.target)
        )
        if self.stage in {"awaiting_grasp_confirmation", "holding", "inserting", "inserted"} and not reached:
            self.fault("grasp_position_or_stopped_state_lost", now)
            return
        next_stage = {"preparing": "ready", "grasping": "awaiting_grasp_confirmation", "releasing": "released"}
        if reached and self.stage in next_stage:
            self._stage(next_stage[self.stage], now)

    def command(self, envelope, now) -> bool:
        self.check(now)
        action = envelope.get("action") if isinstance(envelope, dict) else None
        self._event(
            "candidate", now, action=action, source=envelope.get("source") if isinstance(envelope, dict) else None
        )
        try:
            if not isinstance(envelope, dict) or envelope.get("session_id") != self.session_id:
                raise HandError("wrong session; restart never restores an old grasp")
            sequence = integer(envelope.get("sequence"), 1, 2**63 - 1, "command sequence")
            if sequence <= self.last_command_sequence:
                raise HandError("duplicate or replayed command")
            # Even a rejected request consumes its sequence; it cannot execute later.
            self.last_command_sequence = sequence
            revision = integer(envelope.get("expected_revision"), 0, 2**63 - 1, "expected revision")
            if revision != self.revision:
                raise HandError("stage changed; inspect status before requesting again")
            if action not in ACTIONS:
                raise HandError("unknown action")
            if self.stage == "fault":
                raise HandError("latched fault; inspect and start a new offline session")
            if action == "abort":
                self.fault("operator_abort", now)
                self._event("accepted", now, action=action, sequence=sequence)
                return True
            if not self._fresh(now) or any(status != 2 for status in self.feedback.status):
                raise HandError("need fresh stopped feedback")
            transitions = {
                "prepare": ({"unknown", "released", "ready"}, "preparing", self.profile.open),
                "grasp": ({"ready", "released"}, "grasping", self.profile.grasp),
                "confirm_grasp": ({"awaiting_grasp_confirmation"}, "holding", None),
                "begin_insert": ({"holding"}, "inserting", None),
                "finish_insert": ({"inserting"}, "inserted", None),
                # Explicit release only. Reject during insertion until finish_insert.
                "release": (
                    {"ready", "awaiting_grasp_confirmation", "holding", "inserted"},
                    "releasing",
                    self.profile.open,
                ),
            }
            allowed, stage, target = transitions[action]
            if self.stage not in allowed:
                raise HandError(f"{action} is forbidden in {self.stage}")
            self.pending_target = target
            if target is not None:
                self.sent_at = None
                self.grasp_confirmed = False
            elif action == "confirm_grasp":
                self.grasp_confirmed = True
            self._stage(stage, now)
            self._event("accepted", now, action=action, sequence=sequence, target=target)
            return True
        except HandError as exc:
            self._event(
                "rejected",
                now,
                action=action,
                sequence=envelope.get("sequence") if isinstance(envelope, dict) else None,
                reason=str(exc),
            )
            return False

    def flush_mock(self, transport: MockHand, now):
        if not self.synthetic:
            raise HandError("mock flush cannot send a real session")
        self.flush_transport(transport, now)

    def flush_transport(self, transport, now):
        if self.pending_target is None or self.stage == "fault":
            return
        target, self.pending_target = self.pending_target, None
        request = position_request(target, 2 if self.synthetic else transport.device)
        self._event("sent" if self.synthetic else "send_attempt", now, request=request)
        try:
            result = transport.send_position(target, now)
            self._event("ack", now, response=result)
            if result.get("write_state") is not True or result.get("synthetic") is not self.synthetic:
                raise HandError("invalid hand write acknowledgement")
            if not self.synthetic:
                self._event("sent", time.monotonic(), request=request)
            self.target, self.sent_at = target, now if self.synthetic else time.monotonic()
        except (OSError, HandError) as exc:
            if not self.synthetic:
                self._event("send_outcome_unknown", time.monotonic(), transactions=list(transport.transactions))
            self.fault(f"send_failed:{exc}", now)  # No retry and no open/close on fault.

    def _feedback_dict(self):
        value = self.feedback
        return (
            None
            if value is None
            else {
                "sequence": value.sequence,
                "received_monotonic_s": value.received_monotonic_s,
                "position": value.position,
                "status": value.status,
                "current_ma": value.current_ma,
                "synthetic": value.synthetic,
                "contact_force_n": None,
            }
        )

    def status(self, now):
        return {
            "schema": "roh_mock_session_v1" if self.synthetic else "roh_hardware_session_v1",
            "session_id": self.session_id,
            "stage": self.stage,
            "revision": self.revision,
            "last_command_sequence": self.last_command_sequence,
            "synthetic": self.synthetic,
            "hardware_writes": not self.synthetic,
            "channels": CHANNELS,
            "target": self.target,
            "feedback": self._feedback_dict(),
            "feedback_fresh": self._fresh(now),
            "fault_reason": self.fault_reason,
            "grasp_confirmed_by_operator": self.grasp_confirmed,
            "grasp_evidence": (
                ("operator_confirmation_in_mock_only" if self.synthetic else "operator_confirmation")
                if self.grasp_confirmed
                else None
            ),
            "arm_motion_authorized": False,
            "physical_grasp_verified": False,
        }
