"""Right-controller preset actions; no individual finger commands or arm I/O.

XRoboToolkit PC-Service README-CN defines trigger, primaryButton (A) and
secondaryButton (B). This component shares the existing raw-packet validation.
"""

from __future__ import annotations

from dataclasses import dataclass
import json
import math

from .input_adapter import InputRejected, decode_raw_packet


@dataclass(frozen=True)
class HandButtons:
    timestamp_ns: int
    grip: float
    trigger: float
    a: bool
    b: bool
    synthetic: bool


def decode_hand_buttons(raw: str) -> HandButtons:
    sample = decode_raw_packet(raw)
    envelope = json.loads(raw)
    value = envelope.get("value", envelope)
    value = json.loads(value) if isinstance(value, str) else value
    controller = value["Controller"]["right"]
    trigger = controller.get("trigger")
    if (
        isinstance(trigger, bool)
        or not isinstance(trigger, (int, float))
        or not 0 <= trigger <= 1
        or not math.isfinite(trigger)
    ):
        raise InputRejected("invalid_hand_trigger")
    a, b = controller.get("primaryButton"), controller.get("secondaryButton")
    if type(a) is not bool or type(b) is not bool:
        raise InputRejected("hand_buttons_missing_or_invalid")
    return HandButtons(sample.timestamp_ns, sample.grip, float(trigger), a, b, sample.synthetic)


class HandButtonAdapter:
    """One-shot preset actions and a fresh, uninterrupted B hold for release.

    Startup/reconnect requires all controls neutral. Invalid or frozen input
    only disarms this mapper: it never opens the hand or restores grasp proof.
    B during insertion explicitly marks operator completion then requests release;
    that annotation never asserts physical insertion success.
    """

    def __init__(self, *, timeout_s=0.2, grip_on=0.2, grip_off=0.15, release_hold_s=1.0):
        if not all(
            math.isfinite(v) and not isinstance(v, bool) for v in (timeout_s, grip_on, grip_off, release_hold_s)
        ):
            raise ValueError("hand button settings must be finite")
        if timeout_s <= 0 or release_hold_s <= 0 or not 0 <= grip_off < grip_on <= 1:
            raise ValueError("invalid hand button timing or Grip hysteresis")
        self.timeout_s, self.grip_on, self.grip_off = timeout_s, grip_on, grip_off
        self.release_hold_s = release_hold_s
        self.sample = None
        self.advanced_at = None
        self.armed = False
        self.reason = "release_all_controls_to_start"
        self._trigger = self._grip = self._a = self._b = False
        self._b_started = None
        self._b_consumed = False

    def invalidate(self, reason):
        self.armed = False
        self.reason = reason
        self._b_started = None
        self._trigger = self._grip = self._a = self._b = False

    def check(self, now):
        if self.advanced_at is not None and not 0 <= now - self.advanced_at <= self.timeout_s:
            self.invalidate("hand_controller_timeout")

    def receive_raw(self, raw, now, stage) -> tuple[str, ...]:
        try:
            sample = decode_hand_buttons(raw)
        except (InputRejected, ValueError, TypeError, KeyError) as exc:
            self.invalidate(str(exc))
            return ()
        return self.receive(sample, now, stage)

    def receive(self, sample: HandButtons, now, stage) -> tuple[str, ...]:
        self.check(now)  # A newly arriving frame must not erase an earlier gap.
        if self.sample is not None:
            if sample.timestamp_ns < self.sample.timestamp_ns:
                self.sample = sample
                self.advanced_at = None
                self.invalidate("hand_controller_clock_regressed")
                return ()
            if sample.timestamp_ns == self.sample.timestamp_ns:
                if sample != self.sample:
                    self.invalidate("hand_controller_timestamp_reused")
                return ()  # Neither a timer nor duplicate data completes a long hold.
        self.sample, self.advanced_at = sample, now
        neutral = sample.grip <= self.grip_off and sample.trigger <= 0.25 and not sample.a and not sample.b
        if not self.armed:
            if neutral:
                self.armed, self.reason = True, "hand_buttons_ready"
            else:
                self.reason = "release_all_controls_to_start"
            return ()
        trigger = sample.trigger >= 0.75 or (self._trigger and sample.trigger > 0.25)
        grip = sample.grip >= self.grip_on or (self._grip and sample.grip > self.grip_off)
        trigger_edge, a_edge, b_edge, grip_edge = (
            trigger and not self._trigger,
            sample.a and not self._a,
            sample.b and not self._b,
            grip and not self._grip,
        )
        self._trigger, self._grip, self._a, self._b = trigger, grip, sample.a, sample.b
        if sum((trigger, sample.a, sample.b)) > 1:
            self.invalidate("conflicting_hand_buttons")
            return ()
        if not sample.b:
            self._b_started, self._b_consumed = None, False
        elif b_edge:
            self._b_consumed = sample.grip > self.grip_off
            self._b_started = None if self._b_consumed else (now, sample.timestamp_ns, stage)
            if self._b_consumed:
                self.reason = "release_grip_then_press_b_again"
        elif sample.grip > self.grip_off:
            self._b_started, self._b_consumed = None, True
            self.reason = "release_grip_then_press_b_again"
        if self._b_started is not None and not self._b_consumed:
            started, device_ns, initial_stage = self._b_started
            if stage != initial_stage:
                self._b_consumed = True
                self.reason = "stage_changed_press_b_again"
                return ()
            if now - started >= self.release_hold_s and (sample.timestamp_ns - device_ns) * 1e-9 >= self.release_hold_s:
                self._b_consumed = True
                self.reason = "explicit_b_hold"
                if stage == "unknown":
                    return ("prepare",)
                if stage == "inserting":
                    return ("finish_insert", "release")
                return ("release",)
        if trigger_edge:
            self.reason = "trigger_grasp_requested"
            return ("grasp",)
        if a_edge:
            self.reason = "a_grasp_confirmation_requested"
            return ("confirm_grasp",)
        if grip_edge and stage == "holding":
            self.reason = "grip_insertion_annotation"
            return ("begin_insert",)
        return ()

    def diagnostics(self, now):
        return {
            "armed": self.armed,
            "reason": self.reason,
            "release_hold_s": self.release_hold_s,
            "source_timestamp_ns": None if self.sample is None else self.sample.timestamp_ns,
            "source_synthetic": None if self.sample is None else self.sample.synthetic,
            "source_age_s": None if self.advanced_at is None else max(0.0, now - self.advanced_at),
            "b_hold_started": self._b_started is not None and not self._b_consumed,
        }
