"""One bounded hand worker; ROS callbacks enqueue and never wait for TCP."""

from __future__ import annotations

from collections import deque
import queue
import threading
import time

from .hand_driver import HandConnection, RM65Hand
from .hand_input import HandButtonAdapter
from .hand_protocol import HandError
from .hand_session import HandProfile, HandSession, MockHand
from .peripheral_clock import clock_identity


class HandRuntime:
    def __init__(self, mode, profile, *, acknowledgement="", transport=None, input_config=None):
        if mode not in {"mock", "readonly", "live"}:
            raise HandError("hand runtime mode must be mock, readonly or live")
        self.mode = mode
        self.connection = None if mode == "mock" else HandConnection(**profile["connection"])
        if mode == "live":
            self.connection.authorize(acknowledgement)
        self.transport = transport or (
            MockHand(HandProfile.from_dict(profile))
            if mode == "mock"
            else RM65Hand(self.connection, acknowledgement=acknowledgement if mode == "live" else "")
        )
        # readonly has no task session and cannot accept commands or buttons.
        self.session = (
            None
            if mode == "readonly"
            else HandSession(
                HandProfile.from_dict(profile if mode == "mock" else profile["task"]), synthetic=mode == "mock"
            )
        )
        if mode == "live":
            for target in (self.session.profile.open, self.session.profile.grasp):
                if any(not lo <= v <= hi for v, lo, hi in zip(target, self.connection.lower, self.connection.upper)):
                    raise HandError("preset outside field-verified bounds")
        self.buttons = HandButtonAdapter(**(input_config or {}))
        self.commands = queue.Queue(maxsize=8)
        self.events = deque(maxlen=256)
        self.lock = threading.Lock()
        self.stop = threading.Event()
        self.error = None
        self.status = {"stage": "starting", "synthetic": mode == "mock", "hardware_writes": False}
        self.raw = None
        self.thread = None

    def command(self, envelope):
        if self.mode == "readonly":
            return False
        try:
            self.commands.put_nowait(envelope)
            return True
        except queue.Full:
            self.error = "hand command queue overflow; outcome requires inspection"
            return False

    def xr_input(self, raw):
        if self.mode != "readonly" and isinstance(raw, str) and len(raw) <= 65536:
            with self.lock:
                self.raw = (raw, time.monotonic())

    def snapshot(self):
        with self.lock:
            result, events = dict(self.status), list(self.events)
            self.events.clear()
            return result, events

    def start(self):
        if self.thread is not None:
            raise HandError("runtime cannot be restarted")
        self.thread = threading.Thread(target=self._work, name="roh-worker", daemon=True)
        self.thread.start()

    def _publish(self, status, events):
        with self.lock:
            if len(self.events) + len(events) > self.events.maxlen:
                self.error = self.error or "hand event queue overflow"
            self.events.extend(events)
            self.status = {
                **status,
                "mode": self.mode,
                "clock_identity": clock_identity(),
                "atomic_snapshot": False,
                "clock_source": "host_receive_only",
                "error": self.error,
                "contact_force_n": None,
            }

    def _work(self):
        try:
            if self.mode != "mock":
                self.transport.__enter__()  # Dedicated connection, identification only.
            while not self.stop.is_set() and self.error is None:
                feedback = self.transport.poll(time.monotonic())
                now = time.monotonic()
                events = []
                if self.session is None:
                    status = {
                        "stage": "readonly",
                        "synthetic": False,
                        "hardware_writes": False,
                        "feedback": feedback.__dict__,
                        "physical_grasp_verified": False,
                    }
                else:
                    self.session.observe(feedback, now)
                    with self.lock:
                        raw, self.raw = self.raw, None
                    self.buttons.check(now)
                    if raw is not None and 0 <= now - raw[1] <= self.buttons.timeout_s:
                        actions = self.buttons.receive_raw(raw[0], raw[1], self.session.stage)
                        for action in actions:
                            self.session.command(
                                {
                                    "session_id": self.session.session_id,
                                    "sequence": self.session.last_command_sequence + 1,
                                    "expected_revision": self.session.revision,
                                    "action": action,
                                    "source": {"kind": "xr_buttons"},
                                },
                                now,
                            )
                    for _ in range(8):
                        try:
                            self.session.command(self.commands.get_nowait(), now)
                        except queue.Empty:
                            break
                    if self.error or self.stop.is_set():
                        self.session.fault(self.error or "runtime_closing", now)
                    self.session.flush_transport(self.transport, now)
                    status = self.session.status(time.monotonic())
                    events = list(self.session.events)
                    self.session.events.clear()
                    if self.session.stage == "fault":
                        self.error = self.session.fault_reason
                if self.mode != "mock":
                    events.append(
                        {
                            "kind": "raw_feedback_blocks",
                            "synthetic": False,
                            "blocks": self.transport.last_blocks,
                            "atomic_snapshot": False,
                        }
                    )
                self._publish(status, events)
                self.stop.wait(0.05)
        except Exception as exc:
            self.error = f"hand worker failed: {exc}"
        finally:
            if self.mode != "mock":
                self.transport.close()  # No automatic release, reset or motion.
            if self.session is not None and self.error:
                self.session.fault(self.error, time.monotonic())
            events = list(self.session.events) if self.session else []
            self._publish(
                {
                    **self.status,
                    "stage": "fault" if self.error else "closed",
                    "physical_hand_stopped": None,
                    "hardware_writes": False,
                },
                events,
            )

    def close(self):
        self.stop.set()
        if self.thread:
            self.thread.join(timeout=9.0)  # Four bounded reads plus one in-flight write.
            if self.thread.is_alive():
                self.error = "hand worker shutdown deadline exceeded; physical state unknown"
