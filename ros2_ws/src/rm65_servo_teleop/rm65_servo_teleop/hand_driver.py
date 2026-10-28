"""Dedicated RM65 tool-port ROH driver. Opening and polling NEVER write.

Only six speed registers and six position registers may be written, after an
explicit field profile and operator acknowledgement. No power/Modbus setup,
reset, calibration, reconnect, retry, or purported emergency-stop command.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
import time

from .hand_protocol import HandError, ReadOnlyRM65, encode_words, integer, position_request, six
from .hand_session import HandFeedback

LIVE_ACK = "ROH FIELD MOTION VERIFIED"


@dataclass(frozen=True)
class HandConnection:
    host: str
    device: int
    protocol_major: int
    identity: tuple[int, ...]
    port: int = 8080
    timeout_s: float = 0.5
    transport: str = "rs485"
    field_verified: bool = False
    presets_verified: bool = False
    stop_and_concurrency_verified: bool = False
    speeds: tuple[int, ...] | None = None
    lower: tuple[int, ...] | None = None
    upper: tuple[int, ...] | None = None

    def __post_init__(self):
        integer(self.device, 1, 247, "node ID")
        integer(self.protocol_major, 1, 2, "protocol major")
        integer(self.port, 1, 65535, "TCP port")
        if not isinstance(self.host, str) or not self.host or any(c.isspace() for c in self.host):
            raise HandError("explicit controller host required")
        if self.transport != "rs485":
            raise HandError("this driver implements only the RM65 tool-port RS485 path")
        if type(self.timeout_s) not in (int, float) or not math.isfinite(self.timeout_s) or not 0 < self.timeout_s <= 2:
            raise HandError("invalid transaction timeout")
        object.__setattr__(self, "identity", six(self.identity, "expected identity registers"))
        if self.identity[0] >> 8 != self.protocol_major or self.identity[5] != self.device:
            raise HandError("identity does not match protocol/node ID")
        for name in ("field_verified", "presets_verified", "stop_and_concurrency_verified"):
            if type(getattr(self, name)) is not bool:
                raise HandError(f"{name} must be an explicit boolean")
        for name in ("speeds", "lower", "upper"):
            if getattr(self, name) is not None:
                object.__setattr__(self, name, six(getattr(self, name), name))

    def authorize(self, acknowledgement):
        if acknowledgement != LIVE_ACK or not all(
            (self.field_verified, self.presets_verified, self.stop_and_concurrency_verified)
        ):
            raise HandError("live hand remains closed until field, presets, stop and concurrency are verified")
        if self.speeds is None or self.lower is None or self.upper is None:
            raise HandError("field-measured speeds and position bounds required")
        if any(v == 0 for v in self.speeds) or any(a >= b for a, b in zip(self.lower, self.upper)):
            raise HandError("invalid field speed or position bounds")


class RM65Hand(ReadOnlyRM65):
    """Own TCP socket, serialized requests, latched failure; use in one worker."""

    synthetic = False

    def __init__(self, config: HandConnection, *, acknowledgement=""):
        self.config = config
        self.write_enabled = bool(acknowledgement)
        if self.write_enabled:
            config.authorize(acknowledgement)  # Before any connection is opened.
        super().__init__(config.host, config.device, timeout_s=config.timeout_s, port=config.port)
        self.sequence = 0
        self.identified = False
        self.last_blocks = None
        self.latest_feedback = None
        self.transactions = []  # Drained each cycle by the runtime, never an unbounded log.

    def __enter__(self):
        super().__enter__()
        try:
            identity = self.read("identity")
            if tuple(identity["values"]) != self.config.identity:
                raise HandError("identity/firmware changed; field profile must be rechecked")
            self.identified = True
            return self
        except Exception:
            self.close()
            raise

    def poll(self, now=None):
        if not self.identified:
            raise HandError("identity must be verified before feedback")
        blocks = {name: self.read(name) for name in ("status", "position", "current_ma", "angle_cdeg")}
        self.last_blocks = blocks
        self.sequence += 1
        # Conservative age: the oldest component, never stamp a cached group as new.
        stamp = min(block["received_monotonic_s"] for block in blocks.values())
        self.latest_feedback = HandFeedback(
            self.sequence,
            stamp,
            tuple(blocks["position"]["values"]),
            tuple(blocks["status"]["values"]),
            tuple(blocks["current_ma"]["values"]),
            synthetic=False,
        )
        return self.latest_feedback

    def _require_stopped_feedback(self):
        feedback = self.latest_feedback
        if (
            feedback is None
            or not 0 <= time.monotonic() - feedback.received_monotonic_s <= 0.5
            or any(s != 2 for s in feedback.status)
        ):
            self.close()
            raise HandError("fresh six-channel stopped feedback required before a write")

    def _write_six(self, address, values):
        request = {**position_request(values, self.device), "address": address}
        result = self._exchange(request)
        response = result["response"]
        try:
            if response.get("write_state") is not True:
                raise HandError("write rejected or outcome unknown; never retry automatically")
            for key in ("port", "address", "num", "device"):
                if key in response and (type(response[key]) is not int or response[key] != request[key]):
                    raise HandError("write acknowledgement route mismatch")
        except HandError:
            self.close()
            raise
        return result

    def send_position(self, target, now=None):
        if not self.write_enabled or not self.identified or self._failed:
            raise HandError("live write is disabled or connection failed")
        target = six(target)
        if any(not lo <= v <= hi for v, lo, hi in zip(target, self.config.lower, self.config.upper)):
            raise HandError("target outside field-verified bounds")
        self._require_stopped_feedback()
        self.transactions.clear()
        try:
            # Speed is set only for an explicit movement request, never at startup.
            for address, values in ((1125, self.config.speeds), (1135, target)):
                self._require_stopped_feedback()
                self.transactions.append({"kind": "attempted", "address": address, "data": encode_words(values)})
                self.transactions.append({"kind": "ack", **self._write_six(address, values)})
        except Exception:
            self.close()
            raise
        return {
            "command": "write_registers",
            "write_state": True,
            "synthetic": False,
            "transactions": list(self.transactions),
            "atomic_snapshot": False,
        }
