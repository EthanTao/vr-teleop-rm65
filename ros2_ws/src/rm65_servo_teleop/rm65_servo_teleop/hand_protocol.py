"""ROH six-channel codec and a deliberately read-only public RM65 interface.

References and field limits: docs/灵巧手接入.md. No vendor example is run.
The public read API cannot configure Modbus, power a tool or send a hand target.
The separately guarded hand_driver uses the protected transaction primitive.
"""

from __future__ import annotations

import json
import math
import socket
import threading
import time

CHANNELS = ("thumb_flex", "index_flex", "middle_flex", "ring_flex", "little_flex", "thumb_rotation")
READ_BLOCKS = {"identity": 1000, "status": 1085, "current_ma": 1105, "position": 1145, "angle_cdeg": 1165}
POSITION_TARGET = 1135


class HandError(ValueError):
    """Invalid protocol, stale evidence or inadmissible hand operation."""


def integer(value, low: int, high: int, name: str) -> int:
    if type(value) is not int or not low <= value <= high:
        raise HandError(f"{name} must be an integer in [{low}, {high}]")
    return value


def six(values, name="channels") -> tuple[int, ...]:
    if not isinstance(values, (list, tuple)) or len(values) != 6:
        raise HandError(f"{name} must contain exactly six channels")
    return tuple(integer(value, 0, 65535, name) for value in values)


def encode_words(values) -> list[int]:
    return [byte for value in six(values) for byte in (value >> 8, value & 255)]


def decode_words(data, *, signed=False) -> tuple[int, ...]:
    if not isinstance(data, list) or len(data) != 12:
        raise HandError("six registers require exactly twelve bytes")
    data = [integer(value, 0, 255, "byte") for value in data]
    words = [(data[index] << 8) | data[index + 1] for index in range(0, 12, 2)]
    return tuple(value - 65536 if signed and value >= 32768 else value for value in words)


def read_request(block: str, device: int) -> dict:
    if block not in READ_BLOCKS:
        raise HandError("unsupported read block")
    return {
        "command": "read_multiple_holding_registers",
        "port": 1,
        "address": READ_BLOCKS[block],
        "num": 6,
        "device": integer(device, 1, 247, "device"),
    }


def position_request(values, device: int) -> dict:
    """Encode six position registers; transmission requires hand_driver gates."""
    return {
        "command": "write_registers",
        "port": 1,
        "address": POSITION_TARGET,
        "num": 6,
        "data": encode_words(values),
        "device": integer(device, 1, 247, "device"),
    }


def parse_read(response, request: dict, *, signed=False) -> tuple[int, ...]:
    if not isinstance(response, dict) or response.get("command") != request["command"]:
        raise HandError("unexpected response command")
    if "read_state" in response and response["read_state"] is not True:
        raise HandError("controller rejected or timed out the read")
    # Some firmware echoes routing fields; a conflicting echo is never accepted.
    for key in ("port", "address", "num", "device"):
        if key in response and (type(response[key]) is not int or response[key] != request[key]):
            raise HandError(f"response {key} mismatch")
    return decode_words(response.get("data"), signed=signed)


class ReadOnlyRM65:
    """One explicit connection, one request at a time; fail closed, no reconnect.

    A timeout or unexpected frame poisons this session. Closing the connection
    prevents a late same-command reply being mistaken for the next block.
    Calls belong in a standalone preflight process, never the arm's timer.
    """

    def __init__(self, host: str, device: int, *, timeout_s=0.5, port=8080):
        if not isinstance(host, str) or not host or any(c.isspace() for c in host):
            raise HandError("explicit controller host required")
        self.device = integer(device, 1, 247, "device")
        if isinstance(timeout_s, bool) or not math.isfinite(timeout_s) or not 0 < timeout_s <= 2.0:
            raise HandError("timeout must be positive and at most two seconds")
        self.host, self.port, self.timeout_s = host, integer(port, 1, 65535, "port"), timeout_s
        self._socket = None
        self._failed = False
        self._lock = threading.Lock()

    def __enter__(self):
        if self._failed or self._socket is not None:
            raise HandError("create a new preflight session instead of reconnecting")
        try:
            self._socket = socket.create_connection((self.host, self.port), timeout=self.timeout_s)
        except OSError:
            self._failed = True
            raise
        return self

    def close(self):
        if self._socket is not None:
            self._socket.close()
            self._socket = None
        self._failed = True

    def __exit__(self, *_):
        self.close()

    def read(self, block: str) -> dict:
        request = read_request(block, self.device)  # Only this allowlist reaches sendall.
        result = self._exchange(request)
        try:
            result["values"] = list(parse_read(result["response"], request, signed=block == "angle_cdeg"))
        except (ValueError, TypeError):
            self.close()
            raise
        return result

    def _exchange(self, request: dict) -> dict:
        """Bounded transaction primitive; public read remains allowlisted."""
        with self._lock:
            if self._failed or self._socket is None:
                raise HandError("read-only connection is closed or failed")
            started = time.monotonic()
            try:
                self._socket.sendall(json.dumps(request, separators=(",", ":")).encode("ascii") + b"\r\n")
                received = bytearray()
                while not received.endswith(b"\n"):
                    remaining = self.timeout_s - (time.monotonic() - started)
                    if remaining <= 0:
                        raise TimeoutError("whole read transaction deadline exceeded")
                    self._socket.settimeout(remaining)
                    chunk = self._socket.recv(1)  # Never consume bytes from a second reply.
                    if not chunk:
                        raise HandError("controller closed connection")
                    received.extend(chunk)
                    if len(received) > 4096:
                        raise HandError("JSON frame exceeds 4096 bytes")
                ended = time.monotonic()
                response = json.loads(received.decode("ascii"))
                if not isinstance(response, dict) or response.get("command") != request["command"]:
                    raise HandError("unexpected response command")
            except (OSError, ValueError, TimeoutError):
                self.close()
                raise
            return {
                "request": request,
                "response": response,
                "request_started_monotonic_s": started,
                "received_monotonic_s": ended,
                "clock_source": "host_receive_only",
                "device_sample_time": None,
            }


def probe(host: str, device: int, protocol_major: int, *, timeout_s=0.5) -> dict:
    """No configuration writes, discovery scans, resets, calibration or motion."""
    integer(protocol_major, 1, 2, "protocol major")
    with ReadOnlyRM65(host, device, timeout_s=timeout_s) as client:
        identity = client.read("identity")
        words = identity["values"]
        if words[0] >> 8 != protocol_major or words[5] != device:
            raise HandError("identity protocol major or node ID does not match the explicit selection")
        blocks = {name: client.read(name) for name in READ_BLOCKS if name != "identity"}
    return {
        "schema": "roh_readonly_preflight_v1",
        "synthetic": False,
        "hardware_writes": False,
        "host": host,
        "device": device,
        "identity": identity,
        "blocks": blocks,
        "atomic_snapshot": False,
        "contact_force_n": None,
        "grasp_verified": False,
    }
