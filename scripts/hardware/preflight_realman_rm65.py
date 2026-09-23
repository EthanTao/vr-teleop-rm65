#!/usr/bin/env python3
"""Read-only preflight checks for the Realman RM65 TCP/JSON endpoint.

This script deliberately sends only ``get_current_arm_state``. It never sends
``movel`` or any other motion command, so it is suitable for the checklist
before starting the teleoperation controller.
"""

from __future__ import annotations

import argparse
import json
import math
import socket
import sys
import time
from typing import Any


def _recv_json_lines(sock: socket.socket, timeout_s: float) -> list[dict[str, Any]]:
    deadline = time.monotonic() + timeout_s
    buffer = b""
    messages: list[dict[str, Any]] = []
    while time.monotonic() < deadline:
        try:
            chunk = sock.recv(4096)
        except socket.timeout:
            break
        if not chunk:
            break
        buffer += chunk
        while b"\n" in buffer:
            raw, buffer = buffer.split(b"\n", 1)
            line = raw.decode("utf-8", "ignore").strip()
            if not line:
                continue
            try:
                value = json.loads(line)
            except json.JSONDecodeError:
                continue
            if isinstance(value, dict):
                messages.append(value)
    return messages


def _get_state(host: str, port: int, timeout_s: float) -> dict[str, Any]:
    with socket.create_connection((host, port), timeout=timeout_s) as sock:
        sock.settimeout(timeout_s)
        payload = {"command": "get_current_arm_state"}
        sock.sendall((json.dumps(payload) + "\r\n").encode("utf-8"))
        messages = _recv_json_lines(sock, timeout_s)

    for message in reversed(messages):
        if message.get("state") == "current_arm_state" and isinstance(message.get("arm_state"), dict):
            return message["arm_state"]
    raise RuntimeError("RM65 returned no current_arm_state response")


def _as_numbers(value: Any, name: str) -> list[float]:
    if not isinstance(value, list) or len(value) < 6:
        raise RuntimeError(f"{name} must contain at least 6 values, got {value!r}")
    numbers = [float(item) for item in value[:6]]
    if not all(math.isfinite(item) for item in numbers):
        raise RuntimeError(f"{name} contains a non-finite value: {numbers!r}")
    return numbers


def _infer_pose_scale(position: list[float]) -> float:
    maximum = max(abs(item) for item in position)
    if maximum >= 20_000:
        return 1_000_000.0
    if maximum >= 20:
        return 1_000.0
    return 1.0


def _infer_rotation_scale(rotation: list[float]) -> float:
    maximum = max(abs(item) for item in rotation)
    if maximum >= 100:
        return 1_000.0
    return 1.0


def run(host: str, port: int, timeout_s: float) -> int:
    print(f"[CHECK] connecting to RM65 {host}:{port}")
    try:
        arm_state = _get_state(host, port, timeout_s)
    except (OSError, RuntimeError, ValueError, TypeError) as exc:
        print(f"[FAIL] {exc}")
        return 1

    try:
        joint = _as_numbers(arm_state.get("joint"), "joint")
        pose = _as_numbers(arm_state.get("pose"), "pose")
    except (RuntimeError, ValueError, TypeError) as exc:
        print(f"[FAIL] {exc}")
        return 2

    pose_scale = _infer_pose_scale(pose[:3])
    rot_scale = _infer_rotation_scale(pose[3:6])
    xyz_m = [value / pose_scale for value in pose[:3]]
    rot_rad = [value / rot_scale for value in pose[3:6]]

    print(f"[OK] current_arm_state received; joint_raw={joint}")
    print(f"[OK] pose_raw={pose}")
    print(f"[INFO] inferred position scale={pose_scale:g} raw/m; rotation scale={rot_scale:g} raw/rad")
    print(f"[INFO] tcp xyz_m={[round(value, 6) for value in xyz_m]}")
    print(f"[INFO] tcp rotvec_rad={[round(value, 6) for value in rot_rad]}")

    warnings: list[str] = []
    if math.sqrt(sum(value * value for value in xyz_m)) > 3.0:
        warnings.append("TCP position is more than 3 m from controller origin; verify pose units")
    if math.sqrt(sum(value * value for value in rot_rad)) > 2.0 * math.pi + 0.2:
        warnings.append("TCP rotation vector is larger than 2*pi; verify rotation units")
    for warning in warnings:
        print(f"[WARN] {warning}")

    print("[PASS] read-only RM65 preflight passed; no motion command was sent")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description="Read-only RM65 TCP/JSON preflight")
    parser.add_argument("--arm-host", default="192.168.10.18")
    parser.add_argument("--arm-port", type=int, default=8080)
    parser.add_argument("--timeout-s", type=float, default=3.0)
    args = parser.parse_args()
    return run(args.arm_host, args.arm_port, args.timeout_s)


if __name__ == "__main__":
    sys.exit(main())
