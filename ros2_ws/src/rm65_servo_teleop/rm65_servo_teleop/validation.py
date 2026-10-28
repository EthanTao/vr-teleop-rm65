"""Explicit deployment attestations, never generated automatically by a launch."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import yaml

DRIVER_COMMIT = "61db3c367e8f5743ee68cee52c5574ec1ac645ca"
REQUIRED_CHECKS = (
    "frames_verified",
    "stop_verified",
    "speed_units_verified",
    "firmware_compatible",
    "collision_model_verified",
)


def model_digest(description: str) -> str:
    root = ET.fromstring(description)
    for element in list(root.findall("ros2_control")):
        root.remove(element)
    return hashlib.sha256(ET.tostring(root, encoding="utf-8")).hexdigest()


def deployment_digest(description: str, semantic: str, config: dict, patch_sha256: str, arm_ip: str) -> str:
    value = {
        "model": model_digest(description),
        "semantic": semantic,
        "configuration": config,
        "driver_commit": DRIVER_COMMIT,
        "driver_patch_sha256": patch_sha256,
        "arm_ip": arm_ip,
    }
    return hashlib.sha256(json.dumps(value, sort_keys=True, allow_nan=False).encode("utf-8")).hexdigest()


def require_hardware_validation(path: str, digest: str, *, operator_ack: str, deployment_sha256: str) -> dict:
    if operator_ack != "RM65 LIVE VALIDATION":
        raise ValueError("live execution needs explicit operator acknowledgment after on-site validation")
    if not path:
        raise ValueError("no hardware validation file; hardware execution remains disabled")
    value = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError("invalid hardware validation file")
    if value.get("allow_hardware_motion") is not True or any(value.get(key) is not True for key in REQUIRED_CHECKS):
        raise ValueError("hardware validation is incomplete; do not bypass it to run the arm")
    if value.get("model_sha256") != digest or value.get("driver_commit") != DRIVER_COMMIT:
        raise ValueError("validation does not match this model and driver")
    if not deployment_sha256 or value.get("deployment_sha256") != deployment_sha256:
        raise ValueError("validation does not match this deployment configuration and driver patch")
    if not all(
        isinstance(value.get(key), str) and value[key].strip() for key in ("validated_at", "operator", "evidence")
    ):
        raise ValueError("validation must identify its operator, date and measured evidence")
    return value
