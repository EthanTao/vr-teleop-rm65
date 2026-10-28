"""Model/control separation and explicit live-authorization tests; no robot access."""

from __future__ import annotations

import copy
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest
import yaml

from rm65_servo_teleop.model import build_description, build_semantic
from rm65_servo_teleop.validation import (
    DRIVER_COMMIT,
    REQUIRED_CHECKS,
    deployment_digest,
    model_digest,
    require_hardware_validation,
)

CONFIG = Path(__file__).resolve().parents[1] / "config/teleop.yaml"


@pytest.fixture
def vendor(tmp_path):
    root = ET.Element("robot", name="vendor")
    for name in ["base_link", *[f"Link{i}" for i in range(1, 7)]]:
        ET.SubElement(root, "link", name=name)
    for i in range(1, 7):
        joint = ET.SubElement(root, "joint", name=f"joint{i}", type="revolute")
        ET.SubElement(joint, "origin", xyz="0 -0.144 0" if i == 6 else "0 0 0")
        ET.SubElement(joint, "limit", lower="-6", upper="6", velocity="4", effort="10")
    path = tmp_path / "vendor.urdf"
    path.write_text(ET.tostring(root, encoding="unicode"), encoding="utf-8")
    return path


def test_sim_and_hardware_change_only_device_interfaces(vendor):
    config = yaml.safe_load(CONFIG.read_text(encoding="utf-8"))
    sim = build_description(vendor, config, "sim")
    real = build_description(vendor, config, "hardware-dry-run")
    assert model_digest(sim) == model_digest(real)
    assert "mock_components/GenericSystem" in sim
    assert "topic_based_ros2_control/TopicBasedSystem" in real
    root = ET.fromstring(sim)
    assert root.find("joint[@name='joint6']/origin").get("xyz") == "0 -0.1612 0"
    for i, speed in enumerate(config["model"]["joint_velocity_rad_s"], 1):
        assert float(root.find(f"joint[@name='joint{i}']/limit").get("velocity")) == speed


def test_model_changes_invalidate_hardware_evidence(vendor):
    config = yaml.safe_load(CONFIG.read_text(encoding="utf-8"))
    first = model_digest(build_description(vendor, config, "sim"))
    config["model"]["tool_xyz_m"][2] += 0.001
    assert model_digest(build_description(vendor, config, "sim")) != first


def test_physics_plugin_cannot_be_selected_for_hardware(vendor, tmp_path):
    config = yaml.safe_load(CONFIG.read_text(encoding="utf-8"))
    with pytest.raises(ValueError, match="only be selected for simulation"):
        build_description(
            vendor, config, "hardware-dry-run", simulator="mujoco", mujoco_model=str(tmp_path / "scene.xml")
        )


@pytest.mark.parametrize("field,value", [("d6_m", float("nan")), ("d6_m", 3), ("home_joint_rad", [0] * 5)])
def test_invalid_model_refuses_before_nodes_start(vendor, field, value):
    config = yaml.safe_load(CONFIG.read_text(encoding="utf-8"))
    config["model"][field] = value
    with pytest.raises(ValueError):
        build_description(vendor, config, "sim")


def test_vendor_geometry_drift_refuses_silent_parameter_patch(vendor):
    config = yaml.safe_load(CONFIG.read_text(encoding="utf-8"))
    tree = ET.parse(vendor)
    tree.find("joint[@name='joint6']/origin").set("xyz", "0.01 -0.144 0")
    tree.write(vendor, encoding="unicode")
    with pytest.raises(ValueError, match="geometry changed"):
        build_description(vendor, config, "sim")


def test_live_template_is_not_an_authorization():
    path = CONFIG.with_name("hardware_validation.example.yaml")
    with pytest.raises(ValueError, match="incomplete"):
        require_hardware_validation(
            str(path), "anything", operator_ack="RM65 LIVE VALIDATION", deployment_sha256="test"
        )


def test_live_checks_cannot_be_bypassed_with_truthy_strings(tmp_path):
    record = {key: "true" for key in (*REQUIRED_CHECKS, "allow_hardware_motion")}
    path = tmp_path / "checks.yaml"
    path.write_text(yaml.safe_dump(record), encoding="utf-8")
    with pytest.raises(ValueError):
        require_hardware_validation(str(path), "x", operator_ack="RM65 LIVE VALIDATION", deployment_sha256="test")


def test_hardware_checks_bind_to_this_driver_model_and_operator(tmp_path):
    record = {key: True for key in (*REQUIRED_CHECKS, "allow_hardware_motion")}
    record.update(
        driver_commit=DRIVER_COMMIT,
        model_sha256="abc",
        deployment_sha256="deployment",
        validated_at="test-fixture",
        operator="test",
        evidence="synthetic test only",
    )
    path = tmp_path / "checks.yaml"
    path.write_text(yaml.safe_dump(record), encoding="utf-8")
    assert (
        require_hardware_validation(
            str(path), "abc", operator_ack="RM65 LIVE VALIDATION", deployment_sha256="deployment"
        )
        == record
    )
    with pytest.raises(ValueError, match="does not match"):
        require_hardware_validation(
            str(path), "changed", operator_ack="RM65 LIVE VALIDATION", deployment_sha256="deployment"
        )
    with pytest.raises(ValueError, match="acknowledgment"):
        require_hardware_validation(str(path), "abc", operator_ack="", deployment_sha256="deployment")


def test_deployment_evidence_expires_for_collision_config_or_driver_changes(vendor):
    config = yaml.safe_load(CONFIG.read_text(encoding="utf-8"))
    model = build_description(vendor, config, "sim")
    first = deployment_digest(model, "semantic", config, "patch-v1", "192.168.10.18")
    assert deployment_digest(model, "different-semantic", config, "patch-v1", "192.168.10.18") != first
    assert deployment_digest(model, "semantic", config, "patch-v2", "192.168.10.18") != first
    assert deployment_digest(model, "semantic", config, "patch-v1", "192.168.10.19") != first
    config["servo"]["check_collisions"] = False
    assert deployment_digest(model, "semantic", config, "patch-v1", "192.168.10.18") != first
