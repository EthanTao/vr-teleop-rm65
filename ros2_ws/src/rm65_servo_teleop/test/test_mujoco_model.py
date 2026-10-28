"""Use real MuJoCo to check vendor FK agreement, dynamics and floor contact."""

from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import pytest
import yaml

mujoco = pytest.importorskip("mujoco", reason="MuJoCo physics checks run in the Linux simulation image")
ament = pytest.importorskip("ament_index_python.packages", reason="requires the pinned vendor ROS model")

from rm65_servo_teleop.model import JOINT_NAMES, build_description, build_semantic
from rm65_servo_teleop.mujoco_scene import write_scene
from rm65_servo_teleop.validation import model_digest


@pytest.fixture
def scene(tmp_path):
    share = Path(__file__).resolve().parents[1]
    config = yaml.safe_load((share / "config/teleop.yaml").read_text(encoding="utf-8"))
    physics = yaml.safe_load((share / "config/mujoco.yaml").read_text(encoding="utf-8"))
    vendor = Path(ament.get_package_share_directory("rm_description")) / "urdf/rm_65.urdf"
    semantic_source = Path(ament.get_package_share_directory("rm_65_config")) / "config/rm_65_description.srdf"
    description = build_description(vendor, config, "sim")
    semantic = build_semantic(semantic_source, config)
    path = write_scene(description, semantic, config, physics, tmp_path)
    return path, description, config


def urdf_tcp(description, positions):
    """Independent transform composition, with URDF's Rz(yaw) Ry(pitch) Rx(roll)."""
    root = ET.fromstring(description)
    edges = {j.find("child").get("link"): j for j in root.findall("joint")}
    chain, child = [], "tcp"
    while child in edges:
        joint = edges[child]
        chain.append(joint)
        child = joint.find("parent").get("link")
    result = np.eye(4)
    for joint in reversed(chain):
        origin = joint.find("origin")
        xyz = np.fromstring(origin.get("xyz", "0 0 0"), sep=" ")
        roll, pitch, yaw = np.fromstring(origin.get("rpy", "0 0 0"), sep=" ")
        cr, sr, cp, sp, cy, sy = np.cos(roll), np.sin(roll), np.cos(pitch), np.sin(pitch), np.cos(yaw), np.sin(yaw)
        matrix = np.eye(4)
        matrix[:3, :3] = [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ]
        matrix[:3, 3] = xyz
        result = result @ matrix
        if joint.get("type") == "revolute":
            axis = np.fromstring(joint.find("axis").get("xyz"), sep=" ")
            axis /= np.linalg.norm(axis)
            x, y, z = axis
            cross = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
            angle = positions[joint.get("name")]
            rotation = np.eye(4)
            rotation[:3, :3] = np.eye(3) + np.sin(angle) * cross + (1 - np.cos(angle)) * (cross @ cross)
            result = result @ rotation
    return result


def test_mujoco_tcp_matches_same_urdf_at_multiple_joint_positions(scene):
    path, description, config = scene
    model = mujoco.MjModel.from_xml_path(str(path))
    data = mujoco.MjData(model)
    tcp = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "tcp")
    assert tcp >= 0
    for q in (config["model"]["home_joint_rad"], [0] * 6, [0.2, -0.2, -0.8, 0.1, 0.9, -0.2]):
        for name, value in zip(JOINT_NAMES, q):
            data.qpos[model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)]] = value
        mujoco.mj_forward(model, data)
        expected = urdf_tcp(description, dict(zip(JOINT_NAMES, q)))
        np.testing.assert_allclose(data.xpos[tcp], expected[:3, 3], atol=2e-6)
        np.testing.assert_allclose(data.xmat[tcp].reshape(3, 3), expected[:3, :3], atol=2e-6)
    device = build_description(
        Path(ament.get_package_share_directory("rm_description")) / "urdf/rm_65.urdf",
        config,
        "sim",
        simulator="mujoco",
        mujoco_model=str(path),
    )
    assert model_digest(device) == model_digest(description)


def test_actuator_targets_do_not_teleport_physical_joints(scene):
    path, _, _ = scene
    model = mujoco.MjModel.from_xml_path(str(path))
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, 0)
    initial = data.qpos.copy()
    data.ctrl[0] += 0.1
    mujoco.mj_step(model, data)
    assert abs(data.qpos[0] - initial[0]) < 0.01
    for _ in range(1200):
        mujoco.mj_step(model, data)
    assert np.isfinite(data.qpos).all()
    assert abs(data.qpos[0] - initial[0]) > 0.02


def test_position_actuators_remain_stable_at_representative_rest_poses(scene):
    path, _, config = scene
    model = mujoco.MjModel.from_xml_path(str(path))
    for pose in (config["model"]["home_joint_rad"], [0] * 6, [0.2, -0.2, -0.8, 0.1, 0.9, -0.2]):
        data = mujoco.MjData(model)
        data.qpos[:] = pose
        data.ctrl[:] = pose
        for _ in range(1000):
            mujoco.mj_step(model, data)
            assert np.isfinite(data.qpos).all() and np.isfinite(data.qvel).all()
            assert np.max(np.abs(data.qpos - pose)) < 1e-3
        assert np.max(np.abs(data.qvel)) < 1e-3


def test_generated_scene_has_gravity_and_resolving_floor_contact(scene):
    path, _, _ = scene
    root = ET.parse(path).getroot()
    # An unactuated test object away from the arm must fall and be supported by
    # the scene's floor. It is inserted only into this isolated physics test.
    root.remove(root.find("keyframe"))
    body = ET.SubElement(root.find("worldbody"), "body", name="contact_probe", pos="1 0 0.20")
    ET.SubElement(body, "freejoint")
    ET.SubElement(body, "geom", type="sphere", size="0.025", mass="0.1")
    model = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"))
    data = mujoco.MjData(model)
    probe = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "contact_probe")
    for _ in range(1500):
        mujoco.mj_step(model, data)
    assert data.ncon > 0
    assert abs(data.xpos[probe, 2]) < 0.002
    assert np.isfinite(data.qpos).all()
