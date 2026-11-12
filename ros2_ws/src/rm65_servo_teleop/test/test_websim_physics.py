"""Native MuJoCo 4 physics: commands cannot masquerade as physical feedback."""
import json
from pathlib import Path
from types import SimpleNamespace
import xml.etree.ElementTree as ET

import numpy as np
import pytest

mujoco = pytest.importorskip("mujoco", reason="native MuJoCo 4 physics environment required")
if mujoco.__version__ != "4.0.0":
    pytest.skip("MuJoCo 4 checks use the separate Python 3.11 runtime", allow_module_level=True)

from rm65_servo_teleop.model import JOINT_NAMES
from rm65_servo_teleop.websim_physics import Physics


@pytest.fixture
def physics(tmp_path):
    root = ET.Element("mujoco")
    ET.SubElement(root, "option", timestep=".001", gravity="0 0 0", integrator="implicitfast")
    body = ET.SubElement(ET.SubElement(root, "worldbody"), "body", name="base_link")
    for index, name in enumerate(JOINT_NAMES):
        body = ET.SubElement(body, "body", name="tcp" if index == 5 else f"link{index}", pos="0 0 .1")
        ET.SubElement(body, "joint", name=name, type="hinge", axis="0 1 0")
        ET.SubElement(body, "geom", type="capsule", size=".015 .05", mass=".1")
    actuator = ET.SubElement(root, "actuator")
    for name in JOINT_NAMES:
        ET.SubElement(actuator, "position", name=name, joint=name, kp="20", dampratio="1",
                      ctrllimited="true", ctrlrange="-1 1", forcelimited="true", forcerange="-2 2")
    ET.SubElement(ET.SubElement(root, "keyframe"), "key", name="home", qpos="0 0 0 0 0 0", ctrl="0 0 0 0 0 0")
    path = tmp_path / "scene.xml"
    path.write_text(ET.tostring(root, encoding="unicode"), encoding="utf-8")
    return Physics(path, {"model": {"joint_lower_rad": [-1] * 6, "joint_upper_rad": [1] * 6}})


def test_native_engine_does_not_teleport_joints_to_command(physics):
    physics.receive(JOINT_NAMES, [.05] * 6, 1.0, 1.0, now=0.0)
    np.testing.assert_array_equal(physics.joints, np.zeros(6))
    physics.advance(now=.004)
    assert not np.allclose(physics.joints, [.05] * 6)
    for index in range(1, 101):
        stamp = 1.0 + index * .004
        physics.receive(JOINT_NAMES, [.05] * 6, stamp, stamp, now=index * .004)
        physics.advance(now=index * .004)
    assert np.linalg.norm(physics.joints) > .001
    snapshot = physics.snapshot()
    assert snapshot["hardware_commands_sent"] == 0 and snapshot["engine"] == "4.0.0"
    json.dumps(snapshot, allow_nan=False)


@pytest.mark.parametrize("position,stamp,current", [([2] * 6, 1.0, 1.0), ([0] * 6, .5, 1.0),
                                                      ([0] * 6, 2.0, 1.0), ([float('nan')] * 6, 1.0, 1.0)])
def test_invalid_command_does_not_change_reference(physics, position, stamp, current):
    before = physics.command.copy()
    with pytest.raises(ValueError):
        physics.receive(JOINT_NAMES, position, stamp, current, now=0.0)
    np.testing.assert_array_equal(physics.command, before)
    assert physics.command_at is None


def test_lost_command_stops_integration_instead_of_reusing_it(physics):
    physics.receive(JOINT_NAMES, [.05] * 6, 1.0, 1.0, now=0.0)
    before = physics.data.time
    with pytest.raises(RuntimeError, match="watchdog"):
        physics.advance(now=.201)
    assert physics.data.time == before


def test_old_engine_is_rejected_before_model_load():
    with pytest.raises(ValueError, match="4.0.0"):
        Physics(Path("not-read.xml"), {}, engine=SimpleNamespace(__version__="3.12.0"))
