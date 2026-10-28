"""Construct the same calibrated-candidate model for simulated and real I/O.

The manufacturer's URDF and meshes remain in the pinned rm_description package.
Changing a model parameter is not hardware calibration or permission to move.
"""

from __future__ import annotations

import math
from pathlib import Path
import xml.etree.ElementTree as ET

JOINT_NAMES = tuple(f"joint{i}" for i in range(1, 7))


def finite_vector(value, length: int, name: str) -> list[float]:
    if not isinstance(value, (list, tuple)) or len(value) != length:
        raise ValueError(f"{name} must contain {length} numbers")
    result = [float(x) for x in value]
    if not all(math.isfinite(x) for x in result):
        raise ValueError(f"{name} must be finite")
    return result


def _numbers(values) -> str:
    return " ".join(format(float(value), ".12g") for value in values)


def build_description(
    vendor_urdf: str | Path,
    config: dict,
    mode: str,
    *,
    simulator: str = "mock",
    mujoco_model: str = "",
    headless: bool = True,
) -> str:
    if mode not in {"sim", "hardware-dry-run", "hardware"}:
        raise ValueError("invalid execution mode")
    if simulator not in {"mock", "mujoco"} or (mode != "sim" and simulator != "mock"):
        raise ValueError("MuJoCo can only be selected for simulation")
    root = ET.parse(vendor_urdf).getroot()
    root.set("name", "rm65_servo")
    robot = config["model"]
    home = finite_vector(robot["home_joint_rad"], 6, "home_joint_rad")
    lower = finite_vector(robot["joint_lower_rad"], 6, "joint_lower_rad")
    upper = finite_vector(robot["joint_upper_rad"], 6, "joint_upper_rad")
    speeds = finite_vector(robot["joint_velocity_rad_s"], 6, "joint_velocity_rad_s")
    if any(lo >= hi or not lo <= q <= hi or speed <= 0 for lo, hi, q, speed in zip(lower, upper, home, speeds)):
        raise ValueError("invalid joint limits, speed or Home")
    for index, name in enumerate(JOINT_NAMES):
        joint = root.find(f"joint[@name='{name}']")
        if joint is None or joint.find("limit") is None:
            raise ValueError(f"vendor model missing {name} or its limit")
        limit = joint.find("limit")
        limit.set("lower", str(lower[index]))
        limit.set("upper", str(upper[index]))
        limit.set("velocity", str(speeds[index]))
    # The pinned vendor URDF expresses d6 in joint6's negative parent-y direction.
    origin = root.find("joint[@name='joint6']/origin")
    xyz = finite_vector([float(x) for x in origin.get("xyz").split()], 3, "joint6 origin")
    if abs(xyz[0]) > 1e-9 or abs(xyz[2]) > 1e-9 or xyz[1] >= 0:
        raise ValueError("vendor joint6 geometry changed; review the d6 mapping")
    d6 = float(robot["d6_m"])
    if not math.isfinite(d6) or not 0.05 < d6 < 0.3:
        raise ValueError("invalid d6_m")
    origin.set("xyz", _numbers([0.0, -d6, 0.0]))

    ET.SubElement(root, "link", name="world")
    mount = ET.SubElement(root, "joint", name="mount", type="fixed")
    ET.SubElement(mount, "parent", link="world")
    ET.SubElement(mount, "child", link="base_link")
    ET.SubElement(
        mount,
        "origin",
        xyz=_numbers(finite_vector(robot["mount_xyz_m"], 3, "mount_xyz_m")),
        rpy=_numbers(finite_vector(robot["mount_rpy_rad"], 3, "mount_rpy_rad")),
    )
    ET.SubElement(root, "link", name="tcp")
    tool = ET.SubElement(root, "joint", name="flange_to_tcp", type="fixed")
    ET.SubElement(tool, "parent", link="Link6")
    ET.SubElement(tool, "child", link="tcp")
    ET.SubElement(
        tool,
        "origin",
        xyz=_numbers(finite_vector(robot["tool_xyz_m"], 3, "tool_xyz_m")),
        rpy=_numbers(finite_vector(robot["tool_rpy_rad"], 3, "tool_rpy_rad")),
    )

    system = ET.SubElement(root, "ros2_control", name="RM65System", type="system")
    hardware = ET.SubElement(system, "hardware")
    if mode == "sim" and simulator == "mujoco":
        if not mujoco_model or not Path(mujoco_model).is_absolute():
            raise ValueError("MuJoCo needs an absolute model path")
        ET.SubElement(hardware, "plugin").text = "mujoco_ros2_control/MujocoSystemInterface"
        for name, value in {
            "mujoco_model": mujoco_model,
            "headless": str(headless).lower(),
            "sim_speed_factor": "1.0",
            "initial_keyframe": "home",
        }.items():
            ET.SubElement(hardware, "param", name=name).text = value
    elif mode == "sim":
        ET.SubElement(hardware, "plugin").text = "mock_components/GenericSystem"
        ET.SubElement(hardware, "param", name="calculate_dynamics").text = "true"
    else:
        ET.SubElement(hardware, "plugin").text = "topic_based_ros2_control/TopicBasedSystem"
        for name, value in {
            "joint_commands_topic": "/rm65/transport/joint_commands",
            "joint_states_topic": "/rm65/transport/joint_states",
            "trigger_joint_command_threshold": "-1",
        }.items():
            ET.SubElement(hardware, "param", name=name).text = value
    for index, name in enumerate(JOINT_NAMES):
        joint = ET.SubElement(system, "joint", name=name)
        ET.SubElement(joint, "command_interface", name="position")
        position = ET.SubElement(joint, "state_interface", name="position")
        ET.SubElement(position, "param", name="initial_value").text = str(home[index])
        velocity = ET.SubElement(joint, "state_interface", name="velocity")
        ET.SubElement(velocity, "param", name="initial_value").text = "0.0"
    return ET.tostring(root, encoding="unicode")


def build_semantic(vendor_srdf: str | Path, config: dict) -> str:
    root = ET.Element("robot", name="rm65_servo")
    group = ET.SubElement(root, "group", name="rm65_arm")
    ET.SubElement(group, "chain", base_link="base_link", tip_link="tcp")
    home = ET.SubElement(root, "group_state", name="home", group="rm65_arm")
    for name, value in zip(JOINT_NAMES, config["model"]["home_joint_rad"]):
        ET.SubElement(home, "joint", name=name, value=str(value))
    for pair in ET.parse(vendor_srdf).getroot().findall("disable_collisions"):
        # Manufacturer SRDF matrix, not a freshly sampled safety certification.
        root.append(pair)
    ET.SubElement(root, "disable_collisions", link1="Link6", link2="tcp", reason="Adjacent")
    return ET.tostring(root, encoding="unicode")
