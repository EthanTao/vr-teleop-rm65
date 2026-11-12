"""Generate the MuJoCo execution model from the SAME patched vendor URDF.

No IK or trajectory interpolation lives here. MuJoCo imports the URDF inertias,
joint origins, axes and geometry; finite position actuators execute JTC targets.
"""

from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

from .model import JOINT_NAMES, finite_vector
from .validation import model_digest


def _positive(value, name):
    value = float(value)
    if not math.isfinite(value) or value <= 0:
        raise ValueError(f"{name} must be finite and positive")
    return value


def write_scene(description: str, semantic: str, config: dict, physics: dict, directory: Path) -> Path:
    """Compile, enrich and revalidate a model in a caller-owned temporary folder."""
    import mujoco
    import numpy as np
    from ament_index_python.packages import get_package_share_directory

    timestep = _positive(physics["timestep_s"], "timestep_s")
    if timestep > 0.004:
        raise ValueError("physics timestep must not exceed the 250 Hz controller period")
    gains = finite_vector(physics["position_kp"], 6, "position_kp")
    damping = _positive(physics["damping_ratio"], "damping_ratio")
    if any(gain <= 0 for gain in gains) or type(physics["gravity_compensation"]) is not bool:
        raise ValueError("invalid simulator actuator parameters")
    root = ET.fromstring(description)
    hardware = root.find("ros2_control")
    if hardware is not None:
        root.remove(hardware)
    mesh_hashes = {}
    for mesh in root.iter("mesh"):
        filename = mesh.get("filename", "")
        if not filename.startswith("package://"):
            raise ValueError("vendor meshes must use explicit package:// references")
        package, relative = filename[len("package://") :].split("/", 1)
        share = Path(get_package_share_directory(package)).resolve()
        path = (share / relative).resolve()
        if not path.is_relative_to(share) or not path.is_file():
            raise ValueError(f"invalid vendor mesh: {filename}")
        mesh_hashes[filename] = hashlib.sha256(path.read_bytes()).hexdigest()
        mesh.set("filename", str(path))
    extension = ET.SubElement(root, "mujoco")
    ET.SubElement(extension, "compiler", fusestatic="false", discardvisual="false", balanceinertia="false")
    imported = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"))
    if imported.nq != 6 or imported.nv != 6:
        raise ValueError("the initial RM65 simulator must contain exactly six arm joints")
    directory.mkdir(parents=True, exist_ok=True)
    path = directory / "scene.xml"
    mujoco.mj_saveLastXML(str(path), imported)
    scene = ET.parse(path).getroot()
    option = scene.find("option")
    if option is None:
        option = ET.SubElement(scene, "option")
    option.attrib.update(timestep=str(timestep), integrator="implicitfast", gravity="0 0 -9.81")
    visual = ET.SubElement(scene, "visual")
    ET.SubElement(visual, "global", azimuth="135", elevation="-25")
    ET.SubElement(scene, "statistic", center="0 0 0.35", extent="0.85")
    world = scene.find("worldbody")
    ET.SubElement(world, "light", pos="0 -1 2", dir="0 0 -1", diffuse="0.8 0.8 0.8")
    ET.SubElement(world, "geom", name="floor", type="plane", size="2 2 0.1", pos="0 0 -0.025", rgba="0.25 0.3 0.35 1")
    for body in scene.iter("body"):
        if physics["gravity_compensation"]:
            body.set("gravcomp", "1")
    bodies = {body.get("name") for body in scene.iter("body")}
    contact = ET.SubElement(scene, "contact")
    exclusions = set()
    for pair in ET.fromstring(semantic).findall("disable_collisions"):
        names = tuple(sorted((pair.get("link1"), pair.get("link2"))))
        if all(name in bodies for name in names) and names not in exclusions:
            ET.SubElement(contact, "exclude", body1=names[0], body2=names[1])
            exclusions.add(names)
    actuators = ET.SubElement(scene, "actuator")
    limits = []
    for index, name in enumerate(JOINT_NAMES):
        limit = root.find(f"joint[@name='{name}']/limit")
        torque = _positive(limit.get("effort"), f"{name} vendor effort limit")
        limits.append(torque)
        ET.SubElement(
            actuators,
            "position",
            name=name,
            joint=name,
            kp=str(gains[index]),
            dampratio=str(damping),
            ctrllimited="true",
            ctrlrange=f"{limit.get('lower')} {limit.get('upper')}",
            forcelimited="true",
            forcerange=f"{-torque} {torque}",
        )
    home = finite_vector(config["model"]["home_joint_rad"], 6, "home_joint_rad")
    qpos = np.zeros(6)
    for index, name in enumerate(JOINT_NAMES):
        joint_id = mujoco.mj_name2id(imported, mujoco.mjtObj.mjOBJ_JOINT, name)
        if joint_id < 0 or imported.jnt_type[joint_id] != mujoco.mjtJoint.mjJNT_HINGE:
            raise ValueError(f"missing revolute joint {name}")
        qpos[imported.jnt_qposadr[joint_id]] = home[index]
    ET.SubElement(
        ET.SubElement(scene, "keyframe"),
        "key",
        name="home",
        qpos=" ".join(map(str, qpos)),
        ctrl=" ".join(map(str, home)),
    )
    ET.indent(scene)
    path.write_text(ET.tostring(scene, encoding="unicode"), encoding="utf-8")
    compiled = mujoco.MjModel.from_xml_path(str(path))
    if compiled.nu != 6 or not np.all(compiled.actuator_forcelimited):
        raise ValueError("MuJoCo actuators must be six finite-force position servos")
    metadata = {
        "simulator": "mujoco",
        "mujoco_version": mujoco.__version__,
        "model_sha256": model_digest(description),
        "scene_sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
        "mesh_sha256": mesh_hashes,
        "physics": physics,
        "effort_limit_nm": limits,
        "dynamics_identified": False,
        "includes_hand_or_charger": False,
        "control_clock": "wall_ros_time",
        "physics_clock": "/clock (diagnostic and progress watchdog)",
    }
    (directory / "model.json").write_text(json.dumps(metadata, indent=2), encoding="utf-8")
    return path
