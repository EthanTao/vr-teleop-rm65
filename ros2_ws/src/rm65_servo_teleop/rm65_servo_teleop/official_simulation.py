"""Standalone MuJoCo physics for the same official follower; no robot interfaces.

Uses the shared patched URDF, finite-force actuators and measured MuJoCo q/TCP.
The engine/runtime is isolated from the existing ROS Servo environment.
"""

from __future__ import annotations

import argparse
from collections import Counter, deque
import hashlib
import json
import math
from pathlib import Path
import sys
import time
import xml.etree.ElementTree as ET

import numpy as np
import yaml

from .input_adapter import Pose, quaternion_multiply, rotvec_quaternion
from .model import JOINT_NAMES, build_description, build_semantic
from .mujoco_scene import write_scene
from .official_follow import OfficialFollower, pose_errors
from .official_follow_runner import load_follow_config
from .official_sdk import OfficialAlgo


class Simulation:
    def __init__(self, model, data, solver, config):
        import mujoco
        self.mujoco, self.model, self.data, self.solver, self.config = mujoco, model, data, solver, config
        self.follower = OfficialFollower(config, solver)
        self.qindex = [model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n)] for n in JOINT_NAMES]
        self.uindex = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, n) for n in JOINT_NAMES]
        self.tcp = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "tcp")
        self.base = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base_link")
        self.marker = model.body_mocapid[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "official_goal")]
        if min(self.qindex + self.uindex + [self.tcp, self.base, self.marker]) < 0:
            raise ValueError("simulation requires the six named joints, actuators, base, tcp and goal marker")
        mujoco.mj_forward(model, data)
        self.anchor = self.measured_pose()
        self.counts, self.errors = Counter(), deque(maxlen=5000)
        self.result = None
        self.last_control = -math.inf
        self.closed_loop_samples = 0

    def measured_pose(self):
        base_rotation = self.data.xmat[self.base].reshape(3, 3)
        rotation = base_rotation.T @ self.data.xmat[self.tcp].reshape(3, 3)
        quaternion = np.zeros(4)
        self.mujoco.mju_mat2Quat(quaternion, rotation.ravel())
        return Pose(base_rotation.T @ (self.data.xpos[self.tcp] - self.data.xpos[self.base]),
                    quaternion[[1, 2, 3, 0]])

    def control(self):
        now = float(self.data.time)
        # Four seconds per axis, all six axes, then a released interval. The
        # same solver/policy is used; no simulator-only IK or teleportation.
        cycle = now % 26.0
        enabled = .10 <= cycle < 24.1
        phase = max(cycle - .10, 0.0)
        axis = min(int(phase // 4.0), 5)
        amount = math.sin(2 * math.pi * (phase % 4.0) / 4.0) if enabled else 0.0
        position, quaternion = self.anchor.xyz.copy(), self.anchor.quaternion_xyzw.copy()
        if axis < 3:
            position[axis] += .025 * amount
        else:
            rotation = np.zeros(3)
            rotation[axis - 3] = .08 * amount
            quaternion = quaternion_multiply(rotvec_quaternion(rotation), quaternion)
        desired = Pose(position, quaternion)
        measured = self.measured_pose()
        q = self.data.qpos[self.qindex].copy()
        result = self.follower.step(enabled=enabled, desired=desired, target_at=now,
                                    measured_pose=measured, measured_joints=q, feedback_at=now, now=now)
        self.result = result
        self.counts[result.state] += 1
        self.errors.append(pose_errors(desired, measured))
        self.closed_loop_samples += 1
        self.data.ctrl[self.uindex] = q if result.command_rad is None else result.command_rad
        # A fault holds only the simulation; the production policy stays latched
        # until the next explicit released interval in the demo input.
        world_rotation = self.data.xmat[self.base].reshape(3, 3)
        self.data.mocap_pos[self.marker] = self.data.xpos[self.base] + world_rotation @ desired.xyz
        rotation = self.solver_matrix(desired)
        wxyz = np.zeros(4)
        self.mujoco.mju_mat2Quat(wxyz, (world_rotation @ rotation).ravel())
        self.data.mocap_quat[self.marker] = wxyz

    @staticmethod
    def solver_matrix(pose):
        from .ur_follow_frames import quat_to_matrix
        return np.asarray(quat_to_matrix(pose.quaternion_xyzw))

    def advance(self, seconds):
        steps = max(1, round(seconds / self.model.opt.timestep))
        for _ in range(steps):
            if self.data.time - self.last_control >= self.config.period_s - 1e-8:
                self.last_control = float(self.data.time)
                self.control()
            self.mujoco.mj_step(self.model, self.data)
            if not np.isfinite(self.data.qpos).all() or not np.isfinite(self.data.qvel).all():
                raise RuntimeError("nonfinite_simulation_feedback")

    def summary(self):
        errors = np.asarray(list(self.errors))
        return {"engine": self.mujoco.__version__, "physics_time_s": float(self.data.time),
                "states": dict(self.counts), "samples": self.closed_loop_samples,
                "error_window_samples": len(errors),
                "tcp_position_error_p95_m": float(np.percentile(errors[:, 0], 95)) if len(errors) else None,
                "last": None if self.result is None else self.result.record(),
                "measured_joint_rad": self.data.qpos[self.qindex].tolist(),
                "hardware_commands_sent": 0, "feedback": "MuJoCo physical joints and TCP",
                "dynamics_identified": False}


def build_scene(root, shared, physics, vendor, directory):
    import mujoco
    description = build_description(vendor / "rm_description/urdf/rm_65.urdf", shared, "sim")
    semantic = build_semantic(root / "build/validation/moveit_servo_20261025/vendor_model/rm_65_description.srdf", shared)
    path = write_scene(description, semantic, shared, physics, directory,
                       package_roots={"rm_description": vendor / "rm_description"})
    scene = ET.parse(path)
    body = scene.getroot().find("worldbody")
    marker = ET.SubElement(body, "body", name="official_goal", mocap="true")
    ET.SubElement(marker, "geom", type="sphere", size="0.010", rgba="1 .35 .08 .7", contype="0", conaffinity="0", mass="0")
    tcp = scene.getroot().find(".//body[@name='tcp']")
    ET.SubElement(tcp, "site", name="actual_tcp", size="0.007", rgba="0 .85 1 1")
    path.write_text(ET.tostring(scene.getroot(), encoding="unicode"), encoding="utf-8")
    metadata_path = directory / "model.json"
    metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
    metadata.update(scene_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                    control_clock="standalone_simulation_time", physics_clock="MuJoCo data.time",
                    controller="official_sdk_ik_follow", target_marker_added=True)
    metadata_path.write_text(json.dumps(metadata, indent=2), encoding="utf-8")
    model = mujoco.MjModel.from_xml_path(str(path))
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, 0)
    mujoco.mj_forward(model, data)
    return model, data


def main(argv=None, *, root):
    import mujoco
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--web", action="store_true", help="MuJoCo 4 Studio browser viewer on loopback only")
    parser.add_argument("--port", type=int, default=8094)
    parser.add_argument("--headless-seconds", type=float, default=0)
    parser.add_argument("--sdk-root", type=Path, default=root / "build/reference/RM_API2-a810b9f/Python")
    parser.add_argument("--vendor-root", type=Path, default=root / "build/reference/rm-robot-61db3c3")
    parser.add_argument("--output-dir", type=Path, default=root / "build/validation/official-follow-web4-20261112")
    args = parser.parse_args(argv)
    sys.path.insert(0, str(args.sdk_root.resolve()))
    own = root / "ros2_ws/src/rm65_servo_teleop"
    shared, config = load_follow_config(own / "config/teleop.yaml", own / "config/official_follow.yaml")
    physics = yaml.safe_load((own / "config/mujoco.yaml").read_text(encoding="utf-8"))
    args.output_dir.mkdir(parents=True, exist_ok=True)
    model, data = build_scene(root, shared, physics, args.vendor_root, args.output_dir)
    solver = OfficialAlgo(shared["model"]["tool_xyz_m"], shared["model"]["tool_rpy_rad"], d6_m=shared["model"]["d6_m"])
    simulation = Simulation(model, data, solver, config)
    error = pose_errors(simulation.measured_pose(), solver.forward(data.qpos[simulation.qindex]))
    if error[0] > config.frame_position_tolerance_m or error[1] > config.frame_rotation_tolerance_rad:
        raise RuntimeError(f"official SDK / simulation frames disagree: {error}")
    print(json.dumps({"initial_fk_errors_m_rad": error, "engine": mujoco.__version__}), flush=True)
    status = args.output_dir / "status.json"
    if args.headless_seconds > 0:
        while data.time < args.headless_seconds:
            simulation.advance(config.period_s)
        result = simulation.summary()
        status.write_text(json.dumps(result, indent=2), encoding="utf-8")
        print(json.dumps(result), flush=True)
        return 1 if result["states"].get("fault", 0) else 0
    if not args.web or mujoco.__version__ != "4.0.0":
        raise ValueError("Use --web with the isolated MuJoCo 4.0.0 runtime, or --headless-seconds")
    from .official_simulation_web import serve
    serve(simulation, status, args.port, root / "build/runtime/official-follow-web-assets")
    return 0
