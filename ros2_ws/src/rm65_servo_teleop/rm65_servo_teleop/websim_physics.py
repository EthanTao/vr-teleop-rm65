"""MuJoCo 4 physics only: ordered joint commands in, physical feedback out.

No target generator, IK, robot driver, device connection or qpos playback.
"""
from __future__ import annotations

import math
import time

import numpy as np

from .input_adapter import Pose
from .model import JOINT_NAMES
from .transport_guard import ordered_joints


class Physics:
    def __init__(self, scene, shared, *, engine=None):
        if engine is None:
            import mujoco as engine
        if engine.__version__ != "4.0.0":
            raise ValueError("browser simulation requires MuJoCo 4.0.0")
        self.engine = engine
        self.model = engine.MjModel.from_xml_path(str(scene))
        self.data = engine.MjData(self.model)
        self.lower = np.asarray(shared["model"]["joint_lower_rad"])
        self.upper = np.asarray(shared["model"]["joint_upper_rad"])
        self.qindex, self.vindex, self.aindex = [], [], []
        for name in JOINT_NAMES:
            joint = engine.mj_name2id(self.model, engine.mjtObj.mjOBJ_JOINT, name)
            actuator = engine.mj_name2id(self.model, engine.mjtObj.mjOBJ_ACTUATOR, name)
            if joint < 0 or actuator < 0:
                raise ValueError("missing physical joint or actuator")
            self.qindex.append(self.model.jnt_qposadr[joint])
            self.vindex.append(self.model.jnt_dofadr[joint])
            self.aindex.append(actuator)
        self.base = engine.mj_name2id(self.model, engine.mjtObj.mjOBJ_BODY, "base_link")
        self.tcp = engine.mj_name2id(self.model, engine.mjtObj.mjOBJ_BODY, "tcp")
        if min(self.base, self.tcp) < 0 or self.model.nq != 6 or self.model.nu != 6:
            raise ValueError("expected the six-axis shared RM65 model")
        engine.mj_resetDataKeyframe(self.model, self.data, 0)
        engine.mj_forward(self.model, self.data)
        self.command_at = None
        self.command_stamp = None
        self.command = self.joints.copy()

    @property
    def joints(self):
        return self.data.qpos[self.qindex].copy()

    @property
    def velocity(self):
        return self.data.qvel[self.vindex].copy()

    def pose(self):
        base = self.data.xmat[self.base].reshape(3, 3)
        rotation = base.T @ self.data.xmat[self.tcp].reshape(3, 3)
        quaternion = np.empty(4)
        self.engine.mju_mat2Quat(quaternion, rotation.ravel())
        return Pose(base.T @ (self.data.xpos[self.tcp] - self.data.xpos[self.base]), quaternion[[1, 2, 3, 0]])

    def receive(self, names, position, stamp_s, ros_now_s, *, now=None):
        now = time.monotonic() if now is None else now
        if not all(math.isfinite(t) for t in (stamp_s, ros_now_s, now)):
            raise ValueError("nonfinite command time")
        if not 0 <= ros_now_s - stamp_s <= .2:
            raise ValueError("stale or future simulation command")
        if self.command_stamp is not None and stamp_s <= self.command_stamp:
            raise ValueError("simulation command timestamp did not advance")
        q = np.asarray(ordered_joints(names, position))
        if np.any(q < self.lower) or np.any(q > self.upper):
            raise ValueError("simulation command outside shared joint bounds")
        self.command, self.command_at, self.command_stamp = q.copy(), now, stamp_s

    def advance(self, duration_s=.004, *, now=None):
        now = time.monotonic() if now is None else now
        if self.command_at is not None and not 0 <= now - self.command_at <= .2:
            raise RuntimeError("simulation command watchdog expired")
        self.data.ctrl[self.aindex] = self.command
        steps = round(duration_s / self.model.opt.timestep)
        if not 1 <= steps <= 20 or abs(steps * self.model.opt.timestep - duration_s) > 1e-9:
            raise ValueError("invalid physics integration interval")
        self.engine.mj_step(self.model, self.data, nstep=steps)
        if not np.all(np.isfinite(self.joints)) or not np.all(np.isfinite(self.velocity)):
            raise RuntimeError("nonfinite physical feedback")

    def snapshot(self):
        pose = self.pose()
        return {"time": float(self.data.time), "engine": self.engine.__version__,
                "tcp": pose.xyz.tolist(), "tcp_world": self.data.xpos[self.tcp].tolist(),
                "tcp_quaternion_xyzw": pose.quaternion_xyzw.tolist(),
                "joints": self.joints.tolist(), "velocity": self.velocity.tolist(),
                "geoms": [{"p": p.tolist(), "r": r.tolist()}
                          for p, r in zip(self.data.geom_xpos, self.data.geom_xmat)],
                "hardware_commands_sent": 0}

    def geometry(self):
        model = self.model
        meshes = []
        for i in range(model.nmesh):
            start, count = model.mesh_vertadr[i], model.mesh_vertnum[i]
            first, faces = model.mesh_faceadr[i], model.mesh_facenum[i]
            meshes.append({"vertices": model.mesh_vert[start:start + count].tolist(),
                           "faces": model.mesh_face[first:first + faces].tolist()})
        return {"meshes": meshes,
                "geoms": [{"id": i, "type": int(model.geom_type[i]), "mesh": int(model.geom_dataid[i]),
                           "size": model.geom_size[i].tolist(), "rgba": model.geom_rgba[i].tolist()}
                          for i in range(model.ngeom) if model.geom_group[i] == 1 or model.geom_type[i] == 0]}
