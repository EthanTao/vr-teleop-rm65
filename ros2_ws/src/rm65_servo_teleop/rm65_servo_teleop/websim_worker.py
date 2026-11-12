"""Python 3.11 MuJoCo 4 worker; a private pipe connects ROS Python 3.10.

No generated trajectories or ROS/robot-device imports. EOF or a lost host
heartbeat closes the physics process instead of continuing with stale targets.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import select
import sys
import time

# This file runs under a different Python ABI from its installed ROS package.
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import numpy as np
import yaml

from rm65_servo_teleop.model import build_description, build_semantic
from rm65_servo_teleop.mujoco_scene import write_scene
from rm65_servo_teleop.websim_physics import Physics
from rm65_servo_teleop.websim_viewer import BrowserView


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    for key in ("model-config", "physics-config", "vendor", "semantic", "directory", "page", "assets"):
        parser.add_argument("--" + key, type=Path, required=True)
    parser.add_argument("--controller", choices=["official", "servo"], required=True)
    parser.add_argument("--address", choices=["127.0.0.1", "0.0.0.0"], default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8094)
    args = parser.parse_args(argv)
    shared = yaml.safe_load(args.model_config.read_text(encoding="utf-8"))
    physics_config = yaml.safe_load(args.physics_config.read_text(encoding="utf-8"))
    description = build_description(args.vendor / "urdf/rm_65.urdf", shared, "sim")
    semantic = build_semantic(args.semantic, shared)
    scene = write_scene(description, semantic, shared, physics_config, args.directory,
                        package_roots={"rm_description": args.vendor})
    physics = Physics(scene, shared)
    view = BrowserView(args.page, args.assets, physics.geometry(), address=args.address, port=args.port)
    last_command_stamp = None
    view.start()
    try:
        print(json.dumps({"ready": True, "engine": "4.0.0"}), flush=True)
        while True:
            # Only the owning ROS bridge can advance this worker.
            if not select.select([sys.stdin], [], [], .2)[0]:
                raise RuntimeError("ROS bridge heartbeat expired")
            line = sys.stdin.readline(65537)
            if not line:
                return 0
            if len(line) > 65536 or not line.endswith("\n"):
                raise ValueError("invalid private-pipe message")
            request = json.loads(line)
            command = request.get("command")
            if command is not None and command["stamp_s"] != last_command_stamp:
                physics.receive(command["names"], command["position"], command["stamp_s"], request["ros_now_s"])
                last_command_stamp = command["stamp_s"]
            physics.advance()
            state = physics.snapshot()
            metadata = request["view"]
            desired = metadata.pop("target", None)
            target = None if desired is None else np.asarray(desired, dtype=float)
            if target is not None and (target.shape != (3,) or not np.all(np.isfinite(target))):
                raise ValueError("invalid viewer target")
            state.update(metadata, controller=args.controller,
                         target_world=None if target is None else
                         (physics.data.xmat[physics.base].reshape(3, 3) @ target + physics.data.xpos[physics.base]).tolist(),
                         error_m=None if target is None else float(np.linalg.norm(target - physics.pose().xyz)))
            view.update(state)
            pose = physics.pose()
            print(json.dumps({"time": state["time"], "joints": state["joints"], "velocity": state["velocity"],
                              "pose": [*pose.xyz.tolist(), *pose.quaternion_xyzw.tolist()]}, allow_nan=False), flush=True)
    finally:
        view.close()


if __name__ == "__main__":
    raise SystemExit(main())
