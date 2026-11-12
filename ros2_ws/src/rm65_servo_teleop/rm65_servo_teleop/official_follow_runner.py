"""Compute-only JSONL replay of measured targets or an explicitly synthetic demo."""

from __future__ import annotations

import argparse
from dataclasses import asdict
import hashlib
import json
import math
from pathlib import Path
import sys

import numpy as np
import yaml

from .input_adapter import Pose, quaternion_multiply, rotvec_quaternion
from .official_follow import FollowConfig, OfficialFollower
from .official_sdk import OfficialAlgo


def load_follow_config(model_path, follow_path):
    shared = yaml.safe_load(Path(model_path).read_text(encoding="utf-8"))
    options = yaml.safe_load(Path(follow_path).read_text(encoding="utf-8"))
    model = shared["model"]
    config = FollowConfig(
        joint_lower=tuple(model["joint_lower_rad"]), joint_upper=tuple(model["joint_upper_rad"]),
        joint_velocity=tuple(model["joint_velocity_rad_s"]), **options)
    return shared, config


def demo_rows(solver, joints, config, count=400):
    """Synthetic, ideal one-command-delay feedback; not a robot dynamics model."""
    initial = np.asarray(joints, dtype=float)
    anchor = solver.forward(initial)
    measured = initial.copy()
    latest = initial.copy()
    for index in range(count):
        now = index * config.period_s
        # Release at start/end, preserve Grip for the middle movement and hold.
        enabled = 1 <= index < count - 5
        xyz, quaternion = anchor.xyz.copy(), anchor.quaternion_xyzw.copy()
        phase = max(index - 3, 0)
        axis, local = min(phase // 60, 5), phase % 60
        amount = math.sin(2 * math.pi * local / 60) if phase < 360 else 0.0
        if axis < 3:
            xyz[axis] += 0.006 * amount
        else:
            rotation = np.zeros(3)
            rotation[axis - 3] = 0.02 * amount
            quaternion = quaternion_multiply(rotvec_quaternion(rotation), quaternion)
        feedback = solver.forward(measured)
        yield {
            "time_s": now, "enabled": enabled,
            "target_pose_xyzw": [*xyz.tolist(), *quaternion.tolist()],
            "feedback_pose_xyzw": [*feedback.xyz.tolist(), *feedback.quaternion_xyzw.tolist()],
            "feedback_joint_rad": measured.tolist(),
        }, latest
        measured = latest.copy()


def run_rows(rows, solver, config, output, *, synthetic=False):
    follower = OfficialFollower(config, solver)
    states, times, accepted = {}, [], 0
    maximum_step = np.zeros(6)
    previous = None
    for number, item in enumerate(rows, 1):
        if synthetic:
            row, command_sink = item
        else:
            row, command_sink = item, None
        if type(row.get("enabled")) is not bool:
            raise ValueError(f"row {number}: enabled must be a JSON boolean")
        now = float(row["time_s"])
        result = follower.step(
            enabled=row["enabled"], desired=Pose.from_vector(row["target_pose_xyzw"]),
            target_at=float(row.get("target_time_s", now)),
            measured_pose=Pose.from_vector(row["feedback_pose_xyzw"]),
            measured_joints=row["feedback_joint_rad"], feedback_at=float(row.get("feedback_time_s", now)), now=now)
        states[result.state] = states.get(result.state, 0) + 1
        times.append(result.compute_s)
        accepted += result.accepted
        if result.command_rad is not None:
            command = np.asarray(result.command_rad)
            if previous is not None and result.state != "anchoring":
                maximum_step = np.maximum(maximum_step, np.abs(command - previous))
            previous = command
            if command_sink is not None:
                command_sink[:] = command
        else:
            previous = None
        output.write(json.dumps({"row": number, "time_s": now, "synthetic_feedback": synthetic,
                                 **result.record()}, allow_nan=False) + "\n")
    return {
        "samples": len(times), "states": states, "accepted": accepted,
        "compute_p95_s": float(np.percentile(times, 95)) if times else 0.0,
        "compute_max_s": max(times, default=0.0), "maximum_adjacent_candidate_step_rad": maximum_step.tolist(),
        "hardware_commands_sent": 0, "synthetic_feedback": synthetic,
    }


def main(argv=None, *, default_model_config=None, default_follow_config=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model-config", type=Path, default=default_model_config,
                        required=default_model_config is None)
    parser.add_argument("--follow-config", type=Path, default=default_follow_config,
                        required=default_follow_config is None)
    parser.add_argument("--sdk-root", type=Path, help="Official SDK Python directory containing Robotic_Arm")
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--input", type=Path, help="JSONL with explicit target and measured feedback")
    source.add_argument("--demo", action="store_true", help="Official FK/IK with clearly synthetic feedback")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    if args.sdk_root:
        sys.path.insert(0, str(args.sdk_root.resolve()))
    shared, config = load_follow_config(args.model_config, args.follow_config)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    # Reserve the output before loading native code; never overwrite prior evidence.
    with args.output.open("x", encoding="utf-8") as output:
        solver = OfficialAlgo(shared["model"]["tool_xyz_m"], shared["model"]["tool_rpy_rad"],
                              d6_m=shared["model"]["d6_m"])
        metadata = {"kind": "official_follow_compute_only", "sdk": solver.identity,
                    "configuration": asdict(config), "model_configuration": shared["model"],
                    "config_sha256": {str(p): hashlib.sha256(p.read_bytes()).hexdigest()
                                      for p in (args.model_config, args.follow_config)}}
        output.write(json.dumps(metadata, allow_nan=False) + "\n")
        if args.demo:
            rows = demo_rows(solver, shared["model"]["home_joint_rad"], config)
            summary = run_rows(rows, solver, config, output, synthetic=True)
        else:
            with args.input.open(encoding="utf-8") as source_file:
                rows = (json.loads(line) for line in source_file if line.strip())
                summary = run_rows(rows, solver, config, output)
        output.write(json.dumps({"summary": summary}, allow_nan=False) + "\n")
    print(json.dumps(summary, ensure_ascii=False))
    return 1 if not summary["accepted"] or summary["states"].get("fault", 0) else 0


if __name__ == "__main__":
    raise SystemExit(main())
