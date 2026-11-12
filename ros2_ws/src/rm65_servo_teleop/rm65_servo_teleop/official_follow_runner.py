"""Compute-only JSONL replay of recorded targets and independently measured feedback."""

from __future__ import annotations

import argparse
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import sys

import numpy as np
import yaml

from .input_adapter import Pose
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


def run_rows(rows, solver, config, output):
    follower = OfficialFollower(config, solver)
    states, times, accepted = {}, [], 0
    maximum_step = np.zeros(6)
    previous = None
    for number, item in enumerate(rows, 1):
        row = item
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
        else:
            previous = None
        output.write(json.dumps({"row": number, "time_s": now, "feedback": "input_record",
                                 **result.record()}, allow_nan=False) + "\n")
    return {
        "samples": len(times), "states": states, "accepted": accepted,
        "compute_p95_s": float(np.percentile(times, 95)) if times else 0.0,
        "compute_max_s": max(times, default=0.0), "maximum_adjacent_candidate_step_rad": maximum_step.tolist(),
        "hardware_commands_sent": 0, "feedback": "input_record",
    }


def main(argv=None, *, default_model_config=None, default_follow_config=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model-config", type=Path, default=default_model_config,
                        required=default_model_config is None)
    parser.add_argument("--follow-config", type=Path, default=default_follow_config,
                        required=default_follow_config is None)
    parser.add_argument("--sdk-root", type=Path, help="Official SDK Python directory containing Robotic_Arm")
    parser.add_argument("--input", type=Path, required=True, help="JSONL with explicit target and measured feedback")
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
        with args.input.open(encoding="utf-8") as source_file:
            rows = (json.loads(line) for line in source_file if line.strip())
            summary = run_rows(rows, solver, config, output)
        output.write(json.dumps({"summary": summary}, allow_nan=False) + "\n")
    print(json.dumps(summary, ensure_ascii=False))
    return 1 if not summary["accepted"] or summary["states"].get("fault", 0) else 0


if __name__ == "__main__":
    raise SystemExit(main())
