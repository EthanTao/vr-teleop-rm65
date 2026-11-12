# Adapted from HDU-EILab/ur5-teleop, commit 8691bfae256d9f48db60eeeca56e243b043d1fea.
# Copyright (c) 2026 HDU-EILab. MIT; see ../licenses/ur5-teleop-MIT.txt in the source package.
"""UR5 upstream target clipping and rotation-aware step limiting."""

from __future__ import annotations

import math

from . import ur_follow_frames as frames

Pose6 = tuple[float, float, float, float, float, float]

def clip_pose(pose, workspace_min, workspace_max) -> Pose6:
    """把 pose6 的位置裁进 XYZ 盒；姿态原样保留。"""
    values = tuple(float(value) for value in pose)
    if len(values) != 6:
        raise ValueError(f"位姿必须是 6 维 [x,y,z,rx,ry,rz]，收到 {len(values)} 维")
    return tuple(
        min(max(values[axis], float(workspace_min[axis])), float(workspace_max[axis]))
        if axis < 3 else values[axis]
        for axis in range(6)
    )

def limit_pose_step(previous, desired, max_pos_step_m: float, max_rot_step_rad: float) -> Pose6:
    """相对 ``previous`` 做每帧位置/转角限幅，返回不超过限幅的目标位姿。"""
    previous_values = tuple(float(value) for value in previous)
    desired_values = tuple(float(value) for value in desired)

    step = [desired_values[i] - previous_values[i] for i in range(3)]
    distance = math.sqrt(sum(value * value for value in step))
    if distance > max_pos_step_m:
        scale = max_pos_step_m / distance
        step = [value * scale for value in step]
    position = tuple(previous_values[i] + step[i] for i in range(3))

    previous_rotation = frames.rotvec_to_matrix(previous_values[3:])
    desired_rotation = frames.rotvec_to_matrix(desired_values[3:])
    relative = frames.mat_mul(desired_rotation, frames.mat_transpose(previous_rotation))
    rotvec = frames.matrix_to_rotvec(relative)
    angle = math.sqrt(sum(value * value for value in rotvec))
    if angle > max_rot_step_rad:
        scale = max_rot_step_rad / angle
        relative = frames.rotvec_to_matrix(tuple(value * scale for value in rotvec))
    rotation = frames.mat_mul(relative, previous_rotation)
    return position + frames.matrix_to_rotvec(rotation)
