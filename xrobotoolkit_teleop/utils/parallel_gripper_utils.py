"""Helpers for mapping a normalized XR trigger to parallel-gripper joints."""

from __future__ import annotations

import math


def calc_parallel_gripper_position(open_pos: float, close_pos: float, trigger_value: float) -> float:
    """Interpolate between open and closed positions for an XR trigger."""
    values = (float(open_pos), float(close_pos), float(trigger_value))
    if not all(math.isfinite(value) for value in values):
        raise ValueError("gripper positions and trigger value must be finite")
    trigger = min(1.0, max(0.0, values[2]))
    return values[0] + trigger * (values[1] - values[0])
