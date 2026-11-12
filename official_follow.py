#!/usr/bin/env python3
"""UR5-style RM65 official IK version: recorded-input replay, no robot I/O.

MoveIt Servo remains available through start.py. This entry never selects or
starts the current hardware driver and has no live execution option.
"""

from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT / "ros2_ws/src/rm65_servo_teleop"))

from rm65_servo_teleop.official_follow_runner import main  # noqa: E402

if __name__ == "__main__":
    raise SystemExit(main(
        default_model_config=ROOT / "ros2_ws/src/rm65_servo_teleop/config/teleop.yaml",
        default_follow_config=ROOT / "ros2_ws/src/rm65_servo_teleop/config/official_follow.yaml",
    ))
