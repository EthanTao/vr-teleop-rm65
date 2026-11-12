#!/usr/bin/env python3
"""Run the compute-only RM65 official IK follower; never connect to a robot."""

from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "ros2_ws/src/rm65_servo_teleop"))

from rm65_servo_teleop.official_follow_runner import main  # noqa: E402

if __name__ == "__main__":
    raise SystemExit(main())
