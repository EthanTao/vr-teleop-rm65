"""Run the repository's offline ROH demo or explicit read-only probe."""

from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "ros2_ws/src/rm65_servo_teleop"))

from rm65_servo_teleop.hand_tools import main

if __name__ == "__main__":
    raise SystemExit(main())
