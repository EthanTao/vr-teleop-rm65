"""Run the standalone RGB-D preparation/capture tool, without arm I/O."""

from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "ros2_ws/src/rm65_servo_teleop"))
from rm65_servo_teleop.camera_tools import main

if __name__ == "__main__":
    raise SystemExit(main())
