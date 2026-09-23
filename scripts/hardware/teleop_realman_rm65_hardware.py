"""Compatibility CLI for historical dry-run mapping diagnostics.

Live motion remains disabled inside main, before connecting to the robot.
Use teleop_realman_rm65_safe_hardware.py for the supported safety chain.
"""

import tyro

from xrobotoolkit_teleop.hardware.legacy.single_file_teleop import Args, main

if __name__ == "__main__":
    main(tyro.cli(Args))
