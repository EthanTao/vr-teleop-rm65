"""Compatibility entry for the renamed RM65 safe hardware CLI.

Use teleop_realman_rm65_safe_hardware.py for new commands.
"""

import tyro

if __package__:
    from .teleop_realman_rm65_safe_hardware import main
else:
    from teleop_realman_rm65_safe_hardware import main


if __name__ == "__main__":
    tyro.cli(main)
