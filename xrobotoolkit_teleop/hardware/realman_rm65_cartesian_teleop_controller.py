"""Compatibility import for the historical movel controller.

New hardware callers must use realman_rm65_safe_teleop_controller.
The legacy implementation does not provide the current UDP safety chain.
"""

from xrobotoolkit_teleop.hardware.legacy.cartesian_teleop_controller import RealmanRM65CartesianTeleopController

__all__ = ["RealmanRM65CartesianTeleopController"]
