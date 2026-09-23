"""Verify JointConstraints fix."""
import placo
from xrobotoolkit_teleop.simulation.joint_constraints import JointConstraints

r = placo.RobotWrapper("/workspace/assets/realman/RM65-official/urdf/RM65-official-arm.urdf")
jc = JointConstraints(r, dt=0.01)
print("OK: lower", jc.lower_limits[:3])
print("OK: upper", jc.upper_limits[:3])
print("All joint limits read successfully!")
