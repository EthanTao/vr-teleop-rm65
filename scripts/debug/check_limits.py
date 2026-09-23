"""Check get_joint_limits return type."""
import placo
r = placo.RobotWrapper("/workspace/assets/realman/RM65-official/urdf/RM65-official-arm.urdf")
lim = r.get_joint_limits("joint1")
print(f"type: {type(lim)}")
print(f"dir: {[m for m in dir(lim) if not m.startswith('_')]}")
print(f"values: {lim}")
print(f"lower: {lim[0]}, upper: {lim[1]}")
# Also check the first few joint names
for jn in r.joint_names()[:3]:
    lim2 = r.get_joint_limits(jn)
    print(f"  {jn}: [{lim2[0]:.4f}, {lim2[1]:.4f}]")
