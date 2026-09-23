"""Check placo RobotWrapper API for joint limit access."""
import placo

urdf = "/workspace/assets/realman/RM65-official/urdf/RM65-official-arm.urdf"
robot = placo.RobotWrapper(urdf)

# List all public methods/attrs
print("=== Public attributes ===")
for m in sorted(dir(robot)):
    if not m.startswith("_"):
        print(f"  {m}")

# Check joint names and model
print(f"\n=== Joint names ({len(robot.joint_names())}) ===")
for jn in robot.joint_names():
    print(f"  {jn}")

# Try to access model (pinocchio model)
if hasattr(robot, "model"):
    print(f"\n=== Pinocchio model ===")
    print(f"  nq = {robot.model.nq}")
    print(f"  nv = {robot.model.nv}")
    print(f"  joint names from model: {robot.model.names}")
    print(f"  lower limits: {robot.model.lowerPositionLimit}")
    print(f"  upper limits: {robot.model.upperPositionLimit}")

# Try joint() method
if hasattr(robot, "joint"):
    print(f"\n=== robot.joint() ===")
    j = robot.joint("joint1")
    print(f"  type: {type(j)}")
    print(f"  dir: {[m for m in dir(j) if not m.startswith('_')]}")
else:
    print("\n=== robot has NO 'joint' method ===")

# Check get_joint_limits
if hasattr(robot, "get_joint_limits"):
    print("\n=== get_joint_limits ===")
    print(robot.get_joint_limits())

# Check if there are limit-related methods
limit_methods = [m for m in dir(robot) if 'limit' in m.lower() or 'bound' in m.lower()]
if limit_methods:
    print(f"\n=== Limit-related methods: {limit_methods} ===")
