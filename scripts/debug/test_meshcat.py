"""Minimal test: Placo IK + Meshcat without XR dependency."""
import time
import numpy as np
import placo
import meshcat.transformations as tf
from placo_utils.visualization import robot_viz

urdf = "/workspace/assets/realman/RM65-official/urdf/RM65-official-arm.urdf"
robot = placo.RobotWrapper(urdf)
solver = placo.KinematicsSolver(robot)
solver.dt = 0.01
solver.mask_fbase(True)

# Set initial joint angles
q_init = np.zeros(6)  # home position
robot.state.q[7:] = q_init
robot.update_kinematics()

# Add a frame task for end-effector
ee_target = robot.get_T_world_frame("r_link6")
frame_task = solver.add_frame_task("r_link6", ee_target)
frame_task.configure("ee", "soft", 1.0)

# Add manipulability task
manip = solver.add_manipulability_task("r_link6", "both", 1.0)
manip.configure("manip", "soft", 1e-2)

# Show in Meshcat
vis = robot_viz(robot)
print(f"Meshcat URL: {vis.viewer.url()}")

# Animate: move end-effector in a small circle
t0 = time.time()
while time.time() - t0 < 30:
    t = time.time() - t0
    # Small sinusoidal movement
    dx = 0.02 * np.sin(t * 0.5)
    dy = 0.02 * np.cos(t * 0.5)
    dz = 0.01 * np.sin(t * 1.0)

    target = ee_target.copy()
    target[:3, 3] += np.array([dx, dy, dz])
    frame_task.T_world_frame = target

    solver.solve(True)
    robot.update_kinematics()
    vis.display(robot.state.q)
    time.sleep(0.01)

print("Done.")
