"""Minimal robot display test - no XR dependency."""
import sys
import os

# Force unbuffered output
sys.stdout.reconfigure(line_buffering=True)

import numpy as np
import placo
from placo_utils.visualization import robot_viz

urdf = "/workspace/assets/realman/RM65-official/urdf/RM65-official-arm.urdf"
print(f"[TEST] Loading URDF: {urdf}")

robot = placo.RobotWrapper(urdf)
print(f"[TEST] Robot loaded. Joints: {robot.joint_names()}")

solver = placo.KinematicsSolver(robot)
solver.dt = 0.01
solver.mask_fbase(True)

# Set robot to home
q_init = np.zeros(6)
robot.state.q[7:] = q_init
robot.update_kinematics()

print("[TEST] Creating visualization...")
vis = robot_viz(robot)
url = vis.viewer.url()
print(f"[TEST] Meshcat URL: {url}")
print(f"[TEST] Open your browser to see the robot!")

# Display the robot
vis.display(robot.state.q)
print("[TEST] Robot displayed. Running animation loop...")

# Simple animation - wiggle joint 1
from time import sleep, time
t0 = time()
try:
    while time() - t0 < 120:  # run for 2 minutes
        t = time() - t0
        # Small movement
        q = robot.state.q.copy()
        q[7] = 0.2 * np.sin(t * 0.5)  # wiggle joint1
        robot.state.q = q
        robot.update_kinematics()

        # Compute IK
        vis.display(robot.state.q)
        sleep(0.01)
except KeyboardInterrupt:
    pass

print("[TEST] Done.")
