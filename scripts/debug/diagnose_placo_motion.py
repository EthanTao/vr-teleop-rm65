"""Observe the existing Placo loop; never imports a hardware controller.

Run from the repository root. Defaults match the ordinary Placo entrypoint.
"""
import json
import os
import runpy
import time

import numpy as np


def main():
    os.environ["XROBO_DEBUG"] = "1"
    from xrobotoolkit_teleop.simulation.placo_teleop_controller import PlacoTeleopController

    original_run = PlacoTeleopController.run
    original_send = PlacoTeleopController._send_command
    state = {}

    def observed_send(controller):
        original_send(controller)
        now = time.monotonic()
        q = controller.placo_robot.state.q[7:].copy()
        actual = controller.placo_robot.get_T_world_frame("r_link6")[:3, 3].copy()
        target = controller.effector_task["right_hand"].T_world_frame[:3, 3].copy()
        active = bool(controller.active.get("right_hand", False))
        state["cycles"] += 1
        state["active_cycles"] += int(active)
        state["q_min"] = np.minimum(state["q_min"], q)
        state["q_max"] = np.maximum(state["q_max"], q)
        state["max_actual_delta"] = max(state["max_actual_delta"], float(np.linalg.norm(actual-state["initial_actual"])))
        state["max_target_delta"] = max(state["max_target_delta"], float(np.linalg.norm(target-state["initial_target"])))
        if now >= state["next_print"]:
            state["next_print"] = now + 1.0
            print("[PLACO-PROBE] " + json.dumps({
                "elapsed_s": round(now-state["started"], 2), "active": active,
                "joint_deg": np.rad2deg(q).round(3).tolist(),
                "actual_xyz_m": actual.round(5).tolist(),
                "target_xyz_m": target.round(5).tolist(),
            }), flush=True)
        if now-state["started"] >= 30.0:
            controller._stop_event.set()

    def observed_run(controller):
        q = controller.placo_robot.state.q[7:].copy()
        now = time.monotonic()
        state.update(started=now, next_print=now, cycles=0, active_cycles=0,
                     q_min=q.copy(), q_max=q.copy(), max_actual_delta=0.0,
                     max_target_delta=0.0,
                     initial_actual=controller.placo_robot.get_T_world_frame("r_link6")[:3, 3].copy(),
                     initial_target=controller.effector_task["right_hand"].T_world_frame[:3, 3].copy())
        print("[PLACO-PROBE] READY: hold right Grip and move for 30 seconds", flush=True)
        try:
            original_run(controller)
        finally:
            print("[PLACO-PROBE-SUMMARY] " + json.dumps({
                "cycles": state["cycles"], "active_cycles": state["active_cycles"],
                "joint_range_deg": np.rad2deg(state["q_max"]-state["q_min"]).round(4).tolist(),
                "max_actual_delta_m": round(state["max_actual_delta"], 6),
                "max_target_delta_m": round(state["max_target_delta"], 6),
            }), flush=True)
            controller.xr_client.close()

    PlacoTeleopController.run = observed_run
    PlacoTeleopController._send_command = observed_send
    try:
        runpy.run_path("scripts/simulation/teleop_rm65_sim.py")["main"](ik_backend="placo")
    finally:
        PlacoTeleopController.run = original_run
        PlacoTeleopController._send_command = original_send


if __name__ == "__main__":
    main()
