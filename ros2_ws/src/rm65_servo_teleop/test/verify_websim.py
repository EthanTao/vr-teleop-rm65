"""Isolated ROS/MuJoCo acceptance fixture; never a user simulation input source.

Run only with RM65_WEBSIM_TEST_ONLY=1 inside a --network none test container.
All input packets retain synthetic labels. Feedback must come from MuJoCo 4.
"""
from __future__ import annotations

import argparse
from collections import deque
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time
import urllib.request

import numpy as np
import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Int8, String
from trajectory_msgs.msg import JointTrajectory


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--controller", choices=["servo", "official"], required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    if os.environ.get("RM65_WEBSIM_TEST_ONLY") != "1":
        raise ValueError("this fixture requires the explicit isolated-test environment")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    if args.output.exists():
        raise FileExistsError(args.output)
    logs = args.output.with_suffix(".log")
    output = logs.open("x", encoding="utf-8")
    process = subprocess.Popen(["ros2", "launch", "rm65_servo_teleop", "pico_web.launch.py",
                                "controller:=" + args.controller, "input_source:=external", "test_only:=true"],
                               stdout=output, stderr=subprocess.STDOUT, start_new_session=True)
    rclpy.init()

    class Probe(Node):
        def __init__(self):
            super().__init__("websim_test_fixture")
            self.publisher = self.create_publisher(String, "/rm65/xr/raw",
                                                   QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
            self.inputs, self.frames, self.status = [], [], []
            self.commands, self.trajectories, self.servo_status = deque(maxlen=20), deque(maxlen=20), deque(maxlen=20)
            self.grip, self.x, self.rx = 0.0, 0.0, 0.0
            self.publishing = True
            self.timestamp = 0
            self.create_subscription(String, "/rm65/input_status", lambda m: self.inputs.append(json.loads(m.data)), 10)
            self.create_subscription(JointState, "/joint_states", lambda m: self.frames.append(list(m.position)), 10)
            self.create_subscription(JointState, "/rm65/websim/joint_commands", lambda m: self.commands.append(list(m.position)), 10)
            self.create_subscription(JointTrajectory, "/rm65_arm_controller/joint_trajectory",
                                     lambda m: self.trajectories.append([list(p.positions) for p in m.points]), 10)
            self.create_subscription(Int8, "/rm65/servo/status", lambda m: self.servo_status.append(m.data), 10)
            topic = "/rm65/websim/controller_status" if args.controller == "official" else "/rm65/pose_tracking/status"
            self.create_subscription(String, topic, lambda m: self.status.append(json.loads(m.data)), 10)
            self.create_timer(.02, self.publish_packet)

        def publish_packet(self):
            if not self.publishing:
                return
            self.timestamp = max(self.timestamp + 1, time.monotonic_ns())
            value = {"Input": 1, "timeStampNs": self.timestamp, "_rm65_mock": True,
                     "Controller": {"right": {"status": 3, "grip": self.grip,
                         "pose": [self.x, 0, 0, math.sin(self.rx / 2), 0, 0, math.cos(self.rx / 2)]}}}
            self.publisher.publish(String(data=json.dumps({"functionName": "Tracking", "value": value, "_rm65_mock": True})))

    probe = Probe()
    samples = []

    def spin(seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            if process.poll() is not None:
                raise RuntimeError(f"web simulation exited {process.returncode}; inspect {logs}")
            rclpy.spin_once(probe, timeout_sec=.005)

    def state():
        with urllib.request.urlopen("http://127.0.0.1:8094/state", timeout=2) as response:
            value = json.load(response)
        assert value["engine"] == "4.0.0" and value["controller"] == args.controller
        assert value["hardware_commands_sent"] == 0
        samples.append(value)
        return value

    def enabled():
        return bool(probe.inputs and probe.inputs[-1]["enabled"])

    try:
        deadline = time.monotonic() + 60
        while time.monotonic() < deadline:
            spin(.1)
            tracking_ready = (args.controller == "official" or
                              (probe.status and probe.status[-1].get("state") == "grip_released" and
                               all(probe.status[-1].get(key) is False for key in
                                   ("active", "requested", "release_required", "restart_required"))))
            if tracking_ready and probe.inputs and probe.inputs[-1]["reason"] == "released" and len(probe.frames) > 10:
                try:
                    before = state()
                    break
                except (OSError, KeyError):
                    pass
        else:
            raise AssertionError("physical feedback and release did not become ready")
        # Startup is stationary until the test explicitly supplies a Grip edge.
        spin(.4)
        idle = state()
        assert np.linalg.norm(np.asarray(idle["tcp"]) - before["tcp"]) < .0001
        config = yaml.safe_load(Path("/opt/rm65_ws/src/rm65_servo_teleop/config/teleop.yaml").read_text())
        assert np.max(np.abs(np.asarray(idle["joints"]) - config["model"]["home_joint_rad"])) < .001
        probe.grip = .8
        spin(.5)
        assert enabled(), probe.inputs[-1]
        anchor = state()
        for index in range(20):
            probe.x = (index + 1) * .0002
            spin(.02)
        spin(1.2)
        moved = state()
        displacement = float(np.linalg.norm(np.asarray(moved["tcp"]) - anchor["tcp"]))
        assert displacement > .0003, ("no physical translation", displacement, probe.status[-5:])
        for index in range(20):
            probe.rx = (index + 1) * .001
            spin(.02)
        spin(1.0)
        rotated = state()
        a, b = np.asarray(moved["tcp_quaternion_xyzw"]), np.asarray(rotated["tcp_quaternion_xyzw"])
        rotation = 2 * math.acos(float(np.clip(abs(np.dot(a, b)), 0, 1)))
        assert rotation > .001, ("no physical rotation", rotation, probe.status[-5:])
        probe.grip = 0.0
        spin(.5)
        assert not enabled()
        released = state()
        spin(.5)
        settled = state()
        assert np.linalg.norm(np.asarray(settled["tcp"]) - released["tcp"]) < .001
        # Re-engage, then lose the advancing device samples without a release.
        probe.grip = .8
        spin(.4)
        assert enabled(), probe.inputs[-1]
        probe.publishing = False
        spin(.4)
        assert not enabled() and probe.inputs[-1]["reason"] == "xr_timeout", probe.inputs[-1]
        probe.publishing = True
        spin(.4)
        assert not enabled(), "held Grip resumed after input loss"
        probe.grip = 0.0
        spin(.3)
        probe.grip = .8
        spin(.4)
        assert enabled(), "fresh release and Grip edge did not recover"
        result = {"controller": args.controller, "engine": "4.0.0", "synthetic_input": True,
                  "hardware_commands_sent": 0, "joint_feedback_samples": len(probe.frames),
                  "translation_m": displacement, "rotation_rad": rotation,
                  "stationary_until_grip": True, "release_stopped": True,
                  "timeout_disarmed": True, "held_grip_did_not_resume": True,
                  "fresh_release_recovered": True, "samples": samples}
        args.output.write_text(json.dumps(result, indent=2), encoding="utf-8")
        print(json.dumps({k: v for k, v in result.items() if k != "samples"}), flush=True)
    except BaseException as error:
        args.output.with_suffix(".failure.json").write_text(json.dumps({"error": str(error), "samples": samples,
            "joint_feedback": probe.frames[-10:], "input_status": probe.inputs[-10:], "controller_status": probe.status[-10:],
            "joint_commands": list(probe.commands), "trajectories": list(probe.trajectories),
            "servo_status": list(probe.servo_status)}, indent=2), encoding="utf-8")
        raise
    finally:
        probe.grip = 0
        probe.publish_packet()
        probe.destroy_node()
        rclpy.shutdown()
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=12)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
                process.wait(timeout=5)
        output.close()


if __name__ == "__main__":
    main()
