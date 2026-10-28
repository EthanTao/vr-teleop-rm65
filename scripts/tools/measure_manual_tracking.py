#!/usr/bin/env python3
"""Measure the real manual simulation path inside the hardware-free ROS image.

Example (inside a --network none container, workspace mounted read-only):
  python3 /repo/scripts/tools/measure_manual_tracking.py --output /results/run-01
This tool sends only manual HTTP intent. FK is an observation of measured q;
it never publishes joint feedback, trajectories, or direct robot commands.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import re
import secrets
import signal
import shutil
import subprocess
import sys
import tempfile
import threading
import time
from urllib.request import ProxyHandler, Request, build_opener
import uuid


def protocol(name):
    phases = []

    def add(label, category, seconds, axes=None, pressed=False, axis=None):
        phases.append(
            dict(
                name=label,
                category=category,
                seconds=seconds,
                axes=axes or [0] * 6,
                pressed=pressed,
                axis=axis,
                cycle=0,
            )
        )

    add("idle_start", "idle", 2.0)
    axes_to_test = [1, 3] if name == "probe" else list(range(6))
    if name != "probe":
        axes_to_test += [None, None]
    for number, axis in enumerate(axes_to_test):
        direction = [0] * 6
        if axis is None:
            rotation = number == 7
            direction[3 if rotation else 0 : 6 if rotation else 3] = [1, -1, 1]
            label = "mixed_rotation" if rotation else "mixed_translation"
            duration = 0.5 if rotation else 0.6
        else:
            direction[axis] = 1
            label = ("x", "y", "z", "rx", "ry", "rz")[axis]
            duration = 0.6 if axis < 3 else 0.5
        add(label + "_arm", "arm", 0.2, pressed=True, axis=axis)
        add(label + "_out_move", "move", duration, direction, True, axis)
        add(label + "_out_hold", "hold", 6.0, pressed=True, axis=axis)
        add(label + "_back_move", "move", duration, [-v for v in direction], True, axis)
        add(label + "_back_hold", "hold", 6.0, pressed=True, axis=axis)
        add(label + "_release", "release", 1.0, axis=axis)
    add("release_arm", "arm", 0.2, pressed=True, axis=1)
    add("move_release", "move", 1.0, [0, 1, 0, 0, 0, 0], True, 1)
    add("release_stop", "release", 1.5, axis=1)
    add("dropout_arm", "arm", 0.2, pressed=True, axis=1)
    add("move_dropout", "move", 1.0, [0, -1, 0, 0, 0, 0], True, 1)
    add("dropout_stop", "dropout", 1.5, [0, -1, 0, 0, 0, 0], True, 1)
    add("idle_end", "idle", 10.0 if name == "full" else 3.0)
    return phases


def run(options):
    if sys.platform != "linux" or not Path("/.dockerenv").exists():
        raise RuntimeError("run this measurement inside the isolated Linux simulation container")
    output = options.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    os.environ["ROS_DOMAIN_ID"] = str(170 + secrets.randbelow(40))
    import numpy as np
    import rclpy
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from ament_index_python.packages import get_package_share_directory
    from control_msgs.msg import JointTrajectoryControllerState
    from geometry_msgs.msg import PoseStamped, TwistStamped
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Int8, String
    from rosgraph_msgs.msg import Clock
    from trajectory_msgs.msg import JointTrajectory
    import mujoco
    import yaml
    from rm65_servo_teleop.model import JOINT_NAMES, build_description, build_semantic
    from rm65_servo_teleop.mujoco_scene import write_scene

    share = Path(get_package_share_directory("rm65_servo_teleop"))
    config = yaml.safe_load((share / "config/teleop.yaml").read_text())
    physics = yaml.safe_load((share / "config/mujoco.yaml").read_text())
    vendor = Path(get_package_share_directory("rm_description")) / "urdf/rm_65.urdf"
    semantic_path = Path(get_package_share_directory("rm_65_config")) / "config/rm_65_description.srdf"
    description = build_description(vendor, config, "sim")
    semantic = build_semantic(semantic_path, config)
    model_dir = tempfile.TemporaryDirectory(prefix="rm65-measure-fk-")
    scene_path = write_scene(description, semantic, config, physics, Path(model_dir.name))
    model = mujoco.MjModel.from_xml_path(str(scene_path))
    data = mujoco.MjData(model)
    body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "tcp")
    base_body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base_link")
    joint_ids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n) for n in JOINT_NAMES]
    if min(body, base_body, *joint_ids) < 0:
        raise RuntimeError("measurement FK model is missing arm joints, base_link or tcp")
    qindices = [model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n)] for n in JOINT_NAMES]
    phases = protocol(options.protocol)
    metadata = dict(
        run_id=output.name,
        simulator=options.simulator,
        protocol=options.protocol,
        gui=options.gui,
        config=config,
        physics=physics,
        domain=os.environ["ROS_DOMAIN_ID"],
        source_sha=options.source_sha,
        started_utc=time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        config_sha256=hashlib.sha256((share / "config/teleop.yaml").read_bytes()).hexdigest(),
        controllers_sha256=hashlib.sha256((share / "config/controllers.yaml").read_bytes()).hexdigest(),
        description_sha256=hashlib.sha256(description.encode()).hexdigest(),
        engine=mujoco.__version__ if options.simulator == "mujoco" else "ros2_control/GenericSystem",
        observation_fk_engine="MuJoCo " + mujoco.__version__,
        measurement_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        measurement_log_storage="container-local temporary files, copied after measurement shutdown",
        requested_input_rates=dict(translation_m_s=0.02, rotation_rad_s=0.12),
        input_heartbeat_s=0.01,
        input_scheduler="independent HTTP heartbeat; phase changes serialized with sends",
        phases=phases,
        completed=False,
        failure=None,
        observation="Source-stamped /joint_states; read-only MuJoCo FK of measured joints; no feedback publisher",
        hold_semantics="pressed=True, axes=0: continue tracking fixed target; pressed=False: stop following",
    )
    (output / "meta.json").write_text(json.dumps(metadata, ensure_ascii=False, indent=2))
    # Writing a megabyte to the Windows bind mount can stall the observing
    # executor. Keep measurement I/O on local Linux storage; the normal journal
    # still uses its normal asynchronous writer and the requested results path.
    raw_path = Path(model_dir.name) / "raw.jsonl"
    log_path = Path(model_dir.name) / "stack.log"
    raw = raw_path.open("w", encoding="utf-8", buffering=1024 * 1024)
    child_log = log_path.open("w", encoding="utf-8")
    launch = [
        "ros2",
        "launch",
        "rm65_servo_teleop",
        "teleop.launch.py",
        "mode:=sim",
        "input_source:=manual",
        "simulator:=" + options.simulator,
        "manual_port:=8766",
        "rviz:=false",
        "foxglove:=false",
        "mujoco_viewer:=" + str(options.gui).lower(),
        "record:=true",
        "record_root:=" + str(output / "experiments"),
    ]
    metadata["launch"] = launch
    child = subprocess.Popen(launch, stdout=child_log, stderr=subprocess.STDOUT, start_new_session=True)
    rclpy.init()
    node = rclpy.create_node("manual_tracking_measurement")
    sensor = QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT)
    observed = dict(joints=0, targets=0, controller=0, last_joint_at=None, last_joint_stamp=None, bad=[], received=0)
    latest = dict(q=None, state=None)

    record_lock = threading.Lock()

    def record(kind, **values):
        with record_lock:
            raw.write(json.dumps(dict(kind=kind, host_s=time.monotonic(), **values), allow_nan=False) + "\n")
            observed["received"] += 1

    def stamp(header):
        return header.stamp.sec + header.stamp.nanosec * 1e-9

    def measured_tcp():
        # MuJoCo world may differ from the common planning frame base_link.
        base_rotation = data.xmat[base_body].reshape(3, 3)
        position = base_rotation.T @ (data.xpos[body] - data.xpos[base_body])
        rotation = base_rotation.T @ data.xmat[body].reshape(3, 3)
        quat = np.empty(4)
        mujoco.mju_mat2Quat(quat, rotation.reshape(-1))
        return position, np.r_[quat[1:], quat[0]]

    def joints(message):
        try:
            if (
                len(set(message.name)) != len(message.name)
                or len(message.position) != len(message.name)
                or len(message.velocity) != len(message.name)
            ):
                raise ValueError("invalid actual joint names or vector dimensions")
            indices = [list(message.name).index(n) for n in JOINT_NAMES]
            q = np.array([message.position[i] for i in indices])
            dq = np.array([message.velocity[i] for i in indices])
            if not np.isfinite(np.r_[q, dq]).all():
                raise ValueError("nonfinite actual joint state")
            source_stamp = stamp(message.header)
            receipt = node.get_clock().now().nanoseconds * 1e-9
            if source_stamp <= 0 or not -0.02 <= receipt - source_stamp <= 0.2:
                record("invalid_joint", stamp_s=source_stamp, receipt_ros_s=receipt, age_s=receipt - source_stamp)
                raise ValueError(
                    f"actual JointState source age invalid: age_s={receipt-source_stamp:.6f}; source_s={source_stamp:.9f}; receipt_s={receipt:.9f}"
                )
            previous_stamp = observed["last_joint_stamp"]
            if previous_stamp is not None and source_stamp < previous_stamp:
                raise ValueError("actual JointState source timestamp regressed")
            if source_stamp == previous_stamp:
                return
            data.qpos[qindices] = q
            mujoco.mj_forward(model, data)
            tcp_position, tcp_quaternion = measured_tcp()
            record(
                "joint",
                stamp_s=source_stamp,
                receipt_ros_s=receipt,
                q=q.tolist(),
                dq=dq.tolist(),
                tcp_p=tcp_position.tolist(),
                tcp_q=tcp_quaternion.tolist(),
            )
            observed["joints"] += 1
            observed["last_joint_at"] = time.monotonic()
            observed["last_joint_stamp"] = source_stamp
            latest["q"] = q
        except (ValueError, IndexError, TypeError) as error:
            observed["bad"].append(str(error))

    def pose(kind, message):
        p, q = message.pose.position, message.pose.orientation
        record(kind, stamp_s=stamp(message.header), p=[p.x, p.y, p.z], q=[q.x, q.y, q.z, q.w])
        if kind == "target":
            observed["targets"] += 1

    def controller(message):
        indices = [list(message.joint_names).index(n) for n in JOINT_NAMES]
        values = {}
        for prefix, point in (("desired", message.desired), ("actual", message.actual), ("error", message.error)):
            for field, source in (("q", point.positions), ("dq", point.velocities)):
                values[prefix + "_" + field] = [source[i] for i in indices] if len(source) == 6 else None
        record("controller", stamp_s=stamp(message.header), **values)
        observed["controller"] += 1

    def status(kind, message):
        value = json.loads(message.data)
        record(kind, value=value)
        if kind == "tracking" and value.get("restart_required"):
            observed["bad"].append("tracking restart_required: " + str(value))

    def trajectory(message):
        record(
            "trajectory",
            stamp_s=stamp(message.header),
            names=list(message.joint_names),
            points=[
                dict(
                    q=list(p.positions),
                    dq=list(p.velocities),
                    time_s=p.time_from_start.sec + p.time_from_start.nanosec * 1e-9,
                )
                for p in message.points
            ],
        )

    def twist(message):
        linear, angular = message.twist.linear, message.twist.angular
        record(
            "twist", stamp_s=stamp(message.header), frame=message.header.frame_id,
            linear=[linear.x, linear.y, linear.z], angular=[angular.x, angular.y, angular.z],
        )

    subscriptions = [
        node.create_subscription(JointState, "/joint_states", joints, sensor),
        node.create_subscription(PoseStamped, "/rm65/target_pose", lambda m: pose("target", m), 100),
        node.create_subscription(PoseStamped, "/rm65/internal/target_pose", lambda m: pose("internal_target", m), 100),
        node.create_subscription(
            JointTrajectoryControllerState, "/rm65_arm_controller/controller_state", controller, sensor
        ),
        node.create_subscription(JointTrajectory, "/rm65_arm_controller/joint_trajectory", trajectory, 100),
        node.create_subscription(TwistStamped, "/rm65/servo/delta_twist_cmds", twist, 100),
        node.create_subscription(String, "/rm65/input_status", lambda m: status("input", m), 100),
        node.create_subscription(String, "/rm65/pose_tracking/status", lambda m: status("tracking", m), 100),
        node.create_subscription(Int8, "/rm65/servo/status", lambda m: record("servo", code=int(m.data)), 100),
        node.create_subscription(
            Clock, "/clock", lambda m: record("clock", stamp_s=m.clock.sec + m.clock.nanosec * 1e-9), sensor
        ),
    ]
    opener = build_opener(ProxyHandler({}))
    base, client, sequence, token = "http://127.0.0.1:8766", str(uuid.uuid4()), 0, None

    def request(path, payload=None):
        encoded = None if payload is None else json.dumps(payload).encode()
        req = Request(base + path, data=encoded, headers={"Content-Type": "application/json"})
        with opener.open(req, timeout=0.12) as response:
            body = response.read().decode()
        return body if path == "/" else json.loads(body)

    def send(pressed, axes):
        nonlocal sequence
        sequence += 1
        started = time.monotonic()
        response = request("/api/input", dict(token=token, client_id=client, seq=sequence, pressed=pressed, axes=axes))
        if response != {"ok": True}:
            raise RuntimeError("manual input rejected: " + str(response))
        record("command", seq=sequence, pressed=pressed, axes=axes, http_latency_s=time.monotonic() - started)

    def spin():
        # Drain bounded callbacks without a background executor changing the
        # ordering of phase and receipt timestamps in this measurement file.
        for _ in range(12):
            rclpy.spin_once(node, timeout_sec=0.0)
        if child.poll() is not None:
            raise RuntimeError(f"simulation launch exited {child.returncode}; inspect stack.log")
        if observed["bad"]:
            raise RuntimeError("; ".join(observed["bad"][-4:]))
        if heartbeat_errors:
            raise RuntimeError("input heartbeat failed: " + heartbeat_errors[0])

    heartbeat_stop = threading.Event()
    phase_lock = threading.Lock()
    heartbeat_phase = None
    heartbeat_errors = []
    heartbeat_thread = None

    def heartbeat():
        # Observation callbacks must not delay the operator input lease. A
        # stalled sender still expires normally; no retry re-arms the input.
        try:
            while not heartbeat_stop.is_set():
                started = time.monotonic()
                with phase_lock:
                    phase = heartbeat_phase
                    if phase is not None and phase["category"] != "dropout":
                        send(phase["pressed"], phase["axes"])
                heartbeat_stop.wait(max(0.0, 0.01 - (time.monotonic() - started)))
        except Exception as error:
            heartbeat_errors.append(f"{type(error).__name__}: {error}")

    try:
        deadline, next_release = time.monotonic() + 45.0, 0.0
        stationary_since = None
        while time.monotonic() < deadline:
            spin()
            now = time.monotonic()
            try:
                if token is None:
                    match = re.search(r"[0-9a-f]{64}", request("/"))
                    if match:
                        token = match.group(0)
                if token is not None and now >= next_release:
                    send(False, [0] * 6)
                    next_release = now + 0.01
                    state = request("/api/state")
                    latest["state"] = state
                    velocity = (state.get("joint") or {}).get("velocity_rad_s") or []
                    # Wait for the actual startup transient to settle before
                    # comparing two independently sampled FK/TF observations.
                    stationary = len(velocity) == 6 and max(map(abs, velocity)) < 1e-4
                    ready = state.get("ready") and observed["joints"] >= 20 and observed["controller"] > 0
                    if ready and stationary:
                        stationary_since = now if stationary_since is None else stationary_since
                        if now - stationary_since >= 0.5:
                            break
                    else:
                        stationary_since = None
            except OSError:
                pass
            time.sleep(0.002)
        else:
            raise RuntimeError("manual simulation did not become ready: " + str(latest["state"]))
        # An independent check of this benchmark's observed FK against the normal
        # ROS TF-based panel, while the arm is stationary.
        data.qpos[qindices] = latest["q"]
        mujoco.mj_forward(model, data)
        panel_pose = latest["state"]["actual_pose"]
        tcp_position, tcp_quaternion = measured_tcp()
        fk_error = float(np.linalg.norm(tcp_position - panel_pose["position"]))
        fk_angle_error = 2 * math.acos(min(1.0, abs(float(np.dot(tcp_quaternion, panel_pose["quaternion_xyzw"])))))
        if fk_error > 0.0001:
            raise RuntimeError(f"observer FK disagrees with normal TF at rest: {fk_error} m")
        if fk_angle_error > 0.0001:
            raise RuntimeError(f"observer orientation disagrees with normal TF at rest: {fk_angle_error} rad")
        metadata["initial_q"] = latest["q"].tolist()
        metadata["fk_vs_panel_position_m"] = fk_error
        metadata["fk_vs_panel_orientation_rad"] = fk_angle_error
        record("measurement_start")
        heartbeat_thread = threading.Thread(target=heartbeat, name="manual-input-heartbeat")
        heartbeat_thread.start()
        next_state = 0.0
        for phase in phases:
            with phase_lock:
                started = time.monotonic()
                record("phase", **phase)
                heartbeat_phase = phase
                if phase["category"] != "dropout":
                    send(phase["pressed"], phase["axes"])
            print(json.dumps(dict(event="phase", run=output.name, **phase)), flush=True)
            while time.monotonic() - started < phase["seconds"]:
                spin()
                now = time.monotonic()
                if now >= next_state:
                    state = request("/api/state")
                    record("panel", value=state)
                    latest["state"] = state
                    next_state = now + 0.2
                    if phase["category"] in {"move", "hold"} and now - started > 0.3:
                        if not state.get("ready") or not state.get("control", {}).get("active"):
                            raise RuntimeError("control lost during an enabled phase: " + str(state))
                if observed["last_joint_at"] is None or now - observed["last_joint_at"] > 0.2:
                    raise RuntimeError("actual JointState feedback absent for >200ms")
                time.sleep(0.001)
            flush_started = time.monotonic()
            with record_lock:
                raw.flush()
            record("instrumentation_io", operation="local_phase_flush", duration_s=time.monotonic() - flush_started)
        metadata["completed"] = True
        record("measurement_end")
    except BaseException as error:
        metadata["failure"] = f"{type(error).__name__}: {error}"
        print(metadata["failure"], file=sys.stderr, flush=True)
        record("measurement_abort", reason=metadata["failure"])
    finally:
        heartbeat_stop.set()
        if heartbeat_thread is not None:
            heartbeat_thread.join(timeout=1.0)
        if token is not None:
            try:
                send(False, [0] * 6)
            except OSError:
                pass
        for _ in range(10):
            rclpy.spin_once(node, timeout_sec=0.005)
        node.destroy_node()
        rclpy.shutdown()
        raw.close()
        if child.poll() is None:
            os.killpg(child.pid, signal.SIGINT)
            try:
                child.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(child.pid, signal.SIGTERM)
                child.wait(timeout=5)
        child_log.close()
        shutil.copyfile(raw_path, output / "raw.jsonl")
        shutil.copyfile(log_path, output / "stack.log")
        metadata["launch_exit_code"] = child.returncode
        metadata["observed"] = observed
        metadata["finished_utc"] = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())
        (output / "meta.json").write_text(json.dumps(metadata, ensure_ascii=False, indent=2, allow_nan=False))
        model_dir.cleanup()
    return 0 if metadata["completed"] else 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--simulator", choices=("mujoco", "mock"), default="mujoco")
    parser.add_argument("--protocol", choices=("probe", "full"), default="full")
    parser.add_argument("--gui", action="store_true")
    parser.add_argument("--source-sha", default="unknown")
    return run(parser.parse_args())


if __name__ == "__main__":
    raise SystemExit(main())
