#!/usr/bin/env python3
"""Collect board prerequisites and passive runtime evidence; never authorize motion.

Run from Windows over SSH, or with --local in the board checkout. Existing
board_environment.py supplies the dependency, feedback and input probes, so
this script can inspect the deployed board without syncing/rebuilding it.
"""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import re
import subprocess
import sys
import uuid

GRAPH_PROBE = r'''
import json,time
import rclpy
from rclpy.time import Time
from controller_manager_msgs.srv import ListControllers
from tf2_ros import Buffer, TransformListener
rclpy.init()
node = rclpy.create_node('rm65_field_readonly_observer')
buffer = Buffer()
listener = TransformListener(buffer, node)
client = node.create_client(ListControllers, '/controller_manager/list_controllers')
future = client.call_async(ListControllers.Request()) if client.wait_for_service(timeout_sec=3) else None
deadline = time.monotonic()+4
while time.monotonic()<deadline: rclpy.spin_once(node,timeout_sec=.05)
controllers = {c.name:c.state for c in future.result().controller} if future and future.done() and future.result() else {}
nodes = sorted(name for name,namespace in node.get_node_names_and_namespaces())
result = {'nodes':nodes, 'controllers':controllers, 'observer_published_robot_commands':False}
try:
    tf = buffer.lookup_transform('base_link','tcp',Time())
    stamp = tf.header.stamp.sec+tf.header.stamp.nanosec*1e-9
    result['tf_age_s'] = node.get_clock().now().nanoseconds*1e-9-stamp
    result['tcp_xyz_m'] = [tf.transform.translation.x,tf.transform.translation.y,tf.transform.translation.z]
    result['tcp_quaternion_xyzw'] = [tf.transform.rotation.x,tf.transform.rotation.y,tf.transform.rotation.z,tf.transform.rotation.w]
except Exception as exc:
    result['tf_error'] = str(exc)
required = {'xr_sdk_source','xr_input','rm65_driver_bridge','rm65_pose_tracking','controller_manager'}
result['missing_nodes'] = sorted(required-set(nodes))
result['passed'] = (not result['missing_nodes'] and controllers.get('rm65_arm_controller')=='active'
    and controllers.get('joint_state_broadcaster')=='active' and 0<=result.get('tf_age_s',999)<.2)
print(json.dumps(result))
node.destroy_node()
rclpy.shutdown()
raise SystemExit(0 if result['passed'] else 2)
'''

REMOTE_PROBE = r'''
import json,pathlib,platform,runpy,subprocess,sys

def run(command, input_text=None, timeout=35):
    try:
        p = subprocess.run(command,input=input_text,capture_output=True,text=True,timeout=timeout)
        return {'returncode':p.returncode,'stdout':p.stdout,'stderr':p.stderr[-6000:]}
    except (OSError,subprocess.TimeoutExpired) as exc:
        return {'returncode':None,'error':str(exc)}

report = {'read_only':True,'motion_authorized':False,'architecture':platform.machine(),'probes':{}}
try:
    workspace = pathlib.Path(SETTINGS['workspace']).resolve()
    helper = workspace/'scripts/tools/board_environment.py'
    if platform.system()!='Linux' or not helper.is_file():
        raise RuntimeError('Use the deployed Linux board workspace; board_environment.py is missing')
    board = runpy.run_path(str(helper))
    docker = board['docker_prefix']()
    report['network'] = run(['ip','-br','-4','address'])
    report['route'] = run(['ip','-j','route','get',board['ARM_IP']])
    report['listening_ports'] = run(['ss','-H','-lntu'])
    report['docker_sessions'] = run(docker+['ps','--format','{{.Names}}\t{{.Image}}\t{{.Status}}'])
    current = board['inspect_container'](docker)
    report['container'] = current
    # Avoid extra diagnostic load in a live or unrecognized control session.
    if current and not board['is_our_readonly_session'](current,workspace):
        raise RuntimeError('Unrecognized or live session: passive metadata saved; all runtime probes skipped')
    template = '{"id":{{json .Id}},"source_sha":{{json (index .Config.Labels "org.rm65.source-sha")}},"xr_sdk":{{json (index .Config.Labels "org.rm65.xr-sdk")}}}'
    report['image'] = run(docker+['image','inspect','--format',template,board['IMAGE']])
    # The existing check action creates only a network-isolated disposable
    # dependency-check container. It starts no ROS graph and connects no arm.
    report['probes']['dependencies'] = run([sys.executable,'-B',str(helper),'check'],timeout=45)
    if current is None or current['status']!='running':
        raise RuntimeError('No matching running readonly session; nothing started or restarted')
    setup = board['SETUP_PYTHON']
    for name,source in [('feedback',board['STATUS_PROBE']),('input',board['INPUT_PROBE']),('graph',SETTINGS['graph_probe'])]:
        report['probes'][name] = run(docker+['exec','-i',board['CONTAINER'],'bash','-c',setup],input_text=source)
    report['recent_logs'] = run(docker+['logs','--timestamps','--since','10m','--tail','1000',board['CONTAINER']])
    report['log_window'] = 'last 10 minutes, at most 1000 lines; not a timing acceptance measurement'
except Exception as exc:
    report['error'] = str(exc)
print(json.dumps(report))
'''


def decode_result(result: dict) -> dict:
    if result.get("returncode") != 0:
        return {}
    try:
        value = json.loads(result.get("stdout", ""))
        return value if isinstance(value, dict) else {}
    except (TypeError, json.JSONDecodeError):
        return {}


def assess(remote: dict, *, require_input: bool = False, expected_source_sha: str | None = None) -> dict:
    probes = remote.get("probes", {})
    dependencies = decode_result(probes.get("dependencies", {}))
    feedback = decode_result(probes.get("feedback", {}))
    sample = feedback.get("latest", {})
    headset = decode_result(probes.get("input", {}))
    graph = decode_result(probes.get("graph", {}))
    image = decode_result(remote.get("image", {}))
    checks = {
        "board_reachable": remote.get("read_only") is True and "error" not in remote,
        "image_identity": bool(image.get("id"))
        and image.get("id") == (remote.get("container") or {}).get("image_id")
        and bool(re.fullmatch(r"[0-9a-f]{64}", image.get("source_sha") or ""))
        and (expected_source_sha is None or image.get("source_sha") == expected_source_sha),
        "dependencies": dependencies.get("passed") is True and image.get("xr_sdk") == "ON",
        "fresh_real_feedback": feedback.get("fresh_feedback") is True
        and feedback.get("packet_id_advanced") is True
        and sample.get("mode") == "hardware-dry-run"
        and sample.get("hardware_writes") is False
        and not sample.get("controller_errors")
        and sample.get("joint_errors") == [0] * 6,
        "controllers_and_tf": graph.get("passed") is True,
        "real_pico_input": headset.get("valid_frames", 0) > 1
        and headset.get("advancing_timestamps", 0) > 0
        and headset.get("invalid_frames") == 0
        and headset.get("synthetic_frames") == 0,
    }
    required = list(checks) if require_input else [key for key in checks if key != "real_pico_input"]
    return {
        "checks": checks,
        "software_readonly_checks_passed": all(checks[key] for key in required),
        "input_required_this_run": require_input,
        "motion_authorized": False,
        "onsite_validation_required": ["firmware/SDK", "base/TCP/FK", "speed units", "collision model",
                                       "physical stop/recovery", "timing under load"],
    }


def main(argv=None) -> int:
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="rm", help="SSH alias, or rm@<current board LAN IP>")
    parser.add_argument("--workspace", default="/home/rm/EthanTao")
    parser.add_argument("--local", action="store_true", help="Run in the board's local terminal instead of SSH")
    parser.add_argument("--require-input", action="store_true", help="Fail if real PICO frames are absent")
    parser.add_argument("--output", type=Path, help="New JSON path; never overwrite existing evidence")
    args = parser.parse_args(argv)
    if not re.fullmatch(r"[A-Za-z0-9_][A-Za-z0-9_.@:-]*", args.host):
        parser.error("invalid SSH host")
    if not args.workspace.startswith("/") or any(c in args.workspace for c in "\0\r\n"):
        parser.error("workspace must be an absolute Linux path")
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
    from servo_launcher import source_digest

    expected_source_sha = source_digest()
    settings = {"workspace": args.workspace, "graph_probe": GRAPH_PROBE}
    source = "SETTINGS = " + repr(settings) + "\n" + REMOTE_PROBE
    command = [sys.executable, "-B", "-"] if args.local else [
        "ssh", "-o", "BatchMode=yes", "-o", "StrictHostKeyChecking=yes", "-o", "ConnectTimeout=8",
        "-o", "ServerAliveInterval=5", "-o", "ServerAliveCountMax=2", args.host, "python3 -B -",
    ]
    try:
        process = subprocess.run(command, input=source, capture_output=True, text=True,
                                 encoding="utf-8", errors="replace", timeout=140)
        result = {"returncode": process.returncode, "stdout": process.stdout, "stderr": process.stderr[-6000:]}
    except (OSError, subprocess.TimeoutExpired) as exc:
        result = {"returncode": None, "error": str(exc)}
    remote = decode_result(result)
    assessment = assess(remote, require_input=args.require_input, expected_source_sha=expected_source_sha)
    report = {"checked_at_utc": datetime.now(timezone.utc).isoformat(),
              "scope": "board-local" if args.local else args.host,
              "expected_source_sha": expected_source_sha,
              "assessment": assessment, "remote": remote, "connection": result if not remote else None}
    output = args.output or Path(__file__).resolve().parents[2] / "build/validation/field-readiness" / (
        datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ_") + uuid.uuid4().hex[:8] + ".json")
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("x", encoding="utf-8") as stream:
        json.dump(report, stream, ensure_ascii=False, indent=2)
        stream.write("\n")
    labels = {"board_reachable": "板端完整检查", "image_identity": "运行镜像与当前源码一致", "dependencies": "ROS2/SDK及动态库", "fresh_real_feedback": "真实反馈与错误状态",
              "controllers_and_tf": "控制器/节点/TCP变换", "real_pico_input": "真实手柄输入"}
    for key, passed in assessment["checks"].items():
        print(f"{'PASS' if passed else '待处理'}  {labels[key]}")
    if remote.get("error"):
        print(remote["error"])
    if not remote:
        print(result.get("error") or result.get("stderr") or "未取得有效板端报告")
    print("报告：" + str(output.resolve()))
    print("此检查不批准真机运动；现场验证项目见 docs/真机现场操作清单.md。")
    return 0 if assessment["software_readonly_checks_passed"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
