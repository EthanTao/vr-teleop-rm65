"""Official RM driver bridge. Hardware writes are opt-in and unvalidated by default."""

from __future__ import annotations

import json
from pathlib import Path
import signal
import time

from .transport_guard import GuardConfig, JOINT_NAMES, TransportGuard, parse_snapshot, parse_stream_reply


def main(args=None) -> None:
    import rclpy
    from rclpy.node import Node
    from rclpy.signals import SignalHandlerOptions
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Bool, Int8, String
    from std_srvs.srv import Trigger

    class DriverBridge(Node):
        def __init__(self):
            super().__init__("rm65_driver_bridge")

            def parameter(name, default):
                return self.declare_parameter(name, default).value

            self.mode = parameter("mode", "hardware-dry-run")
            if self.mode not in ("hardware", "hardware-dry-run"):
                raise ValueError("the driver bridge is for hardware or hardware-dry-run only")
            self.live = self.mode == "hardware"
            permissions = [
                parameter("allow_hardware_motion", False),
                parameter("frames_verified", False),
                parameter("stop_verified", False),
                parameter("speed_units_verified", False),
            ]
            if self.live and not all(value is True for value in permissions):
                raise RuntimeError("hardware transport requires explicit motion, frame, stop and speed-unit validation")
            cfg = GuardConfig(
                period_s=float(parameter("control_period_s", 0.02)),
                timeout_s=float(parameter("feedback_timeout_s", 0.20)),
                speed_rad_s=tuple(parameter("joint_velocity_limits", list(GuardConfig().speed_rad_s))),
                lower_rad=tuple(parameter("joint_lower_limits", list(GuardConfig().lower_rad))),
                upper_rad=tuple(parameter("joint_upper_limits", list(GuardConfig().upper_rad))),
            )
            self.guard = TransportGuard(cfg)
            self.guard.wait_for_driver()
            self.clock_id = ""
            clock_file = Path("/proc/sys/kernel/random/boot_id")
            if clock_file.is_file():
                self.clock_id = clock_file.read_text(encoding="ascii").strip()
            if self.live and not self.clock_id:
                raise RuntimeError("live feedback requires the same verified Linux monotonic clock as the driver")
            prefix = str(parameter("driver_prefix", "/rm65/driver")).rstrip("/")
            self.status_pub = self.create_publisher(String, "/rm65/transport/status", 10)
            self.state_pub = self.create_publisher(JointState, "/rm65/transport/joint_states", 1)
            self.pending_futures = []
            self.arm_future = None
            self.raw_feedback_seen = False
            self.submissions = 0
            self.virtual_submissions = 0
            self.last_reason = None
            self.movement_pub = None
            self.stop_clients = {}
            self.arm_client = None
            self.native_generation = None
            if self.live:
                self.movement_pub = self.create_publisher(JointState, prefix + "/servo_joint_command", 1)
                self.stop_clients = {
                    "soft": self.create_client(Trigger, prefix + "/servo_slow_stop"),
                    "hard": self.create_client(Trigger, prefix + "/servo_hard_stop"),
                }
                self.arm_client = self.create_client(Trigger, prefix + "/servo_enable_stream")
            self.create_subscription(String, prefix + "/feedback_snapshot", self.snapshot, 1)
            self.create_subscription(JointState, prefix + "/joint_states", self.raw_state, 1)
            self.create_subscription(String, prefix + "/servo_send_result", self.send_result, 10)
            self.create_subscription(JointState, "/rm65/transport/joint_commands", self.command, 1)
            self.create_subscription(
                Bool, "/rm65/enable", lambda msg: self.guard.update_enable(msg.data, time.monotonic()), 1
            )
            self.create_subscription(
                Int8, "/rm65/servo/status", lambda msg: self.guard.update_servo_status(msg.data, time.monotonic()), 1
            )
            self.timer = self.create_timer(cfg.period_s, self.tick)

        def status(self, event, **fields):
            message = String()
            message.data = json.dumps(
                {
                    "event": event,
                    "mode": self.mode,
                    "hardware_writes": self.live,
                    "monotonic_s": time.monotonic(),
                    **fields,
                },
                allow_nan=False,
            )
            self.status_pub.publish(message)

        def raw_state(self, _message):
            # The unpatched official node restamps cached values in a timer and
            # supplies no JointState.velocity. It is never a safety source.
            if not self.raw_feedback_seen:
                self.raw_feedback_seen = True
                self.status("raw_driver_state_diagnostic_only", coherent_snapshot_required=True)

        def snapshot(self, message):
            now = time.monotonic()
            try:
                frame = parse_snapshot(
                    message.data, now_s=now, clock_id=self.clock_id, max_age_s=self.guard.config.timeout_s
                )
                if not self.guard.update_feedback(frame, now):
                    return
            except (TypeError, ValueError, KeyError, json.JSONDecodeError) as exc:
                self.guard.fault("invalid_feedback_snapshot", now)
                self.status("invalid_feedback_snapshot", detail=str(exc))
                return
            output = JointState()
            output.header.stamp.sec, output.header.stamp.nanosec = divmod(frame.ros_stamp_ns, 1_000_000_000)
            output.name = list(JOINT_NAMES)
            output.position = list(frame.position)
            output.velocity = list(frame.velocity)
            self.state_pub.publish(output)
            self.status(
                "feedback",
                session=frame.session,
                packet_id=frame.packet_id,
                source_s=frame.source_s,
                position_rad=frame.position,
                velocity_rad_s=frame.velocity,
                controller_errors=frame.controller_errors,
                joint_errors=frame.joint_errors,
            )

        def command(self, message):
            try:
                self.guard.update_command(message.name, message.position, time.monotonic())
            except (TypeError, ValueError) as exc:
                self.guard.fault("invalid_jtc_command", time.monotonic())
                self.status("invalid_jtc_command", detail=str(exc))

        def send_result(self, message):
            try:
                result = json.loads(message.data)
                code = result["api_code"]
                if type(code) is not int:
                    raise ValueError("invalid SDK return code")
                self.status("driver_api_result", result=result)
                if code != 0:
                    self.guard.fault("driver_send_failure", time.monotonic())
                elif result.get("event") == "watchdog_hard_stop":
                    self.guard.fault("native_watchdog_stopped_stream", time.monotonic())
                elif self.guard.stream_armed and (
                    result.get("generation") != self.native_generation or result.get("stream_enabled") is not True
                ):
                    self.guard.fault("native_stream_generation_or_state_changed", time.monotonic())
            except (ValueError, TypeError, KeyError):
                self.guard.fault("invalid_driver_send_result", time.monotonic())

        def _dispatch_stop(self):
            request = self.guard.take_stop_request()
            if request is None:
                return
            self.native_generation = None
            self.status("stop_request", request_id=request.request_id, kind=request.kind, reason=request.reason)
            if not self.live:
                self.guard.confirm_stop(request.request_id, True, time.monotonic())
                self.status("virtual_stop_only", request_id=request.request_id, no_driver_request=True)
                return
            client = self.stop_clients[request.kind]
            if not client.service_is_ready():
                self.guard.confirm_stop(request.request_id, False, time.monotonic())
                self.status("stop_service_unavailable", request_id=request.request_id)
                return
            future = client.call_async(Trigger.Request())
            self.pending_futures.append((future, request))

        def _poll_services(self):
            remaining = []
            for future, request in self.pending_futures:
                if not future.done():
                    remaining.append((future, request))
                    continue
                try:
                    response = future.result()
                    reply = parse_stream_reply(response.message, success=response.success, enabled=False)
                    success = True
                    self.status("native_stop_ack", request_id=request.request_id, generation=reply.generation)
                except Exception:
                    success = False
                accepted = self.guard.confirm_stop(request.request_id, success, time.monotonic())
                self.status("stop_result", request_id=request.request_id, success=success, accepted=accepted)
            self.pending_futures = remaining
            if self.arm_future is not None:
                future, epoch, requested_s = self.arm_future
                if not future.done() and time.monotonic() - requested_s > self.guard.config.stop_timeout_s:
                    if epoch == self.guard.epoch:
                        self.guard.revoke_obsolete_enable(time.monotonic())
                        self.status("stream_enable_ack_timeout", epoch=epoch)
                if future.done():
                    self.arm_future = None
                    try:
                        response = future.result()
                        reply = parse_stream_reply(response.message, success=response.success, enabled=True)
                        success = True
                    except Exception:
                        success = False
                    if not self.guard.mark_armed(epoch, success, time.monotonic()):
                        self.guard.revoke_obsolete_enable(time.monotonic())
                    else:
                        self.native_generation = reply.generation
                    self.status("stream_enable_result", success=success, epoch=epoch)

        def tick(self):
            now = time.monotonic()
            self._poll_services()
            services_ready = not self.live or (
                self.arm_client.service_is_ready()
                and all(client.service_is_ready() for client in self.stop_clients.values())
            )
            self.guard.begin_driver_handshake(now, services_ready=services_ready)
            decision = self.guard.tick(now)
            self._dispatch_stop()
            if (
                decision.state != "fault"
                and not self.guard.stream_armed
                and self.arm_future is None
                and self.guard.can_arm(now)
            ):
                if not self.live:
                    self.guard.mark_armed(self.guard.epoch, True, now)
                elif self.arm_client.service_is_ready():
                    self.arm_future = (self.arm_client.call_async(Trigger.Request()), self.guard.epoch, now)
            if decision.target is not None:
                if self.live:
                    if self.native_generation is None:
                        self.guard.fault("native_generation_missing", now)
                        return
                    message = JointState()
                    message.name = list(JOINT_NAMES)
                    message.position = list(decision.target)
                    # The native admission compares CLOCK_REALTIME; do not let a
                    # ROS simulation clock produce a live robot command stamp.
                    message.header.stamp.sec, message.header.stamp.nanosec = divmod(time.time_ns(), 1_000_000_000)
                    message.header.frame_id = str(self.native_generation)
                    try:
                        self.movement_pub.publish(message)
                        self.submissions += 1
                    except Exception as exc:
                        self.guard.fault("ros_command_submission_failed", now)
                        self.status("ros_command_submission_failed", detail=str(exc))
                else:
                    self.virtual_submissions += 1
                self.status(
                    "command_submission",
                    target_rad=decision.target,
                    ros_submitted=self.live,
                    driver_execution_confirmed=False,
                    virtual_only=not self.live,
                )
            key = (decision.state, decision.reason)
            if key != self.last_reason:
                self.status(
                    "admission", state=decision.state, reason=decision.reason, stop_confirmed=self.guard.stop_confirmed
                )
                self.last_reason = key

        def close_safely(self):
            self.timer.cancel()
            if self.live:
                self.guard.request_stop("hard", "bridge_shutdown", time.monotonic())
                self._dispatch_stop()
                deadline = time.monotonic() + self.guard.config.stop_timeout_s
                while self.pending_futures and time.monotonic() < deadline and rclpy.ok():
                    rclpy.spin_once(self, timeout_sec=0.02)
                    self._poll_services()
                self.status("shutdown", stop_confirmed=self.guard.stop_confirmed)

    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = None

    def interrupted(_signal, _frame):
        raise KeyboardInterrupt

    old_term = signal.signal(signal.SIGTERM, interrupted)
    try:
        node = DriverBridge()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.close_safely()
            node.destroy_node()
        signal.signal(signal.SIGTERM, old_term)
        if rclpy.ok():
            rclpy.shutdown()
