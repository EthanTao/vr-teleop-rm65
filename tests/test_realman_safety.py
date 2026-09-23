import sys
import types

import numpy as np
import pytest


# The CI environment does not install Meshcat.  The safety methods under test
# do not use transformations, so a minimal import stub keeps this unit test
# independent from the visualization dependency.
if "meshcat.transformations" not in sys.modules:
    meshcat = types.ModuleType("meshcat")
    transformations = types.ModuleType("meshcat.transformations")
    meshcat.transformations = transformations
    sys.modules.setdefault("meshcat", meshcat)
    sys.modules.setdefault("meshcat.transformations", transformations)

if "xrobotoolkit_sdk" not in sys.modules:
    sys.modules["xrobotoolkit_sdk"] = types.ModuleType("xrobotoolkit_sdk")

from xrobotoolkit_teleop.hardware.interface.realman_rm65 import RealmanRM65Interface
from xrobotoolkit_teleop.hardware.legacy.cartesian_teleop_controller import (
    RealmanRM65CartesianTeleopController,
)
import xrobotoolkit_teleop.hardware.legacy.cartesian_teleop_controller as controller_module
import xrobotoolkit_teleop.hardware.interface.realman_rm65 as realman_module


class _FragmentedSocket:
    def __init__(self, chunks):
        self.chunks = list(chunks)
        self.timeout = 0.2
        self.recv_calls = 0

    def gettimeout(self):
        return self.timeout

    def settimeout(self, value):
        self.timeout = value

    def sendall(self, data):
        self.sent = data

    def recv(self, size):
        self.recv_calls += 1
        return self.chunks.pop(0)


def test_workspace_guard_accepts_target_inside_bounds():
    arm = RealmanRM65Interface()

    within_bounds, reason = arm._pose_is_within_bounds(np.array([0.3, 0.2, 0.1]))

    assert within_bounds
    assert reason == ""


def test_workspace_guard_does_not_apply_radial_limit():
    arm = RealmanRM65Interface()

    within_bounds, reason = arm._pose_is_within_bounds(np.array([0.9, 0.0, 0.0]))

    assert within_bounds
    assert reason == ""


def test_workspace_guard_rejects_target_outside_axis_bounds():
    arm = RealmanRM65Interface(
        workspace_min_xyz_m=[-0.4, -0.4, -0.4],
        workspace_max_xyz_m=[0.4, 0.4, 0.4],
    )

    within_bounds, reason = arm._pose_is_within_bounds(np.array([0.5, 0.0, 0.0]))

    assert not within_bounds
    assert "bounds" in reason


def test_controller_false_response_is_an_error():
    error = RealmanRM65Interface._response_error([{"command": "movel", "receive_state": False}])

    assert error is not None


def test_empty_response_is_not_an_acknowledgement():
    assert not RealmanRM65Interface._command_acknowledged([], "movel")


def test_matching_success_response_is_an_acknowledgement():
    response = [{"command": "movel", "receive_state": True}]

    assert RealmanRM65Interface._command_acknowledged(response, "movel")
    assert not RealmanRM65Interface._command_acknowledged(response, "set_arm_stop")


def test_response_detection_ignores_unrelated_json_lines():
    data = b'{"state":"current_arm_state"}\r\n{"command":"movel","receive_state":true}\r\n'

    assert RealmanRM65Interface._buffer_contains_response(data, "movel")
    assert not RealmanRM65Interface._buffer_contains_response(data, "set_arm_stop")


def test_response_detection_waits_for_line_terminator():
    data = b'{"command":"movel","receive_state":true}'

    assert not RealmanRM65Interface._buffer_contains_response(data, "movel")


def test_send_json_waits_for_fragmented_line_terminator():
    arm = RealmanRM65Interface()
    arm.sock = _FragmentedSocket([b'{"command":"movel","receive_state":true}', b"\r\n"])

    messages = arm.send_json({"command": "movel"}, wait_s=0.1)

    assert messages == [{"command": "movel", "receive_state": True}]
    assert arm.sock.recv_calls == 2


def test_send_only_uses_one_crlf_framed_json_write():
    arm = RealmanRM65Interface()
    arm.sock = _FragmentedSocket([])

    arm.send_only({"command": "movep_follow", "pose_quat": [1, 2, 3, 4, 5, 6, 7]})

    assert arm.sock.sent.endswith(b"\r\n")
    assert b'"command": "movep_follow"' in arm.sock.sent


def test_get_arm_software_info_returns_matching_product_response():
    arm = RealmanRM65Interface()
    arm.send_json = lambda payload, wait_s: [
        {
            "command": "get_arm_software_info",
            "Product_version": "RM65-BI",
            "ctrl_info": {"version": "V1.7.0"},
        }
    ]

    info = arm.get_arm_software_info()

    assert info is not None
    assert info["Product_version"] == "RM65-BI"


def test_stop_without_ack_is_reported_but_does_not_raise():
    arm = RealmanRM65Interface()
    arm.send_json = lambda payload, wait_s: []

    assert not arm.stop()


def test_stop_accepts_realman_arm_stop_acknowledgement():
    arm = RealmanRM65Interface()
    arm.send_json = lambda payload, wait_s: [{"command": "set_arm_stop", "arm_stop": True}]

    assert arm.stop()


def test_slow_stop_accepts_realman_acknowledgement():
    arm = RealmanRM65Interface()
    arm.send_json = lambda payload, wait_s: [
        {"command": "set_arm_slow_stop", "arm_slow_stop": True}
    ]

    assert arm.slow_stop()


def test_movep_follow_sends_normalized_pose_quaternion_without_waiting(monkeypatch):
    arm = RealmanRM65Interface()
    sent_payloads = []
    arm.send_only = sent_payloads.append
    monkeypatch.setattr(arm, "xyz_quat_to_pose6", lambda xyz, quat: [1, 2, 3, 4, 5, 6])

    sent = arm.send_movep_follow_xyz_quat(
        np.array([0.3, -0.1, 0.5]),
        np.array([-2.0, 0.0, 0.0, 0.0]),
    )

    assert sent
    assert sent_payloads == [
        {
            "command": "movep_follow",
            "pose_quat": [300000, -100000, 500000, 1000000, 0, 0, 0],
        }
    ]
    assert arm.get_last_commanded_pose6() == [1, 2, 3, 4, 5, 6]


def test_movep_follow_transport_failure_latches_fault_and_stops():
    arm = RealmanRM65Interface()
    arm.send_only = lambda payload: (_ for _ in ()).throw(OSError("link down"))
    arm.stop = lambda: True

    sent = arm.send_movep_follow_xyz_quat(
        np.array([0.3, 0.0, 0.5]),
        np.array([1.0, 0.0, 0.0, 0.0]),
    )

    assert not sent
    assert arm.fault_latched
    assert "transport failed" in arm.fault_reason


def test_movej_follow_converts_radians_to_controller_millidegrees():
    arm = RealmanRM65Interface()
    sent_payloads = []
    arm.send_only = sent_payloads.append

    sent = arm.send_movej_follow_joint_rad(
        np.deg2rad(np.array([1.0, 0.0, 20.0, 30.0, 0.0, 20.0]))
    )

    assert sent
    assert sent_payloads == [
        {
            "command": "movej_follow",
            "joint": [1000, 0, 20000, 30000, 0, 20000],
        }
    ]


def test_movej_follow_transport_failure_latches_fault_and_stops():
    arm = RealmanRM65Interface()
    arm.send_only = lambda payload: (_ for _ in ()).throw(OSError("link down"))
    arm.stop = lambda: True

    sent = arm.send_movej_follow_joint_rad(np.zeros(6))

    assert not sent
    assert arm.fault_latched
    assert "movej_follow transport failed" in arm.fault_reason


def _patch_identity_quaternion_ops(monkeypatch):
    monkeypatch.setattr(
        realman_module.tf,
        "quaternion_conjugate",
        lambda quat: quat,
        raising=False,
    )
    monkeypatch.setattr(
        realman_module.tf,
        "quaternion_multiply",
        lambda left, right: np.array([1.0, 0.0, 0.0, 0.0]),
        raising=False,
    )


def test_movel_without_ack_uses_feedback_confirmation(monkeypatch):
    arm = RealmanRM65Interface()
    arm._last_commanded_pose6 = [300000, 0, 0, 0, 0, 0]
    arm.send_json = lambda payload, wait_s: []
    monkeypatch.setattr(
        arm,
        "pose6_to_xyz_quat",
        lambda pose6: (np.array([0.3, 0.0, 0.0]), np.array([1.0, 0.0, 0.0, 0.0])),
    )
    monkeypatch.setattr(arm, "xyz_quat_to_pose6", lambda xyz, quat: [310000, 0, 0, 0, 0, 0])
    _patch_identity_quaternion_ops(monkeypatch)

    sent = arm.movel_xyz_quat(
        np.array([0.31, 0.0, 0.0]),
        np.array([1.0, 0.0, 0.0, 0.0]),
    )

    assert sent
    assert not arm.fault_latched
    assert arm.get_last_commanded_pose6() == [310000, 0, 0, 0, 0, 0]


def test_movel_sends_target_without_step_clamping(monkeypatch):
    arm = RealmanRM65Interface()
    arm._last_commanded_pose6 = [300000, 0, 0, 0, 0, 0]
    sent_payloads = []
    arm.send_json = lambda payload, wait_s: sent_payloads.append(payload) or []
    monkeypatch.setattr(
        arm,
        "pose6_to_xyz_quat",
        lambda pose6: (np.array([0.3, 0.0, 0.0]), np.array([1.0, 0.0, 0.0, 0.0])),
    )
    monkeypatch.setattr(
        arm,
        "xyz_quat_to_pose6",
        lambda xyz, quat: [int(round(xyz[0] * 1_000_000)), 0, 0, 0, 0, 0],
    )
    _patch_identity_quaternion_ops(monkeypatch)

    sent = arm.movel_xyz_quat(
        np.array([0.5, 0.0, 0.0]),
        np.array([1.0, 0.0, 0.0, 0.0]),
    )

    assert sent
    assert sent_payloads[0]["pose"] == [500000, 0, 0, 0, 0, 0]


def test_explicit_movel_rejection_still_latches_fault(monkeypatch):
    arm = RealmanRM65Interface()
    arm._last_commanded_pose6 = [300000, 0, 0, 0, 0, 0]

    def response(payload, wait_s):
        if payload["command"] == "movel":
            return [{"command": "movel", "receive_state": False}]
        return [{"command": "set_arm_stop", "arm_stop": True}]

    arm.send_json = response
    monkeypatch.setattr(
        arm,
        "pose6_to_xyz_quat",
        lambda pose6: (np.array([0.3, 0.0, 0.0]), np.array([1.0, 0.0, 0.0, 0.0])),
    )
    monkeypatch.setattr(arm, "xyz_quat_to_pose6", lambda xyz, quat: [310000, 0, 0, 0, 0, 0])
    _patch_identity_quaternion_ops(monkeypatch)

    sent = arm.movel_xyz_quat(
        np.array([0.31, 0.0, 0.0]),
        np.array([1.0, 0.0, 0.0, 0.0]),
    )

    assert not sent
    assert arm.fault_latched
    assert "rejected movel" in arm.fault_reason


def test_forced_state_refresh_does_not_reuse_stale_measurement():
    arm = RealmanRM65Interface()
    arm._measured_pose6 = [1, 2, 3, 4, 5, 6]
    arm._has_pose_measurement = True
    arm.send_json = lambda payload, wait_s: []

    assert arm.get_pose6(force=True) is None


def test_joint_only_response_does_not_refresh_pose():
    arm = RealmanRM65Interface()
    arm._measured_pose6 = [1, 2, 3, 4, 5, 6]
    arm._has_pose_measurement = True
    arm.send_json = lambda payload, wait_s: [
        {
            "state": "current_arm_state",
            "arm_state": {"joint": [0, 0, 0, 0, 0, 0]},
        }
    ]

    assert arm.get_pose6(force=True) is None


def test_clear_fault_rearms_interface():
    arm = RealmanRM65Interface()
    arm.fault_latched = True
    arm.fault_reason = "unreachable"

    arm.clear_fault()

    assert not arm.fault_latched
    assert arm.fault_reason is None


def test_command_reference_is_separate_from_measured_pose():
    arm = RealmanRM65Interface()
    arm._measured_pose6 = [1, 2, 3, 4, 5, 6]
    arm._has_pose_measurement = True

    arm.sync_command_reference([10, 20, 30, 40, 50, 60])

    assert arm.get_last_commanded_pose6() == [10, 20, 30, 40, 50, 60]
    assert arm._measured_pose6 == [1, 2, 3, 4, 5, 6]


class _TimestampXr:
    def __init__(self, value: int):
        self.value = value

    def get_timestamp_ns(self) -> int:
        return self.value


def _watchdog_controller(timestamp_ns: int = 1) -> RealmanRM65CartesianTeleopController:
    controller = object.__new__(RealmanRM65CartesianTeleopController)
    controller.xr = _TimestampXr(timestamp_ns)
    controller.xr_watchdog_timeout_s = 0.2
    controller._last_xr_timestamp_ns = None
    controller._last_xr_update_t = None
    return controller


def test_xr_watchdog_expires_when_timestamp_stops_advancing():
    controller = _watchdog_controller()

    assert controller._xr_sample_is_fresh(10.0)
    assert controller._xr_sample_is_fresh(10.19)
    assert not controller._xr_sample_is_fresh(10.21)


def test_xr_watchdog_refreshes_when_timestamp_advances():
    controller = _watchdog_controller()
    assert controller._xr_sample_is_fresh(10.0)

    controller.xr.value = 2

    assert controller._xr_sample_is_fresh(10.3)
    assert controller._last_xr_update_t == 10.3


def test_xr_watchdog_rejects_non_positive_timestamp():
    controller = _watchdog_controller(timestamp_ns=0)

    assert not controller._xr_sample_is_fresh(10.0)


class _ResetArm:
    fault_latched = False

    def movel_xyz_quat(self, xyz, quat, active=True):
        return True

    def get_pose6(self, force=False):
        return [0, 0, 0, 0, 0, 0]

    def pose6_to_xyz_quat(self, pose6):
        return np.zeros(3), np.array([1.0, 0.0, 0.0, 0.0])


def test_completed_reset_requires_release_before_rearm(monkeypatch):
    controller = object.__new__(RealmanRM65CartesianTeleopController)
    controller.arm = _ResetArm()
    controller._resetting = True
    controller._reset_start_xyz = np.zeros(3)
    controller._reset_start_quat = np.array([1.0, 0.0, 0.0, 0.0])
    controller.home_xyz = np.zeros(3)
    controller.home_quat = np.array([1.0, 0.0, 0.0, 0.0])
    controller._reset_step = 1
    controller._reset_steps_total = 1
    controller._reset_started_t = 0.0
    controller.reset_duration_s = 3.0
    controller.reset_settle_timeout_s = 3.0
    controller.reset_position_tolerance_m = 0.01
    controller.reset_rotation_tolerance_rad = 0.08
    controller._rearm_required = False
    monkeypatch.setattr(
        controller_module,
        "apply_delta_pose",
        lambda source_pos, source_rot, delta_pos, delta_rot: (source_pos, source_rot),
    )
    monkeypatch.setattr(
        controller_module,
        "quat_diff_as_angle_axis",
        lambda source_rot, target_rot: np.zeros(3),
    )

    controller._tick_reset()

    assert not controller._resetting
    assert controller._rearm_required
