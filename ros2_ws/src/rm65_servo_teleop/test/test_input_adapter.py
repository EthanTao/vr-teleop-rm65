"""Input/anchor tests run without ROS, an XR service, or a robot."""

import json
import math

import numpy as np
import pytest

from rm65_servo_teleop.input_adapter import (
    ControllerSample,
    InputAdapter,
    InputConfig,
    InputRejected,
    Pose,
    SIDE_MOUNT_MAPPING,
    decode_raw_packet,
    quaternion_difference,
    quaternion_rotvec,
    rotvec_quaternion,
)
from rm65_servo_teleop.mock_xr import make_packet

TCP = Pose.from_vector([0.4, 0.0, 0.3, 0.0, 0.0, 0.0, 1.0])


def sample(stamp, grip=0.8, xyz=(0.0, 0.0, 0.0), rotation=(0.0, 0.0, 0.0), synthetic=False):
    return ControllerSample(
        stamp, Pose(np.array(xyz, dtype=float), rotvec_quaternion(np.array(rotation))), grip, synthetic=synthetic
    )


def armed_adapter():
    adapter = InputAdapter()
    adapter.receive(sample(1, grip=0.0), 0.0)
    assert not adapter.step(0.0, TCP, 0.0).enabled
    adapter.receive(sample(2), 0.02)
    anchored = adapter.step(0.02, TCP, 0.02)
    assert anchored.reason == "anchoring" and not anchored.enabled
    assert adapter.step(0.03, TCP, 0.03).enabled
    return adapter


def test_protocol_nested_value_and_missing_status_is_not_fabricated():
    raw = {
        "functionName": "Tracking",
        "value": json.dumps(
            {
                "Input": 1,
                "timeStampNs": 1732613438765715200,
                "Controller": {"right": {"pose": "0,0,0,0,0,0,1", "grip": 0.3}},
            }
        ),
    }
    decoded = decode_raw_packet(json.dumps(raw))
    assert decoded.timestamp_ns == 1732613438765715200
    assert decoded.grip == 0.3
    assert decoded.status_present is False


@pytest.mark.parametrize("value", [None, [0] * 6, [0, 0, 0, 0, 0, 0, 0], [float("nan"), 0, 0, 0, 0, 0, 1]])
def test_protocol_rejects_malformed_pose(value):
    packet = {"Input": 1, "timeStampNs": 1, "Controller": {"right": {"pose": value, "grip": 0.0}}}
    with pytest.raises(InputRejected):
        decode_raw_packet(json.dumps(packet))


@pytest.mark.parametrize(
    "change",
    [
        {"status": 0},
        {"status": 2},
        {"status": 128},
        {"isActive": False},
        {"valid": False},
        {"grip": float("inf")},
        {"grip": True},
    ],
)
def test_protocol_rejects_explicit_invalid_quality_and_grip(change):
    controller = {"pose": [0, 0, 0, 0, 0, 0, 1], "grip": 0.0, **change}
    with pytest.raises(InputRejected):
        decode_raw_packet(json.dumps({"Input": 1, "timeStampNs": 1, "Controller": {"right": controller}}))


def test_start_with_grip_held_requires_release():
    adapter = InputAdapter()
    adapter.receive(sample(1), 0.0)
    assert not adapter.step(0.0, TCP, 0.0).enabled
    assert adapter.step(0.0, TCP, 0.0).reason == "release_grip_to_start"


def test_first_grip_anchors_at_measured_tcp_and_free_movement_while_released():
    adapter = InputAdapter()
    adapter.receive(sample(1, grip=0.0, xyz=(1, 2, 3)), 0.0)
    adapter.receive(sample(2, grip=0.8, xyz=(9, 8, 7)), 0.02)
    result = adapter.step(0.02, TCP, 0.02)
    assert result.reason == "anchoring"
    np.testing.assert_allclose(result.target.xyz, TCP.xyz)
    np.testing.assert_allclose(adapter.step(0.03, TCP, 0.03).target.xyz, TCP.xyz)


@pytest.mark.parametrize("axis,sign", [(axis, sign) for axis in range(3) for sign in (-1, 1)])
def test_frozen_translation_mapping(axis, sign):
    adapter = armed_adapter()
    movement = np.zeros(3)
    movement[axis] = sign * 0.01
    adapter.receive(sample(3, xyz=movement), 0.04)
    result = adapter.step(0.04, TCP, 0.04)
    assert result.enabled
    np.testing.assert_allclose(result.target.xyz, TCP.xyz + SIDE_MOUNT_MAPPING @ movement)


def test_orientation_mapping_and_rotation_scale():
    adapter = armed_adapter()
    adapter.receive(sample(3, rotation=(0.1, 0.0, 0.0)), 0.04)
    result = adapter.step(0.04, TCP, 0.04)
    assert result.enabled
    np.testing.assert_allclose(quaternion_rotvec(result.target.quaternion_xyzw), [0.0, 0.025, 0.0], atol=1e-12)


def test_quaternion_sign_change_is_not_a_rotation_jump():
    adapter = armed_adapter()
    opposite = sample(3)
    opposite.pose.quaternion_xyzw[:] *= -1
    adapter.receive(opposite, 0.04)
    assert adapter.step(0.04, TCP, 0.04).enabled


def test_duplicate_device_time_cannot_refresh_source_age():
    adapter = armed_adapter()
    duplicate = sample(2)
    adapter.receive(duplicate, 0.19)
    result = adapter.step(0.23, TCP, 0.23)
    assert not result.enabled and result.reason == "xr_timeout"
    adapter.receive(sample(3), 0.24)
    assert not adapter.step(0.24, TCP, 0.24).enabled
    adapter.receive(sample(4, grip=0.0), 0.26)
    adapter.receive(sample(5), 0.28)
    assert adapter.step(0.28, TCP, 0.28).reason == "anchoring"


def test_reused_timestamp_with_new_values_invalidates():
    adapter = armed_adapter()
    adapter.receive(sample(2, xyz=(0.001, 0.0, 0.0)), 0.04)
    result = adapter.step(0.04, TCP, 0.04)
    assert not result.enabled and result.reason == "device_timestamp_reused"


def test_new_packet_after_outage_cannot_hide_timeout_before_watchdog_tick():
    adapter = armed_adapter()
    # Simulate the raw callback running first after an executor stall. The
    # source-clock advance must not retain the old Grip segment.
    adapter.receive(sample(3), 0.24)
    result = adapter.step(0.24, TCP, 0.24)
    assert not result.enabled and result.reason == "xr_timeout"
    adapter.receive(sample(4, grip=0.0), 0.26)
    adapter.receive(sample(5), 0.28)
    assert adapter.step(0.28, TCP, 0.28).reason == "anchoring"


def test_regressed_device_clock_requires_new_release_in_new_epoch():
    adapter = armed_adapter()
    adapter.receive(sample(1), 0.04)
    assert not adapter.step(0.04, TCP, 0.04).enabled
    adapter.receive(sample(2), 0.06)
    assert not adapter.step(0.06, TCP, 0.06).enabled
    adapter.receive(sample(3, grip=0.0), 0.08)
    adapter.receive(sample(4), 0.10)
    assert adapter.step(0.10, TCP, 0.10).reason == "anchoring"


def test_fresh_release_and_regrip_never_chase_old_anchor():
    adapter = armed_adapter()
    adapter.receive(sample(3, xyz=(0.01, 0.0, 0.0)), 0.04)
    assert adapter.step(0.04, TCP, 0.04).enabled
    adapter.receive(sample(4, grip=0.0), 0.06)
    assert not adapter.step(0.06, TCP, 0.06).enabled
    shifted_tcp = Pose.from_vector([0.5, 0.1, 0.4, 0.0, 0.0, 0.0, 1.0])
    adapter.receive(sample(5, xyz=(-0.4, 0.8, 0.2)), 0.08)
    result = adapter.step(0.08, shifted_tcp, 0.08)
    assert result.anchor_id == 2 and result.reason == "anchoring"
    np.testing.assert_allclose(result.target.xyz, shifted_tcp.xyz)


@pytest.mark.parametrize(
    "new_sample,reason",
    [
        (sample(3, xyz=(0.051, 0.0, 0.0)), "xr_translation_jump"),
        (sample(3, rotation=(0.0, 0.0, 0.261)), "xr_rotation_jump"),
    ],
)
def test_raw_jumps_disarm_and_require_release(new_sample, reason):
    adapter = armed_adapter()
    adapter.receive(new_sample, 0.04)
    result = adapter.step(0.04, TCP, 0.04)
    assert not result.enabled and result.reason == reason
    adapter.receive(sample(4), 0.06)
    assert not adapter.step(0.06, TCP, 0.06).enabled


@pytest.mark.parametrize(
    "feedback,transform,reason",
    [
        (None, TCP, "feedback_unavailable"),
        (-1.0, TCP, "feedback_stale"),
        (0.04, None, "tcp_transform_unavailable"),
    ],
)
def test_no_robot_feedback_or_transform_disarms(feedback, transform, reason):
    adapter = armed_adapter()
    result = adapter.step(0.04, transform, feedback)
    assert not result.enabled and result.reason == reason


def test_synthetic_packets_rejected_outside_explicit_simulation():
    raw = make_packet(1.2, 10)
    assert decode_raw_packet(raw).synthetic
    adapter = InputAdapter()
    adapter.receive_raw(raw, 0.0)
    assert adapter.step(0.0, TCP, 0.0).reason == "synthetic_input_disabled"


def test_mock_sequence_has_release_and_continuous_small_steps():
    previous = None
    maximum_step = 0.0
    for tick in range(901):
        t = tick / 50.0
        current = decode_raw_packet(make_packet(t, tick + 1))
        if previous is not None:
            maximum_step = max(maximum_step, float(np.linalg.norm(current.pose.xyz - previous.pose.xyz)))
        previous = current
    assert previous.grip == 0.0
    assert maximum_step < 0.001


def test_source_event_disarms_immediately():
    adapter = armed_adapter()
    adapter.receive_raw('{"sdk_event":"server_disconnected"}', 0.04)
    result = adapter.step(0.04, TCP, 0.04)
    assert not result.enabled and result.reason == "sdk_server_disconnected"


def test_rotation_envelope_is_continuous_across_the_pi_log_branch():
    adapter = armed_adapter()
    previous = TCP.quaternion_xyzw
    largest_step = 0.0
    for tick, angle in enumerate(np.linspace(0.02, 3.5, 175), start=3):
        now = 0.02 * (tick - 1)
        adapter.receive(sample(tick, rotation=(0.0, 0.0, float(angle))), now)
        result = adapter.step(now, TCP, now)
        assert result.enabled
        current = result.target.quaternion_xyzw
        largest_step = max(
            largest_step, float(np.linalg.norm(quaternion_rotvec(quaternion_difference(previous, current))))
        )
        previous = current
    assert largest_step < 0.006
    np.testing.assert_allclose(quaternion_rotvec(previous), [0.0, 0.0, -0.2], atol=1e-10)


def test_malformed_tcp_pose_cannot_bypass_pure_adapter_validation():
    with pytest.raises(InputRejected):
        Pose(np.array([float("nan"), 0.0, 0.0]), np.array([0.0, 0.0, 0.0, 1.0]))
    with pytest.raises(InputRejected):
        Pose(np.zeros(3), np.zeros(4))


def test_mock_waits_for_tracker_to_acknowledge_initial_grip_release():
    import json
    from rm65_servo_teleop.mock_xr import MockStartupGate

    gate = MockStartupGate()
    gate.input_status(json.dumps({"reason": "released", "enabled": False}), 1.0)
    assert not gate.ready(1.0)  # Input can initialize much earlier than MoveIt.
    released = {
        "state": "grip_released",
        "active": False,
        "requested": False,
        "release_required": False,
        "restart_required": False,
    }
    gate.tracking_status(json.dumps({**released, "state": "awaiting_release", "release_required": True}), 1.1)
    assert not gate.ready(1.1)
    gate.tracking_status(json.dumps(released), 1.2)
    assert gate.ready(1.2)
    assert not gate.ready(1.6)  # A cached earlier input acknowledgement expires.
    gate.input_status(json.dumps({"reason": "released", "enabled": False}), 1.6)
    assert gate.ready(1.6)
    gate.tracking_status(json.dumps({**released, "restart_required": True}), 1.61)
    assert not gate.ready(1.61)


@pytest.mark.parametrize("malformed", ["null", "[]", "bad-json", '{"state":"grip_released"}'])
def test_mock_rejects_malformed_or_incomplete_tracker_readiness(malformed):
    from rm65_servo_teleop.mock_xr import MockStartupGate

    gate = MockStartupGate()
    gate.input_status('{"reason":"released","enabled":false}', 1.0)
    gate.tracking_status(malformed, 1.0)
    assert not gate.ready(1.0)
