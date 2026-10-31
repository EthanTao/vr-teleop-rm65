"""Approved follow changes versus the previous profile; no ROS or devices."""

import copy
import hashlib
import json
import math
from pathlib import Path

import numpy as np
import pytest
import yaml

from rm65_servo_teleop.input_adapter import (
    ControllerSample,
    InputAdapter,
    InputConfig,
    Pose,
    SIDE_MOUNT_MAPPING,
    quaternion_difference,
    quaternion_rotvec,
    rotvec_quaternion,
)

CONFIG = Path(__file__).resolve().parents[1] / "config/teleop.yaml"
TCP = Pose.from_vector([0.4, 0.0, 0.3, 0, 0, 0, 1])


def profile():
    return yaml.safe_load(CONFIG.read_text(encoding="utf-8"))


def previous_profile():
    config = profile()
    config["safety"]["max_linear_velocity_m_s"] = 0.05
    config["input"]["rotation_scale"] = 0.25
    config["input"]["max_rotation_offset_rad"] = 0.20
    return config


def adapter(config):
    fields = InputConfig.__dataclass_fields__
    return InputAdapter(InputConfig(**{key: value for key, value in config["input"].items() if key in fields}))


def sample(stamp, *, grip=0.8, xyz=(0, 0, 0), rotation=(0, 0, 0)):
    return ControllerSample(
        stamp, Pose(np.array(xyz, dtype=float), rotvec_quaternion(np.array(rotation, dtype=float))), grip
    )


def arm(instance):
    instance.receive(sample(1, grip=0), 0)
    assert not instance.step(0, TCP, 0).enabled
    instance.receive(sample(2), 0.02)
    assert instance.step(0.02, TCP, 0.02).reason == "anchoring"
    assert instance.step(0.03, TCP, 0.03).enabled
    return instance


def test_only_three_explicitly_approved_profile_fields_changed():
    config = profile()
    assert config["input"]["rotation_scale"] == 1.0
    assert config["input"]["max_rotation_offset_rad"] == pytest.approx(math.pi / 4)
    assert config["safety"]["max_linear_velocity_m_s"] == 0.15
    protected = copy.deepcopy(config)
    protected["input"].pop("rotation_scale")
    protected["input"].pop("max_rotation_offset_rad")
    protected["safety"].pop("max_linear_velocity_m_s")
    # Frozen BEFORE this authorized change, all other model/input/Servo/PID/
    # hardware-admission/recording settings included, comments excluded.
    encoded = json.dumps(protected, sort_keys=True, separators=(",", ":")).encode()
    assert hashlib.sha256(encoded).hexdigest() == "4d055a41c0bbae9d1ea16457964f4bb65628cb8525ebdc7101b7cde9d643b166"


@pytest.mark.parametrize("axis", range(3))
@pytest.mark.parametrize("sign", [-1, 1])
def test_mapping_is_one_to_one_for_both_translation_and_small_rotation(axis, sign):
    current = arm(adapter(profile()))
    delta = np.zeros(3)
    delta[axis] = sign * 0.01
    current.receive(sample(3, xyz=delta, rotation=delta), 0.04)
    decision = current.step(0.04, TCP, 0.04)
    assert decision.enabled
    np.testing.assert_allclose(decision.target.xyz - TCP.xyz, SIDE_MOUNT_MAPPING @ delta, atol=1e-12)
    np.testing.assert_allclose(
        quaternion_rotvec(quaternion_difference(TCP.quaternion_xyzw, decision.target.quaternion_xyzw)),
        SIDE_MOUNT_MAPPING @ delta,
        atol=1e-12,
    )


def test_smooth_rotation_expands_only_to_approved_45_degree_cap():
    old, new = [arm(adapter(config)) for config in (previous_profile(), profile())]
    for index in range(1, 11):
        now = 0.03 + index * 0.02
        frame = sample(index + 2, rotation=(index * 0.1, 0, 0))
        for instance in (old, new):
            instance.receive(frame, now)
        before, after = [instance.step(now, TCP, now) for instance in (old, new)]
        assert before.enabled == after.enabled
        assert before.reason == after.reason == "tracking"
    assert np.linalg.norm(quaternion_rotvec(before.target.quaternion_xyzw)) == pytest.approx(0.20)
    assert np.linalg.norm(quaternion_rotvec(after.target.quaternion_xyzw)) == pytest.approx(math.pi / 4)


@pytest.mark.parametrize(
    "frame,now,feedback,tf_fresh,reason",
    [
        (sample(3, xyz=(0.051, 0, 0)), 0.04, 0.04, True, "xr_translation_jump"),
        (sample(3, rotation=(0.261, 0, 0)), 0.04, 0.04, True, "xr_rotation_jump"),
        (None, 0.23, 0.23, True, "xr_timeout"),
        (None, 0.04, None, True, "feedback_unavailable"),
        (None, 0.04, -0.20, True, "feedback_stale"),
        (None, 0.04, 0.04, False, "tcp_transform_unavailable"),
        (sample(3, grip=0), 0.04, 0.04, True, "released"),
    ],
)
def test_previously_rejected_inputs_are_still_rejected(frame, now, feedback, tf_fresh, reason):
    decisions = []
    for config in (previous_profile(), profile()):
        instance = arm(adapter(config))
        if frame is not None:
            instance.receive(frame, now)
        decision = instance.step(now, TCP, feedback, tf_fresh=tf_fresh)
        decisions.append(decision)
        assert decision.enabled is False and decision.target is None
        assert decision.reason == reason
    assert decisions[0].reason == decisions[1].reason


def test_speed_change_is_threefold_nominal_envelope_not_hand_position_scaling():
    old, new = previous_profile(), profile()
    gain = max(new["pose_tracking"][f"{axis}_proportional_gain"] for axis in "xyz")
    assert gain == 20.0
    assert old["safety"]["max_linear_velocity_m_s"] / gain == pytest.approx(0.0025)
    assert new["safety"]["max_linear_velocity_m_s"] / gain == pytest.approx(0.0075)
    # Position error at each rolling target is bounded; total Grip travel is not.
    assert new["input"]["position_scale"] == old["input"]["position_scale"] == 1.0
    assert new["safety"]["max_angular_velocity_rad_s"] == old["safety"]["max_angular_velocity_rad_s"] == 0.25
    assert new["safety"]["allow_hardware_motion"] is False
