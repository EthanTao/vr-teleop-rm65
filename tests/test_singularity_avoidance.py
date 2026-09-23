"""Tests for the structural singularity guard used by the RM65 hardware path."""

import math
import xml.etree.ElementTree as ElementTree

import numpy as np
import pytest

from xrobotoolkit_teleop.hardware.singularity_avoidance import (
    RM65_CHAIN,
    JointChain,
    SingularityGuard,
    rm65_forward_kinematics,
    rm65_geometric_jacobian,
)

URDF_PATH = "assets/realman/RM65-official/urdf/RM65-official-arm.urdf"
# The URDF stores rpy with four decimals, so the derived constants differ by design.
URDF_TOLERANCE_RAD = 1e-4

BUDGET_RAD = np.deg2rad(np.array([180.0, 180.0, 225.0, 225.0, 225.0, 225.0])) * 0.10 * 0.02


def deg(values) -> np.ndarray:
    return np.deg2rad(np.array(values, dtype=float))


def ratio(joint_rad: np.ndarray) -> float:
    singular_values = np.linalg.svd(rm65_geometric_jacobian(joint_rad), compute_uv=False)
    return float(singular_values[-1] / singular_values[0])


def numeric_jacobian(joint_rad: np.ndarray, chain: JointChain = RM65_CHAIN) -> np.ndarray:
    step = 1e-7
    base = rm65_forward_kinematics(joint_rad, chain)
    jacobian = np.zeros((6, 6))
    for index in range(6):
        shifted = np.array(joint_rad, dtype=float)
        shifted[index] += step
        moved = rm65_forward_kinematics(shifted, chain)
        jacobian[:3, index] = (moved[:3, 3] - base[:3, 3]) / step
        rotation = moved[:3, :3] @ base[:3, :3].T
        jacobian[3:, index] = [
            (rotation[2, 1] - rotation[1, 2]) / (2.0 * step),
            (rotation[0, 2] - rotation[2, 0]) / (2.0 * step),
            (rotation[1, 0] - rotation[0, 1]) / (2.0 * step),
        ]
    return jacobian


def parse_urdf_chain(path: str = URDF_PATH) -> JointChain:
    root = ElementTree.parse(path).getroot()
    xyz = []
    rpy = []
    for index in range(1, 7):
        joint = root.find(f".//joint[@name='joint{index}']")
        assert joint is not None, f"joint{index} missing from {path}"
        origin = joint.find("origin")
        assert origin is not None, f"joint{index} has no origin in {path}"
        assert joint.find("axis").get("xyz").split() == ["0", "0", "1"]
        xyz.append([float(value) for value in origin.get("xyz").split()])
        rpy.append([float(value) for value in origin.get("rpy").split()])
    return JointChain(origin_xyz_m=np.array(xyz), origin_rpy_rad=np.array(rpy))


def test_hardcoded_chain_matches_the_vendor_urdf():
    parsed = parse_urdf_chain()

    np.testing.assert_allclose(parsed.origin_xyz_m, RM65_CHAIN.origin_xyz_m, atol=1e-9)
    np.testing.assert_allclose(parsed.origin_rpy_rad, RM65_CHAIN.origin_rpy_rad, atol=URDF_TOLERANCE_RAD)


def test_kinematics_match_the_urdf_derived_chain():
    parsed = parse_urdf_chain()
    joints = deg([10.0, -30.0, 55.0, 20.0, 40.0, -15.0])

    np.testing.assert_allclose(
        rm65_forward_kinematics(joints),
        rm65_forward_kinematics(joints, parsed),
        atol=1e-4,
    )
    np.testing.assert_allclose(
        rm65_geometric_jacobian(joints),
        rm65_geometric_jacobian(joints, parsed),
        atol=1e-4,
    )


def test_analytic_jacobian_matches_finite_differences():
    joints = deg([10.0, -30.0, 55.0, 20.0, 40.0, -15.0])

    np.testing.assert_allclose(rm65_geometric_jacobian(joints), numeric_jacobian(joints), atol=1e-5)


def test_chain_rejects_wrong_shapes_and_non_finite_values():
    with pytest.raises(ValueError, match="origin_xyz_m"):
        JointChain(origin_xyz_m=np.zeros((5, 3)), origin_rpy_rad=np.zeros((6, 3)))
    with pytest.raises(ValueError, match="origin_rpy_rad"):
        JointChain(origin_xyz_m=np.zeros((6, 3)), origin_rpy_rad=np.full((6, 3), np.nan))


@pytest.mark.parametrize("index", [2, 4])
def test_documented_joint_singularity_drives_the_ratio_to_zero(index):
    singular = deg([0.0, -30.0, 55.0, 0.0, 30.0, 0.0])
    singular[index] = 0.0
    away = deg([0.0, -30.0, 55.0, 0.0, 30.0, 0.0])
    away[index] = math.radians(30.0)

    assert ratio(singular) < 1e-6
    assert ratio(away) > 100.0 * max(ratio(singular), 1e-12)


def test_shoulder_singularity_is_detected_although_j3_and_j5_look_healthy():
    # The wrist centre of this pose sits ~3 mm from the J1 axis, yet J3 and J5 are mid-range.
    pocket = deg([0.0, -25.0, 55.0, 0.0, 60.0, 0.0])
    nearby = deg([0.0, -20.0, 60.0, 0.0, 60.0, 0.0])
    guard = SingularityGuard()

    assert abs(pocket[2]) > math.radians(30.0) and abs(pocket[4]) > math.radians(30.0)
    assert ratio(pocket) < guard.stop_ratio
    assert guard.state(pocket).zone == "danger"
    assert guard.state(nearby).zone == "clear"


def test_state_zones_and_speed_scale_follow_the_configured_ramps():
    guard = SingularityGuard()
    clear = guard.state(deg([0.0, -30.0, 60.0, 0.0, 60.0, 0.0]))
    slowdown = guard.state(deg([0.0, -30.0, 55.0, 0.0, 10.0, 0.0]))
    danger = guard.state(deg([0.0, -30.0, 55.0, 0.0, 2.0, 0.0]))

    assert clear.zone == "clear" and clear.severity == 0.0 and clear.speed_scale == 1.0
    assert slowdown.zone == "slowdown" and 0.0 < slowdown.severity < 1.0
    assert danger.zone == "danger" and danger.severity == 1.0
    assert danger.speed_scale == pytest.approx(guard.min_speed_scale)
    assert guard.min_speed_scale < slowdown.speed_scale < 1.0


def test_measure_reports_reasons_for_documented_criteria():
    guard = SingularityGuard()
    measure = guard.measure(deg([0.0, -30.0, 3.0, 0.0, 4.0, 0.0]))

    assert any("elbow J3" in reason for reason in measure.reasons)
    assert any("wrist J5" in reason for reason in measure.reasons)
    assert measure.sigma_ratio < guard.slowdown_ratio
    assert measure.score > 2.0
    assert guard.state(deg([0.0, -30.0, 3.0, 0.0, 4.0, 0.0])).zone == "danger"


@pytest.mark.parametrize(
    "kwargs",
    [
        {"slowdown_ratio": 0.001, "stop_ratio": 0.002},
        {"slowdown_ratio": 1.0},
        {"stop_ratio": 0.0},
        {"slowdown_joint_rad": math.radians(5.0), "stop_joint_rad": math.radians(10.0)},
        {"min_speed_scale": 0.0},
        {"escape_speed_scale": 0.0},
        {"min_speed_scale": 0.6, "escape_speed_scale": 0.5},
        {"max_damping": 0.0},
        {"score_tolerance": -1.0},
    ],
)
def test_guard_rejects_inconsistent_thresholds(kwargs):
    with pytest.raises(ValueError):
        SingularityGuard(**kwargs)


def test_damped_step_stays_bounded_where_the_plain_pseudoinverse_explodes():
    guard = SingularityGuard()
    joints = deg([0.0, -25.0, 55.0, 0.0, 60.0, 0.0])
    # The hardest Cartesian direction is the one this near-singular pose cannot serve.
    hardest = np.linalg.svd(rm65_geometric_jacobian(joints))[0][:, -1]
    twist = hardest * 0.01

    plain = guard.damped_joint_step(joints, twist, 0.0)
    damped = guard.damped_joint_step(joints, twist, guard.damping_for(guard.state(joints).severity))

    # Undamped: several hundred degrees of joint motion for a 10 mm request.
    assert np.max(np.abs(plain)) > math.radians(100.0)
    assert np.max(np.abs(damped)) < np.deg2rad(1.0)


def test_damping_is_negligible_far_from_every_singularity():
    guard = SingularityGuard()
    joints = deg([0.0, -30.0, 60.0, 0.0, 60.0, 0.0])
    twist = np.zeros(6)
    twist[:3] = [0.001, 0.0, 0.0]

    plain = guard.damped_joint_step(joints, twist, 0.0)
    damped = guard.damped_joint_step(joints, twist, guard.damping_for(guard.state(joints).severity))

    np.testing.assert_allclose(plain, damped, atol=1e-9)
    assert guard.damping_for(0.0) == 0.0


@pytest.mark.parametrize(
    "twist",
    [np.zeros(5), np.zeros((6, 1)), np.array([0.0, 0.0, np.nan, 0.0, 0.0, 0.0])],
)
def test_damped_step_rejects_invalid_twist(twist):
    with pytest.raises(ValueError, match="twist"):
        SingularityGuard().damped_joint_step(deg([0.0, -30.0, 60.0, 0.0, 60.0, 0.0]), twist)


def test_joint_step_scale_only_reduces_and_only_when_over_budget():
    guard = SingularityGuard()

    assert guard.joint_step_scale(np.zeros(6), BUDGET_RAD) == 1.0
    assert guard.joint_step_scale(BUDGET_RAD * 0.5, BUDGET_RAD) == 1.0
    assert guard.joint_step_scale(BUDGET_RAD * 4.0, BUDGET_RAD) == pytest.approx(0.25)


def test_command_gate_passes_clear_motion_through_untouched():
    guard = SingularityGuard()
    joints = deg([0.0, -30.0, 60.0, 0.0, 60.0, 0.0])
    twist = np.zeros(6)
    twist[:3] = [0.001, 0.0, 0.0]
    twist[3] = 0.005

    gate = guard.command_gate(joints, twist, BUDGET_RAD)

    # Outside every ramp the guard must not intervene at all; "inward" may still be reported
    # for diagnostics but nothing is scaled, capped or held.
    assert gate.zone == "clear"
    assert not gate.hold
    assert gate.speed_scale == 1.0


def test_command_gate_holds_an_approach_and_allows_the_escape():
    guard = SingularityGuard()
    joints = deg([0.0, -30.0, 55.0, 0.0, 3.0, 0.0])
    column = rm65_geometric_jacobian(joints)[:, 4]

    inward = guard.command_gate(joints, column * math.radians(-0.5), BUDGET_RAD)
    outward = guard.command_gate(joints, column * math.radians(0.5), BUDGET_RAD)

    assert inward.hold and inward.inward
    assert inward.speed_scale == pytest.approx(guard.min_speed_scale)
    assert not outward.hold
    assert outward.speed_scale >= guard.escape_speed_scale


def test_command_gate_never_holds_a_stationary_command_at_a_singular_pose():
    guard = SingularityGuard()
    joints = np.zeros(6)

    gate = guard.command_gate(joints, np.zeros(6), BUDGET_RAD)

    assert gate.zone == "danger"
    assert not gate.hold
    assert not gate.inward


def test_command_gate_slows_an_approach_before_the_danger_zone():
    guard = SingularityGuard()
    joints = deg([0.0, -30.0, 55.0, 0.0, 10.0, 0.0])
    column = rm65_geometric_jacobian(joints)[:, 4]

    gate = guard.command_gate(joints, column * math.radians(-0.5), BUDGET_RAD)

    assert gate.zone == "slowdown"
    assert gate.inward and not gate.hold
    assert gate.speed_scale < 1.0


def test_joint_gate_holds_a_target_that_deepens_the_singularity():
    guard = SingularityGuard()
    current = deg([0.0, -25.0, 55.0, 0.0, 60.0, 0.0])
    # +x at this shoulder pocket reduces the ratio (validated in the calibration sweeps).
    step = guard.damped_joint_step(current, np.array([0.001, 0.0, 0.0, 0.0, 0.0, 0.0]), 0.0)
    step *= math.radians(0.5) / float(np.linalg.norm(step))

    held = guard.joint_gate(current, current + step)
    allowed = guard.joint_gate(current, current - step)

    assert guard.state(current).zone == "danger"
    assert held.inward and held.hold
    assert held.speed_scale == pytest.approx(guard.min_speed_scale)
    assert not allowed.hold
    assert allowed.speed_scale >= guard.escape_speed_scale


def test_guard_rejects_invalid_joint_vectors():
    guard = SingularityGuard()
    for joints in (np.zeros(5), np.zeros((6, 1)), np.array([0.0] * 5 + [np.inf])):
        with pytest.raises(ValueError, match="joint_rad"):
            guard.state(joints)
