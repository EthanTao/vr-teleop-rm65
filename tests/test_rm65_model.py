import numpy as np
import pytest

from xrobotoolkit_teleop.hardware.rm65_model import RM65ModelSpec, RM65_B_SPEC


def test_rm65_b_accepts_bi_product_variant():
    assert RM65_B_SPEC.matches_product("RM65-BI")
    assert not RM65_B_SPEC.matches_product("RM65-6F")


def test_rm65_b_joint_limits_are_stored_in_radians():
    np.testing.assert_allclose(
        np.rad2deg(RM65_B_SPEC.joint_limit_rad),
        [[-178, 178], [-130, 130], [-135, 135], [-178, 178], [-128, 128], [-360, 360]],
    )


def test_joint_feedback_more_than_two_degrees_outside_limit_faults():
    q = np.zeros(6)
    q[1] = np.deg2rad(132.1)
    valid, reason = RM65_B_SPEC.validate_joint_feedback(q, tolerance_rad=np.deg2rad(2.0))
    assert not valid
    assert "J2" in reason


def test_joint_feedback_near_q3_or_q5_only_warns():
    valid, reason = RM65_B_SPEC.validate_joint_feedback(np.zeros(6), tolerance_rad=np.deg2rad(2.0))
    warnings = RM65_B_SPEC.singularity_warnings(np.zeros(6), warning_rad=np.deg2rad(10.0))
    assert valid
    assert reason == ""
    assert warnings == ("J3 near 0 deg", "J5 near 0 deg")


@pytest.mark.parametrize(
    "kwargs",
    [
        {"product_prefix": "", "joint_limit_rad": np.zeros((6, 2))},
        {"product_prefix": "RM65-B", "joint_limit_rad": np.zeros((5, 2))},
        {"product_prefix": "RM65-B", "joint_limit_rad": np.full((6, 2), np.nan)},
        {"product_prefix": "RM65-B", "joint_limit_rad": np.zeros((6, 2))},
    ],
)
def test_constructor_rejects_invalid_joint_limit_configuration(kwargs):
    defaults = dict(
        product_prefix="RM65-B",
        joint_limit_rad=np.tile([[-1.0, 1.0]], (6, 1)),
        max_joint_velocity_rad_s=np.ones(6),
        max_tcp_linear_velocity_m_s=1.0,
        reach_m=1.0,
    )
    defaults.update(kwargs)
    with pytest.raises(ValueError):
        RM65ModelSpec(**defaults)


@pytest.mark.parametrize(
    "velocities",
    [np.ones(5), np.full(6, np.nan), np.zeros(6), np.full(6, -1.0)],
)
def test_constructor_rejects_invalid_velocity_configuration(velocities):
    with pytest.raises(ValueError):
        RM65ModelSpec(
            product_prefix="RM65-B",
            joint_limit_rad=np.tile([[-1.0, 1.0]], (6, 1)),
            max_joint_velocity_rad_s=velocities,
            max_tcp_linear_velocity_m_s=1.0,
            reach_m=1.0,
        )


@pytest.mark.parametrize(
    "speed, reach",
    [(np.nan, 1.0), (0.0, 1.0), (1.0, np.inf), (1.0, 0.0)],
)
def test_constructor_rejects_invalid_tcp_speed_or_reach(speed, reach):
    with pytest.raises(ValueError):
        RM65ModelSpec(
            product_prefix="RM65-B",
            joint_limit_rad=np.tile([[-1.0, 1.0]], (6, 1)),
            max_joint_velocity_rad_s=np.ones(6),
            max_tcp_linear_velocity_m_s=speed,
            reach_m=reach,
        )


@pytest.mark.parametrize("feedback", [np.zeros(5), np.zeros((6, 1)), np.full(6, np.nan)])
def test_joint_feedback_rejects_invalid_shape_or_non_finite_values(feedback):
    with pytest.raises(ValueError):
        RM65_B_SPEC.validate_joint_feedback(feedback, tolerance_rad=0.0)


@pytest.mark.parametrize("feedback", [np.zeros(5), np.zeros((6, 1)), np.full(6, np.inf)])
def test_singularity_warnings_return_empty_for_invalid_feedback(feedback):
    assert RM65_B_SPEC.singularity_warnings(feedback, warning_rad=1.0) == ()


@pytest.mark.parametrize("tolerance", [-1.0, np.nan, np.inf])
def test_joint_feedback_rejects_invalid_tolerance(tolerance):
    with pytest.raises(ValueError):
        RM65_B_SPEC.validate_joint_feedback(np.zeros(6), tolerance_rad=tolerance)


@pytest.mark.parametrize("warning", [-1.0, np.nan, np.inf])
def test_singularity_warnings_rejects_invalid_warning_threshold(warning):
    with pytest.raises(ValueError):
        RM65_B_SPEC.singularity_warnings(np.zeros(6), warning_rad=warning)


def test_constructor_copies_arrays_and_keeps_spec_arrays_read_only():
    limits = np.tile([[-1.0, 1.0]], (6, 1))
    velocities = np.ones(6)
    spec = RM65ModelSpec(
        product_prefix="RM65-B",
        joint_limit_rad=limits,
        max_joint_velocity_rad_s=velocities,
        max_tcp_linear_velocity_m_s=1.0,
        reach_m=1.0,
    )

    limits[0, 0] = -0.5
    velocities[0] = 2.0
    assert spec.joint_limit_rad[0, 0] == -1.0
    assert spec.max_joint_velocity_rad_s[0] == 1.0
    with pytest.raises(ValueError):
        spec.joint_limit_rad[0, 0] = -0.5
    with pytest.raises(ValueError):
        spec.max_joint_velocity_rad_s[0] = 2.0


def test_matches_product_strips_whitespace_and_ignores_case():
    assert RM65_B_SPEC.matches_product("  rm65-bi  ")
    assert not RM65_B_SPEC.matches_product("RM65")

