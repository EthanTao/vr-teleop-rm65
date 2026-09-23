import numpy as np
import pytest

from xrobotoolkit_teleop.hardware.teleop_input_conditioning import (
    continuous_radial_deadzone,
    rotation_gain_for_linear_speed,
    xr_pose_step,
)


def test_radial_deadzone_removes_inside_and_subtracts_outside():
    np.testing.assert_allclose(
        continuous_radial_deadzone(np.array([0.003, 0.0, 0.0]), 0.003),
        np.zeros(3),
    )
    np.testing.assert_allclose(
        continuous_radial_deadzone(np.array([0.003001, 0.0, 0.0]), 0.003),
        [0.000001, 0.0, 0.0],
        atol=1e-12,
    )


def test_radial_deadzone_supports_any_nonempty_1d_vector_without_mutation():
    vector = np.array([3.0, 4.0])
    original = vector.copy()
    result = continuous_radial_deadzone(vector, 1.0)
    np.testing.assert_allclose(result, [2.4, 3.2])
    np.testing.assert_array_equal(vector, original)
    result[0] = 99.0
    assert vector[0] == original[0]


@pytest.mark.parametrize("vector", [np.array([]), np.zeros((2, 1)), np.array([1.0, np.nan]), np.array([np.inf])])
def test_radial_deadzone_rejects_invalid_vectors(vector):
    with pytest.raises(ValueError):
        continuous_radial_deadzone(vector, 0.1)


@pytest.mark.parametrize("deadband", [np.nan, np.inf, -1.0])
def test_radial_deadzone_rejects_invalid_deadband(deadband):
    with pytest.raises(ValueError):
        continuous_radial_deadzone(np.array([1.0]), deadband)


def test_rotation_gain_is_clamped_and_linearly_interpolated():
    assert rotation_gain_for_linear_speed(0.05, 0.05, 0.25) == pytest.approx(1.0)
    assert rotation_gain_for_linear_speed(0.15, 0.05, 0.25) == pytest.approx(0.5)
    assert rotation_gain_for_linear_speed(0.25, 0.05, 0.25) == pytest.approx(0.0)
    assert rotation_gain_for_linear_speed(0.0, 0.05, 0.25) == pytest.approx(1.0)
    assert rotation_gain_for_linear_speed(0.3, 0.05, 0.25) == pytest.approx(0.0)


@pytest.mark.parametrize(
    "args",
    [
        (np.nan, 0.05, 0.25),
        (0.1, np.inf, 0.25),
        (0.1, -0.1, 0.25),
        (0.1, 0.25, 0.25),
        (0.1, 0.3, 0.25),
    ],
)
def test_rotation_gain_rejects_invalid_thresholds(args):
    with pytest.raises(ValueError):
        rotation_gain_for_linear_speed(*args)


def test_xr_pose_step_uses_translation_and_shortest_quaternion_angle():
    previous = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0])
    current = np.array([0.051, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0])
    assert xr_pose_step(previous, current) == pytest.approx((0.051, 0.0))

    quarter_turn = np.array([0.0, 0.0, 0.0, 0.0, 0.0, np.sqrt(0.5), np.sqrt(0.5)])
    assert xr_pose_step(previous, quarter_turn) == pytest.approx((0.0, np.pi / 2))


@pytest.mark.parametrize(
    "previous,current",
    [
        (np.zeros(6), np.zeros(7)),
        (np.zeros(7), np.zeros(8)),
        (np.array([0.0, 0.0, 0.0, 0.0, 0.0, np.nan, 1.0]), np.zeros(7)),
        (np.zeros(7), np.array([0.0, 0.0, 0.0, 0.0, 0.0, np.inf, 1.0])),
        (np.zeros(7), np.zeros(7)),
    ],
)
def test_xr_pose_step_rejects_wrong_nonfinite_or_zero_quaternions(previous, current):
    with pytest.raises(ValueError):
        xr_pose_step(previous, current)


def test_xr_pose_step_does_not_mutate_inputs_or_share_output_state():
    previous = np.array([1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 2.0])
    current = np.array([2.0, 2.0, 3.0, 0.0, 0.0, 0.0, 2.0])
    previous_copy, current_copy = previous.copy(), current.copy()
    result = xr_pose_step(previous, current)
    assert result == pytest.approx((1.0, 0.0))
    np.testing.assert_array_equal(previous, previous_copy)
    np.testing.assert_array_equal(current, current_copy)
    assert isinstance(result, tuple)

