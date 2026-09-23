import numpy as np
import pytest

from xrobotoolkit_teleop.hardware.cartesian_command_limiter import (
    CartesianCommandLimiter,
)

DT = 0.02
IDENTITY = np.array([1.0, 0.0, 0.0, 0.0])


def make_limiter() -> CartesianCommandLimiter:
    return CartesianCommandLimiter(
        max_linear_velocity_m_s=0.15,
        max_linear_acceleration_m_s2=0.40,
        max_angular_velocity_rad_s=0.60,
        max_angular_acceleration_rad_s2=1.20,
    )


def quaternion_angle(q0: np.ndarray, q1: np.ndarray) -> float:
    q0 = q0 / np.linalg.norm(q0)
    q1 = q1 / np.linalg.norm(q1)
    return float(2.0 * np.arccos(np.clip(abs(np.dot(q0, q1)), 0.0, 1.0)))


def z_rotation(angle_rad: float) -> np.ndarray:
    return np.array([np.cos(angle_rad / 2.0), 0.0, 0.0, np.sin(angle_rad / 2.0)])


def test_first_translation_step_is_acceleration_limited_before_integration():
    limiter = make_limiter()
    limiter.reset(np.zeros(3), IDENTITY)

    xyz, _ = limiter.step(np.array([1.0, 0.0, 0.0]), IDENTITY, DT)

    np.testing.assert_allclose(limiter.linear_velocity, [0.008, 0.0, 0.0], atol=1e-15)
    np.testing.assert_allclose(xyz, [0.00016, 0.0, 0.0], atol=1e-15)


def test_translation_observed_and_state_rates_stay_within_limits():
    limiter = make_limiter()
    limiter.reset(np.zeros(3), IDENTITY)
    previous_position = limiter.position
    previous_velocity = limiter.linear_velocity

    for _ in range(250):
        position, _ = limiter.step(np.array([1.0, 0.0, 0.0]), IDENTITY, DT)
        velocity = limiter.linear_velocity
        observed_speed = np.linalg.norm(position - previous_position) / 0.02
        state_speed = np.linalg.norm(velocity)
        state_acceleration = np.linalg.norm(velocity - previous_velocity) / 0.02
        assert observed_speed <= 0.15 + 1e-12
        assert state_speed <= 0.15 + 1e-12
        assert state_acceleration <= 0.40 + 1e-12
        previous_position = position
        previous_velocity = velocity


def test_rotation_observed_and_state_rates_stay_within_limits():
    limiter = make_limiter()
    limiter.reset(np.zeros(3), IDENTITY)
    previous_quaternion = limiter.quaternion
    previous_velocity = limiter.angular_velocity
    target = z_rotation(np.pi / 2.0)

    for _ in range(250):
        _, quaternion = limiter.step(np.zeros(3), target, DT)
        velocity = limiter.angular_velocity
        observed_speed = quaternion_angle(previous_quaternion, quaternion) / 0.02
        state_speed = np.linalg.norm(velocity)
        state_acceleration = np.linalg.norm(velocity - previous_velocity) / 0.02
        assert observed_speed <= 0.60 + 1e-10
        assert state_speed <= 0.60 + 1e-12
        assert state_acceleration <= 1.20 + 1e-12
        previous_quaternion = quaternion
        previous_velocity = velocity


def test_translation_target_reversal_respects_acceleration_limit_without_snap():
    limiter = make_limiter()
    limiter.reset(np.zeros(3), IDENTITY)
    for _ in range(10):
        limiter.step(np.array([1.0, 0.0, 0.0]), IDENTITY, DT)
    previous_velocity = limiter.linear_velocity

    limiter.step(np.array([-1.0, 0.0, 0.0]), IDENTITY, DT)

    assert np.linalg.norm(limiter.linear_velocity - previous_velocity) <= 0.008 + 1e-12
    assert limiter.linear_velocity[0] > 0.0


def test_rotation_target_reversal_respects_acceleration_limit_without_snap():
    limiter = make_limiter()
    limiter.reset(np.zeros(3), IDENTITY)
    for _ in range(10):
        limiter.step(np.zeros(3), z_rotation(np.pi / 2.0), DT)
    previous_velocity = limiter.angular_velocity

    limiter.step(np.zeros(3), z_rotation(-np.pi / 2.0), DT)

    assert np.linalg.norm(limiter.angular_velocity - previous_velocity) <= 0.024 + 1e-12
    assert limiter.angular_velocity[2] > 0.0


def test_equivalent_quaternion_sign_produces_no_rotation():
    limiter = make_limiter()
    limiter.reset(np.zeros(3), IDENTITY)

    _, quaternion = limiter.step(np.zeros(3), -IDENTITY, DT)

    np.testing.assert_allclose(quaternion, IDENTITY, atol=1e-15)
    np.testing.assert_array_equal(limiter.angular_velocity, np.zeros(3))


def test_quarter_turn_uses_positive_shortest_world_z_axis():
    limiter = make_limiter()
    limiter.reset(np.zeros(3), IDENTITY)

    _, quaternion = limiter.step(np.zeros(3), z_rotation(np.pi / 2.0), DT)

    np.testing.assert_allclose(limiter.angular_velocity, [0.0, 0.0, 0.024], atol=1e-15)
    np.testing.assert_allclose(quaternion, z_rotation(0.00048), atol=1e-15)


def test_reachable_pose_converges_to_bounded_non_growing_neighborhood():
    limiter = make_limiter()
    limiter.reset(np.zeros(3), IDENTITY)
    target_xyz = np.array([0.08, -0.04, 0.02])
    target_quaternion = z_rotation(0.35)
    translation_errors = []
    rotation_errors = []

    for _ in range(2000):
        xyz, quaternion = limiter.step(target_xyz, target_quaternion, DT)
        translation_errors.append(float(np.linalg.norm(xyz - target_xyz)))
        rotation_errors.append(quaternion_angle(quaternion, target_quaternion))

    translation_envelopes = [max(translation_errors[index : index + 500]) for index in range(500, 2000, 500)]
    rotation_envelopes = [max(rotation_errors[index : index + 500]) for index in range(500, 2000, 500)]
    assert max(translation_errors[-100:]) < 0.001
    assert max(rotation_errors[-100:]) < 0.002
    assert max(translation_envelopes[1:]) <= translation_envelopes[0] + 1e-12
    assert max(rotation_envelopes[1:]) <= rotation_envelopes[0] + 1e-12


def test_reset_normalizes_pose_and_clears_nonzero_velocities():
    limiter = make_limiter()
    limiter.reset(np.zeros(3), IDENTITY)
    limiter.step(np.array([1.0, 0.0, 0.0]), z_rotation(1.0), DT)
    assert np.linalg.norm(limiter.linear_velocity) > 0.0
    assert np.linalg.norm(limiter.angular_velocity) > 0.0

    limiter.reset(np.array([1.0, 2.0, 3.0]), np.array([-2.0, 0.0, 0.0, 0.0]))

    np.testing.assert_array_equal(limiter.position, [1.0, 2.0, 3.0])
    np.testing.assert_array_equal(limiter.quaternion, IDENTITY)
    np.testing.assert_array_equal(limiter.linear_velocity, np.zeros(3))
    np.testing.assert_array_equal(limiter.angular_velocity, np.zeros(3))


def test_clear_invalidates_pose_and_step_until_next_reset():
    limiter = make_limiter()
    limiter.reset(np.zeros(3), IDENTITY)
    limiter.step(np.ones(3), z_rotation(0.5), DT)

    limiter.clear()

    with pytest.raises(RuntimeError):
        _ = limiter.position
    with pytest.raises(RuntimeError):
        _ = limiter.quaternion
    np.testing.assert_array_equal(limiter.linear_velocity, np.zeros(3))
    np.testing.assert_array_equal(limiter.angular_velocity, np.zeros(3))
    with pytest.raises(RuntimeError):
        limiter.step(np.zeros(3), IDENTITY, DT)


def test_inputs_outputs_and_properties_do_not_alias_internal_state():
    limiter = make_limiter()
    reset_xyz = np.array([0.1, 0.2, 0.3])
    reset_quaternion = np.array([2.0, 0.0, 0.0, 0.0])
    limiter.reset(reset_xyz, reset_quaternion)
    reset_xyz[:] = 9.0
    reset_quaternion[:] = 9.0
    target_xyz = np.array([1.0, 0.2, 0.3])
    target_quaternion = z_rotation(0.5)
    target_xyz_copy = target_xyz.copy()
    target_quaternion_copy = target_quaternion.copy()

    position, quaternion = limiter.step(target_xyz, target_quaternion, DT)
    position[:] = 7.0
    quaternion[:] = 7.0
    property_position = limiter.position
    property_quaternion = limiter.quaternion
    property_linear_velocity = limiter.linear_velocity
    property_angular_velocity = limiter.angular_velocity
    property_position[:] = 8.0
    property_quaternion[:] = 8.0
    property_linear_velocity[:] = 8.0
    property_angular_velocity[:] = 8.0

    np.testing.assert_array_equal(target_xyz, target_xyz_copy)
    np.testing.assert_array_equal(target_quaternion, target_quaternion_copy)
    assert limiter.position[0] < 1.0
    assert limiter.quaternion[0] <= 1.0
    assert np.linalg.norm(limiter.linear_velocity) <= 0.15
    assert np.linalg.norm(limiter.angular_velocity) <= 0.60


@pytest.mark.parametrize("value", [0.0, -1.0, np.nan, np.inf, -np.inf, "invalid", None])
@pytest.mark.parametrize("limit_index", range(4))
def test_constructor_rejects_limits_that_are_not_finite_positive_numbers(value, limit_index):
    limits = [0.15, 0.40, 0.60, 1.20]
    limits[limit_index] = value
    with pytest.raises(ValueError):
        CartesianCommandLimiter(*limits)


@pytest.mark.parametrize(
    "xyz,quaternion",
    [
        (np.zeros(2), IDENTITY),
        (np.zeros((3, 1)), IDENTITY),
        (np.array([0.0, np.nan, 0.0]), IDENTITY),
        (np.zeros(3), np.zeros(3)),
        (np.zeros(3), np.zeros(4)),
        (np.zeros(3), np.array([1.0, 0.0, np.inf, 0.0])),
        (["bad", 0.0, 0.0], IDENTITY),
    ],
)
def test_reset_rejects_invalid_pose(xyz, quaternion):
    with pytest.raises(ValueError):
        make_limiter().reset(xyz, quaternion)


@pytest.mark.parametrize(
    "target_xyz,target_quaternion,dt",
    [
        (np.zeros(2), IDENTITY, DT),
        (np.array([0.0, np.inf, 0.0]), IDENTITY, DT),
        (np.zeros(3), np.zeros(3), DT),
        (np.zeros(3), np.zeros(4), DT),
        (np.zeros(3), np.array([1.0, np.nan, 0.0, 0.0]), DT),
        (np.zeros(3), IDENTITY, 0.0),
        (np.zeros(3), IDENTITY, -DT),
        (np.zeros(3), IDENTITY, np.nan),
        (np.zeros(3), IDENTITY, np.inf),
        (np.zeros(3), IDENTITY, "invalid"),
    ],
)
def test_step_rejects_invalid_target_pose_or_dt(target_xyz, target_quaternion, dt):
    limiter = make_limiter()
    limiter.reset(np.zeros(3), IDENTITY)
    with pytest.raises(ValueError):
        limiter.step(target_xyz, target_quaternion, dt)


def test_pose_properties_and_step_fail_before_reset():
    limiter = make_limiter()
    with pytest.raises(RuntimeError):
        _ = limiter.position
    with pytest.raises(RuntimeError):
        _ = limiter.quaternion
    with pytest.raises(RuntimeError):
        limiter.step(np.zeros(3), IDENTITY, DT)


def test_speed_scale_reduces_translation_velocity_without_weakening_braking():
    limiter = make_limiter()
    limiter.reset(np.zeros(3), IDENTITY)
    limiter.set_speed_scale(0.25)
    previous_velocity = limiter.linear_velocity
    peak_speed = 0.0

    for _ in range(300):
        position, _ = limiter.step(np.array([1.0, 0.0, 0.0]), IDENTITY, DT)
        velocity = limiter.linear_velocity
        peak_speed = max(peak_speed, float(np.linalg.norm(velocity)))
        assert float(np.linalg.norm(velocity - previous_velocity)) <= 0.40 * DT + 1e-12
        assert float(np.linalg.norm(position)) <= 0.15 * 0.25 * 300 * DT + 1e-9
        previous_velocity = velocity

    assert peak_speed <= 0.15 * 0.25 + 1e-12
    assert limiter.speed_scale == pytest.approx(0.25)


def test_speed_scale_reduces_angular_velocity():
    limiter = make_limiter()
    limiter.reset(np.zeros(3), IDENTITY)
    limiter.set_speed_scale(0.2)
    target = z_rotation(np.pi / 2.0)
    peak_speed = 0.0

    for _ in range(300):
        limiter.step(np.zeros(3), target, DT)
        peak_speed = max(peak_speed, float(np.linalg.norm(limiter.angular_velocity)))

    assert peak_speed <= 0.60 * 0.2 + 1e-12


def test_speed_scale_of_one_keeps_the_original_limits():
    limiter = make_limiter()
    limiter.reset(np.zeros(3), IDENTITY)
    limiter.set_speed_scale(1.0)

    for _ in range(250):
        limiter.step(np.array([1.0, 0.0, 0.0]), IDENTITY, DT)

    assert limiter.speed_scale == 1.0
    assert float(np.linalg.norm(limiter.linear_velocity)) <= 0.15 + 1e-12


def test_speed_scale_cannot_exceed_one_and_is_clamped_below():
    limiter = make_limiter()

    limiter.set_speed_scale(5.0)
    assert limiter.speed_scale == 1.0

    limiter.set_speed_scale(-2.0)
    assert limiter.speed_scale == 0.0


@pytest.mark.parametrize("value", [np.nan, np.inf, -np.inf, "invalid", None])
def test_speed_scale_rejects_non_finite_values(value):
    limiter = make_limiter()

    with pytest.raises(ValueError):
        limiter.set_speed_scale(value)


def test_speed_scale_composes_with_reset_without_enabling_more_motion():
    limiter = make_limiter()
    limiter.reset(np.zeros(3), IDENTITY)
    limiter.set_speed_scale(0.1)

    limiter.reset(np.zeros(3), IDENTITY)
    for _ in range(200):
        limiter.step(np.array([1.0, 0.0, 0.0]), IDENTITY, DT)

    assert float(np.linalg.norm(limiter.linear_velocity)) <= 0.15 * 0.1 + 1e-12
