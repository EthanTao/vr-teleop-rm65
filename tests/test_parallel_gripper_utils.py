import pytest

from xrobotoolkit_teleop.utils.parallel_gripper_utils import calc_parallel_gripper_position


def test_trigger_is_clamped_to_configured_range():
    assert calc_parallel_gripper_position(0.0, 1.0, -0.5) == pytest.approx(0.0)
    assert calc_parallel_gripper_position(0.0, 1.0, 1.5) == pytest.approx(1.0)


def test_trigger_interpolates_between_positions():
    assert calc_parallel_gripper_position(0.04, 0.01, 0.5) == pytest.approx(0.025)


def test_non_finite_gripper_input_is_rejected():
    with pytest.raises(ValueError):
        calc_parallel_gripper_position(0.0, 1.0, float("nan"))
