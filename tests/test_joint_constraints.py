import numpy as np
import pytest
from unittest.mock import MagicMock
from xrobotoolkit_teleop.simulation.joint_constraints import JointConstraints


def _make_mock_robot(joint_names, lower, upper):
    """创建模拟 placo_robot，提供 joint_names 和 get_joint_limits 接口。"""
    robot = MagicMock()
    robot.joint_names.return_value = joint_names
    limits = {name: np.array([lo, up]) for name, lo, up in zip(joint_names, lower, upper)}
    robot.get_joint_limits = lambda name: limits[name]
    return robot


class TestJointConstraints:
    def test_clips_to_limits(self):
        """关节值超出限位时应被裁剪到限位范围内"""
        robot = _make_mock_robot(
            ["j1", "j2", "j3"],
            np.array([-1.0, -2.0, -3.0]),
            np.array([1.0, 2.0, 3.0]),
        )
        c = JointConstraints(placo_robot=robot, dt=0.01)
        q = np.array([5.0, 0.0, -5.0])  # j1 超上限, j3 超下限
        q_out = c.apply(q)
        assert q_out[0] == pytest.approx(1.0)  # clip 到上限
        assert q_out[1] == pytest.approx(0.0)
        assert q_out[2] == pytest.approx(-3.0)  # clip 到下限

    def test_within_limits_not_modified(self):
        """关节值在限位内时不做裁剪"""
        robot = _make_mock_robot(
            ["j1", "j2"],
            np.array([-1.0, -1.0]),
            np.array([1.0, 1.0]),
        )
        c = JointConstraints(placo_robot=robot, dt=0.01)
        q = np.array([0.5, -0.5])
        q_out = c.apply(q)
        assert q_out[0] == pytest.approx(0.5)
        assert q_out[1] == pytest.approx(-0.5)

    def test_velocity_limit_enforced(self):
        """启用了速度限制时，帧间变化不应超过 max_vel * dt"""
        robot = _make_mock_robot(
            ["j1", "j2"],
            np.array([-10.0, -10.0]),
            np.array([10.0, 10.0]),
        )
        c = JointConstraints(
            placo_robot=robot,
            dt=0.01,
            enable_velocity_limit=True,
            max_joint_velocity=1.0,  # rad/s
        )
        # 第一帧：无历史，透传
        q1 = np.array([0.0, 0.0])
        c.apply(q1)
        # 第二帧：从 0 到 5，超出 1.0*0.01=0.01
        q2 = np.array([5.0, 0.0])
        q_out = c.apply(q2)
        assert q_out[0] == pytest.approx(0.01)  # 限速到 0.01 rad
        assert q_out[1] == pytest.approx(0.0)

    def test_velocity_limit_off_by_default(self):
        """默认不开启速度限制，帧间跳变不被限制"""
        robot = _make_mock_robot(
            ["j1"],
            np.array([-10.0]),
            np.array([10.0]),
        )
        c = JointConstraints(placo_robot=robot, dt=0.01)
        c.apply(np.array([0.0]))
        q_out = c.apply(np.array([5.0]))  # 默认未开启速度限制
        assert q_out[0] == pytest.approx(5.0)  # 直接通过

    def test_boundary_values(self):
        """关节值等于限位边界时不被裁剪"""
        robot = _make_mock_robot(
            ["j1"],
            np.array([-1.0]),
            np.array([1.0]),
        )
        c = JointConstraints(placo_robot=robot, dt=0.01)
        q_out = c.apply(np.array([1.0]))
        assert q_out[0] == pytest.approx(1.0)
        q_out = c.apply(np.array([-1.0]))
        assert q_out[0] == pytest.approx(-1.0)
