import numpy as np
import pytest
from xrobotoolkit_teleop.simulation.motion_filter import MotionFilter


class TestMotionFilter:
    def test_deadband_zeroes_small_deltas(self):
        """平移分量小于 deadband 时应归零"""
        f = MotionFilter(deadband_m=0.01, deadband_rad=1.0, smooth_alpha_pos=1.0)
        delta_xyz = np.array([0.005, 0.02, 0.003])  # 0.005<0.01, 0.02>0.01, 0.003<0.01
        delta_rot = np.array([0.0, 0.0, 0.0])
        out_xyz, out_rot = f.apply(delta_xyz, delta_rot)
        assert out_xyz[0] == 0.0  # 0.005 < 0.01 → 归零
        assert out_xyz[1] == pytest.approx(0.02)  # 0.02 > 0.01 → 保留
        assert out_xyz[2] == 0.0  # 0.003 < 0.01 → 归零

    def test_rot_deadband_zeroes_small_rotations(self):
        """旋转分量小于 deadband 时应归零"""
        f = MotionFilter(deadband_m=1.0, deadband_rad=0.05, smooth_alpha_pos=1.0, smooth_alpha_rot=1.0)
        delta_xyz = np.zeros(3)
        delta_rot = np.array([0.03, 0.06, 0.01])  # 0.03<0.05, 0.06>0.05, 0.01<0.05
        out_xyz, out_rot = f.apply(delta_xyz, delta_rot)
        assert out_rot[0] == 0.0
        assert out_rot[1] == pytest.approx(0.06)
        assert out_rot[2] == 0.0

    def test_ema_smooths_over_time(self):
        """EMA 应以 alpha 系数趋近稳定值"""
        f = MotionFilter(deadband_m=0, deadband_rad=0, smooth_alpha_pos=0.5, smooth_alpha_rot=1.0)
        delta = np.array([1.0, 0.0, 0.0])
        # 第一次: y = 0.5*1 + 0.5*0 = 0.5
        out, _ = f.apply(delta, np.zeros(3))
        assert out[0] == pytest.approx(0.5)
        # 第二次: y = 0.5*1 + 0.5*0.5 = 0.75
        out, _ = f.apply(delta, np.zeros(3))
        assert out[0] == pytest.approx(0.75)
        # 第三次: y = 0.5*1 + 0.5*0.75 = 0.875
        out, _ = f.apply(delta, np.zeros(3))
        assert out[0] == pytest.approx(0.875)

    def test_reset_clears_history(self):
        """reset 后 XYZ 和 Rot 的 EMA 历史都应清空"""
        f = MotionFilter(deadband_m=0, deadband_rad=0, smooth_alpha_pos=0.5, smooth_alpha_rot=0.5)
        delta_xyz = np.array([1.0, 0.0, 0.0])
        delta_rot = np.array([1.0, 0.0, 0.0])
        f.apply(delta_xyz, delta_rot)  # xyz→0.5, rot→0.5
        f.apply(delta_xyz, delta_rot)  # xyz→0.75, rot→0.75
        f.reset()
        out_xyz, out_rot = f.apply(delta_xyz, delta_rot)
        assert out_xyz[0] == pytest.approx(0.5)  # 0 + 0.5*1 → 重新从零开始
        assert out_rot[0] == pytest.approx(0.5)  # 0 + 0.5*1 → 重新从零开始

    def test_alpha_one_passthrough(self):
        """alpha=1.0 时应透传（无平滑效果）"""
        f = MotionFilter(deadband_m=0, deadband_rad=0, smooth_alpha_pos=1.0)
        delta = np.array([0.42, 0.0, 0.0])
        out, _ = f.apply(delta, np.zeros(3))
        assert out[0] == pytest.approx(0.42)

    def test_components_independent(self):
        """每个分量独立 EMA 滤波"""
        f = MotionFilter(deadband_m=0, deadband_rad=0, smooth_alpha_pos=0.3)
        delta_xyz = np.array([1.0, 2.0, 3.0])
        out, _ = f.apply(delta_xyz, np.zeros(3))
        assert out[0] == pytest.approx(0.3)  # 0 + 0.3*1
        assert out[1] == pytest.approx(0.6)  # 0 + 0.3*2
        assert out[2] == pytest.approx(0.9)  # 0 + 0.3*3

    def test_negative_delta_deadband(self):
        """负数 delta 的死区行为应与正数对称"""
        f = MotionFilter(deadband_m=0.01, deadband_rad=0.05, smooth_alpha_pos=1.0, smooth_alpha_rot=1.0)
        delta_xyz = np.array([-0.005, -0.02, -0.003])
        delta_rot = np.array([-0.03, -0.06, -0.01])
        out_xyz, out_rot = f.apply(delta_xyz, delta_rot)
        assert out_xyz[0] == 0.0    # |-0.005| < 0.01 → 归零
        assert out_xyz[1] == pytest.approx(-0.02)  # |-0.02| > 0.01 → 保留
        assert out_xyz[2] == 0.0    # |-0.003| < 0.01 → 归零
        assert out_rot[0] == 0.0    # |-0.03| < 0.05 → 归零
        assert out_rot[1] == pytest.approx(-0.06)  # |-0.06| > 0.05 → 保留
        assert out_rot[2] == 0.0    # |-0.01| < 0.05 → 归零
