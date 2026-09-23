import numpy as np
import pytest

import xrobotoolkit_teleop.hardware.realman_official_ik as official_ik_module
from xrobotoolkit_teleop.hardware.realman_official_ik import (
    RealmanOfficialIKError,
    RealmanOfficialRemoteIK,
)


class _Matrix:
    def __init__(self):
        self.row = 0
        self.col = 0
        self.data = [[0.0 for _ in range(4)] for _ in range(4)]


class _Algo:
    def __init__(self, arm_model, force_type):
        self.arm_model = arm_model
        self.force_type = force_type
        self.init_args = None
        self.last_matrix = None
        self.last_seed_deg = None
        self.result_code = 0

    def rm_algo_ik_remote_init(self, period, tool_or_work):
        self.init_args = (period, tool_or_work)

    def rm_algo_ik_remote(self, matrix, seed_deg, output):
        self.last_matrix = matrix
        self.last_seed_deg = list(seed_deg)
        for index, value in enumerate(seed_deg):
            output[index] = value + 0.1
        return self.result_code

    def rm_algo_forward_kinematics(self, joint_deg, flag):
        self.last_forward_joint_deg = list(joint_deg)
        self.last_forward_flag = flag
        return [0.3, -0.1, 0.5, 2.0, 0.0, 0.0, 0.0]


class _Enum:
    RM_MODEL_RM_65_E = "RM65"
    RM_MODEL_RM_B_E = "B"


class _SDK:
    Algo = _Algo
    rm_Mat_t = _Matrix
    rm_robot_arm_model_e = _Enum
    rm_force_type_e = _Enum


def test_official_remote_ik_initializes_period_and_work_frame():
    solver = RealmanOfficialRemoteIK(0.02, tool_or_work=1, sdk_module=_SDK)

    assert solver._algo.init_args == (0.02, 1)


def test_official_remote_ik_converts_pose_and_joint_units():
    solver = RealmanOfficialRemoteIK(0.02, tool_or_work=1, sdk_module=_SDK)
    seed_rad = np.deg2rad(np.array([1.0, 2.0, 3.0, 4.0, 5.0, 6.0]))

    result = solver.solve(
        np.array([0.3, -0.1, 0.5]),
        np.array([1.0, 0.0, 0.0, 0.0]),
        seed_rad,
    )

    np.testing.assert_allclose(np.rad2deg(result), np.arange(1.1, 7.1, 1.0), atol=1e-6)
    np.testing.assert_allclose(solver._algo.last_seed_deg, np.arange(1.0, 7.0))
    np.testing.assert_allclose(
        np.array(solver._algo.last_matrix.data),
        np.array(
            [
                [1.0, 0.0, 0.0, 0.3],
                [0.0, 1.0, 0.0, -0.1],
                [0.0, 0.0, 1.0, 0.5],
                [0.0, 0.0, 0.0, 1.0],
            ]
        ),
    )


def test_official_remote_ik_normalizes_quaternion():
    transform = RealmanOfficialRemoteIK._target_matrix(
        np.zeros(3),
        np.array([0.0, 0.0, 0.0, 2.0]),
    )

    np.testing.assert_allclose(transform[:3, :3], np.diag([-1.0, -1.0, 1.0]))


def test_official_remote_ik_forward_uses_quaternion_pose_and_joint_degrees():
    solver = RealmanOfficialRemoteIK(0.02, sdk_module=_SDK)
    joint_rad = np.deg2rad(np.arange(1.0, 7.0))

    xyz, quat = solver.forward(joint_rad)

    np.testing.assert_allclose(xyz, [0.3, -0.1, 0.5])
    np.testing.assert_allclose(quat, [1.0, 0.0, 0.0, 0.0])
    np.testing.assert_allclose(solver._algo.last_forward_joint_deg, np.arange(1.0, 7.0))
    assert solver._algo.last_forward_flag == 0


def test_official_remote_ik_surfaces_sdk_error_code():
    solver = RealmanOfficialRemoteIK(0.02, sdk_module=_SDK)
    solver._algo.result_code = -2

    with pytest.raises(RealmanOfficialIKError, match="exceeds IK limits"):
        solver.solve(np.zeros(3), np.array([1.0, 0.0, 0.0, 0.0]), np.zeros(6))


def test_official_remote_ik_reports_missing_sdk(monkeypatch):
    def _missing_sdk(_name):
        raise ImportError("SDK unavailable")

    monkeypatch.setattr(official_ik_module.importlib, "import_module", _missing_sdk)

    with pytest.raises(RealmanOfficialIKError, match="official RealMan Python SDK"):
        RealmanOfficialRemoteIK(0.02)


@pytest.mark.parametrize("tool_or_work", [-1, 2, True])
def test_official_remote_ik_rejects_invalid_frame_mode(tool_or_work):
    with pytest.raises(ValueError, match="tool_or_work"):
        RealmanOfficialRemoteIK(0.02, tool_or_work=tool_or_work, sdk_module=_SDK)


def test_fk_only_adapter_does_not_require_or_initialize_remote_ik():
    class FKOnlyAlgo:
        def __init__(self, *args):
            pass

        def rm_algo_forward_kinematics(self, joints, flag):
            assert flag == 0
            return [0.3, 0, 0.5, 1, 0, 0, 0]

    class SDK(_SDK):
        Algo = FKOnlyAlgo

    solver = RealmanOfficialRemoteIK(0.02, sdk_module=SDK, fk_only=True)
    xyz, quat = solver.forward(np.zeros(6))
    np.testing.assert_allclose(xyz, [0.3, 0, 0.5])
    np.testing.assert_allclose(quat, [1, 0, 0, 0])


class _DH:
    def __init__(self, **values):
        self.values = values


class _ConfiguredAlgo(_Algo):
    def rm_algo_set_dh(self, dh):
        self.dh = dh.values

    def rm_algo_get_dh(self):
        return self.dh

    def rm_algo_euler2quaternion(self, euler):
        self.tool_euler = euler
        return [0.5, 0.5, 0.5, 0.5]


class _ConfiguredSDK(_SDK):
    Algo = _ConfiguredAlgo
    rm_dh_t = _DH


def _controller_dh():
    rows = [
        [0, 0, 240500, 0],
        [90000, 0, 0, 90000],
        [0, 256000, 0, 90000],
        [90000, 0, 210000, 0],
        [-90000, 0, 0, 0],
        [90000, 0, 161200, 0],
    ]
    return {"command": "get_DH_data", **{f"joint_{i+1}": row for i, row in enumerate(rows)}}


def test_controller_dh_uses_actual_dimensions_and_protocol_units():
    solver = RealmanOfficialRemoteIK(0.02, sdk_module=_ConfiguredSDK, fk_only=True)
    solver.configure_controller_dh(_controller_dh())
    np.testing.assert_allclose(solver._algo.dh["d"], [0.2405, 0, 0, 0.21, 0, 0.1612])
    np.testing.assert_allclose(solver._algo.dh["a"], [0, 0, 0.256, 0, 0, 0])
    np.testing.assert_allclose(solver._algo.dh["alpha"], [0, 90, 0, 90, -90, 90])
    np.testing.assert_allclose(solver._algo.dh["offset"], [0, 90, 90, 0, 0, 0])


@pytest.mark.parametrize("bad", [None, [], [0, 0, float("nan"), 0]])
def test_controller_dh_rejects_bad_joint_before_setting_sdk(bad):
    solver = RealmanOfficialRemoteIK(0.02, sdk_module=_ConfiguredSDK, fk_only=True)
    response = _controller_dh()
    response["joint_6"] = bad
    with pytest.raises(ValueError):
        solver.configure_controller_dh(response)
    assert not hasattr(solver._algo, "dh")


def test_controller_dh_readback_must_match():
    solver = RealmanOfficialRemoteIK(0.02, sdk_module=_ConfiguredSDK, fk_only=True)
    solver._algo.rm_algo_get_dh = lambda: dict(d=[0] * 6, a=[0] * 6, alpha=[0] * 6, offset=[0] * 6)
    with pytest.raises(RealmanOfficialIKError, match="readback mismatch"):
        solver.configure_controller_dh(_controller_dh())


def test_controller_dh_cannot_reconfigure_continuous_ik():
    solver = RealmanOfficialRemoteIK(0.02, sdk_module=_ConfiguredSDK)
    with pytest.raises(RealmanOfficialIKError, match="FK-only"):
        solver.configure_controller_dh(_controller_dh())


def test_controller_tool_pose_uses_euler_not_rotation_vector():
    solver = RealmanOfficialRemoteIK(0.02, sdk_module=_ConfiguredSDK, fk_only=True)
    result = solver.controller_tool_pose(dict(state="current_tool_frame", pose=[-3529, 2620, 69860, -50, 38, 1582]))
    np.testing.assert_allclose(solver._algo.tool_euler, [-0.05, 0.038, 1.582])
    np.testing.assert_allclose(result, [-0.003529, 0.002620, 0.069860, 0.5, 0.5, 0.5, 0.5])
