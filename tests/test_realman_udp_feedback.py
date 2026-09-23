import json
import math
import socket
import time

import numpy as np
import pytest

from xrobotoolkit_teleop.hardware.realman_udp_feedback import (
    ArmFeedback,
    RealmanUDPFeedbackReceiver,
    parse_realtime_arm_state,
)


VALID_PACKET = {
    "joint_status": {
        "joint_position": [1000, -2000, 3000, -4000, 5000, -6000],
        "joint_speed": [100, -200, 300, -400, 500, -600],
    },
    "waypoint": {
        "position": [300000, -100000, 500000],
        "quat": [1000000, 0, 0, 0],
    },
    "err": {"err_len": 2, "err_code": [0, 0]},
}


def _packet_bytes(packet=VALID_PACKET):
    return json.dumps(packet).encode("utf-8")


def _wait_until(predicate, timeout_s=1.0):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.01)
    pytest.fail("condition was not met before timeout")


def test_parser_converts_the_documented_raw_units_exactly():
    feedback = parse_realtime_arm_state(_packet_bytes(), received_monotonic_s=12.5)

    assert feedback.received_monotonic_s == 12.5
    np.testing.assert_allclose(feedback.joint_rad, np.deg2rad([1, -2, 3, -4, 5, -6]))
    np.testing.assert_allclose(feedback.tcp_xyz_m, [0.3, -0.1, 0.5])
    np.testing.assert_allclose(feedback.tcp_quat_wxyz, [1.0, 0.0, 0.0, 0.0])
    np.testing.assert_allclose(feedback.joint_speed_rad_s, np.deg2rad([1, -2, 3, -4, 5, -6]))
    assert feedback.arm_error_codes == ()


def test_parser_keeps_joint_speed_optional_for_diagnostics_but_marks_it_missing():
    packet = json.loads(_packet_bytes())
    del packet["joint_status"]["joint_speed"]

    feedback = parse_realtime_arm_state(_packet_bytes(packet), received_monotonic_s=1.0)

    assert feedback.joint_speed_rad_s is None


def test_parser_normalizes_quaternion_and_uses_non_negative_w_sign():
    packet = json.loads(_packet_bytes())
    packet["waypoint"]["quat"] = [-2000000, 0, 0, 0, 123]

    feedback = parse_realtime_arm_state(_packet_bytes(packet), received_monotonic_s=1.0)

    np.testing.assert_allclose(feedback.tcp_quat_wxyz, [1.0, 0.0, 0.0, 0.0])


@pytest.mark.parametrize(
    ("err", "expected"),
    [
        ({}, ()),
        ({"err_code": 7}, (7,)),
        ({"err_code": [0, 3, -2, 0]}, (3, -2)),
    ],
)
def test_parser_extracts_only_nonzero_error_codes(err, expected):
    packet = json.loads(_packet_bytes())
    packet["err"] = err

    feedback = parse_realtime_arm_state(_packet_bytes(packet), received_monotonic_s=1.0)

    assert feedback.arm_error_codes == expected


@pytest.mark.parametrize(
    ("err", "expected"),
    [
        ([0], ()),
        ([], ()),
        ([0, 7, 0], (7,)),
        ([7, -2, 0], (7, -2)),
    ],
)
def test_parser_accepts_array_err_format_from_third_gen_controller(err, expected):
    packet = json.loads(_packet_bytes())
    packet["err"] = err

    feedback = parse_realtime_arm_state(_packet_bytes(packet), received_monotonic_s=1.0)

    assert feedback.arm_error_codes == expected


def test_parser_accepts_real_rm65_bi_v170_datagram():
    real = {
        "err": [0],
        "joint_status": {
            "joint_current": [-79000, -2258000, -1991000, 2000, 492000, -58000],
            "joint_en_flag": [1, 1, 1, 1, 1, 1],
            "joint_err_code": [0, 0, 0, 0, 0, 0],
            "joint_position": [-1968, -35327, 62138, 165412, -9854, -293299],
            "joint_speed": [0, 0, 0, 0, 0, 1],
            "joint_temperature": [35000, 39000, 35000, 37000, 37000, 39000],
            "joint_voltage": [23000, 23000, 23000, 23000, 23000, 23000],
        },
        "state": "realtime_arm_joint_state",
        "waypoint": {
            "euler": [-447, 551, 2325],
            "position": [-86608, 11750, 820234],
            "quat": [316706, -328490, -90433, 885218],
        },
    }

    feedback = parse_realtime_arm_state(
        json.dumps(real).encode("utf-8"), received_monotonic_s=1.0
    )

    assert feedback.arm_error_codes == ()
    np.testing.assert_allclose(
        feedback.joint_rad,
        np.deg2rad([-1.968, -35.327, 62.138, 165.412, -9.854, -293.299]),
    )
    np.testing.assert_allclose(feedback.tcp_xyz_m, [-0.086608, 0.01175, 0.820234])
    assert math.isclose(float(np.linalg.norm(feedback.tcp_quat_wxyz)), 1.0)
    np.testing.assert_allclose(
        feedback.joint_speed_rad_s,
        np.deg2rad([0, 0, 0, 0, 0, 0.01]),
    )


@pytest.mark.parametrize(
    "payload, timestamp",
    [
        (b"\xff", 1.0),
        (b"{", 1.0),
        (b"[]", 1.0),
        (b'{"joint_status": {}}', 1.0),
        (
            _packet_bytes(
                {
                    **VALID_PACKET,
                    "joint_status": {"joint_position": [1000] * 5},
                }
            ),
            1.0,
        ),
        (
            _packet_bytes(
                {
                    **VALID_PACKET,
                    "waypoint": {"position": [0, 0, "x"], "quat": [1, 0, 0, 0]},
                }
            ),
            1.0,
        ),
        (
            _packet_bytes(
                {
                    **VALID_PACKET,
                    "waypoint": {"position": [0, 0, 0], "quat": [0, 0, 0, 0]},
                }
            ),
            1.0,
        ),
        (
            _packet_bytes(
                {
                    **VALID_PACKET,
                    "joint_status": {"joint_position": [float("nan")] * 6},
                }
            ),
            1.0,
        ),
        (_packet_bytes(), float("nan")),
    ],
)
def test_parser_rejects_malformed_packets_and_timestamps(payload, timestamp):
    with pytest.raises(ValueError):
        parse_realtime_arm_state(payload, received_monotonic_s=timestamp)


@pytest.mark.parametrize("code", [True, 1.5, float("nan")])
def test_parser_rejects_non_integer_error_codes(code):
    packet = json.loads(_packet_bytes())
    packet["err"] = {"err_code": code}

    with pytest.raises(ValueError):
        parse_realtime_arm_state(_packet_bytes(packet), received_monotonic_s=1.0)


def test_parser_converts_oversized_json_integers_to_value_errors():
    packet = json.loads(_packet_bytes())
    packet["joint_status"]["joint_position"][0] = 10**400

    with pytest.raises(ValueError):
        parse_realtime_arm_state(_packet_bytes(packet), received_monotonic_s=1.0)


def test_feedback_defensively_copies_and_locks_its_arrays():
    joint = np.zeros(6)
    xyz = np.zeros(3)
    quat = np.array([1.0, 0.0, 0.0, 0.0])
    feedback = ArmFeedback(1.0, joint, xyz, quat, ())
    joint[0] = 99.0
    xyz[0] = 99.0
    quat[0] = 0.0

    assert feedback.joint_rad[0] == 0.0
    assert feedback.tcp_xyz_m[0] == 0.0
    assert feedback.tcp_quat_wxyz[0] == 1.0
    with pytest.raises(ValueError):
        feedback.joint_rad[0] = 1.0
    with pytest.raises(ValueError):
        feedback.tcp_xyz_m[0] = 1.0
    with pytest.raises(ValueError):
        feedback.tcp_quat_wxyz[0] = 0.0


def test_receiver_keeps_only_latest_frame_and_reports_age():
    receiver = RealmanUDPFeedbackReceiver(port=0)
    first = parse_realtime_arm_state(_packet_bytes(), 10.0)
    packet = json.loads(_packet_bytes())
    packet["joint_status"]["joint_position"][0] = 2000
    second = parse_realtime_arm_state(_packet_bytes(packet), 11.0)

    assert math.isinf(receiver.age_s(1.0))
    receiver._store_latest(first)
    receiver._store_latest(second)

    assert receiver.latest() is second
    assert receiver.age_s(12.25) == 1.25
    with pytest.raises(ValueError):
        receiver.age_s(float("inf"))


def test_receiver_counts_malformed_datagram_without_replacing_latest():
    receiver = RealmanUDPFeedbackReceiver(port=0)
    receiver.start()
    assert receiver._socket is not None
    port = receiver._socket.getsockname()[1]
    sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sender.sendto(_packet_bytes(), ("127.0.0.1", port))
        _wait_until(lambda: receiver.valid_packet_count == 1)
        latest = receiver.latest()
        assert latest is not None
        sender.sendto(b"not json", ("127.0.0.1", port))
        _wait_until(lambda: receiver.invalid_packet_count == 1)
        assert receiver.latest() is latest
        assert receiver.valid_packet_count == 1
    finally:
        sender.close()
        receiver.stop()


def test_receiver_counts_oversized_numeric_datagram_and_continues_receiving():
    oversized_packet = json.loads(_packet_bytes())
    oversized_packet["joint_status"]["joint_position"][0] = 10**400
    receiver = RealmanUDPFeedbackReceiver(port=0)
    receiver.start()
    assert receiver._socket is not None
    port = receiver._socket.getsockname()[1]
    sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sender.sendto(_packet_bytes(oversized_packet), ("127.0.0.1", port))
        _wait_until(lambda: receiver.invalid_packet_count == 1)
        assert receiver.latest() is None
        sender.sendto(_packet_bytes(), ("127.0.0.1", port))
        _wait_until(lambda: receiver.valid_packet_count == 1)
        assert receiver.thread_error is None
    finally:
        sender.close()
        receiver.stop()


def test_start_propagates_a_synchronous_bind_failure():
    occupied = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    occupied.bind(("127.0.0.1", 0))
    receiver = RealmanUDPFeedbackReceiver(bind_host="127.0.0.1", port=occupied.getsockname()[1])
    try:
        with pytest.raises(OSError):
            receiver.start()
    finally:
        occupied.close()
        receiver.stop()


def test_receiver_rejects_a_second_start_and_stop_is_idempotent():
    receiver = RealmanUDPFeedbackReceiver(port=0)
    receiver.start()
    try:
        with pytest.raises(RuntimeError):
            receiver.start()
    finally:
        receiver.stop()
        receiver.stop()


def test_concurrent_starts_create_exactly_one_receiver(monkeypatch):
    class BlockingSocket:
        def __init__(self):
            self.closed = threading.Event()

        def bind(self, address):
            bind_entered.set()
            release_bind.wait(timeout=1.0)

        def settimeout(self, timeout):
            self.timeout = timeout

        def recvfrom(self, size):
            if self.closed.wait(timeout=0.01):
                raise OSError("closed")
            raise socket.timeout()

        def close(self):
            self.closed.set()

    import threading
    import xrobotoolkit_teleop.hardware.realman_udp_feedback as feedback_module

    bind_entered = threading.Event()
    release_bind = threading.Event()
    created_sockets = []

    def make_socket(*args, **kwargs):
        created_sockets.append(BlockingSocket())
        return created_sockets[-1]

    monkeypatch.setattr(feedback_module.socket, "socket", make_socket)
    receiver = RealmanUDPFeedbackReceiver(port=0)
    results = []

    def start_receiver():
        try:
            receiver.start()
            results.append("started")
        except RuntimeError:
            results.append("already-started")

    first = threading.Thread(target=start_receiver)
    second = threading.Thread(target=start_receiver)
    first.start()
    assert bind_entered.wait(timeout=1.0)
    second.start()
    release_bind.set()
    first.join(timeout=1.0)
    second.join(timeout=1.0)
    try:
        assert sorted(results) == ["already-started", "started"]
        assert len(created_sockets) == 1
    finally:
        receiver.stop()
        for udp_socket in created_sockets:
            udp_socket.close()


def test_stop_during_bind_leaves_no_published_receiver_resources(monkeypatch):
    class BlockingSocket:
        def __init__(self):
            self.closed = threading.Event()

        def bind(self, address):
            bind_entered.set()
            release_bind.wait(timeout=1.0)

        def settimeout(self, timeout):
            self.timeout = timeout

        def recvfrom(self, size):
            if self.closed.wait(timeout=0.01):
                raise OSError("closed")
            raise socket.timeout()

        def close(self):
            self.closed.set()

    import threading
    import xrobotoolkit_teleop.hardware.realman_udp_feedback as feedback_module

    bind_entered = threading.Event()
    release_bind = threading.Event()
    monkeypatch.setattr(feedback_module.socket, "socket", lambda *args, **kwargs: BlockingSocket())
    receiver = RealmanUDPFeedbackReceiver(port=0)
    starter = threading.Thread(target=receiver.start)
    stopper = threading.Thread(target=receiver.stop)
    starter.start()
    assert bind_entered.wait(timeout=1.0)
    stopper.start()
    release_bind.set()
    starter.join(timeout=1.0)
    stopper.join(timeout=1.0)

    assert receiver._socket is None
    assert receiver._thread is None


def test_stop_clears_references_even_when_close_and_join_fail():
    class CloseFailureSocket:
        def close(self):
            raise OSError("close failed")

    class JoinFailureThread:
        def __init__(self):
            self.join_called = False

        def join(self, timeout):
            self.join_called = True
            raise RuntimeError("join failed")

    receiver = RealmanUDPFeedbackReceiver(port=0)
    thread = JoinFailureThread()
    receiver._socket = CloseFailureSocket()
    receiver._thread = thread
    receiver._lifecycle_state = "running"

    with pytest.raises(OSError, match="close failed"):
        receiver.stop()

    assert thread.join_called
    assert receiver._socket is None
    assert receiver._thread is None


def test_receiver_exposes_unexpected_socket_thread_errors(monkeypatch):
    class FailingSocket:
        def bind(self, address):
            self.address = address

        def settimeout(self, timeout):
            self.timeout = timeout

        def recvfrom(self, size):
            raise OSError("simulated receive failure")

        def close(self):
            pass

    import xrobotoolkit_teleop.hardware.realman_udp_feedback as feedback_module

    monkeypatch.setattr(feedback_module.socket, "socket", lambda *args, **kwargs: FailingSocket())
    receiver = RealmanUDPFeedbackReceiver(port=0)
    receiver.start()
    _wait_until(lambda: receiver.thread_error is not None)
    receiver.stop()

    assert "simulated receive failure" in receiver.thread_error
