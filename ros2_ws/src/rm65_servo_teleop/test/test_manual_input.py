"""Input/HTTP contracts, exercised without ROS or fabricated robot feedback."""

import json
import math
import re
import threading
from urllib.error import HTTPError
from urllib.request import ProxyHandler, Request, build_opener

import numpy as np
import pytest

from rm65_servo_teleop.input_adapter import (
    InputAdapter,
    InputConfig,
    Pose,
    decode_raw_packet,
    quaternion_difference,
    quaternion_rotvec,
    rotvec_quaternion,
)
from rm65_servo_teleop.manual_input import ManualInput
from rm65_servo_teleop.manual_node import make_server

CLIENT_A = "manual_client_a"
CLIENT_B = "manual_client_b"
TCP = Pose([0.32, -0.18, 0.46], rotvec_quaternion(np.array([0.25, -0.10, 0.40])))


def intent(seq, *, pressed=False, axes=None, client=CLIENT_A):
    return {"client_id": client, "seq": seq, "pressed": pressed, "axes": axes or [0] * 6}


class InputChain:
    """Drive the real adapter with a fresh measured TCP supplied by the test."""

    def __init__(self, position_scale=1.0, rotation_scale=0.25):
        self.manual = ManualInput(position_scale=position_scale, rotation_scale=rotation_scale)
        self.adapter = InputAdapter(
            InputConfig(position_scale=position_scale, rotation_scale=rotation_scale, allow_synthetic=True)
        )
        self.now, self.seq = 1.0, 0

    def tick(self, *, ready=True, tcp=TCP):
        raw = self.manual.packet(self.now, ready=ready)
        self.adapter.receive_raw(raw, self.now)
        return self.adapter.step(self.now, tcp, self.now), decode_raw_packet(raw)

    def submit(self, pressed, axes=None, *, dt=0.02, ready=True, tcp=TCP):
        self.now += dt
        self.seq += 1
        self.manual.submit(intent(self.seq, pressed=pressed, axes=axes), self.now)
        return self.tick(ready=ready, tcp=tcp)

    def engage(self, axes):
        released, _ = self.submit(False)
        assert not released.enabled
        anchored, _ = self.submit(True, axes)
        assert anchored.reason == "anchoring"
        assert not anchored.enabled
        np.testing.assert_allclose(anchored.target.xyz, TCP.xyz, atol=1e-12)
        np.testing.assert_allclose(anchored.target.quaternion_xyzw, TCP.quaternion_xyzw, atol=1e-12)


@pytest.mark.parametrize("axis", range(6))
@pytest.mark.parametrize("sign", [-1, 1])
@pytest.mark.parametrize("scales", [(1.0, 0.25), (0.4, 0.7)])
def test_each_manual_axis_is_in_base_frame_after_actual_adapter_mapping(axis, sign, scales):
    chain = InputChain(*scales)
    axes = [0] * 6
    axes[axis] = sign
    chain.engage(axes)
    for _ in range(20):
        decision, packet = chain.submit(True, axes)
        assert decision.enabled
        assert packet.synthetic
    translation = decision.target.xyz - TCP.xyz
    rotation = quaternion_rotvec(quaternion_difference(TCP.quaternion_xyzw, decision.target.quaternion_xyzw))
    expected_translation, expected_rotation = np.zeros(3), np.zeros(3)
    if axis < 3:
        expected_translation[axis] = sign * 0.02 * 0.4
    else:
        expected_rotation[axis - 3] = sign * 0.12 * 0.4
    # A rotated, nonzero TCP anchor distinguishes base-frame motion from local
    # tool-frame motion, and non-default scales catch accidental double scaling.
    np.testing.assert_allclose(translation, expected_translation, atol=1e-12)
    np.testing.assert_allclose(rotation, expected_rotation, atol=1e-12)


def test_diagonal_input_preserves_linear_and_angular_speed_norms():
    chain = InputChain()
    axes = [1, 1, -1, 1, -1, 1]
    chain.engage(axes)
    for _ in range(40):
        decision, _ = chain.submit(True, axes)
        assert decision.enabled
    translation = decision.target.xyz - TCP.xyz
    rotation = quaternion_rotvec(quaternion_difference(TCP.quaternion_xyzw, decision.target.quaternion_xyzw))
    assert np.linalg.norm(translation) == pytest.approx(0.02 * 0.8)
    assert np.linalg.norm(rotation) == pytest.approx(0.12 * 0.8)
    np.testing.assert_allclose(translation / np.linalg.norm(translation), np.array([1, 1, -1]) / math.sqrt(3))
    np.testing.assert_allclose(rotation / np.linalg.norm(rotation), np.array([1, -1, 1]) / math.sqrt(3))


@pytest.mark.parametrize("failure", ["timeout", "not_ready", "loop_stall"])
def test_failure_disarms_and_requires_a_new_release_before_reanchoring(failure):
    chain = InputChain()
    axes = [0, 1, 0, 0, 0, 0]
    chain.engage(axes)
    assert chain.submit(True, axes)[0].enabled
    if failure == "timeout":
        chain.now += 0.031
        stopped, packet = chain.tick()
        expected = "input_timeout"
    elif failure == "not_ready":
        stopped, packet = chain.submit(True, axes, ready=False)
        expected = "control_not_ready"
    else:
        # HTTP can remain alive while the separate publishing executor stalls.
        for _ in range(9):
            chain.now += 0.01
            chain.seq += 1
            chain.manual.submit(intent(chain.seq, pressed=True, axes=axes), chain.now)
        stopped, packet = chain.tick()
        expected = "input_loop_stalled"
    assert packet.grip == 0
    assert not stopped.enabled
    assert chain.manual.status()["reason"] == expected
    assert chain.manual.status()["release_required"]
    for _ in range(3):
        held, packet = chain.submit(True, axes)
        assert packet.grip == 0
        assert not held.enabled
    chain.submit(False)
    moved_tcp = Pose([0.4, 0.1, 0.5], rotvec_quaternion(np.array([-0.2, 0.3, 0.1])))
    anchored, _ = chain.submit(True, axes, tcp=moved_tcp)
    assert anchored.reason == "anchoring"
    np.testing.assert_allclose(anchored.target.xyz, moved_tcp.xyz)
    np.testing.assert_allclose(anchored.target.quaternion_xyzw, moved_tcp.quaternion_xyzw)
    assert chain.submit(True, axes, tcp=moved_tcp)[0].enabled


def test_duplicate_or_reordered_sequence_cannot_renew_an_input_lease():
    manual = ManualInput()
    manual.submit(intent(5), 1.0)
    manual.packet(1.0, ready=True)
    manual.submit(intent(6, pressed=True, axes=[1, 0, 0, 0, 0, 0]), 1.02)
    manual.packet(1.02, ready=True)
    for sequence in (6, 5):
        with pytest.raises(ValueError, match="old input sequence"):
            manual.submit(intent(sequence, pressed=True), 1.15)
    for now in np.arange(1.04, 1.20, 0.02):
        manual.packet(float(now), ready=True)
    packet = decode_raw_packet(manual.packet(1.21, ready=True))
    assert packet.grip == 0
    assert manual.status()["reason"] == "input_timeout"


def test_second_client_cannot_steal_a_live_lease_and_expired_takeover_requires_release():
    manual = ManualInput()
    manual.submit(intent(1), 1.0)
    manual.packet(1.0, ready=True)
    manual.submit(intent(2, pressed=True, axes=[1, 0, 0, 0, 0, 0]), 1.02)
    manual.packet(1.02, ready=True)
    with pytest.raises(PermissionError, match="another control page"):
        manual.submit(intent(1, client=CLIENT_B), 1.04)
    assert manual.status()["active"]
    # Rejected client B did not extend client A's lease. A B press can claim
    # the expired lease, but cannot carry the former client's held state over.
    manual.submit(intent(1, pressed=True, client=CLIENT_B), 1.23)
    assert decode_raw_packet(manual.packet(1.23, ready=True)).grip == 0
    assert manual.status()["release_required"]
    manual.submit(intent(2, client=CLIENT_B), 1.25)
    manual.packet(1.25, ready=True)
    manual.submit(intent(3, pressed=True, axes=[0, 1, 0, 0, 0, 0], client=CLIENT_B), 1.27)
    assert decode_raw_packet(manual.packet(1.27, ready=True)).grip > 0.2


@pytest.mark.parametrize("axis, limit", [(0, 0.10), (3, 0.16)])
def test_a_single_held_segment_stops_at_its_translation_or_rotation_envelope(axis, limit):
    chain = InputChain()
    axes = [0] * 6
    axes[axis] = 1
    chain.engage(axes)
    excursions = []
    for _ in range(300):
        decision, packet = chain.submit(True, axes)
        if packet.grip == 0:
            break
        assert decision.enabled
        delta = (
            decision.target.xyz - TCP.xyz
            if axis < 3
            else quaternion_rotvec(quaternion_difference(TCP.quaternion_xyzw, decision.target.quaternion_xyzw))
        )
        excursions.append(float(np.linalg.norm(delta)))
    else:
        pytest.fail("held segment did not stop at its configured envelope")
    assert chain.manual.status()["reason"] == "segment_limit_release_to_continue"
    assert not decision.enabled
    assert max(excursions) <= limit + 1e-12
    assert max(excursions) > limit * 0.95
    assert chain.submit(True, axes)[1].grip == 0


@pytest.fixture
def panel():
    manual = ManualInput()
    server = make_server(
        ("127.0.0.1", 0),
        manual,
        lambda: {"mode": "simulation_manual", "control": manual.status()},
        '<html><script>const token="__RM65_TOKEN__";</script></html>',
    )
    thread = threading.Thread(target=server.serve_forever, kwargs={"poll_interval": 0.01}, daemon=True)
    thread.start()
    opener = build_opener(ProxyHandler({}))
    base = f"http://127.0.0.1:{server.server_port}"

    def request(path, payload=None, headers=None):
        body = None if payload is None else json.dumps(payload).encode()
        request_headers = {"Content-Type": "application/json", **(headers or {})}
        try:
            response = opener.open(Request(base + path, data=body, headers=request_headers), timeout=1)
        except HTTPError as error:
            response = error
        with response:
            return response.status, response.headers, response.read().decode()

    try:
        yield manual, base, request
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=1)
        assert not thread.is_alive()


def test_http_panel_serves_a_fresh_token_and_accepts_local_authenticated_input(panel):
    manual, base, request = panel
    status, headers, html = request("/")
    assert status == 200
    assert headers["Cache-Control"] == "no-store"
    assert "frame-ancestors 'none'" in headers["Content-Security-Policy"]
    assert "__RM65_TOKEN__" not in html
    token = re.search(r"[0-9a-f]{64}", html).group(0)
    payload = {**intent(1), "token": token}
    assert request("/api/input", payload, {"Origin": base})[0] == 200
    payload.update(seq=2, pressed=True, axes=[0, 1, 0, 0, 0, 0])
    assert request("/api/input", payload)[0] == 200
    assert manual.status()["active"]
    status, _, raw = request("/api/state")
    assert status == 200
    assert json.loads(raw)["control"]["active"] is True
    assert request("/api/input", payload)[0] == 400


@pytest.mark.parametrize(
    "wrong_token, headers",
    [
        (True, {}),
        (False, {"Origin": "http://untrusted.example"}),
        (False, {"Host": "untrusted.example"}),
        (False, {"Host": "user@127.0.0.1"}),
    ],
)
def test_http_rejects_untrusted_requests_without_consuming_the_sequence(panel, wrong_token, headers):
    manual, _, request = panel
    token = re.search(r"[0-9a-f]{64}", request("/")[2]).group(0)
    payload = {**intent(1), "token": "0" * 64 if wrong_token else token}
    assert request("/api/input", payload, headers)[0] == 403
    assert manual.status()["release_required"]
    assert not manual.status()["active"]
    # Same sequence still works after fixing the rejected authentication.
    payload["token"] = token
    assert request("/api/input", payload)[0] == 200
    assert not manual.status()["release_required"]


def test_http_invalid_axes_do_not_acquire_the_input_lease(panel):
    manual, _, request = panel
    token = re.search(r"[0-9a-f]{64}", request("/")[2]).group(0)
    payload = {**intent(1), "token": token, "axes": [True, 0, 0, 0, 0, 0]}
    assert request("/api/input", payload)[0] == 400
    assert manual.status()["release_required"]
    payload["axes"] = [0] * 6
    assert request("/api/input", payload)[0] == 200


def test_shorter_lease_changes_only_the_expected_gap_and_keeps_release_fence():
    previous, current = ManualInput(timeout_s=0.18), ManualInput()
    for manual in (previous, current):
        manual.submit(intent(1), 1.0)
        manual.packet(1.0, ready=True)
        manual.submit(intent(2, pressed=True, axes=[1, 0, 0, 0, 0, 0]), 1.01)
    # The normal 10ms heartbeat path has identical movement and packet content.
    for seq in range(3, 12):
        now = 1.01 + (seq - 2) * 0.01
        for manual in (previous, current):
            manual.submit(intent(seq, pressed=True, axes=[1, 0, 0, 0, 0, 0]), now)
        assert json.loads(current.packet(now, ready=True)) == json.loads(previous.packet(now, ready=True))
    now += 0.031
    previous.packet(now, ready=True)
    current.packet(now, ready=True)
    assert previous.status()["active"]
    assert not current.status()["active"]
    assert current.status()["reason"] == "input_timeout"
    for seq in range(12, 16):
        now += 0.01
        current.submit(intent(seq, pressed=True, axes=[1, 0, 0, 0, 0, 0]), now)
        assert decode_raw_packet(current.packet(now, ready=True)).grip == 0
    current.submit(intent(16), now + 0.01)
    current.packet(now + 0.01, ready=True)
    current.submit(intent(17, pressed=True), now + 0.02)
    assert current.status()["active"]
