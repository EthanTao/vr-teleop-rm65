"""Device-free behavioral tests: TCP emulator, injected SDK and real file I/O."""

from dataclasses import replace
import json
from pathlib import Path
import socketserver
import threading
import time
from types import SimpleNamespace as NS

import numpy as np
import pytest
import yaml

from rm65_servo_teleop.camera import CameraConfig, MockCamera, RealSenseCamera
from rm65_servo_teleop.camera_runtime import CameraRuntime
from rm65_servo_teleop.camera_tools import main as camera_main
from rm65_servo_teleop.hand_driver import HandConnection, LIVE_ACK, RM65Hand
from rm65_servo_teleop.hand_protocol import HandError, encode_words
from rm65_servo_teleop.hand_runtime import HandRuntime
from rm65_servo_teleop.peripheral_recording import PeripheralRecorder, verify_capture
from rm65_servo_teleop.sensor_pairing import pair_capture

CONFIG = Path(__file__).resolve().parents[1] / "config"


def wait(predicate, timeout=3):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.01)
    pytest.fail("condition not reached within deadline")


@pytest.fixture
def controller():
    """Explicit loopback protocol emulator: this is never device evidence."""
    state = NS(requests=[], position=[4000] * 6, fault=None, identity=[256, 256, 1, 1, 1, 2])

    class Handler(socketserver.StreamRequestHandler):
        def handle(self):
            for line in self.rfile:
                req = json.loads(line)
                state.requests.append(req)
                if state.fault == "timeout":
                    time.sleep(0.3)
                    return
                if state.fault == "malformed":
                    self.wfile.write(b"not json\r\n")
                    self.wfile.flush()
                    continue
                command, address = req["command"], req["address"]
                if command == "write_registers":
                    if address == 1135:
                        state.position = [(req["data"][i] << 8) | req["data"][i + 1] for i in range(0, 12, 2)]
                    response = {"command": command, "write_state": state.fault != "write_rejected"}
                else:
                    words = {
                        1000: state.identity,
                        1085: [2] * 6,
                        1145: state.position,
                        1105: [40] * 6,
                        1165: [65535] * 6,
                    }[address]
                    response = {"command": command, "data": encode_words(words)}
                if state.fault == "misroute":
                    response["device"] = 3
                if state.fault == "wrong_command":
                    response["command"] = "wrong"
                try:
                    self.wfile.write(json.dumps(response).encode() + b"\r\n")
                    self.wfile.flush()
                except OSError:
                    return

    class Server(socketserver.ThreadingTCPServer):
        daemon_threads = True
        allow_reuse_address = True

    with Server(("127.0.0.1", 0), Handler) as server:
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        state.config = HandConnection(
            "127.0.0.1",
            2,
            1,
            tuple(state.identity),
            port=server.server_address[1],
            timeout_s=0.12,
            field_verified=True,
            presets_verified=True,
            stop_and_concurrency_verified=True,
            speeds=(1000,) * 6,
            lower=(0,) * 6,
            upper=(30000,) * 6,
        )
        yield state
        server.shutdown()
        thread.join(2)


def test_hand_connect_poll_and_close_never_write(controller):
    with RM65Hand(controller.config) as driver:
        feedback = driver.poll()
        assert feedback.synthetic is False and feedback.current_ma == (40,) * 6
        assert driver.last_blocks["angle_cdeg"]["values"] == [-1] * 6
        assert feedback.received_monotonic_s == min(b["received_monotonic_s"] for b in driver.last_blocks.values())
        with pytest.raises(HandError):
            driver.send_position([100] * 6)
    assert len(controller.requests) == 5
    assert all(r["command"].startswith("read_") for r in controller.requests)


def test_hand_explicit_speed_position_and_feedback(controller):
    with RM65Hand(controller.config, acknowledgement=LIVE_ACK) as driver:
        assert len(controller.requests) == 1  # Identity only at startup, no movement.
        driver.poll()
        ack = driver.send_position([2000] * 6)
        assert ack["synthetic"] is False and len(ack["transactions"]) == 4
        assert driver.poll().position == (2000,) * 6
    writes = [r for r in controller.requests if r["command"] == "write_registers"]
    assert [r["address"] for r in writes] == [1125, 1135]
    assert all(r["port"] == 1 and r["num"] == 6 for r in writes)


@pytest.mark.parametrize("flag", ["field_verified", "presets_verified", "stop_and_concurrency_verified"])
def test_hand_missing_field_gate_rejected_before_connect(controller, flag):
    with pytest.raises(HandError):
        RM65Hand(replace(controller.config, **{flag: False}), acknowledgement=LIVE_ACK)
    assert controller.requests == []


@pytest.mark.parametrize("fault", ["timeout", "malformed", "misroute", "wrong_command", "write_rejected"])
def test_hand_failed_write_poisoned_no_retry_or_target(controller, fault):
    with RM65Hand(controller.config, acknowledgement=LIVE_ACK) as driver:
        driver.poll()
        controller.fault = fault
        with pytest.raises((ValueError, OSError)):
            driver.send_position([2000] * 6)
        count = len(controller.requests)
        with pytest.raises(HandError):
            driver.send_position([2000] * 6)
        assert len(controller.requests) == count
        assert not any(r["address"] == 1135 for r in controller.requests)


def test_hand_mismatched_identity_closes(controller):
    with pytest.raises(HandError):
        with RM65Hand(replace(controller.config, identity=(256, 257, 1, 1, 1, 2))):
            pass
    assert len(controller.requests) == 1


def test_hand_bounds_and_bad_ack_never_send(controller):
    with pytest.raises(HandError):
        RM65Hand(controller.config, acknowledgement="true")
    with RM65Hand(controller.config, acknowledgement=LIVE_ACK) as driver:
        with pytest.raises(HandError):
            driver.send_position([30001] * 6)
    assert len(controller.requests) == 1


def test_field_templates_stay_incomplete():
    profile = yaml.safe_load((CONFIG / "hand_field.template.yaml").read_text())
    with pytest.raises((HandError, TypeError)):
        HandConnection(**profile["connection"])
    with pytest.raises(ValueError):
        RealSenseCamera(CameraConfig())


def test_driver_never_moves_without_post_connect_feedback(controller):
    with RM65Hand(controller.config, acknowledgement=LIVE_ACK) as driver:
        with pytest.raises(HandError, match="stopped feedback"):
            driver.send_position([1000] * 6)
    assert len(controller.requests) == 1


def test_readonly_worker_rejects_commands_and_records_raw_blocks(controller):
    runtime = HandRuntime("readonly", {"connection": controller.config.__dict__})
    runtime.start()
    try:
        wait(lambda: runtime.snapshot()[0].get("stage") == "readonly")
        assert runtime.command({"action": "prepare"}) is False
        assert runtime.snapshot()[0]["hardware_writes"] is False
    finally:
        runtime.close()
    assert not any(r["command"] == "write_registers" for r in controller.requests)


def test_real_worker_task_uses_emulated_feedback_not_synthetic(controller):
    task = yaml.safe_load((CONFIG / "hand_mock.yaml").read_text())
    runtime = HandRuntime("live", {"connection": controller.config.__dict__, "task": task}, acknowledgement=LIVE_ACK)
    runtime.start()
    try:
        wait(lambda: runtime.snapshot()[0].get("feedback_fresh"))
        state = runtime.snapshot()[0]
        runtime.command(
            {
                "session_id": state["session_id"],
                "sequence": 1,
                "expected_revision": state["revision"],
                "action": "prepare",
            }
        )
        wait(lambda: runtime.snapshot()[0].get("stage") == "ready")
        assert runtime.snapshot()[0]["synthetic"] is False
        assert runtime.snapshot()[0]["physical_grasp_verified"] is False
    finally:
        runtime.close()
    assert sum(r["address"] == 1135 for r in controller.requests) == 1


def small_config(**values):
    return CameraConfig(width=32, height=24, fps=30, **values)


def test_capture_roundtrip_and_hash_tamper(tmp_path):
    camera = MockCamera(small_config(align_depth_to_color=True))
    writer = PeripheralRecorder(tmp_path, camera.metadata)
    frame = camera.read()
    assert writer.append(frame)
    writer.close()
    assert verify_capture(writer.path)["frames"] == 1
    with np.load(writer.path / "frames/00000000.npz", allow_pickle=False) as data:
        assert np.array_equal(data["color_rgb8"], frame.color)
        assert np.array_equal(data["depth_z16"], frame.depth)
        assert np.array_equal(data["aligned_depth_z16"], frame.depth)
    with (writer.path / "records.jsonl").open("a") as stream:
        stream.write("{}\n")
    with pytest.raises(ValueError, match="hash"):
        verify_capture(writer.path)


def test_orphan_file_and_incomplete_capture_rejected(tmp_path):
    writer = PeripheralRecorder(tmp_path, {})
    with pytest.raises(ValueError, match="incomplete"):
        verify_capture(writer.path)
    writer.append({"kind": "event", "synthetic": True})
    writer.close()
    (writer.path / "extra.bin").write_bytes(b"orphan")
    with pytest.raises(ValueError, match="file set"):
        verify_capture(writer.path)


def test_writer_disk_failure_is_incomplete(tmp_path, monkeypatch):
    writer = PeripheralRecorder(tmp_path, {})
    monkeypatch.setattr(np, "savez", lambda *a, **k: (_ for _ in ()).throw(OSError("disk full")))
    writer.append(MockCamera(small_config()).read())
    wait(lambda: writer.error is not None)
    writer.close()
    with pytest.raises(ValueError, match="incomplete"):
        verify_capture(writer.path)


def test_late_shutdown_failure_revokes_complete_capture(tmp_path):
    writer = PeripheralRecorder(tmp_path, {})
    writer.append({"kind": "event", "synthetic": True})
    writer.close()
    assert verify_capture(writer.path)["complete"] is True
    writer.fail("shutdown deadline exceeded")
    with pytest.raises(ValueError, match="incomplete"):
        verify_capture(writer.path)


def test_writer_overflow_never_silently_succeeds(tmp_path, monkeypatch):
    blocked = threading.Event()
    entered = threading.Event()
    original = np.savez

    def slow(*args, **kwargs):
        entered.set()
        assert blocked.wait(3)
        return original(*args, **kwargs)

    monkeypatch.setattr(np, "savez", slow)
    writer = PeripheralRecorder(tmp_path, {}, capacity=1)
    try:
        writer.append(MockCamera(small_config()).read())
        assert entered.wait(2)
        assert writer.append({"kind": "event"})
        assert not writer.append({"kind": "overflow"})
    finally:
        blocked.set()
        writer.close()
    assert writer.dropped == 1
    with pytest.raises(ValueError, match="incomplete"):
        verify_capture(writer.path)


@pytest.mark.parametrize("failure", ["disconnect", "replay", "domain", "source"])
def test_camera_failures_latch_and_release_pipeline(tmp_path, failure):
    camera = MockCamera(small_config())
    read = camera.read
    closed = []

    def fail_read():
        frame = read()
        if camera.sequence == 2:
            if failure == "disconnect":
                raise RuntimeError("USB disconnected")
            if failure == "replay":
                frame.metadata["color_frame_number"] = 1
            if failure == "domain":
                frame.metadata["timestamp_domain"] = {"color": "changed", "depth": "synthetic"}
            if failure == "source":
                frame.metadata["synthetic"] = False
        return frame

    camera.read = fail_read
    camera.close = lambda: closed.append(True)
    runtime = CameraRuntime("mock", small_config(), tmp_path, camera=camera)
    runtime.start()
    wait(lambda: runtime.error is not None)
    runtime.close()
    assert len(closed) == 1
    assert runtime.frames == 1
    with pytest.raises(ValueError, match="incomplete"):
        verify_capture(runtime.recorder.path)


def test_mock_capture_cli_and_real_preview_never_import_sdk(tmp_path, capsys):
    assert (
        camera_main(["record", "--profile", str(CONFIG / "camera.yaml"), "--root", str(tmp_path), "--seconds", "0.15"])
        == 0
    )
    result = json.loads(capsys.readouterr().out)
    assert verify_capture(result["path"])["frames"] >= 1
    assert camera_main(["record", "--source", "realsense", "--profile", str(CONFIG / "camera.yaml")]) == 0
    assert json.loads(capsys.readouterr().out)["devices_opened"] is False


@pytest.mark.parametrize("bad", [{"width": True}, {"fps": 0}, {"queue_size": 33}, {"align_depth_to_color": "false"}])
def test_invalid_camera_configuration(bad):
    with pytest.raises(ValueError):
        CameraConfig(**bad)


def test_injected_sdk_native_depth_metadata_and_buffer_ownership():
    """Exercise the real adapter API with an explicitly fake SDK, no pyrealsense2."""
    color = np.full((24, 32, 3), 7, np.uint8)
    depth = np.full((24, 32), 900, np.uint16)

    class Frame:
        def __init__(self, data):
            self.data = data

        def get_data(self):
            return self.data

        def get_timestamp(self):
            return 50.0

        def get_frame_number(self):
            return 3

        def get_frame_timestamp_domain(self):
            return "hardware_clock"

        def supports_frame_metadata(self, key):
            return True

        def get_frame_metadata(self, key):
            return 1200

    frames = NS(get_color_frame=lambda: Frame(color), get_depth_frame=lambda: Frame(depth))
    intrinsic = NS(width=32, height=24, fx=10.0, fy=10.0, ppx=16.0, ppy=12.0, model="brown", coeffs=[0.0] * 5)
    stream = NS(
        get_intrinsics=lambda: intrinsic,
        get_extrinsics_to=lambda s: NS(rotation=np.eye(3).flatten().tolist(), translation=[0.0, 0.0, 0.0]),
    )
    stream.as_video_stream_profile = lambda: stream
    info = {
        "name": "Intel RealSense D435I",
        "serial_number": "TEST",
        "firmware_version": "fixture",
        "usb_type_descriptor": "3.2",
    }
    device = NS(
        get_info=lambda key: info[key],
        supports=lambda key: True,
        first_depth_sensor=lambda: NS(get_depth_scale=lambda: 0.001),
    )
    profile = NS(get_device=lambda: device, get_stream=lambda key: stream)
    stopped, configured = [], []
    pipeline = NS(
        start=lambda config: profile, stop=lambda: stopped.append(True), wait_for_frames=lambda timeout: frames
    )
    config = NS(
        enable_device=lambda serial: configured.append(serial), enable_stream=lambda *args: configured.append(args)
    )
    rs = NS(
        pipeline=lambda: pipeline,
        config=lambda: config,
        stream=NS(color="color", depth="depth"),
        format=NS(rgb8="rgb8", z16="z16"),
        camera_info=NS(**{key: key for key in info}),
        frame_metadata_value=NS(actual_exposure="exposure"),
    )
    camera = RealSenseCamera(small_config(serial="TEST"), rs=rs).start()
    try:
        frame = camera.read()
        color.fill(0)
        depth.fill(0)
        assert np.all(frame.color == 7) and np.all(frame.depth == 900)
        assert frame.metadata["synthetic"] is False
        assert camera.metadata["hand_eye"] is None
        assert camera.metadata["intrinsics"]["depth"]["fx"] == 10
        assert configured[0] == "TEST"
    finally:
        camera.close()
    assert stopped == [True]


@pytest.mark.parametrize(
    "clock,mode", [("different", "hardware"), ("fixture-clock", "sim"), ("fixture-clock", "hardware")]
)
def test_arrival_pairing_clock_gate_actual_packet_dedup_and_unmatched(tmp_path, clock, mode):
    # Explicit offline fixtures. None of these rows are real hardware evidence.
    frame = MockCamera(small_config()).read()
    frame.metadata["synthetic"] = False
    frame.metadata["host_received_monotonic_s"] = 1.0
    writer = PeripheralRecorder(tmp_path, {"camera": {"synthetic": False}})
    writer.metadata["clock_identity"] = "fixture-clock"
    (writer.path / "metadata.json").write_text(json.dumps(writer.metadata), encoding="utf-8")
    writer.append(frame)
    writer.close()
    journal = tmp_path / "arm"
    journal.mkdir()
    (journal / "session.json").write_text(json.dumps({"complete": True, "mode": mode, "clock_identity": clock}))
    packet = {
        "clock_id": "boot",
        "session": "emulated",
        "packet_id": 1,
        "parse_error": 0,
        "position_rad": [0.0] * 6,
        "velocity_rad_s": [0.0] * 6,
    }
    row = {
        "topic": "/rm65/driver/feedback_snapshot",
        "quality": {"valid": True},
        "received_monotonic_s": 1.01,
        "value": {"data": json.dumps(packet)},
    }
    duplicate = {**row, "received_monotonic_s": 1.0}
    (journal / "records.jsonl").write_text(json.dumps(row) + "\n" + json.dumps(duplicate) + "\n")
    output = tmp_path / "pairs.jsonl"
    if clock != "fixture-clock" or mode == "sim":
        with pytest.raises(ValueError):
            pair_capture(writer.path, journal, output)
        assert not output.exists()
    else:
        pair_capture(writer.path, journal, output)
        result = json.loads(output.read_text())
        assert result["matched"] is True and abs(result["arrival_delta_s"] - 0.01) < 1e-10
        assert result["exposure_synchronized"] is False and result["training_ready"] is False
        pair_capture(writer.path, journal, tmp_path / "unmatched.jsonl", max_delta_s=0.005)
        assert json.loads((tmp_path / "unmatched.jsonl").read_text())["feedback"] is None
        with pytest.raises(FileExistsError):
            pair_capture(writer.path, journal, output)
