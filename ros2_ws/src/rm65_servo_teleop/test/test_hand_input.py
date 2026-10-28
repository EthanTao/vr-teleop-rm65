"""Preset button behavior using the official XR JSON shape; no devices or ROS."""

from dataclasses import replace
import json

import pytest

from rm65_servo_teleop.hand_input import HandButtonAdapter, HandButtons, decode_hand_buttons
from rm65_servo_teleop.hand_session import HandProfile, HandSession, MockHand
from rm65_servo_teleop.input_adapter import InputRejected


def packet(stamp=1, *, grip=0.0, trigger=0.0, a=False, b=False):
    return {
        "_rm65_mock": True,
        "Input": 1,
        "timeStampNs": stamp,
        "Controller": {
            "right": {
                "pose": [0, 0, 0, 0, 0, 0, 1],
                "grip": grip,
                "trigger": trigger,
                "primaryButton": a,
                "secondaryButton": b,
            }
        },
    }


class Stream:
    def __init__(self):
        self.adapter = HandButtonAdapter()
        self.now = 0.0
        self.stamp = 1_000_000_000

    def frame(self, stage="ready", *, dt=0.05, device_dt=None, **buttons):
        self.now = round(self.now + dt, 6)
        self.stamp += round((dt if device_dt is None else device_dt) * 1e9)
        return self.adapter.receive_raw(json.dumps(packet(self.stamp, **buttons)), self.now, stage)

    def hold_b(self, stage="ready", **buttons):
        actions = []
        for _ in range(26):
            actions.extend(self.frame(stage, b=True, **buttons))
        return tuple(actions)


def test_official_nested_schema_and_right_controller_buttons():
    raw = {"functionName": "Tracking", "value": json.dumps(packet(10, grip=0.4, trigger=0.8, a=True))}
    assert decode_hand_buttons(json.dumps(raw)) == HandButtons(10, 0.4, 0.8, True, False, True)


@pytest.mark.parametrize("trigger", [None, True, "1", -0.1, 1.1, 10**400, float("nan"), float("inf")])
def test_invalid_trigger_is_not_an_action(trigger):
    with pytest.raises(InputRejected):
        decode_hand_buttons(json.dumps(packet(trigger=trigger)))


@pytest.mark.parametrize("key,value", [("primaryButton", None), ("primaryButton", 1), ("secondaryButton", "false")])
def test_buttons_are_explicit_booleans(key, value):
    raw = packet()
    raw["Controller"]["right"][key] = value
    with pytest.raises(InputRejected):
        decode_hand_buttons(json.dumps(raw))


def test_missing_or_invalid_tracking_disarms_without_action():
    stream = Stream()
    stream.frame()
    raw = packet(2_000_000_000)
    del raw["Controller"]["right"]["trigger"]
    assert stream.adapter.receive_raw(json.dumps(raw), stream.now, "holding") == ()
    assert not stream.adapter.armed
    raw["Controller"]["right"]["trigger"] = 1
    raw["Controller"]["right"]["status"] = 0
    assert stream.adapter.receive_raw(json.dumps(raw), stream.now, "holding") == ()


@pytest.mark.parametrize("held", [{"trigger": 1}, {"grip": 1}, {"a": True}, {"b": True}])
def test_startup_held_controls_do_nothing_until_neutral(held):
    stream = Stream()
    for _ in range(25):
        assert stream.frame(**held) == ()
    assert not stream.adapter.armed
    assert stream.frame() == ()
    assert stream.adapter.armed
    assert stream.frame(trigger=1) == ("grasp",)


def test_trigger_is_one_shot_with_hysteresis_and_release_keeps_hold():
    stream = Stream()
    stream.frame()
    assert stream.frame(trigger=0.74) == ()
    assert stream.frame(trigger=0.75) == ("grasp",)
    for value in [1, 1, 0.7, 0.26, 0.8]:
        assert stream.frame(trigger=value) == ()
    assert stream.frame("holding", trigger=0.25) == ()
    assert stream.frame("released", trigger=0.8) == ("grasp",)


def test_a_confirms_once_and_grip_marks_insertion_only_from_holding():
    stream = Stream()
    stream.frame()
    assert stream.frame("awaiting_grasp_confirmation", a=True) == ("confirm_grasp",)
    assert stream.frame("holding", a=True) == ()
    assert stream.frame("holding") == ()
    assert stream.frame("holding", grip=1) == ("begin_insert",)
    assert stream.frame("inserting", grip=1) == ()
    assert stream.frame("inserting", grip=0) == ()
    assert stream.frame("ready", grip=1) == ()


@pytest.mark.parametrize(
    "stage,actions",
    [
        ("unknown", ("prepare",)),
        ("ready", ("release",)),
        ("holding", ("release",)),
        ("inserting", ("finish_insert", "release")),
    ],
)
def test_long_b_explicit_action_fires_once(stage, actions):
    stream = Stream()
    stream.frame(stage)
    assert stream.hold_b(stage) == actions
    assert stream.hold_b(stage) == ()


def test_short_b_or_device_clock_too_slow_cannot_release():
    stream = Stream()
    stream.frame("holding")
    for _ in range(15):
        assert stream.frame("holding", b=True) == ()
    assert stream.frame("holding") == ()
    for _ in range(25):
        assert stream.frame("holding", b=True, device_dt=0.001) == ()


def test_grip_must_be_released_before_new_b_press():
    stream = Stream()
    stream.frame("inserting")
    assert stream.hold_b("inserting", grip=1) == ()
    assert stream.hold_b("inserting", grip=0) == ()
    stream.frame("inserting")
    assert stream.hold_b("inserting") == ("finish_insert", "release")


def test_grip_during_b_hold_cancels_and_requires_new_press():
    stream = Stream()
    stream.frame("holding")
    stream.frame("holding", b=True)
    stream.frame("holding", b=True, grip=1)
    assert stream.hold_b("holding") == ()
    stream.frame("holding")
    assert stream.hold_b("holding") == ("release",)


def test_stage_change_during_hold_requires_new_b_press():
    stream = Stream()
    stream.frame("grasping")
    stream.frame("grasping", b=True)
    assert stream.hold_b("awaiting_grasp_confirmation") == ()
    assert stream.adapter.reason == "stage_changed_press_b_again"


def test_gap_and_reconnection_cannot_finish_previous_hold():
    stream = Stream()
    stream.frame("holding")
    stream.frame("holding", b=True)
    assert stream.frame("holding", dt=0.3, b=True) == ()
    assert stream.hold_b("holding") == ()
    assert not stream.adapter.armed
    stream.frame("holding")
    assert stream.hold_b("holding") == ("release",)


def test_duplicate_and_reused_timestamp_do_not_complete_hold():
    stream = Stream()
    stream.frame("holding")
    stream.frame("holding", b=True)
    sample = stream.adapter.sample
    assert stream.adapter.receive(sample, stream.now + 1.1, "holding") == ()
    assert not stream.adapter.armed
    altered = replace(sample, b=False)
    assert stream.adapter.receive(altered, stream.now + 1.11, "holding") == ()
    assert stream.adapter.reason == "hand_controller_timestamp_reused"


def test_clock_regression_requires_neutral_before_new_action():
    stream = Stream()
    stream.frame()
    stream.stamp = 1
    assert stream.frame(trigger=1) == ()
    assert not stream.adapter.armed
    assert stream.frame(trigger=1) == ()
    stream.frame()
    assert stream.frame(trigger=1) == ("grasp",)


@pytest.mark.parametrize("buttons", [{"a": True, "trigger": 1}, {"a": True, "b": True}, {"b": True, "trigger": 1}])
def test_conflicting_buttons_require_neutral(buttons):
    stream = Stream()
    stream.frame()
    assert stream.frame(**buttons) == ()
    assert not stream.adapter.armed
    assert stream.frame(trigger=1) == ()
    stream.frame()
    assert stream.frame(trigger=1) == ("grasp",)


def test_complete_preset_cycle_does_not_need_individual_finger_input():
    profile = HandProfile((0,) * 6, (12000,) * 6)
    session, hand, stream = HandSession(profile), MockHand(profile), Stream()
    rows = []

    def step(**buttons):
        session.observe(hand.poll(stream.now), stream.now)
        for action in stream.frame(session.stage, **buttons):
            assert session.command(
                {
                    "action": action,
                    "session_id": session.session_id,
                    "sequence": session.last_command_sequence + 1,
                    "expected_revision": session.revision,
                    "source": {"kind": "xr_buttons"},
                },
                stream.now,
            )
        session.flush_mock(hand, stream.now)
        rows.extend(session.events)
        session.events.clear()

    step()
    for _ in range(26):
        step(b=True)
    for _ in range(20):
        step()
    assert session.stage == "ready" and hand.write_count == 1
    step(trigger=1)
    for _ in range(20):
        step()
    assert session.stage == "awaiting_grasp_confirmation"
    step(a=True)
    step()
    step(grip=1)
    assert session.stage == "inserting"
    for _ in range(30):
        step()  # Releasing Grip never opens the hand.
    assert hand.write_count == 2
    for _ in range(26):
        step(b=True)
    for _ in range(20):
        step()
    assert session.stage == "released" and hand.write_count == 3
    assert not session.status(stream.now)["physical_grasp_verified"]
    assert all(row["source"]["kind"] == "xr_buttons" for row in rows if row["kind"] == "candidate")
    step(trigger=1)  # A second grasp cycle is admitted without restarting.
    assert session.stage == "grasping" and hand.write_count == 4
