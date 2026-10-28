"""Physics progress must not be replaced by freshly stamped cached joint data."""

import time

import pytest

from rm65_servo_teleop.simulation_guard import PhysicsProgress, configure_guard


def test_repeated_clock_packets_do_not_refresh_physics_watchdog():
    guard = PhysicsProgress(now=0)
    guard.receive(1_000_000, 0.01)
    guard.receive(1_000_000, 0.19)
    with pytest.raises(RuntimeError, match="paused/stalled"):
        guard.check(0.22)


def test_clock_regression_requires_session_restart():
    guard = PhysicsProgress(now=0)
    guard.receive(1_000_000, 0.01)
    with pytest.raises(RuntimeError, match="reset/regressed"):
        guard.receive(0, 0.02)


def test_no_physics_clock_cannot_pass_startup():
    guard = PhysicsProgress(startup_timeout_s=2.0, now=0)
    with pytest.raises(RuntimeError, match="did not start"):
        guard.check(2.01)


def test_advancing_physics_keeps_watchdog_healthy():
    guard = PhysicsProgress(now=0)
    for index in range(1, 100):
        guard.receive(index * 1_000_000, index * 0.02)
        guard.check(index * 0.02 + 0.01)


@pytest.fixture
def ros_guard_node():
    rclpy = pytest.importorskip("rclpy", reason="watchdog clock acceptance requires Linux ROS")
    from rclpy.context import Context
    from rclpy.node import Node
    from rclpy.parameter import Parameter

    context = Context()
    rclpy.init(context=context)
    node = Node(
        "simulation_guard_clock_acceptance",
        context=context,
        use_global_arguments=False,
        parameter_overrides=[Parameter("startup_timeout_s", value=0.06)],
    )
    try:
        configure_guard(node)
        yield node
    finally:
        node.destroy_node()
        context.shutdown()


def test_running_watchdog_rejects_switch_to_simulation_time(ros_guard_node):
    from rclpy.parameter import Parameter

    node = ros_guard_node
    result = node.set_parameters([Parameter("use_sim_time", value=True)])[0]
    assert not result.successful
    assert "use_sim_time=false" in result.reason
    assert node.get_parameter("use_sim_time").value is False
    assert node.set_parameters([Parameter("use_sim_time", value=False)])[0].successful


def test_watchdog_timer_expires_even_if_node_ros_clock_stops(ros_guard_node):
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.time_source import TimeSource

    node = ros_guard_node
    # Freeze the actual ROS clock through a second time source. This bypasses
    # parameter rejection to independently verify the steady timer's behavior.
    frozen = TimeSource()
    frozen.attach_clock(node.get_clock())
    frozen.ros_time_is_active = True
    assert node.get_clock().now().nanoseconds == 0
    executor = SingleThreadedExecutor(context=node.context)
    executor.add_node(node)
    try:
        deadline = time.monotonic() + 2.0
        with pytest.raises(RuntimeError, match="physics did not start"):
            while time.monotonic() < deadline:
                executor.spin_once(timeout_sec=0.03)
        assert node.get_clock().now().nanoseconds == 0
    finally:
        executor.remove_node(node)
        executor.shutdown()
        frozen.ros_time_is_active = False
