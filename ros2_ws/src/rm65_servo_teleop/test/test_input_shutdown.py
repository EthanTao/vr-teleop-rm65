"""Run actual Python entrypoints against a fake ROS context, without ROS/SDK."""

import importlib.util
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace

import pytest


@pytest.mark.parametrize("module_name", ["xr_node", "mock_xr", "journal_node"])
@pytest.mark.parametrize(
    "interruption,context_alive",
    [("keyboard", True), ("external", False), ("external", True), ("error", True), ("rcl", False), ("rcl", True)],
)
def test_input_entrypoint_context_shutdown_is_normal_and_cleanup_always_runs(
    monkeypatch, module_name, interruption, context_alive
):
    events = []
    nodes = []
    context = {"ok": context_alive}

    class ExternalShutdownException(Exception):
        pass

    class RCLError(Exception):
        pass

    class Message:
        def __init__(self, **kwargs):
            self.__dict__.update(kwargs)

    class Node:
        def __init__(self, *args, **kwargs):
            self.parameters = {}
            nodes.append(self)

        def declare_parameter(self, name, default):
            self.parameters[name] = True if name == "simulation_only" else default
            return self.get_parameter(name)

        def get_parameter(self, name):
            return SimpleNamespace(value=self.parameters[name])

        def create_publisher(self, *args, **kwargs):
            return SimpleNamespace(publish=lambda message: events.append(("publish", message.data)))

        def create_subscription(self, *args, **kwargs):
            return object()

        def create_timer(self, *args, **kwargs):
            return object()

        def get_logger(self):
            return SimpleNamespace(info=lambda message: None)

        def destroy_node(self):
            events.append("destroy")

    def spin(node):
        exception = {
            "keyboard": KeyboardInterrupt,
            "external": ExternalShutdownException,
            "error": RuntimeError,
            "rcl": RCLError,
        }[interruption]
        raise exception("test context interruption")

    def shutdown():
        assert context["ok"], "a previously shut down ROS context must not be shut down twice"
        events.append("shutdown")
        context["ok"] = False

    def install_module(name, **attributes):
        module = ModuleType(name)
        module.__dict__.update(attributes)
        monkeypatch.setitem(sys.modules, name, module)
        return module

    install_module("rclpy", init=lambda **kwargs: None, spin=spin, ok=lambda: context["ok"], shutdown=shutdown)
    install_module("rclpy.node", Node=Node)
    install_module("rclpy.duration", Duration=Message)
    install_module("rclpy.time", Time=Message)
    install_module("rclpy.executors", ExternalShutdownException=ExternalShutdownException)
    install_module("rclpy.impl.implementation_singleton", rclpy_implementation=SimpleNamespace(RCLError=RCLError))
    install_module(
        "rclpy.qos",
        QoSProfile=Message,
        ReliabilityPolicy=SimpleNamespace(RELIABLE=1, BEST_EFFORT=2),
        HistoryPolicy=SimpleNamespace(KEEP_LAST=1),
        DurabilityPolicy=SimpleNamespace(VOLATILE=1),
    )
    for package, message_types in {
        "geometry_msgs": {"PoseStamped": Message},
        "sensor_msgs": {"JointState": Message},
        "std_msgs": {"Bool": Message, "String": Message, "Int8": Message},
        "control_msgs": {"JointTrajectoryControllerState": Message},
        "trajectory_msgs": {"JointTrajectory": Message},
        "rosgraph_msgs": {"Clock": Message},
    }.items():
        install_module(package)
        install_module(package + ".msg", **message_types)
    install_module("tf2_ros", Buffer=Message, TransformListener=Message, TransformException=Exception)
    install_module("rosidl_runtime_py.convert", message_to_ordereddict=lambda message: message.__dict__)

    path = Path(__file__).resolve().parents[1] / "rm65_servo_teleop" / f"{module_name}.py"
    name = f"rm65_servo_teleop._shutdown_fixture_{module_name}"
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    monkeypatch.setitem(sys.modules, name, module)
    spec.loader.exec_module(module)
    if module_name == "xr_node":
        # Exercise the real main() and disable(), with ROS construction omitted.
        node = object.__new__(module.XrInputNode)
        node.adapter = SimpleNamespace(invalidate=lambda reason: events.append(("invalidate", reason)))
        node._enable_publisher = SimpleNamespace(publish=lambda message: events.append(("publish", message.data)))
        monkeypatch.setattr(module, "XrInputNode", lambda: node)
    if module_name == "journal_node":
        writer = SimpleNamespace(path="fake-journal", close=lambda: events.append("journal_close"))
        monkeypatch.setattr(module, "JournalWriter", lambda *args, **kwargs: writer)
    if interruption == "error" or (interruption == "rcl" and context_alive):
        with pytest.raises((RuntimeError, RCLError), match="test context interruption"):
            module.main()
    else:
        module.main()  # KeyboardInterrupt/ExternalShutdownException must not become exit code 1.
    assert events.count("destroy") == 1
    assert events.count("shutdown") == int(context_alive)
    if module_name == "xr_node":
        assert (("publish", False) in events) is context_alive
        assert ("invalidate", "input_node_stopping") in events
    if module_name == "journal_node":
        assert events.count("journal_close") == 1
