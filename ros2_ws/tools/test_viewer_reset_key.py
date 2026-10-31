#!/usr/bin/env python3
"""Compile the actual patched viewer key adapter against an inert GLFW base.

No window, ROS graph, SDK or robot is started. Pass the patched, pinned
mujoco_simulation.cpp; this extracts its class rather than copying the logic.
"""

import argparse
from pathlib import Path
import subprocess
import tempfile


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path)
    args = parser.parse_args()
    source = args.source.read_text(encoding="utf-8")
    start = source.index("class ROS2ControlGlfwAdapter :")
    end = source.index("\n};", start) + 3
    adapter = source[start:end]
    program = r"""
#include <atomic>
#include <cassert>
#include <functional>
#include <utility>
constexpr int GLFW_KEY_R=82, GLFW_KEY_RIGHT=262;
constexpr int GLFW_PRESS=1, GLFW_REPEAT=2, GLFW_RELEASE=0;
namespace mj {
class GlfwAdapter {
public:
  virtual ~GlfwAdapter() = default;
  int forwarded=0;
protected:
  virtual void OnKey(int, int, int) { ++forwarded; }
};
}
ADAPTER
struct Adapter : ROS2ControlGlfwAdapter {
  using ROS2ControlGlfwAdapter::ROS2ControlGlfwAdapter;
  using ROS2ControlGlfwAdapter::OnKey;
};
int main() {
  std::atomic<bool> step{false};
  Adapter unowned(step);
  unowned.OnKey(GLFW_KEY_R,0,GLFW_PRESS);
  assert(unowned.forwarded==1);  // No session callback: retain vendor behavior.
  int resets=0;
  Adapter owned(step,[&resets]() { ++resets; });
  owned.OnKey(GLFW_KEY_R,0,GLFW_RELEASE);
  owned.OnKey(GLFW_KEY_R,0,GLFW_REPEAT);
  assert(resets==0);
  owned.OnKey(GLFW_KEY_R,0,GLFW_PRESS);
  owned.OnKey(GLFW_KEY_R,0,GLFW_REPEAT);
  owned.OnKey(GLFW_KEY_R,0,GLFW_RELEASE);
  owned.OnKey(GLFW_KEY_R,0,GLFW_PRESS);
  assert(resets==1 && owned.forwarded==0 && !step);
  owned.OnKey(65,0,GLFW_PRESS);
  assert(owned.forwarded==1);
  owned.OnKey(GLFW_KEY_RIGHT,0,GLFW_PRESS);
  assert(step && resets==1);
}
""".replace("ADAPTER", adapter)
    with tempfile.TemporaryDirectory(prefix="rm65-viewer-key-") as folder:
        path = Path(folder) / "key.cpp"
        path.write_text(program, encoding="utf-8")
        executable = Path(folder) / "key-test"
        subprocess.run(["c++", "-std=c++17", "-Wall", "-Wextra", str(path), "-o", str(executable)], check=True)
        subprocess.run([str(executable)], check=True)
    print("Patched viewer adapter: R press/repeat/release, no-session forwarding and existing step behavior PASS")


if __name__ == "__main__":
    main()
