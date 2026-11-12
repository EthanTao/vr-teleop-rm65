# Third-party notices

## Browser simulation dependencies

- The readonly viewer uses Three.js 0.180.0 (MIT), downloaded from the official npm registry at image build time. Its tarball SHA-256 is checked and `THREE-LICENSE.txt` is kept beside the served assets; JavaScript assets are not vendored into this repository.
- The compute-only RealMan Python API2 files are downloaded from fixed commit `a810b9fdea29e3567c5e9155794e6ef9cc17be62`, with Git blob hashes checked. Vendor binaries remain dependencies of the locally built image, not repository files.
- The isolated Python runtime comes from the official `python:3.11.11-slim-bullseye` image pinned by its multi-architecture digest. MuJoCo Core 4.0.0 and NumPy/PyYAML are installed only in that runtime; their package licenses remain installed.

## HDU-EILab/ur5-teleop

- Repository: https://github.com/HDU-EILab/ur5-teleop
- Reviewed and adapted commit: `8691bfae256d9f48db60eeeca56e243b043d1fea`.
- `ur5_teleop/frames.py` is retained as `rm65_servo_teleop/ur_follow_frames.py` with an attribution header.
- `clip_pose` and `limit_pose_step` from `ur5_teleop/target.py` are retained in `rm65_servo_teleop/follow_targets.py`; imports are adapted to the package.
- The RM65 SDK boundary, follower policy, replay and shadow observation code are local adaptations. UR RTDE, URScript, automatic Home and gripper code are not copied.
- Copyright (c) 2026 HDU-EILab. The full upstream MIT license is retained in [ur5-teleop-MIT.txt](ros2_ws/src/rm65_servo_teleop/licenses/ur5-teleop-MIT.txt) and installed with the ROS package. The upstream license's RTDE third-party note is retained verbatim; RTDE itself is not included here.

## CustomTkinter

- Repository: https://github.com/TomSchimansky/CustomTkinter
- Runtime package: customtkinter==6.0.0 (installed as a dependency, not vendored).
- Desktop layout adapts the upstream sidebar/dashboard example: `examples/complex_example.py`.
- Reviewed example blob SHA: `3ac9cb8c9e2339232f22861186154042d1aad4d1`.
- Adaptation: RM65 pages, Chinese labels, existing command adapters and session management.
- Upstream license follows verbatim.

MIT License

Copyright (c) 2023 Tom Schimansky

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
