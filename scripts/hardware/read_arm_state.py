import socket
import sys
import time

import numpy as np

sys.path.insert(0, "/workspace")
from xrobotoolkit_teleop.hardware.realman_udp_feedback import parse_realtime_arm_state


def main():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(("0.0.0.0", 8089))
    s.settimeout(2)
    print("实时读取 RM65 状态 (Ctrl+C 退出)。用示教器摆位，观察 J3/J5 余量。", flush=True)
    while True:
        try:
            d, a = s.recvfrom(65535)
        except socket.timeout:
            print("   [超时] 未收到 UDP 反馈", flush=True)
            continue
        try:
            fb = parse_realtime_arm_state(d, time.monotonic())
        except ValueError as e:
            print("   [解析失败] %s" % e, flush=True)
            continue
        j = np.rad2deg(fb.joint_rad)
        j3_margin = abs(j[2])
        j5_margin = abs(j[4])
        safe = j3_margin >= 30.0 and j5_margin >= 30.0
        print(
            "关节角(deg): J1=%7.2f J2=%7.2f J3=%7.2f J4=%7.2f J5=%7.2f J6=%7.2f"
            % tuple(j),
            flush=True,
        )
        print(
            "  奇异点余量: J3距0°=%5.1f°  J5距0°=%5.1f°  ->  %s"
            % (j3_margin, j5_margin, "安全 ✓" if safe else "⚠️ 太近(需≥30°)"),
            flush=True,
        )
        print("  TCP xyz(m):  [%.4f, %.4f, %.4f]" % tuple(fb.tcp_xyz_m), flush=True)
        print(
            "  TCP quat(wxyz): [%.6f, %.6f, %.6f, %.6f]" % tuple(fb.tcp_quat_wxyz),
            flush=True,
        )
        print("  err: %s" % (fb.arm_error_codes,), flush=True)
        print("-" * 60, flush=True)
        time.sleep(0.4)


if __name__ == "__main__":
    main()
