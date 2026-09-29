#!/usr/bin/env python3
"""只读探针：把 RM65 实时 UDP 主动上报的**原始数据报**完整摊开。

为什么需要它
------------
真机链路目前只解析 6 个通道 —— 关节角、关节速度、TCP 位置、TCP 姿态、错误码
（见 `xrobotoolkit_teleop/hardware/realman_udp_feedback.py` 的 `ArmFeedback`），
**从来没有检查过控制器实际推了哪些字段**。

这件事与「充电枪插拔」直接相关：在不选配六维力传感器（F/T）的前提下，唯一的力替代
通道是**关节电流** —— 用电流做接触检测与堵转判定，必要时再由雅可比粗估末端力。
如果控制器本来就在推电流，这条通道是免费的；如果不推，就必须另想办法。

本脚本只做三件事：收帧、原样摊开、指出哪些字段可能是力/电流类通道，然后退出。
**不连接机械臂、不下发任何指令、不改任何状态。**

⚠️ 它与真机入口抢同一个 UDP 端口（默认 `8089`）：**不要同时运行**。

用法（在能收到 UDP 的机器 / 容器内）
------------------------------------
    python scripts/debug/dump_udp_frame.py                  # 收 1 帧并打印
    python scripts/debug/dump_udp_frame.py --frames 3       # 收 3 帧，标出哪些字段在变
    python scripts/debug/dump_udp_frame.py --out frame.json # 顺便存原始 JSON
"""

from __future__ import annotations

import argparse
import json
import socket
import sys
import time

# 叶子键名里出现这些词，就可能是力/电流类通道（大小写不敏感）。
CANDIDATE_HINTS = ("current", "force", "torque", "wrench", "volt", "temp", "load")

# 这些是链路已经在用的通道，单独说明，避免和"新发现"混在一起。
KNOWN_CHANNELS = {
    "joint_status.joint_position": "关节角（0.001°，已在用）",
    "joint_status.joint_speed": "关节速度（0.01°/s，已在用）",
    "waypoint.position": "TCP 位置（1e-6 m，已在用）",
    "waypoint.quat": "TCP 姿态（1e-6，已在用）",
}


def flatten(node, prefix: str = "") -> dict[str, object]:
    """把嵌套 JSON 摊成 `{路径: 值}`；数组整条当一个叶子（打印长度与前几个元素）。"""
    out: dict[str, object] = {}
    if isinstance(node, dict):
        for key, value in node.items():
            path = f"{prefix}.{key}" if prefix else str(key)
            out.update(flatten(value, path))
    else:
        out[prefix] = node
    return out


def summarize(value: object) -> str:
    if isinstance(value, list):
        head = ", ".join(repr(item) for item in value[:4])
        tail = ", …" if len(value) > 4 else ""
        return f"list[{len(value)}] [{head}{tail}]"
    return f"{type(value).__name__} {value!r}"


def _short(value: object, limit: int = 42) -> str:
    text = summarize(value)
    return text if len(text) <= limit else text[: limit - 1] + "…"


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="只读：把 RM65 实时 UDP 主动上报的原始数据报摊开，检查有没有力/电流类通道。"
    )
    parser.add_argument("--bind-host", default="0.0.0.0", help="本地绑定地址（默认 0.0.0.0）")
    parser.add_argument("--port", type=int, default=8089, help="UDP 端口（默认 8089，与真机入口相同）")
    parser.add_argument("--frames", type=int, default=1, help="收几帧（>1 时会标出哪些字段在变）")
    parser.add_argument("--timeout-s", type=float, default=5.0, help="每帧等待上限（默认 5 s）")
    parser.add_argument("--out", default="", help="把最后一帧的原始 JSON 写到该路径（可选）")
    args = parser.parse_args(argv)

    try:
        sys.stdout.reconfigure(errors="replace")
    except (AttributeError, OSError):
        pass

    frames: list[dict[str, object]] = []
    raw_frames: list[bytes] = []

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((args.bind_host, args.port))
    sock.settimeout(max(0.1, args.timeout_s))
    print(f"监听 UDP {args.bind_host}:{args.port}，等 {args.frames} 帧……", flush=True)

    try:
        while len(frames) < max(1, args.frames):
            try:
                payload, address = sock.recvfrom(65535)
            except socket.timeout:
                print(f"  [超时] {args.timeout_s:g}s 内没收到数据报（端口被占用？或未开启主动上报？）")
                break
            raw_frames.append(payload)
            try:
                parsed = json.loads(payload.decode("utf-8"))
            except (UnicodeDecodeError, json.JSONDecodeError) as exc:
                print(f"  [解析失败] 来自 {address[0]}：{exc}")
                print(f"  原始前 200 字节：{payload[:200]!r}")
                break
            if not isinstance(parsed, dict):
                print(f"  [格式异常] 顶层不是对象，而是 {type(parsed).__name__}")
                break
            frames.append(parsed)
            print(f"  第 {len(frames)} 帧：{len(payload)} 字节，来自 {address[0]}:{address[1]}")
    finally:
        sock.close()
        print()

    if not frames:
        print("没有可分析的数据报。请确认：① 主动上报已开启；② 没有别的程序占用该端口。")
        return 1

    flat_last = flatten(frames[-1])
    print("=== 这一帧的全部字段 ===")
    for path, value in flat_last.items():
        note = KNOWN_CHANNELS.get(path)
        suffix = f"   ← {note}" if note else ""
        print(f"  {path:<44} {_short(value)}{suffix}")

    if len(frames) > 1:
        print()
        print("=== 跨帧变化（只有会变的字段才是活通道）===")
        for path in flat_last:
            values = [flatten(frame).get(path) for frame in frames]
            distinct = {json.dumps(value, sort_keys=True, default=str) for value in values}
            mark = "变化" if len(distinct) > 1 else "不变"
            samples = " | ".join(_short(value, 28) for value in values[:3])
            print(f"  [{mark}] {path:<44} {samples}")

    print()
    print("=== 力 / 电流类候选通道 ===")
    hits = [(path, value) for path, value in flat_last.items() if any(hint in path.lower() for hint in CANDIDATE_HINTS)]
    if hits:
        for path, value in hits:
            print(f"  ✅ {path} = {_short(value, 60)}")
        print()
        print("  → 控制器确实在推力/电流类通道。无 F/T 时，这就是接触检测的免费通道：")
        print("     把它解析进 ArmFeedback、随运动日志落盘，即可做「电流上升 ⇒ 顶上了」的判定。")
    else:
        print("  未找到 current / force / torque / voltage / temperature 类字段。")
        print()
        print("  → 这一帧只推位置类通道。在『不加 F/T』的前提下，接触检测只能靠：")
        print("     · API2 的关节状态查询（rm_get_joint_status 可读电流，但不是 UDP 主动上报）")
        print("     · 控制器自带的碰撞/堵转检测（电流阈值）")
        print("     · 或者纯位置策略（螺旋搜索 + 位置误差上升判定）")

    if args.out:
        text = json.dumps(frames[-1], ensure_ascii=False, indent=2)
        with open(args.out, "w", encoding="utf-8") as handle:
            handle.write(text + "\n")
        print()
        print(f"原始 JSON 已写入 {args.out}")

    print()
    print("=== 请把上面「这一帧的全部字段」整段贴回来 ===")
    print("（它决定无 F/T 条件下还有没有力替代通道；不含敏感信息。）")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
