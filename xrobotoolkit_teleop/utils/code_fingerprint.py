"""控制链源码指纹：回答「机器人上跑的到底是哪一版」。

为什么需要它
------------
2026-09-29 的诊断发现：DLS 奇异保护 v2.2（见 `docs/2026.9.26.md`）在本地工作树里改完了，
但既没有提交，也没有任何机制能确认远端容器里是不是同一份代码。于是真机验收一旦
「不完美」，就无法区分下面两种完全不同的情况：

- **改动没生效**（容器里还是旧版）→ 下一步是修同步，不是改算法；
- **改动生效了但问题还在** → 下一步才是算法或接口取舍。

这两条路的代价差很远，而当时手上没有任何证据能区分它们。本模块就是那个缺失的证据：
对控制链上的关键源文件逐个算 SHA-256，拼成清单再算一次总哈希，取前 16 位。

用法
----
本机（`teleop.py` 同步成功后）与容器内（真机入口启动时）打印**同一个标签、同一个值**：

    [FINGERPRINT] 控制链源码 7 文件 sha256=1a2b3c4d5e6f7788

两行不一致，就说明容器里跑的不是本机这份代码 —— 无论同步脚本打印了多少个 `[OK]`。
单个文件对不上时，用 `python -m xrobotoolkit_teleop.utils.code_fingerprint` 打印逐文件哈希表。

边界
----
- 指纹只证明「源码字节一致」，不证明「行为一致」：它不覆盖 `config/` 快照、不覆盖
  远端容器里的 Python 版本或已装依赖，也不覆盖运行时 CLI 参数。
- 指纹**不进控制链**：不参与任何安全判据、阈值、锁存或发送流程。
"""

from __future__ import annotations

import hashlib
from pathlib import Path

# 决定真机运动行为的源文件（仓库相对路径）。增删这个列表会让指纹发生变化，这是有意的：
# 只要其中任何一个不再匹配，就不该假装「跑的是同一版」。
CRITICAL_SOURCES: tuple[str, ...] = (
    "xrobotoolkit_teleop/hardware/damped_ik.py",
    "xrobotoolkit_teleop/hardware/realman_rm65_safe_teleop_controller.py",
    "xrobotoolkit_teleop/hardware/realman_official_ik.py",
    "xrobotoolkit_teleop/hardware/cartesian_command_limiter.py",
    "xrobotoolkit_teleop/hardware/realman_udp_feedback.py",
    "xrobotoolkit_teleop/hardware/interface/realman_rm65.py",
    "scripts/hardware/teleop_realman_rm65_safe_hardware.py",
)

# 本机与容器两端共用这一个前缀，便于肉眼对照与 grep（改它会同时改两端，不影响判定）。
FINGERPRINT_PREFIX = "[FINGERPRINT]"

# 文件读不到时占位用；它会让指纹与「文件存在」的版本必然不同。
MISSING = "missing"


def repository_root() -> Path:
    """仓库根：本文件位于 `<root>/xrobotoolkit_teleop/utils/`。"""
    return Path(__file__).resolve().parents[2]


def _resolve_base(root: str | Path | None) -> Path:
    return Path(root).resolve() if root is not None else repository_root()


def _file_hash(path: Path) -> str:
    """单个文件的 SHA-256；读不到就返回占位符，不抛异常（诊断功能不该打断启动）。"""
    try:
        return hashlib.sha256(path.read_bytes()).hexdigest()
    except OSError:
        return MISSING


def source_hashes(root: str | Path | None = None) -> dict[str, str]:
    """逐文件哈希，键是仓库相对路径，顺序与 `CRITICAL_SOURCES` 一致。"""
    base = _resolve_base(root)
    return {relative: _file_hash(base / relative) for relative in CRITICAL_SOURCES}


def source_fingerprint(root: str | Path | None = None) -> str:
    """16 位十六进制总指纹；任一关键文件缺失都会让它与完整版本不同。"""
    digest = hashlib.sha256()
    for relative, file_hash in source_hashes(root).items():
        digest.update(relative.encode("utf-8"))
        digest.update(b"\0")
        digest.update(file_hash.encode("ascii"))
    return digest.hexdigest()[:16]


def fingerprint_line(root: str | Path | None = None) -> str:
    """两端打印的同一行；本机与容器应输出完全相同的文本。"""
    return f"{FINGERPRINT_PREFIX} 控制链源码 {len(CRITICAL_SOURCES)} 文件 " f"sha256={source_fingerprint(root)}"


def main() -> None:
    """打印总指纹与逐文件哈希，供指纹不一致时定位是哪个文件。"""
    base = repository_root()
    print(f"仓库根: {base}")
    print(fingerprint_line(base))
    for relative, file_hash in source_hashes(base).items():
        print(f"  {file_hash[:16]}  {relative}")


if __name__ == "__main__":
    main()
