#!/usr/bin/env python3
"""CLI 包装：分析 `--motion-log-path` 的 JSONL，给出「手动了多少 / 臂实际动了多少」。

逻辑全部在 ``xrobotoolkit_teleop/utils/motion_log_analysis.py``（可被单元测试直接覆盖），
本文件只做入口，避免把解析与统计写进脚本里而无法回归。

跑法（仓库根目录）：

    python scripts/debug/analyze_motion_log.py build/motion-2026-09-29.jsonl
    python scripts/debug/analyze_motion_log.py build/motion-2026-09-29.jsonl --top 10 --json

只读一个文件：不连真机、不读串口、不改任何状态。
"""

from __future__ import annotations

import sys
from pathlib import Path

# 允许直接从仓库根运行，而不要求先 `pip install -e .`。
_REPO_ROOT = Path(__file__).resolve().parents[2]
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))

from xrobotoolkit_teleop.utils.motion_log_analysis import main  # noqa: E402

if __name__ == "__main__":
    raise SystemExit(main())
