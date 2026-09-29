#!/usr/bin/env python3
"""CLI：从运动日志估计「手柄 → 末端」映射矩阵，用于逐轴方向定案。

逻辑全部在 `xrobotoolkit_teleop/utils/mapping_estimation.py`（可被单元测试直接覆盖），
本文件只做入口。

跑法（仓库根目录）：

    python scripts/debug/estimate_teleop_mapping.py build/motion-2026-09-30.jsonl
    python scripts/debug/estimate_teleop_mapping.py build/motion-2026-09-30.jsonl --json

只读一个日志文件：不连真机、不读串口、不改任何状态。
"""

from __future__ import annotations

import sys
from pathlib import Path

# 允许直接从仓库根运行，而不要求先 `pip install -e .`。
_REPO_ROOT = Path(__file__).resolve().parents[2]
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))

from xrobotoolkit_teleop.utils.mapping_estimation import main  # noqa: E402

if __name__ == "__main__":
    raise SystemExit(main())
