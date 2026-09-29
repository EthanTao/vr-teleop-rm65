#!/usr/bin/env python3
"""CLI：对一次真机验收做**量化判定**，一键出「过 / 不过」清单。

逻辑全部在 `xrobotoolkit_teleop/utils/acceptance_criteria.py`（可被单元测试直接覆盖），
本文件只做入口。

跑法（仓库根目录）：

    python scripts/debug/report_teleop_acceptance.py build/motion-2026-09-30.jsonl
    python scripts/debug/report_teleop_acceptance.py build/motion-2026-09-30.jsonl --json
    python scripts/debug/report_teleop_acceptance.py build/motion-2026-09-30.jsonl \
        --min-run-seconds 30 --min-follow-ratio 0.6

退出码：`0` = 通过；`2` = 未通过；其它 = 出错。只读一个日志文件，不连真机。
"""

from __future__ import annotations

import sys
from pathlib import Path

# 允许直接从仓库根运行，而不要求先 `pip install -e .`。
_REPO_ROOT = Path(__file__).resolve().parents[2]
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))

from xrobotoolkit_teleop.utils.acceptance_criteria import main  # noqa: E402

if __name__ == "__main__":
    raise SystemExit(main())
