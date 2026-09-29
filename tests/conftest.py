"""pytest 全局配置：把临时目录钉在仓库内的 `build/tmp`。

为什么需要它
------------
在受限沙箱里跑测试时，系统盘上的标准临时目录（`%TEMP%`、`C:\\Windows\\Temp`、
`c:\\temp`、`\\tmp` …）全都不可写，`tempfile.gettempdir()` 会顺位退回到候选列表的
**最后一个：`os.getcwd()`**，也就是仓库根目录。于是：

- `tests/test_data_collector.py` 的 fixture 用 `tempfile.TemporaryDirectory()`
  （内部即 `mkdtemp()`，前缀 `tmp` + 8 位随机串），每个用例在仓库根建一个
  `tmpXXXXXXXX` 目录——8 个用例正好对上 2026-09-26 清理时清掉的 8 个；
- `teleop.py` 的同步清单缓存也写进了仓库根：`tests/test_teleop_launcher.py` 的
  `test_sync_code_skips_upload_when_remote_is_current` 没有打桩 `_save_manifest`，
  于是真的创建了 `vr-teleop-rm65.manifest.json`（体积/SHA-256 与该文件对得上）。

这些目录为什么在会话结束后还留在盘上，现存证据分不清「进程被中断、`TemporaryDirectory`
来不及清理」与「沙箱 ACL 阻止删除」两种可能；机制上确定的是它们被创建在仓库根。
清理清单见 `build/archive/layout-cleanup-2026-09-26.json`。

这里只在「默认临时目录已经落到仓库内」时才改道到 `build/tmp/`；容器与本机正常
环境下 `/tmp`、`%TEMP%` 可写，行为完全不变。必须在测试模块导入前执行，所以放在
模块顶层而不是 fixture 里——`import teleop` 之类的模块级副作用就发生在收集阶段。
"""

from __future__ import annotations

import os
import tempfile
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
REPO_TMP = REPO_ROOT / "build" / "tmp"


def _pin_tempdir_inside_repo() -> None:
    """默认临时目录落在仓库内时，改道到 `build/tmp/`。"""
    try:
        current = Path(tempfile.gettempdir()).resolve()
    except OSError:
        return
    if current != REPO_ROOT and REPO_ROOT not in current.parents:
        return
    try:
        REPO_TMP.mkdir(parents=True, exist_ok=True)
    except OSError:
        # 只读检出等场景：保持原样，让各用例自己失败，不要连测试收集一起打断
        return
    for var in ("TMPDIR", "TEMP", "TMP"):
        os.environ[var] = str(REPO_TMP)
    tempfile.tempdir = str(REPO_TMP)


_pin_tempdir_inside_repo()
