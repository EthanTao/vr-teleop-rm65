"""控制链源码指纹（`xrobotoolkit_teleop/utils/code_fingerprint.py`）的测试。

这些测试刻意**不**断言具体哈希值：指纹的用途就是「代码一变它就变」，写死哈希只会让每次
改动控制代码都要改测试。这里钉住的是它的四条性质 —— 稳定、对内容敏感、对缺失敏感、不抛异常。

最后一条 `test_critical_sources_all_exist_in_this_repository` 是本文件里最重要的一条：
清单里任何一条路径写错，都会让本机与容器**同时**变成 `missing` 而看起来「一致」，
这恰好会把指纹变成它本来要防的那种假证据。
"""

from __future__ import annotations

from pathlib import Path

from xrobotoolkit_teleop.utils import code_fingerprint as cf


def _make_fake_repo(root: Path) -> Path:
    """按 `CRITICAL_SOURCES` 逐条建出可读文件，模拟一份仓库。"""
    for relative in cf.CRITICAL_SOURCES:
        path = root / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(f"# {relative}\n", encoding="utf-8")
    return root


def test_fingerprint_is_stable_for_identical_content(tmp_path):
    _make_fake_repo(tmp_path / "a")
    _make_fake_repo(tmp_path / "b")

    assert cf.source_fingerprint(tmp_path / "a") == cf.source_fingerprint(tmp_path / "b")


def test_fingerprint_changes_when_a_critical_source_changes(tmp_path):
    repo = _make_fake_repo(tmp_path)
    before = cf.source_fingerprint(repo)
    (repo / cf.CRITICAL_SOURCES[0]).write_text("# changed\n", encoding="utf-8")

    assert cf.source_fingerprint(repo) != before


def test_fingerprint_changes_when_a_critical_source_disappears(tmp_path):
    repo = _make_fake_repo(tmp_path)
    before = cf.source_fingerprint(repo)
    (repo / cf.CRITICAL_SOURCES[0]).unlink()

    assert cf.source_fingerprint(repo) != before
    assert cf.source_hashes(repo)[cf.CRITICAL_SOURCES[0]] == cf.MISSING


def test_fingerprint_does_not_raise_on_a_missing_tree(tmp_path):
    assert len(cf.source_fingerprint(tmp_path / "does-not-exist")) == 16


def test_fingerprint_is_16_hex_characters(tmp_path):
    _make_fake_repo(tmp_path)
    value = cf.source_fingerprint(tmp_path)

    assert len(value) == 16
    int(value, 16)


def test_fingerprint_line_uses_the_shared_prefix(tmp_path):
    _make_fake_repo(tmp_path)
    line = cf.fingerprint_line(tmp_path)

    assert line.startswith(cf.FINGERPRINT_PREFIX)
    assert cf.source_fingerprint(tmp_path) in line


def test_path_order_does_not_depend_on_the_filesystem(tmp_path):
    """同一份内容、同样的相对路径集合 → 同样的指纹（不依赖目录遍历顺序）。"""
    _make_fake_repo(tmp_path)
    assert cf.source_fingerprint(tmp_path) == cf.source_fingerprint(str(tmp_path))


def test_critical_sources_all_exist_in_this_repository():
    """清单必须指向真实文件，否则两端会一起退化成 missing 而「看起来一致」。"""
    missing = [relative for relative, digest in cf.source_hashes(cf.repository_root()).items() if digest == cf.MISSING]

    assert missing == []
