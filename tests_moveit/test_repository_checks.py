"""Repository gate failure cases, independent of ROS and connected devices."""

from pathlib import Path

import pytest

from scripts.tools import check_repository as checks


@pytest.mark.parametrize(
    "name,source,valid",
    [
        ("example.py", "answer: int | None = None\n", True),
        ("example.py", "type Answer = int\n", False),
        ("config.yaml", "mode: sim\n", True),
        ("config.yaml", "mode: [\n", False),
        ("package.xml", "<package><name>demo</name></package>\n", True),
        ("package.xml", "<package>\n", False),
        ("entry.sh", "#!/bin/bash\necho ok\n", True),
        ("entry.sh", "#!/bin/bash\r\necho ok\r\n", False),
        (str(checks.PACKAGE / "scripts/entry"), "#!/usr/bin/env python3\ndef broken(:\n", False),
    ],
)
def test_source_contracts(tmp_path, name, source, valid):
    path = Path(name)
    target = tmp_path / path
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_bytes(source.encode("utf-8"))
    assert (not checks.check_sources(tmp_path, [path])) == valid


def test_install_check_rejects_missing_and_uninstalled_scripts(tmp_path):
    package = tmp_path / checks.PACKAGE
    (package / "scripts").mkdir(parents=True)
    (package / "scripts/extra").write_text("#!/usr/bin/env python3\n", encoding="utf-8")
    (package / "CMakeLists.txt").write_text(
        "install(PROGRAMS scripts/missing DESTINATION lib/demo)\n", encoding="utf-8"
    )
    errors = checks.check_installed_scripts(tmp_path)
    assert any("installed script missing: scripts/missing" in error for error in errors)
    assert any("scripts/extra: absent from" in error for error in errors)
    (package / "scripts/missing").write_text("#!/usr/bin/env python3\n", encoding="utf-8")
    (package / "CMakeLists.txt").write_text(
        "install(PROGRAMS scripts/missing scripts/extra DESTINATION lib/demo)\n", encoding="utf-8"
    )
    assert checks.check_installed_scripts(tmp_path) == []


def test_navigation_resolves_relative_and_encoded_links(tmp_path, monkeypatch):
    monkeypatch.setattr(checks, "LINK_DOCUMENTS", ("README.md",))
    (tmp_path / "文档.md").write_text("# 文档\n", encoding="utf-8")
    (tmp_path / "README.md").write_text(
        "[local](%E6%96%87%E6%A1%A3.md#section) [web](https://example.com/missing) "
        "[anchor](#section) [bad](missing.md)\n",
        encoding="utf-8",
    )
    assert checks.check_document_links(tmp_path) == ["README.md: local link missing: missing.md"]


def test_static_failure_stops_before_any_test_or_lint_process(monkeypatch):
    monkeypatch.setattr(checks, "repository_files", lambda root: [])
    monkeypatch.setattr(checks, "check_sources", lambda root, paths: ["invalid source"])
    monkeypatch.setattr(checks, "check_installed_scripts", lambda root: [])
    monkeypatch.setattr(checks, "check_document_links", lambda root: [])
    monkeypatch.setattr(checks.subprocess, "run", lambda *a, **kw: pytest.fail("started a child after a failed gate"))
    assert checks.main(["--static-only"]) == 1
