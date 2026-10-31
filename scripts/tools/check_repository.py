"""Offline repository checks; never launch ROS, WSL, containers, or hardware."""

from __future__ import annotations

import argparse
import ast
from pathlib import Path
import re
import subprocess
import sys
from urllib.parse import unquote, urlsplit
import xml.etree.ElementTree as ET

import yaml

ROOT = Path(__file__).resolve().parents[2]
PACKAGE = Path("ros2_ws/src/rm65_servo_teleop")
GENERATED = {"build", "install", "log", "vendor", "__pycache__"}
LINK_DOCUMENTS = (
    "README.md",
    "CONTRIBUTING.md",
    "docs/README.md",
    "docs/仓库结构.md",
    "scripts/README.md",
    "ros2_ws/README.md",
    "ros2_ws/patches/README.md",
    "config/README.md",
)


def repository_files(root: Path) -> list[Path]:
    """Include new project files while excluding ignored data and old snapshots."""
    result = subprocess.run(
        ["git", "ls-files", "--cached", "--others", "--exclude-standard", "-z"],
        cwd=root,
        check=True,
        capture_output=True,
    )
    paths = {Path(name) for name in result.stdout.decode("utf-8").split("\0") if name}
    return sorted(
        path
        for path in paths
        if (root / path).is_file()
        and not GENERATED.intersection(path.parts)
        and not path.as_posix().startswith(("docs/archive/", ".agents/"))
    )


def check_sources(root: Path, paths: list[Path]) -> list[str]:
    errors = []
    for path in paths:
        python_script = path.suffix == ".py" or path.parent == PACKAGE / "scripts"
        yaml_file = path.suffix in {".yaml", ".yml", ".repos"}
        if not (python_script or yaml_file or path.suffix in {".xml", ".sh", ".patch"}):
            continue
        data = (root / path).read_bytes()
        try:
            source = data.decode("utf-8-sig")
            if python_script:
                ast.parse(source, filename=str(path), feature_version=(3, 10))
            elif yaml_file:
                list(yaml.safe_load_all(source))
            elif path.suffix == ".xml":
                ET.fromstring(source)
            if (path.suffix in {".sh", ".patch"} or path.parent == PACKAGE / "scripts") and b"\r" in data:
                errors.append(f"{path}: Linux script/patch requires LF line endings")
        except (UnicodeError, SyntaxError, ValueError, yaml.YAMLError, ET.ParseError) as exc:
            errors.append(f"{path}: {exc}")
    return errors


def check_installed_scripts(root: Path) -> list[str]:
    """Catch missing ROS executable sources and scripts omitted from installation."""
    package = root / PACKAGE
    cmake = (package / "CMakeLists.txt").read_text(encoding="utf-8")
    installed = set()
    errors = []
    for block in re.findall(r"install\s*\(\s*PROGRAMS\s+(.*?)\s+DESTINATION", cmake, re.S):
        for name in block.split():
            installed.add(name)
            if not (package / name).is_file():
                errors.append(f"{PACKAGE}/CMakeLists.txt: installed script missing: {name}")
    for path in sorted((package / "scripts").iterdir()):
        if path.is_file() and f"scripts/{path.name}" not in installed:
            errors.append(f"{PACKAGE}/scripts/{path.name}: absent from CMake install(PROGRAMS)")
    return errors


def check_document_links(root: Path) -> list[str]:
    errors = []
    for name in LINK_DOCUMENTS:
        document = root / name
        if not document.is_file():
            errors.append(f"{name}: navigation document missing")
            continue
        for target in re.findall(r"\[[^\]]*\]\(([^)]+)\)", document.read_text(encoding="utf-8")):
            target = target.strip().removeprefix("<").removesuffix(">")
            url = urlsplit(target)
            if url.scheme or url.netloc or not url.path:
                continue
            if not (document.parent / unquote(url.path)).exists():
                errors.append(f"{name}: local link missing: {target}")
    return errors


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--static-only", action="store_true", help="Skip pytest; still run source checks and Ruff")
    args = parser.parse_args(argv)
    paths = repository_files(ROOT)
    errors = check_sources(ROOT, paths) + check_installed_scripts(ROOT) + check_document_links(ROOT)
    if errors:
        print("\n".join(errors), file=sys.stderr)
        return 1
    print(f"Repository checks passed ({len(paths)} files; Python 3.10 syntax, YAML/XML, scripts, links)", flush=True)
    commands = [[sys.executable, "-m", "ruff", "check", "."]]
    if not args.static_only:
        commands.append([sys.executable, "-B", "-m", "pytest", "-q", "--junitxml=build/test-results/pytest.xml"])
    for command in commands:
        result = subprocess.run(command, cwd=ROOT, check=False)
        if result.returncode:
            return result.returncode
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
