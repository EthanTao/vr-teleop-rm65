#!/usr/bin/env python3
"""One daily entry point for simulation, PICO simulation and the real robot."""

from __future__ import annotations

import argparse
from typing import Sequence

from scripts.setup import launcher


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="RM65 Studio；不指定模式时打开桌面控制台。",
        epilog="sim：无手柄仿真；pico：PICO 仿真；real：真机菜单。模式后加 --help 查看参数。",
    )
    parser.add_argument("--check-only", action="store_true", help="仅离线预览，不启动环境或连接设备")
    parser.add_argument("--pause", action="store_true", help="退出前等待 Enter（仅交互终端）")
    parser.add_argument("--cli", action="store_true", help="打开终端交互菜单")
    parser.add_argument("mode", nargs="?", choices=["sim", "pico", "real"], help="直接进入指定模式")
    parser.add_argument("options", nargs=argparse.REMAINDER, help="传给所选模式的参数")
    args = parser.parse_args(argv)
    if args.cli and args.mode:
        parser.error("--cli 用于统一菜单，不能同时指定模式")
    if args.mode is None and not args.cli and not args.check_only:
        from scripts.setup.desktop import main as desktop_main

        return desktop_main()
    options = list(args.options)
    if args.check_only:
        options.append("--check-only")
    if args.pause:
        options.append("--pause")
    kind = {None: "menu", "sim": "manual", "pico": "pico", "real": "real"}[args.mode]
    return launcher.cli(kind, options)


if __name__ == "__main__":
    raise SystemExit(main())
