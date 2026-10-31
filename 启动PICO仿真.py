#!/usr/bin/env python3
"""Launch local PICO simulation: python 启动PICO仿真.py --check-only."""
from scripts.setup.launcher import cli

if __name__ == "__main__":
    raise SystemExit(cli("pico"))
