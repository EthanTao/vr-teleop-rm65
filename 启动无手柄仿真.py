#!/usr/bin/env python3
"""Launch local manual-input simulation: python 启动无手柄仿真.py --check-only."""
from scripts.setup.launcher import cli

if __name__ == "__main__":
    raise SystemExit(cli("manual"))
