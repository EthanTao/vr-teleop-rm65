#!/usr/bin/env python3
"""Interactive board menu with no arguments; explicit CLI retains all safety gates."""

from scripts.setup.launcher import cli

if __name__ == "__main__":
    raise SystemExit(cli("real"))
