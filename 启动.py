#!/usr/bin/env python3
"""Interactive shortcuts for the current MoveIt Servo stack, not the old backend."""

from scripts.setup.launcher import cli

if __name__ == "__main__":
    raise SystemExit(cli("menu"))
