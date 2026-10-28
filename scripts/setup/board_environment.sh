#!/bin/sh
# Board-side convenience entry; follows an optional workspace-root symlink.
set -eu
script_path=$(readlink -f -- "$0")
script_dir=$(dirname -- "$script_path")
exec python3 -B "$script_dir/../tools/board_environment.py" "$@"
