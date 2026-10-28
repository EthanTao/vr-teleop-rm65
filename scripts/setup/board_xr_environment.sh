#!/bin/bash
set -eo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
exec python3 -B "$root/scripts/tools/board_xr_environment.py" "$@"
