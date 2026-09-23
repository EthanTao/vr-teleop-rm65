#!/usr/bin/env bash
# Build and run VR teleop stack in Ubuntu Docker on Mac.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

if ! command -v docker &>/dev/null; then
    echo "Docker not found. Install Docker Desktop for Mac first."
    exit 1
fi

# Detect Mac Wi-Fi IP for PICO connection hint
if [[ "$(uname -s)" == "Darwin" ]]; then
    export HOST_IP="${HOST_IP:-$(ipconfig getifaddr en0 2>/dev/null || true)}"
fi

cd "$ROOT"
docker compose -f docker/docker-compose.yml up --build "$@"
