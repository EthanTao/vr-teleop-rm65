#!/bin/bash
set -euo pipefail

source /opt/conda/etc/profile.d/conda.sh
conda activate xrobo

SERVICE_DIR="/opt/apps/roboticsservice"
LOG_DIR="/var/log/xrobo"
mkdir -p "$LOG_DIR"

echo "Starting XRoboToolkit PC Service..."
cd "$SERVICE_DIR"
pkill -f RoboticsServiceProcess 2>/dev/null || true
sleep 1
nohup bash runService.sh >>"${LOG_DIR}/pc-service.log" 2>&1 &

SERVICE_PID=""
for i in $(seq 1 30); do
    if ss -tlnp 2>/dev/null | grep -q 63901 || netstat -tlnp 2>/dev/null | grep -q 63901; then
        SERVICE_PID=$(pgrep -f RoboticsServiceProcess | head -1 || true)
        echo "PC Service listening on port 63901"
        break
    fi
    sleep 1
done

if [[ -z "$SERVICE_PID" ]] && ! ss -tlnp 2>/dev/null | grep -q 63901; then
    echo "PC Service failed to start. Log:"
    tail -50 "${LOG_DIR}/pc-service.log" || true
    exit 1
fi

SERVICE_PID="${SERVICE_PID:-$(pgrep -f RoboticsServiceProcess | head -1 || true)}"

cleanup() {
    echo "Shutting down..."
    if [[ -n "$SERVICE_PID" ]]; then
        kill "$SERVICE_PID" 2>/dev/null || true
        wait "$SERVICE_PID" 2>/dev/null || true
    fi
    pkill -f RoboticsServiceProcess 2>/dev/null || true
}
trap cleanup EXIT INT TERM

HOST_IP_HINT="${HOST_IP:-}"
echo ""
echo "=============================================="
echo " PICO 4 Ultra: connect to this machine's LAN IP"
echo "   PC Service port: 63901 (TCP/UDP)"
if [[ -n "$HOST_IP_HINT" ]]; then
    echo "   Suggested IP (set HOST_IP): $HOST_IP_HINT"
fi
echo "   Placo 3D view: http://localhost:${MESHCAT_HOST_PORT:-18081}/static/"
echo "   Enable 'Send' + check 'Controller' in headset tracking panel"
echo "   Hold GRIP (side button) while moving controller to drive arms"
echo "=============================================="
echo ""

cd /workspace
exec "$@"
