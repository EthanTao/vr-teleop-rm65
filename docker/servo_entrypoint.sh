#!/bin/bash
set -eo pipefail
# ROS/ament setup scripts read optional unset variables. Enable nounset only
# after they finish; keep failure and pipeline checks active throughout.
source /opt/ros/humble/setup.bash
source /opt/rm65_ws/install/setup.bash
set -u

service_pid=""
control_pid=""
cleanup() {
    trap - EXIT INT TERM
    # Only signal process groups created by this entrypoint; never use broad pkill.
    if [[ -n "$control_pid" ]]; then
        # Let ros2 launch mark its own shutdown before it signals child nodes.
        # A simultaneous group signal can make a clean child exit look like a
        # critical process failure before launch handles its SIGINT.
        kill -s INT "$control_pid" 2>/dev/null || true
        for attempt in {1..30}; do
            kill -0 "$control_pid" 2>/dev/null || break
            sleep 0.1
        done
        if kill -0 "$control_pid" 2>/dev/null; then
            kill -s INT -- "-${control_pid}" 2>/dev/null || true
        fi
        wait "$control_pid" 2>/dev/null || true
    fi
    if [[ -n "$service_pid" ]]; then
        kill -s TERM -- "-${service_pid}" 2>/dev/null || true
        wait "$service_pid" 2>/dev/null || true
    fi
}
trap cleanup EXIT
# An explicit stop is successful; unexpected child failures retain their code.
trap 'cleanup; exit 0' INT TERM
if [[ "${RM65_START_XR_SERVICE:-0}" == "1" ]]; then
    service_root=/opt/apps/roboticsservice
    [[ -x "$service_root/RoboticsServiceProcess" ]] || { echo "Build with RM65_WITH_XR_SDK=ON for PICO input" >&2; exit 2; }
    # Port conflicts fail explicitly instead of terminating another running session.
    if ss -lnt | grep -q ':63901 '; then
        echo "An XR service already owns port 63901; refusing a second instance" >&2
        exit 2
    fi
    cd "$service_root"
    # The official runService.sh backgrounds its process and exits immediately.
    # Keep the same library paths, but supervise the actual foreground service.
    case "$(uname -m)" in
        x86_64|amd64) sdk_arch=x64 ;;
        aarch64|arm64) sdk_arch=arm64 ;;
        *) echo "Unsupported XR service architecture" >&2; exit 2 ;;
    esac
    # The vendor Qt6 plugins belong to this service, not to ROS/RViz's Qt5.
    setsid env \
        LD_LIBRARY_PATH="${LD_LIBRARY_PATH:-}:$service_root:$service_root/lib:$service_root/SDK/$sdk_arch" \
        QT_PLUGIN_PATH="$service_root/plugins/:${QT_PLUGIN_PATH:-}" \
        QT_QML_PATH="$service_root/qml/:${QT_QML_PATH:-}" \
        ./RoboticsServiceProcess &
    service_pid=$!
    ready=0
    for attempt in {1..30}; do
        if ! kill -0 "$service_pid" 2>/dev/null; then
            echo "XR service exited before it became ready" >&2
            exit 2
        fi
        if ss -lnt | grep -q ':63901 ' && ss -lnt | grep -q ':60061 '; then ready=1; break; fi
        sleep 0.2
    done
    [[ "$ready" == "1" ]] || { echo "XR service did not become ready" >&2; exit 2; }
fi
cd /opt/rm65_ws
# Bash asynchronous jobs inherit ignored INT/QUIT. ROS preserves that choice,
# so reset these dispositions before launch installs its normal shutdown hooks.
setsid env --default-signal=INT --default-signal=QUIT "$@" &
control_pid=$!
if [[ -n "$service_pid" ]]; then
    child_status=0
    finished_pid=""
    wait -n -p finished_pid "$service_pid" "$control_pid" || child_status=$?
    if [[ "$finished_pid" == "$service_pid" ]]; then
        echo "XR service exited; stopping the control graph" >&2
        exit 2
    fi
    exit "$child_status"
else
    wait "$control_pid"
fi
