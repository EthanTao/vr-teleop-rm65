#!/bin/bash
set -euo pipefail

ARCH="$(uname -m)"
case "$ARCH" in
    aarch64|arm64) SDK_SUBDIR="arm64" ;;
    x86_64|amd64) SDK_SUBDIR="x64" ;;
    *) echo "Unsupported arch: $ARCH"; exit 1 ;;
esac

SDK_ROOT="/opt/apps/roboticsservice/SDK/${SDK_SUBDIR}"
if [[ ! -f "${SDK_ROOT}/libPXREARobotSDK.so" ]]; then
    echo "Missing ${SDK_ROOT}/libPXREARobotSDK.so — install PC Service first."
    exit 1
fi

WORKDIR="/tmp/XRoboToolkit-PC-Service-Pybind"
rm -rf "$WORKDIR"
git clone --depth 1 https://github.com/XR-Robotics/XRoboToolkit-PC-Service-Pybind.git "$WORKDIR"
cd "$WORKDIR"

# CMake layout differs by ISA (see CMakeLists.txt in pybind repo)
if [[ "$ARCH" == "aarch64" || "$ARCH" == "arm64" ]]; then
    mkdir -p lib/aarch64 include/aarch64
    cp "${SDK_ROOT}/libPXREARobotSDK.so" lib/aarch64/
    cp /opt/apps/roboticsservice/SDK/include/PXREARobotSDK.h include/aarch64/
    if [[ -d /opt/apps/roboticsservice/SDK/include/nlohmann ]]; then
        cp -r /opt/apps/roboticsservice/SDK/include/nlohmann include/aarch64/
    else
        git clone --depth 1 https://github.com/nlohmann/json.git /tmp/nlohmann
        cp -r /tmp/nlohmann/include/nlohmann include/aarch64/
    fi
else
    mkdir -p lib include
    cp "${SDK_ROOT}/libPXREARobotSDK.so" lib/
    cp /opt/apps/roboticsservice/SDK/include/PXREARobotSDK.h include/
    if [[ -d /opt/apps/roboticsservice/SDK/include/nlohmann ]]; then
        cp -r /opt/apps/roboticsservice/SDK/include/nlohmann include/
    else
        git clone --depth 1 https://github.com/nlohmann/json.git /tmp/nlohmann
        cp -r /tmp/nlohmann/include/nlohmann include/
    fi
fi

conda install -y -c conda-forge pybind11 libstdcxx-ng
uv pip uninstall -y xrobotoolkit_sdk xrobotoolkit-sdk-mock 2>/dev/null || true
python setup.py install
python -c "import xrobotoolkit_sdk as xrt; xrt.init(); xrt.close(); print('xrobotoolkit_sdk OK')"
