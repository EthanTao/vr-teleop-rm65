#!/bin/bash
set -euo pipefail

ARCH="$(uname -m)"
RELEASE="v1.0.0"
BASE="https://github.com/XR-Robotics/XRoboToolkit-PC-Service/releases/download/${RELEASE}"

case "$ARCH" in
    aarch64|arm64)
        DEB="XRoboToolkit-PC-Service-headless_1.0.0.0_arm64.deb"
        SDK_SUBDIR="arm64"
        ;;
    x86_64|amd64)
        DEB="XRoboToolkit_PC_Service_1.0.0_ubuntu_22.04_amd64.deb"
        SDK_SUBDIR="x64"
        ;;
    *)
        echo "Unsupported architecture: $ARCH"
        exit 1
        ;;
esac

echo "Installing PC Service: $DEB (arch=$ARCH)"
curl -fsSL -o "/tmp/${DEB}" "${BASE}/${DEB}"
apt-get update
apt-get install -y xdg-utils libdouble-conversion3 libb2-1 libpcre2-16-0 libglib2.0-0
apt-get install -y "/tmp/${DEB}" || apt-get install -fy
rm -f "/tmp/${DEB}"

export SDK_LIB="/opt/apps/roboticsservice/SDK/${SDK_SUBDIR}/libPXREARobotSDK.so"
if [[ ! -f "$SDK_LIB" ]]; then
    echo "SDK library not found at $SDK_LIB"
    find /opt/apps/roboticsservice -name 'libPXREARobotSDK.so' || true
    exit 1
fi
echo "PC Service installed. SDK: $SDK_LIB"
