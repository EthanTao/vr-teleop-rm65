#!/bin/bash
set -euo pipefail

ARCH="$(uname -m)"
RELEASE="v1.0.0"
BASE="https://github.com/XR-Robotics/XRoboToolkit-PC-Service/releases/download/${RELEASE}"

case "$ARCH" in
    aarch64|arm64)
        DEB="XRoboToolkit-PC-Service-headless_1.0.0.0_arm64.deb"
        SDK_SUBDIR="arm64"
        PACKAGE_ARCH="arm64"
        SHA256="532c605dfa1a02b05b7c285b856c91771c78623cded30ef5b16ea371de49ed5f"
        ;;
    x86_64|amd64)
        DEB="XRoboToolkit_PC_Service_1.0.0_ubuntu_22.04_amd64.deb"
        SDK_SUBDIR="x64"
        PACKAGE_ARCH="amd64"
        SHA256="61961067eb4b41f81ed7cae35f4690dbb0ddfefb329a12b24e0b90ebc46ada91"
        ;;
    *)
        echo "Unsupported architecture: $ARCH"
        exit 1
        ;;
esac

echo "Installing PC Service: $DEB (arch=$ARCH)"
PACKAGE="${RM65_XR_DEB:-/tmp/${DEB}}"
if [[ -z "${RM65_XR_DEB:-}" ]]; then
    curl --retry 3 --connect-timeout 15 -fsSL -o "$PACKAGE" "${BASE}/${DEB}"
fi
printf '%s  %s\n' "$SHA256" "$PACKAGE" | sha256sum --check --status
[[ "$(dpkg-deb -f "$PACKAGE" Architecture)" == "$PACKAGE_ARCH" ]]

missing=()
for dependency in xdg-utils libdouble-conversion3 libb2-1 libpcre2-16-0 libglib2.0-0; do
    if [[ "$(dpkg-query -W -f='${Status}' "$dependency" 2>/dev/null || true)" != "install ok installed" ]]; then
        missing+=("$dependency")
    fi
done
if (( ${#missing[@]} )); then
    apt-get update
    DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends "${missing[@]}"
fi
# The vendor package declares no dependencies. Extract its verified runtime
# without running desktop integration/host service maintainer scripts.
dpkg-deb --extract "$PACKAGE" /
if [[ -z "${RM65_XR_DEB:-}" ]]; then rm -f "$PACKAGE"; fi

export SDK_LIB="/opt/apps/roboticsservice/SDK/${SDK_SUBDIR}/libPXREARobotSDK.so"
if [[ ! -f "$SDK_LIB" ]]; then
    echo "SDK library not found at $SDK_LIB"
    find /opt/apps/roboticsservice -name 'libPXREARobotSDK.so' || true
    exit 1
fi
service_root=/opt/apps/roboticsservice
for binary in "$SDK_LIB" "$service_root/RoboticsServiceProcess"; do
    dependencies="$(LD_LIBRARY_PATH="$service_root:$service_root/lib:$service_root/SDK/$SDK_SUBDIR" ldd "$binary")"
    if [[ "$dependencies" == *"not found"* ]]; then
        printf '%s\n' "$dependencies" >&2
        exit 1
    fi
done
export RM65_XR_RELEASE="$RELEASE" RM65_XR_PACKAGE="$DEB" RM65_XR_SHA256="$SHA256"
python3 - <<'PY'
import hashlib, json, os
from pathlib import Path
library = Path(os.environ['SDK_LIB'])
Path('/opt/rm65-xr-sdk.json').write_text(json.dumps({
    'release': os.environ['RM65_XR_RELEASE'], 'package': os.environ['RM65_XR_PACKAGE'],
    'package_sha256': os.environ['RM65_XR_SHA256'],
    'sdk_library_sha256': hashlib.sha256(library.read_bytes()).hexdigest(),
    'source': 'https://github.com/XR-Robotics/XRoboToolkit-PC-Service/releases/tag/v1.0.0',
}, indent=2))
PY
echo "PC Service installed. SDK: $SDK_LIB"
