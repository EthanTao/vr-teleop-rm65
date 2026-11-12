"""Pinned compute-only RealMan Algo and browser assets for the isolated websim image."""
import hashlib
import io
import json
from pathlib import Path
import platform
import tarfile
import time
import urllib.request

COMMIT = "a810b9fdea29e3567c5e9155794e6ef9cc17be62"
FILES = {
    "__init__.py": "e69de29bb2d1d6434b8b29ae775ad8c2e48c5391",
    "rm_ctypes_wrap.py": "d0e840ec28d3a192781656905ef427a497913b5d",
    "rm_robot_interface.py": "2c3ef267295d6f13eab536b4ea217533ef8fce67",
    "libs/linux_x86/libapi_c.so": "67e5311e52dc387c104fbc911220f1ee71d85f9f",
    "libs/linux_arm/libapi_c.so": "eb9018149afd6a22805935a7a4cdc00432e6f6fe",
}


def fetch(url):
    for attempt in range(3):
        try:
            with urllib.request.urlopen(url, timeout=90) as response:
                return response.read()
        except OSError:
            if attempt == 2:
                raise
            time.sleep(1 + attempt)


def main():
    arch = platform.machine()
    folder = {"x86_64": "linux_x86", "aarch64": "linux_arm"}.get(arch)
    if folder is None:
        raise RuntimeError(f"unsupported SDK architecture: {arch}")
    destination = Path("/opt/rm65-official-sdk/Robotic_Arm")
    manifest = {"sdk_commit": COMMIT, "architecture": arch, "files": {}}
    for name, expected in FILES.items():
        if name.startswith("libs/") and not name.startswith(f"libs/{folder}/"):
            continue
        data = fetch(f"https://raw.githubusercontent.com/RealManRobot/RM_API2/{COMMIT}/Python/Robotic_Arm/{name}")
        actual = hashlib.sha1(f"blob {len(data)}\0".encode() + data).hexdigest()
        if actual != expected:
            raise RuntimeError(f"SDK Git blob mismatch: {name}")
        path = destination / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(data)
        manifest["files"][name] = hashlib.sha256(data).hexdigest()
    archive = fetch("https://registry.npmjs.org/three/-/three-0.180.0.tgz")
    if hashlib.sha256(archive).hexdigest() != "ad66d724565ee29a2467277fa84daa5ed0211d6b8d446e9ef29f6bae0cd14144":
        raise RuntimeError("three.js archive mismatch")
    assets = Path("/opt/rm65-web-assets")
    assets.mkdir(parents=True, exist_ok=True)
    with tarfile.open(fileobj=io.BytesIO(archive), mode="r:gz") as tar:
        for name, member in {
            "three.module.js": "package/build/three.module.js",
            "three.core.js": "package/build/three.core.js",
            "OrbitControls.js": "package/examples/jsm/controls/OrbitControls.js",
            "THREE-LICENSE.txt": "package/LICENSE",
        }.items():
            (assets / name).write_bytes(tar.extractfile(member).read())
    Path("/opt/rm65-websim-dependencies.json").write_text(json.dumps(manifest, indent=2))


if __name__ == "__main__":
    main()
