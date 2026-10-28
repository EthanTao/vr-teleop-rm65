"""Same-host/boot clock identity; no claim of exposure or controller sync."""

import hashlib
from pathlib import Path


def clock_identity():
    boot = Path("/proc/sys/kernel/random/boot_id")
    if boot.is_file():
        # Container hostnames differ on the same kernel; the boot UUID identifies
        # the monotonic epoch shared by these Linux processes/containers.
        value = boot.read_text().strip()
        return "linux-monotonic:" + hashlib.sha256(value.encode()).hexdigest()
    # No stable Windows boot identity is guessed. Pairing requires Linux or a
    # separately measured clock bridge, rather than comparing unrelated clocks.
    return None
