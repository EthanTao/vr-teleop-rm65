"""Keep local pytest scratch files in the ignored workspace build directory."""

from pathlib import Path
import os
import tempfile

_scratch = Path(__file__).resolve().parent / "build/tmp"
_scratch.mkdir(parents=True, exist_ok=True)
for _name in ("TMPDIR", "TEMP", "TMP"):
    os.environ[_name] = str(_scratch)
tempfile.tempdir = str(_scratch)
