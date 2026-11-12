"""Owned bounded pipe protocol between ROS 3.10 and isolated MuJoCo 3.11."""
import json
import os
from pathlib import Path
import select
import subprocess


class PhysicsProcess:
    def __init__(self, options, *, python=None):
        if python is None:
            python = "/opt/python311/bin/python3.11"
            if not Path(python).is_file():
                raise RuntimeError("isolated MuJoCo Python 3.11 is missing")
        environment = {**os.environ, "PYTHONUNBUFFERED": "1"}
        for name in ("PYTHONPATH", "PYTHONHOME", "LD_LIBRARY_PATH"):
            environment.pop(name, None)  # Never load ROS 3.10 extension modules into Python 3.11.
        if python == "/opt/python311/bin/python3.11":
            environment["LD_LIBRARY_PATH"] = "/opt/python311/lib"
        self.process = subprocess.Popen([python, str(Path(__file__).with_name("websim_worker.py")), *options],
                                        stdin=subprocess.PIPE, stdout=subprocess.PIPE, bufsize=0,
                                        env=environment, start_new_session=True)
        self.buffer = bytearray()
        try:
            ready = self.read(15.0)
            if ready != {"ready": True, "engine": "4.0.0"}:
                raise RuntimeError("unexpected MuJoCo worker handshake")
        except BaseException:
            self.close()
            raise

    def read(self, timeout):
        import time
        deadline = time.monotonic() + timeout
        while b"\n" not in self.buffer:
            remaining = deadline - time.monotonic()
            if remaining <= 0 or not select.select([self.process.stdout], [], [], remaining)[0]:
                raise RuntimeError("MuJoCo worker response deadline expired")
            chunk = os.read(self.process.stdout.fileno(), 65536)
            if not chunk:
                raise RuntimeError("MuJoCo worker exited")
            self.buffer.extend(chunk)
            if len(self.buffer) > 65536:
                raise RuntimeError("oversized MuJoCo worker response")
        line, _, remainder = self.buffer.partition(b"\n")
        self.buffer = bytearray(remainder)
        return json.loads(line)

    def exchange(self, request):
        data = (json.dumps(request, allow_nan=False, separators=(",", ":")) + "\n").encode()
        if len(data) > 65536:
            raise ValueError("oversized physics request")
        self.process.stdin.write(data)
        return self.read(.2)

    def close(self):
        if self.process.stdin is not None:
            self.process.stdin.close()
        try:
            self.process.wait(timeout=3)
        except subprocess.TimeoutExpired:
            self.process.terminate()
            try:
                self.process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout=3)
        if self.process.stdout is not None:
            self.process.stdout.close()
