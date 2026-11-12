"""Read-only browser server; no input uploads, motion or simulator control API."""
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import threading


class BrowserView:
    def __init__(self, page, assets, geometry, *, address="127.0.0.1", port=8094):
        page = page.read_bytes()
        model = json.dumps(geometry, allow_nan=False, separators=(",", ":")).encode()
        static = {"/static/" + name: (assets / name).read_bytes()
                  for name in ("three.module.js", "three.core.js", "OrbitControls.js")}
        self.lock = threading.Lock()
        self.state = b'{}'
        view = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *args):
                pass

            def do_GET(self):
                if self.path == "/":
                    data, kind = page, "text/html; charset=utf-8"
                elif self.path == "/model":
                    data, kind = model, "application/json"
                elif self.path == "/state":
                    with view.lock:
                        data = view.state
                    kind = "application/json"
                elif self.path in static:
                    data, kind = static[self.path], "text/javascript"
                else:
                    self.send_error(404)
                    return
                self.send_response(200)
                self.send_header("Content-Type", kind)
                self.send_header("Content-Length", str(len(data)))
                self.send_header("Cache-Control", "no-store")
                self.send_header("X-Content-Type-Options", "nosniff")
                self.end_headers()
                try:
                    self.wfile.write(data)
                except (BrokenPipeError, ConnectionResetError):
                    pass

        self.server = ThreadingHTTPServer((address, port), Handler)
        self.server.daemon_threads = True
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)

    def update(self, state):
        data = json.dumps(state, allow_nan=False, separators=(",", ":")).encode()
        with self.lock:
            self.state = data

    def start(self):
        self.thread.start()

    def close(self):
        self.server.shutdown()
        self.server.server_close()
        self.thread.join(timeout=3)
