"""Pi background compute worker. It never opens a robot UDP socket."""
from __future__ import annotations

import argparse
import hmac
import json
import os
import signal
import socket
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

from .offload import evaluate


class Worker(ThreadingHTTPServer):
    daemon_threads = True

    def __init__(self, address, token):
        if len(token) < 24:
            raise ValueError("set private SWARM_TOKEN (24+ characters)")
        super().__init__(address, Handler)
        self.token, self.slot, self.completed = token, threading.Semaphore(1), 0


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *_): pass

    def reply(self, code, data):
        raw = json.dumps(data, allow_nan=False).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(raw)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(raw)

    def auth(self):
        if hmac.compare_digest(self.headers.get("Authorization", ""), "Bearer " + self.server.token):
            return True
        self.reply(401, {"error": "authentication required"})
        return False

    def do_GET(self):
        if self.auth():
            self.reply(200, {"role": "background-compute-only", "host": socket.gethostname(),
                "completed_jobs": self.server.completed, "robot_control": False})

    def do_POST(self):
        self.connection.settimeout(2)
        if not self.auth(): return
        if self.path != "/evaluate":
            self.reply(404, {"error": "unknown endpoint"}); return
        if not self.server.slot.acquire(blocking=False):
            self.reply(429, {"error": "worker busy; local control continues"}); return
        try:
            size = int(self.headers.get("Content-Length", 0))
            if not 0 < size <= 8192: raise ValueError("invalid request size")
            data = json.loads(self.rfile.read(size))
            if not isinstance(data, dict): raise ValueError("expected object")
            result = evaluate(data)
            self.server.completed += 1
            self.reply(200, {**result, "host": socket.gethostname(), "completed_jobs": self.server.completed})
        except (ValueError, TypeError, AttributeError, KeyError) as exc:
            self.reply(400, {"error": str(exc)})
        except OSError:
            pass  # client may time out; this does not affect local robot control
        finally:
            self.server.slot.release()


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--port", type=int, default=8770)
    a = ap.parse_args()
    server = Worker((a.host, a.port), os.environ.get("SWARM_TOKEN", ""))
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))
    print(f"Background offload worker on port {server.server_port}; no robot transport", flush=True)
    try:
        server.serve_forever(poll_interval=.2)
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == "__main__": main()
