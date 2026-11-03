"""Authenticated LAN HTTP observations in; Pi-only planning and UDP out."""
from __future__ import annotations

import argparse
import hmac
import json
import os
import socket
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

from .config import arena_from, bots_from, load_config, swarm_params_from
from .coordinator import Coordinator
from .protocol import encode


class PiServer(ThreadingHTTPServer):
    daemon_threads = True

    def __init__(self, address, brain, bots, token, log_path=None):
        if len(token) < 24:
            raise ValueError("set SWARM_TOKEN to at least 24 characters")
        super().__init__(address, Handler)
        self.brain, self.bots, self.token = brain, bots, token
        self.lock, self.done = threading.RLock(), threading.Event()
        self.udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.log_path, self.last_log = log_path, 0.0
        self.worker = threading.Thread(target=self._transmit, daemon=True)
        self.worker.start()

    def _transmit(self):
        while not self.done.is_set():
            now = time.monotonic()
            with self.lock:
                cmds = self.brain.outputs(now)
                seq = int(now * 20) % 1000000000
                for i, c in cmds.items():
                    try:
                        self.udp.sendto(encode(i, seq, c), self.bots[i])
                    except OSError:
                        self.brain.halt("UDP send failed")
                        break
                if self.log_path and now - self.last_log >= 1:
                    self.last_log = now
                    with open(self.log_path, "a", encoding="utf-8") as f:
                        f.write(json.dumps({"monotonic": now, **self.brain.status(now)}) + "\n")
            self.done.wait(0.05)

    def server_close(self):
        self.done.set()
        self.worker.join(timeout=2)
        with self.lock:
            self.brain.halt("shutdown")
        for _ in range(3):
            for i in self.bots:
                try:
                    self.udp.sendto(encode(i, 0, self.brain.stops()[i]), self.bots[i])
                except OSError:
                    pass
        self.udp.close()
        super().server_close()


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *_):
        pass  # tokens and camera observations never go into HTTP logs

    def reply(self, code, data):
        raw = json.dumps(data, allow_nan=False).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(raw)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(raw)

    def authenticated(self):
        if not hmac.compare_digest(self.headers.get("Authorization", ""), "Bearer " + self.server.token):
            self.reply(401, {"error": "authentication required"})
            return False
        return True

    def do_GET(self):
        if not self.authenticated():
            return
        with self.server.lock:
            if self.path == "/status":
                self.reply(200, self.server.brain.status(time.monotonic()))
            else:
                self.reply(404, {"error": "unknown endpoint"})

    def do_POST(self):
        self.connection.settimeout(1)
        if not self.authenticated():
            return
        try:
            length = int(self.headers.get("Content-Length", "0"))
            if not 0 < length <= 8192:
                raise ValueError("body must be 1..8192 bytes")
            data = json.loads(self.rfile.read(length))
            if not isinstance(data, dict):
                raise ValueError("body must be an object")
            with self.server.lock:
                now, brain = time.monotonic(), self.server.brain
                if self.path == "/session":
                    result = brain.new_session(now)
                elif self.path == "/observe":
                    result = brain.observe(data, now)
                elif self.path == "/stop":
                    brain.halt("operator stop")
                    result = brain.status(now)
                else:
                    self.reply(404, {"error": "unknown endpoint"})
                    return
                self.reply(200, result)
        except (ValueError, OSError) as exc:
            with self.server.lock:
                self.server.brain.halt("invalid request")
            self.reply(400, {"error": str(exc)})


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--config", default="config.toml")
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--port", type=int, default=8765)
    ap.add_argument("--bots", default="2")
    ap.add_argument("--mode", choices=["swarm", "led"], default="swarm")
    ap.add_argument("--allow-motion", action="store_true", help="enable only after wheels-raised checks")
    ap.add_argument("--model", default="models/target.json")
    a = ap.parse_args()
    cfg = load_config(a.config)
    available = bots_from(cfg)
    bots = {int(i): available[int(i)] for i in a.bots.split(",")}
    from .target_model import TargetModel
    model = TargetModel.load(a.model) if a.mode == "swarm" else None
    brain = Coordinator(arena_from(cfg), list(bots), swarm_params_from(cfg), a.allow_motion, a.mode, model)
    Path("logs").mkdir(exist_ok=True)
    server = PiServer((a.host, a.port), brain, bots, os.environ.get("SWARM_TOKEN", ""), "logs/pi.jsonl")
    print(f"Pi coordinator on {a.host}:{server.server_port}; bots={list(bots)}; motion={a.allow_motion}", flush=True)
    try:
        server.serve_forever(poll_interval=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
