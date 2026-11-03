"""Pi-only inference and zero-speed LED demo. No arena calibration needed."""
from __future__ import annotations

import argparse
import json
import math
import os
import secrets
import signal
import socket
import subprocess
import threading
import time
from http.server import ThreadingHTTPServer

from .config import load_config, bots_from
from .pi_server import Handler as AuthHandler
from .protocol import Command, encode
from .target_model import TargetModel


def query_bot(address, bot_id, timeout=0.25):
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        s.settimeout(timeout)
        s.sendto(b"SWARM_DISCOVER", address)
        raw, peer = s.recvfrom(1024)
        data = json.loads(raw)
        if peer[0] != address[0] or data.get("id") != bot_id or data.get("project") != "pi-swarm-v1":
            raise ValueError("unexpected robot reply")
        return data


class Preview:
    def __init__(self, model):
        self.model = model
        self.session = self.ticket = ""
        self.seq, self.until, self.ticket_until = -1, 0.0, 0.0
        self.probability = 0.0

    def stop(self):
        self.until, self.probability = 0.0, 0.0

    def reply(self, now):
        self.ticket, self.ticket_until = secrets.token_hex(16), now + 0.5
        return {"session": self.session, "ticket": self.ticket,
                "probability": self.probability, "accepted": self.probability >= self.model.threshold,
                "inference_host": socket.gethostname(), "motors": [0, 0]}

    def new_session(self, now):
        self.stop()
        self.session, self.seq = secrets.token_hex(16), -1
        return self.reply(now)

    def classify(self, data, now):
        try:
            if not self.session or data.get("session") != self.session or data.get("ticket") != self.ticket:
                raise ValueError("invalid session")
            if now >= self.ticket_until:
                raise ValueError("expired frame ticket")
            self.ticket = ""
            seq, age, features = data.get("seq"), data.get("age_s"), data.get("features")
            if type(seq) is not int or seq <= self.seq:
                raise ValueError("replayed frame")
            if type(age) not in (int, float) or not math.isfinite(age) or not 0 <= age < .35:
                raise ValueError("stale frame")
            if features is not None and (not isinstance(features, list) or len(features) != self.model.size
                    or any(type(v) not in (int, float) or not math.isfinite(v) or not -1 <= v <= 1 for v in features)):
                raise ValueError("invalid features")
            self.seq = seq
            self.probability = 0.0 if features is None else self.model.probability(features)
            self.until = now + min(.25, .35 - age)
            return self.reply(now)
        except (ValueError, TypeError, AttributeError):
            self.stop()
            raise

    def command(self, now, locked):
        # Short repeated pulses give marker=0 time to re-arm its 2.5 s cap.
        indicator = locked and now < self.until and self.probability >= self.model.threshold and now % 2.5 < 1.5
        return Command(0, 0, indicator)


class Server(ThreadingHTTPServer):
    daemon_threads = True

    def __init__(self, address, model, bot_id, bot_address, token):
        if len(token) < 24:
            raise ValueError("missing private token")
        super().__init__(address, Handler)
        self.token, self.preview = token, Preview(model)
        self.bot_id, self.bot_address = bot_id, bot_address
        self.lock, self.done = threading.RLock(), threading.Event()
        self.telemetry, self.telemetry_at, self.last_camera = None, 0.0, time.monotonic()
        self.udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.threads = [threading.Thread(target=f, daemon=True) for f in (self.transmit, self.poll)]
        for t in self.threads:
            t.start()

    def status(self, now):
        return {"robot": self.telemetry, "robot_reply_age_s": round(now - self.telemetry_at, 2),
                "robot_online": now - self.telemetry_at < 2, "camera_lease_valid": now < self.preview.until}

    def poll(self):
        while not self.done.is_set():
            try:
                data = query_bot(self.bot_address, self.bot_id)
                with self.lock:
                    self.telemetry, self.telemetry_at = data, time.monotonic()
            except (OSError, ValueError):
                pass
            self.done.wait(.5)

    def transmit(self):
        while not self.done.is_set():
            now = time.monotonic()
            with self.lock:
                locked = (self.telemetry is not None and self.telemetry.get("motors_enabled") is False
                          and now - self.telemetry_at < 2)
                cmd = self.preview.command(now, locked)
                self.udp.sendto(encode(self.bot_id, int(now * 20) % 1000000000, cmd), self.bot_address)
            self.done.wait(.05)

    def server_close(self):
        self.done.set()
        for thread in self.threads:
            thread.join(timeout=1)
        for _ in range(3):
            self.udp.sendto(encode(self.bot_id, 0, Command()), self.bot_address)
        self.udp.close()
        super().server_close()


class Handler(AuthHandler):
    def do_GET(self):
        if self.authenticated():
            with self.server.lock:
                self.reply(200, self.server.status(time.monotonic()))

    def do_POST(self):
        self.connection.settimeout(1)
        if not self.authenticated():
            return
        try:
            length = int(self.headers.get("Content-Length", 0))
            if not 0 < length <= 2048:
                raise ValueError("invalid body length")
            data = json.loads(self.rfile.read(length))
            with self.server.lock:
                now = time.monotonic()
                if self.path == "/session":
                    result = self.server.preview.new_session(now)
                elif self.path == "/classify":
                    result = self.server.preview.classify(data, now)
                    self.server.last_camera = now
                elif self.path == "/stop":
                    self.server.preview.stop()
                    result = {}
                else:
                    raise ValueError("unknown endpoint")
                self.reply(200, {**result, **self.server.status(now)})
        except (ValueError, TypeError, OSError) as exc:
            with self.server.lock:
                self.server.preview.stop()
            self.reply(400, {"error": str(exc)})


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--config", default="config.toml")
    ap.add_argument("--bot", type=int, default=2)
    ap.add_argument("--port", type=int, default=8766)
    a = ap.parse_args()
    address = bots_from(load_config(a.config))[a.bot]
    # This demo is allowed to start only against confirmed motor-disabled firmware.
    status = None
    for _ in range(12):
        try:
            status = query_bot(address, a.bot)
            break
        except (OSError, ValueError):
            time.sleep(.25)
    if status is None or status.get("motors_enabled") is not False:
        raise RuntimeError("cannot confirm robot motor lock; demo refused")
    model = TargetModel.load("models/target.json")
    server = Server(("0.0.0.0", a.port), model, a.bot, address, os.environ.get("SWARM_TOKEN", ""))
    active = subprocess.run(["systemctl", "--user", "is-active", "--quiet", "pi-swarm.service"]).returncode == 0
    if active:
        subprocess.run(["systemctl", "--user", "stop", "pi-swarm.service"], check=True)
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))
    print("Camera LED demo ready; wheel commands are always zero", flush=True)
    try:
        server.timeout = .25
        while time.monotonic() - server.last_camera < 60:
            server.handle_request()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        if active:
            subprocess.run(["systemctl", "--user", "start", "pi-swarm.service"], check=False)


if __name__ == "__main__":
    main()
