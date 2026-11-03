"""Camera-side authenticated observation transport. Never sends UDP motor commands."""
from __future__ import annotations

import json
import urllib.request
from urllib.parse import urlsplit


class PiClient:
    def __init__(self, url: str, token: str):
        p = urlsplit(url)
        if p.scheme not in ("http", "https") or not p.hostname or p.username or p.password:
            raise ValueError("expected a Pi HTTP URL without credentials")
        self.url, self.token, self.seq = url.rstrip("/"), token, 0
        # Bypass system proxies for the local Pi (including laptop VPN proxy settings).
        self.http = urllib.request.build_opener(urllib.request.ProxyHandler({}))
        self.state = self.request("/session", {})

    def request(self, path, data=None):
        req = urllib.request.Request(self.url + path,
            data=None if data is None else json.dumps(data, allow_nan=False).encode(),
            headers={"Authorization": "Bearer " + self.token, "Content-Type": "application/json"})
        with self.http.open(req, timeout=0.4) as response:
            return json.load(response)

    def observe(self, **data):
        self.seq += 1
        self.state = self.request("/observe", {**data, "seq": self.seq,
            "session": self.state["session"], "ticket": self.state["ticket"]})
        return self.state

    def stop(self):
        return self.request("/stop", {})
