"""Latest-goal-only asynchronous Pi jobs. No network call on the control thread."""
import json
import threading
import urllib.request
import uuid
from urllib.parse import urlsplit


class BackgroundOffload:
    def __init__(self, url, token, timeout=20):
        if urlsplit(url).scheme not in ("http", "https"):
            raise ValueError("offload URL must be HTTP(S)")
        self.url, self.token, self.timeout = url.rstrip('/'), token, timeout
        self.lock, self.wake, self.done = threading.Lock(), threading.Event(), threading.Event()
        self.pending, self.current, self.result, self.error = None, None, None, "idle"
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def submit(self, request):
        job_id = uuid.uuid4().hex
        with self.lock:
            self.current, self.pending = job_id, {**request, "job_id": job_id}
            self.result, self.error = None, "computing in background"
        self.wake.set()
        return job_id

    def cancel(self):
        with self.lock:
            self.current, self.pending, self.result, self.error = None, None, None, "idle"

    def snapshot(self):
        with self.lock:
            return {"job_id": self.current, "state": self.error, "result": self.result}

    def _run(self):
        http = urllib.request.build_opener(urllib.request.ProxyHandler({}))
        while not self.done.is_set():
            self.wake.wait(.1)
            self.wake.clear()
            with self.lock:
                request, self.pending = self.pending, None
            if request is None: continue
            result, error = None, "offline; local control continues"
            try:
                req = urllib.request.Request(self.url + '/evaluate', data=json.dumps(request, allow_nan=False).encode(),
                    headers={"Authorization": "Bearer " + self.token, "Content-Type": "application/json"})
                with http.open(req, timeout=self.timeout) as response:
                    result = json.load(response)
                if result.get('job_id') != request['job_id'] or result.get('goal') != request['goal']:
                    raise ValueError("stale/mismatched result")
                error = "completed"
            except (OSError, ValueError, TypeError, AttributeError):
                result = None
            with self.lock:
                if self.current == request['job_id']:
                    self.result, self.error = result, error

    def close(self):
        self.cancel()
        self.done.set()
        self.wake.set()
        self.thread.join(timeout=.1)
