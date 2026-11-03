import json
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import pytest

from swarm.offload import evaluate
from swarm.offload_client import BackgroundOffload


def request():
    return {"job_id": "test", "arena": [1, .75], "pose": [.2, .2, 0], "goal": [.7, .5], "trials": 2}


def test_rollouts_are_bounded_reproducible_and_never_motor_commands():
    a, b = evaluate(request()), evaluate(request())
    assert a['profiles'] == b['profiles']
    assert len(a['profiles']) == 3 and 0 < a['work_steps'] <= 7200
    assert not a['applied_to_motors'] and a['goal'] == [.7, .5]
    assert 'assumed' in a['physics']


@pytest.mark.parametrize('change', [{'trials': 1000}, {'goal': [float('nan'), 0]}, {'goal': [2, 2]}, {'arena': [10, 10]}, {'pose': [1]}])
def test_invalid_jobs_are_rejected(change):
    with pytest.raises(ValueError): evaluate({**request(), **change})


def test_offline_worker_cannot_block_local_loop():
    client = BackgroundOffload('http://127.0.0.1:1', 'test-token', timeout=.1)
    try:
        t = time.monotonic()
        client.submit(request())
        for _ in range(100): client.snapshot()
        assert time.monotonic() - t < .1
        deadline = time.monotonic() + 2
        while 'computing' in client.snapshot()['state'] and time.monotonic() < deadline: time.sleep(.01)
        assert 'local control continues' in client.snapshot()['state']
    finally: client.close()


def test_late_results_for_cancelled_goal_are_discarded():
    entered, release = threading.Event(), threading.Event()
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *_): pass
        def do_POST(self):
            data = json.loads(self.rfile.read(int(self.headers['Content-Length'])))
            entered.set(); release.wait(2)
            raw = json.dumps({'job_id': data['job_id'], 'goal': data['goal']}).encode()
            self.send_response(200); self.end_headers(); self.wfile.write(raw)
    server = ThreadingHTTPServer(('127.0.0.1', 0), Handler)
    thread = threading.Thread(target=server.serve_forever, daemon=True); thread.start()
    client = BackgroundOffload(f'http://127.0.0.1:{server.server_port}', 'test', timeout=3)
    try:
        client.submit(request()); assert entered.wait(2)
        client.cancel(); release.set(); time.sleep(.15)
        assert client.snapshot()['result'] is None and client.snapshot()['job_id'] is None
    finally:
        release.set(); client.close(); server.shutdown(); server.server_close()
