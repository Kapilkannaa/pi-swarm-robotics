import socket
import time

import pytest

from swarm import protocol
from swarm.latency import measure_once, summarise
from swarm.protocol import Command

cv2 = pytest.importorskip("cv2")
np = pytest.importorskip("numpy")

from swarm.io import LatestFrame, UdpLink  # noqa: E402


def test_udp_link_sends_valid_packets_to_the_right_bot_and_stop_repeats():
    rx = {i: socket.socket(socket.AF_INET, socket.SOCK_DGRAM) for i in (1, 2)}
    for s in rx.values():
        s.bind(("127.0.0.1", 0))
        s.settimeout(1.0)
    link = UdpLink({i: ("127.0.0.1", s.getsockname()[1]) for i, s in rx.items()})
    link.send({1: Command(40, -40, False), 2: Command(0, 0, True)})
    p1 = protocol.parse(rx[1].recv(64), expected_id=1)
    p2 = protocol.parse(rx[2].recv(64), expected_id=2)
    assert p1 is not None and p1.cmd == Command(40, -40, False)
    assert p2 is not None and p2.cmd == Command(0, 0, True)
    assert protocol.parse(b"1,1,0,0,0", expected_id=2) is None  # a packet for bot 1 is rejected by bot 2
    link.stop_all()
    stops = [protocol.parse(rx[1].recv(64), expected_id=1) for _ in range(3)]
    assert all(s is not None and s.cmd == Command() for s in stops)
    link.send({9: Command(1, 1)})  # unknown bot id: ignored, no crash


def test_latest_frame_reads_a_video_file_and_returns_the_newest(tmp_path):
    path = str(tmp_path / "clip.avi")
    out = cv2.VideoWriter(path, cv2.VideoWriter_fourcc(*"MJPG"), 30, (64, 48))
    for k in range(30):
        out.write(np.full((48, 64, 3), k * 8, np.uint8))
    out.release()
    src = LatestFrame(path)
    try:
        deadline = time.time() + 3
        while src.count < 30 and time.time() < deadline:
            time.sleep(0.02)
        frame, stamp = src.read()
        assert frame is not None and frame.shape == (48, 64, 3) and stamp > 0
        assert src.count >= 25
    finally:
        src.close()


def test_latest_frame_reports_unopenable_source():
    with pytest.raises(OSError):
        LatestFrame("/nonexistent/video.mp4")


class _FakeLoop:
    """A bot that starts turning `delay` seconds after the first rotate command."""

    def __init__(self, delay):
        self.t, self.delay, self.first, self.theta = 0.0, delay, None, 0.0

    def clock(self): return self.t

    def sleep(self, dt):
        self.t += dt
        if self.first is not None and self.t - self.first > self.delay:
            self.theta += 1.0 * dt  # 1 rad/s once it moves

    def rotate(self):
        if self.first is None:
            self.first = self.t

    def stop(self): pass
    def get_theta(self): return self.theta


@pytest.mark.parametrize("delay", [0.1, 0.35, 0.6])
def test_measure_once_recovers_the_true_delay(delay):
    f = _FakeLoop(delay)
    got = measure_once(f.rotate, f.stop, f.get_theta, f.clock, f.sleep, threshold=0.04)
    assert got is not None and delay < got < delay + 0.2     # delay + time to accumulate the threshold rotation


def test_measure_once_returns_none_when_nothing_moves():
    f = _FakeLoop(delay=99)
    assert measure_once(f.rotate, f.stop, f.get_theta, f.clock, f.sleep, timeout_s=1.0) is None


def test_summarise_recommends_median_plus_margin():
    s = summarise([0.30, 0.34, 0.50, 0.32, 0.31])
    assert s["median"] == pytest.approx(0.32) and s["recommended_latency_s"] == 0.42 and s["worst"] == 0.50
