"""run_live.py in REAL mode (video-file camera + loopback UDP 'bots'): proves the camera -> planner -> UDP plumbing,
that bots are commanded while running, and that a stop is always sent on exit."""
import math
import socket
import subprocess
import sys
import time
from pathlib import Path

import pytest

cv2 = pytest.importorskip("cv2")
np = pytest.importorskip("numpy")

from swarm import protocol  # noqa: E402
from swarm.geometry import Pose  # noqa: E402
from swarm.planner import Arena  # noqa: E402
from swarm.vision.synth import SyntheticCamera  # noqa: E402
from swarm.vision.tracker import ArenaTracker  # noqa: E402

ROOT = Path(__file__).resolve().parent.parent
ARENA = Arena(1.0, 0.75)


def _setup(tmp_path):
    cam = SyntheticCamera(ARENA)
    frame = cam.render({1: Pose(0.3, 0.2, math.pi / 2), 2: Pose(0.7, 0.2, math.pi / 2)}, target=(0.5, 0.55))
    vid = str(tmp_path / "arena.avi")
    w, h = cam.size
    out = cv2.VideoWriter(vid, cv2.VideoWriter_fourcc(*"MJPG"), 20, (w, h))
    for _ in range(120):
        out.write(frame)
    out.release()
    tr = ArenaTracker(ARENA)
    tr.calibrate_from_points([tuple(p) for p in cam.image_corners])
    tr.save(tmp_path / "cal.json")
    rx = {i: socket.socket(socket.AF_INET, socket.SOCK_DGRAM) for i in (1, 2)}
    for s in rx.values():
        s.bind(("127.0.0.1", 0))
        s.settimeout(0.3)
    (tmp_path / "c.toml").write_text(f'''
[arena]
width_m = 1.0
height_m = 0.75
[camera]
url = '{vid}'
[[bots]]
id = 1
ip = "127.0.0.1"
port = {rx[1].getsockname()[1]}
[[bots]]
id = 2
ip = "127.0.0.1"
port = {rx[2].getsockname()[1]}
[control]
latency_s = 0.4
''')
    return rx


def _run(tmp_path, extra, rx, seconds):
    p = subprocess.Popen([sys.executable, str(ROOT / "scripts" / "run_live.py"), "--config", str(tmp_path / "c.toml"),
                          "--cal", str(tmp_path / "cal.json"), "--no-display", *extra], stdout=subprocess.PIPE,
                         stderr=subprocess.STDOUT, text=True, cwd=ROOT)
    got = {1: [], 2: []}
    t0 = time.time()
    while time.time() - t0 < seconds:
        for i, s in rx.items():
            try:
                got[i].append(protocol.parse(s.recv(64), expected_id=i))
            except socket.timeout:
                pass
    out = p.communicate(timeout=20)[0]
    return got, out, p.returncode


def test_real_mode_commands_bots_then_stops(tmp_path):
    rx = _setup(tmp_path)
    got, out, rc = _run(tmp_path, ["--autostart", "--loop-video", "--max-seconds", "5"], rx, 8)
    assert rc == 0 and "Traceback" not in out, out
    for i in (1, 2):
        pk = [x for x in got[i] if x is not None]
        assert len(pk) > 60                                            # ~20 Hz while running
        assert any(x.cmd.left or x.cmd.right for x in pk)              # the planner moved it
        assert all(x.cmd.left == 0 and x.cmd.right == 0 and not x.cmd.laser for x in pk[-3:])  # stop on exit


def test_real_mode_does_not_move_bots_until_started(tmp_path):
    rx = _setup(tmp_path)
    got, out, rc = _run(tmp_path, ["--loop-video", "--max-seconds", "3"], rx, 5)   # no --autostart: stays paused
    assert rc == 0 and "Traceback" not in out, out
    for i in (1, 2):
        pk = [x for x in got[i] if x is not None]
        assert pk and all(x.cmd.left == 0 and x.cmd.right == 0 and not x.cmd.laser for x in pk)


def test_autostart_is_refused_for_non_loopback_bots(tmp_path):
    rx = _setup(tmp_path)
    cfg = (tmp_path / "c.toml").read_text().replace('ip = "127.0.0.1"', 'ip = "10.0.0.5"', 1)
    (tmp_path / "c.toml").write_text(cfg)
    r = subprocess.run([sys.executable, str(ROOT / "scripts" / "run_live.py"), "--config", str(tmp_path / "c.toml"),
                        "--cal", str(tmp_path / "cal.json"), "--no-display", "--autostart"],
                       capture_output=True, text=True, cwd=ROOT, timeout=30)
    assert r.returncode != 0 and "refused" in (r.stdout + r.stderr)


def test_bots_option_selects_a_subset_and_rejects_unknown_ids(tmp_path):
    rx = _setup(tmp_path)
    got, out, rc = _run(tmp_path, ["--bots", "1", "--autostart", "--loop-video", "--max-seconds", "4"], rx, 6)
    assert rc == 0 and "Traceback" not in out, out
    assert any(x is not None for x in got[1])                 # bot 1 is commanded
    assert not any(x is not None for x in got[2])             # bot 2 receives nothing at all
    r = subprocess.run([sys.executable, str(ROOT / "scripts" / "run_live.py"), "--config", str(tmp_path / "c.toml"),
                        "--cal", str(tmp_path / "cal.json"), "--no-display", "--bots", "7"],
                       capture_output=True, text=True, cwd=ROOT, timeout=30)
    assert r.returncode != 0 and "no such bot" in (r.stdout + r.stderr)
