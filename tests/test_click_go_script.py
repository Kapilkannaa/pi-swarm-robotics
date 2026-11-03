"""scripts/click_go.py end to end: simulator mode, and real mode against a looping video file + loopback UDP 'bot'."""
import math
import re
import socket
import subprocess
import sys
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


def _config(tmp_path, ip="127.0.0.1", port=4210, url="x"):
    (tmp_path / "c.toml").write_text(f'''
[arena]
width_m = 1.0
height_m = 0.75
[camera]
url = '{url}'
[[bots]]
id = 2
ip = "{ip}"
port = {port}
[control]
latency_s = 0.4
''')


def test_dry_run_drives_the_simulated_bot_to_the_scripted_goals(tmp_path):
    _config(tmp_path)
    r = subprocess.run([sys.executable, str(ROOT / "scripts" / "click_go.py"), "--config", str(tmp_path / "c.toml"), "--dry-run",
                        "--no-display", "--goals", "0.75,0.5;0.3,0.55", "--max-seconds", "200"],
                       capture_output=True, text=True, cwd=ROOT, timeout=280)
    assert r.returncode == 0 and "Traceback" not in r.stderr, r.stderr
    err_mm = float(re.search(r"error (\d+) mm", r.stdout).group(1))
    assert err_mm < 80, r.stdout


def test_scripted_goals_are_refused_for_a_real_bot(tmp_path):
    _config(tmp_path, ip="10.0.0.9")
    r = subprocess.run([sys.executable, str(ROOT / "scripts" / "click_go.py"), "--config", str(tmp_path / "c.toml"),
                        "--no-display", "--goals", "0.5,0.5"], capture_output=True, text=True, cwd=ROOT, timeout=30)
    assert r.returncode != 0 and "refused" in (r.stdout + r.stderr)


def test_real_mode_sends_nothing_but_stops_until_a_goal_exists_then_drives_and_stops_at_exit(tmp_path):
    cam = SyntheticCamera(ARENA, px_per_m=600, margin_px=90)
    frame = cam.render_leds({2: Pose(0.4, 0.3, math.pi / 2)}, offset=0.05)
    vid = str(tmp_path / "led.avi")
    w, h = cam.size
    out = cv2.VideoWriter(vid, cv2.VideoWriter_fourcc(*"MJPG"), 20, (w, h))
    for _ in range(60):
        out.write(frame)
    out.release()
    tr = ArenaTracker(ARENA)
    tr.calibrate_from_points([tuple(p) for p in cam.image_corners])
    tr.save(tmp_path / "cal.json")
    rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx.bind(("127.0.0.1", 0))
    rx.settimeout(0.3)
    _config(tmp_path, port=rx.getsockname()[1], url=vid)

    def run(extra, seconds):
        p = subprocess.Popen([sys.executable, str(ROOT / "scripts" / "click_go.py"), "--config", str(tmp_path / "c.toml"),
                              "--cal", str(tmp_path / "cal.json"), "--no-display", "--loop-video", "--max-seconds", str(seconds),
                              *extra], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, cwd=ROOT)
        got, import_time = [], __import__("time").time
        t0 = import_time()
        while import_time() - t0 < seconds + 3:
            try:
                got.append(protocol.parse(rx.recv(64), expected_id=2))
            except socket.timeout:
                pass
        return got, p.communicate(timeout=20)[0], p.returncode

    got, out, rc = run([], 4)                                              # no goal: idle
    assert rc == 0 and "Traceback" not in out, out
    assert got and all(x.cmd.left == 0 and x.cmd.right == 0 for x in got if x)

    # drain, then give it a goal (allowed here because the "bot" is on loopback): it must move, and stop at exit
    rx.settimeout(0.05)
    try:
        while True:
            rx.recv(64)
    except socket.timeout:
        pass
    rx.settimeout(0.3)
    got, out, rc = run(["--goals", "0.8,0.6"], 5)
    assert rc == 0 and "Traceback" not in out, out
    pk = [x for x in got if x is not None]
    assert any(x.cmd.left or x.cmd.right for x in pk)                      # it drove
    assert all(x.cmd.left == 0 and x.cmd.right == 0 and not x.cmd.laser for x in pk[-3:])   # and stopped at exit
