"""Compile firmware/bot/logic.h on the laptop and fuzz it against the Python firmware model.

If these agree, the C++ parser and safety rules behave like the spec that the rest of the
simulation was tested against. (This does NOT test pins, PWM or Wi-Fi: only hardware can.)
"""
import random
import os
import shutil
import subprocess
from pathlib import Path

import pytest

from swarm.sim.firmware_model import FirmwareModel

ROOT = Path(__file__).resolve().parent.parent
BOT_ID = 1

pytestmark = pytest.mark.skipif(shutil.which("g++") is None, reason="needs g++")


@pytest.fixture(scope="module")
def harness(tmp_path_factory) -> Path:
    exe = tmp_path_factory.mktemp("fw") / ("fw_harness.exe" if os.name == "nt" else "fw_harness")
    subprocess.run(["g++", "-std=c++17", "-Wall", "-Wextra", "-Werror", f"-I{ROOT / 'firmware/bot'}",
                    str(ROOT / "tests/fw_harness.cpp"), "-o", str(exe)], check=True)
    return exe


def run_cpp(exe: Path, events: list[tuple]) -> list[tuple[int, int, int]]:
    lines = []
    for ev in events:
        if ev[0] == "rx":
            lines.append(f"rx {ev[1]} {ev[2].hex() or '-'}")
        else:
            lines.append(f"tick {ev[1]}")
    out = subprocess.run([str(exe), str(BOT_ID)], input="\n".join(lines) + "\n", capture_output=True,
                         text=True, check=True).stdout.split("\n")
    return [tuple(int(x) for x in ln.split()) for ln in out if ln]


def run_py(events: list[tuple]) -> list[tuple[int, int, int]]:
    fw, res = FirmwareModel(BOT_ID), []
    for ev in events:
        if ev[0] == "rx":
            fw.on_packet(ev[2], ev[1])
        else:
            o = fw.tick(ev[1])
            res.append((round(o.left * 1000), round(o.right * 1000), int(o.laser)))
    return res


GOOD = [b"1,5,50,50,0\n", b"1,6,-100,100,0\n", b"1,7,100,100,1\n", b"1,8,0,0,1\n", b"1,9,-33,77,0\r\n",
        b"1,9,12,-12,1", b"1,10,999,-999,7\n", b"1,11,-5,5,-3\n"]
BAD = [b"", b"x", b"1,2,3\n", b"1,2,3,4,5,6\n", b"2,1,50,50,0\n", b"1,1, 5,5,0\n", b"1,1,+5,5,0\n",
       b"1,1,5.0,5,0\n", b"1,1,5,5,0\n\n", b"1,1,5,5,0,\n", b",1,5,5,0\n", b"1,1,--5,5,0\n",
       b"1_0,1,5,5,0\n", b"1,1,5,5,0\n" + b"x" * 60, b"1,1,99999999999999999999,5,1\n", b"1,1,-,5,0\n",
       b"1,1,5,5,0\r", b"1,1,5,5,0\r\r\n", b"1\x00,1,5,5,0\n"]


def fuzz_events(seed: int) -> list[tuple]:
    """Random packet/tick stream. Time only moves forward, like millis()."""
    r, t, ev = random.Random(seed), 1000, []
    for _ in range(400):
        t += r.choice([1, 10, 40, 50, 200, 499, 500, 501, 800, 2600, 3000])
        if r.random() < 0.55:
            data = r.choice(GOOD) if r.random() < 0.7 else r.choice(BAD)
            if r.random() < 0.1:
                data = bytes(r.randrange(256) for _ in range(r.randrange(0, 20)))
            ev.append(("rx", t, data))
        t += r.choice([0, 1, 49, 50])
        ev.append(("tick", t))
    return ev


@pytest.mark.parametrize("seed", range(30))
def test_cpp_matches_python_model(harness, seed):
    ev = fuzz_events(seed)
    assert run_cpp(harness, ev) == run_py(ev)


def test_scripted_safety_scenario(harness):
    keep = [("rx", t, b"1,1,100,-100,1\n") for t in range(500, 2600, 400)]  # planner keeps asking
    ev = [("tick", 0), ("rx", 100, b"1,1,100,-100,1\n"), ("tick", 100), *keep,
          ("tick", 2599), ("rx", 2600, b"1,1,100,-100,1\n"), ("tick", 2601),
          ("rx", 2700, b"1,2,100,100,1\n"), ("tick", 2700),   # still latched: no re-fire without marker=0
          ("rx", 2800, b"1,3,0,0,0\n"), ("rx", 2850, b"1,4,0,0,1\n"), ("tick", 2850),
          ("tick", 2850 + 501)]                                # watchdog
    got = run_cpp(harness, ev)
    assert got[0] == (0, 0, 0)
    assert got[1] == (720, -720, 1)
    assert got[2] == (720, -720, 1)
    assert got[3] == (720, -720, 0)   # 2.5 s cap hit: motors keep going, marker off
    assert got[4] == (720, 720, 0)
    assert got[5] == (0, 0, 1)        # re-armed by marker=0, fresh marker=1 fires
    assert got[6] == (0, 0, 0)        # failsafe
