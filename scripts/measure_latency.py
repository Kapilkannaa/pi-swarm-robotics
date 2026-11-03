"""Measure the real loop latency and print the `latency_s` to put in config.toml.

    python scripts/measure_latency.py --bot 1

Put ONE bot in the arena (calibrated camera, marker visible, plenty of room: it only spins on the spot at low power).
It does 5 short spins and reports the median delay between the command and the first visible turn.
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import cv2

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
from swarm.config import arena_from, bots_from, load_config  # noqa: E402
from swarm.io import LatestFrame, UdpLink  # noqa: E402
from swarm.latency import measure_once, summarise  # noqa: E402
from swarm.protocol import Command  # noqa: E402
from swarm.vision.tracker import ArenaTracker  # noqa: E402


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--bot", type=int, default=1)
    ap.add_argument("--power", type=int, default=55, help="spin command (-100..100); use what reliably turns the bot")
    ap.add_argument("--runs", type=int, default=5)
    ap.add_argument("--config", default=str(ROOT / "config.toml"))
    ap.add_argument("--cal", default=str(ROOT / "logs" / "calibration.json"))
    a = ap.parse_args()
    cfg = load_config(a.config)
    tracker = ArenaTracker(arena_from(cfg))
    if not Path(a.cal).exists():
        sys.exit("no calibration: run scripts/run_live.py --recalibrate first")
    tracker.load(a.cal)
    stream, link = LatestFrame(cfg["camera"]["url"]), UdpLink(bots_from(cfg))

    def get_theta():
        f, ts = stream.read()
        if f is None or time.time() - ts > 1.0:
            return None
        p = tracker.poses(f).get(a.bot)
        return None if p is None else p.theta

    samples: list[float] = []
    try:
        for k in range(a.runs):
            sign = 1 if k % 2 == 0 else -1  # alternate directions so the bot ends up where it started
            r = measure_once(lambda: link.send({a.bot: Command(-sign * a.power, sign * a.power)}),
                             lambda: link.stop_all(), get_theta, time.time, time.sleep)
            print(f"run {k + 1}: " + ("no motion seen (marker not visible, or power too low?)" if r is None else f"{r:.2f} s"))
            if r is not None:
                samples.append(r)
            time.sleep(1.5)  # let it stop and the video catch up before the next run
    finally:
        link.stop_all()
        stream.close()
    if len(samples) < 3:
        sys.exit("too few valid runs: check the marker is visible and the bot turns")
    s = summarise(samples)
    print(f"\nmedian {s['median']:.2f} s, worst {s['worst']:.2f} s")
    print(f"=> put   latency_s = {s['recommended_latency_s']}   in the [control] section of config.toml")


if __name__ == "__main__":
    main()
