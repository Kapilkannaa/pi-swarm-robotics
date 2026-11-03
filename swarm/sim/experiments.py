"""Simulation experiments for the report. Every row is N seeded trials, so results are reproducible.

These are SIMULATION numbers with the simulator's placeholder physics (see SimParams). They show how the
system's design behaves under stress; they are not measurements of the real robots.
"""
from __future__ import annotations

import math
import statistics as st
from dataclasses import dataclass, replace

from .. import protocol
from ..control import BurstController, ControlParams
from ..geometry import Pose
from ..planner import Arena
from ..swarm import SwarmParams
from .firmware_model import FirmwareModel
from .run import DT, run_trial
from .world import SimBot, SimParams, make_world

ARENA = Arena(1.00, 0.75)  # the demo arena


@dataclass
class Row:
    name: str
    n: int
    success: int
    mean_t: float
    max_t: float
    collisions: int


def trials(name: str, n: int, bots: int = 2, latency_s: float = 0.0, ctl: ControlParams | None = None,
           timeout_s: float = 300.0, **sim_kw) -> Row:
    base = ctl or ControlParams()
    ctl = replace(base, latency_s=latency_s)
    res = []
    for seed in range(n):
        sim = SimParams(detect_radius=0.20, **sim_kw)
        sp = SwarmParams(detect_radius=0.20, control=ctl)
        res.append(run_trial(seed, bots, sim=sim, arena=ARENA, swarm_params=sp, timeout_s=timeout_s))
    ok = [r for r in res if r.success and r.t_mark is not None]
    ts = [r.t_mark for r in ok]
    return Row(name, n, len(ok), st.mean(ts) if ts else math.nan, max(ts) if ts else math.nan,
               sum(r.collisions for r in res))


def baseline_and_scaling(n: int) -> list[Row]:
    return [trials(f"{b} bot{'s' if b > 1 else ''}", n, bots=b) for b in (1, 2, 3, 4)]


def lag_comparison(n: int) -> list[Row]:
    rows = []
    for lag in (0.0, 0.15, 0.3, 0.5):
        rows.append(trials(f"continuous controller, camera lag {lag:g} s", n, latency_s=0.0, camera_latency_s=lag))
        rows.append(trials(f"burst controller,      camera lag {lag:g} s", n, latency_s=lag + 0.15, camera_latency_s=lag))
    return rows


def robustness(n: int) -> list[Row]:
    lag = dict(latency_s=0.4, camera_latency_s=0.3)  # burst controller, realistic phone-camera lag
    cases = [
        ("reference (lag 0.3 s)", {}),
        ("motor mismatch +/-25%", dict(gain_spread=0.25)),
        ("stiction dead-band 40 (> min_cmd 35)", dict(deadband=40)),
        ("slow bots (0.10 m/s)", dict(v_max=0.10)),
        ("fast bots (0.30 m/s)", dict(v_max=0.30)),
        ("marker dropout 10% of frames", dict(dropout_prob=0.10)),
        ("marker dropout 20% of frames", dict(dropout_prob=0.20)),
        ("packet loss 15%", dict(packet_loss=0.15)),
        ("heading noise 0.06 rad", dict(heading_noise=0.06)),
        ("all at once (mismatch 20%, drop 10%, loss 10%)", dict(gain_spread=0.20, dropout_prob=0.10, packet_loss=0.10)),
    ]
    return [trials(name, n, **lag, **kw) for name, kw in cases]


def _lone_bot_run(ctl: BurstController, gain_r: float, deadband: float, lag: float, seed: int) -> tuple[bool, float, float]:
    w = make_world(seed, 1, ARENA, SimParams(gain_spread=0, packet_loss=0.02, deadband=deadband, camera_latency_s=lag,
                                             pos_noise=0.004, heading_noise=0.015))
    w.bots[1] = SimBot(1, Pose(0.15, 0.15, 0.0), 1.0, gain_r, FirmwareModel(1))
    goals, k, seq = [(0.85, 0.2), (0.85, 0.6), (0.2, 0.6)], 0, 0
    travelled_t = None
    while w.t < 200:
        pose = w.observe()[1]
        cmd, done = ctl.go_to(pose, goals[k], DT)
        seq += 1
        w.send(1, protocol.encode(1, seq, cmd))
        w.step(DT)
        if done:
            k += 1
            if k == len(goals):
                travelled_t = w.t
                break
    p = w.bots[1].pose
    return travelled_t is not None, (travelled_t or 200.0), math.hypot(p.x - goals[-1][0], p.y - goals[-1][1])


def learning_ablation(n: int) -> list[tuple[str, int, int, float]]:
    """Burst controller with online identification vs the same controller with learning switched off.

    Every variant drives a bot with a weak right wheel (-15%), stiction above the controller's min_cmd and a 0.3 s camera
    lag through three waypoints. The variants differ in the controller's starting knowledge:
      * learning ON, poor initial guesses (turn rate 0.4 rad/s, speed 0.04 m/s: 3-4x too low)
      * learning OFF, correct initial guesses: isolates what drift/stiction learning is worth on a well-initialised bot
      * learning OFF, poor initial guesses: what happens when nobody measured the bot
    """
    variants = [
        ("learning ON,  poor initial guesses", True, 0.4, 0.04),
        ("learning OFF, correct initial guesses", False, 1.8, 0.09),
        ("learning OFF, poor initial guesses", False, 0.4, 0.04),
    ]
    out = []
    for label, learn, omega0, speed0 in variants:
        ok, times = 0, []
        for seed in range(n):
            ctl = BurstController(ControlParams(latency_s=0.4, learn=learn, omega0=omega0, speed0=speed0, min_cmd=35))
            good, t, _ = _lone_bot_run(ctl, gain_r=0.85, deadband=45, lag=0.3, seed=seed)
            ok += good
            if good:
                times.append(t)
        out.append((label, n, ok, st.mean(times) if times else math.nan))
    return out
