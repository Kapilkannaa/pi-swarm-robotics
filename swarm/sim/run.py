"""Run simulated trials: `python -m swarm.sim.run --trials 20 --bots 2 --csv logs/sim.csv`."""
from __future__ import annotations

import argparse
import csv
import statistics
from dataclasses import asdict, dataclass

from .. import protocol
from ..planner import Arena
from ..protocol import Command
from ..swarm import Phase, Swarm, SwarmParams
from .world import SimParams, World, make_world

DT = 0.05  # 20 Hz control tick, same rate planned for the real UDP loop


@dataclass
class TrialResult:
    seed: int
    n_bots: int
    success: bool
    t_found: float | None
    t_mark: float | None
    collisions: int
    min_separation: float
    max_laser_s: float
    final_phase: str


def run_trial(seed: int, n_bots: int = 2, timeout_s: float = 240.0,
              sim: SimParams | None = None, world: World | None = None,
              swarm_params: SwarmParams | None = None, arena: Arena | None = None) -> TrialResult:
    w = world or make_world(seed, n_bots, arena, sim)
    sp = swarm_params or SwarmParams(detect_radius=w.p.detect_radius)
    swarm = Swarm(w.arena, sorted(w.bots), sp)
    seq = 0
    t_mark: float | None = None
    while w.t < timeout_s and swarm.phase is not Phase.DONE:
        cmds = swarm.update(w.observe(), w.detect(), w.t)
        seq += 1
        for bot_id, cmd in cmds.items():
            w.send(bot_id, protocol.encode(bot_id, seq, cmd))
        w.step(DT)
        if w.marked and t_mark is None:
            t_mark = w.t
    # Idle a moment so the final laser-off command and any in-flight packets land.
    for _ in range(int(1.0 / DT)):
        for bot_id in w.bots:
            seq += 1
            w.send(bot_id, protocol.encode(bot_id, seq, Command()))
        w.step(DT)
    return TrialResult(seed, n_bots, w.marked, w.t_found, t_mark, w.collisions,
                       round(w.min_separation, 3), round(w.max_laser_s, 2), swarm.phase.value)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--trials", type=int, default=20)
    ap.add_argument("--bots", type=int, default=2)
    ap.add_argument("--seed", type=int, default=0, help="first seed")
    ap.add_argument("--csv", help="write per-trial results here")
    a = ap.parse_args()
    results = [run_trial(a.seed + k, a.bots) for k in range(a.trials)]
    ok = [r for r in results if r.success]
    print(f"{len(ok)}/{len(results)} marked ({100 * len(ok) / len(results):.0f}%)")
    if ok:
        t = [r.t_mark for r in ok if r.t_mark is not None]
        print(f"time-to-mark: mean {statistics.mean(t):.1f}s  median {statistics.median(t):.1f}s  max {max(t):.1f}s")
    print(f"collisions: {sum(r.collisions for r in results)}  max laser run: {max(r.max_laser_s for r in results)}s")
    for r in results:
        if not r.success:
            print("FAIL", r)
    if a.csv:
        with open(a.csv, "w", newline="") as f:
            wr = csv.DictWriter(f, fieldnames=list(asdict(results[0])))
            wr.writeheader()
            wr.writerows(asdict(r) for r in results)


if __name__ == "__main__":
    main()
