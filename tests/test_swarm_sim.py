import math
import random

import pytest

from swarm.control import DriveController
from swarm.geometry import Pose, angle_diff
from swarm.planner import Arena, avoid, converge_slots, lawnmower, split_sectors, assign_sectors
from swarm.protocol import Command
from swarm.sim.firmware_model import LASER_MAX_MS
from swarm.sim.run import DT, run_trial
from swarm.sim.world import SimParams, make_world
from swarm.swarm import Phase, Swarm


# ------------------------------------------------------------------ planner
def test_sectors_tile_arena():
    a = Arena(1.2, 0.9)
    for n in (1, 2, 3, 5):
        s = split_sectors(a, n)
        assert sum((x.x1 - x.x0) * (x.y1 - x.y0) for x in s) == pytest.approx(a.width * a.height)


def test_lawnmower_covers_sector_with_bounded_gaps():
    s = split_sectors(Arena(1.2, 0.9), 2)[0]
    pts = lawnmower(s, spacing=0.4, margin=0.1)
    xs = sorted({round(p[0], 6) for p in pts})
    assert all(b - a <= 0.4 + 1e-9 for a, b in zip(xs, xs[1:]))
    assert all(s.x0 <= x <= s.x1 and s.y0 <= y <= s.y1 for x, y in pts)


def test_assignment_prefers_nearest_sector():
    sectors = split_sectors(Arena(1.2, 0.9), 2)
    # Bot 1 starts on the right, bot 2 on the left: assignment must not send them across.
    got = assign_sectors({1: Pose(1.0, 0.1, 0), 2: Pose(0.2, 0.1, 0)}, sectors)
    assert got[1] == sectors[1] and got[2] == sectors[0]


def test_converge_slots_distinct_and_on_ring():
    target = (0.6, 0.5)
    poses = {i: Pose(0.1 * i, 0.1, 0) for i in (1, 2, 3)}
    slots = converge_slots(target, poses, 0.2)
    pts = list(slots.values())
    assert all(math.dist(p, target) == pytest.approx(0.2) for p in pts)
    assert min(math.dist(a, b) for i, a in enumerate(pts) for b in pts[i + 1:]) > 0.2


def test_avoid_sidesteps_blocker_and_ignores_clear_line():
    goal = (1.0, 0.0)
    assert avoid((0, 0), goal, [(0.5, 0.5)], 0.2) == goal
    wp = avoid((0, 0), goal, [(0.5, 0.02)], 0.2)
    assert wp != goal and abs(wp[1]) > 0.2


# ------------------------------------------------------------------ control
def test_drift_trim_learns_motor_mismatch():
    """A bot whose right motor is 12% weak must still reach the goal, and learn a trim."""
    from swarm.sim.world import SimBot, World
    from swarm.sim.firmware_model import FirmwareModel
    w = make_world(1, 1, params=SimParams(gain_spread=0, packet_loss=0, pos_noise=0, heading_noise=0))
    w.bots[1] = SimBot(1, Pose(0.1, 0.1, 0.0), 1.0, 0.88, FirmwareModel(1))
    ctl, goal, seq = DriveController(), (1.1, 0.1), 0
    for _ in range(int(40 / DT)):
        cmd, done = ctl.go_to(w.observe()[1], goal, DT)
        from swarm import protocol
        seq += 1
        w.send(1, protocol.encode(1, seq, cmd))
        w.step(DT)
        if done:
            break
    p = w.bots[1].pose
    assert done and math.dist((p.x, p.y), goal) < 0.08
    assert ctl.trim > 0.5  # weak right wheel -> learned positive (steer-left) offset


# -------------------------------------------------------------- whole system
def test_two_bot_trials_succeed_safely():
    results = [run_trial(s, 2) for s in range(30)]
    assert sum(r.success for r in results) >= 29
    assert sum(r.collisions for r in results) == 0
    assert max(r.max_laser_s for r in results) <= LASER_MAX_MS / 1000 + DT


def test_three_bot_scaling():
    results = [run_trial(100 + s, 3) for s in range(15)]
    assert sum(r.success for r in results) >= 14
    assert sum(r.collisions for r in results) == 0


def test_single_bot_still_works():
    assert sum(run_trial(200 + s, 1).success for s in range(10)) >= 9


def test_survives_heavy_packet_loss():
    sim = SimParams(packet_loss=0.25)
    assert sum(run_trial(300 + s, 2, sim=sim).success for s in range(10)) >= 8


def test_lost_bot_is_stopped_not_driven_blind():
    w = make_world(5, 2)
    sw = Swarm(w.arena, [1, 2])
    poses = w.observe()
    cmds = sw.update({1: poses[1]}, None, 0.0)
    assert cmds[2] == Command()


def test_laser_only_from_one_bot_and_only_in_mark_phase():
    w = make_world(7, 2)
    sw = Swarm(w.arena, [1, 2])
    from swarm import protocol
    seq = 0
    while w.t < 120 and sw.phase is not Phase.DONE:
        cmds = sw.update(w.observe(), w.detect(), w.t)
        assert sum(c.laser for c in cmds.values()) <= 1
        if any(c.laser for c in cmds.values()):
            assert sw.phase is Phase.MARK
        seq += 1
        for i, c in cmds.items():
            w.send(i, protocol.encode(i, seq, c))
        w.step(DT)
    assert sw.phase is Phase.DONE and sw.shooter is not None


def test_avoid_backs_away_from_a_touching_neighbour_before_routing_around():
    # Neighbour 10 cm away, goal straight through it: no room for a side-step, so move away first.
    wp = avoid((0.0, 0.0), (1.0, 0.0), [(0.10, 0.0)], clearance=0.20, sep=0.16)
    assert wp[0] < 0.0                                               # steers AWAY from the neighbour
    # Goal points away from the neighbour: no need to back off, just go.
    assert avoid((0.0, 0.0), (-1.0, 0.0), [(0.10, 0.0)], clearance=0.20, sep=0.16) == (-1.0, 0.0)
    # Neighbour comfortably far: unchanged behaviour (goal returned when the line is clear).
    assert avoid((0.0, 0.0), (1.0, 0.0), [(0.0, 0.5)], clearance=0.20, sep=0.16) == (1.0, 0.0)


def test_four_bots_do_not_touch_each_other():
    from swarm.planner import Arena as A
    res = [run_trial(500 + s, 4, arena=A(1.0, 0.75), sim=SimParams(detect_radius=0.2)) for s in range(15)]
    assert sum(r.collisions for r in res) == 0 and sum(r.success for r in res) >= 14
