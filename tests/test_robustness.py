"""Behaviour that only shows up with a real camera: lag, marker dropouts, pausing, out-of-bounds bots."""
import math

import pytest

from swarm import protocol
from swarm.config import swarm_params_from
from swarm.control import BurstController, ControlParams, DriveController, make_controller
from swarm.geometry import Pose, angle_diff
from swarm.planner import Arena
from swarm.protocol import Command
from swarm.sim.firmware_model import FirmwareModel
from swarm.sim.run import DT, run_trial
from swarm.sim.world import SimBot, SimParams, make_world
from swarm.swarm import Phase, Swarm, SwarmParams

ARENA = Arena(1.00, 0.75)


def _lone_bot(gain_l=1.0, gain_r=1.0, **sim_kw):
    w = make_world(1, 1, ARENA, SimParams(gain_spread=0, packet_loss=0, pos_noise=0.003, heading_noise=0.01, **sim_kw))
    w.bots[1] = SimBot(1, Pose(0.2, 0.2, 0.0), gain_l, gain_r, FirmwareModel(1))
    return w


def _drive(w, ctl, goal, seconds=90.0):
    seq = 0
    while w.t < seconds:
        pose = w.observe()[1]
        cmd, done = ctl.go_to(pose, goal, DT)
        seq += 1
        w.send(1, protocol.encode(1, seq, cmd))
        w.step(DT)
        if done:
            return True
    return False


# ------------------------------------------------------------- controller choice
def test_factory_picks_controller_by_latency():
    assert isinstance(make_controller(ControlParams(latency_s=0.0)), DriveController)
    assert isinstance(make_controller(ControlParams(latency_s=0.4)), BurstController)
    assert isinstance(make_controller(swarm_params_from({"control": {}}).control), BurstController)  # safe default


# ----------------------------------------------------- burst control under lag
@pytest.mark.parametrize("cam_lat", [0.0, 0.3, 0.5])
def test_burst_controller_reaches_goal_despite_camera_lag(cam_lat):
    w = _lone_bot(camera_latency_s=cam_lat)
    ctl = BurstController(ControlParams(latency_s=cam_lat + 0.1))
    goal = (0.8, 0.6)
    assert _drive(w, ctl, goal)
    p = w.bots[1].pose
    assert math.hypot(p.x - goal[0], p.y - goal[1]) < 0.08


def test_burst_controller_learns_turn_rate_speed_and_trim():
    w = _lone_bot(gain_l=1.0, gain_r=0.88, camera_latency_s=0.3)  # right wheel 12% weak: drifts left
    ctl = BurstController(ControlParams(latency_s=0.4, omega0=0.3, speed0=0.03))  # deliberately wrong first guesses
    assert _drive(w, ctl, (0.9, 0.2))
    true_speed = 0.2 * (1 + 0.88) / 2 * ((35 + 15) / 100)  # v_max * mean gain * cmd fraction
    assert ctl.speed == pytest.approx(true_speed, rel=0.4)   # learned from 0.03 to roughly the real value
    assert ctl.trim > 0.5                                    # weak right wheel -> positive (steer-left) offset


def test_burst_controller_raises_power_when_wheels_do_not_move():
    w = _lone_bot(deadband=45, camera_latency_s=0.2)          # stiction above the controller's min_cmd of 35
    ctl = BurstController(ControlParams(latency_s=0.3, min_cmd=35))
    assert _drive(w, ctl, (0.7, 0.5), seconds=120)
    assert ctl.rot_power >= 45 and ctl.fwd_power >= 45


def test_turn_to_holds_aim_inside_double_tolerance():
    ctl = BurstController(ControlParams(latency_s=0.4))
    cmd, ok = ctl.turn_to(Pose(0, 0, 0.0), 0.05)      # inside tolerance: aligned
    assert ok and cmd == Command()
    _, ok = ctl.turn_to(Pose(0, 0, 0.0), 0.15, hold=True)   # 0.15 < 2*0.10, already aligned: do not move
    assert ok


# ---------------------------------------------------------------- swarm dropouts
def _poses(x1=0.2, x2=0.8):
    return {1: Pose(x1, 0.15, math.pi / 2), 2: Pose(x2, 0.15, math.pi / 2)}


def test_one_frame_dropout_does_not_replan():
    sw = Swarm(ARENA, [1, 2], SwarmParams(control=ControlParams(latency_s=0.0)))
    t = 0.0
    for _ in range(5):
        sw.update(_poses(), None, t)
        t += DT
    sectors, paths = dict(sw.sectors), {i: list(p) for i, p in sw.paths.items()}
    for _ in range(4):                               # bot 2's marker vanishes for 0.2 s
        sw.update({1: _poses()[1]}, None, t)
        t += DT
    sw.update(_poses(), None, t)
    assert sw.sectors == sectors and set(sw.paths) == {1, 2}
    assert len(sw.paths[2]) <= len(paths[2])         # progress kept, not reset


def test_long_dropout_reassigns_work_to_the_remaining_bot():
    sw = Swarm(ARENA, [1, 2], SwarmParams())
    t = 0.0
    sw.update(_poses(), None, t)
    for _ in range(int(6.0 / DT)):                   # bot 2 gone for 6 s > lost_timeout_s
        t += DT
        sw.update({1: _poses()[1]}, None, t)
    assert set(sw.paths) == {1} and sw.sectors[1].x1 - sw.sectors[1].x0 == pytest.approx(ARENA.width)


def test_briefly_lost_bot_repeats_last_motion_then_stops():
    sw = Swarm(ARENA, [1, 2], SwarmParams(control=ControlParams(latency_s=0.0)))
    c = sw.update(_poses(), None, 0.0)
    moving = c[2]
    assert moving != Command()
    held = sw.update({1: _poses()[1]}, None, 0.1)[2]
    assert (held.left, held.right) == (moving.left, moving.right) and not held.laser
    assert sw.update({1: _poses()[1]}, None, 0.6)[2] == Command()   # past lost_hold_s


def test_never_fires_blind_when_the_shooter_is_lost():
    sw = Swarm(ARENA, [1, 2], SwarmParams())
    sw.update(_poses(), None, 0.0)
    sw.phase, sw.target, sw.shooter = Phase.MARK, (0.5, 0.4), 1
    sw._t_converge, sw._t_fire = 0.0, 0.0
    sw.slots = {1: (0.5, 0.2), 2: (0.5, 0.6)}
    assert sw.update(_poses(), None, 0.2)[1].laser                        # visible: fires
    brief = sw.update({2: _poses()[2]}, None, 0.4)[1]                     # lost 0.2 s: pulse held, bot stopped
    assert brief.laser and (brief.left, brief.right) == (0, 0)
    assert not sw.update({2: _poses()[2]}, None, 1.0)[1].laser            # lost 0.8 s: laser off


# --------------------------------------------------- full runs under bad conditions
def test_full_run_with_lag_dropouts_and_loss():
    sim = SimParams(detect_radius=0.20, camera_latency_s=0.3, dropout_prob=0.1, packet_loss=0.1)
    sp = SwarmParams(detect_radius=0.20, control=ControlParams(latency_s=0.4))
    res = [run_trial(s, 2, sim=sim, arena=ARENA, swarm_params=sp, timeout_s=300) for s in range(8)]
    assert sum(r.success for r in res) >= 7 and sum(r.collisions for r in res) == 0


def test_old_continuous_controller_really_fails_with_lag():
    """Documents why burst control exists: the continuous controller cannot handle a 0.3 s lag."""
    sim = SimParams(detect_radius=0.20, camera_latency_s=0.3)
    sp = SwarmParams(detect_radius=0.20, control=ControlParams(latency_s=0.0))
    res = [run_trial(s, 2, sim=sim, arena=ARENA, swarm_params=sp, timeout_s=150) for s in range(5)]
    assert sum(r.success for r in res) <= 2
