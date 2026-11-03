"""2D arena with differential-drive bots, a hidden target and an imperfect link/camera.

Every constant in SimParams is a placeholder until measured on the real bots; the sim is
deliberately a bit pessimistic (motor mismatch, dead-band, packet loss, latency) so the
planner is not tuned to a perfect world.
"""
from __future__ import annotations

import math
import random
from collections import deque
from dataclasses import dataclass, field

from ..geometry import Pose, clamp, dist, wrap_angle
from ..planner import Arena
from .firmware_model import PWM_CAP, FirmwareModel, Outputs


@dataclass
class SimParams:
    v_max: float = 0.20            # m/s per wheel at command 100 (ASSUMED, measure on hardware)
    track: float = 0.10            # m between wheels (ASSUMED)
    gain_spread: float = 0.12      # each wheel's gain is uniform in 1 +/- spread (motor mismatch)
    deadband: float = 25.0         # |command| below this does not move the wheel (stiction)
    packet_loss: float = 0.03
    latency_s: float = 0.08
    pos_noise: float = 0.005       # m, camera pose noise (1 sigma)
    heading_noise: float = 0.02    # rad
    camera_latency_s: float = 0.0  # age of the pose the planner sees (IP-camera apps: roughly 0.2-0.5 s)
    dropout_prob: float = 0.0      # chance per bot per frame that its marker is not detected
    detect_radius: float = 0.25    # target is "discovered" only inside this range of a bot
    bot_radius: float = 0.06
    mark_tol: float = 0.03         # m: laser ray must pass this close to the target centre
    mark_hold_s: float = 1.0       # ...for this much TOTAL time during the pulse (not necessarily continuous)


@dataclass
class SimBot:
    bot_id: int
    pose: Pose
    gain_l: float
    gain_r: float
    fw: FirmwareModel
    laser_on: bool = False


@dataclass
class World:
    arena: Arena
    target: tuple[float, float]
    bots: dict[int, SimBot]
    p: SimParams
    rng: random.Random
    t: float = 0.0
    marked: bool = False
    t_found: float | None = None
    collisions: int = 0
    min_separation: float = math.inf
    max_laser_s: float = 0.0
    _inbox: list[tuple[float, int, bytes]] = field(default_factory=list)
    _laser_run: dict[int, float] = field(default_factory=dict)
    hit_time: float = 0.0
    _hist: deque = field(default_factory=lambda: deque(maxlen=400))
    _in_contact: bool = False

    # -------------------------------------------------------------- interface
    def send(self, bot_id: int, packet: bytes) -> None:
        if self.rng.random() < self.p.packet_loss:
            return
        self._inbox.append((self.t + self.p.latency_s, bot_id, packet))

    def observe(self) -> dict[int, Pose]:
        """What the overhead camera reports: noisy, possibly stale poses, with occasional missed markers."""
        r = self.rng
        # Pose history lets us hand back the world as it was `camera_latency_s` ago.
        self._hist.append((self.t, {i: b.pose for i, b in self.bots.items()}))
        stale = self._hist[0][1]
        for t, snap in self._hist:
            if t <= self.t - self.p.camera_latency_s:
                stale = snap
            else:
                break
        out: dict[int, Pose] = {}
        for i, pose in stale.items():
            if r.random() < self.p.dropout_prob:
                continue
            out[i] = Pose(pose.x + r.gauss(0, self.p.pos_noise), pose.y + r.gauss(0, self.p.pos_noise),
                          wrap_angle(pose.theta + r.gauss(0, self.p.heading_noise)))
        return out

    def detect(self) -> tuple[float, float] | None:
        """Target position if any bot is within the sensing radius, else None."""
        for b in self.bots.values():
            if dist((b.pose.x, b.pose.y), self.target) <= self.p.detect_radius:
                if self.t_found is None:
                    self.t_found = self.t
                return (self.target[0] + self.rng.gauss(0, self.p.pos_noise),
                        self.target[1] + self.rng.gauss(0, self.p.pos_noise))
        return None

    # ---------------------------------------------------------------- physics
    def step(self, dt: float) -> None:
        now_ms = int(self.t * 1000)
        due = [m for m in self._inbox if m[0] <= self.t]
        self._inbox = [m for m in self._inbox if m[0] > self.t]
        for _, bot_id, data in due:
            self.bots[bot_id].fw.on_packet(data, now_ms)

        for b in self.bots.values():
            out = b.fw.tick(now_ms)
            self._move(b, out, dt)
            b.laser_on = out.laser
        self._contacts(dt)
        self._laser(dt)
        self.t += dt

    def _wheel_speed(self, duty: float, gain: float) -> float:
        cmd = duty / PWM_CAP * 100
        if abs(cmd) < self.p.deadband:
            return 0.0
        return gain * cmd / 100 * self.p.v_max

    def _move(self, b: SimBot, out: Outputs, dt: float) -> None:
        vl = self._wheel_speed(out.left, b.gain_l)
        vr = self._wheel_speed(out.right, b.gain_r)
        v, w = (vl + vr) / 2, (vr - vl) / self.p.track
        th = b.pose.theta + w * dt / 2  # midpoint heading for a better arc approximation
        x = clamp(b.pose.x + v * math.cos(th) * dt, 0.0, self.arena.width)
        y = clamp(b.pose.y + v * math.sin(th) * dt, 0.0, self.arena.height)
        b.pose = Pose(x, y, wrap_angle(b.pose.theta + w * dt))

    def _contacts(self, dt: float) -> None:
        ids = sorted(self.bots)
        touching = False
        for a in range(len(ids)):
            for c in range(a + 1, len(ids)):
                pa, pc = self.bots[ids[a]].pose, self.bots[ids[c]].pose
                d = dist((pa.x, pa.y), (pc.x, pc.y))
                self.min_separation = min(self.min_separation, d)
                touching |= d < 2 * self.p.bot_radius
        if touching and not self._in_contact:
            self.collisions += 1  # count contact events, not time spent in contact
        self._in_contact = touching

    def _laser(self, dt: float) -> None:
        hit = False
        for b in self.bots.values():
            run = self._laser_run.get(b.bot_id, 0.0) + dt if b.laser_on else 0.0
            self._laser_run[b.bot_id] = run
            self.max_laser_s = max(self.max_laser_s, run)
            if b.laser_on and self._ray_miss(b.pose) <= self.p.mark_tol:
                hit = True
        if hit:
            self.hit_time += dt  # cumulative: the aim chatters around the tolerance, a spot that is on
        if self.hit_time >= self.p.mark_hold_s:  # target >= half the pulse is a mark in any physical sense
            self.marked = True

    def _ray_miss(self, pose: Pose) -> float:
        """Distance from the target to the laser ray (forward half-line from the bot)."""
        dx, dy = self.target[0] - pose.x, self.target[1] - pose.y
        along = dx * math.cos(pose.theta) + dy * math.sin(pose.theta)
        if along < 0:
            return math.inf
        return abs(-dx * math.sin(pose.theta) + dy * math.cos(pose.theta))


def make_world(seed: int, n_bots: int = 2, arena: Arena | None = None,
               params: SimParams | None = None) -> World:
    """Random target and start poses along the bottom edge, like placing bots by hand."""
    rng = random.Random(seed)
    p = params or SimParams()
    arena = arena or Arena(1.20, 0.90)
    bots: dict[int, SimBot] = {}
    for k in range(n_bots):
        x = arena.width * (k + 1) / (n_bots + 1)
        pose = Pose(x, 0.12, math.pi / 2 + rng.uniform(-0.3, 0.3))
        bots[k + 1] = SimBot(k + 1, pose,
                             1 + rng.uniform(-p.gain_spread, p.gain_spread),
                             1 + rng.uniform(-p.gain_spread, p.gain_spread),
                             FirmwareModel(k + 1))
    # Keep the target off the walls and away from the start line so the search is real.
    while True:
        target = (rng.uniform(0.15, arena.width - 0.15), rng.uniform(0.35, arena.height - 0.15))
        if all(dist(target, (b.pose.x, b.pose.y)) > p.detect_radius + 0.1 for b in bots.values()):
            break
    return World(arena, target, bots, p, rng)
