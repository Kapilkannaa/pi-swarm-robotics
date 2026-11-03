"""Camera-independent adaptive LED driving; runs on the Pi with stdlib only."""
from __future__ import annotations
import math
from dataclasses import dataclass
from .geometry import angle_diff, bearing, clamp, dist, wrap_angle
from .protocol import Command

@dataclass
class ClickParams:
    min_cmd: float = 35.0
    max_cmd: float = 80.0
    latency_s: float = 0.4          # command -> visible motion delay (measure with scripts/measure_latency.py)
    settle_extra_s: float = 0.2
    rot_power: float = 0.0          # 0 = auto (min_cmd + 10), raised if a burst moves nothing
    fwd_power: float = 0.0          # 0 = auto (min_cmd + 15)
    rot_max_s: float = 0.5
    fwd_max_s: float = 0.8
    probe_s: float = 0.5            # first forward burst, to discover which way the bot faces
    verify_s: float = 0.3           # short forward burst after a rotation: checks the heading before committing
    undershoot: float = 0.8
    omega0: float = 1.5             # rad/s, first guess of the turn rate (learned)
    speed0: float = 0.10            # m/s, first guess of the forward speed (learned)
    goal_tol: float = 0.04          # m: arrived when the bot's centre is this close to the click
    aim_tol: float = 0.25           # rad: drive forward only when pointing within this of the goal
    led_offset: float = 0.05        # m: how far BEHIND the bot's centre the LED sits (measure it, roughly)
    min_move: float = 0.012         # m: a forward burst that moved less than this counts as "did not move" (stiction)
    learn_offset: bool = True       # refine led_offset from how far the LED swings during a verified in-place turn
    lost_stop_s: float = 0.5        # LED unseen this long -> force a stop
    margin: float = 0.15            # m: a dot further outside the arena than this stops the bot


class ClickDriver:
    def __init__(self, params: ClickParams | None = None):
        self.p = params or ClickParams()
        p = self.p
        self.theta: float | None = None   # heading estimate (rad, arena frame); None until the first probe
        self.omega, self.speed = p.omega0, p.speed0
        self.rot_power = p.rot_power or min(p.max_cmd, p.min_cmd + 10)
        self.fwd_power = p.fwd_power or min(p.max_cmd, p.min_cmd + 15)
        self._t, self._mode, self._until = 0.0, "idle", 0.0   # idle | burst | wait
        self._cmd = Command()
        self._burst: dict | None = None
        self._pending: dict | None = None   # a rotation not yet verified by motion
        self._arrived = False
        self.led_offset = p.led_offset     # starts at the configured guess, refined online
        self._cal = 0 if p.learn_offset else 2   # offset calibration: 0 = to do, 1 = turn done, awaiting the hop, 2 = finished
        self._cal_tries = 0
        self._force_verify = False

    def centre(self, pos: tuple[float, float]) -> tuple[float, float]:
        if self.theta is None:
            return pos
        return (pos[0] + self.led_offset * math.cos(self.theta), pos[1] + self.led_offset * math.sin(self.theta))

    def update(self, pos: tuple[float, float] | None, goal: tuple[float, float] | None,
               dt: float) -> tuple[Command, str]:
        """One tick. pos = the LED's arena position (or None if unseen). Returns (command, status)."""
        p = self.p
        self._t += dt
        if goal is None:
            # An operator stop must cancel an active burst, not just the next goal.
            self._mode, self._burst, self._pending = "idle", None, None
            self._cmd, self._arrived, self._force_verify = Command(), False, False
            self.theta = None  # a partially completed burst makes heading uncertain
            if self._cal != 2:
                self._cal = 0
            return Command(), "idle"
        if self._mode == "burst":
            if self._t < self._until:
                return self._cmd, "moving"
            self._mode, self._until = "wait", self._t + p.latency_s + p.settle_extra_s
            return Command(), "moving"
        if self._mode == "wait":
            if pos is None:
                return Command(), "lost"
            if self._t < self._until:
                return Command(), "moving"
            self._learn(pos)
            self._mode = "idle"
        if pos is None:
            return Command(), "lost"
        c = self.centre(pos)
        d = dist(c, goal)
        if d < p.goal_tol * (2.0 if self._arrived else 1.0):   # hysteresis: do not restart for a few mm of jitter
            self._arrived = True
            return Command(), "arrived"
        self._arrived = False
        if self.theta is None:
            return self._start_fwd(pos, p.probe_s, "probe"), "moving"
        if self._cal == 0 and self._cal_tries < 3 and d > 2 * p.goal_tol:
            # One deliberate in-place turn, then a short hop: the arc the LED swings gives how far it sits behind the centre.
            self._cal, self._cal_tries = 1, self._cal_tries + 1
            return self._start_rot(pos, 1.2), "moving"
        if self._force_verify:
            self._force_verify = False
            return self._start_fwd(pos, p.verify_s, "fwd"), "moving"
        e = angle_diff(bearing(c, goal), self.theta)
        if abs(e) > p.aim_tol:
            return self._start_rot(pos, e), "moving"
        dur = clamp(p.undershoot * d / max(self.speed, 0.02), 0.1, p.fwd_max_s)
        if self._pending is not None:
            dur = min(dur, p.verify_s)  # heading is only dead-reckoned: check it with a short burst first
        return self._start_fwd(pos, dur, "fwd"), "moving"

    # ------------------------------------------------------------ internals
    def _start_rot(self, pos: tuple[float, float], e: float) -> Command:
        p = self.p
        dur = clamp(p.undershoot * abs(e) / max(self.omega, 0.05), 0.05, p.rot_max_s)
        sign = 1.0 if e > 0 else -1.0
        pred = sign * self.omega * dur
        assert self.theta is not None
        if self._pending is None:
            self._pending = {"theta_before": self.theta, "pred": pred, "n": 1, "chord": None}
        else:
            self._pending["pred"] += pred
            self._pending["n"] += 1
        self.theta = wrap_angle(self.theta + pred)
        P = round(self.rot_power)
        self._cmd = Command(left=-sign * P, right=sign * P)
        self._burst = {"kind": "rot", "pos": pos}
        self._mode, self._until = "burst", self._t + dur
        return self._cmd

    def _start_fwd(self, pos: tuple[float, float], dur: float, kind: str) -> Command:
        F = round(self.fwd_power)
        self._cmd = Command(left=F, right=F)
        self._burst = {"kind": kind, "pos": pos, "dur": dur}
        self._mode, self._until = "burst", self._t + dur
        return self._cmd

    def _learn(self, now: tuple[float, float]) -> None:
        b, self._burst = self._burst, None
        if b is None:
            return
        p = self.p
        if b["kind"] == "rot":
            if self._pending is not None and self._pending["n"] == 1:   # only a SINGLE rotation gives a clean arc
                self._pending["chord"] = math.hypot(now[0] - b["pos"][0], now[1] - b["pos"][1])
            if self._cal == 1:
                self._force_verify = True
            return
        dx, dy = now[0] - b["pos"][0], now[1] - b["pos"][1]
        d = math.hypot(dx, dy)
        if d < p.min_move:  # nothing moved: stiction. Push harder; the heading stays unknown/unverified.
            self.fwd_power = min(p.max_cmd, self.fwd_power + 5)
            return
        measured = math.atan2(dy, dx)
        if self._pending is not None:
            actual = angle_diff(measured, self._pending["theta_before"])
            pred = self._pending["pred"]
            if abs(pred) > 0.1 and actual * pred > 0:  # turned the way we asked: scale the turn-rate estimate
                ratio = clamp(actual / pred, 0.3, 3.0)
                self.omega = clamp(self.omega * (0.5 + 0.5 * ratio), 0.2, 10.0)
                chord = self._pending["chord"]
                if p.learn_offset and chord is not None and abs(actual) >= 0.5:
                    # In-place turn by `actual`: the LED swings on a circle around the centre, chord = 2 r sin(phi/2).
                    r = chord / (2.0 * math.sin(abs(actual) / 2.0))
                    w = 1.0 if self._cal == 1 else 0.4   # the deliberate calibration turn is the trusted sample
                    self.led_offset = clamp((1 - w) * self.led_offset + w * r, 0.01, 0.20)
                    if self._cal == 1:
                        self._cal = 2
            self._pending = None
        if self._cal == 1:  # the calibration turn did not give a usable arc (too small / wrong way): try again later
            self._cal = 0
        self.theta = measured
        self.speed = clamp(0.5 * self.speed + 0.5 * d / b["dur"], 0.02, 2.0)


