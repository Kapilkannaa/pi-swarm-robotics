"""Per-bot closed-loop motion: go-to-point, rotate-to-heading, adaptive drift trim.

Commands are in the protocol's -100..100 units. The planner runs this on camera poses, so
heading error is measured, not dead-reckoned; that is what removes motor mismatch drift.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

from .geometry import Pose, angle_diff, bearing, clamp, dist
from .protocol import Command


@dataclass
class ControlParams:
    max_cmd: float = 80.0
    # Smallest command that reliably moves the wheels (stiction). Measure per bot in Phase 6.
    min_cmd: float = 35.0
    k_dist: float = 250.0         # cmd per metre of distance remaining
    k_steer: float = 45.0         # cmd per radian of heading error while driving
    k_turn: float = 60.0          # cmd per radian while rotating in place
    turn_enter: float = 0.7       # rad: error above which we stop and rotate first
    turn_exit: float = 0.3        # rad: hysteresis, avoids chattering around the threshold
    goal_tol: float = 0.05        # m
    heading_tol: float = 0.10     # rad (~5.7 deg)
    trim_ki: float = 12.0         # cmd per (rad*s): learning rate of the drift trim
    trim_limit: float = 20.0      # cmd: cap on how much mismatch we will compensate
    # --- stop-and-measure mode (used when the camera is laggy). latency_s = delay from sending a command to
    # seeing its effect in the pose (network + motor spin-up + camera). Measure it with scripts/measure_latency.py.
    latency_s: float = 0.0        # <= burst_threshold_s: continuous control; above: burst control
    burst_threshold_s: float = 0.1
    settle_extra_s: float = 0.2   # wait this much beyond latency_s before trusting a pose after a burst
    rot_power: float = 0.0        # 0 = auto: min_cmd + 10, learned upward if the wheels do not move
    fwd_power: float = 0.0        # 0 = auto: min_cmd + 15
    rot_max_s: float = 0.5        # longest single rotation burst
    fwd_max_s: float = 0.8        # longest single forward burst
    undershoot: float = 0.8       # aim each burst at this fraction of the remaining error (no overshoot)
    omega0: float = 1.0           # rad/s, first guess of the turn rate at rot_power (learned)
    speed0: float = 0.10          # m/s, first guess of the forward speed at fwd_power (learned)
    heading_tol_drive: float = 0.12  # rad: when driving, re-aim only if the heading error exceeds this
    learn: bool = True            # burst mode: identify turn rate / speed / drift / stiction online (off = ablation only)


class DriveController:
    """One instance per bot; it keeps the learned trim between goals."""

    def __init__(self, params: ControlParams | None = None):
        self.p = params or ControlParams()
        # Adaptive element: differential command offset that cancels this bot's motor
        # mismatch. Integrates heading error while driving straight, persists across goals.
        self.trim = 0.0
        self._turning = False

    def go_to(self, pose: Pose, goal: tuple[float, float], dt: float) -> tuple[Command, bool]:
        """Drive toward goal. Returns (command, reached)."""
        p = self.p
        d = dist((pose.x, pose.y), goal)
        if d < p.goal_tol:
            self._turning = False
            return Command(), True
        err = angle_diff(bearing((pose.x, pose.y), goal), pose.theta)

        if self._turning or abs(err) > p.turn_enter:
            self._turning = abs(err) > p.turn_exit
            if self._turning:
                return self._rotate(err), False

        # Slow when misaligned so we do not arc far off the line, but never below min_cmd.
        fwd = clamp(p.k_dist * d, p.min_cmd, p.max_cmd) * max(0.4, math.cos(err))
        fwd = max(fwd, p.min_cmd)
        if abs(err) < p.turn_exit:
            self.trim = clamp(self.trim + p.trim_ki * err * dt, -p.trim_limit, p.trim_limit)
        steer = p.k_steer * err + self.trim
        return self._mix(fwd, steer), False

    def turn_to(self, pose: Pose, heading: float, dt: float = 0.05, hold: bool = False) -> tuple[Command, bool]:
        """Rotate in place to an absolute heading. Returns (command, aligned). dt/hold: API parity with BurstController."""
        err = angle_diff(heading, pose.theta)
        if abs(err) < self.p.heading_tol:
            return Command(), True
        return self._rotate(err), False

    def _rotate(self, err: float) -> Command:
        p = self.p
        mag = clamp(p.k_turn * abs(err), p.min_cmd, p.max_cmd)
        s = mag if err > 0 else -mag  # positive error = turn left = right wheel forward
        return Command(left=round(-s), right=round(s))

    def _mix(self, fwd: float, steer: float) -> Command:
        left, right = fwd - steer, fwd + steer
        # Scale both down together if one saturates, otherwise steering would be clipped away.
        peak = max(abs(left), abs(right))
        if peak > self.p.max_cmd:
            left, right = left * self.p.max_cmd / peak, right * self.p.max_cmd / peak
        return Command(left=round(left), right=round(right))


class BurstController:
    """Stop-and-measure control for a laggy camera, with online identification of the bot.

    A continuous controller acting on delayed poses overshoots and rings. This one moves in short
    bursts (rotate or drive), stops, waits for the delayed pose to catch up, then re-measures. After
    every burst it learns from what actually happened:
      * turn rate  omega  (rad/s at rot_power)   -> sizes the next rotation burst
      * forward speed     (m/s at fwd_power)     -> sizes the next drive burst
      * heading drift while driving              -> updates `trim`, the steering offset that cancels motor mismatch
      * stiction: if a burst moved nothing, rot_power / fwd_power are raised until it does
    So no speed constant has to be measured in advance, and it adapts per bot (the project's adaptive element).
    Same call API as DriveController, so the planner does not care which one it has.
    """

    def __init__(self, params: ControlParams | None = None):
        self.p = params or ControlParams()
        p = self.p
        self.trim = 0.0
        self.omega = p.omega0
        self.speed = p.speed0
        self.rot_power = p.rot_power or min(p.max_cmd, p.min_cmd + 10)
        self.fwd_power = p.fwd_power or min(p.max_cmd, p.min_cmd + 15)
        self._t = 0.0
        self._mode = "idle"  # idle | burst | wait
        self._until = 0.0
        self._cmd = Command()
        self._burst: dict | None = None  # what we are doing, for learning afterwards
        self._aligned = False

    # ------------------------------------------------------------------ API
    def go_to(self, pose: Pose, goal: tuple[float, float], dt: float = 0.05) -> tuple[Command, bool]:
        busy = self._service(pose, dt)
        if busy is not None:
            return busy, False
        p = self.p
        d = dist((pose.x, pose.y), goal)
        if d < p.goal_tol:
            return Command(), True
        err = angle_diff(bearing((pose.x, pose.y), goal), pose.theta)
        if abs(err) > p.heading_tol_drive:
            return self._start_rotation(pose, err), False
        return self._start_drive(pose, d), False

    def turn_to(self, pose: Pose, heading: float, dt: float = 0.05, hold: bool = False) -> tuple[Command, bool]:
        busy = self._service(pose, dt)
        if busy is not None:
            return busy, False
        err = angle_diff(heading, pose.theta)
        # While holding aim (the laser is on) do not chase noise: stay put inside twice the tolerance.
        tol = self.p.heading_tol * (2.0 if (hold and self._aligned) else 1.0)
        if abs(err) < tol:
            self._aligned = True
            return Command(), True
        self._aligned = False
        return self._start_rotation(pose, err), False

    # ------------------------------------------------------------ internals
    def _service(self, pose: Pose, dt: float) -> Command | None:
        """Advance the burst/wait state machine. Returns a command while busy, None when a decision is due."""
        self._t += dt
        if self._mode == "burst":
            if self._t < self._until:
                return self._cmd
            self._mode, self._until = "wait", self._t + self.p.latency_s + self.p.settle_extra_s
            return Command()
        if self._mode == "wait":
            if self._t < self._until:
                return Command()
            self._learn(pose)
            self._mode = "idle"
        return None

    def _start_rotation(self, pose: Pose, err: float) -> Command:
        p = self.p
        dur = clamp(p.undershoot * abs(err) / max(self.omega, 0.05), 0.05, p.rot_max_s)
        sign = 1.0 if err > 0 else -1.0
        P = round(self.rot_power)
        self._cmd = Command(left=-sign * P, right=sign * P)
        self._begin("rot", pose, dur, sign)
        return self._cmd

    def _start_drive(self, pose: Pose, d: float) -> Command:
        p = self.p
        dur = clamp(p.undershoot * d / max(self.speed, 0.02), 0.05, p.fwd_max_s)
        F, s = self.fwd_power, self.trim
        self._cmd = Command(left=round(F - s), right=round(F + s))
        self._begin("fwd", pose, dur, 1.0)
        return self._cmd

    def _begin(self, kind: str, pose: Pose, dur: float, sign: float) -> None:
        self._burst = {"kind": kind, "pose": pose, "dur": dur, "sign": sign}
        self._mode, self._until = "burst", self._t + dur

    def _learn(self, now: Pose) -> None:
        b, self._burst = self._burst, None
        if b is None or not self.p.learn:
            return
        p, pre, dur = self.p, b["pose"], b["dur"]
        dth = angle_diff(now.theta, pre.theta)
        if b["kind"] == "rot":
            turned = dth * b["sign"]  # signed progress in the commanded direction
            if turned < 0.02:         # (almost) nothing happened: stiction, push harder next time
                self.rot_power = min(p.max_cmd, self.rot_power + 5)
            else:
                self.omega = clamp(0.5 * self.omega + 0.5 * turned / dur, 0.1, 8.0)
        else:
            along = (now.x - pre.x) * math.cos(pre.theta) + (now.y - pre.y) * math.sin(pre.theta)
            if along < 0.005:
                self.fwd_power = min(p.max_cmd, self.fwd_power + 5)
                return
            self.speed = clamp(0.5 * self.speed + 0.5 * along / dur, 0.02, 2.0)
            # Residual drift rate (rad/s) with the current trim applied; convert to a steering correction using
            # the learned turn sensitivity: omega(s) = omega * s / rot_power.
            drift = dth / dur
            self.trim = clamp(self.trim - 0.7 * drift * self.rot_power / max(self.omega, 0.1), -p.trim_limit, p.trim_limit)


def make_controller(params: ControlParams | None = None) -> DriveController | BurstController:
    """Burst control when the camera is laggy, continuous control otherwise."""
    p = params or ControlParams()
    return BurstController(p) if p.latency_s > p.burst_threshold_s else DriveController(p)
