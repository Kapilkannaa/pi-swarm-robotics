"""Required Pi brain. No OpenCV/numpy: runs on the ARMv6 Pi Zero W.

The camera sends observations, never wheel commands. A short-lived one-use ticket
rejects observations delayed in transit without needing synchronized host clocks.
"""
from __future__ import annotations

import math
import secrets
from dataclasses import asdict

from .click_control import ClickDriver, ClickParams
from .geometry import Pose, dist
from .planner import Arena
from .protocol import Command
from .swarm import Swarm, SwarmParams


class Coordinator:
    def __init__(self, arena: Arena, ids: list[int], params: SwarmParams,
                 allow_motion: bool = False, mode: str = "swarm", model=None):
        if not ids or len(set(ids)) != len(ids) or mode not in ("swarm", "led"):
            raise ValueError("select unique bot IDs and mode swarm|led")
        if mode == "led" and len(ids) != 1:
            raise ValueError("LED mode supports one robot only; use ArUco for two")
        self.arena, self.ids, self.params = arena, sorted(ids), params
        self.allow_motion, self.mode, self.model = allow_motion, mode, model
        self.session, self.ticket, self.ticket_until = "", "", 0.0
        self.seq, self.last_rx, self.lease_until = -1, -1.0, -1.0
        self.running, self.reason = False, "bench: motion disabled" if not allow_motion else "paused"
        self.ready = False
        self.cmds = self.stops()
        self.proposed = self.stops()
        self.confidence = None
        self._reset()

    def _reset(self):
        self.swarm = Swarm(self.arena, self.ids, self.params)
        p = self.params.control
        self.click = ClickDriver(ClickParams(min_cmd=p.min_cmd, max_cmd=p.max_cmd, latency_s=p.latency_s))

    def stops(self) -> dict[int, Command]:
        return {i: Command() for i in self.ids}

    def halt(self, reason: str):
        self.running, self.reason = False, reason
        self.ready = False
        self.cmds = self.stops()
        self.lease_until = -1.0
        self._reset()

    def new_session(self, now: float) -> dict:
        self.halt("new camera session; paused")
        self.session, self.seq = secrets.token_hex(16), -1
        return self._reply(now)

    def _reply(self, now: float) -> dict:
        self.ticket, self.ticket_until = secrets.token_hex(16), now + 0.5
        return {**self.status(now), "session": self.session, "ticket": self.ticket}

    def status(self, now: float) -> dict:
        fields = ("speed", "omega", "trim", "rot_power", "fwd_power", "theta", "led_offset")
        controllers = self.swarm.ctl if self.mode == "swarm" else {self.ids[0]: self.click}
        return {"role": "raspberry-pi-coordinator", "mode": self.mode, "bots": self.ids,
                "arena": [self.arena.width, self.arena.height],
                "motion_allowed": self.allow_motion, "running": self.running,
                "reason": self.reason, "phase": self.swarm.phase.value,
                "observation_age_s": None if self.last_rx < 0 else round(now - self.last_rx, 3),
                "commands": {str(i): asdict(c) for i, c in self.cmds.items()},
                "proposed": {str(i): asdict(c) for i, c in self.proposed.items()},
                "ai": {"target_probability": self.confidence, "online_identification": {
                    str(i): {f: getattr(ctl, f) for f in fields if hasattr(ctl, f)}
                    for i, ctl in controllers.items()}}}

    @staticmethod
    def _vector(value, size):
        if not isinstance(value, (list, tuple)) or len(value) != size:
            raise ValueError(f"expected {size} numbers")
        if any(type(v) not in (int, float) or not math.isfinite(v) for v in value):
            raise ValueError("non-finite or non-numeric observation")
        return tuple(float(v) for v in value)

    def _inside(self, xy, margin=0.0):
        return margin <= xy[0] <= self.arena.width - margin and margin <= xy[1] <= self.arena.height - margin

    def observe(self, data: dict, now: float) -> dict:
        try:
            if not self.session or data.get("session") != self.session or data.get("ticket") != self.ticket:
                raise ValueError("invalid session/ticket")
            if now >= self.ticket_until:
                raise ValueError("observation delayed in transit")
            self.ticket = ""  # single-use even when subsequent validation fails
            seq, age = data.get("seq"), data.get("age_s")
            if type(seq) is not int or seq <= self.seq:
                raise ValueError("replayed/out-of-order observation")
            if type(age) not in (int, float) or not math.isfinite(age) or not 0 <= age < 0.35:
                raise ValueError("stale camera frame")
            if type(data.get("run")) is not bool:
                raise ValueError("run must be boolean")
            poses = {int(i): Pose(*self._vector(v, 3)) for i, v in data.get("poses", {}).items()}
            if not set(poses) <= set(self.ids):
                raise ValueError("unknown bot ID")
            target = data.get("target")
            target = self._vector(target, 2) if target is not None else None
            if target is not None and not self._inside(target):
                raise ValueError("target outside arena")
            led = data.get("led")
            led = self._vector(led, 2) if led is not None else None
            goal = data.get("goal")
            goal = self._vector(goal, 2) if goal is not None else None
            if goal is not None and not self._inside(goal, 0.05):
                raise ValueError("goal outside arena")
            self.confidence = None
            if self.model is not None and target is not None:
                features = self._vector(data.get("target_features"), self.model.size)
                self.confidence = self.model.probability(features)
                if self.confidence < self.model.threshold:
                    target = None
            dt = 0.05 if self.last_rx < 0 else min(0.2, max(0.001, now - self.last_rx))
            self.seq, self.last_rx = seq, now
            if not data["run"]:
                self.halt("paused")
                self.ready = True
                self.proposed = self.stops()
                return self._reply(now)
            if not self.ready:
                raise ValueError("send paused observation before restarting")
            if self.mode == "swarm":
                # 2 cm tolerance for image quantization/calibration; not a physical obstacle sensor.
                if set(poses) != set(self.ids) or any(not self._inside((p.x, p.y), -0.02) for p in poses.values()):
                    raise ValueError("all selected robots must be visible inside arena")
            elif led is None or not self._inside(led, -0.02):
                raise ValueError("LED missing/outside arena")
            if not self.running:
                self._reset()
            self.running = True
            if self.mode == "swarm":
                detection = target if target and any(dist((p.x, p.y), target) <= self.params.detect_radius for p in poses.values()) else None
                self.proposed = self.swarm.update(poses, detection, now)
            else:
                cmd, self.reason = self.click.update(led, goal, dt)
                self.proposed = {self.ids[0]: cmd}
            self.cmds = dict(self.proposed) if self.allow_motion else self.stops()
            self.reason = "running" if self.allow_motion else "bench: planning only; outputs zero"
            self.lease_until = now + min(0.25, 0.35 - age)
            return self._reply(now)
        except (ValueError, TypeError, KeyError, AttributeError, OverflowError) as exc:
            self.halt(str(exc))
            raise ValueError(str(exc)) from exc

    def outputs(self, now: float) -> dict[int, Command]:
        if self.running and now >= self.lease_until:
            self.halt("camera/link timeout; restart required")
        return dict(self.cmds)
