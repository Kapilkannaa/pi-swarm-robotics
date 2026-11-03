"""Central swarm coordinator: SEARCH -> CONVERGE -> MARK -> DONE.

update() is called every control tick with the latest camera poses and returns one
Command per known bot. It has no clock of its own and no I/O, so the simulator and the real
camera/UDP loop drive it identically.
"""
from __future__ import annotations

import enum
from dataclasses import dataclass, field

from .control import ControlParams, make_controller
from .geometry import Pose, angle_diff, bearing, dist
from .planner import Arena, Sector, assign_sectors, avoid, converge_slots, lawnmower, orient_path, split_sectors
from .protocol import Command


class Phase(enum.Enum):
    SEARCH = "search"
    CONVERGE = "converge"
    MARK = "mark"
    DONE = "done"


@dataclass
class SwarmParams:
    detect_radius: float = 0.25      # sets lane spacing; lanes are 1.6x radius apart for overlap
    margin: float = 0.12
    standoff: float = 0.18           # m from target when aiming: closer = smaller spot error for the same heading error
    slot_tol: float = 0.07
    clearance: float = 0.20          # steer around teammates closer than this to our line
    separation: float = 0.16         # back away from a teammate closer than this when the goal is toward it
    settle_s: float = 0.6            # all bots must hold aligned this long before firing
    converge_timeout_s: float = 40.0 # fire with whoever is ready rather than wait forever
    pulse_s: float = 2.0             # < firmware cap of 2.5 s so the planner ends it first
    lost_hold_s: float = 0.4         # a bot missing this briefly keeps its last command (camera flicker)
    lost_timeout_s: float = 5.0      # missing longer: stop planning for it and re-split the work
    control: ControlParams = field(default_factory=ControlParams)


class Swarm:
    def __init__(self, arena: Arena, bot_ids: list[int], params: SwarmParams | None = None):
        self.arena = arena
        self.bot_ids = sorted(bot_ids)
        self.p = params or SwarmParams()
        self.ctl = {i: make_controller(self.p.control) for i in self.bot_ids}
        self.phase = Phase.SEARCH
        self.target: tuple[float, float] | None = None
        self.shooter: int | None = None
        self.paths: dict[int, list[tuple[float, float]]] = {}
        self.sectors: dict[int, Sector] = {}
        self.slots: dict[int, tuple[float, float]] = {}
        self._t_prev: float | None = None
        self._t_converge: float | None = None
        self._t_ready: float | None = None
        self._t_fire: float | None = None
        self._last_pose: dict[int, Pose] = {}
        self._last_seen: dict[int, float] = {}
        self._last_cmd: dict[int, Command] = {}

    # ------------------------------------------------------------------ public
    def update(self, poses: dict[int, Pose], detection: tuple[float, float] | None,
               now: float) -> dict[int, Command]:
        dt = 0.05 if self._t_prev is None else max(1e-3, now - self._t_prev)
        self._t_prev = now
        for i in self.bot_ids:
            if i in poses:
                self._last_pose[i], self._last_seen[i] = poses[i], now
        # "Active" = seen recently. A one-frame marker dropout must not re-plan anything (it would
        # thrash sectors and slots); only a bot gone for lost_timeout_s is dropped from the plan.
        active = [i for i in self.bot_ids if i in self._last_seen and now - self._last_seen[i] <= self.p.lost_timeout_s]
        known = {i: self._last_pose[i] for i in active}   # last known pose (current when visible)
        visible = {i for i in active if i in poses}
        cmds = {i: Command() for i in self.bot_ids}
        if not known or self.phase is Phase.DONE:
            return cmds

        if self.phase is Phase.SEARCH:
            if detection is not None:
                self.target = detection
                self.phase = Phase.CONVERGE
                self._t_converge = now
                self.slots = converge_slots(detection, known, self.p.standoff)
            else:
                self._search(known, visible, dt, cmds)
                self._hold_lost(known, visible, now, cmds)
                self._last_cmd.update({i: cmds[i] for i in visible})
                return cmds

        if self.phase in (Phase.CONVERGE, Phase.MARK):
            self._converge_and_mark(known, visible, dt, now, cmds)
        self._hold_lost(known, visible, now, cmds)
        self._last_cmd.update({i: cmds[i] for i in visible})
        return cmds

    def _hold_lost(self, known: dict[int, Pose], visible: set[int], now: float, cmds: dict[int, Command]) -> None:
        """Briefly-missing bots repeat their last motion (never the marker); longer-missing ones stay stopped."""
        for i in known:
            if (i not in visible and now - self._last_seen[i] <= self.p.lost_hold_s and i in self._last_cmd
                    and not (self.phase is Phase.MARK and i == self.shooter)):
                c = self._last_cmd[i]
                cmds[i] = Command(c.left, c.right, laser=False)

    # ----------------------------------------------------------------- phases
    def _search(self, known: dict[int, Pose], visible: set[int], dt: float, cmds: dict[int, Command]) -> None:
        if not self.paths or set(self.paths) != set(known):
            # (Re)assign on the first call or when a bot joins or is dropped for good.
            self.sectors = assign_sectors(known, split_sectors(self.arena, len(known)))
            spacing = 1.6 * self.p.detect_radius
            self.paths = {i: orient_path(lawnmower(s, spacing, self.p.margin), (known[i].x, known[i].y))
                          for i, s in self.sectors.items()}
        for i in visible:
            pose, path = known[i], self.paths[i]
            if not path:
                continue  # finished its sector; idle until a teammate finds the target
            cmd, reached = self.ctl[i].go_to(pose, self._steer_point(i, pose, path[0], known), dt)
            if reached and self._is_close(pose, path[0]):
                path.pop(0)
            cmds[i] = cmd

    def _converge_and_mark(self, known: dict[int, Pose], visible: set[int], dt: float, now: float,
                           cmds: dict[int, Command]) -> None:
        assert self.target is not None and self._t_converge is not None
        if set(self.slots) != set(known):
            self.slots = converge_slots(self.target, known, self.p.standoff)
        ready: dict[int, float] = {}  # bot -> |aim error|, for bots in place and aimed
        for i in visible:
            pose = known[i]
            at_slot = dist((pose.x, pose.y), self.slots[i]) < self.p.slot_tol
            if not at_slot and self.phase is Phase.CONVERGE:
                cmds[i], _ = self.ctl[i].go_to(pose, self._steer_point(i, pose, self.slots[i], known), dt)
                continue
            aim = bearing((pose.x, pose.y), self.target)
            cmds[i], aligned = self.ctl[i].turn_to(pose, aim, dt, hold=self.phase is Phase.MARK)
            if aligned:
                ready[i] = abs(angle_diff(aim, pose.theta))

        if self.phase is Phase.CONVERGE:
            all_ready = len(ready) == len(known)  # a bot that is merely not visible right now is not ready
            timed_out = now - self._t_converge > self.p.converge_timeout_s and bool(ready)
            if all_ready or timed_out:
                self._t_ready = self._t_ready if self._t_ready is not None else now
                if now - self._t_ready >= self.p.settle_s or timed_out:
                    # Best-aligned bot fires; others already hold still as spotters.
                    self.shooter = min(ready, key=ready.__getitem__)
                    self.phase = Phase.MARK
                    self._t_fire = now
            else:
                self._t_ready = None  # someone drifted out of tolerance: restart the settle timer

        if self.phase is Phase.MARK:
            assert self._t_fire is not None and self.shooter is not None
            if now - self._t_fire < self.p.pulse_s:
                # Fire only while the camera can see the shooter. A dropout shorter than lost_hold_s keeps
                # the pulse going (the bot is stopped and was aligned a moment ago); longer ends it: never aim blind.
                if self.shooter in visible:
                    c = cmds[self.shooter]
                    cmds[self.shooter] = Command(c.left, c.right, laser=True)
                elif self.shooter in known and now - self._last_seen[self.shooter] <= self.p.lost_hold_s:
                    cmds[self.shooter] = Command(0, 0, laser=True)
            else:
                self.phase = Phase.DONE

    # ---------------------------------------------------------------- helpers
    def _steer_point(self, me: int, pose: Pose, goal: tuple[float, float],
                     poses: dict[int, Pose]) -> tuple[float, float]:
        others = [(q.x, q.y) for j, q in poses.items() if j != me]  # last known positions, incl. briefly lost bots
        return avoid((pose.x, pose.y), goal, others, self.p.clearance, self.p.separation)

    def _is_close(self, pose: Pose, goal: tuple[float, float]) -> bool:
        return dist((pose.x, pose.y), goal) < self.p.control.goal_tol * 1.5
