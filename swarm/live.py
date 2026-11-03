"""The real-world loop body: camera frame in, bot commands out.

LiveSystem.step() is hardware-free (takes a frame, returns commands), so the exact same code runs
in the offline dry run (synthetic camera + simulated bots) and in scripts/run_live.py (phone camera
+ real bots over UDP).

Target discovery is gated by a virtual sensing radius, as in the simulator: the camera could see
the target from the start, but it only counts as found when a bot is within `detect_radius` of it.
This is a design choice for the "search" task and must be stated in the write-up.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from .geometry import Pose, dist
from .protocol import Command
from .swarm import Phase, Swarm
from .vision.target import Ranges, find_target
from .vision.tracker import ArenaTracker


@dataclass
class LiveState:
    cmds: dict[int, Command]
    poses: dict[int, Pose]
    target_seen: tuple[float, float] | None   # where the camera sees the target (always, if visible)
    detection: tuple[float, float] | None     # what the swarm is told (gated by sensing radius)
    phase: Phase
    out_of_bounds: tuple[int, ...] = ()       # bots found outside the arena: forced to stop


class LiveSystem:
    def __init__(self, tracker: ArenaTracker, swarm: Swarm, target_color: "str | Ranges", detect_radius: float,
                 arena_slack: float = 0.05, oob_margin: float = 0.15):
        self.tracker, self.swarm = tracker, swarm
        self.color, self.detect_radius, self.slack = target_color, detect_radius, arena_slack
        self.oob_margin = oob_margin

    def _target_world(self, frame: np.ndarray) -> tuple[float, float] | None:
        px = find_target(frame, self.color)
        if px is None:
            return None
        x, y = self.tracker.pixel_to_world(px)
        a = self.swarm.arena
        # A "target" outside the arena is a false positive (a shoe, a reflection): ignore it.
        if not (-self.slack <= x <= a.width + self.slack and -self.slack <= y <= a.height + self.slack):
            return None
        return (x, y)

    def step(self, frame: np.ndarray, now: float, run: bool = True) -> LiveState:
        """One control tick. run=False only observes: the planner is NOT updated (so a paused system learns and plans nothing)."""
        poses = self.tracker.poses(frame)
        seen = self._target_world(frame)
        if not run:
            return LiveState({i: Command() for i in self.swarm.bot_ids}, poses, seen, None, self.swarm.phase)
        detection = None
        if seen is not None and any(dist((p.x, p.y), seen) <= self.detect_radius for p in poses.values()):
            detection = seen
        cmds = self.swarm.update(poses, detection, now)
        # Safety net: a bot that is outside the arena (fell off the edge, calibration error, wrong marker) is stopped.
        a, m = self.swarm.arena, self.oob_margin
        oob = tuple(i for i, p in poses.items() if i in cmds and not (-m <= p.x <= a.width + m and -m <= p.y <= a.height + m))
        for i in oob:
            cmds[i] = Command()
        return LiveState(cmds, poses, seen, detection, self.swarm.phase, oob)


def laser_ray_miss(pose: Pose, target: tuple[float, float]) -> float:
    """Distance from the target to the forward ray of a bot; inf if the target is behind it."""
    dx, dy = target[0] - pose.x, target[1] - pose.y
    along = dx * math.cos(pose.theta) + dy * math.sin(pose.theta)
    return math.inf if along < 0 else abs(-dx * math.sin(pose.theta) + dy * math.cos(pose.theta))
