"""Click-to-go: drive one bot to a point the user clicks, tracking it only by a single bright dot (its red LED).

One dot gives the bot's POSITION but not which way it FACES, so the heading is learned from motion:
  * probe: drive forward a little; the direction the dot moved IS the heading.
  * rotate: turn by (turn rate x time), dead-reckoning the heading estimate;
  * verify: drive a short burst toward the goal; the dot's real displacement corrects the heading estimate
    and tells us how far the last rotation really turned, which corrects the turn-rate estimate.
Like swarm/control.py's BurstController it moves in bursts and waits `latency_s` for the (laggy) camera before trusting
the next position, and it learns speed and turn rate online, so no constant has to be measured in advance.
The LED sits `led_offset` behind the bot's centre, so the centre (what should reach the click) is the dot pushed forward.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from .geometry import angle_diff, bearing, clamp, dist, wrap_angle
from .planner import Arena
from .protocol import Command
from .vision.led import LedParams, arena_roi, find_led
from .vision.tracker import ArenaTracker


from .click_control import ClickParams, ClickDriver


@dataclass
class ClickState:
    cmd: Command
    status: str                                   # idle | moving | arrived | lost | out-of-bounds
    led_px: tuple[float, float] | None
    led_world: tuple[float, float] | None
    centre: tuple[float, float] | None
    theta: float | None
    goal: tuple[float, float] | None


class ClickGo:
    """Camera frame + clicks in, bot command out. Hardware-free, so the dry run and the real run share it."""

    def __init__(self, tracker: ArenaTracker, arena: Arena, params: ClickParams | None = None,
                 led: LedParams | None = None):
        self.tracker, self.arena = tracker, arena
        self.p = params or ClickParams()
        self.led = led or LedParams()
        self.driver = ClickDriver(self.p)
        self.goal: tuple[float, float] | None = None
        self._roi: np.ndarray | None = None
        self._roi_shape: tuple[int, int] | None = None
        self._last_px: tuple[float, float] | None = None
        self._lost_for = 0.0

    def set_goal_pixel(self, px: tuple[float, float]) -> tuple[float, float]:
        """A click in the image becomes a goal on the floor, kept inside the arena."""
        x, y = self.tracker.pixel_to_world(px)
        m = 0.05
        self.goal = (clamp(x, m, self.arena.width - m), clamp(y, m, self.arena.height - m))
        return self.goal

    def stop(self) -> None:
        self.goal = None

    def _ensure_roi(self, shape: tuple[int, int]) -> None:
        if self._roi is None or self._roi_shape != shape:
            a, g = self.arena, 2 * self.p.margin  # twice the out-of-bounds margin, so that check can fire before the dot is ignored
            ring = ((-g, -g), (a.width + g, -g), (a.width + g, a.height + g), (-g, a.height + g))
            corners = [self.tracker.world_to_pixel(c) for c in ring]
            self._roi, self._roi_shape = arena_roi(shape, corners, grow_px=2), shape

    def step(self, frame: np.ndarray, dt: float) -> ClickState:
        self._ensure_roi(frame.shape[:2])
        px = find_led(frame, self.led, self._roi, near=self._last_px)
        pos = None
        if px is not None:
            self._last_px = px
            pos = self.tracker.pixel_to_world(px)
        a, m = self.arena, self.p.margin
        if pos is not None and not (-m <= pos[0] <= a.width + m and -m <= pos[1] <= a.height + m):
            return ClickState(Command(), "out-of-bounds", px, pos, None, self.driver.theta, self.goal)
        self._lost_for = 0.0 if pos is not None else self._lost_for + dt
        cmd, status = self.driver.update(pos, self.goal, dt)
        if self._lost_for > self.p.lost_stop_s:
            cmd = Command()  # never keep driving on a position we have not seen for a while
        centre = self.driver.centre(pos) if pos is not None else None
        return ClickState(cmd, status, px, pos, centre, self.driver.theta, self.goal)
