"""Search planning: split the arena into sectors, sweep each, and pick converge slots.

Pure functions, no I/O, so they are unit-testable and identical in sim and on hardware.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from itertools import permutations

from .geometry import Pose, bearing, dist


@dataclass(frozen=True)
class Arena:
    width: float
    height: float


@dataclass(frozen=True)
class Sector:
    x0: float
    y0: float
    x1: float
    y1: float

    @property
    def centre(self) -> tuple[float, float]:
        return ((self.x0 + self.x1) / 2, (self.y0 + self.y1) / 2)


def split_sectors(arena: Arena, n: int) -> list[Sector]:
    """N equal strips along the longer axis, so each strip is as close to square as possible."""
    if n < 1:
        raise ValueError("need at least one sector")
    if arena.width >= arena.height:
        w = arena.width / n
        return [Sector(i * w, 0.0, (i + 1) * w, arena.height) for i in range(n)]
    h = arena.height / n
    return [Sector(0.0, i * h, arena.width, (i + 1) * h) for i in range(n)]


def lawnmower(sector: Sector, spacing: float, margin: float) -> list[tuple[float, float]]:
    """Back-and-forth sweep. Lane spacing should be <= 2x detection radius or gaps appear.

    The margin keeps waypoints off the walls where the bot would scrape or leave the
    camera's reliable region.
    """
    x0, x1 = sector.x0 + margin, sector.x1 - margin
    y0, y1 = sector.y0 + margin, sector.y1 - margin
    if x1 < x0 or y1 < y0:
        return [sector.centre]
    # Lanes run along y; use however many lanes keep the gap at or below `spacing`.
    n_lanes = max(1, math.ceil((x1 - x0) / spacing) + 1) if x1 > x0 else 1
    xs = [x0 + (x1 - x0) * i / (n_lanes - 1) for i in range(n_lanes)] if n_lanes > 1 else [(x0 + x1) / 2]
    pts: list[tuple[float, float]] = []
    for i, x in enumerate(xs):
        ys = (y0, y1) if i % 2 == 0 else (y1, y0)
        pts.extend([(x, ys[0]), (x, ys[1])])
    return pts


def orient_path(path: list[tuple[float, float]], start: tuple[float, float]) -> list[tuple[float, float]]:
    """Start from whichever end of the sweep is nearer the bot."""
    if len(path) > 1 and dist(start, path[-1]) < dist(start, path[0]):
        return path[::-1]
    return path


def _best_assignment(ids: list[int], targets: list[tuple[float, float]],
                     pos: dict[int, tuple[float, float]]) -> dict[int, int]:
    """Min-total-distance matching of bots to targets by brute force (N is tiny)."""
    best: tuple[float, tuple[int, ...]] | None = None
    for perm in permutations(range(len(targets)), len(ids)):
        cost = sum(dist(pos[b], targets[t]) for b, t in zip(ids, perm))
        if best is None or cost < best[0]:
            best = (cost, perm)
    assert best is not None
    return dict(zip(ids, best[1]))


def assign_sectors(poses: dict[int, Pose], sectors: list[Sector]) -> dict[int, Sector]:
    ids = sorted(poses)
    pos = {i: (poses[i].x, poses[i].y) for i in ids}
    m = _best_assignment(ids, [s.centre for s in sectors], pos)
    return {i: sectors[m[i]] for i in ids}


def converge_slots(target: tuple[float, float], poses: dict[int, Pose],
                   standoff: float) -> dict[int, tuple[float, float]]:
    """Evenly spaced points on a ring around the target, one per bot.

    Distinct slots stop the bots driving into each other at the target. The ring is rotated
    so the closest bot's slot lies on its own approach line (no wasted detour).
    """
    ids = sorted(poses)
    n = len(ids)
    nearest = min(ids, key=lambda i: dist((poses[i].x, poses[i].y), target))
    base = bearing(target, (poses[nearest].x, poses[nearest].y))
    slots = [(target[0] + standoff * math.cos(base + 2 * math.pi * k / n),
              target[1] + standoff * math.sin(base + 2 * math.pi * k / n)) for k in range(n)]
    pos = {i: (poses[i].x, poses[i].y) for i in ids}
    m = _best_assignment(ids, slots, pos)
    return {i: slots[m[i]] for i in ids}


def avoid(pos: tuple[float, float], goal: tuple[float, float],
          others: list[tuple[float, float]], clearance: float, sep: float = 0.0) -> tuple[float, float]:
    """Return the point to steer at: the goal, or a way around a bot that is in the way.

    Two layers, both re-evaluated every tick (a tiny swarm needs nothing fancier):
      * separation: if a neighbour is closer than `sep` and the goal lies toward it, back away from it first. A side-step
        needs room, and a bot that starts a trip already touching a neighbour has none.
      * detour: if a neighbour sits on the straight line to the goal, step around it on the side we are already on.
    """
    gx, gy = goal[0] - pos[0], goal[1] - pos[1]
    length = math.hypot(gx, gy)
    if length < 1e-6:
        return goal
    ux, uy = gx / length, gy / length
    if sep > 0:
        nearest = min(others, key=lambda o: math.hypot(o[0] - pos[0], o[1] - pos[1]), default=None)
        if nearest is not None:
            dx, dy = nearest[0] - pos[0], nearest[1] - pos[1]
            d = math.hypot(dx, dy)
            if 1e-6 < d < sep and (dx * ux + dy * uy) / d > 0.0:  # too close, and the goal is toward it
                return (pos[0] - dx / d * clearance, pos[1] - dy / d * clearance)
    worst: tuple[float, float, float] | None = None  # (along, signed lateral, |lateral|)
    for ox, oy in others:
        dx, dy = ox - pos[0], oy - pos[1]
        along = dx * ux + dy * uy
        lateral = -dx * uy + dy * ux
        # Ignore obstacles behind us or beyond the goal (a bot parked at/after the goal is
        # the converge plan's business, not a path obstacle).
        if 0.0 < along < length - clearance * 0.5 and abs(lateral) < clearance:
            if worst is None or along < worst[0]:
                worst = (along, lateral, abs(lateral))
    if worst is None:
        return goal
    along, lateral, _ = worst
    side = 1.0 if lateral <= 0 else -1.0  # pass on the side we are already on
    return (pos[0] + ux * along - uy * side * clearance * 1.4,
            pos[1] + uy * along + ux * side * clearance * 1.4)
