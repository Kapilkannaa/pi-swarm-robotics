"""Pose and angle helpers. World frame: metres, x right, y up, theta counter-clockwise from +x.

OpenCV image coordinates have y pointing down; the vision module must convert once, at
the boundary, so nothing else has to think about it.
"""
from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class Pose:
    x: float
    y: float
    theta: float


def wrap_angle(a: float) -> float:
    """Wrap to (-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


def angle_diff(target: float, current: float) -> float:
    """Signed shortest rotation from current to target (positive = turn left)."""
    return wrap_angle(target - current)


def dist(a: tuple[float, float], b: tuple[float, float]) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def bearing(src: tuple[float, float], dst: tuple[float, float]) -> float:
    return math.atan2(dst[1] - src[1], dst[0] - src[0])


def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))
