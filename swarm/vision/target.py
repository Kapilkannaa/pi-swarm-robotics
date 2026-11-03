"""Target detection by colour: the biggest blob of a bright single-colour object.

Colour thresholding in HSV is deliberately simple and explainable. It breaks under strong glare
or when the floor or bots share the target's colour, so pick a colour the scene does not contain.
"""
from __future__ import annotations

import json
from pathlib import Path

import cv2
import numpy as np

# (low, high) HSV ranges; OpenCV hue is 0-179. Red wraps around so it has two ranges.
PRESETS: dict[str, list[tuple[tuple[int, int, int], tuple[int, int, int]]]] = {
    "red": [((0, 120, 90), (10, 255, 255)), ((170, 120, 90), (179, 255, 255))],
    "orange": [((8, 150, 120), (20, 255, 255))],
    "yellow": [((20, 110, 120), (35, 255, 255))],
    "green": [((40, 80, 60), (85, 255, 255))],
    "blue": [((95, 120, 60), (130, 255, 255))],
}


Ranges = list[tuple[tuple[int, int, int], tuple[int, int, int]]]


def resolve(color: "str | Ranges") -> Ranges:
    """A preset name or an explicit list of (low, high) HSV ranges (e.g. sampled from the real camera)."""
    if isinstance(color, str):
        if color not in PRESETS:
            raise ValueError(f"unknown colour {color!r}; choose from {sorted(PRESETS)} or sample one with pick_target_color.py")
        return PRESETS[color]
    return color


def sample_ranges(frame: np.ndarray, centre: tuple[int, int], radius: int = 6,
                  hue_tol: int = 10, sat_tol: int = 70, val_tol: int = 80) -> Ranges:
    """HSV range around the median colour of a small patch the user clicked on the target.

    The saturation/value floors stay above ~40 so grey floor and shadows are never swallowed. Hue wraps at 0/179,
    so a reddish patch yields two ranges.
    """
    x, y = centre
    patch = cv2.cvtColor(frame[max(0, y - radius):y + radius + 1, max(0, x - radius):x + radius + 1], cv2.COLOR_BGR2HSV)
    h, s, v = (int(np.median(patch[..., k])) for k in range(3))
    s_lo, v_lo = max(40, s - sat_tol), max(40, v - val_tol)
    lo_h, hi_h = h - hue_tol, h + hue_tol
    if lo_h < 0:
        return [((0, s_lo, v_lo), (hi_h, 255, 255)), ((180 + lo_h, s_lo, v_lo), (179, 255, 255))]
    if hi_h > 179:
        return [((lo_h, s_lo, v_lo), (179, 255, 255)), ((0, s_lo, v_lo), (hi_h - 180, 255, 255))]
    return [((lo_h, s_lo, v_lo), (hi_h, 255, 255))]


def save_ranges(path: str | Path, ranges: Ranges) -> None:
    Path(path).write_text(json.dumps([[list(lo), list(hi)] for lo, hi in ranges]))


def load_ranges(path: str | Path) -> Ranges:
    return [(tuple(lo), tuple(hi)) for lo, hi in json.loads(Path(path).read_text())]


def color_mask(frame: np.ndarray, color: "str | Ranges") -> np.ndarray:
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = np.zeros(hsv.shape[:2], np.uint8)
    for lo, hi in resolve(color):
        mask |= cv2.inRange(hsv, np.array(lo), np.array(hi))
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k)  # drop speckle
    return cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)


def find_target(frame: np.ndarray, color: "str | Ranges", min_area_px: float = 80) -> tuple[float, float] | None:
    """Pixel centroid of the largest blob of `color`, or None if nothing big enough."""
    contours, _ = cv2.findContours(color_mask(frame, color), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    best = max(contours, key=cv2.contourArea)
    if cv2.contourArea(best) < min_area_px:
        return None
    m = cv2.moments(best)
    return (m["m10"] / m["m00"], m["m01"] / m["m00"])
