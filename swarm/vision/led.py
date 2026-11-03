"""Find the bot by the red power LED on its motor driver: a tiny, very bright red dot at the bot's rear.

A phone camera usually saturates the middle of such an LED to white, leaving a red halo, so the test is
"bright AND clearly more red than green/blue", and the centroid is taken over the FILLED blob (halo plus core).
Everything here is tunable at run time (scripts/click_go.py has keys for it) because lighting decides.
"""
from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np


@dataclass
class LedParams:
    r_min: int = 170          # red channel at least this bright
    redness_min: int = 60     # red must exceed both green and blue by this much (the halo)
    v_min: int = 190          # brightness (max channel) at least this
    min_area_px: float = 2.0
    max_area_px: float = 900.0  # anything bigger is not an LED (a shirt, a red object)
    max_jump_px: float = 120.0  # when tracking, ignore blobs this far from the last position


def led_mask(frame: np.ndarray, p: LedParams, roi: np.ndarray | None = None) -> np.ndarray:
    b, g, r = (frame[..., k].astype(np.int16) for k in range(3))
    m = (r >= p.r_min) & (r - np.maximum(g, b) >= p.redness_min) & (np.maximum(np.maximum(r, g), b) >= p.v_min)
    mask = m.astype(np.uint8) * 255
    if roi is not None:
        mask = cv2.bitwise_and(mask, roi)
    return cv2.morphologyEx(mask, cv2.MORPH_CLOSE, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3)))


def find_led(frame: np.ndarray, p: LedParams | None = None, roi: np.ndarray | None = None,
             near: tuple[float, float] | None = None) -> tuple[float, float] | None:
    """Pixel centre of the LED, or None. `near` (the previous position) disambiguates when several red blobs exist."""
    p = p or LedParams()
    contours, _ = cv2.findContours(led_mask(frame, p, roi), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cands: list[tuple[float, float, float]] = []  # (x, y, area)
    for c in contours:
        area = cv2.contourArea(c)
        area = max(area, 1.0) if len(c) < 3 else area  # a 1-2 px blob has ~0 polygon area: still a valid dot
        if not (p.min_area_px <= area <= p.max_area_px):
            continue
        filled = np.zeros(frame.shape[:2], np.uint8)
        cv2.drawContours(filled, [c], -1, 255, -1)
        m = cv2.moments(filled, binaryImage=True)
        if m["m00"] > 0:
            cands.append((m["m10"] / m["m00"], m["m01"] / m["m00"], area))
    if near is not None:
        close = [c for c in cands if np.hypot(c[0] - near[0], c[1] - near[1]) <= p.max_jump_px]
        if close:
            best = min(close, key=lambda c: np.hypot(c[0] - near[0], c[1] - near[1]))
            return best[0], best[1]
    if not cands:
        return None
    best = max(cands, key=lambda c: c[2])
    return best[0], best[1]


def arena_roi(shape: tuple[int, int], image_corners: list[tuple[float, float]], grow_px: int = 25) -> np.ndarray:
    """Mask of the arena in the image (plus a margin), so red things outside it are ignored."""
    roi = np.zeros(shape[:2], np.uint8)
    pts = np.array(image_corners, dtype=np.int32)
    cv2.fillConvexPoly(roi, cv2.convexHull(pts), 255)
    return cv2.dilate(roi, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * grow_px + 1, 2 * grow_px + 1)))
