"""Render a synthetic overhead camera frame (markers + coloured target) from known world poses.

Used by tests and the offline dry run: it lets the whole camera -> planner -> bots loop run with
no hardware. The keystoned world->image mapping mimics a slightly tilted camera. It proves the
geometry and the control loop; it says nothing about real lighting, glare or print quality.
"""
from __future__ import annotations

import math

import cv2
import numpy as np

from ..geometry import Pose
from ..planner import Arena
from .markers import marker_image
from .target import PRESETS

BOT_MARKER_M = 0.06


class SyntheticCamera:
    def __init__(self, arena: Arena, px_per_m: float = 1000.0, margin_px: int = 150, keystone: float = 0.04):
        w, h = arena.width * px_per_m, arena.height * px_per_m
        self.size = (int(w + 2 * margin_px), int(h + 2 * margin_px))  # (width, height) of the frame
        k = keystone * w  # top edge is narrower, as if the camera were tilted slightly
        world = np.float32([[0, 0], [arena.width, 0], [arena.width, arena.height], [0, arena.height]])
        self.image_corners = np.float32([[margin_px, margin_px + h], [margin_px + w, margin_px + h],
                                         [margin_px + w - k, margin_px], [margin_px + k, margin_px]])
        self.H_w2i = cv2.getPerspectiveTransform(world, self.image_corners)
        self.px_per_m = px_per_m

    def to_img(self, pts) -> np.ndarray:
        return cv2.perspectiveTransform(np.float32(pts).reshape(-1, 1, 2), self.H_w2i).reshape(-1, 2)

    def _paint_marker(self, frame: np.ndarray, marker_id: int, p: Pose, size_m: float) -> np.ndarray:
        u = np.array([math.cos(p.theta), math.sin(p.theta)])
        r = np.array([math.sin(p.theta), -math.cos(p.theta)])
        c, h = np.array([p.x, p.y]), size_m / 2
        world = [c + u * h - r * h, c + u * h + r * h, c - u * h + r * h, c - u * h - r * h]  # TL TR BR BL
        img = marker_image(marker_id, size_m * 1000)
        cell = img.shape[0] // 8
        src = np.float32([[cell, cell], [7 * cell, cell], [7 * cell, 7 * cell], [cell, 7 * cell]])
        m = cv2.getPerspectiveTransform(src, np.float32(self.to_img(world)))
        warped = cv2.warpPerspective(img, m, self.size, borderValue=255)
        return np.minimum(frame, warped)

    def render(self, bots: dict[int, Pose], target: tuple[float, float] | None = None,
               target_color: str = "red") -> np.ndarray:
        frame = np.full((self.size[1], self.size[0], 3), 235, np.uint8)  # light-grey floor
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        for bid, p in bots.items():
            gray = self._paint_marker(gray, bid, p, BOT_MARKER_M)
        frame = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        if target is not None:
            hsv_lo, hsv_hi = PRESETS[target_color][0]
            hue = (hsv_lo[0] + hsv_hi[0]) // 2
            bgr = cv2.cvtColor(np.uint8([[[hue, 230, 230]]]), cv2.COLOR_HSV2BGR)[0, 0].tolist()
            cx, cy = self.to_img([target])[0]
            cv2.circle(frame, (int(cx), int(cy)), max(4, int(0.03 * self.px_per_m)), bgr, -1)
        return frame

    def render_leds(self, bots: dict[int, Pose], offset: float = 0.06,
                    dim_distractors: list[tuple[float, float]] = (), big_red_object: tuple[float, float] | None = None) -> np.ndarray:
        """A marker-free scene: each bot shows only its red power LED, `offset` m BEHIND its centre.

        dim_distractors: dull red blobs (a red cloth, a reflection) that must NOT be taken for the LED.
        big_red_object: a large saturated red thing (a bright red box) that must be rejected by size.
        """
        frame = np.full((self.size[1], self.size[0], 3), 235, np.uint8)
        for x, y in dim_distractors:
            cx, cy = self.to_img([(x, y)])[0]
            cv2.circle(frame, (int(cx), int(cy)), 7, (60, 60, 175), -1)
        if big_red_object is not None:
            cx, cy = self.to_img([big_red_object])[0]
            cv2.circle(frame, (int(cx), int(cy)), 45, (30, 30, 255), -1)
        for p in bots.values():
            cx, cy = self.to_img([(p.x - offset * math.cos(p.theta), p.y - offset * math.sin(p.theta))])[0]
            cv2.circle(frame, (int(round(cx)), int(round(cy))), 5, (40, 40, 255), -1)   # red halo
            cv2.circle(frame, (int(round(cx)), int(round(cy))), 2, (225, 225, 255), -1)  # saturated near-white core
        return frame
