"""Overhead-camera tracking: ArUco markers -> bot poses in arena metres.

Pipeline: four fixed corner markers define a homography (image pixels -> arena metres), so every
bot marker's corners can be mapped to the floor plane. Pose = marker centre + heading, where
heading is the direction from the marker centre toward its TOP edge. Stick each marker on its bot
with the top edge pointing at the bot's front (away from the caster).

Coordinates follow swarm/geometry.py: metres, x right, y up, theta counter-clockwise from +x.
The image has y pointing down; the homography absorbs that flip, nothing else needs to care.
"""
from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np

from ..geometry import Pose
from ..planner import Arena

DICTIONARY = cv2.aruco.DICT_4X4_50

# Corner marker IDs sit well away from bot IDs (1, 2, 3...). Centres are inset from the arena
# corner by `inset` so the whole marker lies inside the floor area.
CORNER_IDS = {10: "bottom-left", 11: "bottom-right", 12: "top-right", 13: "top-left"}


def corner_positions(arena: Arena, inset: float) -> dict[int, tuple[float, float]]:
    return {10: (inset, inset), 11: (arena.width - inset, inset),
            12: (arena.width - inset, arena.height - inset), 13: (inset, arena.height - inset)}


@dataclass
class Marker:
    marker_id: int
    corners: np.ndarray  # 4x2 pixel coordinates, order TL, TR, BR, BL in the marker's own frame


def detect_markers(frame: np.ndarray) -> dict[int, Marker]:
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if frame.ndim == 3 else frame
    detector = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(DICTIONARY),
                                       cv2.aruco.DetectorParameters())
    corners, ids, _ = detector.detectMarkers(gray)
    if ids is None:
        return {}
    return {int(i): Marker(int(i), c.reshape(4, 2)) for i, c in zip(ids.flatten(), corners)}


class ArenaTracker:
    def __init__(self, arena: Arena, inset: float = 0.06, bot_ids: list[int] | None = None):
        self.arena = arena
        self.inset = inset
        self.bot_ids = bot_ids
        self.H: np.ndarray | None = None  # image px -> arena metres

    # ------------------------------------------------------------ calibration
    def calibrate(self, frame: np.ndarray) -> bool:
        """Fit the homography from the four corner markers' centres. False if any is missing."""
        found = detect_markers(frame)
        world = corner_positions(self.arena, self.inset)
        if not all(i in found for i in world):
            return False
        src = np.array([found[i].corners.mean(axis=0) for i in world], dtype=np.float32)
        dst = np.array([world[i] for i in world], dtype=np.float32)
        self.H = cv2.getPerspectiveTransform(src, dst)
        return True

    def calibrate_from_points(self, image_pts: list[tuple[float, float]]) -> None:
        """Calibrate from four clicked image points: arena corners BL, BR, TR, TL (in that order).

        Replaces the corner markers when you cannot print them. Click the true floor corners of
        the arena (the inside corners of whatever marks it), not the objects' tops.
        """
        if len(image_pts) != 4:
            raise ValueError("need exactly four points: bottom-left, bottom-right, top-right, top-left")
        src = np.array(image_pts, dtype=np.float32)
        if (not np.isfinite(src).all() or not cv2.isContourConvex(src.reshape(-1, 1, 2))
                or abs(cv2.contourArea(src)) < 64):
            raise ValueError("arena corners must form a finite, ordered, non-degenerate quadrilateral")
        dst = np.array([(0, 0), (self.arena.width, 0), (self.arena.width, self.arena.height), (0, self.arena.height)],
                       dtype=np.float32)
        self.H = cv2.getPerspectiveTransform(src, dst)

    def pixel_to_world(self, px: tuple[float, float]) -> tuple[float, float]:
        x, y = self.to_world(np.array([px], dtype=np.float32))[0]
        return float(x), float(y)

    def world_to_pixel(self, xy: tuple[float, float]) -> tuple[int, int]:
        assert self.H is not None
        out = cv2.perspectiveTransform(np.array([[xy]], dtype=np.float32), np.linalg.inv(self.H))
        return int(out[0, 0, 0]), int(out[0, 0, 1])

    def save(self, path: str | Path) -> None:
        assert self.H is not None
        Path(path).write_text(json.dumps({"H": self.H.tolist(), "arena": [self.arena.width, self.arena.height],
                                          "inset": self.inset}))

    def load(self, path: str | Path) -> None:
        d = json.loads(Path(path).read_text())
        h = np.array(d["H"], dtype=np.float64)
        if d.get("arena") != [self.arena.width, self.arena.height]:
            raise ValueError("saved calibration arena differs from config; recalibrate")
        if h.shape != (3, 3) or not np.isfinite(h).all() or abs(np.linalg.det(h)) < 1e-12:
            raise ValueError("invalid saved calibration; recalibrate")
        self.H = h

    # ----------------------------------------------------------------- poses
    def to_world(self, pts: np.ndarray) -> np.ndarray:
        assert self.H is not None, "calibrate() or load() first"
        out = cv2.perspectiveTransform(pts.reshape(-1, 1, 2).astype(np.float32), self.H)
        return out.reshape(-1, 2)

    def poses(self, frame: np.ndarray) -> dict[int, Pose]:
        """Pose for every visible bot marker (corner markers excluded)."""
        result: dict[int, Pose] = {}
        for mid, m in detect_markers(frame).items():
            if mid in CORNER_IDS or (self.bot_ids is not None and mid not in self.bot_ids):
                continue
            c = self.to_world(m.corners)  # TL, TR, BR, BL on the floor, in metres
            centre = c.mean(axis=0)
            front = (c[0] + c[1]) / 2 - centre
            result[mid] = Pose(float(centre[0]), float(centre[1]), math.atan2(front[1], front[0]))
        return result


def draw_overlay(frame: np.ndarray, poses: dict[int, Pose], tracker: ArenaTracker) -> np.ndarray:
    """Annotate a frame: marker outlines plus each bot's pose in metres and degrees."""
    out = frame.copy()
    for mid, m in detect_markers(frame).items():
        cv2.polylines(out, [m.corners.astype(np.int32)], True, (0, 255, 0), 2)
        label = CORNER_IDS.get(mid)
        if label:
            cv2.putText(out, f"corner {mid}", tuple(m.corners[0].astype(int)), cv2.FONT_HERSHEY_SIMPLEX,
                        0.5, (255, 200, 0), 1)
        elif mid in poses:
            p = poses[mid]
            cx, cy = m.corners.mean(axis=0)
            tip = (m.corners[0] + m.corners[1]) / 2
            cv2.arrowedLine(out, (int(cx), int(cy)), (int(tip[0]), int(tip[1])), (0, 0, 255), 3)
            cv2.putText(out, f"bot {mid}: ({p.x:.2f},{p.y:.2f})m {math.degrees(p.theta):.0f}deg",
                        (int(cx) - 60, int(cy) - 40), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 2)
    return out


def click_corners(frame: np.ndarray, window: str = "click the 4 arena corners") -> list[tuple[float, float]]:
    """Let the user click bottom-left, bottom-right, top-right, top-left (right-click undoes). Enter/q to abort."""
    pts: list[tuple[float, float]] = []
    labels = ["bottom-left", "bottom-right", "top-right", "top-left"]

    def on_mouse(event: int, x: int, y: int, *_: object) -> None:
        if event == cv2.EVENT_LBUTTONDOWN and len(pts) < 4:
            pts.append((float(x), float(y)))
        elif event == cv2.EVENT_RBUTTONDOWN and pts:
            pts.pop()

    cv2.namedWindow(window)
    cv2.setMouseCallback(window, on_mouse)
    while len(pts) < 4:
        view = frame.copy()
        for i, (x, y) in enumerate(pts):
            cv2.circle(view, (int(x), int(y)), 6, (0, 0, 255), -1)
            cv2.putText(view, labels[i], (int(x) + 8, int(y)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        nxt = labels[len(pts)]
        cv2.putText(view, f"click {nxt} corner ({len(pts)}/4); right-click = undo; q = abort", (10, 28),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        cv2.imshow(window, view)
        if cv2.waitKey(30) & 0xFF == ord("q"):
            cv2.destroyWindow(window)
            raise KeyboardInterrupt("calibration aborted")
    cv2.destroyWindow(window)
    return pts
