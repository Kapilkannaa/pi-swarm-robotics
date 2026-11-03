"""Printable ArUco marker sheets at an exact physical size.

Print the PDF at 100% / "actual size" (NOT "fit to page"), then check the 50 mm scale bar with a ruler.
A wrong print scale would silently scale every position the camera reports, so verify it.
"""
from __future__ import annotations

import cv2
import numpy as np
from PIL import Image

from .tracker import DICTIONARY

DPI = 300
A4_MM = (210, 297)


def mm_to_px(mm: float) -> int:
    return round(mm / 25.4 * DPI)


def marker_image(marker_id: int, size_mm: float) -> np.ndarray:
    """Black/white marker with a white quiet zone of one cell on every side (needed for detection)."""
    side = mm_to_px(size_mm)
    cell = side // 6  # 4x4 payload + 1-cell black border each side = 6 cells
    inner = cell * 6
    img = cv2.aruco.generateImageMarker(cv2.aruco.getPredefinedDictionary(DICTIONARY), marker_id, inner)
    return cv2.copyMakeBorder(img, cell, cell, cell, cell, cv2.BORDER_CONSTANT, value=255)


def build_sheet(bot_ids: list[int], corner_ids: list[int], bot_mm: float = 60, corner_mm: float = 55) -> Image.Image:
    """One A4 page: bot markers on top, corner markers below, plus a scale bar."""
    w, h = mm_to_px(A4_MM[0]), mm_to_px(A4_MM[1])
    page = np.full((h, w), 255, np.uint8)
    margin = mm_to_px(10)

    def place(ids: list[int], size_mm: float, y: int) -> int:
        x, row_h = margin, 0
        for mid in ids:
            img = marker_image(mid, size_mm)
            ih, iw = img.shape
            if x + iw > w - margin:
                x, y = margin, y + row_h + mm_to_px(8)
                row_h = 0
            if y + ih + 40 > h - margin:
                raise ValueError("markers do not fit on one A4 page: reduce --bot-mm/--corner-mm")
            page[y:y + ih, x:x + iw] = img
            cv2.putText(page, f"ID {mid}  {size_mm:g} mm (+quiet zone)", (x, y + ih + 24), cv2.FONT_HERSHEY_SIMPLEX,
                        0.7, 0, 2)
            x += iw + mm_to_px(8)
            row_h = max(row_h, ih + 40)
        return y + row_h + mm_to_px(8)

    cv2.putText(page, "BOT markers: cut out, stick on bot with the TOP edge toward the bot's FRONT", (margin, margin - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, 0, 2)
    y = place(bot_ids, bot_mm, margin + 20)
    cv2.putText(page, "CORNER markers: tape flat on the arena floor, one per corner", (margin, y + 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, 0, 2)
    y = place(corner_ids, corner_mm, y + 30)
    bar = mm_to_px(50)
    cv2.line(page, (margin, h - margin), (margin + bar, h - margin), 0, 6)
    cv2.putText(page, "this bar must measure 50 mm (print at 100%)", (margin, h - margin - 16),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, 0, 2)
    return Image.fromarray(page)
