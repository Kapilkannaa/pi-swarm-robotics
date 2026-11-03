"""Image measurements only. Target classification happens on the Raspberry Pi."""
import math

import cv2
import numpy as np

from .target import color_mask


def contour_features(frame, contour):
    area = cv2.contourArea(contour)
    perimeter = cv2.arcLength(contour, True)
    _, _, w, h = cv2.boundingRect(contour)
    mask = np.zeros(frame.shape[:2], np.uint8)
    cv2.drawContours(mask, [contour], -1, 255, -1)
    b, g, r, _ = cv2.mean(frame, mask=mask)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    _, s, v, _ = cv2.mean(hsv, mask=mask)
    return [max(-1.0, (r - max(g, b)) / 255), s / 255, v / 255,
            min(1.0, 4 * math.pi * area / max(1, perimeter * perimeter)),
            area / max(1, w * h), min(w, h) / max(w, h)]


def target_candidate(frame, tracker):
    contours, _ = cv2.findContours(color_mask(frame, "red"), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for contour in sorted(contours, key=cv2.contourArea, reverse=True):
        if cv2.contourArea(contour) < 80:
            continue
        m = cv2.moments(contour)
        xy = tracker.pixel_to_world((m["m10"] / m["m00"], m["m01"] / m["m00"]))
        if 0 <= xy[0] <= tracker.arena.width and 0 <= xy[1] <= tracker.arena.height:
            return xy, contour_features(frame, contour)
    return None, None
