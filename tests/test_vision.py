"""Vision tests on synthetic frames: render markers with a known perspective, recover the poses.

This proves the geometry (homography, y-flip, heading convention); it does not prove anything
about the real camera, lighting or print quality: those need a real photo of the arena.
"""
import math

import pytest

cv2 = pytest.importorskip("cv2")
np = pytest.importorskip("numpy")

from swarm.geometry import Pose, angle_diff  # noqa: E402
from swarm.planner import Arena  # noqa: E402
from swarm.vision.markers import marker_image  # noqa: E402
from swarm.vision.tracker import ArenaTracker, corner_positions  # noqa: E402

ARENA = Arena(1.20, 0.90)
INSET = 0.06
CANVAS = (1400, 1100)  # w, h
# World rectangle -> a slightly keystoned quad in the image, y flipped (world y up, image y down).
_WORLD = np.float32([[0, 0], [1.2, 0], [1.2, 0.9], [0, 0.9]])
_IMAGE = np.float32([[150, 1000], [1250, 990], [1200, 120], [200, 130]])
H_W2I = cv2.getPerspectiveTransform(_WORLD, _IMAGE)


def to_img(pts):
    return cv2.perspectiveTransform(np.float32(pts).reshape(-1, 1, 2), H_W2I).reshape(-1, 2)


def draw(frame, marker_id, centre, theta, size_m):
    """Paint a marker whose top edge points along `theta` on the floor."""
    u = np.array([math.cos(theta), math.sin(theta)])
    r = np.array([math.sin(theta), -math.cos(theta)])
    c = np.array(centre)
    h = size_m / 2
    world = [c + u * h - r * h, c + u * h + r * h, c - u * h + r * h, c - u * h - r * h]  # TL TR BR BL
    img = marker_image(marker_id, size_m * 1000)
    cell = img.shape[0] // 8  # 6 cells of marker + 1-cell quiet zone each side
    src = np.float32([[cell, cell], [7 * cell, cell], [7 * cell, 7 * cell], [cell, 7 * cell]])
    m = cv2.getPerspectiveTransform(src, np.float32(to_img(world)))
    warped = cv2.warpPerspective(img, m, CANVAS, borderValue=255)
    return np.minimum(frame, warped)


def scene(bots: dict[int, Pose], corners=(10, 11, 12, 13)):
    frame = np.full((CANVAS[1], CANVAS[0]), 255, np.uint8)
    pos = corner_positions(ARENA, INSET)
    for cid in corners:
        frame = draw(frame, cid, pos[cid], 0.0, 0.08)
    for bid, p in bots.items():
        frame = draw(frame, bid, (p.x, p.y), p.theta, 0.06)
    return cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)


def test_recovers_poses_through_perspective():
    truth = {1: Pose(0.30, 0.25, math.radians(90)), 2: Pose(0.85, 0.60, math.radians(-30)),
             3: Pose(0.60, 0.45, math.radians(179))}
    frame = scene(truth)
    tr = ArenaTracker(ARENA, INSET)
    assert tr.calibrate(frame)
    got = tr.poses(frame)
    assert set(got) == {1, 2, 3}
    for i, p in truth.items():
        assert math.hypot(got[i].x - p.x, got[i].y - p.y) < 0.01, i
        assert abs(angle_diff(got[i].theta, p.theta)) < math.radians(3), i


@pytest.mark.parametrize("deg", [0, 45, 90, 135, 180, -135, -90, -45])
def test_heading_convention_all_directions(deg):
    truth = {1: Pose(0.5, 0.4, math.radians(deg))}
    frame = scene(truth)
    tr = ArenaTracker(ARENA, INSET)
    assert tr.calibrate(frame)
    assert abs(angle_diff(tr.poses(frame)[1].theta, truth[1].theta)) < math.radians(3)


def test_calibration_fails_if_a_corner_is_missing():
    frame = scene({}, corners=(10, 11, 12))
    assert not ArenaTracker(ARENA, INSET).calibrate(frame)


def test_bot_out_of_view_is_absent_and_corners_are_not_bots():
    frame = scene({1: Pose(0.3, 0.3, 0.0)})
    tr = ArenaTracker(ARENA, INSET)
    assert tr.calibrate(frame)
    assert set(tr.poses(frame)) == {1}


def test_calibration_roundtrips_through_file(tmp_path):
    frame = scene({1: Pose(0.3, 0.3, 1.0)})
    a = ArenaTracker(ARENA, INSET)
    assert a.calibrate(frame)
    a.save(tmp_path / "cal.json")
    b = ArenaTracker(ARENA, INSET)
    b.load(tmp_path / "cal.json")
    assert b.poses(frame)[1] == pytest.approx(a.poses(frame)[1], abs=1e-6)
