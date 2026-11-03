"""Closed-loop dry run: synthetic camera -> tracker -> swarm -> UDP packets -> simulated bots -> back to camera."""
import math

import pytest

cv2 = pytest.importorskip("cv2")
np = pytest.importorskip("numpy")

from swarm import protocol  # noqa: E402
from swarm.geometry import Pose  # noqa: E402
from swarm.live import LiveSystem, laser_ray_miss  # noqa: E402
from swarm.planner import Arena  # noqa: E402
from swarm.sim.run import DT  # noqa: E402
from swarm.sim.world import SimParams, make_world  # noqa: E402
from swarm.swarm import Phase, Swarm, SwarmParams  # noqa: E402
from swarm.vision.synth import SyntheticCamera  # noqa: E402
from swarm.vision.target import find_target  # noqa: E402
from swarm.vision.tracker import ArenaTracker  # noqa: E402

ARENA = Arena(1.00, 0.75)


def test_target_found_in_synthetic_frame_with_correct_world_position():
    cam = SyntheticCamera(ARENA)
    tr = ArenaTracker(ARENA)
    tr.calibrate_from_points([tuple(p) for p in cam.image_corners])
    frame = cam.render({}, target=(0.62, 0.41))
    px = find_target(frame, "red")
    assert px is not None
    x, y = tr.pixel_to_world(px)
    assert math.hypot(x - 0.62, y - 0.41) < 0.01
    assert find_target(cam.render({}), "red") is None  # no target in the scene


def test_click_calibration_needs_four_points():
    with pytest.raises(ValueError):
        ArenaTracker(ARENA).calibrate_from_points([(0, 0)] * 3)


def _closed_loop(seed: int, n_bots: int = 2, timeout_s: float = 150.0):
    w = make_world(seed, n_bots, arena=ARENA, params=SimParams(detect_radius=0.20))
    cam = SyntheticCamera(ARENA)
    tr = ArenaTracker(ARENA)
    tr.calibrate_from_points([tuple(p) for p in cam.image_corners])
    sw = Swarm(ARENA, sorted(w.bots), SwarmParams(detect_radius=0.20))
    live = LiveSystem(tr, sw, "red", 0.20)
    seq, shooter_ray_miss = 0, None
    while w.t < timeout_s and sw.phase is not Phase.DONE:
        frame = cam.render({i: b.pose for i, b in w.bots.items()}, target=w.target)
        st = live.step(frame, w.t)
        seq += 1
        for i, c in st.cmds.items():
            w.send(i, protocol.encode(i, seq, c))
        if any(c.laser for c in st.cmds.values()):
            for i, c in st.cmds.items():
                if c.laser:
                    shooter_ray_miss = laser_ray_miss(w.bots[i].pose, w.target)
        w.step(DT)
    return w, sw, shooter_ray_miss


@pytest.mark.parametrize("seed", [1, 2, 3, 4, 5, 6])
def test_full_camera_in_the_loop_marks_target(seed):
    w, sw, miss = _closed_loop(seed)
    assert sw.phase is Phase.DONE, f"timed out in {sw.phase} at t={w.t:.0f}s"
    assert w.marked, f"laser missed the target by {miss}"
    assert w.collisions == 0


def test_detection_is_gated_by_sensing_radius():
    cam = SyntheticCamera(ARENA)
    tr = ArenaTracker(ARENA)
    tr.calibrate_from_points([tuple(p) for p in cam.image_corners])
    sw = Swarm(ARENA, [1])
    live = LiveSystem(tr, sw, "red", 0.20)
    far = live.step(cam.render({1: Pose(0.1, 0.1, 0.0)}, target=(0.9, 0.6)), 0.0)
    assert far.target_seen is not None and far.detection is None and far.phase is Phase.SEARCH
    near = live.step(cam.render({1: Pose(0.85, 0.55, 0.0)}, target=(0.9, 0.6)), 0.05)
    assert near.detection is not None and near.phase is Phase.CONVERGE


def test_target_outside_arena_is_ignored():
    cam = SyntheticCamera(ARENA)
    tr = ArenaTracker(ARENA)
    tr.calibrate_from_points([tuple(p) for p in cam.image_corners])
    live = LiveSystem(tr, Swarm(ARENA, [1]), "red", 0.2)
    st = live.step(cam.render({1: Pose(0.5, 0.4, 0.0)}, target=(1.15, 0.4)), 0.0)  # 15 cm past the wall
    assert st.target_seen is None


def test_paused_system_does_not_plan_or_learn():
    cam = SyntheticCamera(ARENA)
    tr = ArenaTracker(ARENA)
    tr.calibrate_from_points([tuple(p) for p in cam.image_corners])
    sw = Swarm(ARENA, [1, 2])
    live = LiveSystem(tr, sw, "red", 0.2)
    frame = cam.render({1: Pose(0.3, 0.2, 2.0), 2: Pose(0.7, 0.2, 1.0)}, target=(0.5, 0.5))
    for k in range(200):
        st = live.step(frame, k * 0.05, run=False)
    assert sw._t_prev is None and not sw.paths and sw.phase is Phase.SEARCH
    assert all(c.left == 0 and c.right == 0 for c in st.cmds.values())


def test_bot_outside_the_arena_is_stopped():
    cam = SyntheticCamera(ARENA)
    tr = ArenaTracker(ARENA)
    tr.calibrate_from_points([tuple(p) for p in cam.image_corners])
    live = LiveSystem(tr, Swarm(ARENA, [1, 2]), "red", 0.2, oob_margin=0.05)
    frame = cam.render({1: Pose(0.3, 0.2, 1.57), 2: Pose(1.10, 0.3, 1.57)})   # bot 2 is 10 cm past the wall (still in frame)
    st = live.step(frame, 0.0)
    assert 2 in st.out_of_bounds and st.cmds[2].left == 0 and st.cmds[2].right == 0


@pytest.mark.parametrize("bgr", [(40, 120, 230), (60, 200, 60), (230, 120, 40), (30, 30, 220)])
def test_sampled_colour_finds_the_object_and_ignores_the_floor(bgr, tmp_path):
    from swarm.vision.target import load_ranges, sample_ranges, save_ranges
    floor = np.full((300, 400, 3), (170, 190, 200), np.uint8)       # beige-ish floor
    frame = floor.copy()
    cv2.circle(frame, (120, 90), 14, bgr, -1)
    ranges = sample_ranges(frame, (120, 90))
    save_ranges(tmp_path / "c.json", ranges)
    got = find_target(frame, load_ranges(tmp_path / "c.json"))
    assert got is not None and math.hypot(got[0] - 120, got[1] - 90) < 2
    assert find_target(floor, ranges) is None                        # same colour range, no object: nothing found


@pytest.mark.parametrize("seed", [11, 12, 13])
def test_single_bot_full_camera_loop_marks_target(seed):
    """Only one bot on the floor (the other is elsewhere): the same system plans for it alone and still marks."""
    w, sw, miss = _closed_loop(seed, n_bots=1)
    assert sw.phase is Phase.DONE and w.marked, f"phase {sw.phase}, t={w.t:.0f}s, miss {miss}"
