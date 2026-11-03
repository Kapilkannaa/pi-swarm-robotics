"""Click-to-go: find the bot by its red LED alone, learn its heading from motion, drive to a clicked point."""
import math
import random

import pytest

cv2 = pytest.importorskip("cv2")
np = pytest.importorskip("numpy")

from swarm import protocol  # noqa: E402
from swarm.clickgo import ClickGo, ClickParams  # noqa: E402
from swarm.geometry import Pose, angle_diff  # noqa: E402
from swarm.planner import Arena  # noqa: E402
from swarm.protocol import Command  # noqa: E402
from swarm.sim.firmware_model import FirmwareModel  # noqa: E402
from swarm.sim.run import DT  # noqa: E402
from swarm.sim.world import SimBot, SimParams, make_world  # noqa: E402
from swarm.vision.led import LedParams, find_led  # noqa: E402
from swarm.vision.synth import SyntheticCamera  # noqa: E402
from swarm.vision.tracker import ArenaTracker  # noqa: E402

ARENA = Arena(1.00, 0.75)
OFFSET = 0.06


# ----------------------------------------------------------------- LED finding
def test_led_found_at_the_right_place_in_the_world():
    cam = SyntheticCamera(ARENA)
    tr = ArenaTracker(ARENA)
    tr.calibrate_from_points([tuple(p) for p in cam.image_corners])
    pose = Pose(0.55, 0.40, math.radians(30))
    frame = cam.render_leds({1: pose}, OFFSET)
    px = find_led(frame)
    assert px is not None
    x, y = tr.pixel_to_world(px)
    want = (pose.x - OFFSET * math.cos(pose.theta), pose.y - OFFSET * math.sin(pose.theta))
    assert math.hypot(x - want[0], y - want[1]) < 0.01


def test_led_ignores_dim_red_things_and_big_red_objects():
    cam = SyntheticCamera(ARENA)
    frame = cam.render_leds({1: Pose(0.3, 0.3, 0.0)}, OFFSET, dim_distractors=[(0.7, 0.5), (0.2, 0.6)],
                            big_red_object=(0.8, 0.2))
    px = find_led(frame)
    assert px is not None
    clean = find_led(cam.render_leds({1: Pose(0.3, 0.3, 0.0)}, OFFSET))
    assert math.hypot(px[0] - clean[0], px[1] - clean[1]) < 1.0           # still the real LED
    assert find_led(cam.render_leds({}, OFFSET, dim_distractors=[(0.5, 0.4)])) is None   # nothing bright: nothing found


def test_led_tracking_prefers_the_blob_near_the_last_position():
    cam = SyntheticCamera(ARENA)
    frame = cam.render_leds({1: Pose(0.3, 0.3, 0.0), 2: Pose(0.8, 0.5, 0.0)}, OFFSET)
    near_second = cam.to_img([(0.8 - OFFSET, 0.5)])[0]
    px = find_led(frame, near=(float(near_second[0]), float(near_second[1])))
    assert px is not None and math.hypot(px[0] - near_second[0], px[1] - near_second[1]) < 2.0


# --------------------------------------------------------------- click and go
def _world(seed, theta0, gain_r=1.0, **sim_kw):
    w = make_world(seed, 1, ARENA, SimParams(gain_spread=0, packet_loss=0.02, pos_noise=0.002, heading_noise=0.0, **sim_kw))
    w.bots[1] = SimBot(1, Pose(0.25, 0.2, theta0), 1.0, gain_r, FirmwareModel(1))
    return w


def _drive(w, cg, cam, goal, seconds=120.0):
    """Click `goal`, run the closed loop (camera sees ONLY the LED), return (arrived, final true centre error, steps)."""
    px = cam.to_img([goal])[0]
    cg.set_goal_pixel((float(px[0]), float(px[1])))
    seq, t0 = 0, w.t
    while w.t - t0 < seconds:
        frame = cam.render_leds({1: w.bots[1].pose}, OFFSET)
        st = cg.step(frame, DT)
        seq += 1
        w.send(1, protocol.encode(1, seq, st.cmd))
        w.step(DT)
        if st.status == "arrived":
            break
    p = w.bots[1].pose
    return st.status == "arrived", math.hypot(p.x - cg.goal[0], p.y - cg.goal[1])


def _rig(w_lag=0.0):
    cam = SyntheticCamera(ARENA)
    tr = ArenaTracker(ARENA)
    tr.calibrate_from_points([tuple(p) for p in cam.image_corners])
    return cam, tr


@pytest.mark.parametrize("theta0", [0.0, 1.5, 3.0, -2.0])
@pytest.mark.parametrize("lag", [0.0, 0.3])
def test_reaches_the_clicked_point_whatever_way_the_bot_faces_at_the_start(theta0, lag):
    cam, tr = _rig()
    w = _world(7, theta0, camera_latency_s=lag)
    cg = ClickGo(tr, ARENA, ClickParams(latency_s=lag + 0.1, led_offset=OFFSET))
    ok, err = _drive(w, cg, cam, (0.80, 0.55))
    assert ok and err < 0.07, (ok, err)


def test_visits_several_clicks_in_a_row_and_learns_heading_speed_and_turn_rate():
    cam, tr = _rig()
    w = _world(3, 2.2, gain_r=0.9, camera_latency_s=0.3, deadband=40)       # weak wheel, stiction above min_cmd
    cg = ClickGo(tr, ARENA, ClickParams(latency_s=0.4, led_offset=OFFSET, omega0=0.5, speed0=0.03))  # poor first guesses
    for goal in [(0.8, 0.2), (0.8, 0.6), (0.2, 0.6), (0.5, 0.35)]:
        ok, err = _drive(w, cg, cam, goal)
        assert ok and err < 0.08, (goal, ok, err)
    true = w.bots[1].pose.theta
    assert abs(angle_diff(cg.driver.theta, true)) < 0.4                    # the heading estimate tracks reality
    assert cg.driver.fwd_power >= 40                                       # stiction was discovered and overcome


def test_a_click_outside_the_arena_is_clamped_inside():
    cam, tr = _rig()
    cg = ClickGo(tr, ARENA)
    px = cam.to_img([(1.12, 0.40)])[0]                                     # 12 cm past the wall, still in the camera frame
    gx, gy = cg.set_goal_pixel((float(px[0]), float(px[1])))
    assert 0.0 < gx <= ARENA.width - 0.04 and 0.0 < gy < ARENA.height


def test_no_goal_means_no_motion_and_a_lost_led_stops_the_bot():
    cam, tr = _rig()
    cg = ClickGo(tr, ARENA, ClickParams(latency_s=0.1))
    frame = cam.render_leds({1: Pose(0.4, 0.3, 0.0)}, OFFSET)
    for _ in range(30):
        st = cg.step(frame, DT)
    assert st.status == "idle" and st.cmd == Command()
    px = cam.to_img([(0.9, 0.6)])[0]
    cg.set_goal_pixel((float(px[0]), float(px[1])))
    assert cg.step(frame, DT).status == "moving"                          # a click starts it
    empty = cam.render_leds({}, OFFSET)
    last = None
    for _ in range(40):                                                    # LED vanishes for 2 s
        last = cg.step(empty, DT)
    assert last.status == "lost" and last.cmd == Command()


def test_a_bot_found_outside_the_arena_is_stopped():
    cam, tr = _rig()
    cg = ClickGo(tr, ARENA, ClickParams(margin=0.05))
    cg.goal = (0.5, 0.4)
    st = cg.step(cam.render_leds({1: Pose(1.12, 0.40, 0.0)}, OFFSET), DT)
    assert st.status == "out-of-bounds" and st.cmd == Command()


@pytest.mark.parametrize("true_off,guess", [(0.10, 0.03), (0.02, 0.06), (0.12, 0.05)])
def test_learns_how_far_behind_the_centre_the_led_sits(true_off, guess):
    cam, tr = _rig()
    w = _world(5, 0.7, camera_latency_s=0.3)
    cg = ClickGo(tr, ARENA, ClickParams(latency_s=0.4, led_offset=guess))
    px = cam.to_img([(0.8, 0.55)])[0]
    cg.set_goal_pixel((float(px[0]), float(px[1])))
    seq = 0
    while w.t < 60:
        st = cg.step(cam.render_leds({1: w.bots[1].pose}, true_off), DT)
        seq += 1
        w.send(1, protocol.encode(1, seq, st.cmd))
        w.step(DT)
        if st.status == "arrived":
            break
    assert st.status == "arrived"
    assert abs(cg.driver.led_offset - true_off) < 0.035, (cg.driver.led_offset, true_off)
    p = w.bots[1].pose
    assert math.hypot(p.x - cg.goal[0], p.y - cg.goal[1]) < 0.06


def test_offset_learning_can_be_switched_off():
    cam, tr = _rig()
    cg = ClickGo(tr, ARENA, ClickParams(learn_offset=False, led_offset=0.07))
    assert cg.driver.led_offset == 0.07 and cg.driver._cal == 2        # never runs the calibration turn
