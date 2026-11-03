import math

from swarm.click_control import ClickParams
from swarm.marker_go import MarkerGo
from swarm.planner import Arena
from swarm.protocol import Command
from swarm.geometry import Pose
from swarm.vision.synth import SyntheticCamera
from swarm.vision.tracker import ArenaTracker


def setup():
    arena = Arena(1, .75)
    cam = SyntheticCamera(arena, px_per_m=600, margin_px=90)
    tracker = ArenaTracker(arena)
    tracker.calibrate_from_points([tuple(p) for p in cam.image_corners])
    return cam, MarkerGo(tracker, arena, 1, ClickParams(latency_s=.4))


def test_marker_identity_and_heading_support_click_goal():
    cam, driver = setup()
    frame = cam.render({1: Pose(.3, .3, .7)})
    assert driver.step(frame, .05).cmd == Command()
    driver.set_goal_pixel(tuple(cam.to_img([(.8, .55)])[0]))
    state = driver.step(frame, .05)
    assert state.status == 'moving' and state.cmd != Command()
    assert abs(state.theta-.7) < .05
    driver.stop(); assert driver.step(frame, .05).cmd == Command()


def test_missing_or_wrong_marker_stops_immediately():
    cam, driver = setup()
    driver.goal = (.8, .5)
    driver.step(cam.render({1: Pose(.3, .3, 0)}), .05)
    for frame in [cam.render({}), cam.render({2: Pose(.3, .3, 0)})]:
        state = driver.step(frame, .05)
        assert state.status == 'lost' and state.cmd == Command()
