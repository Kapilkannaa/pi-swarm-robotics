import json

import pytest

from swarm.click_control import ClickDriver, ClickParams
from swarm.protocol import Command


def test_cancel_goal_immediately_cancels_active_burst():
    driver = ClickDriver(ClickParams())
    cmd, _ = driver.update((0.2, 0.2), (0.8, 0.5), 0.05)
    assert cmd != Command()
    for _ in range(4):
        cmd, status = driver.update((0.2, 0.2), None, 0.05)
        assert cmd == Command() and status == "idle"


@pytest.mark.parametrize("points", [
    [(0, 0)] * 4,
    [(0, 0), (10, 10), (0, 10), (10, 0)],
    [(0, 0), (10, 0), (10, 10), (float('nan'), 10)],
])
def test_bad_calibration_cannot_silently_create_robot_coordinates(points):
    from swarm.vision.tracker import ArenaTracker
    from swarm.planner import Arena
    with pytest.raises(ValueError):
        ArenaTracker(Arena(1, .75)).calibrate_from_points(points)


def test_calibration_from_other_arena_is_rejected(tmp_path):
    from swarm.vision.tracker import ArenaTracker
    from swarm.planner import Arena
    p = tmp_path / "cal.json"
    p.write_text(json.dumps({"H": [[1, 0, 0], [0, 1, 0], [0, 0, 1]], "arena": [2, 2]}))
    with pytest.raises(ValueError, match="arena"):
        ArenaTracker(Arena(1, .75)).load(p)
