"""Direct laptop click-to-go with a printed ArUco marker and measured heading."""
from .clickgo import ClickState
from .control import ControlParams, make_controller
from .geometry import clamp
from .protocol import Command


class MarkerGo:
    def __init__(self, tracker, arena, marker_id, params):
        self.tracker, self.arena, self.marker_id, self.p = tracker, arena, marker_id, params
        self.goal = None
        self.driver = make_controller(ControlParams(min_cmd=params.min_cmd, latency_s=params.latency_s))

    def set_goal_pixel(self, pixel):
        x, y = self.tracker.pixel_to_world(pixel)
        self.goal = (clamp(x, .05, self.arena.width-.05), clamp(y, .05, self.arena.height-.05))
        return self.goal

    def stop(self):
        self.goal = None
        self.driver = make_controller(self.driver.p)

    def step(self, frame, dt):
        pose = self.tracker.poses(frame).get(self.marker_id)
        cmd, status, centre, px, theta = Command(), "lost", None, None, None
        if pose is not None:
            centre, theta = (pose.x, pose.y), pose.theta
            px = self.tracker.world_to_pixel(centre)
            if not (-.02 <= pose.x <= self.arena.width+.02 and -.02 <= pose.y <= self.arena.height+.02):
                status = "out-of-bounds"
                self.driver = make_controller(self.driver.p)
            elif self.goal is None:
                status = "idle"
            else:
                cmd, reached = self.driver.go_to(pose, self.goal, dt)
                status = "arrived" if reached else "moving"
        else:
            self.driver = make_controller(self.driver.p)
        return ClickState(cmd, status, px, centre, centre, theta, self.goal)
