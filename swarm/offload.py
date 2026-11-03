"""Background trajectory analysis for the Pi. No camera, GPIO or robot transport.

Monte Carlo results use assumed physics and are advice, never motor commands.
"""
from __future__ import annotations

import math
import random
import statistics
import time
from dataclasses import replace

from .control import ControlParams, make_controller
from .geometry import Pose, dist
from .planner import Arena
from .protocol import encode
from .sim.firmware_model import FirmwareModel
from .sim.world import SimBot, SimParams, World


def vector(value, size):
    if not isinstance(value, list) or len(value) != size or any(
            type(v) not in (float, int) or not math.isfinite(v) for v in value):
        raise ValueError(f"expected {size} finite numbers")
    return tuple(float(v) for v in value)


def evaluate(request: dict) -> dict:
    started = time.perf_counter()
    arena = Arena(*vector(request.get("arena"), 2))
    pose = Pose(*vector(request.get("pose"), 3))
    goal = vector(request.get("goal"), 2)
    if not all(.5 <= v <= 3 for v in (arena.width, arena.height)):
        raise ValueError("supported arena dimensions: 0.5..3 metres")
    for point in ((pose.x, pose.y), goal):
        if not (0 <= point[0] <= arena.width and 0 <= point[1] <= arena.height):
            raise ValueError("pose/goal outside arena")
    n = request.get("trials", 12)
    if type(n) is not int or not 2 <= n <= 24:
        raise ValueError("trials must be 2..24 per profile")
    values = request.get("control", {})
    min_cmd, latency = values.get("min_cmd", 35), values.get("latency_s", .4)
    if (type(min_cmd) not in (int, float) or not math.isfinite(min_cmd) or not 10 <= min_cmd <= 60
            or type(latency) not in (int, float) or not math.isfinite(latency) or not 0 <= latency <= 1):
        raise ValueError("invalid control assumptions")
    base = ControlParams(min_cmd=min_cmd, latency_s=max(.15, latency))
    profiles = [("conservative", replace(base, undershoot=.65, fwd_max_s=.5)),
                ("current", base), ("longer-bursts", replace(base, undershoot=.9, fwd_max_s=1.0))]
    rows, steps = [], 0
    for name, params in profiles:
        times, errors, boundary_trials = [], [], 0
        for seed in range(n):
            rng = random.Random(seed + 1701)
            # Seed-matched conditions across profiles; no claim these are measured motors.
            sim = SimParams(v_max=rng.uniform(.12, .26), camera_latency_s=rng.uniform(.1, .45),
                packet_loss=rng.uniform(0, .12), pos_noise=.003, heading_noise=.015)
            bot = SimBot(1, pose, rng.uniform(.82, 1.12), rng.uniform(.82, 1.12), FirmwareModel(1))
            world = World(arena, goal, {1: bot}, sim, rng)
            ctl, reached, near_edge = make_controller(params), False, False
            for step in range(1200):  # 60 simulated seconds at 20 Hz
                observed = world.observe()[1]
                command, reached = ctl.go_to(observed, goal, .05)
                world.send(1, encode(1, step, command))
                world.step(.05)
                steps += 1
                p = bot.pose
                near_edge |= min(p.x, p.y, arena.width-p.x, arena.height-p.y) < .02
                if reached:
                    break
            error = dist((bot.pose.x, bot.pose.y), goal)
            errors.append(error)
            if reached and error < .08:
                times.append(world.t)
            boundary_trials += near_edge
        rows.append({"profile": name, "trials": n, "successes": len(times),
            "mean_time_s": round(statistics.mean(times), 2) if times else None,
            "worst_error_m": round(max(errors), 3), "near_boundary_trials": boundary_trials})
    best = min(rows, key=lambda r: (-r["successes"], r["near_boundary_trials"], r["mean_time_s"] or math.inf))
    return {"job_id": request.get("job_id"), "goal": list(goal), "profiles": rows,
            "recommendation": best["profile"], "work_steps": steps,
            "compute_ms": round((time.perf_counter()-started)*1000, 1),
            "physics": "assumed motor speed/mismatch, camera lag and packet loss; not hardware measurements",
            "applied_to_motors": False}
