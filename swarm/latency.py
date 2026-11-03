"""Measure the command-to-visible-motion delay of the whole loop (UDP + motor spin-up + camera + decode).

The burst controller needs this number (`latency_s`): it waits that long after each burst before trusting a pose. Too small and
the bot acts on stale poses (overshoot); too large and it is needlessly slow.
"""
from __future__ import annotations

import statistics
from typing import Callable

from .geometry import angle_diff


def measure_once(send_rotate: Callable[[], None], send_stop: Callable[[], None], get_theta: Callable[[], float | None],
                 clock: Callable[[], float], sleep: Callable[[float], None], threshold: float = 0.04,
                 timeout_s: float = 3.0, rate_hz: float = 20.0) -> float | None:
    """Seconds from the first rotate command until the observed heading moves by `threshold` rad. None if it never does."""
    theta0 = None
    while theta0 is None:  # wait for a first visible pose
        theta0 = get_theta()
        sleep(1 / rate_hz)
    t0 = clock()
    try:
        while clock() - t0 < timeout_s:
            send_rotate()
            sleep(1 / rate_hz)
            th = get_theta()
            if th is not None and abs(angle_diff(th, theta0)) > threshold:
                return clock() - t0
    finally:
        send_stop()
    return None


def summarise(samples: list[float]) -> dict[str, float]:
    s = sorted(samples)
    return {"median": statistics.median(s), "worst": s[-1], "recommended_latency_s": round(statistics.median(s) + 0.1, 2)}
