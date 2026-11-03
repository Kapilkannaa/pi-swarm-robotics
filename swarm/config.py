"""config.toml loading (copy config.example.toml). Kept tiny: the file is the single source of IPs and sizes."""
from __future__ import annotations

import tomllib
from pathlib import Path

from .control import ControlParams
from .planner import Arena
from .swarm import SwarmParams


def load_config(path: str | Path) -> dict:
    p = Path(path)
    if not p.exists():
        raise FileNotFoundError(f"{p} not found: copy config.example.toml to config.toml and edit it")
    return tomllib.loads(p.read_text())


def arena_from(cfg: dict) -> Arena:
    a = cfg["arena"]
    return Arena(float(a["width_m"]), float(a["height_m"]))


def bots_from(cfg: dict) -> dict[int, tuple[str, int]]:
    return {int(b["id"]): (b["ip"], int(b.get("port", 4210))) for b in cfg["bots"]}


def swarm_params_from(cfg: dict) -> SwarmParams:
    c = cfg.get("control", {})
    ctl = ControlParams(min_cmd=float(c.get("min_cmd", 35)), latency_s=float(c.get("latency_s", 0.4)))
    return SwarmParams(detect_radius=float(c.get("detect_radius_m", 0.25)), control=ctl)
