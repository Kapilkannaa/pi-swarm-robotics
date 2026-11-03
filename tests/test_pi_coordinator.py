import json
import math
import socket
import threading
import time
import urllib.error
import urllib.request
from pathlib import Path

import pytest

from swarm.coordinator import Coordinator
from swarm.pi_client import PiClient
from swarm.pi_server import PiServer
from swarm.planner import Arena
from swarm.protocol import Command, parse
from swarm.swarm import SwarmParams
from swarm.target_model import TargetModel

ROOT = Path(__file__).resolve().parent.parent
TOKEN = "test-only-token-with-32-characters"


def brain(allow=True, mode="swarm", ids=None, model=None):
    return Coordinator(Arena(1.0, 0.75), ids or [2], SwarmParams(), allow, mode, model)


def obs(b, state, seq=1, run=False, **kw):
    return {"session": state["session"], "ticket": state["ticket"], "seq": seq,
            "age_s": 0.01, "run": run, "poses": {"2": [0.5, 0.2, 1.57]}, **kw}


def start(b, now=0):
    state = b.new_session(now)
    return b.observe(obs(b, state), now + 0.01)


def test_pi_defaults_to_bench_and_never_outputs_movement():
    b = brain(False)
    state = start(b)
    state = b.observe(obs(b, state, 2, True), 0.05)
    assert b.proposed[2] != Command()
    assert b.outputs(0.05) == {2: Command()}
    assert not state["motion_allowed"]


def test_operator_start_and_pi_disconnect_stop_and_require_pause_before_restart():
    b = brain()
    state = start(b)
    state = b.observe(obs(b, state, 2, True), 0.05)
    assert b.outputs(0.1)[2] != Command()
    assert b.outputs(0.31)[2] == Command()
    with pytest.raises(ValueError, match="paused"):
        b.observe(obs(b, state, 3, True), 0.32)
    assert b.outputs(0.32)[2] == Command()
    state = start(b, 0.33)
    b.observe(obs(b, state, 2, True), 0.36)
    assert b.outputs(0.36)[2] != Command()


@pytest.mark.parametrize("change", [
    {"age_s": 0.35}, {"age_s": -1}, {"age_s": math.nan}, {"age_s": "0.1"},
    {"poses": {"3": [0.2, 0.2, 0]}}, {"poses": {}},
    {"poses": {"2": [math.inf, 0.2, 0]}}, {"poses": {"2": [-0.1, 0.2, 0]}},
    {"poses": {"2": [0.2, 0.2]}}, {"target": [1.5, 0.3]},
    {"run": "true"}, {"seq": 0}, {"ticket": "wrong"}, {"session": "wrong"},
])
def test_bad_observation_immediately_stops_and_does_not_refresh_lease(change):
    b = brain()
    state = start(b)
    state = b.observe(obs(b, state, 2, True), 0.05)
    with pytest.raises(ValueError):
        b.observe({**obs(b, state, 3, True), **change}, 0.1)
    assert b.outputs(0.1) == {2: Command()}
    assert not b.running


def test_ticket_rejects_delayed_request_and_reuse():
    b = brain()
    state = start(b)
    data = obs(b, state, 2, True)
    with pytest.raises(ValueError, match="delayed"):
        b.observe(data, 0.6)
    state = start(b, 1)
    data = obs(b, state, 2, True)
    b.observe(data, 1.05)
    with pytest.raises(ValueError):
        b.observe(data, 1.1)


def test_one_missing_robot_stops_entire_selected_team():
    b = brain(ids=[1, 2])
    state = start(b)
    with pytest.raises(ValueError, match="visible"):
        b.observe(obs(b, state, 2, True), 0.05)
    assert all(c == Command() for c in b.outputs(0.05).values())


def test_led_mode_no_goal_no_movement_and_model_not_needed():
    b = brain(mode="led")
    state = start(b)
    state = b.observe(obs(b, state, 2, True, led=[0.3, 0.3]), 0.05)
    assert b.outputs(0.05)[2] == Command()
    state = b.observe(obs(b, state, 3, True, led=[0.3, 0.3], goal=[0.7, 0.5]), 0.1)
    assert b.outputs(0.1)[2] != Command()
    with pytest.raises(ValueError):
        b.observe(obs(b, state, 4, True), 0.15)
    assert b.outputs(0.15)[2] == Command()
    with pytest.raises(ValueError, match="one robot"):
        brain(mode="led", ids=[1, 2])


def test_learned_target_classifier_controls_discovery_on_pi():
    model = TargetModel.load(ROOT / "models/target.json")
    b = brain(model=model)
    state = start(b)
    red_round = [0.75, 0.9, 0.9, 0.86, 0.73, 0.95]
    red_strip = [0.75, 0.9, 0.9, 0.25, 0.9, 0.15]
    assert model.probability(red_round) >= model.threshold
    assert model.probability(red_strip) < model.threshold
    state = b.observe(obs(b, state, 2, True, target=[0.5, 0.25], target_features=red_strip), 0.05)
    assert state["phase"] == "search"
    state = b.observe(obs(b, state, 3, True, target=[0.5, 0.25], target_features=red_round), 0.1)
    assert state["phase"] == "converge"


def test_actual_http_to_pi_brain_to_udp_and_stops_after_camera_dies():
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as rx:
        rx.bind(("127.0.0.1", 0))
        rx.settimeout(0.15)
        b = brain()
        server = PiServer(("127.0.0.1", 0), b, {2: rx.getsockname()}, TOKEN)
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        url = f"http://127.0.0.1:{server.server_port}"
        try:
            with pytest.raises(urllib.error.HTTPError) as err:
                urllib.request.urlopen(url + "/status", timeout=1)
            assert err.value.code == 401
            client = PiClient(url, TOKEN)
            client.observe(age_s=0, run=False, poses={"2": [0.5, 0.2, 1.57]})
            client.observe(age_s=0, run=True, poses={"2": [0.5, 0.2, 1.57]})
            packets = []
            deadline = time.monotonic() + 0.7
            while time.monotonic() < deadline:
                try:
                    packets.append(parse(rx.recv(64), 2))
                except socket.timeout:
                    pass
            assert any(p and p.cmd != Command() for p in packets)
            assert len(packets) >= 8
            assert all(p and p.cmd == Command() for p in packets[-4:])
            status = client.request("/status")
            assert status["reason"] == "camera/link timeout; restart required"
            assert "online_identification" in status["ai"]
        finally:
            server.shutdown()
            server.server_close()
            thread.join(timeout=2)
