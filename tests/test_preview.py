import pytest

from swarm.preview import Preview
from swarm.target_model import TargetModel


def model():
    return TargetModel([3, 0, 0, 3, 0, 3], -4)


def frame(state, seq=1, features=None, **extra):
    return {'session': state['session'], 'ticket': state['ticket'], 'seq': seq,
            'age_s': .01, 'features': features, **extra}


def test_camera_inference_can_only_control_marker_and_times_out():
    p = Preview(model())
    s = p.new_session(0)
    s = p.classify(frame(s, features=[1, 1, 1, 1, 1, 1]), .05)
    assert s['accepted'] and s['motors'] == [0, 0]
    c = p.command(.1, True)
    assert c.left == c.right == 0 and c.laser
    assert not p.command(.1, False).laser
    assert not p.command(.31, True).laser


@pytest.mark.parametrize('bad', [{'age_s': .5}, {'features': [1]}, {'features': [float('nan')]*6}, {'seq': -1}])
def test_bad_frames_turn_indicator_off(bad):
    p = Preview(model())
    s = p.new_session(0)
    with pytest.raises(ValueError):
        p.classify(frame(s, **bad), .05)
    assert not p.command(.1, True).laser


def test_expired_ticket_and_replay_cannot_light_indicator():
    p = Preview(model())
    s = p.new_session(0)
    with pytest.raises(ValueError):
        p.classify(frame(s), .6)
    s = p.new_session(1)
    data = frame(s, features=[1]*6)
    p.classify(data, 1.05)
    with pytest.raises(ValueError):
        p.classify(data, 1.06)
    assert not p.command(1.1, True).laser
