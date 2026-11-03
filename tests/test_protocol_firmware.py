import pytest

from swarm import protocol
from swarm.protocol import Command
from swarm.sim.firmware_model import FAILSAFE_MS, LASER_MAX_MS, PWM_CAP, FirmwareModel


def pkt(left=0, right=0, laser=0, bot=1, seq=1) -> bytes:
    return f"{bot},{seq},{left},{right},{laser}\n".encode()


def test_roundtrip():
    p = protocol.parse(protocol.encode(2, 7, Command(-40, 55, True)), expected_id=2)
    assert p is not None and (p.seq, p.cmd) == (7, Command(-40, 55, True))


@pytest.mark.parametrize("data", [b"", b"garbage", b"1,2,3\n", b"1,2,a,b,c\n", b"1,2,3,4,5,6\n",
                                  b"\xff\xfe,1,2,3,4", b"1," + b"9" * 60, b"1,1,1.5,2,0\n"])
def test_malformed_ignored(data):
    assert protocol.parse(data) is None


def test_values_clamped():
    p = protocol.parse(b"1,1,999,-999,7\n")
    assert p is not None and p.cmd == Command(100, -100, True)


def test_other_bots_packets_ignored():
    assert protocol.parse(pkt(bot=2), expected_id=1) is None


def test_failsafe_stops_motors_and_laser():
    fw = FirmwareModel(1)
    assert fw.tick(0) == fw.tick(10)  # nothing received yet: safe
    fw.on_packet(pkt(100, 100, 1), 1000)
    out = fw.tick(1100)
    assert out.left == pytest.approx(PWM_CAP) and out.laser
    out = fw.tick(1000 + FAILSAFE_MS + 1)
    assert (out.left, out.right, out.laser) == (0, 0, False)


def test_malformed_packet_does_not_feed_watchdog():
    fw = FirmwareModel(1)
    fw.on_packet(pkt(50, 50), 0)
    fw.on_packet(b"junk", 400)
    assert fw.tick(FAILSAFE_MS + 1).left == 0


def test_laser_off_at_boot():
    assert FirmwareModel(1).tick(0).laser is False


def test_laser_pulse_capped_and_needs_rearm():
    fw = FirmwareModel(1)
    on_ms = 0
    for t in range(0, 6000, 50):  # planner keeps asking for the laser the whole time
        fw.on_packet(pkt(laser=1), t)
        on_ms += 50 if fw.tick(t).laser else 0
    assert on_ms <= LASER_MAX_MS + 50
    fw.on_packet(pkt(laser=0), 6000)  # a laser=0 command re-arms...
    assert not fw.tick(6000).laser
    fw.on_packet(pkt(laser=1), 6050)  # ...and a fresh laser=1 fires again
    assert fw.tick(6050).laser


def test_duty_cap_respected():
    fw = FirmwareModel(1)
    fw.on_packet(pkt(999, -999), 0)
    out = fw.tick(0)
    assert abs(out.left) <= PWM_CAP and abs(out.right) <= PWM_CAP
