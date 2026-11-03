"""Python model of the bot firmware's behaviour; the C++ in firmware/bot must match it.

It exists so the safety rules (failsafe, laser pulse cap, re-arm) are tested in simulation
before they are written in C++, and so the sim exercises the same packet parser.
"""
from __future__ import annotations

from dataclasses import dataclass

from .. import protocol
from ..protocol import Command

FAILSAFE_MS = 500
LASER_MAX_MS = 2500
# Motor duty cap (fraction of full PWM). At 100% command the motors see ~PWM_CAP * Vbat.
PWM_CAP = 0.72


@dataclass(frozen=True)
class Outputs:
    left: float   # signed duty in [-PWM_CAP, PWM_CAP]
    right: float
    laser: bool


class FirmwareModel:
    def __init__(self, bot_id: int):
        self.bot_id = bot_id
        self._cmd = Command()
        self._last_rx_ms: int | None = None
        self._laser_start_ms: int | None = None
        self._laser_latched_off = False  # set after the cap; cleared only by a laser=0 command

    def on_packet(self, data: bytes, now_ms: int) -> None:
        pkt = protocol.parse(data, expected_id=self.bot_id)
        if pkt is None:
            return  # malformed or not ours: ignored entirely, failsafe timer is NOT refreshed
        self._cmd = pkt.cmd
        self._last_rx_ms = now_ms
        if not pkt.cmd.laser:
            self._laser_latched_off = False
            self._laser_start_ms = None

    def tick(self, now_ms: int) -> Outputs:
        if self._last_rx_ms is None or now_ms - self._last_rx_ms > FAILSAFE_MS:
            self._laser_start_ms = None
            return Outputs(0.0, 0.0, False)
        laser = False
        if self._cmd.laser and not self._laser_latched_off:
            if self._laser_start_ms is None:
                self._laser_start_ms = now_ms
            if now_ms - self._laser_start_ms < LASER_MAX_MS:
                laser = True
            else:
                self._laser_latched_off = True
        return Outputs(self._cmd.left / 100 * PWM_CAP, self._cmd.right / 100 * PWM_CAP, laser)
