"""UDP wire format, laptop -> bot. The firmware parser must behave exactly like parse().

Packet (ASCII, one per datagram):  "<bot_id>,<seq>,<left>,<right>,<laser>\n"
  left/right: integer -100..100 (sign = direction), laser: 0 or 1.

Why text: trivially debuggable with `nc -u`, and ~20 bytes is nothing at 20 Hz.
Out-of-range numbers are clamped (a slightly wrong value is still a usable command);
anything structurally wrong is dropped (we cannot guess what it meant).
"""
from __future__ import annotations

import re
from dataclasses import dataclass

MAX_CMD = 100
MAX_PACKET_LEN = 48  # firmware reads into a fixed buffer; longer = malformed

# Strict grammar, mirrored exactly by firmware/bot/logic.h: five optionally-negative decimal
# integers, comma separated, optional trailing CR/LF. No spaces, no '+', no underscores
# (Python's int() would accept those, a hand-written C++ parser should not).
_PACKET_RE = re.compile(rb"(-?[0-9]+),(-?[0-9]+),(-?[0-9]+),(-?[0-9]+),(-?[0-9]+)\r?\n?")


@dataclass(frozen=True)
class Command:
    left: int = 0
    right: int = 0
    laser: bool = False


@dataclass(frozen=True)
class Packet:
    bot_id: int
    seq: int
    cmd: Command


def _clamp(v: int, lo: int, hi: int) -> int:
    return max(lo, min(hi, v))


def encode(bot_id: int, seq: int, cmd: Command) -> bytes:
    left = _clamp(int(round(cmd.left)), -MAX_CMD, MAX_CMD)
    right = _clamp(int(round(cmd.right)), -MAX_CMD, MAX_CMD)
    return f"{bot_id},{seq},{left},{right},{int(cmd.laser)}\n".encode("ascii")


def parse(data: bytes, expected_id: int | None = None) -> Packet | None:
    """Return a clamped Packet, or None if malformed / addressed to another bot."""
    if not data or len(data) > MAX_PACKET_LEN:
        return None
    m = _PACKET_RE.fullmatch(data)
    if m is None:
        return None
    bot_id, seq, left, right, laser = (int(g) for g in m.groups())
    if expected_id is not None and bot_id != expected_id:
        return None
    return Packet(
        bot_id=bot_id,
        seq=seq,
        cmd=Command(
            left=_clamp(left, -MAX_CMD, MAX_CMD),
            right=_clamp(right, -MAX_CMD, MAX_CMD),
            laser=_clamp(laser, 0, 1) == 1,
        ),
    )
