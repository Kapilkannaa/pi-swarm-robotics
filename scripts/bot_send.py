"""Bring-up tool: stream one fixed command to a bot over UDP at 20 Hz.

    python scripts/bot_send.py --ip 10.72.92.233 --left 50 --right 50 --seconds 1

The stream stops after --seconds and a final all-stop packet is sent, so the bot is never left
running if you press Ctrl+C. (The firmware's 500 ms failsafe is the second line of defence.)
"""
from __future__ import annotations

import argparse
import socket
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from swarm import protocol  # noqa: E402
from swarm.protocol import Command  # noqa: E402


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--ip", required=True)
    ap.add_argument("--port", type=int, default=4210)
    ap.add_argument("--id", type=int, default=1)
    ap.add_argument("--left", type=int, default=0)
    ap.add_argument("--right", type=int, default=0)
    ap.add_argument("--marker", type=int, default=0, choices=(0, 1))
    ap.add_argument("--seconds", type=float, default=1.0)
    ap.add_argument("--hz", type=float, default=20.0)
    a = ap.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    dest = (a.ip, a.port)
    cmd = Command(a.left, a.right, bool(a.marker))
    seq, t_end, n = 0, time.time() + a.seconds, 0
    try:
        while time.time() < t_end:
            seq += 1
            sock.sendto(protocol.encode(a.id, seq, cmd), dest)
            n += 1
            time.sleep(1 / a.hz)
    finally:
        sock.sendto(protocol.encode(a.id, seq + 1, Command()), dest)
    print(f"sent {n} packets of {protocol.encode(a.id, 0, cmd)!r} to {a.ip}:{a.port}, then stop")


if __name__ == "__main__":
    main()
