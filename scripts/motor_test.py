"""Guided motor bring-up over UDP. Wheels OFF THE GROUND. Press Enter to advance, Ctrl+C aborts.

    python scripts/motor_test.py --ip 10.72.92.233            # direction test, then ramp
    python scripts/motor_test.py --ip ... --only direction

Every step streams for a few seconds then sends an all-stop; the firmware failsafe backs that up.
Speeds are low (30-60) on purpose for the first run.
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


class Bot:
    def __init__(self, ip: str, port: int, bot_id: int):
        self.sock, self.dest, self.id, self.seq = socket.socket(socket.AF_INET, socket.SOCK_DGRAM), (ip, port), bot_id, 0

    def send(self, cmd: Command) -> None:
        self.seq += 1
        self.sock.sendto(protocol.encode(self.id, self.seq, cmd), self.dest)

    def run(self, cmd: Command, seconds: float, hz: float = 20.0) -> None:
        t_end = time.time() + seconds
        try:
            while time.time() < t_end:
                self.send(cmd)
                time.sleep(1 / hz)
        finally:
            self.send(Command())


def ask(prompt: str) -> None:
    input(f"\n{prompt}\n  [Enter] to run, Ctrl+C to abort > ")


def direction(bot: Bot) -> None:
    steps = [
        ("LEFT wheel only, FORWARD", Command(left=50), "Left wheel should spin so the top of the tyre moves FORWARD."),
        ("RIGHT wheel only, FORWARD", Command(right=50), "Right wheel should spin so the top of the tyre moves FORWARD."),
        ("BOTH wheels FORWARD", Command(50, 50), "Both spin forward."),
        ("BOTH wheels BACKWARD", Command(-50, -50), "Both spin backward."),
    ]
    for title, cmd, expect in steps:
        ask(f"DIRECTION: {title}\n  Expect: {expect}")
        bot.run(cmd, 2.0)
        print("  stopped. Note which wheel (if any) was wrong or did not move.")


def ramp(bot: Bot) -> None:
    ask("RAMP: both wheels forward, 30 -> 100 in steps of 10, 1.5 s each.\n"
        "  Expect: smooth speed increase. Note the smallest value where BOTH wheels reliably turn.")
    for v in range(30, 101, 10):
        print(f"  command {v}")
        bot.run(Command(v, v), 1.5)
        time.sleep(0.3)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--ip", required=True)
    ap.add_argument("--port", type=int, default=4210)
    ap.add_argument("--id", type=int, default=1)
    ap.add_argument("--only", choices=("direction", "ramp"))
    a = ap.parse_args()
    bot = Bot(a.ip, a.port, a.id)
    try:
        if a.only in (None, "direction"):
            direction(bot)
        if a.only in (None, "ramp"):
            ramp(bot)
    except KeyboardInterrupt:
        print("\naborted")
    finally:
        bot.send(Command())
    print("done, all stop sent")


if __name__ == "__main__":
    main()
