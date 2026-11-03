"""Real-world I/O: newest-frame camera reader and the UDP link to the bots. No planning logic in here."""
from __future__ import annotations

import socket
import threading
import time

import cv2
import numpy as np

from . import protocol
from .protocol import Command


class LatestFrame:
    """Reads a stream on a thread and keeps only the newest frame, so we never act on a queue of stale video."""

    def __init__(self, source: str, loop: bool = False):
        """loop=True restarts a video FILE at its end (tests only; a live stream never ends)."""
        self.loop = loop
        self.cap = cv2.VideoCapture(int(source) if source.isdigit() else source)
        if not self.cap.isOpened():
            raise OSError(f"cannot open camera source {source!r}")
        self.frame: np.ndarray | None = None
        self.stamp = 0.0
        self.count = 0
        self._lock, self._run = threading.Lock(), True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def _loop(self) -> None:
        while self._run:
            ok, f = self.cap.read()
            if ok:
                with self._lock:
                    self.frame, self.stamp, self.count = f, time.time(), self.count + 1
            elif self.loop:
                self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                time.sleep(0.05)  # roughly real-time pacing for a looped file
            else:
                time.sleep(0.02)

    def read(self) -> tuple[np.ndarray | None, float]:
        with self._lock:
            return (None if self.frame is None else self.frame.copy()), self.stamp

    def close(self) -> None:
        self._run = False
        self._thread.join(timeout=1.0)
        self.cap.release()


class UdpLink:
    def __init__(self, bots: dict[int, tuple[str, int]]):
        self.bots, self.sock, self.seq = bots, socket.socket(socket.AF_INET, socket.SOCK_DGRAM), 0

    def send(self, cmds: dict[int, Command]) -> None:
        self.seq += 1
        for i, c in cmds.items():
            if i in self.bots:
                self.sock.sendto(protocol.encode(i, self.seq, c), self.bots[i])

    def stop_all(self) -> None:
        for _ in range(3):  # repeat: UDP can drop a packet, and this is the one that matters
            self.send({i: Command() for i in self.bots})
            time.sleep(0.02)
