"""Label real camera candidate features: y=target, n=distractor, q=quit.

Collect samples across positions and lighting, plus confusing red objects. Train
with train_target.py --data logs/target_labels.jsonl, then copy model to the Pi.
"""
import argparse
import json
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
import cv2
from swarm.config import arena_from, load_config
from swarm.io import LatestFrame
from swarm.vision.features import target_candidate
from swarm.vision.tracker import ArenaTracker


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--source", default="0")
    ap.add_argument("--config", default=str(ROOT / "config.toml"))
    ap.add_argument("--cal", default=str(ROOT / "logs/calibration.json"))
    a = ap.parse_args()
    tracker = ArenaTracker(arena_from(load_config(a.config)))
    tracker.load(a.cal)
    stream = LatestFrame(a.source)
    try:
        with open(ROOT / "logs/target_labels.jsonl", "a") as log:
            while True:
                frame, stamp = stream.read()
                if frame is None or time.time() - stamp > 0.35:
                    time.sleep(0.05)
                    continue
                xy, features = target_candidate(frame, tracker)
                if xy:
                    cv2.circle(frame, tracker.world_to_pixel(xy), 20, (0, 255, 0), 2)
                cv2.imshow("Label green candidate: y target, n distractor, q quit", frame)
                key = cv2.waitKey(30) & 0xff
                if key == ord('q'):
                    break
                if features and key in (ord('y'), ord('n')):
                    log.write(json.dumps({"features": features, "label": int(key == ord('y'))}) + '\n')
                    log.flush()
    finally:
        stream.close()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
