"""Live pose tracking with an overlay. Needs the camera stream and the 4 corner markers in view.

    python scripts/track.py --source http://PHONE_IP:8080/video
    python scripts/track.py --source 0                # laptop webcam, for a dry run

Keys: c = (re)calibrate from the corner markers and save, q = quit.
Arena size and camera URL default to config.toml if it exists (copy config.example.toml first).
"""
from __future__ import annotations

import argparse
import sys
import time
import tomllib
from pathlib import Path

import cv2

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
from swarm.planner import Arena  # noqa: E402
from swarm.vision.tracker import ArenaTracker, draw_overlay  # noqa: E402


def main() -> None:
    cfg_path = ROOT / "config.toml"
    cfg = tomllib.loads(cfg_path.read_text()) if cfg_path.exists() else {}
    ap = argparse.ArgumentParser()
    ap.add_argument("--source", default=cfg.get("camera", {}).get("url", "0"))
    ap.add_argument("--width", type=float, default=cfg.get("arena", {}).get("width_m", 1.2))
    ap.add_argument("--height", type=float, default=cfg.get("arena", {}).get("height_m", 0.9))
    ap.add_argument("--inset", type=float, default=0.06, help="corner marker centre distance from the arena corner (m)")
    ap.add_argument("--cal", default=str(ROOT / "logs" / "calibration.json"))
    a = ap.parse_args()

    src = int(a.source) if a.source.isdigit() else a.source
    cap = cv2.VideoCapture(src)
    if not cap.isOpened():
        sys.exit(f"cannot open camera source {a.source!r}")
    tracker = ArenaTracker(Arena(a.width, a.height), a.inset)
    if Path(a.cal).exists():
        tracker.load(a.cal)
        print(f"loaded calibration from {a.cal}")
    n, t0 = 0, time.time()
    while True:
        ok, frame = cap.read()
        if not ok:
            print("no frame (stream dropped?)")
            time.sleep(0.2)
            continue
        key = cv2.waitKey(1) & 0xFF
        if key == ord("c") or tracker.H is None:
            if tracker.calibrate(frame):
                Path(a.cal).parent.mkdir(exist_ok=True)
                tracker.save(a.cal)
                print(f"calibrated and saved to {a.cal}")
            elif key == ord("c"):
                print("calibration failed: all four corner markers (IDs 10-13) must be visible")
        poses = tracker.poses(frame) if tracker.H is not None else {}
        n += 1
        if n % 30 == 0:
            fps = n / (time.time() - t0)
            print(f"{fps:.1f} fps  " + "  ".join(f"bot{i}=({p.x:.3f},{p.y:.3f},{p.theta:.2f}rad)" for i, p in sorted(poses.items())))
        cv2.imshow("tracker (c=calibrate, q=quit)", draw_overlay(frame, poses, tracker))
        if key == ord("q"):
            break
    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
