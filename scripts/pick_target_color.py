"""Teach the system the target's colour from the real camera image.

    python scripts/pick_target_color.py                 # camera URL from config.toml
    python scripts/pick_target_color.py --source 0      # a webcam, for a dry run

Click once on the middle of the target object. The window then shows what the detector would see (the target should be the
only bright blob in the mask panel). Click again to re-sample, `s` to save to logs/target_color.json, `q` to quit.
Then run: python scripts/run_live.py --target-color custom
"""
from __future__ import annotations

import argparse
import sys
import tomllib
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
from swarm.vision.target import color_mask, find_target, sample_ranges, save_ranges  # noqa: E402


def main() -> None:
    cfg_path = ROOT / "config.toml"
    cfg = tomllib.loads(cfg_path.read_text()) if cfg_path.exists() else {}
    ap = argparse.ArgumentParser()
    ap.add_argument("--source", default=cfg.get("camera", {}).get("url", "0"))
    ap.add_argument("--out", default=str(ROOT / "logs" / "target_color.json"))
    a = ap.parse_args()
    cap = cv2.VideoCapture(int(a.source) if a.source.isdigit() else a.source)
    if not cap.isOpened():
        sys.exit(f"cannot open camera source {a.source!r}")
    state: dict = {"ranges": None, "click": None}

    def on_mouse(event: int, x: int, y: int, *_: object) -> None:
        if event == cv2.EVENT_LBUTTONDOWN:
            state["click"] = (x, y)

    cv2.namedWindow("pick target colour")
    cv2.setMouseCallback("pick target colour", on_mouse)
    while True:
        ok, frame = cap.read()
        if not ok:
            continue
        if state["click"] is not None:
            state["ranges"] = sample_ranges(frame, state["click"])
            state["click"] = None
            print("sampled HSV ranges:", state["ranges"])
        view, msg = frame.copy(), "click the target, s = save, q = quit"
        mask_panel = np.zeros_like(frame)
        if state["ranges"] is not None:
            m = color_mask(frame, state["ranges"])
            mask_panel = cv2.cvtColor(m, cv2.COLOR_GRAY2BGR)
            hit = find_target(frame, state["ranges"])
            if hit is not None:
                cv2.circle(view, (int(hit[0]), int(hit[1])), 16, (0, 255, 255), 3)
                msg = f"found at ({hit[0]:.0f},{hit[1]:.0f}). Only the target should be white on the right. s = save"
            else:
                msg = "nothing found: click the target again"
        cv2.putText(view, msg, (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        cv2.imshow("pick target colour", np.hstack([view, mask_panel]))
        key = cv2.waitKey(30) & 0xFF
        if key == ord("s") and state["ranges"] is not None:
            Path(a.out).parent.mkdir(exist_ok=True)
            save_ranges(a.out, state["ranges"])
            print(f"saved {a.out}")
        elif key == ord("q"):
            break
    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
