"""Train a reproducible red-round-token classifier; synthetic images only.

Optional --data labeled.jsonl uses [{features: [...], label: 0|1}, ...] lines
collected from the actual camera with scripts/label_target.py.
"""
from __future__ import annotations

import argparse
import json
import random
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
from swarm.target_model import TargetModel


def synthetic_rows(seed, count):
    import cv2
    import numpy as np
    from swarm.vision.features import contour_features
    r = random.Random(seed)
    rows = []
    for i in range(count):
        positive = i % 2 == 0
        frame = np.full((128, 128, 3), r.randint(170, 245), np.uint8)
        mask = np.zeros((128, 128), np.uint8)
        red = positive or r.random() < 0.6
        hue = r.choice([0, 3, 6, 175, 179]) if red else r.randint(25, 140)
        color = cv2.cvtColor(np.uint8([[[hue, r.randint(130, 255), r.randint(100, 250)]]]), cv2.COLOR_HSV2BGR)[0, 0].tolist()
        if positive or not red:
            radius = r.randint(10, 35)
            cv2.ellipse(mask, (64, 64), (radius, round(radius * r.uniform(0.82, 1))), r.randint(0, 180), 0, 360, 255, -1)
        else:
            cv2.rectangle(mask, (r.randint(15, 30), 55), (r.randint(95, 115), r.randint(62, 75)), 255, -1)
        frame[mask > 0] = color
        frame = cv2.GaussianBlur(frame, (3, 3), 0)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        rows.append((contour_features(frame, contours[0]), int(positive)))
    return rows


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--data")
    ap.add_argument("--output", default=str(ROOT / "models/target.json"))
    a = ap.parse_args()
    if a.data:
        rows = [(d["features"], int(d["label"])) for d in map(json.loads, Path(a.data).read_text().splitlines())]
        random.Random(42).shuffle(rows)
        n = int(len(rows) * 0.8)
        train, test, source = rows[:n], rows[n:], "operator-labeled real camera; random 80/20 split"
        if len(train) < 20 or len(test) < 5 or {y for _, y in train} != {0, 1}:
            raise ValueError("need >=25 labeled samples covering both classes")
    else:
        train, test, source = synthetic_rows(42, 1200), synthetic_rows(901, 400), "synthetic images; separate seeds 42/901"
    model = TargetModel.train(train)
    tp = fp = tn = fn = 0
    for x, y in test:
        pred = model.probability(x) >= model.threshold
        tp += bool(pred and y); fp += bool(pred and not y)
        tn += bool(not pred and not y); fn += bool(not pred and y)
    metrics = {"source": source, "train_samples": len(train), "test_samples": len(test),
               "tp": tp, "fp": fp, "tn": tn, "fn": fn, "accuracy": (tp + tn) / len(test),
               "real_camera_validated": False, "purpose": "red round target token; not generic object detection"}
    model.save(a.output, metrics)
    print(json.dumps(metrics, indent=2))


if __name__ == "__main__":
    main()
