"""Small supervised logistic target classifier; inference/training use stdlib only.

This learns a *red round token* from labeled image features. It is not a general
object recognizer. Synthetic training accuracy is not real-camera accuracy.
"""
from __future__ import annotations

import json
import math
from pathlib import Path

FEATURES = ["red_excess", "saturation", "brightness", "circularity", "extent", "aspect"]


class TargetModel:
    size = len(FEATURES)

    def __init__(self, weights, bias, threshold=0.8):
        if len(weights) != self.size or not all(math.isfinite(x) for x in [*weights, bias, threshold]):
            raise ValueError("invalid target model")
        self.weights, self.bias, self.threshold = weights, bias, threshold

    def probability(self, features):
        if len(features) != self.size or not all(math.isfinite(x) for x in features):
            raise ValueError("invalid target features")
        z = self.bias + sum(w * x for w, x in zip(self.weights, features))
        return 1 / (1 + math.exp(-max(-50, min(50, z))))

    @classmethod
    def load(cls, path):
        obj = json.loads(Path(path).read_text())
        if obj["features"] != FEATURES:
            raise ValueError("model feature schema mismatch")
        return cls(obj["weights"], obj["bias"], obj["threshold"])

    def save(self, path, metadata):
        Path(path).parent.mkdir(parents=True, exist_ok=True)
        Path(path).write_text(json.dumps({"features": FEATURES, "weights": self.weights,
            "bias": self.bias, "threshold": self.threshold, "metadata": metadata}, indent=2) + "\n")

    @classmethod
    def train(cls, rows, epochs=500, rate=0.5):
        model = cls([0.0] * cls.size, 0.0)
        for _ in range(epochs):
            dw, db = [0.0] * cls.size, 0.0
            for x, label in rows:
                error = model.probability(x) - label
                db += error
                for j in range(cls.size):
                    dw[j] += error * x[j]
            model.bias -= rate * db / len(rows)
            for j in range(cls.size):
                model.weights[j] -= rate * (dw[j] / len(rows) + 0.0005 * model.weights[j])
        return model
