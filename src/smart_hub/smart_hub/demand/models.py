"""Forecast models: persistence baseline and an exported linear (ridge) model.

The ridge model is trained offline with scikit-learn and exported to a small
JSON file (scaler mean/scale, coefficients, intercept, metadata). At runtime it
is evaluated in pure Python, so the ROS node needs no numpy/sklearn and
inference takes microseconds.
"""

import hashlib
import json
import math
from typing import Dict, Sequence

from smart_hub.demand.features import FEATURE_NAMES

PERSISTENCE_ID = "persistence-h30"


class PersistenceModel:
    """Mandatory baseline: q_hat(t+30) = q_t."""

    model_id = PERSISTENCE_ID
    horizon_sec = 30

    def predict(self, features: Sequence[float]) -> float:
        return max(0.0, float(features[0]))


class LinearDemandModel:
    """Standardized linear model: max(0, b + sum(w_i * (x_i - mu_i) / s_i))."""

    def __init__(self, model_id: str, feature_names: Sequence[str],
                 mean: Sequence[float], scale: Sequence[float],
                 coef: Sequence[float], intercept: float,
                 horizon_sec: int = 30, metadata: Dict = None):
        n = len(feature_names)
        if tuple(feature_names) != tuple(FEATURE_NAMES):
            raise ValueError(
                "model features %s do not match code features %s"
                % (list(feature_names), list(FEATURE_NAMES))
            )
        if not (len(mean) == len(scale) == len(coef) == n):
            raise ValueError("model vector lengths do not match feature count")
        values = list(mean) + list(scale) + list(coef) + [intercept]
        if not all(math.isfinite(float(v)) for v in values):
            raise ValueError("model contains non-finite values")
        if any(float(s) <= 0 for s in scale):
            raise ValueError("model scale values must be positive")
        if int(horizon_sec) <= 0:
            raise ValueError("horizon_sec must be positive")
        self.model_id = str(model_id)
        self.feature_names = tuple(feature_names)
        self.mean = [float(v) for v in mean]
        self.scale = [float(v) for v in scale]
        self.coef = [float(v) for v in coef]
        self.intercept = float(intercept)
        self.horizon_sec = int(horizon_sec)
        self.metadata = dict(metadata or {})

    def predict(self, features: Sequence[float]) -> float:
        if len(features) != len(self.coef):
            raise ValueError("wrong feature count")
        y = self.intercept
        for x, m, s, w in zip(features, self.mean, self.scale, self.coef):
            y += w * (float(x) - m) / s
        return max(0.0, y)

    def to_dict(self) -> Dict:
        return {
            "model_id": self.model_id,
            "model_type": "standardized_linear",
            "horizon_sec": self.horizon_sec,
            "feature_names": list(self.feature_names),
            "mean": self.mean,
            "scale": self.scale,
            "coef": self.coef,
            "intercept": self.intercept,
            "metadata": self.metadata,
        }

    @staticmethod
    def make_model_id(prefix: str, payload: Dict) -> str:
        """Content-derived ID, so a changed model can never reuse an ID."""
        blob = json.dumps(payload, sort_keys=True).encode("utf-8")
        return "%s-%s" % (prefix, hashlib.sha256(blob).hexdigest()[:10])


def load_model(path: str) -> LinearDemandModel:
    """Load and validate an exported model; raises on any inconsistency."""
    with open(path, "r", encoding="utf-8") as fh:
        data = json.load(fh)
    if data.get("model_type") != "standardized_linear":
        raise ValueError("unsupported model_type: %r" % data.get("model_type"))
    model = LinearDemandModel(
        model_id=data["model_id"],
        feature_names=data["feature_names"],
        mean=data["mean"],
        scale=data["scale"],
        coef=data["coef"],
        intercept=data["intercept"],
        horizon_sec=data.get("horizon_sec", 30),
        metadata=data.get("metadata", {}),
    )
    # Recompute the content hash so a hand-edited file is detected.
    expected = LinearDemandModel.make_model_id(
        model.model_id.rsplit("-", 1)[0], _hash_payload(model))
    if expected != model.model_id:
        raise ValueError("model_id does not match file content (edited file?)")
    return model


def _hash_payload(model: LinearDemandModel) -> Dict:
    return {
        "features": list(model.feature_names),
        "mean": model.mean,
        "scale": model.scale,
        "coef": model.coef,
        "intercept": model.intercept,
        "horizon_sec": model.horizon_sec,
    }


def build_linear_model(prefix: str, mean, scale, coef, intercept,
                       horizon_sec: int = 30, metadata: Dict = None
                       ) -> LinearDemandModel:
    """Create a model whose ID is the hash of its numeric content."""
    tmp = LinearDemandModel("tmp", FEATURE_NAMES, mean, scale, coef,
                            intercept, horizon_sec, metadata)
    tmp.model_id = LinearDemandModel.make_model_id(prefix, _hash_payload(tmp))
    return tmp
