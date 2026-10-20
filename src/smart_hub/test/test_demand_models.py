import json
import os
import tempfile
import unittest

from smart_hub.demand.features import FEATURE_NAMES
from smart_hub.demand.models import (PersistenceModel, build_linear_model,
                                     load_model)

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SHIPPED = os.path.join(ROOT, "models", "demand_ridge_h30.json")


def tiny_model(intercept=1.0):
    n = len(FEATURE_NAMES)
    return build_linear_model("ridge-demand-h30", [0.0] * n, [1.0] * n,
                              [1.0] + [0.0] * (n - 1), intercept)


class TestModels(unittest.TestCase):
    def test_persistence(self):
        m = PersistenceModel()
        self.assertEqual(m.predict([7, 0, 0, 0, 0, 0]), 7.0)
        self.assertEqual(m.horizon_sec, 30)

    def test_linear_prediction_clipped_at_zero(self):
        m = tiny_model(intercept=-100.0)
        self.assertEqual(m.predict([3, 0, 0, 0, 0, 0]), 0.0)
        self.assertEqual(tiny_model().predict([3, 0, 0, 0, 0, 0]), 4.0)

    def test_roundtrip_and_tamper_detection(self):
        m = tiny_model()
        with tempfile.TemporaryDirectory() as d:
            path = os.path.join(d, "m.json")
            with open(path, "w") as fh:
                json.dump(m.to_dict(), fh)
            self.assertEqual(load_model(path).model_id, m.model_id)
            data = m.to_dict()
            data["intercept"] = 2.0  # edited without a new ID
            with open(path, "w") as fh:
                json.dump(data, fh)
            with self.assertRaises(ValueError):
                load_model(path)

    def test_rejects_wrong_feature_order(self):
        n = len(FEATURE_NAMES)
        with self.assertRaises(ValueError):
            build_linear_model("x", [0] * n, [1] * n, [0] * n, 0.0
                               ).__class__("x", list(reversed(FEATURE_NAMES)),
                                           [0] * n, [1] * n, [0] * n, 0.0)

    def test_rejects_non_finite_and_bad_scale(self):
        n = len(FEATURE_NAMES)
        with self.assertRaises(ValueError):
            build_linear_model("x", [0] * n, [1] * n,
                               [float("nan")] * n, 0.0)
        with self.assertRaises(ValueError):
            build_linear_model("x", [0] * n, [0] * n, [1] * n, 0.0)

    @unittest.skipUnless(os.path.exists(SHIPPED), "shipped model not built")
    def test_shipped_model(self):
        m = load_model(SHIPPED)
        self.assertEqual(m.horizon_sec, 30)
        self.assertTrue(m.model_id.startswith("ridge-"))
        meta = m.metadata
        tr, va, te = (set(meta["train_seeds"]), set(meta["val_seeds"]),
                      set(meta["test_seeds"]))
        self.assertFalse(tr & va or tr & te or va & te)
        self.assertGreaterEqual(len(tr | va | te), 30)
        self.assertGreaterEqual(m.predict([0] * len(FEATURE_NAMES)), 0.0)


if __name__ == "__main__":
    unittest.main()
