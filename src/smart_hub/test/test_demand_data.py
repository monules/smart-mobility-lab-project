"""Simulation determinism, data-leakage guards and the end-to-end T03 run."""

import importlib.util
import json
import os
import tempfile
import unittest

from smart_hub.demand.estimator import DemandEstimator
from smart_hub.demand.models import PersistenceModel
from smart_hub.demand.simulation import (PROFILES, make_spec, run_episode,
                                         supervised_rows)

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
TRAIN = os.path.join(ROOT, "scripts", "train_demand_model.py")


def load_train_module():
    spec = importlib.util.spec_from_file_location("train_demand_model", TRAIN)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


class TestSimulation(unittest.TestCase):
    def test_deterministic(self):
        def strip(ticks):  # wall-clock latency is the only non-seeded field
            return [{k: v for k, v in r.items() if k != "inference_ms"}
                    for r in ticks]
        a = run_episode(make_spec(1003))
        b = run_episode(make_spec(1003))
        self.assertEqual(strip(a.ticks), strip(b.ticks))
        self.assertEqual(a.observations, b.observations)

    def test_arrivals_paired_across_policies(self):
        counts = []
        for mode in ("reactive", "predictive"):
            spec = make_spec(1004)
            spec.bus_mode, spec.policy_mode = "policy", mode
            r = run_episode(spec)
            counts.append(len(r.passengers) + r.turned_away)
        self.assertEqual(counts[0], counts[1])

    def test_one_outstanding_mission_in_closed_loop(self):
        for seed in range(2000, 2014):
            spec = make_spec(seed, "noisy_threshold")
            spec.bus_mode, spec.policy_mode = "policy", "predictive"
            r = run_episode(spec)
            for prev, nxt in zip(r.dispatches, r.dispatches[1:]):
                self.assertIsNotNone(prev["finished"])
                self.assertGreaterEqual(nxt["t"], prev["finished"])
                self.assertGreaterEqual(nxt["t"] - prev["t"], 30.0)

    def test_features_use_only_past_observations(self):
        """Leakage guard: replaying observations up to t reproduces the
        features computed at t; nothing after t is needed."""
        res = run_episode(make_spec(1025))
        rows = {r["t"]: r for r in res.ticks if r["valid"]}
        for t_check in (70, 150, 300):
            if t_check not in rows:
                continue
            est = DemandEstimator(PersistenceModel())
            last = None
            for (t, c, v, conf, off) in res.observations:
                if t > t_check:
                    break
                est.on_queue(c, v, conf, t + off, t + 0.02, t + 0.02)
                if abs(t - round(t)) < 1e-6:
                    last = est.tick(t + 0.05, t + 0.05)
            self.assertEqual(last.features, rows[t_check]["features"])

    def test_labels_are_not_features(self):
        res = run_episode(make_spec(1001))
        for feats, label, _b, _t in supervised_rows(res, 30, "demand"):
            self.assertEqual(len(feats), 6)

    def test_all_profiles_run(self):
        for i, p in enumerate(PROFILES):
            r = run_episode(make_spec(3000 + i, p))
            self.assertTrue(any(t["valid"] for t in r.ticks), p)


@unittest.skipUnless(importlib.util.find_spec("sklearn")
                     and importlib.util.find_spec("numpy")
                     and os.path.exists(TRAIN),
                     "training dependencies not installed")
class TestTrainingPipeline(unittest.TestCase):
    def test_split_is_whole_episode_and_stratified(self):
        mod = load_train_module()
        seeds = list(range(1000, 1050))
        split = mod.split_episodes(seeds, 42)
        self.assertEqual({k: len(v) for k, v in split.items()},
                         {"train": 30, "val": 10, "test": 10})
        for a, b in (("train", "val"), ("train", "test"), ("val", "test")):
            self.assertFalse(set(split[a]) & set(split[b]))
        for k in split:
            profiles = {make_spec(s).profile for s in split[k]}
            self.assertEqual(profiles, set(PROFILES), k)
        self.assertEqual(split, mod.split_episodes(seeds, 42))

    def test_t03_end_to_end(self):
        mod = load_train_module()
        with tempfile.TemporaryDirectory() as d:
            m = mod.main(["--episodes", "30", "--policy-extra-seeds", "0",
                          "--out-dir", d, "--model-out",
                          os.path.join(d, "model.json"), "--no-plots"])
            self.assertTrue(m["pass_criteria"]["T03_ridge_mae_le_persistence"])
            self.assertTrue(m["pass_criteria"]["T03_inference_p95_lt_100ms"])
            with open(os.path.join(d, "split_manifest.json")) as fh:
                self.assertIn("splits", json.load(fh))


if __name__ == "__main__":
    unittest.main()
