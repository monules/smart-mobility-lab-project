import unittest

from smart_hub.demand.features import (FEATURE_NAMES, FEATURE_WINDOW,
                                       DemandHistory, extract_features)


class TestExtractFeatures(unittest.TestCase):
    def test_known_window(self):
        # 31 samples: 20 zeros then 0,1,2,...,10 (rising queue)
        w = [0] * 20 + list(range(11))
        f = dict(zip(FEATURE_NAMES, extract_features(w)))
        self.assertEqual(f["q_t"], 10)
        self.assertAlmostEqual(f["mean_10s"], sum(range(1, 11)) / 10.0)
        self.assertAlmostEqual(f["mean_30s"], sum(range(11)) / 30.0)
        self.assertEqual(f["trend_10s"], 10 - 0)
        self.assertEqual(f["arrivals_30s"], 10)
        self.assertEqual(f["departures_30s"], 0)

    def test_departures_proxy(self):
        w = [5] * 25 + [5, 9, 9, 5, 5, 4]  # +4 then -4 then -1
        f = dict(zip(FEATURE_NAMES, extract_features(w)))
        self.assertEqual(f["arrivals_30s"], 4)
        self.assertEqual(f["departures_30s"], 5)

    def test_proxy_misses_simultaneous_join_and_leave(self):
        # Documented limitation: a constant count hides offsetting events.
        f = dict(zip(FEATURE_NAMES, extract_features([3] * FEATURE_WINDOW)))
        self.assertEqual(f["arrivals_30s"], 0)
        self.assertEqual(f["departures_30s"], 0)

    def test_uses_only_last_window(self):
        a = extract_features([99] * 50 + [1] * FEATURE_WINDOW)
        b = extract_features([1] * FEATURE_WINDOW)
        self.assertEqual(a, b)

    def test_too_short_raises(self):
        with self.assertRaises(ValueError):
            extract_features([1] * (FEATURE_WINDOW - 1))


class TestDemandHistory(unittest.TestCase):
    def test_warmup_requires_60_valid_samples(self):
        h = DemandHistory(60, 60, 60)
        for _ in range(59):
            h.push(3)
            self.assertEqual(h.status(), "WARMUP")
        h.push(3)
        self.assertEqual(h.status(), "OK")
        self.assertEqual(len(h.feature_window()), FEATURE_WINDOW)

    def test_gap_then_recovery_without_filling(self):
        h = DemandHistory(60, 60, 60)
        for _ in range(60):
            h.push(2)
        h.push(None)
        self.assertEqual(h.status(), "STALE_INPUT")
        with self.assertRaises(RuntimeError):
            h.feature_window()
        for i in range(FEATURE_WINDOW - 1):
            h.push(2)
            self.assertEqual(h.status(), "GAP_RECOVERY", i)
        h.push(2)
        self.assertEqual(h.status(), "OK")
        self.assertNotIn(None, h.feature_window())

    def test_long_gap_resets_history(self):
        h = DemandHistory(60, 60, reset_gap_sec=10)
        for _ in range(60):
            h.push(4)
        for _ in range(10):
            h.push(None)
        self.assertEqual(h.valid_total, 0)
        for _ in range(40):
            h.push(4)
        self.assertEqual(h.status(), "WARMUP")

    def test_stale_never_reported_as_zero(self):
        h = DemandHistory(60, 60, 60)
        for _ in range(60):
            h.push(0)
        h.push(None)
        self.assertNotEqual(h.status(), "OK")

    def test_bad_parameters(self):
        with self.assertRaises(ValueError):
            DemandHistory(60, warmup_sec=10)
        with self.assertRaises(ValueError):
            DemandHistory(history_sec=20)
        with self.assertRaises(ValueError):
            DemandHistory(reset_gap_sec=0)


if __name__ == "__main__":
    unittest.main()
