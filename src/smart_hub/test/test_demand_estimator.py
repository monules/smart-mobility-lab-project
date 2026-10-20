import unittest

from smart_hub.demand.estimator import INVALID_VALUE, DemandEstimator
from smart_hub.demand.models import PersistenceModel


def feed(est, seconds, count=4, start=0.0, valid=True, conf=0.9):
    """Publish at 5 Hz and tick at 1 Hz; return the list of predictions."""
    preds = []
    for k in range(int(seconds * 5)):
        t = start + k * 0.2
        est.on_queue(count, valid, conf, t, t, t)
        if k % 5 == 4:
            preds.append(est.tick(t + 0.05, t + 0.05))
    return preds


class Exploding:
    model_id = "exploding"
    horizon_sec = 30

    def predict(self, features):
        raise RuntimeError("boom")


class TestEstimator(unittest.TestCase):
    def setUp(self):
        self.est = DemandEstimator(PersistenceModel())

    def test_warmup_then_valid(self):
        preds = feed(self.est, 61)
        self.assertTrue(all(not p.valid for p in preds[:59]))
        self.assertTrue(all(p.reason == "WARMUP" for p in preds[:59]))
        self.assertTrue(preds[59].valid)
        self.assertEqual(preds[59].predicted_count, 4.0)
        self.assertEqual(preds[59].baseline_count, 4.0)
        self.assertEqual(preds[59].model_id, PersistenceModel.model_id)

    def test_invalid_prediction_is_never_zero(self):
        p = feed(self.est, 5, count=0)[-1]
        self.assertFalse(p.valid)
        self.assertEqual(p.predicted_count, INVALID_VALUE)

    def test_output_stamp_is_observation_time(self):
        p = feed(self.est, 61)[-1]
        self.assertAlmostEqual(p.source_stamp, 60.8)

    def test_feed_freeze_invalid_within_1_2_s(self):
        feed(self.est, 70)
        # Camera freezes at t=70: no more messages. First tick after the
        # freeze must already be invalid (< 1.2 s requirement).
        p = self.est.tick(70.85, 70.85)
        self.assertFalse(p.valid)
        self.assertEqual(p.reason, "STALE_INPUT")
        self.assertEqual(p.baseline_count, INVALID_VALUE)

    def test_frozen_source_stamp_is_stale(self):
        feed(self.est, 70)
        for k in range(10):  # same stamp re-sent for 2 s
            r = self.est.on_queue(4, True, 0.9, 69.8, 70 + k * 0.2,
                                  70 + k * 0.2)
            self.assertEqual(r, "FROZEN_STAMP")
        p = self.est.tick(71.05, 71.05)
        self.assertFalse(p.valid)

    def test_receipt_age_makes_sample_stale(self):
        feed(self.est, 70)
        self.est.on_queue(4, True, 0.9, 70.0, 70.0, 70.0)
        p = self.est.tick(71.5, 71.5)  # 1.5 s after receipt
        self.assertFalse(p.valid)

    def test_old_source_stamp_rejected_even_if_just_received(self):
        feed(self.est, 70)
        self.est.on_queue(4, True, 0.9, 70.0, 72.0, 72.0)  # 2 s old stamp
        p = self.est.tick(72.05, 72.05)
        self.assertFalse(p.valid)

    def test_rejections(self):
        e = self.est
        self.assertEqual(e.on_queue(3, False, 0.9, 1, 1, 1), "INVALID_FLAG")
        self.assertEqual(e.on_queue(3, True, 0.1, 1, 1, 1), "LOW_CONFIDENCE")
        self.assertEqual(e.on_queue(-1, True, 0.9, 1, 1, 1), "BAD_COUNT")
        self.assertEqual(e.on_queue(float("nan"), True, 0.9, 1, 1, 1),
                         "BAD_COUNT")
        self.assertEqual(e.on_queue(3, True, 0.9, 5.0, 1, 1), "FUTURE_STAMP")
        self.assertEqual(e.on_queue(3, True, 0.9, 1.0, 1, 1), "ACCEPTED")

    def test_invalid_message_cancels_pending_sample(self):
        feed(self.est, 70)
        self.est.on_queue(4, True, 0.9, 70.0, 70.0, 70.0)
        self.est.on_queue(0, False, 0.0, 70.2, 70.2, 70.2)
        p = self.est.tick(70.25, 70.25)
        self.assertFalse(p.valid)

    def test_recovery_after_gap_takes_feature_window(self):
        feed(self.est, 70)
        self.est.tick(71.05, 71.05)  # gap second
        preds = feed(self.est, 32, start=71.2)
        reasons = [p.reason for p in preds]
        self.assertEqual(reasons[:30], ["GAP_RECOVERY"] * 30)
        self.assertTrue(preds[30].valid)

    def test_model_fault_gives_invalid_not_crash(self):
        e = DemandEstimator(Exploding())
        p = feed(e, 61)[-1]
        self.assertFalse(p.valid)
        self.assertEqual(p.reason, "MODEL_ERROR")

    def test_inference_latency_under_100ms(self):
        preds = [p for p in feed(self.est, 300) if p.valid]
        lat = sorted(p.inference_ms for p in preds)
        self.assertLess(lat[int(0.95 * len(lat))], 100.0)

    def test_bad_parameters(self):
        with self.assertRaises(ValueError):
            DemandEstimator(PersistenceModel(), stale_timeout_sec=0)
        with self.assertRaises(ValueError):
            DemandEstimator(PersistenceModel(), confidence_min=1.5)


if __name__ == "__main__":
    unittest.main()
