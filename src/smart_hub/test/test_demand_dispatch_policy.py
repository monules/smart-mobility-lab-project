import unittest

from smart_hub.demand.dispatch_policy import DispatchPolicy


def run(policy, signals, t0=0.0, forecast_valid=True):
    out = []
    for i, s in enumerate(signals):
        out.append(policy.update(t0 + i, forecast_valid, s, True, s))
    return out


class TestDispatchPolicy(unittest.TestCase):
    def test_three_confirmations_then_single_dispatch(self):
        p = DispatchPolicy()
        d = run(p, [6, 6, 6, 7, 8, 9])
        self.assertEqual([x.dispatch for x in d],
                         [False, False, True, False, False, False])
        self.assertEqual(d[2].reason, 'PREDICTED')
        self.assertEqual(d[3].reason, 'MISSION_OUTSTANDING')
        self.assertEqual(p.dispatch_count, 1)

    def test_cooldown_after_mission(self):
        p = DispatchPolicy()
        run(p, [6, 6, 6])
        p.on_mission_finished()
        d = p.update(10.0, True, 8, True, 8)
        self.assertEqual(d.reason, 'COOLDOWN')
        d = p.update(32.0, True, 8, True, 8)
        self.assertTrue(d.dispatch)

    def test_hysteresis_clear(self):
        p = DispatchPolicy()
        run(p, [6, 6, 6])
        p.on_mission_finished()
        d = run(p, [3, 3, 3, 3], t0=100)
        self.assertTrue(d[-1].escalated)
        d = run(p, [3], t0=104)
        self.assertFalse(d[-1].escalated)

    def test_noise_around_threshold_never_dispatches(self):
        p = DispatchPolicy()
        d = run(p, [6, 5, 6, 5.9, 6, 6, 5, 6, 6, 5.5] * 5)
        self.assertFalse(any(x.dispatch for x in d))

    def test_invalid_forecast_uses_labelled_fallback(self):
        p = DispatchPolicy()
        d = [p.update(i, False, -1.0, True, 7) for i in range(3)]
        self.assertTrue(d[-1].dispatch)
        self.assertEqual(d[-1].reason, 'FALLBACK')

    def test_no_valid_signal_no_dispatch(self):
        p = DispatchPolicy()
        d = [p.update(i, False, -1.0, False, -1.0) for i in range(10)]
        self.assertFalse(any(x.dispatch for x in d))
        self.assertEqual(d[0].reason, 'NO_VALID_SIGNAL')

    def test_source_switch_restarts_confirmation(self):
        p = DispatchPolicy()
        p.update(0, True, 7, True, 7)
        p.update(1, True, 7, True, 7)
        d = p.update(2, False, -1, True, 7)  # forecast lost: fallback
        self.assertFalse(d.dispatch)

    def test_reactive_ignores_forecast(self):
        p = DispatchPolicy(mode='reactive')
        d = [p.update(i, True, 9, True, 2) for i in range(5)]
        self.assertFalse(any(x.dispatch for x in d))

    def test_bad_thresholds(self):
        with self.assertRaises(ValueError):
            DispatchPolicy(high=4, low=6)
        with self.assertRaises(ValueError):
            DispatchPolicy(mode='greedy')


if __name__ == '__main__':
    unittest.main()
