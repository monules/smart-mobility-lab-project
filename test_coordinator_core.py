import unittest

from coordinator_core import (CrossingArbiter, DispatchPolicy,
                              footprint_overlaps_crossing)


def clear_arbiter(t0=0.0, **kw):
    """Arbiter that has seen the crossing clear for 2.5 s."""
    a = CrossingArbiter(**kw)
    a.update_observation(t0, False, [], True)
    a.update_observation(t0 + 2.5, False, [], True)
    return a, t0 + 2.5


class TestGeometry(unittest.TestCase):
    def test_inside_outside_edge(self):
        self.assertTrue(footprint_overlaps_crossing(5.0, 3.0, 0.3))
        self.assertFalse(footprint_overlaps_crossing(8.0, 4.0, 0.3))
        # 4.25 and 0.25 are exact in binary, so the boundary case is reliable
        self.assertTrue(footprint_overlaps_crossing(4.3125, 3.0, 0.25))   # 0.1875 m away < r
        self.assertFalse(footprint_overlaps_crossing(4.25, 3.0, 0.25))    # exactly r away: touching only


class TestCrossing(unittest.TestCase):
    def test_denied_until_debounce_then_granted(self):
        a = CrossingArbiter()
        a.update_observation(0.0, False, [], True)
        self.assertEqual(a.request(1.0, 'bus_1', 'ACQUIRE', '')[2], 'CLEAR_DEBOUNCE')
        a.update_observation(2.0, False, [], True)
        ok, tok, why = a.request(2.1, 'bus_1', 'ACQUIRE', '')
        self.assertTrue(ok)
        self.assertTrue(tok)

    def test_pedestrian_resets_debounce(self):
        a, t = clear_arbiter()
        a.update_observation(t + 0.1, True, [], True)
        self.assertEqual(a.request(t + 0.2, 'bus_1', 'ACQUIRE', '')[2], 'PEDESTRIAN_PRESENT')
        a.update_observation(t + 1.0, False, [], True)
        self.assertEqual(a.request(t + 2.5, 'bus_1', 'ACQUIRE', '')[2], 'CLEAR_DEBOUNCE')
        a.update_observation(t + 3.1, False, [], True)
        self.assertTrue(a.request(t + 3.2, 'bus_1', 'ACQUIRE', '')[0])

    def test_unknown_state_denies(self):
        a, t = clear_arbiter()
        a.update_observation(t + 0.1, False, [], False)
        self.assertEqual(a.request(t + 0.2, 'bus_1', 'ACQUIRE', '')[2], 'UNKNOWN_STATE')
        self.assertEqual(a.snapshot()['state'], 'UNKNOWN')

    def test_safety_latch_denies(self):
        a, t = clear_arbiter()
        a.safety_latched = True
        self.assertEqual(a.request(t, 'bus_1', 'ACQUIRE', '')[2], 'SAFETY_STOP')

    def test_single_owner(self):
        a, t = clear_arbiter()
        ok, tok, _ = a.request(t, 'bus_1', 'ACQUIRE', '')
        self.assertTrue(ok)
        ok2, _, why = a.request(t + 0.1, 'robot_1', 'ACQUIRE', '')
        self.assertFalse(ok2)
        self.assertEqual(why, 'OWNED_BY_OTHER')

    def test_duplicate_acquire_returns_same_token(self):
        a, t = clear_arbiter()
        _, tok, _ = a.request(t, 'bus_1', 'ACQUIRE', '')
        ok, tok2, why = a.request(t + 0.5, 'bus_1', 'ACQUIRE', '')
        self.assertTrue(ok)
        self.assertEqual(tok, tok2)

    def test_release_only_after_footprint_clear(self):
        a, t = clear_arbiter()
        _, tok, _ = a.request(t, 'bus_1', 'ACQUIRE', '')
        a.update_observation(t + 0.5, False, ['bus_1'], True)
        self.assertEqual(a.request(t + 0.6, 'bus_1', 'RELEASE', tok)[2], 'NOT_CLEAR')
        a.update_observation(t + 1.0, False, [], True)
        self.assertTrue(a.request(t + 1.1, 'bus_1', 'RELEASE', tok)[0])
        self.assertEqual(a.owner, '')

    def test_wrong_token_rejected(self):
        a, t = clear_arbiter()
        a.request(t, 'bus_1', 'ACQUIRE', '')
        self.assertEqual(a.request(t, 'bus_1', 'RELEASE', 'nope')[2], 'BAD_TOKEN')
        self.assertEqual(a.request(t, 'robot_1', 'RENEW', 'nope')[2], 'BAD_TOKEN')

    def test_lease_expiry_does_not_free_occupied_crossing(self):
        a, t = clear_arbiter(lease_sec=3.0)
        a.request(t, 'bus_1', 'ACQUIRE', '')
        for i in range(1, 80):   # bus sits in C and never renews
            a.update_observation(t + i * 0.1, False, ['bus_1'], True)
        self.assertEqual(a.owner, 'bus_1')
        self.assertTrue(a.hold)
        self.assertEqual(a.request(t + 8.0, 'robot_1', 'ACQUIRE', '')[2], 'OWNER_LOST_HOLD')

    def test_hold_reclaimed_only_after_clear_debounce(self):
        a, t = clear_arbiter(lease_sec=1.0)
        a.request(t, 'bus_1', 'ACQUIRE', '')
        a.update_observation(t + 0.5, False, ['bus_1'], True)
        a.update_observation(t + 1.5, False, ['bus_1'], True)   # lease (1 s) expired while inside C
        self.assertTrue(a.hold)
        a.update_observation(t + 2.0, False, [], True)          # bus leaves C, clear starts
        a.update_observation(t + 3.5, False, [], True)          # 1.5 s clear: not enough
        self.assertEqual(a.owner, 'bus_1')
        a.update_observation(t + 4.1, False, [], True)          # 2.1 s clear: reclaimed
        self.assertEqual(a.owner, '')

    def test_expired_lease_on_long_clear_crossing_is_reclaimed(self):
        a, t = clear_arbiter(lease_sec=1.0)
        a.request(t, 'bus_1', 'ACQUIRE', '')                    # bus never enters C
        a.update_observation(t + 1.5, False, [], True)          # crossing already clear > 2 s
        self.assertEqual(a.owner, '')

    def test_renew_keeps_lease(self):
        a, t = clear_arbiter(lease_sec=3.0)
        _, tok, _ = a.request(t, 'bus_1', 'ACQUIRE', '')
        for i in range(1, 10):
            now = t + i * 1.0
            a.update_observation(now, False, ['bus_1'], True)
            self.assertTrue(a.request(now, 'bus_1', 'RENEW', tok)[0])
        self.assertFalse(a.hold)

    def test_fifo_order(self):
        a = CrossingArbiter()
        a.update_observation(0.0, True, [], True)        # pedestrian crossing
        a.request(0.5, 'robot_1', 'ACQUIRE', '')         # robot asked first
        a.request(0.6, 'bus_1', 'ACQUIRE', '')
        a.update_observation(1.0, False, [], True)
        a.update_observation(3.1, False, [], True)       # debounce done
        self.assertEqual(a.request(3.2, 'bus_1', 'ACQUIRE', '')[2], 'WAIT_FIFO')
        self.assertTrue(a.request(3.3, 'robot_1', 'ACQUIRE', '')[0])

    def test_invalid_requests(self):
        a, t = clear_arbiter()
        self.assertEqual(a.request(t, '', 'ACQUIRE', '')[2], 'INVALID_REQUEST')
        self.assertEqual(a.request(t, 'bus_1', 'GRAB', '')[2], 'INVALID_REQUEST')
        self.assertEqual(a.request(t, 'bus_1', 'ACQUIRE', 'tok')[2], 'INVALID_REQUEST')

    def test_never_two_owners_random(self):
        import random
        rnd = random.Random(7)
        a = CrossingArbiter()
        t = 0.0
        grants = 0
        ped = False
        inside = {'bus_1': False, 'robot_1': False}
        for _ in range(6000):
            t += 0.05
            if rnd.random() < 0.01:
                ped = not ped
            for k in inside:
                if rnd.random() < 0.02:
                    inside[k] = not inside[k]
            a.update_observation(t, ped, [k for k, v in inside.items() if v], True)
            who = rnd.choice(['bus_1', 'robot_1'])
            op = rnd.choice(['ACQUIRE', 'ACQUIRE', 'RENEW', 'RELEASE'])
            tok = a.token if op != 'ACQUIRE' and rnd.random() < 0.8 else ''
            before = a.owner
            a.request(t, who, op, tok)
            if not before and a.owner:           # a brand new grant
                grants += 1
                self.assertFalse(a.pedestrian)
                self.assertEqual(a.occupants, frozenset())
                self.assertGreaterEqual(t - a.clear_since, a.clear_debounce)
        self.assertGreater(grants, 0)


class TestDispatch(unittest.TestCase):
    def feed(self, p, values, t0=0.0, valid=True, idle=True):
        out = []
        for i, v in enumerate(values):
            out.append(p.step(t0 + i, valid, v, True, 0, idle))
        return out

    def test_three_highs_dispatch_once(self):
        p = DispatchPolicy()
        res = self.feed(p, [7, 7, 7, 7, 7, 7])
        self.assertEqual([r is not None for r in res], [False, False, True, False, False, False])
        self.assertEqual(res[2].source, 'PREDICTED')

    def test_noise_around_threshold_no_dispatch(self):
        p = DispatchPolicy()
        res = self.feed(p, [7, 7, 5.9, 7, 7, 5.9, 7, 7])
        self.assertTrue(all(r is None for r in res))

    def test_no_second_dispatch_while_outstanding(self):
        p = DispatchPolicy()
        first = self.feed(p, [7, 7, 7])[2]
        self.assertTrue(first)
        res = self.feed(p, [1, 1, 1, 1, 1, 7, 7, 7, 7], t0=100.0)  # lows clear latch, but mission still active
        self.assertTrue(all(r is None for r in res))

    def test_redispatch_needs_finish_cooldown_and_five_lows(self):
        p = DispatchPolicy()
        d = self.feed(p, [7, 7, 7])[2]
        self.assertTrue(p.finished(d.correlation_id))
        # still escalated: no new dispatch even after cooldown
        self.assertTrue(all(r is None for r in self.feed(p, [7, 7, 7, 7], t0=50.0)))
        self.feed(p, [2, 2, 2, 2, 2], t0=60.0)           # latch cleared
        res = self.feed(p, [7, 7, 7], t0=70.0)
        self.assertIsNotNone(res[2])

    def test_cooldown_blocks(self):
        p = DispatchPolicy()
        d = self.feed(p, [7, 7, 7])[2]                    # dispatched at t=2
        p.finished(d.correlation_id)
        self.feed(p, [2] * 5, t0=3.0)                     # latch cleared at t=7
        res = self.feed(p, [7, 7, 7], t0=8.0)             # t=10 < 2+30
        self.assertTrue(all(r is None for r in res))

    def test_invalid_forecast_never_dispatches(self):
        p = DispatchPolicy()
        out = [p.step(i, False, 9, False, 9, True) for i in range(10)]
        self.assertTrue(all(r is None for r in out))

    def test_fallback_uses_observed_count(self):
        p = DispatchPolicy()
        out = [p.step(i, False, 0, True, 7, True) for i in range(4)]
        self.assertEqual(out[2].source, 'FALLBACK')

    def test_source_switch_resets_counter(self):
        p = DispatchPolicy()
        a = p.step(0, True, 7, True, 7, True)
        b = p.step(1, True, 7, True, 7, True)
        c = p.step(2, False, 0, True, 7, True)            # fallback sample #1
        self.assertTrue(a is None and b is None and c is None)

    def test_bus_not_idle_blocks(self):
        p = DispatchPolicy()
        res = self.feed(p, [7] * 5, idle=False)
        self.assertTrue(all(r is None for r in res))

    def test_late_response_ignored(self):
        p = DispatchPolicy()
        d = self.feed(p, [7, 7, 7])[2]
        self.assertFalse(p.finished('dock-999-old'))
        self.assertEqual(p.outstanding, d.correlation_id)
        self.assertTrue(p.finished(d.correlation_id))
        self.assertFalse(p.finished(d.correlation_id))    # duplicate result


if __name__ == '__main__':
    unittest.main()
