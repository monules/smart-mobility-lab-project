"""T08 - S4 expiry & reconciliation, plus barrier interlock and the exact
section 11.3 test sequence.  Pure Python, deterministic (injected clock)."""
import unittest

from helpers import feed, make_engine, paper_scene
from smart_hub.parking_engine import (BarrierController, FREE, OCCUPIED,
                                      RESERVED, UNKNOWN)


class Debounce(unittest.TestCase):
    def test_all_slots_start_unknown_and_are_not_allocatable(self):
        e = make_engine()
        self.assertEqual({v.state for v in e.snapshot(0.0)[0]}, {UNKNOWN})
        r = e.reserve('r', 'C1', '', 30, 0.0)
        self.assertFalse(r.success)
        self.assertEqual(r.reason, 'FULL')

    def test_change_needs_half_a_second_of_stable_readings(self):
        e = make_engine()
        for t in (0.0, 0.2, 0.4):                       # only 0.4 s so far
            e.observe('P2', False, True, 1.0, t)
        self.assertEqual(e.effective_state('P2', 0.4), UNKNOWN)
        e.observe('P2', False, True, 1.0, 0.5)
        self.assertEqual(e.effective_state('P2', 0.5), FREE)

    def test_flicker_never_commits(self):
        e = make_engine()
        t = 0.0
        for i in range(20):                             # alternating readings
            e.observe('P2', i % 2 == 0, True, 1.0, t)
            t += 0.1
        self.assertEqual(e.effective_state('P2', t), UNKNOWN)

    def test_invalid_or_low_confidence_reading_means_unknown(self):
        e = make_engine()
        feed(e, {'P2': 'FREE'}, 0.0)
        self.assertEqual(e.effective_state('P2', 1.0), FREE)
        e.observe('P2', False, valid=False, now=1.1)
        self.assertEqual(e.effective_state('P2', 1.1), UNKNOWN)
        feed(e, {'P2': 'FREE'}, 2.0)
        e.observe('P2', False, True, confidence=0.1, now=3.1)
        self.assertEqual(e.effective_state('P2', 3.1), UNKNOWN)

    def test_slot_not_observed_for_a_second_goes_unknown(self):
        e = make_engine()
        last = feed(e, {'P3': 'FREE'}, 0.0)
        self.assertEqual(e.effective_state('P3', last + 0.9), FREE)
        self.assertEqual(e.effective_state('P3', last + 1.1), UNKNOWN)


class ExpiryAndReconciliation(unittest.TestCase):
    def test_expiry_never_frees_an_occupied_slot(self):
        e = make_engine()
        t = feed(e, {'P2': 'FREE'}, 0.0)
        r = e.reserve('a', 'C1', 'P2', 5.0, t)
        self.assertTrue(r.success)
        t = feed(e, {'P2': 'OCCUPIED'}, t + 0.2)        # C1 parks in its slot
        self.assertEqual(e.effective_state('P2', t), OCCUPIED)
        view = {v.slot_id: v for v in e.snapshot(t)[0]}['P2']
        self.assertEqual(view.reservation_id, r.reservation_id)   # metadata kept
        t_exp = r.expires_mono + 0.01
        feed(e, {'P2': 'OCCUPIED'}, t, duration=t_exp - t)        # sensor keeps seeing the car
        e.tick(t_exp)
        view = {v.slot_id: v for v in e.snapshot(t_exp)[0]}['P2']
        self.assertEqual(view.state, OCCUPIED)                    # NOT FREE
        self.assertEqual(view.reservation_id, '')                 # admin hold dropped

    def test_expiry_with_unobserved_slot_is_unknown_not_free(self):
        e = make_engine()
        t = feed(e, {'P3': 'FREE'}, 0.0)
        r = e.reserve('a', 'C1', 'P3', 5.0, t)
        self.assertEqual(e.effective_state('P3', t), RESERVED)
        t_exp = r.expires_mono + 0.01                             # sensor silent for 5 s
        e.tick(t_exp)
        self.assertEqual(e.effective_state('P3', t_exp), UNKNOWN)
        self.assertFalse(e.reserve('b', 'C2', 'P3', 30, t_exp).success)

    def test_expiry_with_fresh_empty_sensor_returns_to_free(self):
        e = make_engine()
        t = feed(e, {'P2': 'FREE'}, 0.0)
        r = e.reserve('a', 'C1', 'P2', 5.0, t)
        t_exp = r.expires_mono + 0.01
        feed(e, {'P2': 'FREE'}, t, duration=t_exp - t)
        e.tick(t_exp)
        self.assertEqual(e.effective_state('P2', t_exp), FREE)

    def test_slot_becomes_free_only_after_sensor_confirms_empty(self):
        e = make_engine()
        t = feed(e, {'P2': 'OCCUPIED'}, 0.0)
        self.assertEqual(e.effective_state('P2', t), OCCUPIED)
        e.observe('P2', False, True, 1.0, t + 0.1)       # first "empty" reading
        self.assertNotEqual(e.effective_state('P2', t + 0.1), FREE)
        t2 = feed(e, {'P2': 'FREE'}, t + 0.1, duration=0.6)
        self.assertEqual(e.effective_state('P2', t2), FREE)

    def test_reserved_slot_is_excluded_from_available_count(self):
        e = make_engine()
        t = paper_scene(e)
        self.assertEqual(e.snapshot(t)[1], 2)
        e.reserve('a', 'C1', 'P2', 30, t)
        self.assertEqual(e.snapshot(t)[1], 1)

    def test_ttl_bounds(self):
        e = make_engine()
        t = paper_scene(e)
        for i, ttl in enumerate((0.0, 4.99, 120.01, float('nan'), -3.0)):
            self.assertEqual(e.reserve('bad%d' % i, 'V%d' % i, '', ttl, t).reason,
                             'INVALID_TTL')
        self.assertTrue(e.reserve('lo', 'L', 'P2', 5.0, t).success)
        self.assertTrue(e.reserve('hi', 'H', 'P3', 120.0, t).success)

    def test_all_full_returns_reason_and_never_hangs(self):
        e = make_engine()
        t = feed(e, {'P1': 'OCCUPIED', 'P2': 'OCCUPIED', 'P3': 'OCCUPIED'}, 0.0)
        r = e.reserve('x', 'C1', '', 30, t)
        self.assertEqual((r.success, r.reason), (False, 'FULL'))


class TokenHandling(unittest.TestCase):
    def setUp(self):
        self.e = make_engine()
        self.t = paper_scene(self.e)

    def test_duplicate_request_returns_same_token_and_no_second_slot(self):
        a = self.e.reserve('r1', 'C1', 'P2', 30, self.t)
        b = self.e.reserve('r1', 'C1', 'P2', 30, self.t + 1)
        self.assertEqual((a.reservation_id, a.slot_id), (b.reservation_id, b.slot_id))
        self.assertTrue(b.replayed)
        self.assertEqual(sum(1 for v in self.e.snapshot(self.t)[0] if v.reservation_id), 1)

    def test_key_reuse_with_different_payload_is_rejected(self):
        self.e.reserve('r1', 'C1', 'P2', 30, self.t)
        for args in (('C9', 'P2', 30), ('C1', 'P3', 30), ('C1', 'P2', 60)):
            r = self.e.reserve('r1', *args, self.t)
            self.assertEqual((r.success, r.reason), (False, 'KEY_REUSE'))

    def test_one_slot_per_vehicle(self):
        self.e.reserve('r1', 'C1', 'P2', 30, self.t)
        r = self.e.reserve('r2', 'C1', 'P3', 30, self.t)
        self.assertEqual(r.reason, 'VEHICLE_ALREADY_RESERVED')

    def test_wrong_token_and_wrong_vehicle_release_rejected(self):
        a = self.e.reserve('r1', 'C1', 'P2', 30, self.t)
        self.assertEqual(self.e.release('x1', 'deadbeef', 'C1', self.t).reason,
                         'UNKNOWN_TOKEN')
        self.assertEqual(self.e.release('x2', a.reservation_id, 'C2', self.t).reason,
                         'VEHICLE_MISMATCH')
        self.assertEqual(self.e.effective_state('P2', self.t), RESERVED)  # untouched

    def test_valid_release_frees_slot_when_sensor_says_empty(self):
        a = self.e.reserve('r1', 'C1', 'P2', 30, self.t)
        r = self.e.release('x1', a.reservation_id, 'C1', self.t)
        self.assertTrue(r.success)
        self.assertEqual(self.e.effective_state('P2', self.t), FREE)
        # replaying the release is idempotent, a new id on a dead token fails
        self.assertTrue(self.e.release('x1', a.reservation_id, 'C1', self.t).replayed)
        self.assertFalse(self.e.release('x9', a.reservation_id, 'C1', self.t).success)

    def test_release_never_overrides_an_occupied_sensor(self):
        a = self.e.reserve('r1', 'C1', 'P2', 30, self.t)
        t = feed(self.e, {'P2': 'OCCUPIED'}, self.t, duration=0.8)
        r = self.e.release('x1', a.reservation_id, 'C1', t)
        self.assertEqual((r.success, r.reason), (True, 'OK_SLOT_OCCUPIED'))
        self.assertEqual(self.e.effective_state('P2', t), OCCUPIED)

    def test_expired_token_cannot_free_a_new_owners_reservation(self):
        a = self.e.reserve('r1', 'C1', 'P2', 5.0, self.t)
        t = a.expires_mono + 0.01
        feed(self.e, {'P2': 'FREE', 'P3': 'FREE'}, self.t, duration=t - self.t)
        b = self.e.reserve('r2', 'C2', 'P2', 30, t)           # new owner of P2
        self.assertEqual(b.slot_id, 'P2')
        self.assertNotEqual(a.reservation_id, b.reservation_id)
        stale = self.e.release('x1', a.reservation_id, 'C1', t)
        self.assertEqual((stale.success, stale.reason), (False, 'UNKNOWN_TOKEN'))
        self.assertEqual(self.e.effective_state('P2', t), RESERVED)
        stale_by_new_vehicle = self.e.release('x2', a.reservation_id, 'C2', t)
        self.assertFalse(stale_by_new_vehicle.success)

    def test_invalid_requests(self):
        e = self.e
        self.assertEqual(e.reserve('', 'C1', '', 30, self.t).reason, 'INVALID_REQUEST')
        self.assertEqual(e.reserve('r', '', '', 30, self.t).reason, 'INVALID_REQUEST')
        self.assertEqual(e.reserve('r', 'C1', 'P9', 30, self.t).reason, 'INVALID_SLOT')
        self.assertEqual(e.release('', 'tok', 'C1', self.t).reason, 'INVALID_REQUEST')


class Section113Sequence(unittest.TestCase):
    """Assignment 11.3, step by step (clock in seconds)."""

    def test_exact_sequence(self):
        e = make_engine()
        # t = 0..4: P1 OCCUPIED, P2 FREE, P3 FREE
        feed(e, {'P1': 'OCCUPIED', 'P2': 'FREE', 'P3': 'FREE'}, 0.0, duration=4.0)
        # t = 5: C1 and C2 both prefer P2
        c1 = e.reserve('c1-req', 'C1', 'P2', 35.0, 5.0)
        c2 = e.reserve('c2-req', 'C2', 'P2', 35.0, 5.0)
        self.assertEqual(sorted([c1.slot_id, c2.slot_id]), ['P2', 'P3'])
        c1_slot = c1.slot_id
        # t = 6: C1 repeats its request id -> same token, nothing new allocated
        again = e.reserve('c1-req', 'C1', 'P2', 35.0, 6.0)
        self.assertEqual(again.reservation_id, c1.reservation_id)
        self.assertEqual(sum(1 for v in e.snapshot(6.0)[0] if v.reservation_id), 2)
        # t = 8: C1 is confirmed occupying its slot (sensor stable >= 0.5 s)
        for t in (6.0, 6.5, 7.0, 7.5, 8.0):
            e.observe('P1', True, True, 1.0, t)
            e.observe(c1_slot, True, True, 1.0, t)
            other = 'P3' if c1_slot == 'P2' else 'P2'
            e.observe(other, False, True, 1.0, t)
        self.assertEqual(e.effective_state(c1_slot, 8.0), OCCUPIED)
        # t = 10: P3's sensor disconnects (no more readings for it)
        for t in (8.5, 9.0, 9.5, 10.0, 10.5, 11.0, 11.5, 12.0):
            e.observe('P1', True, True, 1.0, t)
            if c1_slot != 'P3':
                e.observe(c1_slot, True, True, 1.0, t)
        e.tick(12.0)
        self.assertEqual(e.effective_state('P3', 12.0), UNKNOWN)
        # any new request cannot be given the unobserved / occupied slots
        late = e.reserve('late', 'C3', '', 30, 12.0)
        self.assertFalse(late.success)
        self.assertEqual(late.reason, 'FULL')
        # t = 40: remaining reservations expire; nothing becomes falsely free
        for t in [x * 0.5 for x in range(26, 81)]:
            e.observe('P1', True, True, 1.0, t)
            if c1_slot != 'P3':
                e.observe(c1_slot, True, True, 1.0, t)
        e.tick(40.0)
        states = {v.slot_id: v for v in e.snapshot(40.0)[0]}
        self.assertEqual(states['P1'].state, OCCUPIED)
        self.assertEqual(states[c1_slot].state, OCCUPIED)         # still physically there
        self.assertEqual(states['P3'].state, UNKNOWN)             # still unobserved
        self.assertTrue(all(v.reservation_id == '' for v in states.values()))
        self.assertEqual(e.snapshot(40.0)[1], 0)                  # advertises nothing free
        self.assertFalse(e.reserve('after', 'C4', '', 30, 40.0).success)


class Barrier(unittest.TestCase):
    def setUp(self):
        self.e = make_engine()
        self.t = paper_scene(self.e)
        self.res = self.e.reserve('r1', 'C1', 'P2', 60, self.t)
        self.b = BarrierController(self.e, hold_sec=8.0, close_clear_sec=1.0,
                                   ack_timeout_sec=2.0)

    def events(self):
        return [(x.event, x.reason) for x in self.e.drain_events()]

    def test_valid_owner_with_fresh_clear_entrance_opens(self):
        self.b.on_entrance(True, self.t)
        ok, reason = self.b.on_claim('C1', self.res.reservation_id, self.t + 0.1)
        self.assertEqual((ok, reason), (True, 'OK'))
        self.assertTrue(self.b.desired_open)
        self.assertIn(('BARRIER_OPEN_REQUEST', 'OK'), self.events())

    def test_wrong_token_wrong_vehicle_unknown_token_denied(self):
        self.b.on_entrance(True, self.t)
        self.assertEqual(self.b.on_claim('C1', 'nope', self.t)[1], 'UNKNOWN_TOKEN')
        self.assertEqual(self.b.on_claim('C2', self.res.reservation_id, self.t)[1],
                         'VEHICLE_MISMATCH')
        self.assertFalse(self.b.desired_open)

    def test_stale_or_occupied_entrance_sensor_denies(self):
        self.assertEqual(self.b.on_claim('C1', self.res.reservation_id, self.t)[1],
                         'ENTRANCE_SENSOR_STALE')               # never heard from it
        self.b.on_entrance(True, self.t)
        t = feed(self.e, {'P2': 'FREE', 'P3': 'FREE'}, self.t, duration=1.6)  # slots stay fresh
        self.assertEqual(self.b.on_claim('C1', self.res.reservation_id, t)[1],
                         'ENTRANCE_SENSOR_STALE')               # entrance sensor went quiet
        self.b.on_entrance(False, t)
        self.assertEqual(self.b.on_claim('C1', self.res.reservation_id, t + 0.1)[1],
                         'ENTRANCE_NOT_CLEAR')
        self.assertFalse(self.b.desired_open)

    def test_expired_reservation_cannot_open(self):
        self.b.on_entrance(True, self.res.expires_mono + 1)
        ok, reason = self.b.on_claim('C1', self.res.reservation_id,
                                     self.res.expires_mono + 1.1)
        self.assertEqual((ok, reason), (False, 'UNKNOWN_TOKEN'))

    def test_never_closes_on_occupied_or_unobserved_passage(self):
        self.b.on_entrance(True, self.t)
        self.b.on_claim('C1', self.res.reservation_id, self.t)
        t = self.t + 9.0                                       # hold time over
        self.b.on_entrance(False, t)                           # car in the gate
        self.assertIsNone(self.b.tick(t + 0.2))
        self.assertTrue(self.b.desired_open)
        self.assertIsNone(self.b.tick(t + 5.0))                # sensor silent: still open
        self.assertTrue(self.b.desired_open)
        self.assertIn('BARRIER_CLOSE_DEFERRED', [x for x, _ in self.events()])

    def test_closes_after_hold_and_continuous_clear(self):
        self.b.on_entrance(True, self.t)
        self.b.on_claim('C1', self.res.reservation_id, self.t)
        t = self.t + 8.0
        self.b.on_entrance(False, t - 1.0)                     # vehicle drives through
        self.b.on_entrance(True, t - 0.2)                      # passage clears again
        self.assertIsNone(self.b.tick(t))                      # clear for only 0.2 s
        self.b.on_entrance(True, t + 0.5)
        self.b.on_entrance(True, t + 1.0)
        self.assertIs(self.b.tick(t + 1.1), False)             # >= 1 s continuous clear
        self.assertFalse(self.b.desired_open)

    def test_missing_acknowledgment_is_logged_and_ack_clears_it(self):
        self.b.on_entrance(True, self.t)
        self.b.on_claim('C1', self.res.reservation_id, self.t)
        self.b.tick(self.t + 2.5)
        self.assertIn('BARRIER_NO_ACK', [x for x, _ in self.events()])

        b2 = BarrierController(self.e, ack_timeout_sec=2.0)
        b2.on_entrance(True, self.t)
        b2.on_claim('C1', self.res.reservation_id, self.t)
        b2.on_ack(True, self.t + 0.5)
        b2.tick(self.t + 2.5)
        self.assertNotIn('BARRIER_NO_ACK', [x for x, _ in self.events()])


if __name__ == '__main__':
    unittest.main()
