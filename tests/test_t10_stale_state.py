"""T10 (S5 freshness) plus the S5 failure cases from section 12.3: incorrect
edge ids, pose outside the claimed corridor, zero vehicles, frozen timestamps,
sensor recovery.  Bonus coverage next to T09 since it uses the same engine."""
import math
import unittest

from traffic_sim import (BLOCKED, CONGESTED, NORMAL, UNKNOWN, Mover, TrafficConfig,
                         TrafficEngine, VehicleObs, first_time, run_feed)

V = VehicleObs


def warm(engine, vehicles, t0=0.0, n=6):
    for i in range(n):
        engine.ingest(t0 + i * 0.1, t0 + i * 0.1, vehicles, source_age_s=0.0)
    return t0 + (n - 1) * 0.1


def label(engine, t, edge='Cs-J'):
    return {r.edge_id: r for r in engine.evaluate(t)}[edge]


class RejectImpossibleSamples(unittest.TestCase):
    def test_incorrect_edge_id_is_rejected_not_reassigned(self):
        e = TrafficEngine()
        car = V('c1', 'car', 7.0, 2.0, 0.3, 'Z-Q')                   # not a map edge
        t = warm(e, [car] * 1)
        self.assertEqual(e.rejections['INVALID_EDGE_ID'], 6)
        self.assertEqual(label(e, t).count, 0)

    def test_pose_outside_claimed_corridor_is_rejected(self):
        e = TrafficEngine()
        t = warm(e, [V('c1', 'car', 7.0, 3.4, 0.3, 'Cs-J')])         # 1.4 m off the edge
        self.assertEqual(e.rejections['OUTSIDE_CORRIDOR'], 6)
        self.assertEqual(label(e, t).count, 0)

    def test_pose_valid_but_wrong_edge_claim_is_rejected(self):
        e = TrafficEngine()
        t = warm(e, [V('c1', 'car', 7.0, 2.0, 0.3, 'N-Cn')])          # on Cs-J, claims N-Cn
        self.assertEqual(e.rejections['OUTSIDE_CORRIDOR'], 6)
        self.assertEqual(label(e, t, 'Cs-J').count, 0)

    def test_impossible_speed_and_nonfinite_values_rejected(self):
        e = TrafficEngine()
        bad = [V('a', 'car', 7.0, 2.0, -0.1, 'Cs-J'),
               V('b', 'car', 7.0, 2.0, 5.0, 'Cs-J'),
               V('c', 'car', 7.0, 2.0, math.nan, 'Cs-J'),
               V('d', 'car', math.nan, 2.0, 0.3, 'Cs-J'),
               V('e', 'car', 7.0, math.inf, 0.3, 'Cs-J')]
        t = warm(e, bad, n=1)
        self.assertEqual(e.rejections['BAD_SPEED'], 3)
        self.assertEqual(e.rejections['BAD_POSE'], 2)
        self.assertEqual(label(e, warm(e, bad, t0=0.1, n=5)).count, 0)

    def test_duplicate_vehicle_id_counted_once(self):
        e = TrafficEngine()
        car = V('same', 'car', 7.0, 2.0, 0.3, 'Cs-J')
        t = warm(e, [car, car])
        self.assertEqual(label(e, t).count, 1)
        self.assertGreater(e.rejections['DUPLICATE_ID'], 0)

    def test_edge_id_spellings_are_equivalent(self):
        for spelling in ('J-E', 'E-J', 'J–E'):
            e = TrafficEngine()
            t = warm(e, [V('c', 'car', 10.0, 2.0, 0.3, spelling)])
            self.assertEqual(label(e, t, 'E-J').count, 1, spelling)


class ZeroVehiclesAndStale(unittest.TestCase):
    def test_zero_vehicles_with_live_feed_is_valid_normal_at_free_flow(self):
        e = TrafficEngine()
        t = warm(e, [])
        r = label(e, t)
        self.assertEqual((r.label, r.valid, r.count, r.density_ratio), (NORMAL, True, 0, 0.0))
        self.assertEqual(r.mean_speed_mps, 0.40)                     # never 0 travel time

    def test_before_any_data_everything_is_unknown_with_sentinels(self):
        e = TrafficEngine()
        for r in e.evaluate(10.0):
            self.assertEqual((r.label, r.valid, r.mean_speed_mps, r.density_ratio),
                             (UNKNOWN, False, -1.0, -1.0))

    def test_warmup_needs_min_samples(self):
        e = TrafficEngine()
        e.ingest(0.0, 0.0, [])
        e.ingest(0.1, 0.1, [])
        self.assertEqual(label(e, 0.1).label, UNKNOWN)
        e.ingest(0.2, 0.2, [])
        self.assertEqual(label(e, 0.2).label, NORMAL)

    def test_frozen_timestamp_goes_unknown_within_1p2s_and_recovers_cleanly(self):
        movers = [Mover('car1', 'Cs-J', 0.35), Mover('car2', 'Cs-J', 0.35, 0.5)]
        ticks, eng = run_feed(movers, 30.0, seed=5, freeze=(10.0, 16.0))
        t_unknown = first_time(ticks, 'Cs-J', UNKNOWN, after=10.0)
        self.assertLessEqual(t_unknown - 9.9, 1.2)                    # last valid obs at 9.9
        self.assertGreater(eng.rejections['STALE_STAMP'], 0)
        for k in ticks:                                               # never "clear" while frozen
            if 11.1 <= k['t'] < 16.0:
                self.assertEqual(k['labels']['Cs-J'], UNKNOWN)
                r = k['results']['Cs-J']
                self.assertEqual((r.mean_speed_mps, r.density_ratio), (-1.0, -1.0))
        t_back = first_time(ticks, 'Cs-J', NORMAL, after=16.0)
        self.assertLessEqual(t_back - 16.0, 1.0)                      # clean recovery
        self.assertTrue(all(k['labels']['Cs-J'] == NORMAL for k in ticks if k['t'] >= t_back))

    def test_repeated_old_stamp_never_refreshes_the_feed(self):
        e = TrafficEngine()
        t = warm(e, [V('c', 'car', 7.0, 2.0, 0.3, 'Cs-J')])
        for i in range(1, 30):                                        # same stamp, new arrival times
            rep = e.ingest(0.5, t + 0.1 * i, [V('c', 'car', 7.0, 2.0, 0.3, 'Cs-J')])
            self.assertEqual(rep.snapshot_reason, 'STALE_STAMP')
        self.assertEqual(label(e, t + 3.0).label, UNKNOWN)

    def test_old_or_future_source_age_is_not_fresh(self):
        e = TrafficEngine()
        self.assertEqual(e.ingest(1.0, 1.0, [], source_age_s=3.0).snapshot_reason, 'SOURCE_AGE')
        self.assertEqual(e.ingest(2.0, 2.0, [], source_age_s=-2.0).snapshot_reason, 'SOURCE_AGE')
        self.assertTrue(e.ingest(3.0, 3.0, [], source_age_s=0.1).snapshot_ok)

    def test_simulator_reset_clock_jump_back_is_accepted(self):
        e = TrafficEngine()
        warm(e, [], t0=100.0)
        self.assertTrue(e.ingest(0.0, 100.6, [], source_age_s=0.0).snapshot_ok)

    def test_vehicle_marked_invalid_makes_only_its_edge_unknown(self):
        e = TrafficEngine()
        good = V('g', 'car', 10.0, 2.0, 0.3, 'J-E')
        bad_on = V('x', 'car', 7.0, 2.0, 0.3, 'Cs-J', True)
        t = warm(e, [good, bad_on])
        t_end = t
        for i in range(1, 16):                                        # x reports valid=False
            t_end = t + i * 0.1
            e.ingest(t_end, t_end, [good, V('x', 'car', 7.0, 2.0, 0.3, 'Cs-J', False)])
        res = {r.edge_id: r for r in e.evaluate(t_end)}
        self.assertEqual(res['Cs-J'].label, UNKNOWN)                  # cannot call it empty
        self.assertEqual(res['E-J'].label, NORMAL)                    # others unaffected
        e.ingest(t_end + 0.1, t_end + 0.1, [good, bad_on])            # sensor recovers
        self.assertEqual({r.edge_id: r for r in e.evaluate(t_end + 0.1)}['Cs-J'].label, NORMAL)

    def test_obstacle_blocks_only_geometrically_intersected_edges(self):
        e = TrafficEngine()
        obstacle = V('O', 'obstacle', 7.0, 2.0, 0.0, '')
        t = warm(e, [obstacle])
        res = {r.edge_id: r.label for r in e.evaluate(t)}
        self.assertEqual(res['Cs-J'], BLOCKED)
        self.assertEqual([k for k, v in res.items() if v == BLOCKED], ['Cs-J'])


class Config(unittest.TestCase):
    def test_invalid_ranges_rejected_at_startup(self):
        for kw in ({'stale_sec': 2.0}, {'density_threshold': 0.0}, {'slow_fraction': 1.5},
                   {'window_sec': 0}, {'safe_headway_m': -1}, {'min_samples': 0}):
            with self.assertRaises(ValueError):
                TrafficEngine(TrafficConfig(**kw))

    def test_capacities_follow_length_and_headway(self):
        e = TrafficEngine()
        self.assertEqual([e.capacity(x) for x in ('A-Cs', 'Cs-J', 'E-J', 'A-N',
                                                  'Cn-N', 'B-Cn', 'B-U', 'E-U')],
                         [4, 4, 2, 2, 4, 3, 3, 2])


if __name__ == '__main__':
    unittest.main()
