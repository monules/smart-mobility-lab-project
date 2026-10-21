"""T09 - S5 congestion: correct NORMAL/CONGESTED/BLOCKED transitions with
precision / recall against scenario labels, detection delay, false alarms,
valid-data fraction, and ten free-flow runs with zero false congestion.

Run:  python3 -m unittest discover -s tests -p 'test_t09*.py' -v
Writes evidence/t09_*.csv|json (per-tick labels, metrics summary).
"""
import csv
import json
import os
import unittest

from traffic_sim import (BLOCKED, CONGESTED, NORMAL, UNKNOWN, EVIDENCE_DIR,
                         Mover, Obstacle, TrafficConfig, first_time, run_feed)

EDGE = 'Cs-J'
FREEZE = (45.0, 52.0)
ALLOWED_DELAY = 2.0            # s after a truth transition that is not scored


def truth(t):
    """Ground-truth label of Cs-J in the section 12.1 event sequence."""
    if t < 20.0:
        return NORMAL
    if t < FREEZE[0]:
        return CONGESTED
    if t < FREEZE[1]:
        return UNKNOWN                      # source frozen: truth is "we cannot know"
    if t < 60.0:
        return CONGESTED
    return BLOCKED


TRANSITIONS = [0.0, 20.0, FREEZE[0], FREEZE[1], 60.0]


def s5_movers():
    return [
        Mover('car1', 'Cs-J', 0.35),                                  # t=0: one car, 0.35 m/s
        Mover('slow1', 'Cs-J', 0.08, 0.15, active_from=20.0),         # t=20: three slow cars
        Mover('slow2', 'Cs-J', 0.08, 0.35, active_from=20.0),
        Mover('slow3', 'Cs-J', 0.08, 0.55, active_from=20.0),
        Mover('bg', 'J-E', 0.35, claimed_edge='J-E'),                 # unaffected edge, other spelling
        Obstacle(7.0, 2.0, active_from=60.0),                         # t=60: O across Cs-J
    ]


def confusion(ticks):
    labels = [NORMAL, CONGESTED, BLOCKED, UNKNOWN]
    m = {a: {b: 0 for b in labels} for a in labels}
    scored = 0
    for k in ticks:
        t = k['t']
        if any(0.0 <= t - tr < ALLOWED_DELAY for tr in TRANSITIONS[1:]):
            continue                       # inside the allowed detection delay
        if t < 1.0:
            continue                       # warm-up
        m[truth(t)][k['labels'][EDGE]] += 1
        scored += 1
    return m, scored


def prf(m, cls):
    tp = m[cls][cls]
    fp = sum(m[o][cls] for o in m if o != cls)
    fn = sum(m[cls][o] for o in m[cls] if o != cls)
    p = tp / (tp + fp) if tp + fp else 1.0
    r = tp / (tp + fn) if tp + fn else 1.0
    return p, r, tp, fp, fn


class T09CongestionLabels(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.ticks, cls.engine = run_feed(s5_movers(), 80.0, seed=7, freeze=FREEZE)

    def test_event_sequence_labels_and_metrics(self):
        ticks = self.ticks
        m, scored = confusion(ticks)
        p, r, tp, fp, fn = prf(m, CONGESTED)

        d_cong = first_time(ticks, EDGE, CONGESTED, after=20.0) - 20.0
        d_unknown = first_time(ticks, EDGE, UNKNOWN, after=FREEZE[0]) - (FREEZE[0] - 0.1)
        d_blocked = first_time(ticks, EDGE, BLOCKED, after=60.0) - 60.0
        d_recover = first_time(ticks, EDGE, CONGESTED, after=FREEZE[1]) - FREEZE[1]
        false_alarms = sum(1 for k in ticks if 1.0 <= k['t'] < 20.0
                           and k['labels'][EDGE] == CONGESTED)
        stale_as_clear = sum(1 for k in ticks
                             if FREEZE[0] + 1.3 <= k['t'] < FREEZE[1]
                             and k['labels'][EDGE] == NORMAL)
        valid_fraction = sum(1 for k in ticks if k['results'][EDGE].valid) / len(ticks)

        summary = {
            'scenario': 'S5 (seed 7, 10 Hz feed, 2 Hz publication, 3% dropout, noisy)',
            'edge': EDGE, 'ticks': len(ticks), 'scored_ticks': scored,
            'congested_precision': round(p, 4), 'congested_recall': round(r, 4),
            'tp': tp, 'fp': fp, 'fn': fn,
            'detection_delay_congested_s': round(d_cong, 2),
            'detection_delay_blocked_s': round(d_blocked, 2),
            'unknown_after_last_valid_obs_s': round(d_unknown, 2),
            'recovery_to_congested_after_unfreeze_s': round(d_recover, 2),
            'false_alarms_free_flow_phase': false_alarms,
            'stale_reported_as_normal': stale_as_clear,
            'valid_data_fraction': round(valid_fraction, 4),
            'confusion_truth_rows_pred_cols': m,
            'rejections': dict(self.engine.rejections),
        }
        os.makedirs(EVIDENCE_DIR, exist_ok=True)
        with open(os.path.join(EVIDENCE_DIR, 't09_metrics.json'), 'w') as f:
            json.dump(summary, f, indent=2)
        with open(os.path.join(EVIDENCE_DIR, 't09_ticks.csv'), 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['t', 'truth', 'label', 'valid', 'count', 'capacity',
                        'density', 'mean_speed', 'blocked'])
            for k in ticks:
                r_ = k['results'][EDGE]
                w.writerow([k['t'], truth(k['t']), r_.label, r_.valid, r_.count,
                            r_.capacity, round(r_.density_ratio, 3),
                            round(r_.mean_speed_mps, 3), r_.blocked])
        print('\nT09 summary:', json.dumps({k: v for k, v in summary.items()
                                            if k != 'confusion_truth_rows_pred_cols'}))

        self.assertGreaterEqual(p, 0.95, 'CONGESTED precision')
        self.assertGreaterEqual(r, 0.95, 'CONGESTED recall')
        self.assertLessEqual(d_cong, 2.0, 'CONGESTED within 2 s of the sustained trigger')
        self.assertLessEqual(d_blocked, 0.5, 'BLOCKED within one publication')
        self.assertLessEqual(d_unknown, 1.2, 'UNKNOWN within 1.2 s of losing the feed')
        self.assertEqual(false_alarms, 0)
        self.assertEqual(stale_as_clear, 0, 'stale data must never read as an empty road')
        self.assertEqual(m[UNKNOWN][NORMAL] + m[UNKNOWN][CONGESTED], 0)

    def test_other_edges_stay_normal_and_speed_is_plausible(self):
        for k in self.ticks:
            if 1.0 <= k['t'] < FREEZE[0]:
                self.assertEqual(k['labels']['E-J'], NORMAL)       # bg car, "J-E" spelling
                self.assertEqual(k['labels']['A-Cs'], NORMAL)      # empty road, live feed
        r = [k for k in self.ticks if 30.0 <= k['t'] < 31.0][0]['results'][EDGE]
        self.assertEqual(r.count, 4)
        self.assertEqual(r.density_ratio, 1.0)
        self.assertLess(r.mean_speed_mps, 0.25)                    # slow mix of 4 cars

    def test_blocked_edge_keeps_numbers_valid_and_flags_blocked(self):
        r = [k for k in self.ticks if k['t'] >= 65.0][0]['results'][EDGE]
        self.assertEqual((r.label, r.blocked, r.valid), (BLOCKED, True, True))
        self.assertFalse(self.ticks[-1]['results']['E-J'].blocked)  # O only blocks Cs-J

    def test_obstacle_clears_after_it_disappears(self):
        movers = [Obstacle(7.0, 2.0, 5.0), Mover('car1', 'Cs-J', 0.35)]
        movers[0].active = lambda t: 5.0 <= t < 10.0
        ticks, _ = run_feed(movers, 20.0, seed=3)
        self.assertEqual(first_time(ticks, EDGE, BLOCKED, after=5.0) - 5.0, 0.0)
        t_clear = first_time(ticks, EDGE, NORMAL, after=10.0)
        self.assertLessEqual(t_clear - 10.0, 2.6)                  # 2 s hold + one publication


class T09FreeFlow(unittest.TestCase):
    def test_ten_free_flow_runs_have_no_false_congestion(self):
        rows = []
        for seed in range(10):
            movers = [Mover('a%d' % seed, 'A-Cs', 0.30 + 0.01 * seed),
                      Mover('b%d' % seed, 'Cs-J', 0.35),
                      Mover('c%d' % seed, 'J-E', 0.40, claimed_edge='J-E'),
                      Mover('a2%d' % seed, 'A-Cs', 0.33, 0.5),   # 2 of K=4 -> rho 0.5
                      Mover('d%d' % seed, 'A-N', 0.35),
                      Mover('e%d' % seed, 'N-Cn', 0.38),
                      Mover('f%d' % seed, 'Cn-B', 0.33),
                      Mover('g%d' % seed, 'B-U', 0.36),
                      Mover('h%d' % seed, 'U-E', 0.35)]
            ticks, _ = run_feed(movers, 60.0, seed=100 + seed)
            scored = [k for k in ticks if k['t'] >= 1.0]
            bad = [(k['t'], e, l) for k in scored for e, l in k['labels'].items()
                   if l != NORMAL]
            rows.append([seed, len(scored), len(bad)])
            self.assertEqual(bad, [], 'seed %d produced non-NORMAL labels' % seed)
        os.makedirs(EVIDENCE_DIR, exist_ok=True)
        with open(os.path.join(EVIDENCE_DIR, 't09_free_flow.csv'), 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['seed', 'scored_ticks', 'non_normal_labels'])
            w.writerows(rows)
        print('\nT09 free flow: 10 runs x 60 s, 0 false CONGESTED/BLOCKED/UNKNOWN labels')


class T09Rules(unittest.TestCase):
    def test_density_rule_three_of_four_is_congested_two_is_not(self):
        three = [Mover('v%d' % i, 'Cs-J', 0.35, 0.1 + 0.25 * i) for i in range(3)]
        ticks, _ = run_feed(three, 6.0, seed=1, noise=False, dropout=0.0)
        r = [k for k in ticks if k['t'] >= 1.0][0]['results'][EDGE]
        self.assertEqual((r.count, r.capacity, r.density_ratio), (3, 4, 0.75))
        self.assertEqual(r.label, CONGESTED)
        two = three[:2]
        ticks, _ = run_feed(two, 6.0, seed=1, noise=False, dropout=0.0)
        self.assertTrue(all(k['labels'][EDGE] == NORMAL for k in ticks if k['t'] >= 1.0))

    def test_density_threshold_is_inclusive_at_0_70(self):
        cfg = TrafficConfig(capacity={'Cs-J': 10})
        seven = [Mover('v%d' % i, 'Cs-J', 0.35, 0.05 + 0.1 * i) for i in range(7)]
        ticks, _ = run_feed(seven, 4.0, seed=1, noise=False, dropout=0.0, config=cfg)
        r = [k for k in ticks if k['t'] >= 1.0][0]['results'][EDGE]
        self.assertAlmostEqual(r.density_ratio, 0.7)
        self.assertEqual(r.label, CONGESTED)

    def test_speed_rule_needs_three_consecutive_publications(self):
        slow = [Mover('s1', 'A-Cs', 0.05, 0.2), Mover('s2', 'A-Cs', 0.05, 0.6)]
        ticks, _ = run_feed(slow, 4.0, seed=1, noise=False, dropout=0.0)
        by_t = {k['t']: k['results']['A-Cs'] for k in ticks}
        self.assertEqual(by_t[1.0].density_ratio, 0.5)             # density alone is fine
        self.assertEqual(by_t[1.0].label, NORMAL)                  # 2nd slow publication
        self.assertEqual(by_t[1.5].label, CONGESTED)               # 3rd slow publication
        self.assertEqual(by_t[0.5].label, NORMAL)

    def test_congestion_clears_with_hysteresis_after_traffic_leaves(self):
        movers = [Mover('v%d' % i, 'Cs-J', 0.35, 0.1 + 0.25 * i, active_until=5.0)
                  for i in range(4)]
        ticks, _ = run_feed(movers, 12.0, seed=1, noise=False, dropout=0.0)
        self.assertEqual(ticks[[k['t'] for k in ticks].index(4.5)]['labels'][EDGE], CONGESTED)
        t_clear = first_time(ticks, EDGE, NORMAL, after=5.0)
        self.assertIsNotNone(t_clear)
        self.assertLessEqual(t_clear, 8.0)


if __name__ == '__main__':
    unittest.main()
