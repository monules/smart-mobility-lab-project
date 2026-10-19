"""T07 - S4 atomicity: twenty concurrent request pairs, zero double allocations.

Run:  python3 -m unittest discover -s tests -p 'test_t07*.py' -v
Writes evidence/t07_trials.csv (one row per trial, per request).
"""
import csv
import os
import threading
import time
import unittest

from helpers import EVIDENCE_DIR, make_engine, paper_scene, percentile


class _NoLock:
    """Stand-in for the engine lock: used ONLY by the negative control."""
    def __enter__(self):
        return self

    def __exit__(self, *exc):
        return False

    def acquire(self, *a, **k):
        return True

    def release(self):
        pass


def run_pair(engine, now, pref_a='P2', pref_b='P2', same_request_id=False):
    """Fire two reserve() calls at the same instant from two threads."""
    barrier = threading.Barrier(2)
    out = [None, None]
    lat = [0.0, 0.0]

    def worker(i, vehicle, pref):
        barrier.wait()
        t0 = time.perf_counter()
        out[i] = engine.reserve('req-same' if same_request_id else 'req-%d' % i,
                                vehicle, pref, 30.0, now)
        lat[i] = time.perf_counter() - t0

    threads = [threading.Thread(target=worker, args=(0, 'C1', pref_a)),
               threading.Thread(target=worker, args=(1, 'C2', pref_b))]
    for t in threads:
        t.start()
    for t in threads:
        t.join()
    return out, lat


def invariants_hold(engine, now):
    """At most one active owner per slot and one slot per vehicle."""
    views, _ = engine.snapshot(now)
    owners = [v.vehicle_id for v in views if v.reservation_id]
    tokens = [v.reservation_id for v in views if v.reservation_id]
    return len(owners) == len(set(owners)) and len(tokens) == len(set(tokens))


class T07Atomicity(unittest.TestCase):
    TRIALS = 20

    def test_twenty_concurrent_pairs_no_double_allocation(self):
        rows, latencies, doubles = [], [], 0
        for trial in range(self.TRIALS):
            e = make_engine()
            now = paper_scene(e)
            (r1, r2), (l1, l2) = run_pair(e, now)
            successes = [r for r in (r1, r2) if r.success]
            slots = [r.slot_id for r in successes]
            if len(slots) != len(set(slots)):
                doubles += 1
            # Both prefer P2: exactly one wins P2; the other gets P3 under the
            # default "nearest FREE" policy (P1 is occupied).
            self.assertEqual(sorted(slots), ['P2', 'P3'], 'trial %d' % trial)
            self.assertEqual(slots.count('P2'), 1)
            self.assertTrue(invariants_hold(e, now), 'trial %d' % trial)
            latencies += [l1, l2]
            for who, r, lat in (('C1', r1, l1), ('C2', r2, l2)):
                rows.append([trial, who, r.success, r.slot_id, r.reason, '%.6f' % lat])
        self.assertEqual(doubles, 0, 'double allocations detected')
        p95 = percentile(latencies, 95)
        # Engine-level latency only; the ROS service p95 (< 250 ms) is measured
        # separately with timestamps on the real node.
        self.assertLess(p95, 0.250)
        os.makedirs(EVIDENCE_DIR, exist_ok=True)
        with open(os.path.join(EVIDENCE_DIR, 't07_trials.csv'), 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['trial', 'vehicle', 'success', 'slot', 'reason', 'latency_s'])
            w.writerows(rows)
        print('\nT07: %d pairs, %d requests, double allocations=%d, '
              'engine p95=%.3f ms (n=%d)' % (self.TRIALS, len(rows), doubles,
                                             p95 * 1e3, len(latencies)))

    def test_strict_preference_policy_rejects_the_loser(self):
        for _ in range(self.TRIALS):
            e = make_engine(strict_preference=True)
            now = paper_scene(e)
            (r1, r2), _ = run_pair(e, now)
            self.assertEqual([r1.success, r2.success].count(True), 1)
            loser = r1 if not r1.success else r2
            self.assertEqual(loser.reason, 'PREFERRED_UNAVAILABLE')

    def test_same_request_id_racing_is_idempotent(self):
        """Two threads using the SAME request id (a retry racing the original)
        must get the same token and consume only one slot."""
        for _ in range(self.TRIALS):
            e = make_engine()
            now = paper_scene(e)
            barrier = threading.Barrier(2)
            res = []

            def w():
                barrier.wait()
                res.append(e.reserve('dup', 'C1', 'P2', 30.0, now))
            ts = [threading.Thread(target=w) for _ in range(2)]
            [t.start() for t in ts]
            [t.join() for t in ts]
            self.assertEqual(res[0].reservation_id, res[1].reservation_id)
            self.assertEqual(res[0].slot_id, res[1].slot_id)
            views, free = e.snapshot(now)
            self.assertEqual(sum(1 for v in views if v.reservation_id), 1)
            self.assertEqual(free, 1)             # P3 still free

    def test_many_threads_never_exceed_free_capacity(self):
        e = make_engine()
        now = paper_scene(e)                      # only P2, P3 allocatable
        barrier = threading.Barrier(16)
        res = []

        def w(i):
            barrier.wait()
            res.append(e.reserve('r%d' % i, 'car%d' % i, '', 30.0, now))
        ts = [threading.Thread(target=w, args=(i,)) for i in range(16)]
        [t.start() for t in ts]
        [t.join() for t in ts]
        ok = [r for r in res if r.success]
        self.assertEqual(len(ok), 2)
        self.assertEqual(len({r.slot_id for r in ok}), 2)
        self.assertEqual({r.reason for r in res if not r.success}, {'FULL'})

    def test_negative_control_without_lock_the_test_detects_double_allocation(self):
        """Proof the test can fail: remove the lock, widen the check->commit
        window, and the same pair-trial MUST report double allocations."""
        doubles = 0
        for _ in range(self.TRIALS):
            e = make_engine()
            now = paper_scene(e)
            e._lock = _NoLock()
            e._race_hook = lambda: time.sleep(0.002)
            (r1, r2), _ = run_pair(e, now)
            if r1.success and r2.success and r1.slot_id == r2.slot_id:
                doubles += 1
        self.assertGreater(doubles, 0, 'negative control did not trigger a race: '
                                       'the T07 test would not have caught a bug')
        print('\nT07 negative control (lock removed): %d/%d trials double-allocated'
              % (doubles, self.TRIALS))


if __name__ == '__main__':
    unittest.main()
