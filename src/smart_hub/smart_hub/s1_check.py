"""s1_check (M1-A): checks S1 on the live ROS graph against simulator truth.

Listens to /hub/queue (the detector's output), /sim/truth (true pedestrian
positions, evaluation only) and /hub/events (scenario timing, track
confirmations) while scenario S1 runs, then checks assignment 8.3:

  T01  stable counts follow the truth (0, 3, 5, 4) once settled (1.0 s)
  T01  count MAE <= 1 over settled samples (exact for the blob baseline)
  T02  no double count: confirmed tracks == distinct people
  T02  invalid within 1.2 s of the camera freeze
  S1   crossing occupied while a person is in C and for the 2 s clear hold

Writes <output_dir>/s1_check.json; exit code 0 only if every check passed.
"""
import json
import os
import sys
import time

import rclpy
from rclpy.node import Node
from smart_hub.perception.zones import CROSSING_C, point_in_polygon, QUEUE_Q
from smart_hub.ros_common import declare, R10, R100
from smart_hub_interfaces.msg import HubEvent, QueueState, VehicleStateArray

SETTLE = 1.0          # s after a truth change before the count must match
FREEZE_LIMIT = 1.2    # s, assignment 8.3


class S1Check(Node):
    def __init__(self, **kwargs):
        super().__init__('s1_check', **kwargs)
        p = declare(self, {'output_dir': 'output', 'timeout_sec': 70.0,
                           'exact_counts': True})
        self.output_dir, self.exact = p['output_dir'], bool(p['exact_counts'])
        self.deadline = time.monotonic() + float(p['timeout_sec'])
        self.truth, self.reported = [], []   # (t, queue, crossing) / (t, count, valid, crossing)
        self.people, self.confirmed = set(), 0
        self.freeze_t = self.complete_t = None
        self.create_subscription(QueueState, '/hub/queue', self.on_queue, R10)
        self.create_subscription(VehicleStateArray, '/sim/truth', self.on_truth, R10)
        self.create_subscription(HubEvent, '/hub/events', self.on_event, R100)

    def on_queue(self, msg):
        self.reported.append((time.monotonic(), msg.count, msg.valid, msg.crossing_occupied))

    def on_truth(self, msg):
        peds = [(v.vehicle_id, v.pose.x, v.pose.y) for v in msg.vehicles
                if v.kind == 'pedestrian']
        self.people.update(pid for pid, _, _ in peds)
        self.truth.append((time.monotonic(),
                           sum(point_in_polygon(x, y, QUEUE_Q) for _, x, y in peds),
                           any(point_in_polygon(x, y, CROSSING_C) for _, x, y in peds)))

    def on_event(self, msg):
        now = time.monotonic()
        if msg.event == 'TRACK_CONFIRMED':
            self.confirmed += 1
        elif msg.event == 'SCENARIO_EVENT' and '"camera"' in msg.reason \
                and '"freeze"' in msg.reason:
            self.freeze_t = now
        elif msg.event == 'SCENARIO_COMPLETE':
            self.complete_t = now

    def done(self):
        return (self.complete_t is not None and time.monotonic() > self.complete_t + 0.5) \
            or time.monotonic() > self.deadline

    # --------------------------------------------------------------- checks
    def segments(self, index):
        """Return runs of constant truth value: [(start, end, value)]."""
        out = []
        for t, *vals in self.truth:
            v = vals[index]
            if out and out[-1][2] == v:
                out[-1][1] = t
            else:
                out.append([t, t, v])
        return out

    def evaluate(self):
        results = []

        def check(name, passed, detail):
            results.append({'check': name, 'passed': bool(passed), 'detail': detail})
            print('[%s] %s: %s' % ('PASS' if passed else 'FAIL', name, detail), flush=True)

        end = self.freeze_t or float('inf')
        q_segs = [s for s in self.segments(0) if s[0] < end]
        long_segs = [s for s in q_segs if min(s[1], end) - s[0] >= 2.0]
        settled, errors, mismatch = 0, [], []
        for start, stop, value in long_segs:
            for t, count, valid, _ in self.reported:
                if start + SETTLE <= t <= min(stop, end) and valid:
                    settled += 1
                    errors.append(abs(count - value))
                    if count != value:
                        mismatch.append((round(t - self.truth[0][0], 1), count, value))
        sequence = [s[2] for s in long_segs]
        check('T01 truth sequence', sequence == [0, 3, 5, 4], 'stable truth %s' % sequence)
        mae = sum(errors) / len(errors) if errors else float('nan')
        exact_ok = not mismatch if self.exact else True
        check('T01 stable counts', settled > 20 and exact_ok and mae <= 1.0,
              '%d settled samples, MAE %.3f, mismatches %s' % (settled, mae, mismatch[:5]))

        # Double counting = more tracks than people. Fewer tracks means missed
        # people (recall), already measured by the MAE check; the exact
        # baseline must find everyone.
        n, people = self.confirmed, len(self.people)
        check('T02 no double count', n == people if self.exact else n <= people,
              '%d confirmed tracks for %d distinct people (recall %.2f)'
              % (n, people, n / people if people else float('nan')))

        invalid_before = [t for t, _, valid, _ in self.reported
                          if not valid and self.truth and self.truth[0][0] + 2.0 < t < end]
        check('S1 valid before freeze', not invalid_before,
              '%d invalid samples before the freeze' % len(invalid_before))

        if self.freeze_t is None:
            check('T02 freeze -> invalid', False, 'no camera freeze event seen')
        else:
            first = next((t for t, _, valid, _ in self.reported
                          if t >= self.freeze_t and not valid), None)
            delay = None if first is None else first - self.freeze_t
            check('T02 freeze -> invalid', delay is not None and delay <= FREEZE_LIMIT,
                  'invalid %s s after the freeze (limit %.1f)'
                  % (delay and round(delay, 3), FREEZE_LIMIT))

        occupied = [s for s in self.segments(1) if s[2] and s[0] < end]
        if not occupied:
            check('S1 crossing hold', False, 'no crossing pedestrian in truth')
        for start, stop, _ in occupied[:1]:
            rep = [(t, c) for t, _, valid, c in self.reported if valid]
            on = next((t for t, c in rep if t >= start and c), None)
            off = next((t for t, c in rep if t >= stop and not c), None)
            ok = (on is not None and on - start <= 1.0 and off is not None
                  and 2.0 <= off - stop <= 3.5)
            check('S1 crossing hold', ok,
                  'occupied %s s after entry; clear %s s after leaving (needs 2.0-3.5)'
                  % (on and round(on - start, 2), off and round(off - stop, 2)))
        return results


def main(args=None):
    rclpy.init(args=args)
    node = S1Check()
    ok = False
    try:
        while rclpy.ok() and not node.done():
            rclpy.spin_once(node, timeout_sec=0.05)
        results = node.evaluate()
        ok = bool(results) and all(r['passed'] for r in results)
        os.makedirs(node.output_dir, exist_ok=True)
        path = os.path.join(node.output_dir, 's1_check.json')
        with open(path, 'w') as fh:
            json.dump({'passed': ok, 'checks': results,
                       'commit': os.environ.get('GIT_COMMIT', 'unknown')}, fh, indent=2)
        print('S1 %s; evidence: %s' % ('PASSED' if ok else 'FAILED', path), flush=True)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
