"""s5_check (M5 for M3): checks S5 congestion detection on the live ROS graph.

Listens to /traffic/edge_labels (traffic_monitor's per-edge label JSON),
/traffic/edges (EdgeStateArray) and /hub/events (scenario timing) while
scenario S5 runs, then checks assignment 12.3 on edge Cs-J:

  T09  no false congestion while one car drives at 0.35 m/s (before t=20)
  T09  CONGESTED once three slow cars join (density 4/4 sustained)
  T09  NORMAL again after the slow cars leave
  T10  every edge UNKNOWN within 1.2 s of the vehicle-source freeze, and
       /traffic/edges marks them invalid with speed/density -1.0
  T10  labels valid again after the freeze ends
  S5   BLOCKED within one publication of obstacle O appearing on Cs-J
  S5   NORMAL again after the obstacle is removed

The scenario times come from the runner's SCENARIO_EVENT messages, so the
checks measure from the moment the simulator applied each event.

Writes <output_dir>/s5_check.json, s5_timeseries.csv and s5_labels.png;
exit code 0 only if every check passed.
"""
import json
import os
import sys
import time

import rclpy
from rclpy.node import Node
from smart_hub.ros_common import declare, R10, R100
from smart_hub_interfaces.msg import EdgeStateArray, HubEvent
from std_msgs.msg import String

EDGE = 'Cs-J'
UNKNOWN_LIMIT = 1.2       # s, assignment 12.3
# "Within one publication": the label timer runs at 2 Hz (0.5 s); the
# obstacle is first seen in the next 10 Hz vehicle frame (0.1 s); plus 0.1 s
# transport slack.
BLOCKED_LIMIT = 0.7
CONGESTED_LIMIT = 3.0     # 3 slow publications at 2 Hz plus presence warm-up
CLEAR_LIMIT = 3.0         # clear_ticks (2) publications plus sample hold (1 s)
RECOVER_LIMIT = 3.0       # min_samples (3) fresh frames plus one publication
UNBLOCK_LIMIT = 3.5       # obstacle_clear_sec (2 s) plus one publication


def scenario_event(reason):
    """Return the event dict of a SCENARIO_EVENT reason, or None."""
    try:
        event = json.loads(reason)
    except ValueError:
        return None
    return event if isinstance(event, dict) else None


class S5Check(Node):
    def __init__(self, **kwargs):
        super().__init__('s5_check', **kwargs)
        p = declare(self, {'output_dir': 'output', 'timeout_sec': 110.0})
        self.output_dir = p['output_dir']
        self.deadline = time.monotonic() + float(p['timeout_sec'])
        self.labels = []          # (t, {edge: label}, cs_j count)
        self.edges = []           # (t, all invalid with -1.0 values)
        self.marks = {}           # name -> monotonic time the simulator applied it
        self.complete_t = None
        self.create_subscription(String, '/traffic/edge_labels', self.on_labels, R10)
        self.create_subscription(EdgeStateArray, '/traffic/edges', self.on_edges, R10)
        self.create_subscription(HubEvent, '/hub/events', self.on_event, R100)

    def on_labels(self, msg):
        try:
            edges = json.loads(msg.data)['edges']
        except (ValueError, KeyError, TypeError):
            return
        self.labels.append((time.monotonic(), {e: v['label'] for e, v in edges.items()},
                            edges.get(EDGE, {}).get('count', 0)))

    def on_edges(self, msg):
        invalid = bool(msg.edges) and all(
            not e.valid and e.mean_speed_mps == -1.0 and e.density_ratio == -1.0
            for e in msg.edges)
        self.edges.append((time.monotonic(), invalid))

    def on_event(self, msg):
        now = time.monotonic()
        if msg.event == 'SCENARIO_COMPLETE':
            self.complete_t = now
            return
        if msg.event != 'SCENARIO_EVENT':
            return
        event = scenario_event(msg.reason) or {}
        kind = event.get('type')
        if kind == 'note' and 'start' not in self.marks:
            self.marks['start'] = now
            self.marks['t0'] = now - float(event.get('t', 0.0))   # scenario t = 0
        elif kind == 'vehicle':
            name = 'slow_cars' if event.get('active', True) else 'cars_gone'
            self.marks.setdefault(name, now)
        elif kind == 'freeze' and event.get('topic') == 'vehicles':
            self.marks['freeze'] = now
            self.marks['thaw'] = now + float(event['duration'])
        elif kind == 'obstacle':
            self.marks['obstacle_on' if event.get('active', True) else 'obstacle_off'] = now

    def done(self):
        return (self.complete_t is not None and time.monotonic() > self.complete_t + 0.5) \
            or time.monotonic() > self.deadline

    # --------------------------------------------------------------- checks
    def first(self, t0, cond, end=float('inf')):
        """Delay from t0 to the first label sample in [t0, end) meeting cond."""
        for t, labels, count in self.labels:
            if t0 <= t < end and cond(labels, count):
                return t - t0
        return None

    def evaluate(self):
        results = []

        def check(name, passed, detail):
            results.append({'check': name, 'passed': bool(passed), 'detail': detail})
            print('[%s] %s: %s' % ('PASS' if passed else 'FAIL', name, detail), flush=True)

        def r(x):
            return None if x is None else round(x, 3)

        m = self.marks
        need = ('start', 'slow_cars', 'cars_gone', 'freeze', 'obstacle_on', 'obstacle_off')
        missing = [k for k in need if k not in m]
        if missing:
            check('S5 scenario events', False, 'missing scenario events: %s' % missing)
            return results

        before = [lab[EDGE] for t, lab, _ in self.labels
                  if m['start'] <= t < m['slow_cars'] and EDGE in lab]
        bad = [x for x in before if x in ('CONGESTED', 'BLOCKED')]
        check('T09 no false congestion', len(before) > 20 and not bad
              and before.count('NORMAL') > 20,
              '%d labels with one car at 0.35 m/s: %d NORMAL, %d CONGESTED/BLOCKED'
              % (len(before), before.count('NORMAL'), len(bad)))

        d = self.first(m['slow_cars'], lambda lab, n: lab.get(EDGE) == 'CONGESTED',
                       m['cars_gone'])
        counts = [n for t, lab, n in self.labels
                  if m['slow_cars'] + d + 1.0 <= t < m['cars_gone']] if d is not None else []
        check('T09 congestion detected', d is not None and d <= CONGESTED_LIMIT
              and counts and min(counts) >= 3,
              'CONGESTED %s s after 3 slow cars joined (limit %.1f); cars counted %s'
              % (r(d), CONGESTED_LIMIT, sorted(set(counts))))
        held = [lab.get(EDGE) for t, lab, _ in self.labels
                if d is not None and m['slow_cars'] + d <= t < m['cars_gone']]
        check('T09 congestion held', held and all(x == 'CONGESTED' for x in held),
              '%d of %d labels CONGESTED while the slow cars stayed'
              % (held.count('CONGESTED'), len(held)))

        d = self.first(m['cars_gone'], lambda lab, n: lab.get(EDGE) == 'NORMAL', m['freeze'])
        check('T09 congestion clears', d is not None and d <= CLEAR_LIMIT,
              'NORMAL %s s after the slow cars left (limit %.1f)' % (r(d), CLEAR_LIMIT))

        d = self.first(m['freeze'], lambda lab, n: lab and all(
            x == 'UNKNOWN' for x in lab.values()), m['thaw'])
        check('T10 stale -> UNKNOWN', d is not None and d <= UNKNOWN_LIMIT,
              'all edges UNKNOWN %s s after the vehicle feed froze (limit %.1f)'
              % (r(d), UNKNOWN_LIMIT))
        frozen = [inv for t, inv in self.edges if m['freeze'] + UNKNOWN_LIMIT <= t < m['thaw']]
        check('T10 invalid values', frozen and all(frozen),
              '%d of %d /traffic/edges messages during the freeze: valid=false, '
              'speed and density -1.0' % (sum(frozen), len(frozen)))
        d = self.first(m['thaw'], lambda lab, n: lab.get(EDGE) in ('NORMAL', 'CONGESTED'),
                       m['obstacle_on'])
        check('T10 recovery', d is not None and d <= RECOVER_LIMIT,
              '%s valid again %s s after the feed resumed (limit %.1f)'
              % (EDGE, r(d), RECOVER_LIMIT))

        d = self.first(m['obstacle_on'], lambda lab, n: lab.get(EDGE) == 'BLOCKED',
                       m['obstacle_off'])
        check('S5 obstacle -> BLOCKED', d is not None and d <= BLOCKED_LIMIT,
              'BLOCKED %s s after obstacle O appeared (limit %.1f = one 2 Hz '
              'publication + one vehicle frame + slack)' % (r(d), BLOCKED_LIMIT))
        d = self.first(m['obstacle_off'], lambda lab, n: lab.get(EDGE) == 'NORMAL')
        check('S5 obstacle cleared', d is not None and d <= UNBLOCK_LIMIT,
              'NORMAL %s s after obstacle O was removed (limit %.1f)' % (r(d), UNBLOCK_LIMIT))
        return results


LEVEL = {'UNKNOWN': 0, 'NORMAL': 1, 'CONGESTED': 2, 'BLOCKED': 3}


def write_timeseries(node, out):
    """Save the Cs-J label and car count over time, as CSV and as a plot."""
    if not node.labels:
        return
    t0 = node.marks.get('t0', node.labels[0][0])
    rows = [(t - t0, lab.get(EDGE, 'UNKNOWN'), n) for t, lab, n in node.labels]
    with open(os.path.join(out, 's5_timeseries.csv'), 'w') as fh:
        fh.write('t,edge,label,count\n')
        for t, label, n in rows:
            fh.write('%.3f,%s,%s,%d\n' % (t, EDGE, label, n))
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8, 4.5), sharex=True)
    ax1.step([t for t, _, _ in rows], [LEVEL.get(lab, 0) for _, lab, _ in rows],
             where='post', linewidth=2)
    ax1.set_yticks(list(LEVEL.values()), list(LEVEL))
    ax1.set_ylabel('%s label' % EDGE)
    for name, color in (('slow_cars', 'orange'), ('cars_gone', 'green'),
                        ('freeze', 'red'), ('thaw', 'red'),
                        ('obstacle_on', 'black'), ('obstacle_off', 'black')):
        if name in node.marks:
            for ax in (ax1, ax2):
                ax.axvline(node.marks[name] - t0, color=color, linestyle='--', linewidth=1)
    ax2.step([t for t, _, _ in rows], [n for _, _, n in rows], where='post')
    ax2.axhline(3, color='grey', linestyle=':', label='0.70 x capacity 4')
    ax2.set_ylabel('cars counted')
    ax2.set_xlabel('scenario time (s)')
    ax2.legend(loc='upper right', fontsize=8)
    fig.tight_layout()
    fig.savefig(os.path.join(out, 's5_labels.png'), dpi=120)
    plt.close(fig)


def main(args=None):
    rclpy.init(args=args)
    node = S5Check()
    ok = False
    try:
        while rclpy.ok() and not node.done():
            rclpy.spin_once(node, timeout_sec=0.05)
        results = node.evaluate()
        ok = bool(results) and all(r['passed'] for r in results)
        os.makedirs(node.output_dir, exist_ok=True)
        path = os.path.join(node.output_dir, 's5_check.json')
        with open(path, 'w') as fh:
            json.dump({'passed': ok, 'checks': results,
                       'commit': os.environ.get('GIT_COMMIT', 'unknown')}, fh, indent=2)
        write_timeseries(node, node.output_dir)
        print('S5 %s; evidence: %s' % ('PASSED' if ok else 'FAILED', path), flush=True)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
