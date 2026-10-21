"""s7_check (M4, written by M5): runs the S7 escort tests on the live graph.

It acts as the EscortPassenger client and as the passenger's "ready" button
(/robot/confirm_ready), resets the scenario before every trial and injects
obstacle O / an odometry freeze through /scenario/inject (test only). It
checks what the real robot_manager, route_planner, hub_coordinator,
safety_supervisor, actuator_bridge and simulator do (assignment 14.3).

Trials:
  1-10  nominal escort R -> D           T13: 10/10, arrival error <= 0.20 m,
                                        feedback >= 2 Hz, no crossing conflict
  +1    no readiness                    FAILED READY_TIMEOUT after 20 s, no motion;
                                        a ready press while idle is refused
  +2    obstacle O before the crossing  reroute via Cn-B-U-E-J-D, arrives, no contact;
                                        travel time compared with the nominal trials
  +3    cancel while moving             request->acceptance and acceptance->zero
                                        /robot/cmd_vel, stop time and distance
  +4    second goal while busy, reused  both rejected (T14)
        request id
  +5    unreachable destination (P1)    FAILED NO_ROUTE, no motion
  +6    odometry freeze while moving    zero candidate, bounded SAFE_STOP ODOM_STALE

Writes <output_dir>/s7_check.json, s7_trials.csv, s7_trace.csv, s7_events.csv
and s7_paths.png. Exit code 0 only if every check passed.
"""
import csv
import json
import math
import os
import sys
import threading
import time

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist, TwistStamped
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from smart_hub import geometry as geo
from smart_hub.ros_common import declare, R10, R100
from smart_hub_interfaces.action import EscortPassenger, NavigateTo
from smart_hub_interfaces.msg import HubEvent, VehicleStateArray
from smart_hub_interfaces.srv import ResetScenario
from std_msgs.msg import String
from std_srvs.srv import Trigger

D = geo.NODES['D']
GOAL_TOL = 0.20
STOPPED = 0.02
STATUS = {GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED', GoalStatus.STATUS_CANCELED: 'CANCELED',
          GoalStatus.STATUS_ABORTED: 'ABORTED'}


def in_c(x, y, r=0.20):
    return geo.circle_overlaps_rect(x, y, r, geo.CROSSING)


class S7Check(Node):
    def __init__(self, **kwargs):
        super().__init__('s7_check', **kwargs)
        p = declare(self, {'output_dir': 'output', 'nominal_trials': 10})
        self.output_dir, self.nominal = p['output_dir'], int(p['nominal_trials'])
        self.lock = threading.Lock()
        self.trial = 0
        self.truth = []       # (t, trial, x, y, speed, others_in_c, obstacles)
        self.final = []       # (t, trial, vx, wz) /robot/cmd_vel
        self.cands = []       # (t, trial, vx, wz) /robot/cmd_vel_candidate
        self.events = []      # (t, trial, entity, event, reason)
        self.feedback = []    # (t, trial, phase, progress, remaining)
        self.rows, self.results = [], []
        self.inject_pub = self.create_publisher(String, '/scenario/inject', R10)
        self.create_subscription(VehicleStateArray, '/sim/truth', self.on_truth, R10)
        self.create_subscription(Twist, '/robot/cmd_vel', self.on_final, R10)
        self.create_subscription(TwistStamped, '/robot/cmd_vel_candidate', self.on_cand, R10)
        self.create_subscription(HubEvent, '/hub/events', self.on_event, R100)
        self.escort = ActionClient(self, EscortPassenger, '/robot/escort')
        self.nav = ActionClient(self, NavigateTo, '/robot/navigate_to')
        self.ready_cli = self.create_client(Trigger, '/robot/confirm_ready')
        self.reset_cli = self.create_client(ResetScenario, '/scenario/reset')

    # ----------------------------------------------------------- callbacks
    def on_truth(self, msg):
        robot, others, obstacles = None, 0, []
        for v in msg.vehicles:
            if v.vehicle_id == 'robot_1':
                robot = v
            elif v.kind == 'obstacle':
                obstacles.append((v.pose.x, v.pose.y))
            elif v.kind == 'pedestrian':
                x0, y0, x1, y1 = geo.CROSSING
                others += x0 <= v.pose.x <= x1 and y0 <= v.pose.y <= y1
            else:
                others += in_c(v.pose.x, v.pose.y, geo.RADIUS_BY_KIND.get(v.kind, 0.3))
        if robot is not None:
            with self.lock:
                self.truth.append((time.monotonic(), self.trial, robot.pose.x, robot.pose.y,
                                   robot.speed_mps, others, obstacles))

    def on_final(self, msg):
        with self.lock:
            self.final.append((time.monotonic(), self.trial, msg.linear.x, msg.angular.z))

    def on_cand(self, msg):
        with self.lock:
            self.cands.append((time.monotonic(), self.trial, msg.twist.linear.x,
                               msg.twist.angular.z))

    def on_event(self, msg):
        if msg.event in ('RESET_ACK', 'SCENARIO_RESET_REQUEST', 'ROUTE_COMPUTED'):
            return
        with self.lock:
            self.events.append((time.monotonic(), self.trial, msg.entity_id, msg.event,
                                msg.reason))

    # ------------------------------------------------------------- helpers
    def check(self, name, passed, detail):
        self.results.append({'check': name, 'passed': bool(passed), 'detail': detail})
        print('[%s] %s: %s' % ('PASS' if passed else 'FAIL', name, detail), flush=True)
        return passed

    def wait_for(self, cond, timeout):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if cond():
                return True
            time.sleep(0.02)
        return cond()

    def wait_future(self, future, timeout):
        return future.result() if self.wait_for(future.done, timeout) else None

    def of(self, rows, trial):
        with self.lock:
            return [r for r in rows if r[1] == trial]

    def robot(self):
        with self.lock:
            return self.truth[-1] if self.truth else None

    def inject(self, **event):
        self.inject_pub.publish(String(data=json.dumps(event)))

    def begin(self, trial):
        res = self.wait_future(self.reset_cli.call_async(ResetScenario.Request(
            scenario_id='S7', seed=trial, preserve_logs=True)), 10.0)
        self.trial = trial
        self.wait_for(lambda: self.robot() and self.robot()[1] == trial
                      and math.dist(self.robot()[2:4], geo.NODES['R']) < 0.05, 3.0)
        time.sleep(2.5)            # traffic labels and obstacle memory settle
        if not (res and res.success):
            self.check('trial %d reset' % trial, False, res and res.reason)
        return bool(res and res.success)

    def send_escort(self, request_id, destination='D', duration=240.0):
        trial = self.trial

        def fb(msg):
            f = msg.feedback
            with self.lock:
                self.feedback.append((time.monotonic(), trial, f.phase, f.progress,
                                      f.distance_remaining_m))
        goal = EscortPassenger.Goal(request_id=request_id, passenger_id='anon-%d' % trial,
                                    pickup_node='R', destination_node=destination,
                                    max_duration_sec=duration)
        return self.wait_future(self.escort.send_goal_async(goal, feedback_callback=fb), 5.0)

    def phase(self, trial):
        fb = self.of(self.feedback, trial)
        return fb[-1][2] if fb else ''

    def press_ready(self):
        res = self.wait_future(self.ready_cli.call_async(Trigger.Request()), 3.0)
        return res

    def wait_ready_and_press(self, trial):
        ok = self.wait_for(lambda: self.phase(trial) == 'WAIT_READY', 15.0)
        return ok and self.press_ready()

    def result(self, handle, timeout):
        r = self.wait_future(handle.get_result_async(), timeout)
        if r is None:
            return 'TIMEOUT', None
        return STATUS.get(r.status, str(r.status)), r.result

    def moved(self, trial, since=0.0):
        pts = [(x, y) for t, _, x, y, *_ in self.of(self.truth, trial) if t >= since]
        return max((math.dist(p, pts[0]) for p in pts), default=0.0)

    def summarize(self, trial, kind, status, result, start):
        trace = self.of(self.truth, trial)
        fb = self.of(self.feedback, trial)
        evs = self.of(self.events, trial)
        row = {'trial': trial, 'kind': kind, 'status': status,
               'reason': result.reason if result else '',
               'final_node': result.final_node if result else '',
               'travelled_m': round(result.travelled_distance_m, 2) if result else '',
               'duration_s': round(time.monotonic() - start, 2)}
        if trace:
            row['arrival_error_m'] = round(math.dist(trace[-1][2:4], D), 3)
        gaps = [b[0] - a[0] for a, b in zip(fb, fb[1:])]
        row['feedback_hz'] = round(len(fb) / (fb[-1][0] - fb[0][0]), 2) if len(fb) > 1 else 0
        row['feedback_max_gap_s'] = round(max(gaps), 3) if gaps else -1
        in_c_t = [t for t, _, x, y, *_ in trace if in_c(x, y)]
        grants = [e[0] for e in evs if e[3] == 'CROSSING_GRANTED']
        releases = [e[0] for e in evs if e[3] == 'CROSSING_RELEASED']
        row['crossing_conflicts'] = sum(1 for t, _, x, y, s, others, _ in trace
                                        if in_c(x, y) and others)
        row['crossing_ok'] = bool(not in_c_t or (grants and releases and grants[0] < in_c_t[0]
                                                 and releases[-1] > in_c_t[-1]))
        row['obstacle_contacts'] = sum(1 for _, _, x, y, _, _, obs in trace for ox, oy in obs
                                       if math.hypot(x - ox, y - oy) < 0.20 + 0.35)
        ready = [e[0] for e in evs if e[3] == 'PASSENGER_READY']
        arrived = [e[0] for e in evs if e[3] == 'ROBOT_ARRIVED']
        row['escort_time_s'] = round(arrived[0] - ready[0], 2) if ready and arrived else ''
        row['route'] = ' | '.join(e[4] for e in evs if e[3] in ('ROUTE', 'REROUTE'))
        self.rows.append(row)
        print('trial %d %s: %s' % (trial, kind, row), flush=True)
        return row

    # -------------------------------------------------------------- trials
    def run(self):
        if not (self.escort.wait_for_server(timeout_sec=20.0)
                and self.reset_cli.wait_for_service(timeout_sec=5.0)
                and self.ready_cli.wait_for_service(timeout_sec=5.0)):
            return self.check('S7 setup', False, 'robot_manager or reset not available')
        for i in range(1, self.nominal + 1):
            self.nominal_trial(i)
        self.report_nominal()
        n = self.nominal
        self.no_ready_trial(n + 1)
        self.obstacle_trial(n + 2)
        self.cancel_trial(n + 3)
        self.busy_trial(n + 4)
        self.unreachable_trial(n + 5)
        self.odom_trial(n + 6)
        return all(r['passed'] for r in self.results)

    def nominal_trial(self, trial):
        if not self.begin(trial):
            return
        start = time.monotonic()
        handle = self.send_escort('esc-%d' % trial)
        if not (handle and handle.accepted and self.wait_ready_and_press(trial)):
            self.summarize(trial, 'nominal', 'NOT_STARTED', None, start)
            return
        status, result = self.result(handle, 240.0)
        self.summarize(trial, 'nominal', status, result, start)

    def report_nominal(self):
        rows = [r for r in self.rows if r['kind'] == 'nominal']
        ok = [r for r in rows if r['status'] == 'SUCCEEDED'
              and r.get('arrival_error_m', 9) <= GOAL_TOL]
        errs = [r.get('arrival_error_m', 9) for r in rows]
        self.check('T13 nominal escort', rows and len(ok) == self.nominal,
                   '%d/%d arrived; arrival error max %.3f m (limit %.2f)'
                   % (len(ok), len(rows), max(errs), GOAL_TOL))
        self.check('S7 feedback rate', rows and all(r['feedback_hz'] >= 2.0 for r in rows),
                   'feedback %.2f-%.2f Hz, largest gap %.3f s'
                   % (min(r['feedback_hz'] for r in rows), max(r['feedback_hz'] for r in rows),
                      max(r['feedback_max_gap_s'] for r in rows)))
        self.check('S7 no crossing conflict or contact',
                   all(r['crossing_conflicts'] == 0 and r['crossing_ok']
                       and r['obstacle_contacts'] == 0 for r in rows),
                   'crossing held while in C in %d/%d trials, 0 conflicts, 0 contacts'
                   % (sum(r['crossing_ok'] for r in rows), len(rows)))

    def no_ready_trial(self, trial):
        if not self.begin(trial):
            return
        idle = self.press_ready()
        start = time.monotonic()
        handle = self.send_escort('esc-%d' % trial)
        status, result = self.result(handle, 40.0) if handle and handle.accepted else \
            ('REJECTED', None)
        row = self.summarize(trial, 'no_ready', status, result, start)
        self.check('T14 no readiness', idle is not None and not idle.success
                   and status == 'ABORTED' and row['reason'] == 'READY_TIMEOUT'
                   and 19.5 <= row['duration_s'] <= 23.0 and self.moved(trial) < 0.02,
                   'ready pressed while idle -> refused (%s); no press -> %s %s after %.1f s, '
                   'moved %.3f m' % (idle and idle.message, status, row['reason'],
                                     row['duration_s'], self.moved(trial)))

    def obstacle_trial(self, trial):
        if not self.begin(trial):
            return
        start = time.monotonic()
        handle = self.send_escort('esc-%d' % trial)
        self.wait_ready_and_press(trial)
        self.wait_for(lambda: self.robot()[2] > 3.6, 30.0)
        fbs = [f[4] for f in self.of(self.feedback, trial)]
        before = fbs[-1] if fbs else -1.0
        self.inject(type='obstacle', id='O', active=True)
        status, result = self.result(handle, 240.0)
        row = self.summarize(trial, 'obstacle', status, result, start)
        after = max(f[4] for f in self.of(self.feedback, trial))
        reroute = [e[4] for e in self.of(self.events, trial) if e[3] == 'REROUTE']
        nominal = [r['escort_time_s'] for r in self.rows
                   if r['kind'] == 'nominal' and r['escort_time_s'] != '']
        mean = sum(nominal) / len(nominal) if nominal else -1
        self.check('S7 obstacle reroute', status == 'SUCCEEDED' and reroute
                   and 'Cn-B-U-E-J-D' in reroute[0] and row['obstacle_contacts'] == 0
                   and row.get('arrival_error_m', 9) <= GOAL_TOL,
                   '%s; remaining distance %.1f m -> up to %.1f m after the reroute; escort '
                   'time %s s vs %.1f s nominal mean (paired runs, same start); arrival '
                   'error %s m' % (reroute[0] if reroute else 'no reroute', before, after,
                                   row['escort_time_s'], mean, row.get('arrival_error_m')))

    def cancel_trial(self, trial):
        if not self.begin(trial):
            return
        start = time.monotonic()
        handle = self.send_escort('esc-%d' % trial)
        self.wait_ready_and_press(trial)
        self.wait_for(lambda: self.robot()[2] > 3.8 and self.robot()[4] > 0.15, 30.0)
        t_req = time.monotonic()
        resp = self.wait_future(handle.cancel_goal_async(), 3.0)
        t_acc = time.monotonic()
        self.wait_for(lambda: any(t >= t_acc and vx == 0 and wz == 0
                                  for t, _, vx, wz in self.of(self.final, trial)), 2.0)
        zero = next((t for t, _, vx, wz in self.of(self.final, trial)
                     if t >= t_acc and vx == 0 and wz == 0), None)
        self.wait_for(lambda: self.robot()[4] < STOPPED, 3.0)
        stop = self.robot()
        pos_acc = next(((x, y) for t, _, x, y, *_ in self.of(self.truth, trial)
                        if t >= t_acc), stop[2:4])
        status, result = self.result(handle, 5.0)
        self.summarize(trial, 'cancel', status, result, start)
        z = zero - t_acc if zero else None
        self.check('T14 cancel while moving',
                   resp is not None and len(resp.goals_canceling) == 1 and z is not None
                   and z <= 0.5 and status == 'CANCELED',
                   'request->acceptance %.3f s, acceptance->zero /robot/cmd_vel %s s; '
                   'simulated robot stopped %.2f s and %.3f m after acceptance; status %s'
                   % (t_acc - t_req, z and round(z, 3), stop[0] - t_acc,
                      math.dist(pos_acc, stop[2:4]), status))

    def busy_trial(self, trial):
        if not self.begin(trial):
            return
        start = time.monotonic()
        handle = self.send_escort('esc-%d' % trial)
        self.wait_for(lambda: self.phase(trial) == 'WAIT_READY', 15.0)
        second = self.wait_future(self.nav.send_goal_async(NavigateTo.Goal(
            request_id='nav-%d' % trial, goal_node='W', max_duration_sec=60.0)), 5.0)
        self.wait_future(handle.cancel_goal_async(), 3.0)
        status, result = self.result(handle, 5.0)
        time.sleep(0.5)
        dup = self.send_escort('esc-1')                     # used in trial 1
        self.summarize(trial, 'busy_and_duplicate', status, result, start)
        reasons = [e[4] for e in self.of(self.events, trial) if e[3].endswith('_REJECTED')]
        self.check('T14 busy and duplicate goals',
                   second is not None and not second.accepted and dup is not None
                   and not dup.accepted and any('BUSY' in r for r in reasons)
                   and any('DUPLICATE_REQUEST' in r for r in reasons),
                   'second goal while busy: accepted=%s; reused request id esc-1: '
                   'accepted=%s; reasons %s' % (second and second.accepted,
                                                dup and dup.accepted, reasons))

    def unreachable_trial(self, trial):
        if not self.begin(trial):
            return
        start = time.monotonic()
        handle = self.send_escort('esc-%d' % trial, destination='P1')
        self.wait_ready_and_press(trial)
        status, result = self.result(handle, 20.0)
        row = self.summarize(trial, 'unreachable', status, result, start)
        moving = [c for c in self.of(self.cands, trial) if c[2] != 0 or c[3] != 0]
        self.check('T14 unreachable destination', status == 'ABORTED'
                   and row['reason'] == 'NO_ROUTE' and not moving
                   and self.moved(trial) < 0.02,
                   'escort to P1 (car-only parking edges): %s %s, %d nonzero candidates, '
                   'moved %.3f m' % (status, row['reason'], len(moving), self.moved(trial)))

    def odom_trial(self, trial):
        if not self.begin(trial):
            return
        start = time.monotonic()
        handle = self.send_escort('esc-%d' % trial)
        self.wait_ready_and_press(trial)
        self.wait_for(lambda: self.robot()[2] > 3.8 and self.robot()[4] > 0.15, 30.0)
        frozen = time.monotonic()
        self.inject(type='freeze', topic='odom', duration=4.0)
        status, result = self.result(handle, 10.0)
        ended = time.monotonic()
        row = self.summarize(trial, 'odom_freeze', status, result, start)
        zero = next((t for t, _, vx, wz in self.of(self.cands, trial)
                     if t >= frozen and vx == 0 and wz == 0), None)
        z = zero - frozen if zero else None
        self.check('T14 stale odometry', status == 'ABORTED' and row['reason'] == 'ODOM_STALE'
                   and z is not None and z <= 0.65 and ended - frozen <= 3.5,
                   'odometry frozen: zero candidate after %s s, %s %s after %.2f s '
                   '(odom timeout 0.5 s + 2 s)' % (z and round(z, 3), status, row['reason'],
                                                   ended - frozen))

    # ------------------------------------------------------------ evidence
    def write(self):
        ok = bool(self.results) and all(r['passed'] for r in self.results)
        out = self.output_dir
        os.makedirs(out, exist_ok=True)
        with open(os.path.join(out, 's7_check.json'), 'w') as fh:
            json.dump({'passed': ok, 'checks': self.results, 'trials': self.rows,
                       'commit': os.environ.get('GIT_COMMIT', 'unknown')}, fh, indent=2)
        if self.rows:
            keys = []
            for r in self.rows:
                keys += [k for k in r if k not in keys]
            with open(os.path.join(out, 's7_trials.csv'), 'w', newline='') as fh:
                w = csv.DictWriter(fh, fieldnames=keys)
                w.writeheader()
                w.writerows(self.rows)
        with self.lock:
            truth, events, feedback = list(self.truth), list(self.events), list(self.feedback)
        t0 = truth[0][0] if truth else 0.0
        with open(os.path.join(out, 's7_trace.csv'), 'w') as fh:
            fh.write('t,trial,x,y,speed\n')
            for t, tr, x, y, s, *_ in truth:
                fh.write('%.3f,%d,%.3f,%.3f,%.3f\n' % (t - t0, tr, x, y, s))
        with open(os.path.join(out, 's7_events.csv'), 'w', newline='') as fh:
            w = csv.writer(fh)
            w.writerow(['t', 'trial', 'source', 'event', 'detail'])
            rows = [(t, tr, ent, ev, rs) for t, tr, ent, ev, rs in events]
            rows += [(t, tr, 'escort_action', 'FEEDBACK', '%s progress=%.2f remaining=%.2f'
                      % (ph, pr, d)) for t, tr, ph, pr, d in feedback]
            for t, tr, ent, ev, rs in sorted(rows):
                w.writerow(['%.3f' % (t - t0), tr, ent, ev, rs])
        plot_paths([(tr, t, x, y) for t, tr, x, y, *_ in truth],
                   {r['trial']: r['kind'] for r in self.rows}, out)
        print('S7 %s; evidence: %s' % ('PASSED' if ok else 'FAILED', out), flush=True)
        return ok


def plot_paths(trace, kinds, out):
    """Robot paths per trial on the map; nominal trials grey."""
    trials = sorted(k for k in kinds if any(r[0] == k for r in trace))
    if not trials:
        return
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle, Rectangle
    fig, ax = plt.subplots(figsize=(8, 5.5))
    x0, y0, x1, y1 = geo.CROSSING
    ax.add_patch(Rectangle((x0, y0), x1 - x0, y1 - y0, color='orange', alpha=0.25,
                           label='crossing C'))
    ax.add_patch(Circle((7.0, 2.0), 0.35, color='red', alpha=0.3, label='obstacle O'))
    for e, (a, b) in geo.EDGES.items():
        ax.plot([a[0], b[0]], [a[1], b[1]], color='lightgrey', linewidth=0.8, zorder=0)
    for trial in trials:
        pts = [(x, y) for tr, t, x, y in trace if tr == trial]
        kind = kinds[trial]
        style = {'color': 'grey', 'linewidth': 0.8} if kind == 'nominal' else \
            {'linewidth': 1.6, 'label': '%d %s' % (trial, kind)}
        ax.plot([p[0] for p in pts], [p[1] for p in pts], **style)
    for name, (x, y) in geo.NODES.items():
        ax.plot(x, y, 'k.', markersize=4)
        ax.annotate(name, (x, y), textcoords='offset points', xytext=(3, 3), fontsize=8)
    ax.set_xlim(0, 12)
    ax.set_ylim(0, 8)
    ax.set_aspect('equal')
    ax.set_title('S7 robot paths (grey: %d nominal escorts R -> D)'
                 % sum(1 for k in trials if kinds[k] == 'nominal'))
    ax.legend(loc='upper right', fontsize=7)
    fig.tight_layout()
    fig.savefig(os.path.join(out, 's7_paths.png'), dpi=120)
    plt.close(fig)


def main(args=None):
    rclpy.init(args=args)
    node = S7Check()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spinner = threading.Thread(target=executor.spin, daemon=True)
    spinner.start()
    ok = False
    try:
        node.run()
    finally:
        ok = node.write()
        executor.shutdown()
        spinner.join(timeout=5.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
