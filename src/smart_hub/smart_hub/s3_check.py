"""s3_check (M2-A, written by M5): runs the S3 docking tests on the live graph.

It acts as the DockBus client (the coordinator's role) for the test trials,
resets the scenario before each trial and injects the disturbances through
/scenario/inject (test only). It checks what the real bus_manager,
hub_coordinator, safety_supervisor, actuator_bridge and simulator do.

Trials (assignment 10.3, tests T05 and T06):
  1-10  nominal docking            T05: 10/10, position <= 0.20 m, yaw <= 0.17 rad,
                                   no open-door movement, crossing held while in C,
                                   ETA error at 5 m <= 3 s
  11    cancel during approach     T06: zero /bus/cmd_vel within 0.5 s of acceptance
  12    pedestrian in C            bus waits outside C, docks after the person leaves
  13    blocked bay                bus waits outside C without the crossing, then docks
  14    stale bus pose             zero candidate within pose_timeout, then docks
  15    coordinator dispatch       7 people in Q: hub_coordinator sends the goal itself;
                                   4 board, 3 stay waiting

Writes <output_dir>/s3_check.json, s3_trials.csv, s3_trace.csv (path trace,
speed, door, phase), s3_events.csv (crossing, door, docking, boarding
events, action feedback and results) and s3_paths.png. Exit code 0 only if
every check passed.
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
from smart_hub.ros_common import declare, R1, R10, R100
from smart_hub_interfaces.action import DockBus
from smart_hub_interfaces.msg import HubEvent, VehicleStateArray
from smart_hub_interfaces.srv import ResetScenario
from std_msgs.msg import Bool, String

BAY = geo.NODES['B']
POS_TOL, YAW_TOL = 0.20, 0.17
CANCEL_LIMIT = 0.5        # s, T06
ETA_LIMIT = 3.0           # s at 5 m remaining
STALE_LIMIT = 0.65        # s: bus pose_timeout 0.5 s + one 10 Hz frame + one tick
STOPPED = 0.02
QUEUE = (2.0, 5.0, 6.0, 7.0)
STATUS = {GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED', GoalStatus.STATUS_CANCELED: 'CANCELED',
          GoalStatus.STATUS_ABORTED: 'ABORTED'}


def in_c(x, y, r=0.30):
    return geo.circle_overlaps_rect(x, y, r, geo.CROSSING)


def in_q(x, y):
    return QUEUE[0] <= x <= QUEUE[2] and QUEUE[1] <= y <= QUEUE[3]


class S3Check(Node):
    def __init__(self, **kwargs):
        super().__init__('s3_check', **kwargs)
        p = declare(self, {'output_dir': 'output', 'nominal_trials': 10,
                           'trial_timeout': 90.0})
        self.output_dir = p['output_dir']
        self.nominal = int(p['nominal_trials'])
        self.timeout = float(p['trial_timeout'])
        self.lock = threading.Lock()
        self.trial = 0
        self.truth = []           # (t, trial, x, y, theta, speed, peds_in_c, peds_in_q, others)
        self.final_cmds = []      # (t, trial, vx, wz) /bus/cmd_vel
        self.cands = []           # (t, trial, vx, wz) /bus/cmd_vel_candidate
        self.doors = []           # (t, trial, open)
        self.events = []          # (t, trial, entity, event, reason)
        self.feedback = []        # (t, trial, phase, distance, eta)
        self.results = []
        self.rows = []            # per-trial summary
        self.inject_pub = self.create_publisher(String, '/scenario/inject', R10)
        self.create_subscription(VehicleStateArray, '/sim/truth', self.on_truth, R10)
        self.create_subscription(Twist, '/bus/cmd_vel', self.on_cmd, R10)
        self.create_subscription(TwistStamped, '/bus/cmd_vel_candidate', self.on_cand, R10)
        self.create_subscription(Bool, '/bus/door_open', self.on_door, R1)
        self.create_subscription(HubEvent, '/hub/events', self.on_event, R100)
        self.dock = ActionClient(self, DockBus, '/bus/dock')
        self.reset_cli = self.create_client(ResetScenario, '/scenario/reset')

    # ----------------------------------------------------------- callbacks
    def on_truth(self, msg):
        bus, peds_c, peds_q, others = None, 0, 0, []
        for v in msg.vehicles:
            if v.vehicle_id == 'bus_1':
                bus = v
            elif v.kind == 'pedestrian':
                x0, y0, x1, y1 = geo.CROSSING
                peds_c += x0 <= v.pose.x <= x1 and y0 <= v.pose.y <= y1
                peds_q += in_q(v.pose.x, v.pose.y)
            else:
                others.append((v.pose.x, v.pose.y, geo.RADIUS_BY_KIND.get(v.kind, 0.3)))
        if bus is not None:
            with self.lock:
                self.truth.append((time.monotonic(), self.trial, bus.pose.x, bus.pose.y,
                                   bus.pose.theta, bus.speed_mps, peds_c, peds_q, others))

    def on_cmd(self, msg):
        with self.lock:
            self.final_cmds.append((time.monotonic(), self.trial, msg.linear.x, msg.angular.z))

    def on_cand(self, msg):
        with self.lock:
            self.cands.append((time.monotonic(), self.trial, msg.twist.linear.x,
                               msg.twist.angular.z))

    def on_door(self, msg):
        with self.lock:
            self.doors.append((time.monotonic(), self.trial, bool(msg.data)))

    def on_event(self, msg):
        if msg.event in ('RESET_ACK', 'SCENARIO_RESET_REQUEST'):
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
        return self.wait_for(future.done, timeout) and future.result()

    def inject(self, **event):
        self.inject_pub.publish(String(data=json.dumps(event)))

    def bus(self):
        with self.lock:
            return self.truth[-1] if self.truth else None

    def reset(self, seed):
        res = self.wait_future(self.reset_cli.call_async(ResetScenario.Request(
            scenario_id='S3', seed=seed, preserve_logs=True)), 10.0)
        self.trial = seed
        # the new world must be on /sim/truth, the bus pose fresh in bus_manager
        self.wait_for(lambda: self.bus() and self.bus()[1] == seed
                      and math.dist(self.bus()[2:4], geo.NODES['A']) < 0.1, 3.0)
        time.sleep(1.0)
        return res and res.success, res and res.reason

    def send_goal(self):
        goal = DockBus.Goal(bus_id='bus_1', bay_id='B', max_duration_sec=self.timeout)
        trial = self.trial

        def on_feedback(fb):
            f = fb.feedback
            with self.lock:
                self.feedback.append((time.monotonic(), trial, f.phase,
                                      f.distance_remaining_m, f.eta_sec))
        handle = self.wait_future(self.dock.send_goal_async(goal, feedback_callback=on_feedback),
                                  5.0)
        return handle if handle and handle.accepted else None

    def result(self, handle, timeout):
        res = self.wait_future(handle.get_result_async(), timeout)
        if not res:
            return 'TIMEOUT', None
        return STATUS.get(res.status, str(res.status)), res.result

    def of(self, rows, trial):
        return [r for r in rows if r[1] == trial]

    # ------------------------------------------------------------ analysis
    def summarize(self, trial, kind, status, result, start):
        trace = self.of(self.truth, trial)
        row = {'trial': trial, 'kind': kind, 'status': status,
               'final_state': result.final_state if result else '',
               'reason': result.reason if result else '',
               'duration_s': round(time.monotonic() - start, 2)}
        if trace:
            _, _, x, y, th, *_ = trace[-1]
            row['pos_err_m'] = round(math.dist((x, y), BAY), 3)
            row['yaw_err_rad'] = round(abs(math.atan2(math.sin(th), math.cos(th))), 3)
        door_t = [t for t, _, opened in self.of(self.doors, trial) if opened]
        moving = [s for t, _, x, y, th, s, *_ in trace
                  if door_t and any(abs(t - d) < 0.05 for d in door_t) and s > STOPPED]
        row['door_open_s'] = round(len(door_t) * 0.1, 1)
        row['moving_with_door_open'] = len(moving)
        row['collisions'] = sum(1 for _, _, x, y, _, _, _, _, others in trace
                                for ox, oy, r in others
                                if math.hypot(x - ox, y - oy) < 0.30 + r)
        row['in_c_with_pedestrian'] = sum(1 for _, _, x, y, _, _, pc, *_ in trace
                                          if pc and in_c(x, y))
        evs = self.of(self.events, trial)
        boarded = [e for e in evs if e[3] == 'PASSENGER_BOARDED']
        row['boarded'] = len(boarded)
        row['left_waiting'] = trace[-1][7] if trace else -1
        in_c_t = [t for t, _, x, y, *_ in trace if in_c(x, y)]
        grant = [e[0] for e in evs if e[2] == 'bus_1' and e[3] == 'CROSSING_GRANTED']
        release = [e[0] for e in evs if e[2] == 'bus_1' and e[3] == 'CROSSING_RELEASED']
        row['crossing_ok'] = bool(not in_c_t or (grant and release and grant[0] < in_c_t[0]
                                                 and release[-1] > in_c_t[-1]))
        row['eta_err_s'] = self.eta_error(trial)
        fb = self.of(self.feedback, trial)
        row['eta_invalid_s'] = round(sum(0.2 for f in fb if f[4] < 0 and f[2] != 'BOARD'), 1)
        self.rows.append(row)
        return row

    def eta_error(self, trial):
        fb = [f for f in self.of(self.feedback, trial) if 0 <= f[3] <= 5.0]
        trace = self.of(self.truth, trial)
        arrival = next((t for t, _, x, y, _, s, *_ in trace
                        if math.dist((x, y), BAY) < POS_TOL and s < STOPPED), None)
        if not fb or fb[0][4] < 0 or arrival is None:
            return None
        t, _, _, _, eta = fb[0]
        return round(abs(t + eta - arrival), 2)

    # ------------------------------------------------------------ trials
    def run(self):
        if not (self.dock.wait_for_server(timeout_sec=20.0)
                and self.reset_cli.wait_for_service(timeout_sec=5.0)):
            return self.check('S3 servers', False, '/bus/dock or /scenario/reset not available')
        for i in range(1, self.nominal + 1):
            self.nominal_trial(i)
        self.report_nominal()
        self.cancel_trial(self.nominal + 1)
        self.pedestrian_trial(self.nominal + 2)
        self.bay_trial(self.nominal + 3)
        self.stale_trial(self.nominal + 4)
        self.dispatch_trial(self.nominal + 5)
        return all(r['passed'] for r in self.results)

    def begin(self, trial):
        ok, reason = self.reset(trial)
        if not ok:
            self.check('trial %d reset' % trial, False, 'reset failed: %s' % reason)
        return ok

    def nominal_trial(self, trial):
        if not self.begin(trial):
            return
        start = time.monotonic()
        handle = self.send_goal()
        status, result = self.result(handle, self.timeout + 5) if handle else ('REJECTED', None)
        row = self.summarize(trial, 'nominal', status, result, start)
        print('trial %d nominal: %s' % (trial, row), flush=True)

    def report_nominal(self):
        rows = [r for r in self.rows if r['kind'] == 'nominal']
        ok = [r for r in rows if r['status'] == 'SUCCEEDED'
              and r.get('pos_err_m', 9) <= POS_TOL and r.get('yaw_err_rad', 9) <= YAW_TOL]
        self.check('T05 nominal docking', len(ok) == self.nominal and rows,
                   '%d/%d docked; position error max %.3f m, yaw error max %.3f rad'
                   % (len(ok), len(rows), max(r.get('pos_err_m', 9) for r in rows),
                      max(r.get('yaw_err_rad', 9) for r in rows)))
        self.check('T05 door interlock',
                   all(r['moving_with_door_open'] == 0 and r['door_open_s'] >= 7.9
                       for r in rows),
                   'moving samples with the door open: %d; door open %.1f-%.1f s per stop'
                   % (sum(r['moving_with_door_open'] for r in rows),
                      min(r['door_open_s'] for r in rows), max(r['door_open_s'] for r in rows)))
        self.check('S3 no collision', all(r['collisions'] == 0 for r in rows),
                   '%d overlapping samples' % sum(r['collisions'] for r in rows))
        self.check('S3 crossing held while in C', all(r['crossing_ok'] for r in rows),
                   'grant before the footprint entered C and release after it left, '
                   '%d/%d trials' % (sum(r['crossing_ok'] for r in rows), len(rows)))
        self.check('S3 boarding', all(r['boarded'] == 4 and r['left_waiting'] == 1
                                      for r in rows),
                   'boarded %s, left waiting %s (5 waiting, capacity 4)'
                   % (sorted({r['boarded'] for r in rows}),
                      sorted({r['left_waiting'] for r in rows})))
        errs = [r['eta_err_s'] for r in rows if r['eta_err_s'] is not None]
        self.check('S3 ETA error at 5 m', len(errs) == len(rows) and max(errs) <= ETA_LIMIT,
                   'mean %.2f s, max %.2f s over %d trials (limit %.1f); ETA invalid '
                   '%.1f s per trial on average (start, before a credible speed)'
                   % (sum(errs) / len(errs) if errs else -1, max(errs) if errs else -1,
                      len(errs), ETA_LIMIT,
                      sum(r['eta_invalid_s'] for r in rows) / len(rows)))

    def wait_bus(self, cond, timeout):
        return self.wait_for(lambda: self.bus() is not None and cond(*self.bus()[2:6]), timeout)

    def cancel_trial(self, trial):
        if not self.begin(trial):
            return
        start = time.monotonic()
        handle = self.send_goal()
        moving = handle and self.wait_bus(lambda x, y, th, s: x > 2.5 and abs(y - 4) < 0.1
                                          and s > 0.2, 40.0)
        if not moving:
            self.check('T06 cancel', False, 'bus never reached the N-Cn approach')
            return
        resp = self.wait_future(handle.cancel_goal_async(), 2.0)
        accepted = time.monotonic()
        zero = self.wait_for(lambda: any(t >= accepted and vx == 0 and wz == 0
                                         for t, _, vx, wz in self.of(self.final_cmds, trial)),
                             2.0)
        first = next((t for t, _, vx, wz in self.of(self.final_cmds, trial)
                      if t >= accepted and vx == 0 and wz == 0), None)
        status, result = self.result(handle, 5.0)
        stop = self.wait_bus(lambda x, y, th, s: s < STOPPED, 3.0)
        stopped_t = self.bus()[0] - accepted if stop else None
        row = self.summarize(trial, 'cancel', status, result, start)
        delay = first - accepted if zero else None
        self.check('T06 cancel during approach',
                   resp is not None and len(resp.goals_canceling) == 1 and zero
                   and delay <= CANCEL_LIMIT and status == 'CANCELED',
                   'cancel accepted; first zero /bus/cmd_vel %s s later (limit %.1f); '
                   'bus stopped after %s s; status %s, final_state %s'
                   % (delay and round(delay, 3), CANCEL_LIMIT,
                      stopped_t and round(stopped_t, 2), status, row['final_state']))
        released = any(e[3] == 'CROSSING_RELEASED' for e in self.of(self.events, trial))
        self.check('T06 crossing released on cancel', released,
                   'crossing permission released after the cancel (bus outside C)')

    def pedestrian_trial(self, trial):
        if not self.begin(trial):
            return
        start = time.monotonic()
        self.inject(type='pedestrian', id='c1', x=5.0, y=3.0)
        handle = self.send_goal()
        waited = handle and self.wait_for(self.waiting(trial, 3.0), 60.0)
        self.inject(type='pedestrian', id='c1', active=False)
        status, result = self.result(handle, self.timeout) if handle else ('REJECTED', None)
        row = self.summarize(trial, 'pedestrian_in_C', status, result, start)
        denied = [e[4] for e in self.of(self.events, trial) if e[3] == 'CROSSING_ACQUIRE_DENIED']
        self.check('S3 pedestrian in C', waited and row['in_c_with_pedestrian'] == 0
                   and status == 'SUCCEEDED',
                   'waited outside C (denied: %s), bus in C with a pedestrian there: %d '
                   'samples; then %s, position error %s m'
                   % (sorted(set(denied)), row['in_c_with_pedestrian'], status,
                      row.get('pos_err_m')))

    def waiting(self, trial, hold):
        """Condition: the bus has reported phase WAIT for hold seconds."""
        def cond():
            fb = [f for f in self.of(self.feedback, trial) if f[2] == 'WAIT']
            return bool(fb) and fb[-1][0] - fb[0][0] >= hold
        return cond

    def bay_trial(self, trial):
        if not self.begin(trial):
            return
        start = time.monotonic()
        self.inject(type='vehicle', id='car_9', kind='car', x=8.0, y=4.0)
        handle = self.send_goal()
        waited = handle and self.wait_for(self.waiting(trial, 3.0), 60.0)
        trace_before = list(self.of(self.truth, trial))
        self.inject(type='vehicle', id='car_9', active=False)
        status, result = self.result(handle, self.timeout) if handle else ('REJECTED', None)
        row = self.summarize(trial, 'blocked_bay', status, result, start)
        reasons = [e[4] for e in self.of(self.events, trial) if e[3] == 'BUS_PHASE']
        entered = any(in_c(x, y) for _, _, x, y, *_ in trace_before)
        self.check('S3 blocked bay', waited and not entered and status == 'SUCCEEDED'
                   and any('BAY_BLOCKED' in r for r in reasons),
                   'WAIT with BAY_BLOCKED outside C (entered C while blocked: %s); '
                   'after the car left: %s, position error %s m'
                   % (entered, status, row.get('pos_err_m')))

    def stale_trial(self, trial):
        if not self.begin(trial):
            return
        start = time.monotonic()
        handle = self.send_goal()
        on_edge = handle and self.wait_bus(lambda x, y, th, s: y > 2.8 and s > 0.1, 30.0)
        frozen = time.monotonic()
        self.inject(type='freeze', topic='vehicles', duration=1.5)
        self.wait_for(lambda: False, 1.6)
        first = next((t for t, _, vx, wz in self.of(self.cands, trial)
                      if t >= frozen and vx == 0 and wz == 0), None)
        status, result = self.result(handle, self.timeout) if handle else ('REJECTED', None)
        row = self.summarize(trial, 'stale_pose', status, result, start)
        phases = {f[2] for f in self.of(self.feedback, trial)}
        delay = first - frozen if first else None
        self.check('S3 stale bus pose', on_edge and delay is not None and delay <= STALE_LIMIT
                   and 'SAFE_STOP' in phases and status == 'SUCCEEDED',
                   'zero candidate %s s after the pose froze (limit %.2f); SAFE_STOP '
                   'seen: %s; after the pose returned: %s, position error %s m'
                   % (delay and round(delay, 3), STALE_LIMIT, 'SAFE_STOP' in phases,
                      status, row.get('pos_err_m')))

    def dispatch_trial(self, trial):
        if not self.begin(trial):
            return
        start = time.monotonic()
        self.inject(type='pedestrian', id='q6', x=2.8, y=5.6)
        self.inject(type='pedestrian', id='q7', x=4.6, y=5.6)

        def ended():
            return any(e[3] in ('BUS_MISSION_SUCCEEDED', 'BUS_MISSION_ABORTED',
                                'BUS_MISSION_CANCELED') for e in self.of(self.events, trial))
        self.wait_for(ended, self.timeout + 20)
        time.sleep(0.5)
        evs = self.of(self.events, trial)
        names = [e[3] for e in evs]
        row = self.summarize(trial, 'coordinator_dispatch',
                             'SUCCEEDED' if 'BUS_MISSION_SUCCEEDED' in names else 'FAILED',
                             None, start)
        source = [e[4] for e in evs if e[3] == 'BUS_DISPATCH_REQUESTED']
        self.check('S3 coordinator dispatch',
                   'BUS_DISPATCH_ACCEPTED' in names and 'BUS_MISSION_SUCCEEDED' in names
                   and row['boarded'] == 4 and row['left_waiting'] == 3,
                   '7 people in Q -> hub_coordinator sent the goal (%s), mission %s; '
                   'boarded %d, %d left waiting'
                   % (','.join(source) or 'none', row['status'], row['boarded'],
                      row['left_waiting']))

    # ------------------------------------------------------------ evidence
    def write(self):
        ok = bool(self.results) and all(r['passed'] for r in self.results)
        out = self.output_dir
        os.makedirs(out, exist_ok=True)
        with open(os.path.join(out, 's3_check.json'), 'w') as fh:
            json.dump({'passed': ok, 'checks': self.results, 'trials': self.rows,
                       'commit': os.environ.get('GIT_COMMIT', 'unknown')}, fh, indent=2)
        if self.rows:
            with open(os.path.join(out, 's3_trials.csv'), 'w', newline='') as fh:
                keys = []
                for r in self.rows:
                    keys += [k for k in r if k not in keys]
                w = csv.DictWriter(fh, fieldnames=keys)
                w.writeheader()
                w.writerows(self.rows)
        with self.lock:
            truth, doors = list(self.truth), list(self.doors)
            events, feedback = list(self.events), list(self.feedback)
        t0 = truth[0][0] if truth else 0.0
        with open(os.path.join(out, 's3_trace.csv'), 'w') as fh:
            fh.write('t,trial,x,y,theta,speed\n')
            for t, trial, x, y, th, s, *_ in truth:
                fh.write('%.3f,%d,%.3f,%.3f,%.3f,%.3f\n' % (t - t0, trial, x, y, th, s))
        with open(os.path.join(out, 's3_events.csv'), 'w', newline='') as fh:
            w = csv.writer(fh)
            w.writerow(['t', 'trial', 'source', 'event', 'detail'])
            rows = [(t, tr, ent, ev, rs) for t, tr, ent, ev, rs in events]
            rows += [(t, tr, 'dock_action', 'FEEDBACK', '%s d=%.2f eta=%.2f' % (ph, d, eta))
                     for t, tr, ph, d, eta in feedback]
            rows += [(t, tr, 'actuator_bridge', 'DOOR_OPEN' if o else 'DOOR_CLOSED', '')
                     for i, (t, tr, o) in enumerate(doors) if i == 0 or doors[i - 1][2] != o]
            for t, tr, ent, ev, rs in sorted(rows):
                w.writerow(['%.3f' % (t - t0), tr, ent, ev, rs])
        self.plot(truth)
        print('S3 %s; evidence: %s' % ('PASSED' if ok else 'FAILED', out), flush=True)
        return ok

    def plot(self, truth):
        plot_paths([(tr, t, x, y) for t, tr, x, y, *_ in truth],
                   {r['trial']: r['kind'] for r in self.rows}, self.output_dir)


def plot_paths(trace, kinds, out):
    """Bus paths (top) and distance to bay B over time (bottom) per trial.

    trace: [(trial, t, x, y)], kinds: {trial: kind}. Also usable on
    s3_trace.csv after a run.
    """
    trials = sorted(k for k in kinds if any(r[0] == k for r in trace))
    if not trials:
        return
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle
    fig, (ax, ax2) = plt.subplots(2, 1, figsize=(8, 7.5),
                                  gridspec_kw={'height_ratios': [1, 1.1]})
    x0, y0, x1, y1 = geo.CROSSING
    ax.add_patch(Rectangle((x0, y0), x1 - x0, y1 - y0, color='orange', alpha=0.3,
                           label='crossing C'))
    nominal = sum(1 for k in trials if kinds[k] == 'nominal')
    for trial in trials:
        pts = [(t, x, y) for tr, t, x, y in trace if tr == trial]
        kind = kinds[trial]
        style = {'color': 'grey', 'linewidth': 0.8} if kind == 'nominal' else \
            {'linewidth': 1.5, 'label': '%d %s' % (trial, kind)}
        ax.plot([p[1] for p in pts], [p[2] for p in pts], **style)
        t0 = pts[0][0]
        ax2.plot([p[0] - t0 for p in pts], [math.dist(p[1:], BAY) for p in pts], **style)
    for name in ('A', 'N', 'Cn', 'B'):
        ax.plot(*geo.NODES[name], 'ko')
        ax.annotate(name, geo.NODES[name], textcoords='offset points', xytext=(4, 4))
    ax.set_xlim(0, 9)
    ax.set_ylim(1.5, 5)
    ax.set_aspect('equal')
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.set_title('S3 bus paths (grey: %d nominal trials)' % nominal)
    ax2.axhline(POS_TOL, color='green', linestyle=':', label='docking tolerance 0.20 m')
    ax2.set_xlabel('time since the trial reset (s)')
    ax2.set_ylabel('straight-line distance to B (m)')
    ax2.legend(loc='upper right', fontsize=7)
    fig.tight_layout()
    fig.savefig(os.path.join(out, 's3_paths.png'), dpi=120)
    plt.close(fig)


def main(args=None):
    rclpy.init(args=args)
    node = S3Check()
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
