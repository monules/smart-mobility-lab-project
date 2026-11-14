"""s9_repeat_check (M5): every S9 fault, five times, at five mission phases.

Assignment 16.3: "Repeat each fault five times at different mission phases.
Report zero-command latency and actual simulator velocity separately."

Faults (assignment 16.1, the ones the M5 stack owns):
  estop            /hub/estop true            -> latched stop, deliberate reset
  candidate_loss   manager stops publishing   -> zero after 0.5 s, resumes
  odom_freeze      /robot/odom silent 2 s     -> zero after 1.0 s, resumes
  vehicles_freeze  /hub/vehicles silent 2 s   -> zero after 1.0 s, resumes
  estop_signal     /hub/estop silent 2 s      -> zero after 1.0 s, resumes
Phases (what the robot is doing when the fault hits):
  start (0.3 s after moving off), cruise, turn, max_speed (0.25 m/s), reverse

Every trial starts from a fresh world (/scenario/reset, which also checks the
reset handshake). For each trial it measures:
  zero_latency_s  fault onset -> first zero /robot/cmd_vel
  sim_stop_s      fault onset -> simulated speed < 0.02 m/s
  reason          the named reason on /hub/safety
  no_motion       no nonzero output while the fault lasts; for a freeze the
                  fault ends when the frozen topic is heard again (see
                  fault_window_end), not after 2 s of wall-clock time
  recovered       latched: reset accepted and robot still until a new
                  command; holds: motion resumes by itself after the fault
Writes <output_dir>/s9_repeat.json and s9_repeat.csv; exit 0 only if every
trial passed.
"""
import csv
import json
import os
import sys
import threading
import time

from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import SingleThreadedExecutor
from smart_hub.ros_common import R10, R100
from smart_hub.s9_fault_check import EPS, S9Check, STOPPED
from smart_hub_interfaces.msg import SafetyDecision, VehicleStateArray
from smart_hub_interfaces.srv import ResetScenario
from std_msgs.msg import String

# phase: (linear m/s, angular rad/s, seconds of motion before the fault)
PHASES = {'start': (0.15, 0.0, 0.3), 'cruise': (0.20, 0.0, 1.5),
          'turn': (0.15, 0.5, 1.0), 'max_speed': (0.25, 0.0, 1.0),
          'reverse': (-0.10, 0.0, 1.0)}
# fault: (expected reason on /hub/safety, zero-latency limit s, latched?)
FAULTS = {'estop': ('LATCHED:ESTOP', 0.20, True),
          'candidate_loss': ('robot:COMMAND_STALE', 0.5 + 0.25, False),
          'odom_freeze': ('ODOM_STALE', 1.0 + 0.25, False),
          'vehicles_freeze': ('VEHICLES_STALE', 1.0 + 0.25, False),
          'estop_signal': ('ESTOP_SIGNAL_STALE', 1.0 + 0.25, False)}
FAULT_SEC = 2.0
FREEZE_GAP_SEC = 0.5           # a silence this long on a topic means it was frozen
RESET_CLIENT_TIMEOUT = 1.0     # s, assignment 7.1 client timeout for short transactions


class S9Repeat(S9Check):
    def __init__(self, **kwargs):
        super().__init__(node_name='s9_repeat_check', **kwargs)
        self.reset_cli = self.create_client(ResetScenario, '/scenario/reset')
        self.trial_rows = []
        self.reasons = []                       # (t, reason) of /hub/safety
        self.last_reset_ms = None
        # the supervisor's own decision records: latch state and commands in
        # one numbered message (no cross-topic ordering, no grace window)
        self.decisions = {'seen': 0, 'last_seq': None, 'seq_gaps': 0,
                          'latched_with_nonzero_command': 0}
        self.create_subscription(SafetyDecision, '/hub/safety_decision', self.on_decision,
                                 R100)
        # receipt times of the two topics a freeze silences, to see when the
        # frozen data really comes back
        self.arrivals = {'odom': [], 'vehicles': []}
        self.create_subscription(Odometry, '/robot/odom',
                                 lambda _m: self.on_arrival('odom'), R10)
        self.create_subscription(VehicleStateArray, '/hub/vehicles',
                                 lambda _m: self.on_arrival('vehicles'), R10)

    def on_arrival(self, topic):
        with self.lock:
            self.arrivals[topic].append(time.monotonic())

    def fault_window_end(self, fault, t0, t_end):
        """Return when the fault really ended, never later than t_end.

        A freeze lasts FAULT_SEC of *simulator* time, which can end a few
        tens of milliseconds before FAULT_SEC of wall-clock time. The robot
        then rightly moves again while a wall-clock window is still open,
        and that restart was once counted as "motion during the fault"
        (vehicles_freeze, reverse, seed 120). So for a freeze the window ends
        at the first message after the silence. Motion before that moment,
        while the data is really missing, still fails the trial.
        """
        topic = {'odom_freeze': 'odom', 'vehicles_freeze': 'vehicles'}.get(fault)
        if topic is None:
            return t_end                 # other faults are ended by this checker at t_end
        with self.lock:
            times = list(self.arrivals[topic])
        for prev, cur in zip(times, times[1:]):
            if cur > t0 and cur - prev >= FREEZE_GAP_SEC:
                return min(cur, t_end)
        return t_end

    def on_safety(self, msg):
        super().on_safety(msg)
        with self.lock:
            self.reasons.append((time.monotonic(), msg.reason))

    def on_decision(self, msg):
        with self.lock:
            dec = self.decisions
            dec['seen'] += 1
            if dec['last_seq'] is not None and msg.seq > dec['last_seq'] + 1:
                dec['seq_gaps'] += msg.seq - dec['last_seq'] - 1
            dec['last_seq'] = msg.seq
            if msg.stop_latched and any(abs(v) > EPS for v in
                                        list(msg.linear_x) + list(msg.angular_z)):
                dec['latched_with_nonzero_command'] += 1

    def call(self, client, req, timeout=5.0):
        fut = client.call_async(req)
        if not self.wait_for(fut.done, timeout):
            return None
        return fut.result()

    def zero_after(self, t0):
        return next((c[0] for c in self.cmds_since(t0)
                     if abs(c[1]) < EPS and abs(c[2]) < EPS), None)

    def reason_seen(self, token, t0, timeout=1.5):
        def seen():
            with self.lock:
                return any(t >= t0 and token in r for t, r in self.reasons)
        return self.wait_for(seen, timeout)

    def fresh_world(self, seed):
        # assignment 7.1: the client waits 1 s for a short transaction
        t0 = time.monotonic()
        res = self.call(self.reset_cli, ResetScenario.Request(scenario_id='S9', seed=seed),
                        timeout=RESET_CLIENT_TIMEOUT)
        self.last_reset_ms = round((time.monotonic() - t0) * 1000, 1)
        if res is None:
            return False, 'reset not answered within %.1f s' % RESET_CLIENT_TIMEOUT
        if not res.success:
            return False, 'reset failed: %s' % res.reason
        if self.latched():
            self.set_cand((0.0, 0.0))
            self.set_estop(False)
            time.sleep(0.7)
            ok, reason = self.clear()
            if not ok:
                return False, 'clear failed: %s' % reason
        return True, res.run_id

    def inject(self, fault):
        if fault == 'estop':
            self.set_estop(True)
        elif fault == 'candidate_loss':
            self.set_cand(None)
        elif fault in ('odom_freeze', 'vehicles_freeze'):
            topic = fault.split('_')[0]
            self.inject_pub.publish(String(data=json.dumps(
                {'type': 'freeze', 'topic': topic, 'duration': FAULT_SEC})))
        elif fault == 'estop_signal':
            self.heartbeat = False

    def end_fault(self, fault, cmd):
        if fault == 'candidate_loss':
            self.set_cand(cmd)
        elif fault == 'estop_signal':
            self.heartbeat = True
        # freezes end by themselves after FAULT_SEC

    def trial(self, fault, phase, seed):
        token, limit, latched = FAULTS[fault]
        v, w, dwell = PHASES[phase]
        row = {'fault': fault, 'phase': phase, 'seed': seed}
        ok, info = self.fresh_world(seed)
        if not ok:
            row.update(passed=False, note=info)
            return row
        row['run_id'] = info
        row['reset_ms'] = self.last_reset_ms
        cmd = (v, w)
        self.set_cand(cmd)
        moving = self.wait_for(lambda: self.moving_cmd(v) and self.robot_speed() > 0.7 * abs(v),
                               5.0)
        time.sleep(dwell)
        speed0 = self.robot_speed()
        t0 = time.monotonic()
        self.inject(fault)
        self.wait_for(lambda: self.zero_after(t0) is not None, limit + 1.0)
        zero_t = self.zero_after(t0)
        self.wait_for(lambda: self.robot_speed() < STOPPED and self.truth_t > t0, 3.0)
        stop_t = self.truth_t if self.robot_speed() < STOPPED else None
        reason_ok = self.reason_seen(token, t0)
        # hold the fault, then check nothing moved while it lasted
        time.sleep(max(0.0, t0 + FAULT_SEC - time.monotonic()))
        t_end = time.monotonic()
        window_end = self.fault_window_end(fault, t0, t_end)
        moved = [c for c in self.cmds_since(zero_t or t0)
                 if c[0] < window_end and (abs(c[1]) > EPS or abs(c[2]) > EPS)]
        self.end_fault(fault, cmd)
        if latched:
            self.set_estop(False)
            time.sleep(1.0)
            still = not [c for c in self.cmds_since(t_end) if abs(c[1]) > EPS]
            self.set_cand((0.0, 0.0))
            time.sleep(0.7)
            cleared, _ = self.clear()
            unlatched = bool(cleared) and self.wait_for(lambda: not self.latched(), 0.5)
            recovered = still and unlatched
        else:
            recovered = self.wait_for(lambda: self.moving_cmd(v), 2.5)
        self.set_cand((0.0, 0.0))
        latency = zero_t - t0 if zero_t else None
        row.update(moving_at_fault=moving and speed0 > 0.05, speed_at_fault=round(speed0, 3),
                   zero_latency_s=latency and round(latency, 4),
                   sim_stop_s=stop_t and round(stop_t - t0, 3),
                   latency_limit_s=limit, reason_named=reason_ok,
                   nonzero_during_fault=len(moved), recovered=bool(recovered))
        row['passed'] = bool(row['moving_at_fault'] and latency is not None and latency <= limit
                             and reason_ok and not moved and recovered)
        return row

    def run(self):
        ready = self.wait_for(lambda: self.safety is not None and self.truth_speed is not None
                              and self.clear_cli.service_is_ready()
                              and self.reset_cli.service_is_ready(), 20.0)
        if not ready:
            print('[FAIL] system not ready', flush=True)
            return False
        seed = 100
        for fault in FAULTS:
            for phase in PHASES:
                seed += 1
                row = self.trial(fault, phase, seed)
                self.trial_rows.append(row)
                print('[%s] %-15s %-9s zero %ss (limit %.2f) sim stop %ss reason %s '
                      'recovered %s' % ('PASS' if row['passed'] else 'FAIL', fault, phase,
                                        row.get('zero_latency_s'), FAULTS[fault][1],
                                        row.get('sim_stop_s'), row.get('reason_named'),
                                        row.get('recovered')), flush=True)
        with self.lock:
            dec = dict(self.decisions)
        print('[%s] decision records: %d seen, %d lost, %d latched with a nonzero command'
              % ('PASS' if dec['seen'] and not dec['latched_with_nonzero_command'] else 'FAIL',
                 dec['seen'], dec['seq_gaps'], dec['latched_with_nonzero_command']),
              flush=True)
        return all(r['passed'] for r in self.trial_rows) and dec['seen'] > 0 \
            and dec['latched_with_nonzero_command'] == 0

    def summary(self):
        out = {}
        for fault in FAULTS:
            rows = [r for r in self.trial_rows if r['fault'] == fault]
            lat = [r['zero_latency_s'] for r in rows if r.get('zero_latency_s') is not None]
            stop = [r['sim_stop_s'] for r in rows if r.get('sim_stop_s') is not None]
            out[fault] = {'trials': len(rows), 'passed': sum(r['passed'] for r in rows),
                          'zero_latency_max_s': max(lat) if lat else None,
                          'zero_latency_mean_s': round(sum(lat) / len(lat), 4) if lat else None,
                          'sim_stop_max_s': max(stop) if stop else None,
                          'latency_limit_s': FAULTS[fault][1]}
        return out

    def reset_summary(self):
        """Client-side /scenario/reset response times over all trials (7.1: 1 s)."""
        resets = [r['reset_ms'] for r in self.trial_rows if r.get('reset_ms') is not None]
        return {'trials': len(self.trial_rows), 'answered_in_time': len(resets),
                'response_max_ms': max(resets) if resets else None,
                'response_mean_ms': round(sum(resets) / len(resets), 1) if resets else None,
                'client_timeout_ms': RESET_CLIENT_TIMEOUT * 1000}

    def write_repeat(self, ok):
        os.makedirs(self.output_dir, exist_ok=True)
        with open(os.path.join(self.output_dir, 's9_repeat.json'), 'w') as fh:
            json.dump({'passed': ok, 'summary': self.summary(),
                       'reset_handshake': self.reset_summary(),
                       'safety_decisions': dict(self.decisions),
                       'trials': self.trial_rows,
                       'commit': os.environ.get('GIT_COMMIT', 'unknown')}, fh, indent=2)
        keys = ['fault', 'phase', 'seed', 'run_id', 'reset_ms', 'speed_at_fault',
                'zero_latency_s', 'latency_limit_s', 'sim_stop_s', 'reason_named',
                'nonzero_during_fault', 'recovered', 'passed']
        with open(os.path.join(self.output_dir, 's9_repeat.csv'), 'w', newline='') as fh:
            writer = csv.DictWriter(fh, keys, extrasaction='ignore')
            writer.writeheader()
            writer.writerows(self.trial_rows)
        n = len(self.trial_rows)
        print('S9 REPEAT %s: %d/%d trials passed; evidence %s/s9_repeat.json'
              % ('PASSED' if ok else 'FAILED', sum(r['passed'] for r in self.trial_rows), n,
                 self.output_dir), flush=True)


def main(args=None):
    rclpy.init(args=args)
    node = S9Repeat()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spinner = threading.Thread(target=executor.spin, daemon=True)
    spinner.start()
    ok = False
    try:
        ok = node.run()
    finally:
        node.set_cand(None)
        node.write_repeat(ok)
        executor.shutdown()
        spinner.join(timeout=5.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
