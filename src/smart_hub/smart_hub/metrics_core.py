"""Run metrics for metrics_logger (pure Python, no ROS).

Measured from the logger's point of view (assignment 21.2):

* stop-command latency: e-stop assertion seen -> first zero final command
  seen, per agent (includes DDS transport, so it is an upper bound on the
  supervisor's own receipt-to-output latency);
* physical stop time: e-stop assertion -> simulated speed below
  stopped_speed (reported separately, as 16.3 requires);
* motion while latched, measured twice (must be zero: "no silent
  continuation"):
  - from /hub/safety_decision, the supervisor's own record of each decision
    (latch state and commands in ONE message, numbered): every latched
    decision with a nonzero command counts, with no grace window; lost
    records show up as sequence gaps;
  - from /hub/safety and /robot|bus/cmd_vel, two topics DDS does not order:
    a nonzero command within ORDERING_GRACE_SEC of a latch change cannot be
    attributed, so it is counted and listed as *ambiguous* instead of being
    dropped; later ones count as violations;
* safety interventions grouped by reason, and every /hub/events row.

Times are seconds from time.monotonic(), supplied by the caller.
"""
import csv
import json
import os

EPS = 1e-6
ORDERING_GRACE_SEC = 0.1
EXAMPLES_KEPT = 50       # listed samples per run (counts beyond that are not kept)


class MetricsCore:
    def __init__(self, run_id, scenario_id='', seed=0, commit='unknown',
                 stopped_speed=0.02):
        self.meta = {'run_id': run_id, 'scenario_id': scenario_id, 'seed': seed,
                     'commit': commit}
        self.stopped_speed = stopped_speed
        self.events = []                 # rows for events.csv
        self.estop_asserted = False
        self.pending_cmd = {}            # agent -> assert time, until zero seen
        self.pending_phys = {}           # agent -> assert time, until stopped
        self.stop_latency = []           # dicts: agent, seconds
        self.physical_stop = []          # dicts: agent, seconds
        self.latched = False
        self.latch_changed_t = float('-inf')
        self.motion_while_latched = 0
        self.ambiguous_count = 0
        self.ambiguous_near_latch_change = []   # first EXAMPLES_KEPT samples
        self.decisions_seen = 0
        self.decision_last_seq = None
        self.decision_seq_gaps = 0       # records lost between two seen ones
        self.decision_restarts = 0       # seq went backwards (supervisor restarted)
        self.latched_nonzero_count = 0
        self.latched_nonzero_decisions = []     # first EXAMPLES_KEPT samples
        self.interventions = {}          # reason -> count
        self.first_t = None
        self.last_t = None

    def _seen(self, t):
        self.first_t = t if self.first_t is None else self.first_t
        self.last_t = t

    def on_event(self, t, entity, event, reason, correlation_id=''):
        self._seen(t)
        self.events.append({'t': round(t, 4), 'entity': entity, 'event': event,
                            'reason': reason, 'correlation_id': correlation_id})
        if event in ('SAFETY_HOLD', 'STOP_LATCHED', 'BRIDGE_WATCHDOG_STOP'):
            key = '%s:%s' % (event, reason)
            self.interventions[key] = self.interventions.get(key, 0) + 1

    def on_estop(self, t, asserted, agents=('robot', 'bus')):
        self._seen(t)
        if asserted and not self.estop_asserted:      # rising edge only
            for agent in agents:
                self.pending_cmd[agent] = t
                self.pending_phys[agent] = t
        self.estop_asserted = bool(asserted)

    def on_safety(self, t, stop_latched):
        self._seen(t)
        if bool(stop_latched) != self.latched:
            self.latch_changed_t = t
        self.latched = bool(stop_latched)

    def on_final_command(self, t, agent, vx, wz):
        self._seen(t)
        zero = abs(vx) < EPS and abs(wz) < EPS
        # DDS does not order messages across topics, so a command sent just
        # before/after a latch change may arrive on the "wrong" side of it.
        # Such a sample is kept as ambiguous, never silently dropped; the
        # unambiguous measure is on_decision().
        since_change = t - self.latch_changed_t
        if self.latched and not zero:
            if since_change > ORDERING_GRACE_SEC:
                self.motion_while_latched += 1
            else:
                self.ambiguous_count += 1
                if len(self.ambiguous_near_latch_change) < EXAMPLES_KEPT:
                    self.ambiguous_near_latch_change.append(
                        {'t': round(t, 4), 'agent': agent, 'vx': round(vx, 4),
                         'wz': round(wz, 4),
                         'dt_since_latch_change': round(since_change, 4)})
        if zero and agent in self.pending_cmd:
            start = self.pending_cmd.pop(agent)
            self.stop_latency.append({'agent': agent, 'seconds': round(t - start, 4)})

    def on_decision(self, t, seq, stop_latched, commands):
        """Count one supervisor decision record; commands = {agent: (vx, wz)}."""
        self._seen(t)
        self.decisions_seen += 1
        last = self.decision_last_seq
        if last is not None:
            if seq > last + 1:
                self.decision_seq_gaps += seq - last - 1
            elif seq <= last:
                self.decision_restarts += 1
        self.decision_last_seq = seq
        if not stop_latched:
            return
        for agent, (vx, wz) in sorted(commands.items()):
            if abs(vx) < EPS and abs(wz) < EPS:
                continue
            self.latched_nonzero_count += 1
            if len(self.latched_nonzero_decisions) < EXAMPLES_KEPT:
                self.latched_nonzero_decisions.append(
                    {'seq': seq, 't': round(t, 4), 'agent': agent,
                     'vx': round(vx, 4), 'wz': round(wz, 4)})

    def on_truth(self, t, agent, speed):
        self._seen(t)
        if agent in self.pending_phys and abs(speed) < self.stopped_speed:
            start = self.pending_phys.pop(agent)
            self.physical_stop.append({'agent': agent, 'seconds': round(t - start, 4)})

    def summary(self):
        lat = [x['seconds'] for x in self.stop_latency]
        return dict(self.meta,
                    duration_sec=round((self.last_t or 0) - (self.first_t or 0), 3),
                    event_count=len(self.events),
                    stop_latency=self.stop_latency,
                    stop_latency_max_sec=max(lat) if lat else None,
                    physical_stop=self.physical_stop,
                    motion_while_latched=self.motion_while_latched,
                    motion_while_latched_ambiguous={
                        'count': self.ambiguous_count,
                        'grace_sec': ORDERING_GRACE_SEC,
                        'samples': self.ambiguous_near_latch_change},
                    safety_decisions={
                        'seen': self.decisions_seen,
                        'seq_gaps': self.decision_seq_gaps,
                        'supervisor_restarts': self.decision_restarts,
                        'latched_with_nonzero_command': self.latched_nonzero_count,
                        'samples': self.latched_nonzero_decisions},
                    interventions=self.interventions)

    def write(self, directory):
        """Write events.csv and summary.json; return the summary."""
        os.makedirs(directory, exist_ok=True)
        with open(os.path.join(directory, 'events.csv'), 'w', newline='') as fh:
            writer = csv.DictWriter(fh, ['t', 'entity', 'event', 'reason', 'correlation_id'])
            writer.writeheader()
            writer.writerows(self.events)
        summary = self.summary()
        with open(os.path.join(directory, 'summary.json'), 'w') as fh:
            json.dump(summary, fh, indent=2)
        return summary
