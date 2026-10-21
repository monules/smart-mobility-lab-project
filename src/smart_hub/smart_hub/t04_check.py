"""t04_check (M2-B, written by M5): T04 dispatch test on the live ROS graph.

It plays the forecaster and the queue sensor: it publishes /hub/demand
(1 Hz, like demand_predictor) and /hub/queue (5 Hz, like passenger_detector)
with exactly the values each phase needs, and watches what the real
hub_coordinator does with the real bus_manager (assignment 9.3, 9.4, T04).

Phases (the scenario is reset before each, so the bus starts at A):
  1  noise around 6 (6.2, 5.8, 6.1, 5.9, ...)   -> no dispatch (never 3 in a row)
  2  rising demand, held >= 6 until boarding,     -> exactly one dispatch
     then 5 (between 4 and 6) while people board     (PREDICTED), no second goal
                                                     while it runs, and none after
                                                     it (escalated, but below 6)
  3  forecast invalid (valid=false, predicted 9)   -> no dispatch from an invalid
     with a low observed queue                       forecast
  4  forecast feed gap (no /hub/demand at all)     -> no dispatch
     with a low observed queue
  5  no forecast, observed queue 7                 -> one dispatch, reason FALLBACK

Writes <output_dir>/t04_check.json and t04_timeline.csv (signal sent,
coordinator events, bus phases). Exit code 0 only if every check passed.
"""
import csv
import json
import os
import sys
import threading
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from smart_hub.ros_common import declare, R10, R100
from smart_hub_interfaces.msg import DemandPrediction, HubEvent, QueueState
from smart_hub_interfaces.srv import ResetScenario

ENDED = ('BUS_MISSION_SUCCEEDED', 'BUS_MISSION_ABORTED', 'BUS_MISSION_CANCELED',
         'BUS_MISSION_ENDED')


class T04Check(Node):
    def __init__(self, **kwargs):
        super().__init__('t04_check', **kwargs)
        p = declare(self, {'output_dir': 'output', 'mission_timeout': 90.0})
        self.output_dir = p['output_dir']
        self.mission_timeout = float(p['mission_timeout'])
        self.lock = threading.Lock()
        self.phase = 0
        self.demand = None        # (predicted, valid) or None = feed gap
        self.queue = 2            # observed count (valid, nobody in C)
        self.sent = []            # (t, phase, predicted, valid, queue)
        self.events = []          # (t, phase, entity, event, reason, correlation)
        self.results = []
        self.demand_pub = self.create_publisher(DemandPrediction, '/hub/demand', R10)
        self.queue_pub = self.create_publisher(QueueState, '/hub/queue', R10)
        self.create_subscription(HubEvent, '/hub/events', self.on_event, R100)
        self.reset_cli = self.create_client(ResetScenario, '/scenario/reset')
        self.create_timer(1.0, self.publish_demand)
        self.create_timer(0.2, self.publish_queue)

    # ----------------------------------------------------------- publishers
    def publish_demand(self):
        with self.lock:
            demand, phase, queue = self.demand, self.phase, self.queue
        if demand is None:
            return                                   # feed gap
        msg = DemandPrediction()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.horizon_sec, msg.model_id = 30, 't04_check'
        msg.predicted_count = float(demand[0]) if demand[1] else -1.0
        msg.baseline_count = float(queue) if demand[1] else -1.0
        msg.valid = bool(demand[1])
        self.demand_pub.publish(msg)
        with self.lock:
            self.sent.append((time.monotonic(), phase, demand[0], demand[1], queue))

    def publish_queue(self):
        with self.lock:
            queue = self.queue
        msg = QueueState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.zone_id, msg.count, msg.confidence = 'Q', int(queue), 1.0
        msg.valid, msg.crossing_occupied = True, False
        self.queue_pub.publish(msg)

    def on_event(self, msg):
        if msg.event in ('RESET_ACK', 'SCENARIO_RESET_REQUEST', 'SCENARIO_EVENT'):
            return
        with self.lock:
            self.events.append((time.monotonic(), self.phase, msg.entity_id, msg.event,
                                msg.reason, msg.correlation_id))

    # ------------------------------------------------------------- helpers
    def set_signal(self, demand, queue=None):
        with self.lock:
            self.demand = demand
            if queue is not None:
                self.queue = queue

    def check(self, name, passed, detail):
        self.results.append({'check': name, 'passed': bool(passed), 'detail': detail})
        print('[%s] %s: %s' % ('PASS' if passed else 'FAIL', name, detail), flush=True)
        return passed

    def wait_for(self, cond, timeout):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if cond():
                return True
            time.sleep(0.05)
        return cond()

    def of(self, phase, *names):
        with self.lock:
            return [e for e in self.events if e[1] == phase and (not names or e[3] in names)]

    def boarding(self, phase):
        return lambda: any('->BOARD' in e[4] for e in self.of(phase, 'BUS_PHASE'))

    def start_phase(self, phase, demand, queue):
        """Low signal, reset the world (bus back at A), then start the phase."""
        self.set_signal((2.0, True), 2)
        fut = self.reset_cli.call_async(ResetScenario.Request(
            scenario_id='S3', seed=phase, preserve_logs=True))
        self.wait_for(fut.done, 10.0)
        time.sleep(2.0)                              # bus_arrival IDLE again
        with self.lock:
            self.phase = phase
        self.set_signal(demand, queue)
        return fut.done() and fut.result().success

    # -------------------------------------------------------------- phases
    def run(self):
        if not self.reset_cli.wait_for_service(timeout_sec=20.0):
            return self.check('T04 setup', False, '/scenario/reset not available')
        time.sleep(3.0)                              # coordinator sees fresh inputs

        # 1. noise around the threshold: never three samples >= 6 in a row
        self.start_phase(1, (6.2, True), 2)
        end = time.monotonic() + 20.0
        values = [6.2, 5.8, 6.1, 5.9, 6.3, 5.7]
        i = 0
        while time.monotonic() < end:
            self.set_signal((values[i % len(values)], True))
            i += 1
            time.sleep(1.0)
        n = len(self.of(1, 'BUS_DISPATCH_REQUESTED'))
        self.check('T04 threshold noise', n == 0,
                   '%d dispatches in 20 s of forecasts alternating around 6 (%s)'
                   % (n, values))

        # 2. rising demand, held high for the whole mission
        self.start_phase(2, (3.0, True), 2)
        for v in (4.0, 5.0, 6.5, 7.0):
            self.set_signal((v, True))
            time.sleep(1.0)
        self.set_signal((7.5, True))
        self.wait_for(self.boarding(2), self.mission_timeout)
        self.set_signal((5.0, True))                 # people board: demand falls to 5
        ended = self.wait_for(lambda: self.of(2, *ENDED), self.mission_timeout)
        time.sleep(10.0)                             # bus IDLE again, signal 5
        requested = self.of(2, 'BUS_DISPATCH_REQUESTED')
        accepted = self.of(2, 'DOCK_GOAL_ACCEPTED')
        busy = [e for e in self.of(2, 'DOCK_GOAL_REJECTED') if e[4] == 'BUSY']
        result = [e[3] for e in self.of(2, *ENDED)]
        self.check('T04 one dispatch per mission',
                   ended and len(requested) == 1 and len(accepted) == 1 and not busy
                   and requested[0][4] == 'PREDICTED' and result == ['BUS_MISSION_SUCCEEDED'],
                   'forecast >= 6 until boarding, then 5 for 10 s after the mission: '
                   '%d dispatch request(s) (%s), %d goal(s) accepted by bus_manager, '
                   '%d BUSY rejections, result %s'
                   % (len(requested), ','.join(e[4] for e in requested), len(accepted),
                      len(busy), result))
        if requested:
            first_high = next(t for t, ph, v, ok, _ in self.sent if ph == 2 and ok and v >= 6)
            delay = requested[0][0] - first_high
            self.check('T04 three consecutive forecasts', 1.5 <= delay <= 4.0,
                       'dispatch %.1f s after the first forecast >= 6 (3 samples at 1 Hz)'
                       % delay)
        self.set_signal((2.0, True))                 # clear escalation (5 samples < 4)
        time.sleep(6.0)

        # 3. invalid forecast, low observed queue
        self.start_phase(3, (9.0, False), 2)
        time.sleep(15.0)
        n = len(self.of(3, 'BUS_DISPATCH_REQUESTED'))
        self.check('T04 no dispatch from invalid forecast', n == 0,
                   '%d dispatches in 15 s of valid=false forecasts (predicted 9) with '
                   'an observed queue of 2' % n)

        # 4. forecast feed gap, low observed queue
        self.start_phase(4, None, 2)
        time.sleep(15.0)
        n = len(self.of(4, 'BUS_DISPATCH_REQUESTED'))
        self.check('T04 feed gap', n == 0,
                   '%d dispatches in 15 s without any /hub/demand, observed queue 2' % n)

        # 5. no forecast, observed queue 7 -> FALLBACK
        self.start_phase(5, None, 7)
        self.wait_for(self.boarding(5), self.mission_timeout)
        self.set_signal(None, 3)                     # people board: queue falls to 3
        ended = self.wait_for(lambda: self.of(5, *ENDED), self.mission_timeout)
        requested = self.of(5, 'BUS_DISPATCH_REQUESTED')
        result = [e[3] for e in self.of(5, *ENDED)]
        self.check('T04 explicit fallback',
                   ended and len(requested) == 1 and requested[0][4] == 'FALLBACK'
                   and result == ['BUS_MISSION_SUCCEEDED'],
                   'no forecast, observed queue 7: %d dispatch(es) with reason %s, result %s'
                   % (len(requested), ','.join(e[4] for e in requested), result))
        self.set_signal(None, 2)
        return all(r['passed'] for r in self.results)

    def write(self):
        ok = bool(self.results) and all(r['passed'] for r in self.results)
        os.makedirs(self.output_dir, exist_ok=True)
        with open(os.path.join(self.output_dir, 't04_check.json'), 'w') as fh:
            json.dump({'passed': ok, 'checks': self.results,
                       'commit': os.environ.get('GIT_COMMIT', 'unknown')}, fh, indent=2)
        with self.lock:
            sent, events = list(self.sent), list(self.events)
        t0 = min([s[0] for s in sent] + [e[0] for e in events] or [0.0])
        rows = [(t, ph, 't04_check', 'DEMAND_SENT',
                 'predicted=%.1f valid=%s queue=%d' % (v, ok_, q), '')
                for t, ph, v, ok_, q in sent]
        rows += [(t, ph, ent, ev, rs, corr) for t, ph, ent, ev, rs, corr in events]
        with open(os.path.join(self.output_dir, 't04_timeline.csv'), 'w', newline='') as fh:
            w = csv.writer(fh)
            w.writerow(['t', 'phase', 'source', 'event', 'detail', 'correlation_id'])
            for t, ph, ent, ev, rs, corr in sorted(rows):
                w.writerow(['%.3f' % (t - t0), ph, ent, ev, rs, corr])
        print('T04 %s; evidence: %s' % ('PASSED' if ok else 'FAILED', self.output_dir),
              flush=True)
        return ok


def main(args=None):
    rclpy.init(args=args)
    node = T04Check()
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
