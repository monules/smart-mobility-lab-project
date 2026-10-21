"""s4_check (M5 for M3): runs the S4 parking test against the live ROS graph.

It plays the cars (C1, C2, ...) that do not exist yet in this repository: it
calls /parking/reserve_slot and /parking/release_slot like a car client, and
injects the parking-sensor changes that depend on its own timing through
/scenario/inject (test only). It checks what the real parking_manager does.

Sequence (assignment 11.3, tests T07 and T08):
  1. wait for the sensors: P1 OCCUPIED, P2 and P3 FREE
  2. T07: C1 and C2 ask for P2 at the same time -> different slots
  3. C1 repeats its request id -> the same reservation (idempotent)
  4. release with a wrong token / wrong vehicle -> rejected, slot kept
  5. C1 arrives: its slot is sensed occupied -> OCCUPIED
  6. P3's sensor goes offline (scenario t=10) -> P3 UNKNOWN
  7. T08: reservations expire -> an occupied or stale slot never turns FREE
  8. the lot is full -> a request fails with a reason, nothing is granted
  9. T07: 20 synchronized request pairs -> zero double allocations,
     95th percentile service latency below 250 ms

Results: printed table + <output_dir>/s4_check.json. Exit code 0 only if
every check passed.
"""
import json
import os
import sys
import threading
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from smart_hub.ros_common import declare, R10, R100, R1T
from smart_hub_interfaces.msg import HubEvent, ParkingState
from smart_hub_interfaces.srv import ReleaseParking, ReserveParking
from std_msgs.msg import String

P95_LIMIT = 0.250         # s, assignment 11.3
TTL = 15.0                # s; long enough to outlive the P3 outage at t=10
PAIRS = 20


def p95(values):
    """95th percentile, nearest-rank method."""
    ordered = sorted(values)
    return ordered[max(0, int(0.95 * len(ordered) + 0.999999) - 1)]


class S4Check(Node):
    def __init__(self, **kwargs):
        super().__init__('s4_check', **kwargs)
        p = declare(self, {'output_dir': 'output', 'pairs': PAIRS})
        self.output_dir, self.pairs = p['output_dir'], int(p['pairs'])
        self.lock = threading.Lock()
        self.state = {}               # slot -> (state, vehicle_id, reservation_id)
        self.offline_t = None
        self.expired = []             # slot ids of RESERVATION_EXPIRED events
        self.results = []
        self.inject_pub = self.create_publisher(String, '/scenario/inject', R10)
        self.create_subscription(ParkingState, '/parking/state', self.on_state, R1T)
        self.create_subscription(HubEvent, '/hub/events', self.on_event, R100)
        self.reserve_cli = self.create_client(ReserveParking, '/parking/reserve_slot')
        self.release_cli = self.create_client(ReleaseParking, '/parking/release_slot')

    # ----------------------------------------------------------- callbacks
    def on_state(self, msg):
        with self.lock:
            self.state = {s.slot_id: (s.state, s.vehicle_id, s.reservation_id)
                          for s in msg.slots}

    def on_event(self, msg):
        with self.lock:
            if msg.event == 'SCENARIO_EVENT' and '"offline"' in msg.reason:
                self.offline_t = time.monotonic()
            elif msg.event == 'RESERVATION_EXPIRED':
                self.expired.append(msg.entity_id)

    # ------------------------------------------------------------- helpers
    def slots(self):
        with self.lock:
            return {k: v[0] for k, v in self.state.items()}

    def wait_for(self, cond, timeout):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if cond():
                return True
            time.sleep(0.02)
        return cond()

    def sense(self, slot, state):
        self.inject_pub.publish(String(data=json.dumps(
            {'type': 'parking', 'slot': slot, 'state': state})))

    def send_reserve(self, request_id, vehicle, preferred='P2', ttl=TTL):
        req = ReserveParking.Request(request_id=request_id, vehicle_id=vehicle,
                                     preferred_slot=preferred, ttl_sec=float(ttl))
        return time.monotonic(), self.reserve_cli.call_async(req)

    def send_release(self, request_id, token, vehicle):
        req = ReleaseParking.Request(request_id=request_id, reservation_id=token,
                                     vehicle_id=vehicle)
        return time.monotonic(), self.release_cli.call_async(req)

    def result(self, sent, timeout=2.0):
        """Wait for a call_async future; return (response, latency s)."""
        t0, future = sent
        end = t0 + timeout
        while not future.done():
            if time.monotonic() > end:
                return None, None
            time.sleep(0.001)
        return future.result(), time.monotonic() - t0

    def check(self, name, passed, detail):
        self.results.append({'check': name, 'passed': bool(passed), 'detail': detail})
        print('[%s] %s: %s' % ('PASS' if passed else 'FAIL', name, detail), flush=True)
        return passed

    # ------------------------------------------------------------ sequence
    def run(self):
        if not (self.reserve_cli.wait_for_service(timeout_sec=15.0)
                and self.release_cli.wait_for_service(timeout_sec=5.0)):
            return self.check('S4 services', False, 'parking services not available')
        start = {'P1': 'OCCUPIED', 'P2': 'FREE', 'P3': 'FREE'}
        ok = self.wait_for(lambda: self.slots() == start, 8.0)
        if not self.check('S4 sensors ready', ok, 'slots %s (want %s)' % (self.slots(), start)):
            return False

        # 2. two cars ask for the same slot at the same moment
        a = self.send_reserve('C1-r1', 'C1')
        b = self.send_reserve('C2-r1', 'C2')
        (r1, _), (r2, _) = self.result(a), self.result(b)
        if r1 is None or r2 is None:
            return self.check('T07 concurrent requests', False, 'no service response')
        granted = sorted(s for s in (r1.slot_id, r2.slot_id) if s)
        self.check('T07 concurrent requests',
                   r1.success and r2.success and granted == ['P2', 'P3']
                   and r1.reservation_id != r2.reservation_id,
                   'C1 -> %s, C2 -> %s (both preferred P2; one P2, the other the next '
                   'free slot)' % (r1.slot_id or r1.reason, r2.slot_id or r2.reason))
        c1_slot, c1_token = r1.slot_id, r1.reservation_id

        # 3. retry with the same request id: same answer, no second slot
        again, _ = self.result(self.send_reserve('C1-r1', 'C1'))
        self.check('S4 idempotent retry',
                   again is not None and again.success
                   and again.reservation_id == c1_token and again.slot_id == c1_slot,
                   'repeat of C1-r1 -> %s (first answer %s)'
                   % (again and (again.slot_id, again.reservation_id[:8]),
                      (c1_slot, c1_token[:8])))

        # 4. wrong token and wrong vehicle cannot release C1's slot
        bad1, _ = self.result(self.send_release('X-rel1', 'not-a-token', 'C1'))
        bad2, _ = self.result(self.send_release('X-rel2', c1_token, 'C2'))
        kept = self.wait_for(lambda: self.slots().get(c1_slot) == 'RESERVED', 1.0)
        self.check('S4 wrong-token release rejected',
                   bad1 is not None and not bad1.success and bad1.reason == 'UNKNOWN_TOKEN'
                   and bad2 is not None and not bad2.success
                   and bad2.reason == 'VEHICLE_MISMATCH' and kept,
                   'bogus token -> %s, C2 with C1 token -> %s, %s still %s'
                   % (bad1 and bad1.reason, bad2 and bad2.reason, c1_slot,
                      self.slots().get(c1_slot)))

        # 5. C1 parks: the sensor sees its slot occupied
        t0 = time.monotonic()
        self.sense(c1_slot, 'occupied')
        ok = self.wait_for(lambda: self.slots().get(c1_slot) == 'OCCUPIED', 3.0)
        self.check('S4 arrival sensed', ok, '%s OCCUPIED %.2f s after C1 parked '
                   '(sensor debounce 0.5 s)' % (c1_slot, time.monotonic() - t0))

        # 6. P3's sensor goes offline at scenario t=10
        ok = self.wait_for(lambda: self.offline_t is not None, 15.0)
        ok = ok and self.wait_for(lambda: self.slots().get('P3') == 'UNKNOWN', 2.0)
        delay = time.monotonic() - self.offline_t if self.offline_t else None
        self.check('S4 offline sensor -> UNKNOWN', ok,
                   'P3 %s %.2f s after its sensor went offline'
                   % (self.slots().get('P3'), delay or -1))

        # 7. both reservations expire; the sensed state must survive
        ok = self.wait_for(lambda: len(self.expired) >= 2, TTL + 5.0)
        time.sleep(0.6)                  # one more state publication
        after = self.slots()
        want = {'P1': 'OCCUPIED', c1_slot: 'OCCUPIED' if c1_slot != 'P3' else 'UNKNOWN',
                'P3': 'UNKNOWN'}
        other = ({'P2', 'P3'} - {c1_slot}).pop()
        if other != 'P3':
            want[other] = 'FREE'         # reserved but empty and sensed free
        self.check('T08 expiry keeps sensed state', ok and after == want,
                   'expired %s; after expiry %s (want %s)'
                   % (sorted(self.expired), after, want))

        # 8. lot full: every slot occupied or unknown -> request fails with a reason
        for slot, state in after.items():
            if state == 'FREE':
                self.sense(slot, 'occupied')
        self.wait_for(lambda: 'FREE' not in self.slots().values(), 3.0)
        full, _ = self.result(self.send_reserve('C3-r1', 'C3', preferred=''))
        self.check('S4 full lot rejected',
                   full is not None and not full.success and full.reason == 'FULL'
                   and not full.reservation_id,
                   'C3 with no free slot -> success=%s reason=%s'
                   % (full and full.success, full and full.reason))

        # 9. synchronized pairs on an empty lot
        for slot in ('P1', 'P2', 'P3'):
            self.sense(slot, 'free')
        ok = self.wait_for(lambda: set(self.slots().values()) == {'FREE'}, 4.0)
        if not self.check('S4 lot emptied', ok, 'slots %s' % self.slots()):
            return False
        return self.pair_test()

    def pair_test(self):
        latencies, doubles, failures = [], 0, []
        for i in range(self.pairs):
            va, vb = 'PA%02d' % i, 'PB%02d' % i
            sent = [self.send_reserve('%s-r' % va, va), self.send_reserve('%s-r' % vb, vb)]
            answers = [self.result(s) for s in sent]
            if any(resp is None for resp, _ in answers):
                failures.append('pair %d: no response' % i)
                continue
            latencies += [lat for _, lat in answers]
            slots = [resp.slot_id for resp, _ in answers if resp.success]
            if len(slots) != 2:
                failures.append('pair %d: %s' % (i, [r.reason for r, _ in answers]))
            if len(set(slots)) != len(slots):
                doubles += 1
            for (resp, _), vehicle in zip(answers, (va, vb)):
                if resp.success:
                    self.result(self.send_release('%s-rel' % vehicle,
                                                  resp.reservation_id, vehicle))
        self.check('T07 no double allocation', doubles == 0 and not failures,
                   '%d pairs, %d double allocations, failures %s'
                   % (self.pairs, doubles, failures[:3]))
        worst = p95(latencies) if latencies else None
        return self.check('T07 p95 latency', worst is not None and worst < P95_LIMIT,
                          'p95 %s ms over %d requests (limit %d ms), max %s ms'
                          % (worst and round(worst * 1000, 1), len(latencies),
                             P95_LIMIT * 1000,
                             latencies and round(max(latencies) * 1000, 1)))

    def write(self):
        ok = bool(self.results) and all(r['passed'] for r in self.results)
        os.makedirs(self.output_dir, exist_ok=True)
        path = os.path.join(self.output_dir, 's4_check.json')
        with open(path, 'w') as fh:
            json.dump({'passed': ok, 'checks': self.results,
                       'commit': os.environ.get('GIT_COMMIT', 'unknown')}, fh, indent=2)
        print('S4 %s; evidence: %s' % ('PASSED' if ok else 'FAILED', path), flush=True)
        return ok


def main(args=None):
    rclpy.init(args=args)
    node = S4Check()
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
