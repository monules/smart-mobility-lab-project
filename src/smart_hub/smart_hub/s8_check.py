"""s8_check (M5 for M2-B / team): judges one S8 rush-hour episode (15.1-15.3).

Everything else in the hub runs for real; this node only provides the two
parking clients of the script (cars C1 and C2 ask for parking at the same
moment at t = 65 s) and observes. Outcome checks are conditional on what it
observes (15.1: "make outcome assertions conditional on observed state").

Safety (T15, counted separately, any violation fails the episode):
  collisions             bodies overlapping, or a vehicle within reach of a
                         pedestrian (ground truth)
  crossing ownership     owner changes X -> Y without being released, two
                         mobile agents inside C at once, an agent entering C
                         without owning it, or entering C while a pedestrian
                         is in it
  double allocation      two cars holding the same parking slot
  duplicate dispatch     a bus dispatch requested while a mission is active

Metrics (per episode, compared across seeds and dispatch modes by
s8_summary): completion of the bus mission, the escort and the parking
requests; queue wait p50/p95 (appearing in Q -> boarding); bus and robot
travel times; crossing waiting time; dispatch time and source; forecast
validity time.

Writes <output_dir>/s8_episode.json and s8_timeline.csv. Exit code 0 when
the episode has no safety violation (missions may fail; that is reported).
"""
import csv
import json
import math
import os
import sys
import threading
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from smart_hub import geometry as geo
from smart_hub.ros_common import declare, R10, R100, R1T
from smart_hub_interfaces.msg import (CrossingState, HubEvent, ParkingState,
                                      VehicleStateArray)
from smart_hub_interfaces.srv import ReserveParking

PED_RADIUS = 0.15
QUEUE = (2.0, 5.0, 6.0, 7.0)
MOBILE = ('bus', 'robot', 'car')


def in_c(x, y, r):
    return geo.circle_overlaps_rect(x, y, r, geo.CROSSING)


def pct(values, q):
    if not values:
        return None
    s = sorted(values)
    return s[max(0, math.ceil(q * len(s)) - 1)]


class S8Check(Node):
    def __init__(self, **kwargs):
        super().__init__('s8_check', **kwargs)
        p = declare(self, {'output_dir': 'output', 'seed': 0, 'mode': 'predictive',
                           'timeout_sec': 300.0})
        self.output_dir, self.seed, self.mode = p['output_dir'], int(p['seed']), p['mode']
        self.deadline = time.monotonic() + float(p['timeout_sec'])
        self.lock = threading.Lock()
        self.t0 = None              # monotonic time of scenario t = 0
        self.complete = None
        self.truth = []             # (t, {id: (kind, x, y)})
        self.owners = []            # (t, owner, pedestrian, valid)
        self.events = []            # (t, entity, event, reason)
        self.parking = []           # (t, {slot: (state, vehicle, reservation)})
        self.reserve = []           # (vehicle, success, slot, reason, latency)
        self.create_subscription(VehicleStateArray, '/sim/truth', self.on_truth, R10)
        self.create_subscription(CrossingState, '/crossing/state', self.on_crossing, R1T)
        self.create_subscription(HubEvent, '/hub/events', self.on_event, R100)
        self.create_subscription(ParkingState, '/parking/state', self.on_parking, R1T)
        self.reserve_cli = self.create_client(ReserveParking, '/parking/reserve_slot')

    # ----------------------------------------------------------- callbacks
    def now(self):
        return time.monotonic() - (self.t0 or time.monotonic())

    def on_truth(self, msg):
        bodies = {v.vehicle_id: (v.kind, v.pose.x, v.pose.y) for v in msg.vehicles}
        with self.lock:
            self.truth.append((self.now(), bodies))

    def on_crossing(self, msg):
        with self.lock:
            if not self.owners or self.owners[-1][1:] != (msg.owner_id,
                                                          msg.pedestrian_present, msg.valid):
                self.owners.append((self.now(), msg.owner_id, msg.pedestrian_present,
                                    msg.valid))

    def on_parking(self, msg):
        with self.lock:
            self.parking.append((self.now(), {s.slot_id: (s.state, s.vehicle_id,
                                                          s.reservation_id)
                                              for s in msg.slots}))

    def on_event(self, msg):
        t = time.monotonic()
        if msg.event == 'SCENARIO_EVENT':
            ev = json.loads(msg.reason)
            if ev.get('type') == 'note' and 'start' in ev.get('text', '') and self.t0 is None:
                self.t0 = t - float(ev['t'])
            if ev.get('type') == 'note' and 'parking' in ev.get('text', ''):
                threading.Thread(target=self.request_parking, daemon=True).start()
        elif msg.event == 'SCENARIO_COMPLETE':
            self.complete = t
        if msg.event in ('ROUTE_COMPUTED',):
            return
        with self.lock:
            self.events.append((self.now(), msg.entity_id, msg.event, msg.reason))

    def request_parking(self):
        """Cars C1 and C2 ask for P2 at the same moment (15.1, t = 65 s)."""
        if not self.reserve_cli.wait_for_service(timeout_sec=2.0):
            self.reserve.append(('C1', False, '', 'NO_SERVICE', -1))
            return
        sent = []
        for car in ('C1', 'C2'):
            req = ReserveParking.Request(request_id='S8-%d-%s' % (self.seed, car),
                                         vehicle_id=car, preferred_slot='P2', ttl_sec=60.0)
            sent.append((car, time.monotonic(), self.reserve_cli.call_async(req)))
        for car, t, fut in sent:
            end = t + 3.0
            while not fut.done() and time.monotonic() < end:
                time.sleep(0.005)
            res = fut.result() if fut.done() else None
            self.reserve.append((car, bool(res and res.success), res.slot_id if res else '',
                                 res.reason if res else 'TIMEOUT',
                                 round(time.monotonic() - t, 4)))

    def done(self):
        return (self.complete is not None and time.monotonic() > self.complete + 1.0) or \
            time.monotonic() > self.deadline

    # ------------------------------------------------------------ analysis
    def ev(self, name, entity=None):
        return [e for e in self.events if e[2] == name and (entity is None or e[1] == entity)]

    def evaluate(self):
        with self.lock:
            truth, owners, events = list(self.truth), list(self.owners), list(self.events)
            parking = list(self.parking)
        v = {'collisions': [], 'crossing': [], 'double_allocation': [],
             'duplicate_dispatch': []}

        # collisions and crossing occupancy from ground truth
        prev_in = {}
        for t, bodies in truth:
            mob = {i: b for i, b in bodies.items() if b[0] in MOBILE}
            peds = [b for b in bodies.values() if b[0] == 'pedestrian']
            solid = {i: b for i, b in bodies.items() if b[0] in MOBILE + ('obstacle',)}
            ids = sorted(solid)
            for a in range(len(ids)):
                for b in range(a + 1, len(ids)):
                    ka, xa, ya = solid[ids[a]]
                    kb, xb, yb = solid[ids[b]]
                    if math.hypot(xa - xb, ya - yb) < geo.RADIUS_BY_KIND.get(ka, 0.3) + \
                            geo.RADIUS_BY_KIND.get(kb, 0.3):
                        v['collisions'].append((round(t, 2), ids[a], ids[b]))
            for i, (k, x, y) in mob.items():
                for _, px, py in peds:
                    if math.hypot(x - px, y - py) < geo.RADIUS_BY_KIND.get(k, 0.3) + PED_RADIUS:
                        v['collisions'].append((round(t, 2), i, 'pedestrian'))
            inside = [i for i, (k, x, y) in mob.items()
                      if in_c(x, y, geo.RADIUS_BY_KIND.get(k, 0.3))]
            if len(inside) > 1:
                v['crossing'].append((round(t, 2), 'two agents in C', inside))
            ped_in_c = any(in_c(px, py, 0.0) for _, px, py in peds)
            for i in inside:
                if not prev_in.get(i):                       # entering C now
                    owner = next((o for ot, o, _, _ in reversed(owners) if ot <= t), '')
                    if owner != i:
                        v['crossing'].append((round(t, 2), 'entered without owning', i, owner))
                    if ped_in_c:
                        v['crossing'].append((round(t, 2), 'entered with a pedestrian', i))
            prev_in = {i: i in inside for i in mob}
        last = ''
        for t, owner, _, _ in owners:
            if owner and last and owner != last:
                v['crossing'].append((round(t, 2), 'owner changed without release', last,
                                      owner))
            last = owner

        # parking: one holder per slot, one slot per car
        for t, slots in parking:
            holders = [s[1] for s in slots.values() if s[1]]
            if len(holders) != len(set(holders)):
                v['double_allocation'].append((round(t, 2), slots))
        granted = [r[2] for r in self.reserve if r[1]]
        if len(granted) != len(set(granted)):
            v['double_allocation'].append(('responses', self.reserve))

        # dispatch: never a second request while a mission is active
        active = False
        for t, _, name, reason in events:
            if name == 'BUS_DISPATCH_REQUESTED':
                if active:
                    v['duplicate_dispatch'].append((round(t, 2), reason))
                active = True
            elif name in ('BUS_MISSION_SUCCEEDED', 'BUS_MISSION_ABORTED',
                          'BUS_MISSION_CANCELED', 'BUS_MISSION_ENDED',
                          'BUS_DISPATCH_FAILED', 'BUS_DISPATCH_REJECTED'):
                active = False

        # metrics
        m = {}
        appear, boarded = {}, {}
        for t, bodies in truth:
            for i, (k, x, y) in bodies.items():
                if k == 'pedestrian' and QUEUE[0] <= x <= QUEUE[2] and \
                        QUEUE[1] <= y <= QUEUE[3]:
                    appear.setdefault(i, t)
        for t, ent, name, _ in events:
            if name == 'PASSENGER_BOARDED':
                boarded[ent] = t
        waits = [boarded[i] - appear[i] for i in boarded if i in appear]
        m['passengers_seen'] = len(appear)
        m['passengers_boarded'] = len(waits)
        m['queue_wait_p50_s'] = pct(waits, 0.5)
        m['queue_wait_p95_s'] = pct(waits, 0.95)
        m['queue_waits_s'] = sorted(round(w, 2) for w in waits)
        disp = self.ev('BUS_DISPATCH_REQUESTED')
        m['dispatches'] = len(disp)
        m['dispatch_t_s'] = round(disp[0][0], 2) if disp else None
        m['dispatch_source'] = disp[0][3] if disp else ''
        valid = self.ev('DEMAND_VALID')
        m['forecast_valid_t_s'] = round(valid[0][0], 2) if valid else None
        m['bus_mission'] = 'SUCCEEDED' if self.ev('BUS_MISSION_SUCCEEDED') else \
            ('FAILED' if disp else 'NOT_DISPATCHED')
        acc, docked = self.ev('DOCK_GOAL_ACCEPTED'), self.ev('BUS_DOCKED')
        m['bus_travel_s'] = round(docked[0][0] - acc[0][0], 2) if acc and docked else None
        m['bus_departed'] = bool(self.ev('BUS_DEPARTED'))
        res = [e for e in self.ev('ROBOT_RESULT') if e[3].startswith('escort')]
        m['robot_mission'] = 'SUCCEEDED' if res and ' success ' in res[0][3] else \
            (res[0][3] if res else 'NO_RESULT')
        ready, arrived = self.ev('PASSENGER_READY'), self.ev('ROBOT_ARRIVED')
        m['robot_travel_s'] = round(arrived[0][0] - ready[0][0], 2) if ready and arrived \
            else None
        m['robot_reroutes'] = [e[3] for e in self.ev('REROUTE')]
        m['parking'] = [list(r) for r in self.reserve]
        m['parking_answered'] = len(self.reserve) == 2 and all(r[3] != 'TIMEOUT'
                                                               for r in self.reserve)
        m['crossing_wait_s'] = round(self.waiting(events), 2)
        m['crossing_grants'] = [(round(t, 1), o) for t, o, _, _ in owners if o]
        m['completed'] = m['bus_mission'] == 'SUCCEEDED' and \
            m['robot_mission'] == 'SUCCEEDED' and m['parking_answered']
        return v, m

    def waiting(self, events):
        """Seconds the bus spent in WAIT and the robot in WAIT_CROSSING."""
        total, since = 0.0, {}
        for t, ent, name, reason in events:
            if name not in ('BUS_PHASE', 'ROBOT_PHASE'):
                continue
            new = reason.split('->')[1].split(' ')[0] if '->' in reason else ''
            if ent in since:
                total += t - since.pop(ent)
            if new in ('WAIT', 'WAIT_CROSSING'):
                since[ent] = t
        return total

    def write(self):
        violations, metrics = self.evaluate()
        n = sum(len(x) for x in violations.values())
        os.makedirs(self.output_dir, exist_ok=True)
        out = {'passed': n == 0, 'seed': self.seed, 'mode': self.mode,
               'safety_violations': {k: len(x) for k, x in violations.items()},
               'violation_details': {k: [str(i) for i in x[:10]] for k, x in violations.items()},
               'metrics': metrics, 'commit': os.environ.get('GIT_COMMIT', 'unknown')}
        with open(os.path.join(self.output_dir, 's8_episode.json'), 'w') as fh:
            json.dump(out, fh, indent=2)
        with open(os.path.join(self.output_dir, 's8_timeline.csv'), 'w', newline='') as fh:
            w = csv.writer(fh)
            w.writerow(['t', 'source', 'event', 'detail'])
            for t, ent, name, reason in self.events:
                w.writerow(['%.2f' % t, ent, name, reason])
            for t, owner, ped, valid in self.owners:
                w.writerow(['%.2f' % t, 'crossing_state', 'OWNER', 'owner=%s pedestrian=%s '
                            'valid=%s' % (owner or '-', ped, valid)])
        print('S8 seed %d %s: %s, %d safety violations %s; %s' % (
            self.seed, self.mode, 'PASSED' if n == 0 else 'FAILED', n,
            out['safety_violations'], json.dumps(metrics)), flush=True)
        return n == 0


def main(args=None):
    rclpy.init(args=args)
    node = S8Check()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spinner = threading.Thread(target=executor.spin, daemon=True)
    spinner.start()
    ok = False
    try:
        while not node.done():
            time.sleep(0.2)
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
