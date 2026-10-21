"""s6_check (M4, written by M5): runs the S6 routing tests on the live graph.

It calls the real /route/compute (route_planner) like a car client while the
real traffic_monitor and simulator provide the edge states, injects the road
changes through /scenario/inject (test only), and drives robot_1 through
/robot/navigate_to for the manager side of T12 (assignment 13.3, T11, T12).

Steps:
  1  empty roads: car A->E takes A-Cs-J-E; costs 10 m and 14 m at equal speed
  2  obstacle O on Cs-J: route through the upper road, no removed edge used
  3  three slow cars on Cs-J (dense, not blocked): the cheapest route by the
     13.2 cost, which is not the geometrically shortest one
  4  O and O2 (Cs-J and Cn-B): car NO_ROUTE; robot goal to E fails with
     NO_ROUTE and zero candidate motion (T12)
  5  roads restored: a new route is computed and validated before the robot
     moves again
  T11 (live)  every answer equals an exhaustive simple-path oracle over the
              planner's own edge costs (relative 1e-6), valid edges only
  latency     p95 of all service calls below 250 ms

Writes <output_dir>/s6_check.json, s6_routes.csv and s6_maps.png (edge costs
and chosen path before/after each change). Exit code 0 only if every check
passed.
"""
import csv
import json
import math
import os
import sys
import threading
import time

from geometry_msgs.msg import TwistStamped
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from smart_hub import geometry as geo
from smart_hub import route_core
from smart_hub.ros_common import declare, R10, R100
from smart_hub_interfaces.action import NavigateTo
from smart_hub_interfaces.msg import HubEvent, VehicleStateArray
from smart_hub_interfaces.srv import ComputeRoute
from std_msgs.msg import String

P95_LIMIT = 0.250
LOW, UP = ['A', 'Cs', 'J', 'E'], ['A', 'N', 'Cn', 'B', 'U', 'E']


def oracle(costs, start, goal):
    """Minimum cost over every simple path (independent of Dijkstra)."""
    adj = {}
    for eid, c in costs.items():
        if c is not None:
            a, b = eid.split('-')
            adj.setdefault(a, []).append((b, c))
            adj.setdefault(b, []).append((a, c))
    best = [math.inf]

    def dfs(node, seen, cost):
        if node == goal:
            best[0] = min(best[0], cost)
            return
        for nxt, c in adj.get(node, []):
            if nxt not in seen:
                dfs(nxt, seen | {nxt}, cost + c)
    dfs(start, {start}, 0.0)
    return best[0]


class S6Check(Node):
    def __init__(self, **kwargs):
        super().__init__('s6_check', **kwargs)
        p = declare(self, {'output_dir': 'output', 'burst_calls': 50})
        self.output_dir, self.burst = p['output_dir'], int(p['burst_calls'])
        self.lock = threading.Lock()
        self.computed = []        # ROUTE_COMPUTED event payloads, in arrival order
        self.robot_events = []
        self.cands = []           # (t, vx, wz) robot candidates
        self.robot_xy = None
        self.calls = []           # rows for s6_routes.csv
        self.snap = {}            # step -> (costs, nodes) for the map figure
        self.results = []
        self.inject_pub = self.create_publisher(String, '/scenario/inject', R10)
        self.create_subscription(HubEvent, '/hub/events', self.on_event, R100)
        self.create_subscription(TwistStamped, '/robot/cmd_vel_candidate', self.on_cand, R10)
        self.create_subscription(VehicleStateArray, '/sim/truth', self.on_truth, R10)
        self.route_cli = self.create_client(ComputeRoute, '/route/compute')
        self.nav = ActionClient(self, NavigateTo, '/robot/navigate_to')

    def on_event(self, msg):
        with self.lock:
            if msg.event == 'ROUTE_COMPUTED':
                self.computed.append(json.loads(msg.reason))
            elif msg.entity_id == 'robot_1':
                self.robot_events.append((time.monotonic(), msg.event, msg.reason))

    def on_cand(self, msg):
        with self.lock:
            self.cands.append((time.monotonic(), msg.twist.linear.x, msg.twist.angular.z))

    def on_truth(self, msg):
        for v in msg.vehicles:
            if v.vehicle_id == 'robot_1':
                self.robot_xy = (v.pose.x, v.pose.y)

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

    def inject(self, **event):
        self.inject_pub.publish(String(data=json.dumps(event)))

    def route(self, step, agent='car', start='A', goal='E'):
        """Call /route/compute; return (response, latency s, event payload)."""
        with self.lock:
            n = len(self.computed)
        t0 = time.monotonic()
        fut = self.route_cli.call_async(ComputeRoute.Request(
            start_node=start, goal_node=goal, agent_type=agent))
        self.wait_for(fut.done, 2.0)
        latency = time.monotonic() - t0
        res = fut.result() if fut.done() else None
        self.wait_for(lambda: len(self.computed) > n, 1.0)
        with self.lock:
            payload = self.computed[n] if len(self.computed) > n else None
        self.calls.append({'step': step, 'agent': agent, 'start': start, 'goal': goal,
                           'success': res.success if res else None,
                           'reason': res.reason if res else 'NO_RESPONSE',
                           'nodes': '-'.join(res.node_ids) if res else '',
                           'cost_s': round(res.estimated_cost_sec, 3) if res else '',
                           'latency_ms': round(latency * 1000, 2),
                           'removed': json.dumps(payload['removed']) if payload else ''})
        return res, latency, payload

    def until_route(self, step, cond, timeout=8.0):
        """Ask every 0.3 s until cond(response) holds (edge states settle)."""
        end = time.monotonic() + timeout
        while True:
            res, lat, payload = self.route(step)
            if (res is not None and cond(res)) or time.monotonic() > end:
                return res, payload
            time.sleep(0.3)

    def oracle_ok(self, res, payload, agent='car', start='A', goal='E'):
        """Live T11: Dijkstra answer == exhaustive oracle over the same costs."""
        if payload is None:
            return False, 'no ROUTE_COMPUTED event'
        costs = payload['costs']
        want = oracle(costs, start, goal)
        if not res.success:
            return math.isinf(want), 'oracle %s' % ('no route' if math.isinf(want) else want)
        edges = route_core.edges_of(list(res.node_ids))
        valid = res.node_ids[0] == start and res.node_ids[-1] == goal and all(
            e in route_core.ALLOWED[agent] and costs.get(e) is not None for e in edges)
        same = abs(res.estimated_cost_sec - want) <= 1e-6 * max(1.0, want) + 1e-4
        return valid and same, 'planner %.4f s, oracle %.4f s, edges valid %s' % (
            res.estimated_cost_sec, want, valid)

    def navigate(self, goal, request_id, timeout):
        handle = self.nav.send_goal_async(NavigateTo.Goal(
            request_id=request_id, goal_node=goal, max_duration_sec=timeout))
        self.wait_for(handle.done, 5.0)
        h = handle.result()
        if h is None or not h.accepted:
            return 'REJECTED', None
        res = h.get_result_async()
        self.wait_for(res.done, timeout + 5)
        r = res.result()
        return (r.result.reason if r else 'TIMEOUT'), (r.result if r else None)

    # --------------------------------------------------------------- steps
    def run(self):
        if not (self.route_cli.wait_for_service(timeout_sec=20.0)
                and self.nav.wait_for_server(timeout_sec=10.0)):
            return self.check('S6 setup', False, 'planner or robot_manager not available')
        live = []

        # 1. empty roads
        res, payload = self.until_route(1, lambda r: r.success and list(r.node_ids) == LOW
                                        and abs(r.estimated_cost_sec - 25.0) < 1e-3)
        costs = payload['costs'] if payload else {}
        up = sum(costs.get(e) or math.inf for e in route_core.edges_of(UP))
        self.snap['1 empty roads'] = (costs, list(res.node_ids) if res else [])
        self.check('S6 baseline costs', res and list(res.node_ids) == LOW
                   and abs(res.estimated_cost_sec - 25.0) < 1e-3 and abs(up - 35.0) < 1e-3,
                   'car A->E %s, %.3f s; lower road 10 m / 0.40 m/s = 25 s, upper detour '
                   '14 m = %.3f s (zero density, zero crossing wait)'
                   % (res and '-'.join(res.node_ids), res.estimated_cost_sec if res else -1, up))
        live.append((res, payload))

        # 2. obstacle O on Cs-J
        self.inject(type='obstacle', id='O', active=True)
        res, payload = self.until_route(2, lambda r: r.success and list(r.node_ids) == UP)
        costs = payload['costs'] if payload else {}
        used_removed = [e for e in route_core.edges_of(list(res.node_ids))
                        if costs.get(e) is None] if res else ['?']
        self.snap['2 obstacle O on Cs-J'] = (costs, list(res.node_ids) if res else [])
        self.check('S6 obstacle reroute', res and list(res.node_ids) == UP and not used_removed,
                   'route %s, Cs-J removed because %s; returned edges checked against the '
                   'removed set: %s' % (
                       res and '-'.join(res.node_ids),
                       payload and payload['removed'].get('Cs-J'),
                       used_removed or 'none used'))
        live.append((res, payload))
        self.inject(type='obstacle', id='O', active=False)
        self.until_route(2, lambda r: r.success and list(r.node_ids) == LOW)

        # 3. dense slow traffic on Cs-J (outside C so the crossing stays free)
        for i, x in enumerate((6.0, 7.0, 8.0)):
            self.inject(type='vehicle', id='slow_%d' % i, kind='car', x=x, y=2.0,
                        cruise=0.10, loop_x=[5.9, 8.6])
        res, payload = self.until_route(3, lambda r: r.success and list(r.node_ids) != LOW,
                                        timeout=10.0)
        costs = payload['costs'] if payload else {}
        low = sum(costs.get(e) or math.inf for e in route_core.edges_of(LOW))
        ok, detail = self.oracle_ok(res, payload) if res else (False, 'no answer')
        self.snap['3 slow dense traffic on Cs-J'] = (costs, list(res.node_ids) if res else [])
        self.check('S6 congestion by cost', res and list(res.node_ids) == UP and ok,
                   'route %s (14 m) chosen over the 10 m lower road: lower road %.1f s vs '
                   '%.1f s; %s' % (res and '-'.join(res.node_ids), low,
                                   res.estimated_cost_sec if res else -1, detail))
        live.append((res, payload))
        for i in range(3):
            self.inject(type='vehicle', id='slow_%d' % i, active=False)

        # 4. O and O2: no route; the robot must not move
        self.inject(type='obstacle', id='O', active=True)
        self.inject(type='obstacle', id='O2', active=True)
        res, payload = self.until_route(4, lambda r: not r.success)
        self.snap['4 O and O2: no route'] = (payload['costs'] if payload else {}, [])
        self.check('T12 car no route', res and not res.success and res.reason == 'NO_ROUTE',
                   'car A->E with Cs-J and Cn-B blocked: success=%s reason=%s'
                   % (res and res.success, res and res.reason))
        live.append((res, payload))
        start_xy = self.robot_xy
        t0 = time.monotonic()
        reason, result = self.navigate('E', 's6-nav-noroute', 30.0)
        with self.lock:
            moving = [c for c in self.cands if c[0] >= t0 and (c[1] != 0 or c[2] != 0)]
        moved = math.dist(start_xy, self.robot_xy) if start_xy and self.robot_xy else -1
        self.check('T12 robot stops on no route', reason == 'NO_ROUTE' and not moving
                   and 0 <= moved < 0.02,
                   'robot_1 NavigateTo E: result %s after %.2f s, %d nonzero candidates, '
                   'moved %.3f m' % (reason, time.monotonic() - t0, len(moving), moved))

        # 5. restore: new validation before movement
        self.inject(type='obstacle', id='O', active=False)
        self.inject(type='obstacle', id='O2', active=False)
        res, payload = self.until_route(5, lambda r: r.success and list(r.node_ids) == LOW)
        self.snap['5 roads restored'] = (payload['costs'] if payload else {},
                                         list(res.node_ids) if res else [])
        live.append((res, payload))
        with self.lock:
            n_ev = len(self.robot_events)
        reason, result = self.navigate('W', 's6-nav-restored', 40.0)
        with self.lock:
            evs = list(self.robot_events[n_ev:])
        planned = [e for e in evs if e[1] == 'ROUTE']
        with self.lock:
            moving = [c for c in self.cands if c[1] > 0 and planned and c[0] >= planned[0][0]]
        self.check('S6 restore needs a new route', res and list(res.node_ids) == LOW
                   and reason == 'ARRIVED' and planned and moving,
                   'car route back to %s; robot NavigateTo W: route %s validated, then %s '
                   '(final error %.3f m)' % (res and '-'.join(res.node_ids),
                                             planned and planned[0][2], reason,
                                             result.final_error_m if result else -1))

        # live T11 on every configuration above
        oks = [self.oracle_ok(r, pl) for r, pl in live if r is not None]
        self.check('T11 live oracle', len(oks) == len(live) and all(o for o, _ in oks),
                   '%d/%d live answers equal the exhaustive simple-path oracle: %s'
                   % (sum(o for o, _ in oks), len(live), '; '.join(d for _, d in oks)))

        # latency
        for i in range(self.burst):
            self.route('latency', *(('robot', 'R', 'D') if i % 2 else ('car', 'A', 'E')))
        lat = sorted(c['latency_ms'] for c in self.calls)
        p95 = lat[max(0, math.ceil(0.95 * len(lat)) - 1)]
        return self.check('S6 planning latency', p95 < P95_LIMIT * 1000,
                          'p95 %.1f ms over %d calls (limit %d ms), max %.1f ms'
                          % (p95, len(lat), P95_LIMIT * 1000, lat[-1]))

    # ------------------------------------------------------------ evidence
    def write(self):
        ok = bool(self.results) and all(r['passed'] for r in self.results)
        out = self.output_dir
        os.makedirs(out, exist_ok=True)
        with open(os.path.join(out, 's6_check.json'), 'w') as fh:
            json.dump({'passed': ok, 'checks': self.results,
                       'commit': os.environ.get('GIT_COMMIT', 'unknown')}, fh, indent=2)
        if self.calls:
            with open(os.path.join(out, 's6_routes.csv'), 'w', newline='') as fh:
                w = csv.DictWriter(fh, fieldnames=list(self.calls[0]))
                w.writeheader()
                w.writerows(self.calls)
        if self.snap:
            plot_maps(self.snap, os.path.join(out, 's6_maps.png'))
        print('S6 %s; evidence: %s' % ('PASSED' if ok else 'FAILED', out), flush=True)
        return ok


def plot_maps(snap, path):
    """One panel per step: car edge costs (s), removed edges dashed, chosen path."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(len(snap), 1, figsize=(7, 3.0 * len(snap)))
    for ax, (title, (costs, nodes)) in zip(axes, snap.items()):
        chosen = set(route_core.edges_of(nodes))
        for eid in sorted(route_core.ALLOWED['car']):
            (ax_, ay), (bx, by) = geo.EDGES[eid]
            c = costs.get(eid)
            style = {'color': 'red', 'linestyle': '--', 'linewidth': 1} if c is None else \
                {'color': 'tab:blue' if eid in chosen else 'grey',
                 'linewidth': 3.5 if eid in chosen else 1}
            ax.plot([ax_, bx], [ay, by], **style)
            ax.annotate('removed' if c is None else '%.0f s' % c,
                        ((ax_ + bx) / 2, (ay + by) / 2), fontsize=7, ha='center',
                        color='red' if c is None else 'black')
        for name, (x, y) in geo.NODES.items():
            if name not in ('R', 'W', 'D'):
                ax.plot(x, y, 'k.', markersize=4)
                ax.annotate(name, (x, y), textcoords='offset points', xytext=(3, 3),
                            fontsize=7)
        x0, y0, x1, y1 = geo.CROSSING
        ax.add_patch(matplotlib.patches.Rectangle((x0, y0), x1 - x0, y1 - y0,
                                                  color='orange', alpha=0.2))
        ax.set_title('%s: %s' % (title, '-'.join(nodes) if nodes else 'NO_ROUTE'),
                     fontsize=9)
        ax.set_xlim(0, 12)
        ax.set_ylim(1, 7.2)
        ax.set_aspect('equal')
        ax.tick_params(labelsize=7)
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)


def main(args=None):
    rclpy.init(args=args)
    node = S6Check()
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
