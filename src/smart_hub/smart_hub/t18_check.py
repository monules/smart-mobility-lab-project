"""t18_check (M4 + M5): T18 planner loss on the live graph (assignment 16.2, 19.2).

The hub runs WITHOUT route_planner. This checker first leaves /route/compute
absent, then serves it itself with a configurable delay, and watches what the
real robot_manager and bus_manager do:

  1  planner absent        robot NavigateTo and bus DockBus each fail with
                           PLANNER_UNAVAILABLE after 1 s timeout + 2 retries,
                           no motion at all
  2  late replies (1.5 s)  the same failure; every late answer is ignored, the
                           returned path is never driven (no stale path)
  3  slow but in time      robot NavigateTo W succeeds with the 0.3 s answer
     (0.3 s)

Writes <output_dir>/t18_check.json. Exit code 0 only if every check passed.
"""
import json
import math
import os
import sys
import threading
import time

from geometry_msgs.msg import Point, TwistStamped
import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.task import Future
from smart_hub import geometry as geo
from smart_hub import route_core
from smart_hub.ros_common import declare, R10, R100
from smart_hub_interfaces.action import DockBus, NavigateTo
from smart_hub_interfaces.msg import HubEvent, VehicleStateArray
from smart_hub_interfaces.srv import ComputeRoute


class T18Check(Node):
    def __init__(self, **kwargs):
        super().__init__('t18_check', **kwargs)
        p = declare(self, {'output_dir': 'output'})
        self.output_dir = p['output_dir']
        self.lock = threading.Lock()
        self.delay = None             # None: service not offered
        self.received = []            # (t, agent, start, goal)
        self.events = []              # (t, entity, event, reason)
        self.cands = {'robot': [], 'bus': []}
        self.xy = {}
        self.results = []
        self.group = ReentrantCallbackGroup()
        self.service = None
        for agent in ('robot', 'bus'):
            self.create_subscription(TwistStamped, '/%s/cmd_vel_candidate' % agent,
                                     lambda m, a=agent: self.on_cand(a, m), R10)
        self.create_subscription(HubEvent, '/hub/events', self.on_event, R100)
        self.create_subscription(VehicleStateArray, '/sim/truth', self.on_truth, R10)
        self.nav = ActionClient(self, NavigateTo, '/robot/navigate_to')
        self.dock = ActionClient(self, DockBus, '/bus/dock')

    def on_cand(self, agent, msg):
        if msg.twist.linear.x != 0 or msg.twist.angular.z != 0:
            with self.lock:
                self.cands[agent].append(time.monotonic())

    def on_event(self, msg):
        with self.lock:
            self.events.append((time.monotonic(), msg.entity_id, msg.event, msg.reason))

    def on_truth(self, msg):
        for v in msg.vehicles:
            if v.vehicle_id in ('robot_1', 'bus_1'):
                self.xy[v.vehicle_id] = (v.pose.x, v.pose.y)

    # ------------------------------------------------- the stand-in planner
    async def on_compute(self, req, res):
        with self.lock:
            self.received.append((time.monotonic(), req.agent_type, req.start_node,
                                  req.goal_node))
        done = Future()
        timer = self.create_timer(self.delay, lambda: done.done() or done.set_result(True),
                                  callback_group=self.group)
        await done
        timer.cancel()
        self.destroy_timer(timer)
        view = route_core.EdgeView(require_traffic=False)
        view.on_bodies([], time.monotonic())
        view.on_crossing('', False, True, time.monotonic())
        nodes, cost = route_core.plan(view.costs(req.agent_type, time.monotonic()),
                                      req.start_node, req.goal_node)
        res.success, res.reason = nodes is not None, 'OK' if nodes else 'NO_ROUTE'
        if nodes:
            res.node_ids = nodes
            res.waypoints = [Point(x=float(geo.NODES[n][0]), y=float(geo.NODES[n][1]))
                             for n in nodes]
            res.estimated_cost_sec = float(cost)
        return res

    def offer(self, delay):
        self.delay = delay
        if self.service is None:
            self.service = self.create_service(ComputeRoute, '/route/compute',
                                               self.on_compute, callback_group=self.group)

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

    def run_goal(self, client, goal, timeout=20.0):
        """Send a goal; return (reason, seconds to the result)."""
        t0 = time.monotonic()
        fut = client.send_goal_async(goal)
        self.wait_for(fut.done, 5.0)
        handle = fut.result()
        if handle is None or not handle.accepted:
            return 'REJECTED', time.monotonic() - t0
        res = handle.get_result_async()
        self.wait_for(res.done, timeout)
        r = res.result()
        return (r.result.reason if r else 'TIMEOUT'), time.monotonic() - t0

    def trial(self, name, agent, client, goal, expect, settle=2.0):
        vid = '%s_1' % agent
        start_xy = self.xy.get(vid)
        with self.lock:
            n_ev, n_req = len(self.events), len(self.received)
        t0 = time.monotonic()
        reason, took = self.run_goal(client, goal)
        time.sleep(settle)                                   # late answers arrive
        with self.lock:
            moving = [t for t in self.cands[agent] if t >= t0]
            evs = [e for e in self.events[n_ev:] if e[1] == vid]
            req = self.received[n_req:]
        moved = math.dist(start_xy, self.xy[vid]) if start_xy else -1
        retries = [e for e in evs if e[2] in ('PLANNER_RETRY', 'PLANNER_NOT_READY')]
        late = [e for e in evs if e[2] == 'ROUTE_LATE_IGNORED']
        ok = reason == expect and not moving and 0 <= moved < 0.02 and 2.9 <= took <= 4.5
        return ok, ('%s: %s after %.2f s; %d request(s) reached the planner, %d retry '
                    'event(s), %d late answer(s) ignored; %d nonzero candidates, moved '
                    '%.3f m' % (name, reason, took, len(req), len(retries), len(late),
                                len(moving), moved)), (req, late)

    # -------------------------------------------------------------- phases
    def run(self):
        if not (self.nav.wait_for_server(timeout_sec=20.0)
                and self.dock.wait_for_server(timeout_sec=10.0)):
            return self.check('T18 setup', False, 'robot_manager or bus_manager missing')
        time.sleep(3.0)
        nav = NavigateTo.Goal(goal_node='W', max_duration_sec=60.0)
        dock = DockBus.Goal(bus_id='bus_1', bay_id='B', max_duration_sec=90.0)

        # 1. absent
        nav.request_id = 't18-absent'
        ok, detail, _ = self.trial('robot, planner absent', 'robot', self.nav, nav,
                                   'PLANNER_UNAVAILABLE')
        self.check('T18 robot: planner absent', ok, detail)
        ok, detail, _ = self.trial('bus, planner absent', 'bus', self.dock, dock,
                                   'PLANNER_UNAVAILABLE')
        self.check('T18 bus: planner absent', ok, detail)

        # 2. late replies
        self.offer(1.5)
        time.sleep(1.0)
        nav.request_id = 't18-late'
        ok, detail, (req, late) = self.trial('robot, replies after 1.5 s', 'robot', self.nav,
                                             nav, 'PLANNER_UNAVAILABLE')
        self.check('T18 robot: late replies ignored', ok and len(req) == 3 and late, detail)
        ok, detail, (req, _) = self.trial('bus, replies after 1.5 s', 'bus', self.dock, dock,
                                          'PLANNER_UNAVAILABLE')
        self.check('T18 bus: late replies ignored', ok and len(req) == 3, detail)

        # 3. slow but in time: bounded retry does not break normal work
        self.offer(0.3)
        nav.request_id = 't18-in-time'
        reason, took = self.run_goal(self.nav, nav, timeout=40.0)
        return self.check('T18 robot: slow planner still works', reason == 'ARRIVED',
                          'replies after 0.3 s: NavigateTo W %s after %.1f s' % (reason, took))

    def write(self):
        ok = bool(self.results) and all(r['passed'] for r in self.results)
        os.makedirs(self.output_dir, exist_ok=True)
        with open(os.path.join(self.output_dir, 't18_check.json'), 'w') as fh:
            json.dump({'passed': ok, 'checks': self.results,
                       'commit': os.environ.get('GIT_COMMIT', 'unknown')}, fh, indent=2)
        print('T18 %s; evidence: %s' % ('PASSED' if ok else 'FAILED', self.output_dir),
              flush=True)
        return ok


def main(args=None):
    rclpy.init(args=args)
    node = T18Check()
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
