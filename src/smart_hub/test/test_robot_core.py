"""RobotCore (M4, S7) in closed loop with the M5 simulator and route_core, no ROS.

The harness plays the rest of the hub: it feeds odometry and obstacles,
answers route requests with route_core.plan (or stays silent / answers late,
for T18), grants the crossing like the coordinator and applies the robot
command to the simulator.
"""
import math

import pytest
from smart_hub import geometry as geo
from smart_hub import route_core as rc
from smart_hub.robot_core import (ARRIVED, BLOCKED, CANCELLED, ESCORT, FAILED, NAVIGATE,
                                  RobotConfig, RobotCore, SAFE_STOP, WAIT_CROSSING,
                                  WAIT_READY)
from smart_hub.sim_core import DT, HubSim


class Harness:
    def __init__(self, planner='ok', deny_crossing=False):
        self.sim = HubSim({'agents': {'robot_1': {'kind': 'robot', 'x': 3.0, 'y': 6.0}},
                           'obstacles': {'O': {'x': 7.0, 'y': 2.0, 'active': False}}},
                          seed=1)
        self.view = rc.EdgeView()
        self.core = RobotCore(view=self.view)
        self.planner = planner            # 'ok', 'silent' or ('late', delay s)
        self.deny = deny_crossing
        self.pending = []                 # (due time, attempt, start, goal)
        self.ops, self.trace, self.outs = [], [], []
        self.feed(odom=True)

    @property
    def robot(self):
        return self.sim.bodies['robot_1']

    def feed(self, odom=True):
        t = self.sim.t
        b = self.robot
        if odom:
            self.core.on_odom(b.x, b.y, b.theta, abs(b.v), t)
        self.view.on_traffic([(e, 0.40, 0.0, False, True) for e in geo.VEHICLE_EDGES], t)
        self.view.on_bodies([(o.x, o.y, 0.35) for o in self.sim.bodies.values()
                             if o.kind == 'obstacle' and o.active], t)
        self.view.on_crossing('robot_1' if self.core.token else '', False, True, t)

    def answer(self, attempt, start, goal, now):
        nodes, cost = rc.plan(self.view.costs('robot', now, agent_id='robot_1'), start, goal)
        self.core.on_route(attempt, nodes is not None, nodes or [], cost,
                           'OK' if nodes else 'NO_ROUTE', now)

    def step(self, odom=True):
        t = self.sim.t
        self.feed(odom)
        out = self.core.step(t)
        self.outs.append((t, out))
        if out.route_request:
            attempt, start, goal = out.route_request
            if self.planner == 'ok':
                self.answer(attempt, start, goal, t)
            elif isinstance(self.planner, tuple):
                self.pending.append((t + self.planner[1], attempt, start, goal))
        for due, attempt, start, goal in [p for p in self.pending if p[0] <= t]:
            self.pending.remove((due, attempt, start, goal))
            self.answer(attempt, start, goal, t)
        if out.crossing_op:
            op, _ = out.crossing_op
            granted = not (self.deny and op == 'ACQUIRE')
            self.ops.append((t, op, granted))
            self.core.on_crossing_response(op, granted, 'tok' if granted else '',
                                           'OK' if granted else 'PEDESTRIAN_PRESENT', t)
        self.sim.set_command('robot_1', *out.cmd)
        self.sim.step()
        b = self.robot
        self.trace.append((t, b.x, b.y, abs(b.v), out.phase,
                           geo.circle_overlaps_rect(b.x, b.y, 0.20, geo.CROSSING)))
        return out

    def run(self, until, stop_when_done=True, ready_at=None):
        while self.sim.t < until:
            if ready_at is not None and self.core.phase == WAIT_READY \
                    and self.sim.t >= ready_at:
                self.core.confirm_ready()
            out = self.step()
            if stop_when_done and out.done:
                return out
        return None

    def escort(self, max_duration=240.0):
        self.core.start('escort', ['R', 'D'], self.sim.t, max_duration)


def moved(h):
    return max(math.dist((x, y), (3.0, 6.0)) for _, x, y, *_ in h.trace)


def test_nominal_escort_waits_for_ready_then_arrives_at_d():
    h = Harness()
    h.escort()
    h.run(2.0, stop_when_done=False)
    assert h.core.phase == WAIT_READY and moved(h) < 0.01
    out = h.run(200.0, ready_at=3.0)
    assert out is not None and out.done[:3] == (True, ARRIVED, 'ARRIVED')
    assert out.done[3] == 'D' and out.done[4] <= 0.20
    assert math.dist((h.robot.x, h.robot.y), geo.NODES['D']) <= 0.20
    assert ('ROUTE', 'R-W-Cn-Cs-J-D') in h.core.events
    in_c = [t for t, *_, c in h.trace if c]
    acquire = [t for t, op, ok in h.ops if op == 'ACQUIRE' and ok]
    release = [t for t, op, ok in h.ops if op == 'RELEASE']
    assert acquire and acquire[0] < in_c[0] and release and release[-1] > in_c[-1]
    assert max(v for _, _, _, v, *_ in h.trace) <= 0.25 + 1e-9


def test_ready_is_only_accepted_while_waiting_at_pickup():
    h = Harness()
    assert h.core.confirm_ready()[0] is False                 # idle
    h.escort()
    h.step()
    assert h.core.confirm_ready()[0] is True
    h.run(30.0, stop_when_done=False)
    assert h.core.phase == ESCORT
    assert h.core.confirm_ready()[0] is False                 # already escorting


def test_no_readiness_times_out_without_moving():
    h = Harness()
    h.escort()
    out = h.run(30.0)
    assert out.done[:3] == (False, FAILED, 'READY_TIMEOUT')
    assert 20.0 <= h.sim.t <= 21.0 and moved(h) < 0.01


def test_obstacle_during_escort_reroutes_through_the_upper_road():
    h = Harness()
    h.escort()
    while not (h.core.phase == ESCORT and h.robot.x > 3.5):
        if h.core.phase == WAIT_READY:
            h.core.confirm_ready()
        h.step()
    before = h.outs[-1][1].distance_remaining
    mark = len(h.outs)
    h.sim.bodies['O'].active = True                           # O appears on Cs-J
    events = []
    out = None
    while h.sim.t < 250 and out is None:
        o = h.step()
        events += h.core.events
        h.core.events = []
        out = o if o.done else None
    assert out.done[:2] == (True, ARRIVED)
    reroutes = [r for e, r in events if e == 'REROUTE']
    assert reroutes and 'BLOCKED:Cs-J' in reroutes[0] and 'W-Cn-B-U-E-J-D' in reroutes[0]
    after = max(o.distance_remaining for _, o in h.outs[mark:mark + 40])
    assert after > before + 3.0                               # remaining follows the detour
    gap = min(math.dist((x, y), (7.0, 2.0)) for _, x, y, *_ in h.trace)
    assert gap > 0.35 + 0.20


def test_obstacle_on_the_current_edge_turns_back_to_a_safe_node():
    h = Harness()
    h.escort()
    while h.robot.x < 6.2 or abs(h.robot.y - 2.0) > 0.05:    # on Cs-J, past C
        if h.core.phase == WAIT_READY:
            h.core.confirm_ready()
        h.step()
    h.sim.bodies['O'].active = True
    out = h.run(300.0)
    assert out is not None and out.done[:2] == (True, ARRIVED)
    assert any(e == 'REROUTE' and 'BLOCKED:Cs-J' in r and 'Cs-Cn-B' in r
               for e, r in h.core.events)
    # it went back through C only with the permission
    granted = [t for t, op, ok in h.ops if op == 'ACQUIRE' and ok]
    assert len(granted) >= 2


def test_cancel_stops_at_once_and_releases_outside_c():
    h = Harness()
    h.escort()
    h.step()
    assert h.core.confirm_ready()[0]
    while h.robot.x < 4.0:
        h.step()
    h.core.cancel()
    out = h.step()
    assert out.cmd == (0.0, 0.0) and out.done[:3] == (False, CANCELLED, 'CANCELED')
    assert h.step().cmd == (0.0, 0.0)


def test_busy_and_unknown_odometry_are_rejected():
    h = Harness()
    h.escort()
    with pytest.raises(RuntimeError, match='BUSY'):
        h.core.start('navigate', ['W'], h.sim.t, 60.0)
    core = RobotCore()
    with pytest.raises(RuntimeError, match='ODOM_UNKNOWN'):
        core.start('navigate', ['W'], 0.0, 60.0)


def test_unreachable_destination_fails_with_no_route_and_no_motion():
    h = Harness()
    h.core.start('navigate', ['P1'], h.sim.t, 60.0)
    out = h.run(5.0)
    assert out.done[:3] == (False, FAILED, 'NO_ROUTE')
    assert moved(h) < 0.01 and all(o.cmd == (0.0, 0.0) for _, o in h.outs)


def test_stale_odometry_zero_then_bounded_safe_stop():
    h = Harness()
    h.core.start('navigate', ['W'], h.sim.t, 60.0)
    h.run(2.0, stop_when_done=False)
    assert h.outs[-1][1].cmd[0] > 0
    outs = []
    for _ in range(int(3.0 / DT)):
        o = h.step(odom=False)
        outs.append(o)
        if o.done:
            break
    late = outs[int(0.5 / DT) + 1:]                           # odometry older than 0.5 s
    assert late and all(o.cmd == (0.0, 0.0) for o in late)
    assert outs[-1].done[:3] == (False, SAFE_STOP, 'ODOM_STALE')
    assert len(outs) * DT <= 0.5 + 2.0 + 2 * DT


def test_planner_absent_retries_twice_then_fails_without_moving():
    """T18: timeout 1 s, at most two retries, explicit failure."""
    h = Harness(planner='silent')
    h.core.start('navigate', ['D'], h.sim.t, 60.0)
    out = h.run(10.0)
    assert out.done[:3] == (False, FAILED, 'PLANNER_UNAVAILABLE')
    assert 3.0 <= h.sim.t <= 3.3
    assert sum(1 for _, o in h.outs if o.route_request) == 3
    assert moved(h) < 0.01


def test_late_planner_answers_are_ignored():
    """T18: answers after the 1 s timeout never start motion."""
    h = Harness(planner=('late', 1.5))
    h.core.start('navigate', ['W'], h.sim.t, 60.0)
    out = h.run(10.0)
    assert out.done[:3] == (False, FAILED, 'PLANNER_UNAVAILABLE')
    h.run(12.0, stop_when_done=False)                         # the last late answer
    assert moved(h) < 0.01
    assert any(e == 'ROUTE_LATE_IGNORED' for e, _ in h.core.events)


def test_crossing_denied_waits_at_the_stop_line():
    h = Harness(deny_crossing=True)
    h.core.start('navigate', ['D'], h.sim.t, 200.0)
    h.run(40.0, stop_when_done=False)
    assert h.core.phase == WAIT_CROSSING and not any(c for *_, c in h.trace)
    h.deny = False
    out = h.run(150.0)
    assert out.done[:2] == (True, ARRIVED)


def test_navigate_progress_and_feedback_fields():
    h = Harness()
    h.core.start('navigate', ['W'], h.sim.t, 60.0)
    out = h.run(40.0)
    assert out.done[:2] == (True, ARRIVED) and out.done[3] == 'W'
    prog = [o.progress for _, o in h.outs if o.phase == NAVIGATE]
    assert prog[0] < 0.1 and max(prog) > 0.9
    assert all(o.distance_remaining >= 0 for _, o in h.outs
               if o.phase == NAVIGATE and o.reason != 'PLANNING')


def test_blocked_pause_for_an_obstacle_beside_the_path():
    h = Harness()
    # 0.65 m from R-W: the planner keeps the edge (needs < 0.60 m to remove it),
    # the robot's own 0.15 m clearance rule must still pause it
    h.sim.apply_event({'type': 'obstacle', 'id': 'X', 'x': 4.0, 'y': 6.65})
    h.core.start('navigate', ['W'], h.sim.t, 60.0)
    h.run(8.0, stop_when_done=False)
    assert h.core.phase == BLOCKED and h.core.reason == 'OBSTACLE_CLEARANCE'
    h.sim.bodies['X'].active = False
    out = h.run(60.0)
    assert out.done[:2] == (True, ARRIVED)


def test_config_validation():
    with pytest.raises(ValueError):
        RobotCore(RobotConfig(cruise_speed=0.3))
    with pytest.raises(ValueError):
        RobotCore(RobotConfig(replan_gain=1.5))
