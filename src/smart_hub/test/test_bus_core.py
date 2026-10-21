"""BusCore (M2-A, S3) in closed loop with the M5 simulator, no ROS.

A small harness stands in for the rest of the hub: it feeds the simulated bus
pose to the core, applies the core's command to the simulator, answers the
crossing requests like the coordinator (grant unless told to deny) and opens
the door like the actuator bridge (only after the bus is still for 0.5 s).
"""
import math

import pytest
from smart_hub import geometry as geo
from smart_hub.bus_core import (APPROACH, BOARD, BusConfig, BusCore, COMPLETE, DOCK,
                                node_route, Path, SAFE_STOP, WAIT)
from smart_hub.sim_core import DT, HubSim


class Harness:
    def __init__(self, deny=False, bay_car=False, cfg=None):
        agents = {'bus_1': {'kind': 'bus', 'x': 1.0, 'y': 2.0, 'theta': math.pi / 2}}
        if bay_car:
            agents['car_9'] = {'kind': 'car', 'x': 8.0, 'y': 4.0}
        self.sim = HubSim({'agents': agents}, seed=1)
        self.core = BusCore(cfg)
        self.deny = deny
        self.ops = []                 # (t, operation, granted)
        self.trace = []               # (t, x, y, v, phase, door_open, in_c)
        self.door, self.still_since = False, None
        self.outs = []
        self.feed()

    @property
    def bus(self):
        return self.sim.bodies['bus_1']

    def feed(self):
        b, t = self.bus, self.sim.t
        self.core.on_pose(b.x, b.y, b.theta, abs(b.v), t)
        self.core.on_others([(o.vehicle_id, o.x, o.y) for o in self.sim.bodies.values()
                             if o.vehicle_id != 'bus_1'])
        self.core.on_door(self.door, t)

    def step(self, feed_pose=True):
        t = self.sim.t
        if feed_pose:
            self.feed()
        else:
            self.core.on_door(self.door, t)
            self.core.on_others([])
        out = self.core.step(t)
        self.outs.append((t, out))
        if out.crossing_op:
            op, _ = out.crossing_op
            granted = not (self.deny and op == 'ACQUIRE')
            self.ops.append((t, op, granted))
            self.core.on_crossing_response(op, granted, 'tok' if granted else '',
                                           'OK' if granted else 'PEDESTRIAN', t)
        v, w = (0.0, 0.0) if self.door else out.cmd     # bridge door interlock
        self.sim.set_command('bus_1', v, w)
        self.still_since = (self.still_since or t) if v == 0 and w == 0 else None
        if out.door_request and self.still_since and t - self.still_since >= 0.5:
            self.door = True
        elif not out.door_request:
            self.door = False
        self.sim.step()
        b = self.bus
        self.trace.append((t, b.x, b.y, abs(b.v), out.phase, self.door,
                           geo.circle_overlaps_rect(b.x, b.y, 0.30, geo.CROSSING)))
        return out

    def run(self, until, stop_when_done=True):
        while self.sim.t < until:
            out = self.step()
            if stop_when_done and out.done:
                return out
        return None


def start(h, max_duration=120.0):
    h.core.start(node_route(), h.sim.t, max_duration)


def test_nominal_docking_meets_tolerance_and_completes():
    h = Harness()
    start(h)
    out = h.run(90.0)
    assert out is not None and out.done == (True, COMPLETE, 'DOCKED_AND_BOARDED')
    pos_err, yaw_err = h.core.final_error
    assert pos_err <= 0.20 and yaw_err <= 0.17
    b = h.bus
    assert math.dist((b.x, b.y), geo.NODES['B']) <= 0.20
    phases = [p for _, _, _, _, p, _, _ in h.trace]
    assert phases.index(APPROACH) < phases.index(DOCK) < phases.index(BOARD)
    # door open only while stopped, for at least the 8 s boarding interval
    open_t = [t for t, *_, door, _ in h.trace if door]
    assert all(v < 0.02 for _, _, _, v, _, door, _ in h.trace if door)
    assert open_t[-1] - open_t[0] >= 8.0 - DT


def test_crossing_acquired_before_entry_renewed_and_released_after():
    h = Harness()
    start(h)
    h.run(90.0)
    in_c = [t for t, *_, c in h.trace if c]
    acquire = [t for t, op, ok in h.ops if op == 'ACQUIRE' and ok]
    release = [t for t, op, ok in h.ops if op == 'RELEASE']
    renew = [t for t, op, ok in h.ops if op == 'RENEW']
    assert acquire and acquire[0] < in_c[0]
    assert release and release[0] > in_c[-1]
    assert len(renew) >= 2 and max(b - a for a, b in zip(renew, renew[1:])) <= 1.0 + DT
    assert h.core.token == ''


def test_denied_crossing_waits_at_the_stop_line_outside_c():
    h = Harness(deny=True)
    start(h)
    h.run(40.0, stop_when_done=False)
    assert not any(c for *_, c in h.trace)
    assert h.trace[-1][4] == WAIT and h.core.reason == 'PEDESTRIAN'
    b = h.bus
    assert 3.5 < b.x < 4.5 - 0.30 and abs(b.v) < 0.02
    h.deny = False                                   # pedestrian gone
    out = h.run(110.0)
    assert out is not None and out.done[0]


def test_blocked_bay_waits_without_asking_for_the_crossing():
    h = Harness(bay_car=True)
    start(h)
    h.run(40.0, stop_when_done=False)
    assert h.trace[-1][4] == WAIT and h.core.reason == 'BAY_BLOCKED'
    assert not h.ops and not any(c for *_, c in h.trace)
    del h.sim.bodies['car_9']                        # bay cleared
    out = h.run(110.0)
    assert out is not None and out.done[0]


def test_cancel_gives_zero_command_at_once_and_releases_outside_c():
    h = Harness()
    start(h)
    while h.bus.x < 2.5 or abs(h.bus.y - 4.0) > 0.1:   # driving on N-Cn
        h.step()
    assert h.outs[-1][1].cmd[0] > 0.1
    h.core.cancel()
    out = h.step()
    assert out.cmd == (0.0, 0.0) and out.done == (False, SAFE_STOP, 'CANCELED')
    assert out.crossing_op == ('RELEASE', 'tok')
    assert h.step().cmd == (0.0, 0.0)


def test_stale_pose_holds_then_resumes():
    h = Harness()
    start(h)
    while h.bus.y < 3.0:
        h.step()
    for _ in range(int(1.5 / DT)):                  # pose frozen for 1.5 s
        out = h.step(feed_pose=False)
    assert out.phase == SAFE_STOP and out.cmd == (0.0, 0.0) and not out.eta_valid
    out = h.run(100.0)
    assert out is not None and out.done[0]


def test_stale_pose_too_long_aborts():
    h = Harness()
    start(h)
    h.run(3.0, stop_when_done=False)
    for _ in range(int(6.0 / DT)):
        out = h.step(feed_pose=False)
        if out.done:
            break
    assert out.done == (False, SAFE_STOP, 'POSE_STALE')


def test_eta_error_at_5_m_within_3_s():
    h = Harness()
    start(h)
    h.run(90.0)
    at5 = next((t, o) for t, o in h.outs if 0 <= o.distance_remaining <= 5.0)
    arrival = next(t for t, x, y, v, *_ in h.trace
                   if math.dist((x, y), geo.NODES['B']) < 0.2 and v < 0.02)
    t5, out = at5
    assert out.eta_valid
    assert abs(t5 + out.eta - arrival) <= 3.0


def test_timeout_and_safety_latch_end_the_goal():
    h = Harness()
    start(h, max_duration=2.0)
    out = h.run(5.0)
    assert out.done == (False, SAFE_STOP, 'TIMEOUT')
    h2 = Harness()
    start(h2)
    h2.run(1.0, stop_when_done=False)
    h2.core.on_safety(True)
    assert h2.step().done == (False, SAFE_STOP, 'SAFETY_LATCHED')


def test_path_geometry_and_config_validation():
    p = Path(node_route(), 0.30)
    assert p.total == pytest.approx(9.0)
    assert set(p.corners) == {1}                     # turn at N; Cn is straight
    assert p.c_in == pytest.approx(4.2 - 1.0 + 2.0, abs=0.03)   # footprint reaches x=4.2
    with pytest.raises(ValueError):
        BusCore(BusConfig(cruise_speed=0.5))
    with pytest.raises(ValueError):
        BusCore(BusConfig(renew_period=3.0))
    core = BusCore()
    with pytest.raises(RuntimeError):
        core.start(node_route(), 0.0, 60.0)          # no pose yet
