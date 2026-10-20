"""Unit tests for the deterministic simulator and the shipped scenario file."""
import math
import os

import pytest
from smart_hub.sim_core import DT, HubSim
import yaml

SCENARIO = {
    'scenario_id': 'T',
    'observation_noise_m': 0.05,
    'agents': {'robot_1': {'kind': 'robot', 'x': 3.0, 'y': 6.0, 'theta': 0.0}},
    'obstacles': {'O': {'x': 7.0, 'y': 2.0, 'active': False}},
    'parking': {'P1': 'occupied'},
    'events': [{'t': 1.0, 'type': 'obstacle', 'id': 'O', 'active': True},
               {'t': 1.0, 'type': 'freeze', 'topic': 'odom', 'duration': 0.5},
               {'t': 2.0, 'type': 'estop', 'value': True}],
}


def drive(sim, steps, v=0.2, w=0.0):
    trace = []
    for _ in range(steps):
        sim.set_command('robot_1', v, w)
        sim.step()
        trace.append(sim.observe(sim.bodies['robot_1']))
    return trace


def test_straight_motion_matches_unicycle():
    sim = HubSim(SCENARIO, seed=1)
    drive(sim, 20)                                 # 1.0 s at 0.2 m/s
    body = sim.bodies['robot_1']
    assert body.x == pytest.approx(3.2) and body.y == pytest.approx(6.0)
    assert sim.t == pytest.approx(20 * DT)


def test_same_seed_same_run_and_reset_restores():
    a, b = HubSim(SCENARIO, seed=7), HubSim(SCENARIO, seed=7)
    assert drive(a, 40, w=0.3) == drive(b, 40, w=0.3)
    first = drive(a, 1, w=0.3)
    a.reset(7)
    assert a.t == 0.0 and a.bodies['robot_1'].x == 3.0
    assert not a.bodies['O'].active and not a.estop
    # reset also rewinds the noise RNG: step 41 is reproduced exactly
    assert drive(a, 41, w=0.3)[-1] == first[0]
    c = HubSim(SCENARIO, seed=8)
    assert drive(c, 5) != drive(HubSim(SCENARIO, seed=7), 5)


def test_drive_timeout_stops_body():
    sim = HubSim(SCENARIO, seed=1, input_timeout=0.5)
    sim.set_command('robot_1', 0.2, 0.0)
    for _ in range(10):
        sim.step()                                  # 0.5 s, command still fresh
    assert sim.bodies['robot_1'].v == pytest.approx(0.2)
    for _ in range(2):
        sim.step()
    assert sim.bodies['robot_1'].v == 0.0


def test_events_fire_at_their_time():
    sim = HubSim(SCENARIO, seed=1)
    applied = []
    for _ in range(int(round(2.1 / DT))):
        applied += [e['type'] for e in sim.step()]
        if sim.t == pytest.approx(1.2):
            assert sim.is_frozen('odom')
    assert applied == ['obstacle', 'freeze', 'estop']
    assert sim.bodies['O'].active and sim.estop and not sim.is_frozen('odom')


def test_injected_event_and_bad_events():
    sim = HubSim(SCENARIO, seed=1)
    sim.apply_event({'type': 'parking', 'slot': 'P2', 'state': 'offline'})
    assert sim.parking['P2'] == 'offline'
    for bad in ({'type': 'teleport'}, {'type': 'freeze', 'topic': 'camera', 'duration': 1},
                {'type': 'estop', 'value': 'yes'}):
        with pytest.raises(ValueError):
            sim.apply_event(bad)
    assert not sim.set_command('O', 1.0, 0.0)       # obstacles cannot be driven
    assert not sim.set_command('robot_1', math.nan, 0.0)


def test_shipped_s9_scenario_is_valid():
    path = os.path.join(os.path.dirname(__file__), '..', 'scenarios', 'S9.yaml')
    with open(path) as fh:
        data = yaml.safe_load(fh)
    assert data['scenario_id'] == 'S9'
    sim = HubSim(data, seed=9)
    assert {'robot_1', 'bus_1', 'O'} <= set(sim.bodies)
