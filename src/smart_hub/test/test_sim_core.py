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
    for bad in ({'type': 'teleport'}, {'type': 'freeze', 'topic': 'lidar', 'duration': 1},
                {'type': 'estop', 'value': 'yes'}):
        with pytest.raises(ValueError):
            sim.apply_event(bad)
    assert not sim.set_command('O', 1.0, 0.0)       # obstacles cannot be driven
    assert not sim.set_command('robot_1', math.nan, 0.0)


def test_pedestrians_appear_hide_walk_and_leave():
    sim = HubSim({'scenario_id': 'P', 'events': [
        {'t': 0.5, 'type': 'pedestrian', 'id': 'p1', 'x': 3.0, 'y': 6.0, 'vx': 0.2},
        {'t': 1.0, 'type': 'occlude', 'id': 'p1', 'duration': 0.3},
        {'t': 2.0, 'type': 'pedestrian', 'id': 'p1', 'active': False}]}, seed=1)
    seen = {}
    for _ in range(int(round(2.5 / DT))):
        sim.step()
        seen[round(sim.t, 2)] = sim.camera_people()
    assert seen[0.25] == []
    assert seen[0.6] == [(pytest.approx(3.0 + 0.2 * 0.10), 6.0)]   # 2 steps of walking
    assert seen[1.15] == []                                         # occluded 0.3 s
    assert len(seen[1.4]) == 1
    assert seen[2.1] == [] and 'p1' not in sim.pedestrians


def test_camera_freeze_modes_validate():
    sim = HubSim(SCENARIO, seed=1)
    for topic in ('camera', 'camera_repeat'):
        sim.apply_event({'type': 'freeze', 'topic': topic, 'duration': 1.0})
        assert sim.is_frozen(topic)
    for bad in ({'type': 'pedestrian', 'id': 'p9'}, {'type': 'occlude', 'id': 'p1'}):
        with pytest.raises(ValueError):
            sim.apply_event(bad)


@pytest.mark.parametrize('name', ['S1', 'S9'])
def test_shipped_scenarios_are_valid(name):
    path = os.path.join(os.path.dirname(__file__), '..', 'scenarios', name + '.yaml')
    with open(path) as fh:
        data = yaml.safe_load(fh)
    assert data['scenario_id'] == name
    sim = HubSim(data, seed=1)
    assert {'robot_1', 'bus_1'} <= set(sim.bodies)


def test_s1_script_matches_assignment_counts():
    """Assignment 8.3: stable queue counts 0, 3, 5, 4 and one crossing person."""
    path = os.path.join(os.path.dirname(__file__), '..', 'scenarios', 'S1.yaml')
    with open(path) as fh:
        sim = HubSim(yaml.safe_load(fh), seed=1)

    def in_queue():
        return sum(2 <= p.x <= 6 and 5 <= p.y <= 7 for p in sim.pedestrians.values())
    counts = {}
    while sim.t < 25.0:
        sim.step()
        counts[round(sim.t, 2)] = (in_queue(), len(sim.pedestrians))
    assert [counts[t][0] for t in (4.0, 8.0, 13.0, 18.0)] == [0, 3, 5, 4]
    assert counts[22.0] == (4, 5) and counts[24.5] == (4, 4)


def test_shipped_s9_scenario_is_valid():
    path = os.path.join(os.path.dirname(__file__), '..', 'scenarios', 'S9.yaml')
    with open(path) as fh:
        data = yaml.safe_load(fh)
    assert data['scenario_id'] == 'S9'
    sim = HubSim(data, seed=9)
    assert {'robot_1', 'bus_1', 'O'} <= set(sim.bodies)


def test_cruise_cars_loop_and_ignore_commands():
    sim = HubSim({'agents': {'car_1': {'kind': 'car', 'x': 8.6, 'y': 2.0,
                                       'cruise': 0.4, 'loop_x': [5.3, 8.7]}}}, seed=1)
    car = sim.bodies['car_1']
    sim.set_command('car_1', 0.0, 1.0)          # scripted traffic ignores commands
    for _ in range(10):                         # 0.5 s at 0.4 m/s = 0.2 m
        sim.step()
    assert car.v == pytest.approx(0.4) and car.theta == pytest.approx(0.0)
    assert car.x == pytest.approx(5.3 + 0.4 * DT * 2, abs=0.05)  # wrapped once
    assert car.y == pytest.approx(2.0)


def test_vehicle_events_add_and_remove():
    sim = HubSim({'agents': {}, 'events': [
        {'t': 0.5, 'type': 'vehicle', 'id': 'car_2', 'x': 6.0, 'y': 2.0, 'cruise': 0.1},
        {'t': 1.0, 'type': 'vehicle', 'id': 'car_2', 'active': False}]}, seed=1)
    seen = set()
    while sim.t < 1.5:
        sim.step()
        seen.add(('car_2' in sim.bodies, round(sim.t, 2)))
    assert (True, 0.75) in seen and 'car_2' not in sim.bodies
    for bad in ({'type': 'vehicle', 'id': 'c'},
                {'type': 'vehicle', 'id': 'c', 'x': 1, 'y': 1, 'kind': 'obstacle'}):
        with pytest.raises(ValueError):
            sim.apply_event(bad)


def _scenario(name):
    path = os.path.join(os.path.dirname(__file__), '..', 'scenarios', name + '.yaml')
    with open(path) as fh:
        return yaml.safe_load(fh)


def test_s5_script_fills_cs_j_then_empties_it():
    """Assignment 12: one car, then 4 cars on the 4-car edge Cs-J, then 1 again."""
    sim = HubSim(_scenario('S5'), seed=5)

    def on_cs_j():
        return sum(1 for b in sim.bodies.values()
                   if b.kind == 'car' and 5.0 <= b.x <= 9.0 and b.y == pytest.approx(2.0))
    counts = {}
    while sim.t < 62.0:
        sim.step()
        counts[round(sim.t, 2)] = on_cs_j()
    assert [counts[t] for t in (10.0, 25.0, 34.0, 40.0)] == [1, 4, 4, 1]
    assert sim.bodies['O'].active and sim.is_frozen('vehicles') is False


def test_s4_script_starts_p1_occupied_and_takes_p3_offline():
    sim = HubSim(_scenario('S4'), seed=4)
    assert sim.parking == {'P1': 'occupied', 'P2': 'free', 'P3': 'free'}
    while sim.t < 10.5:
        sim.step()
    assert sim.parking['P3'] == 'offline'


def test_boarding_takes_at_most_four_waiting_people_while_door_open_at_b():
    """Assignment 10.2 step 5: up to four board per stop, the rest keep waiting."""
    from smart_hub.sim_core import BOARD_FIRST, BOARD_INTERVAL
    sim = HubSim(_scenario('S3'), seed=3)
    sim.bodies['bus_1'].x, sim.bodies['bus_1'].y = 8.05, 4.0      # docked at B
    sim.apply_event({'type': 'pedestrian', 'id': 'walker', 'x': 3.0, 'y': 5.5, 'vx': 0.3})
    for _ in range(40):                       # closed door: nobody boards
        sim.step()
    assert len(sim.pedestrians) == 6 and not sim.pop_boardings()
    sim.set_door(True)
    opened = sim.t
    while sim.t < opened + 12.0:
        sim.step()
    boarded = sim.pop_boardings()
    assert [pid for pid, _ in boarded] == ['q1', 'q2', 'q3', 'q4']
    assert boarded[0][1] == pytest.approx(opened + BOARD_FIRST, abs=DT)
    assert boarded[1][1] - boarded[0][1] == pytest.approx(BOARD_INTERVAL, abs=DT)
    assert set(sim.pedestrians) == {'q5', 'walker'}   # moving people never board


def test_no_boarding_when_bus_is_away_from_b_or_moving():
    sim = HubSim(_scenario('S3'), seed=3)
    sim.set_door(True)                        # door open but bus at A
    for _ in range(100):
        sim.step()
    assert not sim.pop_boardings()
    bus = sim.bodies['bus_1']
    bus.x, bus.y = 8.0, 4.0
    sim.set_command('bus_1', 0.1, 0.0)        # rolling through the bay
    sim.set_door(False)
    sim.set_door(True)
    for _ in range(40):
        sim.set_command('bus_1', 0.1, 0.0)
        sim.step()
    assert not sim.pop_boardings()


def test_vehicle_edges_match_m3_and_exclude_the_crossing():
    from smart_hub import geometry as geo
    from smart_hub.hub_map import VEHICLE_EDGES
    assert geo.VEHICLE_EDGES == set(VEHICLE_EDGES)
    # at node Cn the crossing strip Cn-Cs is as close as the road; a bus must
    # still be reported on a road
    assert geo.nearest_edge(5.0, 3.99, edges=geo.VEHICLE_EDGES) in ('B-Cn', 'Cn-N')
    assert geo.nearest_edge(5.0, 3.99) == 'Cn-Cs'
