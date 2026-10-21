"""route_core (M4, S6): cost formula, edge removal, Dijkstra; T11 oracle test."""
import math
import random

import pytest
from smart_hub import geometry as geo
from smart_hub import route_core as rc

NOW = 100.0


def view(traffic=None, obstacles=(), crossing=('', False, True), **kw):
    """Build an EdgeView with fresh inputs; empty roads at 0.40 m/s by default."""
    v = rc.EdgeView(**kw)
    t = {e: (0.40, 0.0, False, True) for e in geo.VEHICLE_EDGES}
    t.update(traffic or {})
    v.on_traffic([(e,) + s for e, s in t.items()], NOW)
    v.on_bodies(list(obstacles), NOW)
    v.on_crossing(*crossing, NOW)
    return v


def route(v, agent, start, goal, **kw):
    return rc.plan(v.costs(agent, NOW, **kw), start, goal)


OBST_O = (7.0, 2.0, 0.35)     # obstacle O on Cs-J
OBST_O2 = (6.5, 4.0, 0.35)    # second obstacle on Cn-B


def test_baseline_and_detour_costs_at_equal_speed_zero_density_and_wait():
    """13.3 step 1: lower road 10 m, upper detour 14 m."""
    costs = view().costs('car', NOW)
    low, up = ['A', 'Cs', 'J', 'E'], ['A', 'N', 'Cn', 'B', 'U', 'E']
    assert sum(rc.edge_length(e) for e in rc.edges_of(low)) == pytest.approx(10.0)
    assert sum(rc.edge_length(e) for e in rc.edges_of(up)) == pytest.approx(14.0)
    assert rc.path_cost(costs, low)[0] == pytest.approx(10.0 / 0.40)
    assert rc.path_cost(costs, up)[0] == pytest.approx(14.0 / 0.40)
    assert route(view(), 'car', 'A', 'E') == (low, pytest.approx(25.0))


def test_obstacle_removes_cs_j_and_the_route_avoids_it():
    """13.3 step 2: O at (7, 2) blocks Cs-J; plan through the upper road."""
    v = view(obstacles=[OBST_O])
    costs = v.costs('car', NOW)
    assert costs['Cs-J'] == (None, 'OBSTACLE')
    nodes, cost = route(v, 'car', 'A', 'E')
    assert nodes == ['A', 'N', 'Cn', 'B', 'U', 'E']
    assert all(costs[e][0] is not None for e in rc.edges_of(nodes))
    # traffic_monitor's BLOCKED flag alone also removes the edge
    v2 = view(traffic={'Cs-J': (0.40, 0.0, True, True)})
    assert v2.costs('car', NOW)['Cs-J'] == (None, 'BLOCKED')


def test_density_changes_the_choice_by_cost_not_distance():
    """13.3 step 3: a dense, slow lower road loses although it is shorter."""
    v = view(traffic={'Cs-J': (0.10, 1.0, False, True)})
    nodes, cost = route(v, 'car', 'A', 'E')
    assert nodes == ['A', 'N', 'Cn', 'B', 'U', 'E'] and cost == pytest.approx(35.0)
    low = rc.path_cost(v.costs('car', NOW), ['A', 'Cs', 'J', 'E'])[0]
    assert low == pytest.approx(10 + 4 / 0.10 * 3 + 5)
    # the speed floor 0.05 m/s keeps a stopped edge finite
    v = view(traffic={'Cs-J': (0.0, 0.25, False, True)})
    assert v.costs('car', NOW)['Cs-J'][0] == pytest.approx(4 / 0.05 * 1.5)


def test_two_blocked_edges_give_no_route():
    """13.3 step 4 / T12."""
    v = view(obstacles=[OBST_O, OBST_O2])
    assert route(v, 'car', 'A', 'E') == (None, math.inf)
    assert route(v, 'robot', 'R', 'E') == (None, math.inf)


def test_robot_permissions_and_detour():
    assert route(view(), 'robot', 'R', 'D')[0] == ['R', 'W', 'Cn', 'Cs', 'J', 'D']
    assert route(view(obstacles=[OBST_O]), 'robot', 'R', 'D')[0] == \
        ['R', 'W', 'Cn', 'B', 'U', 'E', 'J', 'D']
    assert route(view(), 'robot', 'R', 'P1') == (None, math.inf)     # parking: car only
    assert route(view(), 'bus', 'A', 'P1') == (None, math.inf)       # bus never parks
    assert route(view(), 'car', 'U', 'P1')[0] == ['U', 'P3', 'P2', 'P1']
    assert route(view(), 'car', 'A', 'D') == (None, math.inf)        # J-D: robot only


def test_stale_or_unknown_observations_remove_edges():
    v = view()
    assert v.costs('car', NOW + 1.6)['A-Cs'] == (None, 'STALE_OBSTACLES')
    v.on_bodies([], NOW + 1.6)
    assert v.costs('car', NOW + 1.6)['A-Cs'] == (None, 'STALE_TRAFFIC')
    assert v.costs('robot', NOW + 1.6)['R-W'][0] is not None        # no traffic needed
    v = view(traffic={'A-Cs': (-1.0, -1.0, False, False)})
    assert v.costs('car', NOW)['A-Cs'] == (None, 'TRAFFIC_UNKNOWN')
    assert route(v, 'car', 'A', 'E')[0] == ['A', 'N', 'Cn', 'B', 'U', 'E']
    v = rc.EdgeView(require_traffic=False)
    v.on_bodies([], NOW)
    assert v.costs('car', NOW)['A-Cs'][0] == pytest.approx(10.0 + 5.0)  # crossing unknown


def test_crossing_wait_and_excluded_edges():
    assert rc.CROSSING_EDGES == {'A-Cs', 'Cs-J', 'Cn-N', 'B-Cn', 'Cn-Cs', 'Cn-W'}
    free = view().costs('robot', NOW)['Cn-Cs'][0]
    busy = view(crossing=('', True, True), crossing_wait=5.0).costs('robot', NOW)['Cn-Cs'][0]
    mine = view(crossing=('robot_1', False, True)).costs('robot', NOW, agent_id='robot_1')
    other = view(crossing=('bus_1', False, True)).costs('robot', NOW, agent_id='robot_1')
    assert busy == pytest.approx(free + 5.0)
    assert mine['Cn-Cs'][0] == pytest.approx(free)
    assert other['Cn-Cs'][0] == pytest.approx(free + 5.0)
    assert view().costs('car', NOW, excluded=['J-Cs'])['Cs-J'] == (None, 'EXCLUDED')


def test_segment_rectangle_intersection():
    rect = (0, 0, 1, 1)
    assert rc.segment_hits_rect((-1, 0.5), (2, 0.5), rect)
    assert rc.segment_hits_rect((0.5, 0.5), (0.6, 0.6), rect)
    assert not rc.segment_hits_rect((-1, 2), (2, 2), rect)
    assert not rc.segment_hits_rect((2, -1), (3, 3), rect)


# --------------------------------------------------------------- T11 oracle
def oracle_cost(raw, agent, eid, agent_id):
    """Independent re-implementation of the 13.2 cost from the raw state."""
    if eid not in rc.ALLOWED[agent]:
        return None
    (ax, ay), (bx, by) = geo.EDGES[eid]
    for ox, oy, orad in raw['obstacles']:
        # distance point-segment, written out again
        t = max(0.0, min(1.0, ((ox - ax) * (bx - ax) + (oy - ay) * (by - ay))
                         / ((bx - ax) ** 2 + (by - ay) ** 2)))
        if math.hypot(ox - ax - t * (bx - ax), oy - ay - t * (by - ay)) < orad + \
                rc.RADIUS[agent] + 0.05:
            return None
    v, rho = rc.NOMINAL_SPEED[agent], 0.0
    if eid in geo.VEHICLE_EDGES:
        speed, density, blocked, valid = raw['traffic'][eid]
        if blocked or not valid:
            return None
        v, rho = min(v, speed), density
    owner, ped, valid = raw['crossing']
    busy = ped or not valid or (owner and owner != agent_id)
    wait = 5.0 if eid in rc.CROSSING_EDGES and busy else 0.0
    return math.hypot(bx - ax, by - ay) / max(v, 0.05) * (1 + 2 * rho) + wait


def oracle(raw, agent, start, goal, agent_id):
    """Exhaustive enumeration of every simple path; minimum cost or inf."""
    adj = {}
    for eid in geo.EDGES:
        c = oracle_cost(raw, agent, eid, agent_id)
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


def random_state(rng):
    traffic = {e: (rng.choice([0.0, rng.uniform(0.02, 0.5)]), rng.uniform(0, 1),
                   rng.random() < 0.12, rng.random() > 0.08) for e in geo.VEHICLE_EDGES}
    obstacles = [(rng.uniform(1, 11), rng.uniform(1, 7), 0.35)
                 for _ in range(rng.randint(0, 2))]
    crossing = (rng.choice(['', 'robot_1', 'bus_1']), rng.random() < 0.3, rng.random() > 0.1)
    return {'traffic': traffic, 'obstacles': obstacles, 'crossing': crossing}


@pytest.mark.parametrize('seed', range(20))
def test_t11_planner_matches_exhaustive_oracle(seed):
    rng = random.Random(seed)
    raw = random_state(rng)
    v = rc.EdgeView()
    v.on_traffic([(e,) + s for e, s in raw['traffic'].items()], NOW)
    v.on_bodies(raw['obstacles'], NOW)
    v.on_crossing(*raw['crossing'], NOW)
    pairs = [('car', 'A', 'E'), ('car', 'A', 'P1'), ('bus', 'A', 'B'), ('robot', 'R', 'D'),
             ('robot', 'R', 'E')]
    pairs += [(a, rng.choice(sorted(geo.NODES)), rng.choice(sorted(geo.NODES)))
              for a in ('car', 'bus', 'robot')]
    for agent, start, goal in pairs:
        aid = {'robot': 'robot_1', 'bus': 'bus_1'}.get(agent, '')
        costs = v.costs(agent, NOW, agent_id=aid)
        nodes, cost = rc.plan(costs, start, goal)
        want = oracle(raw, agent, start, goal, aid)
        if math.isinf(want):
            assert nodes is None, (seed, agent, start, goal)
            continue
        assert nodes[0] == start and nodes[-1] == goal
        for eid in rc.edges_of(nodes):                 # continuity, allowed, not removed
            assert eid in geo.EDGES and eid in rc.ALLOWED[agent]
            assert costs[eid][0] is not None
        assert cost == pytest.approx(want, rel=1e-6), (seed, agent, start, goal)


def test_the_requesting_agent_is_not_traffic_for_itself():
    """Found in the S3 reruns: bus_1 parked at A made its own edge A-N congested."""
    parked = {'A-N': (0.0, 0.5, False, True)}          # 1 standing vehicle, K = 2
    v = view(traffic=parked)
    v.on_bodies([], NOW, {'bus_1': (1.004, 2.006)})    # at A: label could be A-N or A-Cs
    assert v.costs('bus', NOW)['A-N'][0] == pytest.approx(2 / 0.05 * 2)   # anonymous car
    assert v.costs('bus', NOW, agent_id='bus_1')['A-N'][0] == pytest.approx(5.0)
    assert route(v, 'bus', 'A', 'B', agent_id='bus_1')[0] == ['A', 'N', 'Cn', 'B']
    # someone else on the edge still counts
    v = view(traffic={'A-N': (0.1, 1.0, False, True)})
    v.on_bodies([], NOW, {'bus_1': (1.0, 2.0)})
    assert v.costs('bus', NOW, agent_id='bus_1')['A-N'][0] == pytest.approx(2 / 0.1 * 2)
