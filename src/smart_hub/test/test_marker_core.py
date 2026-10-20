"""Unit tests for MarkerCore: dashboard marker content from plain data."""
import pytest

from smart_hub import geometry as geo
from smart_hub.marker_core import CROSSING_COLOR, GREY, MarkerCore, SLOT_COLOR

T0 = 100.0


def by_ns(markers, ns):
    return [m for m in markers if m['ns'] == ns]


def text_of(markers, ns, mid):
    return next(m['text'] for m in markers if m['ns'] == ns and m['id'] == mid)


def test_static_map_without_any_input():
    m = MarkerCore().build(T0)
    edges = by_ns(m, 'edges')[0]
    assert edges['type'] == 'line_list' and len(edges['points']) == 2 * len(geo.EDGES)
    assert by_ns(m, 'site')[0]['scale'][:2] == (12.0, 8.0)
    assert by_ns(m, 'queue')[0]['pos'][:2] == (4.0, 6.0)
    assert by_ns(m, 'vehicles') == []
    assert by_ns(m, 'crossing')[0]['color'] == GREY
    assert text_of(m, 'status', 0).startswith('SAFETY: UNKNOWN')


def test_unique_ids_per_namespace():
    c = MarkerCore()
    c.on_vehicles([('robot_1', 'robot', 3, 6, True), ('bus_1', 'bus', 1, 2, True)], T0)
    keys = [(m['ns'], m['id']) for m in c.build(T0)]
    assert len(keys) == len(set(keys))


def test_vehicle_circles_use_footprint_radius():
    c = MarkerCore()
    c.on_vehicles([('robot_1', 'robot', 3.0, 6.0, True), ('O', 'obstacle', 7.0, 2.0, True),
                   ('bus_1', 'bus', 1.0, 2.0, False)], T0)
    v = {m['pos'][:2]: m for m in by_ns(c.build(T0 + 0.1), 'vehicles')}
    assert v[(3.0, 6.0)]['type'] == 'cylinder'
    assert v[(3.0, 6.0)]['scale'][0] == pytest.approx(2 * geo.RADIUS_BY_KIND['robot'])
    assert v[(7.0, 2.0)]['scale'][0] == pytest.approx(0.70)
    assert v[(1.0, 2.0)]['color'] == GREY                 # invalid -> grey


def test_crossing_colour_follows_state():
    c = MarkerCore()
    c.on_crossing('GRANTED', 'robot_1', True, T0)
    m = c.build(T0)
    assert by_ns(m, 'crossing')[0]['color'] == CROSSING_COLOR['GRANTED']
    assert text_of(m, 'crossing', 1) == 'C: GRANTED robot_1'
    c.on_crossing('PEDESTRIAN', '', True, T0)
    assert by_ns(c.build(T0), 'crossing')[0]['color'] == CROSSING_COLOR['PEDESTRIAN']
    c.on_crossing('GRANTED', 'robot_1', False, T0)          # invalid -> UNKNOWN
    assert text_of(c.build(T0), 'crossing', 1) == 'C: UNKNOWN robot_1'


def test_stale_state_drawn_unknown():
    c = MarkerCore(stale_after=1.0)
    c.on_crossing('FREE', '', True, T0)
    c.on_parking({'P1': 'FREE'}, T0)
    c.on_vehicles([('robot_1', 'robot', 3, 6, True)], T0)
    c.on_safety(False, 'OK', T0)
    m = c.build(T0 + 1.5)
    assert by_ns(m, 'crossing')[0]['color'] == GREY
    assert text_of(m, 'parking', 10) == 'P1 UNKNOWN'
    assert by_ns(m, 'vehicles') == []
    assert text_of(m, 'status', 0).startswith('SAFETY: UNKNOWN')


def test_parking_slots():
    c = MarkerCore()
    c.on_parking({'P1': 'OCCUPIED', 'P2': 'FREE'}, T0)
    m = c.build(T0)
    slots = by_ns(m, 'parking')
    assert [s['color'] for s in slots if s['type'] == 'cube'] == \
        [SLOT_COLOR['OCCUPIED'], SLOT_COLOR['FREE'], GREY]
    assert slots[0]['pos'][:2] == geo.NODES['P1']
    assert text_of(m, 'parking', 12) == 'P3 UNKNOWN'


def test_demand_text_valid_invalid_and_stale():
    c = MarkerCore(stale_after=1.0)
    assert text_of(c.build(T0), 'queue', 2) == 'prediction unavailable'
    c.on_demand(True, 7.24, 5.0, 30, T0)
    assert text_of(c.build(T0), 'queue', 2) == 'demand +30s: 7.2 (now 5)'
    assert text_of(c.build(T0 + 1.5), 'queue', 2) == 'prediction unavailable'   # stale
    c.on_demand(False, -1.0, -1.0, 30, T0)                                   # warm-up
    assert text_of(c.build(T0), 'queue', 2) == 'prediction unavailable'


def test_edge_overlays_from_traffic_edges():
    c = MarkerCore()
    assert by_ns(c.build(T0), 'edge_state') == []           # no data: no overlay
    c.on_edges({'Cs-J': (True, True, 0.2), 'A-Cs': (False, True, 0.75),
                'E-J': (False, False, -1.0), 'B-U': (False, True, 0.1),
                'NOT-AN-EDGE': (True, True, 1.0)}, T0)
    overlays = {m['id']: m for m in by_ns(c.build(T0), 'edge_state')}
    assert overlays[0]['points'] == list(geo.EDGES['Cs-J'])    # blocked
    assert overlays[1]['points'] == list(geo.EDGES['E-J'])     # invalid -> unknown
    assert overlays[2]['points'] == list(geo.EDGES['A-Cs'])    # density >= 0.70
    assert all(len(m['points']) % 2 == 0 for m in overlays.values())
    assert by_ns(c.build(T0 + 1.5), 'edge_state') == []        # stale: base map only


def test_safety_text_shows_latch_and_reason():
    c = MarkerCore()
    c.on_safety(True, 'ESTOP', T0)
    c.on_event('safety_supervisor', 'STOP_LATCHED', 'ESTOP')
    m = c.build(T0)
    assert text_of(m, 'status', 0) == 'SAFETY: STOP LATCHED (ESTOP)'
    assert text_of(m, 'status', 1) == 'last event: safety_supervisor STOP_LATCHED ESTOP'
    c.on_safety(False, 'OK', T0)
    assert text_of(c.build(T0), 'status', 0) == 'SAFETY: OK (OK)'
