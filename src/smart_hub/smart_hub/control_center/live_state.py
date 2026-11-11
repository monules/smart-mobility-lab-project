"""Latest hub state for the Control Center's live view (pure Python, no ROS).

The ROS node feeds plain values in; the web page polls snapshot() 5 times a
second. Every sample keeps its receipt time, and snapshot() marks a value
stale once it is older than its limit, so a stopped run fades out instead of
freezing on the screen (the same rule as the RViz dashboard). The view is
read-only: nothing here can command the hub.
"""
import collections
import threading

from smart_hub import geometry as geo

STALE_AFTER = 1.0          # s for 5-20 Hz streams
SLOW_STALE_AFTER = 2.5     # s for the 1 Hz forecast and traffic labels
EVENTS_KEPT = 40
MISSION_ENTITIES = ('robot', 'bus', 'coordinator')
MISSION_EVENTS = ('ESCORT', 'DOCK', 'DISPATCH', 'MISSION', 'ROUTE', 'ARRIVED', 'FAILED',
                  'CANCEL', 'BOARD', 'EXIT', 'READY', 'BLOCKED', 'CROSSING')


class LiveState:
    """Thread-safe store of the most recent hub topics (see module docstring)."""

    def __init__(self):
        self._lock = threading.Lock()
        self.reset()

    def reset(self):
        with self._lock:
            self._latest = {}
            self._routes = {}
            self._events = collections.deque(maxlen=EVENTS_KEPT)
            self._missions = {}

    def _put(self, key, value, now):
        with self._lock:
            self._latest[key] = (value, now)

    def on_vehicles(self, vehicles, now):
        """Store agents as dicts with id, kind, x, y, theta, speed, edge, valid."""
        self._put('vehicles', vehicles, now)

    def on_pedestrians(self, people, now):
        self._put('pedestrians', people, now)

    def on_safety(self, stop_latched, motion_permitted, reason, now):
        self._put('safety', {'stop_latched': stop_latched,
                             'motion_permitted': motion_permitted, 'reason': reason}, now)

    def on_estop(self, asserted, now):
        self._put('estop', bool(asserted), now)

    def on_crossing(self, state, owner_id, pedestrian, valid, now):
        self._put('crossing', {'state': state, 'owner': owner_id,
                               'pedestrian': pedestrian, 'valid': valid}, now)

    def on_parking(self, slots, now):
        self._put('parking', slots, now)

    def on_queue(self, count, confidence, crossing_occupied, valid, now):
        self._put('queue', {'count': count, 'confidence': round(confidence, 2),
                            'crossing_occupied': crossing_occupied, 'valid': valid}, now)

    def on_demand(self, predicted, baseline, horizon_sec, model_id, valid, now):
        self._put('demand', {'predicted': round(predicted, 2), 'baseline': round(baseline, 2),
                             'horizon': horizon_sec, 'model': model_id, 'valid': valid}, now)

    def on_edges(self, edges, now):
        self._put('edges', edges, now)

    def on_route(self, agent, points, now):
        with self._lock:
            self._routes[agent] = (points, now)

    def on_event(self, entity_id, event, reason, stamp_s, now):
        with self._lock:
            self._events.append({'t': round(stamp_s, 2), 'entity': entity_id,
                                 'event': event, 'reason': reason})
            if any(e in entity_id for e in MISSION_ENTITIES) and \
                    any(w in event for w in MISSION_EVENTS):
                self._missions[entity_id] = {'event': event, 'reason': reason, 'at': now}

    def snapshot(self, now):
        """Return everything the page draws, with an age and stale flag per value."""
        with self._lock:
            out = {}
            for key, (value, t) in self._latest.items():
                limit = SLOW_STALE_AFTER if key in ('demand', 'edges') else STALE_AFTER
                age = now - t
                out[key] = {'value': value, 'age': round(age, 2), 'stale': age > limit}
            out['routes'] = {a: pts for a, (pts, t) in self._routes.items()
                             if now - t < 30.0}
            out['events'] = list(self._events)[::-1]
            out['missions'] = {k: dict(v, age=round(now - v['at'], 1))
                               for k, v in self._missions.items()}
            return out


def static_map():
    """Return the fixed map M1 for drawing: site, crossing, queue, nodes, edges."""
    return {'site': geo.SITE, 'crossing': geo.CROSSING, 'queue': (2.0, 5.0, 6.0, 7.0),
            'nodes': geo.NODES,
            'edges': {eid: {'a': ab[0], 'b': ab[1], 'road': eid in geo.VEHICLE_EDGES}
                      for eid, ab in geo.EDGES.items()},
            'radius': geo.RADIUS_BY_KIND, 'slot_size': (1.0, 0.8)}
