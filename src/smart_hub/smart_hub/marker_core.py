"""Marker content for the RViz dashboard (pure Python, no ROS).

MarkerCore keeps the latest hub states and turns them into plain dicts
(one per marker); dashboard_marker_publisher only converts those dicts to
visualization_msgs/Marker. It is read-only: nothing here can command motion.

A state older than stale_after seconds (local receipt time) is drawn as
UNKNOWN, because a retained sample is not necessarily fresh (assignment 5).
"""
from smart_hub import geometry as geo

# Queue Q: x in [2, 6], y in [5, 7] (assignment 2.1).
QUEUE = (2.0, 5.0, 6.0, 7.0)
# Parking slots are drawn as 1.0 m x 0.8 m boxes centred on P1..P3
# (the assignment gives only the centre points).
SLOT_W, SLOT_H = 1.0, 0.8
SLOTS = ('P1', 'P2', 'P3')
# Graph edges as one LINE_LIST: consecutive point pairs, sorted by edge id.
EDGE_POINTS = [p for _, ab in sorted(geo.EDGES.items()) for p in ab]

GREY = (0.5, 0.5, 0.5, 0.6)
# CrossingState.state values from M2-B's coordinator; anything else is grey.
CROSSING_COLOR = {'FREE': (0.1, 0.8, 0.1, 0.5), 'GRANTED': (0.1, 0.4, 1.0, 0.5),
                  'HOLD': (1.0, 0.8, 0.0, 0.5), 'OCCUPIED': (0.6, 0.2, 0.8, 0.5),
                  'PEDESTRIAN': (1.0, 0.5, 0.0, 0.6)}
# ParkingSlot.state: FREE, RESERVED, OCCUPIED; UNKNOWN or missing is grey.
SLOT_COLOR = {'FREE': (0.1, 0.8, 0.1, 0.6), 'RESERVED': (1.0, 0.8, 0.0, 0.6),
              'OCCUPIED': (0.9, 0.1, 0.1, 0.6)}
COLOR_BY_KIND = {'robot': (0.0, 0.7, 1.0, 1.0), 'bus': (1.0, 0.6, 0.0, 1.0),
                 'car': (0.8, 0.8, 0.0, 1.0), 'obstacle': (0.2, 0.2, 0.2, 1.0)}
# Edge overlays from /traffic/edges. EdgeState has no congestion label, so
# only what its fields state is drawn: blocked, invalid (unknown) and a
# density ratio at/above the assignment's congestion threshold (S5: 0.70).
EDGE_OVERLAY = (('BLOCKED', 0, (1.0, 0.1, 0.1, 1.0)),
                ('UNKNOWN', 1, (0.25, 0.25, 0.25, 1.0)),
                ('DENSE', 2, (1.0, 0.55, 0.0, 1.0)))
DENSITY_THRESHOLD = 0.70


def rect_marker(ns, mid, rect, color, z=0.0):
    """Flat box covering rect = (xmin, ymin, xmax, ymax)."""
    xmin, ymin, xmax, ymax = rect
    return {'ns': ns, 'id': mid, 'type': 'cube', 'color': color,
            'pos': ((xmin + xmax) / 2, (ymin + ymax) / 2, z),
            'scale': (xmax - xmin, ymax - ymin, 0.02)}


def text_marker(ns, mid, x, y, text, height=0.3, color=(1.0, 1.0, 1.0, 1.0)):
    return {'ns': ns, 'id': mid, 'type': 'text', 'pos': (x, y, 0.5),
            'scale': (0.0, 0.0, height), 'color': color, 'text': text}


def slot_rect(name):
    x, y = geo.NODES[name]
    return (x - SLOT_W / 2, y - SLOT_H / 2, x + SLOT_W / 2, y + SLOT_H / 2)


class MarkerCore:
    def __init__(self, stale_after=1.0):
        self.stale_after = stale_after
        self.vehicles = None     # (list of (id, kind, x, y, valid), receipt time)
        self.crossing = None     # ((state, owner_id), receipt time)
        self.parking = None      # ({slot_id: state}, receipt time)
        self.safety = None       # ((stop_latched, reason), receipt time)
        self.demand = None       # ((valid, predicted, baseline, horizon), receipt time)
        self.edges = None        # ({edge_id: (blocked, valid, density)}, receipt time)
        self.last_event = ''

    def on_vehicles(self, vehicles, now):
        self.vehicles = (list(vehicles), now)

    def on_crossing(self, state, owner_id, valid, now):
        self.crossing = ((state if valid else 'UNKNOWN', owner_id), now)

    def on_parking(self, slots, now):
        self.parking = (dict(slots), now)

    def on_safety(self, stop_latched, reason, now):
        self.safety = ((stop_latched, reason), now)

    def on_demand(self, valid, predicted, baseline, horizon_sec, now):
        self.demand = ((bool(valid), predicted, baseline, horizon_sec), now)

    def on_edges(self, edges, now):
        """edges: {edge_id: (blocked, valid, density_ratio)} from /traffic/edges."""
        self.edges = (dict(edges), now)

    def on_event(self, entity_id, event, reason):
        self.last_event = '%s %s %s' % (entity_id, event, reason)

    def _fresh(self, sample, now):
        """Return the sample's data if received within stale_after, else None."""
        if sample is None or now - sample[1] > self.stale_after:
            return None
        return sample[0]

    def _demand_text(self, now):
        """Forecast text; assignment 9.3 asks for an explicit 'unavailable'."""
        demand = self._fresh(self.demand, now)
        if demand is None or not demand[0]:
            return 'prediction unavailable'
        _, predicted, baseline, horizon = demand
        return 'demand +%ds: %.1f (now %.0f)' % (horizon, predicted, baseline)

    def _edge_overlays(self, now):
        """One LINE_LIST per edge condition, only for known map edges."""
        edges = self._fresh(self.edges, now) or {}
        groups = {name: [] for name, _, _ in EDGE_OVERLAY}
        for eid, (blocked, valid, density) in sorted(edges.items()):
            if eid not in geo.EDGES:
                continue                              # not a map edge: ignore
            if not valid:
                groups['UNKNOWN'] += geo.EDGES[eid]
            elif blocked:
                groups['BLOCKED'] += geo.EDGES[eid]
            elif density >= DENSITY_THRESHOLD:
                groups['DENSE'] += geo.EDGES[eid]
        return [{'ns': 'edge_state', 'id': mid, 'type': 'line_list', 'pos': (0.0, 0.0, 0.02),
                 'scale': (0.12, 0.0, 0.0), 'color': color, 'points': groups[name]}
                for name, mid, color in EDGE_OVERLAY if groups[name]]

    def build(self, now):
        """Return the list of marker dicts for one 5 Hz publication."""
        m = [rect_marker('site', 0, geo.SITE, (0.15, 0.15, 0.15, 0.3), z=-0.02),
             rect_marker('queue', 0, QUEUE, (0.3, 0.6, 1.0, 0.25)),
             text_marker('queue', 1, 4.0, 7.3, 'Queue Q')]

        m.append({'ns': 'edges', 'id': 0, 'type': 'line_list', 'pos': (0.0, 0.0, 0.01),
                  'scale': (0.05, 0.0, 0.0), 'color': (0.7, 0.7, 0.7, 0.8),
                  'points': EDGE_POINTS})

        m += self._edge_overlays(now)
        m.append(text_marker('queue', 2, 4.0, 5.3, self._demand_text(now), 0.25))

        crossing = self._fresh(self.crossing, now) or ('UNKNOWN', '')
        state, owner = crossing
        m.append(rect_marker('crossing', 0, geo.CROSSING,
                             CROSSING_COLOR.get(state, GREY), z=0.01))
        m.append(text_marker('crossing', 1, 5.0, 5.0,
                             'C: %s %s' % (state, owner) if owner else 'C: %s' % state))

        slots = self._fresh(self.parking, now) or {}
        for i, name in enumerate(SLOTS):
            s = slots.get(name, 'UNKNOWN')
            m.append(rect_marker('parking', i, slot_rect(name), SLOT_COLOR.get(s, GREY)))
            x, y = geo.NODES[name]
            m.append(text_marker('parking', 10 + i, x, y + 0.6, '%s %s' % (name, s), 0.25))

        vehicles = self._fresh(self.vehicles, now)
        for i, (vid, kind, x, y, valid) in enumerate(sorted(vehicles or [])):
            r = geo.RADIUS_BY_KIND.get(kind, 0.3)
            color = COLOR_BY_KIND.get(kind, GREY) if valid else GREY
            m.append({'ns': 'vehicles', 'id': i, 'type': 'cylinder', 'pos': (x, y, 0.1),
                      'scale': (2 * r, 2 * r, 0.2), 'color': color})
            m.append(text_marker('vehicle_labels', i, x, y + r + 0.2, vid, 0.2))

        safety = self._fresh(self.safety, now)
        if safety is None:
            status, color = 'SAFETY: UNKNOWN (no fresh /hub/safety)', GREY
        elif safety[0]:
            status, color = 'SAFETY: STOP LATCHED (%s)' % safety[1], (1.0, 0.2, 0.2, 1.0)
        else:
            status, color = 'SAFETY: OK (%s)' % safety[1], (0.2, 1.0, 0.2, 1.0)
        m.append(text_marker('status', 0, 6.0, 8.6, status, 0.35, color))
        m.append(text_marker('status', 1, 6.0, 8.2, 'last event: %s' % self.last_event, 0.25))
        return m
