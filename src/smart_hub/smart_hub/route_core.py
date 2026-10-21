"""Route planning for S6 (M4), pure Python: no ROS, time is passed in.

Edge cost (assignment 13.2), in seconds:

    c_e = L_e / max(v_e, 0.05) * (1 + 2 * rho_e) + w_e

* L_e: Euclidean edge length (map M1).
* v_e: the agent's nominal speed, bounded by the measured mean speed on the
  edge when traffic_monitor reports a fresh, valid state for it.
* rho_e: measured density ratio (0 for edges traffic_monitor does not cover).
  The requesting agent itself is taken out: on every edge whose 0.5 m
  corridor contains the agent (traffic_monitor's corridor), one vehicle
  (1 / K_e, K_e = floor(L_e / 1.0 m headway) as in traffic_monitor) is
  subtracted, and if nobody else is left the measured speed (its own, often
  0 while it waits) is ignored. Otherwise a bus parked at A makes its own
  first edge look congested and is sent the long way. Position, not the
  edge label, decides: at a node the label flips with observation noise.
* w_e: expected crossing wait: crossing_wait_sec for an edge that intersects
  C while C is occupied, owned by another agent or unknown; 0 otherwise.

An edge is REMOVED (not penalised) when it is forbidden to the agent type
(3.1), listed in excluded_edges, reported blocked, intersected by an
obstacle's footprint (obstacle radius + agent radius + margin), or when its
observation is stale or invalid (13.2: "treat it as blocked"). Dijkstra over
the remaining edges; no remaining path = NO_ROUTE.
"""
import heapq
import math

from smart_hub import geometry as geo

NOMINAL_SPEED = {'robot': 0.25, 'bus': 0.40, 'car': 0.40}   # assignment 3.3 caps
RADIUS = {'robot': 0.20, 'bus': 0.30, 'car': 0.30}
MIN_SPEED = 0.05                     # m/s, the floor in the cost formula
CORRIDOR = 0.5                       # m, traffic_monitor's edge corridor half-width

_PARKING = frozenset(geo.edge_id(a, b)
                     for a, b in (('U', 'P3'), ('P3', 'P2'), ('P2', 'P1')))
_ROBOT = frozenset(geo.edge_id(a, b) for a, b in (
    ('R', 'W'), ('W', 'Cn'), ('Cn', 'Cs'), ('Cs', 'J'), ('J', 'D'),     # baseline
    ('Cn', 'B'), ('B', 'U'), ('U', 'E'), ('E', 'J')))                   # detour links
ALLOWED = {'car': geo.VEHICLE_EDGES | _PARKING, 'bus': geo.VEHICLE_EDGES, 'robot': _ROBOT}


def segment_hits_rect(a, b, rect):
    """Return True if segment ab meets the closed rectangle (Liang-Barsky clip)."""
    (x0, y0), (x1, y1) = a, b
    xmin, ymin, xmax, ymax = rect
    dx, dy = x1 - x0, y1 - y0
    t0, t1 = 0.0, 1.0
    for p, q in ((-dx, x0 - xmin), (dx, xmax - x0), (-dy, y0 - ymin), (dy, ymax - y0)):
        if p == 0:
            if q < 0:
                return False
            continue
        r = q / p
        if p < 0:
            t0 = max(t0, r)
        else:
            t1 = min(t1, r)
        if t0 > t1:
            return False
    return True


CROSSING_EDGES = frozenset(e for e, (a, b) in geo.EDGES.items()
                           if segment_hits_rect(a, b, geo.CROSSING))


def edge_length(eid):
    a, b = geo.EDGES[eid]
    return math.dist(a, b)


def edges_of(nodes):
    return [geo.edge_id(a, b) for a, b in zip(nodes, nodes[1:])]


class EdgeView:
    """Latest observations, turned into per-agent edge costs on demand."""

    def __init__(self, traffic_timeout=1.5, bodies_timeout=1.0, crossing_timeout=1.0,
                 crossing_wait=5.0, obstacle_margin=0.05, require_traffic=True):
        self.traffic_timeout = traffic_timeout
        self.bodies_timeout = bodies_timeout
        self.crossing_timeout = crossing_timeout
        self.crossing_wait = crossing_wait
        self.margin = obstacle_margin
        self.require_traffic = require_traffic
        self.traffic, self.traffic_t = {}, None     # eid -> (speed, density, blocked, valid)
        self.obstacles, self.bodies_t = [], None     # [(x, y, r)]
        self.crossing, self.crossing_t = None, None  # (owner, pedestrian, valid)
        self.agent_pos = {}                          # vehicle id -> (x, y)

    def on_traffic(self, edges, now):
        """edges: iterable of (edge_id, mean_speed, density, blocked, valid)."""
        self.traffic = {}
        for eid, speed, density, blocked, valid in edges:
            canon = geo.normalize_edge(eid)
            if canon:
                self.traffic[canon] = (speed, density, bool(blocked), bool(valid))
        self.traffic_t = now

    def on_bodies(self, obstacles, now, agent_pos=None):
        """Obstacles [(x, y, radius)]; agent_pos {vehicle id: (x, y)}."""
        self.obstacles, self.bodies_t = list(obstacles), now
        if agent_pos is not None:
            self.agent_pos = dict(agent_pos)

    def on_crossing(self, owner, pedestrian, valid, now):
        self.crossing, self.crossing_t = (owner, bool(pedestrian), bool(valid)), now

    def _fresh(self, t, timeout, now):
        return t is not None and now - t <= timeout

    def edge_cost(self, eid, agent, now, agent_id=''):
        """Return (cost in s, 'OK') or (None, reason the edge is removed)."""
        if eid not in ALLOWED.get(agent, ()):
            return None, 'FORBIDDEN'
        r = RADIUS[agent]
        if not self._fresh(self.bodies_t, self.bodies_timeout, now):
            return None, 'STALE_OBSTACLES'
        a, b = geo.EDGES[eid]
        for ox, oy, orad in self.obstacles:
            if geo.point_segment_distance(ox, oy, a, b) < orad + r + self.margin:
                return None, 'OBSTACLE'
        speed, density = NOMINAL_SPEED[agent], 0.0
        if eid in geo.VEHICLE_EDGES and self.require_traffic:
            state = self.traffic.get(eid) if self._fresh(
                self.traffic_t, self.traffic_timeout, now) else None
            if state is None:
                return None, 'STALE_TRAFFIC'
            t_speed, t_density, blocked, valid = state
            if not valid:
                return None, 'TRAFFIC_UNKNOWN'
            if blocked:
                return None, 'BLOCKED'
            density = min(1.0, max(0.0, t_density))
            pos = self.agent_pos.get(agent_id) if agent_id else None
            if pos is not None and geo.point_segment_distance(pos[0], pos[1], a, b) <= \
                    CORRIDOR:
                density = max(0.0, density - 1.0 / max(1, int(edge_length(eid) // 1.0)))
                if density <= 1e-9:
                    t_speed = speed          # only the agent itself was measured
            speed = min(speed, max(0.0, t_speed))
        wait = 0.0
        if eid in CROSSING_EDGES:
            cs = self.crossing if self._fresh(self.crossing_t, self.crossing_timeout,
                                              now) else None
            busy = cs is None or not cs[2] or cs[1] or (cs[0] and cs[0] != agent_id)
            wait = self.crossing_wait if busy else 0.0
        return edge_length(eid) / max(speed, MIN_SPEED) * (1.0 + 2.0 * density) + wait, 'OK'

    def costs(self, agent, now, excluded=(), agent_id=''):
        """{edge_id: (cost or None, reason)} for every map edge."""
        out = {}
        skip = {geo.normalize_edge(e) for e in excluded}
        for eid in geo.EDGES:
            out[eid] = (None, 'EXCLUDED') if eid in skip else \
                self.edge_cost(eid, agent, now, agent_id)
        return out


def plan(costs, start, goal):
    """Dijkstra over the edges with a cost. Return (nodes, cost) or (None, inf)."""
    if start not in geo.NODES or goal not in geo.NODES:
        return None, math.inf
    graph = {}
    for eid, (cost, _) in costs.items():
        if cost is None:
            continue
        a, b = eid.split('-')
        graph.setdefault(a, []).append((b, cost))
        graph.setdefault(b, []).append((a, cost))
    best, prev = {start: 0.0}, {}
    heap = [(0.0, start)]
    while heap:
        d, u = heapq.heappop(heap)
        if u == goal:
            nodes = [goal]
            while nodes[-1] != start:
                nodes.append(prev[nodes[-1]])
            return nodes[::-1], d
        if d > best.get(u, math.inf):
            continue
        for v, w in sorted(graph.get(u, [])):           # sorted: deterministic ties
            if d + w < best.get(v, math.inf):
                best[v], prev[v] = d + w, u
                heapq.heappush(heap, (d + w, v))
    return None, math.inf


def path_cost(costs, nodes):
    """Cost of a node path under costs; (inf, edge) at the first removed edge."""
    total = 0.0
    for eid in edges_of(nodes):
        cost, _ = costs.get(eid, (None, 'NOT_AN_EDGE'))
        if cost is None:
            return math.inf, eid
        total += cost
    return total, ''
