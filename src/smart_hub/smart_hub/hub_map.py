"""Map M1 geometry used by M3's pure-Python engines (no ROS imports).

Coordinates are metres, origin south-west, +x east, +y north (assignment
section 2.1).  Edge identifiers are *canonical*: the two node names sorted
alphabetically and joined with '-', exactly like ``edge_id`` in the handbook's
``route_engine.py``.  So the assignment's "J-E" is ``E-J`` and "Cn-B" is
``B-Cn``.  Incoming ids are normalised with :func:`normalize_edge_id`, so
either spelling is accepted on input.
"""
from __future__ import annotations

import heapq
import math
from typing import Dict, Optional, Tuple

NODES: Dict[str, Tuple[float, float]] = {
    'A': (1.0, 2.0), 'Cs': (5.0, 2.0), 'J': (9.0, 2.0), 'E': (11.0, 2.0),
    'N': (1.0, 4.0), 'Cn': (5.0, 4.0), 'B': (8.0, 4.0), 'U': (11.0, 4.0),
    'R': (3.0, 6.0), 'W': (5.0, 6.0), 'D': (9.0, 1.0),
    'P1': (8.0, 6.5), 'P2': (9.5, 6.5), 'P3': (11.0, 6.5),
}

_VEHICLE_RAW = [('A', 'Cs'), ('Cs', 'J'), ('J', 'E'), ('A', 'N'),
                ('N', 'Cn'), ('Cn', 'B'), ('B', 'U'), ('U', 'E')]
_PARKING_RAW = [('U', 'P3'), ('P3', 'P2'), ('P2', 'P1')]

ACCESS_NODE = 'U'          # parking is entered from U (U-P3, P3-P2, P2-P1)
OBSTACLE_O = (7.0, 2.0, 0.35)   # centre x, centre y, radius


def edge_id(a: str, b: str) -> str:
    """Canonical undirected edge id (matches the handbook route engine)."""
    return '-'.join(sorted((a, b)))


VEHICLE_EDGES: Tuple[str, ...] = tuple(edge_id(a, b) for a, b in _VEHICLE_RAW)
PARKING_EDGES: Tuple[str, ...] = tuple(edge_id(a, b) for a, b in _PARKING_RAW)


def normalize_edge_id(raw: str) -> Optional[str]:
    """Return the canonical id for ``raw`` or None if it is not a map edge."""
    if not isinstance(raw, str):
        return None
    parts = raw.strip().replace('–', '-').split('-')
    if len(parts) != 2 or parts[0] not in NODES or parts[1] not in NODES:
        return None
    return edge_id(parts[0], parts[1])


def edge_endpoints(eid: str):
    a, b = eid.split('-')
    return NODES[a], NODES[b]


def edge_length(eid: str) -> float:
    (ax, ay), (bx, by) = edge_endpoints(eid)
    return math.hypot(bx - ax, by - ay)


def point_segment_distance(px: float, py: float, ax: float, ay: float,
                           bx: float, by: float) -> float:
    """Shortest distance from point P to segment AB (clamped projection)."""
    dx, dy = bx - ax, by - ay
    seg2 = dx * dx + dy * dy
    if seg2 == 0.0:
        return math.hypot(px - ax, py - ay)
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / seg2))
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))


def access_distances(access: str = ACCESS_NODE) -> Dict[str, float]:
    """Graph distance from the parking access node to P1..P3 (Dijkstra over
    the parking edges only), used for the 'nearest free slot' policy."""
    graph: Dict[str, list] = {}
    for a, b in _PARKING_RAW:
        d = math.dist(NODES[a], NODES[b])
        graph.setdefault(a, []).append((b, d))
        graph.setdefault(b, []).append((a, d))
    best = {access: 0.0}
    heap = [(0.0, access)]
    while heap:
        cost, u = heapq.heappop(heap)
        if cost > best.get(u, math.inf):
            continue
        for v, w in graph.get(u, []):
            if cost + w < best.get(v, math.inf):
                best[v] = cost + w
                heapq.heappush(heap, (cost + w, v))
    return {k: v for k, v in best.items() if k.startswith('P')}
