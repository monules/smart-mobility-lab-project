"""Map M1 geometry shared by the M5 nodes (pure Python, no ROS).

Coordinates are metres; origin is the south-west corner (assignment 2.1).
Every body is a circle (assignment 3.3: circular baseline footprints).
"""
import math

# 12 m x 8 m site, as (xmin, ymin, xmax, ymax).
SITE = (0.0, 0.0, 12.0, 8.0)
# Crossing C: x in [4.5, 5.5], y in [1.25, 4.75].
CROSSING = (4.5, 1.25, 5.5, 4.75)

# Footprint radii (assignment 3.3) and obstacle O (assignment 2.1).
RADIUS_BY_KIND = {'robot': 0.20, 'bus': 0.30, 'car': 0.30, 'obstacle': 0.35}

# Graph nodes and edges (assignment 2.1, 3.1).
NODES = {
    'A': (1.0, 2.0), 'Cs': (5.0, 2.0), 'J': (9.0, 2.0), 'E': (11.0, 2.0),
    'N': (1.0, 4.0), 'Cn': (5.0, 4.0), 'B': (8.0, 4.0), 'U': (11.0, 4.0),
    'R': (3.0, 6.0), 'W': (5.0, 6.0), 'D': (9.0, 1.0),
    'P1': (8.0, 6.5), 'P2': (9.5, 6.5), 'P3': (11.0, 6.5),
}
_EDGE_PAIRS = [('A', 'Cs'), ('Cs', 'J'), ('J', 'E'), ('A', 'N'), ('N', 'Cn'),
               ('Cn', 'B'), ('B', 'U'), ('U', 'E'), ('U', 'P3'), ('P3', 'P2'),
               ('P2', 'P1'), ('R', 'W'), ('W', 'Cn'), ('Cn', 'Cs'), ('J', 'D')]


def edge_id(a, b):
    """Canonical undirected edge id: node names sorted, joined by '-'.

    Same convention as M3's hub_map.edge_id, e.g. J-E is 'E-J'.
    """
    return '-'.join(sorted((a, b)))


EDGES = {edge_id(a, b): (NODES[a], NODES[b]) for a, b in _EDGE_PAIRS}


def normalize_edge(raw):
    """Canonical id of a map edge given either spelling ('J-E' or 'E-J'), else ''."""
    parts = str(raw).strip().replace('\u2013', '-').split('-')
    if len(parts) != 2 or not all(n in NODES for n in parts):
        return ''
    eid = edge_id(*parts)
    return eid if eid in EDGES else ''


# Roads: the edges a bus or car may use (no parking lane, walkway or crossing
# strip). Same set as M3's hub_map.VEHICLE_EDGES.
VEHICLE_EDGES = frozenset(edge_id(a, b) for a, b in _EDGE_PAIRS[:8])


def point_segment_distance(px, py, a, b):
    (ax, ay), (bx, by) = a, b
    dx, dy = bx - ax, by - ay
    seg2 = dx * dx + dy * dy
    t = 0.0 if seg2 == 0 else max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / seg2))
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))


def nearest_edge(x, y, max_distance=0.5, edges=None):
    """Id of the closest graph edge strictly within max_distance, else ''.

    edges limits the search (e.g. VEHICLE_EDGES for a bus or car).

    Ties keep the alphabetically first edge, so the result is deterministic.
    """
    best, best_d = '', max_distance
    for eid, (a, b) in sorted(EDGES.items()):
        if edges is not None and eid not in edges:
            continue
        d = point_segment_distance(x, y, a, b)
        if d < best_d:
            best, best_d = eid, d
    return best


def circle_overlaps_rect(x, y, r, rect):
    """Return True if a circle of radius r at (x, y) overlaps the rectangle.

    Touching exactly is not an overlap (same convention as M2-B's
    footprint_overlaps_crossing).
    """
    xmin, ymin, xmax, ymax = rect
    dx = x - min(max(x, xmin), xmax)
    dy = y - min(max(y, ymin), ymax)
    return dx * dx + dy * dy < r * r


def circle_inside_rect(x, y, r, rect):
    """Return True if the whole circle lies inside the rectangle."""
    xmin, ymin, xmax, ymax = rect
    return (xmin + r <= x <= xmax - r) and (ymin + r <= y <= ymax - r)


def gap_between(x1, y1, r1, x2, y2, r2):
    """Free distance between two circles (negative means they overlap)."""
    return math.hypot(x2 - x1, y2 - y1) - r1 - r2


def predict_pose(x, y, theta, v, w, dt):
    """Unicycle pose after dt seconds of constant (v, w)."""
    if abs(w) < 1e-9:
        return x + v * math.cos(theta) * dt, y + v * math.sin(theta) * dt, theta
    theta2 = theta + w * dt
    x2 = x + v / w * (math.sin(theta2) - math.sin(theta))
    y2 = y - v / w * (math.cos(theta2) - math.cos(theta))
    return x2, y2, theta2
