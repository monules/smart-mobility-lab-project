"""Zone polygons and point-in-polygon test (M1-A, assignment 2.1 and 8.2 step 4)."""

# Polygons as lists of (x, y) corners in metres, counter-clockwise.
QUEUE_Q = [(2.0, 5.0), (6.0, 5.0), (6.0, 7.0), (2.0, 7.0)]
CROSSING_C = [(4.5, 1.25), (5.5, 1.25), (5.5, 4.75), (4.5, 4.75)]


def point_in_polygon(x, y, polygon):
    """Ray casting: count edge crossings of a ray from (x, y) to +x.

    Points exactly on an edge count as inside, so a person standing on the
    queue line is counted rather than silently dropped.
    """
    n = len(polygon)
    inside = False
    for i in range(n):
        x1, y1 = polygon[i]
        x2, y2 = polygon[(i + 1) % n]
        # on the edge segment?
        cross = (x2 - x1) * (y - y1) - (y2 - y1) * (x - x1)
        if abs(cross) < 1e-9 and min(x1, x2) - 1e-9 <= x <= max(x1, x2) + 1e-9 \
                and min(y1, y2) - 1e-9 <= y <= max(y1, y2) + 1e-9:
            return True
        if (y1 > y) != (y2 > y):
            x_at_y = x1 + (y - y1) * (x2 - x1) / (y2 - y1)
            if x < x_at_y:
                inside = not inside
    return inside
