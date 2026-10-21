"""Planar homography: image pixels -> floor coordinates in metres (M1-A).

Assignment 8.2 step 3: map the foot point of each person to the map with a
homography fitted to at least four coplanar calibration points. Only points
on the floor plane are mapped correctly; a head or a raised object is not.
"""
import numpy as np


def fit_homography(image_points, map_points):
    """Return the 3x3 H with map ~ H * image, from >= 4 point pairs.

    Direct linear transform: each pair gives two linear equations in the
    nine entries of H; the solution is the null vector of that system
    (smallest singular vector), scaled so H[2, 2] = 1.
    """
    img = np.asarray(image_points, float).reshape(-1, 2)
    world = np.asarray(map_points, float).reshape(-1, 2)
    if len(img) < 4 or len(img) != len(world):
        raise ValueError('need at least 4 matching image/map points')
    rows = []
    for (u, v), (x, y) in zip(img, world):
        rows.append([u, v, 1, 0, 0, 0, -x * u, -x * v, -x])
        rows.append([0, 0, 0, u, v, 1, -y * u, -y * v, -y])
    _, s, vt = np.linalg.svd(np.asarray(rows))
    # A unique H needs rank 8 (nine unknowns, one free scale).
    if len(s) < 8 or s[7] < 1e-9 * s[0]:
        raise ValueError('calibration points are degenerate (e.g. three on a line)')
    h = vt[-1].reshape(3, 3)
    return h / h[2, 2]


def image_to_map(h, u, v):
    """Map one pixel (u, v) to floor (x, y) in metres."""
    x, y, w = h @ np.array([u, v, 1.0])
    if abs(w) < 1e-12:
        raise ValueError('point maps to infinity (above the horizon)')
    return float(x / w), float(y / w)
