"""Unit tests for the shared map geometry."""
import math

import pytest

from smart_hub import geometry as geo


def test_crossing_overlap_touching_is_not_overlap():
    assert not geo.circle_overlaps_rect(5.0, 4.95, 0.2, geo.CROSSING)   # touches y=4.75
    assert geo.circle_overlaps_rect(5.0, 4.94, 0.2, geo.CROSSING)
    assert geo.circle_overlaps_rect(5.0, 3.0, 0.2, geo.CROSSING)


def test_site_containment():
    assert geo.circle_inside_rect(0.2, 0.2, 0.2, geo.SITE)
    assert not geo.circle_inside_rect(0.19, 4.0, 0.2, geo.SITE)


def test_predict_pose_straight_and_turning():
    assert geo.predict_pose(0, 0, 0, 1.0, 0.0, 2.0) == pytest.approx((2.0, 0.0, 0.0))
    x, y, th = geo.predict_pose(0, 0, 0, 1.0, math.pi / 2, 1.0)     # quarter circle, r=2/pi
    assert (x, y, th) == pytest.approx((2 / math.pi, 2 / math.pi, math.pi / 2))


def test_edge_ids_match_m3_convention():
    assert geo.edge_id('J', 'E') == 'E-J'
    assert geo.nearest_edge(7.0, 2.1) == 'Cs-J'
    assert geo.nearest_edge(3.0, 6.0) == 'R-W'
    assert geo.nearest_edge(6.0, 7.5) == ''
    assert len(geo.EDGES) == 15
