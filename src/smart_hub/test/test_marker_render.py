"""Unit tests for the headless dashboard renderer (M5)."""
import pytest

cv2 = pytest.importorskip('cv2')

from smart_hub.marker_core import MarkerCore, ROUTE_COLOR  # noqa: E402
from smart_hub.marker_render import bgr, HEIGHT, render, to_px, WIDTH  # noqa: E402


def test_site_corners_map_inside_the_picture():
    for x, y in ((0, 0), (12, 0), (12, 8), (0, 8)):
        u, v = to_px(x, y)
        assert 0 <= u < WIDTH and 0 <= v < HEIGHT
    assert to_px(0, 8)[1] < to_px(0, 0)[1]               # north is up


def test_dashboard_picture_shows_robot_and_route():
    core = MarkerCore()
    core.on_vehicles([('robot_1', 'robot', 3.0, 6.0, True)], 100.0)
    core.on_route('robot', [(3.0, 6.0), (6.0, 6.0)])
    img = render(core.build(100.0), 'test')
    assert img.shape == (HEIGHT, WIDTH, 3)
    robot = img[to_px(3.0, 6.0)[1], to_px(3.0, 6.0)[0]]
    assert robot[0] > 150 and robot[2] < 80               # blue-ish robot disc (BGR)
    u, v = to_px(5.0, 6.0)
    assert tuple(int(c) for c in img[v, u]) == bgr(ROUTE_COLOR['robot'])


def test_empty_input_still_draws_the_map():
    img = render(MarkerCore().build(0.0))
    assert img.std() > 5                                 # not a blank picture
