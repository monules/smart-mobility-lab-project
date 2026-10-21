"""Unit tests for the simulator's synthetic camera (M5)."""
import pytest

cv2 = pytest.importorskip('cv2')
np = pytest.importorskip('numpy')

from smart_hub.camera_model import HEIGHT, PERSON_COLOR, PinholeCamera, WIDTH  # noqa: E402

QUEUE = (2.0, 5.0, 6.0, 7.0)
CROSSING = (4.5, 1.25, 5.5, 4.75)


def dark_blobs(img):
    mask = cv2.inRange(img, (0, 0, 0), (110, 110, 110))
    n, _, stats, _ = cv2.connectedComponentsWithStats(mask)
    return [tuple(int(v) for v in s[:4]) for s in stats[1:]]


def inside_image(uv):
    return uv is not None and 0 <= uv[0] < WIDTH and 0 <= uv[1] < HEIGHT


def test_queue_and_crossing_are_in_view():
    cam = PinholeCamera()
    for xmin, ymin, xmax, ymax in (QUEUE, CROSSING):
        for x, y in ((xmin, ymin), (xmax, ymin), (xmax, ymax), (xmin, ymax)):
            assert inside_image(cam.project(x, y)), (x, y)


def test_farther_is_higher_in_the_image_and_behind_is_none():
    cam = PinholeCamera()
    assert cam.project(4.0, 6.5)[1] < cam.project(4.0, 3.0)[1]
    assert cam.project(4.2, -10.0, 8.0) is None              # behind the camera


def test_one_figure_per_person_with_feet_at_projection():
    cam = PinholeCamera()
    people = [(2.4, 6.6), (3.3, 6.6), (4.2, 6.6), (5.0, 2.2)]
    img = cam.render(people, (QUEUE, CROSSING))
    blobs = dark_blobs(img)
    assert len(blobs) == len(people)
    feet = sorted((x + w / 2, y + h) for x, y, w, h in blobs)
    expected = sorted(cam.project(px, py) for px, py in people)
    for (u, v), (eu, ev) in zip(feet, expected):
        assert abs(u - eu) <= 2 and abs(v - ev) <= 2


def test_empty_floor_has_no_person_pixels():
    img = PinholeCamera().render([], (QUEUE, CROSSING))
    assert img.shape == (HEIGHT, WIDTH, 3)
    assert not (img == np.array(PERSON_COLOR, np.uint8)).all(axis=2).any()
