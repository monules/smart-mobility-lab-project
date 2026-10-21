"""Unit tests for M1-A perception building blocks (no ROS)."""
import math

import pytest
from smart_hub.perception.homography import fit_homography, image_to_map
from smart_hub.perception.queue_logic import QueueLogic
from smart_hub.perception.tracker import Tracker
from smart_hub.perception.zones import CROSSING_C, point_in_polygon, QUEUE_Q


# ------------------------------------------------------------------ zones
@pytest.mark.parametrize('x, y, inside', [
    (4.0, 6.0, True), (2.0, 6.0, True), (6.0, 7.0, True),     # interior, edge, corner
    (1.99, 6.0, False), (4.0, 7.01, False), (8.0, 6.0, False)])
def test_point_in_queue(x, y, inside):
    assert point_in_polygon(x, y, QUEUE_Q) is inside


def test_point_in_crossing_and_concave_polygon():
    assert point_in_polygon(5.0, 3.0, CROSSING_C)
    assert not point_in_polygon(5.0, 5.0, CROSSING_C)
    l_shape = [(0, 0), (2, 0), (2, 1), (1, 1), (1, 2), (0, 2)]
    assert point_in_polygon(0.5, 1.5, l_shape) and not point_in_polygon(1.5, 1.5, l_shape)


# ------------------------------------------------------------- homography
def test_homography_exact_on_calibration_points():
    img = [(100, 400), (500, 400), (450, 100), (150, 100)]
    world = [(2, 5), (6, 5), (6, 7), (2, 7)]
    h = fit_homography(img, world)
    for (u, v), (x, y) in zip(img, world):
        assert image_to_map(h, u, v) == pytest.approx((x, y), abs=1e-6)


def test_homography_matches_simulator_camera_off_calibration_points():
    cv2 = pytest.importorskip('cv2')  # noqa: F841 - camera_model needs OpenCV
    from smart_hub.camera_model import PinholeCamera
    from smart_hub.passenger_detector import DEFAULT_IMAGE_POINTS, DEFAULT_MAP_POINTS
    cam = PinholeCamera()
    h = fit_homography(DEFAULT_IMAGE_POINTS, DEFAULT_MAP_POINTS)
    for x, y in ((3.3, 6.6), (5.0, 2.2), (4.7, 4.0)):         # inside Q and in C
        assert image_to_map(h, *cam.project(x, y)) == pytest.approx((x, y), abs=0.02)


def test_degenerate_calibration_rejected():
    with pytest.raises(ValueError):
        fit_homography([(0, 0), (1, 1), (2, 2), (3, 3)], [(0, 0), (1, 0), (2, 0), (3, 1)])
    with pytest.raises(ValueError):
        fit_homography([(0, 0), (1, 0), (0, 1)], [(0, 0), (1, 0), (0, 1)])


# ---------------------------------------------------------------- tracker
def feed(tracker, points, t0, t1, dt=0.1):
    t = t0
    while t <= t1 + 1e-9:
        tracker.update(points, round(t, 3))
        t += dt
    return round(t - dt, 3)


def test_debounce_confirms_after_0_4_s_only():
    tr = Tracker()
    feed(tr, [(3.0, 6.0)], 0.0, 0.3)
    assert tr.counted(0.3) == []
    feed(tr, [(3.0, 6.0)], 0.4, 0.4)
    assert len(tr.counted(0.4)) == 1


def test_single_frame_false_detection_never_counts():
    tr = Tracker()
    tr.update([(3.0, 6.0)], 0.0)
    feed(tr, [], 0.1, 1.0)
    assert tr.counted(1.0) == [] and tr.events == []


def test_short_occlusion_keeps_id_long_gap_makes_new_id():
    tr = Tracker()
    feed(tr, [(3.0, 6.0)], 0.0, 1.0)
    first = tr.counted(1.0)[0].track_id
    feed(tr, [], 1.1, 1.3)                                  # 0.3 s occlusion
    assert len(tr.counted(1.3)) == 1                        # still counted (hold 0.4 s)
    feed(tr, [(3.05, 6.0)], 1.4, 2.0)
    assert [t.track_id for t in tr.counted(2.0)] == [first]
    feed(tr, [], 2.1, 2.8)                                  # 0.8 s gap: person left
    assert tr.counted(2.8) == []
    feed(tr, [(3.0, 6.0)], 2.9, 3.5)
    assert tr.counted(3.5)[0].track_id != first


def test_two_people_keep_their_ids_and_repeats_do_not_inflate():
    tr = Tracker()
    feed(tr, [(3.0, 6.0), (4.0, 6.0)], 0.0, 3.0)
    ids = sorted(t.track_id for t in tr.counted(3.0))
    assert ids == [1, 2] and tr.next_id == 3


# ------------------------------------------------------------ queue logic
def run(logic, people, t0, t1, stamp_fn=lambda t: t):
    t = t0
    while t <= t1 + 1e-9:
        logic.on_frame(people, stamp_fn(t), t)
        t = round(t + 0.1, 3)


def test_invalid_before_first_image_and_after_silence():
    q = QueueLogic()
    assert not q.state(0.0)['valid'] and q.state(0.0)['reason'] == 'NO_IMAGE'
    run(q, [(3.0, 6.0, 1.0)], 0.0, 1.0)
    assert q.state(1.0) == {'count': 1, 'confidence': 1.0, 'valid': True,
                            'crossing_occupied': False, 'stamp': 1.0, 'reason': 'OK'}
    assert q.state(2.0)['valid']                            # 1.0 s after last frame
    s = q.state(2.1)
    assert not s['valid'] and s['reason'] == 'FEED_SILENT' and s['crossing_occupied']


def test_frozen_source_stamp_detected_although_frames_arrive():
    q = QueueLogic()
    run(q, [(3.0, 6.0, 1.0)], 0.0, 1.0)
    run(q, [(3.0, 6.0, 1.0)], 1.1, 2.5, stamp_fn=lambda t: 1.0)   # same stamp again
    s = q.state(2.5)
    assert not s['valid'] and s['reason'] == 'FROZEN_STAMP'
    assert ('QUEUE_INVALID', 'FROZEN_STAMP') in q.events


def test_crossing_hold_two_seconds():
    q = QueueLogic()
    run(q, [(5.0, 3.0, 1.0)], 0.0, 1.0)
    assert q.state(1.0)['crossing_occupied']
    # Last frame with the person at 1.0 s; the track still counts for the
    # 0.4 s disappearance debounce, so C is last occupied at 1.4 s and the
    # 2.0 s clear hold ends at 3.4 s.
    run(q, [], 1.1, 3.3)
    assert q.state(3.3)['crossing_occupied']
    run(q, [], 3.4, 3.5)
    assert not q.state(3.5)['crossing_occupied']


def test_confidence_is_mean_detector_score():
    q = QueueLogic()
    run(q, [(3.0, 6.0, 0.6), (4.0, 6.0, 0.8)], 0.0, 1.0)
    assert math.isclose(q.state(1.0)['confidence'], 0.7, abs_tol=1e-9)
