"""T01 and T02 for M1-A on the full pipeline, deterministically (no ROS).

The S1 scenario from scenarios/S1.yaml runs in the simulator core; every
0.1 s the synthetic camera renders a frame, which goes through the real
detector -> homography -> tracker -> queue logic. Simulated time is used
as both the image stamp and the clock, so the run is exactly repeatable.

Assignment 8.3 pass criteria checked here:
  * stable counts 0, 3, 5, 4 after the debounce (T01);
  * repeated frames never inflate the count; a 0.3 s occlusion creates no
    new passenger (T02);
  * the crossing is occupied while a person is in C and for 2 s after;
  * the state turns invalid within 1.2 s of a camera freeze (T02).
"""
import os

import pytest

cv2 = pytest.importorskip('cv2')

from smart_hub.camera_model import PinholeCamera  # noqa: E402
from smart_hub.passenger_detector import DEFAULT_IMAGE_POINTS, DEFAULT_MAP_POINTS  # noqa: E402
from smart_hub.perception.detectors import foot, make_detector  # noqa: E402
from smart_hub.perception.homography import fit_homography, image_to_map  # noqa: E402
from smart_hub.perception.queue_logic import QueueLogic  # noqa: E402
from smart_hub.sim_core import HubSim  # noqa: E402
import yaml  # noqa: E402

FLOOR = ((2.0, 5.0, 6.0, 7.0), (4.5, 1.25, 5.5, 4.75))


def run_s1(detector='blob', until=34.0):
    with open(os.path.join(os.path.dirname(__file__), '..', 'scenarios', 'S1.yaml')) as fh:
        sim = HubSim(yaml.safe_load(fh), seed=1)
    cam, det = PinholeCamera(), make_detector(detector)
    h = fit_homography(DEFAULT_IMAGE_POINTS, DEFAULT_MAP_POINTS)
    logic = QueueLogic()
    states = {}
    while sim.t < until:
        sim.step()
        t = round(sim.t, 2)
        if sim.steps % 2 == 0 and not sim.is_frozen('camera'):          # 10 Hz camera
            img = cam.render(sim.camera_people(), FLOOR)
            people = [image_to_map(h, *foot(d, det.foot_offset)) + (d[4],)
                      for d in det.detect(img)]
            logic.on_frame(people, t, t)
        if sim.steps % 4 == 0:                                          # 5 Hz output
            states[t] = logic.state(t)
    return states, logic


@pytest.fixture(scope='module')
def s1():
    return run_s1()


def test_t01_stable_counts_0_3_5_4(s1):
    states, _ = s1
    assert [states[t]['count'] for t in (4.0, 8.0, 13.0, 18.0)] == [0, 3, 5, 4]
    assert all(states[t]['valid'] for t in (4.0, 8.0, 13.0, 18.0))


def test_t01_counts_settle_within_debounce_and_never_overshoot(s1):
    states, _ = s1
    for start, end, expected in ((5.0, 10.0, 3), (10.0, 15.0, 5), (15.0, 20.0, 4)):
        settled = [s['count'] for t, s in states.items() if start + 1.0 <= t < end]
        assert settled and set(settled) == {expected}
    assert max(s['count'] for s in states.values()) == 5     # repeated frames never inflate


def test_t02_occlusion_creates_no_new_passenger(s1):
    states, logic = s1
    during = [states[t]['count'] for t in (12.0, 12.2, 12.4, 12.8)]
    assert during == [5, 5, 5, 5]
    # 5 queue passengers + 1 crossing pedestrian = 6 people, 6 confirmed tracks
    confirmed = [e for e in logic.tracker.events if e[0] == 'TRACK_CONFIRMED']
    assert len(confirmed) == 6


def test_crossing_occupied_and_2s_clear_hold(s1):
    states, _ = s1
    assert not states[19.6]['crossing_occupied']
    assert states[20.8]['crossing_occupied']                  # after 0.4 s debounce
    assert states[23.6]['crossing_occupied']
    assert states[25.6]['crossing_occupied']                  # left at 24 s: hold 2 s
    assert not states[26.8]['crossing_occupied']


def test_t02_camera_freeze_invalid_within_1_2_s(s1):
    states, _ = s1
    assert states[30.0]['valid']
    invalid_at = min(t for t, s in states.items() if t >= 30.0 and not s['valid'])
    assert invalid_at - 30.0 <= 1.2
    assert states[invalid_at]['crossing_occupied']            # unknown -> deny entry
    assert states[invalid_at]['reason'] == 'FEED_SILENT'


def test_hog_backend_runs_on_simulator_camera():
    """HOG is the real-video backend; on the synthetic figures it is weaker.

    It is required to run and to find most people; exact counts are only
    claimed for the deterministic blob baseline.
    """
    states, _ = run_s1('hog', until=14.0)
    assert 3 <= states[13.0]['count'] <= 5
