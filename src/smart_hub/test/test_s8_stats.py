"""Unit tests for the S8 statistics (Kaplan-Meier, paired bootstrap, co-occupancy)."""
from smart_hub.s8_stats import (cooccupancy_episodes, km_quantile, paired_bootstrap,
                                queue_waits)

RADIUS = {'bus': 0.3, 'robot': 0.2}


def in_c(x, y, r):
    """Crossing = x in [4.5, 5.5] (enough for these tests)."""
    return 4.5 - r < x < 5.5 + r


def test_km_median_counts_people_who_never_boarded():
    # 4 boarded after 10..40 s; 4 more were still waiting after 50 s.
    waits, seen = [10, 20, 30, 40, 50, 50, 50, 50], [True] * 4 + [False] * 4
    assert km_quantile(waits[:4], seen[:4]) == 20            # boarded only: looks short
    assert km_quantile(waits, seen) == 40                    # half of ALL had boarded by 40 s
    assert km_quantile([50, 50, 60], [False, False, False]) is None   # nobody boarded
    assert km_quantile([], []) is None


def test_km_without_censoring_is_the_plain_median():
    assert km_quantile([5, 1, 3], [True, True, True]) == 3
    assert km_quantile([4, 1, 3, 2], [True] * 4) == 2


def test_paired_bootstrap_is_deterministic_and_reports_sign_counts():
    res = paired_bootstrap([-2.0, -1.0, -1.5, 0.5, -0.5])
    assert res == paired_bootstrap([-2.0, -1.0, -1.5, 0.5, -0.5])
    assert res['n'] == 5 and res['mean'] == -0.9
    assert res['negative'] == 4 and res['positive'] == 1
    assert res['ci95'][0] < res['mean'] < res['ci95'][1]
    assert paired_bootstrap([1.0])['ci95'] is None
    assert paired_bootstrap([None, None])['n'] == 0


def frames(bus_xs, ped_in):
    """10 Hz truth: bus x positions and whether a pedestrian stands in C."""
    out = []
    for k, (x, ped) in enumerate(zip(bus_xs, ped_in)):
        bodies = {'bus_1': ('bus', x, 2.0)}
        bodies['p1'] = ('pedestrian', 5.0 if ped else 9.0, 3.0)
        out.append((0.1 * k, bodies))
    return out


def test_pedestrian_steps_in_and_the_vehicle_stops_in_time():
    # bus inside C and moving; pedestrian enters at t=0.5; bus stands still from t=0.8
    xs = [4.6 + 0.02 * min(k, 8) for k in range(30)]
    eps = cooccupancy_episodes(frames(xs, [k >= 5 for k in range(30)]), in_c, RADIUS)
    assert len(eps) == 1
    ep = eps[0]
    assert ep['vehicle'] == 'bus_1' and ep['onset_s'] == 0.5
    assert ep['second_to_enter'] == 'pedestrian'
    assert ep['duration_s'] == 2.4
    assert ep['moved_after_reaction_s'] == 0.0          # stopped within 0.5 s


def test_vehicle_that_keeps_moving_is_measured():
    xs = [4.6 + 0.02 * k for k in range(30)]             # 0.2 m/s the whole time
    eps = cooccupancy_episodes(frames(xs, [k >= 5 for k in range(30)]), in_c, RADIUS)
    assert eps[0]['moved_after_reaction_s'] > 1.0
    assert eps[0]['max_speed_after_reaction'] > 0.15


def test_vehicle_entering_an_occupied_crossing_is_labelled_as_second():
    xs = [3.5 + 0.1 * k for k in range(20)]              # drives in from the west
    eps = cooccupancy_episodes(frames(xs, [True] * 20), in_c, RADIUS)
    assert eps[0]['second_to_enter'] == 'vehicle'
    assert cooccupancy_episodes(frames(xs, [False] * 20), in_c, RADIUS) == []


def test_queue_waits_classifies_boarded_censored_and_left():
    def body(x):
        return ('pedestrian', x, 6.0)
    samples = []
    for k in range(101):                                 # 0 .. 10 s
        t = 0.1 * k
        bodies = {'a': body(3.0), 'c': body(3.5)}        # a boards, c waits to the end
        if t <= 4.0:
            bodies['b'] = body(4.0)                      # b walks away after 4 s
        samples.append((t, bodies))
    q = queue_waits(samples, {'a': 6.0}, lambda x, y: 2.0 <= x <= 6.0, 10.0)
    assert q == {'boarded': [6.0], 'censored': [10.0], 'left': [4.0]}
