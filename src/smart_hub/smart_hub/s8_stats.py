"""Statistics for the S8 / T15 evaluation (M2-B + M5, pure Python, no ROS).

Three things the midterm review asked for:

1. Waiting times that do not ignore people who never boarded. A passenger
   still in the queue when the episode ends has waited AT LEAST that long
   (a right-censored observation). Reporting only boarded passengers makes
   a policy look better the more people it leaves behind. km_quantile() is
   the Kaplan-Meier estimate, which uses both kinds of observation.

2. A paired comparison of two dispatch policies. Both policies run on the
   same seeds, so the unit of evidence is the per-seed difference, and the
   uncertainty is found by resampling whole episodes (paired_bootstrap()),
   never individual passengers of one episode (they are not independent).

3. Physical co-occupancy of crossing C. cooccupancy_episodes() lists every
   stretch of time in which a pedestrian and a vehicle footprint were in C
   together, who came second, and whether the vehicle kept moving after a
   reaction time. A pedestrian may step into C while a vehicle is inside
   (pedestrians are not controlled); the vehicle must then stand still.
   Both cases are recorded; neither is hidden behind a narrow event.
"""
import math
import random


def km_quantile(durations, observed, q=0.5):
    """Return the Kaplan-Meier q-quantile of a waiting time, or None.

    durations[i] is the time passenger i was seen waiting; observed[i] is
    True if the wait ended (boarded) and False if it was cut off (censored).
    None means the estimate is not reached: fewer than a share q of the
    passengers are known to have finished waiting within the data.
    """
    data = sorted(zip(durations, observed))
    at_risk, survival = len(data), 1.0
    i = 0
    while i < len(data):
        t = data[i][0]
        same = [o for d, o in data[i:] if d == t]
        ended = sum(same)
        if ended:
            survival *= 1.0 - ended / at_risk
            if survival <= 1.0 - q:
                return round(t, 2)
        at_risk -= len(same)
        i += len(same)
    return None


def paired_bootstrap(diffs, resamples=5000, seed=15):
    """Return mean and 95 % interval of per-episode differences.

    Whole episodes are resampled (deterministic for a given seed). With
    fewer than two differences there is no interval to report.
    """
    diffs = [d for d in diffs if d is not None]
    if not diffs:
        return {'n': 0, 'mean': None, 'ci95': None, 'negative': 0, 'positive': 0}
    rng = random.Random(seed)
    n = len(diffs)
    means = sorted(sum(rng.choice(diffs) for _ in range(n)) / n for _ in range(resamples))
    ci = [round(means[int(0.025 * resamples)], 3), round(means[int(0.975 * resamples) - 1], 3)]
    return {'n': n, 'mean': round(sum(diffs) / n, 3), 'ci95': ci if n > 1 else None,
            'negative': sum(d < 0 for d in diffs), 'positive': sum(d > 0 for d in diffs)}


def cooccupancy_episodes(samples, in_crossing, radius, reaction_s=0.5, stopped=0.02):
    """List every period a pedestrian and a vehicle footprint shared crossing C.

    samples: [(t, {id: (kind, x, y)})] ground truth in time order; vehicles
    are every kind in `radius` (a dict kind -> footprint radius), pedestrians
    have kind 'pedestrian'. in_crossing(x, y, r) says whether a circle
    overlaps C. Per episode: the vehicle, onset, duration, who entered
    second ('pedestrian', 'vehicle' or 'both' when one sample shows both),
    and how long and how fast the vehicle moved later than reaction_s after
    the onset (speed from consecutive positions; above `stopped` m/s).
    """
    episodes, open_ep = [], {}
    prev_t, prev_pos, prev_in, prev_ped = None, {}, set(), False
    for t, bodies in samples:
        ped = any(k == 'pedestrian' and in_crossing(x, y, 0.0) for k, x, y in bodies.values())
        inside = {i for i, (k, x, y) in bodies.items()
                  if k in radius and in_crossing(x, y, radius[k])}
        for i in sorted(inside):
            if not ped:
                continue
            if i not in open_ep:
                second = ('both' if i not in prev_in and not prev_ped else
                          'vehicle' if i not in prev_in else 'pedestrian')
                open_ep[i] = {'vehicle': i, 'onset_s': round(t, 2), 'end': t,
                              'second_to_enter': second, 'moved_after_reaction_s': 0.0,
                              'max_speed_after_reaction': 0.0, '_t0': t}
            ep = open_ep[i]
            ep['end'] = t
            if prev_t is not None and i in prev_pos and t - ep['_t0'] > reaction_s:
                dt = t - prev_t
                x, y = bodies[i][1], bodies[i][2]
                speed = math.hypot(x - prev_pos[i][0], y - prev_pos[i][1]) / dt if dt > 0 else 0
                if speed > stopped:
                    ep['moved_after_reaction_s'] += dt
                    ep['max_speed_after_reaction'] = max(ep['max_speed_after_reaction'], speed)
        for i in [i for i in open_ep if i not in inside or not ped]:
            episodes.append(_close(open_ep.pop(i)))
        prev_t, prev_in, prev_ped = t, inside, ped
        prev_pos = {i: (b[1], b[2]) for i, b in bodies.items()}
    episodes += [_close(ep) for ep in open_ep.values()]
    return sorted(episodes, key=lambda e: (e['onset_s'], e['vehicle']))


def _close(ep):
    ep['duration_s'] = round(ep.pop('end') - ep.pop('_t0'), 2)
    ep['moved_after_reaction_s'] = round(ep['moved_after_reaction_s'], 2)
    ep['max_speed_after_reaction'] = round(ep['max_speed_after_reaction'], 3)
    return ep


def queue_waits(samples, boarded_at, in_queue, end_t, gone_after=1.0):
    """Classify every passenger seen in the queue zone.

    Returns {'boarded': [wait], 'censored': [wait so far], 'left': [time in
    queue]}: boarded = PASSENGER_BOARDED seen (boarded_at[id] = time);
    left = walked out of the zone without boarding more than gone_after
    seconds before the end; censored = still waiting when the episode ended.
    """
    first, last = {}, {}
    for t, bodies in samples:
        for i, (kind, x, y) in bodies.items():
            if kind == 'pedestrian' and in_queue(x, y):
                first.setdefault(i, t)
                last[i] = t
    out = {'boarded': [], 'censored': [], 'left': []}
    for i, t0 in first.items():
        if i in boarded_at:
            out['boarded'].append(round(boarded_at[i] - t0, 2))
        elif last[i] < end_t - gone_after:
            out['left'].append(round(last[i] - t0, 2))
        else:
            out['censored'].append(round(end_t - t0, 2))
    return {k: sorted(v) for k, v in out.items()}
