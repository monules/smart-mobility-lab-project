"""Crossing-occupancy evaluation for labelled video (M1-A, pure Python).

One accuracy number hides the error that matters. The crossing flag has two
very different mistakes:

  false clear   a person is in C but the detector says "clear": a vehicle
                could be allowed in. This is the dangerous one.
  false busy    C is empty but the detector says "occupied": a vehicle waits
                for nothing. Safe, only slower.

and "clear" is usually the large class, so a detector that always answers
"clear" can still score a high accuracy. crossing_report() therefore returns
the full confusion matrix, the class balance, every false-clear episode with
its onset and length, and how long the detector took to notice a person
entering (and leaving) C.

Input rows are (t, truth, predicted, valid): time in seconds, hand label 0/1,
detector output 0/1 and whether the detector called its own output valid.
An invalid frame is never a "clear": following the fail-safe rule of the
safety chain it is counted as "occupied" for the vehicle's sake and reported
separately as coverage.

Time resolution: labels exist every `dt` seconds (0.5 s in the lab video), so
an episode seen on k consecutive labels lasted between (k-1)*dt and (k+1)*dt;
both bounds are reported instead of one made-up duration.
"""


def _episodes(flags, times, dt):
    """Return runs of consecutive True flags as dicts with onset and bounds."""
    out, start = [], None
    for i, flag in enumerate(list(flags) + [False]):
        if flag and start is None:
            start = i
        elif not flag and start is not None:
            k = i - start
            out.append({'onset_s': round(times[start], 2), 'labels': k,
                        'duration_min_s': round((k - 1) * dt, 2),
                        'duration_max_s': round((k + 1) * dt, 2)})
            start = None
    return out


def _latencies(truth, effective, times, edge):
    """Return, per truth transition to `edge`, how long the output took to follow.

    None for a transition the output never followed before truth changed
    back (a missed episode).
    """
    out = []
    for i in range(1, len(truth)):
        if truth[i] == edge and truth[i - 1] != edge:
            delay = None
            for j in range(i, len(truth)):
                if truth[j] != edge:
                    break
                if effective[j] == edge:
                    delay = round(times[j] - times[i], 2)
                    break
            out.append({'t_s': round(times[i], 2), 'delay_s': delay})
    return out


def crossing_report(rows, dt):
    """Return the crossing evaluation for rows = [(t, truth, predicted, valid), ...].

    "Positive" means occupied. `effective` is what a vehicle would be told:
    the detector's flag, or occupied whenever the frame is invalid.
    """
    rows = sorted(rows)
    if not rows:
        return None
    times = [r[0] for r in rows]
    truth = [int(bool(r[1])) for r in rows]
    valid = [bool(r[3]) for r in rows]
    effective = [int(bool(r[2])) if v else 1 for r, v in zip(rows, valid)]
    n = len(rows)
    tp = sum(t == 1 and e == 1 for t, e in zip(truth, effective))
    tn = sum(t == 0 and e == 0 for t, e in zip(truth, effective))
    false_busy = sum(t == 0 and e == 1 for t, e in zip(truth, effective))
    false_clear = sum(t == 1 and e == 0 for t, e in zip(truth, effective))
    occupied = tp + false_clear
    entering = _latencies(truth, effective, times, 1)
    leaving = _latencies(truth, effective, times, 0)
    noticed = [x['delay_s'] for x in entering if x['delay_s'] is not None]
    return {
        'labelled_frames': n,
        'label_period_s': dt,
        'class_balance': {'occupied': occupied, 'clear': n - occupied,
                          'occupied_share': round(occupied / n, 4)},
        'confusion': {'occupied_reported_occupied': tp, 'clear_reported_clear': tn,
                      'false_busy': false_busy, 'false_clear': false_clear},
        'accuracy': round((tp + tn) / n, 4),
        # of the frames with a person in C, how many were reported occupied
        'occupied_recall': round(tp / occupied, 4) if occupied else None,
        # of the truly empty frames, how many were reported clear
        'clear_specificity': round(tn / (n - occupied), 4) if n - occupied else None,
        'always_clear_baseline_accuracy': round((n - occupied) / n, 4),
        'false_clear_episodes': _episodes(
            [t == 1 and e == 0 for t, e in zip(truth, effective)], times, dt),
        'false_busy_episodes': _episodes(
            [t == 0 and e == 1 for t, e in zip(truth, effective)], times, dt),
        'entries': len(entering),
        'entries_never_noticed': sum(x['delay_s'] is None for x in entering),
        'entry_to_occupied_delay_s': entering,
        'entry_delay_max_s': max(noticed) if noticed else None,
        'exit_to_clear_delay_s': leaving,
        'valid_frames': sum(valid),
        'valid_coverage': round(sum(valid) / n, 4),
        'invalid_counted_as_occupied': n - sum(valid),
    }
