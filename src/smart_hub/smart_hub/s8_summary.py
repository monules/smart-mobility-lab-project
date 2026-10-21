"""s8_summary (M5): T15 verdict and predictive-vs-reactive comparison for S8.

    ros2 run smart_hub s8_summary --root output/s8

Reads <root>/<mode>/seed<k>/s8_episode.json for both dispatch modes and
writes <root>/s8_runs.csv (one row per episode), <root>/s8_summary.json and
<root>/s8_compare.png. T15 (15.3) passes only if every episode has zero
collisions, zero overlapping crossing ownership, zero double allocations
and zero duplicate dispatches; the operating metrics are reported per mode
and paired by seed, and are never traded against a safety violation.
"""
import argparse
import csv
import glob
import json
import math
import os
import sys

MODES = ('predictive', 'reactive')


def pct(values, q):
    if not values:
        return None
    s = sorted(values)
    return round(s[max(0, math.ceil(q * len(s)) - 1)], 2)


def mean(values):
    values = [v for v in values if v is not None]
    return round(sum(values) / len(values), 2) if values else None


def load(root):
    runs = []
    for path in sorted(glob.glob(os.path.join(root, '*', 'seed*', 's8_episode.json'))):
        with open(path) as fh:
            runs.append(json.load(fh))
    return runs


def summarize(runs):
    out = {'episodes': len(runs), 'modes': {}}
    total = {k: 0 for k in ('collisions', 'crossing', 'double_allocation',
                            'duplicate_dispatch')}
    for r in runs:
        for k, n in r['safety_violations'].items():
            total[k] += n
    out['safety_violations_total'] = total
    for mode in MODES:
        rs = [r for r in runs if r['mode'] == mode]
        if not rs:
            continue
        m = [r['metrics'] for r in rs]
        waits = [w for x in m for w in x.get('queue_waits_s', [])]
        out['modes'][mode] = {
            'episodes': len(rs),
            'safe_episodes': sum(r['passed'] for r in rs),
            'completion_rate': round(sum(x['completed'] for x in m) / len(m), 2),
            'bus_mission_success': sum(x['bus_mission'] == 'SUCCEEDED' for x in m),
            'robot_mission_success': sum(x['robot_mission'] == 'SUCCEEDED' for x in m),
            'parking_answered': sum(x['parking_answered'] for x in m),
            'queue_wait_p50_s': pct(waits, 0.5),
            'queue_wait_p95_s': pct(waits, 0.95),
            'passengers_boarded': len(waits),
            'mean_dispatch_t_s': mean([x['dispatch_t_s'] for x in m]),
            'dispatch_sources': sorted({x['dispatch_source'] for x in m}),
            'mean_bus_travel_s': mean([x['bus_travel_s'] for x in m]),
            'mean_robot_travel_s': mean([x['robot_travel_s'] for x in m]),
            'mean_crossing_wait_s': mean([x['crossing_wait_s'] for x in m]),
        }
    seeds = sorted({r['seed'] for r in runs})
    by = {(r['mode'], r['seed']): r['metrics'] for r in runs}
    out['paired'] = [{'seed': s,
                      'dispatch_t_predictive': by.get(('predictive', s), {}).get('dispatch_t_s'),
                      'dispatch_t_reactive': by.get(('reactive', s), {}).get('dispatch_t_s'),
                      'wait_p50_predictive': by.get(('predictive', s), {}).get('queue_wait_p50_s'),
                      'wait_p50_reactive': by.get(('reactive', s), {}).get('queue_wait_p50_s')}
                     for s in seeds]
    out['T15_passed'] = bool(runs) and all(r['passed'] for r in runs) and \
        len(runs) == 2 * len(seeds) and len(seeds) >= 10
    return out


def write(root, runs, summary):
    keys = ['mode', 'seed', 'passed', 'collisions', 'crossing', 'double_allocation',
            'duplicate_dispatch', 'completed', 'dispatch_t_s', 'dispatch_source',
            'forecast_valid_t_s', 'bus_mission', 'bus_travel_s', 'bus_departed',
            'robot_mission', 'robot_travel_s', 'passengers_boarded', 'queue_wait_p50_s',
            'queue_wait_p95_s', 'crossing_wait_s', 'parking', 'robot_reroutes']
    with open(os.path.join(root, 's8_runs.csv'), 'w', newline='') as fh:
        w = csv.DictWriter(fh, fieldnames=keys)
        w.writeheader()
        for r in sorted(runs, key=lambda r: (r['mode'], r['seed'])):
            row = dict(r['metrics'], **r['safety_violations'])
            row.update(mode=r['mode'], seed=r['seed'], passed=r['passed'])
            row['parking'] = ' '.join('%s:%s' % (p[0], p[2] or p[3]) for p in row['parking'])
            row['robot_reroutes'] = ' | '.join(row['robot_reroutes'])
            w.writerow({k: row.get(k) for k in keys})
    with open(os.path.join(root, 's8_summary.json'), 'w') as fh:
        json.dump(summary, fh, indent=2)
    plot(root, summary)


def plot(root, summary):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    paired = summary['paired']
    if not paired:
        return
    seeds = [p['seed'] for p in paired]
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(10, 3.6))
    for mode, mk in (('predictive', 'o'), ('reactive', 's')):
        a1.plot(seeds, [p['dispatch_t_%s' % mode] for p in paired], mk + '-', label=mode)
        a2.plot(seeds, [p['wait_p50_%s' % mode] for p in paired], mk + '-', label=mode)
    a1.set_title('bus dispatch time (scenario s)', fontsize=9)
    a2.set_title('median queue wait until boarding (s)', fontsize=9)
    for a in (a1, a2):
        a.set_xlabel('seed')
        a.set_xticks(seeds)
        a.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(os.path.join(root, 's8_compare.png'), dpi=120)
    plt.close(fig)


def main(args=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--root', default='output/s8')
    a = ap.parse_args(args)
    runs = load(a.root)
    summary = summarize(runs)
    write(a.root, runs, summary)
    print(json.dumps(summary, indent=2))
    print('T15 %s' % ('PASSED' if summary['T15_passed'] else 'FAILED'))
    sys.exit(0 if summary['T15_passed'] else 1)


if __name__ == '__main__':
    main()
