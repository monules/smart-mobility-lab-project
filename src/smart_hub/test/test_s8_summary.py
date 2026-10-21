"""s8_summary (S8 / T15): verdict and per-mode metrics from episode files."""
import json

from smart_hub import s8_summary


def episode(mode, seed, violations=0, dispatch=60.0, waits=(10.0, 20.0)):
    return {'mode': mode, 'seed': seed, 'passed': violations == 0,
            'safety_violations': {'collisions': violations, 'crossing': 0,
                                  'double_allocation': 0, 'duplicate_dispatch': 0},
            'metrics': {'completed': True, 'bus_mission': 'SUCCEEDED',
                        'robot_mission': 'SUCCEEDED', 'parking_answered': True,
                        'queue_waits_s': list(waits), 'queue_wait_p50_s': waits[0],
                        'queue_wait_p95_s': waits[-1], 'dispatch_t_s': dispatch,
                        'dispatch_source': 'PREDICTED', 'bus_travel_s': 38.0,
                        'robot_travel_s': 80.0, 'crossing_wait_s': 5.0,
                        'forecast_valid_t_s': 59.0, 'bus_departed': True,
                        'passengers_boarded': len(waits), 'parking': [['C1', True, 'P2', 'OK']],
                        'robot_reroutes': []}}


def test_t15_needs_ten_seeds_in_both_modes_and_zero_violations(tmp_path):
    runs = [episode(m, s) for m in ('predictive', 'reactive') for s in range(1, 11)]
    summary = s8_summary.summarize(runs)
    assert summary['T15_passed'] and summary['episodes'] == 20
    assert summary['modes']['predictive']['queue_wait_p50_s'] == 10.0
    assert summary['modes']['reactive']['completion_rate'] == 1.0
    runs[3] = episode('predictive', 4, violations=1)       # one collision anywhere fails T15
    bad = s8_summary.summarize(runs)
    assert not bad['T15_passed'] and bad['safety_violations_total']['collisions'] == 1
    assert not s8_summary.summarize(runs[:10])['T15_passed']   # one mode only


def test_files_are_written_and_read_back(tmp_path):
    for m in ('predictive', 'reactive'):
        for s in (1, 2):
            d = tmp_path / m / ('seed%d' % s)
            d.mkdir(parents=True)
            (d / 's8_episode.json').write_text(json.dumps(episode(m, s, dispatch=50.0 + s)))
    runs = s8_summary.load(str(tmp_path))
    summary = s8_summary.summarize(runs)
    s8_summary.write(str(tmp_path), runs, summary)
    assert (tmp_path / 's8_runs.csv').read_text().count('\n') == 5
    assert summary['paired'][1]['dispatch_t_reactive'] == 52.0
    assert (tmp_path / 's8_compare.png').exists()
