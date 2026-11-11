"""Unit tests for the Hub Control Center core (catalog, intent, runs; no ROS)."""
import io
import json
import os
import tempfile
import threading
import time

import pytest
from smart_hub.control_center import catalog, intent
from smart_hub.control_center.live_state import LiveState, static_map
from smart_hub.control_center.runs import read_verdict, RunManager


# ---------------------------------------------------------------- catalog
def test_every_task_has_a_way_to_run_and_unique_id():
    ids = [t['id'] for t in catalog.TASKS]
    assert len(ids) == len(set(ids))
    for t in catalog.TASKS:
        assert bool(t['launch']) != bool(t['command']), t['id']
        values = catalog.resolve_options(t, {})
        assert catalog.build_command(t, values, 'output/ui/x')


def test_launch_files_and_unit_suites_exist():
    here = os.path.dirname(os.path.abspath(__file__))
    for t in catalog.TASKS:
        if t['launch']:
            assert os.path.exists(os.path.join(here, '..', 'launch', t['launch'])), t['launch']
    for files in catalog.UNIT_SUITES.values():
        for f in files:
            assert os.path.exists(os.path.join(here, f)), f


def test_options_are_validated_and_defaults_filled():
    s8 = catalog.BY_ID['S8']
    assert catalog.resolve_options(s8, {'seed': '3'}) == {'seed': 3, 'dispatch': 'predictive'}
    with pytest.raises(ValueError):
        catalog.resolve_options(s8, {'seed': 500})
    with pytest.raises(ValueError):
        catalog.resolve_options(s8, {'dispatch': 'chaos; rm -rf /'})
    with pytest.raises(ValueError):
        catalog.resolve_options(s8, {'nominal_trials': 3})


def test_build_command_is_an_argument_list_with_output_dir():
    s8 = catalog.BY_ID['S8']
    cmd = catalog.build_command(s8, {'seed': 3, 'dispatch': 'reactive'}, 'output/ui/r1')
    assert cmd[:4] == ['ros2', 'launch', 'smart_hub', 's8.launch.py']
    assert 'output_dir:=output/ui/r1' in cmd and 'dispatch:=reactive' in cmd
    s2 = catalog.BY_ID['S2']
    cmd = catalog.build_command(s2, {'seed': 7, 'run_for': 90}, 'output/ui/r2')
    assert 'seed:=7' in cmd and not any(a.startswith('run_for') for a in cmd)
    real = catalog.BY_ID['S1-real']
    cmd = catalog.build_command(real, {'detector': 'hog', 'stabilize': False}, 'o')
    assert '--stabilize' not in cmd and '--hog-upscale' in cmd


def test_public_catalog_marks_missing_inputs():
    with tempfile.TemporaryDirectory() as root:
        real = next(t for t in catalog.public_catalog(root) if t['id'] == 'S1-real')
        assert not real['available'] and catalog.REAL_VIDEO in real['missing']


# ----------------------------------------------------------------- intent
@pytest.mark.parametrize('text, tasks', [
    ('S8', ['S8']),
    ('run T13', ['S7']),
    ('t07 please', ['S4']),
    ('S9x', ['S9x']),
    ('rush hour reactive seed 3', ['S8']),
    ('robot escort with 2 trials then S9', ['S7', 'S9']),
    ('parking then congestion', ['S4', 'S5']),
    ('test the e-stop', ['S9']),
    ('real video without stabilisation', ['S1-real']),
    ('retrain the demand model on the queue target', ['T03']),
    ('unit tests for routing', ['UNIT']),
])
def test_intent_picks_tasks(text, tasks):
    assert [p['task'] for p in intent.parse(text)['plan']] == tasks


def test_intent_reads_options_for_the_chosen_task_only():
    plan = intent.parse('rush hour reactive seed 3')['plan'][0]
    assert plan['options'] == {'seed': 3, 'dispatch': 'reactive'}
    plan = intent.parse('robot escort with 2 trials')['plan'][0]
    assert plan['options']['nominal_trials'] == 2
    plan = intent.parse('real video without stabilisation using hog')['plan'][0]
    assert plan['options'] == {'detector': 'hog', 'stabilize': False}
    plan = intent.parse('unit tests for routing')['plan'][0]
    assert plan['options']['suite'] == 'routing and robot (M4)'
    plan = intent.parse('demand demo for 2 minutes')['plan'][0]
    assert plan['options']['run_for'] == 120


def test_intent_out_of_range_value_falls_back_with_a_note():
    plan = intent.parse('S8 seed 500')['plan'][0]
    assert plan['options']['seed'] == 1 and plan['notes']


def test_intent_help_and_no_match():
    assert intent.parse('what can you do')['help']
    res = intent.parse('make me a sandwich')
    assert not res['plan'] and res['unmatched'] == ['make me a sandwich']
    assert intent.parse('')['help']


# --------------------------------------------------------------- verdicts
def _write(d, name, data):
    os.makedirs(d, exist_ok=True)
    with open(os.path.join(d, name), 'w') as fh:
        json.dump(data, fh)


def test_verdict_from_checker_file_not_exit_code():
    with tempfile.TemporaryDirectory() as d:
        _write(d, 's4_check.json', {'passed': False, 'checks': [
            {'check': 'a', 'passed': True, 'detail': 'x'}, {'check': 'b', 'passed': False}]})
        status, checks, summary = read_verdict(catalog.BY_ID['S4'], d, 0, [], False)
        assert status == 'failed' and summary == '1/2 checks passed'
        status, _, summary = read_verdict(catalog.BY_ID['S5'], d, 0, [], False)
        assert status == 'error' and 's5_check.json' in summary


def test_verdict_s8_episode_and_repeat():
    with tempfile.TemporaryDirectory() as d:
        _write(d, 's8_episode.json', {'passed': True, 'safety_violations': {'collisions': 0},
                                      'metrics': {'bus_mission': 'SUCCEEDED',
                                                  'robot_mission': 'SUCCEEDED'}})
        status, checks, _ = read_verdict(catalog.BY_ID['S8'], d, 0, [], False)
        assert status == 'passed' and all(c['passed'] for c in checks)
        _write(d, 's9_repeat.json', {'passed': True, 'summary': {'estop': {
            'trials': 5, 'passed': 5, 'zero_latency_max_s': 0.001, 'latency_limit_s': 0.2}}})
        status, checks, _ = read_verdict(catalog.BY_ID['S9x'], d, 0, [], False)
        assert status == 'passed' and checks[0]['detail'].startswith('5/5')


def test_verdict_timer_and_pytest():
    status, _, _ = read_verdict(catalog.BY_ID['S2'], '/nonexistent', -2, [], True)
    assert status == 'passed'
    status, _, _ = read_verdict(catalog.BY_ID['S2'], '/nonexistent', 1, [], False)
    assert status == 'failed'
    status, checks, _ = read_verdict(catalog.BY_ID['UNIT'], '/x', 0, ['5 passed in 0.1s'], False)
    assert status == 'passed' and '5 passed' in checks[0]['detail']


# -------------------------------------------------------------- run queue
class FakeProc:
    """Stand-in for Popen: prints lines, writes the checker file, exits."""

    def __init__(self, cmd, cwd, result, **kwargs):
        out = [a.split(':=', 1)[1] for a in cmd if a.startswith('output_dir:=')][0]
        _write(os.path.join(cwd, out), 's4_check.json', result)
        self.stdout = io.StringIO('starting\nS4 verdict\n')
        self.returncode = None
        self.pid = -1

    def wait(self):
        self.returncode = 0

    def poll(self):
        return self.returncode


def _wait_for(pred, timeout=5.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if pred():
            return True
        time.sleep(0.02)
    return False


def test_run_manager_runs_in_order_and_keeps_history():
    with tempfile.TemporaryDirectory() as root:
        result = {'passed': True, 'checks': [{'check': 'ok', 'passed': True}]}
        rm = RunManager(root, popen=lambda cmd, cwd, **kw: FakeProc(cmd, cwd, result, **kw))
        first = rm.submit('S4', {'seed': 4})
        second = rm.submit('S4', {'seed': 5}, source='ask')
        assert first['status'] == 'queued' and first['id'] != second['id']
        assert _wait_for(lambda: len(rm.snapshot()['history']) == 2)
        hist = rm.snapshot()['history']
        assert [h['options']['seed'] for h in hist] == [5, 4]      # newest first
        assert all(h['status'] == 'passed' for h in hist)
        assert 'S4 verdict' in rm.log(first['id'])
        again = RunManager(root, popen=None)                         # history survives
        assert len(again.history) == 2


def test_run_manager_rejects_bad_requests_and_cancels_queued():
    with tempfile.TemporaryDirectory() as root:
        gate = threading.Event()

        class SlowProc(FakeProc):
            def wait(self):
                gate.wait(5)
                self.returncode = 0

        rm = RunManager(root, popen=lambda cmd, cwd, **kw: SlowProc(
            cmd, cwd, {'passed': True, 'checks': []}, **kw))
        with pytest.raises(ValueError):
            rm.submit('S42')
        with pytest.raises(ValueError):
            rm.submit('S4', {'seed': -1})
        rm.submit('S4')
        queued = rm.submit('S4', {'seed': 9})
        assert _wait_for(lambda: rm.snapshot()['active'] is not None)
        assert rm.stop(queued['id'])
        gate.set()
        assert _wait_for(lambda: len(rm.snapshot()['history']) == 2)
        assert {h['status'] for h in rm.snapshot()['history']} == {'cancelled', 'passed'}


# ------------------------------------------------------------- live state
def test_live_state_marks_stale_values_and_keeps_events():
    live = LiveState()
    live.on_queue(3, 0.9, False, True, now=10.0)
    live.on_demand(4.2, 3.0, 30, 'm', True, now=10.0)
    live.on_event('robot_1', 'ESCORT_STARTED', '', 1.0, now=10.0)
    snap = live.snapshot(now=11.5)
    assert snap['queue']['stale'] and not snap['demand']['stale']
    assert snap['events'][0]['event'] == 'ESCORT_STARTED'
    assert 'robot_1' in snap['missions']
    live.reset()
    assert 'queue' not in live.snapshot(now=12.0)
    assert set(static_map()) >= {'site', 'crossing', 'nodes', 'edges'}
