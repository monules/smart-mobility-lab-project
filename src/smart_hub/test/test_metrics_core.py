"""Unit tests for MetricsCore (stop latency, violations, file output)."""
import csv
import json

from smart_hub.metrics_core import MetricsCore


def test_stop_latency_and_physical_stop():
    m = MetricsCore('run-1', 'S9', 9, 'abc123')
    m.on_final_command(10.00, 'robot', 0.2, 0.0)
    m.on_estop(10.10, True)
    m.on_final_command(10.13, 'robot', 0.2, 0.0)   # still in flight
    m.on_final_command(10.15, 'robot', 0.0, 0.0)   # first zero
    m.on_final_command(10.20, 'robot', 0.0, 0.0)
    m.on_truth(10.20, 'robot', 0.1)
    m.on_truth(10.30, 'robot', 0.0)
    s = m.summary()
    assert s['stop_latency'] == [{'agent': 'robot', 'seconds': 0.05}]
    assert s['stop_latency_max_sec'] == 0.05
    assert s['physical_stop'] == [{'agent': 'robot', 'seconds': 0.2}]


def test_only_rising_edge_starts_a_measurement():
    m = MetricsCore('run-1')
    m.on_estop(1.0, True)
    m.on_final_command(1.01, 'robot', 0.0, 0.0)
    m.on_estop(1.1, True)                          # heartbeat repeat, no new edge
    m.on_final_command(1.2, 'robot', 0.0, 0.0)
    assert len(m.summary()['stop_latency']) == 1


def test_motion_while_latched_counted_after_grace():
    m = MetricsCore('run-1')
    m.on_safety(5.0, True)
    m.on_final_command(5.05, 'robot', 0.2, 0.0)    # inside ordering grace
    m.on_final_command(5.30, 'robot', 0.2, 0.0)    # real violation
    m.on_final_command(5.35, 'robot', 0.0, 0.0)
    m.on_safety(6.0, False)
    m.on_final_command(6.5, 'robot', 0.2, 0.0)     # allowed after clear
    assert m.summary()['motion_while_latched'] == 1


def test_interventions_and_files(tmp_path):
    m = MetricsCore('run-9', 'S9', 9, 'abc123')
    m.on_event(1.0, 'robot', 'SAFETY_HOLD', 'COMMAND_STALE', 'robot:COMMAND_STALE')
    m.on_event(2.0, 'robot', 'SAFETY_HOLD', 'COMMAND_STALE', 'robot:COMMAND_STALE')
    m.on_event(3.0, 'safety_supervisor', 'STOP_LATCHED', 'ESTOP', 'stop-1')
    m.on_event(3.5, 'robot', 'SAFETY_RELEASE', 'COMMAND_STALE', 'robot:COMMAND_STALE')
    summary = m.write(str(tmp_path / 'run-9'))
    assert summary['interventions'] == {'SAFETY_HOLD:COMMAND_STALE': 2,
                                        'STOP_LATCHED:ESTOP': 1}
    assert summary['duration_sec'] == 2.5
    with open(tmp_path / 'run-9' / 'summary.json') as fh:
        assert json.load(fh)['commit'] == 'abc123'
    with open(tmp_path / 'run-9' / 'events.csv') as fh:
        rows = list(csv.DictReader(fh))
    assert [r['event'] for r in rows] == ['SAFETY_HOLD', 'SAFETY_HOLD', 'STOP_LATCHED',
                                          'SAFETY_RELEASE']
