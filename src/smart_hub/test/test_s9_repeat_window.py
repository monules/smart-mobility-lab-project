"""When a fault ends for the "no motion during the fault" check (M5, S9x).

s9_repeat_check counts nonzero commands between the first zero command and
the end of the fault. A freeze lasts 2 s of simulator time, which can end a
little before 2 s of wall-clock time; the robot then rightly moves again.
These tests pin the rule: for a freeze, the window ends at the first message
after the silence; for the other faults, at the checker's own end time.
"""
import threading
import types

from smart_hub.s9_repeat_check import S9Repeat


def fake(arrivals):
    """Return a stand-in node with only the fields fault_window_end reads."""
    return types.SimpleNamespace(lock=threading.Lock(),
                                 arrivals={'odom': [], 'vehicles': [], **arrivals})


def every(start, stop, step=0.1):
    out, t = [], start
    while t < stop - 1e-9:
        out.append(round(t, 4))
        t += step
    return out


def test_freeze_window_ends_when_the_frozen_topic_is_heard_again():
    # seed 120: fault at 10.000, vehicles silent, back at 11.975 (sim time ran ahead)
    node = fake({'vehicles': every(8.0, 10.0) + every(11.975, 13.0)})
    end = S9Repeat.fault_window_end(node, 'vehicles_freeze', t0=10.0, t_end=12.0)
    assert end == 11.975


def test_window_never_goes_past_the_checkers_end_time():
    node = fake({'odom': every(8.0, 10.0, 0.05) + every(12.3, 13.0, 0.05)})
    assert S9Repeat.fault_window_end(node, 'odom_freeze', t0=10.0, t_end=12.0) == 12.0


def test_without_a_resume_the_window_is_the_full_fault():
    node = fake({'vehicles': every(8.0, 10.0)})
    assert S9Repeat.fault_window_end(node, 'vehicles_freeze', t0=10.0, t_end=12.0) == 12.0


def test_normal_message_spacing_is_not_mistaken_for_a_resume():
    node = fake({'vehicles': every(8.0, 13.0)})            # never frozen
    assert S9Repeat.fault_window_end(node, 'vehicles_freeze', t0=10.0, t_end=12.0) == 12.0


def test_faults_ended_by_the_checker_use_its_end_time():
    node = fake({'vehicles': every(8.0, 10.0) + every(11.0, 13.0)})
    for fault in ('estop', 'candidate_loss', 'estop_signal'):
        assert S9Repeat.fault_window_end(node, fault, t0=10.0, t_end=12.0) == 12.0
