"""Shared test helpers (pure Python, no ROS)."""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))

from smart_hub.parking_engine import ParkingConfig, ParkingEngine  # noqa: E402

EVIDENCE_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                            '..', '..', 'evidence')


def percentile(values, q):
    """Nearest-rank percentile (q in 0..100); fine for small samples."""
    if not values:
        raise ValueError('no samples')
    ordered = sorted(values)
    k = max(0, min(len(ordered) - 1, int(round(q / 100.0 * len(ordered) + 0.5)) - 1))
    return ordered[k]


def feed(engine, states, t0, duration=1.0, dt=0.2):
    """Feed identical sensor readings for ``duration`` s so the 0.5 s debounce
    commits.  ``states`` maps slot id -> 'FREE' | 'OCCUPIED' | None (no data).
    Returns the time of the last reading."""
    t = t0
    steps = int(round(duration / dt))
    for _ in range(steps + 1):
        for sid, st in states.items():
            if st is not None:
                engine.observe(sid, st == 'OCCUPIED', True, 1.0, t)
        t += dt
    return t - dt


def paper_scene(engine, t0=0.0):
    """Section 11.3 start state: P1 OCCUPIED, P2 FREE, P3 FREE, all fresh."""
    return feed(engine, {'P1': 'OCCUPIED', 'P2': 'FREE', 'P3': 'FREE'}, t0)


def make_engine(**kw):
    return ParkingEngine(ParkingConfig(**kw))
