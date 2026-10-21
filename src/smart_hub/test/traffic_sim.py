"""Deterministic synthetic /hub/vehicles generator + node-like publisher
harness for the S5 tests (no ROS).  Seeded, fixed 10 Hz feed, 2 Hz publication,
10 Hz stale watchdog - the same cadence as traffic_monitor.py."""
import math
import random

from helpers import *  # noqa: F401,F403  (sets sys.path)
from smart_hub.hub_map import NODES, edge_endpoints, normalize_edge_id
from smart_hub.traffic_engine import (BLOCKED, CONGESTED, NORMAL, UNKNOWN,
                                      TrafficConfig, TrafficEngine, VehicleObs)

DT = 0.1                       # 10 Hz vehicle feed
PUBLISH_PERIOD = 0.5           # 2 Hz /traffic/edges


def on_edge(edge, frac):
    (ax, ay), (bx, by) = edge_endpoints(normalize_edge_id(edge))
    return ax + frac * (bx - ax), ay + frac * (by - ay)


class Mover:
    """Vehicle that drives along one edge at constant speed and re-spawns at the
    start when it reaches the end (keeps the occupancy constant)."""

    def __init__(self, vid, edge, speed, start_frac=0.0, kind='car', claimed_edge=None,
                 active_from=0.0, active_until=math.inf):
        self.vid, self.edge, self.speed, self.kind = vid, edge, speed, kind
        self.start_frac = start_frac
        self.claimed_edge = claimed_edge or edge
        self.active_from, self.active_until = active_from, active_until
        (ax, ay), (bx, by) = edge_endpoints(normalize_edge_id(edge))
        self.length = math.hypot(bx - ax, by - ay)

    def active(self, t):
        return self.active_from <= t < self.active_until

    def sample(self, t, rng, noise):
        travelled = self.speed * max(0.0, t - self.active_from)
        frac = (self.start_frac + travelled / self.length) % 1.0
        x, y = on_edge(self.edge, frac)
        speed = self.speed
        if noise:
            speed = max(0.0, speed + rng.gauss(0.0, 0.02))
            x += rng.gauss(0.0, 0.03)
            y += rng.gauss(0.0, 0.03)
        return VehicleObs(self.vid, self.kind, x, y, speed, self.claimed_edge, True)


class Obstacle:
    kind = 'obstacle'

    def __init__(self, x, y, active_from):
        self.x, self.y, self.active_from = x, y, active_from

    def active(self, t):
        return t >= self.active_from

    def sample(self, t, rng, noise):
        return VehicleObs('O', 'obstacle', self.x, self.y, 0.0, '', True)


def run_feed(movers, duration, seed=1, noise=True, dropout=0.03, freeze=None,
             config=None, extra=None):
    """Simulate ``duration`` s.  ``freeze=(t0, t1)`` repeats the last good
    snapshot (same stamp) during [t0, t1) to model a frozen source.
    Returns (ticks, engine) where ticks is a list of dicts
    {t, labels:{edge:label}, results:{edge:EdgeResult}, reason}."""
    rng = random.Random(seed)
    engine = TrafficEngine(config or TrafficConfig())
    ticks = []
    published_stale = False
    last_stamp = None
    steps = int(round(duration / DT))
    next_pub = PUBLISH_PERIOD
    for i in range(steps + 1):
        t = round(i * DT, 6)
        frozen = freeze is not None and freeze[0] <= t < freeze[1]
        if not frozen:
            snap = []
            for m in movers:
                if m.active(t) and (m.kind == 'obstacle' or rng.random() >= dropout):
                    snap.append(m.sample(t, rng, noise))
            if extra:
                snap += extra(t)
            last_stamp = t
            engine.ingest(t, t, snap, source_age_s=0.02)
        else:
            engine.ingest(last_stamp, t, [], source_age_s=0.02)    # same stamp again
        stale = engine.is_stale(t)
        publish = False
        if t + 1e-9 >= next_pub:
            publish = True
            next_pub += PUBLISH_PERIOD
        elif stale and not published_stale:                       # watchdog path
            publish = True
        if publish:
            res = engine.evaluate(t)
            published_stale = all(r.label == UNKNOWN for r in res)
            ticks.append({'t': t, 'labels': {r.edge_id: r.label for r in res},
                          'results': {r.edge_id: r for r in res}})
        elif not stale:
            published_stale = False
    return ticks, engine


def first_time(ticks, edge, label, after=0.0):
    for k in ticks:
        if k['t'] >= after and k['labels'][edge] == label:
            return k['t']
    return None
