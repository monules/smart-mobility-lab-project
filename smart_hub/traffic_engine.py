"""Per-edge speed / density / congestion estimation for Scenario S5 (no ROS).

Pipeline per /hub/vehicles snapshot (10 Hz):
  1. snapshot gate: stamp must advance (a repeated stamp is a frozen source),
     source age must be sane, otherwise the snapshot is NOT fresh;
  2. per-vehicle gate: finite pose, plausible speed, a map edge id, and the pose
     must lie inside that edge's corridor (no silent re-assignment);
  3. obstacles (kind == 'obstacle') are intersected with edge corridors.

Every 0.5 s (2 Hz) :meth:`TrafficEngine.evaluate` produces one label per edge:

  UNKNOWN    no fresh data (gap > stale_sec = 1.0 s), warm-up, or a sensor reports
             valid=false on this edge for longer than stale_sec
  BLOCKED    an obstacle intersects the edge corridor (immediate)
  CONGESTED  rho >= 0.70, OR mean speed < 40 % of free-flow for 3 publications
  NORMAL     otherwise (including an empty road with a live feed)

Definitions (documented design interpretation of assignment 12.2):
  n_e   vehicles whose latest accepted sample lies on edge e and is at most
        ``presence_hold_sec`` old (tolerates single dropped frames);
  K_e   floor(length / safe_headway), at least 1 (or an explicit override);
  rho_e min(1, n_e / K_e);
  speed per vehicle = median of its samples on e in the 5 s window; edge mean =
        mean over present vehicles ("robust": spikes and impossible samples are
        rejected or median-filtered);
  empty edge with a live feed -> valid NORMAL at free-flow speed, so the route
        planner never sees a zero or infinite travel time for an empty road.
"""
from __future__ import annotations

import math
import statistics
import threading
from collections import Counter, deque
from dataclasses import dataclass, field
from typing import Deque, Dict, List, Optional, Sequence, Tuple

from .hub_map import (VEHICLE_EDGES, edge_endpoints, edge_length,
                      normalize_edge_id, point_segment_distance)

NORMAL = 'NORMAL'
CONGESTED = 'CONGESTED'
BLOCKED = 'BLOCKED'
UNKNOWN = 'UNKNOWN'


@dataclass(frozen=True)
class VehicleObs:
    vehicle_id: str
    kind: str
    x: float
    y: float
    speed: float
    edge_id: str
    valid: bool = True


@dataclass
class IngestReport:
    snapshot_ok: bool = False
    snapshot_reason: str = ''
    accepted: int = 0
    rejected: List[Tuple[str, str]] = field(default_factory=list)


@dataclass(frozen=True)
class EdgeResult:
    edge_id: str
    label: str
    valid: bool
    blocked: bool
    mean_speed_mps: float        # -1.0 when invalid
    density_ratio: float         # -1.0 when invalid
    count: int
    capacity: int
    obs_age_sec: float
    prev_label: str
    changed: bool


@dataclass
class TrafficConfig:
    edges: Tuple[str, ...] = VEHICLE_EDGES
    window_sec: float = 5.0
    stale_sec: float = 1.0               # gap > this -> UNKNOWN; must be <= 1.2 (assignment 12.3)
    presence_hold_sec: float = 1.0
    max_source_age_sec: float = 1.0
    future_tolerance_sec: float = 0.5
    clock_reset_sec: float = 5.0         # stamp jumping back this far = sim reset
    corridor_halfwidth_m: float = 0.5    # vehicle radius 0.30 + clearance 0.20
    safe_headway_m: float = 1.0          # centre-to-centre spacing
    free_flow_mps: float = 0.40          # bus/vehicle speed cap
    max_speed_mps: float = 1.0           # above this a sample is impossible
    density_threshold: float = 0.70
    slow_fraction: float = 0.40
    slow_ticks: int = 3
    clear_ticks: int = 2                 # evaluations below threshold to leave CONGESTED
    min_samples: int = 3                 # fresh snapshots needed after warm-up/flush
    obstacle_radius_m: float = 0.35
    obstacle_clear_sec: float = 2.0
    capacity: Dict[str, int] = field(default_factory=dict)
    free_flow: Dict[str, float] = field(default_factory=dict)

    def validate(self) -> None:
        checks = [
            (self.window_sec > 0, 'window_sec must be > 0'),
            (0.0 < self.stale_sec <= 1.2, 'stale_sec must be in (0, 1.2]: UNKNOWN is required within 1.2 s'),
            (0.0 < self.presence_hold_sec <= self.window_sec,
             'presence_hold_sec must be in (0, window_sec]'),
            (0.0 < self.corridor_halfwidth_m <= 2.0, 'corridor_halfwidth_m in (0, 2]'),
            (self.safe_headway_m > 0, 'safe_headway_m must be > 0'),
            (0.0 < self.free_flow_mps <= self.max_speed_mps,
             'free_flow_mps must be in (0, max_speed_mps]'),
            (0.0 < self.density_threshold <= 1.0, 'density_threshold in (0, 1]'),
            (0.0 < self.slow_fraction < 1.0, 'slow_fraction in (0, 1)'),
            (self.slow_ticks >= 1 and self.clear_ticks >= 1, 'tick counts must be >= 1'),
            (self.min_samples >= 1, 'min_samples must be >= 1'),
            (self.obstacle_radius_m >= 0 and self.obstacle_clear_sec >= 0,
             'obstacle parameters must be >= 0'),
        ]
        for ok, msg in checks:
            if not ok:
                raise ValueError(msg)
        for e in self.edges:
            if normalize_edge_id(e) != e:
                raise ValueError('edge %r is not a canonical map edge id' % e)


@dataclass
class _EdgeRT:
    samples: Deque[Tuple[float, str, float]] = field(default_factory=deque)
    obstacle_seen: float = -math.inf
    slow_count: int = 0
    clear_count: int = 0
    congested: bool = False
    label: str = UNKNOWN


class TrafficEngine:
    def __init__(self, config: Optional[TrafficConfig] = None) -> None:
        self.cfg = config or TrafficConfig()
        self.cfg.validate()
        self._lock = threading.RLock()
        self._rt: Dict[str, _EdgeRT] = {e: _EdgeRT() for e in self.cfg.edges}
        self._geom = {e: edge_endpoints(e) for e in self.cfg.edges}
        self._cap = {e: self.cfg.capacity.get(
            e, max(1, int(edge_length(e) // self.cfg.safe_headway_m)))
            for e in self.cfg.edges}
        self._vff = {e: self.cfg.free_flow.get(e, self.cfg.free_flow_mps)
                     for e in self.cfg.edges}
        self._vehicles: Dict[str, Tuple[float, str, float]] = {}   # vid -> (t, edge, speed)
        self._invalid: Dict[str, Tuple[float, str]] = {}           # vid -> (since, edge)
        self._last_stamp: Optional[float] = None
        self._last_fresh: Optional[float] = None
        self._obs_stamp: Optional[float] = None
        self._snap_count = 0
        self.rejections: Counter = Counter()

    # ---------------- public helpers ----------------
    def capacity(self, edge: str) -> int:
        return self._cap[edge]

    def last_observation_stamp(self) -> Optional[float]:
        """Source timestamp of the newest *fresh* snapshot (None before any)."""
        with self._lock:
            return self._obs_stamp

    def is_stale(self, now: float) -> bool:
        with self._lock:
            return self._last_fresh is None or now - self._last_fresh > self.cfg.stale_sec

    # ---------------- ingest ----------------
    def ingest(self, stamp_s: float, now_s: float, vehicles: Sequence[VehicleObs],
               source_age_s: Optional[float] = None) -> IngestReport:
        rep = IngestReport()
        with self._lock:
            if not math.isfinite(stamp_s):
                return self._reject_snapshot(rep, 'BAD_STAMP')
            if self._last_stamp is not None and stamp_s <= self._last_stamp:
                if self._last_stamp - stamp_s > self.cfg.clock_reset_sec:
                    self._flush()            # simulator reset: accept new epoch
                    self._last_stamp = None
                else:                        # repeated or older stamp = frozen source
                    return self._reject_snapshot(rep, 'STALE_STAMP')
            if source_age_s is not None and not (
                    -self.cfg.future_tolerance_sec <= source_age_s
                    <= self.cfg.max_source_age_sec):
                return self._reject_snapshot(rep, 'SOURCE_AGE')
            if self._last_fresh is not None and \
                    now_s - self._last_fresh > self.cfg.stale_sec:
                self._flush()                # recovering from a gap: start clean
            self._last_stamp = stamp_s
            self._last_fresh = now_s
            self._obs_stamp = stamp_s
            self._snap_count += 1
            rep.snapshot_ok = True

            seen = set()
            obstacles: List[Tuple[float, float]] = []
            for v in vehicles:
                self._ingest_one(v, now_s, seen, obstacles, rep)
            for vid in [k for k in self._invalid if k not in seen]:
                del self._invalid[vid]       # absent: may simply have exited

            reach = self.cfg.corridor_halfwidth_m + self.cfg.obstacle_radius_m
            for ox, oy in obstacles:
                for e, ((ax, ay), (bx, by)) in self._geom.items():
                    if point_segment_distance(ox, oy, ax, ay, bx, by) <= reach:
                        self._rt[e].obstacle_seen = now_s
            self._prune(now_s)
            return rep

    def _reject_snapshot(self, rep: IngestReport, reason: str) -> IngestReport:
        rep.snapshot_reason = reason
        self.rejections[reason] += 1
        return rep

    def _ingest_one(self, v: VehicleObs, now_s, seen, obstacles, rep) -> None:
        cfg = self.cfg

        def reject(reason):
            rep.rejected.append((v.vehicle_id, reason))
            self.rejections[reason] += 1

        if (v.kind or '').lower() == 'obstacle':
            if math.isfinite(v.x) and math.isfinite(v.y):
                obstacles.append((v.x, v.y))
            else:
                reject('BAD_POSE')
            return
        vid = v.vehicle_id
        if not vid or vid in seen:
            reject('DUPLICATE_ID')
            return
        seen.add(vid)
        eid = normalize_edge_id(v.edge_id)
        if not v.valid:                      # sensor flags this vehicle as unreliable
            last = self._vehicles.pop(vid, None)
            edge = eid if eid in self._rt else (last[1] if last else None)
            if edge is not None:
                self._invalid.setdefault(vid, (now_s, edge))
            return
        if eid is None or eid not in self._rt:
            reject('INVALID_EDGE_ID')
            return
        if not (math.isfinite(v.x) and math.isfinite(v.y)):
            reject('BAD_POSE')
            return
        if not (math.isfinite(v.speed) and 0.0 <= v.speed <= cfg.max_speed_mps):
            reject('BAD_SPEED')
            return
        (ax, ay), (bx, by) = self._geom[eid]
        if point_segment_distance(v.x, v.y, ax, ay, bx, by) > cfg.corridor_halfwidth_m:
            reject('OUTSIDE_CORRIDOR')
            return
        self._vehicles[vid] = (now_s, eid, v.speed)
        self._rt[eid].samples.append((now_s, vid, v.speed))
        self._invalid.pop(vid, None)
        rep.accepted += 1

    def _prune(self, now_s: float) -> None:
        cutoff = now_s - self.cfg.window_sec
        for rt in self._rt.values():
            while rt.samples and rt.samples[0][0] < cutoff:
                rt.samples.popleft()

    def _flush(self) -> None:
        """Forget windowed data (obstacle memory is kept: do not forget a block)."""
        self._vehicles.clear()
        self._invalid.clear()
        self._snap_count = 0
        for rt in self._rt.values():
            rt.samples.clear()
            rt.slow_count = rt.clear_count = 0
            rt.congested = False

    # ---------------- evaluate (2 Hz) ----------------
    def evaluate(self, now_s: float) -> List[EdgeResult]:
        cfg = self.cfg
        with self._lock:
            stale = self.is_stale(now_s)
            age = math.inf if self._last_fresh is None else now_s - self._last_fresh
            if stale:
                self._flush()
            self._prune(now_s)
            out: List[EdgeResult] = []
            for e in cfg.edges:
                rt = self._rt[e]
                blocked = now_s - rt.obstacle_seen <= cfg.obstacle_clear_sec
                lost = any(ed == e and now_s - since > cfg.stale_sec
                           for since, ed in self._invalid.values())
                valid = (not stale) and self._snap_count >= cfg.min_samples and not lost
                if not valid:
                    rt.slow_count = rt.clear_count = 0
                    rt.congested = False
                    speed, rho, n, label = -1.0, -1.0, 0, UNKNOWN
                else:
                    present = [vid for vid, (t, ed, _) in self._vehicles.items()
                               if ed == e and now_s - t <= cfg.presence_hold_sec]
                    n = len(present)
                    rho = min(1.0, n / self._cap[e])
                    if n:
                        per_vehicle = []
                        for vid in present:
                            sp = [s for (_, v2, s) in rt.samples if v2 == vid]
                            per_vehicle.append(statistics.median(sp))
                        speed = sum(per_vehicle) / len(per_vehicle)
                    else:
                        speed = self._vff[e]          # empty road, live feed
                    slow = n > 0 and speed < cfg.slow_fraction * self._vff[e]
                    rt.slow_count = rt.slow_count + 1 if slow else 0
                    raw = rho >= cfg.density_threshold or rt.slow_count >= cfg.slow_ticks
                    if raw:
                        rt.congested, rt.clear_count = True, 0
                    elif rt.congested:
                        rt.clear_count += 1
                        if rt.clear_count >= cfg.clear_ticks:
                            rt.congested, rt.clear_count = False, 0
                    label = BLOCKED if blocked else CONGESTED if rt.congested else NORMAL
                prev = rt.label
                rt.label = label
                out.append(EdgeResult(e, label, valid, blocked, speed, rho, n,
                                      self._cap[e], age, prev, prev != label))
            return out
