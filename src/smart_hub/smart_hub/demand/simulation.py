"""Seeded synthetic queue episodes for S2 training, evaluation and replay.

This is a *labelled synthetic event source* (assignment 9.2): passengers
arrive by a time-varying Poisson process, may leave the queue (balk), and
board a bus in groups of at most four over an eight-second dwell. The
simulated detector publishes QueueState-like observations at 5 Hz with count
flicker and occasional outages, which exercises the same validity paths as
the real /hub/queue stream.

Determinism and pairing: every random stream is seeded from the episode seed
with its own label, so the passenger arrival stream (including each
passenger's patience) is identical whichever bus policy runs. Predictive and
reactive dispatch can therefore be compared on exactly the same passengers.

Ground truth (true queue length, individual waits) is private to the
evaluator; the estimator only ever sees the noisy observations.
"""

import math
import random
from dataclasses import dataclass, field
from typing import Dict, List, Optional

from smart_hub.demand.dispatch_policy import DispatchPolicy
from smart_hub.demand.estimator import DemandEstimator
from smart_hub.demand.models import PersistenceModel

PROFILES = ("steady", "ramp_up", "ramp_down", "peak", "wave", "burst",
            "noisy_threshold")

DT = 0.2                 # 5 Hz observation / simulation step
BOARD_CAPACITY = 4       # passengers per bus visit (assignment 10.2)
BOARD_DWELL_SEC = 8.0    # door-open boarding interval (assignment 10.2)
QUEUE_CAPACITY = 16      # zone Q is 4 m x 2 m; arrivals beyond this walk away


@dataclass
class EpisodeSpec:
    seed: int
    profile: str
    bus_mode: str = "scheduled"      # scheduled | policy
    policy_mode: str = "reactive"    # used when bus_mode == policy
    duration_sec: int = 360
    gaps: List = field(default_factory=list)  # (start, length, kind)


@dataclass
class Passenger:
    pid: int
    entered: float
    patience: float
    left: Optional[float] = None
    boarded: Optional[float] = None


@dataclass
class EpisodeResult:
    spec: EpisodeSpec
    ticks: List[Dict]
    passengers: List[Passenger]
    turned_away: int
    dispatches: List[Dict]
    # (t, count, valid, confidence, stamp_offset) per 5 Hz detector message;
    # stamp_offset is 0 normally and negative while the source stamp is
    # frozen. Silent outages produce no entry. Used by fake_queue_publisher.
    observations: List = field(default_factory=list)


# --------------------------------------------------------------------- specs
def make_spec(seed: int, profile: Optional[str] = None,
              duration_sec: int = 360) -> EpisodeSpec:
    """Deterministic episode description derived from the seed."""
    rng = random.Random("%d-spec" % seed)
    if profile is None:
        profile = PROFILES[seed % len(PROFILES)]
    bus_mode = "scheduled" if rng.random() < 0.5 else "policy"
    gaps = []
    n_gaps = rng.choice((0, 0, 1, 1, 2))
    for _ in range(n_gaps):
        start = rng.uniform(70.0, duration_sec - 20.0)
        length = rng.uniform(2.0, 12.0)
        kind = rng.choice(("invalid", "silent", "frozen"))
        gaps.append((round(start, 1), round(length, 1), kind))
    return EpisodeSpec(seed, profile, bus_mode, "reactive", duration_sec,
                       sorted(gaps))


class RateProfile:
    """Arrival rate lambda(t) in passengers per second."""

    def __init__(self, profile: str, seed: int, duration: float):
        if profile not in PROFILES:
            raise ValueError("unknown profile %r" % profile)
        r = random.Random("%d-profile" % seed)
        self.profile = profile
        self.bursts = []
        self.initial_queue = r.randint(0, 3)
        self.flicker = 0.02
        self.patience_mean = 420.0
        if profile == "steady":
            self.p = dict(rate=r.uniform(0.02, 0.07))
        elif profile == "ramp_up":
            t0 = r.uniform(30, 150)
            self.p = dict(r0=r.uniform(0.0, 0.03), r1=r.uniform(0.08, 0.16),
                          t0=t0, t1=t0 + r.uniform(60, 150))
        elif profile == "ramp_down":
            t0 = r.uniform(30, 150)
            self.initial_queue = r.randint(4, 9)
            self.p = dict(r0=r.uniform(0.08, 0.15), r1=r.uniform(0.0, 0.03),
                          t0=t0, t1=t0 + r.uniform(60, 150))
        elif profile == "peak":
            self.p = dict(base=r.uniform(0.01, 0.03),
                          amp=r.uniform(0.07, 0.15),
                          centre=r.uniform(110, 260), width=r.uniform(30, 70))
        elif profile == "wave":
            self.p = dict(base=r.uniform(0.04, 0.07), amp=r.uniform(0.02, 0.05),
                          period=r.uniform(100, 220),
                          phase=r.uniform(0, 2 * math.pi))
        elif profile == "burst":
            self.p = dict(rate=r.uniform(0.01, 0.04))
            for _ in range(r.randint(2, 4)):  # e.g. a train unloading
                self.bursts.append((r.uniform(20, duration - 40),
                                    r.randint(2, 5)))
            self.bursts.sort()
        elif profile == "noisy_threshold":
            # Queue hovers around the dispatch threshold of 6 with extra
            # detector flicker: the hard case for dispatch hysteresis.
            self.initial_queue = r.randint(5, 7)
            self.p = dict(rate=r.uniform(0.04, 0.07))
            self.flicker = 0.08
            self.patience_mean = 120.0

    def rate(self, t: float) -> float:
        p = self.p
        if self.profile in ("steady", "burst", "noisy_threshold"):
            return p["rate"]
        if self.profile in ("ramp_up", "ramp_down"):
            if t <= p["t0"]:
                return p["r0"]
            if t >= p["t1"]:
                return p["r1"]
            a = (t - p["t0"]) / (p["t1"] - p["t0"])
            return p["r0"] + a * (p["r1"] - p["r0"])
        if self.profile == "peak":
            z = (t - p["centre"]) / p["width"]
            return p["base"] + p["amp"] * math.exp(-0.5 * z * z)
        if self.profile == "wave":
            return max(0.0, p["base"] + p["amp"] * math.sin(
                2 * math.pi * t / p["period"] + p["phase"]))
        raise AssertionError(self.profile)


def _poisson(rng: random.Random, lam: float) -> int:
    if lam <= 0:
        return 0
    limit, k, prod = math.exp(-lam), 0, rng.random()
    while prod > limit:
        k += 1
        prod *= rng.random()
    return k


# ------------------------------------------------------------------ episode
def run_episode(spec: EpisodeSpec, model=None,
                estimator_kwargs: Optional[Dict] = None,
                policy_kwargs: Optional[Dict] = None) -> EpisodeResult:
    """Simulate one episode and run the online estimator on its observations.

    Returns per-second records (truth, observation, prediction, features,
    dispatch decision) plus individual passenger outcomes.
    """
    model = model or PersistenceModel()
    est = DemandEstimator(model, **(estimator_kwargs or {}))
    policy = None
    if spec.bus_mode == "policy":
        policy = DispatchPolicy(mode=spec.policy_mode, **(policy_kwargs or {}))

    rng_arr = random.Random("%d-arrivals" % spec.seed)
    rng_obs = random.Random("%d-obs" % spec.seed)
    rng_bus = random.Random("%d-bus" % spec.seed)
    rates = RateProfile(spec.profile, spec.seed, spec.duration_sec)

    passengers: List[Passenger] = []
    waiting: List[Passenger] = []
    turned_away = 0

    def add_passenger(t: float) -> None:
        nonlocal turned_away
        patience = rng_arr.expovariate(1.0 / rates.patience_mean)
        if len(waiting) >= QUEUE_CAPACITY:
            turned_away += 1
            return
        p = Passenger(len(passengers), t, patience)
        passengers.append(p)
        waiting.append(p)

    for _ in range(rates.initial_queue):
        add_passenger(0.0)
    bursts = list(rates.bursts)

    # Bus timeline state
    if spec.bus_mode == "scheduled":
        next_bus = rng_bus.uniform(30.0, 120.0)
    else:
        next_bus = None
    boarding_until = None
    board_slots: List[float] = []
    dispatches: List[Dict] = []

    ticks: List[Dict] = []
    observations: List = []
    stamp_frozen_at = None
    steps = int(round(spec.duration_sec / DT))
    for k in range(steps + 1):
        t = round(k * DT, 3)

        # --- world: arrivals, balking, bus ---------------------------------
        if k > 0:
            for _ in range(_poisson(rng_arr, rates.rate(t) * DT)):
                add_passenger(t)
            while bursts and bursts[0][0] <= t:
                for _ in range(bursts.pop(0)[1]):
                    add_passenger(t)
            for p in list(waiting):
                if t - p.entered >= p.patience:
                    p.left = t
                    waiting.remove(p)

        if next_bus is not None and t >= next_bus and boarding_until is None:
            boarding_until = t + BOARD_DWELL_SEC
            board_slots = [t + 1.0 + 2.0 * i for i in range(BOARD_CAPACITY)]
            next_bus = None
        if boarding_until is not None:
            while board_slots and t >= board_slots[0]:
                board_slots.pop(0)
                if waiting:
                    p = waiting.pop(0)  # FIFO boarding
                    p.boarded = t
            if t >= boarding_until:
                boarding_until = None
                if spec.bus_mode == "scheduled":
                    next_bus = t + rng_bus.uniform(60.0, 150.0)
                elif policy is not None:
                    policy.on_mission_finished()
                    dispatches[-1]["finished"] = t

        true_count = len(waiting)

        # --- detector observation at 5 Hz ----------------------------------
        gap = next((g for g in spec.gaps if g[0] <= t < g[0] + g[1]), None)
        recv = t + 0.02
        if gap is None:
            stamp_frozen_at = None
            obs = true_count
            if rng_obs.random() < rates.flicker:
                obs = max(0, obs + rng_obs.choice((-1, 1)))
            est.on_queue(obs, True, 0.9, t, recv, recv)
            observations.append((t, obs, True, 0.9, 0.0))
        elif gap[2] == "invalid":
            est.on_queue(0, False, 0.0, t, recv, recv)
            observations.append((t, 0, False, 0.0, 0.0))
        elif gap[2] == "frozen":
            if stamp_frozen_at is None:
                stamp_frozen_at = t - DT
            est.on_queue(true_count, True, 0.9, stamp_frozen_at, recv, recv)
            observations.append((t, true_count, True, 0.9,
                                 round(stamp_frozen_at - t, 3)))
        # "silent": nothing published at all

        # --- 1 Hz predictor tick and dispatch policy -----------------------
        if k % int(round(1.0 / DT)) == 0:
            pred = est.tick(t + 0.05, t + 0.05)
            row = {
                "t": int(round(t)),
                "true_count": true_count,
                "valid": pred.valid,
                "reason": pred.reason,
                "predicted": pred.predicted_count,
                "baseline": pred.baseline_count,
                "features": pred.features,
                "inference_ms": pred.inference_ms,
                "decision": None,
            }
            if policy is not None:
                d = policy.update(
                    t, pred.valid, pred.predicted_count,
                    pred.baseline_count >= 0, pred.baseline_count)
                row["decision"] = d.reason
                if d.dispatch:
                    travel = rng_bus.uniform(22.0, 40.0)
                    next_bus = t + travel
                    dispatches.append({"t": t, "reason": d.reason,
                                       "arrive": round(next_bus, 2),
                                       "finished": None})
            ticks.append(row)

    return EpisodeResult(spec, ticks, passengers, turned_away, dispatches,
                         observations)


TARGETS = ("demand", "queue")


def unserved_demand(result: EpisodeResult, t: float, horizon: int) -> int:
    """Passengers who would be waiting at t+horizon if no bus boarded after t.

    Counts everyone who has entered by t+horizon, had not boarded by t, and
    whose patience has not run out by t+horizon. This removes the effect of
    future bus service from the label (see README, "Forecast target").
    """
    end = t + horizon
    n = 0
    for p in result.passengers:
        if p.entered > end:
            continue
        if p.boarded is not None and p.boarded <= t:
            continue
        if p.entered + p.patience <= end:
            continue
        n += 1
    return n


def supervised_rows(result: EpisodeResult, horizon: int = 30,
                    target: str = "demand"):
    """(features, label, baseline, t) for every tick with a valid forecast.

    Labels come from private ground truth and are never input features:
      * ``queue``:  true queue length at t+horizon (includes the effect of
                    any bus that boards in between);
      * ``demand``: unserved demand at t+horizon, i.e. the queue length that
                    would exist if no bus boarded after t.
    """
    if target not in TARGETS:
        raise ValueError("target must be one of %s" % (TARGETS,))
    truth = {r["t"]: r["true_count"] for r in result.ticks}
    out = []
    for r in result.ticks:
        if not r["valid"] or (r["t"] + horizon) not in truth:
            continue
        if target == "queue":
            label = float(truth[r["t"] + horizon])
        else:
            label = float(unserved_demand(result, r["t"], horizon))
        out.append((r["features"], label, r["baseline"], r["t"]))
    return out


def wait_stats(result: EpisodeResult) -> Dict:
    """Passenger waiting statistics; censored passengers reported separately."""
    end = float(result.spec.duration_sec)
    boarded = sorted(p.boarded - p.entered for p in result.passengers
                     if p.boarded is not None)
    censored = [end - p.entered for p in result.passengers
                if p.boarded is None and p.left is None]
    left = [p for p in result.passengers if p.left is not None]

    def pct(xs, q):
        if not xs:
            return None
        i = min(len(xs) - 1, max(0, int(math.ceil(q * len(xs))) - 1))
        return xs[i]

    return {
        "boarded": len(boarded),
        "mean_wait_boarded": sum(boarded) / len(boarded) if boarded else None,
        "p50_wait_boarded": pct(boarded, 0.50),
        "p95_wait_boarded": pct(boarded, 0.95),
        "censored": len(censored),
        "censored_wait_so_far_mean": (sum(censored) / len(censored)
                                      if censored else None),
        "left_queue": len(left),
        "turned_away": result.turned_away,
        "dispatches": len(result.dispatches),
    }
