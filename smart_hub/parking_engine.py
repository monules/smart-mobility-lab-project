"""Parking domain logic for Scenario S4 (no ROS imports, fully unit-testable).

Design rules (assignment sections 6.1, 11 and 18.2):

*Physical state (what the sensor says) and administrative state (who holds a
  reservation) are stored separately.  The effective state shown to the rest
  of the hub is derived from both, on demand, at the current time.
*Only a FRESH, debounced FREE slot is allocatable.  UNKNOWN / OCCUPIED /
  RESERVED slots are never advertised as free.
*Every mutation happens under one re-entrant lock: validation, request-id
  cache, slot choice, token creation and expiry cannot interleave.
*TTL expiry and release only drop the administrative reservation. They
  never change the sensed state, so a physically occupied (or unobserved) slot
  can not become FREE because a timer fired.

All times are monotonic seconds supplied by the caller (``now``), which makes
the engine deterministic in tests.
"""
from __future__ import annotations

import math
import threading
import uuid
from collections import OrderedDict, deque
from dataclasses import dataclass, field, replace
from typing import Callable, Deque, Dict, List, Optional, Tuple

from .hub_map import access_distances

FREE = 'FREE'
RESERVED = 'RESERVED'
OCCUPIED = 'OCCUPIED'
UNKNOWN = 'UNKNOWN'

TTL_MIN_SEC = 5.0
TTL_MAX_SEC = 120.0


# --------------------------------------------------------------------------
# Value objects
# --------------------------------------------------------------------------
@dataclass(frozen=True)
class ReserveResult:
    success: bool
    reservation_id: str = ''
    slot_id: str = ''
    expires_mono: float = 0.0
    reason: str = ''
    replayed: bool = False          # True when served from the request-id cache


@dataclass(frozen=True)
class ReleaseResult:
    success: bool
    reason: str = ''
    replayed: bool = False


@dataclass(frozen=True)
class SlotView:
    slot_id: str
    state: str
    vehicle_id: str
    reservation_id: str
    expires_mono: float


@dataclass(frozen=True)
class ParkingEvent:
    event: str
    entity_id: str
    reason: str
    correlation_id: str


@dataclass
class ParkingConfig:
    slot_ids: Tuple[str, ...] = ('P1', 'P2', 'P3')
    slot_distance: Dict[str, float] = field(default_factory=dict)
    freshness_sec: float = 1.0       # perception freshness (assignment 3.3)
    debounce_sec: float = 0.5        # stable valid readings before a change
    min_confidence: float = 0.5      # below this an observation is not "valid"
    strict_preference: bool = False  # True: preferred slot or reject
    cache_size: int = 4096           # bounded request-id caches

    def validate(self) -> None:
        if not self.slot_ids or len(set(self.slot_ids)) != len(self.slot_ids):
            raise ValueError('slot_ids must be a non-empty list of unique ids')
        if not 0.0 < self.freshness_sec <= 10.0:
            raise ValueError('freshness_sec must be in (0, 10]')
        if not 0.0 <= self.debounce_sec <= 5.0:
            raise ValueError('debounce_sec must be in [0, 5]')
        if not 0.0 <= self.min_confidence <= 1.0:
            raise ValueError('min_confidence must be in [0, 1]')
        if self.cache_size < 16:
            raise ValueError('cache_size must be >= 16')


@dataclass
class _Slot:
    committed: str = UNKNOWN        # debounced sensor state: FREE/OCCUPIED/UNKNOWN
    last_seen: float = -math.inf    # last reading that agreed with `committed`
    cand: str = UNKNOWN             # reading currently being debounced
    cand_since: float = 0.0
    token: str = ''                 # administrative reservation (may be '')
    vehicle: str = ''
    expiry: float = 0.0


# --------------------------------------------------------------------------
# Engine
# --------------------------------------------------------------------------
class ParkingEngine:
    """Thread-safe slot table with atomic, idempotent reservations."""

    def __init__(self, config: Optional[ParkingConfig] = None) -> None:
        self.cfg = config or ParkingConfig()
        self.cfg.validate()
        defaults = access_distances()
        self._dist = {sid: self.cfg.slot_distance.get(sid, defaults.get(sid, math.inf))
                      for sid in self.cfg.slot_ids}
        self._slots: 'OrderedDict[str, _Slot]' = OrderedDict(
            (sid, _Slot()) for sid in self.cfg.slot_ids)
        self._lock = threading.RLock()
        self._reserve_cache: OrderedDict = OrderedDict()
        self._release_cache: OrderedDict = OrderedDict()
        self._events: Deque[ParkingEvent] = deque(maxlen=10000)
        self._last_eff: Dict[str, str] = {sid: UNKNOWN for sid in self.cfg.slot_ids}
        # Test-only seam: called between "slot chosen" and "slot committed".
        # The T07 negative control uses it (with the lock removed) to prove
        # that the double-allocation test can actually fail.
        self._race_hook: Optional[Callable[[], None]] = None

    # ---------------- sensing ----------------
    def observe(self, slot_id: str, occupied: bool, valid: bool = True,
                confidence: float = 1.0, now: float = 0.0) -> None:
        """Feed one sensor reading.  A change is committed only after
        ``debounce_sec`` of consecutive identical valid readings."""
        with self._lock:
            s = self._slots.get(slot_id)
            if s is None:
                return                       # unknown slot id: ignore
            if (not valid) or confidence < self.cfg.min_confidence \
                    or not math.isfinite(confidence):
                s.committed = UNKNOWN        # sensor says "I do not know"
                s.cand = UNKNOWN
                self._track_locked(now)
                return
            raw = OCCUPIED if occupied else FREE
            if raw == s.committed:
                s.last_seen = now            # reading confirms current state
                s.cand = raw
            else:
                if raw != s.cand:
                    s.cand = raw
                    s.cand_since = now
                if now - s.cand_since >= self.cfg.debounce_sec:
                    s.committed = raw
                    s.last_seen = now
            self._track_locked(now)

    def _eff(self, s: _Slot, now: float) -> str:
        if s.committed == UNKNOWN or now - s.last_seen > self.cfg.freshness_sec:
            return UNKNOWN                   # never observed, dropped out, or stale
        if s.cand not in (UNKNOWN, s.committed):
            return UNKNOWN                   # contradicting reading being debounced
        if s.committed == OCCUPIED:
            return OCCUPIED                  # ownership metadata is kept, state is physical
        if s.token and s.expiry > now:
            return RESERVED
        return FREE

    def effective_state(self, slot_id: str, now: float) -> str:
        with self._lock:
            return self._eff(self._slots[slot_id], now)

    # ---------------- reservation ----------------
    def reserve(self, request_id: str, vehicle_id: str, preferred_slot: str,
                ttl_sec: float, now: float) -> ReserveResult:
        with self._lock:
            self._expire_locked(now)
            if not request_id or not vehicle_id:
                return ReserveResult(False, reason='INVALID_REQUEST')
            preferred = preferred_slot or ''
            ttl = float(ttl_sec)
            payload = (vehicle_id, preferred, round(ttl, 3) if math.isfinite(ttl) else ttl)

            cached = self._reserve_cache.get(request_id)
            if cached is not None:           # idempotency key already seen
                old_payload, old = cached
                if old_payload == payload:
                    self._emit('RESERVE_REPLAY', vehicle_id, old.reason,
                               old.reservation_id or request_id)
                    return replace(old, replayed=True)
                self._emit('RESERVE_REJECTED', vehicle_id, 'KEY_REUSE', request_id)
                return ReserveResult(False, reason='KEY_REUSE')

            if not math.isfinite(ttl) or not TTL_MIN_SEC <= ttl <= TTL_MAX_SEC:
                return ReserveResult(False, reason='INVALID_TTL')
            if preferred and preferred not in self._slots:
                return ReserveResult(False, reason='INVALID_SLOT')

            if any(s.token and s.vehicle == vehicle_id for s in self._slots.values()):
                result = ReserveResult(False, reason='VEHICLE_ALREADY_RESERVED')
            else:
                free = [sid for sid, s in self._slots.items()
                        if self._eff(s, now) == FREE]
                chosen = ''
                if preferred and preferred in free:
                    chosen = preferred
                elif preferred and self.cfg.strict_preference:
                    result = ReserveResult(False, reason='PREFERRED_UNAVAILABLE')
                    return self._finish_reserve(request_id, payload, result,
                                                vehicle_id, now)
                elif free:
                    order = {sid: i for i, sid in enumerate(self._slots)}
                    chosen = min(free, key=lambda sid: (self._dist[sid], order[sid]))
                if self._race_hook is not None:
                    self._race_hook()
                if not chosen:
                    result = ReserveResult(False, reason='FULL')
                else:
                    s = self._slots[chosen]
                    s.token = uuid.uuid4().hex
                    s.vehicle = vehicle_id
                    s.expiry = now + ttl
                    result = ReserveResult(True, s.token, chosen, s.expiry, 'OK')
            return self._finish_reserve(request_id, payload, result, vehicle_id, now)

    def _finish_reserve(self, request_id, payload, result, vehicle_id, now):
        self._remember(self._reserve_cache, request_id, (payload, result))
        self._emit('RESERVED' if result.success else 'RESERVE_REJECTED',
                   result.slot_id or vehicle_id, result.reason,
                   result.reservation_id or request_id)
        self._track_locked(now)
        return result

    def release(self, request_id: str, reservation_id: str, vehicle_id: str,
                now: float) -> ReleaseResult:
        """Token- and vehicle-validated release of the *administrative*
        reservation.  The sensed state is never touched."""
        with self._lock:
            self._expire_locked(now)
            if not request_id or not reservation_id or not vehicle_id:
                return ReleaseResult(False, 'INVALID_REQUEST')
            payload = (reservation_id, vehicle_id)
            cached = self._release_cache.get(request_id)
            if cached is not None:
                old_payload, old = cached
                if old_payload == payload:
                    return replace(old, replayed=True)
                return ReleaseResult(False, 'KEY_REUSE')

            slot_id = next((sid for sid, s in self._slots.items()
                            if s.token and s.token == reservation_id), '')
            if not slot_id:
                result = ReleaseResult(False, 'UNKNOWN_TOKEN')   # wrong or expired
            elif self._slots[slot_id].vehicle != vehicle_id:
                result = ReleaseResult(False, 'VEHICLE_MISMATCH')
            else:
                s = self._slots[slot_id]
                s.token, s.vehicle, s.expiry = '', '', 0.0
                state = self._eff(s, now)
                result = ReleaseResult(True, 'OK' if state == FREE
                                       else 'OK_SLOT_' + state)
            self._remember(self._release_cache, request_id, (payload, result))
            self._emit('RELEASED' if result.success else 'RELEASE_REJECTED',
                       slot_id or vehicle_id, result.reason, reservation_id)
            self._track_locked(now)
            return result

    # ---------------- expiry / queries ----------------
    def tick(self, now: float) -> None:
        """Periodic housekeeping: expire TTLs and report state transitions."""
        with self._lock:
            self._expire_locked(now)
            self._track_locked(now)

    def _expire_locked(self, now: float) -> None:
        for sid, s in self._slots.items():
            if s.token and s.expiry <= now:
                self._emit('RESERVATION_EXPIRED', sid,
                           'TTL_ELAPSED (sensor: %s)' % self._eff(s, now), s.token)
                # Administrative ownership only.  s.committed is untouched, so an
                # occupied/unobserved slot stays OCCUPIED/UNKNOWN.
                s.token, s.vehicle, s.expiry = '', '', 0.0

    def snapshot(self, now: float) -> Tuple[Tuple[SlotView, ...], int]:
        """Consistent view of all slots plus the count of allocatable (FREE)
        slots.  RESERVED slots are excluded from the count."""
        with self._lock:
            self._expire_locked(now)
            views = tuple(SlotView(sid, self._eff(s, now), s.vehicle, s.token,
                                   s.expiry) for sid, s in self._slots.items())
            return views, sum(1 for v in views if v.state == FREE)

    def lookup_reservation(self, token: str, now: float):
        """(slot_id, vehicle_id, effective_state) of an active token or None."""
        with self._lock:
            self._expire_locked(now)
            for sid, s in self._slots.items():
                if token and s.token == token:
                    return sid, s.vehicle, self._eff(s, now)
            return None

    # ---------------- events ----------------
    def emit_event(self, event: str, entity_id: str, reason: str,
                   correlation_id: str = '') -> None:
        with self._lock:
            self._emit(event, entity_id, reason, correlation_id)

    def drain_events(self) -> List[ParkingEvent]:
        with self._lock:
            out = list(self._events)
            self._events.clear()
            return out

    def _emit(self, event, entity, reason, correlation) -> None:
        self._events.append(ParkingEvent(event, entity, reason, correlation))

    def _track_locked(self, now: float) -> None:
        for sid, s in self._slots.items():
            eff = self._eff(s, now)
            prev = self._last_eff[sid]
            if eff != prev:
                self._last_eff[sid] = eff
                self._emit('SLOT_STATE', sid, '%s->%s' % (prev, eff), s.token)

    def _remember(self, cache: OrderedDict, key, value) -> None:
        cache[key] = value
        while len(cache) > self.cfg.cache_size:
            cache.popitem(last=False)


# --------------------------------------------------------------------------
# Barrier interlock
# --------------------------------------------------------------------------
class BarrierController:
    """Decides the gate command published on /parking/barrier_request.

    Open only when (a) the presented token is a live reservation owned by the
    presenting vehicle on a slot that is still RESERVED, and (b) the entrance
    sensor is FRESH and reports clear.  Close only after the hold time AND the
    entrance has been continuously clear: the controller never requests a
    closure on an occupied or unobserved passage.  A request is not an
    acknowledgment: /parking/barrier_open is tracked separately and a missing
    acknowledgment is logged.
    """

    def __init__(self, engine: ParkingEngine, entrance_fresh_sec: float = 1.0,
                 hold_sec: float = 8.0, close_clear_sec: float = 1.0,
                 ack_timeout_sec: float = 2.0) -> None:
        self.engine = engine
        self.entrance_fresh_sec = entrance_fresh_sec
        self.hold_sec = hold_sec
        self.close_clear_sec = close_clear_sec
        self.ack_timeout_sec = ack_timeout_sec
        self._lock = threading.RLock()
        self._entrance_clear = False
        self._entrance_seen = -math.inf
        self._clear_since: Optional[float] = None
        self.desired_open = False
        self._opened_at = 0.0
        self._owner: Tuple[str, str] = ('', '')
        self._acked = False
        self._ack_warned = False
        self._defer_warned = False

    def on_entrance(self, clear: bool, now: float) -> None:
        with self._lock:
            self._entrance_seen = now
            if clear:
                if self._clear_since is None or not self._entrance_clear:
                    self._clear_since = now
            else:
                self._clear_since = None
            self._entrance_clear = clear

    def _entrance_ok(self, now: float) -> bool:
        return self._entrance_clear and now - self._entrance_seen <= self.entrance_fresh_sec

    def on_claim(self, vehicle_id: str, token: str, now: float) -> Tuple[bool, str]:
        with self._lock:
            found = self.engine.lookup_reservation(token, now)
            if found is None:
                return self._deny(vehicle_id, token, 'UNKNOWN_TOKEN')
            slot_id, owner, state = found
            if owner != vehicle_id:
                return self._deny(vehicle_id, token, 'VEHICLE_MISMATCH')
            if state != RESERVED:
                return self._deny(vehicle_id, token, 'SLOT_NOT_RESERVED_' + state)
            if now - self._entrance_seen > self.entrance_fresh_sec:
                return self._deny(vehicle_id, token, 'ENTRANCE_SENSOR_STALE')
            if not self._entrance_clear:
                return self._deny(vehicle_id, token, 'ENTRANCE_NOT_CLEAR')
            if self.desired_open and self._owner != (vehicle_id, token):
                return self._deny(vehicle_id, token, 'BARRIER_BUSY')
            if not self.desired_open:
                self.desired_open = True
                self._opened_at = now
                self._owner = (vehicle_id, token)
                self._acked = self._ack_warned = self._defer_warned = False
                self.engine.emit_event('BARRIER_OPEN_REQUEST', slot_id, 'OK', token)
            return True, 'OK'

    def _deny(self, vehicle_id, token, reason):
        self.engine.emit_event('BARRIER_DENIED', vehicle_id, reason, token)
        return False, reason

    def on_ack(self, is_open: bool, now: float) -> None:
        with self._lock:
            if is_open and self.desired_open:
                self._acked = True

    def tick(self, now: float) -> Optional[bool]:
        """Return the new command if it changed on this tick, else None."""
        with self._lock:
            if not self.desired_open:
                return None
            if not self._acked and not self._ack_warned \
                    and now - self._opened_at > self.ack_timeout_sec:
                self._ack_warned = True
                self.engine.emit_event('BARRIER_NO_ACK', self._owner[0],
                                       'request not acknowledged', self._owner[1])
            if now - self._opened_at < self.hold_sec:
                return None
            clear_for = (now - self._clear_since) if self._clear_since is not None else 0.0
            if self._entrance_ok(now) and clear_for >= self.close_clear_sec:
                self.desired_open = False
                self.engine.emit_event('BARRIER_CLOSE_REQUEST', self._owner[0],
                                       'entrance clear', self._owner[1])
                self._owner = ('', '')
                return False
            if not self._defer_warned:
                self._defer_warned = True
                self.engine.emit_event('BARRIER_CLOSE_DEFERRED', self._owner[0],
                                       'entrance not clear or stale: holding open',
                                       self._owner[1])
            return None
