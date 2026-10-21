"""Pure-Python core of hub_coordinator (M2-B). No ROS imports.

CrossingArbiter  - single-owner crossing mutex (S8, T15)
DispatchPolicy   - demand -> bus dispatch with hysteresis/cooldown (S2/S8, T04)

All times are plain floats (seconds, monotonic) passed in by the caller, so
every behaviour can be unit tested without a clock or ROS.
"""
from collections import OrderedDict
from dataclasses import dataclass
from uuid import uuid4

# Crossing zone C from map M1
C_X = (4.5, 5.5)
C_Y = (1.25, 4.75)


def footprint_overlaps_crossing(x, y, radius):
    """Circle (agent footprint) vs rectangle C. Touching only = no overlap."""
    dx = max(C_X[0] - x, 0.0, x - C_X[1])
    dy = max(C_Y[0] - y, 0.0, y - C_Y[1])
    return dx * dx + dy * dy < radius * radius


class CrossingArbiter:
    """One owner at a time. Pedestrians win. FIFO among mobile agents.

    Rules taken from the assignment (sec 7.1, 15.2, 18.4):
    - deny if pedestrian present, state unknown, or someone else owns it
    - new grant only after the crossing was continuously clear for
      `clear_debounce` seconds
    - lease expiry never frees an occupied crossing: owner goes to HOLD and is
      only reclaimed after observed clearance
    - release only when the owner's footprint is out of C
    """

    def __init__(self, lease_sec=3.0, clear_debounce=2.0, wait_ttl=5.0):
        self.lease_sec = lease_sec
        self.clear_debounce = clear_debounce
        self.wait_ttl = wait_ttl
        self.owner = ''
        self.token = ''
        self.lease_expiry = 0.0
        self.hold = False
        self.valid = False            # observation fresh and trustworthy
        self.pedestrian = False
        self.occupants = frozenset()  # agent ids whose footprint overlaps C
        self.clear_since = None
        self.safety_latched = False
        self.waiting = OrderedDict()  # requester -> last time it asked (FIFO)
        self.events = []              # (event, entity_id, reason, correlation)

    # ---- helpers -------------------------------------------------------
    def _emit(self, event, entity, reason, corr=''):
        self.events.append((event, entity, reason, corr))

    def _is_clear(self):
        return self.valid and not self.pedestrian and not self.occupants

    def _clear_long_enough(self, now):
        return (self._is_clear() and self.clear_since is not None
                and now - self.clear_since >= self.clear_debounce)

    def _clear_owner(self):
        self.owner = ''
        self.token = ''
        self.lease_expiry = 0.0
        self.hold = False

    # ---- inputs --------------------------------------------------------
    def update_observation(self, now, pedestrian, occupants, valid):
        self.valid = bool(valid)
        self.pedestrian = bool(pedestrian)
        self.occupants = frozenset(occupants)
        if self._is_clear():
            if self.clear_since is None:
                self.clear_since = now
        else:
            self.clear_since = None
        self.tick(now)

    def tick(self, now):
        for r in [r for r, t in self.waiting.items() if now - t > self.wait_ttl]:
            del self.waiting[r]
        if self.owner and not self.hold and now > self.lease_expiry:
            self.hold = True
            self._emit('CROSSING_LEASE_EXPIRED', self.owner,
                       'HOLD_UNTIL_CLEAR', self.token)
        if self.owner and self.hold and self._clear_long_enough(now):
            self._emit('CROSSING_OWNER_RECLAIMED', self.owner,
                       'OBSERVED_CLEAR', self.token)
            self._clear_owner()

    # ---- service logic -------------------------------------------------
    def request(self, now, requester, operation, token):
        """Returns (granted, token, reason)."""
        self.tick(now)
        op = (operation or '').upper()
        if not requester or op not in ('ACQUIRE', 'RENEW', 'RELEASE'):
            return False, '', 'INVALID_REQUEST'
        if op == 'ACQUIRE':
            return self._acquire(now, requester, token)
        if op == 'RENEW':
            return self._renew(now, requester, token)
        return self._release(requester, token)

    def _acquire(self, now, requester, token):
        if token:
            return False, '', 'INVALID_REQUEST'
        if requester == self.owner:  # retry of a timed-out request: same token
            self.lease_expiry = now + self.lease_sec
            self.hold = False
            return True, self.token, 'ALREADY_OWNER'
        self.waiting[requester] = now  # keeps first-arrival position
        if self.safety_latched:
            return False, '', 'SAFETY_STOP'
        if not self.valid:
            return False, '', 'UNKNOWN_STATE'
        if self.pedestrian:
            return False, '', 'PEDESTRIAN_PRESENT'
        if self.owner:
            return False, '', 'OWNER_LOST_HOLD' if self.hold else 'OWNED_BY_OTHER'
        if self.occupants:
            return False, '', 'NOT_CLEAR'
        if not self._clear_long_enough(now):
            return False, '', 'CLEAR_DEBOUNCE'
        if next(iter(self.waiting)) != requester:
            return False, '', 'WAIT_FIFO'
        self.owner = requester
        self.token = uuid4().hex
        self.lease_expiry = now + self.lease_sec
        self.hold = False
        del self.waiting[requester]
        self._emit('CROSSING_GRANTED', requester, 'OK', self.token)
        return True, self.token, 'OK'

    def _renew(self, now, requester, token):
        if not self.owner or requester != self.owner or token != self.token:
            return False, '', 'BAD_TOKEN'
        self.lease_expiry = now + self.lease_sec
        if self.hold:
            self.hold = False
            self._emit('CROSSING_RENEWED_AFTER_HOLD', requester, 'OK', self.token)
        return True, self.token, 'OK'

    def _release(self, requester, token):
        if not self.owner or requester != self.owner or token != self.token:
            return False, '', 'BAD_TOKEN'
        if not self.valid or requester in self.occupants:
            return False, '', 'NOT_CLEAR'
        self._emit('CROSSING_RELEASED', requester, 'OK', self.token)
        self._clear_owner()
        return True, '', 'OK'

    # ---- output --------------------------------------------------------
    def snapshot(self):
        """Fields for CrossingState."""
        if not self.valid:
            state = 'UNKNOWN'
        elif self.owner:
            state = 'HOLD' if self.hold else 'GRANTED'
        elif self.pedestrian:
            state = 'PEDESTRIAN'
        elif self.occupants:
            state = 'OCCUPIED'
        else:
            state = 'FREE'
        return dict(owner_id=self.owner, token=self.token, state=state,
                    pedestrian_present=self.pedestrian, valid=self.valid)


@dataclass(frozen=True)
class Decision:
    source: str          # PREDICTED or FALLBACK
    correlation_id: str


class DispatchPolicy:
    """Decides when to send ONE DockBus goal.

    - escalation starts after `need_high` consecutive samples >= `high` and
      clears after `need_low` consecutive samples < `low` (hysteresis)
    - while escalated, send a bus only if the current signal is still >=
      `high`, none is outstanding, the bus is idle and the cooldown has
      passed, so escalation persists across missions but two goals are never
      active at once
    - based on Tuymurod's demand/dispatch_policy.py; the one difference is
      the "current signal still >= high" check (his version could dispatch
      on a falling queue while escalation had not cleared yet)
    - uses forecast when valid, otherwise fresh observed count (FALLBACK);
      counters reset whenever the source changes or input goes invalid, so an
      invalid forecast can never contribute to a dispatch
    """

    def __init__(self, high=6.0, low=4.0, need_high=3, need_low=5,
                 cooldown=30.0):
        self.high, self.low = high, low
        self.need_high, self.need_low = need_high, need_low
        self.cooldown = cooldown
        self.high_n = 0
        self.low_n = 0
        self.source = None
        self.escalated = False
        self.outstanding = ''          # correlation id of active mission
        self.last_dispatch = float('-inf')
        self._n = 0
        self.last_reason = 'NO_VALID_SIGNAL'

    def step(self, now, pred_valid, predicted, queue_valid, queue_count,
             bus_idle):
        """Call at 1 Hz. Returns Decision or None."""
        if pred_valid:
            source, value = 'PREDICTED', float(predicted)
        elif queue_valid:
            source, value = 'FALLBACK', float(queue_count)
        else:
            self.source = None
            self.high_n = self.low_n = 0
            self.last_reason = 'NO_VALID_SIGNAL'
            return None
        if source != self.source:
            self.source = source
            self.high_n = self.low_n = 0
        self.high_n = self.high_n + 1 if value >= self.high else 0
        self.low_n = self.low_n + 1 if value < self.low else 0
        if self.high_n >= self.need_high:
            self.escalated = True
        if self.low_n >= self.need_low:
            self.escalated = False
        if not self.escalated:
            self.last_reason = 'BELOW_THRESHOLD'
            return None
        if value < self.high:        # falling demand: stay escalated, but no new bus
            self.last_reason = 'SIGNAL_BELOW_HIGH'
            return None
        if self.outstanding:
            self.last_reason = 'MISSION_OUTSTANDING'
            return None
        if now - self.last_dispatch < self.cooldown:
            self.last_reason = 'COOLDOWN'
            return None
        if not bus_idle:
            self.last_reason = 'BUS_UNAVAILABLE'
            return None
        self._n += 1
        corr = 'dock-%03d-%s' % (self._n, uuid4().hex[:6])
        self.outstanding = corr              # reserved before the goal is sent
        self.last_dispatch = now
        self.last_reason = source
        return Decision(source, corr)

    def finished(self, correlation_id):
        """Mission ended/rejected. Late or unknown ids are ignored."""
        if correlation_id and correlation_id == self.outstanding:
            self.outstanding = ''
            return True
        return False
