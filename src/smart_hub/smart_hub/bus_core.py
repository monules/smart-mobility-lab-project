"""Bus docking logic for S3 (M2-A), pure Python: no ROS, time is passed in.

State machine (assignment 10.2):

    IDLE -> APPROACH -> [WAIT] -> DOCK -> BOARD -> COMPLETE
                 any moving state -> SAFE_STOP

* APPROACH: follow the route (A-N-Cn-B baseline) with a speed profile that
  slows for corners, the crossing stop line and the bay.
* WAIT: stopped at the stop line in front of crossing C because the crossing
  was not granted yet or bay B is blocked (no permission is held while the
  bay is blocked, so the bus never blocks the crossing for others).
* DOCK: final approach (last dock_zone metres); docked only after position
  <= 0.20 m, yaw <= 0.17 rad and measured speed < 0.02 m/s held for 1 s.
* BOARD: door requested only when docked; the 8 s boarding interval starts
  when the bridge reports the door open; then the door is closed and the goal
  completes once the door is measured closed.
* SAFE_STOP: zero command. A stale bus pose holds the bus and resumes when
  the pose is fresh again (abort after stale_abort_sec); cancel, timeout, a
  latched safety stop or a door / docking failure end the goal here.

Crossing C (7.1): ACQUIRE before the swept footprint reaches C, RENEW every
renew_period while holding it, RELEASE once the footprint has left C.

ETA (10.2 step 2): remaining route length / smoothed measured speed + the
remaining corner turn time + a declared final-approach allowance. Invalid
(-1) while waiting, stale or before a credible speed exists.
"""
from dataclasses import dataclass
import math

from smart_hub import geometry as geo

IDLE, APPROACH, WAIT, DOCK, BOARD = 'IDLE', 'APPROACH', 'WAIT', 'DOCK', 'BOARD'
COMPLETE, SAFE_STOP = 'COMPLETE', 'SAFE_STOP'
MOVING = (APPROACH, WAIT, DOCK)
BASELINE_ROUTE = ('A', 'N', 'Cn', 'B')


@dataclass
class BusConfig:
    bus_id: str = 'bus_1'
    radius: float = 0.30            # bus footprint (assignment 2.2)
    cruise_speed: float = 0.35      # below the 0.40 m/s bus limit
    min_speed: float = 0.06         # creep speed near a stop point
    slow_gain: float = 0.8          # v <= slow_gain * distance to the next stop point
    turn_rate: float = 0.5          # rad/s, the bus turn limit
    heading_gain: float = 1.5
    align_tol: float = 0.30         # rad: rotate in place above this heading error
    lookahead: float = 0.40         # m, pursuit point ahead on the path
    corner_tol: float = 0.10        # m, corner reached
    pos_tol: float = 0.20           # m, docking (10.3)
    yaw_tol: float = 0.17           # rad, docking
    stopped_speed: float = 0.02     # m/s
    dock_hold: float = 1.0          # s, tolerance held before "docked"
    dock_zone: float = 1.0          # m remaining: APPROACH -> DOCK
    dock_timeout: float = 6.0       # s stopped at the bay without meeting tolerance
    board_sec: float = 8.0          # door-open boarding interval (10.2 step 5)
    door_timeout: float = 3.0       # s for the bridge to report the door state
    pose_timeout: float = 0.5       # s, bus pose older than this = stale
    stale_abort: float = 5.0        # s stale before the goal is aborted
    acquire_ahead: float = 2.5      # m before the stop line: start asking for C
    stop_margin: float = 0.25       # m between footprint and C at the stop line
    clear_margin: float = 0.05      # m, footprint must be this far out of C to release
    retry_period: float = 0.5       # s between ACQUIRE attempts
    renew_period: float = 1.0       # s between RENEW calls (lease 3 s)
    bay_clear_radius: float = 0.8   # m around B that must be free of other bodies
    eta_tau: float = 2.0            # s, speed smoothing time constant
    eta_min_speed: float = 0.05     # m/s, below this the speed is not credible
    approach_allowance: float = 1.5  # s, declared extra time of the final slow-down

    def validate(self):
        bad = [n for n in ('radius', 'cruise_speed', 'min_speed', 'slow_gain', 'turn_rate',
                           'pos_tol', 'yaw_tol', 'dock_hold', 'board_sec', 'pose_timeout',
                           'renew_period', 'eta_tau') if not getattr(self, n) > 0]
        if self.cruise_speed > 0.40:
            bad.append('cruise_speed (> 0.40 m/s bus limit)')
        if self.min_speed >= self.cruise_speed:
            bad.append('min_speed')
        if self.renew_period >= 3.0:
            bad.append('renew_period (must be below the 3 s lease)')
        if bad:
            raise ValueError('invalid bus parameters: %s' % ', '.join(bad))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def node_route(names=BASELINE_ROUTE):
    return [geo.NODES[n] for n in names]


class Path:
    """Polyline with arc length, corners and the crossing span."""

    def __init__(self, points, radius):
        self.p = [tuple(map(float, pt)) for pt in points]
        if len(self.p) < 2:
            raise ValueError('a route needs at least two points')
        self.seg_len = [math.dist(a, b) for a, b in zip(self.p, self.p[1:])]
        self.cum = [0.0]
        for d in self.seg_len:
            self.cum.append(self.cum[-1] + d)
        self.total = self.cum[-1]
        self.heading = [math.atan2(b[1] - a[1], b[0] - a[0]) for a, b in zip(self.p, self.p[1:])]
        # corners where the bus must stop and turn (colinear points are not)
        self.corners = {k: abs(wrap(self.heading[k] - self.heading[k - 1]))
                        for k in range(1, len(self.p) - 1)
                        if abs(wrap(self.heading[k] - self.heading[k - 1])) > 0.2}
        inside = [s for s in self.samples(0.02)
                  if geo.circle_overlaps_rect(*self.point(s), radius, geo.CROSSING)]
        self.c_in = inside[0] if inside else None     # arc length where C begins
        self.c_out = inside[-1] if inside else None

    def samples(self, step):
        n = int(self.total / step)
        return [i * step for i in range(n + 1)] + [self.total]

    def point(self, s):
        s = max(0.0, min(self.total, s))
        for k, length in enumerate(self.seg_len):
            if s <= self.cum[k + 1] or k == len(self.seg_len) - 1:
                f = 0.0 if length == 0 else (s - self.cum[k]) / length
                (ax, ay), (bx, by) = self.p[k], self.p[k + 1]
                return ax + f * (bx - ax), ay + f * (by - ay)

    def project(self, k, x, y):
        """Arc length of (x, y) projected onto segment k (clipped)."""
        (ax, ay), (bx, by) = self.p[k], self.p[k + 1]
        length = self.seg_len[k]
        t = 0.0 if length == 0 else ((x - ax) * (bx - ax) + (y - ay) * (by - ay)) / length
        return self.cum[k] + max(0.0, min(length, t))

    def turn_time_after(self, s, rate):
        return sum(a / rate for k, a in self.corners.items() if self.cum[k] > s + 1e-6)


@dataclass
class Output:
    cmd: tuple = (0.0, 0.0)
    door_request: bool = False
    crossing_op: tuple = None       # (operation, token) to send now, or None
    phase: str = IDLE
    reason: str = ''
    distance_remaining: float = -1.0
    eta: float = -1.0
    eta_valid: bool = False
    done: tuple = None              # (success, final_state, reason) when the goal ends


class BusCore:
    def __init__(self, cfg=None):
        self.cfg = cfg or BusConfig()
        self.cfg.validate()
        self.pose = None            # (x, y, theta, speed, t)
        self.others = []            # [(id, x, y)] other bodies, latest snapshot
        self.door_open = None       # (bool, t) measured
        self.safety_latched = False
        self.events = []            # (event, reason) for /hub/events
        self.token = ''             # crossing permission held (survives a goal end)
        self.cross_inflight = False
        self._clear_goal()

    # ------------------------------------------------------------- inputs
    def on_pose(self, x, y, theta, speed, now):
        self.pose = (x, y, theta, speed, now)

    def on_others(self, bodies):
        self.others = list(bodies)

    def on_door(self, is_open, now):
        self.door_open = (bool(is_open), now)

    def on_safety(self, latched):
        self.safety_latched = bool(latched)

    def on_crossing_response(self, operation, granted, token, reason, now):
        self.cross_inflight = False
        if operation in ('ACQUIRE', 'RENEW'):
            if granted:
                if not self.token:
                    self.events.append(('CROSSING_GRANTED', reason or 'OK'))
                self.token, self.last_renew = token or self.token, now
            else:
                if self.token:
                    self.events.append(('CROSSING_LOST', '%s %s' % (operation, reason)))
                self.token, self.deny_reason = '', reason or 'DENIED'
        elif operation == 'RELEASE':
            if granted or reason == 'BAD_TOKEN':     # BAD_TOKEN: lease already gone
                self.events.append(('CROSSING_RELEASED', reason or 'OK'))
                self.token = ''

    # ------------------------------------------------------------- goal
    def _clear_goal(self):
        self.phase, self.reason = IDLE, ''
        self.path = None
        self.seg = 0
        self.s = 0.0
        self.goal_start = self.deadline = None
        self.v_smooth = None
        self.last_try = -math.inf
        self.last_renew = -math.inf
        self.deny_reason = ''
        self.dock_since = None
        self.stop_since = None
        self.door_asked = self.board_start = None
        self.closing_since = None
        self.stale_since = None
        self.before_stale = None
        self.cancel_requested = False
        self.final_error = None     # (position m, yaw rad) when docked

    @property
    def active(self):
        return self.phase not in (IDLE, COMPLETE) and self.path is not None

    def start(self, route, now, max_duration):
        """Begin a docking goal on route [(x, y), ...] ending at the bay."""
        if self.active:
            raise RuntimeError('a goal is already active')
        if self.pose is None:
            raise RuntimeError('no bus pose yet')
        pts = list(route)
        x, y = self.pose[0], self.pose[1]
        if math.dist((x, y), pts[0]) > 0.05:
            pts.insert(0, (x, y))        # start from where the bus really is
        self._clear_goal()
        self.path = Path(pts, self.cfg.radius)
        self.goal_start, self.deadline = now, now + float(max_duration)
        self._set(APPROACH, 'GOAL_ACCEPTED')

    def cancel(self):
        self.cancel_requested = True

    def reset(self):
        """Scenario reset: forget the goal and every input.

        The crossing token is kept so that the node can still release it.
        """
        self._clear_goal()
        self.pose = self.door_open = None
        self.others = []
        self.safety_latched = False

    def _set(self, phase, reason):
        if phase != self.phase or reason != self.reason:
            if phase != self.phase:
                self.events.append(('BUS_PHASE', '%s->%s %s' % (self.phase, phase, reason)))
            self.phase, self.reason = phase, reason

    def _end(self, out, success, final, reason):
        self._set(final, reason)
        out.done = (success, final, reason)
        out.phase, out.reason = final, reason
        self.path = None
        return out

    # ------------------------------------------------------------- helpers
    def _pose_fresh(self, now):
        return self.pose is not None and now - self.pose[4] <= self.cfg.pose_timeout

    def _door_state(self, now):
        if self.door_open is None or now - self.door_open[1] > self.cfg.door_timeout:
            return None
        return self.door_open[0]

    def _in_c(self, x, y, margin=0.0):
        return geo.circle_overlaps_rect(x, y, self.cfg.radius + margin, geo.CROSSING)

    def bay_blocked(self):
        bx, by = self.path.p[-1] if self.path else geo.NODES['B']
        return any(math.hypot(ox - bx, oy - by) < self.cfg.bay_clear_radius
                   for _, ox, oy in self.others)

    def _track(self, x, y):
        """Advance the segment index and return the arc length of the bus."""
        p = self.path
        while self.seg < len(p.seg_len) - 1:
            corner = self.seg + 1
            if corner in p.corners:
                if math.dist((x, y), p.p[corner]) < self.cfg.corner_tol:
                    self.seg += 1
                    continue
                break
            if p.project(self.seg, x, y) >= p.cum[corner] - 1e-6:
                self.seg += 1
                continue
            break
        # never go backwards along the route (pose noise near a corner)
        self.s = max(self.s, p.project(self.seg, x, y))
        return self.s

    def _crossing(self, now, s, x, y, out):
        """Decide the crossing request; return True if the bus may pass the stop line."""
        p, c = self.path, self.cfg
        if p.c_in is None:
            return True
        if self.token:
            # release once the footprint has left C behind the bus
            if s > p.c_in and not self._in_c(x, y, c.clear_margin):
                if not self.cross_inflight:
                    out.crossing_op = ('RELEASE', self.token)
            elif now - self.last_renew >= c.renew_period and not self.cross_inflight:
                out.crossing_op = ('RENEW', self.token)
            return True
        if s >= p.c_out:
            return True                    # already past C (released or never needed)
        stop = p.c_in - c.stop_margin
        if self.bay_blocked():
            self.deny_reason = 'BAY_BLOCKED'
            return False
        if stop - s <= c.acquire_ahead and not self.cross_inflight \
                and now - self.last_try >= c.retry_period:
            self.last_try = now
            out.crossing_op = ('ACQUIRE', '')
        return False

    # ------------------------------------------------------------- 20 Hz
    def step(self, now):
        c, out = self.cfg, Output()
        if self.path is None:
            out.phase = self.phase
            if self.token and self.pose is not None and not self.cross_inflight \
                    and not self._in_c(self.pose[0], self.pose[1], c.clear_margin):
                out.crossing_op = ('RELEASE', self.token)   # left over from an ended goal
            return out
        if self.cancel_requested:
            self._release_if_clear(out)
            return self._end(out, False, SAFE_STOP, 'CANCELED')
        if now > self.deadline:
            self._release_if_clear(out)
            return self._end(out, False, SAFE_STOP, 'TIMEOUT')
        if self.safety_latched:
            self._release_if_clear(out)
            return self._end(out, False, SAFE_STOP, 'SAFETY_LATCHED')
        if not self._pose_fresh(now):
            self.stale_since = self.stale_since or now
            if now - self.stale_since > c.stale_abort:
                return self._end(out, False, SAFE_STOP, 'POSE_STALE')
            if self.before_stale is None:
                self.before_stale = (self.phase, self.reason)
            self._set(SAFE_STOP, 'POSE_STALE')
            out.phase, out.reason = self.phase, self.reason
            out.door_request = self.before_stale[0] == BOARD and self.board_start is not None
            return out
        if self.stale_since is not None:
            self.stale_since = None
            self._set(*self.before_stale)          # resume where the bus was
            self.before_stale = None

        x, y, theta, speed, _ = self.pose
        p = self.path
        if self.phase == BOARD:
            return self._board(now, speed, out)
        s = self._track(x, y)
        remaining = max(0.0, p.total - s)
        out.distance_remaining = remaining
        may_pass = self._crossing(now, s, x, y, out)
        door = self._door_state(now)
        if door is not False:              # propulsion only with a confirmed closed door
            out.phase, out.reason = self.phase, 'DOOR_NOT_CLOSED'
            return out

        # speed limit from the next stop points ahead
        stops = [p.cum[k] for k in p.corners if p.cum[k] > s - c.corner_tol]
        stops.append(p.total)
        line = None
        if not may_pass and p.c_in is not None and s <= p.c_in - c.stop_margin + 0.05:
            line = p.c_in - c.stop_margin       # never stop inside C: once past, go on
            stops.append(line)
        v = c.cruise_speed
        for sp in stops:
            d = sp - s
            if d < 0.02:
                if sp == line or sp == p.total:
                    v = 0.0
                continue
            v = min(v, max(c.min_speed, c.slow_gain * d))

        # heading: aim at a point ahead, never past the next corner
        nxt = min([p.cum[k] for k in p.corners if p.cum[k] > s + 1e-6] + [p.total])
        target = min(s + c.lookahead, nxt)
        if target - s < 0.05:
            want = p.heading[min(self.seg, len(p.heading) - 1)]
        else:
            tx, ty = p.point(target)
            want = math.atan2(ty - y, tx - x)
        err = wrap(want - theta)
        if remaining < 0.03:
            v, err = 0.0, wrap(p.heading[-1] - theta)
        w = max(-c.turn_rate, min(c.turn_rate, c.heading_gain * err))
        if abs(err) > c.align_tol:
            v = 0.0                        # turn in place first
        out.cmd = (v, w)

        # phase and ETA
        at_line = line is not None and line - s < 0.05 and speed < c.stopped_speed
        if at_line:
            self._set(WAIT, self.deny_reason or 'CROSSING_PENDING')
        elif remaining <= c.dock_zone:
            self._set(DOCK, 'FINAL_APPROACH')
        else:
            self._set(APPROACH, 'ON_ROUTE')
        if v > 0 and speed > c.eta_min_speed:
            a = min(1.0, 0.05 / c.eta_tau)
            self.v_smooth = speed if self.v_smooth is None else \
                self.v_smooth + a * (speed - self.v_smooth)
        if self.phase != WAIT and self.v_smooth and self.v_smooth >= c.eta_min_speed:
            out.eta = remaining / self.v_smooth + p.turn_time_after(s, c.turn_rate) \
                + c.approach_allowance
            out.eta_valid = True

        # docked?
        if remaining < 0.05 + c.pos_tol:
            pos_err = math.dist((x, y), p.p[-1])
            yaw_err = abs(wrap(theta - p.heading[-1]))
            ok = pos_err <= c.pos_tol and yaw_err <= c.yaw_tol and speed < c.stopped_speed
            if ok and self.dock_since is None:
                self.dock_since = now
            elif not ok:
                self.dock_since = None
            if speed >= c.stopped_speed:
                self.stop_since = None
            elif self.stop_since is None:
                self.stop_since = now
            if self.dock_since is not None and now - self.dock_since >= c.dock_hold:
                self.final_error = (pos_err, yaw_err)
                self.events.append(('BUS_DOCKED', 'pos_err=%.3f yaw_err=%.3f'
                                    % (pos_err, yaw_err)))
                self._set(BOARD, 'DOCKED')
                out.cmd = (0.0, 0.0)
            elif self.stop_since is not None and now - self.stop_since > c.dock_timeout:
                return self._end(out, False, SAFE_STOP, 'DOCK_TOLERANCE')
        out.phase, out.reason = self.phase, self.reason
        return out

    def _release_if_clear(self, out):
        if self.token and self.pose is not None and not self.cross_inflight \
                and not self._in_c(self.pose[0], self.pose[1], self.cfg.clear_margin):
            out.crossing_op = ('RELEASE', self.token)

    def _board(self, now, speed, out):
        """Docked: open the door, board for board_sec, close, complete."""
        c = self.cfg
        out.distance_remaining, out.eta, out.eta_valid = 0.0, 0.0, True
        door = self._door_state(now)
        if self.door_asked is None:
            self.door_asked = now
        if self.board_start is None:
            out.door_request = True
            if door:
                self.board_start = now
                self.events.append(('DOOR_OPENED', 'boarding %.0f s' % c.board_sec))
            elif now - self.door_asked > c.door_timeout:
                return self._end(out, False, SAFE_STOP, 'DOOR_DID_NOT_OPEN')
        elif now - self.board_start < c.board_sec:
            out.door_request = True
        else:
            self.closing_since = self.closing_since or now
            if door is False:
                self.events.append(('DOOR_CLOSED', 'boarding done'))
                out.phase, out.reason = self.phase, self.reason
                return self._end(out, True, COMPLETE, 'DOCKED_AND_BOARDED')
            if now - self.closing_since > c.door_timeout:
                return self._end(out, False, SAFE_STOP, 'DOOR_DID_NOT_CLOSE')
        out.phase, out.reason = self.phase, self.reason
        return out
