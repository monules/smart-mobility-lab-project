"""Robot escort / navigation logic for S7 (M4), pure Python: no ROS.

Escort state machine (assignment 14.2):

    IDLE -> TO_PICKUP -> WAIT_READY -> ESCORT -> ARRIVED
    pauses: BLOCKED, WAIT_CROSSING (motion zero, mission continues)
    terminal: CANCELLED, FAILED, SAFE_STOP

NavigateTo uses the same machinery with one leg (phase NAVIGATE).

* Routes come from /route/compute only. The core asks; the node calls the
  service. Each request has an attempt id: no answer within planner_timeout
  -> retry, at most planner_retries times, then FAILED PLANNER_UNAVAILABLE;
  answers to an older attempt are ignored (T18: no stale-path execution).
  NO_ROUTE -> FAILED with zero motion (T12).
* While moving, the remaining route is checked against the planner's edge
  rules (route_core.EdgeView, same inputs as route_planner). A removed edge
  ahead -> stop at the next node (never on the blocked edge) and replan from
  it; if the robot's own edge is removed, it stops and replans from the node
  it came from (reversal along the edge it just used). Movement resumes only
  with a newly validated route. Congestion-only changes: at most one replan
  every replan_period, switched only if >= replan_gain cheaper (13.2).
* Crossing C: ACQUIRE before the footprint reaches C (stop line otherwise,
  phase WAIT_CROSSING), RENEW every renew_period, RELEASE once clear. The
  permission is kept across replans while the robot is inside C.
* Progress = travelled / (travelled + remaining on the current leg). It can
  DECREASE after a detour, because the remaining distance always follows
  the new route (14.3).
"""
from dataclasses import dataclass
import math

from smart_hub import geometry as geo
from smart_hub import route_core
from smart_hub.bus_core import Path, wrap

IDLE, TO_PICKUP, WAIT_READY, ESCORT, ARRIVED = 'IDLE', 'TO_PICKUP', 'WAIT_READY', \
    'ESCORT', 'ARRIVED'
NAVIGATE, BLOCKED, WAIT_CROSSING, PLANNING = 'NAVIGATE', 'BLOCKED', 'WAIT_CROSSING', \
    'PLANNING'
CANCELLED, FAILED, SAFE_STOP = 'CANCELLED', 'FAILED', 'SAFE_STOP'


@dataclass
class RobotConfig:
    robot_id: str = 'robot_1'
    radius: float = 0.20
    cruise_speed: float = 0.22      # below the 0.25 m/s robot limit
    min_speed: float = 0.05
    slow_gain: float = 0.8
    turn_rate: float = 0.9          # rad/s, below the 1.0 rad/s supervisor cap
    heading_gain: float = 2.0
    align_tol: float = 0.35         # rad: no forward motion above this heading error
    lookahead: float = 0.30
    corner_tol: float = 0.08
    goal_tol: float = 0.20          # m, arrival (14.3)
    arrive_hold: float = 1.0        # s within tolerance before ARRIVED
    stopped_speed: float = 0.02
    odom_timeout: float = 0.5       # s
    stale_abort: float = 2.0        # s of stale odometry -> SAFE_STOP (T14 bounded result)
    ready_timeout: float = 20.0     # s at pickup (14.2 step 2)
    planner_timeout: float = 1.0    # s per attempt (19.2)
    planner_retries: int = 2        # retries after the first attempt
    acquire_ahead: float = 1.5
    stop_margin: float = 0.20
    clear_margin: float = 0.05
    retry_period: float = 0.5
    renew_period: float = 1.0
    clearance: float = 0.15         # m kept from obstacles along the next metre
    replan_period: float = 2.0      # s between congestion-only replans
    replan_gain: float = 0.15       # alternative must be this much cheaper
    snap_dist: float = 0.30         # m: robot counts as "at" a node

    def validate(self):
        bad = [n for n in ('radius', 'cruise_speed', 'min_speed', 'turn_rate', 'goal_tol',
                           'odom_timeout', 'ready_timeout', 'planner_timeout',
                           'renew_period', 'replan_period') if not getattr(self, n) > 0]
        if self.cruise_speed > 0.25:
            bad.append('cruise_speed (> 0.25 m/s robot limit)')
        if self.planner_retries < 0 or self.renew_period >= 3.0:
            bad.append('planner_retries / renew_period')
        if not 0 < self.replan_gain < 1:
            bad.append('replan_gain')
        if bad:
            raise ValueError('invalid robot parameters: %s' % ', '.join(bad))


@dataclass
class Output:
    cmd: tuple = (0.0, 0.0)
    crossing_op: tuple = None       # (operation, token)
    route_request: tuple = None     # (attempt id, start node, goal node)
    phase: str = IDLE
    reason: str = ''
    distance_remaining: float = -1.0
    progress: float = 0.0
    done: tuple = None              # (success, final phase, reason, final node, error m)


def nearest_node(x, y, limit):
    name, (nx, ny) = min(geo.NODES.items(), key=lambda kv: math.dist(kv[1], (x, y)))
    return name if math.dist((nx, ny), (x, y)) <= limit else ''


class RobotCore:
    def __init__(self, cfg=None, view=None):
        self.cfg = cfg or RobotConfig()
        self.cfg.validate()
        self.view = view or route_core.EdgeView()
        self.pose = None            # (x, y, theta, speed, t)
        self.safety_latched = False
        self.token = ''
        self.cross_inflight = False
        self.events = []
        self.attempt = 0
        self._clear()

    # ------------------------------------------------------------- inputs
    def on_odom(self, x, y, theta, speed, now):
        if self.pose is not None and self.mission:
            self.travelled += math.dist(self.pose[:2], (x, y))
        self.pose = (x, y, theta, speed, now)

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
        elif operation == 'RELEASE' and (granted or reason == 'BAD_TOKEN'):
            self.events.append(('CROSSING_RELEASED', reason or 'OK'))
            self.token = ''

    def on_route(self, attempt, success, nodes, cost, reason, now):
        """Planner answer for request `attempt`; late answers are ignored."""
        req = self.request
        if req is None or attempt != req['id']:
            self.events.append(('ROUTE_LATE_IGNORED', 'attempt %d' % attempt))
            return
        self.request = None
        if req['why'] == 'congestion':
            self._maybe_switch(nodes if success else None, cost, now)
            return
        if not success:
            self.fail_reason = reason or 'NO_ROUTE'
            return
        if req['why'] == 'leg' and now + cost > self.mission['deadline']:
            self.fail_reason = 'DEADLINE_INFEASIBLE'        # 14.2 step 1
            return
        self._set_route(list(nodes), now, req['why'])

    # ------------------------------------------------------------- goals
    def _clear(self):
        self.mission = None         # dict: kind, legs, deadline
        self.phase, self.reason = IDLE, ''
        self.path = None
        self.path_nodes = []
        self.seg, self.s = 0, 0.0
        self.travelled = 0.0
        self.request = None
        self.fail_reason = ''
        self.cancel_requested = False
        self.ready_since = None
        self.ready = False
        self.arrive_since = None
        self.stale_since = None
        self.deny_reason = ''
        self.last_try = self.last_renew = -math.inf
        self.last_congestion = -math.inf
        self.last_node = ''
        self._arrived = False

    @property
    def active(self):
        return self.mission is not None

    def start(self, kind, legs, now, max_duration):
        """Start a mission: 'escort' (legs [pickup, destination]) or 'navigate' ([goal])."""
        if self.active:
            raise RuntimeError('BUSY')
        if self.pose is None or now - self.pose[4] > self.cfg.odom_timeout:
            raise RuntimeError('ODOM_UNKNOWN')
        here = nearest_node(self.pose[0], self.pose[1], self.cfg.snap_dist)
        if not here:
            raise RuntimeError('OFF_GRAPH')
        self._clear()
        self.mission = {'kind': kind, 'legs': list(legs), 'leg': 0,
                        'deadline': now + float(max_duration), 'start': now}
        self.last_node = here
        self._begin_leg(now)

    def _begin_leg(self, now):
        m = self.mission
        goal = m['legs'][m['leg']]
        here = nearest_node(self.pose[0], self.pose[1], self.cfg.snap_dist)
        phase = TO_PICKUP if m['kind'] == 'escort' and m['leg'] == 0 else \
            ESCORT if m['kind'] == 'escort' else NAVIGATE
        if here == goal and phase == TO_PICKUP:      # already at the pickup
            self._leg_done(now)
            return
        self._set(phase, 'PLANNING')
        if here == goal:                             # settle on the goal node
            self._set_route([goal], now, 'leg')
            return
        self._ask(here or self.last_node, goal, now, 'leg')

    def _leg_done(self, now):
        m = self.mission
        if m['kind'] == 'escort' and m['leg'] == 0:
            m['leg'] = 1
            self.path = None
            self.ready_since, self.ready = now, False
            self._set(WAIT_READY, 'PROMPT_PASSENGER')
            self.events.append(('PASSENGER_PROMPT', 'press /robot/confirm_ready within %.0f s'
                                % self.cfg.ready_timeout))
            return
        self._arrived = True

    def confirm_ready(self):
        """/robot/confirm_ready: accepted only while waiting at pickup."""
        if self.phase != WAIT_READY:
            self.events.append(('READY_REJECTED', 'not waiting at pickup (phase %s)'
                                % self.phase))
            return False, 'NOT_WAITING_AT_PICKUP (phase %s)' % self.phase
        self.ready = True
        self.events.append(('PASSENGER_READY', 'confirmed'))
        return True, 'READY_ACCEPTED'

    def cancel(self):
        self.cancel_requested = True

    def reset(self):
        self._clear()
        self.pose = None
        self.safety_latched = False

    def _ask(self, start, goal, now, why):
        self.attempt += 1
        self.request = {'id': self.attempt, 'start': start, 'goal': goal, 'why': why,
                        'sent': now, 'tries': 1, 'emit': True}

    def _set(self, phase, reason):
        if phase != self.phase:
            self.events.append(('ROBOT_PHASE', '%s->%s %s' % (self.phase, phase, reason)))
        self.phase, self.reason = phase, reason

    def _set_route(self, nodes, now, why):
        x, y = self.pose[0], self.pose[1]
        pts = [geo.NODES[n] for n in nodes]
        names = list(nodes)
        if math.dist((x, y), pts[0]) > 0.05:
            pts.insert(0, (x, y))
            names.insert(0, None)
        if len(pts) == 1:
            pts.append(pts[0])
            names.append(nodes[0])
        old = '-'.join(n for n in self.path_nodes if n) if self.path else ''
        self.path = Path(pts, self.cfg.radius)
        self.path_nodes = names
        self.seg, self.s = 0, 0.0
        self.arrive_since = None
        if why != 'leg':
            self.events.append(('REROUTE', '%s: %s -> %s' % (why, old, '-'.join(nodes))))
        self.events.append(('ROUTE', '-'.join(nodes)))

    def _remaining_nodes(self):
        return [n for n in self.path_nodes[self.seg + 1:] if n]

    def _maybe_switch(self, nodes, cost, now):
        if not nodes or self.path is None:
            return
        rest = self._remaining_nodes()
        if nodes == rest:
            return
        costs = self.view.costs('robot', now, agent_id=self.cfg.robot_id)
        current, _ = route_core.path_cost(costs, rest)
        if cost <= (1.0 - self.cfg.replan_gain) * current:
            self._set_route(nodes, now, 'CHEAPER %.1f s < %.1f s' % (cost, current))

    # ------------------------------------------------------------- helpers
    def _in_c(self, x, y, margin=0.0):
        return geo.circle_overlaps_rect(x, y, self.cfg.radius + margin, geo.CROSSING)

    def _release_if_clear(self, out):
        if self.token and self.pose is not None and not self.cross_inflight \
                and not self._in_c(self.pose[0], self.pose[1], self.cfg.clear_margin):
            out.crossing_op = ('RELEASE', self.token)

    def _end(self, out, success, final, reason):
        self._release_if_clear(out)
        x, y = self.pose[:2] if self.pose else (math.nan, math.nan)
        goal = self.mission['legs'][-1]
        node = nearest_node(x, y, self.cfg.goal_tol) if self.pose else ''
        err = math.dist((x, y), geo.NODES[goal]) if self.pose else -1.0
        self._set(final, reason)
        out.phase, out.reason = final, reason
        out.done = (success, final, reason, node, err, self.travelled)
        self.mission, self.path, self.request = None, None, None
        return out

    def _track(self, x, y):
        p = self.path
        while self.seg < len(p.seg_len) - 1:
            corner = self.seg + 1
            reached = math.dist((x, y), p.p[corner]) < self.cfg.corner_tol \
                if corner in p.corners else p.project(self.seg, x, y) >= p.cum[corner] - 1e-6
            if not reached:
                break
            self.seg += 1
            if self.path_nodes[self.seg]:
                self.last_node = self.path_nodes[self.seg]
        self.s = max(self.s, p.project(self.seg, x, y))
        return self.s

    def _validate_route(self, now):
        """Return (stop arc length or None, replan start node) for removed edges."""
        names = self.path_nodes
        costs = self.view.costs('robot', now, agent_id=self.cfg.robot_id)
        here_edge = None
        if names[self.seg] and names[self.seg + 1]:
            here_edge = geo.edge_id(names[self.seg], names[self.seg + 1])
        if here_edge and costs[here_edge][0] is None and \
                self.s < self.path.cum[self.seg + 1] - 0.05:
            return self.s, names[self.seg], here_edge          # on it: back off
        nodes = [n for n in names[self.seg + 1:] if n]
        _, bad = route_core.path_cost(costs, nodes)
        if bad:
            return self.path.cum[self.seg + 1], names[self.seg + 1], bad
        return None, '', ''

    def _obstacle_ahead(self, s):
        p, c = self.path, self.cfg
        for k in range(21):
            px, py = p.point(s + 0.05 * k)
            for ox, oy, orad in self.view.obstacles:
                if math.hypot(px - ox, py - oy) < orad + c.radius + c.clearance:
                    return True
        return False

    def _crossing(self, now, s, x, y, out):
        p, c = self.path, self.cfg
        if p.c_in is None:
            return True
        if self.token:
            if s > p.c_in and not self._in_c(x, y, c.clear_margin):
                if not self.cross_inflight:
                    out.crossing_op = ('RELEASE', self.token)
            elif now - self.last_renew >= c.renew_period and not self.cross_inflight:
                out.crossing_op = ('RENEW', self.token)
            return True
        if s >= p.c_out:
            return True
        stop = p.c_in - c.stop_margin
        if stop - s <= c.acquire_ahead and not self.cross_inflight \
                and now - self.last_try >= c.retry_period:
            self.last_try = now
            out.crossing_op = ('ACQUIRE', '')
        return False

    def _planner(self, now, out):
        """Emit / retry the pending route request; False once it has failed."""
        req, c = self.request, self.cfg
        if req is None:
            return True
        if req['emit']:
            req['emit'] = False
            out.route_request = (req['id'], req['start'], req['goal'])
        elif now - req['sent'] > c.planner_timeout:
            if req['tries'] > c.planner_retries:
                self.events.append(('PLANNER_TIMEOUT', 'no answer after %d attempts'
                                    % req['tries']))
                if req['why'] == 'congestion':
                    self.request = None      # optional replan: keep the valid route
                    return True
                return False
            self.events.append(('PLANNER_RETRY', 'attempt %d timed out' % req['tries']))
            self.attempt += 1
            req.update(id=self.attempt, sent=now, tries=req['tries'] + 1)
            out.route_request = (req['id'], req['start'], req['goal'])
        return True

    # ------------------------------------------------------------- 20 Hz
    def step(self, now):
        c, out = self.cfg, Output()
        if not self.active:
            out.phase = self.phase
            self._release_if_clear(out)
            return out
        self._arrived = False
        if self.cancel_requested:
            return self._end(out, False, CANCELLED, 'CANCELED')
        if now > self.mission['deadline']:
            return self._end(out, False, FAILED, 'TIMEOUT')
        if self.safety_latched:
            return self._end(out, False, SAFE_STOP, 'SAFETY_LATCHED')
        if self.pose is None or now - self.pose[4] > c.odom_timeout:
            self.stale_since = self.stale_since or now
            if now - self.stale_since > c.stale_abort:
                return self._end(out, False, SAFE_STOP, 'ODOM_STALE')
            out.phase, out.reason = self.phase, 'ODOM_STALE'
            return out                                     # zero command
        self.stale_since = None
        if self.fail_reason:
            return self._end(out, False, FAILED, self.fail_reason)
        if not self._planner(now, out):
            return self._end(out, False, FAILED, 'PLANNER_UNAVAILABLE')
        if self.phase == WAIT_READY:
            out.phase, out.reason = self.phase, self.reason
            if self.ready:
                self._begin_leg(now)
                self._planner(now, out)
            elif now - self.ready_since > c.ready_timeout:
                return self._end(out, False, FAILED, 'READY_TIMEOUT')
            return out
        if self.path is None:                              # waiting for the first route
            out.phase, out.reason = self.phase, 'PLANNING'
            return out
        return self._drive(now, out)

    def _drive(self, now, out):
        c, p = self.cfg, self.path
        x, y, theta, speed, _ = self.pose
        s = self._track(x, y)
        remaining = max(0.0, p.total - s)
        out.distance_remaining = remaining
        out.progress = self.travelled / (self.travelled + remaining) \
            if self.travelled + remaining > 0 else 1.0
        base = ESCORT if self.mission['kind'] == 'escort' and self.mission['leg'] == 1 else \
            TO_PICKUP if self.mission['kind'] == 'escort' else NAVIGATE

        stops = [p.cum[k] for k in p.corners if p.cum[k] > s - c.corner_tol] + [p.total]
        hold, pause = None, None
        stop_at, start_node, bad = self._validate_route(now)
        if bad:
            if self.request is None or self.request['why'] == 'congestion':
                self.events.append(('ROUTE_INVALID', 'edge %s removed (%s)' % (
                    bad, self.view.costs('robot', now)[bad][1])))
                self._ask(start_node, self.mission['legs'][self.mission['leg']], now,
                          'BLOCKED:%s' % bad)
                self._planner(now, out)
            hold, pause = stop_at, (BLOCKED, 'ROUTE_BLOCKED:%s' % bad)
        elif self.request is None and now - self.last_congestion >= c.replan_period \
                and self._remaining_nodes():
            self.last_congestion = now
            self._ask(self._remaining_nodes()[0], self.mission['legs'][self.mission['leg']],
                      now, 'congestion')
            self._planner(now, out)
        if self._obstacle_ahead(s):
            hold, pause = s, (BLOCKED, 'OBSTACLE_CLEARANCE')
        if not self._crossing(now, s, x, y, out) and p.c_in is not None \
                and s < p.c_in and not self._in_c(x, y):
            # outside C without permission: hold at the stop line, or right here
            # if a new route (e.g. a reversal) starts closer to C than that
            line = max(p.c_in - c.stop_margin, s)
            hold = line if hold is None else min(hold, line)
            if pause is None and line - s < 0.05:
                pause = (WAIT_CROSSING, self.deny_reason or 'CROSSING_PENDING')
        if hold is not None:
            stops.append(hold)

        v = c.cruise_speed
        for sp in stops:
            d = sp - s
            if d < 0.02:
                if sp == hold or sp == p.total:
                    v = 0.0
                continue
            v = min(v, max(c.min_speed, c.slow_gain * d))
        nxt = min([p.cum[k] for k in p.corners if p.cum[k] > s + 1e-6] + [p.total])
        target = min(s + c.lookahead, nxt)
        if target - s < 0.05:
            want = p.heading[min(self.seg, len(p.heading) - 1)]
        else:
            tx, ty = p.point(target)
            want = math.atan2(ty - y, tx - x)
        err = wrap(want - theta)
        at_goal = remaining < 0.03
        if at_goal:
            v, err = 0.0, 0.0
        w = max(-c.turn_rate, min(c.turn_rate, c.heading_gain * err))
        if abs(err) > c.align_tol:
            v = 0.0
        if hold is not None and hold - s < 0.02 and v == 0.0 and abs(err) <= c.align_tol:
            w = 0.0                                     # paused: no spinning on the spot
        out.cmd = (v, w)
        self._set(*(pause or (base, 'ON_ROUTE')))

        goal = self.path.p[-1]
        if math.dist((x, y), goal) <= c.goal_tol and speed < c.stopped_speed and at_goal:
            self.arrive_since = self.arrive_since or now
            if now - self.arrive_since >= c.arrive_hold:
                out.cmd = (0.0, 0.0)
                self._leg_done(now)
                if self._arrived:
                    self.events.append(('ROBOT_ARRIVED', '%s error %.3f m' % (
                        self.mission['legs'][-1], math.dist((x, y), goal))))
                    return self._end(out, True, ARRIVED, 'ARRIVED')
        else:
            self.arrive_since = None
        out.phase, out.reason = self.phase, self.reason
        return out
