"""Safety supervisor decision logic (pure Python, no ROS).

safety_supervisor.py feeds incoming messages into SafetyCore and calls
tick() 20 times a second. Every time value is seconds from time.monotonic(),
passed in by the caller, so unit tests control time exactly.

Two kinds of stop (assignment 16.2 + 18.7; the split is an M5 design
decision, see README_M5.md):

* LATCHED stop: e-stop, boundary breach, collision. Every agent is held at
  zero until /safety/clear_stop succeeds. A later e-stop "false" never clears
  it, and clearing never restarts motion by itself.
* HOLD: stale command, stale sensor, unknown e-stop signal, door open,
  insufficient clearance ahead, missing crossing permission. Only the
  affected agent(s) are zeroed, and only while the condition lasts.

Fail-safe rule used everywhere: missing or stale data is treated as unsafe.
"""
from dataclasses import dataclass, field
import math

from smart_hub import geometry as geo

INF = float('inf')


@dataclass
class SafetyConfig:
    robot_id: str = 'robot_1'
    bus_id: str = 'bus_1'
    robot_speed_limit: float = 0.25    # m/s, assignment 3.3
    bus_speed_limit: float = 0.40      # m/s, assignment 3.3
    robot_turn_limit: float = 1.0      # rad/s, M5 choice (not in assignment)
    bus_turn_limit: float = 0.5        # rad/s, M5 choice (not in assignment)
    command_timeout: float = 0.5       # s, assignment 3.3 / 16.1
    sensor_timeout: float = 1.0        # s, assignment 3.3 / 16.1
    estop_timeout: float = 1.0         # s, /hub/estop is sent at 10 Hz (5)
    future_tolerance: float = 0.2      # s, assignment 5: reject future stamps
    stopped_speed: float = 0.02        # m/s, "stopped" threshold (10.2)
    clearance: float = 0.20            # m, added clearance (3.3)
    lookahead: float = 0.5             # s, motion-footprint horizon
    require_estop_heartbeat: bool = True
    enforce_crossing: bool = True

    def validate(self):
        bad = []
        positive = ('robot_speed_limit', 'bus_speed_limit', 'robot_turn_limit',
                    'bus_turn_limit', 'command_timeout', 'sensor_timeout',
                    'estop_timeout', 'stopped_speed', 'lookahead')
        for name in positive:
            value = getattr(self, name)
            if not (math.isfinite(value) and value > 0):
                bad.append(name)
        if not 0.0 <= self.clearance <= 2.0:
            bad.append('clearance')
        if not 0.0 <= self.future_tolerance <= 1.0:
            bad.append('future_tolerance')
        if self.robot_id == self.bus_id or not self.robot_id or not self.bus_id:
            bad.append('robot_id/bus_id')
        if bad:
            raise ValueError('invalid safety parameter(s): ' + ', '.join(bad))


@dataclass
class Sample:
    """A value plus the monotonic time it was observed."""

    value: object
    t: float


def age(sample, now):
    """Seconds since the sample was observed; infinite if never received."""
    return INF if sample is None else now - sample.t


@dataclass
class Agent:
    name: str              # 'robot' or 'bus' (topic prefix)
    vehicle_id: str        # id used in /hub/vehicles and /crossing/request
    radius: float
    speed_limit: float
    turn_limit: float
    command: Sample = None  # value: (vx, wz)
    pose: Sample = None     # value: (x, y, theta, measured_speed)


@dataclass
class Decision:
    commands: dict          # agent name -> (vx, wz) to publish
    stop_latched: bool
    motion_permitted: bool  # False when a latch or system-wide hold applies
    reason: str             # 'OK' or every active reason
    holds: dict             # agent name -> list of hold reasons
    clamped: list = field(default_factory=list)


class SafetyCore:
    def __init__(self, cfg=None):
        self.cfg = cfg or SafetyConfig()
        self.cfg.validate()
        c = self.cfg
        self.agents = {
            'robot': Agent('robot', c.robot_id, geo.RADIUS_BY_KIND['robot'],
                           c.robot_speed_limit, c.robot_turn_limit),
            'bus': Agent('bus', c.bus_id, geo.RADIUS_BY_KIND['bus'],
                         c.bus_speed_limit, c.bus_turn_limit),
        }
        self.estop = None        # Sample(bool)
        self.vehicles = None     # Sample(list of (id, kind, x, y, valid))
        self.crossing = None     # Sample((owner_id, valid))
        self.door = None         # Sample(bool), measured bus door state
        self.latched = False
        self.latch_reasons = []
        self.events = []         # (event, entity, reason); drained by the node
        self._last_holds = {}
        self._last_system = []

    # ------------------------------------------------------------ inputs
    def observed_at(self, now, src_age):
        """Monotonic time the data was observed, or None to reject it.

        src_age is the header stamp's age (ROS clock) at receipt, or None for
        messages without a stamp. Using the older of receipt time and source
        time means re-sent old data never looks fresh.
        """
        if src_age is None:
            return now
        if not math.isfinite(src_age) or src_age < -self.cfg.future_tolerance:
            return None
        return now - max(0.0, src_age)

    def on_command(self, agent, vx, wz, now, src_age=None):
        if not (math.isfinite(vx) and math.isfinite(wz)):
            return False                      # NaN/inf: ignore -> goes stale
        t = self.observed_at(now, src_age)
        if t is None:
            return False
        self.agents[agent].command = Sample((vx, wz), t)
        return True

    def on_robot_odom(self, x, y, theta, speed, now, src_age=None):
        t = self.observed_at(now, src_age)
        if t is None or not all(map(math.isfinite, (x, y, theta, speed))):
            return False
        self.agents['robot'].pose = Sample((x, y, theta, speed), t)
        return True

    def on_vehicles(self, entries, now, src_age=None):
        """entries: list of (vehicle_id, kind, x, y, theta, speed, valid)."""
        t = self.observed_at(now, src_age)
        if t is None:
            return False
        self.vehicles = Sample([(e[0], e[1], e[2], e[3], e[6]) for e in entries], t)
        bus = self.agents['bus']
        mine = [e for e in entries if e[0] == bus.vehicle_id]
        if mine and mine[0][6]:
            vid, kind, x, y, theta, speed, valid = mine[0]
            bus.pose = Sample((x, y, theta, speed), t)
        else:
            bus.pose = None                   # missing or invalid = unknown
        return True

    def on_crossing(self, owner_id, valid, now, src_age=None):
        t = self.observed_at(now, src_age)
        if t is not None:
            self.crossing = Sample((owner_id, bool(valid)), t)

    def on_door(self, is_open, now):
        self.door = Sample(bool(is_open), now)

    def on_estop(self, asserted, now):
        """Return True if this message newly latched the stop."""
        self.estop = Sample(bool(asserted), now)
        if asserted and 'ESTOP' not in self.latch_reasons:
            self._latch('ESTOP')
            return True
        return False                          # False never clears the latch

    # ------------------------------------------------------------ helpers
    def _latch(self, reason):
        if reason not in self.latch_reasons:
            self.latch_reasons.append(reason)
            self.events.append(('STOP_LATCHED', 'safety_supervisor', reason))
        self.latched = True

    def _others(self, agent):
        """Other bodies from /hub/vehicles: (id, x, y, radius, valid)."""
        if self.vehicles is None:
            return []
        out = []
        for vid, kind, x, y, valid in self.vehicles.value:
            if vid != agent.vehicle_id:
                out.append((vid, x, y, geo.RADIUS_BY_KIND.get(kind, 0.35), valid))
        return out

    def _hazards(self, agent, now):
        """Latching hazards at the agent's current (fresh) pose."""
        if age(agent.pose, now) > self.cfg.sensor_timeout:
            return []
        x, y = agent.pose.value[0], agent.pose.value[1]
        found = []
        if not geo.circle_inside_rect(x, y, agent.radius, geo.SITE):
            found.append('BOUNDARY:' + agent.name)
        for oid, ox, oy, orad, valid in self._others(agent):
            if valid and geo.gap_between(x, y, agent.radius, ox, oy, orad) < 0:
                found.append('COLLISION:%s-%s' % (agent.name, oid))
        return found

    def _clamp(self, agent, vx, wz):
        cvx = max(-agent.speed_limit, min(agent.speed_limit, vx))
        cwz = max(-agent.turn_limit, min(agent.turn_limit, wz))
        return cvx, cwz, (cvx != vx or cwz != wz)

    def _agent_holds(self, agent, now):
        """Return (hold reasons, safe command) for one agent."""
        c = self.cfg
        reasons = []
        if age(agent.pose, now) > c.sensor_timeout:
            reasons.append('ODOM_STALE' if agent.name == 'robot' else 'POSE_STALE')
        if age(agent.command, now) > c.command_timeout:
            reasons.append('COMMAND_STALE')
        if agent.name == 'bus':
            if age(self.door, now) > c.sensor_timeout:
                reasons.append('DOOR_UNKNOWN')
            elif self.door.value:
                reasons.append('DOOR_OPEN')
        if reasons:
            return reasons, (0.0, 0.0), False

        vx, wz, clamped = self._clamp(agent, *agent.command.value)
        x, y, theta, _ = agent.pose.value
        # Motion footprint: where will this command put the agent soon?
        px, py, _ = geo.predict_pose(x, y, theta, vx, wz, c.lookahead)
        if not geo.circle_inside_rect(px, py, agent.radius, geo.SITE):
            reasons.append('BOUNDARY_AHEAD')
        for oid, ox, oy, orad, valid in self._others(agent):
            if not valid:
                reasons.append('UNKNOWN_BODY:' + oid)
                continue
            gap_now = geo.gap_between(x, y, agent.radius, ox, oy, orad)
            gap_next = geo.gap_between(px, py, agent.radius, ox, oy, orad)
            # Only block motion that closes in; moving away is allowed.
            if gap_next < c.clearance and gap_next < gap_now:
                reasons.append('CLEARANCE:' + oid)
        if c.enforce_crossing:
            r = self._crossing_reason(agent, x, y, px, py, now)
            if r:
                reasons.append(r)
        if reasons:
            return reasons, (0.0, 0.0), False
        return [], (vx, wz), clamped

    def _crossing_reason(self, agent, x, y, px, py, now):
        inside = geo.circle_overlaps_rect(x, y, agent.radius, geo.CROSSING)
        entering = geo.circle_overlaps_rect(px, py, agent.radius, geo.CROSSING)
        if not (inside or entering):
            return None
        cs = self.crossing
        if age(cs, now) > self.cfg.sensor_timeout or not cs.value[1]:
            return 'CROSSING_UNKNOWN'
        if cs.value[0] != agent.vehicle_id:
            return 'CROSSING_NOT_GRANTED'
        return None

    # ------------------------------------------------------------ 20 Hz tick
    def tick(self, now):
        c = self.cfg
        if self.estop is not None and self.estop.value:
            self._latch('ESTOP')
        for agent in self.agents.values():
            for hazard in self._hazards(agent, now):
                self._latch(hazard)

        system = []                     # holds that stop every agent
        if c.require_estop_heartbeat and age(self.estop, now) > c.estop_timeout:
            system.append('ESTOP_SIGNAL_STALE')
        if age(self.vehicles, now) > c.sensor_timeout:
            system.append('VEHICLES_STALE')

        commands, holds, clamped = {}, {}, []
        for name, agent in self.agents.items():
            reasons, cmd, was_clamped = self._agent_holds(agent, now)
            holds[name] = reasons
            if self.latched or system or reasons:
                commands[name] = (0.0, 0.0)
            else:
                commands[name] = cmd
                if was_clamped:
                    clamped.append(name)

        self._record_transitions(system, holds)
        parts = []
        if self.latched:
            parts.append('LATCHED:' + ','.join(self.latch_reasons))
        parts += system
        parts += ['%s:%s' % (n, '/'.join(r)) for n, r in holds.items() if r]
        return Decision(commands=commands, stop_latched=self.latched,
                        motion_permitted=not (self.latched or system),
                        reason='; '.join(parts) if parts else 'OK',
                        holds=holds, clamped=clamped)

    def _record_transitions(self, system, holds):
        """Emit events only when a hold starts or ends (assignment 6.1)."""
        for r in set(system) - set(self._last_system):
            self.events.append(('SAFETY_HOLD', 'system', r))
        for r in set(self._last_system) - set(system):
            self.events.append(('SAFETY_RELEASE', 'system', r))
        for name, reasons in holds.items():
            before = set(self._last_holds.get(name, []))
            for r in set(reasons) - before:
                self.events.append(('SAFETY_HOLD', name, r))
            for r in before - set(reasons):
                self.events.append(('SAFETY_RELEASE', name, r))
        self._last_system = list(system)
        self._last_holds = {n: list(r) for n, r in holds.items()}

    # ------------------------------------------------------------ reset
    def request_clear(self, now):
        """Validate /safety/clear_stop. Returns (success, reason).

        Rejected while the stop input is asserted or unknown, a hazard
        remains, a critical sensor is stale, an agent is still moving, or a
        manager is still commanding motion (assignment 7.1, 16.3, 18.7).
        Success only re-enables readiness; no motion starts by itself.
        """
        c = self.cfg
        if not self.latched:
            return True, 'NO_LATCHED_STOP'
        problems = []
        if self.estop is not None and self.estop.value:
            problems.append('ESTOP_ASSERTED')
        elif c.require_estop_heartbeat and age(self.estop, now) > c.estop_timeout:
            problems.append('ESTOP_SIGNAL_STALE')
        if age(self.vehicles, now) > c.sensor_timeout:
            problems.append('VEHICLES_STALE')
        for agent in self.agents.values():
            if age(agent.pose, now) > c.sensor_timeout:
                kind = 'ODOM_STALE' if agent.name == 'robot' else 'POSE_STALE'
                problems.append('%s:%s' % (agent.name, kind))
            else:
                if abs(agent.pose.value[3]) > c.stopped_speed:
                    problems.append('%s:MOVING' % agent.name)
                problems += self._hazards(agent, now)
            fresh_cmd = age(agent.command, now) <= c.command_timeout
            if fresh_cmd and any(abs(v) > 1e-6 for v in agent.command.value):
                problems.append('%s:COMMAND_NONZERO' % agent.name)
        if problems:
            reason = ';'.join(problems)
            self.events.append(('CLEAR_REJECTED', 'safety_supervisor', reason))
            return False, reason
        cleared = ','.join(self.latch_reasons)
        self.latched = False
        self.latch_reasons = []
        self.events.append(('STOP_CLEARED', 'safety_supervisor', cleared))
        return True, 'CLEARED:' + cleared
