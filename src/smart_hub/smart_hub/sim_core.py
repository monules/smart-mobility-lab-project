"""Deterministic 2D hub simulator for scenario_runner (pure Python, no ROS).

Assignment 3.2: fixed 20 Hz step, unicycle kinematics, scenario events
injected at configured times from a YAML file, fixed random seed, full reset
between trials. Time here is *simulation* time (step count x DT), so a run
with the same scenario and seed is repeatable.

The simulator is also the last motor layer: a body whose applied command is
older than input_timeout coasts to zero, like a motor driver timeout.
"""
from dataclasses import dataclass
import math
import random

from smart_hub import geometry as geo

DT = 0.05                      # s, 20 Hz
MOVABLE = ('robot', 'bus', 'car')
EVENT_TYPES = ('estop', 'freeze', 'obstacle', 'parking', 'note', 'pedestrian', 'occlude',
               'vehicle', 'escort', 'ready')
# 'escort' and 'ready' do not change the world: scenario_runner turns them into
# an ESCORT_REQUEST event and a /robot/confirm_ready press (S8 stimuli).
EXIT = (11.0, 2.0)            # exit E: a bus or car stopped here leaves the map (2.1)
# 'camera' silences /camera/image_raw; 'camera_repeat' keeps re-sending the
# last frame with its old stamp (a frozen source, assignment 16.1).
FREEZABLE = ('odom', 'vehicles', 'parking', 'camera', 'camera_repeat')
# Boarding (S3, assignment 10.2 step 5): while the measured door is open and
# the bus stands still at bay B, waiting people in queue Q board one by one.
BAY = (8.0, 4.0)
BAY_TOLERANCE = 0.30          # m, bus centre to B
QUEUE = (2.0, 5.0, 6.0, 7.0)  # zone Q (xmin, ymin, xmax, ymax)
BUS_CAPACITY = 4              # people per door opening
BOARD_FIRST = 1.0             # s after the door opens
BOARD_INTERVAL = 1.5          # s between two people


@dataclass
class Pedestrian:
    """A person on the floor; seen only by the camera (not in /hub/vehicles)."""

    ped_id: str
    x: float
    y: float
    vx: float = 0.0
    vy: float = 0.0
    hidden_until: float = -math.inf   # occluded from the camera until this time


@dataclass
class Body:
    vehicle_id: str
    kind: str
    x: float
    y: float
    theta: float = 0.0
    v: float = 0.0               # actual linear speed (m/s)
    w: float = 0.0               # actual angular speed (rad/s)
    active: bool = True          # obstacles can be inserted/removed
    cmd: tuple = (0.0, 0.0)
    cmd_time: float = -math.inf  # sim time the command was received
    cruise: float = None         # scripted traffic: constant speed, ignores commands
    loop_x: tuple = None         # (xmin, xmax): wrap back to xmin after passing xmax


def validate_event(event):
    """Raise ValueError for a malformed scenario event."""
    kind = event.get('type')
    if kind not in EVENT_TYPES:
        raise ValueError('unknown event type: %r' % (kind,))
    if kind == 'freeze':
        if event.get('topic') not in FREEZABLE or float(event.get('duration', 0)) <= 0:
            raise ValueError('freeze needs topic in %s and duration > 0' % (FREEZABLE,))
    if kind == 'estop' and not isinstance(event.get('value'), bool):
        raise ValueError('estop needs value: true/false')
    if kind == 'obstacle' and 'id' not in event:
        raise ValueError('obstacle needs id')
    if kind == 'parking' and event.get('state') not in ('free', 'occupied', 'offline'):
        raise ValueError('parking needs state free/occupied/offline')
    if kind == 'pedestrian':
        if 'id' not in event or (event.get('active', True) and not
                                 ('x' in event and 'y' in event)):
            raise ValueError('pedestrian needs id, and x/y unless active: false')
    if kind == 'occlude' and ('id' not in event or float(event.get('duration', 0)) <= 0):
        raise ValueError('occlude needs id and duration > 0')
    needed = ('request_id', 'passenger_id', 'pickup', 'destination')
    if kind == 'escort' and not all(event.get(k) for k in needed):
        raise ValueError('escort needs request_id, passenger_id, pickup, destination')
    if 'jitter' in event and not 0 <= float(event['jitter']) <= 30:
        raise ValueError('jitter must be in [0, 30] s')
    if kind == 'vehicle':
        if 'id' not in event or (event.get('active', True) and not
                                 ('x' in event and 'y' in event)):
            raise ValueError('vehicle needs id, and x/y unless active: false')
        if event.get('kind', 'car') not in MOVABLE:
            raise ValueError('vehicle kind must be one of %s' % (MOVABLE,))


class HubSim:
    def __init__(self, scenario, seed, input_timeout=0.5):
        self.scenario = scenario
        self.input_timeout = input_timeout
        for event in scenario.get('events', []):
            validate_event(event)
        self.reset(seed)

    def reset(self, seed):
        """Restore the initial world. Same scenario + seed = same run."""
        sc = self.scenario
        self.seed = int(seed)
        self.rng = random.Random(self.seed)
        self.steps = 0
        self.t = 0.0
        self.noise = float(sc.get('observation_noise_m', 0.0))
        self.bodies = {}
        for vid, spec in sc.get('agents', {}).items():
            self._add_vehicle(dict(spec, id=vid))
        for vid, spec in sc.get('obstacles', {}).items():
            self.bodies[vid] = Body(vid, 'obstacle', float(spec['x']), float(spec['y']),
                                    active=bool(spec.get('active', True)))
        # 'jitter: j' moves an event by a seeded uniform offset in [-j, j], so
        # every seed is a different but reproducible episode (S8). Sort is
        # stable, so events with equal times keep file order.
        events = []
        for e in sc.get('events', []):
            j = float(e.get('jitter', 0.0))
            t = max(0.0, float(e['t']) + (self.rng.uniform(-j, j) if j > 0 else 0.0))
            events.append(dict(e, t=round(t, 2)))
        self.pending = sorted(events, key=lambda e: float(e['t']))
        self.frozen_until = {}
        self.estop = bool(sc.get('estop', False))
        self.parking = dict(sc.get('parking', {}))
        self.pedestrians = {}
        for pid, spec in sc.get('pedestrians', {}).items():
            self._place_pedestrian(dict(spec, id=pid))
        self.door_open = False
        self.door_since = None
        self.boarded_this_stop = 0
        self.boardings = []          # (person id, sim time) not yet reported
        self.exits = []              # vehicle ids that left the map at E, not yet reported

    # --------------------------------------------------------------- inputs
    def set_command(self, vehicle_id, v, w):
        body = self.bodies.get(vehicle_id)
        if body is None or body.kind not in MOVABLE:
            return False
        if not (math.isfinite(v) and math.isfinite(w)):
            return False
        body.cmd = (float(v), float(w))
        body.cmd_time = self.t
        return True

    def set_door(self, is_open):
        """Measured bus door state (from the actuator bridge)."""
        if is_open and not self.door_open:
            self.door_since, self.boarded_this_stop = self.t, 0
        self.door_open = bool(is_open)

    def pop_boardings(self):
        done, self.boardings = self.boardings, []
        return done

    def _board(self):
        bus = next((b for b in self.bodies.values() if b.kind == 'bus'), None)
        if not (self.door_open and bus is not None and abs(bus.v) < 0.02
                and math.hypot(bus.x - BAY[0], bus.y - BAY[1]) <= BAY_TOLERANCE
                and self.boarded_this_stop < BUS_CAPACITY):
            return
        due = self.door_since + BOARD_FIRST + self.boarded_this_stop * BOARD_INTERVAL
        if self.t + 1e-9 < due:
            return
        x0, y0, x1, y1 = QUEUE
        waiting = sorted(pid for pid, p in self.pedestrians.items()
                         if x0 <= p.x <= x1 and y0 <= p.y <= y1 and p.vx == p.vy == 0)
        if waiting:
            del self.pedestrians[waiting[0]]
            self.boarded_this_stop += 1
            self.boardings.append((waiting[0], self.t))

    def apply_event(self, event):
        """Apply one event now (scenario file or test-only injection)."""
        validate_event(event)
        kind = event['type']
        if kind == 'estop':
            self.estop = event['value']
        elif kind == 'freeze':
            self.frozen_until[event['topic']] = self.t + float(event['duration'])
        elif kind == 'obstacle':
            body = self.bodies.get(event['id'])
            if body is None:
                spec = event
                body = Body(event['id'], 'obstacle', float(spec['x']), float(spec['y']))
                self.bodies[event['id']] = body
            body.active = bool(event.get('active', True))
        elif kind == 'parking':
            self.parking[event['slot']] = event['state']
        elif kind == 'pedestrian':
            if event.get('active', True):
                self._place_pedestrian(event)
            else:
                self.pedestrians.pop(event['id'], None)    # walked away
        elif kind == 'vehicle':
            if event.get('active', True):
                self._add_vehicle(event)
            else:
                self.bodies.pop(event['id'], None)        # left the map
        elif kind == 'occlude':
            ped = self.pedestrians.get(event['id'])
            if ped is not None:
                ped.hidden_until = self.t + float(event['duration'])
        return event

    def _add_vehicle(self, spec):
        loop = spec.get('loop_x')
        cruise = spec.get('cruise')
        self.bodies[spec['id']] = Body(
            spec['id'], spec.get('kind', 'car'), float(spec['x']), float(spec['y']),
            float(spec.get('theta', 0.0)),
            cruise=None if cruise is None else float(cruise),
            loop_x=None if loop is None else (float(loop[0]), float(loop[1])))

    def _place_pedestrian(self, spec):
        self.pedestrians[spec['id']] = Pedestrian(
            spec['id'], float(spec['x']), float(spec['y']),
            float(spec.get('vx', 0.0)), float(spec.get('vy', 0.0)))

    # ----------------------------------------------------------------- step
    def step(self):
        """Advance one fixed step. Returns the scenario events applied."""
        applied = []
        while self.pending and float(self.pending[0]['t']) <= self.t + 1e-9:
            applied.append(self.apply_event(self.pending.pop(0)))
        for body in self.bodies.values():
            if body.kind not in MOVABLE:
                continue
            if body.cruise is not None:
                body.v, body.w = body.cruise, 0.0   # scripted traffic (S5)
            elif self.t - body.cmd_time > self.input_timeout:
                body.v, body.w = 0.0, 0.0      # drive timeout: no fresh command
            else:
                body.v, body.w = body.cmd
            body.x, body.y, body.theta = geo.predict_pose(
                body.x, body.y, body.theta, body.v, body.w, DT)
            body.theta = math.atan2(math.sin(body.theta), math.cos(body.theta))
            if body.loop_x and body.x > body.loop_x[1]:
                body.x = body.loop_x[0]        # keep scripted traffic on its edge
        for ped in self.pedestrians.values():
            ped.x += ped.vx * DT
            ped.y += ped.vy * DT
        self._board()
        for vid in [v for v, b in self.bodies.items() if b.kind in ('bus', 'car')
                    and b.cruise is None and abs(b.v) < 0.02
                    and math.hypot(b.x - EXIT[0], b.y - EXIT[1]) <= BAY_TOLERANCE]:
            del self.bodies[vid]                  # completed its trip at exit E
            self.exits.append(vid)
        self.steps += 1
        self.t = self.steps * DT               # no float drift over long runs
        return applied

    # -------------------------------------------------------------- outputs
    def is_frozen(self, topic):
        return self.t < self.frozen_until.get(topic, -1.0)

    def observe(self, body):
        """Noisy observed (x, y). Noise comes only from the seeded RNG."""
        if self.noise <= 0:
            return body.x, body.y
        return (body.x + self.rng.gauss(0, self.noise),
                body.y + self.rng.gauss(0, self.noise))

    def visible_bodies(self):
        return [b for b in self.bodies.values() if b.kind in MOVABLE or b.active]

    def camera_people(self):
        """(x, y) of pedestrians the camera can see right now (not occluded)."""
        return [(p.x, p.y) for _, p in sorted(self.pedestrians.items())
                if self.t >= p.hidden_until]

    def finished(self):
        duration = float(self.scenario.get('duration_sec', 0))
        return duration > 0 and self.t >= duration
