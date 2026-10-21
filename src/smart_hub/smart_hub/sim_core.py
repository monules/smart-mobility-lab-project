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
               'vehicle')
# 'camera' silences /camera/image_raw; 'camera_repeat' keeps re-sending the
# last frame with its old stamp (a frozen source, assignment 16.1).
FREEZABLE = ('odom', 'vehicles', 'parking', 'camera', 'camera_repeat')


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
        # sort is stable, so events with equal times keep file order
        self.pending = sorted(sc.get('events', []), key=lambda e: float(e['t']))
        self.frozen_until = {}
        self.estop = bool(sc.get('estop', False))
        self.parking = dict(sc.get('parking', {}))
        self.pedestrians = {}
        for pid, spec in sc.get('pedestrians', {}).items():
            self._place_pedestrian(dict(spec, id=pid))

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
