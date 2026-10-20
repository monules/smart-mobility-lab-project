"""Actuator bridge logic (pure Python, no ROS).

The bridge sits between safety_supervisor and the motors (here: the
simulator). It does not trust its input (assignment 16.2, 18.7 ACTUATOR_TICK):

* a final command older than command_timeout -> zero, so a crashed or frozen
  supervisor cannot leave a nonzero motor command running (test T17);
* its own speed caps, so a supervisor bug cannot overspeed the motors;
* the bus door opens only after the bus has been commanded to zero for
  door_settle_sec, and while the door is open the bus command is zero.

Times are seconds from time.monotonic(), supplied by the caller.
"""
from dataclasses import dataclass
import math

AGENTS = ('robot', 'bus')


@dataclass
class BridgeConfig:
    command_timeout: float = 0.5     # s; same value as the supervisor's
    robot_speed_limit: float = 0.25  # m/s
    bus_speed_limit: float = 0.40    # m/s
    robot_turn_limit: float = 1.0    # rad/s
    bus_turn_limit: float = 0.5      # rad/s
    door_settle_sec: float = 0.5     # bus held at zero this long before opening

    def validate(self):
        for name, value in vars(self).items():
            if not (math.isfinite(value) and value > 0):
                raise ValueError('invalid bridge parameter: ' + name)


class BridgeCore:
    def __init__(self, cfg=None):
        self.cfg = cfg or BridgeConfig()
        self.cfg.validate()
        self.cmd = {a: None for a in AGENTS}       # (vx, wz, receipt time)
        self.door_requested = False
        self.door_open = False
        self.bus_last_moving = -math.inf           # last time bus output != 0
        self.watchdog = {a: True for a in AGENTS}  # True = stopped by timeout

    def limits(self, agent):
        c = self.cfg
        if agent == 'robot':
            return c.robot_speed_limit, c.robot_turn_limit
        return c.bus_speed_limit, c.bus_turn_limit

    def on_command(self, agent, vx, wz, now):
        if not (math.isfinite(vx) and math.isfinite(wz)):
            return False                           # rejected -> will go stale
        self.cmd[agent] = (vx, wz, now)
        return True

    def on_door_request(self, want_open):
        self.door_requested = bool(want_open)

    def tick(self, now):
        """Return ({agent: (vx, wz)}, door_open, events)."""
        events = []
        out = {}
        for agent in AGENTS:
            stale = self.cmd[agent] is None or now - self.cmd[agent][2] > self.cfg.command_timeout
            if stale != self.watchdog[agent]:
                events.append(('BRIDGE_WATCHDOG_STOP' if stale else 'BRIDGE_WATCHDOG_RELEASE',
                               agent, 'FINAL_COMMAND_STALE' if stale else 'FRESH'))
                self.watchdog[agent] = stale
            if stale:
                out[agent] = (0.0, 0.0)
                continue
            vmax, wmax = self.limits(agent)
            vx, wz, _ = self.cmd[agent]
            out[agent] = (max(-vmax, min(vmax, vx)), max(-wmax, min(wmax, wz)))

        # Door interlock: open only when the bus has been still long enough.
        if not self.door_requested:
            if self.door_open:
                events.append(('DOOR_CLOSED', 'bus', 'REQUESTED'))
            self.door_open = False
        elif not self.door_open and now - self.bus_last_moving >= self.cfg.door_settle_sec \
                and out['bus'] == (0.0, 0.0):
            self.door_open = True
            events.append(('DOOR_OPENED', 'bus', 'BUS_STOPPED'))
        if self.door_open:
            out['bus'] = (0.0, 0.0)                # never move with the door open
        if out['bus'] != (0.0, 0.0):
            self.bus_last_moving = now
        return out, self.door_open, events
