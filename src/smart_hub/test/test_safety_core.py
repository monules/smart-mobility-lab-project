"""Unit tests for SafetyCore (pure Python; time is passed in, no sleeps)."""
import math

import pytest

from smart_hub.safety_core import SafetyConfig, SafetyCore

T0 = 100.0
ROBOT_AT_R = (3.0, 6.0, 0.0)      # facing east, far from C and other bodies
BUS_AT_A = (1.0, 2.0, 0.0)


def feed(core, now, robot=ROBOT_AT_R, bus=BUS_AT_A, robot_speed=0.0, bus_speed=0.0,
         estop=False, door=False, extra=()):
    """Give the core one fresh sample of every critical input."""
    core.on_robot_odom(*robot, robot_speed, now, 0.0)
    entries = [('robot_1', 'robot', robot[0], robot[1], robot[2], robot_speed, True),
               ('bus_1', 'bus', bus[0], bus[1], bus[2], bus_speed, True)] + list(extra)
    core.on_vehicles(entries, now, 0.0)
    core.on_estop(estop, now)
    core.on_door(door, now)


def ready_core(cfg=None, now=T0):
    core = SafetyCore(cfg or SafetyConfig())
    feed(core, now)
    return core


# ----------------------------------------------------------------- normal
def test_fresh_command_passes_through():
    core = ready_core()
    core.on_command('robot', 0.2, 0.1, T0, 0.0)
    d = core.tick(T0 + 0.05)
    assert d.commands['robot'] == (0.2, 0.1)
    assert d.motion_permitted and not d.stop_latched


def test_nothing_received_means_zero():
    core = SafetyCore()
    d = core.tick(T0)
    assert d.commands == {'robot': (0.0, 0.0), 'bus': (0.0, 0.0)}
    assert not d.motion_permitted
    assert 'ESTOP_SIGNAL_STALE' in d.reason and 'VEHICLES_STALE' in d.reason


@pytest.mark.parametrize('agent, request_v, expected', [
    ('robot', 0.6, 0.25), ('robot', -0.6, -0.25), ('bus', 1.0, 0.40), ('bus', 0.3, 0.3)])
def test_speed_clamp(agent, request_v, expected):
    core = ready_core()
    core.on_command(agent, request_v, 0.0, T0, 0.0)
    d = core.tick(T0)
    assert d.commands[agent][0] == pytest.approx(expected)
    assert (agent in d.clamped) == (request_v != expected)


def test_turn_rate_clamp():
    core = ready_core()
    core.on_command('robot', 0.0, 5.0, T0, 0.0)
    assert core.tick(T0).commands['robot'][1] == pytest.approx(1.0)


# ---------------------------------------------------------- stale inputs
def test_command_stale_after_half_second():
    core = ready_core()
    core.on_command('robot', 0.2, 0.0, T0, 0.0)
    feed(core, T0 + 0.49)
    assert core.tick(T0 + 0.49).commands['robot'] == (0.2, 0.0)
    feed(core, T0 + 0.51)
    d = core.tick(T0 + 0.51)
    assert d.commands['robot'] == (0.0, 0.0)
    assert 'COMMAND_STALE' in d.holds['robot']
    assert not d.stop_latched                      # a hold, not a latch


def test_old_source_stamp_counts_as_old():
    core = ready_core()
    core.on_command('robot', 0.2, 0.0, T0, src_age=0.45)    # sent 0.45 s ago
    assert core.tick(T0 + 0.04).commands['robot'] == (0.2, 0.0)
    assert core.tick(T0 + 0.06).commands['robot'] == (0.0, 0.0)


def test_future_stamp_rejected():
    core = ready_core()
    assert not core.on_command('robot', 0.2, 0.0, T0, src_age=-0.5)
    assert core.tick(T0).commands['robot'] == (0.0, 0.0)


def test_nan_command_rejected():
    core = ready_core()
    assert not core.on_command('robot', math.nan, 0.0, T0, 0.0)
    assert core.tick(T0).commands['robot'] == (0.0, 0.0)


def test_stale_odometry_stops_robot_only():
    core = ready_core()
    core.on_command('robot', 0.2, 0.0, T0, 0.0)
    core.on_command('bus', 0.3, 0.0, T0, 0.0)
    t = T0 + 0.3
    # everything but robot odometry stays fresh
    core.on_vehicles([('robot_1', 'robot', 3, 6, 0, 0, True),
                      ('bus_1', 'bus', 1, 2, 0, 0, True)], t, 0.0)
    core.on_estop(False, t)
    core.on_door(False, t)
    core.on_command('robot', 0.2, 0.0, t, 0.0)
    core.on_command('bus', 0.3, 0.0, t, 0.0)
    assert core.tick(t).commands['robot'] == (0.2, 0.0)      # odom 0.3 s old
    t2 = T0 + 1.01
    core.on_vehicles([('robot_1', 'robot', 3, 6, 0, 0, True),
                      ('bus_1', 'bus', 1, 2, 0, 0, True)], t2, 0.0)
    core.on_estop(False, t2)
    core.on_door(False, t2)
    core.on_command('robot', 0.2, 0.0, t2, 0.0)
    core.on_command('bus', 0.3, 0.0, t2, 0.0)
    d = core.tick(t2)
    assert d.commands['robot'] == (0.0, 0.0) and 'ODOM_STALE' in d.holds['robot']
    assert d.commands['bus'] == (0.3, 0.0)


def test_stale_vehicle_snapshot_stops_everyone():
    core = ready_core()
    t = T0 + 1.2
    core.on_robot_odom(*ROBOT_AT_R, 0.0, t, 0.0)
    core.on_estop(False, t)
    core.on_command('robot', 0.2, 0.0, t, 0.0)
    d = core.tick(t)
    assert d.commands['robot'] == (0.0, 0.0)
    assert 'VEHICLES_STALE' in d.reason and not d.motion_permitted


def test_estop_heartbeat_loss_stops_everyone():
    core = ready_core()
    t = T0 + 1.2
    core.on_robot_odom(*ROBOT_AT_R, 0.0, t, 0.0)
    core.on_vehicles([('bus_1', 'bus', 1, 2, 0, 0, True)], t, 0.0)
    core.on_command('robot', 0.2, 0.0, t, 0.0)
    d = core.tick(t)
    assert d.commands['robot'] == (0.0, 0.0) and 'ESTOP_SIGNAL_STALE' in d.reason


def test_heartbeat_requirement_can_be_disabled():
    core = SafetyCore(SafetyConfig(require_estop_heartbeat=False))
    core.on_robot_odom(*ROBOT_AT_R, 0.0, T0, 0.0)
    core.on_vehicles([('bus_1', 'bus', 1, 2, 0, 0, True)], T0, 0.0)
    core.on_command('robot', 0.2, 0.0, T0, 0.0)
    assert core.tick(T0).commands['robot'] == (0.2, 0.0)


def test_invalid_bus_entry_means_unknown_pose():
    core = ready_core()
    core.on_vehicles([('robot_1', 'robot', 3, 6, 0, 0, True),
                      ('bus_1', 'bus', 1, 2, 0, 0, False)], T0, 0.0)
    core.on_command('bus', 0.3, 0.0, T0, 0.0)
    assert 'POSE_STALE' in core.tick(T0).holds['bus']


# ------------------------------------------------------------- e-stop latch
def test_estop_latches_and_false_does_not_clear():
    core = ready_core()
    core.on_command('robot', 0.2, 0.0, T0, 0.0)
    assert core.on_estop(True, T0) is True
    assert core.tick(T0).commands['robot'] == (0.0, 0.0)
    feed(core, T0 + 0.1, estop=False)
    core.on_command('robot', 0.2, 0.0, T0 + 0.1, 0.0)
    d = core.tick(T0 + 0.1)
    assert d.stop_latched and d.commands['robot'] == (0.0, 0.0)
    assert 'LATCHED:ESTOP' in d.reason


def test_second_estop_true_does_not_relatch_event():
    core = ready_core()
    assert core.on_estop(True, T0) is True
    assert core.on_estop(True, T0 + 0.1) is False
    assert [e for e in core.events if e[0] == 'STOP_LATCHED'] == [
        ('STOP_LATCHED', 'safety_supervisor', 'ESTOP')]


# ------------------------------------------------------------------ reset
def latched_core():
    core = ready_core()
    core.on_estop(True, T0)
    core.tick(T0)
    return core


def test_clear_rejected_while_estop_asserted():
    core = latched_core()
    ok, reason = core.request_clear(T0 + 0.1)
    assert not ok and 'ESTOP_ASSERTED' in reason and core.latched


def test_clear_rejected_while_estop_signal_stale():
    core = latched_core()
    core.on_estop(False, T0)
    t = T0 + 1.5
    core.on_robot_odom(*ROBOT_AT_R, 0.0, t, 0.0)
    core.on_vehicles([('bus_1', 'bus', 1, 2, 0, 0, True)], t, 0.0)
    ok, reason = core.request_clear(t)
    assert not ok and 'ESTOP_SIGNAL_STALE' in reason


def test_clear_rejected_while_odometry_stale():
    core = latched_core()
    t = T0 + 1.5
    core.on_vehicles([('bus_1', 'bus', 1, 2, 0, 0, True)], t, 0.0)
    core.on_estop(False, t)
    ok, reason = core.request_clear(t)
    assert not ok and 'robot:ODOM_STALE' in reason


def test_clear_rejected_while_moving():
    core = latched_core()
    feed(core, T0 + 0.1, robot_speed=0.1)
    ok, reason = core.request_clear(T0 + 0.1)
    assert not ok and 'robot:MOVING' in reason


def test_clear_rejected_while_command_nonzero():
    core = latched_core()
    feed(core, T0 + 0.1)
    core.on_command('robot', 0.2, 0.0, T0 + 0.1, 0.0)
    ok, reason = core.request_clear(T0 + 0.1)
    assert not ok and 'robot:COMMAND_NONZERO' in reason


def test_clear_rejected_while_hazard_remains():
    core = latched_core()
    feed(core, T0 + 0.1, robot=(11.95, 6.0, 0.0))   # footprint leaves the site
    ok, reason = core.request_clear(T0 + 0.1)
    assert not ok and 'BOUNDARY:robot' in reason


def test_clear_accepted_and_no_automatic_restart():
    core = latched_core()
    feed(core, T0 + 0.1)
    core.on_command('robot', 0.0, 0.0, T0 + 0.1, 0.0)
    ok, reason = core.request_clear(T0 + 0.1)
    assert ok and reason == 'CLEARED:ESTOP' and not core.latched
    d = core.tick(T0 + 0.15)
    assert d.commands['robot'] == (0.0, 0.0)        # still the zero command
    core.on_command('robot', 0.2, 0.0, T0 + 0.2, 0.0)   # a new goal
    assert core.tick(T0 + 0.2).commands['robot'] == (0.2, 0.0)


def test_clear_without_latch_is_harmless():
    core = ready_core()
    assert core.request_clear(T0) == (True, 'NO_LATCHED_STOP')


# --------------------------------------------------------- latched hazards
def test_boundary_breach_latches():
    core = ready_core()
    feed(core, T0, robot=(0.1, 6.0, 0.0))
    d = core.tick(T0)
    assert d.stop_latched and 'BOUNDARY:robot' in d.reason


def test_collision_latches():
    core = ready_core()
    feed(core, T0, extra=[('O', 'obstacle', 3.3, 6.0, 0.0, 0.0, True)])
    d = core.tick(T0)
    assert d.stop_latched and 'COLLISION:robot-O' in d.reason


# ------------------------------------------------------------------- holds
def test_boundary_ahead_holds_without_latching():
    core = ready_core()
    feed(core, T0, robot=(11.7, 6.0, 0.0))          # inside now (x <= 11.8)
    core.on_command('robot', 0.25, 0.0, T0, 0.0)     # 0.5 s ahead: x = 11.825
    d = core.tick(T0)
    assert 'BOUNDARY_AHEAD' in d.holds['robot'] and not d.stop_latched


def test_clearance_blocks_approach_but_allows_retreat():
    core = ready_core()
    obstacle = [('O', 'obstacle', 3.8, 6.0, 0.0, 0.0, True)]   # free gap 0.25 m now
    feed(core, T0, extra=obstacle)
    core.on_command('robot', 0.25, 0.0, T0, 0.0)               # gap in 0.5 s: 0.125 m
    assert 'CLEARANCE:O' in core.tick(T0).holds['robot']
    core.on_command('robot', -0.25, 0.0, T0, 0.0)              # backing away
    assert core.tick(T0).commands['robot'] == (-0.25, 0.0)


def test_unknown_body_holds_agent():
    core = ready_core()
    feed(core, T0, extra=[('car_7', 'car', 9.0, 2.0, 0.0, 0.0, False)])
    core.on_command('robot', 0.2, 0.0, T0, 0.0)
    assert 'UNKNOWN_BODY:car_7' in core.tick(T0).holds['robot']


@pytest.mark.parametrize('crossing, expected', [
    (None, 'CROSSING_UNKNOWN'),
    (('bus_1', True), 'CROSSING_NOT_GRANTED'),
    (('robot_1', False), 'CROSSING_UNKNOWN'),
    (('robot_1', True), None)])
def test_crossing_entry_needs_fresh_grant(crossing, expected):
    core = ready_core()
    robot = (5.0, 4.98, -math.pi / 2)           # 0.03 m north of C, heading south
    feed(core, T0, robot=robot)
    if crossing:
        core.on_crossing(crossing[0], crossing[1], T0, 0.0)
    core.on_command('robot', 0.2, 0.0, T0, 0.0)
    holds = core.tick(T0).holds['robot']
    if expected:
        assert expected in holds
    else:
        assert holds == []


def test_bus_door_open_or_unknown_holds_bus():
    core = ready_core()
    core.on_command('bus', 0.3, 0.0, T0, 0.0)
    core.on_door(True, T0)
    assert 'DOOR_OPEN' in core.tick(T0).holds['bus']
    core.on_door(False, T0)
    assert core.tick(T0).commands['bus'] == (0.3, 0.0)
    assert 'DOOR_UNKNOWN' in core.tick(T0 + 1.5).holds['bus']


def test_hold_events_only_on_transitions():
    core = ready_core()
    core.events.clear()
    for k in range(5):
        core.tick(T0 + 0.01 * k)                 # robot has no command: one hold
    holds = [e for e in core.events if e[0] == 'SAFETY_HOLD' and e[1] == 'robot']
    assert holds == [('SAFETY_HOLD', 'robot', 'COMMAND_STALE')]


@pytest.mark.parametrize('field, value', [
    ('robot_speed_limit', 0.0), ('command_timeout', -1.0), ('sensor_timeout', math.nan),
    ('clearance', 5.0), ('bus_id', 'robot_1')])
def test_invalid_config_rejected(field, value):
    with pytest.raises(ValueError):
        SafetyCore(SafetyConfig(**{field: value}))
