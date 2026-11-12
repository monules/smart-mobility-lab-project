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


def test_stopping_distance_follows_the_assignment_formula():
    cfg = SafetyConfig()                            # tau 0.25 s, a 0.5 m/s^2
    assert cfg.stopping_distance(0.0) == 0.0
    assert cfg.stopping_distance(0.25) == pytest.approx(0.25 * 0.25 + 0.25 ** 2 / 1.0)
    assert cfg.stopping_distance(0.40) == pytest.approx(0.26)     # bus at its cap
    assert cfg.stopping_distance(-0.10) == cfg.stopping_distance(0.10)   # reversing
    # the prediction horizon covers exactly that distance at constant speed
    for v in (0.05, 0.25, 0.40):
        assert v * cfg.stop_horizon(v) == pytest.approx(cfg.stopping_distance(v))


def test_required_gap_grows_with_speed():
    # Obstacle 0.24 m ahead of the robot's footprint. A slow approach can
    # still stop with the 0.20 m margin; a fast one cannot.
    obstacle = [('O', 'obstacle', 3.79, 6.0, 0.0, 0.0, True)]
    core = ready_core()
    feed(core, T0, extra=obstacle)
    core.on_command('robot', 0.10, 0.0, T0, 0.0)    # stops after 0.035 m: gap 0.205
    assert core.tick(T0).commands['robot'] == (0.10, 0.0)
    core.on_command('robot', 0.25, 0.0, T0, 0.0)    # stops after 0.125 m: gap 0.115
    assert 'CLEARANCE:O' in core.tick(T0).holds['robot']


def test_approaching_body_counts_with_its_own_motion():
    # A car 0.60 m ahead: harmless while parked, too close when it drives
    # towards the robot at 0.30 m/s (it covers 0.15 m during the robot's stop).
    def car(speed):
        return [('car_1', 'car', 3.0 + 0.20 + 0.30 + 0.60, 6.0, math.pi, speed, True)]
    core = ready_core()
    feed(core, T0, extra=car(0.0))
    core.on_command('robot', 0.25, 0.0, T0, 0.0)
    assert core.tick(T0).commands['robot'] == (0.25, 0.0)
    feed(core, T0, extra=car(0.30))
    core.on_command('robot', 0.25, 0.0, T0, 0.0)
    assert core.tick(T0).holds['robot'] == []      # 0.60 - 0.125 - 0.15 = 0.325 m left
    feed(core, T0, extra=[('car_1', 'car', 3.0 + 0.20 + 0.30 + 0.45, 6.0, math.pi, 0.30, True)])
    core.on_command('robot', 0.25, 0.0, T0, 0.0)
    assert 'CLEARANCE:car_1' in core.tick(T0).holds['robot']   # 0.45 - 0.275 = 0.175 m


def test_other_agent_is_predicted_from_its_own_final_command():
    # Bus 0.55 m east of the robot's footprint, facing east. /hub/vehicles
    # has no speed sign; the supervisor's last command for the bus has.
    bus = (3.0 + 0.20 + 0.30 + 0.55, 6.0, 0.0)
    core = ready_core()
    feed(core, T0, bus=bus, bus_speed=0.10)
    core.on_command('bus', -0.10, 0.0, T0, 0.0)     # bus reverses towards the robot
    core.on_command('robot', 0.25, 0.0, T0, 0.0)
    first = core.tick(T0)                            # no bus command known yet
    assert first.commands['bus'] == (-0.10, 0.0)
    feed(core, T0 + 0.05, bus=bus, bus_speed=0.10)
    core.on_command('bus', -0.10, 0.0, T0 + 0.05, 0.0)
    core.on_command('robot', 0.25, 0.0, T0 + 0.05, 0.0)
    # robot needs 0.125 m, the reversing bus covers 0.05 m in the same 0.5 s:
    # 0.55 - 0.175 = 0.375 m left, still fine ...
    assert core.tick(T0 + 0.05).holds['robot'] == []
    near = (3.0 + 0.20 + 0.30 + 0.36, 6.0, 0.0)      # ... but not from 0.36 m
    feed(core, T0 + 0.10, bus=near, bus_speed=0.10)
    core.on_command('bus', -0.10, 0.0, T0 + 0.10, 0.0)
    core.on_command('robot', 0.25, 0.0, T0 + 0.10, 0.0)
    assert 'CLEARANCE:bus_1' in core.tick(T0 + 0.10).holds['robot']


def test_reversing_back_into_the_crossing_needs_a_new_grant():
    # The bus has left C eastwards and released it (no owner). Reversing
    # would put its footprint back inside C: the supervisor holds it until
    # the crossing is granted to the bus again. Driving on forwards is free.
    bus = (5.82, 2.0, 0.0)                         # footprint 0.02 m east of C, facing east
    core = ready_core()
    feed(core, T0, bus=bus)
    core.on_crossing('', True, T0, 0.0)            # released: nobody owns C
    core.on_command('bus', -0.10, 0.0, T0, 0.0)
    assert 'CROSSING_NOT_GRANTED' in core.tick(T0).holds['bus']
    core.on_command('bus', 0.10, 0.0, T0, 0.0)
    assert core.tick(T0).commands['bus'] == (0.10, 0.0)
    core.on_crossing('robot_1', True, T0, 0.0)     # somebody else owns it: still held
    core.on_command('bus', -0.10, 0.0, T0, 0.0)
    assert 'CROSSING_NOT_GRANTED' in core.tick(T0).holds['bus']
    core.on_crossing('bus_1', True, T0, 0.0)       # a NEW grant for the bus
    core.on_command('bus', -0.10, 0.0, T0, 0.0)
    assert core.tick(T0).commands['bus'] == (-0.10, 0.0)


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
    ('clearance', 5.0), ('bus_id', 'robot_1'), ('reaction_time', 0.0), ('brake_decel', -0.5)])
def test_invalid_config_rejected(field, value):
    with pytest.raises(ValueError):
        SafetyCore(SafetyConfig(**{field: value}))


# ------------------------------------------------------------ scenario reset
def test_scenario_reset_forgets_samples_but_keeps_latch():
    core = ready_core()
    core.on_command('robot', 0.2, 0.0, T0, 0.0)
    core.on_estop(True, T0)
    reason = core.reset_inputs()
    assert reason == 'inputs cleared; stop_latched=True'
    d = core.tick(T0 + 0.01)
    assert d.stop_latched and d.commands['robot'] == (0.0, 0.0)
    assert 'ODOM_STALE' in d.holds['robot'] and 'VEHICLES_STALE' in d.reason
    ok, why = core.request_clear(T0 + 0.01)       # reset is no way around clear_stop
    assert not ok and 'ESTOP_ASSERTED' in why


def test_scenario_reset_without_latch_still_holds_until_fresh_data():
    core = ready_core()
    core.on_command('robot', 0.2, 0.0, T0, 0.0)
    core.reset_inputs()
    assert core.tick(T0).commands['robot'] == (0.0, 0.0)
    feed(core, T0 + 0.1)
    core.on_command('robot', 0.2, 0.0, T0 + 0.1, 0.0)
    assert core.tick(T0 + 0.1).commands['robot'] == (0.2, 0.0)
