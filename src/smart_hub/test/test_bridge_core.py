"""Unit tests for BridgeCore: the independent motor-side safety layer."""
import math

import pytest

from smart_hub.bridge_core import BridgeConfig, BridgeCore

T0 = 50.0


def test_starts_stopped_before_any_command():
    out, door, events = BridgeCore().tick(T0)
    assert out == {'robot': (0.0, 0.0), 'bus': (0.0, 0.0)} and door is False


def test_fresh_command_forwarded():
    b = BridgeCore()
    b.on_command('robot', 0.2, 0.3, T0)
    out, _, events = b.tick(T0 + 0.1)
    assert out['robot'] == (0.2, 0.3)
    assert ('BRIDGE_WATCHDOG_RELEASE', 'robot', 'FRESH') in events


def test_supervisor_silence_forces_zero():
    """T17 core: no final command for > 0.5 s -> zero, without any help."""
    b = BridgeCore()
    b.on_command('robot', 0.2, 0.0, T0)
    assert b.tick(T0 + 0.49)[0]['robot'] == (0.2, 0.0)
    out, _, events = b.tick(T0 + 0.51)
    assert out['robot'] == (0.0, 0.0)
    assert ('BRIDGE_WATCHDOG_STOP', 'robot', 'FINAL_COMMAND_STALE') in events


def test_bridge_clamps_independently():
    b = BridgeCore()
    b.on_command('robot', 3.0, -9.0, T0)
    b.on_command('bus', -2.0, 0.0, T0)
    out = b.tick(T0)[0]
    assert out['robot'] == (0.25, -1.0) and out['bus'] == (-0.40, 0.0)


def test_nan_rejected():
    b = BridgeCore()
    assert not b.on_command('robot', math.inf, 0.0, T0)
    assert b.tick(T0)[0]['robot'] == (0.0, 0.0)


def test_door_waits_for_bus_to_settle_and_blocks_motion():
    b = BridgeCore()
    b.on_command('bus', 0.3, 0.0, T0)
    b.tick(T0)                                   # bus moving
    b.on_door_request(True)
    b.on_command('bus', 0.0, 0.0, T0 + 0.1)
    assert b.tick(T0 + 0.1)[1] is False          # moved 0.1 s ago: stays shut
    b.on_command('bus', 0.0, 0.0, T0 + 0.6)
    out, door, events = b.tick(T0 + 0.6)
    assert door is True and ('DOOR_OPENED', 'bus', 'BUS_STOPPED') in events
    b.on_command('bus', 0.4, 0.0, T0 + 0.7)       # someone asks to drive
    out, door, _ = b.tick(T0 + 0.7)
    assert door is True and out['bus'] == (0.0, 0.0)
    b.on_door_request(False)
    out, door, events = b.tick(T0 + 0.75)
    assert door is False and out['bus'] == (0.4, 0.0)
    assert ('DOOR_CLOSED', 'bus', 'REQUESTED') in events


def test_invalid_config_rejected():
    with pytest.raises(ValueError):
        BridgeCore(BridgeConfig(command_timeout=0.0))


def test_scenario_reset_zeroes_output_and_closes_door():
    b = BridgeCore()
    b.on_command('robot', 0.2, 0.0, T0)
    b.on_command('bus', 0.0, 0.0, T0)
    b.on_door_request(True)
    b.tick(T0 + 0.6)
    assert b.tick(T0 + 0.6)[1] is True
    b.reset()
    out, door, _ = b.tick(T0 + 0.65)
    assert out == {'robot': (0.0, 0.0), 'bus': (0.0, 0.0)} and door is False
