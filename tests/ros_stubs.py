"""Minimal stand-ins for rclpy and the smart_hub_interfaces messages so the
node wrappers can be smoke-tested where ROS 2 is not installed.

NOT a substitute for running the nodes on ROS 2 Jazzy: it checks that the node
code runs, wires the right topics/services, fills the right fields and rejects
bad parameters.  It cannot check QoS compatibility, executors or real timing.
Field lists mirror assignment section 6 and the service definitions in 7.1.
"""
import sys
import time
import types

from helpers import *  # noqa: F401,F403  (sets sys.path)


def msg_class(name, **fields):
    def __init__(self, **kw):
        for k, d in fields.items():
            setattr(self, k, d() if callable(d) else d)
        for k, v in kw.items():
            setattr(self, k, v)
    return type(name, (), {'__init__': __init__, '_fields': tuple(fields)})


def _module(name, **attrs):
    m = types.ModuleType(name)
    m.__dict__.update(attrs)
    sys.modules[name] = m
    return m


class FakeTime:
    def __init__(self, ns):
        self.nanoseconds = ns

    def to_msg(self):
        return Time(sec=self.nanoseconds // 10**9, nanosec=self.nanoseconds % 10**9)


class FakeClock:
    def now(self):
        return FakeTime(time.time_ns())


class FakePub:
    def __init__(self, topic, qos):
        self.topic, self.qos, self.sent = topic, qos, []

    def publish(self, msg):
        self.sent.append(msg)

    @property
    def last(self):
        return self.sent[-1]


class FakeLogger:
    def __init__(self):
        self.lines = []

    def _log(self, level, msg, **kw):
        self.lines.append((level, msg))

    def info(self, m, **kw): self._log('info', m)
    def warning(self, m, **kw): self._log('warning', m)
    def warn(self, m, **kw): self._log('warning', m)
    def error(self, m, **kw): self._log('error', m)
    def debug(self, m, **kw): self._log('debug', m)


class FakeNode:
    def __init__(self, name, parameter_overrides=None, **kw):
        self._name, self._over, self._params = name, parameter_overrides or {}, {}
        self.pubs, self.subs, self.srvs, self.timers = {}, {}, {}, []
        self._logger, self._clock = FakeLogger(), FakeClock()

    def declare_parameter(self, name, default=None):
        self._params[name] = self._over.get(name, default)

    def get_parameter(self, name):
        return types.SimpleNamespace(value=self._params[name])

    def create_publisher(self, typ, topic, qos, **kw):
        self.pubs[topic] = FakePub(topic, qos)
        return self.pubs[topic]

    def create_subscription(self, typ, topic, cb, qos, **kw):
        self.subs[topic] = (typ, cb, qos)

    def create_service(self, typ, name, cb, **kw):
        self.srvs[name] = (typ, cb)

    def create_timer(self, period, cb, **kw):
        self.timers.append((period, cb))

    def get_clock(self):
        return self._clock

    def get_logger(self):
        return self._logger

    def destroy_node(self):
        pass


# ---- message definitions (mirror assignment section 6 / 7.1) ----
Time = msg_class('Time', sec=0, nanosec=0)
Header = msg_class('Header', stamp=lambda: Time(), frame_id='')
Bool = msg_class('Bool', data=False)
String = msg_class('String', data='')
Pose2D = msg_class('Pose2D', x=0.0, y=0.0, theta=0.0)
HubEvent = msg_class('HubEvent', header=lambda: Header(), run_id='', scenario_id='',
                     entity_id='', event='', reason='', correlation_id='')
ParkingSlot = msg_class('ParkingSlot', slot_id='', state='', vehicle_id='',
                        reservation_id='', expires_at=lambda: Time())
ParkingState = msg_class('ParkingState', header=lambda: Header(), slots=list,
                         available_count=0)
SlotObservation = msg_class('SlotObservation', slot_id='', occupied=False,
                            confidence=1.0, valid=True)
SlotObservationArray = msg_class('SlotObservationArray', header=lambda: Header(),
                                 slots=list)
VehicleState = msg_class('VehicleState', vehicle_id='', kind='car',
                         pose=lambda: Pose2D(), speed_mps=0.0, edge_id='', valid=True)
VehicleStateArray = msg_class('VehicleStateArray', header=lambda: Header(), vehicles=list)
EdgeState = msg_class('EdgeState', edge_id='', mean_speed_mps=0.0, density_ratio=0.0,
                      blocked=False, valid=False)
EdgeStateArray = msg_class('EdgeStateArray', header=lambda: Header(), edges=list)


class ReserveParking:
    Request = msg_class('Request', request_id='', vehicle_id='', preferred_slot='',
                        ttl_sec=0.0)
    Response = msg_class('Response', success=False, reservation_id='', slot_id='',
                         expires_at=lambda: Time(), reason='')


class ReleaseParking:
    Request = msg_class('Request', request_id='', reservation_id='', vehicle_id='')
    Response = msg_class('Response', success=False, reason='')


def install():
    """Install the fakes in sys.modules (idempotent)."""
    if 'smart_hub_interfaces.msg' in sys.modules and \
            getattr(sys.modules['smart_hub_interfaces.msg'], '_is_stub', False):
        return
    enum = lambda **kw: types.SimpleNamespace(**kw)           # noqa: E731
    _module('rclpy', init=lambda *a, **k: None, shutdown=lambda: None,
            spin=lambda n: None)
    _module('rclpy.node', Node=FakeNode)
    _module('rclpy.qos', QoSProfile=lambda **kw: types.SimpleNamespace(**kw),
            ReliabilityPolicy=enum(RELIABLE='RELIABLE'),
            DurabilityPolicy=enum(VOLATILE='VOLATILE', TRANSIENT_LOCAL='TRANSIENT_LOCAL'),
            HistoryPolicy=enum(KEEP_LAST='KEEP_LAST'))
    _module('rclpy.callback_groups', MutuallyExclusiveCallbackGroup=object,
            ReentrantCallbackGroup=object)
    _module('rclpy.executors', MultiThreadedExecutor=object)
    _module('builtin_interfaces')
    _module('builtin_interfaces.msg', Time=Time)
    _module('std_msgs')
    _module('std_msgs.msg', Bool=Bool, String=String, Header=Header)
    _module('geometry_msgs')
    _module('geometry_msgs.msg', Pose2D=Pose2D)
    _module('smart_hub_interfaces')
    _module('smart_hub_interfaces.msg', _is_stub=True, HubEvent=HubEvent,
            ParkingSlot=ParkingSlot, ParkingState=ParkingState,
            SlotObservation=SlotObservation, SlotObservationArray=SlotObservationArray,
            VehicleState=VehicleState, VehicleStateArray=VehicleStateArray,
            EdgeState=EdgeState, EdgeStateArray=EdgeStateArray)
    _module('smart_hub_interfaces.srv', ReserveParking=ReserveParking,
            ReleaseParking=ReleaseParking)
