"""hub_coordinator (M2-B): crossing arbiter service + AI bus dispatch.

NOT run against ROS in the authoring environment (no ROS 2 installed there).
Build it with smart_hub_interfaces and check every field name against the
real .msg/.srv/.action files before trusting it.

Everything runs in the default mutually-exclusive callback group, so service,
timer, subscription and action-client callbacks never run at the same time.
Callbacks are short; nothing blocks.
"""
import time
from functools import partial

import rclpy
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time

from smart_hub_interfaces.action import DockBus, EscortPassenger
from smart_hub_interfaces.msg import (BusArrival, CrossingState, DemandPrediction,
                                      HubEvent, QueueState, SafetyState,
                                      VehicleStateArray)
from smart_hub_interfaces.srv import RequestCrossing

from smart_hub.coordinator_core import (CrossingArbiter, DispatchPolicy,
                                        footprint_overlaps_crossing)


def _qos(depth, transient=False):
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        history=HistoryPolicy.KEEP_LAST, depth=depth,
        durability=(DurabilityPolicy.TRANSIENT_LOCAL if transient
                    else DurabilityPolicy.VOLATILE))


R10, R100, R1T = _qos(10), _qos(100), _qos(1, True)

# footprint radii (m) from the assignment; unknown kinds use the larger one
RADIUS = {'bus': 0.30, 'car': 0.30, 'robot': 0.20}


class HubCoordinator(Node):
    def __init__(self):
        super().__init__('hub_coordinator')
        p = self.declare_parameter
        p('run_id', 'run-0'); p('scenario_id', 'S8')
        p('bus_id', 'bus_1'); p('bay_id', 'B'); p('dock_max_duration_sec', 120.0)
        p('lease_sec', 3.0); p('crossing_clear_sec', 2.0)
        p('freshness_sec', 1.0); p('future_tolerance_sec', 0.2)
        p('dispatch_high', 6.0); p('dispatch_low', 4.0)
        p('dispatch_need_high', 3); p('dispatch_need_low', 5)
        p('dispatch_cooldown_sec', 30.0)
        p('require_safety_state', True)
        g = lambda n: self.get_parameter(n).value
        self._validate(g)

        self.run_id, self.scenario_id = g('run_id'), g('scenario_id')
        self.bus_id, self.bay_id = g('bus_id'), g('bay_id')
        self.dock_max = float(g('dock_max_duration_sec'))
        self.fresh = float(g('freshness_sec'))
        self.future_tol = float(g('future_tolerance_sec'))
        self.require_safety = bool(g('require_safety_state'))

        self.arbiter = CrossingArbiter(g('lease_sec'), g('crossing_clear_sec'))
        self.policy = DispatchPolicy(g('dispatch_high'), g('dispatch_low'),
                                     g('dispatch_need_high'), g('dispatch_need_low'),
                                     g('dispatch_cooldown_sec'))

        # latest message + local receipt time (monotonic) per input
        self.latest = {k: None for k in ('demand', 'queue', 'vehicles', 'bus', 'safety')}
        self.sub = [
            self.create_subscription(DemandPrediction, '/hub/demand', partial(self._rx, 'demand'), R10),
            self.create_subscription(QueueState, '/hub/queue', partial(self._rx, 'queue'), R10),
            self.create_subscription(VehicleStateArray, '/hub/vehicles', partial(self._rx, 'vehicles'), R10),
            self.create_subscription(BusArrival, '/hub/bus_arrival', partial(self._rx, 'bus'), R10),
            self.create_subscription(SafetyState, '/hub/safety', partial(self._rx, 'safety'), R1T),
        ]
        self.crossing_pub = self.create_publisher(CrossingState, '/crossing/state', R1T)
        self.event_pub = self.create_publisher(HubEvent, '/hub/events', R100)
        self.create_service(RequestCrossing, '/crossing/request', self._on_crossing)

        self.dock_client = ActionClient(self, DockBus, '/bus/dock')
        self.escort_client = ActionClient(self, EscortPassenger, '/robot/escort')
        self.escort_outstanding = ''
        self._last_deny = {}

        self.create_timer(0.1, self._crossing_tick)    # 10 Hz
        self.create_timer(1.0, self._dispatch_tick)    # 1 Hz
        self.get_logger().info('hub_coordinator ready (propulsion is not controlled here)')

    # ------------------------------------------------------------------ setup
    @staticmethod
    def _validate(g):
        bad = []
        if not 0.5 <= g('lease_sec') <= 30: bad.append('lease_sec')
        if not 0.5 <= g('crossing_clear_sec') <= 10: bad.append('crossing_clear_sec')
        if not 0.1 <= g('freshness_sec') <= 5: bad.append('freshness_sec')
        if g('dispatch_low') >= g('dispatch_high'): bad.append('dispatch_low/high')
        if g('dispatch_need_high') < 1 or g('dispatch_need_low') < 1: bad.append('dispatch_need_*')
        if g('dispatch_cooldown_sec') < 0: bad.append('dispatch_cooldown_sec')
        if g('dock_max_duration_sec') <= 0: bad.append('dock_max_duration_sec')
        if bad:
            raise ValueError('invalid parameter(s): ' + ', '.join(bad))

    # ----------------------------------------------------------------- inputs
    def _rx(self, key, msg):
        self.latest[key] = (time.monotonic(), msg)

    def _fresh_msg(self, key):
        """Message if fresh by BOTH local receipt time and source stamp."""
        item = self.latest[key]
        if item is None:
            return None
        rx, msg = item
        if time.monotonic() - rx > self.fresh:
            return None
        stamp = msg.header.stamp
        if stamp.sec == 0 and stamp.nanosec == 0:
            return None                                   # unset stamp = unknown
        age = (self.get_clock().now() - Time.from_msg(stamp)).nanoseconds / 1e9
        if age > self.fresh or age < -self.future_tol:
            return None
        return msg

    # --------------------------------------------------------------- crossing
    def _observe(self, now):
        """Pedestrian + which agents overlap C. valid=False means UNKNOWN."""
        q = self._fresh_msg('queue')
        v = self._fresh_msg('vehicles')
        valid = bool(q is not None and q.valid and v is not None)
        occupants = set()
        if v is not None:
            for veh in v.vehicles:
                if not veh.valid:
                    valid = False        # an agent we cannot locate might be in C
                    continue
                r = RADIUS.get(veh.kind, 0.30)
                if footprint_overlaps_crossing(veh.pose.x, veh.pose.y, r):
                    occupants.add(veh.vehicle_id)
        ped = bool(q.crossing_occupied) if q is not None else False
        # safety: unknown or latched => no new grants
        if self.require_safety:
            s = self._fresh_msg('safety')
            self.arbiter.safety_latched = (s is None or s.stop_latched)
        else:
            self.arbiter.safety_latched = False
        self.arbiter.update_observation(now, ped, occupants, valid)

    def _crossing_tick(self):
        self._observe(time.monotonic())
        snap = self.arbiter.snapshot()
        msg = CrossingState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.owner_id, msg.token, msg.state = snap['owner_id'], snap['token'], snap['state']
        msg.pedestrian_present, msg.valid = snap['pedestrian_present'], snap['valid']
        self.crossing_pub.publish(msg)
        self._flush_events()

    def _on_crossing(self, req, res):
        granted, token, reason = self.arbiter.request(
            time.monotonic(), req.requester_id, req.operation, req.token)
        res.granted, res.token, res.reason = granted, token, reason
        key = (req.requester_id, req.operation)
        if not granted and self._last_deny.get(key) != reason:   # log changes, not every retry
            self._event('CROSSING_DENIED', req.requester_id, reason, req.operation)
        if granted:
            self._last_deny.pop(key, None)
        else:
            self._last_deny[key] = reason
        self._flush_events()
        return res

    # --------------------------------------------------------------- dispatch
    def _dispatch_tick(self):
        d = self._fresh_msg('demand')
        q = self._fresh_msg('queue')
        b = self._fresh_msg('bus')
        s = self._fresh_msg('safety')
        safety_ok = (s is not None and not s.stop_latched) if self.require_safety else True
        bus_idle = bool(b is not None and b.state == 'IDLE' and safety_ok)
        decision = self.policy.step(
            time.monotonic(),
            pred_valid=bool(d is not None and d.valid),
            predicted=d.predicted_count if d is not None else 0.0,
            queue_valid=bool(q is not None and q.valid),
            queue_count=q.count if q is not None else 0,
            bus_idle=bus_idle)
        if decision is not None:
            self._send_dock(decision)

    def _send_dock(self, decision):
        corr = decision.correlation_id
        if not self.dock_client.server_is_ready():
            self._event('BUS_DISPATCH_FAILED', self.bus_id, 'DOCK_SERVER_UNAVAILABLE', corr)
            self.policy.finished(corr)           # cooldown still applies
            return
        goal = DockBus.Goal()
        goal.bus_id, goal.bay_id = self.bus_id, self.bay_id
        goal.max_duration_sec = self.dock_max
        self._event('BUS_DISPATCH_REQUESTED', self.bus_id, decision.source, corr)
        fut = self.dock_client.send_goal_async(goal)
        fut.add_done_callback(partial(self._on_dock_accept, corr))

    def _on_dock_accept(self, corr, fut):
        try:
            handle = fut.result()
        except Exception as exc:                 # noqa: BLE001
            self._event('BUS_DISPATCH_FAILED', self.bus_id, 'SEND_ERROR:%s' % exc, corr)
            self.policy.finished(corr)
            return
        if corr != self.policy.outstanding:      # late reply: never start anything new
            self._event('LATE_GOAL_RESPONSE_IGNORED', self.bus_id, 'DOCK', corr)
            return
        if not handle.accepted:
            self._event('BUS_DISPATCH_REJECTED', self.bus_id, 'GOAL_REJECTED', corr)
            self.policy.finished(corr)
            return
        self._event('BUS_DISPATCH_ACCEPTED', self.bus_id, 'OK', corr)
        handle.get_result_async().add_done_callback(partial(self._on_dock_result, corr))

    def _on_dock_result(self, corr, fut):
        try:
            wrapped = fut.result()
            status, result = wrapped.status, wrapped.result
            name = {GoalStatus.STATUS_SUCCEEDED: 'BUS_MISSION_SUCCEEDED',
                    GoalStatus.STATUS_CANCELED: 'BUS_MISSION_CANCELED',
                    GoalStatus.STATUS_ABORTED: 'BUS_MISSION_ABORTED'}.get(status, 'BUS_MISSION_ENDED')
            self._event(name, self.bus_id, result.reason or result.final_state, corr)
        except Exception as exc:                 # noqa: BLE001
            self._event('BUS_MISSION_ENDED', self.bus_id, 'RESULT_ERROR:%s' % exc, corr)
        if not self.policy.finished(corr):
            self._event('LATE_RESULT_IGNORED', self.bus_id, 'DOCK', corr)

    # ---------------------------------------------------------------- escort
    def send_escort(self, request_id, passenger_id, pickup='R', destination='D',
                    max_duration_sec=240.0):
        """Send one escort goal. One outstanding at a time.

        TODO(team): decide who calls this at t=70 s (scenario_runner request?).
        The assignment lists no trigger interface, so none is invented here.
        """
        if self.escort_outstanding:
            self._event('ESCORT_REJECTED', passenger_id, 'BUSY', request_id)
            return False
        if not self.escort_client.server_is_ready():
            self._event('ESCORT_FAILED', passenger_id, 'ESCORT_SERVER_UNAVAILABLE', request_id)
            return False
        goal = EscortPassenger.Goal()
        goal.request_id, goal.passenger_id = request_id, passenger_id
        goal.pickup_node, goal.destination_node = pickup, destination
        goal.max_duration_sec = float(max_duration_sec)
        self.escort_outstanding = request_id
        self._event('ESCORT_REQUESTED', passenger_id, 'OK', request_id)
        self.escort_client.send_goal_async(goal).add_done_callback(
            partial(self._on_escort_accept, request_id, passenger_id))
        return True

    def _on_escort_accept(self, request_id, passenger_id, fut):
        try:
            handle = fut.result()
        except Exception as exc:                 # noqa: BLE001
            self._event('ESCORT_FAILED', passenger_id, 'SEND_ERROR:%s' % exc, request_id)
            self.escort_outstanding = ''
            return
        if request_id != self.escort_outstanding:
            self._event('LATE_GOAL_RESPONSE_IGNORED', passenger_id, 'ESCORT', request_id)
            return
        if not handle.accepted:
            self._event('ESCORT_REJECTED', passenger_id, 'GOAL_REJECTED', request_id)
            self.escort_outstanding = ''
            return
        handle.get_result_async().add_done_callback(
            partial(self._on_escort_result, request_id, passenger_id))

    def _on_escort_result(self, request_id, passenger_id, fut):
        try:
            wrapped = fut.result()
            ok = wrapped.status == GoalStatus.STATUS_SUCCEEDED
            self._event('ESCORT_SUCCEEDED' if ok else 'ESCORT_ENDED', passenger_id,
                        wrapped.result.reason or ('OK' if ok else 'NOT_SUCCEEDED'), request_id)
        except Exception as exc:                 # noqa: BLE001
            self._event('ESCORT_ENDED', passenger_id, 'RESULT_ERROR:%s' % exc, request_id)
        if request_id == self.escort_outstanding:
            self.escort_outstanding = ''

    # ----------------------------------------------------------------- events
    def _event(self, event, entity, reason, corr=''):
        msg = HubEvent()
        if hasattr(msg, 'header'):
            msg.header.stamp = self.get_clock().now().to_msg()
        msg.run_id, msg.scenario_id = self.run_id, self.scenario_id
        msg.entity_id, msg.event, msg.reason = entity, event, reason
        msg.correlation_id = corr
        self.event_pub.publish(msg)

    def _flush_events(self):
        events, self.arbiter.events = self.arbiter.events, []
        for event, entity, reason, corr in events:
            self._event(event, entity, reason, corr)


def main(args=None):
    rclpy.init(args=args)
    node = HubCoordinator()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
