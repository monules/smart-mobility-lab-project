"""bus_manager (M2-A): S3 bus arrival and docking, DockBus action server.

Serves      /bus/dock               DockBus action (client: hub_coordinator)
Subscribes  /hub/vehicles           VehicleStateArray  R10 (bus pose, other bodies)
            /bus/door_open          std_msgs/Bool      R1  (measured, actuator_bridge)
            /hub/safety             SafetyState        R1T
            /hub/events             HubEvent           R100 (scenario reset handshake)
Publishes   /bus/cmd_vel_candidate  TwistStamped       R1, 20 Hz (zero while idle)
            /bus/door_request       std_msgs/Bool      R1, 20 Hz
            /hub/bus_arrival        BusArrival         R10, 2 Hz
            /bus/route              nav_msgs/Path      R1T (empty path = no route)
            /hub/events             HubEvent           R100 (phases, crossing, docking)
Calls       /crossing/request       RequestCrossing    (hub_coordinator, M2-B)
            /route/compute          ComputeRoute       (route_planner, M4)

The manager only proposes velocities; safety_supervisor decides the final
/bus/cmd_vel. All docking logic is in bus_core.py (pure Python, unit tested).

Route: /route/compute is asked for B (agent_type bus) when the service exists;
if it is not available or does not answer within route_timeout_sec the
baseline A-N-Cn-B from the map is used and a ROUTE_FALLBACK event says so.

One single-threaded executor: the action server has its own callback group
and its execute callback awaits a future that the 20 Hz timer completes, so
the control loop never waits for the action machinery (same pattern as the
scenario_runner reset service).
"""
import time

from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Path
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from rclpy.task import Future
from smart_hub import bus_core
from smart_hub.ros_common import (declare, R1, R10, R100, R1T, reset_ack,
                                  RESET_REQUEST, stamp_age)
from smart_hub_interfaces.action import DockBus
from smart_hub_interfaces.msg import BusArrival, HubEvent, SafetyState, VehicleStateArray
from smart_hub_interfaces.srv import ComputeRoute, RequestCrossing
from std_msgs.msg import Bool

RATE = 20.0
CONFIG_KEYS = ('cruise_speed', 'pos_tol', 'yaw_tol', 'dock_hold', 'board_sec',
               'pose_timeout', 'stale_abort', 'acquire_ahead', 'stop_margin',
               'renew_period', 'bay_clear_radius', 'approach_allowance')


class BusManager(Node):
    def __init__(self, **kwargs):
        super().__init__('bus_manager', **kwargs)
        cfg = bus_core.BusConfig()
        p = declare(self, dict({'bus_id': 'bus_1', 'bay_id': 'B', 'run_id': '',
                                'scenario_id': '', 'route_timeout_sec': 1.0,
                                'future_tolerance_sec': 0.2},
                               **{k: float(getattr(cfg, k)) for k in CONFIG_KEYS}))
        for k in CONFIG_KEYS:
            setattr(cfg, k, float(p[k]))
        cfg.bus_id = p['bus_id']
        self.core = bus_core.BusCore(cfg)            # validates the parameters
        self.bus_id, self.bay_id = p['bus_id'], p['bay_id']
        self.run_id, self.scenario_id = p['run_id'], p['scenario_id']
        self.route_timeout = float(p['route_timeout_sec'])
        self.future_tol = float(p['future_tolerance_sec'])
        if not 0.1 <= self.route_timeout <= 10.0:
            raise ValueError('route_timeout_sec must be in [0.1, 10]')

        self.goal = None            # active ServerGoalHandle
        self.goal_done = None       # Future the execute coroutine awaits
        self.planning = None        # (start time, route future or None)
        self.door_req = False
        self._reset_id = ''
        self._ticks = 0
        self.last_out = bus_core.Output()

        self.cmd_pub = self.create_publisher(TwistStamped, '/bus/cmd_vel_candidate', R1)
        self.door_pub = self.create_publisher(Bool, '/bus/door_request', R1)
        self.arrival_pub = self.create_publisher(BusArrival, '/hub/bus_arrival', R10)
        self.route_pub = self.create_publisher(Path, '/bus/route', R1T)
        self.event_pub = self.create_publisher(HubEvent, '/hub/events', R100)
        self.create_subscription(VehicleStateArray, '/hub/vehicles', self.on_vehicles, R10)
        self.create_subscription(Bool, '/bus/door_open', self.on_door, R1)
        self.create_subscription(SafetyState, '/hub/safety', self.on_safety, R1T)
        self.create_subscription(HubEvent, '/hub/events', self.on_hub_event, R100)
        self.crossing_cli = self.create_client(RequestCrossing, '/crossing/request')
        self.route_cli = self.create_client(ComputeRoute, '/route/compute')
        self.action = ActionServer(
            self, DockBus, '/bus/dock', execute_callback=self.execute,
            goal_callback=self.on_goal, cancel_callback=self.on_cancel,
            callback_group=MutuallyExclusiveCallbackGroup())
        self.create_timer(1.0 / RATE, self.tick)
        self.publish_route([])
        self.get_logger().info('bus_manager up: %s, bay %s' % (self.bus_id, self.bay_id))

    # ------------------------------------------------------------- inputs
    def on_vehicles(self, msg):
        age = stamp_age(self, msg.header.stamp)
        if age is None or age > self.core.cfg.pose_timeout or age < -self.future_tol:
            return                                      # old or future data: not a pose
        now = time.monotonic()
        others = []
        for v in msg.vehicles:
            if v.vehicle_id == self.bus_id:
                if v.valid:
                    self.core.on_pose(v.pose.x, v.pose.y, v.pose.theta, v.speed_mps, now)
            elif v.kind != 'pedestrian':
                others.append((v.vehicle_id, v.pose.x, v.pose.y))
        self.core.on_others(others)

    def on_door(self, msg):
        self.core.on_door(msg.data, time.monotonic())

    def on_safety(self, msg):
        self.core.on_safety(msg.stop_latched)

    def on_hub_event(self, msg):
        """Scenario reset (contract §7): end any goal, zero output, then acknowledge."""
        if msg.event != RESET_REQUEST:
            return
        if msg.correlation_id != self._reset_id:        # once per run id
            self._reset_id = msg.correlation_id
            if self.goal is not None:
                self.finish((False, bus_core.SAFE_STOP, 'SCENARIO_RESET'))
            self.planning = None
            self.core.reset()
            self.publish_cmd(0.0, 0.0)
            self.door_pub.publish(Bool(data=False))
        self.event_pub.publish(reset_ack(self, msg.correlation_id, 'bus idle'))

    # ------------------------------------------------------------- action
    def on_goal(self, goal):
        reason = ''
        if goal.bus_id != self.bus_id:
            reason = 'UNKNOWN_BUS'
        elif goal.bay_id not in ('', self.bay_id):
            reason = 'UNKNOWN_BAY'
        elif not goal.max_duration_sec > 0:
            reason = 'INVALID_DURATION'
        elif self.goal is not None or self.core.active:
            reason = 'BUSY'
        elif not self.core._pose_fresh(time.monotonic()):
            reason = 'POSE_UNKNOWN'
        elif self.core.safety_latched:
            reason = 'SAFETY_LATCHED'
        self.event('DOCK_GOAL_REJECTED' if reason else 'DOCK_GOAL_ACCEPTED', reason or 'OK')
        return GoalResponse.REJECT if reason else GoalResponse.ACCEPT

    def on_cancel(self, goal_handle):
        self.core.cancel()
        self.publish_cmd(0.0, 0.0)                      # zero at once, not on the next tick
        self.event('DOCK_CANCEL_ACCEPTED', 'client request')
        return CancelResponse.ACCEPT

    async def execute(self, goal_handle):
        self.goal, self.goal_done = goal_handle, Future()
        request = None
        if self.route_cli.service_is_ready():
            request = self.route_cli.call_async(ComputeRoute.Request(
                start_node='A', goal_node=self.bay_id, agent_type='bus'))
        self.planning = (time.monotonic(), request)
        return await self.goal_done

    def finish(self, done):
        success, final_state, reason = done
        handle, self.goal, self.planning = self.goal, None, None
        result = DockBus.Result(success=success, final_state=final_state, reason=reason)
        if handle is not None and handle.is_active:
            if success:
                handle.succeed()
            elif reason == 'CANCELED' and handle.is_cancel_requested:
                handle.canceled()
            else:
                handle.abort()
        self.event('DOCK_RESULT', '%s %s %s' % ('success' if success else 'failed',
                                                final_state, reason))
        self.publish_route([])
        if self.goal_done is not None and not self.goal_done.done():
            self.goal_done.set_result(result)

    def _start_route(self, now):
        started, request = self.planning
        route = None
        if request is not None and request.done():
            res = request.result()
            if res is not None and res.success and res.waypoints:
                route = [(w.x, w.y) for w in res.waypoints]
                self.event('ROUTE_PLANNED', '%s cost %.1f s'
                           % ('-'.join(res.node_ids), res.estimated_cost_sec))
        elif request is not None and now - started < self.route_timeout:
            return                                       # still waiting for M4's planner
        if route is None:
            route = bus_core.node_route()
            self.event('ROUTE_FALLBACK', 'baseline %s (no route from /route/compute)'
                       % '-'.join(bus_core.BASELINE_ROUTE))
        self.planning = None
        try:
            self.core.start(route, now, self.goal.request.max_duration_sec)
        except (RuntimeError, ValueError) as exc:
            self.finish((False, bus_core.SAFE_STOP, 'START_FAILED:%s' % exc))
            return
        self.publish_route(self.core.path.p)

    # ------------------------------------------------------------- 20 Hz
    def tick(self):
        now = time.monotonic()
        if self.goal is not None:
            if self.goal.is_cancel_requested:
                self.core.cancel()
            if self.planning is not None:
                if self.goal.is_cancel_requested:
                    self.finish((False, bus_core.SAFE_STOP, 'CANCELED'))
                else:
                    self._start_route(now)
        out = self.core.step(now)
        self.last_out = out
        self.publish_cmd(*out.cmd)
        if out.door_request != self.door_req:
            self.event('DOOR_REQUEST', 'open' if out.door_request else 'close')
        self.door_req = out.door_request
        self.door_pub.publish(Bool(data=out.door_request))
        if out.crossing_op:
            self.send_crossing(*out.crossing_op)
        for event, reason in self.core.events:
            self.event(event, reason)
        self.core.events = []
        self._ticks += 1
        if self.goal is not None and self.planning is None and self._ticks % 4 == 0:
            self.goal.publish_feedback(DockBus.Feedback(
                phase=out.phase, distance_remaining_m=float(out.distance_remaining),
                eta_sec=float(out.eta if out.eta_valid else -1.0)))
        if self._ticks % 10 == 0:
            self.publish_arrival(out)
        if out.done and self.goal is not None:
            self.finish(out.done)

    # ------------------------------------------------------------- outputs
    def send_crossing(self, operation, token):
        self.core.cross_inflight = True
        if not self.crossing_cli.service_is_ready():
            self.core.on_crossing_response(operation, False, '', 'NO_ARBITER', time.monotonic())
            return
        future = self.crossing_cli.call_async(RequestCrossing.Request(
            requester_id=self.bus_id, operation=operation, token=token))

        def done(fut):
            res = fut.result()
            if res is None:
                self.core.on_crossing_response(operation, False, '', 'NO_RESPONSE',
                                               time.monotonic())
                return
            self.core.on_crossing_response(operation, res.granted, res.token, res.reason,
                                           time.monotonic())
            if not res.granted:
                self.event('CROSSING_%s_DENIED' % operation, res.reason)
        future.add_done_callback(done)

    def publish_cmd(self, v, w):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = '%s/base_link' % self.bus_id
        msg.twist.linear.x, msg.twist.angular.z = float(v), float(w)
        self.cmd_pub.publish(msg)

    def publish_arrival(self, out):
        msg = BusArrival()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.bus_id = self.bus_id
        msg.state = out.phase if self.goal is not None or out.done else bus_core.IDLE
        msg.eta_valid = bool(out.eta_valid)
        msg.eta_sec = float(out.eta) if out.eta_valid else -1.0
        door = self.core.door_open
        msg.door_open = bool(door and door[0])
        self.arrival_pub.publish(msg)

    def publish_route(self, points):
        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = self.get_clock().now().to_msg()
        for x, y in points:
            pose = PoseStamped(header=path.header)
            pose.pose.position.x, pose.pose.position.y = float(x), float(y)
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        self.route_pub.publish(path)

    def event(self, event, reason):
        msg = HubEvent()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.run_id, msg.scenario_id = self.run_id, self.scenario_id
        msg.entity_id, msg.event, msg.reason = self.bus_id, event, reason
        msg.correlation_id = self._reset_id
        self.event_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = BusManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
