"""robot_manager (M4): S7 passenger escort and NavigateTo for robot_1.

Serves      /robot/escort           EscortPassenger action (coordinator, s7_check)
            /robot/navigate_to      NavigateTo action
            /robot/confirm_ready    std_srvs/Trigger: the passenger's "ready" button;
                                    accepted only while waiting at the pickup
Subscribes  /robot/odom             nav_msgs/Odometry R10 (pose and progress)
            /hub/safety             SafetyState R1T
            /traffic/edges, /hub/vehicles, /crossing/state  (route validity, the
                                    same rules as route_planner)
            /hub/events             HubEvent R100 (scenario reset handshake)
Publishes   /robot/cmd_vel_candidate TwistStamped R1, 20 Hz (zero while idle)
            /robot/route            nav_msgs/Path R1T (empty path = no route)
            /hub/events             phases, routes, reroutes, crossing, arrival
Calls       /route/compute          ComputeRoute (route_planner): 1 s timeout,
                                    at most 2 retries, then explicit failure (19.2)
            /crossing/request       RequestCrossing (hub_coordinator), requester robot_1

The manager only proposes velocities; safety_supervisor decides
/robot/cmd_vel. All mission logic is in robot_core.py (pure Python).
One single-threaded executor; each action server has its own callback group
and its execute callback awaits a future that the 20 Hz timer completes.
"""
import math
import time

from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry, Path
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.task import Future
from smart_hub import geometry as geo
from smart_hub import robot_core, route_core
from smart_hub.ros_common import (declare, R1, R10, R100, R1T, reset_ack,
                                  RESET_REQUEST, stamp_age)
from smart_hub_interfaces.action import EscortPassenger, NavigateTo
from smart_hub_interfaces.msg import (CrossingState, EdgeStateArray, HubEvent,
                                      SafetyState, VehicleStateArray)
from smart_hub_interfaces.srv import ComputeRoute, RequestCrossing
from std_srvs.srv import Trigger

RATE = 20.0
CONFIG_KEYS = ('cruise_speed', 'turn_rate', 'goal_tol', 'arrive_hold', 'odom_timeout',
               'stale_abort', 'ready_timeout', 'planner_timeout', 'clearance',
               'replan_period', 'replan_gain')


class RobotManager(Node):
    def __init__(self, **kwargs):
        super().__init__('robot_manager', **kwargs)
        cfg = robot_core.RobotConfig()
        p = declare(self, dict({'robot_id': 'robot_1', 'run_id': '', 'scenario_id': '',
                                'planner_retries': 2, 'future_tolerance_sec': 0.2},
                               **{k: float(getattr(cfg, k)) for k in CONFIG_KEYS}))
        for k in CONFIG_KEYS:
            setattr(cfg, k, float(p[k]))
        cfg.robot_id, cfg.planner_retries = p['robot_id'], int(p['planner_retries'])
        self.view = route_core.EdgeView()
        self.core = robot_core.RobotCore(cfg, self.view)         # validates parameters
        self.robot_id = p['robot_id']
        self.run_id, self.scenario_id = p['run_id'], p['scenario_id']
        self.future_tol = float(p['future_tolerance_sec'])
        self.goal = None            # (kind, ServerGoalHandle)
        self.goal_done = None
        self.seen_requests = set()  # request ids already accepted (deduplication)
        self._reset_id = ''
        self._ticks = 0

        self.cmd_pub = self.create_publisher(TwistStamped, '/robot/cmd_vel_candidate', R1)
        self.route_pub = self.create_publisher(Path, '/robot/route', R1T)
        self.event_pub = self.create_publisher(HubEvent, '/hub/events', R100)
        self.create_subscription(Odometry, '/robot/odom', self.on_odom, R10)
        self.create_subscription(SafetyState, '/hub/safety', self.on_safety, R1T)
        self.create_subscription(EdgeStateArray, '/traffic/edges', self.on_edges, R10)
        self.create_subscription(VehicleStateArray, '/hub/vehicles', self.on_vehicles, R10)
        self.create_subscription(CrossingState, '/crossing/state', self.on_crossing, R1T)
        self.create_subscription(HubEvent, '/hub/events', self.on_hub_event, R100)
        self.crossing_cli = self.create_client(RequestCrossing, '/crossing/request')
        self.route_cli = self.create_client(ComputeRoute, '/route/compute')
        self.create_service(Trigger, '/robot/confirm_ready', self.on_ready)
        self.escort_srv = ActionServer(
            self, EscortPassenger, '/robot/escort', execute_callback=self.execute_escort,
            goal_callback=self.on_escort_goal, cancel_callback=self.on_cancel,
            callback_group=MutuallyExclusiveCallbackGroup())
        self.nav_srv = ActionServer(
            self, NavigateTo, '/robot/navigate_to', execute_callback=self.execute_navigate,
            goal_callback=self.on_navigate_goal, cancel_callback=self.on_cancel,
            callback_group=MutuallyExclusiveCallbackGroup())
        self.create_timer(1.0 / RATE, self.tick)
        self.publish_route([])
        self.get_logger().info('robot_manager up: %s' % self.robot_id)

    # ------------------------------------------------------------- inputs
    def on_odom(self, msg):
        age = stamp_age(self, msg.header.stamp)
        if age is None or age > self.core.cfg.odom_timeout or age < -self.future_tol:
            return
        q = msg.pose.pose.orientation
        theta = 2.0 * math.atan2(q.z, q.w)
        self.core.on_odom(msg.pose.pose.position.x, msg.pose.pose.position.y, theta,
                          abs(msg.twist.twist.linear.x), time.monotonic())

    def on_safety(self, msg):
        self.core.on_safety(msg.stop_latched)

    def on_edges(self, msg):
        self.view.on_traffic([(e.edge_id, e.mean_speed_mps, e.density_ratio, e.blocked,
                               e.valid) for e in msg.edges], time.monotonic())

    def on_vehicles(self, msg):
        self.view.on_bodies([(v.pose.x, v.pose.y, geo.RADIUS_BY_KIND['obstacle'])
                             for v in msg.vehicles if v.kind == 'obstacle' and v.valid],
                            time.monotonic(),
                            {v.vehicle_id: (v.pose.x, v.pose.y) for v in msg.vehicles
                             if v.valid and v.kind in ('bus', 'robot')})

    def on_crossing(self, msg):
        self.view.on_crossing(msg.owner_id, msg.pedestrian_present, msg.valid,
                              time.monotonic())

    def on_hub_event(self, msg):
        """Scenario reset (contract §7): end any mission, zero output, acknowledge."""
        if msg.event != RESET_REQUEST:
            return
        if msg.correlation_id != self._reset_id:
            self._reset_id = msg.correlation_id
            if self.goal is not None:
                self.finish((False, robot_core.SAFE_STOP, 'SCENARIO_RESET', '', -1.0, 0.0))
            self.core.reset()
            self.publish_cmd(0.0, 0.0)
        self.event_pub.publish(reset_ack(self, msg.correlation_id, 'robot idle'))

    def on_ready(self, request, response):
        response.success, response.message = self.core.confirm_ready()
        return response

    # ------------------------------------------------------------- actions
    def _check_goal(self, request_id, nodes, duration):
        if not request_id:
            return 'INVALID_REQUEST_ID'
        if request_id in self.seen_requests:
            return 'DUPLICATE_REQUEST'
        if any(n not in geo.NODES for n in nodes):
            return 'INVALID_NODE'
        if not duration > 0:
            return 'INVALID_DURATION'
        if self.goal is not None or self.core.active:
            return 'BUSY'
        if self.core.safety_latched:
            return 'SAFETY_LATCHED'
        return ''

    def _decide(self, kind, request_id, reason):
        self.event('%s_GOAL_%s' % (kind.upper(), 'REJECTED' if reason else 'ACCEPTED'),
                   '%s %s' % (request_id, reason or 'OK'))
        if reason:
            return GoalResponse.REJECT
        self.seen_requests.add(request_id)
        return GoalResponse.ACCEPT

    def on_escort_goal(self, goal):
        reason = self._check_goal(goal.request_id, [goal.pickup_node, goal.destination_node],
                                  goal.max_duration_sec)
        if not reason and not goal.passenger_id:
            reason = 'INVALID_PASSENGER_ID'
        return self._decide('escort', goal.request_id, reason)

    def on_navigate_goal(self, goal):
        reason = self._check_goal(goal.request_id, [goal.goal_node], goal.max_duration_sec)
        return self._decide('navigate', goal.request_id, reason)

    def on_cancel(self, goal_handle):
        self.core.cancel()
        self.publish_cmd(0.0, 0.0)                      # zero at once
        self.event('ROBOT_CANCEL_ACCEPTED', 'client request')
        return CancelResponse.ACCEPT

    async def execute_escort(self, goal_handle):
        g = goal_handle.request
        return await self._execute('escort', goal_handle,
                                   [g.pickup_node, g.destination_node], g.max_duration_sec)

    async def execute_navigate(self, goal_handle):
        g = goal_handle.request
        return await self._execute('navigate', goal_handle, [g.goal_node], g.max_duration_sec)

    async def _execute(self, kind, goal_handle, legs, duration):
        self.goal, self.goal_done = (kind, goal_handle), Future()
        try:
            self.core.start(kind, legs, time.monotonic(), duration)
        except RuntimeError as exc:
            self.finish((False, robot_core.FAILED, str(exc), '', -1.0, 0.0))
        return await self.goal_done

    def finish(self, done):
        success, final, reason, node, err, travelled = done
        kind, handle = self.goal
        self.goal = None
        if kind == 'escort':
            result = EscortPassenger.Result(success=success, final_node=node,
                                            travelled_distance_m=float(travelled),
                                            reason=reason)
        else:
            result = NavigateTo.Result(success=success, final_node=node,
                                       final_error_m=float(err), reason=reason)
        if handle.is_active:
            if success:
                handle.succeed()
            elif reason == 'CANCELED' and handle.is_cancel_requested:
                handle.canceled()
            else:
                handle.abort()
        self.event('ROBOT_RESULT', '%s %s %s %s final_error=%.3f travelled=%.2f' % (
            kind, 'success' if success else 'failed', final, reason, err, travelled))
        self.publish_route([])
        if self.goal_done is not None and not self.goal_done.done():
            self.goal_done.set_result(result)

    # ------------------------------------------------------------- 20 Hz
    def tick(self):
        now = time.monotonic()
        if self.goal is not None and self.goal[1].is_cancel_requested:
            self.core.cancel()
        had_path = self.core.path
        out = self.core.step(now)
        self.publish_cmd(*out.cmd)
        if out.route_request:
            self.send_route(*out.route_request)
        if out.crossing_op:
            self.send_crossing(*out.crossing_op)
        if self.core.path is not None and self.core.path is not had_path:
            self.publish_route(self.core.path.p)
        for event, reason in self.core.events:
            self.event(event, reason)
        self.core.events = []
        self._ticks += 1
        if self.goal is not None and not out.done and self._ticks % 4 == 0:
            kind, handle = self.goal
            fb = (EscortPassenger if kind == 'escort' else NavigateTo).Feedback(
                phase=out.phase, progress=float(out.progress),
                distance_remaining_m=float(out.distance_remaining))
            handle.publish_feedback(fb)
        if out.done and self.goal is not None:
            self.finish(out.done)

    # ------------------------------------------------------------- outputs
    def send_route(self, attempt, start, goal):
        if not self.route_cli.service_is_ready():
            self.event('PLANNER_NOT_READY', 'attempt %d %s->%s' % (attempt, start, goal))
            return                                       # core times the attempt out
        future = self.route_cli.call_async(ComputeRoute.Request(
            start_node=start, goal_node=goal, agent_type='robot'))

        def done(fut):
            res = fut.result()
            if res is None:
                return
            self.core.on_route(attempt, res.success, list(res.node_ids),
                               res.estimated_cost_sec, res.reason, time.monotonic())
        future.add_done_callback(done)

    def send_crossing(self, operation, token):
        self.core.cross_inflight = True
        if not self.crossing_cli.service_is_ready():
            self.core.on_crossing_response(operation, False, '', 'NO_ARBITER', time.monotonic())
            return
        future = self.crossing_cli.call_async(RequestCrossing.Request(
            requester_id=self.robot_id, operation=operation, token=token))

        def done(fut):
            res = fut.result()
            if res is None:
                self.core.on_crossing_response(operation, False, '', 'NO_RESPONSE',
                                               time.monotonic())
                return
            self.core.on_crossing_response(operation, res.granted, res.token, res.reason,
                                           time.monotonic())
        future.add_done_callback(done)

    def publish_cmd(self, v, w):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = '%s/base_link' % self.robot_id
        msg.twist.linear.x, msg.twist.angular.z = float(v), float(w)
        self.cmd_pub.publish(msg)

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
        msg.entity_id, msg.event, msg.reason = self.robot_id, event, reason
        msg.correlation_id = self._reset_id
        self.event_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = RobotManager()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
