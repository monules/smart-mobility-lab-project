"""safety_supervisor (M5): the only node that publishes final motor commands.

Subscribes  /robot/cmd_vel_candidate  geometry_msgs/TwistStamped  R1   (M4)
            /bus/cmd_vel_candidate    geometry_msgs/TwistStamped  R1   (M2-A)
            /robot/odom               nav_msgs/Odometry           R10  (scenario_runner)
            /hub/vehicles             VehicleStateArray           R10  (scenario_runner)
            /crossing/state           CrossingState               R1T  (M2-B)
            /bus/door_open            std_msgs/Bool               R1   (actuator_bridge)
            /hub/estop                std_msgs/Bool               R1T  (operator/runner)
Publishes   /robot/cmd_vel, /bus/cmd_vel  geometry_msgs/Twist     R1, 20 Hz
            /hub/safety               SafetyState                 R1T, 10 Hz + on change
            /hub/events               HubEvent                    R100, transitions only
Serves      /safety/clear_stop        ClearSafetyStop

All decisions are made by SafetyCore (safety_core.py). This file converts
messages and time. Everything runs in one mutually exclusive callback group,
so callbacks never run concurrently and no locks are needed. Freshness uses
time.monotonic() (never jumps) plus the message header stamp.
"""
import math
import time

from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from smart_hub.ros_common import declare, R1, R10, R100, R1T, stamp_age
from smart_hub.safety_core import SafetyConfig, SafetyCore
from smart_hub_interfaces.msg import CrossingState, HubEvent, SafetyState, VehicleStateArray
from smart_hub_interfaces.srv import ClearSafetyStop
from std_msgs.msg import Bool


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class SafetySupervisor(Node):
    def __init__(self, **kwargs):
        super().__init__('safety_supervisor', **kwargs)
        cfg_fields = dict(vars(SafetyConfig()))
        p = declare(self, dict(cfg_fields, rate_hz=20.0, run_id='run-0', scenario_id=''))
        if not 10.0 <= p['rate_hz'] <= 100.0:
            raise ValueError('rate_hz must be in [10, 100]')
        self.core = SafetyCore(SafetyConfig(**{k: p[k] for k in cfg_fields}))
        self.run_id, self.scenario_id = p['run_id'], p['scenario_id']
        self.stop_seq = 0
        self._episode_open = False
        self.rejected = 0                       # messages refused (bad stamp/NaN)
        self._tick_count = 0
        self._last_latched = False

        self.cmd_pub = {a: self.create_publisher(Twist, '/%s/cmd_vel' % a, R1)
                        for a in ('robot', 'bus')}
        self.safety_pub = self.create_publisher(SafetyState, '/hub/safety', R1T)
        self.event_pub = self.create_publisher(HubEvent, '/hub/events', R100)

        for agent in ('robot', 'bus'):
            self.create_subscription(TwistStamped, '/%s/cmd_vel_candidate' % agent,
                                     lambda m, a=agent: self.on_candidate(a, m), R1)
        self.create_subscription(Odometry, '/robot/odom', self.on_odom, R10)
        self.create_subscription(VehicleStateArray, '/hub/vehicles', self.on_vehicles, R10)
        self.create_subscription(CrossingState, '/crossing/state', self.on_crossing, R1T)
        self.create_subscription(Bool, '/bus/door_open', self.on_door, R1)
        self.create_subscription(Bool, '/hub/estop', self.on_estop, R1T)
        self.create_service(ClearSafetyStop, '/safety/clear_stop', self.on_clear)
        self.create_timer(1.0 / p['rate_hz'], self.tick)
        self.get_logger().info('safety_supervisor ready: %s' % self.core.cfg)

    # ------------------------------------------------------------- inputs
    def _age_or_reject(self, header):
        age = stamp_age(self, header.stamp)
        if age is None:
            self.rejected += 1                  # unstamped data is never fresh
        return age

    def on_candidate(self, agent, msg):
        age = self._age_or_reject(msg.header)
        if age is None or not self.core.on_command(
                agent, msg.twist.linear.x, msg.twist.angular.z, time.monotonic(), age):
            self.rejected += 1

    def on_odom(self, msg):
        age = self._age_or_reject(msg.header)
        if age is None:
            return
        pose, twist = msg.pose.pose, msg.twist.twist
        speed = math.hypot(twist.linear.x, twist.linear.y)
        self.core.on_robot_odom(pose.position.x, pose.position.y,
                                yaw_from_quaternion(pose.orientation), speed,
                                time.monotonic(), age)

    def on_vehicles(self, msg):
        age = self._age_or_reject(msg.header)
        if age is None:
            return
        entries = [(v.vehicle_id, v.kind, v.pose.x, v.pose.y, v.pose.theta,
                    v.speed_mps, v.valid) for v in msg.vehicles]
        self.core.on_vehicles(entries, time.monotonic(), age)

    def on_crossing(self, msg):
        age = self._age_or_reject(msg.header)
        if age is not None:
            self.core.on_crossing(msg.owner_id, msg.valid, time.monotonic(), age)

    def on_door(self, msg):
        self.core.on_door(msg.data, time.monotonic())

    def on_estop(self, msg):
        t_rx = time.monotonic()
        if self.core.on_estop(msg.data, t_rx):
            # Do not wait for the next tick (up to 50 ms): stop right now.
            self._publish_zero()
            t_out = time.monotonic()
            self._publish_safety(True, False, 'LATCHED:ESTOP')
            self._flush_events()
            self._event('ESTOP_ZERO_OUTPUT', 'safety_supervisor',
                        'receipt_to_zero_ms=%.3f' % ((t_out - t_rx) * 1000.0))

    def on_clear(self, req, res):
        ok, reason = self.core.request_clear(time.monotonic())
        res.success, res.reason = ok, reason
        self.get_logger().info('clear_stop by %r: %s %s' % (req.operator_id, ok, reason))
        self._flush_events(extra=req.operator_id)
        return res

    # ------------------------------------------------------------- output
    def tick(self):
        d = self.core.tick(time.monotonic())
        self._tick_count += 1
        # Safety state first: listeners learn about a latch change before
        # they see the commands that follow from it.
        if self._tick_count % 2 == 0 or d.stop_latched != self._last_latched:
            self._publish_safety(d.stop_latched, d.motion_permitted, d.reason)
        for agent, (vx, wz) in d.commands.items():
            msg = Twist()
            msg.linear.x, msg.angular.z = float(vx), float(wz)
            self.cmd_pub[agent].publish(msg)
        self._flush_events()

    def _publish_zero(self):
        for pub in self.cmd_pub.values():
            pub.publish(Twist())

    def _publish_safety(self, latched, permitted, reason):
        self._last_latched = latched
        msg = SafetyState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.stop_latched, msg.motion_permitted, msg.reason = latched, permitted, reason
        self.safety_pub.publish(msg)

    def _event(self, event, entity, reason, corr=None):
        msg = HubEvent()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.run_id, msg.scenario_id = self.run_id, self.scenario_id
        msg.entity_id, msg.event, msg.reason = entity, event, reason
        msg.correlation_id = corr if corr is not None else 'stop-%d' % self.stop_seq
        self.event_pub.publish(msg)

    def _flush_events(self, extra=''):
        # One correlation id ("stop-N") per stop episode: latch .. clear.
        if self.core.latched and not self._episode_open:
            self.stop_seq += 1
            self._episode_open = True
        events, self.core.events = self.core.events, []
        for event, entity, reason in events:
            corr = '%s:%s' % (entity, reason) if event.startswith('SAFETY_') else None
            if extra and event in ('STOP_CLEARED', 'CLEAR_REJECTED'):
                reason = '%s by %s' % (reason, extra)
            self._event(event, entity, reason, corr)
        if not self.core.latched:
            self._episode_open = False


def main(args=None):
    rclpy.init(args=args)
    node = SafetySupervisor()
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
