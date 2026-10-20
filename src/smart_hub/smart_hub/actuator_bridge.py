"""actuator_bridge (M5): final commands -> simulated motors, with its own watchdog.

Subscribes  /robot/cmd_vel, /bus/cmd_vel   geometry_msgs/Twist  R1  (safety_supervisor only)
            /bus/door_request              std_msgs/Bool        R1  (M2-A bus_manager)
            /parking/barrier_request       std_msgs/Bool        R1  (M3 parking_manager)
Publishes   /robot/cmd_vel_applied,
            /bus/cmd_vel_applied           geometry_msgs/Twist  R1, 20 Hz -> scenario_runner
            /bus/door_open                 std_msgs/Bool        R1, 10 Hz (measured door state)
            /parking/barrier_open          std_msgs/Bool        R1, 10 Hz (measured barrier state)
            /hub/events                    HubEvent             R100, transitions only

/robot/cmd_vel has no header, so freshness is judged by local receipt time
(time.monotonic()). If the supervisor stops publishing for longer than
command_timeout, the bridge outputs zero on its own (assignment 16.2, T17).
The *_applied topics are an M5-internal contract between this bridge and the
simulator; a hardware build would replace them with motor-driver calls.
"""
import time

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from smart_hub.bridge_core import AGENTS, BridgeConfig, BridgeCore
from smart_hub.ros_common import declare, R1, R100
from smart_hub_interfaces.msg import HubEvent
from std_msgs.msg import Bool


class ActuatorBridge(Node):
    def __init__(self, **kwargs):
        super().__init__('actuator_bridge', **kwargs)
        cfg_fields = dict(vars(BridgeConfig()))
        p = declare(self, dict(cfg_fields, rate_hz=20.0, mode='sim', run_id='run-0'))
        if p['mode'] != 'sim':
            # Hardware needs a measured driver and an independent power cut
            # (assignment 22); refuse rather than pretend.
            raise ValueError('only mode:=sim is implemented')
        self.core = BridgeCore(BridgeConfig(**{k: p[k] for k in cfg_fields}))
        self.run_id = p['run_id']
        self.barrier_requested = False
        self._tick = 0

        self.out_pub = {a: self.create_publisher(Twist, '/%s/cmd_vel_applied' % a, R1)
                        for a in AGENTS}
        self.door_pub = self.create_publisher(Bool, '/bus/door_open', R1)
        self.barrier_pub = self.create_publisher(Bool, '/parking/barrier_open', R1)
        self.event_pub = self.create_publisher(HubEvent, '/hub/events', R100)
        for agent in AGENTS:
            self.create_subscription(Twist, '/%s/cmd_vel' % agent,
                                     lambda m, a=agent: self.on_cmd(a, m), R1)
        self.create_subscription(Bool, '/bus/door_request',
                                 lambda m: self.core.on_door_request(m.data), R1)
        self.create_subscription(Bool, '/parking/barrier_request', self.on_barrier, R1)
        self.create_timer(1.0 / p['rate_hz'], self.tick)
        self.get_logger().info('actuator_bridge ready (sim): %s' % self.core.cfg)

    def on_cmd(self, agent, msg):
        self.core.on_command(agent, msg.linear.x, msg.angular.z, time.monotonic())

    def on_barrier(self, msg):
        # Simulated barrier follows the request; occupancy interlocks are in
        # M3's parking_manager. A physical bridge must re-check them here.
        self.barrier_requested = bool(msg.data)

    def tick(self):
        out, door_open, events = self.core.tick(time.monotonic())
        for agent, (vx, wz) in out.items():
            msg = Twist()
            msg.linear.x, msg.angular.z = float(vx), float(wz)
            self.out_pub[agent].publish(msg)
        self._tick += 1
        if self._tick % 2 == 0:
            self.door_pub.publish(Bool(data=door_open))
            self.barrier_pub.publish(Bool(data=self.barrier_requested))
        for event, entity, reason in events:
            ev = HubEvent()
            ev.header.stamp = self.get_clock().now().to_msg()
            ev.run_id, ev.entity_id, ev.event, ev.reason = self.run_id, entity, event, reason
            ev.correlation_id = 'bridge:%s' % entity
            self.event_pub.publish(ev)
            self.get_logger().info('%s %s %s' % (event, entity, reason))


def main(args=None):
    rclpy.init(args=args)
    node = ActuatorBridge()
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
