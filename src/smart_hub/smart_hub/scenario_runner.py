"""scenario_runner (M5): 20 Hz deterministic simulator and scenario driver.

Subscribes  /robot/cmd_vel_applied, /bus/cmd_vel_applied  Twist  R1 (actuator_bridge)
            /scenario/inject   std_msgs/String (JSON event)  R10  TEST-ONLY fault injection
Publishes   /robot/odom        nav_msgs/Odometry       20 Hz, R10
            /hub/vehicles      VehicleStateArray       10 Hz, R10 (observed state)
            /sim/truth         VehicleStateArray       10 Hz, R10 (private ground truth,
                                                       for metrics only; never frozen)
            /parking/observed  SlotObservationArray    5 Hz, R10
            /hub/estop         std_msgs/Bool           10 Hz, R1T (if publish_estop)
            /hub/events        HubEvent                R100, scenario events
Serves      /scenario/reset    ResetScenario

The world advances by a fixed 0.05 s step per timer tick (assignment 3.2), so
the same scenario file and seed give the same run. A "freeze" event stops the
named publisher for its duration to simulate a sensor dropout.
"""
import json
import math
import os

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from smart_hub import geometry as geo
from smart_hub.ros_common import declare, R1, R10, R100, R1T
from smart_hub.sim_core import DT, HubSim
from smart_hub_interfaces.msg import (HubEvent, SlotObservation, SlotObservationArray,
                                      VehicleState, VehicleStateArray)
from smart_hub_interfaces.srv import ResetScenario
from std_msgs.msg import Bool, String
import yaml


def default_scenario_dir():
    try:
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(get_package_share_directory('smart_hub'), 'scenarios')
    except Exception:  # noqa: BLE001 - running from a source checkout
        return os.path.join(os.path.dirname(os.path.dirname(__file__)), 'scenarios')


def load_scenario(directory, scenario_id):
    path = os.path.join(directory, '%s.yaml' % scenario_id)
    with open(path) as fh:
        data = yaml.safe_load(fh)
    if not isinstance(data, dict) or data.get('scenario_id') != scenario_id:
        raise ValueError('%s: scenario_id must be %r' % (path, scenario_id))
    return data


class ScenarioRunner(Node):
    def __init__(self, **kwargs):
        super().__init__('scenario_runner', **kwargs)
        p = declare(self, {'scenario': 'S9', 'seed': 9, 'scenario_dir': '',
                           'publish_estop': True, 'robot_id': 'robot_1',
                           'bus_id': 'bus_1', 'input_timeout': 0.5})
        self.dir = p['scenario_dir'] or default_scenario_dir()
        self.publish_estop = bool(p['publish_estop'])
        self.ids = {'robot': p['robot_id'], 'bus': p['bus_id']}
        self.input_timeout = float(p['input_timeout'])
        self.reset_count = 0
        self._start(p['scenario'], int(p['seed']))

        self.odom_pub = self.create_publisher(Odometry, '/robot/odom', R10)
        self.vehicles_pub = self.create_publisher(VehicleStateArray, '/hub/vehicles', R10)
        self.truth_pub = self.create_publisher(VehicleStateArray, '/sim/truth', R10)
        self.parking_pub = self.create_publisher(SlotObservationArray, '/parking/observed', R10)
        self.estop_pub = self.create_publisher(Bool, '/hub/estop', R1T)
        self.event_pub = self.create_publisher(HubEvent, '/hub/events', R100)
        for agent in ('robot', 'bus'):
            self.create_subscription(Twist, '/%s/cmd_vel_applied' % agent,
                                     lambda m, a=agent: self.on_applied(a, m), R1)
        self.create_subscription(String, '/scenario/inject', self.on_inject, R10)
        self.create_service(ResetScenario, '/scenario/reset', self.on_reset)
        self.create_timer(DT, self.tick)
        self._event('SCENARIO_STARTED', 'scenario_runner', 'seed=%d' % self.sim.seed)
        self.get_logger().info('scenario %s seed %d run %s' % (
            self.scenario_id, self.sim.seed, self.run_id))

    def _start(self, scenario_id, seed):
        # Load and validate first: a failed reset must leave the old run intact.
        sim = HubSim(load_scenario(self.dir, scenario_id), seed, self.input_timeout)
        self.scenario_id, self.sim = scenario_id, sim
        self.reset_count += 1
        self.run_id = '%s-s%d-r%d' % (scenario_id, seed, self.reset_count)
        self.completed = False

    # ------------------------------------------------------------- inputs
    def on_applied(self, agent, msg):
        self.sim.set_command(self.ids[agent], msg.linear.x, msg.angular.z)

    def on_inject(self, msg):
        try:
            event = self.sim.apply_event(json.loads(msg.data))
        except (ValueError, TypeError, KeyError) as exc:
            self.get_logger().warning('rejected injection %r: %s' % (msg.data, exc))
            return
        self._event('FAULT_INJECTED', 'scenario_runner', json.dumps(event, sort_keys=True))

    def on_reset(self, req, res):
        scenario_id = req.scenario_id or self.scenario_id
        try:
            self._start(scenario_id, req.seed)
        except (OSError, ValueError, yaml.YAMLError) as exc:
            res.success, res.run_id, res.reason = False, '', 'RESET_FAILED: %s' % exc
            return res
        # The new world starts with every body at rest; the stale command
        # timeout keeps it there until fresh commands arrive.
        res.success, res.run_id = True, self.run_id
        res.reason = 'RESET preserve_logs=%s' % req.preserve_logs
        self._event('SCENARIO_RESET', 'scenario_runner', res.reason)
        return res

    # --------------------------------------------------------------- tick
    def tick(self):
        for event in self.sim.step():
            self._event('SCENARIO_EVENT', 'scenario_runner', json.dumps(event, sort_keys=True))
        stamp = self.get_clock().now().to_msg()
        step = self.sim.steps
        if not self.sim.is_frozen('odom'):
            self._publish_odom(stamp)
        if step % 2 == 0:
            self.truth_pub.publish(self._vehicles(stamp, noisy=False))
            if not self.sim.is_frozen('vehicles'):
                self.vehicles_pub.publish(self._vehicles(stamp, noisy=True))
            if self.publish_estop:
                self.estop_pub.publish(Bool(data=self.sim.estop))
        if step % 4 == 0 and not self.sim.is_frozen('parking'):
            self._publish_parking(stamp)
        if self.sim.finished() and not self.completed:
            self.completed = True
            self._event('SCENARIO_COMPLETE', 'scenario_runner', 't=%.2f' % self.sim.t)

    def _publish_odom(self, stamp):
        body = self.sim.bodies.get(self.ids['robot'])
        if body is None:
            return
        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = 'robot/odom'          # map->odom is identity here
        msg.child_frame_id = 'robot/base_link'
        msg.pose.pose.position.x, msg.pose.pose.position.y = body.x, body.y
        msg.pose.pose.orientation.z = math.sin(body.theta / 2.0)
        msg.pose.pose.orientation.w = math.cos(body.theta / 2.0)
        msg.twist.twist.linear.x, msg.twist.twist.angular.z = body.v, body.w
        self.odom_pub.publish(msg)

    def _vehicles(self, stamp, noisy):
        msg = VehicleStateArray()
        msg.header.stamp = stamp
        msg.header.frame_id = 'map'
        for body in self.sim.visible_bodies():
            x, y = self.sim.observe(body) if noisy else (body.x, body.y)
            v = VehicleState()
            v.vehicle_id, v.kind = body.vehicle_id, body.kind
            v.pose.x, v.pose.y, v.pose.theta = float(x), float(y), float(body.theta)
            v.speed_mps = float(abs(body.v))
            v.edge_id = geo.nearest_edge(x, y) if body.kind != 'obstacle' else ''
            v.valid = True
            msg.vehicles.append(v)
        return msg

    def _publish_parking(self, stamp):
        msg = SlotObservationArray()
        msg.header.stamp = stamp
        msg.header.frame_id = 'map'
        for slot, state in sorted(self.sim.parking.items()):
            msg.slots.append(SlotObservation(
                slot_id=slot, occupied=(state == 'occupied'), confidence=1.0,
                valid=(state != 'offline')))
        self.parking_pub.publish(msg)

    def _event(self, event, entity, reason):
        msg = HubEvent()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.run_id, msg.scenario_id = self.run_id, self.scenario_id
        msg.entity_id, msg.event, msg.reason = entity, event, reason
        msg.correlation_id = self.run_id
        self.event_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ScenarioRunner()
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
