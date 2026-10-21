"""scenario_runner (M5): 20 Hz deterministic simulator and scenario driver.

Subscribes  /robot/cmd_vel_applied, /bus/cmd_vel_applied  Twist  R1 (actuator_bridge)
            /scenario/inject   std_msgs/String (JSON event)  R10  TEST-ONLY fault injection
            /bus/door_open     std_msgs/Bool  R1 (measured door; boarding at bay B)
Publishes   /robot/odom        nav_msgs/Odometry       20 Hz, R10
            /hub/vehicles      VehicleStateArray       10 Hz, R10 (observed state)
            /sim/truth         VehicleStateArray       10 Hz, R10 (private ground truth,
                                                       for metrics only; never frozen)
            /parking/observed  SlotObservationArray    5 Hz, R10
            /hub/estop         std_msgs/Bool           10 Hz, R1T (if publish_estop)
            /camera/image_raw  sensor_msgs/Image bgr8  10 Hz, sensor QoS (if publish_camera)
            /hub/events        HubEvent                R100, scenario events and
                                                       PASSENGER_BOARDED (one per person)
Serves      /scenario/reset    ResetScenario (waits for RESET_ACK from every
                               node in reset_participants, contract §7)

The world advances by a fixed 0.05 s step per timer tick (assignment 3.2), so
the same scenario file and seed give the same run. A "freeze" event stops the
named publisher for its duration to simulate a sensor dropout;
"camera_repeat" re-sends the last frame with its old stamp (frozen source).
Pedestrians are seen only by the camera; their true positions are in
/sim/truth with kind 'pedestrian' (for evaluation, never for decisions).
"""
import json
import math
import os
import time

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.task import Future
from sensor_msgs.msg import Image
from smart_hub import geometry as geo
from smart_hub.ros_common import declare, R1, R10, R100, R1T, RESET_ACK, RESET_REQUEST
from smart_hub.sim_core import DT, HubSim
from smart_hub_interfaces.msg import (HubEvent, SlotObservation, SlotObservationArray,
                                      VehicleState, VehicleStateArray)
from smart_hub_interfaces.srv import ResetScenario
from std_msgs.msg import Bool, String
import yaml

# Floor outlines painted for the camera: queue Q and crossing C (assignment 2.1).
CAMERA_FLOOR = ((2.0, 5.0, 6.0, 7.0), geo.CROSSING)
# Nodes started by hub.launch.py that must acknowledge a reset. Launch files
# add more (e.g. passenger_detector) through the reset_participants parameter.
DEFAULT_PARTICIPANTS = ('safety_supervisor,actuator_bridge,metrics_logger,'
                        'dashboard_marker_publisher')
RESEND_SEC = 0.25    # /hub/events is volatile: repeat the request until all ack


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
                           'bus_id': 'bus_1', 'input_timeout': 0.5,
                           'publish_camera': True,
                           # nodes that must acknowledge /scenario/reset (comma list)
                           'reset_participants': DEFAULT_PARTICIPANTS,
                           'reset_ack_timeout': 2.0})
        self.dir = p['scenario_dir'] or default_scenario_dir()
        self.participants = {n.strip() for n in p['reset_participants'].split(',') if n.strip()}
        self.ack_timeout = float(p['reset_ack_timeout'])
        if not 0.1 <= self.ack_timeout <= 10.0:
            raise ValueError('reset_ack_timeout must be in [0.1, 10]')
        self.pending = None                    # reset waiting for acknowledgements
        self.camera = None
        if p['publish_camera']:
            from smart_hub.camera_model import PinholeCamera   # needs OpenCV
            self.camera = PinholeCamera()
        self.last_frame = None                 # (Image msg) for camera_repeat
        self.publish_estop = bool(p['publish_estop'])
        self.ids = {'robot': p['robot_id'], 'bus': p['bus_id']}
        self.input_timeout = float(p['input_timeout'])
        self.reset_count = 0
        self._install(HubSim(load_scenario(self.dir, p['scenario']), int(p['seed']),
                             self.input_timeout), p['scenario'])

        self.odom_pub = self.create_publisher(Odometry, '/robot/odom', R10)
        self.vehicles_pub = self.create_publisher(VehicleStateArray, '/hub/vehicles', R10)
        self.truth_pub = self.create_publisher(VehicleStateArray, '/sim/truth', R10)
        self.parking_pub = self.create_publisher(SlotObservationArray, '/parking/observed', R10)
        self.estop_pub = self.create_publisher(Bool, '/hub/estop', R1T)
        self.event_pub = self.create_publisher(HubEvent, '/hub/events', R100)
        self.camera_pub = self.create_publisher(Image, '/camera/image_raw',
                                                qos_profile_sensor_data)
        # One single-threaded executor runs everything, so callbacks never
        # overlap and need no locks. The reset service is a coroutine in its
        # own callback group: while it awaits the acknowledgements, the 20 Hz
        # tick and the event listener (default group) keep running. In the
        # default group the waiting service would block them (deadlock), and
        # a multi-threaded executor slowed the tick to ~6 Hz.
        for agent in ('robot', 'bus'):
            self.create_subscription(Twist, '/%s/cmd_vel_applied' % agent,
                                     lambda m, a=agent: self.on_applied(a, m), R1)
        self.create_subscription(Bool, '/bus/door_open',
                                 lambda m: self.sim.set_door(m.data), R1)
        self.create_subscription(String, '/scenario/inject', self.on_inject, R10)
        self.create_subscription(HubEvent, '/hub/events', self.on_event, R100)
        self.create_service(ResetScenario, '/scenario/reset', self.on_reset,
                            callback_group=MutuallyExclusiveCallbackGroup())
        self.create_timer(DT, self.tick)
        self._event('SCENARIO_STARTED', 'scenario_runner', 'seed=%d' % self.sim.seed)
        self.get_logger().info('scenario %s seed %d run %s' % (
            self.scenario_id, self.sim.seed, self.run_id))

    def _install(self, sim, scenario_id):
        """Make sim the running world with a new run id."""
        self.scenario_id, self.sim = scenario_id, sim
        self.reset_count += 1
        self.run_id = '%s-s%d-r%d' % (scenario_id, sim.seed, self.reset_count)
        self.completed = False
        self.last_frame = None
        return self.run_id

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

    def on_event(self, msg):
        pending = self.pending
        if pending and msg.event == RESET_ACK and msg.correlation_id == pending['run_id']:
            pending['acked'].add(msg.entity_id)
            if self.participants <= pending['acked'] and not pending['done'].done():
                pending['done'].set_result(True)

    def _drive_handshake(self, now):
        """Resend the reset request or give up; runs every tick while a reset waits."""
        pending = self.pending
        if pending is None or pending['done'].done():
            return
        if now >= pending['deadline']:
            pending['done'].set_result(False)
        elif now >= pending['resend_at']:
            pending['resend_at'] = now + RESEND_SEC
            self._request_reset()

    def _request_reset(self):
        self._event(RESET_REQUEST, 'scenario_runner',
                    'participants=%s' % ','.join(sorted(self.participants)))

    async def on_reset(self, req, res):
        """Reset the world, ask every participant to reset, wait for their acks.

        Assignment 7.1: ResetScenario "returns a new run identifier after all
        nodes acknowledge reset". success=false if the scenario cannot be
        loaded (old run untouched) or a participant did not acknowledge
        within reset_ack_timeout (the new world is running; reason names the
        silent nodes).
        """
        scenario_id = req.scenario_id or self.scenario_id
        try:
            # Load and validate first: a failed reset must leave the old run intact.
            sim = HubSim(load_scenario(self.dir, scenario_id), req.seed, self.input_timeout)
        except (OSError, ValueError, yaml.YAMLError) as exc:
            res.success, res.run_id, res.reason = False, '', 'RESET_FAILED: %s' % exc
            return res
        # The new world starts with every body at rest; the stale command
        # timeout keeps it there until fresh commands arrive.
        run_id = self._install(sim, scenario_id)
        now = time.monotonic()
        self.pending = {'run_id': run_id, 'acked': set(), 'done': Future(),
                        'deadline': now + self.ack_timeout, 'resend_at': now + RESEND_SEC}
        if self.participants:
            self._request_reset()
        else:
            self.pending['done'].set_result(True)
        await self.pending['done']           # tick() resends and enforces the timeout
        missing = sorted(self.participants - self.pending['acked'])
        self.pending = None
        res.run_id, res.success = run_id, not missing
        if missing:
            res.reason = 'MISSING_ACK: %s' % ','.join(missing)
        else:
            res.reason = 'RESET preserve_logs=%s acks=%d' % (req.preserve_logs,
                                                             len(self.participants))
        self._event('SCENARIO_RESET', 'scenario_runner', res.reason)
        return res

    # --------------------------------------------------------------- tick
    def tick(self):
        self._drive_handshake(time.monotonic())
        for event in self.sim.step():
            self._event('SCENARIO_EVENT', 'scenario_runner', json.dumps(event, sort_keys=True))
        for pid, t in self.sim.pop_boardings():
            self._event('PASSENGER_BOARDED', pid, 't=%.2f' % t)
        stamp = self.get_clock().now().to_msg()
        step = self.sim.steps
        if not self.sim.is_frozen('odom'):
            self._publish_odom(stamp)
        if step % 2 == 0:
            self.truth_pub.publish(self._truth(stamp))
            if not self.sim.is_frozen('vehicles'):
                self.vehicles_pub.publish(self._vehicles(stamp, noisy=True))
            if self.publish_estop:
                self.estop_pub.publish(Bool(data=self.sim.estop))
            self._publish_camera(stamp)
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
            if body.kind in ('bus', 'car'):        # roads only (never the crossing strip)
                v.edge_id = geo.nearest_edge(x, y, edges=geo.VEHICLE_EDGES)
            else:
                v.edge_id = geo.nearest_edge(x, y) if body.kind != 'obstacle' else ''
            v.valid = True
            msg.vehicles.append(v)
        return msg

    def _truth(self, stamp):
        """Ground truth: every body plus pedestrians (kind 'pedestrian')."""
        msg = self._vehicles(stamp, noisy=False)
        for pid, ped in sorted(self.sim.pedestrians.items()):
            v = VehicleState(vehicle_id=pid, kind='pedestrian', valid=True)
            v.pose.x, v.pose.y = float(ped.x), float(ped.y)
            v.speed_mps = float(math.hypot(ped.vx, ped.vy))
            msg.vehicles.append(v)
        return msg

    def _publish_camera(self, stamp):
        if self.camera is None or self.sim.is_frozen('camera'):
            return
        if self.sim.is_frozen('camera_repeat') and self.last_frame is not None:
            self.camera_pub.publish(self.last_frame)    # same pixels, same old stamp
            return
        img = self.camera.render(self.sim.camera_people(), CAMERA_FLOOR)
        msg = Image()
        msg.header.stamp, msg.header.frame_id = stamp, 'camera'
        msg.height, msg.width = img.shape[0], img.shape[1]
        msg.encoding, msg.is_bigendian, msg.step = 'bgr8', 0, img.shape[1] * 3
        msg.data = img.tobytes()
        self.last_frame = msg
        self.camera_pub.publish(msg)

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
