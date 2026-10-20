"""dashboard_marker_publisher (M5): read-only RViz overlay of the hub.

Subscribes  /hub/vehicles     VehicleStateArray  R10   agents and obstacle O
            /crossing/state   CrossingState      R1T   crossing C colour (M2-B)
            /parking/state    ParkingState       R1T   slot colours (M3)
            /hub/safety       SafetyState        R1T   stop_latched + reason text
            /hub/events       HubEvent           R100  last event text
            /hub/demand       DemandPrediction   R10   forecast text (M1-B)
            /traffic/edges    EdgeStateArray     R10   blocked/unknown/dense edges (M3)
Publishes   /hub/markers      MarkerArray        R10, 5 Hz, frame 'map'

Not drawn: planned routes. No interface carries them (ComputeRoute is a
service between a manager and the planner, not a topic).

No control authority: the node has exactly one publisher (/hub/markers) and
never publishes any cmd_vel, request or service call. Marker content is built
by MarkerCore; this file only converts it to visualization_msgs.
"""
import time

from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point
import rclpy
from rclpy.node import Node
from smart_hub.marker_core import MarkerCore
from smart_hub.ros_common import declare, R10, R100, R1T
from smart_hub_interfaces.msg import (CrossingState, DemandPrediction, EdgeStateArray,
                                      HubEvent, ParkingState, SafetyState,
                                      VehicleStateArray)
from visualization_msgs.msg import Marker, MarkerArray

TYPES = {'cube': Marker.CUBE, 'cylinder': Marker.CYLINDER,
         'line_list': Marker.LINE_LIST, 'text': Marker.TEXT_VIEW_FACING}


class DashboardMarkerPublisher(Node):
    def __init__(self, **kwargs):
        super().__init__('dashboard_marker_publisher', **kwargs)
        p = declare(self, {'rate_hz': 5.0, 'frame_id': 'map', 'lifetime': 1.0,
                           'stale_after': 1.0})
        self.frame_id = p['frame_id']
        # Lifetime > one period: a vanished vehicle's marker disappears by itself.
        self.lifetime = Duration(sec=int(p['lifetime']),
                                 nanosec=int(p['lifetime'] % 1 * 1e9))
        self.core = MarkerCore(float(p['stale_after']))

        self.pub = self.create_publisher(MarkerArray, '/hub/markers', R10)
        self.create_subscription(VehicleStateArray, '/hub/vehicles', self.on_vehicles, R10)
        self.create_subscription(CrossingState, '/crossing/state', self.on_crossing, R1T)
        self.create_subscription(ParkingState, '/parking/state', self.on_parking, R1T)
        self.create_subscription(SafetyState, '/hub/safety', self.on_safety, R1T)
        self.create_subscription(HubEvent, '/hub/events', self.on_event, R100)
        self.create_subscription(DemandPrediction, '/hub/demand', self.on_demand, R10)
        self.create_subscription(EdgeStateArray, '/traffic/edges', self.on_edges, R10)
        self.create_timer(1.0 / float(p['rate_hz']), self.publish)

    def on_vehicles(self, msg):
        self.core.on_vehicles([(v.vehicle_id, v.kind, v.pose.x, v.pose.y, v.valid)
                               for v in msg.vehicles], time.monotonic())

    def on_crossing(self, msg):
        self.core.on_crossing(msg.state, msg.owner_id, msg.valid, time.monotonic())

    def on_parking(self, msg):
        self.core.on_parking({s.slot_id: s.state for s in msg.slots}, time.monotonic())

    def on_safety(self, msg):
        self.core.on_safety(msg.stop_latched, msg.reason, time.monotonic())

    def on_event(self, msg):
        self.core.on_event(msg.entity_id, msg.event, msg.reason)

    def on_demand(self, msg):
        self.core.on_demand(msg.valid, msg.predicted_count, msg.baseline_count,
                            msg.horizon_sec, time.monotonic())

    def on_edges(self, msg):
        self.core.on_edges({e.edge_id: (e.blocked, e.valid, e.density_ratio)
                            for e in msg.edges}, time.monotonic())

    def to_marker(self, spec, stamp):
        m = Marker()
        m.header.frame_id, m.header.stamp = self.frame_id, stamp
        m.ns, m.id, m.type = spec['ns'], spec['id'], TYPES[spec['type']]
        m.action, m.lifetime = Marker.ADD, self.lifetime
        m.pose.position.x, m.pose.position.y, m.pose.position.z = spec['pos']
        m.pose.orientation.w = 1.0
        m.scale.x, m.scale.y, m.scale.z = spec['scale']
        m.color.r, m.color.g, m.color.b, m.color.a = spec['color']
        m.points = [Point(x=float(x), y=float(y)) for x, y in spec.get('points', [])]
        m.text = spec.get('text', '')
        return m

    def publish(self):
        stamp = self.get_clock().now().to_msg()
        specs = self.core.build(time.monotonic())
        self.pub.publish(MarkerArray(markers=[self.to_marker(s, stamp) for s in specs]))


def main(args=None):
    rclpy.init(args=args)
    node = DashboardMarkerPublisher()
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
