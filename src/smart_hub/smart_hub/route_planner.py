"""route_planner (M4): S6 congestion-aware routing, /route/compute service.

Serves      /route/compute       ComputeRoute  (bus_manager, robot_manager, s6_check)
Subscribes  /traffic/edges       EdgeStateArray R10 (traffic_monitor, M3)
            /hub/vehicles        VehicleStateArray R10 (obstacle positions)
            /crossing/state      CrossingState R1T (hub_coordinator, M2-B)
Publishes   /route/edge_costs    std_msgs/String JSON, 1 Hz: cost per edge and
                                 agent type, null = removed with the reason
            /hub/events          ROUTE_COMPUTED (JSON: request, path, cost,
                                 latency, every edge cost used), R100

All rules are in route_core.py (cost formula 13.2, removal of blocked,
forbidden and stale edges, Dijkstra). A route computation is a short
service; executing the route is the managers' job (13.1).
"""
import json
import time

from geometry_msgs.msg import Point
import rclpy
from rclpy.node import Node
from smart_hub import geometry as geo
from smart_hub import route_core
from smart_hub.ros_common import declare, R10, R100, R1T
from smart_hub_interfaces.msg import (CrossingState, EdgeStateArray, HubEvent,
                                      VehicleStateArray)
from smart_hub_interfaces.srv import ComputeRoute
from std_msgs.msg import String


class RoutePlanner(Node):
    def __init__(self, **kwargs):
        super().__init__('route_planner', **kwargs)
        p = declare(self, {'traffic_timeout_sec': 1.5, 'bodies_timeout_sec': 1.0,
                           'crossing_timeout_sec': 1.0, 'crossing_wait_sec': 5.0,
                           'obstacle_margin_m': 0.05, 'require_traffic': True})
        for name in ('traffic_timeout_sec', 'bodies_timeout_sec', 'crossing_timeout_sec'):
            if not 0.1 <= p[name] <= 10.0:
                raise ValueError('%s must be in [0.1, 10]' % name)
        if not 0.0 <= p['crossing_wait_sec'] <= 120.0 or not 0.0 <= p['obstacle_margin_m'] <= 1.0:
            raise ValueError('crossing_wait_sec in [0, 120], obstacle_margin_m in [0, 1]')
        self.view = route_core.EdgeView(
            traffic_timeout=p['traffic_timeout_sec'], bodies_timeout=p['bodies_timeout_sec'],
            crossing_timeout=p['crossing_timeout_sec'], crossing_wait=p['crossing_wait_sec'],
            obstacle_margin=p['obstacle_margin_m'], require_traffic=bool(p['require_traffic']))
        self.create_subscription(EdgeStateArray, '/traffic/edges', self.on_edges, R10)
        self.create_subscription(VehicleStateArray, '/hub/vehicles', self.on_vehicles, R10)
        self.create_subscription(CrossingState, '/crossing/state', self.on_crossing, R1T)
        self.event_pub = self.create_publisher(HubEvent, '/hub/events', R100)
        self.cost_pub = self.create_publisher(String, '/route/edge_costs', R10)
        self.create_service(ComputeRoute, '/route/compute', self.on_compute)
        self.create_timer(1.0, self.publish_costs)
        self.get_logger().info('route_planner up')

    # ------------------------------------------------------------- inputs
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

    # ------------------------------------------------------------- service
    def on_compute(self, req, res):
        t0 = time.perf_counter()
        now = time.monotonic()
        agent = req.agent_type
        start, goal = req.start_node, req.goal_node
        costs = {}
        if agent not in route_core.ALLOWED:
            res.success, res.reason = False, 'INVALID_AGENT'
        elif start not in geo.NODES or goal not in geo.NODES:
            res.success, res.reason = False, 'INVALID_NODE'
        else:
            agent_id = {'robot': 'robot_1', 'bus': 'bus_1'}.get(agent, '')
            costs = self.view.costs(agent, now, req.excluded_edges, agent_id)
            nodes, cost = route_core.plan(costs, start, goal)
            if nodes is None:
                res.success, res.reason = False, 'NO_ROUTE'
            else:
                res.success, res.reason = True, 'OK'
                res.node_ids = nodes
                res.waypoints = [Point(x=float(geo.NODES[n][0]), y=float(geo.NODES[n][1]))
                                 for n in nodes]
                res.estimated_cost_sec = float(cost)
        latency_ms = (time.perf_counter() - t0) * 1000.0
        self.event('ROUTE_COMPUTED', agent, json.dumps({
            'start': start, 'goal': goal, 'agent': agent, 'success': res.success,
            'reason': res.reason, 'nodes': list(res.node_ids),
            'cost': round(res.estimated_cost_sec, 6), 'latency_ms': round(latency_ms, 3),
            'excluded': list(req.excluded_edges),
            'costs': {e: (round(c, 6) if c is not None else None) for e, (c, _) in costs.items()},
            'removed': {e: why for e, (c, why) in costs.items()
                        if c is None and why != 'FORBIDDEN'}}, sort_keys=True))
        return res

    # ------------------------------------------------------------- outputs
    def publish_costs(self):
        now = time.monotonic()
        out = {}
        for agent in ('car', 'bus', 'robot'):
            costs = self.view.costs(agent, now)
            out[agent] = {e: (round(c, 3) if c is not None else why)
                          for e, (c, why) in costs.items() if why != 'FORBIDDEN'}
        self.cost_pub.publish(String(data=json.dumps(out, sort_keys=True)))

    def event(self, event, entity, reason):
        msg = HubEvent()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.entity_id, msg.event, msg.reason = entity, event, reason
        self.event_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = RoutePlanner()
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
