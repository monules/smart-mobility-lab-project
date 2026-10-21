"""traffic_monitor (M3) - Scenario S5: congestion detection.

Interfaces (assignment sections 5, 12):
  subscribes /hub/vehicles       VehicleStateArray (10 Hz, R10)
  publishes  /traffic/edges      EdgeStateArray    (2 Hz, R10)
  publishes  /hub/events         HubEvent          (label transitions, R100)

ADDITIVE output (EdgeState has no label field, but the dashboard must show
NORMAL / CONGESTED / BLOCKED / UNKNOWN per edge):
  publishes  /traffic/edge_labels std_msgs/String, JSON, 2 Hz

Invalid-value convention (document in the README): when ``valid`` is false,
``mean_speed_mps`` and ``density_ratio`` are -1.0, never 0.  The header stamp is
the SOURCE time of the newest fresh vehicle snapshot, not the publication time,
so a consumer can age-check the data.

Obstacles arrive as ``VehicleState`` entries with kind == 'obstacle' in the same
/hub/vehicles stream (assumption: confirm with M5, who owns the simulator).
"""
import json
import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from smart_hub_interfaces.msg import (EdgeState, EdgeStateArray, HubEvent,
                                      VehicleStateArray)
from std_msgs.msg import String

from .traffic_engine import TrafficConfig, TrafficEngine, VehicleObs


def _qos(depth):
    return QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.VOLATILE,
                      history=HistoryPolicy.KEEP_LAST, depth=depth)


R10, R100 = _qos(10), _qos(100)


class TrafficMonitor(Node):
    def __init__(self, **node_kwargs):
        super().__init__('traffic_monitor', **node_kwargs)
        p = self._param
        self.run_id = p('run_id', 'run0')
        self.scenario_id = p('scenario_id', 'S5')
        publish_hz = float(p('publish_hz', 2.0))
        if not 0.5 <= publish_hz <= 10.0:
            raise ValueError('publish_hz must be in [0.5, 10]')
        cfg = TrafficConfig(
            window_sec=float(p('window_sec', 5.0)),
            stale_sec=float(p('stale_sec', 1.0)),
            presence_hold_sec=float(p('presence_hold_sec', 1.0)),
            max_source_age_sec=float(p('max_source_age_sec', 1.0)),
            corridor_halfwidth_m=float(p('corridor_halfwidth_m', 0.5)),
            safe_headway_m=float(p('safe_headway_m', 1.0)),
            free_flow_mps=float(p('free_flow_speed_mps', 0.40)),
            density_threshold=float(p('congestion_density_threshold', 0.70)),
            slow_fraction=float(p('slow_fraction', 0.40)),
            slow_ticks=int(p('slow_ticks', 3)),
            clear_ticks=int(p('clear_ticks', 2)),
            min_samples=int(p('min_samples', 3)),
            obstacle_radius_m=float(p('obstacle_radius_m', 0.35)),
            obstacle_clear_sec=float(p('obstacle_clear_sec', 2.0)))
        cfg.validate()                       # reject invalid ranges at startup
        self.engine = TrafficEngine(cfg)
        self._seq = 0
        self._published_stale = False

        group = ReentrantCallbackGroup()
        self.create_subscription(VehicleStateArray, '/hub/vehicles', self.on_vehicles,
                                 R10, callback_group=group)
        self.edge_pub = self.create_publisher(EdgeStateArray, '/traffic/edges', R10)
        self.label_pub = self.create_publisher(String, '/traffic/edge_labels', R10)
        self.event_pub = self.create_publisher(HubEvent, '/hub/events', R100)
        self.create_timer(1.0 / publish_hz, self.on_publish_timer, callback_group=group)
        # 10 Hz watchdog: publish UNKNOWN as soon as the feed goes stale (> 1.2 s
        # gap) instead of waiting for the next 2 Hz slot.
        self.create_timer(0.1, self.on_watchdog, callback_group=group)
        self.get_logger().info('traffic_monitor up: edges=%s' % list(cfg.edges))

    def _param(self, name, default):
        self.declare_parameter(name, default)
        return self.get_parameter(name).value

    def on_vehicles(self, msg):
        now = time.monotonic()
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        age = self.get_clock().now().nanoseconds * 1e-9 - stamp
        obs = [VehicleObs(v.vehicle_id, v.kind, v.pose.x, v.pose.y,
                          v.speed_mps, v.edge_id, bool(v.valid))
               for v in msg.vehicles]
        rep = self.engine.ingest(stamp, now, obs, source_age_s=age)
        if rep.snapshot_reason:
            self.get_logger().warning('vehicle snapshot not fresh: %s'
                                      % rep.snapshot_reason, throttle_duration_sec=2.0)
        for vid, reason in rep.rejected:
            self.get_logger().warning('rejected sample %s: %s' % (vid, reason),
                                      throttle_duration_sec=2.0)

    def on_watchdog(self):
        stale = self.engine.is_stale(time.monotonic())
        if stale and not self._published_stale:
            self.publish_now()
        elif not stale:
            self._published_stale = False

    def on_publish_timer(self):
        self.publish_now()

    def publish_now(self):
        results = self.engine.evaluate(time.monotonic())
        self._published_stale = all(r.label == 'UNKNOWN' for r in results)
        now_msg = self.get_clock().now().to_msg()
        src = self.engine.last_observation_stamp()
        out = EdgeStateArray()
        out.header.frame_id = 'map'
        if src is None:
            out.header.stamp = now_msg
        else:
            out.header.stamp.sec = int(src)
            out.header.stamp.nanosec = int((src - int(src)) * 1e9)
        edges = []
        labels = {}
        for r in results:
            e = EdgeState()
            e.edge_id = r.edge_id
            e.mean_speed_mps = float(r.mean_speed_mps)
            e.density_ratio = float(r.density_ratio)
            e.blocked = bool(r.blocked)
            e.valid = bool(r.valid)
            edges.append(e)
            labels[r.edge_id] = {'label': r.label, 'count': r.count,
                                 'capacity': r.capacity,
                                 'obs_age_sec': round(r.obs_age_sec, 3)
                                 if r.obs_age_sec != float('inf') else None}
            if r.changed:
                self._seq += 1
                ev = HubEvent()
                ev.header.stamp = now_msg
                ev.header.frame_id = 'map'
                ev.run_id, ev.scenario_id = self.run_id, self.scenario_id
                ev.entity_id, ev.event = r.edge_id, r.label
                ev.reason = '%s->%s rho=%.2f v=%.2f n=%d' % (
                    r.prev_label, r.label, r.density_ratio, r.mean_speed_mps, r.count)
                ev.correlation_id = '%s:%d' % (r.edge_id, self._seq)
                self.event_pub.publish(ev)
        out.edges = edges
        self.edge_pub.publish(out)
        text = String()
        text.data = json.dumps({'stamp': out.header.stamp.sec, 'edges': labels})
        self.label_pub.publish(text)


def main(args=None):
    rclpy.init(args=args)
    node = TrafficMonitor()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass                      # Ctrl+C / launch shutdown: not an error
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():            # the signal handler may have shut down already
            rclpy.shutdown()


if __name__ == '__main__':
    main()
