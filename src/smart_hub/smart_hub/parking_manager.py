"""parking_manager (M3) - Scenario S4: parking availability and reservation.

Interfaces (assignment sections 5, 7, 11):
  serves     /parking/reserve_slot   smart_hub_interfaces/srv/ReserveParking
  serves     /parking/release_slot   smart_hub_interfaces/srv/ReleaseParking
  subscribes /parking/observed       SlotObservationArray   (5 Hz, R10)
  publishes  /parking/state          ParkingState           (2 Hz + on change, R1T)
  publishes  /parking/barrier_request std_msgs/Bool          (R1; true = open)
  subscribes /parking/barrier_open   std_msgs/Bool           (R1; measured status)
  publishes  /hub/events             HubEvent                (on change, R100)

ADDITIVE inputs for the barrier interlock (not in the assignment's topic table;
names are parameters, confirm them with M5 / the actuator bridge):
  subscribes /parking/entry_claim     std_msgs/String  "vehicle_id|reservation_id"
  subscribes /parking/entrance_clear  std_msgs/Bool    true = passage clear

All slot logic lives in parking_engine.py; this node only adapts it to ROS:
header-stamp freshness gating, QoS, services, timers and event publishing.
"""
import threading
import time

import rclpy
from builtin_interfaces.msg import Time
from rclpy.callback_groups import (MutuallyExclusiveCallbackGroup,
                                   ReentrantCallbackGroup)
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from smart_hub_interfaces.msg import (HubEvent, ParkingSlot, ParkingState,
                                      SlotObservationArray)
from smart_hub_interfaces.srv import ReleaseParking, ReserveParking
from std_msgs.msg import Bool, String

from .parking_engine import (BarrierController, ParkingConfig, ParkingEngine,
                             TTL_MAX_SEC, TTL_MIN_SEC)


def _qos(depth, durable=False):
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=(DurabilityPolicy.TRANSIENT_LOCAL if durable
                    else DurabilityPolicy.VOLATILE),
        history=HistoryPolicy.KEEP_LAST, depth=depth)


R1, R10, R100, R1T = _qos(1), _qos(10), _qos(100), _qos(1, durable=True)


class ParkingManager(Node):
    def __init__(self, **node_kwargs):
        super().__init__('parking_manager', **node_kwargs)
        p = self._param
        self.run_id = p('run_id', 'run0')
        self.scenario_id = p('scenario_id', 'S4')
        slot_ids = tuple(p('slot_ids', ['P1', 'P2', 'P3']))
        self.default_ttl = float(p('reservation_ttl_sec', 30.0))
        self.future_tol = float(p('source_future_tolerance_sec', 0.5))
        state_hz = float(p('state_publish_hz', 2.0))
        freshness = float(p('obs_freshness_sec', 1.0))
        cfg = ParkingConfig(
            slot_ids=slot_ids, freshness_sec=freshness,
            debounce_sec=float(p('debounce_sec', 0.5)),
            min_confidence=float(p('min_confidence', 0.5)),
            strict_preference=bool(p('strict_preference', False)))
        # Reject invalid ranges at startup (assignment section 17).
        if not TTL_MIN_SEC <= self.default_ttl <= TTL_MAX_SEC:
            raise ValueError('reservation_ttl_sec must be in [5, 120]')
        if not 0.0 <= self.future_tol <= 5.0:
            raise ValueError('source_future_tolerance_sec must be in [0, 5]')
        if not 0.1 <= state_hz <= 20.0:
            raise ValueError('state_publish_hz must be in [0.1, 20]')
        self.state_period = 1.0 / state_hz
        cfg.validate()
        self.engine = ParkingEngine(cfg)
        self.barrier = BarrierController(
            self.engine,
            entrance_fresh_sec=float(p('entrance_fresh_sec', 1.0)),
            hold_sec=float(p('barrier_hold_sec', 8.0)),
            close_clear_sec=float(p('barrier_close_clear_sec', 1.0)),
            ack_timeout_sec=float(p('barrier_ack_timeout_sec', 2.0)))

        # Services share ONE mutually-exclusive group (serialised callbacks);
        # the engine lock is the real atomicity guarantee either way.
        svc_group = MutuallyExclusiveCallbackGroup()
        io_group = ReentrantCallbackGroup()
        self.create_service(ReserveParking, '/parking/reserve_slot',
                            self.on_reserve, callback_group=svc_group)
        self.create_service(ReleaseParking, '/parking/release_slot',
                            self.on_release, callback_group=svc_group)
        self.create_subscription(SlotObservationArray, '/parking/observed',
                                 self.on_observed, R10, callback_group=io_group)
        self.create_subscription(Bool, p('barrier_ack_topic', '/parking/barrier_open'),
                                 self.on_ack, R1, callback_group=io_group)
        self.create_subscription(String, p('entry_claim_topic', '/parking/entry_claim'),
                                 self.on_claim, R10, callback_group=io_group)
        self.create_subscription(Bool, p('entrance_clear_topic', '/parking/entrance_clear'),
                                 self.on_entrance, R1, callback_group=io_group)
        self.state_pub = self.create_publisher(ParkingState, '/parking/state', R1T)
        self.barrier_pub = self.create_publisher(Bool, '/parking/barrier_request', R1)
        self.event_pub = self.create_publisher(HubEvent, '/hub/events', R100)

        self._pub_lock = threading.RLock()
        self._last_key = None
        self._last_forced = 0.0
        self.create_timer(0.1, self.on_tick, callback_group=io_group)
        self._publish_barrier(False)
        self._publish_state(force=True)
        self.get_logger().info(
            'parking_manager up: slots=%s ttl_default=%.0fs freshness=%.1fs'
            % (list(slot_ids), self.default_ttl, freshness))

    # ---------------- helpers ----------------
    def _param(self, name, default):
        self.declare_parameter(name, default)
        return self.get_parameter(name).value

    def _ros_now_ns(self):
        return self.get_clock().now().nanoseconds

    def _to_time(self, mono_deadline, mono_now):
        ns = self._ros_now_ns() + int((mono_deadline - mono_now) * 1e9)
        t = Time()
        t.sec, t.nanosec = ns // 1_000_000_000, ns % 1_000_000_000
        return t

    def _flush_events(self):
        for ev in self.engine.drain_events():
            msg = HubEvent()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'map'
            msg.run_id, msg.scenario_id = self.run_id, self.scenario_id
            msg.entity_id, msg.event = ev.entity_id, ev.event
            msg.reason, msg.correlation_id = ev.reason, ev.correlation_id
            self.event_pub.publish(msg)

    def _publish_barrier(self, cmd):
        msg = Bool()
        msg.data = bool(cmd)
        self.barrier_pub.publish(msg)

    def _publish_state(self, force=False):
        now = time.monotonic()
        with self._pub_lock:
            views, available = self.engine.snapshot(now)
            key = tuple((v.slot_id, v.state, v.vehicle_id, v.reservation_id)
                        for v in views)
            if not force and key == self._last_key:
                return
            self._last_key = key
            msg = ParkingState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'map'
            slots = []
            for v in views:
                s = ParkingSlot()
                s.slot_id, s.state = v.slot_id, v.state
                s.vehicle_id, s.reservation_id = v.vehicle_id, v.reservation_id
                if v.reservation_id:
                    s.expires_at = self._to_time(v.expires_mono, now)
                slots.append(s)
            msg.slots = slots
            msg.available_count = int(available)
            self.state_pub.publish(msg)

    # ---------------- services ----------------
    def on_reserve(self, request, response):
        now = time.monotonic()
        r = self.engine.reserve(request.request_id, request.vehicle_id,
                                request.preferred_slot, float(request.ttl_sec), now)
        response.success = r.success
        response.reservation_id = r.reservation_id
        response.slot_id = r.slot_id
        response.reason = r.reason
        if r.success:
            response.expires_at = self._to_time(r.expires_mono, now)
        self._flush_events()
        self._publish_state()
        return response

    def on_release(self, request, response):
        r = self.engine.release(request.request_id, request.reservation_id,
                                request.vehicle_id, time.monotonic())
        response.success = r.success
        response.reason = r.reason
        self._flush_events()
        self._publish_state()
        return response

    # ---------------- subscriptions ----------------
    def on_observed(self, msg):
        now = time.monotonic()
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        age = self._ros_now_ns() * 1e-9 - stamp
        if age > self.engine.cfg.freshness_sec or age < -self.future_tol:
            # Old or future-stamped snapshot: do NOT refresh any slot; slots
            # will age out to UNKNOWN by themselves.
            self.get_logger().warning(
                'dropping /parking/observed snapshot, source age %.2fs' % age,
                throttle_duration_sec=2.0)
            return
        for o in msg.slots:
            self.engine.observe(o.slot_id, o.occupied, o.valid, o.confidence, now)
        self._flush_events()
        self._publish_state()

    def on_claim(self, msg):
        vehicle_id, _, token = msg.data.partition('|')
        ok, reason = self.barrier.on_claim(vehicle_id.strip(), token.strip(),
                                           time.monotonic())
        self.get_logger().info('entry claim %s: %s' % (vehicle_id, reason))
        if ok:
            self._publish_barrier(True)
        self._flush_events()

    def on_entrance(self, msg):
        self.barrier.on_entrance(bool(msg.data), time.monotonic())

    def on_ack(self, msg):
        self.barrier.on_ack(bool(msg.data), time.monotonic())

    # ---------------- 10 Hz housekeeping ----------------
    def on_tick(self):
        now = time.monotonic()
        self.engine.tick(now)                       # TTL expiry, transitions
        cmd = self.barrier.tick(now)
        if cmd is not None:
            self._publish_barrier(cmd)
        self._flush_events()
        forced = now - self._last_forced >= self.state_period
        if forced:
            self._last_forced = now
            self._publish_barrier(self.barrier.desired_open)   # heartbeat
        self._publish_state(force=forced)           # 2 Hz, plus any change


def main(args=None):
    rclpy.init(args=args)
    node = ParkingManager()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
