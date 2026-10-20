"""fake_queue_publisher: replay a seeded synthetic episode on /hub/queue.

Stand-in for passenger_detector (M1-A) so demand_predictor can be developed
and demonstrated before S1 exists. It publishes QueueState at 5 Hz exactly as
the simulator's detector model produced it, including count flicker and the
episode's outages (valid=false, silent, or frozen source stamp).

    ros2 run smart_hub fake_queue_publisher --ros-args -p seed:=1025

This is test tooling: it is clearly labelled synthetic and must not be
launched together with the real passenger_detector.
"""

import rclpy
from builtin_interfaces.msg import Time as TimeMsg
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)

from smart_hub_interfaces.msg import QueueState

from smart_hub.demand.simulation import PROFILES, make_spec, run_episode


class FakeQueuePublisher(Node):
    def __init__(self):
        super().__init__("fake_queue_publisher")
        self.declare_parameter("seed", 1025)
        self.declare_parameter("profile", "")       # empty = from seed
        self.declare_parameter("loop", False)
        self.declare_parameter("topic", "/hub/queue")
        seed = int(self.get_parameter("seed").value)
        profile = self.get_parameter("profile").value or None
        if profile is not None and profile not in PROFILES:
            raise ValueError("profile must be one of %s" % (PROFILES,))
        self.loop = bool(self.get_parameter("loop").value)
        spec = make_spec(seed, profile)
        spec.bus_mode = "scheduled"
        self.obs = run_episode(spec).observations
        self.idx = 0
        self.t0 = self._now()
        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE,
                         history=HistoryPolicy.KEEP_LAST, depth=10)
        self.pub = self.create_publisher(
            QueueState, self.get_parameter("topic").value, qos)
        self.create_timer(0.02, self.tick)
        self.get_logger().info(
            "SYNTHETIC replay: seed=%d profile=%s gaps=%s (%d messages)"
            % (seed, spec.profile, spec.gaps, len(self.obs)))

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def tick(self):
        elapsed = self._now() - self.t0
        while self.idx < len(self.obs) and self.obs[self.idx][0] <= elapsed:
            t, count, valid, conf, stamp_offset = self.obs[self.idx]
            stamp = self.t0 + t + stamp_offset
            msg = QueueState()
            msg.header.stamp = TimeMsg(sec=int(stamp),
                                       nanosec=int((stamp % 1.0) * 1e9))
            msg.header.frame_id = "map"
            msg.zone_id = "Q"
            msg.count = int(count)
            msg.confidence = float(conf)
            msg.valid = bool(valid)
            msg.crossing_occupied = False
            self.pub.publish(msg)
            self.idx += 1
        if self.idx >= len(self.obs):
            if self.loop:
                self.idx = 0
                self.t0 = self._now()
            else:
                self.get_logger().info("episode finished", once=True)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = FakeQueuePublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
