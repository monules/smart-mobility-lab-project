"""passenger_detector (M1-A): camera -> passenger queue count and crossing occupancy.

Subscribes  /camera/image_raw  sensor_msgs/Image (bgr8/rgb8/mono8)  sensor QoS, 10 Hz
Publishes   /hub/queue         smart_hub_interfaces/QueueState      R10, 5 Hz
            /hub/events        HubEvent                             R100, transitions

Pipeline per frame (assignment 8.2): detect people -> foot point of each box
-> homography to the floor (metres) -> track ids over time -> count
confirmed tracks inside queue Q, flag crossing C. The 5 Hz timer publishes
the latest state; header.stamp is the observation time of the frame used,
or the publication time while invalid (stale data never looks fresh).

All decisions are in smart_hub.perception (pure Python, unit-tested).
"""
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Image
from smart_hub.perception.detectors import foot, make_detector
from smart_hub.perception.homography import fit_homography, image_to_map
from smart_hub.perception.queue_logic import QueueLogic
from smart_hub.perception.tracker import Tracker
from smart_hub.ros_common import declare, R10, R100
from smart_hub_interfaces.msg import HubEvent, QueueState

# Default calibration: the four painted corners of queue Q as they appear in
# the simulator camera image (pixels), and their map coordinates (metres).
# For a real camera, click the same four floor points in one frame.
DEFAULT_IMAGE_POINTS = [187.9, 205.4, 428.1, 205.4, 416.6, 131.8, 201.9, 131.8]
DEFAULT_MAP_POINTS = [2.0, 5.0, 6.0, 5.0, 6.0, 7.0, 2.0, 7.0]


def image_to_bgr(msg):
    """sensor_msgs/Image -> HxWx3 uint8 BGR array (no cv_bridge needed)."""
    channels = {'bgr8': 3, 'rgb8': 3, 'mono8': 1}.get(msg.encoding)
    if channels is None:
        raise ValueError('unsupported encoding %r' % msg.encoding)
    raw = np.frombuffer(bytes(msg.data), np.uint8)
    img = raw.reshape(msg.height, msg.step)[:, :msg.width * channels]
    img = img.reshape(msg.height, msg.width, channels)
    if msg.encoding == 'rgb8':
        return img[:, :, ::-1]
    if msg.encoding == 'mono8':
        return np.repeat(img, 3, axis=2)
    return img


class PassengerDetector(Node):
    def __init__(self, **kwargs):
        super().__init__('passenger_detector', **kwargs)
        p = declare(self, {
            'detector': 'blob', 'hog_threshold': 0.5, 'hog_upscale': 1.5,
            'hog_foot_offset': 0.15, 'blob_max_gray': 110, 'blob_min_area': 150,
            'image_points': DEFAULT_IMAGE_POINTS, 'map_points': DEFAULT_MAP_POINTS,
            'stale_timeout_sec': 1.0, 'confirm_sec': 0.4, 'hold_sec': 0.4,
            'occlusion_keep_sec': 0.5, 'match_distance_m': 0.6,
            'crossing_clear_sec': 2.0, 'publish_rate_hz': 5.0, 'zone_id': 'Q',
            'run_id': '', 'scenario_id': ''})
        self._validate(p)
        self.detector = make_detector(
            p['detector'], threshold=p['hog_threshold'], upscale=p['hog_upscale'],
            foot_offset=p['hog_foot_offset'], max_gray=p['blob_max_gray'],
            min_area=p['blob_min_area'])
        self.h = fit_homography(p['image_points'], p['map_points'])
        tracker = Tracker(p['match_distance_m'], p['confirm_sec'], p['hold_sec'],
                          p['occlusion_keep_sec'])
        self.logic = QueueLogic(p['stale_timeout_sec'], p['crossing_clear_sec'], tracker)
        self.zone_id, self.run_id, self.scenario_id = p['zone_id'], p['run_id'], p['scenario_id']
        self.rejected = 0
        self.pub = self.create_publisher(QueueState, '/hub/queue', R10)
        self.event_pub = self.create_publisher(HubEvent, '/hub/events', R100)
        self.create_subscription(Image, '/camera/image_raw', self.on_image,
                                 qos_profile_sensor_data)
        self.create_timer(1.0 / p['publish_rate_hz'], self.publish)
        self.get_logger().info('passenger_detector ready: detector=%s' % self.detector.name)

    @staticmethod
    def _validate(p):
        bad = []
        if p['detector'] not in ('blob', 'hog'):
            bad.append('detector')
        if len(p['image_points']) != len(p['map_points']) or len(p['image_points']) < 8:
            bad.append('image_points/map_points (>= 4 pairs)')
        for name in ('stale_timeout_sec', 'confirm_sec', 'hold_sec', 'occlusion_keep_sec',
                     'match_distance_m', 'crossing_clear_sec', 'publish_rate_hz'):
            if not p[name] > 0:
                bad.append(name)
        if p['occlusion_keep_sec'] < p['hold_sec']:
            bad.append('occlusion_keep_sec must be >= hold_sec')
        if bad:
            raise ValueError('invalid parameter(s): ' + ', '.join(bad))

    def on_image(self, msg):
        now = time.monotonic()
        try:
            img = image_to_bgr(msg)
        except ValueError as exc:
            self.rejected += 1
            self.get_logger().warning(str(exc), throttle_duration_sec=5.0)
            return
        people = []
        for det in self.detector.detect(img):
            u, v = foot(det, self.detector.foot_offset)
            try:
                x, y = image_to_map(self.h, u, v)
            except ValueError:
                continue                         # above the horizon: not on the floor
            people.append((x, y, det[4]))
        stamp = Time.from_msg(msg.header.stamp).nanoseconds / 1e9
        self.logic.on_frame(people, stamp, now)

    def publish(self):
        s = self.logic.state(time.monotonic())
        msg = QueueState()
        if s['valid']:
            msg.header.stamp = Time(nanoseconds=int(s['stamp'] * 1e9)).to_msg()
        else:
            msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.zone_id, msg.count = self.zone_id, s['count']
        msg.confidence, msg.valid = float(s['confidence']), s['valid']
        msg.crossing_occupied = s['crossing_occupied']
        self.pub.publish(msg)
        for event, detail in self.logic.events + self.logic.tracker.events:
            ev = HubEvent()
            ev.header.stamp = self.get_clock().now().to_msg()
            ev.run_id, ev.scenario_id = self.run_id, self.scenario_id
            ev.entity_id, ev.event = 'passenger_detector', event
            ev.reason = str(detail)
            ev.correlation_id = ('track-%s' % detail) if event.startswith('TRACK') else 'queue'
            self.event_pub.publish(ev)
        self.logic.events.clear()
        self.logic.tracker.events.clear()


def main(args=None):
    rclpy.init(args=args)
    node = PassengerDetector()
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
