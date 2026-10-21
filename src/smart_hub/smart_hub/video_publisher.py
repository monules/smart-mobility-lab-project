"""video_publisher (M1-A tool): play a recorded video as /camera/image_raw.

For the camera/video detection demonstration (assignment 8.2 step 1, 8.3:
at least 100 annotated video frames). Publishes bgr8 frames at the file's
frame rate with the current time as stamp, using the sensor-data QoS like a
real camera. Use it instead of scenario_runner's synthetic camera, and run
passenger_detector with detector:=hog and the video's own 4-point
calibration (image_points / map_points).
"""
import time

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from smart_hub.ros_common import declare


class VideoPublisher(Node):
    def __init__(self, **kwargs):
        super().__init__('video_publisher', **kwargs)
        p = declare(self, {'path': '', 'loop': False, 'rate_hz': 0.0})
        self.cap = cv2.VideoCapture(p['path'])
        if not p['path'] or not self.cap.isOpened():
            raise ValueError('cannot open video %r' % p['path'])
        fps = p['rate_hz'] or self.cap.get(cv2.CAP_PROP_FPS) or 10.0
        self.loop, self.frames = bool(p['loop']), 0
        self.pub = self.create_publisher(Image, '/camera/image_raw', qos_profile_sensor_data)
        self.create_timer(1.0 / fps, self.tick)
        self.t0 = time.monotonic()
        self.get_logger().info('playing %s at %.1f fps' % (p['path'], fps))

    def tick(self):
        ok, frame = self.cap.read()
        if not ok and self.loop:
            self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            ok, frame = self.cap.read()
        if not ok:
            # End of file: stop publishing, so the detector reports invalid.
            return
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera'
        msg.height, msg.width = frame.shape[:2]
        msg.encoding, msg.step = 'bgr8', frame.shape[1] * 3
        msg.data = frame.tobytes()
        self.pub.publish(msg)
        self.frames += 1


def main(args=None):
    rclpy.init(args=args)
    node = VideoPublisher()
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
