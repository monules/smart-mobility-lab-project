"""Node-level tests of passenger_detector (real rclpy, no other processes)."""
import importlib.util
import unittest

HAVE_ROS = (importlib.util.find_spec('rclpy') is not None
            and importlib.util.find_spec('smart_hub_interfaces') is not None
            and importlib.util.find_spec('cv2') is not None)


@unittest.skipUnless(HAVE_ROS, 'ROS 2, smart_hub_interfaces and OpenCV needed')
class TestPassengerDetectorNode(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        import rclpy
        cls.ctx = rclpy.Context()
        rclpy.init(context=cls.ctx, domain_id=92)

    @classmethod
    def tearDownClass(cls):
        import rclpy
        rclpy.shutdown(context=cls.ctx)

    def image(self, encoding, people=((3.3, 6.6), (4.2, 6.6))):
        import numpy as np
        from sensor_msgs.msg import Image
        from smart_hub.camera_model import PinholeCamera
        bgr = PinholeCamera().render(list(people))
        data = {'bgr8': bgr, 'rgb8': bgr[:, :, ::-1],
                'mono8': bgr[:, :, :1]}[encoding]
        msg = Image(height=bgr.shape[0], width=bgr.shape[1], encoding=encoding)
        msg.step = bgr.shape[1] * data.shape[2]
        msg.data = np.ascontiguousarray(data).tobytes()
        return msg

    def test_image_conversion_all_encodings(self):
        from smart_hub.passenger_detector import image_to_bgr
        for enc in ('bgr8', 'rgb8', 'mono8'):
            img = image_to_bgr(self.image(enc))
            self.assertEqual(img.shape, (480, 640, 3))
        with self.assertRaises(ValueError):
            msg = self.image('bgr8')
            msg.encoding = 'yuv422'
            image_to_bgr(msg)

    def test_frames_become_queue_count(self):
        import smart_hub.passenger_detector as pd
        node = pd.PassengerDetector(context=self.ctx)
        clock = [1000.0]
        orig = pd.time.monotonic
        pd.time.monotonic = lambda: clock[0]          # fake receipt clock, no sleeps
        try:
            for k in range(8):                       # 0.7 s of frames > 0.4 s debounce
                clock[0] = 1000.0 + 0.1 * k
                msg = self.image('bgr8')
                msg.header.stamp.sec, msg.header.stamp.nanosec = 100, k * 100_000_000
                node.on_image(msg)
            state = node.logic.state(clock[0])
            self.assertTrue(state['valid'])
            self.assertEqual(state['count'], 2)
            self.assertFalse(state['crossing_occupied'])
        finally:
            pd.time.monotonic = orig
            node.destroy_node()

    def test_parameters_validated(self):
        from rclpy.parameter import Parameter
        from smart_hub.passenger_detector import PassengerDetector
        for bad in (Parameter('detector', value='yolo'),
                    Parameter('image_points', value=[1.0, 2.0, 3.0, 4.0]),
                    Parameter('occlusion_keep_sec', value=0.1)):
            with self.assertRaises(ValueError):
                PassengerDetector(context=self.ctx, parameter_overrides=[bad]).destroy_node()

    def test_scenario_reset_clears_tracks_once_per_run(self):
        from smart_hub.passenger_detector import PassengerDetector
        from smart_hub.ros_common import RESET_REQUEST
        from smart_hub_interfaces.msg import HubEvent
        node = PassengerDetector(context=self.ctx)
        try:
            node.logic.on_frame([(3.0, 6.0, 1.0)], 1.0, 1.0)
            node.logic.on_frame([(3.0, 6.0, 1.0)], 1.5, 1.5)
            before = node.logic
            node.on_hub_event(HubEvent(event=RESET_REQUEST, correlation_id='S1-s1-r2'))
            after = node.logic
            self.assertIsNot(before, after)
            self.assertEqual(after.state(1.6)['reason'], 'NO_IMAGE')   # unknown, not zero
            node.on_hub_event(HubEvent(event=RESET_REQUEST, correlation_id='S1-s1-r2'))
            self.assertIs(node.logic, after)                            # resend: no 2nd reset
        finally:
            node.destroy_node()
