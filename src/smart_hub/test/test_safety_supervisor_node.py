"""Node-level tests of safety_supervisor's message handling (real rclpy)."""
import importlib.util
import unittest

HAVE_ROS = (importlib.util.find_spec('rclpy') is not None
            and importlib.util.find_spec('smart_hub_interfaces') is not None)


@unittest.skipUnless(HAVE_ROS, 'ROS 2 + smart_hub_interfaces needed')
class TestSafetySupervisorNode(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        import rclpy
        cls.ctx = rclpy.Context()
        rclpy.init(context=cls.ctx, domain_id=90)

    @classmethod
    def tearDownClass(cls):
        import rclpy
        rclpy.shutdown(context=cls.ctx)

    def setUp(self):
        from smart_hub.safety_supervisor import SafetySupervisor
        self.node = SafetySupervisor(context=self.ctx)

    def tearDown(self):
        self.node.destroy_node()

    def candidate(self, stamp_offset_sec=None):
        from geometry_msgs.msg import TwistStamped
        from rclpy.duration import Duration
        msg = TwistStamped()
        if stamp_offset_sec is not None:
            t = self.node.get_clock().now() + Duration(seconds=stamp_offset_sec)
            msg.header.stamp = t.to_msg()
        msg.twist.linear.x = 0.2
        return msg

    def test_unstamped_candidate_is_never_fresh(self):
        self.node.on_candidate('robot', self.candidate(None))
        self.assertIsNone(self.node.core.agents['robot'].command)
        self.assertGreater(self.node.rejected, 0)

    def test_future_stamp_rejected_and_valid_stamp_accepted(self):
        self.node.on_candidate('robot', self.candidate(+1.0))
        self.assertIsNone(self.node.core.agents['robot'].command)
        self.node.on_candidate('robot', self.candidate(-0.01))
        self.assertEqual(self.node.core.agents['robot'].command.value, (0.2, 0.0))

    def test_estop_latches_immediately(self):
        from std_msgs.msg import Bool
        self.node.on_estop(Bool(data=True))
        self.assertTrue(self.node.core.latched)
        self.node.on_estop(Bool(data=False))
        self.assertTrue(self.node.core.latched)        # false never clears

    def test_parameters_validated(self):
        from rclpy.parameter import Parameter
        from smart_hub.safety_supervisor import SafetySupervisor
        with self.assertRaises(ValueError):
            SafetySupervisor(context=self.ctx, parameter_overrides=[
                Parameter('robot_speed_limit', value=-1.0)]).destroy_node()
