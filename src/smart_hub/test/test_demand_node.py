"""Node-level test of demand_predictor with real rclpy.

Runs under `colcon test` (ROS 2 installed); skipped elsewhere. It drives the
node's own callbacks through warm-up -> valid -> stale -> gap recovery ->
valid, which is the path that crashed the node on its first valid forecast
when one logging call site switched between warning and info severity.
"""

import importlib.util
import time
import unittest

HAVE_ROS = (importlib.util.find_spec("rclpy") is not None
            and importlib.util.find_spec("smart_hub_interfaces") is not None)


@unittest.skipUnless(HAVE_ROS, "ROS 2 (rclpy + smart_hub_interfaces) needed")
class TestDemandPredictorNode(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        import rclpy
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        import rclpy
        if rclpy.ok():
            rclpy.shutdown()

    def setUp(self):
        from rclpy.parameter import Parameter
        import smart_hub.demand_predictor as dp
        self.dp = dp
        self.node = dp.DemandPredictor(parameter_overrides=[
            Parameter("warmup_sec", value=31),
            Parameter("run_id", value="unit-test"),
        ])
        # Fake monotonic clock so a 'second' passes per tick without sleeping.
        self.mono = [1000.0]
        self._orig_monotonic = dp.time.monotonic
        dp.time.monotonic = lambda: self.mono[0]

    def tearDown(self):
        self.dp.time.monotonic = self._orig_monotonic
        self.node.destroy_node()

    def second(self, k, send=True, count=4):
        from smart_hub_interfaces.msg import QueueState
        self.mono[0] = 1000.0 + k
        if send:
            msg = QueueState()
            msg.zone_id = "Q"
            msg.count = count
            msg.confidence = 0.9
            msg.valid = True
            time.sleep(0.002)  # strictly newer stamp than the previous one
            msg.header.stamp = self.node.get_clock().now().to_msg()
            self.node.on_queue(msg)
        self.mono[0] = 1000.0 + k + 0.05
        self.node._tick()  # call the unguarded body: any exception fails
        return self.node.last_output

    def test_full_validity_cycle_publishes_every_second(self):
        outs = [self.second(k) for k in range(31)]
        self.assertTrue(all(not o.valid for o in outs[:30]))
        self.assertTrue(all(o.predicted_count == -1.0 for o in outs[:30]))
        self.assertTrue(outs[30].valid, "first valid forecast")
        self.assertGreaterEqual(outs[30].predicted_count, 0.0)
        self.assertEqual(outs[30].baseline_count, 4.0)
        self.assertEqual(outs[30].horizon_sec, 30)
        self.assertTrue(outs[30].model_id)

        stale = self.second(31, send=False)
        self.assertFalse(stale.valid)
        self.assertEqual(stale.baseline_count, -1.0)

        recovery = [self.second(k) for k in range(32, 63)]
        self.assertTrue(all(not o.valid for o in recovery[:30]))
        self.assertTrue(recovery[30].valid, "valid again after gap")

    def test_valid_output_stamp_is_observation_time(self):
        from smart_hub_interfaces.msg import QueueState
        for k in range(30):
            self.second(k)
        self.mono[0] = 1030.0
        msg = QueueState()
        msg.zone_id = "Q"
        msg.count = 5
        msg.confidence = 0.9
        msg.valid = True
        time.sleep(0.002)
        msg.header.stamp = self.node.get_clock().now().to_msg()
        self.node.on_queue(msg)
        self.mono[0] = 1030.05
        self.node._tick()
        out = self.node.last_output
        self.assertTrue(out.valid)
        self.assertEqual((out.header.stamp.sec, out.header.stamp.nanosec),
                         (msg.header.stamp.sec, msg.header.stamp.nanosec))

    def test_wrong_zone_ignored(self):
        from smart_hub_interfaces.msg import QueueState
        msg = QueueState()
        msg.zone_id = "C"
        msg.count = 9
        msg.confidence = 0.9
        msg.valid = True
        msg.header.stamp = self.node.get_clock().now().to_msg()
        self.node.on_queue(msg)
        self.assertEqual(self.node._wrong_zone, 1)


if __name__ == "__main__":
    unittest.main()
