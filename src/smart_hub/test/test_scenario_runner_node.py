"""Node-level tests of scenario_runner with real rclpy (no other processes).

Calls the node's own callbacks directly, so it is fast and deterministic.
"""
import importlib.util
import unittest

HAVE_ROS = (importlib.util.find_spec('rclpy') is not None
            and importlib.util.find_spec('smart_hub_interfaces') is not None)


@unittest.skipUnless(HAVE_ROS, 'ROS 2 + smart_hub_interfaces needed')
class TestScenarioRunnerNode(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        import rclpy
        cls.ctx = rclpy.Context()
        rclpy.init(context=cls.ctx, domain_id=89)

    @classmethod
    def tearDownClass(cls):
        import rclpy
        rclpy.shutdown(context=cls.ctx)

    def setUp(self):
        from smart_hub.scenario_runner import ScenarioRunner
        self.node = ScenarioRunner(context=self.ctx)

    def tearDown(self):
        self.node.destroy_node()

    def reset(self, scenario_id, seed):
        from smart_hub_interfaces.srv import ResetScenario
        req = ResetScenario.Request(scenario_id=scenario_id, seed=seed, preserve_logs=False)
        return self.node.on_reset(req, ResetScenario.Response())

    def test_reset_gives_new_run_id_and_initial_world(self):
        first_run = self.node.run_id
        for _ in range(20):
            self.node.sim.set_command('robot_1', 0.2, 0.0)
            self.node.tick()
        self.assertGreater(self.node.sim.bodies['robot_1'].x, 3.0)
        res = self.reset('S9', 42)
        self.assertTrue(res.success)
        self.assertNotEqual(res.run_id, first_run)
        self.assertEqual(res.run_id, self.node.run_id)
        self.assertEqual(self.node.sim.bodies['robot_1'].x, 3.0)
        self.assertEqual(self.node.sim.seed, 42)

    def test_failed_reset_keeps_current_run(self):
        run, sim = self.node.run_id, self.node.sim
        res = self.reset('DOES_NOT_EXIST', 1)
        self.assertFalse(res.success)
        self.assertIn('RESET_FAILED', res.reason)
        self.assertEqual((self.node.run_id, self.node.sim, self.node.scenario_id),
                         (run, sim, 'S9'))

    def test_bad_injection_is_ignored(self):
        from std_msgs.msg import String
        self.node.on_inject(String(data='{"type": "teleport"}'))
        self.node.on_inject(String(data='not json'))
        self.node.on_inject(String(data='{"type": "freeze", "topic": "odom", "duration": 1}'))
        self.assertTrue(self.node.sim.is_frozen('odom'))
