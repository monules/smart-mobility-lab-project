"""Node-level tests of scenario_runner with real rclpy (no other processes).

The reset service is a coroutine; run_reset() drives it on a private
executor exactly as rclpy.spin would, so the 20 Hz tick keeps running while
it waits for acknowledgements.
"""
import importlib.util
import time
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

    def make(self, participants='', timeout=0.5):
        from rclpy.parameter import Parameter
        from smart_hub.scenario_runner import ScenarioRunner
        return ScenarioRunner(context=self.ctx, parameter_overrides=[
            Parameter('reset_participants', value=participants),
            Parameter('reset_ack_timeout', value=timeout)])

    def setUp(self):
        self.node = self.make()            # no participants: reset returns at once

    def tearDown(self):
        self.node.destroy_node()

    def run_reset(self, node, scenario_id, seed, ack_from=(), ack_run_id=None):
        """Call /scenario/reset through a real service client.

        Going through the service (not calling on_reset directly) exercises
        rclpy's callback-group rules: the waiting service must not block the
        tick or the event listener. ack_from nodes answer the broadcast.
        """
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node
        from smart_hub.ros_common import RESET_ACK
        from smart_hub_interfaces.msg import HubEvent
        from smart_hub_interfaces.srv import ResetScenario
        client_node = Node('reset_client_%d' % seed, context=self.ctx)
        client = client_node.create_client(ResetScenario, '/scenario/reset')
        executor = SingleThreadedExecutor(context=self.ctx)
        executor.add_node(node)
        executor.add_node(client_node)
        end = time.monotonic() + 5.0
        while not client.service_is_ready() and time.monotonic() < end:
            executor.spin_once(timeout_sec=0.02)
        future = client.call_async(ResetScenario.Request(
            scenario_id=scenario_id, seed=seed, preserve_logs=False))
        acked = False
        while not future.done() and time.monotonic() < end:
            executor.spin_once(timeout_sec=0.02)
            if node.pending and not acked:
                for name in ack_from:
                    node.on_event(HubEvent(event=RESET_ACK, entity_id=name,
                                           correlation_id=ack_run_id or node.pending['run_id']))
                acked = True
        executor.shutdown()
        client_node.destroy_node()
        self.assertTrue(future.done(), 'reset never answered (callback-group deadlock?)')
        return future.result()

    def test_reset_gives_new_run_id_and_initial_world(self):
        first_run = self.node.run_id
        for _ in range(20):
            self.node.sim.set_command('robot_1', 0.2, 0.0)
            self.node.tick()
        self.assertGreater(self.node.sim.bodies['robot_1'].x, 3.0)
        res = self.run_reset(self.node, 'S9', 42)
        self.assertTrue(res.success)
        self.assertNotEqual(res.run_id, first_run)
        self.assertEqual(res.run_id, self.node.run_id)
        self.assertEqual(self.node.sim.bodies['robot_1'].x, 3.0)
        self.assertEqual(self.node.sim.seed, 42)

    def test_failed_reset_keeps_current_run(self):
        run, sim = self.node.run_id, self.node.sim
        res = self.run_reset(self.node, 'DOES_NOT_EXIST', 1)
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

    def test_reset_waits_for_every_ack(self):
        node = self.make('safety_supervisor,actuator_bridge', timeout=3.0)
        t0 = time.monotonic()
        res = self.run_reset(node, 'S9', 3, ack_from=('safety_supervisor', 'actuator_bridge'))
        node.destroy_node()
        self.assertTrue(res.success, res.reason)
        self.assertIn('acks=2', res.reason)
        self.assertLess(time.monotonic() - t0, 1.5)   # answered on the acks, not the timeout

    def test_missing_ack_is_reported_by_name_and_old_acks_do_not_count(self):
        node = self.make('safety_supervisor,metrics_logger', timeout=0.4)
        res = self.run_reset(node, 'S9', 4, ack_from=('metrics_logger',), ack_run_id='old-run')
        node.destroy_node()
        self.assertFalse(res.success)
        self.assertEqual(res.reason, 'MISSING_ACK: metrics_logger,safety_supervisor')
        self.assertTrue(res.run_id.startswith('S9-s4-'))   # the new world is running

    def test_request_is_resent_while_waiting(self):
        from smart_hub.ros_common import RESET_REQUEST
        node = self.make('safety_supervisor', timeout=1.0)
        sent = []
        original = node._event
        node._event = lambda event, *a: (sent.append(event), original(event, *a))
        self.run_reset(node, 'S9', 5)
        node.destroy_node()
        # first request + resends every 0.25 s until the 1.0 s timeout
        self.assertGreaterEqual(sent.count(RESET_REQUEST), 3)
