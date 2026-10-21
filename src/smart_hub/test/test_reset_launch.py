"""launch_testing: /scenario/reset handshake and route drawing on the real graph.

Starts hub.launch.py, then from the test process:
  1. latches an e-stop, calls /scenario/reset and requires success with all
     five participants acknowledged (M5's four nodes + bus_manager, contract
     §7), and the latch still set afterwards (a reset is not a way around
     /safety/clear_stop);
  2. publishes a nav_msgs/Path on /robot/route (contract §3) and requires a
     LINE_STRIP with those points on /hub/markers;
  3. bus_manager (M2-A) rejects DockBus goals it must not take: a wrong bus
     id, an unknown bay and a zero duration.
"""
import os
import time
import unittest

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing
import launch_testing.actions
import pytest

DOMAIN = 93


@pytest.mark.launch_test
def generate_test_description():
    share = get_package_share_directory('smart_hub')
    return LaunchDescription([
        SetEnvironmentVariable('ROS_DOMAIN_ID', str(DOMAIN)),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'hub.launch.py')),
            launch_arguments={'scenario': 'S9', 'seed': '5', 'publish_estop': 'false',
                              'output_dir': '/tmp/reset_launch'}.items()),
        launch_testing.actions.ReadyToTest(),
    ])


class TestResetAndRoutes(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        import rclpy
        from rclpy.node import Node
        cls.rclpy = rclpy
        cls.ctx = rclpy.Context()
        rclpy.init(context=cls.ctx, domain_id=DOMAIN)
        cls.node = Node('reset_launch_test', context=cls.ctx)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        cls.rclpy.shutdown(context=cls.ctx)

    def spin_until(self, cond, timeout):
        """Spin our private context until cond() is true or timeout passes."""
        from rclpy.executors import SingleThreadedExecutor
        if not hasattr(self, '_exec'):
            self._exec = SingleThreadedExecutor(context=self.ctx)
            self._exec.add_node(self.node)
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            self._exec.spin_once(timeout_sec=0.05)
            if cond():
                return True
        return cond()

    def test_reset_handshake_keeps_latch(self):
        from smart_hub.ros_common import R1T
        from smart_hub_interfaces.msg import SafetyState
        from smart_hub_interfaces.srv import ResetScenario
        from std_msgs.msg import Bool
        safety = []
        self.node.create_subscription(SafetyState, '/hub/safety', safety.append, R1T)
        estop = self.node.create_publisher(Bool, '/hub/estop', R1T)
        client = self.node.create_client(ResetScenario, '/scenario/reset')
        self.assertTrue(self.spin_until(client.service_is_ready, 20.0))

        def latched():
            estop.publish(Bool(data=True))
            return bool(safety) and safety[-1].stop_latched
        self.assertTrue(self.spin_until(latched, 10.0), 'e-stop never latched')

        fut = client.call_async(ResetScenario.Request(scenario_id='S9', seed=6))
        self.assertTrue(self.spin_until(fut.done, 10.0))
        res = fut.result()
        self.assertTrue(res.success, res.reason)
        self.assertIn('acks=5', res.reason)
        self.assertTrue(res.run_id.startswith('S9-s6-'))
        n = len(safety)
        self.assertTrue(self.spin_until(lambda: len(safety) > n + 3, 5.0))
        self.assertTrue(safety[-1].stop_latched, 'reset must not clear the latch')

    def test_route_is_drawn(self):
        from geometry_msgs.msg import PoseStamped
        from nav_msgs.msg import Path
        from smart_hub.ros_common import R10, R1T
        from visualization_msgs.msg import MarkerArray
        markers = []
        self.node.create_subscription(MarkerArray, '/hub/markers', markers.append, R10)
        pub = self.node.create_publisher(Path, '/robot/route', R1T)
        route = Path()
        route.header.frame_id = 'map'
        for x, y in ((3.0, 6.0), (5.0, 6.0), (5.0, 4.0)):
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x, pose.pose.position.y = x, y
            route.poses.append(pose)

        def drawn():
            pub.publish(route)
            for array in markers[-3:]:
                for m in array.markers:
                    if m.ns == 'routes' and m.points:
                        return [(p.x, p.y) for p in m.points] == [(3.0, 6.0), (5.0, 6.0),
                                                                  (5.0, 4.0)]
            return False
        self.assertTrue(self.spin_until(drawn, 15.0), 'route never appeared on /hub/markers')

    def test_bus_manager_rejects_invalid_goals(self):
        from rclpy.action import ActionClient
        from smart_hub_interfaces.action import DockBus
        client = ActionClient(self.node, DockBus, '/bus/dock')
        self.assertTrue(self.spin_until(client.server_is_ready, 20.0))
        for goal in (DockBus.Goal(bus_id='bus_9', bay_id='B', max_duration_sec=60.0),
                     DockBus.Goal(bus_id='bus_1', bay_id='Z', max_duration_sec=60.0),
                     DockBus.Goal(bus_id='bus_1', bay_id='B', max_duration_sec=0.0)):
            fut = client.send_goal_async(goal)
            self.assertTrue(self.spin_until(fut.done, 5.0))
            self.assertFalse(fut.result().accepted, goal)
