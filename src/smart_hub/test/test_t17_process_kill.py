"""T17 on the real ROS graph: kill the controller, then the supervisor.

Starts scenario_runner, safety_supervisor, actuator_bridge and a mock robot
controller as separate OS processes, then SIGKILLs them one at a time
(assignment 16.3: "use process termination tests in simulation first") and
checks that each remaining layer stops the robot on its own:

  1. controller killed  -> safety_supervisor outputs zero (command timeout)
  2. supervisor killed  -> actuator_bridge outputs zero (its own watchdog)
  3. bridge killed      -> the simulator's drive timeout stops the body

Needs ROS 2 and the built workspace; skipped otherwise. Runs on its own ROS
domain so it cannot disturb other tests.
"""
import importlib.util
import os
import signal
import subprocess
import time
import unittest

HAVE_ROS = (importlib.util.find_spec('rclpy') is not None
            and importlib.util.find_spec('smart_hub_interfaces') is not None)
DOMAIN = 87
TIMEOUT = 0.5          # command timeout used by supervisor, bridge and simulator
MARGIN = 0.25          # one 20 Hz tick + transport + test polling


def start(executable, *params):
    args = ['ros2', 'run', 'smart_hub', executable, '--ros-args']
    for p in params:
        args += ['-p', p]
    env = dict(os.environ, ROS_DOMAIN_ID=str(DOMAIN))
    # own process group, so SIGKILL reaches the node and not only 'ros2 run'
    return subprocess.Popen(args, env=env, start_new_session=True,
                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def kill(proc):
    try:
        os.killpg(proc.pid, signal.SIGKILL)
    except ProcessLookupError:
        pass
    proc.wait(timeout=5)


@unittest.skipUnless(HAVE_ROS, 'ROS 2 + smart_hub_interfaces needed')
class TestT17Watchdogs(unittest.TestCase):
    def setUp(self):
        from geometry_msgs.msg import Twist
        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node
        from smart_hub.ros_common import R10
        from smart_hub_interfaces.msg import VehicleStateArray

        self.rclpy = rclpy
        self.ctx = rclpy.Context()
        rclpy.init(context=self.ctx, domain_id=DOMAIN)
        self.node = Node('t17_observer', context=self.ctx)
        # An executor bound to our private context (the global one is not).
        self.executor = SingleThreadedExecutor(context=self.ctx)
        self.executor.add_node(self.node)
        self.final, self.applied, self.speed = [], [], [None]
        self.node.create_subscription(
            Twist, '/robot/cmd_vel',
            lambda m: self.final.append((time.monotonic(), m.linear.x)), R10)
        self.node.create_subscription(
            Twist, '/robot/cmd_vel_applied',
            lambda m: self.applied.append((time.monotonic(), m.linear.x)), R10)

        def on_truth(msg):
            for v in msg.vehicles:
                if v.vehicle_id == 'robot_1':
                    self.speed[0] = v.speed_mps
        self.node.create_subscription(VehicleStateArray, '/sim/truth', on_truth, R10)
        self.procs = {
            'runner': start('scenario_runner', 'scenario:=S9', 'seed:=17'),
            'supervisor': start('safety_supervisor'),
            'bridge': start('actuator_bridge'),
            'controller': start('mock_candidate_publisher', 'agent:=robot', 'linear_x:=0.2'),
        }

    def tearDown(self):
        for proc in self.procs.values():
            kill(proc)
        self.executor.shutdown()
        self.node.destroy_node()
        self.rclpy.shutdown(context=self.ctx)

    def spin_until(self, cond, timeout):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            self.executor.spin_once(timeout_sec=0.01)
            if cond():
                return True
        return False

    def first_zero_after(self, series, t0):
        return next((t - t0 for t, vx in series if t >= t0 and abs(vx) < 1e-6), None)

    def assert_moving(self, what):
        ok = self.spin_until(lambda: self.speed[0] is not None and self.speed[0] > 0.15, 30.0)
        self.assertTrue(ok, 'robot never reached 0.2 m/s (%s); speed=%s' % (what, self.speed[0]))

    def test_each_layer_stops_the_robot_alone(self):
        self.assert_moving('nominal')

        t0 = time.monotonic()
        kill(self.procs['controller'])
        self.spin_until(lambda: self.first_zero_after(self.final, t0) is not None, 3.0)
        dt = self.first_zero_after(self.final, t0)
        self.assertIsNotNone(dt, 'supervisor kept a nonzero command after controller death')
        self.assertLessEqual(dt, TIMEOUT + MARGIN)
        self.assertTrue(self.spin_until(lambda: self.speed[0] < 0.02, 3.0))
        print('T17 controller killed -> supervisor zero after %.3f s' % dt)

        self.procs['controller'] = start('mock_candidate_publisher', 'agent:=robot',
                                         'linear_x:=0.2')
        self.assert_moving('after controller restart')

        t1 = time.monotonic()
        kill(self.procs['supervisor'])
        self.spin_until(lambda: self.first_zero_after(self.applied, t1) is not None, 3.0)
        dt = self.first_zero_after(self.applied, t1)
        self.assertIsNotNone(dt, 'bridge kept moving after supervisor death')
        self.assertLessEqual(dt, TIMEOUT + MARGIN)
        self.assertTrue(self.spin_until(lambda: self.speed[0] < 0.02, 3.0))
        late = [vx for t, vx in self.applied if t >= t1 + dt and abs(vx) > 1e-6]
        self.assertEqual(late, [], 'bridge restarted motion without a supervisor')
        print('T17 supervisor killed -> bridge zero after %.3f s' % dt)

        # Third layer: with supervisor and bridge gone, nothing drives the sim.
        kill(self.procs['bridge'])
        self.spin_until(lambda: False, 1.0)
        self.assertLess(self.speed[0], 0.02)
